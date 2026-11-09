#include "carvetto/geometry/offset.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <initializer_list>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <utility>
#include <vector>

#include "box_index.hpp"
#include "box_tree.hpp"
#include "cancel_poll.hpp"
#include "clipper_interop.hpp"
#include "hole_nesting.hpp"
#include "offset_batch.hpp"
#include "offset_dust.hpp"
#include "offset_junction.hpp"
#include "offset_shortcuts.hpp"

namespace carvetto::geometry {

using detail::pollCancel;
using detail::pollCancelEvery;

// ---------------------------------------------------------------------------------------------
// Closed contours: Clipper2.
// ---------------------------------------------------------------------------------------------

namespace {

Clipper2Lib::JoinType toClipperJoin(JoinType join) {
    switch (join) {
        case JoinType::Round:
            return Clipper2Lib::JoinType::Round;
        case JoinType::Sharp:
            return Clipper2Lib::JoinType::Miter;
        case JoinType::Square:
            return Clipper2Lib::JoinType::Square;
    }
    return Clipper2Lib::JoinType::Round;
}

// Grid-level dust (T-0138). Within a few nanometres of collapse Clipper2 can return rings no
// wider than a few grid units: genuine regions shrunk to the rounding level, and debris wound
// against its nesting (ADR-0006 §8). A ring is dust when no disk of radius kDustRadiusUnits
// (scaled up for coordinates where double rounding exceeds the grid) fits inside it, as judged
// by Clipper2's erosion, which is not exact on acute rings (removed rings have an inradius of up
// to about 5.6 units, kept ones down to about 3.6).
constexpr double kDustRadiusUnits = 4.0;

double dustRadiusUnits(const Clipper2Lib::Paths64& paths) {
    double maxAbs = 0.0;
    for (const auto& path : paths) {
        for (const auto& p : path) {
            const double ax = std::abs(static_cast<double>(p.x));
            const double ay = std::abs(static_cast<double>(p.y));
            maxAbs = std::max({maxAbs, ax, ay});
        }
    }
    // Clipper2 computes offset and intersection points in double from the int64 grid: a few
    // ulps of the coordinates, which exceed one unit only beyond ~2.8e14 units (2.8e8 mm).
    const double rounding = 16.0 * std::numeric_limits<double>::epsilon() * maxAbs;
    return kDustRadiusUnits * std::max(1.0, rounding);
}

double perimeterUnits(const Clipper2Lib::Path64& path) {
    double len = 0.0;
    for (std::size_t i = 0, n = path.size(); i < n; ++i) {
        const auto& a = path[i];
        const auto& b = path[(i + 1) % n];
        len += std::hypot(static_cast<double>(b.x - a.x), static_cast<double>(b.y - a.y));
    }
    return len;
}

// True when no disk of radius `radius` (grid units) fits inside the ring. A ring of inradius r
// satisfies |area| <= r x perimeter (every point lies within r of the boundary), so rings with
// |area| / perimeter >= radius are kept without the erosion test.
bool isDustRing(const Clipper2Lib::Path64& path, double radius) {
    if (path.size() < 3) {
        return true;
    }
    const double perimeter = perimeterUnits(path);
    if (!(perimeter > 0.0)) {
        return true;
    }
    if (std::abs(Clipper2Lib::Area(path)) >= radius * perimeter) {
        return false;
    }
    // A lone ring shrinks whichever way it is wound (Clipper2 takes the orientation of the
    // outermost path of a group as "outwards").
    const Clipper2Lib::Paths64 eroded = Clipper2Lib::InflatePaths(
        Clipper2Lib::Paths64{path}, -radius, Clipper2Lib::JoinType::Round, Clipper2Lib::EndType::Polygon);
    return eroded.empty();
}

// Removes dust rings (see isDustRing) and any ring nested inside a removed one.
void dropDustRings(Clipper2Lib::Paths64& paths) {
    if (paths.empty()) {
        return;
    }
    const double radius = dustRadiusUnits(paths);
    std::vector<bool> dust(paths.size(), false);
    bool any = false;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        dust[i] = isDustRing(paths[i], radius);
        any = any || dust[i];
    }
    if (!any) {
        return;
    }
    // A ring inside a dust ring is normally narrower still, so the erosion test drops it too,
    // but that test is not exact (ADR-0006 §8); drop such rings explicitly so that no hole
    // outlives its outer.
    std::vector<detail::GridBox> boxes(paths.size());
    std::vector<bool> usable(paths.size());
    for (std::size_t i = 0; i < paths.size(); ++i) {
        const Clipper2Lib::Rect64 b = Clipper2Lib::GetBounds(paths[i]);
        boxes[i] = {b.left, b.top, b.right, b.bottom};
        // Outers need 3 points; inner rings any point (an empty path is never tested).
        usable[i] = !paths[i].empty();
    }
    detail::markNestedInDust(boxes, usable, dust, [&paths](std::size_t outer, std::size_t inner) {
        if (paths[outer].size() < 3) {
            return false;
        }
        bool inside = false;
        for (const auto& p : paths[inner]) {
            const auto where = Clipper2Lib::PointInPolygon(p, paths[outer]);
            if (where == Clipper2Lib::PointInPolygonResult::IsOutside) {
                return false;
            }
            inside = inside || where == Clipper2Lib::PointInPolygonResult::IsInside;
        }
        return inside;
    });
    std::size_t kept = 0;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        if (!dust[i]) {
            if (kept != i) {
                paths[kept] = std::move(paths[i]);
            }
            ++kept;
        }
    }
    paths.resize(kept);
}

}  // namespace

namespace detail {

void markNestedInDust(const std::vector<GridBox>& boxes, const std::vector<bool>& usable,
                      std::vector<bool>& dust,
                      const std::function<bool(std::size_t outer, std::size_t inner)>& nested) {
    std::vector<std::size_t> byLeft;
    std::vector<std::size_t> work;
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        if (!usable[i]) {
            continue;
        }
        if (dust[i]) {
            work.push_back(i);
        } else {
            byLeft.push_back(i);
        }
    }
    if (work.empty()) {
        return;
    }
    std::vector<std::size_t> byTop = byLeft;
    std::sort(byLeft.begin(), byLeft.end(), [&boxes](std::size_t a, std::size_t b) {
        return boxes[a].left < boxes[b].left || (boxes[a].left == boxes[b].left && a < b);
    });
    std::sort(byTop.begin(), byTop.end(), [&boxes](std::size_t a, std::size_t b) {
        return boxes[a].top < boxes[b].top || (boxes[a].top == boxes[b].top && a < b);
    });
    // The rings whose left (top) edge lies within [lo, hi]: a range of byLeft (byTop).
    auto range = [&boxes](const std::vector<std::size_t>& sorted, bool alongY, std::int64_t lo, std::int64_t hi) {
        auto key = [&boxes, alongY](std::size_t a) { return alongY ? boxes[a].top : boxes[a].left; };
        const auto first = std::lower_bound(sorted.begin(), sorted.end(), lo,
                                            [&key](std::size_t a, std::int64_t v) { return key(a) < v; });
        const auto last = std::upper_bound(first, sorted.end(), hi,
                                           [&key](std::int64_t v, std::size_t a) { return v < key(a); });
        return std::make_pair(first, last);
    };
    while (!work.empty()) {
        const std::size_t i = work.back();
        work.pop_back();
        const GridBox& box = boxes[i];
        // Scan whichever axis holds fewer candidates (a column of rings shares its x-range).
        const auto inX = range(byLeft, false, box.left, box.right);
        const auto inY = range(byTop, true, box.top, box.bottom);
        const bool scanY = inY.second - inY.first < inX.second - inX.first;
        const auto first = scanY ? inY.first : inX.first;
        const auto last = scanY ? inY.second : inX.second;
        for (auto it = first; it != last; ++it) {
            const std::size_t j = *it;
            const GridBox& bj = boxes[j];
            if (dust[j] || bj.left < box.left || bj.right > box.right || bj.top < box.top || bj.bottom > box.bottom) {
                continue;
            }
            if (nested(i, j)) {
                dust[j] = true;
                work.push_back(j);
            }
        }
    }
}

}  // namespace detail

namespace {

// Lowest point of a path by Clipper2's rule for the "lowest closed path" of an offset group
// (largest y, then smallest x; the first of equal points wins).
struct LowestPoint {
    std::int64_t x{0};
    std::int64_t y{0};
};

LowestPoint lowestPointOf(const Clipper2Lib::Path64& path) {
    LowestPoint best{std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::min()};
    for (const auto& p : path) {
        if (p.y < best.y || (p.y == best.y && p.x >= best.x)) {
            continue;
        }
        best = {p.x, p.y};
    }
    return best;
}

// True when `a` is strictly lower than `b` by that rule (so the earlier path wins ties).
bool lowerThan(const LowestPoint& a, const LowestPoint& b) {
    return a.y > b.y || (a.y == b.y && a.x < b.x);
}

// One Clipper2 offset of closed paths, all in one group (the lowest path decides orientation).
Clipper2Lib::Paths64 offsetClosedGroup(const Clipper2Lib::Paths64& paths, double delta, Clipper2Lib::JoinType join,
                                       double mitreLimit, double arcTolerance) {
    Clipper2Lib::ClipperOffset offsetter(std::max(mitreLimit, 1.0), arcTolerance);
    offsetter.PreserveCollinear(false);
    offsetter.AddPaths(paths, join, Clipper2Lib::EndType::Polygon);
    Clipper2Lib::Paths64 out;
    offsetter.Execute(delta, out);
    return out;
}

// The batches closed paths are offset in (T-0141, ADR-0006 §9), or nothing when one Clipper2
// call over all of them is cheap enough (fewer than `minDepth` batches side by side) or the
// input does not split.
//
// Every point Clipper2 emits for a path offset by delta lies within reach x |delta| of a path
// vertex or edge, plus half a unit of rounding: reach is 1 for round joins (offset edges and
// arcs at |delta|; the original vertex of a concave join lies on the path), sqrt 2 for square
// corners, max(sqrt 2, mitreLimit) for mitres (square beyond the limit). Boxes grown by that
// and separated by a gap keep their raw offsets apart, so each batch's union is the global
// union restricted to it.
// Clipper2 offsets a whole group the way its lowest path is wound (ADR-0006 §4); a batch whose
// own lowest path is wound the other way would flip, so it joins the batch holding the global
// lowest path, which then decides for it as in one call.
std::optional<std::vector<std::vector<std::size_t>>> planBatches(const Clipper2Lib::Paths64& paths, double delta,
                                                                 JoinType join, double mitreLimit,
                                                                 std::size_t minDepth) {
    if (paths.size() < 2 || paths.size() < minDepth) {
        return std::nullopt;
    }
    constexpr double kSqrt2 = 1.4142135623730951;
    const double reach = join == JoinType::Round ? 1.0 : (join == JoinType::Square ? kSqrt2 : std::max(kSqrt2, mitreLimit));
    // Per axis: at most reach x |delta| (with a relative margin for the double arithmetic) plus
    // half a unit of rounding to the grid.
    const double margin = std::ceil(std::abs(delta) * reach * (1.0 + 1e-9)) + 1.0;
    // Keep grown boxes far inside int64 (Clipper2's own range is +-2.3e18 units).
    constexpr double kLimit = 4.0e18;
    if (!(margin < kLimit)) {
        return std::nullopt;
    }
    const auto grow = static_cast<std::int64_t>(margin);
    std::vector<detail::GridBox> boxes(paths.size());
    for (std::size_t i = 0; i < paths.size(); ++i) {
        const Clipper2Lib::Rect64 b = Clipper2Lib::GetBounds(paths[i]);
        const double lo = std::min(static_cast<double>(b.left), static_cast<double>(b.top));
        const double hi = std::max(static_cast<double>(b.right), static_cast<double>(b.bottom));
        if (!(lo - margin > -kLimit && hi + margin < kLimit)) {
            return std::nullopt;
        }
        boxes[i] = {b.left - grow, b.top - grow, b.right + grow, b.bottom + grow};
    }
    std::vector<std::vector<std::size_t>> batches = detail::separableBatches(boxes);
    if (batches.size() < 2 || batches.size() < minDepth) {
        return std::nullopt;
    }
    std::vector<LowestPoint> lowest(paths.size());
    std::size_t globalLowest = 0;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        lowest[i] = lowestPointOf(paths[i]);
        if (lowerThan(lowest[i], lowest[globalLowest])) {
            globalLowest = i;
        }
    }
    const bool reversed = Clipper2Lib::Area(paths[globalLowest]) < 0.0;
    std::size_t home = 0;
    std::vector<bool> flips(batches.size(), false);
    for (std::size_t b = 0; b < batches.size(); ++b) {
        std::size_t low = batches[b].front();
        for (const std::size_t i : batches[b]) {
            if (lowerThan(lowest[i], lowest[low])) {
                low = i;
            }
            if (i == globalLowest) {
                home = b;
            }
        }
        flips[b] = (Clipper2Lib::Area(paths[low]) < 0.0) != reversed;
    }
    std::vector<std::vector<std::size_t>> kept;
    kept.reserve(batches.size());
    std::vector<std::size_t> merged;
    for (std::size_t b = 0; b < batches.size(); ++b) {
        if (b == home || flips[b]) {
            merged.insert(merged.end(), batches[b].begin(), batches[b].end());
        }
    }
    std::sort(merged.begin(), merged.end());
    for (std::size_t b = 0; b < batches.size(); ++b) {
        if (b != home && !flips[b]) {
            kept.push_back(std::move(batches[b]));
        }
    }
    kept.push_back(std::move(merged));
    std::sort(kept.begin(), kept.end(), [](const auto& a, const auto& b) { return a.front() < b.front(); });
    if (kept.size() < 2 || detail::batchDepth(boxes, kept) < minDepth) {
        return std::nullopt;
    }
    return kept;
}

// Rings wound against the group's lowest path that are not nested in a ring wound like it
// (T-0142, ADR-0006 §10). On an outset (holes shrink) Clipper2 joins every corner of such a ring
// through the original vertex (the T-0018 backport, §6), which leaves a loop of positive winding
// at each corner; inside an enclosing ring those loops merge into its region, but with nothing
// around them they survive the final union as leftover rings (a lone 1 mm circle: 71 leftover
// rings at +0.3 mm, chordal 0.001), and an over-shrunk one leaves a whole phantom region. Such input describes no region
// there (its winding is not positive), so the closed paths are first resolved into the region
// they describe (Clipper2's union with the group's fill rule: Positive, or Negative for a group
// whose lowest path is clockwise, keeping that orientation), which has only nested holes.
//
// A ring counts as nested when a ring wound like the lowest path has a bounding box containing
// its own and holds its first vertex strictly inside. Rings that cross each other are not
// detected. Returns true when the paths were resolved.
bool resolveUnnestedHoles(Clipper2Lib::Paths64& paths, double delta) {
    if (paths.size() < 2 || !(delta >= 0.5)) {
        return false;  // insets and zero offsets (below half a unit Clipper2 copies the input)
    }
    std::size_t lowestIdx = 0;
    LowestPoint lowest = lowestPointOf(paths[0]);
    for (std::size_t i = 1; i < paths.size(); ++i) {
        const LowestPoint l = lowestPointOf(paths[i]);
        if (lowerThan(l, lowest)) {
            lowest = l;
            lowestIdx = i;
        }
    }
    const bool reversed = Clipper2Lib::Area(paths[lowestIdx]) < 0.0;
    std::vector<int> kind(paths.size(), 0);  // 1 wound like the lowest, 2 against it, 0 neither
    bool anyAgainst = false;
    for (std::size_t i = 0; i < paths.size(); ++i) {
        const double a = Clipper2Lib::Area(paths[i]);
        if (a == 0.0) {
            continue;
        }
        kind[i] = (a < 0.0) == reversed ? 1 : 2;
        anyAgainst = anyAgainst || kind[i] == 2;
    }
    if (!anyAgainst) {
        return false;
    }
    // Candidates from a box hierarchy, smallest box first, and indexed point-in-ring tests on big
    // rings (hole_nesting.hpp): near-linear for many shapes with their holes, whether their boxes
    // are apart, touch or overlap (T-0142 round 3; a scan over the rings left of each hole was
    // quadratic for a column of touching shapes, and each hole in a comb's notches walked the comb).
    const bool allNested = detail::everyHoleNested(paths, kind);
    if (allNested) {
        return false;
    }
    paths = detail::clipClosedPaths(Clipper2Lib::ClipType::Union,
                                    reversed ? Clipper2Lib::FillRule::Negative : Clipper2Lib::FillRule::Positive,
                                    reversed, std::move(paths), {});
    return true;
}

}  // namespace

namespace detail {

std::vector<std::vector<std::size_t>> separableBatches(const std::vector<GridBox>& boxes) {
    struct Job {
        std::vector<std::size_t> items;
        bool alongY{false};
        bool otherAxisFailed{false};
    };
    std::vector<std::vector<std::size_t>> batches;
    if (boxes.empty()) {
        return batches;
    }
    std::vector<Job> stack;
    {
        Job all;
        all.items.resize(boxes.size());
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            all.items[i] = i;
        }
        stack.push_back(std::move(all));
    }
    while (!stack.empty()) {
        Job job = std::move(stack.back());
        stack.pop_back();
        const bool y = job.alongY;
        auto lo = [&boxes, y](std::size_t i) { return y ? boxes[i].top : boxes[i].left; };
        auto hi = [&boxes, y](std::size_t i) { return y ? boxes[i].bottom : boxes[i].right; };
        std::sort(job.items.begin(), job.items.end(), [&lo](std::size_t a, std::size_t b) {
            return lo(a) < lo(b) || (lo(a) == lo(b) && a < b);
        });
        std::vector<std::vector<std::size_t>> parts;
        std::int64_t reach = 0;
        for (const std::size_t i : job.items) {
            if (parts.empty() || lo(i) > reach) {
                parts.emplace_back();
                reach = hi(i);
            } else {
                reach = std::max(reach, hi(i));
            }
            parts.back().push_back(i);
        }
        if (parts.size() == 1) {
            if (job.otherAxisFailed) {
                batches.push_back(std::move(parts.front()));
            } else {
                stack.push_back(Job{std::move(parts.front()), !y, true});
            }
            continue;
        }
        for (auto& part : parts) {
            if (part.size() == 1) {
                batches.push_back(std::move(part));
            } else {
                stack.push_back(Job{std::move(part), !y, false});
            }
        }
    }
    for (auto& batch : batches) {
        std::sort(batch.begin(), batch.end());
    }
    std::sort(batches.begin(), batches.end(), [](const auto& a, const auto& b) { return a.front() < b.front(); });
    return batches;
}

std::size_t batchDepth(const std::vector<GridBox>& boxes, const std::vector<std::vector<std::size_t>>& batches) {
    // +1 at a batch's top, -1 just after its bottom; opening events sort first at equal y.
    std::vector<std::pair<std::int64_t, int>> events;
    events.reserve(2 * batches.size());
    for (const auto& batch : batches) {
        if (batch.empty()) {
            continue;
        }
        std::int64_t top = boxes[batch.front()].top;
        std::int64_t bottom = boxes[batch.front()].bottom;
        for (const std::size_t i : batch) {
            top = std::min(top, boxes[i].top);
            bottom = std::max(bottom, boxes[i].bottom);
        }
        events.emplace_back(top, -1);
        events.emplace_back(bottom, 1);
    }
    std::sort(events.begin(), events.end());
    std::size_t depth = 0;
    std::size_t best = 0;
    for (const auto& [at, kind] : events) {
        (void)at;
        if (kind < 0) {
            best = std::max(best, ++depth);
        } else {
            --depth;
        }
    }
    return best;
}

static std::vector<Contour> offsetContoursBatchedImpl(const std::vector<Contour>& contours, double distance,
                                                      JoinType join, const Tolerance& tol, double mitreLimit,
                                                      std::size_t minBatchDepth) {
    if (!std::isfinite(distance)) {
        return {};
    }
    // Clipper2 approximates round joins with chords; its arc tolerance is in grid units.
    const double arcTolerance = detail::toClipperDistance(tol.chordal);
    const bool zeroDistance = tol.isZero(distance);
    // All closed contours go into ONE Clipper2 group so that the outermost one decides the
    // orientation convention and opposite-oriented contours are treated as holes.
    Clipper2Lib::Paths64 closedPaths;
    Clipper2Lib::Paths64 openPaths;
    for (const Contour& c : contours) {
        if (!c.closed && zeroDistance) {
            continue;  // outlining an open contour with zero width yields nothing
        }
        Clipper2Lib::Path64 path = detail::toClipperPath(c, tol);
        pollCancel();
        if (path.empty()) {
            continue;
        }
        (c.closed ? closedPaths : openPaths).push_back(std::move(path));
    }
    if (closedPaths.empty() && openPaths.empty()) {
        return {};
    }
    const double delta = detail::toClipperDistance(distance);
    const Clipper2Lib::JoinType clipperJoin = toClipperJoin(join);
    pollCancel();
    resolveUnnestedHoles(closedPaths, delta);
    pollCancel();
    if (closedPaths.empty() && openPaths.empty()) {
        return {};
    }
    Clipper2Lib::Paths64 out;
    std::optional<std::vector<std::vector<std::size_t>>> batches;
    if (openPaths.empty()) {
        batches = planBatches(closedPaths, delta, join, mitreLimit, minBatchDepth);
    }
    if (batches) {
        // Batches in the order of their first input contour, each in Clipper2's own order.
        Clipper2Lib::Paths64 part;
        for (const auto& batch : *batches) {
            pollCancel();  // T-0228: one Clipper2 call is not interruptible, the gaps between are
            part.clear();
            part.reserve(batch.size());
            for (const std::size_t i : batch) {
                part.push_back(std::move(closedPaths[i]));
            }
            Clipper2Lib::Paths64 res = offsetClosedGroup(part, delta, clipperJoin, mitreLimit, arcTolerance);
            out.insert(out.end(), std::make_move_iterator(res.begin()), std::make_move_iterator(res.end()));
        }
    } else {
        Clipper2Lib::ClipperOffset offsetter(std::max(mitreLimit, 1.0), arcTolerance);
        offsetter.PreserveCollinear(false);
        if (!closedPaths.empty()) {
            offsetter.AddPaths(closedPaths, clipperJoin, Clipper2Lib::EndType::Polygon);
        }
        if (!openPaths.empty()) {
            const auto end = join == JoinType::Round ? Clipper2Lib::EndType::Round : Clipper2Lib::EndType::Square;
            offsetter.AddPaths(openPaths, clipperJoin, end);
        }
        offsetter.Execute(delta, out);
    }
    pollCancel();
    dropDustRings(out);
    pollCancel();
    return detail::contoursFromClipper(out, true);
}

std::vector<Contour> offsetContoursBatched(const std::vector<Contour>& contours, double distance, JoinType join,
                                           const Tolerance& tol, double mitreLimit, std::size_t minBatchDepth) {
    try {
        return offsetContoursBatchedImpl(contours, distance, join, tol, mitreLimit, minBatchDepth);
    } catch (const GeometryCancelled&) {
        return {};  // T-0228: cancelled; the caller discards it
    }
}

}  // namespace detail

std::vector<Contour> offsetContours(const std::vector<Contour>& contours, double distance, JoinType join,
                                    const Tolerance& tol, double mitreLimit) {
    return detail::offsetContoursBatched(contours, distance, join, tol, mitreLimit, detail::kOffsetBatchMinDepth);
}

// ---------------------------------------------------------------------------------------------
// Open contours: exact single-sided offset of lines and arcs.
//
// Raw offset + pruning (the approach used by robust polyline offsetters such as
// CavalierContours, described here from first principles):
//  1. Offset every input line/arc on its own (lines shift along their normal, arcs change
//     radius; an arc whose radius would go negative is inverted). Chain the pieces: tangent-
//     continuous junctions need nothing; outside corners get a join (Round: arc of radius d about
//     the vertex; Sharp: both neighbours extended along their own carriers to where they meet,
//     falling back to Square beyond the mitre limit; Square: both neighbours extended to a
//     chamfer line at distance d from the vertex, falling back to Round when an arc cannot
//     reach it); inside corners get a straight connector.
//  2. Discard raw elements lying wholly within d of one input line. Split the rest at all
//     their self-intersections (and where elements overlap on one carrier) and where they
//     cross the rest of the d-neighbourhood boundary (the outer arcs of the circles about the
//     input vertices, offsets on both sides, inverted arcs). A uniform grid supplies the
//     candidate pairs, so this is close to linear in the number of spans.
//  3. Discard every piece that comes closer than d to the input, or that lies strictly inside
//     the region a Sharp/Square corner adds beyond the round neighbourhood (so a neighbour
//     running past a mitre loses its stub), and one of any two pieces that coincide. What
//     remains bounds the d-neighbourhood (plus those corner regions) on the requested side, so
//     spans consumed between inside corners vanish and nothing backtracks.
//  4. Stitch the surviving pieces into components by following their end points.
//  5. Cut every component where it returns to a point it already passed, into open chains and
//     closed loops that never meet themselves, and order them along the input (ADR-0012).
//  6. Snap each chain's junctions so consecutive spans share their end points exactly
//     (T-0006; see snapChain).
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double kParamEps = 1e-9;
constexpr double kInf = std::numeric_limits<double>::infinity();

// Flattened input span list: lines and arcs only, Beziers polygonised, degenerate pieces gone.
std::vector<Span> flattenForOffset(const Contour& c, const Tolerance& tol) {
    std::vector<Span> out;
    std::size_t index = 0;
    for (const Span& s : c.spans) {
        pollCancelEvery(index++);
        if (isDegenerate(s, tol)) {
            continue;
        }
        if (const auto* bez = std::get_if<BezierSpan>(&s)) {
            std::vector<Vec2> pts{bez->p0};
            polygoniseInto(s, tol, pts);
            for (std::size_t i = 1; i < pts.size(); ++i) {
                if (!tol.equalPoints(pts[i - 1], pts[i])) {
                    out.emplace_back(LineSpan{pts[i - 1], pts[i]});
                }
            }
        } else {
            out.push_back(s);
        }
    }
    return out;
}

// Radius below which an offset arc has collapsed to its centre point (rounding level only: a
// tiny but real offset arc is kept so that the raw chain still passes through the centre).
double collapseRadius(const ArcSpan& arc, double d) {
    return 1e-12 * std::max({1.0, arc.radius, d});
}

// Offsets one line or arc span by `d` towards `sign` (+1 left, -1 right). Returns nullopt when
// an arc collapses (its offset radius is not positive).
std::optional<Span> offsetSpan(const Span& s, double d, double sign) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 n = line->tangentAt(0.0).perp() * (d * sign);
        return Span{LineSpan{line->start + n, line->end + n}};
    }
    const auto& arc = std::get<ArcSpan>(s);
    // On a CCW arc the left normal points at the centre; on a CW arc it points away.
    const double towardsCentre = arc.direction == ArcDirection::CCW ? sign : -sign;
    const double r = arc.radius - towardsCentre * d;
    if (r <= collapseRadius(arc, d)) {
        return std::nullopt;
    }
    ArcSpan out = arc;
    out.radius = r;
    return Span{out};
}

// Intersections of the carriers (infinite line / full circle) of two line-or-arc spans.
std::vector<Vec2> carrierIntersections(const Span& a, const Span& b) {
    std::vector<Vec2> out;
    const auto* la = std::get_if<LineSpan>(&a);
    const auto* lb = std::get_if<LineSpan>(&b);
    const auto* ca = std::get_if<ArcSpan>(&a);
    const auto* cb = std::get_if<ArcSpan>(&b);
    if (la && lb) {
        const Vec2 u = la->end - la->start;
        const Vec2 v = lb->end - lb->start;
        const double den = u.cross(v);
        const bool parallel = detail::directionsParallel(u, v);  // T-0170: no hypot unless close
        if (parallel) {
            return out;
        }
        out.push_back(la->start + u * ((lb->start - la->start).cross(v) / den));
        return out;
    }
    if (la || lb) {
        const LineSpan& line = la ? *la : *lb;
        const ArcSpan& arc = ca ? *ca : *cb;
        const Vec2 u = line.end - line.start;
        const Vec2 w = line.start - arc.centre;
        const double qa = u.lengthSquared();
        const double qb = 2.0 * w.dot(u);
        const double qc = w.lengthSquared() - arc.radius * arc.radius;
        double disc = qb * qb - 4.0 * qa * qc;
        // A line at distance r from the centre to rounding (an offset line touching the circle
        // of radius d about its own input vertex, where two collinear input lines meet) is one
        // tangent contact. The discriminant's cancellation leaves noise of either sign there:
        // positive noise gave two crossings ~1e-7 mm apart, and the sliver between them, dropped
        // by the pruning, left a gap the junction snap closed or bridged depending on its last
        // bits (with FMA contraction, GCC 15 at x86-64-v3, an extra span, T-0159). The distance
        // from the centre to the line is accurate to a few ulps of the coordinates, so within
        // that band the contact is tangent whatever the noise.
        const double h = std::fabs(w.cross(u)) / std::sqrt(qa);
        const double scale = std::max({1.0, arc.radius, std::fabs(line.start.x), std::fabs(line.start.y),
                                       std::fabs(line.end.x), std::fabs(line.end.y), std::fabs(arc.centre.x),
                                       std::fabs(arc.centre.y)});
        if (std::fabs(h - arc.radius) <= 64.0 * std::numeric_limits<double>::epsilon() * scale) {
            disc = 0.0;
        }
        if (disc < 0.0) {
            if (disc < -1e-9 * qa * arc.radius * arc.radius) {
                return out;
            }
            disc = 0.0;  // tangent contact
        }
        const double root = std::sqrt(disc);
        out.push_back(line.start + u * ((-qb - root) / (2.0 * qa)));
        if (root > 0.0) {
            out.push_back(line.start + u * ((-qb + root) / (2.0 * qa)));
        }
        return out;
    }
    const Vec2 delta = cb->centre - ca->centre;
    const double dist = delta.length();
    const double r1 = ca->radius;
    const double r2 = cb->radius;
    if (dist <= 1e-12 || dist > r1 + r2 + 1e-9 || dist < std::fabs(r1 - r2) - 1e-9) {
        return out;
    }
    const Vec2 u = delta / dist;
    // along = (r1^2 - r2^2 + D^2) / 2D from the first centre, written as r1 - s with
    // s = (r2^2 - (D - r1)^2) / 2D and h^2 = r1^2 - along^2 = s (2 r1 - s): without the
    // cancellation in r1^2 - along^2, which near tangency (a tiny circle touching a large one, as at a
    // U-turn of radius d (1 + eps)) cost ~1e-7 mm and left arc-to-arc junctions apart (T-0017).
    const double outside = dist - r1;
    const double s = (r2 - outside) * (r2 + outside) / (2.0 * dist);
    const double h = std::sqrt(std::max(0.0, s * (2.0 * r1 - s)));
    const Vec2 base = ca->centre + u * (r1 - s);
    out.push_back(base + u.perp() * h);
    if (h > 0.0) {
        out.push_back(base - u.perp() * h);
    }
    return out;
}

// Parameter of carrier point `p` relative to the span: 0 at the start, 1 at the end, linear in
// arc length, negative before the start and > 1 past the end. On an arc the gap outside the
// sweep is split between "past the end" and "before the start" by whichever is nearer.
double carrierParam(const Span& s, Vec2 p) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 u = line->end - line->start;
        return (p - line->start).dot(u) / u.lengthSquared();
    }
    const auto& arc = std::get<ArcSpan>(s);
    const double sweep = std::fabs(arc.sweepAngle());
    const double angle = (p - arc.centre).angle();
    const double phi = arc.direction == ArcDirection::CCW ? normalizeAngle(angle - arc.startAngle)
                                                          : normalizeAngle(arc.startAngle - angle);
    if (phi <= sweep) {
        return phi / sweep;
    }
    const double after = phi - sweep;
    const double before = kTwoPi - phi;
    return after < before ? phi / sweep : -before / sweep;
}

// Parameter of carrier point `p` for extending `s` past its end (`pastEnd`) or before its start:
// like carrierParam, but on an arc the WHOLE gap outside the sweep counts as extension in the
// requested direction (carrierParam splits it in half, which made an arc whose mitre point lay
// more than half the gap away fall back to Square/Round, T-0006). Nothing when reaching `p` would
// take the arc (within 1e-9 rad) all the way round to its own other end.
std::optional<double> extensionParam(const Span& s, Vec2 p, bool pastEnd) {
    if (std::holds_alternative<LineSpan>(s)) {
        return carrierParam(s, p);
    }
    const auto& arc = std::get<ArcSpan>(s);
    const double sweep = std::fabs(arc.sweepAngle());
    const double angle = (p - arc.centre).angle();
    const double phi = arc.direction == ArcDirection::CCW ? normalizeAngle(angle - arc.startAngle)
                                                          : normalizeAngle(arc.startAngle - angle);
    if (phi <= sweep) {
        return phi / sweep;
    }
    const double after = phi - sweep;    // travelled past the end
    const double before = kTwoPi - phi;  // travelled back from the start
    const double gap = kTwoPi - sweep;
    if ((pastEnd ? after : before) > gap - 1e-9) {
        return std::nullopt;
    }
    return pastEnd ? 1.0 + after / sweep : -before / sweep;
}

// Same carrier, end moved to `p` (which must lie on the carrier).
Span withEnd(Span s, Vec2 p) {
    if (auto* line = std::get_if<LineSpan>(&s)) {
        line->end = p;
    } else {
        auto& arc = std::get<ArcSpan>(s);
        arc.endAngle = (p - arc.centre).angle();
    }
    return s;
}

// Same carrier, start moved to `p` (which must lie on the carrier).
Span withStart(Span s, Vec2 p) {
    if (auto* line = std::get_if<LineSpan>(&s)) {
        line->start = p;
    } else {
        auto& arc = std::get<ArcSpan>(s);
        arc.startAngle = (p - arc.centre).angle();
    }
    return s;
}

bool inUnit(double t) {
    return t >= -kParamEps && t <= 1.0 + kParamEps;
}

// Point of carrier contact between `a` (ending) and `b` (starting) beyond a's end and before
// b's start, within `maxDist` of `vertex`: where the two extensions first meet (smallest total
// extension length). An arc may be extended over the whole gap outside its sweep.
// Cap on extending an arc to a mitre or chamfer (T-0006): the extension may sweep at most a
// quarter turn and its new end point must stay within `arcReach` (mitreLimit x d) of the corner
// vertex. Beyond that a tiny offset arc would be dragged round much of its circle to meet the
// other side, which is no longer a corner. Lines are bounded by the callers' own reach.
constexpr double kMaxArcExtension = kPi / 2.0;

bool arcExtensionOk(const Span& s, double t, bool pastEnd, Vec2 p, Vec2 vertex, double arcReach) {
    const auto* arc = std::get_if<ArcSpan>(&s);
    if (!arc) {
        return true;
    }
    const double sweep = std::fabs(arc->sweepAngle()) * (pastEnd ? t - 1.0 : -t);
    return sweep <= kMaxArcExtension + 1e-12 && geometry::distance(p, vertex) <= arcReach;
}

std::optional<Vec2> extensionMeet(const Span& a, const Span& b, Vec2 vertex, double maxDist,
                                  double arcReach) {
    std::optional<Vec2> best;
    double bestCost = std::numeric_limits<double>::infinity();
    const double la = length(a);
    const double lb = length(b);
    for (const Vec2 p : carrierIntersections(a, b)) {
        const std::optional<double> ta = extensionParam(a, p, true);
        const std::optional<double> tb = extensionParam(b, p, false);
        if (!ta || !tb || *ta < 1.0 - kParamEps || *tb > kParamEps) {
            continue;
        }
        if (geometry::distance(p, vertex) > maxDist || !arcExtensionOk(a, *ta, true, p, vertex, arcReach) ||
            !arcExtensionOk(b, *tb, false, p, vertex, arcReach)) {
            continue;
        }
        const double cost = std::max(0.0, *ta - 1.0) * la + std::max(0.0, -*tb) * lb;
        if (cost < bestCost) {
            bestCost = cost;
            best = p;
        }
    }
    return best;
}

// Square join: extends `a` and `b` to the chamfer line at distance d from the vertex and
// returns the chamfer; nullopt when an element cannot reach it (caller falls back to Round).
std::optional<LineSpan> squareJoin(Span& a, Span& b, Vec2 vertex, Vec2 tangentIn, double d, double arcReach) {
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    Vec2 bisector = ((e - vertex) + (s - vertex)).normalized();
    if (bisector.lengthSquared() == 0.0) {
        bisector = tangentIn;  // full reversal: cap perpendicular to the direction of travel
    }
    const Vec2 q = vertex + bisector * d;
    const Span chamferCarrier = LineSpan{q, q + bisector.perp()};
    const double reach = 1.5 * d;  // chamfer ends lie within d / cos(45 deg) of the vertex
    // First hit of each extension (an arc's may use the whole gap outside its sweep).
    std::optional<Vec2> h1;
    double t1 = 0.0;
    for (const Vec2 p : carrierIntersections(a, chamferCarrier)) {
        const std::optional<double> t = extensionParam(a, p, true);
        if (t && *t >= 1.0 - kParamEps && geometry::distance(p, vertex) <= reach &&
            arcExtensionOk(a, *t, true, p, vertex, arcReach) && (!h1 || *t < t1)) {
            h1 = p;
            t1 = *t;
        }
    }
    std::optional<Vec2> h2;
    double t2 = 0.0;
    for (const Vec2 p : carrierIntersections(b, chamferCarrier)) {
        const std::optional<double> t = extensionParam(b, p, false);
        if (t && *t <= kParamEps && geometry::distance(p, vertex) <= reach &&
            arcExtensionOk(b, *t, false, p, vertex, arcReach) && (!h2 || *t > t2)) {
            h2 = p;
            t2 = *t;
        }
    }
    if (!h1 || !h2 || (*h2 - *h1).dot(s - e) < 0.0) {
        return std::nullopt;
    }
    a = withEnd(a, *h1);
    b = withStart(b, *h2);
    return LineSpan{*h1, *h2};
}

// The closed region a Sharp or Square join adds beyond the round d-neighbourhood at one
// corner: vertex -> offset end of the incoming element -> its extension -> (chamfer) -> the
// outgoing element's extension -> its offset start -> vertex. Raw pieces strictly inside it are
// interior to the offset shape (e.g. the stub of a neighbour running past a mitre) and pruned.
using Wedge = std::vector<Span>;

// Piece of `carrier`'s line or circle from `from` to `to` (both on it), or nothing when the two
// points coincide (an arc with equal angles would read as a full circle).
void appendCarrierPiece(Wedge& w, const Span& carrier, Vec2 from, Vec2 to) {
    if (geometry::distance(from, to) <= 1e-12 * std::max(1.0, geometry::distance(from, Vec2{}))) {
        return;
    }
    w.push_back(withStart(withEnd(carrier, to), from));
}

// Joins `prev` (the last raw element) to `next` around an outside corner at `vertex`. Sharp and
// Square modify the two elements in place (extension) and record the region they add in
// `wedges`; any join element is appended to `raw`.
void outsideCorner(std::vector<Span>& raw, Span& next, Vec2 vertex, Vec2 prevEnd, Vec2 tangentIn, double d,
                   double sign, JoinType join, double mitreLimit, const Tolerance& tol,
                   std::vector<Wedge>& wedges) {
    Span& prev = raw.back();
    const Vec2 e = endPoint(prev);
    const Vec2 s = startPoint(next);
    const double arcReach = std::max(mitreLimit, 1.0) * d;
    auto mitre = [&](double reach) {
        const auto m = extensionMeet(prev, next, vertex, reach, arcReach);
        if (!m) {
            return false;
        }
        Wedge w{LineSpan{vertex, e}};
        appendCarrierPiece(w, prev, e, *m);
        appendCarrierPiece(w, next, *m, s);
        w.emplace_back(LineSpan{s, vertex});
        wedges.push_back(std::move(w));
        prev = withEnd(prev, *m);
        next = withStart(next, *m);
        return true;
    };
    if (join == JoinType::Sharp && mitre(std::max(mitreLimit, 1.0) * d)) {
        return;
    }
    // Sharp beyond the mitre limit (or where the carriers never meet) squares off the corner.
    if (join != JoinType::Round) {
        Span a = prev;
        Span b = next;
        if (const auto chamfer = squareJoin(a, b, vertex, tangentIn, d, arcReach)) {
            // A chamfer shorter than the tolerance (a nearly tangent corner) would be dropped
            // and leave a gap; meeting the extended neighbours there is the same corner.
            if (tol.equalPoints(chamfer->start, chamfer->end) && mitre(1.5 * d)) {
                return;
            }
            Wedge w{LineSpan{vertex, e}};
            appendCarrierPiece(w, prev, e, chamfer->start);
            w.emplace_back(*chamfer);
            appendCarrierPiece(w, next, chamfer->end, s);
            w.emplace_back(LineSpan{s, vertex});
            wedges.push_back(std::move(w));
            prev = a;
            next = b;
            if (!tol.equalPoints(chamfer->start, chamfer->end)) {
                raw.emplace_back(*chamfer);
            }
            return;
        }
        // An arc that cannot reach the chamfer line: fall back to a round corner.
    }
    const ArcDirection dir = sign > 0.0 ? ArcDirection::CW : ArcDirection::CCW;
    // Input with a gap at this vertex (the previous span ends at `prevEnd`, not at `vertex`):
    // one arc about `vertex` through `e` has radius d + (up to) the gap and dips inside d of
    // `prevEnd`, by ~1e-8 mm for a 5e-6 mm gap, enough for the pruning to cut that stretch out
    // and break the chain or leave the corner open (T-0017 0xe30005b6). The corner's boundary
    // is that of the two discs of radius d about `prevEnd` and `vertex`, so both turn arcs go
    // in: about `prevEnd` from `e`, and about `vertex` to `s`. The split and pruning steps keep
    // the outer stretch of each (whichever way the gap runs), every point of it exactly d from
    // its centre; the raw chain jumps by the gap between them, as it does across the input gap.
    const double gap = geometry::distance(prevEnd, vertex);
    const double scale = 1e-9 * std::max({1.0, d, std::fabs(e.x), std::fabs(e.y)});
    const double ulp =
        16.0 * std::numeric_limits<double>::epsilon() * std::max({1.0, std::fabs(e.x), std::fabs(e.y)});
    if (gap > ulp && gap < d && std::fabs(geometry::distance(prevEnd, e) - d) <= scale &&
        std::fabs(geometry::distance(vertex, s) - d) <= scale) {
        const Vec2 nIn = (e - prevEnd) * (1.0 / geometry::distance(prevEnd, e));
        const Vec2 nOut = (s - vertex) * (1.0 / geometry::distance(vertex, s));
        raw.emplace_back(ArcSpan::fromCentre(prevEnd, e, prevEnd + nOut * d, dir));
        raw.emplace_back(ArcSpan::fromCentre(vertex, vertex + nIn * d, s, dir));
        return;
    }
    raw.emplace_back(ArcSpan::fromCentre(vertex, e, s, dir));
}

enum class CornerKind { Continuous, Outside, Inside };

CornerKind classifyCorner(Vec2 tangentIn, Vec2 tangentOut, double sign) {
    const double cross = tangentIn.cross(tangentOut);
    const double dot = tangentIn.dot(tangentOut);
    if (std::fabs(cross) <= 1e-9) {
        return dot > 0.0 ? CornerKind::Continuous : CornerKind::Outside;  // reversal: cap it
    }
    return sign * cross < 0.0 ? CornerKind::Outside : CornerKind::Inside;
}

// Connects the raw chain to `next` across the corner at `vertex`.
void connect(std::vector<Span>& raw, Span& next, Vec2 vertex, Vec2 prevEnd, Vec2 tangentIn, Vec2 tangentOut,
             double d, double sign, JoinType join, double mitreLimit, const Tolerance& tol,
             std::vector<Wedge>& wedges) {
    const Vec2 e = endPoint(raw.back());
    if (tol.equalPoints(e, startPoint(next))) {
        return;
    }
    if (classifyCorner(tangentIn, tangentOut, sign) == CornerKind::Outside) {
        outsideCorner(raw, next, vertex, prevEnd, tangentIn, d, sign, join, mitreLimit, tol, wedges);
    } else {
        // Inside corner (or a gap left by an inverted arc): a straight connector. It lies within
        // d of the vertex, so it is always discarded by the pruning step.
        raw.emplace_back(LineSpan{e, startPoint(next)});
    }
}

// Exact distance from `p` to a line or arc span.
double distanceToSpan(Vec2 p, const Span& s) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 ab = line->end - line->start;
        const double len2 = ab.lengthSquared();
        const double t = len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0;
        return geometry::distance(p, line->start + ab * t);
    }
    const auto& arc = std::get<ArcSpan>(s);
    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
        return std::fabs(geometry::distance(p, arc.centre) - arc.radius);
    }
    return std::min(geometry::distance(p, arc.startPoint()), geometry::distance(p, arc.endPoint()));
}

// Exactly `distanceToSpan(p, s) < limit`. For a line the squared distance decides everything
// except within a relative 1e-12 of the limit (std::hypot, which distanceToSpan uses, and a plain
// square sum differ by an ulp or two); only there is the exact distance computed. hypot was a
// quarter of the time of an offset with nothing to prune (T-0170).
bool nearerThan(Vec2 p, const Span& s, double limit) {
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const double l2 = limit * limit;
        if (l2 > 1e-280 && l2 < 1e280) {
            const Vec2 ab = line->end - line->start;
            const double len2 = ab.lengthSquared();
            const double t = len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0;
            const Vec2 r = p - (line->start + ab * t);
            const double d2 = r.lengthSquared();
            if (d2 < l2 * (1.0 - 1e-12)) {
                return true;
            }
            if (d2 > l2 * (1.0 + 1e-12)) {
                return false;
            }
        }
    }
    if (const auto* arc = std::get_if<ArcSpan>(&s)) {
        // T-0176: every point of the arc lies on its circle, so the distance is at least
        // |rho - r| (rho = distance of p from the centre), whichever branch distanceToSpan takes.
        // When that is clearly at least `limit` the answer is "no" without the angle (atan2,
        // two fmods) and the two hypots. The margin covers the ulps of the square root and of
        // the endpoints' coordinates (computed from angles) at the scale of the inputs.
        if (detail::arcClearlyFarther(p, arc->centre, arc->radius, limit)) {
            return false;
        }
    }
    return distanceToSpan(p, s) < limit;
}

// Sub-span of a line or arc between parameters t0 < t1 (arc-length linear).
Span subSpan(const Span& s, double t0, double t1) {
    if (t0 <= 0.0 && t1 >= 1.0) {
        return s;
    }
    const Vec2 a = pointAt(s, t0);
    const Vec2 b = pointAt(s, t1);
    return withStart(withEnd(s, b), a);
}

// True when two line or arc elements lie on the same infinite line or the same circle.
bool sameCarrier(const Span& a, const Span& b) {
    const auto* la = std::get_if<LineSpan>(&a);
    const auto* lb = std::get_if<LineSpan>(&b);
    if (la && lb) {
        const Vec2 u = la->end - la->start;
        const double c0 = std::fabs(u.cross(lb->start - la->start));
        const double c1 = std::fabs(u.cross(lb->end - la->start));
        // Same test as with len = u.length() below, decided with sqrt unless a cross product is
        // within 1e-9 of the bound (hypot was a visible share of the time, T-0170).
        const double approx = std::sqrt(u.lengthSquared());
        const double bound = 1e-9 * std::max(1.0, approx) * approx;
        if (bound > 1e-140 && bound < 1e140) {
            if (c0 < bound * (1.0 - 1e-9) && c1 < bound * (1.0 - 1e-9)) {
                return true;
            }
            if (c0 > bound * (1.0 + 1e-9) || c1 > bound * (1.0 + 1e-9)) {
                return false;
            }
        }
        const double len = u.length();
        const double scale = 1e-9 * std::max(1.0, len);
        return c0 <= scale * len && c1 <= scale * len;
    }
    const auto* ca = std::get_if<ArcSpan>(&a);
    const auto* cb = std::get_if<ArcSpan>(&b);
    if (ca && cb) {
        const double scale = 1e-9 * std::max(1.0, ca->radius);
        return geometry::distance(ca->centre, cb->centre) <= scale &&
               std::fabs(ca->radius - cb->radius) <= scale;
    }
    return false;
}

// Uniform grid over a fixed set of boxes, used to find candidate pairs for the intersection,
// split and pruning steps in roughly linear time instead of testing every pair. The cell size
// is chosen so that the grid holds about as many cells as boxes (capped at 1024 per axis); a
// query visits the cells its box overlaps and reports each item once, in no particular order
// (callers only accumulate results, so the order never affects the output).
class BoxGrid {
   public:
    explicit BoxGrid(const std::vector<Box2>& boxes, double maxCells = kMaxCells)
        : boxes_(boxes), stamp_(boxes.size(), 0), maxCells_(maxCells) {
        for (const Box2& b : boxes_) {
            extent_.expand(b);
        }
        filled_ = extent_;
        if (boxes_.empty() || extent_.isEmpty()) {
            return;
        }
        layOut(boxes_.size());
        for (std::size_t i = 0; i < boxes_.size(); ++i) {
            insert(i);
        }
    }

    // Calls f(i) once for every item whose box intersects `query`.
    template <class F>
    void query(const Box2& query, F&& f) {
        queryUntil(query, [&](std::size_t i) {
            f(i);
            return false;
        });
    }

    // Like query(), but stops as soon as f(i) returns true; returns whether it did.
    template <class F>
    bool queryUntil(const Box2& query, F&& f) {
        if (cells_.empty() || !query.intersects(filled_)) {
            return false;
        }
        ++epoch_;
        const auto [x0, x1, y0, y1] = cellRange(query);
        for (std::size_t y = y0; y <= y1; ++y) {
            for (std::size_t x = x0; x <= x1; ++x) {
                for (const std::size_t i : cells_[y * nx_ + x]) {
                    if (stamp_[i] != epoch_ && boxes_[i].intersects(query)) {
                        stamp_[i] = epoch_;
                        if (f(i)) {
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    // Side of the smaller dimension of a cell (0 when the grid is empty).
    double cellSize() const { return cells_.empty() ? 0.0 : std::min(cw_, ch_); }

   private:
    static constexpr double kMaxCells = 1024.0;

    // Cells of about the size that gives as many cells as `n` boxes over extent_.
    void layOut(std::size_t n) {
        const double w = extent_.width();
        const double h = extent_.height();
        double cell = std::sqrt(std::max(w * h, 1e-300) / static_cast<double>(n));
        // Boxes much larger than a cell (dense input at a large distance: every corner region
        // and long raw line spans a good part of the grid) were inserted into hundreds of cells
        // each, which grew with the square of their number; cells of at least a quarter of the
        // mean box side keep that to ~25 cells per box and a query to the boxes it meets.
        double sideW = 0.0;
        double sideH = 0.0;
        std::size_t counted = 0;
        for (const Box2& b : boxes_) {
            if (!b.isEmpty()) {
                sideW += b.width();
                sideH += b.height();
                ++counted;
            }
        }
        if (counted > 0) {
            cell = std::max({cell, sideW / (4.0 * static_cast<double>(counted)),
                             sideH / (4.0 * static_cast<double>(counted))});
        }
        cell = std::max({cell, w / maxCells_, h / maxCells_, 1e-9});
        nx_ = static_cast<std::size_t>(std::clamp(std::ceil(w / cell), 1.0, maxCells_));
        ny_ = static_cast<std::size_t>(std::clamp(std::ceil(h / cell), 1.0, maxCells_));
        cw_ = w > 0.0 ? w / static_cast<double>(nx_) : 1.0;
        ch_ = h > 0.0 ? h / static_cast<double>(ny_) : 1.0;
        cells_.resize(nx_ * ny_);
    }

    void insert(std::size_t i) {
        if (boxes_[i].isEmpty()) {
            return;  // never reported
        }
        const auto [x0, x1, y0, y1] = cellRange(boxes_[i]);
        for (std::size_t y = y0; y <= y1; ++y) {
            for (std::size_t x = x0; x <= x1; ++x) {
                cells_[y * nx_ + x].push_back(i);
            }
        }
    }

    struct Range {
        std::size_t x0, x1, y0, y1;
    };

    std::size_t index(double v, double origin, double size, std::size_t count) const {
        const double k = std::floor((v - origin) / size);
        return static_cast<std::size_t>(std::clamp(k, 0.0, static_cast<double>(count - 1)));
    }

    Range cellRange(const Box2& b) const {
        return {index(b.min.x, extent_.min.x, cw_, nx_), index(b.max.x, extent_.min.x, cw_, nx_),
                index(b.min.y, extent_.min.y, ch_, ny_), index(b.max.y, extent_.min.y, ch_, ny_)};
    }

    std::vector<Box2> boxes_;
    Box2 extent_;  // the cell layout
    Box2 filled_;  // the boxes held
    std::size_t nx_{0};
    std::size_t ny_{0};
    double cw_{1.0};
    double ch_{1.0};
    std::vector<std::vector<std::size_t>> cells_;
    std::vector<std::uint64_t> stamp_;
    std::uint64_t epoch_{0};
    double maxCells_{kMaxCells};
};

// True when some input span is closer than `limit` to `p`. `witness` is the span that last
// answered yes; it is tried first and updated, because neighbouring samples along the raw chain
// are almost always rejected by the same span (most raw pieces of a dense path at a large
// distance lie deep inside the neighbourhood). Without it each rejected sample visited every
// span within `limit`, which dominated the cost at large d (T-0006). Only spans whose boxes meet
// the query square can be closer than `limit`, so the answer is exactly that of a full scan.
bool closerThan(Vec2 p, double limit, const std::vector<Span>& input, BoxGrid& grid, std::size_t& witness) {
    if (witness < input.size() && nearerThan(p, input[witness], limit)) {
        return true;
    }
    return grid.queryUntil(Box2{p - Vec2{limit, limit}, p + Vec2{limit, limit}}, [&](std::size_t i) {
        if (i != witness && nearerThan(p, input[i], limit)) {
            witness = i;
            return true;
        }
        return false;
    });
}

// Points whose convex hull contains the line or arc `s`: a line's end points; for an arc, the
// end points of sub-arcs of at most 90 degrees plus, for each, the meeting point of its end
// tangents (at radius r / cos(half sweep) on the bisector).
void hullPoints(const Span& s, std::vector<Vec2>& out) {
    out.clear();
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        out.push_back(line->start);
        out.push_back(line->end);
        return;
    }
    const auto& arc = std::get<ArcSpan>(s);
    const double sweep = arc.sweepAngle();
    const int parts = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / (kPi / 2.0))));
    const double step = sweep / parts;
    const double apex = arc.radius / std::cos(0.5 * step);
    out.push_back(arc.startPoint());
    for (int k = 0; k < parts; ++k) {
        const double a0 = arc.startAngle + step * k;
        out.push_back(arc.centre + Vec2::fromAngle(a0 + 0.5 * step, apex));
        out.push_back(arc.centre + Vec2::fromAngle(a0 + step, arc.radius));
    }
}

// A sub-range [t0, t1] of one raw element.
struct Piece {
    std::size_t element;
    double t0;
    double t1;
};

// Arc whose offset towards its centre would have a negative radius, reflected through the
// centre (radius d - r): the part of the d-neighbourhood boundary that lies beyond the centre.
std::optional<Span> invertedOffset(const Span& s, double d, double sign) {
    const auto* arc = std::get_if<ArcSpan>(&s);
    if (!arc) {
        return std::nullopt;
    }
    const double towardsCentre = arc->direction == ArcDirection::CCW ? sign : -sign;
    ArcSpan inverted = *arc;
    inverted.radius = towardsCentre * d - arc->radius;
    if (inverted.radius <= collapseRadius(*arc, d)) {
        return std::nullopt;
    }
    inverted.startAngle += kPi;
    inverted.endAngle += kPi;
    return Span{inverted};
}

// Points bucketed on a grid of a few tolerances, to find the ones within tol.distance of a
// query point without scanning them all.
class PointIndex {
   public:
    explicit PointIndex(double tolDistance) : tol_(tolDistance), cell_(std::max(4.0 * tolDistance, 1e-9)) {}

    void add(Vec2 p, std::size_t id) { map_[key(p)].push_back({p, id}); }
    void clear() { map_.clear(); }

    // Calls f(id) for every stored point within tolerance of `p`.
    template <class F>
    void forEachNear(Vec2 p, F&& f) const {
        const auto [cx, cy] = key(p);
        for (long long dy = -1; dy <= 1; ++dy) {
            for (long long dx = -1; dx <= 1; ++dx) {
                const auto it = map_.find({cx + dx, cy + dy});
                if (it == map_.end()) {
                    continue;
                }
                for (const auto& [q, id] : it->second) {
                    if (geometry::distance(q, p) <= tol_) {
                        f(id);
                    }
                }
            }
        }
    }

    // Smallest id stored within tolerance of `p`, if any.
    std::optional<std::size_t> findFirst(Vec2 p) const {
        std::optional<std::size_t> best;
        forEachNear(p, [&](std::size_t id) {
            if (!best || id < *best) {
                best = id;
            }
        });
        return best;
    }

   private:
    std::pair<long long, long long> key(Vec2 p) const {
        return {static_cast<long long>(std::floor(p.x / cell_)),
                static_cast<long long>(std::floor(p.y / cell_))};
    }

    double tol_;
    double cell_;
    std::map<std::pair<long long, long long>, std::vector<std::pair<Vec2, std::size_t>>> map_;
};

// True when `p` lies inside the closed curve `edges` (lines and arcs, in order), by counting
// crossings of the ray from `p` towards +x. Arcs are cut at their top and bottom points so that
// every piece is monotone in y and crosses the ray at most once.
bool insideRegion(Vec2 p, const Wedge& edges) {
    bool inside = false;
    auto crossLine = [&](Vec2 a, Vec2 b) {
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x > p.x) {
                inside = !inside;
            }
        }
    };
    for (const Span& s : edges) {
        if (const auto* line = std::get_if<LineSpan>(&s)) {
            crossLine(line->start, line->end);
            continue;
        }
        const auto& arc = std::get<ArcSpan>(s);
        const double a0 = arc.startAngle;
        const double a1 = arc.startAngle + arc.sweepAngle();
        const double lo = std::min(a0, a1);
        const double hi = std::max(a0, a1);
        std::vector<double> angles{lo};
        for (double k = std::ceil((lo - kPi / 2.0) / kPi); kPi / 2.0 + k * kPi < hi; k += 1.0) {
            const double theta = kPi / 2.0 + k * kPi;
            if (theta > lo) {
                angles.push_back(theta);
            }
        }
        angles.push_back(hi);
        for (std::size_t k = 0; k + 1 < angles.size(); ++k) {
            const double y0 = arc.centre.y + arc.radius * std::sin(angles[k]);
            const double y1 = arc.centre.y + arc.radius * std::sin(angles[k + 1]);
            if ((y0 > p.y) == (y1 > p.y)) {
                continue;
            }
            const double dy = p.y - arc.centre.y;
            const double half = std::sqrt(std::max(0.0, arc.radius * arc.radius - dy * dy));
            const double x =
                arc.centre.x + (std::cos(0.5 * (angles[k] + angles[k + 1])) >= 0.0 ? half : -half);
            if (x > p.x) {
                inside = !inside;
            }
        }
    }
    return inside;
}

// Moves the end of `a` and the start of `b` to the common point `j` (on both carriers), unless
// that would reverse either span or wrap an arc round to its other end; returns whether it did.
bool moveJunction(Span& a, Span& b, Vec2 j) {
    const std::optional<double> ta = extensionParam(a, j, true);
    const std::optional<double> tb = extensionParam(b, j, false);
    if (!ta || !tb || *ta <= kParamEps || *tb >= 1.0 - kParamEps) {
        return false;
    }
    a = withEnd(a, j);
    b = withStart(b, j);
    return true;
}

// What the junction snapping needs from the offset: the side, the tolerance, and a test that a
// point is at least d (to the pruning margin) from the input.
struct SnapContext {
    double sign;
    const Tolerance& tol;
    std::function<bool(Vec2)> clear;   // at least d from the input (to the snapping margin)
    std::function<bool(Vec2)> notFar;  // Round only: at most d (to the same margin) from it
    std::function<double(Vec2)> distance;  // to the input (infinity if further than d + 2 tol)
    std::function<Vec2(Vec2)> nearest;     // closest input point (the point itself if none near)
    double d;
    double allowance;  // how far snapped extensions and bends may come inside d
    bool round;
    // True when the stretch of `carrier`'s line or circle from `from` to `to` runs along a kept
    // piece of another chain (or another part of this one): extending onto it would traverse
    // that stretch twice.
    std::function<bool(const Span& carrier, Vec2 from, Vec2 to)> occupied;
};

// How far beyond d a Round offset point lies (0 for the other joins, or inside d).
double beyond(const SnapContext& ctx, Vec2 p) {
    return ctx.round ? std::max(0.0, ctx.distance(p) - ctx.d) : 0.0;
}

// True when every point of the curve point(t), t in [0, 1], passes `accept`: checked at n + 1
// samples (t = k / n, the first one only if `withStart`), and at each interior local maximum and
// minimum of the distance to the input found by golden-section search between the samples next
// to it. The distance along a straight or circular stretch is the smallest of the distances to the
// input spans near it, each smooth: it peaks sharply where the nearest span changes, which a
// fixed sampling misses (T-0017: a Round bend 4.2e-7 mm beyond d between samples tol / 8 apart,
// 0xd4001762).
template <class PointAt, class Accept>
bool profileOk(PointAt point, int n, bool withStart, const SnapContext& ctx, Accept accept) {
    std::vector<double> dist(static_cast<std::size_t>(n) + 1);
    for (int k = 0; k <= n; ++k) {
        const Vec2 q = point(static_cast<double>(k) / n);
        if ((k > 0 || withStart) && !accept(q)) {
            return false;
        }
        dist[static_cast<std::size_t>(k)] = ctx.distance(q);
    }
    for (int k = 1; k < n; ++k) {
        const double here = dist[static_cast<std::size_t>(k)];
        const double before = dist[static_cast<std::size_t>(k) - 1];
        const double after = dist[static_cast<std::size_t>(k) + 1];
        const bool peak = here >= before && here >= after;
        const bool dip = here <= before && here <= after;
        if (!(peak || dip) || !std::isfinite(here)) {
            continue;
        }
        // A flat stretch (an arc about a vertex, a line along its input line: all three samples
        // agree to rounding) has no extremum worth refining; searching each of its samples made
        // Round offsets of dense polylines 3-6x slower.
        const Vec2 at = point(static_cast<double>(k) / n);
        const double noise = 1e-12 * std::max({1.0, ctx.d, std::fabs(at.x), std::fabs(at.y)});
        if (std::fabs(here - before) <= noise && std::fabs(here - after) <= noise) {
            continue;
        }
        // Golden-section search for the extremum on [t(k-1), t(k+1)].
        double lo = static_cast<double>(k - 1) / n;
        double hi = static_cast<double>(k + 1) / n;
        constexpr double kGolden = 0.6180339887498949;
        auto score = [&](double t) { return (peak ? 1.0 : -1.0) * ctx.distance(point(t)); };
        double x1 = hi - kGolden * (hi - lo);
        double x2 = lo + kGolden * (hi - lo);
        double f1 = score(x1);
        double f2 = score(x2);
        for (int it = 0; it < 40; ++it) {
            if (f1 >= f2) {
                hi = x2;
                x2 = x1;
                f2 = f1;
                x1 = hi - kGolden * (hi - lo);
                f1 = score(x1);
            } else {
                lo = x1;
                x1 = x2;
                f1 = f2;
                x2 = lo + kGolden * (hi - lo);
                f2 = score(x2);
            }
        }
        if (!accept(point(f1 >= f2 ? x1 : x2))) {
            return false;
        }
    }
    return true;
}

// True when the stretch of `carrier`'s line or circle from `from` to `to` (both on it, the
// short way round) passes ctx.clear and ctx.notFar everywhere (profileOk, samples no more than
// tol / 8 apart), `from` itself excepted (already on the offset); with a non-negative `farSlack`
// a Round point may also lie up to that far beyond d. Checking only the far end and the middle
// let a Round extension next to a wiggly input reach 1.3e-6 mm beyond d between them: there the
// distance to the input is not monotone along the stretch (T-0017, gen3 0x9300156).
bool stretchClear(const Span* carrier, Vec2 from, Vec2 to, const SnapContext& ctx, double farSlack = -1.0) {
    const double len = geometry::distance(from, to);
    const int n = static_cast<int>(std::clamp(std::ceil(len / (0.125 * ctx.tol.distance)), 2.0, 512.0));
    const auto* arc = carrier ? std::get_if<ArcSpan>(carrier) : nullptr;
    auto point = [&](double t) {
        Vec2 q = lerp(from, to, t);
        if (arc && q != arc->centre) {
            q = arc->centre + (q - arc->centre).normalized() * arc->radius;
        }
        return q;
    };
    return profileOk(point, n, false, ctx, [&](Vec2 q) {
        return ctx.clear(q) && (ctx.notFar(q) || (farSlack >= 0.0 && beyond(ctx, q) <= farSlack));
    });
}

// Geometric snap of the junction between `a` and `b`, consecutive spans of an output chain whose
// end points agree to the tolerance. Both are trimmed or extended along their own carriers to
// where those meet, so the junction lies on both. That is only done where it keeps both spans on
// the offset: a trimmed span stays on itself, and an extension (e.g. the mitre of a corner whose
// sub-tolerance join piece was dropped, at most about tol^2 / (8d) beyond d) must be at least d
// from the input (and, for Round, not beyond d), within a few gaps of the old ends. Gaps of
// rounding size need no such care: a line end moves onto the other span's end, or an arc end
// onto the angle of a line's end. Returns false, leaving both spans unchanged, where the
// junction cannot be closed that way (e.g. a sub-tolerance S-step: the carriers meet only
// behind one span and ahead of the other).
bool snapJunction(Span& a, Span& b, const SnapContext& ctx) {
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    if (e == s) {
        return true;
    }
    const double gap = geometry::distance(e, s);
    // A few units in the last place of the coordinates: two computations of one point. Not
    // 1e-9 x |x|, which at 1e3..1e4 mm swallowed the 2 eps d gap of a U-turn of radius d (1 + eps)
    // and tilted a line by it (T-0017).
    const double rounding =
        16.0 * std::numeric_limits<double>::epsilon() * std::max({1.0, std::fabs(e.x), std::fabs(e.y)});
    const double reach = std::max(2.0 * ctx.tol.distance, 4.0 * gap);
    const Vec2 mid = (e + s) * 0.5;
    std::optional<Vec2> best;
    double bestGap = kInf;
    int bestExtensions = 3;
    // A candidate reached with fewer extensions (trimming) wins unless the other is under half as far:
    // where two circles cross twice close to a reversal (Round arcs about vertices almost exactly 2d
    // apart), meeting at the crossing that needs extensions leaves the arcs crossing at the other
    // one (T-0017, gen3 0x9300047).
    const double tie = std::max(gap, rounding);
    auto preferred = [&](double nearness, int extensions) {
        if (!best) {
            return true;
        }
        if (extensions < bestExtensions) {
            return nearness <= 2.0 * bestGap + tie;  // trimming wins unless twice as far
        }
        if (extensions > bestExtensions) {
            return 2.0 * nearness + tie < bestGap;
        }
        return nearness < bestGap;
    };
    for (const Vec2 p : carrierIntersections(a, b)) {
        const std::optional<double> ta = extensionParam(a, p, true);
        const std::optional<double> tb = extensionParam(b, p, false);
        if (!ta || !tb || geometry::distance(p, e) > reach || geometry::distance(p, s) > reach) {
            continue;
        }
        const int extensions = (*ta > 1.0 ? 1 : 0) + (*tb < 0.0 ? 1 : 0);
        const double nearness = geometry::distance(p, mid);
        if (!preferred(nearness, extensions)) {
            continue;
        }
        // Trimming keeps a span on itself. An extension beyond rounding size must stay clear of
        // the input (the dropped piece may have been a stretch of another part's offset cutting
        // across this corner) and, for Round, not stray beyond d.
        // Only a move at the level of the coordinates' last bits (`rounding`) goes unchecked: at
        // 1e4 mm a move of 1e-9 x |x| = 1e-5 mm at a steep angle to another part of the input came
        // 9.4e-7 mm inside d (T-0017, gen3 0x9400bf7).
        // Nor may an extension run along a piece of another chain: closing a three-span loop by
        // extending a line over the start of the next chain duplicated 1.5e-3 mm of it and poked
        // the corner 1.1e-3 mm past the offset (T-0017 round 3, B-2, fam5 0xf500163e).
        auto extensionOk = [&](const Span& span, Vec2 end) {
            return geometry::distance(p, end) <= rounding ||
                   (stretchClear(&span, end, p, ctx) && !ctx.occupied(span, end, p));
        };
        if ((*ta <= 1.0 || extensionOk(a, e)) && (*tb >= 0.0 || extensionOk(b, s))) {
            bestGap = nearness;
            bestExtensions = extensions;
            best = p;
        }
    }
    if (best) {
        Span ma = a;
        Span mb = b;
        if (moveJunction(ma, mb, *best)) {
            // Two arcs whose circles only nearly touch (a Round cusp between vertices 2d + g
            // apart, g ~ 1e-10) meet at a point on neither circle: their recomputed ends stay up
            // to g apart. Beyond what exactJunction closes (1e-10 x the scale) that is a gap,
            // bridged below instead (T-0017 round 3, B-1).
            const auto* arcA = std::get_if<ArcSpan>(&ma);
            const auto* arcB = std::get_if<ArcSpan>(&mb);
            const Vec2 j = endPoint(ma);
            if (!arcA || !arcB ||
                geometry::distance(j, startPoint(mb)) <=
                    1e-10 * std::max({1.0, std::fabs(j.x), std::fabs(j.y), arcB->radius})) {
                a = ma;
                b = mb;
                return true;
            }
        }
    }
    // A line end may also move onto the other span's end across a gap up to 1e-9 x |x| (two
    // nearly collinear chamfer copies 4e-14 mm apart at ~9 mm, where a bridge would be a
    // zero-length span, fam2 0xe2000055), but only where the tilted line passes the checks below.
    if (gap > 1e-9 * std::max({1.0, std::fabs(e.x), std::fabs(e.y)})) {
        return false;
    }
    // Moving a line end tilts the whole line: the end that moves outwards (away from the input)
    // is preferred, and either move must leave the line clear of the input (and for Round not
    // beyond d) along it (profileOk: samples at most tol / 8 apart and the distance's local
    // extremes between them).
    const bool lineA = std::holds_alternative<LineSpan>(a);
    const bool lineB = std::holds_alternative<LineSpan>(b);
    const bool bOutwards = (e - s).dot(tangentAt(b, 0.0).perp() * ctx.sign) >= 0.0;
    // The tilt grows linearly towards the moved end, and a kept line comes nearest the limits at
    // its ends (where it was split against another boundary), so the samples crowd towards the
    // moved end (t -> 1 - (1 - t)^3): a fixed 4 samples let a Sharp line at 1e4 mm come 3.8e-6 mm
    // inside d just before its end (hp sweep), and tol / 8 spacing along the whole line made the
    // fuzz families 4x slower.
    auto lineOk = [&](Vec2 from, Vec2 to) {
        const double len = geometry::distance(from, to);
        const int n = static_cast<int>(std::clamp(std::ceil(len / (0.125 * ctx.tol.distance)), 4.0, 24.0));
        return profileOk(
            [&](double t) {
                const double u = 1.0 - t;
                return lerp(from, to, 1.0 - u * u * u);
            },
            n, false, ctx, [&](Vec2 q) { return ctx.clear(q) && ctx.notFar(q); });
    };
    // Two copies of one line (a stretch of a group on one carrier handed over between elements,
    // mergeOnCarriers) whose ends differ in the last bits only: moving either end that far is a
    // rounding-level change, unchecked as above. Checking it cost two profiles of the distance to
    // the input per junction, which dominated Sharp/Square offsets of dense zig-zags (T-0016).
    const bool unchecked = lineA && lineB && gap <= rounding && sameCarrier(a, b);
    const bool moveB = lineB && (unchecked || lineOk(endPoint(b), e));
    // (moveA only matters when b may not move, or moves inwards.)
    const bool moveA = !(moveB && bOutwards) && lineA && (unchecked || lineOk(startPoint(a), s));
    if (moveB && (bOutwards || !moveA)) {
        std::get<LineSpan>(b).start = e;
    } else if (moveA) {
        std::get<LineSpan>(a).end = s;
    } else if (lineA || lineB) {
        return false;
    } else {
        // Two arcs whose circles do not meet here: only a rounding-level residual may remain
        // (exactJunction then removes it); anything larger is bridged instead.
        if (gap > 1e-12 * std::max({1.0, std::fabs(e.x), std::fabs(e.y), std::get<ArcSpan>(b).radius})) {
            return false;
        }
        b = withStart(b, e);
    }
    return true;
}

// Closes a junction snapJunction could not (a sub-tolerance step sideways, left where a
// tolerance-sized piece was dropped) by bending the last tolerance of one neighbour onto the
// other span's end point: the neighbour is cut at P, one tolerance along it from the junction,
// and the straight P -> (other end point) replaces the stretch beyond P (a short line moves as
// a whole). Only a neighbour whose end then moves away from the input (towards the offset side)
// and whose new stretch stays at least d from it is bent; the offset moves by at most the gap,
// and only within the tolerance of the junction. `extra` receives the new span between `a` and
// `b`, if any. Returns false where neither neighbour can be bent.
bool stepJunction(Span& a, Span& b, const SnapContext& ctx, std::optional<Span>& extra) {
    extra.reset();
    const double tol = ctx.tol.distance;
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    if (geometry::distance(e, s) > tol) {
        return false;
    }
    // Not at a needle tip (the spans run back along each other): there the gap runs across the
    // tip, not sideways, and bending a tolerance of one side onto the other's end puts a spike in
    // (two nearly co-circular arcs from noisy input, 1.5e-8 mm apart at the tip, fam4
    // 0xe40016aa); the gap bridge closes it instead.
    if (tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) < -0.99) {
        return false;
    }
    // Where `span` is cut when its `atEnd` end moves to `to` (nothing when it may not bend):
    // the parameter of P, or the far end itself for a short line.
    auto cut = [&](const Span& span, bool atEnd, Vec2 to) -> std::optional<double> {
        const double len = length(span);
        const bool line = std::holds_alternative<LineSpan>(span);
        if (len <= (line ? tol : 2.0 * tol)) {
            return std::nullopt;
        }
        const Vec2 from = atEnd ? endPoint(span) : startPoint(span);
        const Vec2 outward = tangentAt(span, atEnd ? 1.0 : 0.0).perp() * ctx.sign;
        if ((to - from).dot(outward) < 0.0) {
            return std::nullopt;
        }
        const double t = len <= 2.0 * tol ? (atEnd ? 0.0 : 1.0) : (atEnd ? 1.0 - tol / len : tol / len);
        const Vec2 p = pointAt(span, t);
        if (geometry::distance(p, to) <= tol) {
            return std::nullopt;  // the new stretch would itself be shorter than the tolerance
        }
        if (!stretchClear(nullptr, p, to, ctx)) {
            return std::nullopt;
        }
        return t;
    };
    if (const auto t = cut(a, true, s)) {
        if (*t == 0.0) {
            std::get<LineSpan>(a).end = s;
        } else {
            const Vec2 p = pointAt(a, *t);
            a = withEnd(a, p);
            extra = LineSpan{p, s};
        }
        return true;
    }
    if (const auto t = cut(b, false, e)) {
        if (*t == 1.0) {
            std::get<LineSpan>(b).start = e;
        } else {
            const Vec2 p = pointAt(b, *t);
            b = withStart(b, p);
            extra = LineSpan{e, p};
        }
        return true;
    }
    return false;
}

// Makes the junction of `a` and `b` bit-identical, changing only a's end or b's start. A line
// end simply takes the other span's computed end point. Between two arcs, whose end points are
// computed from centre, radius and angles (ADR-0005), b's centre is moved by the residual (at
// most 1e-10 x max(1, |x|, |y|, r)) and then nudged by a few ulps until
// its computed start equals a's computed end; that can fail where the centre's coordinates are
// much larger than the point's. Then b is kept if it starts level with or ahead of a's end along
// a's direction of travel within 1e-12 x the scale, else it takes the nearest nudged candidate
// that does; where none does either, b is left as it was, so it may then start behind a's end
// (or up to the 1e-10 x the scale residual away from it).
void exactJunction(Span& a, Span& b) {
    if (auto* line = std::get_if<LineSpan>(&a)) {
        line->end = startPoint(b);
        return;
    }
    const Vec2 j = endPoint(a);
    if (auto* line = std::get_if<LineSpan>(&b)) {
        line->start = j;
        return;
    }
    auto& arc = std::get<ArcSpan>(b);
    if (arc.startPoint() == j) {
        return;
    }
    // Residuals up to 1e-10 x the scale are closed by moving b's centre that far: rounding, or
    // two circles that touch to within it (a cusp between Round arcs about vertices 2d apart:
    // circles 7.2e-12 mm apart at ~1 mm, 0xe300070e). That is far below every distance allowance.
    const double scale = std::max({1.0, std::fabs(j.x), std::fabs(j.y), arc.radius});
    if (geometry::distance(arc.startPoint(), j) > 1e-10 * scale) {
        return;  // not a rounding-level residual: leave the arc where it is
    }
    ArcSpan moved = arc;
    moved.centre = j - Vec2::fromAngle(arc.angleAt(0.0), arc.radius);
    for (int iter = 0; iter < 16 && moved.startPoint() != j; ++iter) {
        // Shift the centre by the remaining difference; where that is below the centre
        // coordinate's ulp, by one ulp. A centre coordinate much smaller than the point's (a
        // circle centred on an axis, 0xe300070e) needs the first: ulp steps of ~1e-28 never get
        // there.
        const Vec2 got = moved.startPoint();
        for (const bool alongX : {true, false}) {
            double& c = alongX ? moved.centre.x : moved.centre.y;
            const double want = alongX ? j.x : j.y;
            const double have = alongX ? got.x : got.y;
            if (have == want) {
                continue;
            }
            const double shifted = c + (want - have);
            c = shifted != c ? shifted : std::nextafter(c, have < want ? kInf : -kInf);
        }
    }
    // Shifting can oscillate between two neighbours of the target, and a centre coordinate
    // whose ulp is coarser than the point's reaches only every other value; then search the few
    // ulps around the centre and the radius for an exact hit.
    auto steps = [](double v, int k) {
        for (; k > 0; --k) {
            v = std::nextafter(v, kInf);
        }
        for (; k < 0; ++k) {
            v = std::nextafter(v, -kInf);
        }
        return v;
    };
    const ArcSpan base = moved;
    auto candidate = [&](int da, int dx, int dy) {
        // 0, +1, -1, +2, -2, ... ulps
        auto off = [](int i) { return (i % 2 == 1) ? (i + 1) / 2 : -(i / 2); };
        ArcSpan c = base;
        c.radius = steps(base.radius, off(da));
        c.centre = Vec2{steps(base.centre.x, off(dx)), steps(base.centre.y, off(dy))};
        return c;
    };
    for (int da = 0; da < 9 && moved.startPoint() != j; ++da) {
        for (int dx = 0; dx < 9 && moved.startPoint() != j; ++dx) {
            for (int dy = 0; dy < 9 && moved.startPoint() != j; ++dy) {
                moved = candidate(da, dx, dy);
            }
        }
    }
    if (moved.startPoint() == j) {
        arc = moved;
        return;
    }
    // No exact hit (the point's coordinate reachable only on every other ulp, e.g. radius 4e4
    // about a centre 3e4 mm away from a point at 8e3 mm): the residual is a few ulps whose
    // direction is rounding noise (it differed between platforms' sin/cos, 0xe400137f), so b
    // could start a hair behind a's end. Keep b where it is if it starts level with or ahead of
    // a's end along a's direction of travel within the documented 1e-12 x the scale; otherwise
    // take the nearest of the searched candidates that does. (b kept wherever it started ahead
    // left the whole residual, up to 1e-10 x the scale, open: a cusp between Round arcs about
    // vertices 2d + 3.2e-11 apart kept a 3.2e-11 mm gap when sin/cos were 1 ulp off glibc's.)
    const Vec2 ta = tangentAt(a, 1.0);
    auto ahead = [&](const ArcSpan& c) { return (c.startPoint() - j).dot(ta) >= 0.0; };
    if (ahead(arc) && geometry::distance(arc.startPoint(), j) <= 1e-12 * scale) {
        return;
    }
    double bestGap = 1e-12 * scale;
    std::optional<ArcSpan> best;
    for (int da = 0; da < 9; ++da) {
        for (int dx = 0; dx < 9; ++dx) {
            for (int dy = 0; dy < 9; ++dy) {
                const ArcSpan c = candidate(da, dx, dy);
                const double gap = geometry::distance(c.startPoint(), j);
                if (gap <= bestGap && ahead(c)) {
                    bestGap = gap;
                    best = c;
                }
            }
        }
    }
    if (best) {
        arc = *best;
    }
}

// True when b starts behind a's end along a's direction of travel.
bool startsBehind(const Span& a, const Span& b) {
    return (startPoint(b) - endPoint(a)).dot(tangentAt(a, 1.0)) < 0.0;
}

// Closes the junction a -> b by turning b's start angle only, which leaves b's centre, radius
// and end angle alone: b's start becomes a's end exactly if some angle near the direction of a's
// end gives it, else the nearest such start that lies level with or ahead of a's end along a's
// travel, within 1e-12 x max(1, |x|, |y|, r). Where a's end lies off b's circle by rho and the
// arcs meet at a corner, the start must move round b by about rho (n . ta) / (t . ta) to stand
// level with it; that turn is solved for, and the angles around it and around the direction
// of a's end are searched. One ulp of the angle moves the start by only r x ulp(angle) (1e-15 to
// 1e-14 mm at r = 21), far below one ulp of the coordinates at 1e4 mm (1.8e-12), so the angles tried are
// spaced a quarter of a coordinate ulp apart along the circle, 64 either side. Returns false
// (b unchanged) where no angle does.
bool exactJunctionByAngle(const Span& a, ArcSpan& b) {
    const Vec2 j = endPoint(a);
    const Vec2 bEnd = b.endPoint();
    const Vec2 ta = tangentAt(a, 1.0);
    const double scale = std::max({1.0, std::fabs(j.x), std::fabs(j.y), b.radius});
    // The angle of a's end about b's centre, taken next to b's current start angle.
    const Vec2 radial = j - b.centre;
    double wrapped = normalizeAngle(radial.angle() - b.startAngle);
    if (wrapped > 0.5 * kTwoPi) {
        wrapped -= kTwoPi;
    }
    const double target = b.startAngle + wrapped;
    // The turn (as an angle) that brings b's start level with a's end along ta.
    const double dist = radial.length();
    double turn = 0.0;
    if (dist > 0.0) {
        const Vec2 n = radial / dist;
        const Vec2 increasing = n.perp();  // where a larger angle moves the point
        const double rho = dist - b.radius;
        const double along = increasing.dot(ta);
        if (std::fabs(along) > 1e-3) {
            turn = rho * n.dot(ta) / along / b.radius;
        }
    }
    const double coord = std::max({1.0, std::fabs(j.x), std::fabs(j.y)});
    const double coordUlp = std::nextafter(coord, kInf) - coord;
    const double angleUlp = std::nextafter(std::fabs(target), kInf) - std::fabs(target);
    const double step = std::max(angleUlp, 0.25 * coordUlp / b.radius);
    std::optional<ArcSpan> best;
    double bestGap = 1e-12 * scale;
    for (const double centre : {target, target + turn}) {
        for (int k = -64; k <= 64; ++k) {
            ArcSpan c = b;
            c.startAngle = centre + k * step;
            // endPoint() is taken at startAngle + normalize(endAngle - startAngle), so a new
            // start angle can round the end point differently (seen for 47k of 12.9M turns);
            // it must stay where the previous junction put it.
            if (!(c.endPoint() == bEnd)) {
                continue;
            }
            const Vec2 st = c.startPoint();
            if (st == j) {
                b = c;
                return true;
            }
            const double gap = geometry::distance(st, j);
            if (gap <= bestGap && (st - j).dot(ta) >= 0.0) {
                bestGap = gap;
                best = c;
            }
        }
        if (turn == 0.0) {
            break;
        }
    }
    if (best) {
        b = *best;
        return true;
    }
    return false;
}

// exactJunction over every junction of a chain, in order. A closed chain is walked from a line
// if it has one: each call may move the second span's start (an arc's centre), so the walk must
// end at a junction whose second span is a line, or the closing call would undo the first one
// (T-0017 round 3: a retrace removed from a closed chain left its first junction 3.5e-14 open).
// A closed chain of arcs only has no such junction: the closing call moves the first arc's
// centre and so reopens the first junction by a few ulps, in a direction that is rounding noise.
// Where that leaves the second arc starting behind the first's end, the loop is closed instead
// by turning the first arc's start angle, which keeps its end point (0xa5000ba6, a 4-arc loop
// at 1e5 mm: 7.2e-12 mm behind). A turn cannot help where the previous arc arrives almost
// square to the first arc's circle and ends just off it: turning moves the start across its
// travel, not along it (continuous input too: a 6-arc loop at (3e4, 7e4), 1.7e-11 mm behind).
// Where a junction is still behind, the walk is tried from each other arc in turn and the first
// result without a backward junction is kept; if none is (never seen), the walk from the first
// arc stays, a few coordinate ulps behind at one junction.
void exactJunctions(std::vector<Span>& spans, bool closed) {
    const std::size_t m = spans.size();
    if (m < 2) {
        return;
    }
    std::size_t first = 0;
    bool allArcs = false;
    if (closed) {
        const auto line = std::find_if(spans.begin(), spans.end(),
                                       [](const Span& sp) { return std::holds_alternative<LineSpan>(sp); });
        allArcs = line == spans.end();
        first = allArcs ? 0 : static_cast<std::size_t>(line - spans.begin());
    }
    const std::size_t junctions = closed ? m : m - 1;
    // The walk from `from`; for a loop of arcs, closed by turning arc `from` where needed.
    auto walk = [&](std::vector<Span>& sp, std::size_t from) {
        for (std::size_t k = 0; k + 1 < junctions; ++k) {
            const std::size_t i = (from + k) % m;
            exactJunction(sp[i], sp[(i + 1) % m]);
        }
        const std::size_t i = (from + junctions - 1) % m;
        const std::size_t b = (i + 1) % m;
        const Span before = sp[b];
        exactJunction(sp[i], sp[b]);
        if (allArcs && startsBehind(sp[b], sp[(b + 1) % m])) {
            ArcSpan turned = std::get<ArcSpan>(before);
            if (exactJunctionByAngle(sp[i], turned)) {
                sp[b] = turned;
            }
        }
    };
    if (!allArcs) {
        walk(spans, first);
        return;
    }
    auto anyBehind = [&](const std::vector<Span>& sp) {
        for (std::size_t i = 0; i < m; ++i) {
            if (startsBehind(sp[i], sp[(i + 1) % m])) {
                return true;
            }
        }
        return false;
    };
    const std::vector<Span> original = spans;
    walk(spans, 0);
    for (std::size_t from = 1; from < m && anyBehind(spans); ++from) {
        std::vector<Span> attempt = original;
        walk(attempt, from);
        if (!anyBehind(attempt)) {
            spans = std::move(attempt);
        }
    }
}

// The span that closes a junction nothing else could: a straight line from a's end to b's
// start. Where that line would leave the offset (beyond d for Round: a needle tip of two offset
// lines whose rounded end, an arc about an input point, was not among the kept pieces), the
// missing arc is put in instead if it can be: about the input point nearest one end, through that
// end, with the other span trimmed or extended onto its circle like any snapped junction, and
// every point of it passing the distance checks (T-0017, gen3 0x93010bf: the line ran 1.3e-5 mm
// beyond d across the tip). `a` or `b` may be changed.
std::optional<Span> gapBridge(Span& a, Span& b, const SnapContext& ctx) {
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    const LineSpan line{e, s};
    {
        // The straight bridge as documented: it may come inside d by its sagitta against the
        // round neighbourhood (at most tol^2 / (8d)), and for Round it must not go beyond d.
        const double tol = ctx.tol.distance;
        const double inside = std::max(ctx.allowance, tol * tol / (8.0 * ctx.d));
        const double len = geometry::distance(e, s);
        const int n = static_cast<int>(std::clamp(std::ceil(len / (0.125 * tol)), 2.0, 512.0));
        const bool ok = profileOk(
            [&](double t) { return lerp(e, s, t); }, n, false, ctx,
            [&](Vec2 q) {
                const double slack = 1e-12 * std::max({1.0, ctx.d, std::fabs(q.x), std::fabs(q.y)});
                const double dq = ctx.distance(q);
                return dq >= ctx.d - inside - slack && (!ctx.round || dq <= ctx.d + ctx.allowance + slack);
            });
        if (ok) {
            return Span{line};
        }
    }
    const Vec2 tin = tangentAt(a, 1.0);
    const Vec2 tout = tangentAt(b, 0.0);
    auto arcOk = [&](const ArcSpan& arc) {
        if (std::fabs(arc.sweepAngle()) > kPi / 2.0 || isDegenerate(Span{arc}, ctx.tol)) {
            return false;
        }
        const int n =
            static_cast<int>(std::clamp(std::ceil(arc.length() / (0.125 * ctx.tol.distance)), 4.0, 512.0));
        return profileOk([&](double t) { return arc.pointAt(t); }, n, false, ctx,
                         [&](Vec2 q) { return ctx.clear(q) && ctx.notFar(q); });
    };
    const double gap = geometry::distance(e, s);
    const double reach = std::max(2.0 * ctx.tol.distance, 4.0 * gap);
    for (const bool aboutEnd : {false, true}) {
        // An arc about the input point nearest b's start, ending there, meeting a's carrier; else
        // one about the point nearest a's end, starting there, meeting b's carrier.
        const Vec2 through = aboutEnd ? e : s;
        const Vec2 v = ctx.nearest(through);
        const double r = geometry::distance(through, v);
        if (r <= 1e-9 * std::max({1.0, std::fabs(v.x), std::fabs(v.y)})) {
            continue;
        }
        const Span& other = aboutEnd ? b : a;
        const Vec2 otherEnd = aboutEnd ? s : e;
        const Vec2 t = aboutEnd ? tin : tout;
        const ArcDirection dir = (through - v).cross(t) > 0.0 ? ArcDirection::CCW : ArcDirection::CW;
        std::optional<ArcSpan> best;
        std::optional<Span> moved;
        double bestDist = kInf;
        for (const Vec2 y : carrierIntersections(other, Span{ArcSpan::fullCircle(v, r)})) {
            const std::optional<double> ty = extensionParam(other, y, !aboutEnd);
            const double dy = geometry::distance(y, otherEnd);
            if (!ty || dy > reach || dy >= bestDist ||
                (aboutEnd ? *ty >= 1.0 - kParamEps : *ty <= kParamEps)) {
                continue;
            }
            const bool extends = aboutEnd ? *ty < 0.0 : *ty > 1.0;
            if (extends && !stretchClear(&other, otherEnd, y, ctx)) {
                continue;
            }
            const ArcSpan arc =
                aboutEnd ? ArcSpan::fromCentre(v, e, y, dir) : ArcSpan::fromCentre(v, y, s, dir);
            const Span changed = aboutEnd ? withStart(other, y) : withEnd(other, y);
            const Vec2 before = aboutEnd ? tin : tangentAt(changed, 1.0);
            const Vec2 after = aboutEnd ? tangentAt(changed, 0.0) : tout;
            if (isDegenerate(changed, ctx.tol) || arc.tangentAt(0.0).dot(before) <= 0.0 ||
                arc.tangentAt(1.0).dot(after) <= 0.0 || !arcOk(arc)) {
                continue;
            }
            best = arc;
            moved = changed;
            bestDist = dy;
        }
        if (best) {
            (aboutEnd ? b : a) = *moved;
            return Span{*best};
        }
    }
    return std::nullopt;
}

// Snaps the junctions of an output chain (and its closing junction when `closed`) so that
// consecutive spans share their end point exactly. `dropped[k]` holds the sub-tolerance pieces
// left out before span k (`dropped[0]`: those across the closing junction of a closed chain).
// Each junction is first snapped geometrically (snapJunction, else stepJunction); where both
// fail the dropped pieces are put back (they are the true offset there), and where there are
// none a straight bridge spans the gap (gapBridge). Where even that cannot stay on the offset (a
// Round needle tip whose rounded end is missing, T-0017) the chain is cut there instead: the
// result then holds several chains, open, each continuous. Then the junctions are made
// bit-identical in order (exactJunction), which never disturbs an end matched before; a closed
// chain is walked from a line if it has one, so its closing junction is exact too.
std::vector<std::vector<Span>> snapChain(const std::vector<Span>& spans,
                                         const std::vector<std::vector<Span>>& dropped, bool closed,
                                         const SnapContext& ctx, bool& stillClosed) {
    std::vector<std::vector<Span>> result;
    stillClosed = false;
    if (spans.empty()) {
        return result;
    }
    // A cut between two spans is marked by a Bezier span, which never occurs in an offset chain.
    const Span cut = BezierSpan{};
    auto isCut = [](const Span& sp) { return std::holds_alternative<BezierSpan>(sp); };
    // Closes the junction a -> b; `between` receives any spans that go between them.
    auto join = [&](Span& a, Span& b, const std::vector<Span>& before, std::vector<Span>& between) {
        std::optional<Span> extra;
        if (snapJunction(a, b, ctx)) {
            return;
        }
        if (stepJunction(a, b, ctx, extra)) {
            if (extra) {
                between.push_back(*extra);
            }
            return;
        }
        std::vector<Span> chain{a};
        chain.insert(chain.end(), before.begin(), before.end());
        chain.push_back(b);
        std::vector<Span> joined{chain.front()};
        for (std::size_t k = 1; k < chain.size(); ++k) {
            if (!snapJunction(joined.back(), chain[k], ctx)) {
                const std::optional<Span> bridge = gapBridge(joined.back(), chain[k], ctx);
                joined.push_back(bridge ? *bridge : cut);
            }
            joined.push_back(chain[k]);
        }
        a = joined.front();
        b = joined.back();
        between.assign(joined.begin() + 1, joined.end() - 1);
    };
    std::vector<Span> body = spans;
    std::vector<Span> closing;
    if (closed && body.size() > 1) {
        // The closing junction first, so that every later junction sees both ends of its spans
        // in their final place (no span can then be trimmed past its own other end).
        join(body.back(), body.front(), dropped[0], closing);
    }
    std::vector<Span> out{body.front()};
    for (std::size_t k = 1; k < body.size(); ++k) {
        std::vector<Span> between;
        join(out.back(), body[k], dropped[k], between);
        out.insert(out.end(), between.begin(), between.end());
        out.push_back(body[k]);
    }
    out.insert(out.end(), closing.begin(), closing.end());
    // A closed chain that was cut becomes open, starting after its first cut.
    const auto firstCut = std::find_if(out.begin(), out.end(), isCut);
    const bool wasCut = firstCut != out.end();
    if (wasCut && closed) {
        std::rotate(out.begin(), firstCut + 1, out.end());
    }
    std::vector<std::vector<Span>> pieces(1);
    for (const Span& sp : out) {
        if (isCut(sp)) {
            pieces.emplace_back();
        } else {
            pieces.back().push_back(sp);
        }
    }
    stillClosed = closed && !wasCut;
    for (std::vector<Span>& piece : pieces) {
        if (piece.empty()) {
            continue;
        }
        exactJunctions(piece, stillClosed);
        result.push_back(std::move(piece));
    }
    return result;
}

// True when the short span `s` steps back between `prev` and `next`: no longer than the
// tolerance, and its chord runs against both neighbours while they go the same general way, or it
// sets off straight back along `prev` (a retrace), or `next` sets off straight back along it.
bool stepsBack(const Span& prev, const Span& s, const Span& next, const Tolerance& tol) {
    if (!isDegenerate(s, tol)) {
        return false;
    }
    const Vec2 chord = endPoint(s) - startPoint(s);
    const Vec2 tin = tangentAt(prev, 1.0);
    const Vec2 tout = tangentAt(next, 0.0);
    if (tin.dot(tout) > 0.0 && chord.dot(tin) < 0.0 && chord.dot(tout) < 0.0) {
        return true;
    }
    return tin.dot(tangentAt(s, 0.0)) < -0.9999 || tangentAt(s, 1.0).dot(tout) < -0.9999;
}

// Safety net after snapChain (T-0006): where three or more offset carriers nearly meet within the
// tolerance (e.g. lines a few hundredths of a radian apart crossing in a sub-tolerance triangle),
// the pieces between their crossings can leave a span shorter than the tolerance that steps back
// or sideways against the direction of travel. Such a span is removed, with a short neighbour
// if need be, and the spans on either side are joined: by snapJunction (trimmed or extended along
// their carriers to where they meet, extensions at least d from the input and, for Round, not
// beyond it), else by a straight bridge (at most 2 x tol long) if that runs forwards with both and
// stays clear in the same way (the two sides on one carrier, say), possibly taking over the first
// or last tolerance of one of them. Two arcs on nearly one circle on either side are merged into
// one arc instead (T-0017). Where none of this works the chain is left as it is. Chains of any
// length from three spans are checked (T-0017: [arc, backward line, arc] was skipped). New
// junctions are made exact like the others.
void removeBackSteps(std::vector<Span>& spans, bool closed, const SnapContext& ctx) {
    auto bridgeOk = [&](const Span& a, const Span& b) {
        const Vec2 e = endPoint(a);
        const Vec2 s = startPoint(b);
        const Vec2 chord = s - e;
        if (geometry::distance(e, s) > 2.0 * ctx.tol.distance || chord.dot(tangentAt(a, 1.0)) <= 0.0 ||
            chord.dot(tangentAt(b, 0.0)) <= 0.0) {
            return false;
        }
        // Where the input has a gap a Round join next to it lies up to about that gap beyond d
        // (see offset.hpp); a bridge from it may stay that far out, but no farther (T-0017, gen3
        // 0x930046a: a 1.4e-5 mm input gap left a 3.2e-6 mm step back that no bridge could close).
        return stretchClear(nullptr, e, s, ctx, std::max(beyond(ctx, e), beyond(ctx, s)));
    };
    for (int pass = 0; pass < 4; ++pass) {
        bool changed = false;
        for (std::size_t i = 0; i < spans.size() && spans.size() >= 3; ++i) {
            const std::size_t n = spans.size();
            if (!closed && (i == 0 || i + 1 == n)) {
                continue;
            }
            if (!stepsBack(spans[(i + n - 1) % n], spans[i], spans[(i + 1) % n], ctx.tol)) {
                continue;
            }
            // Candidate windows (first, last) of spans to remove, all no longer than the tolerance:
            // the span itself, then with its previous or next neighbour.
            std::vector<std::pair<std::size_t, std::size_t>> windows{{i, i}};
            if (closed || i >= 2) {
                windows.emplace_back((i + n - 1) % n, i);
            }
            if (closed || i + 2 < n) {
                windows.emplace_back(i, (i + 1) % n);
            }
            for (const auto& [first, last] : windows) {
                if (!isDegenerate(spans[first], ctx.tol) || !isDegenerate(spans[last], ctx.tol)) {
                    continue;
                }
                const std::size_t left = (first + n - 1) % n;
                const std::size_t right = (last + 1) % n;
                if (left == right || left == last || right == first) {
                    continue;
                }
                // What goes in each slot of the chain afterwards (removed spans leave theirs empty).
                std::vector<std::vector<Span>> slots(n);
                for (std::size_t k = 0; k < n; ++k) {
                    slots[k].push_back(spans[k]);
                }
                slots[first].clear();
                slots[last].clear();
                bool joined = false;
                // Two arcs on nearly the same circle (e.g. the offsets of an input arc split in two
                // with ~1e-8 mm of noise, whose circles do not meet) become one arc on the circle of
                // either, where every point of it stays clear like a snapped extension; the
                // junction its moved end then has with the next (previous) span is snapped again.
                const auto* arcA = std::get_if<ArcSpan>(&spans[left]);
                const auto* arcB = std::get_if<ArcSpan>(&spans[right]);
                if (arcA && arcB && arcA->direction == arcB->direction &&
                    geometry::distance(arcA->centre, arcB->centre) + std::fabs(arcA->radius - arcB->radius) <=
                        0.5 * ctx.tol.distance) {
                    const double want = std::fabs(arcA->sweepAngle()) + std::fabs(arcB->sweepAngle());
                    for (const bool onA : {true, false}) {
                        ArcSpan merged = onA ? *arcA : *arcB;
                        if (onA) {
                            merged.endAngle = (arcB->endPoint() - merged.centre).angle();
                        } else {
                            merged.startAngle = (arcA->startPoint() - merged.centre).angle();
                        }
                        // The far neighbour of the end that moved, if there is one.
                        const std::size_t outer = onA ? (right + 1) % n : (left + n - 1) % n;
                        const bool hasFar =
                            closed ? outer != left && outer != right && outer != first && outer != last
                                   : (onA ? right + 1 < n : left > 0);
                        if (closed && !hasFar) {
                            continue;
                        }
                        bool ok = std::fabs(std::fabs(merged.sweepAngle()) - want) <=
                                  4.0 * ctx.tol.distance / merged.radius;
                        const int samples = static_cast<int>(
                            std::clamp(std::ceil(merged.length() / (0.125 * ctx.tol.distance)), 16.0, 512.0));
                        ok = ok && profileOk([&](double t) { return merged.pointAt(t); }, samples, true, ctx,
                                             [&](Vec2 q) { return ctx.clear(q) && ctx.notFar(q); });
                        Span m = merged;
                        Span neighbour = hasFar ? spans[outer] : m;
                        if (ok && hasFar) {
                            ok = onA ? snapJunction(m, neighbour, ctx) : snapJunction(neighbour, m, ctx);
                            ok = ok && !isDegenerate(neighbour, ctx.tol);
                        }
                        if (!ok || isDegenerate(m, ctx.tol)) {
                            continue;
                        }
                        slots[left].assign(1, m);
                        slots[right].clear();
                        if (hasFar) {
                            slots[outer].assign(1, neighbour);
                        }
                        joined = true;
                        break;
                    }
                }
                if (!joined) {
                    Span a = spans[left];
                    Span b = spans[right];
                    std::optional<Span> bridge;
                    if (!snapJunction(a, b, ctx) || isDegenerate(a, ctx.tol) || isDegenerate(b, ctx.tol)) {
                        a = spans[left];
                        b = spans[right];
                        // A straight bridge between them, else one that also takes over the first
                        // tolerance of `b` or the last of `a` (a sub-tolerance notch of the exact
                        // offset that a direct bridge would cut inside d).
                        // The cut is tried first at exactly the length that turns the step from
                        // backwards to sideways (where the two neighbours overlap along the way,
                        // the end of one is hidden behind the other, T-0017 0xe40010cb), then at a
                        // tolerance and shorter (a chord across a small arc dips inside d by its
                        // sagitta, so a tolerance-long one may not pass).
                        const double tol = ctx.tol.distance;
                        const Vec2 step = startPoint(b) - endPoint(a);
                        const double needA = -step.dot(tangentAt(a, 1.0));
                        const double needB = -step.dot(tangentAt(b, 0.0));
                        std::vector<std::pair<double, bool>> cuts;  // (length, cut b rather than a)
                        if (needB > 0.0) {
                            cuts.emplace_back(needB * (1.0 + 1e-6) + 1e-12, true);
                        }
                        if (needA > 0.0) {
                            cuts.emplace_back(needA * (1.0 + 1e-6) + 1e-12, false);
                        }
                        for (double c = tol; c >= 0.1 * tol; c *= 0.5) {
                            cuts.emplace_back(c, true);
                            cuts.emplace_back(c, false);
                        }
                        bool bridged = bridgeOk(a, b);
                        for (const auto& [c, onB] : cuts) {
                            if (bridged) {
                                break;
                            }
                            const Span& span = onB ? b : a;
                            const double len = length(span);
                            if (c > 2.0 * tol || len <= c + ctx.tol.distance * 1e-3) {
                                continue;  // keep a remnant of the span (cutting it all is another window)
                            }
                            const Span cutSpan = onB ? withStart(b, pointAt(b, c / len))
                                                     : withEnd(a, pointAt(a, 1.0 - c / len));
                            if (onB ? bridgeOk(a, cutSpan) : bridgeOk(cutSpan, b)) {
                                (onB ? b : a) = cutSpan;
                                bridged = true;
                            }
                        }
                        if (!bridged) {
                            continue;
                        }
                        bridge = LineSpan{endPoint(a), startPoint(b)};
                    }
                    slots[left].assign(1, a);
                    slots[right].assign(1, b);
                    if (bridge) {
                        slots[last].push_back(*bridge);  // in place of the removed spans
                    }
                }
                std::vector<Span> next;
                next.reserve(n);
                for (const std::vector<Span>& slot : slots) {
                    next.insert(next.end(), slot.begin(), slot.end());
                }
                spans.swap(next);
                exactJunctions(spans, closed);
                changed = true;
                break;
            }
        }
        if (!changed) {
            return;
        }
    }
}

// Closes the gap between consecutive spans `a` and `b` left by removeRetraces (at most a few
// rounding units of the coordinates): snapped like any junction, else a straight bridge that stays
// clear of the input. Returns whether it closed; `bridge` receives the bridge, if one is used.
bool closeSmallGap(Span& a, Span& b, const SnapContext& ctx, std::optional<Span>& bridge) {
    bridge.reset();
    Span x = a;
    Span y = b;
    if (snapJunction(x, y, ctx) && !isDegenerate(x, ctx.tol) && !isDegenerate(y, ctx.tol)) {
        a = x;
        b = y;
        return true;
    }
    const Vec2 e = endPoint(a);
    const Vec2 s = startPoint(b);
    if (!stretchClear(nullptr, e, s, ctx)) {
        return false;
    }
    bridge = LineSpan{e, s};
    return true;
}

// Which side of `level` (in [-1, 1]) the cosine `tangentAt(a, 1.0).dot(tangentAt(b, 0.0))` of the
// turn from `a` to `b` lies: -1 below, +1 above, 0 when that is not clear. Two lines are decided
// on the raw direction vectors unless the cosine is within 1e-9 of `level` (normalising costs a
// hypot each, which showed in profiles of offsets with nothing to prune, T-0170); callers compute
// the exact expression for 0.
int cosineSide(const Span& a, const Span& b, double level) {
    const auto* la = std::get_if<LineSpan>(&a);
    const auto* lb = std::get_if<LineSpan>(&b);
    return la && lb ? detail::cosineSideOf(la->end - la->start, lb->end - lb->start, level) : 0;
}

// Exactly `tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) < 0.0`.
bool endsReverse(const Span& a, const Span& b) {
    const int side = cosineSide(a, b, 0.0);
    return side != 0 ? side < 0 : tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) < 0.0;
}

// A span that sets off straight back along its predecessor (or that its successor sets off back
// along) and lies wholly within a rounding unit of the coordinates (2e-9 x max(1, d, |x|, |y|))
// of it (two arcs also on one circle to that unit): a needle tip so thin that, at those
// coordinates, the two sides are the same stretch traversed twice (T-0017, 0x840117a: 2.9e-3 mm
// at 3.6e-6 mm sideways, ~1e4 mm from the origin). The span is removed, its neighbour cut back to
// where the span's far end lies alongside it, and the new junction closed (closeSmallGap). The
// geometry moves by at most that unit.
void removeRetraces(std::vector<Span>& spans, bool closed, const SnapContext& ctx, double d) {
    for (int pass = 0; pass < 8; ++pass) {
        bool changed = false;
        const std::size_t n = spans.size();
        if (n < (closed ? 3u : 2u)) {
            return;
        }
        for (std::size_t k = closed ? 0 : 1; k < n && !changed; ++k) {
            const std::size_t ia = (k + n - 1) % n;
            const std::size_t ib = k;
            const Span& a = spans[ia];
            const Span& b = spans[ib];
            const int side = cosineSide(a, b, -0.99);
            if (side > 0 || (side == 0 && tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) > -0.99)) {
                continue;
            }
            const Vec2 j = startPoint(b);
            // Fixed, not scaled by the coordinates: the same needle is kept or removed wherever it
            // lies (beyond ~1e6 mm the rounding of the coordinates themselves takes over).
            const double unit =
                std::max(2e-9 * std::max(1.0, d), 64.0 * std::numeric_limits<double>::epsilon() *
                                                      std::max(std::fabs(j.x), std::fabs(j.y)));
            auto alongside = [&](const Span& x, const Span& y) {
                // Arcs only on one circle to that unit: two different circles crossing at a shallow
                // angle make a real (if thin) needle, e.g. 0.3 mm long and 5e-5 mm wide.
                const auto* ax = std::get_if<ArcSpan>(&x);
                const auto* ay = std::get_if<ArcSpan>(&y);
                if (ax && ay &&
                    geometry::distance(ax->centre, ay->centre) + std::fabs(ax->radius - ay->radius) > unit) {
                    return false;
                }
                for (const double f : {0.25, 0.5, 0.75, 1.0}) {
                    if (distanceToSpan(pointAt(x, f), y) > unit) {
                        return false;
                    }
                }
                return true;
            };
            // Remove b (cutting a back) or a (cutting b forward); the cut neighbour must keep a
            // positive length, and the span beyond the removed one must exist and differ from it.
            for (const bool dropB : {true, false}) {
                const Span& gone = dropB ? b : a;
                const Span& kept = dropB ? a : b;
                if (!alongside(gone, kept)) {
                    continue;
                }
                const std::size_t ik = dropB ? ia : ib;
                const std::size_t ig = dropB ? ib : ia;
                const bool hasBeyond = closed || (dropB ? ig + 1 < n : ig > 0);
                const std::size_t beyond = dropB ? (ig + 1) % n : (ig + n - 1) % n;
                if (hasBeyond && beyond == ik) {
                    continue;
                }
                const double t =
                    std::clamp(carrierParam(kept, dropB ? endPoint(gone) : startPoint(gone)), 0.0, 1.0);
                Span cut = dropB ? subSpan(kept, 0.0, t) : subSpan(kept, t, 1.0);
                if ((dropB ? t : 1.0 - t) <= kParamEps || isDegenerate(cut, ctx.tol)) {
                    continue;
                }
                std::vector<std::vector<Span>> slots(n);
                for (std::size_t q = 0; q < n; ++q) {
                    slots[q].push_back(spans[q]);
                }
                slots[ig].clear();
                if (hasBeyond) {
                    Span other = spans[beyond];
                    std::optional<Span> bridge;
                    const bool ok = dropB ? closeSmallGap(cut, other, ctx, bridge)
                                          : closeSmallGap(other, cut, ctx, bridge);
                    if (!ok) {
                        continue;
                    }
                    slots[beyond].assign(1, other);
                    if (bridge) {
                        slots[ig].push_back(*bridge);
                    }
                }
                slots[ik].assign(1, cut);
                std::vector<Span> next;
                for (const std::vector<Span>& slot : slots) {
                    next.insert(next.end(), slot.begin(), slot.end());
                }
                spans.swap(next);
                exactJunctions(spans, closed);
                changed = true;
                break;
            }
        }
        if (!changed) {
            return;
        }
    }
}

// Consecutive arcs of a chain on one circle to rounding (centres and radii within
// 1e-12 x max(1, |x|, |y|, r), so the junctions stay exact) and running the same way become one arc on the
// first one's circle: a split there is only an artefact of the pieces (e.g. offsets of an input arc cut in
// three, with noise), and the rounding between their circles can make the next arc start a hair behind the
// previous one's end (T-0017, gen3 0x94010e5: a full circle as three arcs). A closed chain that
// is then one arc going all the way round becomes a full circle.
void mergeCoCircularArcs(std::vector<Span>& spans, bool closed) {
    auto same = [](const Span& a, const Span& b) {
        const auto* x = std::get_if<ArcSpan>(&a);
        const auto* y = std::get_if<ArcSpan>(&b);
        if (!x || !y || x->direction != y->direction) {
            return false;
        }
        const double scale =
            1e-12 * std::max({1.0, std::fabs(x->centre.x), std::fabs(x->centre.y), x->radius});
        return geometry::distance(x->centre, y->centre) + std::fabs(x->radius - y->radius) <= scale;
    };
    bool changed = false;
    for (std::size_t k = 0; k + 1 < spans.size();) {
        if (!same(spans[k], spans[k + 1])) {
            ++k;
            continue;
        }
        auto& a = std::get<ArcSpan>(spans[k]);
        const auto& b = std::get<ArcSpan>(spans[k + 1]);
        const double sweep = std::fabs(a.sweepAngle()) + std::fabs(b.sweepAngle());
        if (sweep >= kTwoPi - 1e-9) {
            ++k;  // would wrap round: only a closed chain's last merge may close the circle
            continue;
        }
        ArcSpan merged = a;
        merged.endAngle = b.endAngle;  // the same circle to rounding: b's end, as b states it
        if (std::fabs(std::fabs(merged.sweepAngle()) - sweep) > 1e-9) {
            ++k;
            continue;
        }
        a = merged;
        spans.erase(spans.begin() + static_cast<std::ptrdiff_t>(k) + 1);
        changed = true;
    }
    if (closed && spans.size() == 2 && same(spans[0], spans[1])) {
        const ArcSpan a = std::get<ArcSpan>(spans[0]);
        spans.assign(1, Span{ArcSpan{a.centre, a.radius, a.startAngle, a.startAngle, a.direction}});
        changed = true;
    }
    if (changed) {
        exactJunctions(spans, closed);
    }
}

// A closed chain that encloses no width a tool could follow and would only retrace itself,
// dropped like the opposite pairs of mergeOnCarriers: two spans between the same two points that
// never part by more than half the tolerance (two lines the snapping closed up, or a lens of two
// arcs on nearly one circle running opposite ways, T-0017 0x82004c8), or any loop whose mean
// width, 2 x area / perimeter, is below tol / 8 and whose every point lies within tol / 8 of
// another of its spans (a lens of two nearly co-circular opposite arcs, gen3 0x9300698: 1.45 mm
// long, ~1e-5 mm wide; what is left between two nearly tangent circles once the sliver along
// them is dropped, 0x9100821: 0.017 mm round, crossing itself).
bool isSliverLoop(const Contour& c, const Tolerance& tol) {
    if (!c.closed || c.spans.size() < 2) {
        return false;
    }
    const double half = 0.5 * tol.distance;
    if (c.spans.size() == 2) {
        bool apart = false;
        for (int k = 1; !apart && k < 16; ++k) {
            apart = distanceToSpan(pointAt(c.spans[0], k / 16.0), c.spans[1]) > half ||
                    distanceToSpan(pointAt(c.spans[1], k / 16.0), c.spans[0]) > half;
        }
        if (!apart) {
            return true;
        }
    }
    double perimeter = 0.0;
    for (const Span& sp : c.spans) {
        perimeter += length(sp);
    }
    const Vec2 o = startPoint(c.spans.front());
    double twiceArea = 0.0;
    for (const Span& sp : c.spans) {
        for (int k = 0; k < 16; ++k) {
            twiceArea += (pointAt(sp, k / 16.0) - o).cross(pointAt(sp, (k + 1) / 16.0) - o);
        }
    }
    if (perimeter <= 0.0 || std::fabs(twiceArea) / perimeter > 0.125 * tol.distance) {
        return false;
    }
    for (std::size_t i = 0; i < c.spans.size(); ++i) {
        for (int k = 1; k < 16; ++k) {
            const Vec2 q = pointAt(c.spans[i], k / 16.0);
            double nearest = kInf;
            for (std::size_t j = 0; j < c.spans.size(); ++j) {
                if (j != i) {
                    nearest = std::min(nearest, distanceToSpan(q, c.spans[j]));
                }
            }
            if (nearest > 0.125 * tol.distance) {
                return false;
            }
        }
    }
    return true;
}

// Spans no longer than the tolerance at either end of an open strand go (ADR-0012 section 7).
void dropTinyEnds(std::vector<Span>& spans, const Tolerance& tol) {
    while (!spans.empty() && isDegenerate(spans.front(), tol)) {
        spans.erase(spans.begin());
    }
    while (!spans.empty() && isDegenerate(spans.back(), tol)) {
        spans.pop_back();
    }
}

// The part of a chain between positions u0 < u1, where position i + f is fraction f along span
// i; on a closed chain u1 may run past the end (up to u0 + n) and wraps round.
std::vector<Span> chainPart(const std::vector<Span>& spans, double u0, double u1) {
    std::vector<Span> out;
    const std::size_t n = spans.size();
    if (n == 0 || !(u1 > u0)) {
        return out;
    }
    const auto first = static_cast<std::size_t>(std::floor(u0));
    for (std::size_t g = first; static_cast<double>(g) < u1 && g < first + 2 * n; ++g) {
        const double t0 = std::max(0.0, u0 - static_cast<double>(g));
        const double t1 = std::min(1.0, u1 - static_cast<double>(g));
        if (t1 > t0) {
            // Only the ends that move are recomputed, so the junctions inside stay exact.
            Span part = spans[g % n];
            if (t1 < 1.0) {
                part = withEnd(part, pointAt(spans[g % n], t1));
            }
            if (t0 > 0.0) {
                part = withStart(part, pointAt(spans[g % n], t0));
            }
            out.push_back(part);
        }
    }
    return out;
}

// True when two line-or-arc spans cross or touch: a point where their carriers meet lies on both,
// to within kParamEps of their lengths beyond their ends (the pairwise test of T-0128).
bool spansMeet(const Span& a, const Span& b) {
    for (const Vec2 p : carrierIntersections(a, b)) {
        if (inUnit(carrierParam(a, p)) && inUnit(carrierParam(b, p))) {
            return true;
        }
    }
    return false;
}

// How far beyond its box a point can lie at which spansMeet finds `s` meeting another span, so
// that two spans whose boxes, each grown by this much, are apart never meet. A meeting point is
// within inUnit's slack (kParamEps of the length, here 4x for carrierParam's rounding) of each
// span's ends, and within carrierIntersections' contact tolerances of each carrier: circles whose
// centres are up to 1e-9 mm farther apart than tangency (or nearer, inside) touch, and then
// points of the two arcs at the meeting angles are at most that gap apart; a line touches a circle
// up to 1.25e-10 x r beyond it (disc >= -1e-9 qa r^2), or within 64 ulps of the coordinates, and
// the rounding of the discriminant's cancellation, at most ~10 eps |w|^2 / qa with w from the
// centre to the line's start, |w| <= len + r, adds up to sqrt(10 eps) (len + r) < 5e-8 (len + r)
// to that gap (1e-7 here); every other rounding is below 1e-12 x max(1, |x|, |y|, r).
double meetReach(const Span& s, const Box2& b) {
    double scale =
        std::max({1.0, std::fabs(b.min.x), std::fabs(b.min.y), std::fabs(b.max.x), std::fabs(b.max.y)});
    const double len = length(s);
    double reach = 4.0 * kParamEps * len;
    if (const auto* arc = std::get_if<ArcSpan>(&s)) {
        scale = std::max(scale, arc->radius);
        reach += 1e-9 + 1e-7 * arc->radius;
    } else {
        reach += 1e-7 * len;
    }
    return reach + 1e-12 * scale;
}

// Where a chain has to be cut so that no part of it crosses or touches itself (two spans of a
// part that are not neighbours meet, spansMeet): the index of the first span of every part after
// the first, empty when the chain does not cross itself (`firstOnly`: stop at the first cut).
// Greedy, in one pass: each span is tested only against the earlier spans of its part whose boxes,
// grown by meetReach, meet its own, so the answer is that of testing every pair. They are found
// through a BoxTree of all the spans, queried for the ids of the part's earlier spans, so the
// cost is O(n log n) plus the pairs whose grown boxes meet (T-0172: the pairwise test it
// replaces was quadratic, ~3 ms per 500 spans, and so was limited to strays of 512 spans, longer
// ones being split into one chain per span; round 1's clustering grid blew up on nested boxes).
std::vector<std::size_t> selfCrossingCuts(const std::vector<Span>& spans, bool firstOnly = false) {
    std::vector<std::size_t> cuts;
    const std::size_t n = spans.size();
    if (n < 3) {
        return cuts;
    }
    std::vector<Box2> boxes;
    boxes.reserve(n);
    for (const Span& s : spans) {
        const Box2 b = bounds(s);
        boxes.push_back(b.inflated(meetReach(s, b)));
    }
    detail::BoxTree tree(boxes);
    std::size_t from = 0;  // first span of the current part
    for (std::size_t i = from + 2; i < n; ++i) {
        // The earlier spans of the part other than the neighbour: ids from..i - 2.
        const bool meets = tree.queryUntil(boxes[i], from, i - 2,
                                           [&](std::size_t j) { return spansMeet(spans[j], spans[i]); });
        if (meets) {
            cuts.push_back(i);
            if (firstOnly) {
                return cuts;
            }
            from = i;
            i = from + 1;  // the loop goes on at from + 2
        }
    }
    return cuts;
}

// True when two spans of the chain that are not neighbours cross or touch.
bool partCrossesItself(const std::vector<Span>& spans) {
    return !selfCrossingCuts(spans, true).empty();
}

// A strand put back by offsetOpenContour step 7, as chains that do not cross themselves: the
// strand itself, or where it would cross itself (never seen from an offset), its consecutive parts
// between selfCrossingCuts, each without spans no longer than the tolerance at its ends (like any
// open chain); parts left empty go.
std::vector<std::vector<Span>> nonCrossingRuns(std::vector<Span> part, const Tolerance& tol) {
    std::vector<std::vector<Span>> runs;
    const std::vector<std::size_t> cuts = selfCrossingCuts(part);
    if (cuts.empty()) {
        runs.push_back(std::move(part));
        return runs;
    }
    std::size_t first = 0;
    for (std::size_t c = 0; c <= cuts.size(); ++c) {
        const std::size_t last = c < cuts.size() ? cuts[c] : part.size();
        const auto begin = part.begin();
        std::vector<Span> run(begin + static_cast<std::ptrdiff_t>(first),
                              begin + static_cast<std::ptrdiff_t>(last));
        dropTinyEnds(run, tol);
        if (!run.empty()) {
            runs.push_back(std::move(run));
        }
        first = last;
    }
    return runs;
}

// A strand pieced together from raw pieces whose ends meet only to rounding: it is split where
// consecutive pieces are farther apart than rounding (64 ulps of the coordinates; closing a wider
// gap by moving an end would tilt a line unchecked: 1e-9 x the coordinates let a strand at 1e5 mm
// dip 1.56e-6 mm inside d, fam5 0x2d5004b2) or the strand would turn back, the junctions of each
// part are made exact
// (exactJunctions), and a part is split again wherever that cannot be done (two arcs whose ends
// differ by more than exactJunction closes).
constexpr double kRoundingUnits = 64.0;
// Shortest stray (a thin loop's strand or a stretch of dropped opposite copies, put back by
// offsetOpenContour step 7) returned when no other chain covers any of it, in tolerances.
constexpr double kStrayMinimum = 10.0;
constexpr double kEps = std::numeric_limits<double>::epsilon();

std::vector<std::vector<Span>> exactParts(const std::vector<Span>& spans) {
    auto joins = [](const Span& a, const Span& b, bool exactly) {
        const Vec2 e = endPoint(a);
        const Vec2 s = startPoint(b);
        const double scale = std::max({1.0, std::fabs(e.x), std::fabs(e.y)});
        const bool arcs = std::holds_alternative<ArcSpan>(a) && std::holds_alternative<ArcSpan>(b);
        const bool met = exactly ? e == s || (arcs && geometry::distance(e, s) <= 1e-12 * scale)
                                 : geometry::distance(e, s) <= kRoundingUnits * kEps * scale;
        return met && tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) > 0.0;
    };
    auto split = [&](const std::vector<Span>& sp, bool exactly, std::vector<std::vector<Span>>& out) {
        std::size_t from = 0;
        for (std::size_t i = 1; i <= sp.size(); ++i) {
            if (i < sp.size() && joins(sp[i - 1], sp[i], exactly)) {
                continue;
            }
            const auto begin = sp.begin();
            out.emplace_back(begin + static_cast<std::ptrdiff_t>(from), begin + static_cast<std::ptrdiff_t>(i));
            from = i;
        }
    };
    std::vector<std::vector<Span>> rough;
    split(spans, false, rough);
    std::vector<std::vector<Span>> parts;
    for (std::vector<Span>& part : rough) {
        exactJunctions(part, false);
        split(part, true, parts);
    }
    return parts;
}

// One strand of a closed chain too thin to enclose anything (isSliverLoop), given as the raw
// pieces it was stitched from (not the snapped spans: far from the origin, closing the tip of a
// needle narrower than 1e-9 x the coordinates moves a line end across the needle and tilts the
// whole strand by its width, T-0128 channel sweep at 1.2e4 mm). The loop is cut at the two points
// farthest apart (two farthest-point sweeps over samples of its pieces) into two strands that run
// side by side, and the longer is returned, open, with spans no longer than the tolerance dropped
// at its ends; its junctions are left to the caller (exactParts). Empty when the loop's whole
// extent is within the tolerance.
// A tool centre following the strand passes within the loop's width (below tol / 8) of every
// point of the loop (T-0128: such a needle, 3.6 mm long and 5e-6 mm wide, was dropped).
std::vector<Span> sliverStrand(const std::vector<Span>& loop, const Tolerance& tol) {
    constexpr int kSamples = 8;
    const std::size_t n = loop.size();
    std::vector<std::pair<double, Vec2>> samples;
    for (std::size_t i = 0; i < n; ++i) {
        for (int k = 0; k < kSamples; ++k) {
            const double f = static_cast<double>(k) / kSamples;
            samples.emplace_back(static_cast<double>(i) + f, pointAt(loop[i], f));
        }
    }
    auto farthest = [&](Vec2 from) {
        std::size_t best = 0;
        double farDist = -1.0;
        for (std::size_t k = 0; k < samples.size(); ++k) {
            const double dk = geometry::distance(samples[k].second, from);
            if (dk > farDist) {
                farDist = dk;
                best = k;
            }
        }
        return std::pair{best, farDist};
    };
    if (samples.empty()) {
        return {};
    }
    const std::size_t p = farthest(samples.front().second).first;
    const auto [q, extent] = farthest(samples[p].second);
    if (extent <= tol.distance) {
        return {};
    }
    const double lo = std::min(samples[p].first, samples[q].first);
    const double hi = std::max(samples[p].first, samples[q].first);
    std::vector<Span> one = chainPart(loop, lo, hi);
    std::vector<Span> other = chainPart(loop, hi, lo + static_cast<double>(n));
    auto total = [](const std::vector<Span>& sp) {
        double len = 0.0;
        for (const Span& s : sp) {
            len += length(s);
        }
        return len;
    };
    std::vector<Span> strand = total(other) > total(one) ? std::move(other) : std::move(one);
    dropTinyEnds(strand, tol);
    if (partCrossesItself(strand)) {
        // Not a plain needle: fall back to the loop's longest span, which never crosses itself.
        const auto longest = std::max_element(loop.begin(), loop.end(), [](const Span& a, const Span& b) {
            return length(a) < length(b);
        });
        strand.assign(1, *longest);
        dropTinyEnds(strand, tol);
    }
    return strand;
}

// A reversal cusp of a chain (consecutive spans turning back by more than a right angle) where
// the two spans cross again within the tolerance of their junction: the chain runs round a
// sub-tolerance loop there, e.g. the lens between two Round arcs about vertices a hair less than
// 2d apart (T-0017 round 3: 2.1e-4 x 1.9e-9 mm, 0xc500076c), and so crosses itself. Both spans
// are cut back to the crossing farthest from the junction, which drops the loop; what remains
// is unchanged geometry, so the distance bounds still hold.
void removeCuspLoops(std::vector<Span>& spans, bool closed, const Tolerance& tol) {
    const std::size_t n = spans.size();
    if (n < 2) {
        return;
    }
    bool changed = false;
    for (std::size_t k = closed ? 0 : 1; k < n; ++k) {
        Span& a = spans[(k + n - 1) % n];
        Span& b = spans[k];
        if (n == 2 && closed && k == 0) {
            continue;  // a two-span loop: the same pair as k = 1
        }
        if (!endsReverse(a, b)) {
            continue;
        }
        const Vec2 j = startPoint(b);
        const double la = length(a);
        const double lb = length(b);
        const double same = 1e-9 * std::max({1.0, std::fabs(j.x), std::fabs(j.y)});
        std::optional<std::pair<double, double>> cut;
        double reach = 0.0;
        for (const Vec2 x : carrierIntersections(a, b)) {
            const double ta = carrierParam(a, x);
            const double tb = carrierParam(b, x);
            if (!(ta > 0.0 && ta < 1.0 && tb > 0.0 && tb < 1.0)) {
                continue;
            }
            const double back = (1.0 - ta) * la;
            const double ahead = tb * lb;
            if (geometry::distance(x, j) <= same || back > tol.distance || ahead > tol.distance) {
                continue;
            }
            // A real crossing, not rounding: two circles that only touch (to rounding) give two
            // crossings ~1e-8 mm apart whose places are noise (fam5 0x28500192); the crossing must
            // stand off the junction by much more than its own uncertainty, rounding / sin(angle).
            const double sine = std::fabs(tangentAt(a, ta).cross(tangentAt(b, tb)));
            const double unit = std::numeric_limits<double>::epsilon() *
                                std::max({1.0, std::fabs(x.x), std::fabs(x.y), length(a), length(b)});
            if (geometry::distance(x, j) * sine <= 100.0 * unit) {
                continue;
            }
            if (back + ahead > reach) {
                reach = back + ahead;
                cut = std::pair{ta, tb};
            }
        }
        if (!cut) {
            continue;
        }
        const Span cutA = withEnd(a, pointAt(a, cut->first));
        const Span cutB = withStart(b, pointAt(b, cut->second));
        // The two cut spans may meet only at their new junction.
        const Vec2 nj = startPoint(cutB);
        bool clear = true;
        for (const Vec2 x : carrierIntersections(cutA, cutB)) {
            clear = clear && !(inUnit(carrierParam(cutA, x)) && inUnit(carrierParam(cutB, x)) &&
                               geometry::distance(x, nj) > same);
        }
        if (clear) {
            a = cutA;
            b = cutB;
            changed = true;
        }
    }
    if (changed) {
        exactJunctions(spans, closed);
    }
}

// The raw offset chain of one side, with each element's position along the input path (span
// index + fraction at its start and end) for ordering the output chains.
struct RawChain {
    std::vector<Span> spans;
    std::vector<double> posA;
    std::vector<double> posB;
    std::vector<bool> connector;  // a straight connector across an inside corner (always pruned)
    // Parameter range of each element that lies exactly d from the input: an offset span's own
    // extent before any corner extended or trimmed it, a whole Round join arc; empty (lo > hi) for
    // chamfers, connectors and inverted arcs (T-0128: where dropped opposite copies are a path).
    std::vector<std::pair<double, double>> exact;
};

// Raw offset of one side, in input order: every element offset on its own, chained with joins
// (outside corners) and straight connectors (inside corners). An arc whose offset radius would
// be negative is inverted (radius d - r on the far side of its centre); it is pruned later but
// keeps the corner geometry, so the joins about its end points survive.
RawChain buildRaw(const std::vector<Span>& input, double d, double sign, JoinType join, double mitreLimit,
                  const Tolerance& tol, bool closed, std::vector<Wedge>& wedges) {
    RawChain rc;
    std::vector<Span>& raw = rc.spans;
    bool inside = false;  // the elements being recorded are an inside corner's connector
    auto record = [&](double a, double b) {
        while (rc.posA.size() < raw.size()) {
            rc.posA.push_back(a);
            rc.posB.push_back(b);
            rc.connector.push_back(inside);
        }
        inside = false;
    };
    std::vector<std::optional<Span>> unextended;  // per element: the offset span as first computed
    for (std::size_t i = 0; i < input.size(); ++i) {
        pollCancelEvery(i);
        std::optional<Span> next = offsetSpan(input[i], d, sign);
        const std::optional<Span> own = next;
        if (!next) {
            next = invertedOffset(input[i], d, sign);
            if (!next) {
                continue;  // offsets to (nearly) the arc's centre point: nothing to add
            }
        }
        const double at = static_cast<double>(i);
        if (!raw.empty()) {
            inside = classifyCorner(tangentAt(input[i - 1], 1.0), tangentAt(input[i], 0.0), sign) !=
                     CornerKind::Outside;
            connect(raw, *next, startPoint(input[i]), endPoint(input[i - 1]), tangentAt(input[i - 1], 1.0),
                    tangentAt(input[i], 0.0), d, sign, join, mitreLimit, tol, wedges);
            record(at, at);  // joins and connectors sit at the vertex
        }
        raw.push_back(*next);
        record(at, at + 1.0);
        unextended.resize(raw.size());
        unextended.back() = own;
    }
    if (closed && raw.size() > 1) {
        Span first = raw.front();
        inside = classifyCorner(tangentAt(input.back(), 1.0), tangentAt(input.front(), 0.0), sign) !=
                 CornerKind::Outside;
        connect(raw, first, startPoint(input.front()), endPoint(input.back()), tangentAt(input.back(), 1.0),
                tangentAt(input.front(), 0.0), d, sign, join, mitreLimit, tol, wedges);
        raw.front() = first;
        const double at = static_cast<double>(input.size());
        record(at, at);
    }
    unextended.resize(raw.size());
    rc.exact.assign(raw.size(), {1.0, 0.0});
    for (std::size_t i = 0; i < raw.size(); ++i) {
        if (unextended[i]) {
            rc.exact[i] = {std::clamp(carrierParam(raw[i], startPoint(*unextended[i])), 0.0, 1.0),
                           std::clamp(carrierParam(raw[i], endPoint(*unextended[i])), 0.0, 1.0)};
        } else if (join == JoinType::Round && !rc.connector[i] && std::holds_alternative<ArcSpan>(raw[i]) &&
                   std::fabs(std::get<ArcSpan>(raw[i]).radius - d) <=
                       std::min(5e-7 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d)) {
            rc.exact[i] = {0.0, 1.0};  // a Round join: an arc of radius d about an input point
        }
    }
    return rc;
}

// Pruning of the pieces of overlapping raw lines (T-0016). On a dense zig-zag offset by many
// times its pitch, every Sharp or Square corner's mitre or chamfer, and every inside-corner
// connector, lies on one line with hundreds of its neighbours' and overlaps them. Each copy is
// split at all its partners' end points, so a group of k copies has ~k^2 pieces, and every one of
// them was tested against the input and against the ~k corner regions over it: the boundary
// pieces (all the copies of the shared chamfer line) made Sharp and Square 10-30x as slow as
// Round. mergeOnCarriers keeps each stretch of such a group at most once, on the outermost kept
// copy covering it, and only if the kept copies covering it all run one way; whether the other
// copies would have passed does not matter to it. So per stretch and direction the copies are
// tested in element order only until one is kept (all of them where all are pruned, as on the
// inside connectors, where each test is cheap); then the copy merge would start the stretch on is
// tested (by merge's own rule over the copies not known to be pruned: outermost at the stretch's
// start, the previous stretch's element among equals, then outermost at its end, then the lowest
// element), plus any copy that overtakes it before the stretch ends (merge hands over to it; exact
// copies agree to rounding and never do). Only when that copy is pruned are all the others
// tested, as before. A direction therefore has a kept copy
// over a stretch exactly when it had one before, the copy merge keeps is the same one up to
// rounding-level ties, and the boundary is tested about once per stretch instead of k times.
// Stretches come from all the group's piece ends, merged to rounding as in mergeOnCarriers
// (which rebuilds its own from the kept pieces). `state[k]` is -1 for an untested candidate, else
// whether it is kept; `keep(piece)` is the pruning test. Candidates of a group that cover no
// stretch (no longer than the merging distance) and those of other elements are left to the
// caller (`handled` false). Lines only: Sharp and Square add line segments; arcs on one circle
// keep the full test. Inside-corner connectors (`connector`) are left out too: they lie within d
// of their vertex and are always pruned, cheaply, so grouping them only cost time (Round offsets
// of dense zig-zags pile hundreds of them on one line).
void pruneLineCopies(const std::vector<Span>& raw, const std::vector<bool>& connector,
                     const std::vector<Piece>& pieces,
                     const std::vector<std::pair<std::size_t, std::size_t>>& overlapping, double d, double sign,
                     std::vector<signed char>& state, std::vector<bool>& handled,
                     const std::function<bool(const Piece&)>& keep) {
    const std::size_t m = raw.size();
    std::vector<std::size_t> parent(m);
    for (std::size_t i = 0; i < m; ++i) {
        parent[i] = i;
    }
    auto find = [&](std::size_t i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    };
    std::vector<bool> shared(m, false);
    for (const auto& [i, j] : overlapping) {
        if (!std::holds_alternative<LineSpan>(raw[i]) || !std::holds_alternative<LineSpan>(raw[j]) ||
            connector[i] || connector[j]) {
            continue;
        }
        shared[i] = true;
        shared[j] = true;
        const std::size_t a = find(i);
        const std::size_t b = find(j);
        if (a != b) {
            parent[std::max(a, b)] = std::min(a, b);  // the root is the group's lowest element
        }
    }
    std::map<std::size_t, std::vector<std::size_t>> groups;  // root -> candidate indices
    for (std::size_t k = 0; k < pieces.size(); ++k) {
        if (shared[pieces[k].element]) {
            groups[find(pieces[k].element)].push_back(k);
        }
    }
    auto test = [&](std::size_t k) {
        if (state[k] < 0) {
            state[k] = keep(pieces[k]) ? 1 : 0;
        }
        return state[k] == 1;
    };
    struct Copy {
        double lo;
        double hi;
        bool forward;
        std::size_t index;  // into pieces
    };
    using Key = std::pair<std::size_t, std::size_t>;  // (element, copy), ordered as in mergeOnCarriers
    for (const auto& [root, members] : groups) {
        const auto& ref = std::get<LineSpan>(raw[root]);
        const Vec2 origin = ref.start;
        const Vec2 axis = (ref.end - ref.start).normalized();
        double scale = std::max(1.0, d);
        std::vector<Copy> copies;
        copies.reserve(members.size());
        for (const std::size_t k : members) {
            const Piece& p = pieces[k];
            const Vec2 a = pointAt(raw[p.element], p.t0);
            const Vec2 b = pointAt(raw[p.element], p.t1);
            scale = std::max({scale, std::fabs(a.x), std::fabs(a.y), std::fabs(b.x), std::fabs(b.y)});
            const double ca = (a - origin).dot(axis);
            const double cb = (b - origin).dot(axis);
            copies.push_back({std::min(ca, cb), std::max(ca, cb), cb > ca, k});
        }
        const double eps = 1e-9 * scale;
        std::vector<double> ends;
        ends.reserve(2 * copies.size());
        for (const Copy& c : copies) {
            ends.push_back(c.lo);
            ends.push_back(c.hi);
        }
        std::sort(ends.begin(), ends.end());
        std::vector<double> nodes;
        for (const double c : ends) {
            if (nodes.empty() || c > nodes.back() + eps) {
                nodes.push_back(c);
            }
        }
        auto nodeOf = [&](double c) {
            const auto it = std::upper_bound(nodes.begin(), nodes.end(), c);
            return static_cast<std::size_t>(it - nodes.begin()) - 1;
        };
        std::vector<std::vector<std::pair<bool, Key>>> enter(nodes.size());
        std::vector<std::vector<std::pair<bool, Key>>> leave(nodes.size());
        for (std::size_t c = 0; c < copies.size(); ++c) {
            const std::size_t a = nodeOf(copies[c].lo);
            const std::size_t b = nodeOf(copies[c].hi);
            if (a < b) {
                handled[copies[c].index] = true;
                const Key key{pieces[copies[c].index].element, c};
                enter[a].emplace_back(copies[c].forward, key);
                leave[b].emplace_back(copies[c].forward, key);
            }
        }
        // How far the copy lies towards the offset side of the reference at coordinate c (as in
        // mergeOnCarriers).
        auto outwardAt = [&](const Copy& cp, double c) {
            const Piece& p = pieces[cp.index];
            const auto& line = std::get<LineSpan>(raw[p.element]);
            const double along = (line.end - line.start).dot(axis);
            const double t =
                std::clamp(along != 0.0 ? (c - (line.start - origin).dot(axis)) / along : p.t0, 0.0, 1.0);
            const Span& s = raw[p.element];
            return (pointAt(s, t) - (origin + axis * c)).dot(tangentAt(s, t).perp() * sign);
        };
        const double level = 1e-12 * scale;
        std::set<Key> active[2];
        // Direction and element of the copy kept over the previous stretch (mergeOnCarriers'
        // preference among equals).
        std::optional<std::pair<int, std::size_t>> previous;
        // The copy mergeOnCarriers would start the stretch [c0, c1] on, among the covering copies
        // not known to be pruned (only kept ones with `keptOnly`); none when there is no such copy.
        auto choose = [&](const std::set<Key>& covering, int dir, double c0, double c1, bool keptOnly,
                          double& bestEnd) {
            std::optional<std::size_t> best;
            double best0 = -kInf;
            double best1 = -kInf;
            bool bestSticky = false;
            for (const Key& key : covering) {
                const signed char s = state[copies[key.second].index];
                if (s == 0 || (keptOnly && s != 1)) {
                    continue;
                }
                const double v0 = outwardAt(copies[key.second], c0);
                const double v1 = outwardAt(copies[key.second], c1);
                const bool sticky = previous && previous->first == dir && previous->second == key.first;
                const bool better = !best || v0 > best0 + level ||
                                    (v0 >= best0 - level &&
                                     ((sticky && !bestSticky) || (sticky == bestSticky && v1 > best1 + level)));
                if (better) {
                    best0 = std::max(best0, v0);
                    best1 = v1;
                    bestSticky = sticky;
                    best = key.second;
                }
            }
            bestEnd = best1;
            return best;
        };
        for (std::size_t k = 0; k + 1 < nodes.size(); ++k) {
            for (const auto& [fw, key] : leave[k]) {
                active[fw ? 1 : 0].erase(key);
            }
            for (const auto& [fw, key] : enter[k]) {
                active[fw ? 1 : 0].insert(key);
            }
            const double c0 = nodes[k];
            const double c1 = nodes[k + 1];
            std::optional<std::size_t> keptBy[2];
            for (int dir = 0; dir < 2; ++dir) {
                const std::set<Key>& covering = active[dir];
                if (covering.empty()) {
                    continue;
                }
                // First whether any copy is kept at all: where every copy is pruned (the inside
                // connectors of a dense zig-zag, deep in the neighbourhood) that is all the work,
                // each test cheap, and nothing needs choosing.
                bool anyKept = false;
                for (const Key& key : covering) {
                    if (test(copies[key.second].index)) {
                        anyKept = true;
                        break;
                    }
                }
                if (!anyKept) {
                    continue;
                }
                double end = 0.0;
                const std::optional<std::size_t> best = choose(covering, dir, c0, c1, false, end);
                if (best && test(copies[*best].index)) {
                    keptBy[dir] = pieces[copies[*best].index].element;
                    // Copies that overtake it before c1 (mergeOnCarriers hands the stretch over to
                    // them): tested too. Exact copies agree to rounding and never do.
                    for (const Key& key : covering) {
                        if (key.second != *best && state[copies[key.second].index] < 0 &&
                            outwardAt(copies[key.second], c1) > end + level) {
                            test(copies[key.second].index);
                        }
                    }
                    continue;
                }
                // Pruned: then every copy is tested, as before T-0016, and merge chooses among
                // the kept ones.
                for (const Key& key : covering) {
                    test(copies[key.second].index);
                }
                if (const auto kept = choose(covering, dir, c0, c1, true, end)) {
                    keptBy[dir] = pieces[copies[*kept].index].element;
                }
            }
            if (keptBy[0].has_value() == keptBy[1].has_value()) {
                previous.reset();  // nothing kept, or copies both ways: merge drops the stretch
            } else {
                const int dir = keptBy[1] ? 1 : 0;
                previous = std::pair{dir, *keptBy[dir]};
            }
        }
    }
}

// Kept pieces of raw elements that lie on one line or circle, merged into one boundary piece per
// stretch. Overlapping elements (the mitre extensions of a slot narrower than 2d, the chamfers
// or mitres of neighbouring peaks of a steep zig-zag, the inside-corner connectors of a dense
// zig-zag at a large d) keep copies of the same stretch. A copy running the same way is the
// same boundary piece; two copies running opposite ways bound a region of zero width inside the
// offset shape and both go. Matching copies by their end points failed where the copies were
// cut differently: each element computes its own boundary crossings, and a nearly tangent one
// (the bottom of a vertex circle touching a chamfer line) lands up to ~1e-7 mm apart on two
// copies, and pieces shorter than the tolerance could not be matched to the tolerance at all.
// Both copies of such an overlap survived and the chain stepped back over it (T-0006).
//
// So every group of elements on one carrier (pairs from `overlapping`, joined transitively) is
// swept along a common coordinate: distance along the line, or angle round the circle. All
// piece ends of the group are merged where they agree to rounding (1e-9 x max(1, d, |x|, |y|),
// or that over the radius for angles), and each elementary stretch between consecutive ends is
// kept once if all the copies covering it run one way, on the copy outermost along it (see
// the sweep below), and dropped otherwise. Every piece end stays a piece end, so the stitching
// can still turn at any point where another piece starts or ends. Cost O(P log P) per group of P
// pieces, plus per stretch a pass over the copies covering it for each hand-over; pieces of elements outside
// every group are untouched. The result is ordered by element and parameter, like the input.
std::vector<Piece> mergeOnCarriers(const std::vector<Span>& raw, const std::vector<Piece>& kept,
                                   const std::vector<std::pair<std::size_t, std::size_t>>& overlapping,
                                   double d, double sign, double lateral, double opposite,
                                   const std::vector<std::pair<double, double>>& exact,
                                   std::vector<Piece>& dropped) {
    const std::size_t m = raw.size();
    std::vector<std::size_t> parent(m);
    for (std::size_t i = 0; i < m; ++i) {
        parent[i] = i;
    }
    auto find = [&](std::size_t i) {
        while (parent[i] != i) {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        return i;
    };
    std::vector<bool> shared(m, false);
    auto unite = [&](std::size_t i, std::size_t j) {
        shared[i] = true;
        shared[j] = true;
        const std::size_t a = find(i);
        const std::size_t b = find(j);
        if (a != b) {
            parent[std::max(a, b)] = std::min(a, b);  // the root is the group's lowest element
        }
    };
    for (const auto& [i, j] : overlapping) {
        unite(i, j);
    }
    // Nearly the same carrier: kept pieces of two elements that overlap along a common stretch
    // and stay within `lateral` of each other's line or circle there. Raw elements whose carriers
    // differ by more than rounding but less than the tolerance (vertex noise of 1e-9..1e-5 mm in
    // DXF or computed input, coordinates far from the origin) meet at ill-conditioned crossings
    // that do not cut their copies consistently either; both copies are boundary (each passed the
    // distance pruning), so keeping the outer one per stretch keeps the offset at least d from the
    // input and moves it by at most `lateral`. Only nearly parallel elements qualify, by a real
    // angle limit (kGroupSin) as well as by their sideways drift over their length: a short piece
    // crossing a carrier at 11-20 degrees (T-0017 R4) is a different boundary piece, not a copy.
    // Pieces that merely continue each other end to end share no stretch and are not grouped, and
    // an element joins a group only if it is also nearly parallel to the group's first element,
    // so a gently turning run of short elements cannot chain into one group (T-0017).
    constexpr double kGroupSin = 1e-3;
    std::vector<Span> pieceSpans;
    std::vector<Box2> pieceBoxes;
    pieceSpans.reserve(kept.size());
    pieceBoxes.reserve(kept.size());
    for (const Piece& p : kept) {
        pieceSpans.push_back(subSpan(raw[p.element], p.t0, p.t1));
        pieceBoxes.push_back(bounds(pieceSpans.back()).inflated(lateral));
    }
    auto onCarrierOf = [&](Vec2 q, const Span& s, double band) {
        if (const auto* line = std::get_if<LineSpan>(&s)) {
            const Vec2 u = line->end - line->start;
            return std::fabs(u.cross(q - line->start)) <= band * u.length();
        }
        const auto& arc = std::get<ArcSpan>(s);
        return std::fabs(geometry::distance(q, arc.centre) - arc.radius) <= band;
    };
    // Elements nearly parallel or antiparallel (lines) or nearly concentric (arcs); with a
    // non-negative `band`, also within it sideways over their whole length (a line's drift, an
    // arc's centre and radius).
    auto nearlyParallel = [&](const Span& ea, const Span& eb, double angleLimit, double band) {
        if (ea.index() != eb.index()) {
            return false;
        }
        if (const auto* arcA = std::get_if<ArcSpan>(&ea)) {
            const auto& arcB = std::get<ArcSpan>(eb);
            const double off = geometry::distance(arcA->centre, arcB.centre);
            return off <= angleLimit * std::min(arcA->radius, arcB.radius) &&
                   (band < 0.0 || (off <= 2.0 * band && std::fabs(arcA->radius - arcB.radius) <= band));
        }
        const double s = std::fabs(tangentAt(ea, 0.0).cross(tangentAt(eb, 0.0)));
        return s <= angleLimit && (band < 0.0 || s * std::max(length(ea), length(eb)) <= band);
    };
    // Some point of `y` (an end or its middle) lies strictly inside `x`'s extent, measured along
    // x's carrier: the two share a stretch of positive length.
    auto reachesInto = [&](const Span& x, const Span& y) {
        const double len = length(x);
        if (len <= 0.0) {
            return false;
        }
        for (const double f : {0.0, 0.5, 1.0}) {
            const Vec2 q = pointAt(y, f);
            const double margin = 1e-9 * std::max({1.0, std::fabs(q.x), std::fabs(q.y)}) / len;
            const double t = carrierParam(x, q);
            if (t > margin && t < 1.0 - margin) {
                return true;
            }
        }
        return false;
    };
    // Copies running the same way are grouped within `lateral`. Copies running opposite ways
    // bound a sliver that the merge drops (as many copies run each way), so their neighbours are
    // then joined sideways across its width: only within `opposite` (the snapping allowance) is
    // that a rounding repair. Wider opposite pairs are a real thin feature, e.g. a needle between
    // two strands 1e-4 mm apart (T-0017 R6), and stay apart.
    auto nearCarrier = [&](std::size_t a, std::size_t b) {
        const Span& sa = pieceSpans[a];
        const Span& sb = pieceSpans[b];
        const Span& ea = raw[kept[a].element];
        const Span& eb = raw[kept[b].element];
        if (ea.index() != eb.index()) {
            return false;
        }
        const bool same = std::holds_alternative<LineSpan>(ea)
                              ? tangentAt(ea, 0.0).dot(tangentAt(eb, 0.0)) > 0.0
                              : std::get<ArcSpan>(ea).direction == std::get<ArcSpan>(eb).direction;
        const double band = same ? lateral : opposite;
        if (!nearlyParallel(ea, eb, kGroupSin, band)) {
            return false;
        }
        if (std::holds_alternative<LineSpan>(sa) &&
            (!onCarrierOf(startPoint(sb), ea, band) || !onCarrierOf(endPoint(sb), ea, band) ||
             !onCarrierOf(startPoint(sa), eb, band) || !onCarrierOf(endPoint(sa), eb, band))) {
            return false;
        }
        return reachesInto(sa, sb) || reachesInto(sb, sa);
    };
    {
        BoxGrid pieceGrid(pieceBoxes);
        for (std::size_t a = 0; a < kept.size(); ++a) {
            pieceGrid.query(pieceBoxes[a], [&](std::size_t b) {
                const std::size_t ea = kept[a].element;
                const std::size_t eb = kept[b].element;
                if (b <= a || ea == eb || find(ea) == find(eb)) {
                    return;
                }
                if (nearCarrier(a, b) && nearlyParallel(raw[find(ea)], raw[eb], 2.0 * kGroupSin, -1.0) &&
                    nearlyParallel(raw[find(eb)], raw[ea], 2.0 * kGroupSin, -1.0) &&
                    nearlyParallel(raw[find(ea)], raw[find(eb)], 2.0 * kGroupSin, -1.0)) {
                    unite(ea, eb);
                }
            });
        }
    }
    std::vector<Piece> out;
    std::map<std::size_t, std::vector<Piece>> groups;
    for (const Piece& p : kept) {
        if (shared[p.element]) {
            groups[find(p.element)].push_back(p);
        } else {
            out.push_back(p);  // not on a shared carrier
        }
    }

    // A stretch [lo, hi] of the common coordinate covered by one piece, which way it runs, and
    // the piece it came from (element and parameter range).
    struct Stretch {
        double lo;
        double hi;
        bool forward;
        Piece piece;
    };
    for (const auto& [root, pieces] : groups) {
        const Span& ref = raw[root];
        const auto* refLine = std::get_if<LineSpan>(&ref);
        const auto* refArc = std::get_if<ArcSpan>(&ref);
        if (!refLine && !refArc) {
            out.insert(out.end(), pieces.begin(), pieces.end());
            continue;
        }
        const Vec2 origin = refLine ? refLine->start : refArc->centre;
        const Vec2 axis = refLine ? (refLine->end - refLine->start).normalized() : Vec2{};
        const double radius = refArc ? refArc->radius : 1.0;
        const double refAngle = refArc ? refArc->startAngle : 0.0;
        auto coordOf = [&](Vec2 p) {
            return refLine ? (p - origin).dot(axis) : normalizeAngle((p - origin).angle() - refAngle);
        };
        auto pointOf = [&](double c) {
            return refLine ? origin + axis * c : origin + Vec2::fromAngle(refAngle + c, radius);
        };
        double scale = std::max(1.0, d);
        std::vector<Stretch> stretches;
        for (const Piece& p : pieces) {
            const Span& s = raw[p.element];
            const Vec2 a = pointAt(s, p.t0);
            const Vec2 b = pointAt(s, p.t1);
            scale = std::max({scale, std::fabs(a.x), std::fabs(a.y), std::fabs(b.x), std::fabs(b.y)});
            if (refLine) {
                const double ca = coordOf(a);
                const double cb = coordOf(b);
                stretches.push_back({std::min(ca, cb), std::max(ca, cb), cb > ca, p});
                continue;
            }
            // Angles about the reference centre: a piece runs CCW (increasing angle) or CW from
            // its start, by about its own sweep; one that passes the reference angle is cut there.
            const auto& arc = std::get<ArcSpan>(s);
            const bool ccw = arc.direction == ArcDirection::CCW;
            const double own = std::fabs(arc.sweepAngle()) * (p.t1 - p.t0);
            const double ca = coordOf(a);
            double span = normalizeAngle(ccw ? coordOf(b) - ca : ca - coordOf(b));
            if (std::fabs(span + kTwoPi - own) < std::fabs(span - own)) {
                span += kTwoPi;  // a (nearly) full circle: the end angle wrapped round to the start
            }
            const double lo = ccw ? ca : ca - span;
            const double hi = ccw ? ca + span : ca;
            if (lo < 0.0) {
                stretches.push_back({lo + kTwoPi, kTwoPi, ccw, p});
                stretches.push_back({0.0, hi, ccw, p});
            } else if (hi > kTwoPi) {
                stretches.push_back({lo, kTwoPi, ccw, p});
                stretches.push_back({0.0, hi - kTwoPi, ccw, p});
            } else {
                stretches.push_back({lo, hi, ccw, p});
            }
        }
        // Parameter on a stretch's own element of the point at coordinate c: the exact inverse of
        // coordOf restricted to that element (a line's point whose projection is c; an arc's point
        // on the ray from the reference centre at angle c). Every stretch handed to an owner lies
        // within its own piece's coordinates, up to the merging of nearby ends into one node, so a
        // merged stretch is part of a kept piece of its owner (extended by at most that rounding,
        // which keeps it continuous with a neighbour whose sub-rounding piece was merged away),
        // never a projection of it onto someone else's carrier (T-0017 R4).
        auto paramAt = [&](const Stretch& st, double c) {
            const Span& s = raw[st.piece.element];
            double t = 0.0;
            if (const auto* line = std::get_if<LineSpan>(&s)) {
                const double along = (line->end - line->start).dot(axis);
                t = along != 0.0 ? (c - (line->start - origin).dot(axis)) / along : st.piece.t0;
            } else {
                const auto& arc = std::get<ArcSpan>(s);
                const Vec2 u = Vec2::fromAngle(refAngle + c);
                const Vec2 w = origin - arc.centre;
                const double b = w.dot(u);
                const double rho =
                    -b + std::sqrt(std::max(0.0, b * b - (w.lengthSquared() - arc.radius * arc.radius)));
                const double ang = (origin + u * rho - arc.centre).angle();
                const double a0 = arc.angleAt(st.piece.t0);
                const double sweep = std::fabs(arc.sweepAngle());
                const double pieceSweep = sweep * (st.piece.t1 - st.piece.t0);
                double adv =
                    arc.direction == ArcDirection::CCW ? normalizeAngle(ang - a0) : normalizeAngle(a0 - ang);
                if (adv > pieceSweep + 0.5 * (kTwoPi - pieceSweep)) {
                    adv -= kTwoPi;  // just before the piece's start
                }
                t = st.piece.t0 + adv / sweep;
            }
            return std::clamp(t, 0.0, 1.0);
        };
        // How far the copy lies towards the offset side of the reference at coordinate c.
        auto outwardAt = [&](const Stretch& st, double c) {
            const Span& s = raw[st.piece.element];
            const double t = paramAt(st, c);
            return (pointAt(s, t) - pointOf(c)).dot(tangentAt(s, t).perp() * sign);
        };
        const double eps = 1e-9 * scale / (refLine ? 1.0 : radius);
        // Merged end coordinates; each stretch then covers ends [first, last).
        std::vector<double> ends;
        for (const Stretch& st : stretches) {
            ends.push_back(st.lo);
            ends.push_back(st.hi);
        }
        std::sort(ends.begin(), ends.end());
        std::vector<double> nodes;
        for (const double c : ends) {
            if (nodes.empty() || c > nodes.back() + eps) {
                nodes.push_back(c);
            }
        }
        auto nodeOf = [&](double c) {
            // The last node at or below c: the node c was merged into (a node is the smallest
            // value of its run, and the next node lies more than eps above that run's start).
            const auto it = std::upper_bound(nodes.begin(), nodes.end(), c);
            return static_cast<std::size_t>(it - nodes.begin()) - 1;
        };
        // Sweep: copies (element, stretch index) entering and leaving at each node.
        using Copy = std::pair<std::size_t, std::size_t>;
        std::vector<std::vector<std::pair<bool, Copy>>> enter(nodes.size());
        std::vector<std::vector<std::pair<bool, Copy>>> leave(nodes.size());
        for (std::size_t i = 0; i < stretches.size(); ++i) {
            const Stretch& st = stretches[i];
            const std::size_t a = nodeOf(st.lo);
            const std::size_t b = nodeOf(st.hi);
            if (a < b) {
                enter[a].emplace_back(st.forward, Copy{st.piece.element, i});
                leave[b].emplace_back(st.forward, Copy{st.piece.element, i});
            }
        }
        std::set<Copy> forward;
        std::set<Copy> backward;
        std::optional<std::pair<bool, std::size_t>> previous;  // direction and element of the last stretch
        const double level = 1e-12 * scale;  // rounding only: any real difference picks the outer
        auto emitOn = [&](std::size_t st, double c0, double c1, bool fw) {
            const Stretch& s = stretches[st];
            const double t0 = paramAt(s, fw ? c0 : c1);
            const double t1 = paramAt(s, fw ? c1 : c0);
            if (t1 > t0) {
                out.push_back(Piece{s.piece.element, t0, t1});
            }
            previous = std::pair{fw, s.piece.element};
        };
        for (std::size_t k = 0; k + 1 < nodes.size(); ++k) {
            for (const auto& [fw, copy] : leave[k]) {
                (fw ? forward : backward).erase(copy);
            }
            for (const auto& [fw, copy] : enter[k]) {
                (fw ? forward : backward).insert(copy);
            }
            if (forward.empty() == backward.empty()) {
                // Nothing, or copies running both ways: they bound a region of zero width
                // whatever their numbers (two the same way are one boundary), T-0017 0xe20006d8.
                // The stretch is still a path a tool centre can follow (the middle of a slot one
                // tool width wide): one copy is handed back (the first, forward copies before
                // backward ones, that lies exactly d from the input there, see RawChain::exact),
                // and put back where the chains leave it uncovered (T-0128, offsetOpenContour
                // step 7).
                auto exactOver = [&](const Piece& p) {
                    return exact[p.element].first <= p.t0 + kParamEps &&
                           p.t1 <= exact[p.element].second + kParamEps;
                };
                std::optional<Piece> back;
                std::optional<Piece> fallback;
                for (const bool fw : {true, false}) {
                    for (const Copy& copy : fw ? forward : backward) {
                        const Stretch& st = stretches[copy.second];
                        const Piece p{st.piece.element, paramAt(st, fw ? nodes[k] : nodes[k + 1]),
                                      paramAt(st, fw ? nodes[k + 1] : nodes[k])};
                        if (p.t1 > p.t0 && !back) {
                            if (exactOver(p)) {
                                back = p;
                            } else if (!fallback) {
                                fallback = p;
                            }
                        }
                    }
                }
                if (!back) {
                    back = fallback;
                }
                if (back) {
                    dropped.push_back(*back);
                }
                previous.reset();
                continue;
            }
            const bool fw = !forward.empty();
            const std::set<Copy>& covering = fw ? forward : backward;
            // Keep the outermost copy (farthest towards the offset side), which is at least d
            // from the inputs of all the copies, along the whole stretch: start with the one
            // outermost at the stretch's start (level to rounding: the previous stretch's element,
            // so no sideways jog, then the one outermost at the far end, then the lowest-numbered)
            // and hand over wherever another copy overtakes it (their carriers cross).
            double c0 = nodes[k];
            const double c1 = nodes[k + 1];
            std::size_t cur = covering.begin()->second;
            {
                double best0 = -kInf;
                double best1 = -kInf;
                bool bestSticky = false;
                for (const Copy& copy : covering) {
                    const double v0 = outwardAt(stretches[copy.second], c0);
                    const double v1 = outwardAt(stretches[copy.second], c1);
                    const bool sticky = previous && previous->first == fw && previous->second == copy.first;
                    const bool better =
                        v0 > best0 + level ||
                        (v0 >= best0 - level &&
                         ((sticky && !bestSticky) || (sticky == bestSticky && v1 > best1 + level)));
                    if (better) {
                        best0 = std::max(best0, v0);
                        best1 = v1;
                        bestSticky = sticky;
                        cur = copy.second;
                    }
                }
            }
            for (std::size_t guard = 0; guard <= covering.size(); ++guard) {
                std::optional<std::size_t> next;
                double handOver = c1;
                const double cur1 = outwardAt(stretches[cur], c1);
                for (const Copy& copy : covering) {
                    if (copy.second == cur || outwardAt(stretches[copy.second], c1) <= cur1 + level) {
                        continue;
                    }
                    // Overtakes `cur` before c1: bisect for where the two cross.
                    double lo = c0;
                    double hi = c1;
                    for (int it = 0; it < 60 && hi - lo > eps * 1e-3; ++it) {
                        const double mid = 0.5 * (lo + hi);
                        if (outwardAt(stretches[copy.second], mid) > outwardAt(stretches[cur], mid) + level) {
                            hi = mid;
                        } else {
                            lo = mid;
                        }
                    }
                    if (hi < handOver) {
                        handOver = hi;
                        next = copy.second;
                    }
                }
                if (!next) {
                    break;
                }
                emitOn(cur, c0, handOver, fw);
                c0 = handOver;
                cur = *next;
            }
            emitOn(cur, c0, c1, fw);
        }
    }
    std::sort(out.begin(), out.end(), [](const Piece& a, const Piece& b) {
        return a.element != b.element ? a.element < b.element : a.t0 < b.t0;
    });
    return out;
}

// A thin loop's strand or a stretch of dropped opposite copies (T-0128), with the input position
// it is ordered by.
struct Stray {
    double key;
    std::vector<Span> spans;
};

// offsetOpenContour step 7 (see there): puts the strays back into `result` (whose chains start
// at `resultKeys` along the input) where no chain covers them.
void putBackStrays(std::vector<Contour>& result, const std::vector<double>& resultKeys,
                   const std::vector<Stray>& strays, const Tolerance& tol) {
    std::vector<const Span*> outSpans;
    std::vector<Box2> outBoxes;
    for (const Contour& c : result) {
        for (const Span& sp : c.spans) {
            outSpans.push_back(&sp);
            outBoxes.push_back(bounds(sp));
        }
    }
    const double reach = 0.5 * tol.distance;
    std::vector<Stray> added;
    // The emitted chains and the strays put back as they are added (a linear scan over those made
    // the recovery quadratic in their number: a hatch at pitch exactly 2d, T-0128 round 1), each
    // in a BoxIndex laid out over the chains or over the strays (every piece put back lies on
    // one): a grid for the boxes that spread evenly, a tree for those that crowd a cell or cover
    // many (T-0172: main's uniform grids put a whole hatch at pitch 2d next to one far feature
    // into one cell; a grid that subdivided crowded cells blew up on a spiral slot's nested arcs;
    // a k-d tree alone cost ~3.4 sqrt(n) per query on a hatch with varied line ends).
    detail::BoxIndex outCover(outBoxes);
    std::vector<Span> addedSpans;
    Box2 strayExtent;
    std::size_t straySpans = 0;
    for (const Stray& st : strays) {
        for (const Span& sp : st.spans) {
            strayExtent.expand(bounds(sp));
        }
        straySpans += st.spans.size();
    }
    detail::BoxIndex addedCover(strayExtent, straySpans);
    auto add = [&](double key, std::vector<Span>&& part) {
        for (const Span& sp : part) {
            addedSpans.push_back(sp);
            addedCover.add(bounds(sp));
        }
        added.push_back(Stray{key, std::move(part)});
    };
    // Samples every tol / 4 along each strand. The distance g to the chains is 1-Lipschitz along
    // it, so a sample vouches for the next |g - tol / 2| (all covered or all uncovered), which
    // is skipped.
    const double step = 0.25 * tol.distance;
    // The query reach starts at 16 tol and doubles, up to 1024 tol, while nothing lies within
    // it, so a long uncovered strand costs a few samples per mm, not one per 16 tol (T-0128
    // round 1: a hatch at pitch exactly 2d spent most of its time sampling such strands).
    const double skipCap = reach + 16.0 * tol.distance;
    const double maxCap = reach + 1024.0 * tol.distance;
    // Distance from q to the emitted chains and the strays put back, exact up to `cap`.
    auto coverGap = [&](Vec2 q, double cap) {
        const Box2 around{q - Vec2{cap, cap}, q + Vec2{cap, cap}};
        double best = cap;
        outCover.query(around,
                       [&](std::size_t i) { best = std::min(best, distanceToSpan(q, *outSpans[i])); });
        addedCover.query(around,
                         [&](std::size_t i) { best = std::min(best, distanceToSpan(q, addedSpans[i])); });
        return best;
    };
    struct Sample {
        double u;  // span index + fraction
        bool covered;
    };
    for (const Stray& st : strays) {
        std::vector<Sample> samples;
        double cap = skipCap;
        for (std::size_t i = 0; i < st.spans.size(); ++i) {
            const double len = length(st.spans[i]);
            const bool lastSpan = i + 1 == st.spans.size();
            for (double s = 0.0;;) {
                const double f = len > 0.0 ? std::min(s / len, 1.0) : 1.0;
                if (f >= 1.0 && !lastSpan) {
                    break;  // the next span starts here
                }
                const Vec2 q = pointAt(st.spans[i], f);
                const double g = coverGap(q, cap);
                cap = g >= cap ? std::min(2.0 * cap, maxCap) : skipCap;
                samples.push_back(Sample{static_cast<double>(i) + f, g <= reach});
                const double advance = std::max(step, std::fabs(g - reach));
                if (f >= 1.0) {
                    break;
                }
                s += advance;
            }
        }
        for (std::size_t k = 0; k < samples.size();) {
            if (samples[k].covered) {
                ++k;
                continue;
            }
            std::size_t e = k;
            while (e + 1 < samples.size() && !samples[e + 1].covered) {
                ++e;
            }
            // Widened into the covered neighbours.
            const std::size_t from = k > 0 ? k - 1 : k;
            const std::size_t to = e + 1 < samples.size() ? e + 1 : e;
            std::vector<Span> part = chainPart(st.spans, samples[from].u, samples[to].u);
            dropTinyEnds(part, tol);
            double len = 0.0;
            for (const Span& sp : part) {
                len += length(sp);
            }
            // A run widened at neither end is the whole strand, none of it covered: such an
            // isolated piece is put back only from kStrayMinimum x the tolerance (owner decision
            // 2026-11-05: an isolated 0.005 mm stray, fam0 0x71280859, is noise to the user; a
            // needle or slot that matters is mm long).
            const bool isolated = from == k && to == e;
            if (len > (isolated ? kStrayMinimum : 1.0) * tol.distance) {
                exactJunctions(part, false);
                for (std::vector<Span>& run : nonCrossingRuns(std::move(part), tol)) {
                    add(st.key, std::move(run));
                }
            }
            k = e + 1;
        }
    }
    // Each goes after the chains (and earlier strays) whose keys are not greater: a stable
    // merge, linear (inserting one at a time was quadratic too).
    std::stable_sort(added.begin(), added.end(),
                     [](const Stray& a, const Stray& b) { return a.key < b.key; });
    std::vector<Contour> merged;
    merged.reserve(result.size() + added.size());
    std::size_t r = 0;
    for (Stray& a : added) {
        while (r < result.size() && !(a.key < resultKeys[r])) {
            merged.push_back(std::move(result[r++]));
        }
        Contour c;
        c.spans = std::move(a.spans);
        merged.push_back(std::move(c));
    }
    while (r < result.size()) {
        merged.push_back(std::move(result[r++]));
    }
    result = std::move(merged);
}


// Nearest-point search over a fixed set of points (an implicit k-d tree: the median of each
// index range is its node, the axis alternates with depth).
class PointTree {
   public:
    explicit PointTree(std::vector<Vec2> points)
        : points_(std::move(points)), order_(points_.size()), lo_(points_.size()), hi_(points_.size()) {
        for (std::size_t i = 0; i < order_.size(); ++i) {
            order_[i] = i;
        }
        build(0, order_.size(), 0);
    }

    // Distance from `p` to the nearest point and that point. The search gives up after a fixed
    // number of nodes (points all about as far from `p` as each other, e.g. the vertices of a
    // circle seen from its centre, defeat the pruning): the point returned is then the nearest
    // seen, never nearer than the true one.
    double nearest(Vec2 p, Vec2& found) const {
        double best = std::numeric_limits<double>::infinity();
        std::size_t budget = kBudget;
        search(0, order_.size(), p, best, found, budget);
        return std::sqrt(best);
    }

   private:
    static constexpr std::size_t kBudget = 1000;

    static double coord(Vec2 v, int axis) { return axis == 0 ? v.x : v.y; }

    // Fills lo_/hi_ (the bounds of the points of [lo, hi), kept at the node's median slot).
    void build(std::size_t lo, std::size_t hi, int axis) {
        if (lo >= hi) {
            return;
        }
        const std::size_t mid = lo + (hi - lo) / 2;
        if (hi - lo >= 2) {
            std::nth_element(order_.begin() + static_cast<std::ptrdiff_t>(lo),
                             order_.begin() + static_cast<std::ptrdiff_t>(mid),
                             order_.begin() + static_cast<std::ptrdiff_t>(hi), [&](std::size_t a, std::size_t b) {
                                 return coord(points_[a], axis) < coord(points_[b], axis);
                             });
        }
        build(lo, mid, 1 - axis);
        build(mid + 1, hi, 1 - axis);
        Vec2 low = points_[order_[mid]];
        Vec2 high = low;
        auto include = [&](std::size_t from, std::size_t to) {
            if (from < to) {
                const std::size_t child = from + (to - from) / 2;
                low = Vec2{std::min(low.x, lo_[child].x), std::min(low.y, lo_[child].y)};
                high = Vec2{std::max(high.x, hi_[child].x), std::max(high.y, hi_[child].y)};
            }
        };
        include(lo, mid);
        include(mid + 1, hi);
        lo_[mid] = low;
        hi_[mid] = high;
    }

    // Squared distance from `p` to the bounds of the points of [lo, hi) (infinite when empty).
    double gap2(std::size_t lo, std::size_t hi, Vec2 p) const {
        if (lo >= hi) {
            return std::numeric_limits<double>::infinity();
        }
        const std::size_t mid = lo + (hi - lo) / 2;
        const double gx = std::max({lo_[mid].x - p.x, p.x - hi_[mid].x, 0.0});
        const double gy = std::max({lo_[mid].y - p.y, p.y - hi_[mid].y, 0.0});
        return gx * gx + gy * gy;
    }

    // The nearer child (by its bounds) is searched first.
    void search(std::size_t lo, std::size_t hi, Vec2 p, double& best, Vec2& found, std::size_t& budget) const {
        if (lo >= hi || budget == 0) {
            return;
        }
        --budget;
        const std::size_t mid = lo + (hi - lo) / 2;
        const Vec2 q = points_[order_[mid]];
        const double dx = p.x - q.x;
        const double dy = p.y - q.y;
        const double d2 = dx * dx + dy * dy;
        if (d2 < best) {
            best = d2;
            found = q;
        }
        const double left = gap2(lo, mid, p);
        const double right = gap2(mid + 1, hi, p);
        if (left <= right) {
            if (left < best) {
                search(lo, mid, p, best, found, budget);
            }
            if (right < best) {
                search(mid + 1, hi, p, best, found, budget);
            }
        } else {
            if (right < best) {
                search(mid + 1, hi, p, best, found, budget);
            }
            if (left < best) {
                search(lo, mid, p, best, found, budget);
            }
        }
    }

    std::vector<Vec2> points_;
    std::vector<std::size_t> order_;
    std::vector<Vec2> lo_;
    std::vector<Vec2> hi_;
};

// The part [t0, t1] of a line or arc as an element of its own (built from the parameters, so a
// piece of a full circle keeps its sweep).
Span rangeOf(const Span& s, double t0, double t1) {
    if (const auto* arc = std::get_if<ArcSpan>(&s)) {
        const double sweep = arc->sweepAngle();
        return ArcSpan{arc->centre, arc->radius, arc->startAngle + t0 * sweep, arc->startAngle + t1 * sweep,
                       arc->direction};
    }
    return LineSpan{pointAt(s, t0), pointAt(s, t1)};
}

using ParamRanges = std::vector<std::pair<double, double>>;

bool inRanges(const ParamRanges& ranges, double t) {
    const auto it = std::lower_bound(ranges.begin(), ranges.end(), t,
                                     [](const std::pair<double, double>& r, double v) { return r.second < v; });
    return it != ranges.end() && it->first <= t;
}

// Parameter ranges of `s` that are dominated: `covers(piece, hull)` says that a piece (with its
// hull points, see hullPoints) lies wholly inside one disc of radius `limit` about an input point
// or strictly inside one Sharp/Square corner region, so every sample of it is pruned like those
// of the elements `dead` marks. The test is local: a long raw line or arc of a path that
// wanders inside its own d-neighbourhood is mostly covered by *different* nearby discs or
// regions, which the whole-element test cannot see. A piece that is not covered as a whole is
// halved, and given up on, as live, when `covered(point)` is false for its ends and its middle
// or when it is shorter than `minLength`.
// A range left out only costs time.
template <class Covers, class Covered>
void deadRangesOf(const Span& s, double t0, double t1, int depth, double minLength, const Covers& covers,
                  const Covered& covered, ParamRanges& out, std::vector<Vec2>& hull) {
    constexpr int kMaxDepth = 14;
    const double tm = 0.5 * (t0 + t1);
    const Vec2 mid = pointAt(s, tm);
    hullPoints(rangeOf(s, t0, t1), hull);
    if (covers(mid, hull)) {
        if (!out.empty() && out.back().second == t0) {
            out.back().second = t1;
        } else {
            out.emplace_back(t0, t1);
        }
        return;
    }
    if (depth >= kMaxDepth || (t1 - t0) * length(s) < minLength) {
        return;
    }
    if (!covered(mid) && !covered(pointAt(s, t0)) && !covered(pointAt(s, t1))) {
        return;
    }
    deadRangesOf(s, t0, tm, depth + 1, minLength, covers, covered, out, hull);
    deadRangesOf(s, tm, t1, depth + 1, minLength, covers, covered, out, hull);
}

// Sharp/Square corner regions that are convex polygons (the usual mitre and chamfer shapes),
// for the strictly-inside test without the general ray-crossing count.
class ConvexRegions {
   public:
    ConvexRegions(const std::vector<Wedge>& wedges, double margin) : margin_(margin) {
        std::vector<Box2> boxes;
        for (const Wedge& w : wedges) {
            std::vector<Vec2> pts;
            bool ok = w.size() >= 3;
            for (const Span& e : w) {
                const auto* line = std::get_if<LineSpan>(&e);
                if (!line) {
                    ok = false;
                    break;
                }
                pts.push_back(line->start);
            }
            if (ok) {
                ok = orient(pts);
            }
            if (ok) {
                Box2 b;
                for (const Vec2 p : pts) {
                    b.expand(Box2{p, p});
                }
                boxes.push_back(b);
                std::vector<Edge> edges;
                for (std::size_t i = 0; i < pts.size(); ++i) {
                    const Vec2 a = pts[i];
                    const Vec2 e = pts[(i + 1) % pts.size()] - a;
                    const double len = e.length();
                    if (len > 0.0) {
                        // Inward unit normal of a counter-clockwise polygon: the left of the edge.
                        const double nx = -e.y / len;
                        const double ny = e.x / len;
                        edges.push_back(Edge{nx, ny, nx * a.x + ny * a.y});
                    }
                }
                polygons_.push_back(std::move(edges));
            }
        }
        if (!polygons_.empty()) {
            grid_.emplace(boxes, 32.0);
        }
    }

    bool empty() const { return polygons_.empty(); }

    // True when `p` is strictly inside one region (more than the margin from its edges); looks at
    // the cached region first and then at no more than kScan of those whose box holds p.
    bool contains(Vec2 p) const { return find(p, [](std::size_t) { return true; }); }

    // True when all of `points` are strictly inside one region, found as for contains().
    bool containsAll(Vec2 anchor, const std::vector<Vec2>& points) const {
        return find(anchor, [&](std::size_t w) {
            return std::all_of(points.begin(), points.end(), [&](Vec2 q) { return inside(w, q); });
        });
    }

   private:
    // Points p with nx * p.x + ny * p.y - offset > 0 are inside this edge's half-plane.
    struct Edge {
        double nx;
        double ny;
        double offset;
    };

    static constexpr std::size_t kScan = 240;

    // Counter-clockwise order; false when not convex.
    static bool orient(std::vector<Vec2>& pts) {
        double area2 = 0.0;
        const std::size_t n = pts.size();
        for (std::size_t i = 0; i < n; ++i) {
            area2 += pts[i].cross(pts[(i + 1) % n]);
        }
        if (!(std::fabs(area2) > 0.0)) {
            return false;
        }
        if (area2 < 0.0) {
            std::reverse(pts.begin(), pts.end());
        }
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = pts[i];
            const Vec2 b = pts[(i + 1) % n];
            const Vec2 c = pts[(i + 2) % n];
            if ((b - a).cross(c - b) < 0.0) {
                return false;
            }
        }
        return true;
    }

    bool inside(std::size_t w, Vec2 p) const {
        for (const Edge& e : polygons_[w]) {
            if (e.nx * p.x + e.ny * p.y - e.offset <= margin_) {
                return false;
            }
        }
        return true;
    }

    template <class Accept>
    bool find(Vec2 anchor, Accept&& accept) const {
        if (cached_ < polygons_.size() && inside(cached_, anchor) && accept(cached_)) {
            return true;
        }
        bool found = false;
        std::size_t scanned = 0;
        if (grid_) {
            grid_->queryUntil(Box2{anchor, anchor}, [&](std::size_t w) {
                if (++scanned > kScan) {
                    return true;
                }
                if (inside(w, anchor) && accept(w)) {
                    cached_ = w;
                    found = true;
                    return true;
                }
                return false;
            });
        }
        return found;
    }

    double margin_;
    std::vector<std::vector<Edge>> polygons_;
    mutable std::optional<BoxGrid> grid_;
    mutable std::size_t cached_{std::numeric_limits<std::size_t>::max()};
};

}  // namespace

static std::vector<Contour> offsetOpenContourImpl(const Contour& contour, double distance, OffsetSide side,
                                                  JoinType join, const Tolerance& tol, double mitreLimit) {
    std::vector<Contour> result;
    const std::vector<Span> input = flattenForOffset(contour, tol);
    if (input.empty() || !std::isfinite(distance)) {
        return result;
    }
    double sign = side == OffsetSide::Left ? 1.0 : -1.0;
    if (distance < 0.0) {
        sign = -sign;
    }
    const double d = std::fabs(distance);
    if (tol.isZero(d)) {
        Contour same;
        same.closed = contour.closed;
        same.spans = input;
        result.push_back(std::move(same));
        return result;
    }

    // 1. Raw offset (see buildRaw); Sharp/Square corners record the region they add.
    std::vector<Wedge> wedges;
    const bool cyclic = contour.closed && input.size() > 1;
    const RawChain rc = buildRaw(input, d, sign, join, mitreLimit, tol, contour.closed, wedges);
    const std::vector<Span>& raw = rc.spans;
    const std::size_t m = raw.size();
    if (m == 0) {
        return result;
    }

    // The margin only absorbs rounding (pieces on the boundary sit at d to ~1e-15 relative). It
    // stays below tol^2 / (8d), the depth of the shallowest connector or overshoot longer than
    // the tolerance at a nearly straight inside corner, so no such sliver survives as a cusp.
    const double keepAbove = d - std::min(1e-9 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d);
    std::vector<Box2> inputBoxes;
    inputBoxes.reserve(input.size());
    for (const Span& s : input) {
        inputBoxes.push_back(bounds(s));
    }
    BoxGrid inputGrid(inputBoxes);

    // Dead elements: distance to a segment is a convex function of the point, so a raw element
    // whose convex hull points (see hullPoints) are all closer than keepAbove to one input line
    // lies wholly that close to it, and every piece of it would be pruned. At a large distance
    // relative to the feature size most of the raw chain is like that; dropping these elements
    // before splitting removes most of the boundary-split and pruning work (T-0006). Splits a
    // live element would get from crossing a dead one lie strictly inside the neighbourhood,
    // between two pieces that are pruned anyway.
    // A scan that has looked at more than kScanCap candidate spans gives up on the line test
    // and asks the input points instead (a piece within `keepAbove` of one input point is in the
    // neighbourhood); a dense path made the scans of the elements that are not dead visit
    // every span in reach, which grew with the square of the number of spans (T-0168).
    constexpr std::size_t kScanCap = 64;
    std::optional<PointTree> pointTree;
    bool pointsFinite = true;
    auto inputPoints = [&]() -> const PointTree* {
        if (!pointTree && pointsFinite) {
            std::vector<Vec2> points;
            points.reserve(input.size() + 1);
            for (const Span& sp : input) {
                points.push_back(startPoint(sp));
            }
            points.push_back(endPoint(input.back()));
            pointsFinite = std::all_of(points.begin(), points.end(),
                                       [](Vec2 p) { return std::isfinite(p.x) && std::isfinite(p.y); });
            if (pointsFinite) {
                pointTree.emplace(std::move(points));
            }
        }
        return pointTree ? &*pointTree : nullptr;
    };
    std::vector<bool> dead(m, false);
    std::vector<Vec2> hull;
    for (std::size_t i = 0; i < m; ++i) {
        pollCancelEvery(i);
        hullPoints(raw[i], hull);
        const Vec2 a = hull.front();
        std::size_t scanned = 0;
        bool gaveUp = false;
        dead[i] = inputGrid.queryUntil(
            Box2{a - Vec2{keepAbove, keepAbove}, a + Vec2{keepAbove, keepAbove}}, [&](std::size_t j) {
                if (++scanned > kScanCap) {
                    gaveUp = true;
                    return true;
                }
                if (!std::holds_alternative<LineSpan>(input[j])) {
                    return false;
                }
                return std::all_of(hull.begin(), hull.end(),
                                   [&](Vec2 p) { return nearerThan(p, input[j], keepAbove); });
            });
        if (gaveUp) {
            dead[i] = false;
            if (const PointTree* tree = inputPoints()) {
                Vec2 centre;
                if (tree->nearest(pointAt(raw[i], 0.5), centre) < keepAbove) {
                    dead[i] = std::all_of(hull.begin(), hull.end(),
                                          [&](Vec2 p) { return geometry::distance(p, centre) < keepAbove; });
                }
            }
        }
    }

    // 2. Split the raw chain at its self-intersections (candidate pairs from a uniform grid).
    std::vector<std::vector<double>> splits(m);
    std::vector<Box2> boxes(m);
    for (std::size_t i = 0; i < m; ++i) {
        if (!dead[i]) {
            boxes[i] = bounds(raw[i]).inflated(1e-9);
        }
    }
    // Near-tolerance walks (T-0168): where the path lies within d of itself over a length much
    // below d, hundreds of long raw elements overlap each other and nearly all of each lies
    // inside the neighbourhood, but not inside that of any one span, so `dead` misses it and the
    // pairwise crossings and boundary splits grow with the square of the number of spans. For
    // elements that overlap many others the ranges wholly inside one input point's disc are found
    // (deadRangesOf); they take no splits and no pieces, and the grid holds only the rest.
    std::vector<ParamRanges> deadRanges(m);
    std::vector<Box2> runBoxes;
    std::vector<std::size_t> runOwner;
    std::vector<std::vector<std::size_t>> runsOf;
    bool refined = false;
    {
        // Elements with many input spans within reach: only they can overlap many others.
        constexpr std::size_t kDenseSpans = 48;
        std::vector<std::size_t> dense;
        for (std::size_t i = 0; i < m; ++i) {
            if (dead[i]) {
                continue;
            }
            std::size_t count = 0;
            if (inputGrid.queryUntil(boxes[i].inflated(keepAbove), [&](std::size_t) { return ++count > kDenseSpans; })) {
                dense.push_back(i);
            }
        }
        if (!dense.empty()) {
            if (const PointTree* tree = inputPoints()) {
                const ConvexRegions regions(wedges, 1e-9 * std::max(1.0, d));
                auto covers = [&](Vec2 mid, const std::vector<Vec2>& hullPts) {
                    Vec2 centre;
                    if (tree->nearest(mid, centre) < keepAbove &&
                        std::all_of(hullPts.begin(), hullPts.end(),
                                    [&](Vec2 p) { return geometry::distance(p, centre) < keepAbove; })) {
                        return true;
                    }
                    return !regions.empty() && regions.containsAll(mid, hullPts);
                };
                auto covered = [&](Vec2 p) {
                    Vec2 centre;
                    return tree->nearest(p, centre) < keepAbove || (!regions.empty() && regions.contains(p));
                };
                std::vector<Vec2> scratch;
                // Dense input where no range is found (the vertices of a fine circle, all about
                // as far from the offset as each other) would only cost time: stop when fewer
                // than one element in 32 has one after the first 128.
                std::size_t tried = 0;
                std::size_t found = 0;
                for (const std::size_t i : dense) {
                    deadRangesOf(raw[i], 0.0, 1.0, 0, tol.distance, covers, covered, deadRanges[i], scratch);
                    ++tried;
                    if (!deadRanges[i].empty()) {
                        ++found;
                        refined = true;
                    }
                    if (tried >= 128 && found * 32 < tried) {
                        break;
                    }
                }
            }
        }
        if (refined) {
            runsOf.resize(m);
            for (std::size_t i = 0; i < m; ++i) {
                if (dead[i]) {
                    continue;
                }
                const double len = length(raw[i]);
                double from = 0.0;
                auto addRun = [&](double t0, double t1) {
                    if ((t1 - t0) * len <= 1e-9) {
                        return;
                    }
                    runsOf[i].push_back(runBoxes.size());
                    runOwner.push_back(i);
                    runBoxes.push_back(deadRanges[i].empty() ? boxes[i]
                                                             : bounds(rangeOf(raw[i], t0, t1)).inflated(1e-9));
                };
                for (const auto& [a, b] : deadRanges[i]) {
                    addRun(from, a);
                    from = b;
                    splits[i].push_back(a);
                    splits[i].push_back(b);
                }
                addRun(from, 1.0);
            }
        }
    }
    BoxGrid rawGrid(refined ? runBoxes : boxes);
    // Calls f(j) for every live element j > i whose (live ranges') box meets element i's.
    std::vector<std::size_t> partners;
    auto forPartners = [&](std::size_t i, auto&& f) {
        if (!refined) {
            rawGrid.query(boxes[i], [&](std::size_t j) {
                if (j > i) {
                    f(j);
                }
            });
            return;
        }
        partners.clear();
        for (const std::size_t r : runsOf[i]) {
            rawGrid.query(runBoxes[r], [&](std::size_t run) {
                if (runOwner[run] > i) {
                    partners.push_back(runOwner[run]);
                }
            });
        }
        std::sort(partners.begin(), partners.end());
        partners.erase(std::unique(partners.begin(), partners.end()), partners.end());
        for (const std::size_t j : partners) {
            f(j);
        }
    };
    std::vector<std::pair<std::size_t, std::size_t>> overlapping;  // element pairs on one carrier
    auto adjacentShared = [&](std::size_t i, std::size_t j, double ti, double tj) {
        if (j == i + 1 && ti >= 1.0 - kParamEps && tj <= kParamEps) {
            return true;
        }
        return cyclic && m > 1 && i == 0 && j + 1 == m && ti <= kParamEps && tj >= 1.0 - kParamEps;
    };
    for (std::size_t i = 0; i < m; ++i) {
        pollCancelEvery(i);
        if (dead[i]) {
            continue;
        }
        forPartners(i, [&](std::size_t j) {
            if (sameCarrier(raw[i], raw[j])) {
                // Overlapping elements (e.g. the mitre extensions of a slot narrower than 2d):
                // split each at the other's end points so the overlap becomes whole pieces. The
                // carriers may still cross (sameCarrier allows ~1e-9 of the length sideways, 1e-7
                // on a 100 mm chamfer); that crossing is split too, below, so that on each side of
                // it the outer copy can be kept (mergeOnCarriers) and nothing dips inside d.
                overlapping.emplace_back(i, j);
                for (const auto& [a, b, splitsOfA] :
                     {std::tuple{i, j, &splits[i]}, std::tuple{j, i, &splits[j]}}) {
                    for (const Vec2 p : {startPoint(raw[b]), endPoint(raw[b])}) {
                        const double t = carrierParam(raw[a], p);
                        if (t > kParamEps && t < 1.0 - kParamEps) {
                            splitsOfA->push_back(t);
                        }
                    }
                }
            }
            for (const Vec2 p : carrierIntersections(raw[i], raw[j])) {
                const double ti = carrierParam(raw[i], p);
                const double tj = carrierParam(raw[j], p);
                if (!inUnit(ti) || !inUnit(tj) || adjacentShared(i, j, ti, tj)) {
                    continue;
                }
                if (refined && (inRanges(deadRanges[i], ti) || inRanges(deadRanges[j], tj))) {
                    continue;
                }
                splits[i].push_back(ti);
                splits[j].push_back(tj);
            }
        });
    }
    // The rest of the d-neighbourhood boundary: circles about every input vertex, the offsets
    // on both sides and, for arcs with r < d, the inverted offsets beyond the centre. A piece
    // can only enter the neighbourhood by crossing that boundary, so after these splits each
    // piece lies wholly inside or outside and an interior sample decides it.
    //    Of a vertex circle only the arc between the two neighbours' offsets on the outer side
    //    of the turn can be on that boundary: every other point of it lies in the interior of the
    //    neighbourhood of one of the two spans meeting there (the d-neighbourhood of a span is
    //    bounded by its two offsets and the halves of its end circles facing away from it). So
    //    a vertex contributes that arc (angle = turn angle, widened by a hair so the split set
    //    is a superset), a reversal a full circle, and the path's two ends their full circles.
    //    So does a "vertex" where the flattened input is not continuous (a sub-tolerance span
    //    was left out): both ends there get full circles. Splitting at whole circles cost one
    //    split per raw element per vertex within d, which dominated dense input at a large d
    //    (T-0006).
    std::vector<Span> boundary;
    for (std::size_t i = 0; i < input.size(); ++i) {
        pollCancelEvery(i);
        const Span& s = input[i];
        const Span& prev = input[i == 0 ? input.size() - 1 : i - 1];
        if (i == 0 && !cyclic) {
            boundary.emplace_back(ArcSpan::fullCircle(startPoint(s), d));
        } else if (geometry::distance(endPoint(prev), startPoint(s)) > 1e-9 * std::max(1.0, d)) {
            boundary.emplace_back(ArcSpan::fullCircle(endPoint(prev), d));
            boundary.emplace_back(ArcSpan::fullCircle(startPoint(s), d));
        } else {
            const Vec2 tin = tangentAt(prev, 1.0);
            const Vec2 tout = tangentAt(s, 0.0);
            const double turn = std::atan2(tin.cross(tout), tin.dot(tout));
            constexpr double kWiden = 1e-6;
            if (std::fabs(turn) >= kPi - 2.0 * kWiden) {
                boundary.emplace_back(ArcSpan::fullCircle(startPoint(s), d));
            } else {
                // Outer side: right of travel for a left turn, left for a right turn; the
                // outward normal turns with the tangent, from n(tin) to n(tout).
                const double outer = turn > 0.0 ? -1.0 : 1.0;
                const double a0 = (tin.perp() * outer).angle();
                const double grow = turn > 0.0 ? kWiden : -kWiden;
                boundary.emplace_back(ArcSpan{startPoint(s), d, a0 - grow, a0 + turn + grow,
                                              turn > 0.0 ? ArcDirection::CCW : ArcDirection::CW});
            }
        }
        for (const double sd : {1.0, -1.0}) {
            if (auto o = offsetSpan(s, d, sd)) {
                boundary.push_back(*o);
            } else if (auto inv = invertedOffset(s, d, sd)) {
                boundary.push_back(*inv);
            }
        }
    }
    if (!cyclic) {
        boundary.emplace_back(ArcSpan::fullCircle(endPoint(input.back()), d));
    }
    // The candidates are listed first and then split, in the same order: with the splitting inside
    // the query callback, the grid's scan of its many rejected candidates is compiled as part of
    // this function and its loop variable can end up kept in memory, which cost ~12% on dense
    // zig-zags at large d (T-0172, after unrelated code was added to the function).
    std::vector<std::size_t> boundaryHits;
    pollCancel();
    for (const Span& b : boundary) {
        const auto* arc = std::get_if<ArcSpan>(&b);
        const bool wholeCarrier = arc && arc->isFullCircle();
        boundaryHits.clear();
        auto collect = [&](const Box2& box) {
            rawGrid.query(box, [&](std::size_t r) { boundaryHits.push_back(refined ? runOwner[r] : r); });
        };
        collect(bounds(b).inflated(1e-9));
        if (refined) {
            // A long boundary arc's box meets every element of a dense cluster; the pieces of the
            // arc itself meet only those it passes.
            if (boundaryHits.size() > 64) {
                boundaryHits.clear();
                const double cell = rawGrid.cellSize();
                const int chunks = cell > 0.0 ? static_cast<int>(std::clamp(std::ceil(length(b) / cell), 2.0, 4096.0)) : 1;
                for (int c = 0; c < chunks; ++c) {
                    collect(bounds(rangeOf(b, static_cast<double>(c) / chunks, static_cast<double>(c + 1) / chunks))
                                .inflated(1e-9));
                }
            }
            std::sort(boundaryHits.begin(), boundaryHits.end());
            boundaryHits.erase(std::unique(boundaryHits.begin(), boundaryHits.end()), boundaryHits.end());
        }
        for (const std::size_t i : boundaryHits) {
            for (const Vec2 p : carrierIntersections(raw[i], b)) {
                const double t = carrierParam(raw[i], p);
                if (inUnit(t) && (wholeCarrier || inUnit(carrierParam(b, p))) &&
                    !(refined && inRanges(deadRanges[i], t))) {
                    splits[i].push_back(t);
                }
            }
        }
    }

    // 3. Keep the pieces that stay at least d from the input and outside every Sharp/Square
    //    corner region.
    std::vector<Box2> wedgeBoxes;
    for (const Wedge& w : wedges) {
        Box2 b;
        for (const Span& s : w) {
            b.expand(bounds(s));
        }
        wedgeBoxes.push_back(b);
    }
    BoxGrid wedgeGrid(wedgeBoxes);
    // Strictly inside a corner region; points on its boundary (the extensions themselves) do
    // not count.
    const double onBoundary = 1e-9 * std::max(1.0, d);
    auto inWedge = [&](Vec2 p) {
        return wedgeGrid.queryUntil(Box2{p, p}, [&](std::size_t w) {
            if (!insideRegion(p, wedges[w])) {
                return false;
            }
            double gap = std::numeric_limits<double>::infinity();
            for (const Span& s : wedges[w]) {
                gap = std::min(gap, distanceToSpan(p, s));
            }
            return gap > onBoundary;
        });
    };
    std::vector<Piece> candidates;
    for (std::size_t i = 0; i < m; ++i) {
        pollCancelEvery(i);
        if (dead[i]) {
            continue;
        }
        std::vector<double>& ts = splits[i];
        ts.push_back(0.0);
        ts.push_back(1.0);
        for (double& t : ts) {
            t = std::clamp(t, 0.0, 1.0);
        }
        std::sort(ts.begin(), ts.end());
        const double len = length(raw[i]);
        for (std::size_t k = 0; k + 1 < ts.size(); ++k) {
            if ((ts[k + 1] - ts[k]) * len > 1e-9 &&
                !(refined && inRanges(deadRanges[i], 0.5 * (ts[k] + ts[k + 1])))) {
                candidates.push_back(Piece{i, ts[k], ts[k + 1]});
            }
        }
    }
    std::size_t witness = input.size();
    auto keep = [&](const Piece& piece) {
        for (const double f : {0.25, 0.5, 0.75}) {
            const Vec2 q = pointAt(raw[piece.element], piece.t0 + (piece.t1 - piece.t0) * f);
            if (closerThan(q, keepAbove, input, inputGrid, witness) || inWedge(q)) {
                return false;
            }
        }
        return true;
    };
    // Copies on one line: only the ones that can matter to mergeOnCarriers are tested.
    std::vector<signed char> state(candidates.size(), -1);
    std::vector<bool> handled(candidates.size(), false);
    pruneLineCopies(raw, rc.connector, candidates, overlapping, d, sign, state, handled, keep);
    std::vector<Piece> kept;
    for (std::size_t k = 0; k < candidates.size(); ++k) {
        pollCancelEvery(k);
        if (!handled[k] && state[k] < 0) {
            state[k] = keep(candidates[k]) ? 1 : 0;
        }
        if (state[k] == 1) {
            kept.push_back(candidates[k]);
        }
    }
    // Two points computed separately at the same place on the offset (a piece's end and its
    // continuation's start) agree to rounding: much closer than this, while distinct split
    // points lie much further apart.
    auto coincide = [&](Vec2 p) { return 1e-9 * std::max({1.0, d, std::fabs(p.x), std::fabs(p.y)}); };
    // Overlapping elements on one carrier (see mergeOnCarriers): one boundary piece per stretch.
    std::vector<Piece> droppedOpposite;  // one copy of each stretch copies cover both ways
    kept = mergeOnCarriers(raw, kept, overlapping, d, sign, 0.5 * tol.distance,
                           std::min(5e-7 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d), rc.exact,
                           droppedOpposite);
    if (kept.empty() && droppedOpposite.empty()) {
        return result;
    }

    // 4. Stitch the kept pieces into components by following their end points. Usually a
    //    piece continues with its successor along the raw chain, but where the offset turns onto
    //    a later or earlier part of it (around a mitre that pokes out over a neighbour, say) it
    //    continues with whichever kept piece starts where it ends. Every end is linked to at most
    //    one start and every start to at most one end, nearest first over all pieces (a start
    //    exactly at an end, to rounding, beats one merely within the tolerance: where split points
    //    lie closer together than the tolerance, as on the overlapping chamfers of a dense
    //    zig-zag, a start a little behind the end would be a backward step, T-0006); among equally
    //    near starts the raw successor wins, then the nearest piece after it in raw order. The
    //    links are chosen for all pieces at once: chosen walk by walk, a stray sub-rounding piece
    //    walked first took over the start that another piece ended exactly at, cutting a chain in
    //    two where nothing crosses (T-0017, 0xe2000124). Components start from pieces nothing
    //    leads into, in raw order; pieces left over form cycles.
    auto pieceStart = [&](const Piece& p) { return pointAt(raw[p.element], p.t0); };
    auto pieceEnd = [&](const Piece& p) { return pointAt(raw[p.element], p.t1); };
    const std::size_t nk = kept.size();
    PointIndex starts(tol.distance);
    for (std::size_t k = 0; k < nk; ++k) {
        starts.add(pieceStart(kept[k]), k);
    }
    struct Link {
        double gap;  // 0 when the start is at the end to rounding
        std::size_t rank;
        std::size_t from;
        std::size_t to;
    };
    std::vector<Link> links;
    for (std::size_t k = 0; k < nk; ++k) {
        const Vec2 end = pieceEnd(kept[k]);
        const double same = coincide(end);
        starts.forEachNear(end, [&](std::size_t j) {
            if (j == k) {
                return;
            }
            const double gap = geometry::distance(pieceStart(kept[j]), end);
            const std::size_t rank = j == (k + 1) % nk ? 0 : (j + nk - k) % nk;
            links.push_back(Link{gap <= same ? 0.0 : gap, rank, k, j});
        });
    }
    std::sort(links.begin(), links.end(), [](const Link& a, const Link& b) {
        return std::tie(a.gap, a.rank, a.from, a.to) < std::tie(b.gap, b.rank, b.from, b.to);
    });
    constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
    std::vector<std::size_t> nextOf(nk, kNone);
    std::vector<std::size_t> prevOf(nk, kNone);
    // Linked runs as a union-find with their lengths: a link from a run's tail to its own head
    // closes it into a loop. A loop no longer than the emitted-loop minimum (4 x tol) is dropped
    // on output anyway, so such a link is skipped: taken first because its gap was small, a loop
    // of two sub-tolerance pieces took the starts that the chain through them needed and split
    // it (T-0017 fix round 2, fam4 0xe40003fb).
    std::vector<std::size_t> runOf(nk);
    std::vector<double> runLength(nk);
    for (std::size_t k = 0; k < nk; ++k) {
        runOf[k] = k;
        runLength[k] = length(raw[kept[k].element]) * (kept[k].t1 - kept[k].t0);
    }
    auto findRun = [&](std::size_t k) {
        while (runOf[k] != k) {
            runOf[k] = runOf[runOf[k]];
            k = runOf[k];
        }
        return k;
    };
    for (const Link& l : links) {
        if (nextOf[l.from] != kNone || prevOf[l.to] != kNone) {
            continue;
        }
        const std::size_t ra = findRun(l.from);
        const std::size_t rb = findRun(l.to);
        if (ra == rb && runLength[ra] <= 4.0 * tol.distance) {
            continue;
        }
        nextOf[l.from] = l.to;
        prevOf[l.to] = l.from;
        if (ra != rb) {
            runOf[rb] = ra;
            runLength[ra] += runLength[rb];
        }
    }
    std::vector<bool> used(nk, false);
    std::vector<std::vector<Piece>> components;
    auto walk = [&](std::size_t k) {
        std::vector<Piece> comp;
        while (k != kNone && !used[k]) {
            used[k] = true;
            comp.push_back(kept[k]);
            k = nextOf[k];
        }
        components.push_back(std::move(comp));
    };
    for (std::size_t k = 0; k < nk; ++k) {
        if (!used[k] && prevOf[k] == kNone) {
            walk(k);
        }
    }
    for (std::size_t k = 0; k < nk; ++k) {
        if (!used[k]) {
            walk(k);
        }
    }

    // 5. Order: a component that closes on itself starts at its piece earliest along the input,
    //    and the components are sorted along the input by where they start.
    auto pathPos = [&](const Piece& p) {
        return rc.posA[p.element] + (rc.posB[p.element] - rc.posA[p.element]) * p.t0;
    };
    for (std::vector<Piece>& comp : components) {
        if (comp.size() > 1 && tol.equalPoints(pieceEnd(comp.back()), pieceStart(comp.front()))) {
            std::size_t best = 0;
            for (std::size_t k = 1; k < comp.size(); ++k) {
                if (pathPos(comp[k]) < pathPos(comp[best])) {
                    best = k;
                }
            }
            std::rotate(comp.begin(), comp.begin() + static_cast<std::ptrdiff_t>(best), comp.end());
        }
    }
    std::stable_sort(components.begin(), components.end(),
                     [&](const std::vector<Piece>& a, const std::vector<Piece>& b) {
                         return pathPos(a.front()) < pathPos(b.front());
                     });

    // Junction snapping (step 6) may move the offset towards the input by a hair more than the
    // pruning margin: up to 5e-7 relative, and below 0.1 tol^2 / d like that margin.
    const double snapAbove =
        d - std::max(d - keepAbove, std::min(5e-7 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d));
    const double snapBelow = 2.0 * d - snapAbove;
    std::size_t snapWitness = input.size();
    std::size_t farWitness = input.size();
    // Kept pieces, for the occupancy test; the pieces of the chain being emitted are excluded.
    std::vector<Span> keptSpans;
    std::vector<Box2> keptBoxes;
    keptSpans.reserve(kept.size());
    keptBoxes.reserve(kept.size());
    for (const Piece& p : kept) {
        keptSpans.push_back(subSpan(raw[p.element], p.t0, p.t1));
        keptBoxes.push_back(bounds(keptSpans.back()));
    }
    BoxGrid keptGrid(keptBoxes);
    std::vector<std::pair<std::size_t, double>> emitting;  // (element, t0) of the chain's pieces
    auto isEmitting = [&](const Piece& p) {
        return std::binary_search(emitting.begin(), emitting.end(), std::make_pair(p.element, p.t0));
    };
    auto occupied = [&](const Span& carrier, Vec2 from, Vec2 to) {
        const double eps = 1e-9 * std::max({1.0, d, std::fabs(from.x), std::fabs(from.y)});
        const auto* arc = std::get_if<ArcSpan>(&carrier);
        auto onCarrier = [&](double t) {
            Vec2 q = lerp(from, to, t);
            if (arc && q != arc->centre) {
                q = arc->centre + (q - arc->centre).normalized() * arc->radius;
            }
            return q;
        };
        Box2 box;
        box.expand(Box2{from, from});
        box.expand(Box2{to, to});
        bool hit = false;
        keptGrid.query(box.inflated(eps + geometry::distance(from, to)), [&](std::size_t k) {
            if (hit || isEmitting(kept[k]) || !sameCarrier(carrier, raw[kept[k].element])) {
                return;
            }
            for (const double f : {0.25, 0.5, 0.75}) {
                if (distanceToSpan(onCarrier(f), keptSpans[k]) <= eps) {
                    hit = true;
                    return;
                }
            }
        });
        return hit;
    };
    const SnapContext snap{
        sign,
        tol,
        [&](Vec2 p) { return !closerThan(p, snapAbove, input, inputGrid, snapWitness) && !inWedge(p); },
        [&](Vec2 p) {
            return join != JoinType::Round || closerThan(p, snapBelow, input, inputGrid, farWitness);
        },
        [&](Vec2 p) {
            // Only distances near d matter to the checks (beyond d + 2 tol a Round point already
            // fails notFar), so the query box stays small: with 2d it cost 1.5-2x on dense input.
            const double reach = d + 2.0 * tol.distance;
            double nearest = kInf;
            inputGrid.query(Box2{p - Vec2{reach, reach}, p + Vec2{reach, reach}},
                            [&](std::size_t i) { nearest = std::min(nearest, distanceToSpan(p, input[i])); });
            return nearest;
        },
        [&](Vec2 p) {
            const double reach = 2.0 * d + 2.0 * tol.distance;
            double best = kInf;
            Vec2 at = p;
            inputGrid.query(Box2{p - Vec2{reach, reach}, p + Vec2{reach, reach}}, [&](std::size_t i) {
                Vec2 q;
                if (const auto* line = std::get_if<LineSpan>(&input[i])) {
                    const Vec2 ab = line->end - line->start;
                    const double len2 = ab.lengthSquared();
                    q = line->start +
                        ab * (len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0);
                } else {
                    const auto& arc = std::get<ArcSpan>(input[i]);
                    if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
                        q = arc.centre + (p - arc.centre).normalized() * arc.radius;
                    } else {
                        q = geometry::distance(p, arc.startPoint()) <= geometry::distance(p, arc.endPoint())
                                ? arc.startPoint()
                                : arc.endPoint();
                    }
                }
                const double dq = geometry::distance(p, q);
                if (dq < best) {
                    best = dq;
                    at = q;
                }
            });
            return at;
        },
        d,
        d - snapAbove,
        join == JoinType::Round,
        occupied};
    // Output keys (where each chain's component starts along the input) and the strays: thin
    // loops reduced to one strand and dropped opposite copies, put back at the end where nothing
    // else covers them (T-0128).
    std::vector<double> resultKeys;
    std::vector<Stray> strays;
    double componentKey = 0.0;
    auto emit = [&](std::vector<Piece>::const_iterator b, std::vector<Piece>::const_iterator e, bool loop) {
        emitting.clear();
        for (auto it = b; it != e; ++it) {
            emitting.emplace_back(it->element, it->t0);
        }
        std::sort(emitting.begin(), emitting.end());
        std::vector<Piece> merged;
        for (auto it = b; it != e; ++it) {
            if (!merged.empty() && merged.back().element == it->element &&
                it->t0 >= merged.back().t1 - kParamEps) {
                // Same element split at a touching point (or around a sub-tolerance sliver the
                // chain stepped over): re-merge.
                merged.back().t1 = it->t1;
            } else {
                merged.push_back(*it);
            }
        }
        // Pieces no longer than the tolerance are dropped, except where dropping one would open
        // a gap wider than the tolerance (a run of tiny pieces round a sub-tolerance bump); the
        // chain thus stays continuous within the tolerance.
        std::vector<Span> spans;
        std::vector<bool> tiny;
        for (const Piece& p : merged) {
            spans.push_back(subSpan(raw[p.element], p.t0, p.t1));
            tiny.push_back(isDegenerate(spans.back(), tol));
        }
        // The dropped pieces are remembered next to the span they preceded, in case the junction
        // they leave cannot be snapped shut (see snapChain).
        const auto lastSolid = std::find(tiny.rbegin(), tiny.rend(), false);
        // A loop of nothing but sub-tolerance pieces (a fine polygon of a small circle, T-0168)
        // has no solid piece to measure against: it is measured from its own start, so the
        // pieces that carry it more than the tolerance away are kept and the others dropped.
        const bool allTiny = loop && lastSolid == tiny.rend();
        const std::size_t solidEnd =
            allTiny ? spans.size() : static_cast<std::size_t>(tiny.rend() - lastSolid);  // one past
        double total = 0.0;
        std::vector<Span> solid;
        std::vector<std::vector<Span>> dropped(1);
        std::vector<Span> leading;
        for (std::size_t k = 0; k < solidEnd; ++k) {
            const Vec2 reference = solid.empty() ? startPoint(spans.front()) : endPoint(solid.back());
            if (tiny[k] && ((solid.empty() && !allTiny) ||
                            geometry::distance(reference, endPoint(spans[k])) <= tol.distance)) {
                (solid.empty() ? leading : dropped.back()).push_back(spans[k]);
                continue;
            }
            total += length(spans[k]);
            solid.push_back(spans[k]);
            dropped.emplace_back();
        }
        if (total > tol.distance * (loop ? 4.0 : 1.0)) {
            // dropped[k] precedes solid[k]; dropped[0] crosses the closing junction (trailing
            // pieces, then leading ones).
            dropped.pop_back();
            dropped[0].assign(spans.begin() + static_cast<std::ptrdiff_t>(solidEnd), spans.end());
            dropped[0].insert(dropped[0].end(), leading.begin(), leading.end());
            bool stillClosed = false;
            std::vector<std::vector<Span>> pieces = snapChain(solid, dropped, loop, snap, stillClosed);
            for (std::vector<Span>& spansOfPiece : pieces) {
                Contour piece;
                piece.closed = stillClosed;
                piece.spans = std::move(spansOfPiece);
                removeBackSteps(piece.spans, piece.closed, snap);
                removeRetraces(piece.spans, piece.closed, snap, d);
                mergeCoCircularArcs(piece.spans, piece.closed);
                removeCuspLoops(piece.spans, piece.closed, tol);
                // An open chain neither starts nor ends with a span no longer than the
                // tolerance (ADR-0012 section 7): where a cut or a node left one there (fam5
                // 0xf50009c9: 7.8e-4 mm), it goes, as the leading and trailing tiny pieces do.
                if (!piece.closed) {
                    while (piece.spans.size() > 1 && isDegenerate(piece.spans.front(), tol)) {
                        piece.spans.erase(piece.spans.begin());
                    }
                    while (piece.spans.size() > 1 && isDegenerate(piece.spans.back(), tol)) {
                        piece.spans.pop_back();
                    }
                    if (piece.spans.size() == 1 && isDegenerate(piece.spans.front(), tol)) {
                        continue;
                    }
                }
                if (!isSliverLoop(piece, tol)) {
                    result.push_back(std::move(piece));
                    resultKeys.push_back(componentKey);
                } else {
                    // Too thin to enclose anything, but a tool centre would still run along it. A
                    // closed piece is the whole loop, so its raw pieces are `spans`.
                    for (std::vector<Span>& part : exactParts(sliverStrand(spans, tol))) {
                        strays.push_back(Stray{componentKey, std::move(part)});
                    }
                }
            }
        }
    };
    // 6. Every crossing or touching point of the raw chain is a split point, so a component
    //    can only meet itself at piece boundaries ("nodes"): walk each one and cut it whenever
    //    it returns to a node it already visited. The part before the node becomes an open
    //    chain, the part from the node back to itself a closed loop, and the walk continues
    //    from the node, so each emitted chain visits every node once and is simple.
    PointIndex index(tol.distance);
    for (const std::vector<Piece>& comp : components) {
        componentKey = pathPos(comp.front());
        std::vector<Vec2> nodes{pieceStart(comp.front())};
        index.clear();
        index.add(nodes.front(), 0);
        auto chainBegin = comp.begin();
        for (auto it = comp.begin(); it != comp.end(); ++it) {
            const Vec2 e = pieceEnd(*it);
            if (tol.equalPoints(e, nodes.back())) {
                // A piece shorter than the tolerance: keep it in the chain (it is dropped as
                // degenerate on output) and treat its end as the same node.
                nodes.push_back(nodes.back());
                continue;
            }
            if (const std::optional<std::size_t> hit = index.findFirst(e)) {
                // Back at an earlier node: split off the tail before it and the loop through it.
                const auto loopBegin = chainBegin + static_cast<std::ptrdiff_t>(*hit);
                if (*hit > 0) {
                    emit(chainBegin, loopBegin, false);
                }
                emit(loopBegin, it + 1, true);
                chainBegin = it + 1;
                nodes.assign(1, e);
                index.clear();
                index.add(e, 0);
                continue;
            }
            nodes.push_back(e);
            index.add(e, nodes.size() - 1);
        }
        if (chainBegin != comp.end()) {
            emit(chainBegin, comp.end(), false);
        }
    }
    // 7. Strays (T-0128): a tool centre would still follow a stretch where opposite copies were
    //    dropped (the middle of a slot one tool width wide, give or take rounding) or a loop too
    //    thin to enclose anything (reduced to one strand above), so they are put back as open
    //    chains wherever no emitted chain passes within half the tolerance. The pieces of dropped
    //    stretches are first joined: those of one element that continue each other, then pieces
    //    whose ends coincide to rounding (exactParts splits where a junction cannot be made exact
    //    or turns back). Each strand is sampled every tol / 4 (stretches the Lipschitz bound
    //    vouches for skipped); each run of uncovered samples, widened by one sample either way so
    //    it meets the chains that cover its neighbours, is emitted when longer than the tolerance,
    //    or than kStrayMinimum x the tolerance if no chain covers any of it. Strands put back
    //    count as cover for later ones. Every stray lies at d from the input or on a corner region
    //    by construction: a loop's strand is cut from the raw pieces it was stitched from (offset
    //    spans, Round arcs, inverted arcs, each exactly d from its own input element, and Sharp/Square
    //    corner elements), and the opposite copies are cut to RawChain::exact first.
    if (!droppedOpposite.empty()) {
        // Only the parts exactly d from the input are a path that touches it: opposite copies of
        // mitre extensions or chamfers over a slot narrower than 2d lie farther out, inside the
        // corner regions of the offset shape (fam5 0x285001c5: 8.4e-6 mm beyond d).
        std::vector<Piece> runs;
        for (const Piece& p : droppedOpposite) {
            const auto [lo, hi] = rc.exact[p.element];
            if (std::min(p.t1, hi) > std::max(p.t0, lo)) {
                runs.push_back(Piece{p.element, std::max(p.t0, lo), std::min(p.t1, hi)});
            }
        }
        std::sort(runs.begin(), runs.end(), [](const Piece& a, const Piece& b) {
            return a.element != b.element ? a.element < b.element : a.t0 < b.t0;
        });
        std::vector<Piece> joined;
        for (const Piece& r : runs) {
            if (!joined.empty() && joined.back().element == r.element &&
                r.t0 <= joined.back().t1 + kParamEps) {
                joined.back().t1 = std::max(joined.back().t1, r.t1);
            } else {
                joined.push_back(r);
            }
        }
        const std::size_t nr = joined.size();
        PointIndex heads(tol.distance);
        for (std::size_t k = 0; k < nr; ++k) {
            heads.add(pieceStart(joined[k]), k);
        }
        std::vector<std::size_t> succ(nr, kNone);
        std::vector<std::size_t> pred(nr, kNone);
        for (std::size_t k = 0; k < nr; ++k) {
            const Vec2 end = pieceEnd(joined[k]);
            double bestGap = kRoundingUnits * kEps * std::max({1.0, std::fabs(end.x), std::fabs(end.y)});
            std::size_t best = kNone;
            heads.forEachNear(end, [&](std::size_t j) {
                const double gap = geometry::distance(pieceStart(joined[j]), end);
                if (j != k && pred[j] == kNone && (gap < bestGap || (gap == bestGap && j < best))) {
                    bestGap = gap;
                    best = j;
                }
            });
            if (best != kNone) {
                succ[k] = best;
                pred[best] = k;
            }
        }
        std::vector<bool> done(nr, false);
        auto walkRuns = [&](std::size_t k) {
            Stray st{pathPos(joined[k]), {}};
            while (k != kNone && !done[k]) {
                done[k] = true;
                st.spans.push_back(subSpan(raw[joined[k].element], joined[k].t0, joined[k].t1));
                k = succ[k];
            }
            for (std::vector<Span>& part : exactParts(st.spans)) {
                strays.push_back(Stray{st.key, std::move(part)});
            }
        };
        for (std::size_t k = 0; k < nr; ++k) {
            if (!done[k] && pred[k] == kNone) {
                walkRuns(k);
            }
        }
        for (std::size_t k = 0; k < nr; ++k) {
            if (!done[k]) {
                walkRuns(k);
            }
        }
    }
    if (!strays.empty()) {
        putBackStrays(result, resultKeys, strays, tol);
    }
    return result;
}

std::vector<Contour> offsetOpenContour(const Contour& contour, double distance, OffsetSide side,
                                       JoinType join, const Tolerance& tol, double mitreLimit) {
    try {
        return offsetOpenContourImpl(contour, distance, side, join, tol, mitreLimit);
    } catch (const detail::GeometryCancelled&) {
        return {};  // T-0228: cancelled; the caller discards it
    }
}

namespace detail {

bool turnArcOntoJunction(const Span& a, ArcSpan& b) {
    return exactJunctionByAngle(a, b);
}

void closeJunction(Span& a, Span& b) {
    exactJunction(a, b);
}

std::vector<std::size_t> selfCrossingCuts(const std::vector<Span>& spans) {
    return geometry::selfCrossingCuts(spans);
}

bool spansMeet(const Span& a, const Span& b) {
    return geometry::spansMeet(a, b);
}

double meetReach(const Span& s) {
    return geometry::meetReach(s, bounds(s));
}

std::vector<std::vector<Span>> nonCrossingRuns(const std::vector<Span>& strand, const Tolerance& tol) {
    return geometry::nonCrossingRuns(strand, tol);
}

}  // namespace detail

}  // namespace carvetto::geometry
