#include "carvetto/geometry/offset.hpp"

#include <algorithm>
#include <array>
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

#include "clipper_interop.hpp"
#include "offset_batch.hpp"
#include "offset_dust.hpp"
#include "offset_junction.hpp"

namespace carvetto::geometry {

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

std::vector<Contour> offsetContoursBatched(const std::vector<Contour>& contours, double distance, JoinType join,
                                           const Tolerance& tol, double mitreLimit, std::size_t minBatchDepth) {
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
    Clipper2Lib::Paths64 out;
    std::optional<std::vector<std::vector<std::size_t>>> batches;
    if (openPaths.empty()) {
        batches = planBatches(closedPaths, delta, join, mitreLimit, minBatchDepth);
    }
    if (batches) {
        // Batches in the order of their first input contour, each in Clipper2's own order.
        Clipper2Lib::Paths64 part;
        for (const auto& batch : *batches) {
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
    dropDustRings(out);
    return detail::contoursFromClipper(out, true);
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

// Flattened input span list: lines and arcs only, Beziers polygonised. Every span of positive
// length stays, however short (T-0009): leaving out spans no longer than the tolerance broke the
// distance and coverage guarantees next to them (up to ~tol short of d, ~1 mm of offset missing).
// Only spans of zero length go: they have no direction, and the corner is taken between their
// neighbours.
std::vector<Span> flattenForOffset(const Contour& c, const Tolerance& tol) {
    std::vector<Span> out;
    auto usable = [](const Span& s) {
        const Vec2 t0 = tangentAt(s, 0.0);
        const Vec2 t1 = tangentAt(s, 1.0);
        return length(s) > 0.0 && std::isfinite(t0.x) && std::isfinite(t0.y) && std::isfinite(t1.x) &&
               std::isfinite(t1.y) && t0.lengthSquared() > 0.5 && t1.lengthSquared() > 0.5;
    };
    for (const Span& s : c.spans) {
        if (const auto* bez = std::get_if<BezierSpan>(&s)) {
            std::vector<Vec2> pts{bez->p0};
            polygoniseInto(s, tol, pts);
            for (std::size_t i = 1; i < pts.size(); ++i) {
                if (pts[i - 1] != pts[i]) {
                    out.emplace_back(LineSpan{pts[i - 1], pts[i]});
                }
            }
        } else if (usable(s)) {
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
        if (std::fabs(den) <= 1e-12 * u.length() * v.length()) {
            return out;  // parallel
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
        const double len = u.length();
        const double scale = 1e-9 * std::max(1.0, len);
        return std::fabs(u.cross(lb->start - la->start)) <= scale * len &&
               std::fabs(u.cross(lb->end - la->start)) <= scale * len;
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
    explicit BoxGrid(const std::vector<Box2>& boxes) : boxes_(boxes), stamp_(boxes.size(), 0) {
        for (const Box2& b : boxes_) {
            extent_.expand(b);
        }
        if (boxes_.empty() || extent_.isEmpty()) {
            return;
        }
        const double w = extent_.width();
        const double h = extent_.height();
        const double n = static_cast<double>(boxes_.size());
        double cell = std::sqrt(std::max(w * h, 1e-300) / n);
        cell = std::max({cell, w / kMaxCells, h / kMaxCells, 1e-9});
        nx_ = static_cast<std::size_t>(std::clamp(std::ceil(w / cell), 1.0, kMaxCells));
        ny_ = static_cast<std::size_t>(std::clamp(std::ceil(h / cell), 1.0, kMaxCells));
        cw_ = w > 0.0 ? w / static_cast<double>(nx_) : 1.0;
        ch_ = h > 0.0 ? h / static_cast<double>(ny_) : 1.0;
        cells_.resize(nx_ * ny_);
        for (std::size_t i = 0; i < boxes_.size(); ++i) {
            if (boxes_[i].isEmpty()) {
                continue;  // never reported
            }
            const auto [x0, x1, y0, y1] = cellRange(boxes_[i]);
            for (std::size_t y = y0; y <= y1; ++y) {
                for (std::size_t x = x0; x <= x1; ++x) {
                    cells_[y * nx_ + x].push_back(i);
                }
            }
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
        if (cells_.empty() || !query.intersects(extent_)) {
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

   private:
    static constexpr double kMaxCells = 1024.0;

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
    Box2 extent_;
    std::size_t nx_{0};
    std::size_t ny_{0};
    double cw_{1.0};
    double ch_{1.0};
    std::vector<std::vector<std::size_t>> cells_;
    std::vector<std::uint64_t> stamp_;
    std::uint64_t epoch_{0};
};

// True when some input span is closer than `limit` to `p`. `witness` is the span that last
// answered yes; it is tried first and updated, because neighbouring samples along the raw chain
// are almost always rejected by the same span (most raw pieces of a dense path at a large
// distance lie deep inside the neighbourhood). Without it each rejected sample visited every
// span within `limit`, which dominated the cost at large d (T-0006). Only spans whose boxes meet
// the query square can be closer than `limit`, so the answer is exactly that of a full scan.
bool closerThan(Vec2 p, double limit, const std::vector<Span>& input, BoxGrid& grid, std::size_t& witness) {
    if (witness < input.size() && distanceToSpan(p, input[witness]) < limit) {
        return true;
    }
    return grid.queryUntil(Box2{p - Vec2{limit, limit}, p + Vec2{limit, limit}}, [&](std::size_t i) {
        if (i != witness && distanceToSpan(p, input[i]) < limit) {
            witness = i;
            return true;
        }
        return false;
    });
}

// Convex hull of `pts` (Andrew's monotone chain), counter-clockwise, collinear points dropped.
std::vector<Vec2> convexHull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x != b.x ? a.x < b.x : a.y < b.y; });
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    if (pts.size() < 3) {
        return pts;
    }
    std::vector<Vec2> h(2 * pts.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && (h[k - 1] - h[k - 2]).cross(pts[i] - h[k - 2]) <= 0.0) {
            --k;
        }
        h[k++] = pts[i];
    }
    for (std::size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && (h[k - 1] - h[k - 2]).cross(pts[i - 1] - h[k - 2]) <= 0.0) {
            --k;
        }
        h[k++] = pts[i - 1];
    }
    h.resize(k - 1);
    return h;
}

// Parameter intervals (open, within [0, 1]) of the line or arc `s` whose points lie closer than
// `limit` to the point `w`, slightly shrunk so that every point reported is strictly closer.
void closeIntervals(const Span& s, Vec2 w, double limit, std::vector<std::pair<double, double>>& out) {
    constexpr double kShrink = 1e-12;
    if (const auto* line = std::get_if<LineSpan>(&s)) {
        const Vec2 u = line->end - line->start;
        const Vec2 f = line->start - w;
        const double qa = u.lengthSquared();
        if (qa <= 0.0) {
            return;
        }
        const double qb = 2.0 * u.dot(f);
        const double qc = f.lengthSquared() - limit * limit;
        const double disc = qb * qb - 4.0 * qa * qc;
        if (disc <= 0.0) {
            return;
        }
        const double root = std::sqrt(disc);
        const double t0 = std::max(0.0, (-qb - root) / (2.0 * qa) + kShrink);
        const double t1 = std::min(1.0, (-qb + root) / (2.0 * qa) - kShrink);
        if (t1 > t0) {
            out.emplace_back(t0, t1);
        }
        return;
    }
    const auto& arc = std::get<ArcSpan>(s);
    const double sweep = std::fabs(arc.sweepAngle());
    const Vec2 cw = w - arc.centre;
    const double m = cw.length();
    // |C + R u - w|^2 < L^2  <=>  (w - C).u > (R^2 + m^2 - L^2) / 2R.
    const double k = (arc.radius * arc.radius + m * m - limit * limit) / (2.0 * arc.radius);
    if (m <= 0.0 || k <= -m) {
        if (k < -m || (m <= 0.0 && k < 0.0)) {
            out.emplace_back(0.0, 1.0);
        }
        return;
    }
    if (k >= m) {
        return;
    }
    const double half = std::acos(k / m) - kShrink;
    if (half <= 0.0) {
        return;
    }
    // Relative angle (along the arc's direction from its start) of the interval's first end.
    const double phi = cw.angle();
    const double first = arc.direction == ArcDirection::CCW ? normalizeAngle(phi - half - arc.startAngle)
                                                            : normalizeAngle(arc.startAngle - (phi + half));
    for (const double shift : {0.0, -kTwoPi}) {
        const double a0 = std::max(0.0, first + shift);
        const double a1 = std::min(sweep, first + shift + 2.0 * half);
        if (a1 > a0) {
            out.emplace_back(a0 / sweep, a1 / sweep);
        }
    }
}

// The parts of `s` not covered by `dominated` (merged), as intervals of [0, 1].
std::vector<std::pair<double, double>> uncovered(std::vector<std::pair<double, double>>& dominated) {
    std::sort(dominated.begin(), dominated.end());
    std::vector<std::pair<double, double>> live;
    double at = 0.0;
    for (const auto& [a, b] : dominated) {
        if (a > at) {
            live.emplace_back(at, a);
        }
        at = std::max(at, b);
    }
    if (at < 1.0) {
        live.emplace_back(at, 1.0);
    }
    return live;
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

// Bounding volume hierarchy over the input spans in path order, for distance queries next to
// runs of sub-tolerance spans (T-0009). There thousands of spans lie within a few micrometres of
// each other and of every query point's distance d, so a grid query visits all of them: a query
// that has to show that no span is closer than d (a kept piece) cost O(n). Each node bounds its
// consecutive spans by their convex hull (or, for a hull of more than kMaxHull points, their
// box); a query skips a node whose bound is no nearer than its limit, so near a cluster it
// stops at the cluster's hull, at distance d from the points on its outer boundary.
class SpanTree {
   public:
    explicit SpanTree(const std::vector<Span>& input) : input_(input) {
        if (!input.empty()) {
            build(0, input.size());
        }
    }

    // True when some span is closer than `limit` to `p` (`witness`: last span that was).
    bool closer(Vec2 p, double limit, std::size_t& witness) const {
        if (witness < input_.size() && distanceToSpan(p, input_[witness]) < limit) {
            return true;
        }
        bool found = false;
        visit(p, [&] { return limit; }, [&](std::size_t i) {
            if (distanceToSpan(p, input_[i]) < limit) {
                witness = i;
                found = true;
            }
            return found;
        });
        return found;
    }

    // Distance from `p` to the nearest span if below `cap` (otherwise `cap`), and that span's
    // closest point in `at` (unchanged when none is nearer than `cap`).
    double nearest(Vec2 p, double cap, Vec2* at = nullptr) const {
        double best = cap;
        visit(p, [&] { return best; }, [&](std::size_t i) {
            const double dist = distanceToSpan(p, input_[i]);
            if (dist < best) {
                best = dist;
                if (at) {
                    *at = closestOn(input_[i], p);
                }
            }
            return false;
        });
        return best;
    }

   private:
    static constexpr std::size_t kNoChild = std::numeric_limits<std::size_t>::max();
    static constexpr std::size_t kLeaf = 8;
    static constexpr std::size_t kMaxHull = 48;

    // Calls leaf(i) for the spans of every node whose bound is below bound(), nearer nodes first,
    // until leaf returns true.
    template <class Bound, class Leaf>
    void visit(Vec2 p, Bound bound, Leaf leaf) const {
        stack_.clear();
        stack_.emplace_back(0, lowerBound(nodes_[0], p));
        while (!stack_.empty()) {
            const auto [id, lb] = stack_.back();
            stack_.pop_back();
            if (lb >= bound()) {
                continue;
            }
            const Node& n = nodes_[id];
            if (n.left == kNoChild) {
                for (std::size_t i = n.lo; i < n.hi; ++i) {
                    if (leaf(i)) {
                        return;
                    }
                }
                continue;
            }
            const double bl = lowerBound(nodes_[n.left], p);
            const double br = lowerBound(nodes_[n.right], p);
            if (bl <= br) {
                stack_.emplace_back(n.right, br);
                stack_.emplace_back(n.left, bl);
            } else {
                stack_.emplace_back(n.left, bl);
                stack_.emplace_back(n.right, br);
            }
        }
    }

    struct Node {
        std::size_t lo{0};
        std::size_t hi{0};
        std::size_t left{kNoChild};
        std::size_t right{kNoChild};
        Box2 box;
        std::vector<Vec2> hull;  // empty: use the box
    };

    static Vec2 closestOn(const Span& s, Vec2 p) {
        if (const auto* line = std::get_if<LineSpan>(&s)) {
            const Vec2 ab = line->end - line->start;
            const double len2 = ab.lengthSquared();
            return line->start + ab * (len2 > 0.0 ? std::clamp((p - line->start).dot(ab) / len2, 0.0, 1.0) : 0.0);
        }
        const auto& arc = std::get<ArcSpan>(s);
        if (arc.containsAngle((p - arc.centre).angle(), 0.0)) {
            return arc.centre + (p - arc.centre).normalized() * arc.radius;
        }
        return geometry::distance(p, arc.startPoint()) <= geometry::distance(p, arc.endPoint()) ? arc.startPoint()
                                                                                                 : arc.endPoint();
    }

    std::size_t build(std::size_t lo, std::size_t hi) {
        const std::size_t id = nodes_.size();
        Node node;
        node.lo = lo;
        node.hi = hi;
        nodes_.push_back(std::move(node));
        std::vector<Vec2> pts;
        if (hi - lo <= kLeaf) {
            std::vector<Vec2> sp;
            for (std::size_t i = lo; i < hi; ++i) {
                nodes_[id].box.expand(bounds(input_[i]));
                hullPointsOf(input_[i], sp);
                pts.insert(pts.end(), sp.begin(), sp.end());
            }
        } else {
            const std::size_t mid = lo + (hi - lo) / 2;
            const std::size_t l = build(lo, mid);
            const std::size_t r = build(mid, hi);
            nodes_[id].left = l;
            nodes_[id].right = r;
            nodes_[id].box = nodes_[l].box;
            nodes_[id].box.expand(nodes_[r].box);
            if (nodes_[l].hull.empty() || nodes_[r].hull.empty()) {
                return id;
            }
            pts = nodes_[l].hull;
            pts.insert(pts.end(), nodes_[r].hull.begin(), nodes_[r].hull.end());
        }
        std::vector<Vec2> h = convexHull(std::move(pts));
        if (h.size() <= kMaxHull && h.size() >= 3 && clearlyConvex(h)) {
            nodes_[id].hull = std::move(h);
        }
        return id;
    }

    // Rounding in the bounds and in the exact distances they stand in for, at the coordinates' scale.
    static double slack(Vec2 p) { return 1e-12 * std::max({1.0, std::fabs(p.x), std::fabs(p.y)}); }

    // True when every turn of the hull is clearly to the left: points nearly on top of each other
    // (the hull points of an arc of rounding-size radius) can leave a polygon that is not convex,
    // 13 points visiting one point twice, whose edges then cut through it (T-0009); such a node
    // uses its box.
    static bool clearlyConvex(const std::vector<Vec2>& h) {
        const std::size_t k = h.size();
        for (std::size_t i = 0; i < k; ++i) {
            const Vec2 e1 = h[(i + 1) % k] - h[i];
            const Vec2 e2 = h[(i + 2) % k] - h[(i + 1) % k];
            if (!(e1.cross(e2) > 1e-9 * e1.length() * e2.length())) {
                return false;
            }
        }
        return true;
    }

    // Distance from `p` to the node's hull (0 inside), or to its box.
    static double lowerBound(const Node& n, Vec2 p) {
        if (n.hull.empty()) {
            const double dx = std::max({n.box.min.x - p.x, 0.0, p.x - n.box.max.x});
            const double dy = std::max({n.box.min.y - p.y, 0.0, p.y - n.box.max.y});
            return std::max(0.0, std::sqrt(dx * dx + dy * dy) - slack(p));
        }
        bool inside = true;
        double best = kInf;
        const std::size_t k = n.hull.size();
        for (std::size_t i = 0; i < k; ++i) {
            const Vec2 a = n.hull[i];
            const Vec2 b = n.hull[(i + 1) % k];
            const Vec2 ab = b - a;
            if (ab.cross(p - a) < 0.0) {
                inside = false;
            }
            const double len2 = ab.lengthSquared();
            const double t = len2 > 0.0 ? std::clamp((p - a).dot(ab) / len2, 0.0, 1.0) : 0.0;
            best = std::min(best, geometry::distance(p, a + ab * t));
        }
        return inside ? 0.0 : std::max(0.0, best - slack(p));
    }

    static void hullPointsOf(const Span& s, std::vector<Vec2>& out) { hullPoints(s, out); }

    const std::vector<Span>& input_;
    std::vector<Node> nodes_;
    mutable std::vector<std::pair<std::size_t, double>> stack_;
};

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
        // An arc reaches a meeting point just behind its own start only the long way round (the
        // whole gap outside its sweep counts as extension), and the checks below look at the short
        // way: an arc 3e-8 rad from the meeting point was extended into a full circle 0.59 mm deep
        // inside d (T-0009, nearly coincident Round joins about the ends of a 2e-10 mm jog).
        auto shortWay = [&](const Span& span, double t, bool pastEnd, Vec2 end) {
            const double extension = (pastEnd ? t - 1.0 : -t) * length(span);
            return !std::holds_alternative<ArcSpan>(span) ||
                   extension <= 2.0 * geometry::distance(p, end) + rounding;
        };
        if ((*ta > 1.0 && !shortWay(a, *ta, true, e)) || (*tb < 0.0 && !shortWay(b, *tb, false, s))) {
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
    const bool moveB = lineB && lineOk(endPoint(b), e);
    const bool moveA = lineA && lineOk(startPoint(a), s);
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
// much larger than the point's, leaving a rounding-level residual (within 1e-12 x the scale)
// that never puts b's start behind a's end along a's direction of travel.
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
    // a's end along a's direction of travel; otherwise take the nearest of the searched
    // candidates that does, within the documented 1e-12 x the scale.
    const Vec2 ta = tangentAt(a, 1.0);
    auto ahead = [&](const ArcSpan& c) { return (c.startPoint() - j).dot(ta) >= 0.0; };
    if (ahead(arc)) {
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
            if (tangentAt(a, 1.0).dot(tangentAt(b, 0.0)) > -0.99) {
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

// The raw offset chain of one side, with each element's position along the input path (span
// index + fraction at its start and end) for ordering the output chains.
struct RawChain {
    std::vector<Span> spans;
    std::vector<double> posA;
    std::vector<double> posB;
};

// Raw offset of one side, in input order: every element offset on its own, chained with joins
// (outside corners) and straight connectors (inside corners). An arc whose offset radius would
// be negative is inverted (radius d - r on the far side of its centre); it is pruned later but
// keeps the corner geometry, so the joins about its end points survive.
RawChain buildRaw(const std::vector<Span>& input, double d, double sign, JoinType join, double mitreLimit,
                  const Tolerance& tol, bool closed, std::vector<Wedge>& wedges) {
    RawChain rc;
    std::vector<Span>& raw = rc.spans;
    auto record = [&](double a, double b) {
        while (rc.posA.size() < raw.size()) {
            rc.posA.push_back(a);
            rc.posB.push_back(b);
        }
    };
    // A Round offset of an arc whose radius is within the snapping allowance of zero (a corner
    // rounded off at rounding level, T-0009) is that of a vertex at which the direction turns
    // on the offset side: its inverted offset (radius d - r about the centre) runs straight back
    // along the Round join about its start, within r of it, and pruning and the merging of
    // opposite copies would drop both. Such an arc gets the straight connector of an inside
    // corner instead, between its end normals, so the joins about its two end points are those
    // of the arc's own end tangents.
    const double tinyArc = std::min(5e-7 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d);
    // A corner between two sub-tolerance spans is Round whatever the join (T-0009). Inside a run of
    // such spans (noise along a drawn line, a densely sampled curve, a micro-cluster at a vertex)
    // each turn would otherwise add a mitre or chamfer reaching out up to mitreLimit x d in the
    // direction of a sub-tolerance step, a spike far larger than the step itself, and thousands of
    // overlapping spikes made the cost grow with the cube of their number. The corners of such a
    // run with its longer neighbours keep the requested join.
    auto cornerJoin = [&](const Span& a, const Span& b) {
        return join != JoinType::Round && length(a) <= tol.distance && length(b) <= tol.distance
                   ? JoinType::Round
                   : join;
    };
    // An arc whose offset towards its centre collapses to (nearly) its centre point (radius d to
    // rounding, a needle tip) adds nothing of its own, but the joins about its end points still
    // need a raw element to run to: without one the next corner joined the previous span's offset
    // straight to the following one about the far end, an arc of radius 0.614 for d = 0.603
    // (T-0009, verifier 0xd93009e2). It gets the connector between its end normals too, with Round
    // corners on both sides (the centre point is a round tip).
    bool roundAfter = false;
    // Sub-tolerance near-reversals (owner decision (c), T-0009 round 3): a corner that turns by more
    // than 135 degrees at a span no longer than the tolerance gets no join on either side, only the
    // straight connector of an inside corner. Its outside join would sweep round the turn to the
    // other side of the path: a near-reversal inside a micro-cluster made the Round offset of a
    // closed loop wrap round and come back open (the missing stretch emitted on the other side),
    // and a Sharp or Square corner at a 171-degree turn onto a 3.6e-15 mm end span reached out
    // 1.5 d along that span and swallowed 1.6 mm of valid offset. The offset stays on the
    // requested side; the corner region of such a sub-tolerance turn is not part of it.
    auto nearReversal = [&](const Span& a, const Span& b) {
        if (length(a) > tol.distance && length(b) > tol.distance) {
            return false;
        }
        const Vec2 ta = tangentAt(a, 1.0);
        const Vec2 tb = tangentAt(b, 0.0);
        return std::fabs(std::atan2(ta.cross(tb), ta.dot(tb))) > 0.75 * kPi;
    };
    // The run of sub-tolerance spans round such a turn: consecutive spans no longer than the
    // tolerance, all within a box of diagonal 2 tol. Where the run as a whole reverses (a turn of more
    // than 135 degrees from the longer span before it to the one after it) it is joined as one corner
    // between those two spans, with its ends as a gap, as on main. Otherwise its extreme points still
    // bound the neighbourhood where they stick out past its neighbours' offsets on the requested
    // side: Round arcs of radius d about the run's convex hull points, over its net turn and a quarter
    // turn beyond on either side, keep that part (pruning cuts the rest). At an open path's end
    // neither: the offset ends with the longer neighbour's.
    const std::size_t n = input.size();
    auto isShort = [&](std::size_t k) { return length(input[k]) <= tol.distance; };
    struct Run {
        std::size_t first, last;
        bool hasBefore, hasAfter;
    };
    // Runs found once, greedily along the path from a longer span (so a closed contour's run is
    // not cut at the seam): each sub-tolerance span belongs to one run.
    std::vector<std::size_t> runOf(n, n);
    std::vector<Run> runs;
    {
        // Open: in path order. Closed: from a longer span, round the seam.
        std::size_t start = 0;
        if (closed) {
            for (std::size_t k = 0; k < n; ++k) {
                if (!isShort(k)) {
                    start = k;
                    break;
                }
            }
        }
        for (std::size_t step = 0; step < n;) {
            const std::size_t k = (start + step) % n;
            if (!isShort(k)) {
                ++step;
                continue;
            }
            Box2 box = bounds(input[k]);
            std::size_t len = 1;
            while (step + len < n) {
                const std::size_t j = (start + step + len) % n;
                if (!isShort(j)) {
                    break;
                }
                Box2 g = box;
                g.expand(bounds(input[j]));
                if (geometry::distance(g.min, g.max) > 2.0 * tol.distance) {
                    break;
                }
                box = g;
                ++len;
            }
            const std::size_t last = (start + step + len - 1) % n;
            runs.push_back(Run{k, last, closed || k > 0, closed || last + 1 < n});
            for (std::size_t q = 0; q < len; ++q) {
                runOf[(k + q) % n] = runs.size() - 1;
            }
            step += len;
        }
    }
    auto runAround = [&](std::size_t at) { return runs[runOf[at]]; };
    std::vector<char> runDone(n, 0);
    std::vector<char> skip(n, 0);
    std::vector<std::size_t> collapsedBefore(n, n);  // by the span after a collapsed run
    for (std::size_t k = closed ? 0 : 1; k < n; ++k) {
        const std::size_t a = (k + n - 1) % n;
        if (!nearReversal(input[a], input[k])) {
            continue;
        }
        const Run r = runAround(isShort(a) ? a : k);
        if (!r.hasBefore || !r.hasAfter) {
            continue;
        }
        const std::size_t before = (r.first + n - 1) % n;
        const std::size_t after = (r.last + 1) % n;
        if (isShort(before) || isShort(after) || before == after || collapsedBefore[after] != n) {
            continue;
        }
        const Vec2 ta = tangentAt(input[before], 1.0);
        const Vec2 tb = tangentAt(input[after], 0.0);
        if (std::fabs(std::atan2(ta.cross(tb), ta.dot(tb))) > 0.75 * kPi) {
            for (std::size_t q = r.first;; q = (q + 1) % n) {
                skip[q] = 1;
                if (q == r.last) {
                    break;
                }
            }
            collapsedBefore[after] = before;
        }
    }
    auto runArcs = [&](std::size_t at) {
        if (!isShort(at)) {
            return;
        }
        const Run r = runAround(at);
        if (runDone[r.first] || !r.hasBefore || !r.hasAfter) {
            return;
        }
        runDone[r.first] = 1;
        const Vec2 ta = tangentAt(input[(r.first + n - 1) % n], 1.0);
        const Vec2 tb = tangentAt(input[(r.last + 1) % n], 0.0);
        std::vector<Vec2> pts;
        for (std::size_t k = r.first;; k = (k + 1) % n) {
            pts.push_back(startPoint(input[k]));
            pts.push_back(pointAt(input[k], 0.5));
            pts.push_back(endPoint(input[k]));
            if (k == r.last) {
                break;
            }
        }
        // Always in the direction of an outside join (so that an arc on the circle of a join about
        // the same point is a copy running the same way, merged, not one running against it, which
        // would cancel both), from a quarter turn before the first normal of the turn to a quarter
        // turn past the last.
        const double turn = std::atan2(ta.cross(tb), ta.dot(tb));
        const ArcDirection dir = sign > 0.0 ? ArcDirection::CW : ArcDirection::CCW;
        const double ccw = dir == ArcDirection::CCW ? 1.0 : -1.0;
        const CornerKind kind = classifyCorner(ta, tb, sign);
        const double sweep = kind == CornerKind::Continuous ? 0.0 : std::fabs(turn);
        const double width = std::min(sweep + kPi, kTwoPi - 0.1);
        // The normals turn in the outside direction at an outside corner, against it at an inside one.
        const Vec2 firstNormal = (kind == CornerKind::Inside ? tb : ta).perp() * sign;
        const double from = firstNormal.angle() - ccw * 0.5 * (width - sweep);
        for (const Vec2 h : convexHull(pts)) {
            raw.emplace_back(ArcSpan{h, d, from, from + ccw * width, dir});
        }
    };
    std::size_t lastEmitted = n;
    std::size_t firstEmitted = n;
    auto connectOrBridge = [&](Span& next, std::size_t a, std::size_t b, JoinType j) {
        if (nearReversal(input[a], input[b])) {
            const Vec2 e = endPoint(raw.back());
            if (!tol.equalPoints(e, startPoint(next))) {
                raw.emplace_back(LineSpan{e, startPoint(next)});
            }
            runArcs(length(input[a]) <= tol.distance ? a : b);
            return;
        }
        connect(raw, next, startPoint(input[b]), endPoint(input[a]), tangentAt(input[a], 1.0),
                tangentAt(input[b], 0.0), d, sign, j, mitreLimit, tol, wedges);
    };
    for (std::size_t i = 0; i < input.size(); ++i) {
        if (skip[i]) {
            continue;
        }
        std::optional<Span> next = offsetSpan(input[i], d, sign);
        bool collapsed = false;
        if (!next) {
            const auto* arc = std::get_if<ArcSpan>(&input[i]);
            if (arc && join == JoinType::Round && arc->radius <= tinyArc) {
                collapsed = true;
            } else {
                next = invertedOffset(input[i], d, sign);
                collapsed = arc && !next;
            }
            if (collapsed) {
                const Vec2 a = startPoint(input[i]) + tangentAt(input[i], 0.0).perp() * (d * sign);
                const Vec2 b = endPoint(input[i]) + tangentAt(input[i], 1.0).perp() * (d * sign);
                if (a == b) {
                    continue;
                }
                next = LineSpan{a, b};
            }
            if (!next) {
                continue;
            }
        }
        const double at = static_cast<double>(i);
        if (!raw.empty()) {
            if (collapsedBefore[i] != n && collapsedBefore[i] == lastEmitted) {
                connect(raw, *next, startPoint(input[i]), endPoint(input[lastEmitted]),
                        tangentAt(input[lastEmitted], 1.0), tangentAt(input[i], 0.0), d, sign, join, mitreLimit,
                        tol, wedges);
            } else {
                const JoinType j =
                    collapsed || roundAfter ? JoinType::Round : cornerJoin(input[i - 1], input[i]);
                connectOrBridge(*next, i - 1, i, j);
            }
            record(at, at);  // joins and connectors sit at the vertex
        }
        raw.push_back(*next);
        record(at, at + 1.0);
        roundAfter = collapsed;
        lastEmitted = i;
        if (firstEmitted == n) {
            firstEmitted = i;
        }
    }
    if (closed && raw.size() > 1) {
        Span first = raw.front();
        if (firstEmitted != n && collapsedBefore[firstEmitted] != n && collapsedBefore[firstEmitted] == lastEmitted) {
            connect(raw, first, startPoint(input[firstEmitted]), endPoint(input[lastEmitted]),
                    tangentAt(input[lastEmitted], 1.0), tangentAt(input[firstEmitted], 0.0), d, sign, join,
                    mitreLimit, tol, wedges);
        } else {
            connectOrBridge(first, input.size() - 1, 0,
                            roundAfter ? JoinType::Round : cornerJoin(input.back(), input.front()));
        }
        raw.front() = first;
        const double at = static_cast<double>(input.size());
        record(at, at);
    }
    return rc;
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
                                   double d, double sign, double lateral, double opposite) {
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

}  // namespace

std::vector<Contour> offsetOpenContour(const Contour& contour, double distance, OffsetSide side,
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
    std::optional<SpanTree> spanTree;
    // Crowded vertices: input vertices with another vertex within the tolerance, the two ends of
    // a sub-tolerance span or a vertex the path comes back to (T-0009). Vertex i is the start of
    // input span i (for an open path, vertex n is the end of the last span).
    const std::size_t vertexCount = input.size() + (cyclic ? 0 : 1);
    auto vertexAt = [&](std::size_t v) {
        return v < input.size() ? startPoint(input[v]) : endPoint(input.back());
    };
    std::vector<bool> crowded(vertexCount, false);
    {
        // Buckets of the tolerance: the first other vertex found within it settles a vertex, so a
        // bucket holding thousands of them costs no more than a few distance tests each.
        const double cell = std::max(tol.distance, 1e-9);
        std::map<std::pair<long long, long long>, std::vector<std::size_t>> buckets;
        auto key = [&](Vec2 p) {
            return std::pair<long long, long long>{static_cast<long long>(std::floor(p.x / cell)),
                                                   static_cast<long long>(std::floor(p.y / cell))};
        };
        for (std::size_t v = 0; v < vertexCount; ++v) {
            buckets[key(vertexAt(v))].push_back(v);
        }
        for (std::size_t v = 0; v < vertexCount && tol.distance > 0.0; ++v) {
            const Vec2 p = vertexAt(v);
            const auto [cx, cy] = key(p);
            for (long long dy = -1; dy <= 1 && !crowded[v]; ++dy) {
                for (long long dx = -1; dx <= 1 && !crowded[v]; ++dx) {
                    const auto it = buckets.find({cx + dx, cy + dy});
                    if (it == buckets.end()) {
                        continue;
                    }
                    for (const std::size_t w : it->second) {
                        if (w != v && geometry::distance(vertexAt(w), p) <= tol.distance) {
                            crowded[v] = true;
                            break;
                        }
                    }
                }
            }
        }
    }
    // Domination near crowded vertices (T-0009): every raw or boundary element passing closer
    // than keepAbove to an input vertex is pruned there anyway, and next to a run of
    // sub-tolerance spans (a cluster of thousands of vertices within a few micrometres, a tiny
    // zig-zag) nearly all of every join arc is: it sweeps the turn at its vertex, but only a
    // sliver of it, where the vertex sticks out, lies on the boundary. Testing those elements
    // against each other made the cost grow with the square (and the pieces with the cube) of the
    // number of such spans. So each element's stretches strictly within keepAbove of a crowded
    // vertex near it are found exactly (a circle meets a line or circle in one interval) and left
    // out: the element is dead if nothing is left, and otherwise only the box of what is left is
    // used to find candidate pairs. The crowded vertices near a point are those on the convex
    // hulls of the crowded vertices in the tolerance-sized grid cells around it (a vertex inside
    // that hull is in the interior of the neighbourhood wherever it would dominate).
    const double cellSize = std::max(tol.distance, 1e-9);
    std::map<std::pair<long long, long long>, std::vector<Vec2>> crowdedCells;
    auto cellOf = [&](Vec2 p) {
        return std::pair<long long, long long>{static_cast<long long>(std::floor(p.x / cellSize)),
                                               static_cast<long long>(std::floor(p.y / cellSize))};
    };
    for (std::size_t v = 0; v < vertexCount; ++v) {
        if (crowded[v]) {
            crowdedCells[cellOf(vertexAt(v))].push_back(vertexAt(v));
        }
    }
    for (auto& [key, pts] : crowdedCells) {
        pts = convexHull(pts);
    }
    if (!crowdedCells.empty()) {
        spanTree.emplace(input);
    }
    // Some input span closer than `limit` to `p` (the same answer with or without the tree).
    auto closer = [&](Vec2 p, double limit, std::size_t& w) {
        return spanTree ? spanTree->closer(p, limit, w) : closerThan(p, limit, input, inputGrid, w);
    };
    std::vector<Vec2> dominators;
    auto collectDominators = [&](Vec2 near) {
        const auto [cx, cy] = cellOf(near);
        for (long long dy = -1; dy <= 1; ++dy) {
            for (long long dx = -1; dx <= 1; ++dx) {
                const auto it = crowdedCells.find({cx + dx, cy + dy});
                if (it != crowdedCells.end()) {
                    dominators.insert(dominators.end(), it->second.begin(), it->second.end());
                }
            }
        }
    };
    const double dominateBelow =
        keepAbove - 1e-12 * std::max(1.0, d);  // strictly inside, whatever the rounding
    // Live parameter intervals of `s` given the crowded vertices near `anchors`; returns false
    // (and leaves `live` alone) when none is near.
    std::vector<std::pair<double, double>> dominated;
    auto liveParts = [&](const Span& s, std::initializer_list<Vec2> anchors,
                         std::vector<std::pair<double, double>>& live) {
        dominators.clear();
        for (const Vec2 a : anchors) {
            collectDominators(a);
        }
        if (dominators.empty()) {
            return false;
        }
        dominated.clear();
        for (const Vec2 w : dominators) {
            closeIntervals(s, w, dominateBelow - 1e-12 * std::max(std::fabs(w.x), std::fabs(w.y)), dominated);
        }
        live = uncovered(dominated);
        return true;
    };
    auto liveBox = [&](const Span& s, const std::vector<std::pair<double, double>>& live) {
        return bounds(subSpan(s, live.front().first, live.back().second)).inflated(1e-9);
    };

    // Dead elements: distance to a segment is a convex function of the point, so a raw element
    // whose convex hull points (see hullPoints) are all closer than keepAbove to one input line
    // lies wholly that close to it, and every piece of it would be pruned. At a large distance
    // relative to the feature size most of the raw chain is like that; dropping these elements
    // before splitting removes most of the boundary-split and pruning work (T-0006). Splits a
    // live element would get from crossing a dead one lie strictly inside the neighbourhood,
    // between two pieces that are pruned anyway.
    std::vector<bool> dead(m, false);
    std::vector<Vec2> hull;
    std::vector<std::vector<std::pair<double, double>>> liveOf(m);
    std::vector<bool> trimmed(m, false);
    auto vertexIndex = [&](double pos) {
        const auto v = static_cast<std::size_t>(std::max(0.0, pos));
        return cyclic ? v % vertexCount : std::min(v, vertexCount - 1);
    };
    for (std::size_t i = 0; i < m; ++i) {
        if (!crowdedCells.empty() &&
            liveParts(raw[i], {vertexAt(vertexIndex(rc.posA[i])), vertexAt(vertexIndex(rc.posB[i]))},
                      liveOf[i]) &&
            !(liveOf[i].size() == 1 && liveOf[i].front().first <= 0.0 && liveOf[i].front().second >= 1.0)) {
            trimmed[i] = true;
            dead[i] = liveOf[i].empty();
            continue;
        }
        hullPoints(raw[i], hull);
        const Vec2 a = hull.front();
        dead[i] = inputGrid.queryUntil(
            Box2{a - Vec2{keepAbove, keepAbove}, a + Vec2{keepAbove, keepAbove}}, [&](std::size_t j) {
                if (!std::holds_alternative<LineSpan>(input[j])) {
                    return false;
                }
                return std::all_of(hull.begin(), hull.end(),
                                   [&](Vec2 p) { return distanceToSpan(p, input[j]) < keepAbove; });
            });
    }

    // 2. Split the raw chain at its self-intersections (candidate pairs from a uniform grid).
    std::vector<std::vector<double>> splits(m);
    std::vector<Box2> boxes(m);
    for (std::size_t i = 0; i < m; ++i) {
        if (!dead[i]) {
            boxes[i] = trimmed[i] ? liveBox(raw[i], liveOf[i]) : bounds(raw[i]).inflated(1e-9);
        }
    }
    BoxGrid rawGrid(boxes);
    std::vector<std::pair<std::size_t, std::size_t>> overlapping;  // element pairs on one carrier
    auto adjacentShared = [&](std::size_t i, std::size_t j, double ti, double tj) {
        if (j == i + 1 && ti >= 1.0 - kParamEps && tj <= kParamEps) {
            return true;
        }
        return cyclic && m > 1 && i == 0 && j + 1 == m && ti <= kParamEps && tj >= 1.0 - kParamEps;
    };
    for (std::size_t i = 0; i < m; ++i) {
        if (dead[i]) {
            continue;
        }
        rawGrid.query(boxes[i], [&](std::size_t j) {
            if (j <= i) {
                return;
            }
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
    //    So does a "vertex" where the input is not continuous (an input gap): both ends there
    //    get full circles. Splitting at whole circles cost one split per raw element per vertex
    //    within d, which dominated dense input at a large d (T-0006).
    std::vector<Span> boundary;
    std::vector<std::array<Vec2, 3>> boundaryAnchors;  // input points each element belongs to
    // End points of the boundary elements at crowded vertices (see below).
    std::vector<Vec2> clusterCorners;
    for (std::size_t i = 0; i < input.size(); ++i) {
        const Span& s = input[i];
        const Span& prev = input[i == 0 ? input.size() - 1 : i - 1];
        const std::size_t firstOfVertex = boundary.size();
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
        while (boundaryAnchors.size() < boundary.size()) {
            boundaryAnchors.push_back({endPoint(prev), startPoint(s), endPoint(s)});
        }
        if (crowded[i] || crowded[(i + 1) % vertexCount]) {
            for (std::size_t b = firstOfVertex; b < boundary.size(); ++b) {
                const auto* arc = std::get_if<ArcSpan>(&boundary[b]);
                if (!arc || !arc->isFullCircle()) {
                    clusterCorners.push_back(startPoint(boundary[b]));
                    clusterCorners.push_back(endPoint(boundary[b]));
                }
            }
        }
    }
    if (!cyclic) {
        boundary.emplace_back(ArcSpan::fullCircle(endPoint(input.back()), d));
        const Vec2 e = endPoint(input.back());
        boundaryAnchors.push_back({e, e, e});
    }
    std::vector<std::pair<double, double>> boundaryLive;
    for (std::size_t bi = 0; bi < boundary.size(); ++bi) {
        const Span& b = boundary[bi];
        const auto* arc = std::get_if<ArcSpan>(&b);
        const bool wholeCarrier = arc && arc->isFullCircle();
        Box2 queryBox = bounds(b).inflated(1e-9);
        // A stretch of the boundary strictly within keepAbove of an input vertex is not on the
        // boundary of the neighbourhood: only the rest can separate kept from pruned pieces.
        const auto& an = boundaryAnchors[bi];
        if (!crowdedCells.empty() && liveParts(b, {an[0], an[1], an[2]}, boundaryLive)) {
            if (boundaryLive.empty()) {
                continue;
            }
            queryBox = liveBox(b, boundaryLive);
        }
        rawGrid.query(queryBox, [&](std::size_t i) {
            for (const Vec2 p : carrierIntersections(raw[i], b)) {
                const double t = carrierParam(raw[i], p);
                if (inUnit(t) && (wholeCarrier || inUnit(carrierParam(b, p)))) {
                    splits[i].push_back(t);
                }
            }
        });
    }
    //    Next to a sub-tolerance span the vertices lie closer together than any feature, and
    //    the circles about them (raw joins, inverted offsets of tiny arcs) nearly coincide. Such a
    //    circle can pass within the pruning margin inside the end point of a neighbouring
    //    boundary element without crossing it, and run from "on the boundary" (kept) straight
    //    into the interior of that neighbour's offset strip (T-0009: 0.03 d deep). So each raw
    //    element is also split where it passes that close to the end point of a boundary
    //    element at such a vertex (at its closest point); a split too many is harmless.
    const double cornerTouch = 2.0 * (d - keepAbove);
    for (const Vec2 e : clusterCorners) {
        const double reach = cornerTouch + 16.0 * std::numeric_limits<double>::epsilon() *
                                               std::max({1.0, std::fabs(e.x), std::fabs(e.y), d});
        rawGrid.query(Box2{e - Vec2{reach, reach}, e + Vec2{reach, reach}}, [&](std::size_t i) {
            Vec2 foot;
            if (const auto* line = std::get_if<LineSpan>(&raw[i])) {
                const Vec2 u = line->end - line->start;
                foot = line->start + u * ((e - line->start).dot(u) / u.lengthSquared());
            } else {
                const auto& arc = std::get<ArcSpan>(raw[i]);
                const Vec2 v = e - arc.centre;
                if (v.lengthSquared() == 0.0) {
                    return;
                }
                foot = arc.centre + v * (arc.radius / v.length());
            }
            const double t = carrierParam(raw[i], foot);
            if (geometry::distance(foot, e) <= reach && t > kParamEps && t < 1.0 - kParamEps) {
                splits[i].push_back(t);
            }
        });
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
        bool hit = false;
        wedgeGrid.query(Box2{p, p}, [&](std::size_t w) {
            if (hit || !insideRegion(p, wedges[w])) {
                return;
            }
            double gap = std::numeric_limits<double>::infinity();
            for (const Span& s : wedges[w]) {
                gap = std::min(gap, distanceToSpan(p, s));
            }
            hit = gap > onBoundary;
        });
        return hit;
    };
    std::vector<Piece> kept;
    std::size_t witness = input.size();
    // Next to a sub-tolerance span the circles about its end points (and about the ends of a
    // tiny arc) nearly coincide, and a raw element can run from within the pruning margin of
    // the neighbourhood boundary into the interior by a few times that margin without crossing
    // any boundary element (T-0009: a Round join about one end of a 5e-9 mm arc lies up to
    // 3e-9 mm inside the circle about its other end, the split between them sitting at the
    // start of the join). Judged by three samples such a piece goes as a whole, losing the
    // stretch that is within the margin, where the circles are still one boundary to rounding.
    // So a failing piece near such a span whose samples all lie within a few margins of d is
    // sampled finely, and if it is clear at exactly one end, its stretch at least keepAbove from
    // the input from that end (the other end of it located by bisection) is kept.
    // Crowded vertices bucketed by d + 2 tol, to tell whether a point is that close to one (boxes
    // of that size in a BoxGrid would each fill many cells).
    const double clusterReach = d + 2.0 * tol.distance;
    std::map<std::pair<long long, long long>, std::vector<Vec2>> clusterCells;
    auto clusterKey = [&](Vec2 p) {
        return std::pair<long long, long long>{static_cast<long long>(std::floor(p.x / clusterReach)),
                                               static_cast<long long>(std::floor(p.y / clusterReach))};
    };
    for (std::size_t v = 0; v < vertexCount; ++v) {
        if (crowded[v]) {
            clusterCells[clusterKey(vertexAt(v))].push_back(vertexAt(v));
        }
    }
    auto nearCluster = [&](Vec2 p) {
        const auto [cx, cy] = clusterKey(p);
        for (long long dy = -1; dy <= 1; ++dy) {
            for (long long dx = -1; dx <= 1; ++dx) {
                const auto it = clusterCells.find({cx + dx, cy + dy});
                if (it == clusterCells.end()) {
                    continue;
                }
                for (const Vec2 v : it->second) {
                    if (std::fabs(v.x - p.x) <= clusterReach && std::fabs(v.y - p.y) <= clusterReach) {
                        return true;
                    }
                }
            }
        }
        return false;
    };
    auto clearAt = [&](std::size_t i, double t) {
        const Vec2 q = pointAt(raw[i], t);
        return !closer(q, keepAbove, witness) && !inWedge(q);
    };
    // True when every sample of the piece lies within a few pruning margins of d from the
    // input (and outside the corner regions): only then can part of it be within the margin.
    const double band = 8.0 * (d - keepAbove);
    auto marginal = [&](const Piece& piece) {
        for (const double f : {0.25, 0.5, 0.75}) {
            const Vec2 q = pointAt(raw[piece.element], piece.t0 + (piece.t1 - piece.t0) * f);
            if (closer(q, d - band, witness) || inWedge(q)) {
                return false;
            }
        }
        return true;
    };
    auto keepClearStretches = [&](const Piece& piece) {
        constexpr std::size_t kFine = 32;
        const double len = length(raw[piece.element]);
        std::vector<double> ts(kFine + 1);
        std::vector<char> clear(kFine + 1);
        for (std::size_t k = 0; k <= kFine; ++k) {
            ts[k] = piece.t0 + (piece.t1 - piece.t0) * (static_cast<double>(k) / static_cast<double>(kFine));
            clear[k] = clearAt(piece.element, ts[k]) ? char{1} : char{0};
        }
        // Only a piece that runs from within the margin at one end into the interior at the
        // other is refined; one that dips in the middle (clear at both ends, a sliver between
        // them) or is cut at both ends goes as before, so no new sub-tolerance gap appears
        // inside a raw element.
        if (clear.front() == clear.back()) {
            return;
        }
        // Boundary between a clear and a cut sample, to the clear side.
        auto edge = [&](double good, double bad) {
            for (int it = 0; it < 50; ++it) {
                const double mid = 0.5 * (good + bad);
                (clearAt(piece.element, mid) ? good : bad) = mid;
            }
            return good;
        };
        std::size_t k = 0;
        if (clear.front()) {
            while (k + 1 <= kFine && clear[k + 1]) {
                ++k;
            }
            const double hi = edge(ts[k], ts[k + 1]);
            if ((hi - piece.t0) * len > 1e-9) {
                kept.push_back(Piece{piece.element, piece.t0, hi});
            }
        } else {
            k = kFine;
            while (k > 0 && clear[k - 1]) {
                --k;
            }
            const double lo = edge(ts[k], ts[k - 1]);
            if ((piece.t1 - lo) * len > 1e-9) {
                kept.push_back(Piece{piece.element, lo, piece.t1});
            }
        }
    };
    for (std::size_t i = 0; i < m; ++i) {
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
            if ((ts[k + 1] - ts[k]) * len <= 1e-9) {
                continue;
            }
            const Piece piece{i, ts[k], ts[k + 1]};
            if (trimmed[i] && std::none_of(liveOf[i].begin(), liveOf[i].end(), [&](const auto& iv) {
                    return iv.first < piece.t1 && iv.second > piece.t0;
                })) {
                continue;  // wholly within keepAbove of an input vertex
            }
            bool ok = true;
            for (const double f : {0.25, 0.5, 0.75}) {
                const Vec2 q = pointAt(raw[i], piece.t0 + (piece.t1 - piece.t0) * f);
                if (closer(q, keepAbove, witness) || inWedge(q)) {
                    ok = false;
                    break;
                }
            }
            if (ok) {
                kept.push_back(piece);
            } else if (!clusterCells.empty()) {
                const Vec2 mid = pointAt(raw[i], 0.5 * (piece.t0 + piece.t1));
                if (nearCluster(mid) && marginal(piece)) {
                    keepClearStretches(piece);
                }
            }
        }
    }
    // Two points computed separately at the same place on the offset (a piece's end and its
    // continuation's start) agree to rounding: much closer than this, while distinct split
    // points lie much further apart.
    auto coincide = [&](Vec2 p) { return 1e-9 * std::max({1.0, d, std::fabs(p.x), std::fabs(p.y)}); };
    // Overlapping elements on one carrier (see mergeOnCarriers): one boundary piece per stretch.
    kept = mergeOnCarriers(raw, kept, overlapping, d, sign, 0.5 * tol.distance,
                           std::min(5e-7 * std::max(1.0, d), 0.1 * tol.distance * tol.distance / d));
    if (kept.empty()) {
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
        [&](Vec2 p) { return !closer(p, snapAbove, snapWitness) && !inWedge(p); },
        [&](Vec2 p) {
            return join != JoinType::Round || closer(p, snapBelow, farWitness);
        },
        [&](Vec2 p) {
            // Only distances near d matter to the checks (beyond d + 2 tol a Round point already
            // fails notFar), so the query box stays small: with 2d it cost 1.5-2x on dense input.
            const double reach = d + 2.0 * tol.distance;
            if (spanTree) {
                const double cap = 1.5 * reach;
                const double near = spanTree->nearest(p, cap);
                return near < cap ? near : kInf;
            }
            double nearest = kInf;
            inputGrid.query(Box2{p - Vec2{reach, reach}, p + Vec2{reach, reach}},
                            [&](std::size_t i) { nearest = std::min(nearest, distanceToSpan(p, input[i])); });
            return nearest;
        },
        [&](Vec2 p) {
            const double reach = 2.0 * d + 2.0 * tol.distance;
            double best = kInf;
            Vec2 at = p;
            if (spanTree) {
                spanTree->nearest(p, 1.5 * reach, &at);
                return at;
            }
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
    // The stretch of a raw element between two of its pieces that the chain steps over (a
    // sub-tolerance sliver) is re-emitted with them only where it stays within the bridge sagitta
    // tol^2 / (8d) of d: elsewhere the pieces stay apart and the junction is closed like any other
    // (T-0009: a re-merged line 2.7e-8 mm inside d at d = 5.4, beyond that bound).
    std::size_t gapWitness = input.size();
    const double gapBelow = d - tol.distance * tol.distance / (8.0 * d);
    auto gapClear = [&](std::size_t element, double t0, double t1) {
        if (t1 <= t0) {
            return true;
        }
        for (int k = 1; k <= 7; ++k) {
            if (closer(pointAt(raw[element], t0 + (t1 - t0) * k / 8.0), gapBelow, gapWitness)) {
                return false;
            }
        }
        return true;
    };
    auto emit = [&](std::vector<Piece>::const_iterator b, std::vector<Piece>::const_iterator e, bool loop) {
        emitting.clear();
        for (auto it = b; it != e; ++it) {
            emitting.emplace_back(it->element, it->t0);
        }
        std::sort(emitting.begin(), emitting.end());
        std::vector<Piece> merged;
        for (auto it = b; it != e; ++it) {
            // Only across a gap no longer than the tolerance: on a raw arc extended nearly all
            // the way round (a Sharp corner at a rounding-size arc sweeping 4.85 rad), its last
            // piece starts within the tolerance of its first piece's end, and merging the two
            // re-emitted the whole circle, pruned stretches through the input included (T-0009).
            if (!merged.empty() && merged.back().element == it->element &&
                it->t0 >= merged.back().t1 - kParamEps &&
                (it->t0 - merged.back().t1) * length(raw[it->element]) <= tol.distance &&
                gapClear(it->element, merged.back().t1, it->t0)) {
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
        // Trailing tiny pieces go too, as far as they stay within the tolerance of the chain's
        // end (one past the last piece that may be kept).
        std::size_t solidEnd = spans.size();
        while (solidEnd > 0 && tiny[solidEnd - 1] &&
               geometry::distance(startPoint(spans[solidEnd - 1]), endPoint(spans.back())) <= tol.distance) {
            --solidEnd;
        }
        double total = 0.0;
        std::vector<Span> solid;
        std::vector<std::vector<Span>> dropped(1);
        std::vector<Span> leading;
        for (std::size_t k = 0; k < solidEnd; ++k) {
            // A tiny piece is dropped while it ends within the tolerance of the last piece kept
            // (or, before the first, of the chain's start). Leading tiny pieces used to be
            // dropped however far they reached, so an offset made only of sub-tolerance pieces
            // (a polyline sampled every 0.4 um, T-0009) vanished.
            const Vec2 anchor = solid.empty() ? startPoint(spans.front()) : endPoint(solid.back());
            if (tiny[k] && geometry::distance(anchor, endPoint(spans[k])) <= tol.distance) {
                (solid.empty() ? leading : dropped.back()).push_back(spans[k]);
                continue;
            }
            total += length(spans[k]);
            solid.push_back(spans[k]);
            dropped.emplace_back();
        }
        // The chain's own length decides, not that of the pieces kept above: a chain of
        // sub-tolerance pieces (the lens arcs along the tops of a tiny zig-zag, T-0009) keeps
        // only every few of them and went although it was millimetres long.
        double chainLength = 0.0;
        for (const Span& sp : spans) {
            chainLength += length(sp);
        }
        if (!solid.empty() && std::max(total, chainLength) > tol.distance * (loop ? 4.0 : 1.0)) {
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
                // An open chain neither starts nor ends with a span no longer than the
                // tolerance (ADR-0012 section 7): where a cut or a node left one there (fam5
                // 0xf50009c9: 7.8e-4 mm), it goes, as the leading and trailing tiny pieces do.
                // Only as far as the tolerance from either end, though: a chain made only of such
                // spans (along the tops of a tiny zig-zag, T-0009) keeps the rest of them.
                if (!piece.closed) {
                    double cut = 0.0;
                    while (piece.spans.size() > 1 && isDegenerate(piece.spans.front(), tol) &&
                           cut + length(piece.spans.front()) <= tol.distance) {
                        cut += length(piece.spans.front());
                        piece.spans.erase(piece.spans.begin());
                    }
                    cut = 0.0;
                    while (piece.spans.size() > 1 && isDegenerate(piece.spans.back(), tol) &&
                           cut + length(piece.spans.back()) <= tol.distance) {
                        cut += length(piece.spans.back());
                        piece.spans.pop_back();
                    }
                    double remaining = 0.0;
                    for (const Span& sp : piece.spans) {
                        remaining += length(sp);
                    }
                    if (remaining <= tol.distance) {
                        continue;
                    }
                }
                if (!isSliverLoop(piece, tol)) {
                    result.push_back(std::move(piece));
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
    return result;
}

namespace detail {

bool turnArcOntoJunction(const Span& a, ArcSpan& b) {
    return exactJunctionByAngle(a, b);
}

}  // namespace detail

}  // namespace carvetto::geometry
