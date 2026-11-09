#include "carvetto/toolpath/profile.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <limits>
#include <map>
#include <new>
#include <numbers>
#include <numeric>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fixed_point.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/clearance.hpp"
#include "carvetto/toolpath/path_walk.hpp"
#include "block_settle.hpp"
#include "enclosed_pieces.hpp"
#include "carvetto/geometry/cancel.hpp"
#include "work_meter.hpp"

namespace carvetto::toolpath {

using geometry::Contour;
using geometry::Orientation;
using geometry::Span;
using geometry::Tolerance;
using geometry::Vec2;

double profileToolRadius(const tooling::Tool& tool, const PassPlan& passes) {
    return 0.5 * tooling::cuttingWidthAtDepth(tool, passes.startDepth + passes.cutDepth);
}

namespace {

// One cut: a path machined through its levels, with what planning decided about it.
/// A stretch of a V-bit's sharp-corner excursion as the tab layer sees it: the cone's disc there
/// runs from radius `ra` about `a` to `rb` about `b` (T-0156), the convex hull of the two discs.
struct Taper {
    Vec2 a{};
    Vec2 b{};
    double ra{0.0};
    double rb{0.0};
};

/// The sharp-internal-corner moves of a V-bit at one inside corner of a cut (cornerMoves), planned
/// before the tabs are judged so the tab check and the loose-piece search see the cuts they make.
/// It happens only where the cut is at full depth, so not within a kept tab's footprint.
/// A tool position that cuts a disc smaller than the tab-layer radius (a V-bit's corner moves).
struct ToolDisc {
    Vec2 p{};
    double rho{0.0};
};

struct Excursion {
    std::size_t kink{0};        // index in cut.walk.kinks(1e-3)
    double u{0.0};              // arc length of the corner on the cut
    std::vector<Move> moves;    // as emitted (from the cut at q, back to it)
    std::vector<Taper> tapers;  // the tab layer's view of those moves
};

struct Cut {
    Contour path;
    std::vector<std::size_t> sources;  // input vectors it was offset from
    bool contourOnLeft{true};          // the vector lies on the left of the direction of travel
    bool fromClosed{true};
    bool exact{true};      // exact offset of its source (trusted against it); false = merged region
    double distance{0.0};  // |offset| from the vectors
    double offsetBy{0.0};  // distance the path was offset by (a merged region: + the chordal tolerance)
    double offsetSlack{0.0};  // how much nearer a polygonal (region) path may come: its chords' sagitta
    ArcLengthPath walk;
    std::vector<TabSpan> tabs;  // raised stretches (arc length on `walk`)
    // How the tabs were placed (placeTabsOn), so matchRoughingTabs can place them again without
    // one it refuses: the centres asked for, the requested centre each tab came from, the points
    // kept clear, the move margin, and how many fitting dropped.
    std::vector<double> tabRequested;
    std::vector<std::size_t> tabOrigin;
    std::vector<double> tabKeepClear;
    double tabMoveMargin{0.0};
    std::size_t tabFitDropped{0};
    Contour leadIn;  // planned lead-in (empty: none or none fits)
    std::vector<Excursion> excursions;  // V-bit sharp-corner moves (sharpInternal_ only)
};

/// Index of the cuts' start points (T-0131), for otherStarts.
BoxIndex startIndex(const std::vector<Cut>& cuts) {
    std::vector<geometry::Box2> boxes;
    boxes.reserve(cuts.size());
    for (const Cut& c : cuts) {
        const Vec2 p = c.walk.pointAt(0.0);
        boxes.push_back(geometry::Box2{p, p});
    }
    return BoxIndex(std::move(boxes));
}

/// Index of the boxes of the cuts' walked paths (T-0131).
BoxIndex walkIndex(const std::vector<Cut>& cuts) {
    std::vector<geometry::Box2> boxes;
    boxes.reserve(cuts.size());
    for (const Cut& c : cuts) {
        boxes.push_back(c.walk.contour().bounds());
    }
    return BoxIndex(std::move(boxes));
}

struct Input {
    std::vector<Contour> contours;  // closed ones oriented by nesting parity
    std::vector<bool> valid;
    std::vector<int> depth;
    std::vector<double> area;
};

class DisjointSets {
   public:
    explicit DisjointSets(std::size_t n) : parent_(n) { std::iota(parent_.begin(), parent_.end(), std::size_t{0}); }
    std::size_t find(std::size_t x) {
        while (parent_[x] != x) {
            parent_[x] = parent_[parent_[x]];
            x = parent_[x];
        }
        return x;
    }
    void unite(std::size_t a, std::size_t b) {
        a = find(a);
        b = find(b);
        if (a != b) {
            parent_[std::max(a, b)] = std::min(a, b);
        }
    }

   private:
    std::vector<std::size_t> parent_;
};

// A closed cut may close only within the tolerance; snap a line end so the loop closes exactly.
void closeExactly(Contour& c) {
    if (!c.closed || c.spans.empty()) {
        return;
    }
    const Vec2 first = geometry::startPoint(c.spans.front());
    if (auto* last = std::get_if<geometry::LineSpan>(&c.spans.back())) {
        last->end = first;
    } else if (auto* head = std::get_if<geometry::LineSpan>(&c.spans.front())) {
        head->start = geometry::endPoint(c.spans.back());
    }
}

// The input edges that meet at the vertex a cut is asked to start at, in the vector's own order.
struct TipEdges {
    Span in;   // ends at the vertex
    Span out;  // starts there
};

// Arc length on the closed walk at which a cut asked to start nearest `target` starts (ADR-0016
// §11, T-0158). Some targets have several equally near points, and the plain projection then
// follows rounding (the direction of a near-zero vector, or which of two equal distances comes
// out smaller), which differs between compilers, FMA contraction and libm. Two such ties are
// broken by rule, comparing distances within `eps`:
//  1. Round corner: `target` lies at the centre of an arc of the walk that is at the nearest
//     distance (a convex tip's round join on an Outside cut, a circle round its bounding-box
//     centre): start at the beginning of the first such arc (lowest span index), if that point is
//     itself at the nearest distance.
//  2. Sharp corner: `target` lies on the vector (the nearest distance is `cutDistance`, the
//     distance the cut was offset by — a merged region is offset by the chordal tolerance more —
//     or up to `slack` less, where a region's polygonal round join cuts inside by its sagitta).
//     When `target` is a vertex of the cut's vectors (`tipEdges`: the input edges into and out of
//     it), the start is the corner the offset built there, found from those two edges alone: the
//     spans on each edge's offset (its line or circle at `cutDistance`, to rounding) bound it, and
//     what lies between, in travel, from the last span on the first edge's offset to the next on
//     the other's, is the corner: the mitre, a chamfer, a join arc or a region's join chords. The
//     cut starts at its first junction: exactly the mitre, the chamfer's first corner, the join's
//     first point (`target`'s foot on the first edge's offset). Spans within rounding of both
//     offsets (a sliver of the first edge's offset at the mitre, a join chord too short to tell)
//     put the start at the first of their junctions on the circle of `cutDistance` about `target`,
//     else after them. Neighbouring vertices, collinear or not, and split points of the offset
//     lines never bound the corner, so they cannot take the start; no turn or length is compared.
//     Otherwise (no vertex, or no such corner with a nearest span, e.g. where an edge's offset is
//     consumed) the spans at the nearest distance must form one run of two or more consecutive
//     spans; when every span is near (a loop of two or three spans) the loop is broken at the
//     clearly widest gap between consecutive feet. Then:
//     - with round joins (`roundJoins`): the first point of the run in travel on that circle;
//     - otherwise, of the run's junctions at which `target` is equidistant from the two spans'
//       carrier lines or circles to rounding, the one whose feet of `target` lie most evenly
//       either side of it (with none equidistant, of all junctions).
//     Near spans in separate places (a box anchor on a symmetric shape) stay with the projection.
// Otherwise the nearest point (`ArcLengthPath::project`).
double tieFreeStart(const ArcLengthPath& walk, Vec2 target, double eps, double cutDistance, double slack,
                    bool roundJoins, const std::function<std::optional<TipEdges>()>& tipEdges) {
    const Contour& c = walk.contour();
    const std::size_t n = c.spans.size();
    if (n == 0) {
        return 0.0;
    }
    std::vector<double> dist(n);
    std::vector<double> param(n);
    double nearest = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
        const geometry::ClosestPoint cp = geometry::closestPoint(c.spans[i], target);
        dist[i] = cp.distance;
        param[i] = cp.t;
        nearest = std::min(nearest, dist[i]);
    }
    for (std::size_t i = 0; i < n; ++i) {
        if (const auto* a = std::get_if<geometry::ArcSpan>(&c.spans[i])) {
            if (geometry::distance(a->centre, target) <= eps && a->radius <= nearest + eps &&
                geometry::distance(a->startPoint(), target) <= nearest + eps) {
                return walk.spanStart(i);
            }
        }
    }
    if (walk.closed() && n >= 2 && nearest <= cutDistance + eps && nearest >= cutDistance - slack - eps) {
        auto isNear = [&](std::size_t i) { return dist[i] <= nearest + eps; };
        // Where the projection already lands exactly on the chosen point, keep its arc length (the
        // same point, and the same output as before this rule).
        auto startAt = [&](double u) {
            const double projected = walk.project(target);
            return walk.pointAt(projected) == walk.pointAt(u) ? projected : u;
        };
        const double scale = std::max({1.0, std::fabs(target.x), std::fabs(target.y), cutDistance});
        // Coordinates of a merged region lie on Clipper2's grid; exact offsets carry rounding and
        // the offset's own closing of junction residuals (up to 1e-10 x the scale).
        const double coord = slack > 0.0 ? 1.0 / geometry::kFixedPointScale : 1e-10 * scale;
        // A region's round join is a polygon whose vertices Clipper2 puts on the circle of radius
        // cutDistance about the (grid-rounded) tip, starting with the foot on the incoming offset
        // edge; each is off by up to half a grid unit per axis, as is the tip itself. An exact
        // offset's chord across a sub-tolerance join arc ends on that circle to rounding (and the
        // offset's closing of residuals, 1e-10 x the scale).
        const double onCircle = slack > 0.0 ? 3.0 / geometry::kFixedPointScale : 4e-10 * scale;
        if (const std::optional<TipEdges> tip = tipEdges()) {
            // Which spans lie on the offset of the input edge entering the target vertex (1) or
            // leaving it (2): start, middle and end at the cut distance from that edge's line or
            // circle, to rounding (a line's direction comes from its end points, so a point
            // farther along than the edge is long is less exact) and, on a region, to its chords'
            // sagitta. The spans the offset builds at the corner itself (mitre, chamfer, join arc
            // or chords) lie off both by far more: they turn against both edges.
            auto onOffsetOf = [&](const Span& s, const Span& edge) {
                const std::array<Vec2, 3> pts{geometry::startPoint(s), geometry::pointAt(s, 0.5), geometry::endPoint(s)};
                if (const auto* l = std::get_if<geometry::LineSpan>(&edge)) {
                    const Vec2 d = l->end - l->start;
                    const double len = d.length();
                    if (!(len > 0.0)) {
                        return false;
                    }
                    for (const Vec2 p : pts) {
                        const double g = std::fabs((p - l->start).cross(d)) / len;
                        const double t = (p - l->start).dot(d) / len;
                        const double beyond = std::max({0.0, -t, t - len});
                        const double noise = 4.0 * coord * (2.0 + 2.0 * beyond / len);
                        if (std::fabs(g - cutDistance) > noise) {
                            return false;
                        }
                    }
                    return true;
                }
                if (const auto* a = std::get_if<geometry::ArcSpan>(&edge)) {
                    const double noise = 8.0 * coord + 4e-12 * a->radius + slack;
                    for (const Vec2 p : pts) {
                        const double rho = geometry::distance(p, a->centre);
                        if (std::min(std::fabs(rho - (a->radius + cutDistance)),
                                     std::fabs(rho - std::fabs(a->radius - cutDistance))) > noise) {
                            return false;
                        }
                    }
                    return true;
                }
                return false;
            };
            // A span within rounding of both (a sliver at the mitre; every span when the vertex
            // is no corner) counts as on either.
            std::vector<int> label(n, 0);
            for (std::size_t i = 0; i < n; ++i) {
                label[i] = (onOffsetOf(c.spans[i], tip->in) ? 1 : 0) + (onOffsetOf(c.spans[i], tip->out) ? 2 : 0);
            }
            // Does the cut run the vector's way (entering edge first) or against it? Asked of the
            // nearest spans on one edge's offset, which must agree.
            double nearestOn = std::numeric_limits<double>::infinity();
            for (std::size_t i = 0; i < n; ++i) {
                if (label[i] == 1 || label[i] == 2) {
                    nearestOn = std::min(nearestOn, dist[i]);
                }
            }
            int met = 0;  // the edge whose offset comes first in travel (1 entering, 2 leaving)
            for (std::size_t i = 0; i < n; ++i) {
                if ((label[i] == 1 || label[i] == 2) && dist[i] <= nearestOn + eps) {
                    const Vec2 along = geometry::tangentAt(c.spans[i], param[i]);
                    const Vec2 edgeDir =
                        label[i] == 1 ? geometry::tangentAt(tip->in, 1.0) : geometry::tangentAt(tip->out, 0.0);
                    const int m = along.dot(edgeDir) > 0.0 ? 1 : 2;
                    met = met == 0 || met == m ? m : -1;
                }
            }
            if (met > 0) {
                // The corner construction lies between a span on the offset of the edge met first
                // and the next labelled span, on the other edge's: the mitre between them, or the
                // chamfer, join arc or chords. It is the one with a nearest span; start at its
                // first junction, which is the mitre, the chamfer's first corner or the join's
                // first point in travel.
                std::size_t pick = n;
                std::size_t count = 0;
                for (std::size_t i = 0; i < n; ++i) {
                    if ((label[i] & met) == 0) {
                        continue;
                    }
                    std::size_t j = (i + 1) % n;
                    bool near = isNear(i);
                    while (j != i && label[j] == 0) {
                        near = near || isNear(j);
                        j = (j + 1) % n;
                    }
                    if (j == i || label[j] != 3 - met) {
                        continue;
                    }
                    // Spans within rounding of both edges' offsets just before (the end of the first
                    // edge's offset, or a join chord too short to tell) leave the first junction
                    // within their length: the first of them on the circle of the cut distance about
                    // the target (where a join starts), else the last.
                    std::size_t from = (i + 1) % n;
                    if (label[i] == 3) {
                        std::size_t k0 = i;
                        while (label[(k0 + n - 1) % n] == 3 && (k0 + n - 1) % n != i) {
                            k0 = (k0 + n - 1) % n;
                        }
                        from = (label[(k0 + n - 1) % n] & met) != 0 ? k0 : (k0 + 1) % n;
                        for (std::size_t k = k0; k != (i + 1) % n; k = (k + 1) % n) {
                            near = near || isNear(k);
                        }
                    }
                    if (!near && !isNear(j)) {
                        continue;
                    }
                    std::size_t at = (i + 1) % n;
                    for (std::size_t k = from; k != (i + 1) % n; k = (k + 1) % n) {
                        if (std::fabs(geometry::distance(geometry::startPoint(c.spans[k]), target) - cutDistance) <=
                            onCircle) {
                            at = k;
                            break;
                        }
                    }
                    pick = at;
                    ++count;
                }
                if (count == 1) {
                    return startAt(walk.spanStart(pick));
                }
            }
        }
        std::size_t nearCount = 0;
        std::size_t runs = 0;
        std::size_t first = n;
        bool longRun = false;
        for (std::size_t i = 0; i < n; ++i) {
            if (isNear(i)) {
                ++nearCount;
            }
            if (isNear(i) && !isNear((i + n - 1) % n)) {
                ++runs;
                first = i;
                longRun = isNear((i + 1) % n);
            }
        }
        bool found = runs == 1 && longRun;
        if (nearCount == n) {
            // Every span is near (a loop of few spans, e.g. an arc closed by one line, mitred at
            // both ends): the run is the whole loop, broken where consecutive feet lie farthest
            // apart along it; it must be clearly the largest gap.
            const double len = walk.length();
            auto foot = [&](std::size_t i) { return walk.spanStart(i) + param[i] * walk.spanLength(i); };
            std::size_t widest = 0;
            double gap1 = -1.0;
            double gap2 = -1.0;
            for (std::size_t i = 0; i < n; ++i) {
                const double g = i + 1 < n ? foot(i + 1) - foot(i) : foot(0) + len - foot(i);
                if (g > gap1) {
                    gap2 = gap1;
                    gap1 = g;
                    widest = i;
                } else if (g > gap2) {
                    gap2 = g;
                }
            }
            first = (widest + 1) % n;
            found = gap1 - gap2 > eps;
        }
        if (found) {
            std::size_t runLength = 0;
            while (runLength < n && isNear((first + runLength) % n)) {
                ++runLength;
            }
            if (nearCount == n) {
                runLength = n;
            }
            // Junction k of the run joins its spans k and k + 1 (span indices first + k ...).
            auto spanOf = [&](std::size_t k) { return (first + k) % n; };
            const std::size_t junctions = runLength - 1;
            if (roundJoins) {
                auto onJoin = [&](std::size_t i) {
                    return std::fabs(geometry::distance(geometry::startPoint(c.spans[i]), target) - cutDistance) <=
                           onCircle;
                };
                for (std::size_t k = 0; k <= junctions; ++k) {
                    std::size_t i = spanOf(k);
                    if (onJoin(i)) {
                        // The join's last chord is shorter than the others, so less near: where the
                        // cut runs the join that way, it is not part of the run. Back up over it.
                        for (std::size_t back = 0; back + 1 < n && onJoin((i + n - 1) % n); ++back) {
                            i = (i + n - 1) % n;
                        }
                        return startAt(walk.spanStart(i));
                    }
                }
            }
            // Distance from `target` to a span's carrier (its line or circle), and how far that may
            // be off: a line's direction comes from its end points, so a short line is less exact.
            auto carrier = [&](std::size_t i, double& noise) {
                const Span& sp = c.spans[i];
                if (const auto* a = std::get_if<geometry::ArcSpan>(&sp)) {
                    noise = coord;
                    return std::fabs(geometry::distance(a->centre, target) - a->radius);
                }
                const Vec2 p0 = geometry::startPoint(sp);
                const Vec2 d = geometry::endPoint(sp) - p0;
                const double len = d.length();
                noise = coord * (1.0 + (len > 0.0 ? cutDistance / len : scale));
                return len > 0.0 ? std::fabs((target - p0).cross(d)) / len : geometry::distance(target, p0);
            };
            std::vector<char> equi(junctions, 0);
            std::vector<double> lopsided(junctions, 0.0);
            bool anyEqui = false;
            for (std::size_t k = 0; k < junctions; ++k) {
                const std::size_t a = spanOf(k);
                const std::size_t b = spanOf(k + 1);
                double na = 0.0;
                double nb = 0.0;
                const double ga = carrier(a, na);
                const double gb = carrier(b, nb);
                equi[k] = std::fabs(ga - gb) <= 4.0 * (na + nb) ? 1 : 0;
                anyEqui = anyEqui || equi[k] != 0;
                // Feet of `target` on the two spans either side of the junction.
                lopsided[k] = std::fabs((1.0 - param[a]) * walk.spanLength(a) - param[b] * walk.spanLength(b));
            }
            std::size_t pick = junctions;
            for (std::size_t k = 0; k < junctions; ++k) {
                if ((equi[k] != 0 || !anyEqui) && (pick == junctions || lopsided[k] < lopsided[pick])) {
                    pick = k;
                }
            }
            if (pick < junctions) {
                return startAt(walk.spanStart(spanOf(pick + 1)));
            }
        }
    }
    return walk.project(target);
}

// A cross-section of a tab's material: the segment across the kerf at one station of the tab.
struct TabSection {
    Vec2 p0;
    Vec2 p1;
};

// Distance from `q` to the cross-section `s`.
double sectionDistance(Vec2 q, const TabSection& s) {
    const Vec2 d = s.p1 - s.p0;
    const double dd = d.x * d.x + d.y * d.y;
    const double t = dd > 0.0 ? std::clamp(((q.x - s.p0.x) * d.x + (q.y - s.p0.y) * d.y) / dd, 0.0, 1.0) : 0.0;
    return geometry::distance(q, s.p0 + d * t);
}

// Distance from `q` to the nearest of `sections` (infinity when there are none).
double sectionDistance(Vec2 q, const std::vector<TabSection>& sections) {
    double best = std::numeric_limits<double>::infinity();
    for (const TabSection& s : sections) {
        best = std::min(best, sectionDistance(q, s));
    }
    return best;
}

// A raster of square cells over a box with discs stamped row by row: a disc adds one interval
// of cells (those whose centres it covers) to every row it crosses, kept as a difference array;
// mask() sums each row once, so a disc costs one step per row it crosses.
class DiscRaster {
   public:
    DiscRaster(const geometry::Box2& box, double h) : box_(box), h_(h) {
        // Work meter (T-0186): one unit per eight cells, charged before the cells are allocated.
        const double cellCount = (std::ceil(box.width() / h) + 1.0) * (std::ceil(box.height() / h) + 1.0);
        detail::chargeWork(detail::unitsOf(cellCount / 8.0));  // (an empty box counts as one)
        nx_ = static_cast<int>(std::ceil(box.width() / h)) + 1;
        ny_ = static_cast<int>(std::ceil(box.height() / h)) + 1;
        diff_.assign(static_cast<std::size_t>(nx_ + 1) * static_cast<std::size_t>(ny_), 0);
    }
    int nx() const { return nx_; }
    int ny() const { return ny_; }
    std::size_t cells() const { return static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_); }
    std::size_t index(int i, int j) const {
        return static_cast<std::size_t>(j) * static_cast<std::size_t>(nx_) + static_cast<std::size_t>(i);
    }
    Vec2 centre(int i, int j) const { return {box_.min.x + (i + 0.5) * h_, box_.min.y + (j + 0.5) * h_}; }
    void add(Vec2 q, double r) {
        const int j0 = std::max(0, static_cast<int>(std::ceil((q.y - r - box_.min.y) / h_ - 0.5)));
        const int j1 = std::min(ny_ - 1, static_cast<int>(std::floor((q.y + r - box_.min.y) / h_ - 0.5)));
        detail::chargeWork(j1 >= j0 ? static_cast<std::uint64_t>(j1 - j0) + 1 : 1);
        for (int j = j0; j <= j1; ++j) {
            const double dy = box_.min.y + (j + 0.5) * h_ - q.y;
            const double w2 = r * r - dy * dy;
            if (w2 < 0.0) continue;
            const double w = std::sqrt(w2);
            const int i0 = std::max(0, static_cast<int>(std::ceil((q.x - w - box_.min.x) / h_ - 0.5)));
            const int i1 = std::min(nx_ - 1, static_cast<int>(std::floor((q.x + w - box_.min.x) / h_ - 0.5)));
            if (i0 > i1) continue;
            const std::size_t row = static_cast<std::size_t>(j) * static_cast<std::size_t>(nx_ + 1);
            ++diff_[row + static_cast<std::size_t>(i0)];
            --diff_[row + static_cast<std::size_t>(i1) + 1];
        }
    }
    /// 1 for every cell covered by a stamped disc; clears the stamps.
    std::vector<char> mask() {
        detail::chargeWork(cells() / 8 + 1);
        std::vector<char> out(cells(), 0);
        for (int j = 0; j < ny_; ++j) {
            const std::size_t row = static_cast<std::size_t>(j) * static_cast<std::size_t>(nx_ + 1);
            int depth = 0;
            for (int i = 0; i < nx_; ++i) {
                depth += diff_[row + static_cast<std::size_t>(i)];
                diff_[row + static_cast<std::size_t>(i)] = 0;
                if (depth > 0) out[index(i, j)] = 1;
            }
            diff_[row + static_cast<std::size_t>(nx_)] = 0;
        }
        return out;
    }

   private:
    geometry::Box2 box_;
    double h_;
    int nx_{0};
    int ny_{0};
    std::vector<int> diff_;  // per row: +1 where an interval starts, -1 past its end
};

Vec2 probePoint(const Contour& c) {
    return geometry::pointAt(c.spans.front(), 0.5);
}

bool hasBezier(const std::vector<Contour>& cs) {
    for (const Contour& c : cs) {
        for (const Span& s : c.spans) {
            if (std::holds_alternative<geometry::BezierSpan>(s)) {
                return true;
            }
        }
    }
    return false;
}

std::vector<int> nestingDepths(const std::vector<Contour>& cs, const std::vector<bool>& use, const Tolerance& tol) {
    const std::size_t n = cs.size();
    std::vector<int> depth(n, 0);
    std::vector<geometry::Box2> boxes(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (use[i]) {
            boxes[i] = cs[i].bounds();
        }
    }
    const BoxIndex index(boxes);
    for (std::size_t i = 0; i < n; ++i) {
        if (!use[i]) {
            continue;
        }
        const Vec2 p = probePoint(cs[i]);
        for (std::size_t j : index.containing(p)) {
            if (j == i || !use[j]) {
                continue;
            }
            if (geometry::locatePoint(cs[j], p, geometry::FillRule::EvenOdd, tol) == geometry::PointLocation::Inside) {
                ++depth[i];
            }
        }
    }
    return depth;
}

Vec2 anchorOf(const geometry::Box2& b, BoxAnchor a) {
    const double xs[3] = {b.min.x, 0.5 * (b.min.x + b.max.x), b.max.x};
    const double ys[3] = {b.min.y, 0.5 * (b.min.y + b.max.y), b.max.y};
    const int k = static_cast<int>(a);
    return {xs[k % 3], ys[k / 3]};
}

std::string vecName(std::size_t i) {
    return "vector " + std::to_string(i);
}

// A length in the shortest form that reads back to the same double, so an allowance just below
// the limit never prints the same as the limit.
std::string mmText(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr) + " mm";
}

// --- Work meter (T-0186, ADR-0016 section 19) -----------------------------------------------------

/// Why a run was stopped by the builder itself (the meter stops runs for the work limit and cancel).
struct TooManyMoves {};

/// A vector or merged group whose single offset (one uncancellable geometry call) is charged more
/// than `kMaxSingleCallUnits`: refused with its input index (round 3).
struct TooComplexForOneStep {
    std::size_t contour;
    double units;
};

/// The most a single uncancellable geometry call may be charged: 6e8 units (10 s only at about 16 ns
/// a unit; up to 26 s measured on a merged V-bit starburst group). A cancel waits for the call in progress, so a vector that needs more
/// is refused instead (ADR-0016 s19; the trade-off against refusing slow-but-finishing jobs is there).
constexpr double kMaxSingleCallUnits = 6.0e8;

/// Vertices the polygonisation of `span` will produce at `chordal` (lines 1, arcs by sag, Beziers
/// bounded): the size the offset and region calls work on.
double spanVertices(const Span& span, double chordal) {
    return std::visit(
        [&](const auto& sp) -> double {
            using S = std::decay_t<decltype(sp)>;
            if constexpr (std::is_same_v<S, geometry::ArcSpan>) {
                return static_cast<double>(sp.segmentCountFor(chordal));
            } else if constexpr (std::is_same_v<S, geometry::BezierSpan>) {
                const double len = geometry::distance(sp.p0, sp.p1) + geometry::distance(sp.p1, sp.p2) +
                                   geometry::distance(sp.p2, sp.p3);
                return 1.0 + std::sqrt(len / chordal);
            } else {
                return 1.0;
            }
        },
        span);
}

/// An upper bound of the length of `c`, cheap whatever the spans (lines and arcs exact, a Bezier by
/// its control polygon): the exact length of a Bezier costs milliseconds, which the up-front limit
/// check must not.
double lengthBound(const Contour& c) {
    double total = 0.0;
    for (const Span& span : c.spans) {
        total += std::visit(
            [](const auto& sp) -> double {
                using S = std::decay_t<decltype(sp)>;
                if constexpr (std::is_same_v<S, geometry::BezierSpan>) {
                    return geometry::distance(sp.p0, sp.p1) + geometry::distance(sp.p1, sp.p2) +
                           geometry::distance(sp.p2, sp.p3);
                } else {
                    return geometry::length(geometry::Span{sp});
                }
            },
            span);
    }
    return total;
}

double workVertices(const Contour& c, double chordal) {
    double n = 0.0;
    for (const Span& span : c.spans) {
        n += spanVertices(span, chordal);
    }
    return n;
}

/// A bounding box of one segment, for counting overlapping pairs.
struct SegBox {
    double x0;
    double x1;
    double y0;
    double y1;
};

SegBox segBox(Vec2 a, Vec2 b, double grow) {
    return {std::min(a.x, b.x) - grow, std::max(a.x, b.x) + grow, std::min(a.y, b.y) - grow, std::max(a.y, b.y) + grow};
}

/// Number of overlapping pairs among `boxes` (a sweep over x). Stops at `kPairCap` pairs or
/// `kScanCap` sweep steps, which cost more than any limit; `capped` is then set.
double overlapPairs(std::vector<SegBox>& boxes, double pairCap, bool& capped) {
    const double kPairCap = pairCap;
    constexpr double kScanCap = 4.0e8;
    std::sort(boxes.begin(), boxes.end(), [](const SegBox& a, const SegBox& b) { return a.x0 < b.x0; });
    double pairs = 0.0;
    double scans = 0.0;
    std::vector<std::size_t> active;
    for (std::size_t i = 0; i < boxes.size() && pairs < kPairCap && scans < kScanCap; ++i) {
        std::size_t kept = 0;
        for (std::size_t k : active) {
            if (boxes[k].x1 >= boxes[i].x0) {
                active[kept++] = k;
                if (boxes[k].y0 <= boxes[i].y1 && boxes[i].y0 <= boxes[k].y1) {
                    pairs += 1.0;
                }
            }
        }
        scans += static_cast<double>(active.size()) + 1.0;
        active.resize(kept);
        active.push_back(i);
    }
    capped = pairs >= kPairCap || scans >= kScanCap;
    return pairs;
}

/// Work units of the exact offset of `c`: it works span by span (arcs stay arcs), a Bezier by its
/// polygonised pieces. About 10 us (350 units) per span, growing with the contour's span count.
void chargeOffset(const Contour& c, double chordal, double distance, bool inwards, std::size_t index) {
    double spans = 0.0;
    double bezierUnits = 0.0;
    std::vector<SegBox> boxes;
    for (const Span& sp : c.spans) {
        const geometry::Box2 box = geometry::bounds(sp);
        boxes.push_back(segBox(box.min, box.max, distance));
        if (const auto* bz = std::get_if<geometry::BezierSpan>(&sp)) {
            // Polygon pieces of a curve cost about 14 units each (a 200-piece spike curve offsets in
            // 94 us); only the span itself counts towards the contour's superlinear term.
            spans += 1.0;
            bezierUnits += 14.0 * spanVertices(sp, chordal);
            // The exact length the offset computes: pieces ~ (control polygon excess) / tolerance, so
            // time ~ sqrt(excess / polygon). Measured on 40 random cubics: 10 us at 1e-6, 80 us at
            // 1e-4, 800 us at 1e-2, 4 to 8 ms for loops and quarter circles (x ~ 0.2 to 0.7).
            const double poly = geometry::distance(bz->p0, bz->p1) + geometry::distance(bz->p1, bz->p2) +
                                geometry::distance(bz->p2, bz->p3);
            const double excess = poly - geometry::distance(bz->p0, bz->p3);
            const double x = poly > 0.0 && excess > 0.0 ? excess / poly : 0.0;
            bezierUnits += 200.0 + 5.0e5 * std::sqrt(x);
        } else {
            spans += 1.0;
        }
    }
    // Measured on closed polygons (Release): 11 us a span at 1,000 spans, 23 us at 64,000, 53 us at
    // 250,000: about 10 us + 0.16 ns per span of the contour, a span (T-0186 round 2).
    double total = spans * (350.0 + spans / 188.0) + bezierUnits;
    // Spans close together relative to the offset distance crowd the point search: a polygon of
    // 100,000 chords of 1.3 micrometres (R20, offset 1.5) took 27 s against 2 s for the same count of 63 mm
    // chords (R1000, offset 3): 8 units (0.26 us) for each pair of span boxes that overlap once
    // grown by the distance. A sweep that finds more than 7.5e7 pairs refuses the vector.
    bool capped = false;
    total += 8.0 * overlapPairs(boxes, 7.5e7, capped);
    // Sharp junctions (turning by more than 60 degrees): a comb of thin teeth offset inwards cost
    // 6.5 s at 20,000 teeth, 41 s at 40,000 and 122 s at 60,000, i.e. n^2.7 in the junction count
    // S = 2 x teeth; 5 x S^1.7 units reproduces those within 1.5x. Outwards the same comb costs
    // 2.5 s at 20,000 teeth: 1.5 x S^1.7.
    double sharp = 0.0;
    const std::size_t n = c.spans.size();
    for (std::size_t i = 0; i < n; ++i) {
        if (i + 1 == n && !c.closed) {
            break;
        }
        const geometry::Vec2 a = geometry::tangentAt(c.spans[i], 1.0);
        const geometry::Vec2 b = geometry::tangentAt(c.spans[(i + 1) % n], 0.0);
        if (a.dot(b) < 0.5) {
            sharp += 1.0;
        }
    }
    total += (inwards ? 5.0 : 1.5) * std::pow(sharp, 1.7);
    if (capped || !(total <= kMaxSingleCallUnits)) {
        throw TooComplexForOneStep{index, capped ? 1e18 : total};
    }
    detail::chargeWork(detail::unitsOf(total));
}

/// Charges the union and exact offset of the region of `members` (T-0186 round 2). Measured
/// (Release, Clipper2 through geometry::unionContours + offsetContours): about 10 us per polygon
/// vertex, plus a cost that grows faster than linearly with the number P of overlapping
/// pairs of polygon segment boxes (crossings and near-coincident edges): 2.1e-8 s * P^1.35, plus
/// 1.5 us a pair. Concentric star rings (500 spikes x 3..16) took 0.06 to 5 s for P = 6e4 to 1.6e6,
/// circles in a grid and nested circles 0.05 to 0.9 s; the model is within a factor of 3 of all of them.
void chargeRegion(const std::vector<Contour>& members, const Tolerance& tol, std::size_t index) {
    std::vector<SegBox> segs;
    for (const Contour& m : members) {
        const std::vector<Vec2> pts = m.polygonise(tol).points;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            segs.push_back(segBox(pts[i], pts[(i + 1) % pts.size()], 0.0));
        }
    }
    bool capped = false;
    const double pairs = overlapPairs(segs, 1.2e7, capped);
    const double total = 333.0 * static_cast<double>(segs.size()) + 0.7 * std::pow(pairs, 1.35) + 50.0 * pairs;
    if (capped || !(total <= kMaxSingleCallUnits)) {
        throw TooComplexForOneStep{index, capped ? 1e18 : total};
    }
    detail::chargeWork(detail::unitsOf(total));
}

/// Charges `factor` work units per vertex of `c` (a region call over it).
void chargeVertices(const Contour& c, double chordal, double factor) {
    detail::chargeWork(detail::unitsOf(factor * workVertices(c, chordal)));
}

class ProfileBuilder {
   public:
    ProfileBuilder(const std::vector<Contour>& contours, const tooling::Tool& tool, const ProfileParams& params,
                   double moveLimit)
        : raw_(contours), tool_(tool), p_(params), tol_(params.tolerance), moveLimit_(moveLimit) {}

    ProfileResult run();

   private:
    void diag(ProfileIssue issue, std::optional<std::size_t> contour, std::string message) {
        result_.diagnostics.push_back({issue, contour, std::move(message), std::nullopt});
    }
    bool validate();
    bool withinLimits();
    void refuseWork(std::string message) { diag(ProfileIssue::WorkLimitExceeded, std::nullopt, std::move(message)); }
    /// Counts the moves made since the last call (work units) and stops the run past the move limit.
    void chargeMoves();
    bool allowanceWithinLimit();
    bool allowanceOnWithinLimit(double roughAllowance);
    void prepareInputs();
    std::vector<Cut> buildCuts(double extra, bool finishing);
    void orient(Cut& c, bool finishing) const;
    std::vector<Cut> trim(std::vector<Cut> cuts);
    void chooseStarts(std::vector<Cut>& cuts);
    void reportVanished(const char* pass);
    std::optional<TipEdges> tipEdges(const Cut& c, Vec2 target) const;
    double startOn(const Cut& c, const ArcLengthPath& walk, Vec2 target) const;
    std::vector<std::size_t> order(const std::vector<Cut>& cuts) const;
    void placeTabsOn(std::vector<Cut>& primary, const std::vector<Cut>& otherPass, bool asT0127 = false,
                     bool refuseAtGap = false);
    /// What matchRoughingTabs did beyond T-0127's matching.
    struct MatchExtras {
        bool addedBack{false};  // add-back brought a tab back
        bool retried{false};    // a tab was matched only with the cover placed from the vector out
    };
    MatchExtras matchRoughingTabs(std::vector<Cut>& finish, std::vector<Cut>& rough, bool addBack, bool retry);
    void dropSweptTabs(std::vector<Cut>& primary, std::vector<Cut>& secondary);
    void reportLoosePieces(const std::vector<Cut>& primary, const std::vector<Cut>& secondary);
    bool mapTab(const Cut& from, const TabSpan& t, const Cut& onto, TabSpan& out, bool& near,
                bool coverOnly = false) const;
    std::vector<TabSection> tabSections(const Cut& c, const TabSpan& t, bool coverOnly = false, double kerf = -1.0) const;
    bool tabHolds(const Cut& c, const TabSpan& t, const std::vector<Vec2>& tools,
                  const std::vector<ToolDisc>& toolsX) const;
    /// What becomes of a tab: it holds, the tools cut it through, or it would hold nothing (the
    /// cut leaves fewer than two pieces of material near it).
    enum class TabFate { Holds, Severed, Empty };
    /// Sided tabs (sidedTabs_): what becomes of the tab whose raised stretches (every pass) stand at
    /// the tool positions `own`, with every other tool position (open ones and the other live tabs')
    /// in `tools` (every one within `margin` + the tab-layer radius + 0.1 mm of `own`), judged on a
    /// raster reaching `margin` past `own`. `limited` is set when the tab fails but may only seem to:
    /// two groups of the pieces near it, unjoined by it, reach the raster's frame (they may join
    /// beyond it).
    TabFate judgeSidedTab(const std::vector<Vec2>& own, const std::vector<Vec2>& tools,
                          const std::vector<ToolDisc>& ownX, const std::vector<ToolDisc>& toolsX, double spacing,
                          const std::vector<Vec2>& ends, double margin, bool& limited, bool lenient = false) const;
    enum class PieceVerdict { NoHold, Hold, Partial, Severed };
    PieceVerdict sidedPieces(const std::vector<Vec2>& own, const std::vector<Vec2>& tools,
                             const std::vector<ToolDisc>& ownX, const std::vector<ToolDisc>& toolsX,
                             const geometry::Box2& box, double h, double cutRadius, const std::vector<char>& nearOwn,
                             bool& limited) const;
    /// Sided tabs' cross-sections (which place the roughing cover) span the whole kerf: how far the
    /// kerf of the primary cut (and of its roughing cut, with a separate last pass) reaches from
    /// the cut towards the vector side, and towards the other side.
    double kerfVectorSide() const;
    double kerfOtherSide() const;
    /// Radius the tool cuts just below the tab top: a V-bit's cone there (tip at the bottom of the
    /// cut, tab thickness below), at most the radius at the bottom; any other tool's radius.
    double tabLayerRadius() const {
        if (!vbit_) return radius_;
        const double tanHalf = std::tan(0.5 * tool_.includedAngle);
        return tanHalf > 0.0 && std::isfinite(tanHalf) ? std::min(radius_, p_.tabs.thickness * tanHalf) : radius_;
    }
    /// How far round a tab's core tabHolds rasterises: across the widest kerf from the vector side
    /// (the vector side is at most c.distance <= the radius + |allowance| away) to 0.7 mm past it,
    /// and a tool diameter further so a way round the tools can be found.
    double rasterReach() const {
        // (T-0167: the kerf at the tab layer is the tab-layer radius wide; the vector side is still a
        // tool radius + allowance from the cut.)
        const double rt = tabLayerRadius();
        return std::fabs(p_.allowance) + radius_ + rt + (p_.separateLastPass ? p_.lastPassAllowance : 0.0) + 2.0 * rt +
               0.7;
    }
    bool spanFits(const Cut& c, const TabSpan& t, const std::vector<double>& keepClear) const;
    std::vector<double> otherStarts(const Cut& c, const std::vector<Cut>& otherPass,
                                    const BoxIndex& starts) const;
    void reportTabs();
    void collectTabMaterial(const std::vector<Cut>& cuts);
    double leadFloor(const Contour& lead) const;
    std::vector<Vec2> tabMaterial_;  // sample points of every tab's material (all cuts)
    std::map<std::size_t, std::size_t> tabsRequested_;  // per vector, over its primary cuts
    // For each primary (finishing, or only) cut and tab index: the matching spans on the other
    // pass's cuts (cut index, span), so a tab and its roughing cover are kept or dropped together.
    std::vector<std::vector<std::vector<std::pair<std::size_t, TabSpan>>>> tabLinks_;
    std::map<std::size_t, std::size_t> tabsDropped_;    // all left out, per vector
    std::map<std::size_t, std::size_t> tabsUnmatched_;  // of those: no match on the roughing cut
    std::map<std::size_t, std::size_t> tabsSevered_;    // of those: a cut would cut them through
    std::map<std::size_t, std::size_t> tabsEmpty_;      // of those: the cut leaves nothing for them to hold
    std::map<std::size_t, std::size_t> tabsOnCorner_;
    void emit(const Cut& cut, const std::vector<double>& levels, bool finishing);
    void emitLeg(const Cut& cut, const PathLeg& leg, bool lastLevel, std::vector<bool>& cornerDone);
    template <class Make>
    bool leadFits(Make make, double scale, Contour& out, bool& gentler) const;
    template <class Make>
    Contour fitLead(Make make, std::size_t contour, const char* what);
    void planLeadIns(std::vector<Cut>& cuts);
    std::vector<Move> cornerMoves(const Cut& cut, Vec2 q, double turn, double bottom) const;
    struct LegStep {
        PathLeg leg;
        bool last;  // on the last level (where the corner moves are made)
    };
    /// `endU` (if given) is where the cut ends, the overcut included (the lead-out leaves from it).
    std::vector<LegStep> legsOf(const Cut& cut, const std::vector<double>& levels, bool finishing,
                                double* endU = nullptr) const;
    void planExcursions(std::vector<Cut>& cuts, const std::vector<double>& levels, bool finishing) const;
    /// True when the corner moves of `e` are made with the kept tabs of `cut`: its corner lies
    /// outside every tab's footprint (raised stretch and the 3D ramps beside it).
    bool excursionActive(const Cut& cut, const Excursion& e) const;
    std::vector<Taper> tapersOf(Vec2 q, double bottom, const std::vector<Move>& moves) const;
    double coneClearance(Vec2 p, double tipZ) const;
    /// Margin and resolution of the safety checks (trim, leads): 5 % of the distance tolerance.
    double guardTol() const { return 0.05 * tol_.distance; }
    std::vector<LeadOptions> leadVariants() const;

    const std::vector<Contour>& raw_;
    const tooling::Tool& tool_;
    ProfileParams p_;
    Tolerance tol_;
    Tolerance offTol_;
    tooling::CuttingData cd_;
    PassPlan passes_;
    double radius_{0.0};
    bool vbit_{false};
    bool sharpExternal_{false};
    bool sharpInternal_{false};
    double vbitTan_{1.0};  // tan(half included angle) of a V-bit
    // Tabs are judged by the sides of the cut path (On the vector, or a negative allowance, where
    // the kerf crosses the vector); otherwise by the distance to the vectors (ADR-0016 §14).
    bool sidedTabs_{false};
    Input in_;
    ClearanceField allField_;  // every valid vector, by input index
    BoxIndex inputIndex_;      // boxes of the valid vectors, by input index
    // Rounding allowance for the T-0131 index queries: a candidate box is grown by this beyond the
    // distance it is tested at (well above the rounding of distances at the job's coordinates).
    double indexSlack_{1e-9};
    ProfileResult result_;
    std::vector<bool> tooLargeReported_;
    std::vector<bool> lostCut_;  // vectors that had a cut removed as a sliver or a point (T-0165)
    std::vector<bool> keptCut_;  // vectors that still have a cut after the starts are chosen
    std::size_t chargedMoves_{0};  // moves already counted by chargeMoves
    double moveLimit_;             // estimated and emitted moves are refused past this
};

bool ProfileBuilder::validate() {
    if (tool_.type == tooling::ToolType::DiamondDrag) {
        // T-0154 (owner decision 2026-11-03): a drag tool must not turn; only Quick Engrave runs
        // toolpaths with the spindle off.
        diag(ProfileIssue::InvalidTool, std::nullopt,
             "tool '" + tool_.name +
                 "' is a diamond drag tool, which must not turn; use Quick Engrave for diamond-drag tools");
        return false;
    }
    cd_ = effectiveCuttingData(tool_, p_.cuttingData);
    passes_ = p_.passes;
    if (!(passes_.passDepth > 0.0)) {
        passes_.passDepth = cd_.passDepth;
    }
    radius_ = profileToolRadius(tool_, passes_);
    const double vals[] = {p_.allowance,       p_.safeZ,           passes_.startDepth, passes_.cutDepth,
                           passes_.passDepth,  p_.lastPassAllowance, p_.tabs.length,   p_.tabs.thickness,
                           p_.tabs.distance,   p_.ramp.distance,   p_.ramp.angle,      p_.leads.length,
                           p_.leads.angle,     p_.leads.radius,    p_.leads.sweep,     p_.leads.overcut,
                           p_.corners.mitreLimit, radius_};
    for (double v : vals) {
        if (!std::isfinite(v)) {
            diag(ProfileIssue::InvalidParameters, std::nullopt, "a parameter is not a finite number");
            return false;
        }
    }
    // T-0141: the same tolerance bounds as Pocket (ADR-0016 §18). The offsets and the dust
    // filter's width budget (ADR-0006 §8) assume chordal >= 1e-4 mm. T-0142: distance >= 1e-6 mm,
    // one unit of the offset grid (ADR-0006 §2): below it the exact offsets cannot close their
    // junctions, and Bezier input was polygonised at distance / 2 (minutes at 1e-8 mm).
    const Tolerance& t = p_.tolerance;
    if (!std::isfinite(t.chordal) || t.chordal < 1e-4 || t.chordal > 1.0 || !std::isfinite(t.distance) ||
        !(t.distance >= 1e-6) || t.distance > t.chordal) {
        diag(ProfileIssue::InvalidParameters, std::nullopt,
             "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance between 0.000001 mm and chordal");
        return false;
    }
    if (!(radius_ > 0.0)) {
        diag(ProfileIssue::InvalidParameters, std::nullopt, "the tool has no cutting width at the cut depth");
        return false;
    }
    if (passes_.startDepth < 0.0 || p_.safeZ <= -passes_.startDepth) {
        diag(ProfileIssue::InvalidParameters, std::nullopt, "safe Z must be above the start of the cut");
        return false;
    }
    if (p_.separateLastPass && p_.lastPassAllowance < 0.0) {
        diag(ProfileIssue::InvalidParameters, std::nullopt, "last pass allowance must not be negative");
        return false;
    }
    if (!allowanceWithinLimit()) {
        return false;
    }
    if (!withinLimits()) {
        return false;
    }
    if (passes_.levels().empty()) {
        diag(ProfileIssue::NoPasses, std::nullopt, "the cut depth yields no passes");
        return false;
    }
    if (p_.tabs.enabled && p_.tabs.thickness > passes_.cutDepth) {
        diag(ProfileIssue::TabThicknessClamped, std::nullopt,
             "tab thickness " + std::to_string(p_.tabs.thickness) + " mm is more than the cut depth; clamped to " +
                 std::to_string(passes_.cutDepth) + " mm");
        p_.tabs.thickness = passes_.cutDepth;
    }
    sidedTabs_ = p_.side == ProfileSide::On || p_.allowance < 0.0;
    vbit_ = tool_.type == tooling::ToolType::VBit;
    if ((p_.corners.sharpExternal || p_.corners.sharpInternal) && !vbit_) {
        diag(ProfileIssue::CornerNeedsVBit, std::nullopt, "sharp corner options need a V-bit; ignored");
    }
    sharpExternal_ = vbit_ && p_.corners.sharpExternal;
    sharpInternal_ = vbit_ && p_.corners.sharpInternal;
    if (sharpInternal_) {
        vbitTan_ = std::tan(0.5 * tool_.includedAngle);
        if (!(vbitTan_ > 0.0) || !std::isfinite(vbitTan_)) {
            diag(ProfileIssue::InvalidParameters, std::nullopt, "the V-bit has no valid included angle");
            return false;
        }
    }
    return true;
}

// T-0129 (owner decision 2026-11-01): an allowance below minus the tool radius puts the tool
// centre across the vector, so Outside would cut like Inside (and the reverse) and On, Left and
// Right would swap sides. Each pass's distance (the allowance; for the roughing of a separate last
// pass, the allowance + the last-pass allowance) must be at least -r, compared exactly: in IEEE
// arithmetic r + a == 0 exactly when a == -r and r + a < 0 exactly when a < -r, so this is the
// same boundary buildCuts keys the side on (s == 0, T-0127). Exactly -r stays valid. When either
// pass is refused nothing is cut, roughing included (ADR-0016 §4).
bool ProfileBuilder::allowanceWithinLimit() {
    const double limit = -radius_;
    // validate() has refused a negative last-pass allowance, and rounded addition is monotone, so
    // the roughing allowance is never below the finishing one: only the allowance itself decides.
    const double roughAllowance = p_.separateLastPass ? p_.allowance + p_.lastPassAllowance : p_.allowance;
    if (!(p_.allowance < limit) && !(roughAllowance < limit)) {
        return allowanceOnWithinLimit(roughAllowance);
    }
    const std::string limitText = "minus the tool radius (" + mmText(limit) + " at the cut depth)";
    std::string msg;
    if (!p_.separateLastPass) {
        msg = "allowance " + mmText(p_.allowance) + " is below " + limitText +
              ": the tool would cross the vector to its other side; nothing is cut";
    } else if (roughAllowance < limit) {
        msg = "allowance " + mmText(p_.allowance) + " and allowance + last pass allowance " + mmText(roughAllowance) +
              " are below " + limitText + ": the tool would cross the vector to its other side; nothing is cut";
    } else {
        msg = "allowance " + mmText(p_.allowance) + " is below " + limitText +
              ": the last pass would cross the vector to its other side; nothing is cut (the roughing at allowance"
              " + last pass allowance " +
              mmText(roughAllowance) + " is not cut on its own)";
    }
    diag(ProfileIssue::InvalidParameters, std::nullopt, std::move(msg));
    return false;
}

// T-0137 (owner decision 2026-11-02): On the vector the allowance alone is the tool centre's
// distance from it, so above +r the tool runs entirely off the vector just as below -r. Each
// pass's distance (allowance; allowance + last-pass allowance for the roughing, which can be the
// only one above the limit) must be at most +r, compared exactly: buildCuts offsets by that same
// double, so exactly +r (the tool's edge on the vector) stays valid and one ulp above is refused.
// Inside/Outside, and open Left/Right (their aliases), keep only the -r limit: a large positive
// allowance there is a normal oversize cut on the profile side (ADR-0016 §4).
bool ProfileBuilder::allowanceOnWithinLimit(double roughAllowance) {
    if (p_.side != ProfileSide::On) {
        return true;
    }
    const double limit = radius_;
    const bool finishAbove = p_.allowance > limit;
    const bool roughAbove = roughAllowance > limit;
    if (!finishAbove && !roughAbove) {
        return true;
    }
    const std::string limitText = "the tool radius (" + mmText(limit) + " at the cut depth)";
    std::string msg;
    if (!p_.separateLastPass) {
        msg = "allowance " + mmText(p_.allowance) + " is above " + limitText +
              ": the tool would run entirely off the vector; nothing is cut";
    } else if (finishAbove) {
        msg = "allowance " + mmText(p_.allowance) + " and allowance + last pass allowance " + mmText(roughAllowance) +
              " are above " + limitText + ": the tool would run entirely off the vector; nothing is cut";
    } else {
        msg = "allowance + last pass allowance " + mmText(roughAllowance) + " is above " + limitText +
              ": the roughing would run entirely off the vector; nothing is cut (the last pass at allowance " +
              mmText(p_.allowance) + " is not cut on its own)";
    }
    diag(ProfileIssue::InvalidParameters, std::nullopt, std::move(msg));
    return false;
}

// T-0186 (ADR-0016 section 19): what the profile refuses up front, before any allocation that
// grows with the parameters. Everything here is a cheap closed-form count over the input; the work
// meter (WorkMeter) covers what only the run can tell. Every refusal is `WorkLimitExceeded` and
// nothing is cut. A value that is not finite is not judged here (validate() and the importers'
// own checks deal with it).
bool ProfileBuilder::withinLimits() {
    auto mm = [](double v) { return mmText(v); };
    auto count = [](double v) { return v < 9e18 ? std::to_string(static_cast<long long>(v)) : std::string("more than 9e18"); };
    // Parameter ranges: the distances a job can mean are well inside +-kProfileMaxOffsetMm.
    struct Named {
        const char* name;
        double value;
        bool used;
    };
    const bool leads = p_.leads.type != LeadType::None;
    const Named ranges[] = {
        {"allowance", p_.allowance, true},
        {"last pass allowance", p_.lastPassAllowance, p_.separateLastPass},
        {"lead length", p_.leads.length, p_.leads.type == LeadType::Straight},
        {"lead radius", p_.leads.radius, p_.leads.type == LeadType::Circular},
        {"overcut", p_.leads.overcut, leads},
        {"ramp distance", p_.ramp.distance, p_.ramp.type != RampType::None},
        {"tab length", p_.tabs.length, p_.tabs.enabled},
    };
    for (const Named& r : ranges) {
        if (r.used && std::isfinite(r.value) && std::fabs(r.value) > kProfileMaxOffsetMm) {
            refuseWork(std::string(r.name) + " " + mm(r.value) + " is beyond the limit of " + mm(kProfileMaxOffsetMm) +
                       "; nothing is cut");
            return false;
        }
    }
    // Depth passes. A closed form first; PassPlan's own count decides only near the limit.
    double levels = 1.0;
    if (passes_.cutDepth > 0.0 && passes_.passDepth > 0.0) {
        const double estimate = std::ceil(passes_.cutDepth / passes_.passDepth) + 2.0;
        levels = estimate;
        if (!(estimate <= static_cast<double>(kProfileMaxLevels))) {
            levels = estimate <= 4.0 * static_cast<double>(kProfileMaxLevels)
                         ? static_cast<double>(passes_.levels().size())
                         : std::numeric_limits<double>::infinity();
        }
        if (!(levels <= static_cast<double>(kProfileMaxLevels))) {
            refuseWork("more than " + std::to_string(kProfileMaxLevels) + " passes (cut depth " + mm(passes_.cutDepth) +
                       " in steps of " + mm(passes_.passDepth) + "); nothing is cut");
            return false;
        }
    }
    // The input: spans, coordinates and total length.
    std::size_t spans = 0;
    double totalLength = 0.0;
    std::vector<double> lengths(raw_.size(), 0.0);
    const auto outside = [](double v) { return std::fabs(v) > kProfileMaxCoordinateMm; };
    for (std::size_t i = 0; i < raw_.size(); ++i) {
        for (const Span& sp : raw_[i].spans) {
            const bool beyond = std::visit(
                [&](const auto& x) {
                    using S = std::decay_t<decltype(x)>;
                    if constexpr (std::is_same_v<S, geometry::LineSpan>) {
                        return outside(x.start.x) || outside(x.start.y) || outside(x.end.x) || outside(x.end.y);
                    } else if constexpr (std::is_same_v<S, geometry::ArcSpan>) {
                        // The end points, not the centre: a nearly straight arc has a far-away centre.
                        return std::fabs(x.radius) > kProfileMaxRadiusMm ||
                               (std::isfinite(x.startAngle) && std::fabs(x.startAngle) >= 1e6) ||
                               (std::isfinite(x.endAngle) && std::fabs(x.endAngle) >= 1e6) ||
                               (std::fabs(x.radius) <= kProfileMaxRadiusMm &&
                                (outside(x.startPoint().x) || outside(x.startPoint().y) || outside(x.endPoint().x) ||
                                 outside(x.endPoint().y)));
                    } else {
                        // A non-finite control point is refused too: the Bezier polygonisation and the
                        // Clipper2 conversion behind it are undefined for it (found by the T-0186 verifier).
                        const auto bad = [&](double v) { return !std::isfinite(v) || outside(v); };
                        return bad(x.p0.x) || bad(x.p0.y) || bad(x.p1.x) || bad(x.p1.y) || bad(x.p2.x) ||
                               bad(x.p2.y) || bad(x.p3.x) || bad(x.p3.y);
                    }
                },
                sp);
            if (beyond) {
                refuseWork("vector #" + std::to_string(i + 1) + " has a non-finite coordinate (a Bezier) or one beyond " + mm(kProfileMaxCoordinateMm) +
                           " (or an arc with a radius beyond " + mm(kProfileMaxRadiusMm) +
                           " or an angle beyond a million radians); nothing is cut");
                return false;
            }
        }
        spans += raw_[i].spans.size();
        if (spans > kProfileMaxSpans) {
            refuseWork("more than " + std::to_string(kProfileMaxSpans) + " spans in the vectors; nothing is cut");
            return false;
        }
        const double len = lengthBound(raw_[i]);
        if (std::isfinite(len)) {
            lengths[i] = len;
            totalLength += len;
        }
    }
    if (!(totalLength <= kProfileMaxLengthMm)) {
        refuseWork("the vectors are " + mm(totalLength) + " long in all; the limit is " + mm(kProfileMaxLengthMm) +
                   "; nothing is cut");
        return false;
    }
    // Tabs asked for, over all vectors.
    double tabs = 0.0;
    if (p_.tabs.enabled) {
        for (std::size_t i = 0; i < raw_.size(); ++i) {
            if (raw_[i].isEmpty()) {
                continue;
            }
            if (p_.tabs.spacing == TabSpacing::ConstantNumber) {
                tabs += static_cast<double>(std::max(p_.tabs.count, 0));
            } else if (p_.tabs.distance > 0.0) {
                tabs += std::max(1.0, lengths[i] / p_.tabs.distance);
            }
        }
        if (!(tabs <= kProfileMaxTabs)) {
            refuseWork("more than " + std::to_string(static_cast<long long>(kProfileMaxTabs)) +
                       " tabs would be placed (tab count or spacing); nothing is cut");
            return false;
        }
        // Work meter (T-0186 round 4): placing, matching and cutting a tab costs about 16 us (500
        // units), whatever the tab count (it was quadratic before the sample covering and the tab
        // ceiling were made linear: 128 s at 24,000 mm of square with a tab every 3 mm).
        detail::chargeWork(detail::unitsOf(500.0 * tabs));
    }
    // Moves: a closed-form estimate that follows the actual count closely for ordinary jobs and
    // is far below it only for curve-heavy ones (those are stopped as they are emitted).
    const double contours = static_cast<double>(raw_.size());
    double perLevel = 2.0 * static_cast<double>(spans);
    if (p_.tabs.enabled) {
        const double step = passes_.passDepth > 0.0 ? passes_.passDepth : passes_.cutDepth;
        const double tabLevels = step > 0.0 ? std::min(levels, std::floor(p_.tabs.thickness / step) + 1.0) : 1.0;
        perLevel += tabs * 4.0 * std::max(tabLevels, 1.0) / std::max(levels, 1.0);
    }
    if (p_.ramp.type == RampType::ZigZag && p_.ramp.useAngle && p_.ramp.distance > 0.0 && p_.ramp.angle > 0.0) {
        const double slope = p_.ramp.distance * std::tan(p_.ramp.angle);
        const double step = passes_.passDepth > 0.0 ? passes_.passDepth : passes_.cutDepth;
        if (slope > 0.0 && std::isfinite(slope) && step > 0.0) {
            perLevel += contours * 2.0 * std::ceil(step / slope);
        }
    }
    const double moves = levels * perLevel;
    if (!(moves <= moveLimit_)) {
        refuseWork("the toolpath would have about " + count(moves) + " moves; the limit is " +
                   std::to_string(static_cast<long long>(moveLimit_)) +
                   " (too many passes, vectors, tabs or ramp legs); nothing is cut");
        return false;
    }
    return true;
}

void ProfileBuilder::chargeMoves() {
    const std::size_t n = result_.toolpath.moves.size();
    if (n > chargedMoves_) {
        detail::chargeWork(n - chargedMoves_);
        chargedMoves_ = n;
    }
    if (static_cast<double>(n) > moveLimit_) {
        throw TooManyMoves{};
    }
}

void ProfileBuilder::prepareInputs() {
    const std::size_t n = raw_.size();
    in_.contours = raw_;
    {
        // Work meter (T-0186): validity, area, nesting, clearance field. Beziers are measured at the
        // tightened offset tolerance they are polygonised at, 20 units a vertex (their length and
        // area integrate at that resolution); lines and arcs 4.
        const double chordal = hasBezier(raw_) ? std::min(tol_.chordal, 0.5 * tol_.distance) : tol_.chordal;
        for (const Contour& c : raw_) {
            chargeVertices(c, chordal, hasBezier({c}) ? 20.0 : 4.0);
        }
    }
    in_.valid.assign(n, false);
    in_.area.assign(n, 0.0);
    tooLargeReported_.assign(n, false);
    lostCut_.assign(n, false);
    keptCut_.assign(n, false);
    std::vector<bool> closedValid(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        const Contour& c = raw_[i];
        if (c.isEmpty()) {
            diag(ProfileIssue::EmptyContour, i, vecName(i) + " has no spans; skipped");
            continue;
        }
        if (!c.isContinuous(tol_)) {
            diag(ProfileIssue::DiscontinuousContour, i, vecName(i) + " has gaps between spans; skipped");
            continue;
        }
        in_.valid[i] = true;
        closedValid[i] = c.closed;
        if (c.closed) {
            in_.area[i] = geometry::area(c);
        }
    }
    in_.depth = nestingDepths(in_.contours, closedValid, tol_);
    for (std::size_t i = 0; i < n; ++i) {
        if (closedValid[i]) {
            geometry::setOrientation(in_.contours[i], in_.depth[i] % 2 == 0 ? Orientation::CCW : Orientation::CW, tol_);
        }
    }
    offTol_ = tol_;
    if (hasBezier(raw_)) {
        // Bezier spans are polygonised before offsetting; keep the chords within half the
        // distance tolerance of the curve so the offset stays within tolerance of r (at least
        // 5e-7 mm: tol.distance >= 1e-6 mm since T-0142).
        offTol_.chordal = std::min(tol_.chordal, 0.5 * tol_.distance);
    }
    // Every valid vector, indexed as the input (invalid ones excluded), and their boxes.
    std::vector<std::size_t> invalid;
    std::vector<geometry::Box2> boxes(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (in_.valid[i]) {
            boxes[i] = in_.contours[i].bounds();
        } else {
            invalid.push_back(i);
        }
    }
    allField_ = ClearanceField(in_.contours).excluding(std::move(invalid));
    inputIndex_ = BoxIndex(std::move(boxes));
    const geometry::Box2& all = inputIndex_.bounds();
    const double scale = all.isEmpty() ? 0.0
                                       : std::max({std::fabs(all.min.x), std::fabs(all.min.y), std::fabs(all.max.x),
                                                   std::fabs(all.max.y)});
    indexSlack_ = std::max(tol_.distance, 1e-9) + (std::isfinite(scale) ? 1e-9 * scale : 0.0);
}

// Signed outward distance for a pass with `extra` added (last-pass allowance for roughing).
std::vector<Cut> ProfileBuilder::buildCuts(double extra, bool finishing) {
    std::vector<Cut> cuts;
    const std::size_t n = in_.contours.size();
    double s = 0.0;
    switch (p_.side) {
        case ProfileSide::Outside:
            s = radius_ + p_.allowance + extra;
            break;
        case ProfileSide::Inside:
            s = -(radius_ + p_.allowance + extra);
            break;
        case ProfileSide::On:
            s = p_.allowance + extra;
            break;
    }
    const double d = std::fabs(s);
    const geometry::OffsetSide side = s >= 0.0 ? geometry::OffsetSide::Right : geometry::OffsetSide::Left;
    const geometry::JoinType join = sharpExternal_ ? geometry::JoinType::Sharp : geometry::JoinType::Round;
    const bool guard = p_.side != ProfileSide::On && d > 0.0;
    // The vector lies left of the cut when the cut runs right of it. With no offset at all (an
    // allowance of exactly minus the radius) the profile side decides, so Inside keeps its own
    // direction and part side rather than turning into Outside at r + allowance = 0 (T-0127).
    const bool vectorLeft = s > 0.0 || (s == 0.0 && p_.side != ProfileSide::Inside);

    std::vector<std::vector<Contour>> chains(n);
    std::vector<bool> selfBad(n, false);
    for (std::size_t i = 0; i < n; ++i) {
        if (!in_.valid[i]) {
            continue;
        }
        const Contour& c = in_.contours[i];
        if (d == 0.0) {
            chains[i] = {ArcLengthPath(c, offTol_).contour()};
        } else {
            chargeOffset(c, offTol_.chordal, d, side == geometry::OffsetSide::Left, i);  // work meter (T-0186)
            chains[i] = geometry::offsetOpenContour(c, d, side, join, offTol_, p_.corners.mitreLimit);
            detail::pollCancel();  // T-0228: a cancelled call returned early (empty)
        }
        for (Contour& ch : chains[i]) {
            if (!ch.closed && !ch.isEmpty() && geometry::distance(ch.startPoint(), ch.endPoint()) <= tol_.distance) {
                ch.closed = true;
            }
            if (c.closed && !ch.closed) {
                selfBad[i] = true;
            }
        }
        if (chains[i].empty() && !tooLargeReported_[i]) {
            tooLargeReported_[i] = true;
            diag(ProfileIssue::ToolTooLarge, i,
                 vecName(i) + ": no room for the tool (offset by " + std::to_string(d) + " mm vanishes)");
        }
    }

    // Closed vectors whose offsets come too close to another closed vector are offset together.
    DisjointSets sets(n);
    std::vector<bool> merge(n, false);
    if (guard) {
        for (std::size_t i = 0; i < n; ++i) {
            if (!in_.valid[i] || !in_.contours[i].closed) {
                continue;
            }
            if (selfBad[i]) {
                merge[i] = true;
            }
            for (const Contour& ch : chains[i]) {
                const ArcLengthPath walk(ch, offTol_);
                const geometry::Box2 cb = ch.bounds().inflated(d);
                for (std::size_t j : inputIndex_.query(cb)) {
                    if (j == i || !in_.valid[j] || !in_.contours[j].closed || sets.find(i) == sets.find(j)) {
                        continue;
                    }
                    const ClearanceField fj({in_.contours[j]});
                    if (!isClear(walk, fj, d - tol_.distance, 0.5 * tol_.distance)) {
                        sets.unite(i, j);
                        merge[i] = merge[j] = true;
                    }
                }
            }
        }
    }

    for (std::size_t i = 0; i < n; ++i) {
        if (!in_.valid[i] || merge[i]) {
            continue;
        }
        for (Contour& ch : chains[i]) {
            Cut cut;
            cut.path = std::move(ch);
            cut.sources = {i};
            cut.contourOnLeft = vectorLeft;
            cut.fromClosed = in_.contours[i].closed;
            cut.distance = d;
            cut.offsetBy = d;
            cuts.push_back(std::move(cut));
        }
    }

    // Region offsets of merged groups (polygonal). The extra chordal margin keeps the result at
    // least d from the true curves, which bulge up to `chordal` beyond their polygonisation.
    std::vector<std::vector<std::size_t>> groups(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (merge[i]) {
            groups[sets.find(i)].push_back(i);
        }
    }
    for (std::size_t root = 0; root < n; ++root) {
        const std::vector<std::size_t> group = std::move(groups[root]);
        if (group.empty()) {
            continue;
        }
        // The group's region under the even-odd rule: every edge of every member is a region
        // boundary, also where members cross. Outers CCW, holes CW (region on the left).
        std::vector<Contour> members;
        std::size_t outer = group.front();
        for (std::size_t i : group) {
            members.push_back(in_.contours[i]);
            if (in_.area[i] > in_.area[outer]) {
                outer = i;
            }
        }
        chargeRegion(members, offTol_, group.front());  // work meter (T-0186): union and offset of the region
        const std::vector<Contour> region = geometry::unionContours(members, offTol_, geometry::FillRule::EvenOdd);
        detail::pollCancel();  // T-0228
        // Nesting parity of the group as a whole: how many other closed vectors contain it. When
        // odd, the group sits in a hole and the global region is the complement of its own.
        int base = 0;
        {
            const Vec2 probe = probePoint(in_.contours[outer]);
            for (std::size_t j : inputIndex_.containing(probe)) {
                if (!in_.valid[j] || !in_.contours[j].closed || std::find(group.begin(), group.end(), j) != group.end()) {
                    continue;
                }
                if (geometry::locatePoint(in_.contours[j], probe, geometry::FillRule::EvenOdd, tol_) ==
                    geometry::PointLocation::Inside) {
                    base ^= 1;
                }
            }
        }
        double dd = (s >= 0.0 ? 1.0 : -1.0) * (d + offTol_.chordal);
        if (base == 1) {
            dd = -dd;
        }
        std::vector<Contour> res = geometry::offsetContours(region, dd, join, offTol_, p_.corners.mitreLimit);
        detail::pollCancel();  // T-0228
        for (Contour& r : res) {
            if (base == 1) {
                r.reverse();  // keep the global region on the left
            }
            Cut cut;
            cut.path = std::move(r);
            cut.sources = group;
            cut.contourOnLeft = vectorLeft;
            cut.fromClosed = true;
            cut.exact = false;
            cut.distance = d;
            cut.offsetBy = std::fabs(dd);
            cut.offsetSlack = offTol_.chordal;
            cuts.push_back(std::move(cut));
        }
        for (std::size_t i : group) {
            diag(ProfileIssue::MergedWithNeighbour, i,
                 vecName(i) + (group.size() > 1 ? ": cut merged with nearby vectors (offset as one region, polygonal)"
                                                : ": offset as a region (polygonal); its exact offset was not closed"));
        }
        if (res.empty()) {
            for (std::size_t i : group) {
                if (!tooLargeReported_[i]) {
                    tooLargeReported_[i] = true;
                    diag(ProfileIssue::ToolTooLarge, i, vecName(i) + ": no room for the tool");
                }
            }
        }
    }
    for (Cut& c : cuts) {
        orient(c, finishing);
    }
    if (guard) {
        cuts = trim(std::move(cuts));
    }
    return cuts;
}

void ProfileBuilder::orient(Cut& c, bool finishing) const {
    const bool climb = p_.direction == CutDirection::Climb;
    // Climb keeps the vector on the right of travel.
    bool reverse = c.contourOnLeft == climb;
    if (p_.side == ProfileSide::On && !c.fromClosed) {
        reverse = !climb;  // open vectors cut "on" keep their direction when climbing
    }
    if (finishing && p_.lastPassReverse) {
        reverse = !reverse;
    }
    if (reverse) {
        c.path.reverse();
        c.contourOnLeft = !c.contourOnLeft;
    }
}

std::vector<Cut> ProfileBuilder::trim(std::vector<Cut> cuts) {
    std::vector<Cut> out;
    for (Cut& cut : cuts) {
        // An exact offset is at least its distance from its own vector by construction; a
        // merged region offset is checked against every vector.
        const ClearanceField field = cut.exact ? allField_.excluding(cut.sources) : allField_;
        const ArcLengthPath walk(cut.path, offTol_);
        const double len = walk.length();
        // Strict: trimmed cuts keep within 1.5 x guardTol() of the full distance (merged regions are
        // offset by d + chordal, so this rarely removes anything there).
        const auto bad = findViolations(walk, 0.0, len, field, cut.distance - guardTol(), guardTol());
        if (bad.empty()) {
            out.push_back(std::move(cut));
            continue;
        }
        for (std::size_t s : cut.sources) {
            diag(ProfileIssue::TrimmedNearVector, s,
                 vecName(s) + ": part of the cut was left out because it passes within the tool radius of "
                              "another vector");
        }
        // Keep the complement of the violating stretches.
        std::vector<std::pair<double, double>> keep;
        if (walk.closed()) {
            for (std::size_t k = 0; k < bad.size(); ++k) {
                const double a = bad[k].second;
                const double b = k + 1 < bad.size() ? bad[k + 1].first : bad.front().first + len;
                keep.push_back({a, b});
            }
        } else {
            double a = 0.0;
            for (const auto& v : bad) {
                keep.push_back({a, v.first});
                a = v.second;
            }
            keep.push_back({a, len});
        }
        for (const auto& [a, b] : keep) {
            if (b - a <= tol_.distance) {
                continue;
            }
            Cut piece;
            piece.path.spans = walk.extract(a, b);
            piece.path.closed = false;
            piece.sources = cut.sources;
            piece.contourOnLeft = cut.contourOnLeft;
            piece.fromClosed = cut.fromClosed;
            piece.exact = cut.exact;
            piece.distance = cut.distance;
            piece.offsetBy = cut.offsetBy;
            piece.offsetSlack = cut.offsetSlack;
            if (!piece.path.isEmpty()) {
                out.push_back(std::move(piece));
            }
        }
    }
    return out;
}

// The input edges meeting at `target` when it is a vertex of one of the cut's vectors (to rounding;
// the first such vertex in the order of the cut's sources), for the cut start (T-0158).
std::optional<TipEdges> ProfileBuilder::tipEdges(const Cut& c, Vec2 target) const {
    const double near = 4e-10 * std::max({1.0, std::fabs(target.x), std::fabs(target.y)});
    for (std::size_t src : c.sources) {
        const Contour& v = raw_[src];
        const std::size_t n = v.spans.size();
        for (std::size_t i = 0; i < n; ++i) {
            if (geometry::distance(geometry::startPoint(v.spans[i]), target) > near || (i == 0 && !v.closed)) {
                continue;
            }
            // A Bezier is offset as its polygon (at offTol_): its chord at the vertex.
            auto chordAt = [&](const Span& sp, bool last) -> std::optional<Span> {
                if (!std::holds_alternative<geometry::BezierSpan>(sp)) {
                    return sp;
                }
                std::vector<Vec2> pts{geometry::startPoint(sp)};
                geometry::polygoniseInto(sp, offTol_, pts);
                std::optional<Span> chord;
                for (std::size_t k = 1; k < pts.size(); ++k) {
                    if (!offTol_.equalPoints(pts[k - 1], pts[k])) {
                        chord = geometry::LineSpan{pts[k - 1], pts[k]};
                        if (!last) {
                            break;
                        }
                    }
                }
                return chord;
            };
            const std::optional<Span> in = chordAt(v.spans[i == 0 ? n - 1 : i - 1], true);
            const std::optional<Span> out = chordAt(v.spans[i], false);
            if (!in || !out) {
                return std::nullopt;
            }
            return TipEdges{*in, *out};
        }
    }
    return std::nullopt;
}

double ProfileBuilder::startOn(const Cut& c, const ArcLengthPath& walk, Vec2 target) const {
    return tieFreeStart(walk, target, tol_.distance, c.offsetBy, c.offsetSlack, !sharpExternal_,
                        [&] { return tipEdges(c, target); });
}

void ProfileBuilder::chooseStarts(std::vector<Cut>& cuts) {
    lostCut_.assign(lostCut_.size(), false);
    keptCut_.assign(keptCut_.size(), false);
    // Slivers no longer than the tolerance carry no cut. The vectors they came from are noted, so
    // a vector left with no cut at all is reported (T-0165).
    auto dropLost = [&](auto isLost) {
        cuts.erase(std::remove_if(cuts.begin(), cuts.end(),
                                  [&](const Cut& c) {
                                      if (!isLost(c)) {
                                          return false;
                                      }
                                      for (std::size_t src : c.sources) {
                                          lostCut_[src] = true;
                                      }
                                      return true;
                                  }),
                   cuts.end());
    };
    dropLost([&](const Cut& c) { return c.path.isEmpty() || c.path.length() <= tol_.distance; });
    for (Cut& c : cuts) {
        const ArcLengthPath walk(c.path, offTol_);
        if (walk.closed() && p_.startAt != StartAt::Optimise) {
            Vec2 target = walk.pointAt(0.0);
            if (p_.startAt == StartAt::VectorStart) {
                target = raw_[c.sources.front()].startPoint();
            } else {
                target = anchorOf(c.path.bounds(), p_.startAnchor);
            }
            const double u = startOn(c, walk, target);
            c.path.spans = walk.extract(u, u + walk.length());
        }
        closeExactly(c.path);
        c.walk = ArcLengthPath(c.path, offTol_);
    }
    // A closed cut that collapsed to a point when its start was chosen has no spans left: it never
    // reaches ordering or the tabs' keep-clear points (T-0165).
    dropLost([&](const Cut& c) { return c.walk.contour().isEmpty(); });
    for (const Cut& c : cuts) {
        for (std::size_t src : c.sources) {
            keptCut_[src] = true;
        }
    }
}

// A vector whose cuts in a pass all vanished (offset collapsed to a sliver or a point) is
// reported once, like any other vector with no room for the tool (T-0165). Call after chooseStarts.
void ProfileBuilder::reportVanished(const char* pass) {
    for (std::size_t i = 0; i < lostCut_.size(); ++i) {
        if (lostCut_[i] && !keptCut_[i] && !tooLargeReported_[i]) {
            tooLargeReported_[i] = true;
            diag(ProfileIssue::ToolTooLarge, i, vecName(i) + ": no room for the tool (its offset collapses in the " + pass + " pass)");
        }
    }
}

std::vector<std::size_t> ProfileBuilder::order(const std::vector<Cut>& cuts) const {
    std::vector<OrderItem> items;
    items.reserve(cuts.size());
    for (const Cut& c : cuts) {
        OrderItem it;
        it.path = c.walk.contour();
        it.selectionIndex = *std::min_element(c.sources.begin(), c.sources.end());
        it.start = c.walk.pointAt(0.0);
        it.end = c.walk.closed() ? it.start : c.walk.pointAt(c.walk.length());
        items.push_back(std::move(it));
    }
    return orderCuts(items, p_.order, p_.startPosition, tol_);
}

// Positions on `c` (arc length) of the start points of the other pass's cuts that lie near it:
// where the other pass plunges and its lead arrives, so `c` must not have a tab there either.
// `starts` indexes the other pass's start points (startIndex); only those within reach of c's
// box can be near it, and the index returns them in pass order (T-0131).
std::vector<double> ProfileBuilder::otherStarts(const Cut& c, const std::vector<Cut>& otherPass,
                                                const BoxIndex& starts) const {
    std::vector<double> out;
    // Generous reach: at a corner the other pass's start lies allowance / sin(half angle) away,
    // and an extra keep-clear point only shifts a tab.
    const double reach = radius_ + p_.lastPassAllowance + 2.0 * tol_.distance;
    const geometry::Box2 around = c.walk.contour().bounds().inflated(reach + indexSlack_);
    for (std::size_t i : starts.query(around)) {
        const Vec2 p = otherPass[i].walk.pointAt(0.0);
        const double v = c.walk.project(p);
        if (geometry::distance(c.walk.pointAt(v), p) <= reach) {
            out.push_back(v);
        }
    }
    return out;
}

void ProfileBuilder::placeTabsOn(std::vector<Cut>& primary, const std::vector<Cut>& otherPass, bool asT0127,
                                 bool refuseAtGap) {
    if (!p_.tabs.enabled) {
        return;
    }
    const BoxIndex otherIndex = startIndex(otherPass);
    const double raised = p_.tabs.length + 2.0 * radius_;
    for (Cut& c : primary) {
        // At least a tool radius cut to full depth between neighbouring tabs, so they stay apart. A
        // tab moved off the other pass's start keeps two tolerances + 0.01 mm more from it: its
        // roughing cover (its ends projected and widened by the tolerance) must clear that start.
        // (`asT0127`: as T-0127 placed them, a moved tab at exactly the gap. T-0127 then let rounding
        // decide whether it clashed with the point it was moved off; that depends on the compiler's
        // floating-point contraction (T-0159), so both outcomes are placed deterministically
        // instead: accepted within the slack, or with `refuseAtGap` refused within it.)
        const double moveMargin = otherPass.empty() || asT0127 ? 0.0 : 2.0 * tol_.distance + 0.01;
        c.tabKeepClear = otherStarts(c, otherPass, otherIndex);
        TabPlacement tp = placeTabs(c.walk, raised, p_.tabs, c.tabKeepClear, radius_, moveMargin,
                                    asT0127 && refuseAtGap ? -kKeepClearSlack : kKeepClearSlack);
        c.tabs.clear();
        for (double u : tp.centres) {
            c.tabs.push_back({u - 0.5 * raised, u + 0.5 * raised});
        }
        c.tabRequested = std::move(tp.requested);
        c.tabOrigin = std::move(tp.origin);
        c.tabMoveMargin = moveMargin;
        c.tabFitDropped = tp.dropped;
        tabsRequested_[c.sources.front()] += tp.centres.size() + tp.dropped;
        tabsDropped_[c.sources.front()] += tp.dropped;
        tabsOnCorner_[c.sources.front()] += tp.notAvoided;
    }
}

// True when `t` (a raised stretch on `c`) can be cut on `c`: its footprint (3D ramps included)
// clears the start of a closed cut and the ends of an open one, every point in `keepClear`, and
// the cut's other tabs, each by kMinTabGap.
bool ProfileBuilder::spanFits(const Cut& c, const TabSpan& t, const std::vector<double>& keepClear) const {
    const double ramp = p_.tabs.threeD ? p_.tabs.thickness : 0.0;
    const double a = t.a - ramp;
    const double b = t.b + ramp;
    const double len = c.walk.length();
    if (a < kMinTabGap || b > len - kMinTabGap) {
        return false;
    }
    for (double f : keepClear) {
        if (f > a - kMinTabGap && f < b + kMinTabGap) {
            return false;
        }
    }
    for (const TabSpan& o : c.tabs) {
        if (a < o.b + ramp + kMinTabGap && o.a - ramp < b + kMinTabGap) {
            return false;
        }
    }
    return true;
}

// The material a tab leaves, as cross-sections in order along its core (the raised stretch less a
// tool radius at each end): every 0.1 mm (exact ends included), plus a fan every 5° at a corner of
// the cut, where the material runs across the corner. Each runs along the normal from the vector
// out to the far wall of the widest kerf at the tab (a tool diameter, plus the last-pass allowance
// when the roughing cut runs outside it), less 0.02 mm at each end so the walls do not count.
// `coverOnly` (Inside/Outside with a negative allowance only): the sections stop at the vector, as
// for allowance >= 0, instead of crossing it with the kerf (see matchRoughingTabs).
std::vector<TabSection> ProfileBuilder::tabSections(const Cut& c, const TabSpan& t, bool coverOnly, double kerf) const {
    std::vector<TabSection> out;
    // (`kerf`, when given, replaces the tool radius as the reach beyond the cut: T-0167, the tab layer's.)
    const double ca = t.a + radius_;
    const double cb = t.b - radius_;
    if (cb < ca) {
        return out;
    }
    double outer = (kerf >= 0.0 ? kerf : radius_) + (p_.separateLastPass ? p_.lastPassAllowance : 0.0) - 0.02;
    double inner = c.distance - 0.02;
    if (sidedTabs_ && !coverOnly) {
        // The kerf of the cut itself on each side (it crosses the vector, or runs on it).
        inner = kerfVectorSide() - 0.02;
        outer = kerfOtherSide() - 0.02;
    }
    auto section = [&](Vec2 p, Vec2 tg) {
        Vec2 toVector{-tg.y, tg.x};  // left of travel
        if (!c.contourOnLeft) {
            toVector = toVector * -1.0;
        }
        out.push_back({p + toVector * inner, p - toVector * outer});
    };
    constexpr double fanStep = 5.0 * geometry::kPi / 180.0;
    std::vector<ArcLengthPath::Kink> kinks = c.walk.kinks(fanStep);
    std::size_t k = 0;
    const int n = std::max(1, static_cast<int>(std::ceil((cb - ca) / 0.1)));
    for (int i = 0; i <= n; ++i) {
        const double u = ca + (cb - ca) * i / n;
        for (; k < kinks.size() && kinks[k].u <= u; ++k) {
            if (kinks[k].u <= ca) {
                continue;
            }
            const Vec2 t0 = c.walk.tangentAt(kinks[k].u - 1e-6);
            const int m = static_cast<int>(std::ceil(std::fabs(kinks[k].turn) / fanStep));
            for (int j = 1; j < m; ++j) {
                const double phi = kinks[k].turn * j / m;
                section(c.walk.pointAt(kinks[k].u),
                        Vec2{t0.x * std::cos(phi) - t0.y * std::sin(phi), t0.x * std::sin(phi) + t0.y * std::cos(phi)});
            }
        }
        section(c.walk.pointAt(u), c.walk.tangentAt(u));
    }
    return out;
}

// The roughing cut of a separate last pass runs the last-pass allowance further along the offset
// direction: away from the vector for Inside/Outside and On with allowance >= 0, but towards (and
// across) the vector On with a negative allowance (the finishing cut runs left of the vector there).
double ProfileBuilder::kerfVectorSide() const {
    const bool roughingThisSide = p_.separateLastPass && p_.side == ProfileSide::On && p_.allowance < 0.0;
    return radius_ + (roughingThisSide ? p_.lastPassAllowance : 0.0);
}

double ProfileBuilder::kerfOtherSide() const {
    const bool roughingThisSide = p_.separateLastPass && !(p_.side == ProfileSide::On && p_.allowance < 0.0);
    return radius_ + (roughingThisSide ? p_.lastPassAllowance : 0.0);
}

// Maps the raised stretch `t` of cut `from` onto cut `onto` (a cut of the other pass): its two
// ends and its middle are projected, and the stretch between them (following the direction that
// passes through the middle) is widened by the distance tolerance, so the mapped stretch covers
// the tab on `onto` round corners and curves too (projection keeps the angle on arcs). `near` is
// false when `onto` does not pass the tab at all (nothing to map); a partial or ambiguous match
// returns false with `near` true.
bool ProfileBuilder::mapTab(const Cut& from, const TabSpan& t, const Cut& onto, TabSpan& out, bool& near,
                            bool coverOnly) const {
    const double reach = radius_ + p_.lastPassAllowance + 2.0 * tol_.distance;
    const Vec2 pts[3] = {from.walk.pointAt(t.a), from.walk.pointAt(0.5 * (t.a + t.b)), from.walk.pointAt(t.b)};
    double v[3];
    int hits = 0;
    for (int i = 0; i < 3; ++i) {
        v[i] = onto.walk.project(pts[i]);
        hits += geometry::distance(onto.walk.pointAt(v[i]), pts[i]) <= reach ? 1 : 0;
    }
    near = hits > 0;
    if (hits < 3) {
        return false;
    }
    const double len = onto.walk.length();
    double a = 0.0;
    double b = 0.0;
    if (onto.walk.closed()) {
        auto fwd = [&](double x, double y) {
            const double d = std::fmod(y - x, len);
            return d < 0.0 ? d + len : d;
        };
        const double forward = fwd(v[0], v[1]) + fwd(v[1], v[2]);
        const double backward = fwd(v[1], v[0]) + fwd(v[2], v[1]);
        if (forward <= backward) {
            a = v[0];
            b = v[0] + forward;
        } else {  // `onto` runs the other way (last pass reversed)
            a = v[2];
            b = v[2] + backward;
        }
        if (b - a >= 0.5 * len) {
            return false;
        }
    } else {
        a = std::min(v[0], v[2]);
        b = std::max(v[0], v[2]);
        if (v[1] < a || v[1] > b) {
            return false;
        }
    }
    // Consistency: the stretch must be about as long as the tab (the cuts lie the last-pass
    // allowance apart; corners change lengths by less than that plus a tool radius per end), and
    // every point along the tab must project into it — otherwise the projections landed on
    // different parts of `onto` (a nearby lobe) and the match is ambiguous.
    {
        const double slack = 2.0 * (p_.lastPassAllowance + radius_) + tol_.distance;
        if (std::fabs((b - a) - (t.b - t.a)) > slack) {
            return false;
        }
        const int n = std::max(2, static_cast<int>(std::ceil((t.b - t.a) / 0.25)));
        for (int i = 0; i <= n; ++i) {
            const Vec2 q = from.walk.pointAt(t.a + (t.b - t.a) * i / n);
            const double w = onto.walk.project(q);
            if (geometry::distance(onto.walk.pointAt(w), q) > reach) {
                return false;
            }
            double rel = w - a;
            if (onto.walk.closed()) {
                rel = std::fmod(rel, len);
                if (rel < 0.0) rel += len;
            }
            if (rel < -tol_.distance || rel > (b - a) + tol_.distance) {
                return false;
            }
        }
    }
    // The ends are projected along the normals and widened by the tolerance. That alone is not
    // enough: past a corner, or where the roughing cut runs round a concave curve (shorter than
    // the finishing cut), the roughing tool standing at or beyond a mapped end still reaches the
    // tab's material. So every roughing position from each end out to a window of two tool radii
    // plus the last-pass allowance is tested against the tab's cross-sections (the material from
    // the vector out to the far wall of the roughing kerf, along the tab's core), and the end is
    // moved past the furthest one that reaches it, 0.05 mm at a time, by at most two tool
    // diameters plus the allowance (beyond that the match fails: the tab is left out and
    // reported). Positions further away that still reach it (a lobe coming back) are found by
    // dropSweptTabs.
    out = {a - tol_.distance, b + tol_.distance};
    const std::vector<TabSection> material = tabSections(from, t, coverOnly);
    if (material.empty()) {
        return false;
    }
    constexpr double step = 0.05;
    const double window = 2.0 * radius_ + p_.lastPassAllowance;
    const double cap = 4.0 * radius_ + p_.lastPassAllowance;
    auto reaches = [&](double u) { return sectionDistance(onto.walk.pointAt(u), material) < radius_; };
    // Extension needed at one end (dir -1: before out.a, +1: after out.b), or a negative value
    // when it exceeds the cap.
    auto extension = [&](double end, double dir) {
        double ext = 0.0;
        for (;;) {
            double furthest = -1.0;
            for (double k = 0.0; k <= window + 1e-9; k += step) {
                if (reaches(end + dir * (ext + k))) {
                    furthest = k;
                }
            }
            if (furthest < 0.0) {
                return ext;
            }
            ext += furthest + step;
            if (ext > cap) {
                return -1.0;
            }
        }
    };
    const double extA = extension(out.a, -1.0);
    const double extB = extension(out.b, 1.0);
    if (extA < 0.0 || extB < 0.0) {
        return false;
    }
    // Both ends by the same amount, so the mapped stretch stays centred on the tab.
    const double ext = std::max(extA, extB);
    out.a -= ext;
    out.b += ext;
    if (onto.walk.closed() && out.b - out.a >= len - 2.0 * kMinTabGap) {
        return false;
    }
    if (!onto.walk.closed() && (out.a < 0.0 || out.b > len)) {
        return false;
    }
    return true;
}

// Separate last pass: the finishing cuts' tabs (placed first) are carried onto the roughing cuts.
// Every finishing tab gets a covering raised stretch on each roughing cut that passes it; when
// one cannot be placed there (it would cover a roughing start, a finishing start or another tab),
// the finishing tab is left out too, so the two passes always agree. Inside/Outside with a negative
// allowance: the cross-sections that place the cover run across the whole kerf, past the vector,
// and round a concave corner of the part the roughing cut on the other leg comes within reach of
// their far ends, so the cover grows and may then run into a start or another tab. A tab refused
// that way is tried again with the cover placed as for allowance >= 0 (sections from the vector
// out); the shorter cover only nicks the tab's material on the part side, and dropSweptTabs judges
// the result exactly (the tab is dropped as cut through if it no longer holds).
ProfileBuilder::MatchExtras ProfileBuilder::matchRoughingTabs(std::vector<Cut>& finish, std::vector<Cut>& rough,
                                                              bool addBack, bool retryCover) {
    MatchExtras extras;
    for (Cut& r : rough) {
        r.tabs.clear();
    }
    std::vector<std::vector<double>> roughKeepClear(rough.size());
    const BoxIndex finishStarts = startIndex(finish);
    const BoxIndex roughIndex = walkIndex(rough);
    const double mapReach = radius_ + p_.lastPassAllowance + 2.0 * tol_.distance;  // as mapTab
    for (std::size_t k = 0; k < rough.size(); ++k) {
        roughKeepClear[k] = otherStarts(rough[k], finish, finishStarts);
    }
    tabLinks_.assign(finish.size(), {});
    const double raised = p_.tabs.length + 2.0 * radius_;
    const double footprint = tabFootprint(raised, p_.tabs);
    for (std::size_t fi = 0; fi < finish.size(); ++fi) {
        Cut& f = finish[fi];
        const std::size_t src = f.sources.front();
        // Add-back (`addBack`): a refused tab may have kept a neighbour out when the tabs were
        // fitted (they overlapped). Then the tabs fitting dropped are added back where they clear
        // every tab still kept, and this cut is matched again. (run() keeps this result only if,
        // after the tab check, it has more tabs than matching without add-back.)
        std::vector<char> excluded(f.tabRequested.size(), 0);
        std::vector<std::size_t> roughSizes(rough.size());
        for (std::size_t k = 0; k < rough.size(); ++k) roughSizes[k] = rough[k].tabs.size();
        std::vector<TabSpan> kept;
        std::vector<std::vector<std::pair<std::size_t, TabSpan>>> links;
        std::size_t unmatched = 0;
        bool again = true;
        while (again) {
        again = false;
        kept.clear();
        links.clear();
        unmatched = 0;
        for (std::size_t k = 0; k < rough.size(); ++k) rough[k].tabs.resize(roughSizes[k]);
        for (std::size_t ti = 0; ti < f.tabs.size(); ++ti) {
            const TabSpan& t = f.tabs[ti];
            std::vector<std::pair<std::size_t, TabSpan>> mapped;
            bool ok = false;
            const bool retry = retryCover && sidedTabs_ && p_.side != ProfileSide::On;
            // Only roughing cuts within mapTab's reach of the tab's ends or middle can pass it
            // (the others are not `near`); the index returns them in order (T-0131).
            geometry::Box2 tabBox;
            tabBox.expand(f.walk.pointAt(t.a));
            tabBox.expand(f.walk.pointAt(0.5 * (t.a + t.b)));
            tabBox.expand(f.walk.pointAt(t.b));
            const std::vector<std::size_t> passing =
                roughIndex.query(tabBox.inflated(mapReach + indexSlack_));
            for (int attempt = 0; attempt < (retry ? 2 : 1) && !ok; ++attempt) {
                mapped.clear();
                ok = true;
                for (std::size_t pi = 0; pi < passing.size() && ok; ++pi) {
                    const std::size_t k = passing[pi];
                    TabSpan m;
                    bool near = false;
                    if (mapTab(f, t, rough[k], m, near, attempt == 1)) {
                        if (attempt == 1) extras.retried = true;
                        ok = spanFits(rough[k], m, roughKeepClear[k]);
                        mapped.push_back({k, m});
                    } else if (near) {
                        ok = false;  // passes the tab only in part: cannot cover it
                    }
                }
            }
            if (!ok) {
                if (addBack && ti < f.tabOrigin.size()) {
                    // Keep the other tabs where they are and add back, where they now fit (their
                    // footprints clear of every tab kept), the asked-for tabs that fitting dropped;
                    // match this cut again only if one comes back.
                    std::vector<char> trial = excluded;
                    trial[f.tabOrigin[ti]] = 1;
                    std::vector<std::pair<double, std::size_t>> keep;  // centre, requested index
                    std::vector<char> isKept(f.tabRequested.size(), 0);
                    for (std::size_t q = 0; q < f.tabs.size(); ++q) {
                        isKept[f.tabOrigin[q]] = 1;
                        if (q != ti) keep.push_back({0.5 * (f.tabs[q].a + f.tabs[q].b), f.tabOrigin[q]});
                    }
                    const double len = f.walk.length();
                    const double gap = footprint + std::max(radius_, kMinTabGap);
                    auto clear = [&](double c) {
                        return std::none_of(keep.begin(), keep.end(), [&](const std::pair<double, std::size_t>& o) {
                            const double d = std::fabs(c - o.first);
                            return d < gap || (f.walk.closed() && len - d < gap);
                        });
                    };
                    std::vector<std::pair<double, std::size_t>> back;
                    for (std::size_t q = 0; q < f.tabRequested.size(); ++q) {
                        if (trial[q] || isKept[q]) continue;
                        const TabPlacement one = fitTabs(f.walk, {f.tabRequested[q]}, footprint, f.tabKeepClear,
                                                         radius_, f.tabMoveMargin);
                        if (!one.centres.empty()) back.push_back({one.centres.front(), q});
                    }
                    std::sort(back.begin(), back.end());
                    std::size_t added = 0;
                    for (const auto& b : back) {
                        if (!clear(b.first)) continue;
                        keep.push_back(b);
                        ++added;
                    }
                    if (added > 0) {
                        extras.addedBack = true;
                        excluded = std::move(trial);
                        ++tabsDropped_[src];
                        ++tabsUnmatched_[src];
                        tabsDropped_[src] -= added;  // fitting had dropped them
                        f.tabFitDropped -= added;
                        std::sort(keep.begin(), keep.end());
                        f.tabs.clear();
                        f.tabOrigin.clear();
                        for (const auto& [c, q] : keep) {
                            f.tabs.push_back({c - 0.5 * raised, c + 0.5 * raised});
                            f.tabOrigin.push_back(q);
                        }
                        again = true;
                        break;
                    }
                }
                ++unmatched;
                continue;
            }
            for (const auto& [k, m] : mapped) {
                rough[k].tabs.push_back(m);
            }
            kept.push_back(t);
            links.push_back(mapped);
        }
        }
        tabsDropped_[src] += unmatched;
        tabsUnmatched_[src] += unmatched;
        f.tabs = std::move(kept);
        f.tabOrigin.clear();
        tabLinks_[fi] = std::move(links);
    }
    for (Cut& r : rough) {
        std::sort(r.tabs.begin(), r.tabs.end(), [](const TabSpan& x, const TabSpan& y) { return x.a < y.a; });
    }
    return extras;
}

// Inside/Outside with allowance >= 0 (otherwise judgeSidedTab): true when the material of tab `t`
// on cut `c` still joins the part to the waste beyond the kerf once tools (below the tab top) stand at `tools` (every open tool position near the tab). A raster
// (0.05 mm cells, coarser for big tools: a fortieth of the radius) round the tab's core is
// classified by the distance to the vectors (the cut runs c.distance from them): on the vector
// side only the tab's material counts (cells whose nearest core point, every 0.05 mm, lies along
// the normal within the cross-sections' extent, so round a corner of the cut the material across
// it counts too); on the waste side every cell counts, and those beyond the widest kerf (0.2 mm
// past it) are the waste to reach. A cell is cut when a tool comes within its radius plus 1.5
// cells. The tab holds when uncut cells join (4-neighbour) the material next to the vector to the
// waste beyond the kerf; it also holds when there is no such waste at all near it (a hole smaller
// than the kerf). So a tab whose material the tools seal off from the waste (both ends of a short
// raised stretch round a sharp turn of the cut, or a narrow arm) does not hold.
bool ProfileBuilder::tabHolds(const Cut& c, const TabSpan& t, const std::vector<Vec2>& tools,
                              const std::vector<ToolDisc>& toolsX) const {
    const double h = std::max(0.05, radius_ / 40.0);
    // The tool at the tab layer (T-0167: a V-bit's cone there, not its radius at the top of the cut).
    const double rk = tabLayerRadius();
    const double ca = t.a + rk;
    const double cb = t.b - rk;
    if (cb < ca) {
        return false;
    }
    const double inner = c.distance - 0.02;
    const double outer = rk + (p_.separateLastPass ? p_.lastPassAllowance : 0.0) - 0.02;
    const double beyond = c.distance + outer + 0.2;  // distance from the vectors of the waste to reach
    struct Station {
        Vec2 p;
        Vec2 t;
    };
    std::vector<Station> core;
    const int ns = std::max(1, static_cast<int>(std::ceil((cb - ca) / 0.05)));
    geometry::Box2 box;
    for (int i = 0; i <= ns; ++i) {
        const double u = ca + (cb - ca) * i / ns;
        core.push_back({c.walk.pointAt(u), c.walk.tangentAt(u)});
        box.expand(core.back().p);
    }
    box = box.inflated(rasterReach());
    const int nx = static_cast<int>(std::ceil(box.width() / h)) + 1;
    const int ny = static_cast<int>(std::ceil(box.height() / h)) + 1;
    const auto cells = static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny);
    auto centre = [&](int i, int j) { return Vec2{box.min.x + (i + 0.5) * h, box.min.y + (j + 0.5) * h}; };
    auto index = [&](int i, int j) { return static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i); };
    // Core stations bucketed by cells of `reach`, for the nearest-station search.
    const double reach = std::max(std::max(inner, outer), 0.5);
    std::map<std::pair<long, long>, std::vector<std::size_t>> near;
    auto bkey = [&](Vec2 p) {
        return std::pair<long, long>{static_cast<long>(std::floor(p.x / reach)), static_cast<long>(std::floor(p.y / reach))};
    };
    for (std::size_t k = 0; k < core.size(); ++k) {
        near[bkey(core[k].p)].push_back(k);
    }
    // 0 = blocked, 1 = open, 2 = tab material next to the vector, 3 = waste beyond the kerf.
    std::vector<char> state(cells, 0);
    constexpr int kBlock = detail::kSettleBlock;
    const int bx_n = (nx + kBlock - 1) / kBlock;
    const int by_n = (ny + kBlock - 1) / kBlock;
    std::vector<signed char> blockMode(static_cast<std::size_t>(bx_n) * static_cast<std::size_t>(by_n), -1);
    bool anyWaste = false;
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const Vec2 x = centre(i, j);
            // The distance to the vectors is 1-Lipschitz: a block of cells whose centre is more than the
            // block's radius beyond a threshold needs no distance of its own (T-0167, cost).
            const std::size_t blk = static_cast<std::size_t>(j / kBlock) * static_cast<std::size_t>(bx_n) +
                                    static_cast<std::size_t>(i / kBlock);
            if (blockMode[blk] < 0) {
                blockMode[blk] = static_cast<signed char>(detail::settleBlock(
                    i / kBlock, j / kBlock, nx, ny, h, box.min, beyond, c.distance,
                    [&](Vec2 q) { return allField_.distance(q); }));
            }
            const double dv = blockMode[blk] == 1   ? std::numeric_limits<double>::infinity()
                              : blockMode[blk] == 2 ? 0.0
                                                    : allField_.distance(x);
            const auto [bx, by] = bkey(x);
            double best = std::numeric_limits<double>::infinity();
            std::size_t bk = 0;
            for (long dx = -1; dx <= 1; ++dx) {
                for (long dy = -1; dy <= 1; ++dy) {
                    const auto it = near.find({bx + dx, by + dy});
                    if (it == near.end()) continue;
                    for (std::size_t k : it->second) {
                        const double d = geometry::distance(x, core[k].p);
                        if (d < best) {
                            best = d;
                            bk = k;
                        }
                    }
                }
            }
            if (dv >= c.distance) {  // waste side
                state[index(i, j)] = dv >= beyond ? char{3} : char{1};
                anyWaste = anyWaste || dv >= beyond;
                continue;
            }
            const double limit = inner;
            if (best > limit) continue;
            const Vec2 rel = x - core[bk].p;
            const double along = rel.x * core[bk].t.x + rel.y * core[bk].t.y;
            if ((bk == 0 && along < -h) || (bk + 1 == core.size() && along > h)) continue;  // past the core
            state[index(i, j)] = best < limit - 0.15 ? char{1} : char{2};
        }
    }
    if (!anyWaste) {
        return true;  // no waste beyond the kerf near the tab: nothing to hold on to
    }
    // Cut cells, a row of each tool's disc at a time.
    auto stamp = [&](Vec2 q, double rr) {
        const int j0 = std::max(0, static_cast<int>(std::floor((q.y - rr - box.min.y) / h)));
        const int j1 = std::min(ny - 1, static_cast<int>(std::ceil((q.y + rr - box.min.y) / h)));
        for (int j = j0; j <= j1; ++j) {
            const double dy = box.min.y + (j + 0.5) * h - q.y;
            const double w2 = rr * rr - dy * dy;
            if (w2 <= 0.0) continue;
            const double w = std::sqrt(w2);
            const int i0 = std::max(0, static_cast<int>(std::ceil((q.x - w - box.min.x) / h - 0.5)));
            const int i1 = std::min(nx - 1, static_cast<int>(std::floor((q.x + w - box.min.x) / h - 0.5)));
            for (int i = i0; i <= i1; ++i) {
                state[index(i, j)] = 0;
            }
        }
    };
    const double rr = rk - 1e-3 + 1.5 * h;
    for (const Vec2& q : tools) {
        stamp(q, rr);
    }
    // (A V-bit's corner moves cut smaller discs: the cone's radius at the tab layer.)
    for (const ToolDisc& d : toolsX) {
        stamp(d.p, d.rho - 1e-3 + 1.5 * h);
    }
    std::vector<char> seen(cells, 0);
    std::vector<std::size_t> queue;
    for (std::size_t k = 0; k < cells; ++k) {
        if (state[k] == 2) {
            seen[k] = 1;
            queue.push_back(k);
        }
    }
    for (std::size_t q = 0; q < queue.size(); ++q) {
        const int i = static_cast<int>(queue[q] % static_cast<std::size_t>(nx));
        const int j = static_cast<int>(queue[q] / static_cast<std::size_t>(nx));
        const int di[4] = {1, -1, 0, 0};
        const int dj[4] = {0, 0, 1, -1};
        for (int e = 0; e < 4; ++e) {
            const int ii = i + di[e];
            const int jj = j + dj[e];
            if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
            const std::size_t k = index(ii, jj);
            if (seen[k] || state[k] == 0) continue;
            if (state[k] == 3) return true;
            seen[k] = 1;
            queue.push_back(k);
        }
    }
    return false;
}

// Sided tabs (On the vector, or a negative allowance: the kerf crosses or straddles the vector, so
// the distance to the vectors no longer tells the part from the waste). The tab is judged by the
// pieces of material the cut leaves at its layer, just below the tab top, where every tool position
// cuts a disc of the tab-layer radius (tabLayerRadius: a V-bit's cone there). On a raster round the
// tab's raised stretches: the pieces are the cells no tool cuts with every tab lowered (this one
// too); the tab's own material is the cells only this tab's lowered stretches would cut; the tab
// joins the pieces its material touches. A piece counts when it covers 0.01 mm² and is thicker than
// 0.05 mm somewhere; the tab must join every counting piece within the tab-layer radius + 0.3 mm of
// its raised stretches, and there must be two (else it would hold nothing). The other live tabs
// count as cut, so a tab never relies on another's bridge. This is judged with the tools exactly as
// wide (+ 0.01 mm, which closes hairline gaps between cell centres), and again widened by 0.055 mm
// (where that fails on 0.05 mm cells, again on 0.025 mm cells), where the tab must not leave a near
// piece unjoined: so every kept tab joins the part to the waste through a neck of about 0.1 mm or
// more, and leaves no piece near it loose.
ProfileBuilder::TabFate ProfileBuilder::judgeSidedTab(const std::vector<Vec2>& own, const std::vector<Vec2>& tools,
                                                      const std::vector<ToolDisc>& ownX,
                                                      const std::vector<ToolDisc>& toolsX, double spacing,
                                                      const std::vector<Vec2>& ends, double margin,
                                                      bool& limited, bool lenient) const {
    limited = false;
    if (own.empty()) {
        return TabFate::Severed;
    }
    const double rt = tabLayerRadius();
    // Tool positions `spacing` apart along the cuts: discs grown by the sagitta of the gap cover
    // what the tool sweeps between them.
    const double gap = spacing * spacing / (8.0 * rt);
    geometry::Box2 box;
    for (const Vec2& q : own) box.expand(q);
    for (const ToolDisc& d : ownX) box.expand(d.p);
    box = box.inflated(margin);
    const double area = box.width() * box.height();
    // One check on cells of `h` with the tools `cutRadius` wide.
    auto check = [&](double h, double cutRadius) {
        h = std::max(h, std::sqrt(area / 3e6));
        DiscRaster nearRas(box, h);
        // (Near: within reach of the raised stretches, their exact ends included.)
        for (const Vec2& q : own) nearRas.add(q, rt + 0.3 + 2.0 * h);
        for (const Vec2& q : ends) nearRas.add(q, rt + 0.3 + 2.0 * h);
        bool lim = false;
        PieceVerdict v = sidedPieces(own, tools, ownX, toolsX, box, h, cutRadius, nearRas.mask(), lim);
        // Lenient (a V-bit's corner moves in play): a tab that still joins two pieces, or one that
        // the moves leave holding nothing, is kept, as without them.
        if (lenient && (v == PieceVerdict::Partial || v == PieceVerdict::NoHold)) v = PieceVerdict::Hold;
        if (v == PieceVerdict::Partial || v == PieceVerdict::Severed) limited = limited || lim;
        return v;
    };
    const PieceVerdict exact = check(0.05, rt + 0.01 + gap);
    if (exact == PieceVerdict::NoHold) return TabFate::Empty;
    if (exact != PieceVerdict::Hold) return TabFate::Severed;
    // The widened check; where it fails on 0.05 mm cells, again on cells half as big, so a neck
    // just over 0.1 mm is resolved.
    PieceVerdict wide = check(0.05, rt + 0.055 + gap);
    if (wide == PieceVerdict::Partial || wide == PieceVerdict::Severed) wide = check(0.025, rt + 0.055 + gap);
    return wide == PieceVerdict::Partial || wide == PieceVerdict::Severed ? TabFate::Severed : TabFate::Holds;
}

ProfileBuilder::PieceVerdict ProfileBuilder::sidedPieces(const std::vector<Vec2>& own, const std::vector<Vec2>& tools,
                                                         const std::vector<ToolDisc>& ownX,
                                                         const std::vector<ToolDisc>& toolsX, const geometry::Box2& box,
                                                         double h, double cutRadius, const std::vector<char>& nearO,
                                                         bool& limited) const {
    limited = false;
    DiscRaster ras(box, h);
    const geometry::Box2 toolBox = box.inflated(cutRadius + 0.05);
    auto stamp = [&](const std::vector<Vec2>& pts, double r) {
        for (const Vec2& q : pts) {
            if (toolBox.contains(q)) ras.add(q, r);
        }
        return ras.mask();
    };
    // (Corner moves of a V-bit cut the cone's smaller disc: the same widening on its radius.)
    // (They are widened by the hairline allowance only, not by the 0.055 mm of the widened check: a
    // tab is dropped for a corner move only where the move really crosses its neck, never on the
    // margin alone.)
    const double widen = std::min(cutRadius - tabLayerRadius(), 0.012);
    auto stampX = [&](const std::vector<ToolDisc>& discs) {
        for (const ToolDisc& d : discs) {
            if (toolBox.contains(d.p)) ras.add(d.p, widen + d.rho);
        }
    };
    stampX(toolsX);
    const std::vector<char> cutT = stamp(tools, cutRadius);
    stampX(ownX);
    const std::vector<char> cutO = stamp(own, cutRadius);
    // Thickness: a cell farther than cutRadius + 0.025 from every tool position (own ones too),
    // looked up in buckets of that size.
    const double thickR = cutRadius + 0.025;
    std::map<std::pair<long, long>, std::vector<Vec2>> buckets;
    auto bucketOf = [&](Vec2 q) {
        return std::pair<long, long>{static_cast<long>(std::floor(q.x / thickR)), static_cast<long>(std::floor(q.y / thickR))};
    };
    for (const std::vector<Vec2>* pts : {&tools, &own}) {
        for (const Vec2& q : *pts) {
            if (toolBox.contains(q)) buckets[bucketOf(q)].push_back(q);
        }
    }
    std::map<std::pair<long, long>, std::vector<ToolDisc>> bucketsX;
    for (const std::vector<ToolDisc>* discs : {&toolsX, &ownX}) {
        for (const ToolDisc& d : *discs) {
            if (toolBox.contains(d.p)) bucketsX[bucketOf(d.p)].push_back(d);
        }
    }
    auto isThick = [&](Vec2 x) {
        const auto [bx, by] = bucketOf(x);
        for (long dx = -1; dx <= 1; ++dx) {
            for (long dy = -1; dy <= 1; ++dy) {
                const auto it = buckets.find({bx + dx, by + dy});
                if (it != buckets.end()) {
                    for (const Vec2& q : it->second) {
                        if (geometry::distance(q, x) <= thickR) return false;
                    }
                }
                const auto ix = bucketsX.find({bx + dx, by + dy});
                if (ix != bucketsX.end()) {
                    for (const ToolDisc& d : ix->second) {
                        if (geometry::distance(d.p, x) <= widen + d.rho + 0.025) return false;
                    }
                }
            }
        }
        return true;
    };
    const int nx = ras.nx();
    const int ny = ras.ny();
    const std::size_t cells = ras.cells();
    // Pieces with every tab lowered (4-neighbour components).
    std::vector<int> label(cells, -1);
    std::vector<std::size_t> area;
    std::vector<char> thick;  // 0 unknown, 1 thick, 2 thin
    std::vector<std::size_t> firstCell;
    std::vector<std::size_t> stack;
    const int di[4] = {1, -1, 0, 0};
    const int dj[4] = {0, 0, 1, -1};
    for (std::size_t k0 = 0; k0 < cells; ++k0) {
        if (label[k0] >= 0 || cutT[k0] || cutO[k0]) continue;
        const int l = static_cast<int>(area.size());
        area.push_back(0);
        thick.push_back(0);
        firstCell.push_back(k0);
        label[k0] = l;
        stack.assign(1, k0);
        while (!stack.empty()) {
            const std::size_t k = stack.back();
            stack.pop_back();
            ++area[static_cast<std::size_t>(l)];
            const int i = static_cast<int>(k % static_cast<std::size_t>(nx));
            const int j = static_cast<int>(k / static_cast<std::size_t>(nx));
            for (int e = 0; e < 4; ++e) {
                const int ii = i + di[e];
                const int jj = j + dj[e];
                if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
                const std::size_t kk = ras.index(ii, jj);
                if (label[kk] >= 0 || cutT[kk] || cutO[kk]) continue;
                label[kk] = l;
                stack.push_back(kk);
            }
        }
    }
    // The counting pieces near the tab (thickness looked up only for these).
    std::vector<char> counts(area.size(), 0);
    std::vector<int> nearPieces;
    for (std::size_t k = 0; k < cells; ++k) {
        const int l = label[k];
        if (l < 0 || !nearO[k] || counts[static_cast<std::size_t>(l)]) continue;
        const auto li = static_cast<std::size_t>(l);
        counts[li] = 1;
        if (static_cast<double>(area[li]) * h * h < 0.01) continue;
        if (thick[li] == 0) {
            thick[li] = 2;
            for (std::size_t c2 = firstCell[li]; c2 < cells && thick[li] == 2; ++c2) {
                if (label[c2] == l && isThick(ras.centre(static_cast<int>(c2 % static_cast<std::size_t>(nx)),
                                                          static_cast<int>(c2 / static_cast<std::size_t>(nx))))) {
                    thick[li] = 1;
                }
            }
        }
        if (thick[li] == 1) nearPieces.push_back(l);
    }
    if (nearPieces.size() <= 1) {
        return PieceVerdict::NoHold;
    }
    // The tab's own material joins the pieces each of its components touches.
    std::vector<int> parent(area.size());
    std::iota(parent.begin(), parent.end(), 0);
    auto find = [&](int x) {
        while (parent[static_cast<std::size_t>(x)] != x) {
            parent[static_cast<std::size_t>(x)] = parent[static_cast<std::size_t>(parent[static_cast<std::size_t>(x)])];
            x = parent[static_cast<std::size_t>(x)];
        }
        return x;
    };
    std::vector<char> seen(cells, 0);
    for (std::size_t k0 = 0; k0 < cells; ++k0) {
        if (seen[k0] || cutT[k0] || !cutO[k0]) continue;
        seen[k0] = 1;
        stack.assign(1, k0);
        int first = -1;
        while (!stack.empty()) {
            const std::size_t k = stack.back();
            stack.pop_back();
            const int i = static_cast<int>(k % static_cast<std::size_t>(nx));
            const int j = static_cast<int>(k / static_cast<std::size_t>(nx));
            for (int e = 0; e < 4; ++e) {
                const int ii = i + di[e];
                const int jj = j + dj[e];
                if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
                const std::size_t kk = ras.index(ii, jj);
                if (label[kk] >= 0) {
                    if (first < 0) {
                        first = find(label[kk]);
                    } else {
                        parent[static_cast<std::size_t>(find(label[kk]))] = first;
                        first = find(first);
                    }
                } else if (!seen[kk] && !cutT[kk] && cutO[kk]) {
                    seen[kk] = 1;
                    stack.push_back(kk);
                }
            }
        }
    }
    std::map<int, std::size_t> groupsOf;
    for (int l : nearPieces) ++groupsOf[find(l)];
    std::size_t largest = 0;
    for (const auto& [root, n] : groupsOf) largest = std::max(largest, n);
    if (largest == nearPieces.size()) return PieceVerdict::Hold;
    // Groups with a near piece at the frame: two of them may join beyond it.
    {
        std::vector<char> frame(area.size(), 0);
        auto mark = [&](int i, int j) {
            const int l = label[ras.index(i, j)];
            if (l >= 0) frame[static_cast<std::size_t>(l)] = 1;
        };
        for (int i = 0; i < nx; ++i) {
            mark(i, 0);
            mark(i, ny - 1);
        }
        for (int j = 0; j < ny; ++j) {
            mark(0, j);
            mark(nx - 1, j);
        }
        std::set<int> atFrame;
        for (int l : nearPieces) {
            if (frame[static_cast<std::size_t>(l)]) atFrame.insert(find(l));
        }
        limited = atFrame.size() >= 2;
    }
    return largest >= 2 ? PieceVerdict::Partial : PieceVerdict::Severed;
}

// A tab only holds if no cut sweeps through its material below the tab top: in a slot narrower
// than the kerf, past a nearby corner, or where a lobe of a cut comes back, the tool removes the
// material and severs the tab. Each primary tab, with its matched stretches on the other pass, is
// dropped (and reported) when any cut, outside the raised stretches of the tabs still kept,
// comes closer than a tool radius (less 1 µm) to the tab's material: its cross-sections from the
// vector out to the far wall of the kerf (tabSections). The tab's own cut near the tab (within two
// tool radii of its raised stretch) is treated apart: on a convex curve, and round a corner of the
// cut, it always nicks some cross-sections near the ends of the core, which only narrows the tab,
// so it only has to leave a run of whole cross-sections at least min(0.5 mm, half the tab) wide;
// on a sharp convex corner it can cut through every cross-section of a short tab.
// Repeated until nothing changes, since a dropped tab exposes more of its cut.
void ProfileBuilder::dropSweptTabs(std::vector<Cut>& primary, std::vector<Cut>& secondary) {
    constexpr double h = 0.05;  // sampling step of the cut paths
    const bool linked = !tabLinks_.empty();
    struct Ref {
        bool second;
        std::size_t cut;
        TabSpan span;
    };
    // Groups: a primary tab and its linked secondary spans.
    std::vector<std::vector<Ref>> groups;
    std::vector<std::size_t> groupSource;
    for (std::size_t c = 0; c < primary.size(); ++c) {
        for (std::size_t t = 0; t < primary[c].tabs.size(); ++t) {
            std::vector<Ref> g{{false, c, primary[c].tabs[t]}};
            if (linked && c < tabLinks_.size() && t < tabLinks_[c].size()) {
                for (const auto& [k, m] : tabLinks_[c][t]) {
                    g.push_back({true, k, m});
                }
            }
            groups.push_back(std::move(g));
            groupSource.push_back(primary[c].sources.front());
        }
    }
    if (groups.empty()) {
        return;
    }
    // Each group's material, from its primary tab.
    std::vector<std::vector<TabSection>> material(groups.size());
    for (std::size_t g = 0; g < groups.size(); ++g) {
        const Ref& r = groups[g].front();
        // (Judged at the tab layer: a V-bit's cut there is the cone's disc, narrower than at the top.)
        material[g] = sidedTabs_ ? tabSections(primary[r.cut], r.span)
                                 : tabSections(primary[r.cut], r.span, false, tabLayerRadius());
    }
    std::vector<bool> alive(groups.size(), true);
    // Samples of every cut: every h along it, plus the exact ends of every raised stretch (where
    // the tool goes back down). A sample is protected while a raised stretch of a live group
    // covers it (strictly inside, cyclically on a closed cut).
    struct Sample {
        bool second;
        std::size_t cut;
        double u;
        Vec2 p;
        std::vector<std::size_t> coveredBy;  // groups whose raised stretches cover it
        int seq;  // index among the samples every 0.05 mm along the cut (-1: an exact stretch end)
        // >= 0: a V-bit corner move's tool position (T-0156), cutting this radius at the tab layer;
        // `u` is then its corner, and it is covered by the tabs whose footprint (ramps too) holds it.
        double rho{-1.0};
    };
    std::vector<Sample> samples;
    {
        auto add = [&](const std::vector<Cut>& cuts, bool second) {
            for (std::size_t c = 0; c < cuts.size(); ++c) {
                std::vector<std::pair<std::size_t, TabSpan>> spans;
                for (std::size_t g = 0; g < groups.size(); ++g) {
                    for (const Ref& r : groups[g]) {
                        if (r.second == second && r.cut == c) spans.push_back({g, r.span});
                    }
                }
                const ArcLengthPath& w = cuts[c].walk;
                std::vector<std::pair<double, int>> us;
                for (const auto& [g, t] : spans) {
                    us.push_back({t.a, -1});
                    us.push_back({t.b, -1});
                }
                const int n = std::max(1, static_cast<int>(std::ceil(w.length() / h)));
                detail::chargeWork(detail::unitsOf(5.0 * n));  // work meter (T-0186): the samples of the cut
                for (int i = 0; i <= n; ++i) {
                    us.push_back({w.length() * i / n, i == n ? -1 : i});
                }
                // The stretches covering a sample, found by position (T-0186 round 4: every sample
                // against every stretch was quadratic in the tab count); the result is the same list,
                // in the same order (ascending stretch index).
                std::vector<std::size_t> byStart(spans.size());
                std::iota(byStart.begin(), byStart.end(), std::size_t{0});
                std::sort(byStart.begin(), byStart.end(), [&](std::size_t x, std::size_t y) {
                    return spans[x].second.a < spans[y].second.a;
                });
                double longest = 0.0;
                for (const auto& sp : spans) longest = std::max(longest, sp.second.b - sp.second.a);
                const double wlen = w.length();
                std::vector<std::size_t> hit;
                for (const auto& [u, seq] : us) {
                    Sample q{second, c, u, w.pointAt(u), {}, seq};
                    hit.clear();
                    const int shifts = w.closed() ? 3 : 1;
                    for (int sh = 0; sh < shifts; ++sh) {
                        const double x = w.closed() ? (sh == 0 ? u - wlen : sh == 1 ? u : u + wlen) : u;
                        // Stretches with a < x < b: a in (x - longest, x).
                        auto lo = std::lower_bound(byStart.begin(), byStart.end(), x - longest, [&](std::size_t i, double v) {
                            return spans[i].second.a < v;
                        });
                        for (; lo != byStart.end() && spans[*lo].second.a < x; ++lo) {
                            if (x > spans[*lo].second.a && x < spans[*lo].second.b) hit.push_back(*lo);
                        }
                    }
                    std::sort(hit.begin(), hit.end());
                    hit.erase(std::unique(hit.begin(), hit.end()), hit.end());
                    for (std::size_t i : hit) q.coveredBy.push_back(spans[i].first);
                    samples.push_back(std::move(q));
                }
            }
        };
        add(primary, false);
        add(secondary, true);
        // The corner moves of a V-bit: tool positions every 0.02 mm along each taper of the cone's
        // disc at the tab layer, made only where no kept tab covers the corner.
        auto addExcursions = [&](const std::vector<Cut>& cuts, bool second) {
            const double ramp = p_.tabs.threeD ? std::max(0.0, p_.tabs.thickness) : 0.0;
            for (std::size_t c = 0; c < cuts.size(); ++c) {
                const ArcLengthPath& w = cuts[c].walk;
                for (const Excursion& e : cuts[c].excursions) {
                    std::vector<std::size_t> covering;
                    for (std::size_t g = 0; g < groups.size(); ++g) {
                        for (const Ref& r : groups[g]) {
                            if (r.second != second || r.cut != c) continue;
                            for (double x : {e.u - w.length(), e.u, e.u + w.length()}) {
                                if (x > r.span.a - ramp + 1e-9 && x < r.span.b + ramp - 1e-9) covering.push_back(g);
                                if (!w.closed()) break;
                            }
                        }
                    }
                    for (const Taper& t : e.tapers) {
                        // (A move straight up at the corner cuts the disc the cut itself cuts there.)
                        if (geometry::distance(t.a, t.b) <= 1e-9) continue;
                        const int n = std::max(1, static_cast<int>(std::ceil(geometry::distance(t.a, t.b) / 0.02)));
                        for (int i = 0; i <= n; ++i) {
                            const double f = static_cast<double>(i) / n;
                            const double r = t.ra + (t.rb - t.ra) * f;
                            if (!(r > 0.0)) continue;
                            Sample q{second, c, e.u, geometry::lerp(t.a, t.b, f), covering, -1};
                            q.rho = r;
                            samples.push_back(std::move(q));
                        }
                    }
                }
            }
        };
        addExcursions(primary, false);
        addExcursions(secondary, true);
    }
    auto open = [&](const Sample& q) {
        return std::none_of(q.coveredBy.begin(), q.coveredBy.end(), [&](std::size_t g) { return alive[g]; });
    };
    // Bucket the samples on a grid of tool-radius cells.
    const double cell = std::max(radius_, 1e-3);
    std::map<std::pair<long, long>, std::vector<std::size_t>> grid;
    auto cellOf = [&](double x) { return static_cast<long>(std::floor(x / cell)); };
    for (std::size_t i = 0; i < samples.size(); ++i) {
        grid[{cellOf(samples[i].p.x), cellOf(samples[i].p.y)}].push_back(i);
    }
    // Each group's material box (inflated by the reach of a tool), and the box of the samples its raised
    // stretches protect (exposed when it is dropped).
    std::vector<geometry::Box2> materialBox(groups.size());
    std::vector<geometry::Box2> protectBox(groups.size());
    for (std::size_t g = 0; g < groups.size(); ++g) {
        for (const TabSection& sct : material[g]) {
            materialBox[g].expand(sct.p0);
            materialBox[g].expand(sct.p1);
        }
        materialBox[g] = materialBox[g].inflated(radius_ + 0.6);  // tools reaching the material
    }
    for (const Sample& q : samples) {
        for (std::size_t g : q.coveredBy) protectBox[g].expand(q.p);
    }
    // Path distance from `u` to the stretch `t` on `w` (cyclic on a closed cut).
    auto pathGap = [](const ArcLengthPath& w, double u, const TabSpan& t) {
        auto gap = [&](double x) { return x < t.a ? t.a - x : (x > t.b ? x - t.b : 0.0); };
        if (!w.closed()) return gap(u);
        const double len = w.length();
        return std::min({gap(u - len), gap(u), gap(u + len)});
    };
    const double localReach = 2.0 * radius_;
    // True when sample q is covered by a raised stretch of group g.
    auto coveredBy = [&](const Sample& q, std::size_t g) {
        return std::find(q.coveredBy.begin(), q.coveredBy.end(), g) != q.coveredBy.end();
    };
    // A V-bit corner move as group g sees it: 0 = not made while a live tab of another group covers
    // its corner (it comes back, and g is judged again, when that tab is dropped), 1 = made only
    // when g is lowered, 2 = made anyway.
    auto cornerClass = [&](const Sample& q, std::size_t g) {
        bool own = false;
        for (std::size_t other : q.coveredBy) {
            if (other == g) {
                own = true;
            } else if (alive[other]) {
                return 0;
            }
        }
        return own ? 1 : 2;
    };
    // True when group g's tab is swept (or severed) by the samples open now. With sidedTabs_ the
    // quick answer also counts the raised stretches of the other live tabs as cut, so it only
    // accepts a bridge of the tab's own material.
    auto sweptNow = [&](std::size_t g) -> TabFate {
        if (sidedTabs_) {
            // The tool positions of this tab's raised stretches, and every other one near them.
            std::vector<Vec2> ownPos;
            std::vector<Vec2> toolPos;
            std::vector<ToolDisc> ownX;   // V-bit corner moves this tab's lowering would expose
            std::vector<ToolDisc> toolX;  // and the others
            std::vector<Vec2> quickPos;   // every tool position but the tab's own, corner moves too
            if (protectBox[g].isEmpty()) return TabFate::Severed;
            const double rt = tabLayerRadius();
            const geometry::Box2 reach = protectBox[g].inflated(3.0 * rt + 1.2);
            // Quick exact answer: every tool position within 2 r + 0.7 mm of the tab's raised
            // stretches (r the tab-layer radius) lies on one of the tab's own cuts (this pass, its
            // roughing cover's) close to the stretch along it, the stretch and that window have no
            // corner and bend gently (between samples 0.25 mm apart no turn over 2 degrees or the
            // turn of an arc of radius 2 r + 1 mm) and the stretch is
            // long enough for the widened check. Then no other cut reaches the
            // material within r + 0.3 mm of the stretch, the cut leaves exactly its two sides as
            // pieces there, both thicker than the widening, and the tab's material joins them.
            // (Every k-th sample, at most sqrt(0.04 r) apart, stands for the cut, with the stretch
            // ends; judgeSidedTab grows its discs by the sagitta of the gap: 5 µm for a tab-layer radius of
            // 0.0625 mm or more, more below it, about 11 µm at 0.028 mm.)
            const int stride = std::max(1, static_cast<int>(std::floor(std::sqrt(0.04 * rt) / h)));
            const double spacing = stride * h;
            const double near2 = 2.0 * rt + 0.7 + spacing;
            const double window = 2.5 * near2;
            bool quick = true;
            for (const Ref& r : groups[g]) {
                if (r.span.b - r.span.a < 2.0 * rt + 0.6) quick = false;
            }
            std::vector<std::size_t> toolIdx;
            for (long cx = cellOf(reach.min.x); cx <= cellOf(reach.max.x); ++cx) {
                for (long cy = cellOf(reach.min.y); cy <= cellOf(reach.max.y); ++cy) {
                    const auto it = grid.find({cx, cy});
                    if (it == grid.end()) continue;
                    for (std::size_t i : it->second) {
                        const Sample& q = samples[i];
                        if (q.rho >= 0.0) {
                            const int cls = cornerClass(q, g);
                            if (cls == 1) {
                                ownX.push_back({q.p, q.rho});
                            } else if (cls == 2) {
                                toolX.push_back({q.p, q.rho});
                                quickPos.push_back(q.p);  // (as a full disc: the quick answers stay sound)
                                toolIdx.push_back(i);
                            }
                            continue;
                        }
                        if (q.seq >= 0 && q.seq % stride != 0) continue;
                        if (coveredBy(q, g)) {
                            ownPos.push_back(q.p);
                        } else {
                            toolPos.push_back(q.p);
                            quickPos.push_back(q.p);
                            toolIdx.push_back(i);
                        }
                    }
                }
            }
            if (quick) {
                geometry::Box2 ob;
                for (const Vec2& o : ownPos) ob.expand(o);
                const geometry::Box2 nb = ob.inflated(near2);
                for (std::size_t i : toolIdx) {
                    const Sample& q = samples[i];
                    if (!nb.contains(q.p)) continue;
                    const bool close = std::any_of(ownPos.begin(), ownPos.end(),
                                                   [&](Vec2 o) { return geometry::distance(o, q.p) < near2; });
                    if (!close) continue;
                    const auto ref = std::find_if(groups[g].begin(), groups[g].end(), [&](const Ref& r) {
                        return r.second == q.second && r.cut == q.cut;
                    });
                    if (ref == groups[g].end() ||
                        pathGap((q.second ? secondary : primary)[q.cut].walk, q.u, ref->span) > window) {
                        quick = false;
                        break;
                    }
                }
            }
            bool gentle = quick;  // no corner, gentle bends
            bool turnsLittle = quick;  // at most 120 degrees in all
            if (quick) {
                for (const Ref& r : groups[g]) {
                    const ArcLengthPath& w = (r.second ? secondary : primary)[r.cut].walk;
                    const double from = r.span.a - window;
                    const double to = r.span.b + window;
                    if (!w.closed() && (from < 0.0 || to > w.length())) {
                        gentle = turnsLittle = false;
                        break;
                    }
                    const int n = std::max(2, static_cast<int>(std::ceil((to - from) / 0.25)));
                    const double step = (to - from) / n;
                    const double maxTurn = std::max(2.0 * geometry::kPi / 180.0, step / (2.0 * rt + 1.0));
                    // Turning over any stretch of 2 r + 1 mm (a spike or hairpin turns more).
                    const auto span = static_cast<std::size_t>(std::ceil((2.0 * rt + 1.0) / step));
                    std::vector<double> turns;
                    double total = 0.0;
                    Vec2 prev = w.tangentAt(from);
                    for (int k = 1; k <= n; ++k) {
                        const Vec2 tg = w.tangentAt(from + step * k);
                        const double turn = std::fabs(std::atan2(prev.cross(tg), prev.dot(tg)));
                        turns.push_back(turn);
                        total += turn;
                        if (turns.size() > span) total -= turns[turns.size() - 1 - span];
                        if (turn > maxTurn) gentle = false;
                        if (total > geometry::kPi * 2.0 / 3.0) gentle = turnsLittle = false;
                        prev = tg;
                    }
                }
            }
            // The plain-stretch answer also needs the core of every raised stretch of the tab (the
            // stretch less r + 0.17 mm and the sample gap at each end) clear of every tool by r +
            // 0.12 mm and the sample gap (the corner answer checks its cross-sections instead):
            // a roughing cover shorter than the finishing stretch leaves roughing tools there.
            if (gentle) {
                const double margin = rt + 0.12 + spacing * spacing / (8.0 * rt);
                for (const Ref& r : groups[g]) {
                    const ArcLengthPath& w = (r.second ? secondary : primary)[r.cut].walk;
                    const double a = r.span.a + margin + 0.05;
                    const double b = r.span.b - margin - 0.05;
                    if (b <= a) {
                        gentle = false;
                        break;
                    }
                    const int n = std::max(1, static_cast<int>(std::ceil((b - a) / 0.1)));
                    for (int k = 0; k <= n && gentle; ++k) {
                        const Vec2 x = w.pointAt(a + (b - a) * k / n);
                        if (std::any_of(quickPos.begin(), quickPos.end(), [&](Vec2 q) { return geometry::distance(q, x) < margin; })) {
                            gentle = false;
                        }
                    }
                }
            }
            if (gentle) return TabFate::Holds;
            // Round a corner (no other cut near, at most 120 degrees of turn over 2 r + 1 mm): a run of at least
            // three cross-sections of the tab's material (judged as the cut it lies in) with no tool
            // within the widened radius + 0.06 mm, whose ends lie 0.2 mm apart at both walls, and
            // whose middle one, carried on 0.4 mm past both walls, stays as clear: a strip of the
            // tab's own material, thicker than the widening, joining material beyond the kerf on
            // both sides, with no other piece near. Otherwise rasterise.
            if (turnsLittle) {
                const std::vector<TabSection>& sections = material[g];
                const double thr = rt + 0.17 + spacing * spacing / (8.0 * rt);
                auto clearOf = [&](Vec2 a, Vec2 b) {
                    const TabSection sg{a, b};
                    return std::none_of(quickPos.begin(), quickPos.end(), [&](Vec2 q) { return sectionDistance(q, sg) < thr; });
                };
                std::size_t runStart = 0;
                for (std::size_t k = 0; k <= sections.size(); ++k) {
                    if (k < sections.size() && clearOf(sections[k].p0, sections[k].p1)) continue;
                    if (k >= runStart + 3) {
                        const TabSection& f = sections[runStart];
                        const TabSection& l = sections[k - 1];
                        const TabSection& m = sections[(runStart + k - 1) / 2];
                        const Vec2 dir = m.p1 - m.p0;
                        const double len = std::hypot(dir.x, dir.y);
                        if (len > 0.0 && geometry::distance(f.p0, l.p0) >= 0.2 && geometry::distance(f.p1, l.p1) >= 0.2 &&
                            clearOf(m.p1, m.p1 + dir * (0.4 / len)) && clearOf(m.p0, m.p0 - dir * (0.4 / len))) {
                            return TabFate::Holds;
                        }
                    }
                    runStart = k + 1;
                }
            }
            std::vector<Vec2> ends;
            for (const Ref& r : groups[g]) {
                const ArcLengthPath& w = (r.second ? secondary : primary)[r.cut].walk;
                ends.push_back(w.pointAt(r.span.a));
                ends.push_back(w.pointAt(r.span.b));
            }
            // The raster reaches 2 r + 0.6 mm past the stretches. A piece split by its frame looks
            // like two, one of them unjoined: then judge again on rasters reaching twice and four
            // times as far (with the tools there), which only finds connections the whole scene
            // has (the tools are the same; only the window grows).
            double margin = 2.0 * rt + 0.6;
            bool limited = false;
            // A V-bit's corner moves make the judgement stricter only where they really cut the tab
            // through: if the tab holds without them and still joins two pieces with them, keep it
            // (the loose-piece search reports any piece they do leave free).
            auto judge = [&](bool& lim) {
                TabFate f = judgeSidedTab(ownPos, toolPos, ownX, toolX, spacing, ends, margin, lim);
                if (f != TabFate::Holds && (!ownX.empty() || !toolX.empty())) {
                    bool l0 = false;
                    if (judgeSidedTab(ownPos, toolPos, {}, {}, spacing, ends, margin, l0) == TabFate::Holds &&
                        judgeSidedTab(ownPos, toolPos, ownX, toolX, spacing, ends, margin, l0, true) == TabFate::Holds) {
                        f = TabFate::Holds;
                    }
                }
                return f;
            };
            TabFate fate = judge(limited);
            for (int grow = 0; grow < 2 && fate == TabFate::Severed && limited; ++grow) {
                margin *= 2.0;
                const geometry::Box2 wide = protectBox[g].inflated(margin + rt + 0.6);
                ownPos.clear();
                toolPos.clear();
                ownX.clear();
                toolX.clear();
                for (long cx = cellOf(wide.min.x); cx <= cellOf(wide.max.x); ++cx) {
                    for (long cy = cellOf(wide.min.y); cy <= cellOf(wide.max.y); ++cy) {
                        const auto it = grid.find({cx, cy});
                        if (it == grid.end()) continue;
                        for (std::size_t i : it->second) {
                            const Sample& q = samples[i];
                            if (q.rho >= 0.0) {
                                const int cls = cornerClass(q, g);
                                if (cls != 0) (cls == 1 ? ownX : toolX).push_back({q.p, q.rho});
                                continue;
                            }
                            if (q.seq >= 0 && q.seq % stride != 0) continue;
                            (coveredBy(q, g) ? ownPos : toolPos).push_back(q.p);
                        }
                    }
                }
                fate = judge(limited);
            }
            return fate;
        }
        const double rk = tabLayerRadius();
        const std::vector<TabSection>& sections = material[g];
        const Ref& own = groups[g].front();
        const geometry::Box2& box = materialBox[g];
        std::vector<Vec2> nicks;   // tools of the tab's own cut near it that reach its material
        std::vector<Vec2> nearby;  // every open tool near the tab
        std::vector<ToolDisc> nicksX;   // the same for V-bit corner moves
        std::vector<ToolDisc> nearbyX;
        for (long cx = cellOf(box.min.x); cx <= cellOf(box.max.x); ++cx) {
            for (long cy = cellOf(box.min.y); cy <= cellOf(box.max.y); ++cy) {
                const auto it = grid.find({cx, cy});
                if (it == grid.end()) continue;
                for (std::size_t i : it->second) {
                    const Sample& q = samples[i];
                    if (!open(q)) continue;
                    const bool local = !q.second && q.cut == own.cut &&
                                       pathGap(primary[own.cut].walk, q.u, own.span) < localReach;
                    const double d = sectionDistance(q.p, sections);
                    if (q.rho >= 0.0) {
                        if (!local && d < q.rho - 1e-3) {
                            return TabFate::Severed;
                        }
                        nearbyX.push_back({q.p, q.rho});
                        if (local && d < q.rho + 0.1) {
                            nicksX.push_back({q.p, q.rho});
                        }
                        continue;
                    }
                    if (!local && d < rk - 1e-3) {
                        return TabFate::Severed;
                    }
                    nearby.push_back(q.p);
                    if (local && d < radius_ + 0.1) {
                        nicks.push_back(q.p);
                    }
                }
            }
        }
        if (nicks.empty() && nicksX.empty()) {
            return TabFate::Holds;
        }
        // Quick answer: a run of at least three whole cross-sections (no tool within a radius +
        // 0.1 mm) whose ends lie at least 0.2 mm apart at both walls, where the middle one, carried
        // on 0.4 mm past the far wall, reaches the waste beyond the kerf with no tool within a radius
        // + 0.1 mm of it either: material joins the part to the waste along it. Otherwise (round a
        // corner or a sharp turn of the cut, in a narrow arm) rasterise.
        const Cut& ownCut = primary[own.cut];
        const double beyond = ownCut.distance + rk + (p_.separateLastPass ? p_.lastPassAllowance : 0.0) + 0.18;
        auto clear = [&](const TabSection& sc) {
            return std::none_of(nicks.begin(), nicks.end(), [&](Vec2 q) { return sectionDistance(q, sc) < rk + 0.1; }) &&
                   std::none_of(nicksX.begin(), nicksX.end(),
                                [&](const ToolDisc& d) { return sectionDistance(d.p, sc) < d.rho + 0.1; });
        };
        std::size_t runStart = 0;
        for (std::size_t k = 0; k <= sections.size(); ++k) {
            if (k < sections.size() && clear(sections[k])) {
                continue;
            }
            if (k >= runStart + 3) {
                const TabSection& f = sections[runStart];
                const TabSection& l = sections[k - 1];
                const TabSection& m = sections[(runStart + k - 1) / 2];
                const Vec2 dir = m.p1 - m.p0;
                const double len = std::hypot(dir.x, dir.y);
                if (len > 0.0 && geometry::distance(f.p0, l.p0) >= 0.2 && geometry::distance(f.p1, l.p1) >= 0.2) {
                    const TabSection ext{m.p1, m.p1 + dir * (0.4 / len)};
                    if (allField_.distance(ext.p1) >= beyond &&
                        std::none_of(nearby.begin(), nearby.end(),
                                     [&](Vec2 q) { return sectionDistance(q, ext) < radius_ + 0.1; }) &&
                        std::none_of(nearbyX.begin(), nearbyX.end(),
                                     [&](const ToolDisc& d) { return sectionDistance(d.p, ext) < d.rho + 0.1; })) {
                        return TabFate::Holds;
                    }
                }
            }
            runStart = k + 1;
        }
        // Every open tool that can reach the raster (the core inflated by rasterReach).
        std::vector<Vec2> tools;
        std::vector<ToolDisc> toolsX;
        geometry::Box2 rb;
        for (const TabSection& sct : sections) {
            rb.expand(sct.p0);
            rb.expand(sct.p1);
        }
        rb = rb.inflated(rasterReach() + radius_ + 0.5);
        for (long cx = cellOf(rb.min.x); cx <= cellOf(rb.max.x); ++cx) {
            for (long cy = cellOf(rb.min.y); cy <= cellOf(rb.max.y); ++cy) {
                const auto it = grid.find({cx, cy});
                if (it == grid.end()) continue;
                for (std::size_t i : it->second) {
                    if (!open(samples[i])) continue;
                    if (samples[i].rho >= 0.0) {
                        toolsX.push_back({samples[i].p, samples[i].rho});
                    } else {
                        tools.push_back(samples[i].p);
                    }
                }
            }
        }
        return tabHolds(ownCut, own.span, tools, toolsX) ? TabFate::Holds : TabFate::Severed;
    };
    // Worklist, lowest group first: dropping a group exposes the samples its raised stretches
    // protected, so the live groups whose material lies within reach of them are checked again.
    std::set<std::size_t> work;
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (!material[g].empty()) work.insert(g);
    }
    while (!work.empty()) {
        const std::size_t g = *work.begin();
        work.erase(work.begin());
        if (!alive[g]) continue;
        const TabFate fate = sweptNow(g);
        if (fate == TabFate::Holds) continue;
        alive[g] = false;
        ++tabsDropped_[groupSource[g]];
        ++(fate == TabFate::Empty ? tabsEmpty_ : tabsSevered_)[groupSource[g]];
        for (std::size_t o = 0; o < groups.size(); ++o) {
            if (alive[o] && !material[o].empty() && !protectBox[g].isEmpty() &&
                materialBox[o].intersects(protectBox[g].inflated(h))) {
                work.insert(o);
            }
        }
    }
    for (Cut& c : primary) c.tabs.clear();
    for (Cut& c : secondary) c.tabs.clear();
    for (std::size_t g = 0; g < groups.size(); ++g) {
        if (!alive[g]) continue;
        for (const Ref& r : groups[g]) {
            (r.second ? secondary[r.cut] : primary[r.cut]).tabs.push_back(r.span);
        }
    }
    auto byStart = [](const TabSpan& x, const TabSpan& y) { return x.a < y.a; };
    for (Cut& c : primary) std::sort(c.tabs.begin(), c.tabs.end(), byStart);
    for (Cut& c : secondary) std::sort(c.tabs.begin(), c.tabs.end(), byStart);
}

// The band of a corner move's taper (T-0156): the convex hull of its two end discs, each grown by
// `grow`, as a counter-clockwise polygon reaching at most `sag` beyond the discs.
detail::PointRing taperRing(const Taper& t, double grow, double sag) {
    std::vector<Vec2> pts;
    for (int e = 0; e < 2; ++e) {
        const Vec2 c = e == 0 ? t.a : t.b;
        const double r = (e == 0 ? t.ra : t.rb) + grow;
        // n vertices on the circumscribed circle of radius r / cos(pi / n): at most `sag` outside.
        int n = 8;
        while (n < 4096 && r * (1.0 / std::cos(geometry::kPi / n) - 1.0) > sag) {
            n *= 2;
        }
        const double rc = r / std::cos(geometry::kPi / n);
        for (int i = 0; i < n; ++i) {
            const double ang = 2.0 * geometry::kPi * i / n;
            pts.push_back(c + Vec2{rc * std::cos(ang), rc * std::sin(ang)});
        }
    }
    // Convex hull (monotone chain).
    std::sort(pts.begin(), pts.end(), [](Vec2 x, Vec2 y) { return x.x < y.x || (x.x == y.x && x.y < y.y); });
    std::vector<Vec2> hull(2 * pts.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && (hull[k - 1] - hull[k - 2]).cross(pts[i] - hull[k - 2]) <= 0.0) --k;
        hull[k++] = pts[i];
    }
    for (std::size_t i = pts.size() - 1, lower = k + 1; i-- > 0;) {
        while (k >= lower && (hull[k - 1] - hull[k - 2]).cross(pts[i] - hull[k - 2]) <= 0.0) --k;
        hull[k++] = pts[i];
    }
    hull.resize(k > 0 ? k - 1 : 0);
    return hull;
}

// The loose-piece search (T-0140 round 3): the band polygons cover the true bands and
// reach at most this beyond their arcs (or the chordal tolerance, when smaller), so a neck of
// material narrower than about twice this + 4 um between two convex band arcs can count as cut;
// and the proof that most stretches enclose nothing (detail::pathsToSearch) splits arcs into
// stretches whose sagitta is at most kLooseProofSagitta. A hole of the bands the proof might hide
// lies within the sum of the two (0.017 mm) of the bands: at most 0.034 mm across, where the
// thickness test (erosion by 0.025 mm, its chords at most 0.005 mm inside the arcs) needs more
// than 0.04 mm.
constexpr double kLooseBandSagitta = 0.005;
constexpr double kLooseProofSagitta = 0.012;



// Loose pieces (whenever tabs are on, also when every tab was left out): with the kept tabs, every
// piece of material the tools leave must still join what lies outside it, through some kept tab.
// For every kind of tab (T-0156; T-0140 for Inside/Outside with allowance >= 0, sided tabs were
// searched region by region) the pieces that the bands of every stretch of every cut enclose in the
// whole scene are found at once, leaf by leaf (enclosed_pieces.hpp): each is bounded by cut all
// round, so no kept tab joins it to the outside. A V-bit's corner moves add their own bands.
// Each counts when its area (less its islands) is 0.04 mm² or more and it is thicker than 0.05 mm
// somewhere (what is left of it eroded by 0.025 mm), and is reported against the vector of the
// innermost closed primary cut that contains it, or else of the cut nearest it.
void ProfileBuilder::reportLoosePieces(const std::vector<Cut>& primary, const std::vector<Cut>& secondary) {
    if (!p_.tabs.enabled) {
        return;
    }
    const double rt = tabLayerRadius();
    struct Piece {
        double area{0.0};
        std::size_t source{0};
    };
    std::vector<Piece> found;
    // The stretches of a cut that cut the tab layer: all of it, or (`raised`) all but its
    // raised stretches, whose exact ends stay in (the tool goes back down there).
    auto stretches = [&](const Cut& o, bool raised) {
        std::vector<Contour> out;
        if (!raised || o.tabs.empty()) {
            Contour whole = o.walk.contour();
            whole.closed = false;
            out.push_back(std::move(whole));
            return out;
        }
        std::vector<TabSpan> ts = o.tabs;
        std::sort(ts.begin(), ts.end(), [](const TabSpan& x, const TabSpan& y) { return x.a < y.a; });
        const double len = o.walk.length();
        std::vector<std::pair<double, double>> open;
        if (o.walk.closed()) {
            for (std::size_t i = 0; i < ts.size(); ++i) {
                const double u0 = ts[i].b;
                const double u1 = i + 1 < ts.size() ? ts[i + 1].a : ts.front().a + len;
                open.push_back({u0, u1});
            }
        } else {
            double u = 0.0;
            for (const TabSpan& t : ts) {
                open.push_back({u, t.a});
                u = t.b;
            }
            open.push_back({u, len});
        }
        for (const auto& [u0, u1] : open) {
            if (u1 - u0 <= 1e-9) continue;
            Contour piece;
            piece.closed = false;
            piece.spans = o.walk.extract(u0, u1);
            if (!piece.spans.empty()) out.push_back(std::move(piece));
        }
        return out;
    };
    std::vector<const Cut*> cuts;  // the primary cuts first, then the others
    std::size_t primaryCuts = 0;
    for (const std::vector<Cut>* pass : {&primary, &secondary}) {
        for (const Cut& o : *pass) {
            if (!o.walk.isEmpty()) cuts.push_back(&o);
        }
        if (pass == &primary) primaryCuts = cuts.size();
    }
    // The bands of every stretch (bandOutline; Clipper2's offset for a Bezier span). They are
    // exact, so they reach the tab-layer radius + 2 um only (closing hairline gaps where two
    // bands just touch), without the sagitta of T-0127's sampled positions; their arcs are
    // polygons at most `bandSag` outside them (round 3).
    const double reach = rt + 2e-3;
    const double bandSag = std::min(offTol_.chordal, kLooseBandSagitta);
    std::vector<Contour> paths;
    std::vector<char> whole;
    for (const Cut* o : cuts) {
        const bool closedWhole = o->walk.closed() && o->tabs.empty();
        for (Contour& path : stretches(*o, true)) {
            paths.push_back(std::move(path));
            whole.push_back(closedWhole ? 1 : 0);
        }
    }
    // Most stretches provably enclose nothing (a part held by its tabs, far from other cuts):
    // their bands are not built at all (detail::pathsToSearch, round 3; the proof's pieces
    // reach bandSag + kLooseProofSagitta beyond the true bands, so what they could hide is
    // thinner than the 0.05 mm that counts).
    std::vector<char> search = detail::pathsToSearch(paths, whole, reach + bandSag, kLooseProofSagitta);
    // A V-bit's corner moves (T-0156) are tapered bands off the cut that the proof does not cover:
    // with any, every band is built.
    std::vector<detail::PointRing> bands;
    for (const Cut* o : cuts) {
        for (const Excursion& e : o->excursions) {
            if (!excursionActive(*o, e)) continue;
            for (const Taper& t : e.tapers) {
                detail::PointRing ring = taperRing(t, 2e-3, bandSag);
                if (ring.size() >= 3) bands.push_back(std::move(ring));
            }
            std::fill(search.begin(), search.end(), char{1});
        }
    }
    for (std::size_t i = 0; i < paths.size(); ++i) {
        if (!search[i]) continue;
        const Contour& path = paths[i];
        chargeVertices(path, offTol_.chordal, 20.0);  // work meter (T-0186): the band
        std::vector<detail::PointRing> band = detail::bandOutline(path, reach, bandSag, whole[i] != 0);
        if (!band.empty()) {
            for (detail::PointRing& r : band) bands.push_back(std::move(r));
        } else {
            const std::vector<Contour> banded = geometry::offsetContours({path}, reach, geometry::JoinType::Round,
                                                                         offTol_, p_.corners.mitreLimit);
            detail::pollCancel();  // T-0228
            for (const Contour& b : banded) {
                bands.push_back(b.polygonise(offTol_).points);
            }
        }
    }
    {  // Work meter (T-0186): the union of the bands, about 2 us (60 units) per vertex.
        std::uint64_t vertices = 0;
        for (const detail::PointRing& r : bands) {
            vertices += r.size();
        }
        detail::chargeWork(60 * vertices + 1);
    }
    // The pieces they enclose, leaf by leaf (enclosed_pieces.hpp: one union over the whole
    // scene is a single sweep over every band side by side, quadratic in a row or grid of
    // shapes whose kerfs overlap).
    detail::EnclosedOptions opt;
    opt.minArea = 0.04;
    opt.clearance = 0.025;
    opt.tol = offTol_;
    opt.mitreLimit = p_.corners.mitreLimit;
    const std::vector<detail::EnclosedPiece> pieces =
        bands.empty() ? std::vector<detail::EnclosedPiece>{} : detail::enclosedPieces(std::move(bands), opt);
    // (The cuts' boxes, for attribution, only when there is something to attribute.)
    std::vector<geometry::Box2> cutBox;
    if (!pieces.empty()) {
        for (const Cut* o : cuts) cutBox.push_back(o->walk.contour().bounds());
    }
    const BoxIndex cutIndex(cutBox);
    std::vector<double> cutArea(cuts.size(), -1.0);  // of the closed primary cuts, when needed
    for (const detail::EnclosedPiece& e : pieces) {
        if (!e.inner) continue;  // not thicker than 0.05 mm anywhere
        // Attributed to the innermost closed primary cut that contains it (any point inside the
        // piece gives the same answer), or else to the cut nearest its anchor (a vertex of the
        // piece, the same at every leaf size; it lies on bands of the cuts round it, so cuts
        // within 0.01 mm of the nearest count as tied and the first of them is taken).
        const Vec2 probe = *e.inner;
        std::size_t src = cuts.front()->sources.front();
        double loopArea = std::numeric_limits<double>::infinity();
        for (std::size_t k : cutIndex.containing(probe)) {
            const Cut* o = cuts[k];
            if (k >= primaryCuts || !o->walk.closed()) continue;
            if (cutArea[k] < 0.0) cutArea[k] = geometry::area(o->walk.contour());
            const double a = cutArea[k];
            if (a < loopArea && geometry::locatePoint(o->walk.contour(), probe, geometry::FillRule::EvenOdd, tol_) ==
                                    geometry::PointLocation::Inside) {
                loopArea = a;
                src = o->sources.front();
            }
        }
        if (loopArea == std::numeric_limits<double>::infinity()) {
            // The nearest: the cuts whose boxes reach within `rad` of the anchor, `rad` doubled
            // until the nearest of them and its ties lie within it (so no other can be nearer).
            constexpr double kTie = 0.01;
            const Vec2 anchor = e.anchor;
            double rad = std::max(1.0, 4.0 * reach);
            for (;;) {
                const std::vector<std::size_t> nearby =
                    cutIndex.query(geometry::Box2{anchor - Vec2{rad, rad}, anchor + Vec2{rad, rad}});
                std::vector<std::pair<std::size_t, double>> dist;
                double best = std::numeric_limits<double>::infinity();
                for (std::size_t k : nearby) {
                    dist.emplace_back(k, geometry::distanceTo(cuts[k]->walk.contour(), anchor));
                    best = std::min(best, dist.back().second);
                }
                if (best + kTie <= rad || nearby.size() == cuts.size()) {
                    std::size_t pick = cuts.size();
                    for (const auto& [k, d] : dist) {
                        if (d <= best + kTie) pick = std::min(pick, k);
                    }
                    if (pick < cuts.size()) src = cuts[pick]->sources.front();
                    break;
                }
                rad *= 2.0;
            }
        }
        found.push_back(Piece{e.area, src});
    }
    // One message per piece, numbered per vector (largest first), so no two are alike and the
    // de-duplication in run() keeps every one; beyond kMaxPieceMessages per vector, one more
    // message sums up the rest.
    constexpr std::size_t kMaxPieceMessages = 50;
    std::map<std::size_t, std::vector<double>> loose;  // vector -> its loose pieces (mm²)
    for (const Piece& f : found) loose[f.source].push_back(f.area);
    for (auto& [src, areas] : loose) {
        std::sort(areas.begin(), areas.end(), std::greater<double>());
        const std::size_t shown = std::min(areas.size(), kMaxPieceMessages);
        for (std::size_t k = 0; k < shown; ++k) {
            char buf[32];
            std::snprintf(buf, sizeof buf, "%.2f", areas[k]);
            std::string text = vecName(src) + ": loose piece " + std::to_string(k + 1) + " of " +
                               std::to_string(areas.size()) + ": a piece of material inside the cut (about " +
                               buf + " square mm) is not held by any kept tab and comes free";
            diag(ProfileIssue::TabsNotPlaced, src, text);
        }
        if (areas.size() > shown) {
            const double rest = std::accumulate(areas.begin() + static_cast<std::ptrdiff_t>(shown), areas.end(), 0.0);
            char buf[32];
            std::snprintf(buf, sizeof buf, "%.2f", rest);
            const std::size_t more = areas.size() - shown;
            diag(ProfileIssue::TabsNotPlaced, src,
                 vecName(src) + ": and " + std::to_string(more) +
                     (more == 1 ? std::string(" more loose piece (about ") + buf +
                                      " square mm) is not held by any kept tab and comes free"
                                : std::string(" more loose pieces (about ") + buf +
                                      " square mm in all) are not held by any kept tab and come free"));
        }
    }
}

// Points (every 0.1 mm) along the stretch of each tab that leaves material: its footprint minus a
// tool radius at each end (the raised stretch includes the tool diameter).
void ProfileBuilder::collectTabMaterial(const std::vector<Cut>& cuts) {
    const double ramp = p_.tabs.threeD ? p_.tabs.thickness : 0.0;
    for (const Cut& c : cuts) {
        for (const TabSpan& t : c.tabs) {
            const double a = t.a - ramp + radius_;
            const double b = t.b + ramp - radius_;
            if (b <= a) {
                continue;
            }
            const int n = std::max(1, static_cast<int>(std::ceil((b - a) / 0.1)));
            for (int i = 0; i <= n; ++i) {
                tabMaterial_.push_back(c.walk.pointAt(a + (b - a) * i / n));
            }
        }
    }
}

// Lowest height a lead may run at: a lead whose tool comes within reach of tab material (closer
// than a tool diameter, plus the 0.1 mm sampling) runs no lower than the tab top, so it never
// notches a tab; the plunge to the level then happens at the cut start, which tabs keep clear.
double ProfileBuilder::leadFloor(const Contour& lead) const {
    if (lead.isEmpty() || tabMaterial_.empty()) {
        return kNoCeiling;
    }
    const double reach = 2.0 * radius_ + 0.1;
    const ArcLengthPath lw(lead, offTol_);
    const int n = std::max(1, static_cast<int>(std::ceil(lw.length() / 0.1)));
    const geometry::Box2 box = lead.bounds().inflated(reach);
    for (const Vec2& m : tabMaterial_) {
        if (!box.contains(m)) {
            continue;
        }
        for (int i = 0; i <= n; ++i) {
            if (geometry::distance(lw.pointAt(lw.length() * i / n), m) < reach) {
                return passes_.bottomZ() + p_.tabs.thickness;
            }
        }
    }
    return kNoCeiling;
}

// One diagnostic per vector with the totals over all its cuts (so kept + left out adds up).
void ProfileBuilder::reportTabs() {
    for (const auto& [src, n] : tabsDropped_) {
        if (n > 0) {
            // Reasons, most specific first; the rest had no room where they were placed.
            const std::size_t severed = tabsSevered_[src];
            const std::size_t unmatched = tabsUnmatched_[src];
            const std::size_t empty = tabsEmpty_[src];
            const std::size_t noRoom = n - severed - unmatched - empty;
            std::string why;
            auto add = [&](std::size_t k, const char* text) {
                if (k == 0) return;
                why += (why.empty() ? "" : "; ") + std::to_string(k) + " " + text;
            };
            add(noRoom, "with no room (they would overlap, run past an open end, cover where the tool plunges or "
                        "leave no stretch cut to full depth)");
            add(unmatched, "could not be matched on the roughing cut");
            add(severed, "would be cut through by the tool (at a corner or turn of the cut, or by another stretch "
                         "of a cut)");
            add(empty, "would hold nothing (the cut removes all the material on one side of them)");
            diag(ProfileIssue::TabsNotPlaced, src,
                 vecName(src) + ": " + std::to_string(n) + " of " + std::to_string(tabsRequested_[src]) +
                     " tab(s) left out (" + why + ")");
        }
    }
    for (const auto& [src, n] : tabsOnCorner_) {
        if (n > 0) {
            diag(ProfileIssue::TabsNotPlaced, src,
                 vecName(src) + ": " + std::to_string(n) +
                     " tab(s) left on a corner or curve (no straight stretch long enough)");
        }
    }
}

// Lead options to try, in order: as asked, then a gentler variant that clears an adjacent wall
// more often (straight: perpendicular; circular: half the sweep).
std::vector<LeadOptions> ProfileBuilder::leadVariants() const {
    std::vector<LeadOptions> v{p_.leads};
    LeadOptions alt = p_.leads;
    if (alt.type == LeadType::Straight && alt.angle != geometry::kPi / 2.0) {
        alt.angle = geometry::kPi / 2.0;
        v.push_back(alt);
    } else if (alt.type == LeadType::Circular) {
        alt.sweep *= 0.5;
        v.push_back(alt);
    }
    return v;
}

// True (and the lead in `out`) when some variant of `make(options, scale)` stays at least the tool
// radius from every vector.
template <class Make>
bool ProfileBuilder::leadFits(Make make, double scale, Contour& out, bool& gentler) const {
    const std::vector<LeadOptions> variants = leadVariants();
    for (std::size_t k = 0; k < variants.size(); ++k) {
        Contour cand = make(variants[k], scale);
        if (cand.isEmpty()) {
            continue;
        }
        if (p_.side == ProfileSide::On ||
            isClear(ArcLengthPath(cand, offTol_), allField_, radius_ + std::max(0.0, p_.allowance) - guardTol(),
                    guardTol())) {
            out = std::move(cand);
            gentler = k > 0;
            return true;
        }
    }
    return false;
}

// The lead `make(options, scale)` at full size, or halved up to three times until it stays at
// least the tool radius from every vector; empty (with a diagnostic) when even the smallest gouges.
template <class Make>
Contour ProfileBuilder::fitLead(Make make, std::size_t contour, const char* what) {
    for (double scale : {1.0, 0.5, 0.25, 0.125}) {
        Contour lead;
        bool gentler = false;
        if (leadFits(make, scale, lead, gentler)) {
            if (scale < 1.0 || gentler) {
                diag(ProfileIssue::LeadShortened, contour,
                     vecName(contour) + ": " + what + (gentler ? " made gentler" : "") +
                         (scale < 1.0 ? " shortened" : "") + " to keep clear of the vectors");
            }
            return lead;
        }
    }
    diag(ProfileIssue::LeadDropped, contour, vecName(contour) + ": " + what + " left out (it would gouge a vector)");
    return {};
}

// Lead-ins are planned before tabs: when no lead fits at the chosen start of a closed cut (for
// example an inside corner, where the approach would cross the adjacent wall), the start is moved
// to the middle of a span (the start span first, then the others from the longest). At each start
// the lead is tried at full size and then halved up to three times.
void ProfileBuilder::planLeadIns(std::vector<Cut>& cuts) {
    if (p_.leads.type == LeadType::None) {
        return;
    }
    for (Cut& c : cuts) {
        // A closed cut that collapsed to a point when its start was chosen (an offset of a tiny
        // vector that closes within the tolerance) has no spans; it emits nothing, so it needs no
        // lead (T-0163).
        if (c.walk.isEmpty()) {
            continue;
        }
        const std::size_t src = c.sources.front();
        const bool wasteLeft = !c.contourOnLeft;
        std::vector<double> starts{0.0};
        if (c.walk.closed()) {
            std::vector<std::size_t> spans(c.walk.spanCount());
            std::iota(spans.begin(), spans.end(), std::size_t{0});
            std::stable_sort(spans.begin() + 1, spans.end(), [&](std::size_t x, std::size_t y) {
                return c.walk.spanLength(x) > c.walk.spanLength(y);
            });
            for (std::size_t i : spans) {
                starts.push_back(c.walk.spanStart(i) + 0.5 * c.walk.spanLength(i));
            }
        }
        bool found = false;
        for (double u : starts) {
            for (double scale : {1.0, 0.5, 0.25, 0.125}) {
                const Vec2 at = c.walk.pointAt(u);
                const Vec2 t = c.walk.tangentAt(u);
                Contour lead;
                bool gentler = false;
                if (!leadFits([&](const LeadOptions& o, double sc) { return leadIn(at, t, wasteLeft, o, sc); }, scale,
                              lead, gentler)) {
                    continue;
                }
                if (u != 0.0) {
                    c.path.spans = c.walk.extract(u, u + c.walk.length());
                    closeExactly(c.path);
                    c.walk = ArcLengthPath(c.path, offTol_);
                    diag(ProfileIssue::StartMoved, src,
                         vecName(src) + ": start moved along the cut so that the lead-in fits");
                }
                if (scale < 1.0 || gentler) {
                    diag(ProfileIssue::LeadShortened, src,
                         vecName(src) + ": lead-in" + (gentler ? " made gentler" : "") +
                             (scale < 1.0 ? " shortened" : "") + " to keep clear of the vectors");
                }
                c.leadIn = std::move(lead);
                found = true;
                break;
            }
            if (found) {
                break;
            }
        }
        if (!found) {
            diag(ProfileIssue::LeadDropped, src, vecName(src) + ": lead-in left out (it would gouge a vector)");
        }
    }
}

// How far the V-bit's cone stays inside the walls with the tip at `p`, `tipZ`: distance from p to
// the nearest vector, minus the allowance, minus the radius the cone cuts at the material surface
// (Z = 0; capped at the tool radius). Negative = it would cut past a wall.
double ProfileBuilder::coneClearance(Vec2 p, double tipZ) const {
    const double cone = std::min(std::max(0.0, -tipZ) * vbitTan_, 0.5 * tool_.diameter);
    return allField_.distance(p) - std::max(0.0, p_.allowance) - cone;
}

// The sharp-internal-corner excursion for a V-bit at the path's inside corner `q` (tip at
// `bottom`), returning to q; empty when there is no genuine corner of the vector there.
//
// The corner V must be a junction of a source vector whose direction turns by more than 5° (so
// polyline vertices of polygonised curves and merged-region offsets do not count), on an exact
// offset (merged polygonal regions get none). The tip runs from q, at the "knee" depth where the
// cone itself is r wide, towards V and rises linearly to the surface where the distance to the
// walls equals the allowance (V itself when the allowance is 0). That straight climb is exact
// between two straight walls; it is checked against the real walls every 0.005 mm, and when it
// would cut past one (a curved wall next to the corner) the climb follows the walls instead: at
// samples 0.05 mm apart the tip is held at (distance - allowance - 0.05) / tan(half angle) below
// the surface, which keeps the cone inside the walls between samples too (distance changes by at
// most the sample spacing).
std::vector<Move> ProfileBuilder::cornerMoves(const Cut& cut, Vec2 q, double turn, double bottom) const {
    constexpr double kCornerTurn = 0.0872664625997165;  // 5 degrees
    // The path itself must turn sharply here: polyline vertices of polygonised curves turn by
    // fractions of a degree and are not corners.
    if (!cut.exact || p_.side == ProfileSide::On || std::fabs(turn) <= kCornerTurn) {
        return {};
    }
    Vec2 v = q;
    double best = std::numeric_limits<double>::infinity();
    for (std::size_t s : cut.sources) {
        const Contour& c = in_.contours[s];
        const std::size_t n = c.spans.size();
        for (std::size_t j = 0; j < n; ++j) {
            if (j == 0 && !c.closed) {
                continue;
            }
            const Span& prev = c.spans[(j + n - 1) % n];
            const Span& next = c.spans[j];
            const Vec2 a = geometry::tangentAt(prev, 1.0);
            const Vec2 b = geometry::tangentAt(next, 0.0);
            if (std::fabs(std::atan2(a.cross(b), a.dot(b))) <= kCornerTurn) {
                continue;
            }
            const Vec2 corner = geometry::startPoint(next);
            const double d = geometry::distance(corner, q);
            if (d < best) {
                best = d;
                v = corner;
            }
        }
    }
    const double r = radius_;
    const double a = std::max(0.0, p_.allowance);
    // Between straight walls turning by `turn`, Q lies (r + a) / cos(turn / 2) from the corner.
    // Accept the nearest genuine corner only if it is about that far (twice, for curved walls).
    const double expected = (r + a) / std::cos(0.5 * std::min(std::fabs(turn), geometry::kPi - kCornerTurn));
    if (!(best <= 2.0 * expected) || best <= tol_.distance) {
        return {};
    }
    const double knee = std::max(bottom, -r / vbitTan_);
    // Where the climb ends: the point towards V whose distance to the walls equals the allowance.
    Vec2 target = v;
    if (a > 0.0) {
        double lo = 0.0;
        double hi = 1.0;
        for (int it = 0; it < 60; ++it) {
            const double mid = 0.5 * (lo + hi);
            (allField_.distance(geometry::lerp(q, v, mid)) > a ? lo : hi) = mid;
        }
        target = geometry::lerp(q, v, lo);
    }
    const double len = geometry::distance(q, target);
    bool straight = true;
    const int checks = std::max(2, static_cast<int>(std::ceil(len / 0.005)));
    for (int i = 0; i <= checks && straight; ++i) {
        const double s = static_cast<double>(i) / checks;
        straight = coneClearance(geometry::lerp(q, target, s), knee * (1.0 - s)) >= -1e-9;
    }
    std::vector<Move> up;
    if (straight) {
        if (knee > bottom) {
            up.push_back(Move::feed(Vec3{q, knee}));
        }
        up.push_back(Move::feed(Vec3{target, 0.0}));
    } else {
        constexpr double h = 0.05;
        auto heldZ = [&](Vec2 p) {
            const double room = allField_.distance(p) - a - h;
            return std::max(knee, -std::max(0.0, room) / vbitTan_);
        };
        up.push_back(Move::feed(Vec3{q, heldZ(q)}));
        const int n = std::max(1, static_cast<int>(std::ceil(len / h)));
        for (int i = 1; i <= n; ++i) {
            const Vec2 p = geometry::lerp(q, target, static_cast<double>(i) / n);
            up.push_back(Move::feed(Vec3{p, heldZ(p)}));
        }
    }
    // Back down the same way, then to the bottom at q.
    // Vertical descents are plunges; sloped ones (along the bisector) are feeds.
    std::vector<Move> moves = up;
    auto down = [&](Vec3 to) {
        const Vec3 from = moves.back().to;
        if (from == to) {
            return;  // no zero-length moves
        }
        moves.push_back(from.xy() == to.xy() ? Move::plunge(to) : Move::feed(to));
    };
    for (std::size_t i = up.size() - 1; i-- > 0;) {
        down(up[i].to);
    }
    down(Vec3{q, bottom});  // straight climb without a knee: one sloped feed back to q
    return moves;
}

// The positions (distance travelled from the leg's start) at which a leg passes the corner at arc
// length `u` of the cut: one per lap of a closed cut within the leg.
std::vector<double> kinkLambdas(const ArcLengthPath& walk, const PathLeg& leg, double u) {
    const double len = walk.length();
    const double dir = leg.u1 >= leg.u0 ? 1.0 : -1.0;
    std::vector<double> lambdas;
    if (walk.closed()) {
        for (double m = std::floor((std::min(leg.u0, leg.u1) - u) / len) - 1.0;
             u + m * len <= std::max(leg.u0, leg.u1) + 1e-9; m += 1.0) {
            lambdas.push_back((u + m * len - leg.u0) * dir);
        }
    } else {
        lambdas.push_back((u - leg.u0) * dir);
    }
    return lambdas;
}

// The legs a cut is walked in, in order, as emit() makes them (ramps, the spiral's extra lap, the
// overcut).
std::vector<ProfileBuilder::LegStep> ProfileBuilder::legsOf(const Cut& cut, const std::vector<double>& levels,
                                                            bool finishing, double* endU) const {
    std::vector<LegStep> out;
    if (endU != nullptr) {
        *endU = 0.0;
    }
    const ArcLengthPath& walk = cut.walk;
    if (walk.isEmpty() || levels.empty()) {
        return out;
    }
    const double len = walk.length();
    const double z0 = 0.0 - passes_.startDepth;
    RampOptions ramp = finishing ? RampOptions{} : p_.ramp;
    if (ramp.type == RampType::Spiral && !walk.closed()) {
        ramp.type = RampType::Smooth;
    }
    const bool rampFirst = ramp.type != RampType::None && levels.front() < z0;
    double zFrom = levels.front();
    if (rampFirst && !(!cut.leadIn.isEmpty() && ramp.onLeadIn)) {
        zFrom = z0;
    }
    double u = 0.0;
    for (std::size_t k = 0; k < levels.size(); ++k) {
        const bool last = k + 1 == levels.size();
        const PassLegs plan = planPass(walk, u, zFrom, levels[k], ramp);
        for (const PathLeg& leg : plan.legs) {
            out.push_back({leg, last});
        }
        u = plan.endU;
        zFrom = levels[k];
    }
    const double zn = levels.back();
    if (walk.closed() && ramp.type == RampType::Spiral && levels.front() < z0) {
        out.push_back({PathLeg{u, u + len, ZProfile::flat(zn)}, true});
    }
    if (walk.closed() && p_.leads.overcut > 0.0) {
        out.push_back({PathLeg{u, u + p_.leads.overcut, ZProfile::flat(zn)}, true});
        u += p_.leads.overcut;
    }
    if (endU != nullptr) {
        *endU = u;
    }
    return out;
}

// The tab layer's view of corner moves `moves` made from (q, bottom): for every move the cone's
// disc there (radius (tab top - tip height) * tan(half angle), at most the tool radius, nothing
// once the tip is above the tab top) runs linearly from one end to the other, so what it sweeps is
// the convex hull of the two end discs.
std::vector<Taper> ProfileBuilder::tapersOf(Vec2 q, double bottom, const std::vector<Move>& moves) const {
    const double zTab = bottom + p_.tabs.thickness;
    auto radiusAt = [&](double z) { return std::min(radius_, std::max(0.0, (zTab - z) * vbitTan_)); };
    std::vector<Taper> out;
    geometry::Vec3 from{q, bottom};
    for (const Move& m : moves) {
        const geometry::Vec3 to = m.to;
        if (from.z < zTab || to.z < zTab) {
            Vec2 a = from.xy();
            Vec2 b = to.xy();
            double za = from.z;
            double zb = to.z;
            if (za >= zTab || zb >= zTab) {  // only the stretch below the tab top
                const double t = (zTab - za) / (zb - za);
                const Vec2 c = geometry::lerp(a, b, t);
                if (za >= zTab) {
                    a = c;
                    za = zTab;
                } else {
                    b = c;
                    zb = zTab;
                }
            }
            out.push_back({a, b, radiusAt(za), radiusAt(zb)});
        }
        from = to;
    }
    return out;
}

// V-bit sharp-corner moves of every cut, as emitLeg makes them. Only the corners the cut passes
// at full depth in some leg have any (a ramp can hide a corner); whether a kept tab covers the
// corner is decided later (excursionActive).
void ProfileBuilder::planExcursions(std::vector<Cut>& cuts, const std::vector<double>& levels, bool finishing) const {
    if (!sharpInternal_) {
        return;
    }
    const double bottom = passes_.bottomZ();
    for (Cut& cut : cuts) {
        cut.excursions.clear();
        if (cut.walk.isEmpty()) {
            continue;
        }
        const auto kinks = cut.walk.kinks(1e-3);
        std::vector<char> visited(kinks.size(), 0);
        for (const LegStep& step : legsOf(cut, levels, finishing)) {
            if (!step.last) {
                continue;
            }
            const double travel = std::fabs(step.leg.u1 - step.leg.u0);
            for (std::size_t k = 0; k < kinks.size(); ++k) {
                if (visited[k] || !(cut.contourOnLeft ? kinks[k].turn < 0.0 : kinks[k].turn > 0.0)) {
                    continue;
                }
                for (double lam : kinkLambdas(cut.walk, step.leg, kinks[k].u)) {
                    if (lam < -1e-9 || lam > travel + 1e-9 || std::fabs(step.leg.z.after(lam) - bottom) > 1e-9) {
                        continue;
                    }
                    visited[k] = 1;
                    break;
                }
            }
        }
        for (std::size_t k = 0; k < kinks.size(); ++k) {
            if (!visited[k]) {
                continue;
            }
            const Vec2 q = cut.walk.pointAt(kinks[k].u);
            std::vector<Move> moves = cornerMoves(cut, q, kinks[k].turn, bottom);
            if (moves.empty()) {
                continue;
            }
            Excursion e;
            e.kink = k;
            e.u = kinks[k].u;
            e.tapers = p_.tabs.enabled ? tapersOf(q, bottom, moves) : std::vector<Taper>{};
            e.moves = std::move(moves);
            cut.excursions.push_back(std::move(e));
        }
    }
}

bool ProfileBuilder::excursionActive(const Cut& cut, const Excursion& e) const {
    if (cut.tabs.empty()) {
        return true;
    }
    const double len = cut.walk.length();
    const double ramp = p_.tabs.threeD ? std::max(0.0, p_.tabs.thickness) : 0.0;
    for (const TabSpan& t : cut.tabs) {
        for (double x : {e.u - len, e.u, e.u + len}) {
            if (x > t.a - ramp + 1e-9 && x < t.b + ramp - 1e-9) {
                return false;
            }
            if (!cut.walk.closed()) {
                break;
            }
        }
    }
    return true;
}

void ProfileBuilder::emitLeg(const Cut& cut, const PathLeg& leg, bool lastLevel, std::vector<bool>& cornerDone) {
    const double bottom = passes_.bottomZ();
    ZProfile z = leg.z;
    if (!cut.tabs.empty()) {
        const double zTab = bottom + p_.tabs.thickness;
        z = ZProfile::max(z, tabCeiling(cut.walk, leg.u0, leg.u1, cut.tabs, zTab, bottom, p_.tabs.threeD));
    }
    std::vector<LegInsert> inserts;
    if (lastLevel && sharpInternal_) {
        const double travel = std::fabs(leg.u1 - leg.u0);
        const auto kinks = cut.walk.kinks(1e-3);
        for (std::size_t k = 0; k < kinks.size(); ++k) {
            const bool inside = cut.contourOnLeft ? kinks[k].turn < 0.0 : kinks[k].turn > 0.0;
            if (!inside || cornerDone[k]) {
                continue;
            }
            for (double lam : kinkLambdas(cut.walk, leg, kinks[k].u)) {
                if (lam < -1e-9 || lam > travel + 1e-9 || std::fabs(z.after(lam) - bottom) > 1e-9) {
                    continue;
                }
                // (The moves were planned with the cut: planExcursions; a corner without any is
                // done with nothing to insert.)
                const auto e = std::find_if(cut.excursions.begin(), cut.excursions.end(),
                                            [&](const Excursion& x) { return x.kink == k; });
                if (e != cut.excursions.end()) {
                    inserts.push_back({lam, e->moves});
                }
                cornerDone[k] = true;
                break;
            }
        }
    }
    appendLeg(cut.walk, leg.u0, leg.u1, z, result_.toolpath, inserts);
    chargeMoves();
}

void ProfileBuilder::emit(const Cut& cut, const std::vector<double>& levels, bool finishing) {
    Toolpath& out = result_.toolpath;
    const ArcLengthPath& walk = cut.walk;
    if (walk.isEmpty() || levels.empty()) {
        return;
    }
    const double len = walk.length();
    const double z0 = 0.0 - passes_.startDepth;  // top of the cut (never -0)
    RampOptions ramp = finishing ? RampOptions{} : p_.ramp;
    if (ramp.type == RampType::Spiral && !walk.closed()) {
        diag(ProfileIssue::SpiralNeedsClosed, cut.sources.front(),
             vecName(cut.sources.front()) + ": spiral ramp needs a closed path; smooth ramp used");
        ramp.type = RampType::Smooth;
    }
    const bool rampFirst = ramp.type != RampType::None && levels.front() < z0;
    const std::size_t src = cut.sources.front();

    const Contour& in = cut.leadIn;  // planned and checked by planLeadIns
    const Vec2 entry = in.isEmpty() ? walk.pointAt(0.0) : in.startPoint();
    out.append(Move::rapid(Vec3{entry, p_.safeZ}));
    if (!in.isEmpty()) {
        const ArcLengthPath lw(in, offTol_);
        const ZProfile floor = ZProfile::flat(leadFloor(in));  // keeps the lead out of tabs
        if (rampFirst && ramp.onLeadIn) {
            const ZProfile z = ZProfile::max(ZProfile::linear(z0, levels.front(), lw.length()), floor);
            out.append(Move::plunge(Vec3{entry, z.after(0.0)}));
            appendLeg(lw, 0.0, lw.length(), z, out);
        } else if (rampFirst) {
            const ZProfile z = ZProfile::max(ZProfile::flat(z0), floor);
            out.append(Move::plunge(Vec3{entry, z.after(0.0)}));
            appendLeg(lw, 0.0, lw.length(), z, out);
        } else {
            const ZProfile z = ZProfile::max(ZProfile::flat(levels.front()), floor);
            out.append(Move::plunge(Vec3{entry, z.after(0.0)}));
            appendLeg(lw, 0.0, lw.length(), z, out);
        }
    } else if (rampFirst) {
        out.append(Move::plunge(Vec3{entry, z0}));
    } else {
        out.append(Move::plunge(Vec3{entry, levels.front()}));
    }

    std::vector<bool> cornerDone(walk.kinks(1e-3).size(), false);
    double u = 0.0;
    for (const LegStep& step : legsOf(cut, levels, finishing, &u)) {
        emitLeg(cut, step.leg, step.last, cornerDone);
    }

    // Lead-out from where the cut ends, in the direction of travel there.
    if (p_.leads.type != LeadType::None && p_.leads.leadOut) {
        Vec2 at = walk.pointAt(u);
        Vec2 t = walk.tangentAt(u);
        bool wasteLeft = !cut.contourOnLeft;
        if (!walk.closed()) {
            const bool forward = u >= 0.5 * len;  // ended at the far end: travelled forwards
            at = walk.pointAt(forward ? len : 0.0);
            t = forward ? walk.tangentAt(len) : -walk.tangentAt(0.0);
            wasteLeft = forward ? !cut.contourOnLeft : cut.contourOnLeft;
        }
        const Contour lo =
            fitLead([&](const LeadOptions& o, double scale) { return leadOut(at, t, wasteLeft, o, scale); }, src,
                    "lead-out");
        if (!lo.isEmpty()) {
            // Leave at the height the cut ended at: inside a tab (e.g. after an overcut) that is
            // the tab top, so the lead-out never cuts into the tab.
            const ArcLengthPath lw(lo, offTol_);
            const double z = std::max(out.moves.back().to.z, leadFloor(lo));
            if (z > out.moves.back().to.z) {
                out.append(Move::feed(Vec3{out.moves.back().to.xy(), z}));  // up out of the way of a tab
            }
            appendLeg(lw, 0.0, lw.length(), ZProfile::flat(z), out);
        }
    }
    out.append(Move::retract(Vec3{out.moves.back().to.xy(), p_.safeZ}));
    chargeMoves();
}

ProfileResult ProfileBuilder::run() {
    result_.toolpath.name = p_.name;
    if (!validate()) {
        return std::move(result_);
    }
    result_.toolpath.tool = toolSnapshot(tool_, cd_);

    prepareInputs();
    detail::pollCancel();
    const std::vector<double> levels = passes_.levels();
    const bool separate = p_.separateLastPass;
    std::vector<Cut> rough = buildCuts(separate ? p_.lastPassAllowance : 0.0, false);
    std::vector<Cut> finish;
    if (separate) {
        finish = buildCuts(0.0, true);
    }
    detail::pollCancel();
    chooseStarts(rough);
    reportVanished(separate ? "roughing" : "cutting");
    chooseStarts(finish);
    if (separate) {
        reportVanished("finishing");
    }
    auto applyOptimisedStarts = [&](std::vector<Cut>& cuts, std::vector<std::size_t>& ord, const char* pass) {
        if (p_.startAt != StartAt::Optimise) {
            return;
        }
        Vec2 prev = p_.startPosition;
        for (std::size_t i : ord) {
            Cut& c = cuts[i];
            if (c.walk.closed()) {
                const double u = startOn(c, c.walk, prev);
                c.path.spans = c.walk.extract(u, u + c.walk.length());
                closeExactly(c.path);
                c.walk = ArcLengthPath(c.path, offTol_);
            }
            prev = c.walk.pointAt(0.0);
        }
        // A closed cut that collapsed to a point has no spans left: it is dropped here, and
        // reported if it was the vector's only cut in this pass (T-0165).
        lostCut_.assign(lostCut_.size(), false);
        keptCut_.assign(keptCut_.size(), false);
        std::vector<std::size_t> newIndex(cuts.size(), cuts.size());
        std::vector<Cut> kept;
        for (std::size_t i = 0; i < cuts.size(); ++i) {
            const bool empty = cuts[i].walk.contour().isEmpty();
            for (std::size_t src : cuts[i].sources) {
                (empty ? lostCut_ : keptCut_)[src] = true;
            }
            if (!empty) {
                newIndex[i] = kept.size();
                kept.push_back(std::move(cuts[i]));
            }
        }
        const bool dropped = kept.size() != cuts.size();
        cuts = std::move(kept);
        if (dropped) {
            std::vector<std::size_t> remapped;
            for (std::size_t i : ord) {
                if (newIndex[i] != newIndex.size()) {
                    remapped.push_back(newIndex[i]);
                }
            }
            ord = std::move(remapped);
            reportVanished(pass);
        }
    };
    std::vector<std::size_t> roughOrder = order(rough);
    applyOptimisedStarts(rough, roughOrder, separate ? "roughing" : "cutting");
    // Finishing cuts follow the roughing order: each takes the position of the roughing cut
    // nearest to it (they lie the last-pass allowance apart).
    std::vector<std::size_t> finishOrder(finish.size());
    {
        std::vector<std::size_t> rank(rough.size());
        for (std::size_t k = 0; k < roughOrder.size(); ++k) {
            rank[roughOrder[k]] = k;
        }
        // Counterpart = the roughing cut with the smallest median distance over 17 points along
        // the finishing cut (robust where trimmed or merged pieces lie close together), if that
        // median is within the last-pass allowance + a tool radius. A finishing cut without one
        // (e.g. a small loop that vanished at the roughing distance) takes the rank of the
        // roughing cut nearest its start point.
        //
        // Indexed (T-0131) with the same result as trying every roughing cut in index order: a
        // median within `match` needs 9 of the 17 points within `match` of the roughing cut, so
        // only cuts whose box, grown by `match`, meets the points' box can supply it (the others
        // have medians above `match` and never win). The nearest start is found by a square
        // search round the start, grown ×4 until the best distance found is below the square's
        // half-width by a tolerance (every cut outside the square is farther) or the square
        // covers every cut. Candidates are visited in ascending order, so ties go as before.
        std::vector<double> key(finish.size(), static_cast<double>(rough.size()));
        const double match = p_.lastPassAllowance + radius_ + tol_.distance;
        const double margin = indexSlack_;
        std::vector<geometry::Box2> roughBoxes(rough.size());
        for (std::size_t r = 0; r < rough.size(); ++r) {
            roughBoxes[r] = rough[r].walk.contour().bounds();
        }
        const BoxIndex roughIndex(std::move(roughBoxes));
        for (std::size_t f = 0; f < finish.size(); ++f) {
            double best = std::numeric_limits<double>::infinity();
            const Vec2 start = finish[f].walk.pointAt(0.0);
            std::array<Vec2, 17> pts;
            geometry::Box2 ptsBox;
            for (int k = 0; k <= 16; ++k) {
                pts[static_cast<std::size_t>(k)] = finish[f].walk.pointAt(finish[f].walk.length() * k / 16.0);
                ptsBox.expand(pts[static_cast<std::size_t>(k)]);
            }
            for (std::size_t r : roughIndex.query(ptsBox.inflated(match + margin))) {
                std::vector<double> ds;
                detail::chargeWork(17 * (4 * rough[r].walk.spanCount() + 1));  // work meter (T-0186)
                for (const Vec2& p : pts) {
                    ds.push_back(geometry::distanceTo(rough[r].walk.contour(), p));
                }
                std::nth_element(ds.begin(), ds.begin() + 8, ds.end());
                if (ds[8] < best) {
                    best = ds[8];
                    key[f] = static_cast<double>(rank[r]);
                }
            }
            if (!(best <= match)) {
                double nearestStart = std::numeric_limits<double>::infinity();
                double startKey = static_cast<double>(rough.size());
                for (double h = match + 2.0 * margin;; h *= 4.0) {
                    const geometry::Box2 square{start - Vec2{h, h}, start + Vec2{h, h}};
                    const bool all = !(h < std::numeric_limits<double>::max()) ||
                                     square.contains(roughIndex.bounds()) || roughIndex.bounds().isEmpty();
                    nearestStart = std::numeric_limits<double>::infinity();
                    startKey = static_cast<double>(rough.size());
                    const std::vector<std::size_t> found =
                        all ? roughIndex.query(geometry::Box2{Vec2{-std::numeric_limits<double>::max(),
                                                                   -std::numeric_limits<double>::max()},
                                                              Vec2{std::numeric_limits<double>::max(),
                                                                   std::numeric_limits<double>::max()}})
                            : roughIndex.query(square);
                    for (std::size_t r : found) {
                        detail::chargeWork(4 * rough[r].walk.spanCount() + 1);
                        const double ds0 = geometry::distanceTo(rough[r].walk.contour(), start);
                        if (ds0 < nearestStart) {
                            nearestStart = ds0;
                            startKey = static_cast<double>(rank[r]);
                        }
                    }
                    if (all || nearestStart + margin < h) {
                        break;
                    }
                }
                key[f] = startKey;
            }
        }
        std::iota(finishOrder.begin(), finishOrder.end(), std::size_t{0});
        std::stable_sort(finishOrder.begin(), finishOrder.end(),
                         [&](std::size_t x, std::size_t y) { return key[x] < key[y]; });
    }
    applyOptimisedStarts(finish, finishOrder, "finishing");
    detail::pollCancel();
    planLeadIns(rough);
    planLeadIns(finish);
    planExcursions(rough, levels, false);
    planExcursions(finish, {levels.back()}, true);
    if (separate) {
        // Three candidates, each placed, matched and checked: (A) exactly T-0127's matching: the
        // tabs placed as T-0127 placed them (a tab moved off the other pass's start goes to exactly
        // the gap; T-0127 let rounding decide whether it clashes, which made the result depend on
        // the compiler's floating-point contraction, so A is run both ways where that places the
        // tabs differently, T-0159), matched without add-back and without
        // the second try with the cover placed from the vector out; (C) placed with the slack and
        // the move margin, matched with that second try; (B) as C with add-back (a refused tab
        // gives way to tabs fitting dropped beside it). The one that ends with the most kept tabs
        // wins, a tie going to A, then C; so no scene keeps fewer tabs than T-0127's matching (the
        // tab check only adds to it: a tab judged again on a wider window), nor than C without
        // add-back. A and C are only run when they would differ (A: some cut's tabs are placed
        // differently, or a run used the second try; C: add-back changed something).
        struct TabState {
            std::vector<Cut> finish, rough;
            std::map<std::size_t, std::size_t> requested, dropped, unmatched, severed, empty, onCorner;
            std::vector<std::vector<std::vector<std::pair<std::size_t, TabSpan>>>> links;
        };
        auto save = [&]() {
            return TabState{finish,         rough,          tabsRequested_, tabsDropped_, tabsUnmatched_,
                            tabsSevered_,   tabsEmpty_,     tabsOnCorner_,  tabLinks_};
        };
        auto restore = [&](const TabState& st) {
            finish = st.finish;
            rough = st.rough;
            tabsRequested_ = st.requested;
            tabsDropped_ = st.dropped;
            tabsUnmatched_ = st.unmatched;
            tabsSevered_ = st.severed;
            tabsEmpty_ = st.empty;
            tabsOnCorner_ = st.onCorner;
            tabLinks_ = st.links;
        };
        auto keptTabs = [&]() {
            std::size_t n = 0;
            for (const Cut& c : finish) n += c.tabs.size();
            return n;
        };
        const TabState start = save();
        auto samePlacement = [&](const TabState& x, const TabState& y) {
            for (std::size_t k = 0; k < x.finish.size(); ++k) {
                if (x.finish[k].tabs != y.finish[k].tabs) return false;
            }
            return true;
        };
        placeTabsOn(finish, rough, true, true);
        const TabState placedArefused = save();
        restore(start);
        placeTabsOn(finish, rough, true, false);
        const TabState placedA = save();
        restore(start);
        placeTabsOn(finish, rough, false);
        const TabState placedC = save();
        const MatchExtras fromB = matchRoughingTabs(finish, rough, true, true);
        dropSweptTabs(finish, rough);
        TabState best = save();
        std::size_t bestKept = keptTabs();  // B for now
        bool retried = fromB.retried;
        if (fromB.addedBack) {  // C
            restore(placedC);
            retried = matchRoughingTabs(finish, rough, false, true).retried || retried;
            dropSweptTabs(finish, rough);
            if (keptTabs() >= bestKept) {
                bestKept = keptTabs();
                best = save();
            }
        }
        // A, with a tab moved to exactly the gap refused (where that places the tabs differently),
        // then accepted (a tie going to the accepted one).
        for (const TabState* a : {&placedArefused, &placedA}) {
            if (a == &placedArefused && samePlacement(placedArefused, placedA)) continue;
            if (samePlacement(*a, placedC) && !retried) continue;
            restore(*a);
            matchRoughingTabs(finish, rough, false, false);
            dropSweptTabs(finish, rough);
            if (keptTabs() >= bestKept) {
                bestKept = keptTabs();
                best = save();
            }
        }
        restore(best);
        reportLoosePieces(finish, rough);
    } else {
        placeTabsOn(rough, {});
        dropSweptTabs(rough, finish);
        reportLoosePieces(rough, finish);
    }
    reportTabs();
    collectTabMaterial(rough);
    collectTabMaterial(finish);
    detail::pollCancel();
    for (std::size_t i : roughOrder) {
        emit(rough[i], levels, false);
        detail::pollCancel();
    }
    for (std::size_t i : finishOrder) {
        emit(finish[i], {levels.back()}, true);
        detail::pollCancel();
    }
    // Roughing and finishing report the same vector-level issue once.
    std::vector<ProfileDiagnostic> unique;
    std::set<std::tuple<ProfileIssue, std::optional<std::size_t>, std::string>> seen;
    for (ProfileDiagnostic& d : result_.diagnostics) {
        if (seen.insert({d.issue, d.contour, d.message}).second) {
            unique.push_back(std::move(d));
        }
    }
    result_.diagnostics = std::move(unique);
    if (result_.toolpath.isEmpty() && result_.diagnostics.empty()) {
        diag(ProfileIssue::ToolTooLarge, std::nullopt, "no vector produced a cut");
    }
    return std::move(result_);
}

}  // namespace

ProfileResult profileToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                              const ProfileParams& params, const ToolpathRunControl& control) {
    // The work meter counts what the run does and stops it at the limit or a cancel; the throw
    // unwinds the whole builder, so no partial toolpath or diagnostic leaves this function.
    constexpr std::uint64_t kMaxLimit = std::uint64_t{1} << 60;
    detail::WorkMeter meter(control.workLimit == 0 ? kProfileDefaultWorkUnits : std::min(control.workLimit, kMaxLimit),
                            control.cancel);
    detail::MeterScope scope(meter);
    // T-0228: the geometry calls poll the same flag, so a cancel stops a huge single offset in ms.
    const geometry::CancelScope cancelScope(control.cancel);
    ProfileResult stopped;
    stopped.toolpath.name = params.name;
    const auto stop = [&](ProfileIssue issue, std::string message) {
        stopped.diagnostics.push_back({issue, std::nullopt, std::move(message), std::nullopt});
        stopped.workUnits = meter.used();
        return stopped;
    };
    try {
        meter.poll();
        ProfileBuilder builder(contours, tool, params,
                               control.moveLimit == 0 ? kProfileMaxMoves : static_cast<double>(control.moveLimit));
        ProfileResult result = builder.run();
        result.workUnits = meter.used();
        return result;
    } catch (const detail::WorkAborted& stopReason) {
        if (stopReason.cancelled) {
            return stop(ProfileIssue::Cancelled, "the profile was cancelled; nothing is cut");
        }
        return stop(ProfileIssue::WorkLimitExceeded,
                    "the profile needs more work than the limit allows (" + std::to_string(meter.limit()) +
                        " work units; the job is too large, too detailed or has very long vectors); nothing is cut");
    } catch (const TooComplexForOneStep& big) {
        return stop(ProfileIssue::WorkLimitExceeded,
                    "vector #" + std::to_string(big.contour + 1) + " (or the group of vectors merged with it) is too" +
                        " complex to offset in one step (about " +
                        (big.units < 9e17 ? std::to_string(static_cast<long long>(big.units)) : std::string("over 1e17")) +
                        " work units; a single step may take at most " +
                        std::to_string(static_cast<long long>(kMaxSingleCallUnits)) + "); nothing is cut");
    } catch (const TooManyMoves&) {
        return stop(ProfileIssue::WorkLimitExceeded,
                    "the toolpath would have more than " +
                        std::to_string(control.moveLimit == 0 ? static_cast<unsigned long long>(kProfileMaxMoves)
                                                              : static_cast<unsigned long long>(control.moveLimit)) +
                        " moves; nothing is cut");
    } catch (const std::bad_alloc&) {
        return stop(ProfileIssue::WorkLimitExceeded, "the profile ran out of memory; nothing is cut");
    } catch (const std::length_error&) {
        return stop(ProfileIssue::WorkLimitExceeded, "the profile needs more memory than can be allocated; nothing is cut");
    }
}

ProfileResult profileToolpath(const std::vector<Contour>& contours, const tooling::Tool& tool,
                              const ProfileParams& params) {
    return profileToolpath(contours, tool, params, ToolpathRunControl{});
}

}  // namespace carvetto::toolpath
