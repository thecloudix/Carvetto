#include "carvetto/toolpath/pocket.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/toolpath/pass_plan.hpp"
#include "carvetto/toolpath/ramp_entry.hpp"
#include "../geometry/robust_predicates.hpp"
#include "pocket_distance.hpp"
#include "pocket_guard.hpp"
#include "pocket_order.hpp"

// Pocket toolpath (T-0121, ADR-0017; T-0123, ADR-0018).
//
// All region work is done on polygons through the public geometry API: the even-odd union of
// the input gives the pocket region R; offsetContours(R, −(radius + allowance + margin)) gives
// the region C the tool centre may visit. The Offset strategy builds the level sets
// offset(C, −k·stepover) as a containment tree and cuts it in post-order (inside out); when the
// stepover exceeds the radius, extra rings offset(C, −(k·stepover + radius)) are inserted
// wherever the next level would leave material uncovered. The Raster strategy clips lines to C
// with an open-path intersection. Rest tools build their own pattern over their rest areas (the
// centre region where the tool disc reaches rest material). Small polygon helpers (point in
// ring, segment inside region) live here and are deliberately simple: they work on the line-only polygons Clipper2 returns.

namespace carvetto::toolpath {

using geometry::BooleanOp;
using geometry::Contour;
using geometry::FillRule;
using geometry::JoinType;
using geometry::Polyline;
using geometry::Tolerance;

namespace {

using Ring = std::vector<Vec2>;

/// A connected polygonal region: one counter-clockwise outer ring and its clockwise holes.
struct Group {
    Ring outer;
    std::vector<Ring> holes;
};

// ------------------------------------------------------------------------- polygon helpers

Ring ringOf(const Contour& c) {
    Ring r;
    r.reserve(c.spans.size());
    for (const auto& s : c.spans) {
        r.push_back(geometry::startPoint(s));
    }
    return r;
}

Contour contourOf(const Ring& r) {
    return Contour::fromPolyline(Polyline{r, true});
}

double ringArea(const Ring& r) {
    if (r.empty()) {
        return 0.0;
    }
    // About the first vertex, so a small ring far from the origin does not cancel (T-0147).
    const Vec2 o = r.front();
    double a = 0.0;
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        a += (r[i] - o).cross(r[(i + 1) % n] - o);
    }
    return 0.5 * a;
}

double distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = ab.lengthSquared();
    const double t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
    return geometry::distance(p, a + ab * t);
}

/// Distance between segments ab and cd (`detail::segmentDistance`, below).
double distSegments(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    return detail::segmentDistance(a, b, c, d);
}

bool onRing(const Ring& r, Vec2 p, double tol) {
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        if (distToSegment(p, r[i], r[(i + 1) % n]) <= tol) {
            return true;
        }
    }
    return false;
}

/// Even-odd crossing parity of a +X ray against one ring (half-open rule on y).
bool rayParity(const Ring& r, Vec2 p) {
    bool inside = false;
    for (std::size_t i = 0, n = r.size(), j = n - 1; i < n; j = i++) {
        const Vec2 a = r[j];
        const Vec2 b = r[i];
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x > p.x) {
                inside = !inside;
            }
        }
    }
    return inside;
}

/// True when `p` lies inside the even-odd region of `rings` or within `tol` of its boundary.
bool insideRegion(const std::vector<Ring>& rings, Vec2 p, double tol) {
    bool inside = false;
    for (const Ring& r : rings) {
        if (onRing(r, p, tol)) {
            return true;
        }
        inside ^= rayParity(r, p);
    }
    return inside;
}

/// The tolerances of `segmentInside` (T-0169, ADR-0017 §8a).
struct MoveTolerance {
    /// What a segment is allowed to get away with: ends this far off an edge it follows,
    /// crossings this close to an end, a midpoint this far outside. Every point of an accepted
    /// segment is within about twice this of the region.
    double accept{0.0};
    /// A boundary vertex this close to the segment's interior rejects it (conservative only).
    double reject{0.0};
};

/// Clipper2 rounds every vertex to its 1e-6 mm grid (≤ 0.71e-6 mm in the plane, ADR-0006), and the
/// raster ends are moved back onto their exact line from there: four grid units hold both.
constexpr double kMoveSlack = 4e-6;

/// True when the straight segment a→b stays inside the region of `rings` (boundary included,
/// within `tol.accept`): no edge is crossed away from the end points and the midpoint is inside.
/// An edge the whole segment lies along (both ends within `tol.accept` of the edge segment, hence
/// every point of a→b too) is not a crossing, so links that follow the boundary are accepted even
/// when their ends sit a rounding error off it; the edge's infinite line is not enough (a nearly
/// parallel edge far away would hide a real exit). Conservative: a boundary vertex within
/// `tol.reject` of the segment's interior (and not within `tol.accept` of an end) rejects it
/// (rounding could hide a crossing through that vertex). Every point of an accepted segment is
/// within about 2·`tol.accept` of the region (ADR-0017 §8a).
bool segmentInside(Vec2 a, Vec2 b, const std::vector<Ring>& rings, MoveTolerance tol) {
    const Vec2 d = b - a;
    auto nearEnd = [&](Vec2 x) {
        return geometry::distance(x, a) <= tol.accept || geometry::distance(x, b) <= tol.accept;
    };
    for (const Ring& r : rings) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2 c = r[i];
            const Vec2 c2 = r[(i + 1) % n];
            const Vec2 e = c2 - c;
            if (e.lengthSquared() > 0.0 && distToSegment(a, c, c2) <= tol.accept &&
                distToSegment(b, c, c2) <= tol.accept) {
                continue;  // within tol of this edge all along: following the boundary
            }
            // A boundary vertex within tol of the segment's interior: a touch (or a crossing
            // through the vertex that rounding could hide).
            if (distToSegment(c, a, b) <= tol.reject && !nearEnd(c)) {
                return false;
            }
            // A proper crossing away from the end points.
            const double o1 = d.cross(c - a);
            const double o2 = d.cross(c2 - a);
            const double o3 = e.cross(a - c);
            const double o4 = e.cross(b - c);
            if (((o1 < 0.0 && o2 > 0.0) || (o1 > 0.0 && o2 < 0.0)) &&
                ((o3 < 0.0 && o4 > 0.0) || (o3 > 0.0 && o4 < 0.0))) {
                const Vec2 x = a + d * (o3 / (o3 - o4));
                if (!nearEnd(x)) {
                    return false;
                }
            }
        }
    }
    return insideRegion(rings, (a + b) * 0.5, tol.accept);
}

std::vector<Ring> ringsOf(const Group& g) {
    std::vector<Ring> out{g.outer};
    out.insert(out.end(), g.holes.begin(), g.holes.end());
    return out;
}

std::vector<Contour> contoursOf(const Group& g) {
    std::vector<Contour> out;
    out.push_back(contourOf(g.outer));
    for (const Ring& h : g.holes) {
        out.push_back(contourOf(h));
    }
    return out;
}

std::vector<Contour> contoursOf(const std::vector<Group>& gs) {
    std::vector<Contour> out;
    for (const Group& g : gs) {
        for (Contour& c : contoursOf(g)) {
            out.push_back(std::move(c));
        }
    }
    return out;
}

Box2 ringBox(const Ring& r) {
    Box2 b;
    for (const Vec2 q : r) {
        b.expand(q);
    }
    return b;
}

/// Groups polygonal Clipper2 output (outers CCW, holes CW) into regions. Each hole goes to the
/// smallest outer that contains it. Order follows the outers' order in `contours`.
std::vector<Group> groupsOf(const std::vector<Contour>& contours, double tol) {
    std::vector<Group> groups;
    std::vector<double> areas;
    std::vector<Ring> holes;
    for (const Contour& c : contours) {
        Ring r = ringOf(c);
        if (r.size() < 3) {
            continue;
        }
        const double a = ringArea(r);
        if (a > 0.0) {
            groups.push_back(Group{std::move(r), {}});
            areas.push_back(a);
        } else if (a < 0.0) {
            holes.push_back(std::move(r));
        }
    }
    // Only outers whose box holds the hole's box can contain it (T-0123: a spatial index keeps
    // this near-linear for thousands of regions).
    std::vector<Box2> outerBoxes;
    outerBoxes.reserve(groups.size());
    for (const Group& g : groups) {
        outerBoxes.push_back(ringBox(g.outer));
    }
    const BoxIndex index(std::move(outerBoxes));
    for (Ring& h : holes) {
        Vec2 probe = h.front();
        std::optional<std::size_t> best;
        for (const std::size_t gi : index.query(ringBox(h).inflated(tol))) {
            const Ring& o = groups[gi].outer;
            for (const Vec2 v : h) {
                if (!onRing(o, v, tol)) {
                    probe = v;
                    break;
                }
            }
            if (rayParity(o, probe) && (!best || areas[gi] < areas[*best])) {
                best = gi;
            }
        }
        if (best) {
            groups[*best].holes.push_back(std::move(h));
        }
    }
    return groups;
}

std::vector<Contour> offsetBy(const std::vector<Contour>& cs, double d, const Tolerance& tol) {
    if (cs.empty()) {
        return {};
    }
    return geometry::offsetContours(cs, d, JoinType::Round, tol);
}

/// Uniform grid over the edges of a ring set, for "is any edge closer than d to p (or to a
/// segment)" and "nearest edge within d" queries.
/// Cells are at least `cell` wide (at most 256 per axis); an edge is listed in every cell its
/// bounding box touches. Storage is flat (CSR), built in two passes.
class EdgeGrid {
   public:
    EdgeGrid(const std::vector<Ring>& rings, double cell) {
        for (const Ring& r : rings) {
            for (std::size_t i = 0, n = r.size(); i < n; ++i) {
                edges_.emplace_back(r[i], r[(i + 1) % n]);
                box_.expand(r[i]);
            }
        }
        if (edges_.empty()) {
            return;
        }
        const double extent = std::max(box_.width(), box_.height());
        cell_ = std::max({cell, extent / 256.0, 1e-9});
        nx_ = static_cast<std::size_t>(box_.width() / cell_) + 1;
        ny_ = static_cast<std::size_t>(box_.height() / cell_) + 1;
        start_.assign(nx_ * ny_ + 1, 0);
        for (int pass = 0; pass < 2; ++pass) {
            std::vector<std::size_t> fill;
            if (pass == 1) {
                for (std::size_t c = 1; c < start_.size(); ++c) {
                    start_[c] += start_[c - 1];
                }
                items_.resize(start_.back());
                fill.assign(start_.begin(), start_.end() - 1);
            }
            for (std::size_t e = 0; e < edges_.size(); ++e) {
                const auto [a, b] = edges_[e];
                const std::size_t i0 = col(std::min(a.x, b.x)), i1 = col(std::max(a.x, b.x));
                const std::size_t j0 = row(std::min(a.y, b.y)), j1 = row(std::max(a.y, b.y));
                for (std::size_t j = j0; j <= j1; ++j) {
                    for (std::size_t i = i0; i <= i1; ++i) {
                        const std::size_t c = j * nx_ + i;
                        if (pass == 0) {
                            ++start_[c + 1];
                        } else {
                            items_[fill[c]++] = e;
                        }
                    }
                }
            }
        }
    }

    /// Smallest distance from `p` to an edge, if some edge is closer than `d`; `d` otherwise.
    double nearestWithin(Vec2 p, double d) const {
        double best = d;
        if (edges_.empty()) {
            return best;
        }
        for (std::size_t j = row(p.y - d); j <= row(p.y + d); ++j) {
            for (std::size_t i = col(p.x - d); i <= col(p.x + d); ++i) {
                const std::size_t c = j * nx_ + i;
                for (std::size_t k = start_[c]; k < start_[c + 1]; ++k) {
                    const auto [a, b] = edges_[items_[k]];
                    if (!apart(p, p, a, b, best)) {
                        best = std::min(best, distToSegment(p, a, b));
                    }
                }
            }
        }
        return best;
    }

    /// Smallest exact distance from the segment ab to an edge, if some edge is closer than `d`;
    /// `d` otherwise.
    double nearestWithin(Vec2 a, Vec2 b, double d) const {
        double best = d;
        if (edges_.empty()) {
            return best;
        }
        const std::size_t i0 = col(std::min(a.x, b.x) - d), i1 = col(std::max(a.x, b.x) + d);
        const std::size_t j0 = row(std::min(a.y, b.y) - d), j1 = row(std::max(a.y, b.y) + d);
        for (std::size_t j = j0; j <= j1; ++j) {
            for (std::size_t i = i0; i <= i1; ++i) {
                const std::size_t c = j * nx_ + i;
                for (std::size_t k = start_[c]; k < start_[c + 1]; ++k) {
                    const auto [p, q] = edges_[items_[k]];
                    if (!apart(a, b, p, q, best)) {
                        best = std::min(best, distSegments(a, b, p, q));
                    }
                }
            }
        }
        return best;
    }

    /// True when some edge lies closer than `d` to the segment ab (stops at the first one). Only
    /// the cells of each row that the segment, widened by `d`, can reach are visited, so a long
    /// diagonal segment costs its length in cells, not its bounding box.
    bool anyCloser(Vec2 a, Vec2 b, double d) const {
        if (edges_.empty()) {
            return false;
        }
        const std::size_t j0 = row(std::min(a.y, b.y) - d), j1 = row(std::max(a.y, b.y) + d);
        for (std::size_t j = j0; j <= j1; ++j) {
            // The part of the segment within d of this row's band [y0, y1], widened by d in x
            // (whole rows at the clamped ends of the grid).
            const double y0 = j == 0 ? -std::numeric_limits<double>::infinity()
                                     : box_.min.y + static_cast<double>(j) * cell_ - d;
            const double y1 = j + 1 == ny_ ? std::numeric_limits<double>::infinity()
                                           : box_.min.y + static_cast<double>(j + 1) * cell_ + d;
            double t0 = 0.0;
            double t1 = 1.0;
            const double dy = b.y - a.y;
            if (dy != 0.0) {
                const double ta = (y0 - a.y) / dy;
                const double tb = (y1 - a.y) / dy;
                t0 = std::max(t0, std::min(ta, tb));
                t1 = std::min(t1, std::max(ta, tb));
            }
            if (!(t0 <= t1)) {
                continue;  // (a row the widened bounding box reaches but the segment does not)
            }
            const double xa = a.x + (b.x - a.x) * t0;
            const double xb = a.x + (b.x - a.x) * t1;
            const std::size_t i0 = col(std::min(xa, xb) - d), i1 = col(std::max(xa, xb) + d);
            for (std::size_t i = i0; i <= i1; ++i) {
                const std::size_t c = j * nx_ + i;
                for (std::size_t k = start_[c]; k < start_[c + 1]; ++k) {
                    const auto [p, q] = edges_[items_[k]];
                    if (!apart(a, b, p, q, d) && distSegments(a, b, p, q) < d) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    /// True when some edge lies closer than `d` to `p`.
    bool anyCloser(Vec2 p, double d) const {
        if (edges_.empty()) {
            return false;
        }
        const std::size_t i0 = col(p.x - d), i1 = col(p.x + d);
        const std::size_t j0 = row(p.y - d), j1 = row(p.y + d);
        for (std::size_t j = j0; j <= j1; ++j) {
            for (std::size_t i = i0; i <= i1; ++i) {
                const std::size_t c = j * nx_ + i;
                for (std::size_t k = start_[c]; k < start_[c + 1]; ++k) {
                    const auto [a, b] = edges_[items_[k]];
                    if (!apart(p, p, a, b, d) && distToSegment(p, a, b) < d) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

   private:
    /// True when the bounding boxes of segments ab and cd lie more than `d` apart along an axis,
    /// so the segments are certainly farther apart than `d` (a cheap test before the exact
    /// distance; the 1e-9 mm keeps it clear of rounding, so results are as without it).
    static bool apart(Vec2 a, Vec2 b, Vec2 c, Vec2 d2, double d) {
        const double gx =
            std::max(std::min(c.x, d2.x) - std::max(a.x, b.x), std::min(a.x, b.x) - std::max(c.x, d2.x));
        const double gy =
            std::max(std::min(c.y, d2.y) - std::max(a.y, b.y), std::min(a.y, b.y) - std::max(c.y, d2.y));
        return std::max(gx, gy) > d + 1e-9;
    }
    std::size_t col(double x) const {
        const double f = std::floor((x - box_.min.x) / cell_);
        return static_cast<std::size_t>(std::clamp(f, 0.0, static_cast<double>(nx_ - 1)));
    }
    std::size_t row(double y) const {
        const double f = std::floor((y - box_.min.y) / cell_);
        return static_cast<std::size_t>(std::clamp(f, 0.0, static_cast<double>(ny_ - 1)));
    }

    std::vector<std::pair<Vec2, Vec2>> edges_;
    Box2 box_;
    double cell_{1.0};
    std::size_t nx_{1};
    std::size_t ny_{1};
    std::vector<std::size_t> start_;
    std::vector<std::size_t> items_;
};

/// A region (even-odd rings) with its edges in a uniform grid, for the same `segmentInside` and
/// point-in-region tests as the free functions but in time proportional to the edges near the
/// query (and along one grid row for the parity ray) instead of all edges. Used where those
/// tests run per link or per travel move.
class RegionIndex {
   public:
    explicit RegionIndex(std::vector<Ring> rings) : rings_(std::move(rings)) {
        for (std::size_t ri = 0; ri < rings_.size(); ++ri) {
            for (std::size_t i = 0, n = rings_[ri].size(); i < n; ++i) {
                edges_.emplace_back(rings_[ri][i], rings_[ri][(i + 1) % n]);
                box_.expand(rings_[ri][i]);
            }
        }
        stamp_.assign(edges_.size(), 0);
        if (edges_.empty()) {
            return;
        }
        // About one edge per cell, at most 2048 cells along the longer side. Square cells over the
        // area, not over the longer side (T-0131): a long thin region (a comb) got only a few rows
        // of cells holding hundreds of edges each. The cell size does not change any answer (every
        // query combines its edges with OR / XOR).
        cell_ = std::max({std::sqrt(box_.width() * box_.height() / static_cast<double>(edges_.size())),
                          std::max(box_.width(), box_.height()) / 2048.0, 1e-9});
        nx_ = static_cast<std::size_t>(box_.width() / cell_) + 1;
        ny_ = static_cast<std::size_t>(box_.height() / cell_) + 1;
        start_.assign(nx_ * ny_ + 1, 0);
        for (int pass = 0; pass < 2; ++pass) {
            std::vector<std::size_t> fill;
            if (pass == 1) {
                for (std::size_t c = 1; c < start_.size(); ++c) {
                    start_[c] += start_[c - 1];
                }
                items_.resize(start_.back());
                fill.assign(start_.begin(), start_.end() - 1);
            }
            for (std::size_t e = 0; e < edges_.size(); ++e) {
                const auto [a, b] = edges_[e];
                for (std::size_t j = row(std::min(a.y, b.y)); j <= row(std::max(a.y, b.y)); ++j) {
                    for (std::size_t i = col(std::min(a.x, b.x)); i <= col(std::max(a.x, b.x)); ++i) {
                        const std::size_t c = j * nx_ + i;
                        if (pass == 0) {
                            ++start_[c + 1];
                        } else {
                            items_[fill[c]++] = e;
                        }
                    }
                }
            }
        }
    }

    const std::vector<Ring>& rings() const { return rings_; }

    /// As the free `insideRegion`.
    bool inside(Vec2 p, double tol) const {
        bool onBoundary = false;
        forEdges(p.x - tol, p.y - tol, p.x + tol, p.y + tol,
                 [&](Vec2 a, Vec2 b) { onBoundary = onBoundary || distToSegment(p, a, b) <= tol; });
        if (onBoundary) {
            return true;
        }
        return parity(p, tol);
    }

    /// `inside(p, small)` and `inside(p, large)` (small <= large) with one boundary scan and at
    /// most one parity ray.
    std::pair<bool, bool> inside(Vec2 p, double small, double large) const {
        double nearest = std::numeric_limits<double>::infinity();
        forEdges(p.x - large, p.y - large, p.x + large, p.y + large,
                 [&](Vec2 a, Vec2 b) { nearest = std::min(nearest, distToSegment(p, a, b)); });
        if (nearest <= small) {
            return {true, true};
        }
        const bool in = parity(p, small);
        return {in, in || nearest <= large};
    }

    /// Even-odd membership of `p`, every edge being more than `tol` from it.
    bool parity(Vec2 p, double tol) const {
        // Here every edge is more than `tol` from p. The crossing of a ray from p with an edge is
        // then more than `tol` from p, and its computed position is off by a few ulps of the
        // coordinates, so the side it is counted on is exact whenever `tol` is well above those
        // ulps: the parity of a ray in any direction is the exact even-odd membership, the same
        // answer as the ray to the right. So the ray goes the way with the fewest grid cells
        // (T-0131: in a comb, a ray along the raster crossed every tooth, per link check). With a
        // tolerance too small for that argument, the ray to the right as before.
        const double scale = std::max({std::fabs(box_.min.x), std::fabs(box_.max.x), std::fabs(box_.min.y),
                                       std::fabs(box_.max.y), std::fabs(p.x), std::fabs(p.y)});
        if (tol > 1e-9 * (1.0 + scale) && std::isfinite(scale) && std::isfinite(p.x) && std::isfinite(p.y)) {
            const std::size_t i = col(p.x);
            const std::size_t j = row(p.y);
            const std::size_t cells[4] = {nx_ - i, i + 1, ny_ - j, j + 1};  // right, left, up, down
            const std::size_t way = static_cast<std::size_t>(std::min_element(cells, cells + 4) - cells);
            if (way != 0) {
                bool parity = false;
                if (way == 1) {
                    forEdges(box_.min.x - 1.0, p.y, p.x, p.y, [&](Vec2 a, Vec2 b) {
                        if ((a.y > p.y) != (b.y > p.y)) {
                            const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                            if (x < p.x) {
                                parity = !parity;
                            }
                        }
                    });
                } else {
                    const bool up = way == 2;
                    const double y0 = up ? p.y : box_.min.y - 1.0;
                    const double y1 = up ? box_.max.y + 1.0 : p.y;
                    forEdges(p.x, y0, p.x, y1, [&](Vec2 a, Vec2 b) {
                        if ((a.x > p.x) != (b.x > p.x)) {
                            const double y = a.y + (p.x - a.x) * (b.y - a.y) / (b.x - a.x);
                            if (up ? y > p.y : y < p.y) {
                                parity = !parity;
                            }
                        }
                    });
                }
                return parity;
            }
        }
        bool in = false;
        forEdges(p.x, p.y, box_.max.x + 1.0, p.y, [&](Vec2 a, Vec2 b) {
            if ((a.y > p.y) != (b.y > p.y)) {
                const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if (x > p.x) {
                    in = !in;
                }
            }
        });
        return in;
    }

    /// The free `segmentInside` with two tolerances at once (`strict` no looser than `lenient` in
    /// every respect, as `MoveCheck` builds them), in one pass over the nearby edges. A segment
    /// passing the strict test passes the lenient one too.
    struct Verdicts {
        bool lenient{false};
        bool strict{false};
    };
    Verdicts segmentInside(Vec2 a, Vec2 b, MoveTolerance strict, MoveTolerance lenient) const {
        const Vec2 d = b - a;
        auto nearEnd = [&](Vec2 x, double tol) {
            return geometry::distance(x, a) <= tol || geometry::distance(x, b) <= tol;
        };
        bool okL = true;
        bool okS = true;
        const double reach = std::max(lenient.accept, lenient.reject);
        forEdges(std::min(a.x, b.x) - reach, std::min(a.y, b.y) - reach, std::max(a.x, b.x) + reach,
                 std::max(a.y, b.y) + reach, [&](Vec2 c, Vec2 c2) {
                     if (!okL) {
                         return;  // (the strict test fails on every edge where the lenient one does)
                     }
                     const Vec2 e = c2 - c;
                     bool followL = false;
                     bool followS = false;
                     if (e.lengthSquared() > 0.0) {
                         const double da = distToSegment(a, c, c2);
                         if (da <= lenient.accept) {
                             const double db = distToSegment(b, c, c2);
                             followL = db <= lenient.accept;
                             followS = followL && da <= strict.accept && db <= strict.accept;
                         }
                     }
                     if (followS || (followL && !okS)) {
                         return;  // following the boundary
                     }
                     // A boundary vertex within tol of the segment's interior, or a proper crossing
                     // away from the end points (as the free `segmentInside`).
                     const double dc = distToSegment(c, a, b);
                     bool crossChecked = false;
                     bool crosses = false;
                     Vec2 x;
                     auto fails = [&](const MoveTolerance& tol) {
                         if (dc <= tol.reject && !nearEnd(c, tol.accept)) {
                             return true;
                         }
                         if (!crossChecked) {
                             crossChecked = true;
                             const double o1 = d.cross(c - a);
                             const double o2 = d.cross(c2 - a);
                             const double o3 = e.cross(a - c);
                             const double o4 = e.cross(b - c);
                             crosses = ((o1 < 0.0 && o2 > 0.0) || (o1 > 0.0 && o2 < 0.0)) &&
                                       ((o3 < 0.0 && o4 > 0.0) || (o3 > 0.0 && o4 < 0.0));
                             if (crosses) {
                                 x = a + d * (o3 / (o3 - o4));
                             }
                         }
                         return crosses && !nearEnd(x, tol.accept);
                     };
                     if (!followL && fails(lenient)) {
                         okL = false;
                         okS = false;
                         return;
                     }
                     if (okS && fails(strict)) {
                         okS = false;
                     }
                 });
        Verdicts v;
        if (okL) {
            const auto [inStrict, inLenient] = inside((a + b) * 0.5, strict.accept, lenient.accept);
            v.strict = okS && inStrict;
            v.lenient = inLenient;
        }
        return v;
    }

   private:
    template <class F>
    void forEdges(double x0, double y0, double x1, double y1, F&& f) const {
        if (edges_.empty()) {
            return;
        }
        ++tick_;
        for (std::size_t j = row(y0); j <= row(y1); ++j) {
            for (std::size_t i = col(x0); i <= col(x1); ++i) {
                const std::size_t c = j * nx_ + i;
                for (std::size_t k = start_[c]; k < start_[c + 1]; ++k) {
                    const std::size_t e = items_[k];
                    if (stamp_[e] != tick_) {
                        stamp_[e] = tick_;
                        f(edges_[e].first, edges_[e].second);
                    }
                }
            }
        }
    }
    std::size_t col(double x) const {
        const double f = std::floor((x - box_.min.x) / cell_);
        return static_cast<std::size_t>(std::clamp(f, 0.0, static_cast<double>(nx_ - 1)));
    }
    std::size_t row(double y) const {
        const double f = std::floor((y - box_.min.y) / cell_);
        return static_cast<std::size_t>(std::clamp(f, 0.0, static_cast<double>(ny_ - 1)));
    }

    std::vector<Ring> rings_;
    std::vector<std::pair<Vec2, Vec2>> edges_;
    Box2 box_;
    double cell_{1.0};
    std::size_t nx_{1};
    std::size_t ny_{1};
    std::vector<std::size_t> start_;
    std::vector<std::size_t> items_;
    mutable std::vector<std::uint64_t> stamp_;
    mutable std::uint64_t tick_{0};
};

/// The pocket region's polygon as an `EdgeGrid`, built on first use (most jobs never need it).
class WallIndex {
   public:
    explicit WallIndex(const std::vector<Contour>& region) : region_(region) {}
    const EdgeGrid& grid() const {
        if (!grid_) {
            std::vector<Ring> rings;
            Box2 box;
            std::size_t edges = 0;
            for (const Contour& c : region_) {
                rings.push_back(ringOf(c));
                for (const Vec2 q : rings.back()) {
                    box.expand(q);
                }
                edges += rings.back().size();
            }
            // About one edge per cell (the grid caps it at 256 cells per axis).
            const double cell =
                edges > 0 ? std::sqrt(box.width() * box.height() / static_cast<double>(edges)) : 0.0;
            grid_.emplace(rings, cell);
        }
        return *grid_;
    }

   private:
    const std::vector<Contour>& region_;
    mutable std::optional<EdgeGrid> grid_;
};

/// Decides whether the tool may move straight from a to b inside a centre region: raster links,
/// links at depth, travel, ring bridges and route shortcuts (ADR-0017 §8a, T-0169).
///
/// The inside test with the user's distance tolerance (as before T-0169) lets a segment cut a
/// reflex corner of the region by up to about twice that tolerance: where a round join meets a
/// straight edge of the centre region (under the crossbar of an "A", verifier T-0153 round 3) a
/// raster link came 45 nm inside the tool radius at distance = chordal/3, and with
/// distance up to chordal up to 0.029 mm. The safety margin (§3) has no room for it.
/// A move is therefore allowed when it stays inside the region within rounding (`kMoveSlack`),
/// or when it passes the old test and its exact distance to the polygonised walls is at least
/// radius + allowance + the polygon tolerance (0 when every input span is a line), so the true
/// walls keep radius + allowance. The second case keeps the old links wherever they were safe
/// (no change of output there).
class MoveCheck {
   public:
    /// `walls`: the pocket region's polygon (null: strict test only); `need`: radius + allowance +
    /// the polygon tolerance (none for line-only input); `distance`: the user's distance tolerance.
    MoveCheck(const WallIndex* walls, double need, double distance)
        : walls_(walls),
          need_(need),
          strict_{std::min(distance, kMoveSlack), distance},
          lenient_{distance, distance} {}

    bool allows(const RegionIndex& region, Vec2 a, Vec2 b) const {
        const RegionIndex::Verdicts v = region.segmentInside(a, b, strict_, lenient_);
        return v.strict || (v.lenient && clearOfWalls(a, b));
    }
    /// For travel at the surface (the first level's travel, T-0175), where the tool touches no
    /// material, so the walls need no clearance: the inside test with the distance tolerance
    /// alone, as before T-0169 (it keeps the tool within about twice that tolerance of the region).
    bool allowsInAir(const RegionIndex& region, Vec2 a, Vec2 b) const {
        return region.segmentInside(a, b, strict_, lenient_).lenient;
    }
    bool allows(const std::vector<Ring>& rings, Vec2 a, Vec2 b) const {
        if (segmentInside(a, b, rings, strict_)) {
            return true;
        }
        return segmentInside(a, b, rings, lenient_) && clearOfWalls(a, b);
    }
    /// Points of a way closer than this are merged (a merged point moves the way by at most this):
    /// rounding below the surface, the distance tolerance at the surface (`allowsInAir`).
    double mergeDistance(bool inAir) const { return inAir ? lenient_.accept : strict_.accept; }

   private:
    bool clearOfWalls(Vec2 a, Vec2 b) const {
        return walls_ != nullptr && !walls_->grid().anyCloser(a, b, need_);
    }

    const WallIndex* walls_;
    double need_;
    MoveTolerance strict_;
    MoveTolerance lenient_;
};

/// How `insetBy` tells a genuine inset region from an inside-out phantom, and what it does with
/// a genuine region that overshoots.
///
/// Phantom test (both guards, T-0126): a result region is genuine when some point of it (an outer
/// vertex, or the centroid of its outer ring when that lies in the region) keeps the full
/// distance from the source and lies inside it. Such a point belongs to the true inset, so the
/// region does too. A phantom (Clipper2 1.4.0's inside-out result past the inradius) has no such
/// point: every point of it is closer to the walls than the inset. The test is by depth, not by
/// vertex count, so a large region whose small bulge holds most of its vertices is kept. Since
/// T-0018 `offsetContours` no longer returns phantoms (ADR-0006 §6); the test is kept as a cheap
/// local defence of a gouge-relevant invariant (ADR-0017 §4); it is no longer expected to fire.
enum class InsetGuard {
    /// For the tool-centre region, where a vertex or an edge too close to a wall would be a gouge.
    /// Outer and hole vertices are checked. In a genuine region, failing convex vertices are
    /// pulled in (or removed) so the region only shrinks where it overshot (Clipper2's grid, join
    /// sag, zero-width spikes), and every edge so changed is checked against the walls exactly;
    /// if a failing vertex is reflex, or a changed edge comes too close, the region is inset
    /// further until a valid region results. A region is never dropped for overshoot alone; one
    /// that cannot be kept, or is trimmed by more than the join tolerance, is counted and
    /// reported (T-0125, T-0126; ADR-0017 §4).
    Strict,
    /// Phantoms are rejected and genuine regions kept as they come: used everywhere else (offset
    /// levels, rest material, openings), where a vertex near a cusp of the input that comes out
    /// a little too close costs nothing (T-0121 round 3: a 15 x 6 mm neck of rest material was
    /// dropped whole for one such vertex).
    Lenient,
};

using detail::InsetReport;

double groupArea(const Group& g) {
    double a = ringArea(g.outer);
    for (const Ring& h : g.holes) {
        a += ringArea(h);
    }
    return a;
}

double groupPerimeter(const Group& g) {
    double len = 0.0;
    auto ring = [&](const Ring& r) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            len += geometry::distance(r[i], r[(i + 1) % n]);
        }
    };
    ring(g.outer);
    for (const Ring& h : g.holes) {
        ring(h);
    }
    return len;
}

/// Insets the region of `cs` by d > 0 (offsetContours with −d) and keeps only genuine result
/// regions, as `guard` describes. Unpatched Clipper2 1.4.0 turned a polygon inset beyond its
/// inradius inside out (a circle inset by more than its radius came back as a growing circle);
/// T-0018 fixed that at the source, and the phantom rejection stays as defence (every point of a
/// true inset is at distance >= d from the input). Distances use an edge grid; the inside
/// test is made once per region at a point that keeps the distance. With `Strict`, a genuine
/// region that cannot be repaired, or is trimmed noticeably, is counted in `*report` (never
/// silently lost).
/// `insetBy` on a given raw inset `raw` of `cs` (Clipper2's, or a hand-made one in tests).
std::vector<Contour> guardInset(const std::vector<Contour>& cs, const std::vector<Contour>& raw, double d,
                                const Tolerance& tol, InsetGuard guard, InsetReport* report) {
    if (raw.empty()) {
        return raw;
    }
    std::vector<Ring> base;
    for (const Contour& c : cs) {
        base.push_back(ringOf(c));
    }
    // Where two round joins meet, the vertex lies inside both arcs by up to the join tolerance
    // (tol.chordal), which the pocket's safety margin already budgets for.
    const double slack = tol.chordal + 1e-6;
    const double need = d - slack;
    const EdgeGrid grid(base, need);
    const RegionIndex baseRegion(base);  // inside tests in time proportional to nearby edges
    auto passes = [&](Vec2 v) { return !grid.anyCloser(v, need); };
    // Pulls each failing vertex that is convex for the region (a left turn: the region lies on
    // the left of every ring, outers counter-clockwise, holes clockwise) towards the midpoint of
    // its neighbours, just far enough (bisection) that it keeps the distance. This covers both a
    // vertex a hair past the slack (moved by about that hair) and the tip of a zero-width
    // out-and-back spike Clipper2 leaves in disc-union insets (moved back down the spike). The
    // new vertex lies inside the triangle (prev, vertex, next), so the region only shrinks, and
    // by no more than needed. When the midpoint fails too (a spike over several vertices) the
    // vertex is dropped, which cuts that triangle off, and its neighbours follow in later passes
    // (rings keep at least three vertices). A failing reflex vertex is left for the inset repair
    // below. A vertex that keeps the distance does not make its edges keep it (a chord across a
    // wall corner can sag towards it), so every edge with a moved end, and every chord left by a
    // removal, is then checked exactly against the walls (T-0126). Returns true when every
    // vertex and every such edge of every ring keeps the distance, and no other edge comes
    // closer than d less twice the join tolerance (the no-gouge line: radius + allowance for the
    // centre region; Clipper2's own chords keep about d less one join tolerance).
    const double edgeNeed = need - tol.chordal;
    double shortfall = 0.0;  // largest vertex or edge shortfall left by the last pull-in
    auto pullIn = [&](Group& g) {
        bool allOk = true;
        auto ring = [&](Ring& r) {
            std::vector<char> touched(r.size(), 0);  // vertex moved, or a neighbour removed
            for (int pass = 0; pass < 64; ++pass) {
                bool changed = false;
                for (std::size_t i = 0, n = r.size(); i < n; ++i) {
                    if (i >= r.size() || passes(r[i])) {
                        continue;
                    }
                    const Vec2 prev = r[(i + n - 1) % n];
                    const Vec2 next = r[(i + 1) % n];
                    const Vec2 in = r[i] - prev;
                    const Vec2 out = next - r[i];
                    const Vec2 mid = (prev + next) * 0.5;
                    if (in.cross(out) < -1e-12 * in.length() * out.length()) {
                        continue;  // reflex: left for the inset repair
                    }
                    if (!passes(mid)) {
                        // The neighbours fail too (a spike over several vertices): drop this
                        // vertex, cutting the triangle off; the neighbours follow in later passes.
                        if (n > 3) {
                            r.erase(r.begin() + static_cast<std::ptrdiff_t>(i));
                            touched.erase(touched.begin() + static_cast<std::ptrdiff_t>(i));
                            --n;
                            touched[(i + n - 1) % n] = 1;  // the new chord prev -> next
                            touched[i % n] = 1;
                            --i;
                            changed = true;
                        }
                        continue;
                    }
                    double lo = 0.0, hi = 1.0;  // fails at lo, passes at hi
                    for (int it = 0; it < 40; ++it) {
                        const double t = 0.5 * (lo + hi);
                        (passes(r[i] + (mid - r[i]) * t) ? hi : lo) = t;
                    }
                    r[i] = r[i] + (mid - r[i]) * hi;
                    touched[i] = 1;
                    changed = true;
                }
                if (!changed) {
                    break;
                }
            }
            for (std::size_t i = 0, n = r.size(); i < n; ++i) {
                const std::size_t j = (i + 1) % n;
                const double limit = touched[i] != 0 || touched[j] != 0 ? need : edgeNeed;
                const double vertexDist = grid.nearestWithin(r[i], need);
                const double edgeDist = grid.nearestWithin(r[i], r[j], limit);
                shortfall = std::max({shortfall, need - vertexDist, limit - edgeDist});
                allOk = allOk && vertexDist >= need && edgeDist >= limit;
            }
        };
        ring(g.outer);
        for (Ring& h : g.holes) {
            ring(h);
        }
        return allOk;
    };
    std::vector<Group> kept;
    for (Group& g : groupsOf(raw, tol.distance)) {
        // Phantom test by depth (see InsetGuard): a point of the region at full distance from
        // the walls and inside the source.
        std::optional<Vec2> good;
        for (const Vec2 v : g.outer) {
            if (passes(v)) {
                good = v;
                break;
            }
        }
        if (good && !baseRegion.inside(*good, 0.0)) {
            continue;  // keeps the distance, but outside the source: a phantom
        }
        // No vertex at full depth: the centroid can still show a genuine region, unless the
        // region is a sliver (see below), where a centroid "at depth" is Clipper2 debris of a
        // collapsing lobe, not room for the tool.
        if (!good && g.outer.size() >= 3 && groupArea(g) > 0.5 * tol.chordal * groupPerimeter(g)) {
            Vec2 c{0.0, 0.0};
            for (const Vec2 v : g.outer) {
                c = c + v;
            }
            c = c * (1.0 / static_cast<double>(g.outer.size()));
            if (passes(c) && insideRegion(ringsOf(g), c, 0.0) && baseRegion.inside(c, 0.0)) {
                good = c;
            }
        }
        if (!good) {
            continue;  // phantom (or nothing)
        }
        if (guard == InsetGuard::Lenient) {
            kept.push_back(std::move(g));
            continue;
        }
        const double areaBefore = groupArea(g);
        const double perimeter = groupPerimeter(g);
        // Strict: hole vertices count too (one too close to its island would be a gouge).
        shortfall = 0.0;
        if (pullIn(g)) {
            kept.push_back(std::move(g));
            continue;
        }
        if (areaBefore <= 0.5 * tol.chordal * perimeter) {
            // A sliver (mean half-width, area / perimeter, under half the join tolerance:
            // Clipper2 debris where a lobe collapses, zero area in practice) that does not pass
            // as it is holds nothing the tolerance does not already allow for; an inset only
            // makes it vanish. Dropped without a report.
            continue;
        }
        // Some failing vertex could not be pulled in, or an edge comes too close: inset this
        // region further, by twice the worst shortfall left (at least 2e-6 mm) and then
        // doubling, until a further inset yields regions whose every vertex (and every edge the
        // pull-in changes) passes. Whether the repair is possible is decided by that result, not
        // by the size of the overshoot; a region that cannot be repaired is reported, never
        // silent, and so is one trimmed by more than the join tolerance.
        bool repaired = false;
        double extra = std::max(2.0 * shortfall, 2e-6);
        for (int attempt = 0; attempt < 12 && !repaired && extra < d; ++attempt) {
            std::vector<Group> parts = groupsOf(offsetBy(contoursOf(g), -extra, tol), tol.distance);
            repaired = !parts.empty() &&
                       std::all_of(parts.begin(), parts.end(), [&](Group& part) { return pullIn(part); });
            if (repaired) {
                double areaAfter = 0.0;
                for (Group& part : parts) {
                    areaAfter += groupArea(part);
                    kept.push_back(std::move(part));
                }
                const bool large =
                    extra > tol.chordal ||
                    areaBefore - areaAfter > perimeter * tol.chordal + tol.chordal * tol.chordal;
                if (large && report != nullptr) {
                    ++report->trimmed;
                }
            } else {
                extra *= 2.0;
            }
        }
        if (!repaired && report != nullptr) {
            ++report->dropped;
        }
    }
    return contoursOf(kept);
}

std::vector<Contour> insetBy(const std::vector<Contour>& cs, double d, const Tolerance& tol,
                             InsetGuard guard = InsetGuard::Lenient, InsetReport* report = nullptr) {
    return guardInset(cs, offsetBy(cs, -d, tol), d, tol, guard, report);
}

/// `r` with the vertices dropped that lie within `eps` of the chord kept round them
/// (Douglas-Peucker, closed ring split at vertex 0 and the vertex farthest from it). Every
/// dropped vertex is within `eps` of the result, so the region moves by at most `eps` either way.
Ring simplified(const Ring& r, double eps) {
    const std::size_t n = r.size();
    if (n <= 4) {
        return r;
    }
    std::size_t apex = 0;
    for (std::size_t i = 1; i < n; ++i) {
        if (geometry::distance(r[i], r[0]) > geometry::distance(r[apex], r[0])) {
            apex = i;
        }
    }
    if (apex == 0) {
        return r;
    }
    std::vector<char> keep(n, 0);
    keep[0] = keep[apex] = 1;
    // Index ranges (a, b), b may wrap past n (taken modulo n), with a < b.
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, apex}, {apex, n}};
    while (!stack.empty()) {
        const auto [a, b] = stack.back();
        stack.pop_back();
        double worst = 0.0;
        std::size_t at = a;
        for (std::size_t i = a + 1; i < b; ++i) {
            const double dd = distToSegment(r[i % n], r[a % n], r[b % n]);
            if (dd > worst) {
                worst = dd;
                at = i;
            }
        }
        if (worst > eps) {
            keep[at % n] = 1;
            stack.emplace_back(a, at);
            stack.emplace_back(at, b);
        }
    }
    Ring out;
    for (std::size_t i = 0; i < n; ++i) {
        if (keep[i] != 0) {
            out.push_back(r[i]);
        }
    }
    return out.size() >= 3 ? out : r;
}

/// Morphological opening: removes slivers narrower than 2·eps (numeric residue of booleans).
std::vector<Contour> opened(const std::vector<Contour>& cs, double eps, const Tolerance& tol) {
    return offsetBy(insetBy(cs, eps, tol), eps, tol);
}

std::vector<Contour> difference(const std::vector<Contour>& a, const std::vector<Contour>& b,
                                const Tolerance& tol) {
    if (a.empty()) {
        return {};
    }
    if (b.empty()) {
        return a;
    }
    return geometry::booleanOp(BooleanOp::Difference, a, b, tol).closed;
}

std::vector<Contour> intersection(const std::vector<Contour>& a, const std::vector<Contour>& b,
                                  const Tolerance& tol) {
    if (a.empty() || b.empty()) {
        return {};
    }
    return geometry::booleanOp(BooleanOp::Intersection, a, b, tol).closed;
}

/// Orients a ring for the cut direction: climb = counter-clockwise outer, clockwise hole.
Ring oriented(Ring r, bool isHole, CutDirection dir) {
    const bool wantCcw = (dir == CutDirection::Climb) != isHole;
    if ((ringArea(r) > 0.0) != wantCcw) {
        std::reverse(r.begin(), r.end());
    }
    return r;
}

/// Rotates a closed ring so it starts at its point nearest to `from` (inserting that point when
/// it falls inside an edge; ties go to the lowest y, then x); without `from` it starts at the
/// vertex with the smallest x, then y.
Ring startNear(Ring r, std::optional<Vec2> from, double tol) {
    const std::size_t n = r.size();
    if (n == 0) {
        return r;
    }
    std::size_t idx = 0;
    if (!from) {
        for (std::size_t i = 1; i < n; ++i) {
            if (r[i].x < r[idx].x || (r[i].x == r[idx].x && r[i].y < r[idx].y)) {
                idx = i;
            }
        }
        std::rotate(r.begin(), r.begin() + static_cast<std::ptrdiff_t>(idx), r.end());
        return r;
    }
    double best = std::numeric_limits<double>::infinity();
    std::size_t bestEdge = 0;
    Vec2 bestPoint = r[0];
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = r[i];
        const Vec2 ab = r[(i + 1) % n] - a;
        const double l2 = ab.lengthSquared();
        const double t = l2 > 0.0 ? std::clamp((*from - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
        const Vec2 q = a + ab * t;
        const double d = geometry::distance(*from, q);
        // Ties (equidistant feet, e.g. a corner of the previous ring) go to the lowest y, then
        // the lowest x, so the choice never depends on Clipper2's vertex order.
        const bool tie = std::fabs(d - best) <= 1e-9;
        if (d < best - 1e-9 || (tie && (q.y < bestPoint.y || (q.y == bestPoint.y && q.x < bestPoint.x)))) {
            best = d;
            bestEdge = i;
            bestPoint = q;
        }
    }
    const Vec2 a = r[bestEdge];
    const Vec2 b = r[(bestEdge + 1) % n];
    if (geometry::distance(bestPoint, a) <= tol) {
        idx = bestEdge;
    } else if (geometry::distance(bestPoint, b) <= tol) {
        idx = (bestEdge + 1) % n;
    } else {
        r.insert(r.begin() + static_cast<std::ptrdiff_t>(bestEdge + 1), bestPoint);
        idx = bestEdge + 1;
    }
    std::rotate(r.begin(), r.begin() + static_cast<std::ptrdiff_t>(idx), r.end());
    return r;
}

/// Appends the rings of `g` as closed paths in greedy nearest-first order, updating `cursor`.
void appendGroupRings(const Group& g, CutDirection dir, PocketPathRole role, std::size_t cluster,
                      std::optional<Vec2>& cursor, double tol, std::vector<PocketPath>& out) {
    std::vector<std::pair<Ring, bool>> todo;
    todo.emplace_back(g.outer, false);
    for (const Ring& h : g.holes) {
        todo.emplace_back(h, true);
    }
    std::vector<bool> used(todo.size(), false);
    for (std::size_t k = 0; k < todo.size(); ++k) {
        std::size_t pick = 0;
        double best = std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < todo.size(); ++i) {
            if (used[i]) {
                continue;
            }
            double d = 0.0;
            if (cursor) {
                d = std::numeric_limits<double>::infinity();
                const Ring& r = todo[i].first;
                for (std::size_t j = 0; j < r.size(); ++j) {
                    d = std::min(d, distToSegment(*cursor, r[j], r[(j + 1) % r.size()]));
                }
            }
            if (d < best) {
                best = d;
                pick = i;
            }
        }
        used[pick] = true;
        Ring r = startNear(oriented(todo[pick].first, todo[pick].second, dir), cursor, tol);
        cursor = r.front();
        out.push_back(PocketPath{Polyline{std::move(r), true}, role, cluster});
    }
}

// ------------------------------------------------------------------------- patterns

constexpr std::size_t kNoNode = std::numeric_limits<std::size_t>::max();

/// Containment tree of one cluster's Offset level sets: the rings of every node (outer + holes)
/// and its parent (kNoNode for the root, the cluster's centre region). Used to route travel moves
/// through the cleared centre region (ADR-0017 §8).
struct RingBridge {
    std::size_t ringA{0};
    std::size_t ringB{0};
    Vec2 a;  // on ringA
    Vec2 b;  // on ringB
};

class RingNearest;

struct RingTree {
    std::vector<std::vector<Ring>> rings;
    std::vector<std::size_t> parent;
    /// Per node, built on first use by `route`: bridges joining all of the node's rings.
    mutable std::vector<std::optional<std::vector<RingBridge>>> bridges;
    /// Per node, built on first use by `route`: an index of the node's ring edges (T-0131).
    mutable std::vector<std::shared_ptr<const RingNearest>> nearest;
};

/// A tool's planned paths plus what the emitter needs to route travel moves between them.
struct Pattern {
    std::vector<PocketPath> paths;
    /// Per path: its RingTree node (raster paths: node 0, the part itself).
    std::vector<std::size_t> node;
    /// Per cluster: the Offset level tree, or one node (the part) for Raster.
    std::vector<RingTree> trees;

    void add(std::vector<PocketPath>& fresh, std::size_t nodeId) {
        for (PocketPath& pp : fresh) {
            paths.push_back(std::move(pp));
            node.push_back(nodeId);
        }
        fresh.clear();
    }
};

/// Everything the pattern builders share.
struct PatternContext {
    const PocketParams& params;
    double stepover;
    double diameter;
    /// Polygon work (chordal = half the user's chordal tolerance).
    Tolerance tol;
    /// Swept areas and reach tests: round joins at ≤ 1e-4 mm, so join chords do not show up as
    /// fake residual material.
    Tolerance fine;
    /// Residual material narrower than 2·sliver is ignored (ADR-0017 §5, §10).
    double sliver;
    /// Where the tool may move straight inside the centre region (raster links; T-0169).
    const MoveCheck& moves;
    /// Raster pieces the toolpath may hold within the move cap (ADR-0017 §12, T-0123): the raster
    /// pattern is clipped in batches of lines and stops, setting `overBudget`, as soon as it
    /// holds more, so its memory stays bounded however many pieces the lines would split into.
    std::size_t pieceBudget{std::numeric_limits<std::size_t>::max()};
    /// Raster pieces built so far over all parts (may be null: no budget kept).
    std::size_t* piecesUsed{nullptr};
    /// Set by the raster builder when `pieceBudget` was exceeded (the pattern is then incomplete).
    bool* overBudget{nullptr};
};

/// Rings of one connected centre region `g`, cut inside out; the root (boundary) rings first,
/// last or as ordinary rings according to the Profile Pass.
void offsetPattern(const Group& g, const PatternContext& ctx, std::size_t cluster,
                   std::optional<Vec2>& cursor, Pattern& out) {
    const Tolerance& tol = ctx.tol;
    const double stepover = ctx.stepover;
    const double radius = 0.5 * ctx.diameter;
    struct Node {
        Group g;
        std::vector<std::size_t> children;
    };
    const std::vector<Contour> base = contoursOf(g);
    std::vector<std::vector<Group>> ordinals;  // level sets in order of increasing inset
    ordinals.push_back({g});
    std::vector<Group> next = groupsOf(insetBy(base, stepover, tol), tol.distance);
    const bool needExtra = stepover > radius + tol.distance;
    // Insets are measured from the region itself (not compounded) so errors do not accumulate.
    for (std::size_t k = 0;; ++k) {
        if (needExtra) {
            // Points deeper than `radius` inside level k that the next level's rings do not reach.
            const double extraInset = static_cast<double>(k) * stepover + radius;
            const std::vector<Contour> extra = insetBy(base, extraInset, tol);
            if (!extra.empty()) {
                const std::vector<Contour> reach = offsetBy(contoursOf(next), radius, ctx.fine);
                const std::vector<Contour> residual =
                    opened(difference(extra, reach, tol), ctx.sliver, ctx.fine);
                if (!residual.empty()) {
                    std::vector<Group> kept;
                    for (Group& eg : groupsOf(extra, tol.distance)) {
                        if (!intersection(contoursOf(eg), residual, tol).empty()) {
                            kept.push_back(std::move(eg));
                        }
                    }
                    if (!kept.empty()) {
                        ordinals.push_back(std::move(kept));
                    }
                }
            }
        }
        if (next.empty()) {
            break;
        }
        ordinals.push_back(next);
        next = groupsOf(insetBy(base, static_cast<double>(k + 2) * stepover, tol), tol.distance);
    }

    // Containment tree: each group's parent is the nearest earlier level group containing it.
    std::vector<Node> nodes;
    RingTree tree;
    std::vector<std::vector<std::size_t>> byOrdinal(ordinals.size());
    for (std::size_t o = 0; o < ordinals.size(); ++o) {
        for (Group& grp : ordinals[o]) {
            const Vec2 probe = grp.outer.front();
            const std::size_t id = nodes.size();
            tree.rings.push_back(ringsOf(grp));
            nodes.push_back(Node{std::move(grp), {}});
            byOrdinal[o].push_back(id);
            if (o == 0) {
                tree.parent.push_back(kNoNode);
                continue;
            }
            std::optional<std::size_t> parent;
            for (std::size_t q = o; q-- > 0 && !parent;) {
                for (std::size_t cand : byOrdinal[q]) {
                    if (insideRegion(tree.rings[cand], probe, tol.distance)) {
                        parent = cand;
                        break;
                    }
                }
            }
            tree.parent.push_back(parent.value_or(0));
            nodes[parent.value_or(0)].children.push_back(id);
        }
    }

    const ProfilePass pass = ctx.params.profilePass;
    const CutDirection dir = ctx.params.direction;
    std::vector<PocketPath> fresh;
    if (pass == ProfilePass::First) {
        appendGroupRings(nodes[0].g, dir, PocketPathRole::Profile, cluster, cursor, tol.distance, fresh);
        out.add(fresh, 0);
    }
    // Post-order (children first): the innermost rings are cut first.
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, 0}};
    while (!stack.empty()) {
        auto& [id, childIdx] = stack.back();
        if (childIdx < nodes[id].children.size()) {
            const std::size_t c = nodes[id].children[childIdx++];
            stack.emplace_back(c, 0);
            continue;
        }
        const std::size_t done = id;
        stack.pop_back();
        if (done == 0 && pass == ProfilePass::First) {
            continue;
        }
        const PocketPathRole role =
            done == 0 && pass == ProfilePass::Last ? PocketPathRole::Profile : PocketPathRole::Ring;
        appendGroupRings(nodes[done].g, dir, role, cluster, cursor, tol.distance, fresh);
        out.add(fresh, done);
    }
    if (out.trees.size() <= cluster) {
        out.trees.resize(cluster + 1);
    }
    out.trees[cluster] = std::move(tree);
}

struct RasterPiece {
    Vec2 a;  // lower u
    Vec2 b;  // higher u
    bool used{false};
};

void rasterPattern(const Group& g, const PatternContext& ctx, std::size_t cluster,
                   std::optional<Vec2>& cursor, Pattern& pattern) {
    const PocketParams& p = ctx.params;
    const Tolerance& tol = ctx.tol;
    std::vector<PocketPath> out;
    if (p.profilePass == ProfilePass::First) {
        appendGroupRings(g, p.direction, PocketPathRole::Profile, cluster, cursor, tol.distance, out);
    }
    const Vec2 u{std::cos(p.rasterAngle), std::sin(p.rasterAngle)};
    const Vec2 v = u.perp();
    double umin = std::numeric_limits<double>::infinity();
    double umax = -umin;
    double vmin = umin;
    double vmax = -umin;
    for (const Ring& r : ringsOf(g)) {
        for (const Vec2 q : r) {
            umin = std::min(umin, q.dot(u));
            umax = std::max(umax, q.dot(u));
            vmin = std::min(vmin, q.dot(v));
            vmax = std::max(vmax, q.dot(v));
        }
    }
    const double span = vmax - vmin;
    const auto n = static_cast<std::size_t>(std::max(1.0, std::ceil(span / ctx.stepover - 1e-9)));
    const double pitch = span / static_cast<double>(n);
    // Lines are clipped in batches (open paths do not interact, so the pieces are the same as in
    // one call), and the pieces counted against the budget after each batch. A big clip is cut
    // down to each batch's strip across it (T-0142, ADR-0006 §10).
    constexpr std::size_t kBatch = 256;
    const geometry::OpenPathClipper clipper(contoursOf(g), v, tol, FillRule::EvenOdd);
    std::vector<std::vector<RasterPiece>> rows(n);
    std::size_t pieces = 0;
    for (std::size_t k0 = 0; k0 < n; k0 += kBatch) {
        std::vector<Contour> lines;
        for (std::size_t k = k0; k < std::min(n, k0 + kBatch); ++k) {
            const double vk = vmin + (static_cast<double>(k) + 0.5) * pitch;
            Contour c;
            c.append(geometry::LineSpan{u * (umin - 1.0) + v * vk, u * (umax + 1.0) + v * vk});
            lines.push_back(std::move(c));
        }
        for (const Contour& c : clipper.clip(BooleanOp::Intersection, lines)) {
            if (c.isEmpty()) {
                continue;
            }
            Vec2 a = c.startPoint();
            Vec2 b = c.endPoint();
            if (geometry::distance(a, b) <= tol.distance) {
                continue;
            }
            if (a.dot(u) > b.dot(u)) {
                std::swap(a, b);
            }
            const double row = (0.5 * (a + b).dot(v) - vmin) / (pitch > 0.0 ? pitch : 1.0) - 0.5;
            const auto k =
                static_cast<std::size_t>(std::clamp(std::llround(row), 0LL, static_cast<long long>(n - 1)));
            // Clipper2 returns the ends on its 1e-6 mm grid; put them back on the exact line so
            // every raster piece is exactly parallel to the raster angle.
            const double vk = vmin + (static_cast<double>(k) + 0.5) * pitch;
            a = u * a.dot(u) + v * vk;
            b = u * b.dot(u) + v * vk;
            rows[k].push_back(RasterPiece{a, b});
            ++pieces;
        }
        if ((ctx.piecesUsed != nullptr ? *ctx.piecesUsed : 0) + pieces > ctx.pieceBudget) {
            if (ctx.overBudget != nullptr) {
                *ctx.overBudget = true;
            }
            return;
        }
    }
    if (ctx.piecesUsed != nullptr) {
        *ctx.piecesUsed += pieces;
    }
    for (auto& row : rows) {
        std::sort(row.begin(), row.end(),
                  [&](const RasterPiece& x, const RasterPiece& y) { return x.a.dot(u) < y.a.dot(u); });
    }

    // Zig-zag: continue on the next row with the piece nearest to the current end when the link
    // stays inside the region and is short; otherwise restart at the first unused piece.
    //
    // The next row is searched only round the cursor (T-0131; the whole row made a 1,000-tooth
    // comb quadratic): a piece is never nearer than the gap along the raster direction between
    // the cursor and the piece's span [a·u, b·u], so a piece whose span misses the window
    // cursor·u ± (linkLimit + slack) is farther than linkLimit, and if the nearest piece is that
    // far the run ends anyway. Rows are sorted by a·u; with the running maximum of b·u the pieces
    // whose span meets the window are found from the last one starting before its end, backwards,
    // and are then tried in index order as before (same choice, same ties).
    const RegionIndex region(ringsOf(g));
    const double linkLimit = 2.0 * ctx.diameter;
    std::vector<std::vector<double>> rowStart(n);  // a·u per piece, ascending
    std::vector<std::vector<double>> rowReach(n);  // running maximum of b·u
    for (std::size_t k = 0; k < n; ++k) {
        rowStart[k].reserve(rows[k].size());
        rowReach[k].reserve(rows[k].size());
        double reach = -std::numeric_limits<double>::infinity();
        for (const RasterPiece& piece : rows[k]) {
            rowStart[k].push_back(piece.a.dot(u));
            reach = std::max(reach, piece.b.dot(u));
            rowReach[k].push_back(reach);
        }
    }
    std::vector<std::size_t> window;
    auto emit = [&](std::size_t k, std::size_t i, bool fromA) {
        RasterPiece& piece = rows[k][i];
        piece.used = true;
        const Vec2 s = fromA ? piece.a : piece.b;
        const Vec2 e = fromA ? piece.b : piece.a;
        out.push_back(PocketPath{Polyline{{s, e}, false}, PocketPathRole::Raster, cluster});
        cursor = e;
    };
    for (std::size_t k0 = 0; k0 < n; ++k0) {
        for (std::size_t i0 = 0; i0 < rows[k0].size(); ++i0) {
            if (rows[k0][i0].used) {
                continue;
            }
            const RasterPiece& first = rows[k0][i0];
            const bool fromA =
                !cursor || geometry::distance(*cursor, first.a) <= geometry::distance(*cursor, first.b);
            emit(k0, i0, fromA);
            for (std::size_t k = k0 + 1; k < n; ++k) {
                std::optional<std::size_t> best;
                bool bestFromA = true;
                double bestD = std::numeric_limits<double>::infinity();
                const double cu = cursor->dot(u);
                const double slack = linkLimit + tol.distance + 1e-9 * (1.0 + std::fabs(cu) + linkLimit);
                window.clear();
                {
                    const auto last = std::upper_bound(rowStart[k].begin(), rowStart[k].end(), cu + slack);
                    for (auto i = static_cast<std::size_t>(last - rowStart[k].begin()); i-- > 0;) {
                        if (rowReach[k][i] < cu - slack) {
                            break;  // this piece and every earlier one end before the window
                        }
                        window.push_back(i);
                    }
                    std::reverse(window.begin(), window.end());
                }
                for (const std::size_t i : window) {
                    if (rows[k][i].used) {
                        continue;
                    }
                    const double da = geometry::distance(*cursor, rows[k][i].a);
                    const double db = geometry::distance(*cursor, rows[k][i].b);
                    if (std::min(da, db) < bestD) {
                        bestD = std::min(da, db);
                        best = i;
                        bestFromA = da <= db;
                    }
                }
                if (!best) {
                    break;
                }
                const Vec2 s = bestFromA ? rows[k][*best].a : rows[k][*best].b;
                if (bestD > linkLimit || !ctx.moves.allows(region, *cursor, s)) {
                    break;
                }
                emit(k, *best, bestFromA);
            }
        }
    }
    if (p.profilePass == ProfilePass::Last) {
        appendGroupRings(g, p.direction, PocketPathRole::Profile, cluster, cursor, tol.distance, out);
    }
    // One tree node: the part's own rings. Travel between runs (islands split the lines) is
    // routed along them like the Offset strategy's (ADR-0017 §8, T-0123).
    pattern.add(out, 0);
    if (pattern.trees.size() <= cluster) {
        pattern.trees.resize(cluster + 1);
    }
    pattern.trees[cluster].rings = {ringsOf(g)};
    pattern.trees[cluster].parent = {kNoNode};
}

/// An order for `parts` by their bounding-box centres (`detail::nearestFirstOrder`).
std::vector<std::size_t> nearestFirst(const std::vector<Group>& parts, const std::vector<std::size_t>& partOf) {
    std::vector<Vec2> centres(parts.size());
    for (std::size_t i = 0; i < parts.size(); ++i) {
        centres[i] = ringBox(parts[i].outer).centre();
    }
    return detail::nearestFirstOrder(centres, partOf);
}

/// The full pattern of one tool over every connected part of its centre region.
Pattern fullPattern(const std::vector<Group>& centre, const PatternContext& ctx) {
    Pattern out;
    std::optional<Vec2> cursor;
    for (std::size_t ci = 0; ci < centre.size() && !(ctx.overBudget != nullptr && *ctx.overBudget); ++ci) {
        if (ctx.params.strategy == PocketStrategy::Offset) {
            offsetPattern(centre[ci], ctx, ci, cursor, out);
        } else {
            rasterPattern(centre[ci], ctx, ci, cursor, out);
        }
    }
    return out;
}

// ------------------------------------------------------------------------- travel routing

/// The point of `rings` nearest to `p`, with the ring and edge it lies on.
struct RingPoint {
    std::size_t ring{0};
    std::size_t edge{0};
    double t{0.0};
    Vec2 point;
};

/// Distance from `p` to edge `i` of ring `r` (from r[i] to the next vertex), with the parameter
/// and the point of the edge nearest to `p`.
double edgeDistance(const Ring& r, std::size_t i, Vec2 p, double& t, Vec2& q) {
    const Vec2 a = r[i];
    const Vec2 ab = r[(i + 1) % r.size()] - a;
    const double l2 = ab.lengthSquared();
    t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
    q = a + ab * t;
    return geometry::distance(p, q);
}

RingPoint nearestOn(const std::vector<Ring>& rings, Vec2 p) {
    RingPoint best;
    double bestD = std::numeric_limits<double>::infinity();
    for (std::size_t ri = 0; ri < rings.size(); ++ri) {
        const Ring& r = rings[ri];
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            double t = 0.0;
            Vec2 q;
            const double d = edgeDistance(r, i, p, t, q);
            if (d < bestD) {
                bestD = d;
                best = RingPoint{ri, i, t, q};
            }
        }
    }
    return best;
}

/// `nearestOn` for one fixed set of rings, through a `BoxIndex` of the edges (T-0131: travel
/// routes along a ring of thousands of edges searched all of them per route). A square of
/// half-width h round p is searched, ×4 until the best distance found is below h by a margin
/// (an edge whose box misses the square is farther than h) or the square covers every edge;
/// candidates are tried in (ring, edge) order with the same strict comparison, so the result is
/// `nearestOn`'s, ties included. `only` restricts the search to one ring (the result's `ring`
/// is then that ring).
class RingNearest {
   public:
    explicit RingNearest(const std::vector<Ring>& rings) {
        std::vector<geometry::Box2> boxes;
        double scale = 0.0;
        for (std::size_t ri = 0; ri < rings.size(); ++ri) {
            const Ring& r = rings[ri];
            for (std::size_t i = 0, n = r.size(); i < n; ++i) {
                edges_.emplace_back(ri, i);
                boxes.emplace_back(r[i], r[(i + 1) % n]);
                scale = std::max({scale, std::fabs(r[i].x), std::fabs(r[i].y)});
            }
        }
        scale_ = scale;
        index_ = BoxIndex(std::move(boxes));
        const geometry::Box2& b = index_.bounds();
        const double count = static_cast<double>(std::max<std::size_t>(edges_.size(), 1));
        start_ = std::max(0.5 * std::max(b.width(), b.height()) / std::sqrt(count), 1e-6);
    }

    RingPoint nearest(const std::vector<Ring>& rings, Vec2 p, std::size_t only = kNoNode) const {
        RingPoint best;
        if (only != kNoNode) {
            best.ring = only;
        }
        auto scan = [&](const std::vector<std::size_t>& candidates, bool all) {
            RingPoint b = best;
            double bestD = std::numeric_limits<double>::infinity();
            auto visit = [&](std::size_t e) {
                const auto [ri, i] = edges_[e];
                if (only != kNoNode && ri != only) {
                    return;
                }
                double t = 0.0;
                Vec2 q;
                const double d = edgeDistance(rings[ri], i, p, t, q);
                if (d < bestD) {
                    bestD = d;
                    b = RingPoint{only != kNoNode ? only : ri, i, t, q};
                }
            };
            if (all) {
                for (std::size_t e = 0; e < edges_.size(); ++e) visit(e);
            } else {
                for (const std::size_t e : candidates) visit(e);
            }
            return std::pair<RingPoint, double>{b, bestD};
        };
        if (edges_.size() <= 32 || !std::isfinite(p.x) || !std::isfinite(p.y)) {
            return scan({}, true).first;
        }
        const geometry::Box2& bounds = index_.bounds();
        const double margin = 1e-9 + 1e-10 * std::max({scale_, std::fabs(p.x), std::fabs(p.y)});
        for (double h = start_;; h *= 4.0) {
            const geometry::Box2 square{p - Vec2{h, h}, p + Vec2{h, h}};
            const bool all = !std::isfinite(h) || square.contains(bounds);
            const auto [b, d] = scan(all ? std::vector<std::size_t>{} : index_.query(square), all);
            if (all || d + margin < h) {
                return b;
            }
        }
    }

   private:
    std::vector<std::pair<std::size_t, std::size_t>> edges_;  // (ring, edge), in nearestOn's order
    BoxIndex index_;
    double scale_{0.0};
    double start_{1e-6};
};

/// The shorter way along ring `r` from `a` to `b` (both on `r`), end points included: forward
/// (a, r[a.edge + 1], …, r[b.edge], b) or backward (a, r[a.edge], …, r[b.edge + 1], b), the
/// forward one when its length (summed in order) is <= the backward one's.
///
/// Both ways are walked together, the one with the shorter length so far stepping (T-0131: a
/// ring of thousands of edges was walked all round per route). A sum of lengths in order never
/// decreases, so once one way is complete and the other's partial sum already decides the
/// comparison, the rest of the longer way cannot change it: the choice, and the points, are
/// those of comparing the two complete sums.
std::vector<Vec2> walkRing(const Ring& r, const RingPoint& a, const RingPoint& b) {
    const std::size_t n = r.size();
    struct Way {
        std::size_t count;    // ring vertices between the end points
        bool forward;
        std::size_t done;     // points visited after a (vertices, then b)
        double length;
        Vec2 last;
    };
    const std::size_t fwdCount = a.edge == b.edge && a.t <= b.t ? 0 : (b.edge + 2 * n - a.edge - 1) % n + 1;
    const std::size_t bwdCount = a.edge == b.edge && a.t >= b.t ? 0 : (a.edge + 2 * n - b.edge - 1) % n + 1;
    Way fwd{fwdCount, true, 0, 0.0, a.point};
    Way bwd{bwdCount, false, 0, 0.0, a.point};
    auto pointOf = [&](const Way& w, std::size_t k) {  // k-th point after a, k < count + 1
        if (k == w.count) {
            return b.point;
        }
        return w.forward ? r[(a.edge + 1 + k) % n] : r[(a.edge + n - k % n) % n];
    };
    auto complete = [](const Way& w) { return w.done == w.count + 1; };
    auto step = [&](Way& w) {
        const Vec2 q = pointOf(w, w.done);
        w.length += geometry::distance(w.last, q);
        w.last = q;
        ++w.done;
    };
    bool forward = true;
    for (;;) {
        if (complete(fwd) && complete(bwd)) {
            forward = fwd.length <= bwd.length;
            break;
        }
        if (complete(fwd) && bwd.length >= fwd.length) {
            forward = true;  // backward total >= its partial sum >= forward total
            break;
        }
        if (complete(bwd) && fwd.length > bwd.length) {
            forward = false;  // forward total >= its partial sum > backward total
            break;
        }
        if (!complete(fwd) && (complete(bwd) || fwd.length <= bwd.length)) {
            step(fwd);
        } else {
            step(bwd);
        }
    }
    const Way& w = forward ? fwd : bwd;
    std::vector<Vec2> out{a.point};
    out.reserve(w.count + 2);
    for (std::size_t k = 0; k <= w.count; ++k) {
        out.push_back(pointOf(w, k));
    }
    return out;
}

/// Closest pair of points between rings `ra` and `rb` (vertex/edge pairs, both ways).
std::pair<Vec2, Vec2> closestPair(const Ring& ra, const Ring& rb) {
    double best = std::numeric_limits<double>::infinity();
    std::pair<Vec2, Vec2> out{ra.front(), rb.front()};
    auto scan = [&](const Ring& from, const Ring& to, bool swapped) {
        for (const Vec2 v : from) {
            const RingPoint q = nearestOn({to}, v);
            const double d = geometry::distance(v, q.point);
            if (d < best) {
                best = d;
                out = swapped ? std::pair<Vec2, Vec2>{q.point, v} : std::pair<Vec2, Vec2>{v, q.point};
            }
        }
    };
    scan(ra, rb, false);
    scan(rb, ra, true);
    return out;
}

/// Bridges joining the rings of one region (outer + holes) into one connected set: Kruskal over
/// the closest pairs between rings, shortest first. Such a bridge cannot be blocked by a third
/// ring: that ring would be closer to one end and would already have been joined to it. Each
/// bridge is still checked with `segmentInside` (rounding); a rejected one is skipped, so the
/// set may stay disconnected, and `acrossRings` then fails and the route climbs a level.
std::vector<RingBridge> ringBridges(const std::vector<Ring>& rings, const MoveCheck& moves) {
    struct Candidate {
        double length;
        std::size_t i;
        std::size_t j;
        Vec2 a;
        Vec2 b;
    };
    std::vector<Candidate> cands;
    for (std::size_t i = 0; i < rings.size(); ++i) {
        for (std::size_t j = i + 1; j < rings.size(); ++j) {
            const auto [x, y] = closestPair(rings[i], rings[j]);
            cands.push_back({geometry::distance(x, y), i, j, x, y});
        }
    }
    std::sort(cands.begin(), cands.end(), [](const Candidate& p, const Candidate& q) {
        return p.length < q.length || (p.length == q.length && (p.i < q.i || (p.i == q.i && p.j < q.j)));
    });
    std::vector<std::size_t> root(rings.size());
    for (std::size_t i = 0; i < root.size(); ++i) {
        root[i] = i;
    }
    auto find = [&](std::size_t x) {
        while (root[x] != x) {
            x = root[x] = root[root[x]];
        }
        return x;
    };
    std::vector<RingBridge> out;
    for (const Candidate& c : cands) {
        const std::size_t ri = find(c.i);
        const std::size_t rj = find(c.j);
        if (ri == rj || !moves.allows(rings, c.a, c.b)) {
            continue;
        }
        root[ri] = rj;
        out.push_back(RingBridge{c.i, c.j, c.a, c.b});
    }
    return out;
}

/// A way inside one region from `a` to `b` (points on its rings): along the shared ring, or along
/// rings and across bridges, ring to ring by the fewest bridges (BFS). Empty when the bridges do
/// not connect the two rings.
std::vector<Vec2> acrossRings(const std::vector<Ring>& rings, const std::vector<RingBridge>& bridges,
                              const RingNearest& nearest, const RingPoint& a, const RingPoint& b) {
    if (a.ring == b.ring) {
        return walkRing(rings[a.ring], a, b);
    }
    // BFS over rings; `via[r]` is the bridge index that reached ring r.
    std::vector<std::size_t> via(rings.size(), kNoNode);
    std::vector<bool> seen(rings.size(), false);
    std::vector<std::size_t> queue{a.ring};
    seen[a.ring] = true;
    for (std::size_t qi = 0; qi < queue.size() && !seen[b.ring]; ++qi) {
        const std::size_t r = queue[qi];
        for (std::size_t bi = 0; bi < bridges.size(); ++bi) {
            const RingBridge& br = bridges[bi];
            const std::size_t other = br.ringA == r ? br.ringB : (br.ringB == r ? br.ringA : kNoNode);
            if (other != kNoNode && !seen[other]) {
                seen[other] = true;
                via[other] = bi;
                queue.push_back(other);
            }
        }
    }
    if (!seen[b.ring]) {
        return {};
    }
    // Bridges from a.ring to b.ring, each as (point on the ring we leave, point on the next).
    std::vector<std::pair<RingPoint, RingPoint>> hops;
    for (std::size_t r = b.ring; r != a.ring;) {
        const RingBridge& br = bridges[via[r]];
        const bool forward = br.ringB == r;  // crossing ringA -> ringB
        const std::size_t from = forward ? br.ringA : br.ringB;
        const Vec2 pf = forward ? br.a : br.b;
        const Vec2 pt = forward ? br.b : br.a;
        const RingPoint leave = nearest.nearest(rings, pf, from);
        const RingPoint arrive = nearest.nearest(rings, pt, r);
        hops.emplace_back(leave, arrive);
        r = from;
    }
    std::reverse(hops.begin(), hops.end());
    std::vector<Vec2> way;
    RingPoint cur = a;
    for (const auto& [leave, arrive] : hops) {
        const std::vector<Vec2> walk = walkRing(rings[cur.ring], cur, leave);
        way.insert(way.end(), walk.begin(), walk.end());
        cur = arrive;
    }
    const std::vector<Vec2> last = walkRing(rings[cur.ring], cur, b);
    way.insert(way.end(), last.begin(), last.end());
    return way;
}

/// A way from P (on node `na`'s rings) to Q (on node `nb`'s rings) that stays inside the
/// cluster's centre region, or empty. Climbing to a parent goes straight to the parent
/// boundary's nearest point: the open disc around a point of a region up to its nearest boundary
/// point lies inside the region, so that segment is inside. The two climbs meet at the lowest
/// common ancestor, where the way follows its rings and the bridges between them
/// (`ringBridges`, `acrossRings`); only if those do not connect does it climb further. The result
/// is shortened greedily (a later point replaces the ones in between when the straight segment
/// to it stays inside `region`). `inAir`: the way is at the surface (`MoveCheck::allowsInAir` for
/// the shortening; the bridges, cached per node for every level, always use `allows`).
std::vector<Vec2> route(const RingTree& tree, std::size_t na, Vec2 P, std::size_t nb, Vec2 Q,
                        const RegionIndex& region, const MoveCheck& moves, bool inAir) {
    auto ancestors = [&](std::size_t n) {
        std::vector<std::size_t> out;
        for (std::size_t x = n; x != kNoNode; x = tree.parent[x]) {
            out.push_back(x);
        }
        return out;
    };
    const std::vector<std::size_t> ancA = ancestors(na);
    const std::vector<std::size_t> ancB = ancestors(nb);
    std::size_t lca = kNoNode;
    std::size_t ia = 0;
    for (; ia < ancA.size() && lca == kNoNode; ++ia) {
        if (std::find(ancB.begin(), ancB.end(), ancA[ia]) != ancB.end()) {
            lca = ancA[ia];
        }
    }
    if (lca == kNoNode) {
        return {};
    }
    if (tree.nearest.size() < tree.rings.size()) {
        tree.nearest.resize(tree.rings.size());
    }
    auto nearestIn = [&](std::size_t node) -> const RingNearest& {
        if (!tree.nearest[node]) {
            tree.nearest[node] = std::make_shared<const RingNearest>(tree.rings[node]);
        }
        return *tree.nearest[node];
    };
    auto climb = [&](const std::vector<std::size_t>& anc, Vec2 from) {
        std::vector<Vec2> pts{from};
        for (std::size_t i = 1; i < anc.size() && anc[i - 1] != lca; ++i) {
            pts.push_back(nearestIn(anc[i]).nearest(tree.rings[anc[i]], pts.back()).point);
        }
        return pts;
    };
    std::vector<Vec2> up = climb(ancA, P);
    std::vector<Vec2> down = climb(ancB, Q);
    std::vector<Vec2> mid;
    if (tree.bridges.size() < tree.rings.size()) {
        tree.bridges.resize(tree.rings.size());
    }
    for (std::size_t node = lca;;) {
        const std::vector<Ring>& rings = tree.rings[node];
        if (!tree.bridges[node]) {
            tree.bridges[node] = ringBridges(rings, moves);
        }
        const RingNearest& edges = nearestIn(node);
        const RingPoint a = edges.nearest(rings, up.back());
        const RingPoint b = edges.nearest(rings, down.back());
        mid = acrossRings(rings, *tree.bridges[node], edges, a, b);
        if (!mid.empty()) {
            break;
        }
        if (tree.parent[node] == kNoNode) {
            return {};
        }
        node = tree.parent[node];
        up.push_back(nearestIn(node).nearest(tree.rings[node], up.back()).point);
        down.push_back(nearestIn(node).nearest(tree.rings[node], down.back()).point);
    }
    std::vector<Vec2> way = up;
    way.insert(way.end(), mid.begin(), mid.end());
    way.insert(way.end(), down.rbegin(), down.rend());
    // Drop repeated points, then pull the string.
    std::vector<Vec2> clean;
    for (const Vec2 q : way) {
        if (clean.empty() || geometry::distance(clean.back(), q) > moves.mergeDistance(inAir)) {
            clean.push_back(q);
        }
    }
    if (clean.size() < 2) {
        return clean.empty() ? std::vector<Vec2>{P, Q} : std::vector<Vec2>{clean.front(), Q};
    }
    std::vector<Vec2> pulled{clean.front()};
    for (std::size_t i = 0; i + 1 < clean.size();) {
        std::size_t j = i + 1;
        while (j + 1 < clean.size() &&
               (inAir ? moves.allowsInAir(region, clean[i], clean[j + 1]) : moves.allows(region, clean[i], clean[j + 1]))) {
            ++j;
        }
        pulled.push_back(clean[j]);
        i = j;
    }
    pulled.back() = Q;
    return pulled;
}

// ------------------------------------------------------------------------- move emission

class Emitter {
   public:
    /// `allowTravel` false: every way that is not a short link at depth retracts (used when the
    /// floor of the previous level is not known to be clear, ADR-0017 §8).
    Emitter(Toolpath& tp, const PocketParams& p, double linkLimit, bool allowTravel, const MoveCheck& moves)
        : tp_(tp),
          p_(p),
          moves_(moves),
          linkLimit_(linkLimit),
          minRamp_(10.0 * p.tolerance.distance),
          allowTravel_(allowTravel) {}

    /// Cuts the paths of one cluster at every level. Short links inside the centre region feed at
    /// depth; longer ways (and the way back to the first path of the next level) travel inside the
    /// centre region at the floor of the previous level (the surface for the first), which is
    /// already cleared there, along a straight segment or a route through `tree`; only when
    /// neither exists does the tool retract to safe Z. Between clusters (separate regions, or
    /// separate rest areas: owner decision 2026-11-01) the tool always retracts.
    void cluster(const std::vector<const PocketPath*>& paths, const std::vector<std::size_t>& nodes,
                 const std::vector<double>& levels, const RegionIndex& region, const RingTree* tree) {
        for (std::size_t k = 0; k < levels.size(); ++k) {
            const double z = levels[k];
            const double zTop = k == 0 ? -p_.startDepth : levels[k - 1];
            for (std::size_t i = 0; i < paths.size(); ++i) {
                const Polyline& path = paths[i]->path;
                const Vec2 s = path.points.front();
                const std::size_t node = nodes[i];
                if (!started_) {
                    rapidTo(s);
                    enter(path, zTop, z);
                } else if (!atSafe_ && z_ == z && geometry::distance(pos_, s) <= linkLimit_ &&
                           moves_.allows(region, pos_, s)) {
                    feedTo(s, z);
                } else if (allowTravel_ && !atSafe_ && inCluster_ && z_ <= zTop &&
                           travel(s, zTop, node, region, tree)) {
                    enter(path, zTop, z);
                } else {
                    retract();
                    rapidTo(s);
                    enter(path, zTop, z);
                }
                cut(path, z);
                node_ = node;
                inCluster_ = true;
            }
        }
        inCluster_ = false;
    }

    void finish() { retract(); }

   private:
    /// Lifts to `zTravel` and feeds to `s` inside the centre region; false when there is no way.
    bool travel(Vec2 s, double zTravel, std::size_t node, const RegionIndex& region, const RingTree* tree) {
        // At the surface (the first level) the tool touches no material (T-0175, ADR-0017 §8a).
        const bool inAir = zTravel >= -p_.startDepth;
        std::vector<Vec2> way;
        if (inAir ? moves_.allowsInAir(region, pos_, s) : moves_.allows(region, pos_, s)) {
            way = {pos_, s};
        } else if (tree != nullptr && !tree->rings.empty() && node != kNoNode && node_ != kNoNode) {
            way = route(*tree, node_, pos_, node, s, region, moves_, inAir);
        }
        if (way.size() < 2) {
            return false;
        }
        if (z_ < zTravel) {
            feedTo(pos_, zTravel);  // lift onto the cleared floor above
        }
        for (std::size_t i = 1; i < way.size(); ++i) {
            feedTo(way[i], zTravel);
        }
        return true;
    }
    void retract() {
        if (!atSafe_) {
            tp_.append(Move::retract(Vec3{pos_, p_.safeZ}));
            atSafe_ = true;
            z_ = p_.safeZ;
        }
    }
    void rapidTo(Vec2 q) {
        tp_.append(Move::rapid(Vec3{q, p_.safeZ}));
        pos_ = q;
        z_ = p_.safeZ;
        started_ = true;
        atSafe_ = true;
    }
    void feedTo(Vec2 q, double z) {
        if (geometry::distance(q, pos_) > 0.0 || z != z_) {
            tp_.append(Move::feed(Vec3{q, z}));
        }
        pos_ = q;
        z_ = z;
    }
    void enter(const Polyline& path, double zFrom, double zTo) {
        if (p_.rampPlunge && p_.rampDistance > 0.0) {
            if (z_ > zFrom) {
                tp_.append(Move::plunge(Vec3{pos_, zFrom}));
            }
            appendPathRamp(path, zFrom, zTo, p_.rampDistance, minRamp_, p_.tolerance, tp_);
        } else {
            tp_.append(Move::plunge(Vec3{pos_, zTo}));
        }
        z_ = zTo;
        atSafe_ = false;
    }
    void cut(const Polyline& path, double z) {
        for (std::size_t i = 1; i < path.points.size(); ++i) {
            feedTo(path.points[i], z);
        }
        if (path.closed) {
            feedTo(path.points.front(), z);
        }
    }

    Toolpath& tp_;
    const PocketParams& p_;
    const MoveCheck& moves_;
    double linkLimit_;
    double minRamp_;
    bool allowTravel_;
    bool started_{false};
    bool atSafe_{true};
    bool inCluster_{false};
    std::size_t node_{kNoNode};
    Vec2 pos_;
    double z_{0.0};
};

bool finite(double v) {
    return std::isfinite(v);
}

/// The region the tool centre may visit: the pocket region inset by radius + allowance + margin
/// with the strict guard (ADR-0017 §3, §4). Shared by `pocketToolpath` and `pocketCentreRegion`.
std::vector<Contour> centreRegionOf(const std::vector<Contour>& region, double radius, double allowance,
                                    double margin, const Tolerance& tol, InsetReport& report) {
    return insetBy(region, radius + allowance + margin, tol, InsetGuard::Strict, &report);
}

/// A diagnostic, optionally about tool `tool` of the list.
ToolpathDiagnostic note(ToolpathIssue issue, std::string message, std::optional<std::size_t> tool = std::nullopt) {
    return ToolpathDiagnostic{issue, std::nullopt, std::move(message), tool};
}

/// The diagnostics for what the strict guard could not keep exactly (ADR-0017 §4); `prefix`
/// names the tool.
std::vector<ToolpathDiagnostic> centreRegionNotes(const InsetReport& report, const std::string& prefix,
                                                  std::optional<std::size_t> tool) {
    std::vector<ToolpathDiagnostic> out;
    if (report.dropped > 0) {
        out.push_back(note(ToolpathIssue::RegionNotKept,
                           prefix + std::to_string(report.dropped) +
                               " part(s) of the pocket could not be kept within the safety margin and are not cut",
                           tool));
    }
    if (report.trimmed > 0) {
        out.push_back(note(ToolpathIssue::RegionTrimmed,
                           prefix + std::to_string(report.trimmed) +
                               " part(s) of the pocket were trimmed by more than the tolerance to keep the safety "
                               "margin; material near them may be left",
                           tool));
    }
    return out;
}

/// Vertices the polygonisation of `c` will produce at `tol` (lines 1, arcs by sag, Beziers
/// bounded), for the vertex cap (ADR-0017 §12).
double estimatedVertices(const Contour& c, const Tolerance& tol) {
    double n = 0.0;
    for (const auto& span : c.spans) {
        n += std::visit(
            [&](const auto& sp) -> double {
                using S = std::decay_t<decltype(sp)>;
                if constexpr (std::is_same_v<S, geometry::ArcSpan>) {
                    return static_cast<double>(sp.segmentCountFor(tol.chordal));
                } else if constexpr (std::is_same_v<S, geometry::BezierSpan>) {
                    const double len = geometry::distance(sp.p0, sp.p1) + geometry::distance(sp.p1, sp.p2) +
                                       geometry::distance(sp.p2, sp.p3);
                    return 1.0 + std::sqrt(len / tol.chordal);
                } else {
                    return 1.0;
                }
            },
            span);
    }
    return n;
}

/// The polygon tolerance used for region work: half the user's chordal tolerance (ADR-0017 §3).
Tolerance regionTolerance(const Tolerance& user) {
    Tolerance tol = user;
    tol.chordal = 0.5 * user.chordal;
    return tol;
}

// Work caps (ADR-0017 §12): anything beyond them is refused with a message before any
// expensive work is done.
constexpr double kMinChordal = 1e-4;       // mm
constexpr double kMinDistance = 1e-6;      // mm, one unit of the offset grid (T-0142, as Profile)
constexpr double kMaxLevels = 10000.0;     // depth passes per tool
constexpr double kMaxPasses = 20000.0;     // rings or raster lines per cluster
constexpr double kMaxVertices = 200000.0;  // vertices of the polygonised pocket region
constexpr double kMaxMoves = 20000000.0;   // estimated moves per toolpath

}  // namespace

bool PocketResult::ok() const {
    return std::any_of(toolpaths.begin(), toolpaths.end(), [](const Toolpath& t) { return !t.isEmpty(); });
}

PocketResult pocketToolpath(const std::vector<Contour>& contours, const std::vector<PocketTool>& tools,
                            const PocketParams& params) {
    PocketResult result;
    const Tolerance& userTol = params.tolerance;
    // Polygonise inputs and approximate round joins at half the chordal tolerance, and keep the
    // tool centre one chordal tolerance further from the walls: the polygon error on the unsafe
    // side (≤ chordal/2) plus the join chords (≤ chordal/2) never bring the tool closer than
    // radius + allowance to the true walls (ADR-0017).
    Tolerance tol = regionTolerance(userTol);
    Tolerance fine = tol;
    fine.chordal = std::min(tol.chordal, 1e-4);
    const double margin = userTol.chordal;
    // Residual material thinner than one chordal tolerance (at most 0.01 mm) is noise.
    const double sliver = std::min(0.5 * userTol.chordal, 0.005);

    result.toolpaths.resize(tools.size());
    result.plans.resize(tools.size());
    // The Tool -> pass tool conversion (T-0123): radius at the bottom of the pocket, stepover and
    // pass depth from the cutting data, snapshot for the toolpath.
    std::vector<PassTool> passTools;
    for (std::size_t i = 0; i < tools.size(); ++i) {
        passTools.push_back(passTool(tools[i].tool, effectiveCuttingData(tools[i].tool, tools[i].cuttingData),
                                     params.startDepth + params.cutDepth));
        result.toolpaths[i].name =
            tools.size() > 1 ? params.name + " [" + tools[i].tool.name + "]" : params.name;
        result.toolpaths[i].tool = passTools[i].snapshot;
    }
    auto say = [&](ToolpathIssue issue, std::string message, std::optional<std::size_t> tool = std::nullopt) {
        result.diagnostics.push_back(note(issue, std::move(message), tool));
    };
    if (tools.empty()) {
        say(ToolpathIssue::NoTool, "no tool selected");
        return result;
    }
    if (!finite(params.cutDepth) || !(params.cutDepth > 0.0) || !finite(params.startDepth) ||
        params.startDepth < 0.0 || !finite(params.allowance) || params.allowance < 0.0 ||
        !finite(params.safeZ) || !finite(params.rasterAngle) || !finite(params.rampDistance) ||
        !finite(params.lastPassThickness)) {
        say(ToolpathIssue::InvalidParameters,
            "invalid pocket parameters: cut depth must be > 0, start depth and allowance >= 0, all values "
            "finite");
        return result;
    }
    if (!finite(userTol.chordal) || userTol.chordal < kMinChordal || userTol.chordal > 1.0 ||
        !finite(userTol.distance) || !(userTol.distance >= kMinDistance) || userTol.distance > userTol.chordal) {
        say(ToolpathIssue::InvalidParameters,
            "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance between 0.000001 mm and chordal");
        return result;
    }
    if (params.safeZ <= -params.startDepth) {
        say(ToolpathIssue::InvalidParameters, "safe Z must be above the start depth");
        return result;
    }

    std::vector<Contour> closedInput;
    std::size_t openCount = 0;
    double inputVertices = 0.0;
    for (const Contour& c : contours) {
        if (c.closed) {
            inputVertices += estimatedVertices(c, tol);
            closedInput.push_back(c);
        } else if (!c.isEmpty()) {
            ++openCount;
        }
    }
    if (openCount > 0) {
        say(ToolpathIssue::OpenContourIgnored,
            std::to_string(openCount) + " open contour(s) ignored: a pocket needs closed vectors");
    }
    if (!(inputVertices <= kMaxVertices)) {
        say(ToolpathIssue::WorkLimitExceeded,
            "pocket boundary too detailed at this chordal tolerance (more than " +
                std::to_string(static_cast<long long>(kMaxVertices)) + " vertices)");
        return result;
    }
    const std::vector<Contour> region = geometry::unionContours(closedInput, tol, FillRule::EvenOdd);
    if (region.empty()) {
        say(ToolpathIssue::NoRegion, "no closed region to pocket");
        return result;
    }
    // The polygonised walls, for the moves that leave a centre region by more than rounding
    // (MoveCheck, ADR-0017 §8a), and how far the true walls may lie from them: the polygon
    // tolerance, or nothing when every input span is a line (the polygon is then the walls).
    const bool curvedWalls = std::any_of(closedInput.begin(), closedInput.end(), [](const Contour& c) {
        return std::any_of(c.spans.begin(), c.spans.end(),
                           [](const geometry::Span& sp) { return !std::holds_alternative<geometry::LineSpan>(sp); });
    });
    const double wallSag = curvedWalls ? tol.chordal : 0.0;
    const WallIndex walls(region);
    std::optional<std::vector<Contour>> target;  // pocket less allowance; only rest tools need it
    std::vector<Box2> regionBoxes;
    double regionVertices = 0.0;
    for (const Group& g : groupsOf(region, tol.distance)) {
        Box2 box;
        for (const Ring& ring : ringsOf(g)) {
            regionVertices += static_cast<double>(ring.size());
            for (const Vec2 q : ring) {
                box.expand(q);
            }
        }
        regionBoxes.push_back(box);
    }

    std::vector<Contour> swept;  // area cut by the tools so far (pocket floor)
    for (std::size_t ti = 0; ti < tools.size(); ++ti) {
        const PassTool& pt = passTools[ti];
        const double d = 2.0 * pt.radius;  // cutting width at the bottom of the pocket
        const std::string label = "tool " + std::to_string(ti + 1) + " (" + tools[ti].tool.name + ")";
        if (tools[ti].tool.type == tooling::ToolType::DiamondDrag) {
            // T-0154 (owner decision 2026-11-03): a drag tool must not turn; only Quick Engrave
            // runs toolpaths with the spindle off (its Fill hands Pocket an end mill of the same width).
            say(ToolpathIssue::InvalidTool,
                label + ": skipped, a diamond drag tool must not turn; use Quick Engrave for diamond-drag tools", ti);
            continue;
        }
        if (!finite(d) || !(d > 0.0) || !finite(pt.stepover) || !(pt.stepover > 0.0) ||
            pt.stepover > d * (1.0 + 1e-12) || !finite(pt.passDepth) || !(pt.passDepth > 0.0)) {
            say(ToolpathIssue::InvalidTool,
                label + ": skipped, needs a cutting width > 0 at the pocket depth, 0 < stepover <= that "
                        "width and pass depth > 0", ti);
            continue;
        }
        if (pt.stepover < 10.0 * userTol.distance) {
            say(ToolpathIssue::InvalidTool, label + ": skipped, stepover below 10 x the distance tolerance", ti);
            continue;
        }
        const double levelEstimate = std::ceil(params.cutDepth / pt.passDepth) + 2.0;
        if (!(levelEstimate <= kMaxLevels)) {
            say(ToolpathIssue::WorkLimitExceeded,
                label + ": skipped, more than " + std::to_string(static_cast<long long>(kMaxLevels)) +
                    " depth passes (pass depth too small)",
                ti);
            continue;
        }
        const double r = 0.5 * d;
        PocketToolPlan& plan = result.plans[ti];
        PassPlan passes;
        passes.startDepth = params.startDepth;
        passes.cutDepth = params.cutDepth;
        passes.passDepth = pt.passDepth;
        passes.maintainExactStep = params.maintainExactStep;
        passes.lastPassThickness = params.lastPassThickness;

        // Size of the pattern before any geometry work, from the pocket region (an upper bound
        // for the centre region): rings (inradius / stepover) or raster lines (extent /
        // stepover) per part, times the region's vertices, times the levels.
        double passEstimate = 0.0;
        for (const Box2& box : regionBoxes) {
            const double shrink = 2.0 * (r + params.allowance);
            const double w = std::max(0.0, box.width() - shrink);
            const double h = std::max(0.0, box.height() - shrink);
            const double extent =
                params.strategy == PocketStrategy::Offset ? 0.5 * std::min(w, h) : std::hypot(w, h);
            passEstimate = std::max(passEstimate, extent / pt.stepover + 2.0);
        }
        const double moveEstimate =
            levelEstimate * (params.strategy == PocketStrategy::Offset ? passEstimate * regionVertices
                                                                       : 2.0 * passEstimate + regionVertices);
        if (!(passEstimate <= kMaxPasses) || !(moveEstimate <= kMaxMoves)) {
            say(ToolpathIssue::WorkLimitExceeded,
                label + ": skipped, the pattern would be too large (stepover too small for the pocket size)", ti);
            continue;
        }
        InsetReport guardReport;
        const std::vector<Contour> centreRegion =
            centreRegionOf(region, r, params.allowance, margin, tol, guardReport);
        for (ToolpathDiagnostic& n : centreRegionNotes(guardReport, label + ": ", ti)) {
            result.diagnostics.push_back(std::move(n));
        }
        const std::vector<Group> centre = groupsOf(centreRegion, tol.distance);
        if (centre.empty()) {
            say(ToolpathIssue::ToolTooLarge, label + ": too large for the pocket, nothing to cut", ti);
            continue;
        }
        plan.levels = passes.levels();
        plan.centreRegion = centreRegion;

        // Each raster piece costs at least 6 moves per level (ADR-0017 §12): more pieces than the
        // move cap allows stop the pattern early instead of filling memory first.
        std::size_t piecesUsed = 0;
        bool overBudget = false;
        const auto pieceBudget = static_cast<std::size_t>(
            kMaxMoves / (6.0 * static_cast<double>(std::max<std::size_t>(plan.levels.size(), 1))));
        // The tool's straight moves inside its centre region (ADR-0017 §8a): within rounding of
        // the region, or clear of the polygonised walls by radius + allowance + their tolerance.
        const MoveCheck moves(&walls, r + params.allowance + wallSag, userTol.distance);
        const PatternContext ctx{params, pt.stepover, d, tol, fine, sliver, moves, pieceBudget, &piecesUsed, &overBudget};
        const bool isRest = ti > 0 && !swept.empty();
        // Raster without a boundary pass leaves scallops along the walls: its swept area is the
        // tool disc along the actual paths, not the whole reach, and the floor of a level is not
        // clear everywhere, so the tool retracts instead of travelling (ADR-0017 §8, §10).
        const bool scalloped =
            params.strategy == PocketStrategy::Raster && params.profilePass == ProfilePass::None;
        auto sweptByPaths = [&](const std::vector<PocketPath>& paths) {
            std::vector<Contour> open;
            for (const PocketPath& pp : paths) {
                Polyline pl = pp.path;
                if (pl.closed && !pl.points.empty()) {
                    pl.points.push_back(pl.points.front());
                    pl.closed = false;
                }
                open.push_back(Contour::fromPolyline(pl));
            }
            return offsetBy(open, r, fine);
        };
        std::vector<Contour> cutArea;
        // The connected parts the pattern is built over, one cluster each: the centre region's
        // parts for the clearance tool, the rest areas (need ∩ centre) for a rest tool.
        std::vector<Group> parts;
        if (!isRest) {
            parts = centre;
        } else {
            if (!target) {
                target = insetBy(region, params.allowance + margin, tol);
            }
            plan.restMaterial = opened(difference(*target, swept, tol), sliver, fine);
            // Thinned like the swept area (T-0175): the boolean and the opening leave many
            // micro-edges where the target and the swept boundaries meet; every part kept is at
            // least two slivers wide, so moving its boundary by a tenth of a sliver keeps it.
            for (Contour& c : plan.restMaterial) {
                c = contourOf(simplified(ringOf(c), 0.1 * sliver));
            }
            if (plan.restMaterial.empty()) {
                say(ToolpathIssue::NoRestMaterial, label + ": no rest material left, nothing to cut", ti);
                continue;
            }
            // Centres whose disc overlaps the rest material by more than 10 x the distance
            // tolerance: a tool that would only touch it (e.g. the same tool again) gets nothing.
            // The pattern's rings follow this boundary, so it is built at the region tolerance
            // (not the fine one), and the swept area's arcs (round joins at 1e-4 mm), which carry
            // over from the rest material, are thinned to the region tolerance too: the need region
            // moves by at most that, and is then clipped to the centre region.
            std::vector<Contour> needContours = offsetBy(plan.restMaterial, r - 10.0 * userTol.distance, tol);
            for (Contour& c : needContours) {
                c = contourOf(simplified(ringOf(c), tol.chordal));
            }
            // The rest areas: where the centre may go and its disc reaches rest material. The
            // pattern is built over them only (T-0123), not over the whole centre region and
            // then clipped: its rings or lines cover every point within a radius of the area,
            // hence all rest material the tool can reach, and every path lies inside the centre
            // region. A rest cluster travels only inside its own area: the floor there is clear
            // at the previous level, while other areas may still hold material at every depth.
            const std::vector<Contour> restAreas = intersection(needContours, plan.centreRegion, tol);
            parts = groupsOf(restAreas, tol.distance);
            if (parts.empty()) {
                say(ToolpathIssue::RestUnreachable, label + ": cannot reach the rest material, nothing to cut", ti);
                continue;
            }
            if (!scalloped) {
                cutArea = offsetBy(restAreas, r, fine);
            }
        }
        // Rest areas are cut part of the centre region by part, nearest first, which keeps the
        // rapids between them short (the tool retracts between rest areas: owner decision
        // 2026-11-01).
        if (isRest) {
            std::vector<std::size_t> partOf(parts.size(), kNoNode);
            std::vector<Box2> centreBoxes;
            for (const Group& g : centre) {
                centreBoxes.push_back(ringBox(g.outer).inflated(tol.distance));
            }
            const BoxIndex centreIndex(std::move(centreBoxes));
            for (std::size_t c = 0; c < parts.size(); ++c) {
                for (const std::size_t k : centreIndex.containing(parts[c].outer.front())) {
                    if (insideRegion(ringsOf(centre[k]), parts[c].outer.front(), tol.distance)) {
                        partOf[c] = k;
                        break;
                    }
                }
            }
            const std::vector<std::size_t> order = nearestFirst(parts, partOf);
            std::vector<Group> sortedParts;
            for (const std::size_t c : order) {
                sortedParts.push_back(std::move(parts[c]));
            }
            parts = std::move(sortedParts);
        }
        Pattern pattern = fullPattern(parts, ctx);
        if (overBudget) {
            say(ToolpathIssue::WorkLimitExceeded,
                label + ": skipped, the toolpath would exceed " +
                    std::to_string(static_cast<long long>(kMaxMoves)) +
                    " moves (stepover or pass depth too small for the pocket)",
                ti);
            plan.levels.clear();
            continue;
        }
        plan.paths = std::move(pattern.paths);
        const std::vector<std::size_t> nodes = std::move(pattern.node);
        const std::vector<RingTree> trees = std::move(pattern.trees);
        if (plan.paths.empty()) {
            say(ToolpathIssue::RestUnreachable, label + ": cannot reach the rest material, nothing to cut", ti);
            continue;
        }
        // Work cap on the actual pattern (the estimate above cannot see raster pieces per line).
        double patternMoves = 0.0;
        for (const PocketPath& pp : plan.paths) {
            patternMoves += static_cast<double>(pp.path.points.size()) + 4.0;
        }
        if (!(patternMoves * static_cast<double>(plan.levels.size()) <= kMaxMoves)) {
            say(ToolpathIssue::WorkLimitExceeded,
                label + ": skipped, the toolpath would exceed " +
                    std::to_string(static_cast<long long>(kMaxMoves)) +
                    " moves (stepover or pass depth too small for the pocket)",
                ti);
            plan.paths.clear();
            plan.levels.clear();
            continue;
        }
        if (ti + 1 < tools.size()) {  // only later tools need the swept area
            if (cutArea.empty()) {
                cutArea = scalloped ? sweptByPaths(plan.paths) : offsetBy(plan.centreRegion, r, fine);
            }
            // The swept area is bookkeeping for the rest tools, not a wall: its round joins (at
            // 1e-4 mm) are thinned to a tenth of the sliver width, so it moves by at most that (material
            // thinner than a sliver is noise, `opened` below). Since T-0175 the booleans keep
            // every vertex (no merge at the distance tolerance), and without this the rest
            // material carried them all through every later offset (up to 2× slower).
            for (Contour& c : cutArea) {
                c = contourOf(simplified(ringOf(c), 0.1 * sliver));
            }
            swept = swept.empty() ? cutArea : geometry::unionContours([&] {
                std::vector<Contour> all = swept;
                all.insert(all.end(), cutArea.begin(), cutArea.end());
                return all;
            }());
        }

        Toolpath& tp = result.toolpaths[ti];
        Emitter emitter(tp, params, 2.0 * d, !scalloped, moves);
        for (std::size_t start = 0; start < plan.paths.size();) {
            const std::size_t cl = plan.paths[start].cluster;
            std::vector<const PocketPath*> members;
            std::vector<std::size_t> memberNodes;
            std::size_t end = start;
            while (end < plan.paths.size() && plan.paths[end].cluster == cl) {
                members.push_back(&plan.paths[end]);
                memberNodes.push_back(nodes[end]);
                ++end;
            }
            // Travel within the cluster stays inside its own part (of the centre region, or rest
            // area); between clusters the tool retracts.
            const RingTree* tree = cl < trees.size() && !trees[cl].rings.empty() ? &trees[cl] : nullptr;
            emitter.cluster(members, memberNodes, plan.levels, RegionIndex(ringsOf(parts[cl])), tree);
            start = end;
        }
        emitter.finish();
    }
    return result;
}

std::vector<Contour> pocketCentreRegion(const std::vector<Contour>& contours, double toolDiameter,
                                        const PocketParams& params, std::vector<ToolpathDiagnostic>* diagnostics) {
    const Tolerance& userTol = params.tolerance;
    auto say = [&](ToolpathIssue issue, std::string m) {
        if (diagnostics != nullptr) {
            diagnostics->push_back(note(issue, std::move(m)));
        }
    };
    if (!finite(toolDiameter) || !(toolDiameter > 0.0) || !finite(params.allowance) ||
        params.allowance < 0.0 || !finite(userTol.chordal) || userTol.chordal < kMinChordal ||
        userTol.chordal > 1.0 || !finite(userTol.distance) || !(userTol.distance >= kMinDistance) ||
        userTol.distance > userTol.chordal) {
        say(ToolpathIssue::InvalidParameters, "invalid tool diameter, allowance or tolerance");
        return {};
    }
    const Tolerance tol = regionTolerance(userTol);
    std::vector<Contour> closedInput;
    std::size_t openCount = 0;
    double inputVertices = 0.0;
    for (const Contour& c : contours) {
        if (c.closed) {
            inputVertices += estimatedVertices(c, tol);
            closedInput.push_back(c);
        } else if (!c.isEmpty()) {
            ++openCount;
        }
    }
    if (openCount > 0) {
        say(ToolpathIssue::OpenContourIgnored,
            std::to_string(openCount) + " open contour(s) ignored: a pocket needs closed vectors");
    }
    if (!(inputVertices <= kMaxVertices)) {
        say(ToolpathIssue::WorkLimitExceeded, "pocket boundary too detailed at this chordal tolerance (more than " +
            std::to_string(static_cast<long long>(kMaxVertices)) + " vertices)");
        return {};
    }
    const std::vector<Contour> region = geometry::unionContours(closedInput, tol, FillRule::EvenOdd);
    InsetReport guardReport;
    std::vector<Contour> centre =
        centreRegionOf(region, 0.5 * toolDiameter, params.allowance, userTol.chordal, tol, guardReport);
    if (diagnostics != nullptr) {
        for (ToolpathDiagnostic& n : centreRegionNotes(guardReport, "", std::nullopt)) {
            diagnostics->push_back(std::move(n));
        }
    }
    return centre;
}

namespace detail {

double segmentDistance(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    using geometry::detail::orient2dSign;
    const int o1 = orient2dSign(a, b, c);
    const int o2 = orient2dSign(a, b, d);
    if (o1 * o2 < 0) {
        const int o3 = orient2dSign(c, d, a);
        const int o4 = orient2dSign(c, d, b);
        if (o3 * o4 < 0) {
            return 0.0;
        }
    }
    return std::min(std::min(distToSegment(a, c, d), distToSegment(b, c, d)),
                    std::min(distToSegment(c, a, b), distToSegment(d, a, b)));
}

std::vector<std::size_t> nearestFirstOrder(const std::vector<Vec2>& centres, const std::vector<std::size_t>& partOf) {
    const std::size_t n = centres.size();
    Box2 all;
    for (std::size_t i = 0; i < n; ++i) {
        all.expand(centres[i]);
    }
    const double extent = std::max({all.width(), all.height(), 1e-9});
    std::vector<char> used(n, 0);
    std::vector<std::size_t> out;
    // Members of each group, ascending (T-0131: one index per group, so a search for the group's
    // next part never returns the other groups' parts, and a group is left as soon as all its
    // parts are used, instead of growing the square over the whole page to find nothing). The
    // squares, the candidates and their order are those of one index over all parts filtered by
    // group, so the order is the same.
    std::vector<std::pair<std::size_t, std::size_t>> byGroup(n);  // (group, part)
    for (std::size_t i = 0; i < n; ++i) {
        byGroup[i] = {partOf[i], i};
    }
    std::sort(byGroup.begin(), byGroup.end());
    for (std::size_t g0 = 0; g0 < n;) {
        const std::size_t gid = byGroup[g0].first;
        std::size_t g1 = g0;
        std::vector<std::size_t> members;
        std::vector<Box2> points;
        for (; g1 < n && byGroup[g1].first == gid; ++g1) {
            members.push_back(byGroup[g1].second);
            points.push_back(Box2{centres[members.back()], centres[members.back()]});
        }
        g0 = g1;
        const BoxIndex index(std::move(points));
        std::optional<std::size_t> cur;
        for (const std::size_t i : members) {
            if (!cur || centres[i].y < centres[*cur].y ||
                (centres[i].y == centres[*cur].y && centres[i].x < centres[*cur].x)) {
                cur = i;
            }
        }
        std::size_t left = members.size();
        while (cur) {
            used[*cur] = 1;
            --left;
            out.push_back(*cur);
            const Vec2 at = centres[*cur];
            cur.reset();
            if (left == 0) {
                break;
            }
            double best = std::numeric_limits<double>::infinity();
            for (double h = extent / std::sqrt(static_cast<double>(n)) + 1e-9;; h *= 2.0) {
                for (const std::size_t k : index.query(Box2{at - Vec2{h, h}, at + Vec2{h, h}})) {
                    const std::size_t j = members[k];
                    const double dd = geometry::distance(at, centres[j]);
                    if (used[j] == 0 && dd < best) {
                        best = dd;
                        cur = j;
                    }
                }
                // Anything outside the square is farther than h; done once the square covers all.
                if (best <= h || h > 2.0 * extent) {
                    break;
                }
            }
        }
    }
    return out;
}

std::vector<Contour> strictInset(const std::vector<Contour>& source, const std::vector<Contour>& raw,
                                 double d, const Tolerance& tol, InsetReport* report) {
    return guardInset(source, raw, d, tol, InsetGuard::Strict, report);
}

}  // namespace detail

}  // namespace carvetto::toolpath
