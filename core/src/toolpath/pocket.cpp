#include "carvetto/toolpath/pocket.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
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
#include "carvetto/toolpath/pass_plan.hpp"
#include "carvetto/toolpath/pocket_ramp.hpp"
#include "pocket_guard.hpp"

// Pocket toolpath (T-0121, ADR-0017).
//
// All region work is done on polygons through the public geometry API: the even-odd union of
// the input gives the pocket region R; offsetContours(R, −(radius + allowance + margin)) gives
// the region C the tool centre may visit. The Offset strategy builds the level sets
// offset(C, −k·stepover) as a containment tree and cuts it in post-order (inside out); when the
// stepover exceeds the radius, extra rings offset(C, −(k·stepover + radius)) are inserted
// wherever the next level would leave material uncovered. The Raster strategy clips lines to C
// with an open-path intersection. Rest tools clip their own pattern to the neighbourhood of the
// rest material. Small polygon helpers (point in ring, segment inside region) live here and are
// deliberately simple: they work on the line-only polygons Clipper2 returns.

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
    double a = 0.0;
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        a += r[i].cross(r[(i + 1) % n]);
    }
    return 0.5 * a;
}

double distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = ab.lengthSquared();
    const double t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
    return geometry::distance(p, a + ab * t);
}

/// Exact distance between segments ab and cd (0 when they cross or touch).
double distSegments(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    const Vec2 ab = b - a;
    const Vec2 cd = d - c;
    const double o1 = ab.cross(c - a);
    const double o2 = ab.cross(d - a);
    const double o3 = cd.cross(a - c);
    const double o4 = cd.cross(b - c);
    if (((o1 <= 0.0 && o2 >= 0.0) || (o1 >= 0.0 && o2 <= 0.0)) &&
        ((o3 <= 0.0 && o4 >= 0.0) || (o3 >= 0.0 && o4 <= 0.0)) && ab.lengthSquared() > 0.0 &&
        cd.lengthSquared() > 0.0 && !(o1 == 0.0 && o2 == 0.0)) {
        return 0.0;
    }
    return std::min(std::min(distToSegment(a, c, d), distToSegment(b, c, d)),
                    std::min(distToSegment(c, a, b), distToSegment(d, a, b)));
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

/// True when the straight segment a→b stays inside the region of `rings` (boundary included,
/// within `tol`): no edge is crossed away from the end points and the midpoint is inside. An
/// edge the whole segment lies along (both ends within `tol` of the edge segment, hence every
/// point of a→b too) is not a crossing, so links that follow the boundary are accepted even when
/// their ends sit a rounding error off it; the edge's infinite line is not enough (a nearly
/// parallel edge far away would hide a real exit). Conservative: a boundary vertex within `tol`
/// of the segment's interior rejects it (rounding could hide a crossing through that vertex).
bool segmentInside(Vec2 a, Vec2 b, const std::vector<Ring>& rings, double tol) {
    const Vec2 d = b - a;
    auto nearEnd = [&](Vec2 x) { return geometry::distance(x, a) <= tol || geometry::distance(x, b) <= tol; };
    for (const Ring& r : rings) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2 c = r[i];
            const Vec2 c2 = r[(i + 1) % n];
            const Vec2 e = c2 - c;
            if (e.lengthSquared() > 0.0 && distToSegment(a, c, c2) <= tol && distToSegment(b, c, c2) <= tol) {
                continue;  // within tol of this edge all along: following the boundary
            }
            // A boundary vertex within tol of the segment's interior: a touch (or a crossing
            // through the vertex that rounding could hide).
            if (distToSegment(c, a, b) <= tol && !nearEnd(c)) {
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
    return insideRegion(rings, (a + b) * 0.5, tol);
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

std::vector<Ring> ringsOf(const std::vector<Group>& gs) {
    std::vector<Ring> out;
    for (const Group& g : gs) {
        for (Ring& r : ringsOf(g)) {
            out.push_back(std::move(r));
        }
    }
    return out;
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
    for (Ring& h : holes) {
        Vec2 probe = h.front();
        std::optional<std::size_t> best;
        for (std::size_t gi = 0; gi < groups.size(); ++gi) {
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
                    best = std::min(best, distToSegment(p, a, b));
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
                    best = std::min(best, distSegments(a, b, p, q));
                }
            }
        }
        return best;
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
                    if (distToSegment(p, a, b) < d) {
                        return true;
                    }
                }
            }
        }
        return false;
    }

   private:
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
        const double perAxis = std::clamp(std::sqrt(static_cast<double>(edges_.size())), 1.0, 512.0);
        cell_ = std::max({box_.width() / perAxis, box_.height() / perAxis, 1e-9});
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

    /// As the free `segmentInside`.
    bool segmentInside(Vec2 a, Vec2 b, double tol) const {
        const Vec2 d = b - a;
        auto nearEnd = [&](Vec2 x) {
            return geometry::distance(x, a) <= tol || geometry::distance(x, b) <= tol;
        };
        bool ok = true;
        forEdges(std::min(a.x, b.x) - tol, std::min(a.y, b.y) - tol, std::max(a.x, b.x) + tol,
                 std::max(a.y, b.y) + tol, [&](Vec2 c, Vec2 c2) {
                     if (!ok) {
                         return;
                     }
                     const Vec2 e = c2 - c;
                     if (e.lengthSquared() > 0.0 && distToSegment(a, c, c2) <= tol &&
                         distToSegment(b, c, c2) <= tol) {
                         return;  // following the boundary
                     }
                     if (distToSegment(c, a, b) <= tol && !nearEnd(c)) {
                         ok = false;
                         return;
                     }
                     const double o1 = d.cross(c - a);
                     const double o2 = d.cross(c2 - a);
                     const double o3 = e.cross(a - c);
                     const double o4 = e.cross(b - c);
                     if (((o1 < 0.0 && o2 > 0.0) || (o1 > 0.0 && o2 < 0.0)) &&
                         ((o3 < 0.0 && o4 > 0.0) || (o3 > 0.0 && o4 < 0.0)) &&
                         !nearEnd(a + d * (o3 / (o3 - o4)))) {
                         ok = false;
                     }
                 });
        return ok && inside((a + b) * 0.5, tol);
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

/// How `insetBy` tells a genuine inset region from an inside-out phantom, and what it does with
/// a genuine region that overshoots.
///
/// Phantom test (both guards, T-0126): a result region is genuine when some point of it (an outer
/// vertex, or the centroid of its outer ring when that lies in the region) keeps the full
/// distance from the source and lies inside it. Such a point belongs to the true inset, so the
/// region does too. A phantom (Clipper2's inside-out result past the inradius) has no such
/// point: every point of it is closer to the walls than the inset. The test is by depth, not by
/// vertex count, so a large region whose small bulge holds most of its vertices is kept.
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
/// regions, as `guard` describes. Clipper2 1.4.0 turns a polygon inset beyond its inradius
/// inside out (a circle inset by more than its radius comes back as a growing circle), so a
/// valid-looking region can appear where nothing fits; every point of a true inset is at
/// distance >= d from the input, which rejects those. Distances use an edge grid; the inside
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
        if (good && !insideRegion(base, *good, 0.0)) {
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
            if (passes(c) && insideRegion(ringsOf(g), c, 0.0) && insideRegion(base, c, 0.0)) {
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

struct RingTree {
    std::vector<std::vector<Ring>> rings;
    std::vector<std::size_t> parent;
    /// Per node, built on first use by `route`: bridges joining all of the node's rings.
    mutable std::vector<std::optional<std::vector<RingBridge>>> bridges;
};

/// A tool's planned paths plus what the emitter needs to route travel moves between them.
struct Pattern {
    std::vector<PocketPath> paths;
    /// Per path: its RingTree node, or kNoNode (raster pieces, rest pieces).
    std::vector<std::size_t> node;
    /// Per cluster; empty for raster and rest clusters.
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
    std::vector<Contour> lines;
    for (std::size_t k = 0; k < n; ++k) {
        const double vk = vmin + (static_cast<double>(k) + 0.5) * pitch;
        Contour c;
        c.append(geometry::LineSpan{u * (umin - 1.0) + v * vk, u * (umax + 1.0) + v * vk});
        lines.push_back(std::move(c));
    }
    const geometry::BooleanResult clipped =
        geometry::booleanOp(BooleanOp::Intersection, lines, contoursOf(g), tol, FillRule::EvenOdd);
    std::vector<std::vector<RasterPiece>> rows(n);
    for (const Contour& c : clipped.open) {
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
    }
    for (auto& row : rows) {
        std::sort(row.begin(), row.end(),
                  [&](const RasterPiece& x, const RasterPiece& y) { return x.a.dot(u) < y.a.dot(u); });
    }

    // Zig-zag: continue on the next row with the piece nearest to the current end when the link
    // stays inside the region and is short; otherwise restart at the first unused piece.
    const RegionIndex region(ringsOf(g));
    const double linkLimit = 2.0 * ctx.diameter;
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
                for (std::size_t i = 0; i < rows[k].size(); ++i) {
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
                if (bestD > linkLimit || !region.segmentInside(*cursor, s, tol.distance)) {
                    break;
                }
                emit(k, *best, bestFromA);
            }
        }
    }
    if (p.profilePass == ProfilePass::Last) {
        appendGroupRings(g, p.direction, PocketPathRole::Profile, cluster, cursor, tol.distance, out);
    }
    pattern.add(out, kNoNode);
    if (pattern.trees.size() <= cluster) {
        pattern.trees.resize(cluster + 1);
    }
}

/// The full pattern of one tool over every connected part of its centre region.
Pattern fullPattern(const std::vector<Group>& centre, const PatternContext& ctx) {
    Pattern out;
    std::optional<Vec2> cursor;
    for (std::size_t ci = 0; ci < centre.size(); ++ci) {
        if (ctx.params.strategy == PocketStrategy::Offset) {
            offsetPattern(centre[ci], ctx, ci, cursor, out);
        } else {
            rasterPattern(centre[ci], ctx, ci, cursor, out);
        }
    }
    return out;
}

// ------------------------------------------------------------------------- rest machining

/// Arc-length parameter of the point of `path` nearest to `q`; ties go to the earliest (or, with
/// `late`, the latest) parameter.
double paramOn(const Polyline& path, Vec2 q, bool late) {
    double best = std::numeric_limits<double>::infinity();
    double bestParam = 0.0;
    double acc = 0.0;
    for (std::size_t i = 0, n = path.segmentCount(); i < n; ++i) {
        const Vec2 a = path.segmentStart(i);
        const Vec2 ab = path.segmentEnd(i) - a;
        const double len = ab.length();
        const double t = len > 0.0 ? std::clamp((q - a).dot(ab) / (len * len), 0.0, 1.0) : 0.0;
        const double d = geometry::distance(q, a + ab * t);
        if (d < best - 1e-9 || (late && d <= best + 1e-9)) {
            best = std::min(best, d);
            bestParam = acc + t * len;
        }
        acc += len;
    }
    return bestParam;
}

/// Direction of `path` at the segment nearest to `q`.
Vec2 directionNear(const Polyline& path, Vec2 q) {
    double best = std::numeric_limits<double>::infinity();
    Vec2 dir{1.0, 0.0};
    for (std::size_t i = 0, n = path.segmentCount(); i < n; ++i) {
        const double d = distToSegment(q, path.segmentStart(i), path.segmentEnd(i));
        if (d < best) {
            best = d;
            dir = path.segmentEnd(i) - path.segmentStart(i);
        }
    }
    return dir;
}

/// Clips every path of `pattern` to `need` and returns the pieces in pattern order, each running
/// in its path's direction; clusters are the connected parts of `need`.
std::vector<PocketPath> clipPattern(const std::vector<PocketPath>& pattern, const std::vector<Group>& need,
                                    const Tolerance& tol, std::vector<std::size_t>& clusterGroup) {
    // Each path is clipped only against the parts of `need` whose bounding box it overlaps.
    std::vector<std::vector<Contour>> needParts;
    std::vector<Box2> needBoxes;
    for (const Group& g : need) {
        needParts.push_back(contoursOf(g));
        Box2 b;
        for (const Vec2 q : g.outer) {
            b.expand(q);
        }
        needBoxes.push_back(b);
    }
    std::vector<PocketPath> pieces;
    for (std::size_t pi = 0; pi < pattern.size(); ++pi) {
        const Polyline& src = pattern[pi].path;
        const Box2 pathBox = src.bounds();
        std::vector<Contour> clip;
        for (std::size_t g = 0; g < need.size(); ++g) {
            const Box2& b = needBoxes[g];
            if (b.min.x <= pathBox.max.x + tol.distance && pathBox.min.x <= b.max.x + tol.distance &&
                b.min.y <= pathBox.max.y + tol.distance && pathBox.min.y <= b.max.y + tol.distance) {
                clip.insert(clip.end(), needParts[g].begin(), needParts[g].end());
            }
        }
        if (clip.empty()) {
            continue;
        }
        Polyline open = src;
        if (src.closed && !src.points.empty()) {
            open.points.push_back(src.points.front());
            open.closed = false;
        }
        const double total = open.length();
        Contour subject = Contour::fromPolyline(open);
        const geometry::BooleanResult r =
            geometry::booleanOp(BooleanOp::Intersection, {subject}, clip, tol, FillRule::EvenOdd);
        std::vector<std::pair<double, Polyline>> local;  // (start param, piece)
        for (const Contour& c : r.open) {
            Polyline pl = c.polygonise(tol);
            if (pl.points.size() < 2 || pl.length() <= tol.distance) {
                continue;
            }
            const Vec2 mid = (pl.points[0] + pl.points[1]) * 0.5;
            if (directionNear(open, mid).dot(pl.points[1] - pl.points[0]) < 0.0) {
                pl.reverse();
            }
            local.emplace_back(paramOn(open, pl.points.front(), false), std::move(pl));
        }
        std::sort(local.begin(), local.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
        // A closed ring cut across its seam comes back as a tail and a head piece: rejoin them.
        if (src.closed && local.size() >= 2) {
            Polyline& head = local.front().second;
            Polyline& tail = local.back().second;
            if (local.front().first <= tol.distance &&
                paramOn(open, tail.points.back(), true) >= total - tol.distance &&
                geometry::distance(tail.points.back(), head.points.front()) <= tol.distance) {
                tail.points.insert(tail.points.end(), head.points.begin() + 1, head.points.end());
                local.erase(local.begin());
            }
        }
        for (auto& entry : local) {
            pieces.push_back(PocketPath{std::move(entry.second), pattern[pi].role, 0});
        }
    }
    // Clusters: the part of `need` holding the piece, in order of first appearance.
    std::vector<std::size_t> clusterOrder;
    std::vector<std::vector<std::size_t>> members;
    for (std::size_t i = 0; i < pieces.size(); ++i) {
        const auto& pts = pieces[i].path.points;
        const Vec2 probe = (pts[0] + pts[1]) * 0.5;
        std::size_t gi = 0;
        for (std::size_t g = 0; g < need.size(); ++g) {
            if (insideRegion(ringsOf(need[g]), probe, tol.distance)) {
                gi = g;
                break;
            }
        }
        auto it = std::find(clusterOrder.begin(), clusterOrder.end(), gi);
        if (it == clusterOrder.end()) {
            clusterOrder.push_back(gi);
            members.emplace_back();
            it = clusterOrder.end() - 1;
        }
        members[static_cast<std::size_t>(it - clusterOrder.begin())].push_back(i);
    }
    clusterGroup = clusterOrder;
    std::vector<PocketPath> out;
    for (std::size_t c = 0; c < members.size(); ++c) {
        for (std::size_t i : members[c]) {
            PocketPath pp = std::move(pieces[i]);
            pp.cluster = c;
            out.push_back(std::move(pp));
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

RingPoint nearestOn(const std::vector<Ring>& rings, Vec2 p) {
    RingPoint best;
    double bestD = std::numeric_limits<double>::infinity();
    for (std::size_t ri = 0; ri < rings.size(); ++ri) {
        const Ring& r = rings[ri];
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2 a = r[i];
            const Vec2 ab = r[(i + 1) % n] - a;
            const double l2 = ab.lengthSquared();
            const double t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
            const Vec2 q = a + ab * t;
            const double d = geometry::distance(p, q);
            if (d < bestD) {
                bestD = d;
                best = RingPoint{ri, i, t, q};
            }
        }
    }
    return best;
}

double polyLength(const std::vector<Vec2>& pts) {
    double len = 0.0;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        len += geometry::distance(pts[i - 1], pts[i]);
    }
    return len;
}

/// The shorter way along ring `r` from `a` to `b` (both on `r`), end points included.
std::vector<Vec2> walkRing(const Ring& r, const RingPoint& a, const RingPoint& b) {
    const std::size_t n = r.size();
    std::vector<Vec2> fwd{a.point};
    if (!(a.edge == b.edge && a.t <= b.t)) {
        for (std::size_t i = (a.edge + 1) % n;; i = (i + 1) % n) {
            fwd.push_back(r[i]);
            if (i == b.edge) {
                break;
            }
        }
    }
    fwd.push_back(b.point);
    std::vector<Vec2> bwd{a.point};
    if (!(a.edge == b.edge && a.t >= b.t)) {
        for (std::size_t i = a.edge;; i = (i + n - 1) % n) {
            bwd.push_back(r[i]);
            if (i == (b.edge + 1) % n) {
                break;
            }
        }
    }
    bwd.push_back(b.point);
    return polyLength(fwd) <= polyLength(bwd) ? fwd : bwd;
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
std::vector<RingBridge> ringBridges(const std::vector<Ring>& rings, double tol) {
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
        if (ri == rj || !segmentInside(c.a, c.b, rings, tol)) {
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
                              const RingPoint& a, const RingPoint& b) {
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
        RingPoint leave = nearestOn({rings[from]}, pf);
        leave.ring = from;
        RingPoint arrive = nearestOn({rings[r]}, pt);
        arrive.ring = r;
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
/// to it stays inside `region`).
std::vector<Vec2> route(const RingTree& tree, std::size_t na, Vec2 P, std::size_t nb, Vec2 Q,
                        const RegionIndex& region, double tol) {
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
    auto climb = [&](const std::vector<std::size_t>& anc, Vec2 from) {
        std::vector<Vec2> pts{from};
        for (std::size_t i = 1; i < anc.size() && anc[i - 1] != lca; ++i) {
            pts.push_back(nearestOn(tree.rings[anc[i]], pts.back()).point);
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
            tree.bridges[node] = ringBridges(rings, tol);
        }
        const RingPoint a = nearestOn(rings, up.back());
        const RingPoint b = nearestOn(rings, down.back());
        mid = acrossRings(rings, *tree.bridges[node], a, b);
        if (!mid.empty()) {
            break;
        }
        if (tree.parent[node] == kNoNode) {
            return {};
        }
        node = tree.parent[node];
        up.push_back(nearestOn(tree.rings[node], up.back()).point);
        down.push_back(nearestOn(tree.rings[node], down.back()).point);
    }
    std::vector<Vec2> way = up;
    way.insert(way.end(), mid.begin(), mid.end());
    way.insert(way.end(), down.rbegin(), down.rend());
    // Drop repeated points, then pull the string.
    std::vector<Vec2> clean;
    for (const Vec2 q : way) {
        if (clean.empty() || geometry::distance(clean.back(), q) > tol) {
            clean.push_back(q);
        }
    }
    if (clean.size() < 2) {
        return clean.empty() ? std::vector<Vec2>{P, Q} : std::vector<Vec2>{clean.front(), Q};
    }
    std::vector<Vec2> pulled{clean.front()};
    for (std::size_t i = 0; i + 1 < clean.size();) {
        std::size_t j = i + 1;
        while (j + 1 < clean.size() && region.segmentInside(clean[i], clean[j + 1], tol)) {
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
    Emitter(Toolpath& tp, const PocketParams& p, double linkLimit, bool allowTravel)
        : tp_(tp),
          p_(p),
          linkLimit_(linkLimit),
          minRamp_(10.0 * p.tolerance.distance),
          allowTravel_(allowTravel) {}

    /// Cuts the paths of one cluster at every level. Short links inside the centre region feed at
    /// depth; longer ways (and the way back to the first path of the next level) travel inside the
    /// centre region at the floor of the previous level (the surface for the first), which is
    /// already cleared there, along a straight segment or a route through `tree`; only when
    /// neither exists does the tool retract to safe Z.
    void cluster(const std::vector<const PocketPath*>& paths, const std::vector<std::size_t>& nodes,
                 const std::vector<double>& levels, const RegionIndex& region, const RingTree* tree) {
        const double tol = p_.tolerance.distance;
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
                           region.segmentInside(pos_, s, tol)) {
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
        const double tol = p_.tolerance.distance;
        std::vector<Vec2> way;
        if (region.segmentInside(pos_, s, tol)) {
            way = {pos_, s};
        } else if (tree != nullptr && !tree->rings.empty() && node != kNoNode && node_ != kNoNode) {
            way = route(*tree, node_, pos_, node, s, region, tol);
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
            appendPathRamp(path, zFrom, zTo, p_.rampDistance, minRamp_, tp_);
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

/// The messages for what the strict guard could not keep exactly (ADR-0017 §4).
std::vector<std::string> centreRegionMessages(const InsetReport& report) {
    std::vector<std::string> out;
    if (report.dropped > 0) {
        out.push_back(std::to_string(report.dropped) +
                      " part(s) of the pocket could not be kept within the safety margin and are not cut");
    }
    if (report.trimmed > 0) {
        out.push_back(std::to_string(report.trimmed) +
                      " part(s) of the pocket were trimmed by more than the tolerance to keep the safety "
                      "margin; material near them may be left");
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
constexpr double kMaxLevels = 10000.0;     // depth passes per tool
constexpr double kMaxPasses = 20000.0;     // rings or raster lines per cluster
constexpr double kMaxVertices = 200000.0;  // vertices of the polygonised pocket region
constexpr double kMaxMoves = 20000000.0;   // estimated moves per toolpath

}  // namespace

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
    for (std::size_t i = 0; i < tools.size(); ++i) {
        result.toolpaths[i].name =
            tools.size() > 1 ? params.name + " [" + tools[i].tool.name + "]" : params.name;
        result.toolpaths[i].tool = tools[i].tool;
    }
    if (tools.empty()) {
        result.messages.push_back("no tool selected");
        return result;
    }
    if (!finite(params.cutDepth) || !(params.cutDepth > 0.0) || !finite(params.startDepth) ||
        params.startDepth < 0.0 || !finite(params.allowance) || params.allowance < 0.0 ||
        !finite(params.safeZ) || !finite(params.rasterAngle) || !finite(params.rampDistance) ||
        !finite(params.lastPassThickness)) {
        result.messages.push_back(
            "invalid pocket parameters: cut depth must be > 0, start depth and allowance >= 0, all values "
            "finite");
        return result;
    }
    if (!finite(userTol.chordal) || userTol.chordal < kMinChordal || userTol.chordal > 1.0 ||
        !finite(userTol.distance) || !(userTol.distance > 0.0) || userTol.distance > userTol.chordal) {
        result.messages.push_back(
            "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal");
        return result;
    }
    if (params.safeZ <= -params.startDepth) {
        result.messages.push_back("safe Z must be above the start depth");
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
        result.messages.push_back(std::to_string(openCount) +
                                  " open contour(s) ignored: a pocket needs closed vectors");
    }
    if (!(inputVertices <= kMaxVertices)) {
        result.messages.push_back("pocket boundary too detailed at this chordal tolerance (more than " +
                                  std::to_string(static_cast<long long>(kMaxVertices)) + " vertices)");
        return result;
    }
    const std::vector<Contour> region = geometry::unionContours(closedInput, tol, FillRule::EvenOdd);
    if (region.empty()) {
        result.messages.push_back("no closed region to pocket");
        return result;
    }
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
        const PocketTool& pt = tools[ti];
        const double d = pt.tool.diameter;
        const std::string label = "tool " + std::to_string(ti + 1) + " (" + pt.tool.name + ")";
        if (!finite(d) || !(d > 0.0) || !finite(pt.stepover) || !(pt.stepover > 0.0) ||
            pt.stepover > d * (1.0 + 1e-12) || !finite(pt.passDepth) || !(pt.passDepth > 0.0)) {
            result.messages.push_back(
                label + ": skipped, needs diameter > 0, 0 < stepover <= diameter and pass depth > 0");
            continue;
        }
        if (pt.stepover < 10.0 * userTol.distance) {
            result.messages.push_back(label + ": skipped, stepover below 10 x the distance tolerance");
            continue;
        }
        const double levelEstimate = std::ceil(params.cutDepth / pt.passDepth) + 2.0;
        if (!(levelEstimate <= kMaxLevels)) {
            result.messages.push_back(label + ": skipped, more than " +
                                      std::to_string(static_cast<long long>(kMaxLevels)) +
                                      " depth passes (pass depth too small)");
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
            result.messages.push_back(label +
                                      ": skipped, the pattern would be too large (stepover too small " +
                                      "for the pocket size)");
            continue;
        }
        InsetReport guardReport;
        const std::vector<Contour> centreRegion =
            centreRegionOf(region, r, params.allowance, margin, tol, guardReport);
        for (const std::string& m : centreRegionMessages(guardReport)) {
            result.messages.push_back(label + ": " + m);
        }
        const std::vector<Group> centre = groupsOf(centreRegion, tol.distance);
        if (centre.empty()) {
            result.messages.push_back(label + ": too large for the pocket, nothing to cut");
            continue;
        }
        plan.levels = passes.levels();
        plan.centreRegion = centreRegion;

        const PatternContext ctx{params, pt.stepover, d, tol, fine, sliver};
        Pattern pattern = fullPattern(centre, ctx);
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
        std::vector<std::size_t> nodes;
        std::vector<RingTree> trees;
        std::vector<std::vector<Ring>> restRegions;  // per rest cluster: travel region
        if (!isRest) {
            plan.paths = std::move(pattern.paths);
            nodes = std::move(pattern.node);
            trees = std::move(pattern.trees);
        } else {
            if (!target) {
                target = insetBy(region, params.allowance + margin, tol);
            }
            plan.restMaterial = opened(difference(*target, swept, tol), sliver, fine);
            if (plan.restMaterial.empty()) {
                result.messages.push_back(label + ": no rest material left, nothing to cut");
                continue;
            }
            // Centres whose disc overlaps the rest material by more than 10 x the distance
            // tolerance: a tool that would only touch it (e.g. the same tool again) gets nothing.
            const std::vector<Contour> needContours =
                offsetBy(plan.restMaterial, r - 10.0 * userTol.distance, fine);
            // The pattern already lies inside the centre region; clipping it against
            // need ∩ centre would put the outermost ring exactly on the clip boundary, where
            // open-path clipping is ambiguous.
            const std::vector<Group> need = groupsOf(needContours, tol.distance);
            std::vector<std::size_t> clusterGroup;
            plan.paths = clipPattern(pattern.paths, need, tol, clusterGroup);
            if (plan.paths.empty()) {
                result.messages.push_back(label + ": cannot reach the rest material, nothing to cut");
                continue;
            }
            nodes.assign(plan.paths.size(), kNoNode);
            // A rest cluster travels only inside its own part of the need region (within the
            // centre region): the floor there is clear at the previous level, while other rest
            // clusters may still hold material at every depth.
            for (const std::size_t g : clusterGroup) {
                restRegions.push_back(ringsOf(
                    groupsOf(intersection(contoursOf(need[g]), plan.centreRegion, tol), tol.distance)));
            }
            if (!scalloped) {
                cutArea = offsetBy(intersection(needContours, plan.centreRegion, tol), r, fine);
            }
        }
        // Work cap on the actual pattern (the estimate above cannot see raster pieces per line).
        double patternMoves = 0.0;
        for (const PocketPath& pp : plan.paths) {
            patternMoves += static_cast<double>(pp.path.points.size()) + 4.0;
        }
        if (!(patternMoves * static_cast<double>(plan.levels.size()) <= kMaxMoves)) {
            result.messages.push_back(label + ": skipped, the toolpath would exceed " +
                                      std::to_string(static_cast<long long>(kMaxMoves)) +
                                      " moves (stepover or pass depth too small for the pocket)");
            plan.paths.clear();
            plan.levels.clear();
            continue;
        }
        if (cutArea.empty()) {
            cutArea = scalloped ? sweptByPaths(plan.paths) : offsetBy(plan.centreRegion, r, fine);
        }
        swept = swept.empty() ? cutArea : geometry::unionContours([&] {
            std::vector<Contour> all = swept;
            all.insert(all.end(), cutArea.begin(), cutArea.end());
            return all;
        }());

        Toolpath& tp = result.toolpaths[ti];
        Emitter emitter(tp, params, 2.0 * d, !scalloped);
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
            // Travel stays inside this cluster's own part of the centre region.
            const bool own = !isRest && cl < centre.size();
            const std::vector<Ring> clusterRegion =
                own ? ringsOf(centre[cl])
                    : (isRest && cl < restRegions.size() ? restRegions[cl] : ringsOf(centre));
            const RingTree* tree =
                own && cl < trees.size() && !trees[cl].rings.empty() ? &trees[cl] : nullptr;
            emitter.cluster(members, memberNodes, plan.levels, RegionIndex(clusterRegion), tree);
            start = end;
        }
        emitter.finish();
    }
    return result;
}

std::vector<Contour> pocketCentreRegion(const std::vector<Contour>& contours, double toolDiameter,
                                        const PocketParams& params, std::vector<std::string>* messages) {
    const Tolerance& userTol = params.tolerance;
    auto say = [&](const std::string& m) {
        if (messages != nullptr) {
            messages->push_back(m);
        }
    };
    if (!finite(toolDiameter) || !(toolDiameter > 0.0) || !finite(params.allowance) ||
        params.allowance < 0.0 || !finite(userTol.chordal) || userTol.chordal < kMinChordal ||
        userTol.chordal > 1.0 || !finite(userTol.distance) || !(userTol.distance > 0.0) ||
        userTol.distance > userTol.chordal) {
        say("invalid tool diameter, allowance or tolerance");
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
        say(std::to_string(openCount) + " open contour(s) ignored: a pocket needs closed vectors");
    }
    if (!(inputVertices <= kMaxVertices)) {
        say("pocket boundary too detailed at this chordal tolerance (more than " +
            std::to_string(static_cast<long long>(kMaxVertices)) + " vertices)");
        return {};
    }
    const std::vector<Contour> region = geometry::unionContours(closedInput, tol, FillRule::EvenOdd);
    InsetReport guardReport;
    std::vector<Contour> centre =
        centreRegionOf(region, 0.5 * toolDiameter, params.allowance, userTol.chordal, tol, guardReport);
    for (const std::string& m : centreRegionMessages(guardReport)) {
        say(m);
    }
    return centre;
}

namespace detail {

std::vector<Contour> strictInset(const std::vector<Contour>& source, const std::vector<Contour>& raw,
                                 double d, const Tolerance& tol, InsetReport* report) {
    return guardInset(source, raw, d, tol, InsetGuard::Strict, report);
}

}  // namespace detail

}  // namespace carvetto::toolpath
