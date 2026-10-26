#include "carvetto/toolpath/pocket.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
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

/// Uniform grid over the edges of a ring set, for "is any edge closer than d to p" queries.
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

/// Insets the region of `cs` by d > 0 (offsetContours with −d) and keeps only result regions
/// whose outer vertices all lie at least d (less the join tolerance) from the input's boundary
/// and inside it. Clipper2 1.4.0 turns a polygon inset beyond its inradius inside out (a circle
/// inset by more than its radius comes back as a growing circle), so a valid-looking region can
/// appear where nothing fits; every vertex of a true inset is at distance >= d from the input,
/// which rejects those. Distances use an edge grid; a region whose vertices all keep that
/// distance cannot cross the input boundary, so one inside test per region suffices.
std::vector<Contour> insetBy(const std::vector<Contour>& cs, double d, const Tolerance& tol) {
    const std::vector<Contour> raw = offsetBy(cs, -d, tol);
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
    std::vector<Group> kept;
    for (Group& g : groupsOf(raw, tol.distance)) {
        bool ok = !g.outer.empty() && insideRegion(base, g.outer.front(), 0.0);
        for (std::size_t i = 0; i < g.outer.size() && ok; ++i) {
            ok = !grid.anyCloser(g.outer[i], need);
        }
        if (ok) {
            kept.push_back(std::move(g));
        }
    }
    return contoursOf(kept);
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
struct RingTree {
    std::vector<std::vector<Ring>> rings;
    std::vector<std::size_t> parent;
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
    const std::vector<Ring> region = ringsOf(g);
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
                if (bestD > linkLimit || !segmentInside(*cursor, s, region, tol.distance)) {
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
                                    const Tolerance& tol) {
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

/// A way from P (on node `na`'s rings) to Q (on node `nb`'s rings) that stays inside the
/// cluster's centre region, or empty. Climbing to a parent goes straight to the parent
/// boundary's nearest point: the open disc around a point of a region up to its nearest boundary
/// point lies inside the region, so that segment is inside. The two climbs meet at the lowest
/// common ancestor, where the way follows the shared ring, or crosses between two of its rings
/// at their closest pair when that segment is inside; otherwise it climbs further. The result
/// is shortened greedily (a later point replaces the ones in between when the straight segment
/// to it stays inside `region`).
std::vector<Vec2> route(const RingTree& tree, std::size_t na, Vec2 P, std::size_t nb, Vec2 Q,
                        const std::vector<Ring>& region, double tol) {
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
    for (std::size_t node = lca;;) {
        const std::vector<Ring>& rings = tree.rings[node];
        const RingPoint a = nearestOn(rings, up.back());
        const RingPoint b = nearestOn(rings, down.back());
        if (a.ring == b.ring) {
            mid = walkRing(rings[a.ring], a, b);
            break;
        }
        const auto [x, y] = closestPair(rings[a.ring], rings[b.ring]);
        if (segmentInside(x, y, rings, tol)) {
            const RingPoint xa = nearestOn({rings[a.ring]}, x);
            const RingPoint yb = nearestOn({rings[b.ring]}, y);
            mid = walkRing(rings[a.ring], a, RingPoint{a.ring, xa.edge, xa.t, xa.point});
            const std::vector<Vec2> tail =
                walkRing(rings[b.ring], RingPoint{b.ring, yb.edge, yb.t, yb.point}, b);
            mid.insert(mid.end(), tail.begin(), tail.end());
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
        while (j + 1 < clean.size() && segmentInside(clean[i], clean[j + 1], region, tol)) {
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
    Emitter(Toolpath& tp, const PocketParams& p, double linkLimit)
        : tp_(tp), p_(p), linkLimit_(linkLimit), minRamp_(10.0 * p.tolerance.distance) {}

    /// Cuts the paths of one cluster at every level. Short links inside the centre region feed at
    /// depth; longer ways (and the way back to the first path of the next level) travel inside the
    /// centre region at the floor of the previous level (the surface for the first), which is
    /// already cleared there, along a straight segment or a route through `tree`; only when
    /// neither exists does the tool retract to safe Z.
    void cluster(const std::vector<const PocketPath*>& paths, const std::vector<std::size_t>& nodes,
                 const std::vector<double>& levels, const std::vector<Ring>& region, const RingTree* tree) {
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
                           segmentInside(pos_, s, region, tol)) {
                    feedTo(s, z);
                } else if (!atSafe_ && inCluster_ && z_ <= zTop && travel(s, zTop, node, region, tree)) {
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
    bool travel(Vec2 s, double zTravel, std::size_t node, const std::vector<Ring>& region,
                const RingTree* tree) {
        const double tol = p_.tolerance.distance;
        std::vector<Vec2> way;
        if (segmentInside(pos_, s, region, tol)) {
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
    Tolerance tol = userTol;
    tol.chordal = 0.5 * userTol.chordal;
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
            for (const auto& s : c.spans) {
                // Same count polygonisation will produce (lines 1, arcs by sag, Beziers bounded).
                inputVertices += std::visit(
                    [&](const auto& sp) -> double {
                        using S = std::decay_t<decltype(sp)>;
                        if constexpr (std::is_same_v<S, geometry::ArcSpan>) {
                            return static_cast<double>(sp.segmentCountFor(tol.chordal));
                        } else if constexpr (std::is_same_v<S, geometry::BezierSpan>) {
                            const double len = geometry::distance(sp.p0, sp.p1) +
                                               geometry::distance(sp.p1, sp.p2) +
                                               geometry::distance(sp.p2, sp.p3);
                            return 1.0 + std::sqrt(len / tol.chordal);
                        } else {
                            return 1.0;
                        }
                    },
                    s);
            }
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
        const std::vector<Contour> centreRegion = insetBy(region, r + params.allowance + margin, tol);
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
        std::vector<Contour> cutArea;
        std::vector<std::size_t> nodes;
        std::vector<RingTree> trees;
        if (!isRest) {
            plan.paths = std::move(pattern.paths);
            nodes = std::move(pattern.node);
            trees = std::move(pattern.trees);
            cutArea = offsetBy(plan.centreRegion, r, fine);
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
            plan.paths = clipPattern(pattern.paths, need, tol);
            if (plan.paths.empty()) {
                result.messages.push_back(label + ": cannot reach the rest material, nothing to cut");
                continue;
            }
            nodes.assign(plan.paths.size(), kNoNode);
            cutArea = offsetBy(intersection(needContours, plan.centreRegion, tol), r, fine);
        }
        swept = swept.empty() ? cutArea : geometry::unionContours([&] {
            std::vector<Contour> all = swept;
            all.insert(all.end(), cutArea.begin(), cutArea.end());
            return all;
        }());

        Toolpath& tp = result.toolpaths[ti];
        Emitter emitter(tp, params, 2.0 * d);
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
            // Travel stays inside this cluster's own part of the centre region (rest clusters,
            // which are parts of the rest material, use the whole centre region).
            const bool own = !isRest && cl < centre.size();
            const std::vector<Ring> clusterRegion = own ? ringsOf(centre[cl]) : ringsOf(centre);
            const RingTree* tree =
                own && cl < trees.size() && !trees[cl].rings.empty() ? &trees[cl] : nullptr;
            emitter.cluster(members, memberNodes, plan.levels, clusterRegion, tree);
            start = end;
        }
        emitter.finish();
    }
    return result;
}

}  // namespace carvetto::toolpath
