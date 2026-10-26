#include "carvetto/toolpath/pocket.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
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

/// True when the straight segment a→b stays inside the region of `rings` (boundary included):
/// no edge is crossed away from the end points and the midpoint is inside. Conservative: a
/// touch at a boundary vertex in the middle of the segment also rejects it.
bool segmentInside(Vec2 a, Vec2 b, const std::vector<Ring>& rings, double tol) {
    const Vec2 d = b - a;
    for (const Ring& r : rings) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2 c = r[i];
            const Vec2 e = r[(i + 1) % n] - c;
            const double denom = d.cross(e);
            if (std::fabs(denom) <= 1e-15 * d.length() * e.length()) {
                continue;  // parallel or collinear: running along the boundary is allowed
            }
            const double t = (c - a).cross(e) / denom;
            const double u = (c - a).cross(d) / denom;
            const double slack = 1e-12;
            if (t < -slack || t > 1.0 + slack || u < -slack || u > 1.0 + slack) {
                continue;
            }
            const Vec2 x = a + d * t;
            if (geometry::distance(x, a) > tol && geometry::distance(x, b) > tol) {
                return false;
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

/// Insets the region of `cs` by d > 0 (offsetContours with −d) and keeps only result regions
/// whose outer vertices all lie inside the input and at least d (less the join tolerance) from its
/// boundary. Clipper2 1.4.0 turns a polygon inset beyond its inradius inside out (a circle inset by
/// more than its radius comes back as a growing circle), so a valid-looking region can appear
/// where nothing fits; every vertex of a true inset is at distance >= d from the input, which
/// rejects those. Large inputs check an evenly spaced sample of at most ~4e6 / edges vertices.
std::vector<Contour> insetBy(const std::vector<Contour>& cs, double d, const Tolerance& tol) {
    const std::vector<Contour> raw = offsetBy(cs, -d, tol);
    if (raw.empty()) {
        return raw;
    }
    std::vector<Ring> base;
    std::size_t edges = 0;
    for (const Contour& c : cs) {
        base.push_back(ringOf(c));
        edges += base.back().size();
    }
    // Where two round joins meet, the vertex lies inside both arcs by up to the join tolerance
    // (tol.chordal), which the pocket's safety margin already budgets for.
    const double slack = tol.chordal + 1e-6;
    const std::size_t budget = std::max<std::size_t>(64, 4000000 / std::max<std::size_t>(1, edges));
    std::vector<Group> kept;
    for (Group& g : groupsOf(raw, tol.distance)) {
        const std::size_t n = g.outer.size();
        const std::size_t step = std::max<std::size_t>(1, n / budget);
        bool ok = true;
        for (std::size_t i = 0; i < n && ok; i += step) {
            const Vec2 v = g.outer[i];
            double dist = std::numeric_limits<double>::infinity();
            for (const Ring& r : base) {
                for (std::size_t j = 0, m = r.size(); j < m; ++j) {
                    dist = std::min(dist, distToSegment(v, r[j], r[(j + 1) % m]));
                }
            }
            ok = dist >= d - slack && insideRegion(base, v, 0.0);
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

// ------------------------------------------------------------------------- offset strategy

/// Rings of one connected centre region `g`, cut inside out.
void offsetPattern(const Group& g, double stepover, double radius, CutDirection dir, std::size_t cluster,
                   std::optional<Vec2>& cursor, const Tolerance& tol, double sliver,
                   std::vector<PocketPath>& out) {
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
                const std::vector<Contour> reach = offsetBy(contoursOf(next), radius, tol);
                const std::vector<Contour> residual = opened(difference(extra, reach, tol), sliver, tol);
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
    std::vector<std::vector<std::size_t>> byOrdinal(ordinals.size());
    for (std::size_t o = 0; o < ordinals.size(); ++o) {
        for (Group& grp : ordinals[o]) {
            const Vec2 probe = grp.outer.front();
            const std::size_t id = nodes.size();
            nodes.push_back(Node{std::move(grp), {}});
            byOrdinal[o].push_back(id);
            if (o == 0) {
                continue;
            }
            std::optional<std::size_t> parent;
            for (std::size_t q = o; q-- > 0 && !parent;) {
                for (std::size_t cand : byOrdinal[q]) {
                    if (insideRegion(ringsOf(nodes[cand].g), probe, tol.distance)) {
                        parent = cand;
                        break;
                    }
                }
            }
            nodes[parent.value_or(0)].children.push_back(id);
        }
    }

    // Post-order (children first): the innermost rings are cut first, the boundary last.
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, 0}};
    while (!stack.empty()) {
        auto& [id, childIdx] = stack.back();
        if (childIdx < nodes[id].children.size()) {
            const std::size_t c = nodes[id].children[childIdx++];
            stack.emplace_back(c, 0);
            continue;
        }
        appendGroupRings(nodes[id].g, dir, PocketPathRole::Ring, cluster, cursor, tol.distance, out);
        stack.pop_back();
    }
}

// ------------------------------------------------------------------------- raster strategy

struct RasterPiece {
    Vec2 a;  // lower u
    Vec2 b;  // higher u
    bool used{false};
};

void rasterPattern(const Group& g, const PocketParams& p, double stepover, double toolDiameter,
                   std::size_t cluster, std::optional<Vec2>& cursor, const Tolerance& tol,
                   std::vector<PocketPath>& out) {
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
    const auto n = static_cast<std::size_t>(std::max(1.0, std::ceil(span / stepover - 1e-9)));
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
    const double linkLimit = 2.0 * toolDiameter;
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
}

/// The full pattern of one tool over every connected part of its centre region.
std::vector<PocketPath> fullPattern(const std::vector<Group>& centre, const PocketParams& p, double stepover,
                                    double diameter, const Tolerance& tol, double sliver) {
    std::vector<PocketPath> out;
    std::optional<Vec2> cursor;
    for (std::size_t ci = 0; ci < centre.size(); ++ci) {
        if (p.strategy == PocketStrategy::Offset) {
            offsetPattern(centre[ci], stepover, 0.5 * diameter, p.direction, ci, cursor, tol, sliver, out);
        } else {
            rasterPattern(centre[ci], p, stepover, diameter, ci, cursor, tol, out);
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
    const std::vector<Contour> needContours = contoursOf(need);
    std::vector<PocketPath> pieces;
    for (std::size_t pi = 0; pi < pattern.size(); ++pi) {
        const Polyline& src = pattern[pi].path;
        Polyline open = src;
        if (src.closed && !src.points.empty()) {
            open.points.push_back(src.points.front());
            open.closed = false;
        }
        const double total = open.length();
        Contour subject = Contour::fromPolyline(open);
        const geometry::BooleanResult r =
            geometry::booleanOp(BooleanOp::Intersection, {subject}, needContours, tol, FillRule::EvenOdd);
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

// ------------------------------------------------------------------------- move emission

class Emitter {
   public:
    Emitter(Toolpath& tp, const PocketParams& p, double linkLimit)
        : tp_(tp), p_(p), linkLimit_(linkLimit), minRamp_(10.0 * p.tolerance.distance) {}

    /// Cuts the paths of one cluster at every level. `levelLinks`: the region is fully cleared at
    /// each level, so the tool may travel back to the first path at the level just cut.
    void cluster(const std::vector<const PocketPath*>& paths, const std::vector<double>& levels,
                 const std::vector<Ring>& region, bool levelLinks) {
        const double tol = p_.tolerance.distance;
        for (std::size_t k = 0; k < levels.size(); ++k) {
            const double z = levels[k];
            const double zTop = k == 0 ? -p_.startDepth : levels[k - 1];
            for (std::size_t i = 0; i < paths.size(); ++i) {
                const Polyline& path = paths[i]->path;
                const Vec2 s = path.points.front();
                if (!started_) {
                    rapidTo(s);
                    enter(path, zTop, z);
                } else if (!atSafe_ && z_ == z && geometry::distance(pos_, s) <= linkLimit_ &&
                           segmentInside(pos_, s, region, tol)) {
                    feedTo(s, z);
                } else if (!atSafe_ && levelLinks && i == 0 && k > 0 && z_ == zTop &&
                           segmentInside(pos_, s, region, tol)) {
                    feedTo(s, z_);
                    enter(path, zTop, z);
                } else {
                    retract();
                    rapidTo(s);
                    enter(path, zTop, z);
                }
                cut(path, z);
            }
        }
    }

    void finish() { retract(); }

   private:
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
    Vec2 pos_;
    double z_{0.0};
};

bool finite(double v) {
    return std::isfinite(v);
}

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
    const double margin = userTol.chordal;
    const double sliver = std::max(5.0 * userTol.chordal, 10.0 * userTol.distance);

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
        !finite(params.safeZ) || !finite(params.rasterAngle) || !(userTol.chordal > 0.0) ||
        !finite(params.rampDistance)) {
        result.messages.push_back(
            "invalid pocket parameters: cut depth must be > 0, start depth and allowance >= 0, all values "
            "finite");
        return result;
    }
    if (params.safeZ <= -params.startDepth) {
        result.messages.push_back("safe Z must be above the start depth");
        return result;
    }

    std::vector<Contour> closedInput;
    std::size_t openCount = 0;
    for (const Contour& c : contours) {
        if (c.closed) {
            closedInput.push_back(c);
        } else if (!c.isEmpty()) {
            ++openCount;
        }
    }
    if (openCount > 0) {
        result.messages.push_back(std::to_string(openCount) +
                                  " open contour(s) ignored: a pocket needs closed vectors");
    }
    const std::vector<Contour> region = geometry::unionContours(closedInput, tol, FillRule::EvenOdd);
    if (region.empty()) {
        result.messages.push_back("no closed region to pocket");
        return result;
    }
    const std::vector<Contour> target = insetBy(region, params.allowance + margin, tol);

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
        const double r = 0.5 * d;
        PocketToolPlan& plan = result.plans[ti];
        PassPlan passes;
        passes.startDepth = params.startDepth;
        passes.cutDepth = params.cutDepth;
        passes.passDepth = pt.passDepth;
        passes.maintainExactStep = params.maintainExactStep;
        passes.lastPassThickness = params.lastPassThickness;
        plan.levels = passes.levels();

        plan.centreRegion = insetBy(region, r + params.allowance + margin, tol);
        const std::vector<Group> centre = groupsOf(plan.centreRegion, tol.distance);
        if (centre.empty()) {
            result.messages.push_back(label + ": too large for the pocket, nothing to cut");
            continue;
        }
        const std::vector<PocketPath> pattern = fullPattern(centre, params, pt.stepover, d, tol, sliver);
        const bool isRest = ti > 0 && !swept.empty();
        std::vector<Contour> cutArea;
        if (!isRest) {
            plan.paths = pattern;
            cutArea = offsetBy(plan.centreRegion, r, tol);
        } else {
            plan.restMaterial = opened(difference(target, swept, tol), sliver, tol);
            if (plan.restMaterial.empty()) {
                result.messages.push_back(label + ": no rest material left, nothing to cut");
                continue;
            }
            const std::vector<Contour> needContours = offsetBy(plan.restMaterial, r + sliver, tol);
            // The pattern already lies inside the centre region; clipping it against
            // need ∩ centre would put the outermost ring exactly on the clip boundary, where
            // open-path clipping is ambiguous.
            const std::vector<Group> need = groupsOf(needContours, tol.distance);
            plan.paths = clipPattern(pattern, need, tol);
            if (plan.paths.empty()) {
                result.messages.push_back(label + ": cannot reach the rest material, nothing to cut");
                continue;
            }
            cutArea = offsetBy(intersection(needContours, plan.centreRegion, tol), r, tol);
        }
        swept = swept.empty() ? cutArea : geometry::unionContours([&] {
            std::vector<Contour> all = swept;
            all.insert(all.end(), cutArea.begin(), cutArea.end());
            return all;
        }());

        Toolpath& tp = result.toolpaths[ti];
        Emitter emitter(tp, params, 2.0 * d);
        const std::vector<Ring> centreRings = ringsOf(centre);
        for (std::size_t start = 0; start < plan.paths.size();) {
            std::vector<const PocketPath*> members;
            std::size_t end = start;
            while (end < plan.paths.size() && plan.paths[end].cluster == plan.paths[start].cluster) {
                members.push_back(&plan.paths[end]);
                ++end;
            }
            emitter.cluster(members, plan.levels, centreRings, !isRest);
            start = end;
        }
        emitter.finish();
    }
    return result;
}

}  // namespace carvetto::toolpath
