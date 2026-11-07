#include "v_carve_corners.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <map>
#include <optional>
#include <utility>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/v_carve.hpp"

namespace carvetto::toolpath::detail {

namespace {

using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;

/// Most passes per stretch (a stepover tiny against the stretch would otherwise make the work
/// explode; Pocket refuses such stepovers for the carving tool's rest clearing before this).
constexpr double kMaxPasses = 200000.0;

/// Margin for Clipper2's integer grid (1e-6 mm, ADR-0006): the offsets below are taken this much
/// smaller, so their rounding never makes an inset larger or a swept area wider than it is.
constexpr double kGridMargin = 4e-6;

/// The end mills' swept area is polygonised at the chordal tolerance divided by this, never finer
/// than kFinestChordal.
constexpr double kFineDivisor = 64.0;
constexpr double kFinestChordal = 2e-6;

/// A vertex of a part cut out of a patch by Clipper2 lies within this of the patch (its grid).
constexpr double kOnPatch = 1e-5;

/// Golden-section steps when locating the farthest point of a patch edge from the wall rings.
constexpr int kGoldenSteps = 30;

double distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = ab.lengthSquared();
    const double t = l2 > 0.0 ? std::clamp((p - a).dot(ab) / l2, 0.0, 1.0) : 0.0;
    return geometry::distance(p, a + ab * t);
}

Box2 segmentBox(Vec2 a, Vec2 b) {
    Box2 box;
    box.expand(a);
    box.expand(b);
    return box;
}

std::vector<Vec2> verticesOf(const Contour& c) {
    std::vector<Vec2> out;
    out.reserve(c.spans.size());
    for (const geometry::Span& s : c.spans) {
        out.push_back(geometry::startPoint(s));
    }
    return out;
}

/// Twice the signed area of a polygon, taken about its first vertex (exact enough far from the origin).
double doubleArea(const std::vector<Vec2>& p) {
    double a = 0.0;
    for (std::size_t i = 1; i + 1 < p.size(); ++i) {
        a += (p[i] - p[0]).cross(p[i + 1] - p[0]);
    }
    return a;
}

/// Crossing-number membership of p in a closed polygon.
bool insideRing(const std::vector<Vec2>& ring, Vec2 p) {
    bool in = false;
    for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
        const Vec2 a = ring[i];
        const Vec2 b = ring[(i + 1) % n];
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x > p.x) {
                in = !in;
            }
        }
    }
    return in;
}

/// True when p lies inside the closed polygon or within `eps` of its edges.
bool insideOrOn(const std::vector<Vec2>& ring, Vec2 p, double eps) {
    for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
        if (distToSegment(p, ring[i], ring[(i + 1) % n]) <= eps) {
            return true;
        }
    }
    return insideRing(ring, p);
}

/// The polygon's centroid (area-weighted), or its first vertex when it has no area.
Vec2 centroidOf(const std::vector<Vec2>& p) {
    double a = 0.0;
    Vec2 c{0.0, 0.0};
    for (std::size_t i = 1; i + 1 < p.size(); ++i) {
        const Vec2 u = p[i] - p[0];
        const Vec2 v = p[i + 1] - p[0];
        const double w = u.cross(v);
        a += w;
        c = c + (u + v) * (w / 3.0);
    }
    return a != 0.0 ? p[0] + c / a : p[0];
}

/// The edges of polygons, indexed by box, for distances to their union's boundary.
class EdgeIndex {
   public:
    explicit EdgeIndex(const std::vector<std::vector<Vec2>>& polygons) {
        std::vector<Box2> boxes;
        for (const std::vector<Vec2>& p : polygons) {
            for (std::size_t i = 0, n = p.size(); i < n; ++i) {
                const Vec2 a = p[i];
                const Vec2 b = p[(i + 1) % n];
                if (a == b) {
                    continue;
                }
                edges_.emplace_back(a, b);
                boxes.push_back(segmentBox(a, b));
            }
        }
        index_ = BoxIndex(std::move(boxes));
    }
    /// Distance from p to the nearest edge, or `cap` when none is closer.
    double distance(Vec2 p, double cap) const {
        double best = cap;
        const Vec2 h{cap, cap};
        for (const std::size_t i : index_.query(Box2{p - h, p + h})) {
            best = std::min(best, distToSegment(p, edges_[i].first, edges_[i].second));
        }
        return best;
    }
    /// Distance from p to the nearest edge (searching from `start` outwards); infinity without edges.
    double nearest(Vec2 p, double start) const {
        if (edges_.empty()) {
            return std::numeric_limits<double>::infinity();
        }
        const Box2& all = index_.bounds();
        const double span = geometry::distance(all.min, all.max) + geometry::distance(p, all.centre());
        for (double cap = std::max(start, 1e-9);; cap *= 2.0) {
            const double d = distance(p, cap);
            if (d < cap || cap > span) {
                return d;
            }
        }
    }

   private:
    std::vector<std::pair<Vec2, Vec2>> edges_;
    BoxIndex index_;
};

/// A closed polygon with its edges indexed by box: crossing-number membership (as `insideRing`)
/// in the time of the edges a ray from the point meets, not of all of them.
class IndexedRing {
   public:
    explicit IndexedRing(const std::vector<Vec2>& ring) {
        std::vector<Box2> boxes;
        for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
            const Vec2 a = ring[i];
            const Vec2 b = ring[(i + 1) % n];
            if (a == b) {
                continue;
            }
            edges_.emplace_back(a, b);
            boxes.push_back(segmentBox(a, b));
            bounds_.expand(a);
        }
        index_ = BoxIndex(std::move(boxes));
        alongY_ = bounds_.width() > bounds_.height();
    }
    bool contains(Vec2 p) const {
        // The ray runs along the shorter side of the ring's box: a long row of parts has few
        // edges across it, and many along it.
        if (edges_.empty()) {
            return false;
        }
        bool in = false;
        if (alongY_) {
            if (!(bounds_.max.y > p.y)) {
                return false;
            }
            for (const std::size_t i : index_.query(Box2{p, Vec2{p.x, bounds_.max.y + 1.0}})) {
                const Vec2 a = edges_[i].first;
                const Vec2 b = edges_[i].second;
                if ((a.x > p.x) != (b.x > p.x)) {
                    const double y = a.y + (p.x - a.x) * (b.y - a.y) / (b.x - a.x);
                    if (y > p.y) {
                        in = !in;
                    }
                }
            }
            return in;
        }
        if (!(bounds_.max.x > p.x)) {
            return false;
        }
        for (const std::size_t i : index_.query(Box2{p, Vec2{bounds_.max.x + 1.0, p.y}})) {
            const Vec2 a = edges_[i].first;
            const Vec2 b = edges_[i].second;
            if ((a.y > p.y) != (b.y > p.y)) {
                const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
                if (x > p.x) {
                    in = !in;
                }
            }
        }
        return in;
    }

   private:
    std::vector<std::pair<Vec2, Vec2>> edges_;
    BoxIndex index_;
    Box2 bounds_;
    bool alongY_{false};
};

}  // namespace

// ----------------------------------------------------------------------------- RingRegion

RingRegion::RingRegion(std::vector<std::vector<Vec2>> rings, double eps) : rings_(std::move(rings)), eps_(eps) {
    std::vector<Box2> boxes;
    for (const std::vector<Vec2>& r : rings_) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            const Vec2 a = r[i];
            const Vec2 b = r[(i + 1) % n];
            if (a == b) {
                continue;
            }
            edges_.push_back(Edge{a, b});
            boxes.push_back(segmentBox(a, b));
            bounds_.expand(a);
        }
    }
    index_ = BoxIndex(std::move(boxes));
    alongY_ = bounds_.width() > bounds_.height();
}

bool RingRegion::inside(Vec2 p) const {
    if (edges_.empty()) {
        return false;
    }
    const Vec2 h{eps_, eps_};
    for (const std::size_t i : index_.query(Box2{p - h, p + h})) {
        if (distToSegment(p, edges_[i].a, edges_[i].b) <= eps_) {
            return true;
        }
    }
    // The ray runs along the shorter side of the rings' box: a long row of parts has few edges
    // across it, and many along it.
    bool in = false;
    if (alongY_) {
        if (!(bounds_.max.y > p.y)) {
            return false;
        }
        for (const std::size_t i : index_.query(Box2{p, Vec2{p.x, bounds_.max.y + 1.0}})) {
            const Vec2 a = edges_[i].a;
            const Vec2 b = edges_[i].b;
            if ((a.x > p.x) != (b.x > p.x)) {
                const double y = a.y + (p.x - a.x) * (b.y - a.y) / (b.x - a.x);
                if (y > p.y) {
                    in = !in;
                }
            }
        }
        return in;
    }
    if (!(bounds_.max.x > p.x)) {
        return false;
    }
    for (const std::size_t i : index_.query(Box2{p, Vec2{bounds_.max.x + 1.0, p.y}})) {
        const Vec2 a = edges_[i].a;
        const Vec2 b = edges_[i].b;
        if ((a.y > p.y) != (b.y > p.y)) {
            const double x = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (x > p.x) {
                in = !in;
            }
        }
    }
    return in;
}

double RingRegion::distance(Vec2 p, double cap) const {
    double best = cap;
    const Vec2 h{cap, cap};
    for (const std::size_t i : index_.query(Box2{p - h, p + h})) {
        best = std::min(best, distToSegment(p, edges_[i].a, edges_[i].b));
    }
    return best;
}

std::vector<std::pair<double, double>> RingRegion::clip(Vec2 a, Vec2 b) const {
    std::vector<std::pair<double, double>> out;
    const Vec2 d = b - a;
    const double len2 = d.lengthSquared();
    if (!(len2 > 0.0)) {
        if (inside(a)) {
            out.emplace_back(0.0, 1.0);
        }
        return out;
    }
    // Every parameter where the segment may change sides: proper crossings with an edge, and the
    // feet of edge end points within eps of the segment (touches, collinear overlaps).
    std::vector<double> ts{0.0, 1.0};
    for (const std::size_t i : index_.query(segmentBox(a, b).inflated(eps_))) {
        const Vec2 c = edges_[i].a;
        const Vec2 e = edges_[i].b - c;
        const double denom = d.cross(e);
        if (denom != 0.0) {
            const double t = (c - a).cross(e) / denom;
            const double u = (c - a).cross(d) / denom;
            if (t > 0.0 && t < 1.0 && u >= 0.0 && u <= 1.0) {
                ts.push_back(t);
            }
        }
        for (const Vec2 q : {c, edges_[i].b}) {
            if (distToSegment(q, a, b) <= eps_) {
                ts.push_back(std::clamp((q - a).dot(d) / len2, 0.0, 1.0));
            }
        }
    }
    std::sort(ts.begin(), ts.end());
    ts.erase(std::unique(ts.begin(), ts.end()), ts.end());
    for (std::size_t k = 0; k + 1 < ts.size(); ++k) {
        const double t0 = ts[k];
        const double t1 = ts[k + 1];
        if (!inside(a + d * (0.5 * (t0 + t1)))) {
            continue;
        }
        if (!out.empty() && out.back().second == t0) {
            out.back().second = t1;
        } else {
            out.emplace_back(t0, t1);
        }
    }
    return out;
}

bool RingRegion::segmentInside(Vec2 a, Vec2 b) const {
    const std::vector<std::pair<double, double>> in = clip(a, b);
    return in.size() == 1 && in.front().first == 0.0 && in.front().second == 1.0;
}

// ----------------------------------------------------------------------------- stretches

/// Appends to `out` the polylines of `moves` that run level at `floorZ` (each feed whose start
/// and end are at that height, consecutive ones chained): where an end mill of `radius` swept
/// the floor flat. Ramps, plunges and rapids are left out (they sweep less, so leaving them out
/// only makes the swept area smaller than it is).
void appendSweptPaths(const std::vector<Move>& moves, double floorZ, double radius, std::vector<SweptPath>& out) {
    const double tolZ = 1e-9 * std::max(1.0, std::fabs(floorZ));
    auto atFloor = [&](double z) { return std::fabs(z - floorZ) <= tolZ; };
    SweptPath cur;
    cur.radius = radius;
    auto flush = [&]() {
        if (cur.points.size() >= 2) {
            out.push_back(std::move(cur));
        }
        cur = SweptPath{};
        cur.radius = radius;
    };
    for (std::size_t i = 1; i < moves.size(); ++i) {
        const geometry::Vec3 a = moves[i - 1].to;
        const geometry::Vec3 b = moves[i].to;
        const bool level = (moves[i].kind == MoveKind::Feed || moves[i].kind == MoveKind::Plunge) && atFloor(a.z) &&
                           atFloor(b.z) && !(a.x == b.x && a.y == b.y);
        if (!level) {
            flush();
            continue;
        }
        if (cur.points.empty()) {
            cur.points.push_back(Vec2{a.x, a.y});
        }
        cur.points.push_back(Vec2{b.x, b.y});
    }
    flush();
}

std::vector<SharpStretch> sharpStretches(const RingRegion& walls, const std::vector<SweptPath>& swept,
                                         const CornerSpec& spec) {
    std::vector<SharpStretch> out;
    if (walls.isEmpty() || swept.empty() || spec.tool == nullptr || !(spec.stepover > 0.0) ||
        !(spec.chordal > 0.0) || !(spec.limit > spec.chordal)) {
        return out;  // min(F, h) never exceeds the chordal tolerance when F does not
    }
    // δ: points nearer the wall ring keep at most h(δ) ≤ min(chordal, h(s/2)) next to it.
    const double rc = 0.5 * tooling::cuttingWidthAtDepth(*spec.tool, spec.chordal);
    const double delta = std::min(rc, 0.5 * spec.stepover);
    if (!(delta > 0.0) || !std::isfinite(delta)) {
        return out;
    }
    const double margin = std::min(kGridMargin, 0.25 * delta);
    geometry::Tolerance tol;
    tol.chordal = 0.25 * spec.pocketChordal;
    tol.distance = std::min(spec.distance, tol.chordal);

    // The floor farther than δ from the wall rings (a superset: the round joins' chords at the
    // rings' reflex vertices lie inside the circles of radius δ, and the inset is a grid margin
    // short).
    std::vector<Contour> rings;
    for (const std::vector<Vec2>& r : walls.rings()) {
        if (r.size() >= 3) {
            rings.push_back(Contour::fromPolyline(geometry::Polyline{r, true}));
        }
    }
    std::vector<Contour> patches = geometry::offsetContours(rings, -(delta - margin), geometry::JoinType::Round, tol);
    if (patches.empty()) {
        return out;
    }
    // Less what the end mills swept at the flat depth (a subset: the round joins' and caps'
    // vertices lie on the circles of each end mill's radius, a grid margin short). Done per
    // locality: one Clipper2 call over a long row of parts costs more than the sum of its parts
    // (its sweep keeps every edge crossing a scanline active), so the patches and polylines are
    // grouped by the overlap of their boxes (a polyline's inflated by its radius) and each group
    // is offset and differenced alone. Groups share nothing, so the result is that of one call.
    // Finer round joins here: their chords decide how closely the patches, and so the threshold,
    // follow the end mills' fillets.
    geometry::Tolerance fine = tol;
    fine.chordal = std::max(spec.chordal / kFineDivisor, kFinestChordal);
    fine.distance = std::min(spec.distance, fine.chordal);
    std::vector<std::vector<Vec2>> sweptPolygons;
    {
        struct PathItem {
            double radius;
            Contour open;
        };
        std::vector<PathItem> items;
        std::vector<Box2> boxes;
        for (const Contour& c : patches) {
            Box2 box;
            for (const Vec2 v : verticesOf(c)) {
                box.expand(v);
            }
            boxes.push_back(box.inflated(kOnPatch));
        }
        const std::size_t patchCount = patches.size();
        for (const SweptPath& p : swept) {
            if (p.points.size() >= 2 && p.radius > margin) {
                Box2 box;
                for (const Vec2 v : p.points) {
                    box.expand(v);
                }
                boxes.push_back(box.inflated(p.radius + kOnPatch));
                items.push_back(PathItem{p.radius, Contour::fromPolyline(geometry::Polyline{p.points, false})});
            }
        }
        const BoxIndex index(boxes);
        std::vector<std::size_t> root(boxes.size());
        for (std::size_t i = 0; i < root.size(); ++i) {
            root[i] = i;
        }
        auto find = [&](std::size_t i) {
            while (root[i] != i) {
                root[i] = root[root[i]];
                i = root[i];
            }
            return i;
        };
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            for (const std::size_t j : index.query(boxes[i])) {
                const std::size_t ri = find(i);
                const std::size_t rj = find(j);
                if (ri != rj) {
                    root[std::max(ri, rj)] = std::min(ri, rj);  // the group's root: its first member
                }
            }
        }
        // Groups in the order of their first patch; groups without a polyline keep their patches.
        std::map<std::size_t, std::pair<std::vector<std::size_t>, std::vector<std::size_t>>> groups;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            auto& g = groups[find(i)];
            (i < patchCount ? g.first : g.second).push_back(i);
        }
        std::vector<Contour> kept;
        for (auto& entry : groups) {
            auto& members = entry.second;
            std::vector<Contour> part;
            for (const std::size_t i : members.first) {
                part.push_back(std::move(patches[i]));
            }
            std::map<double, std::vector<Contour>> paths;  // by end-mill radius
            for (const std::size_t i : members.second) {
                paths[items[i - patchCount].radius].push_back(std::move(items[i - patchCount].open));
            }
            for (const auto& [radius, open] : paths) {
                const std::vector<Contour> area =
                    geometry::offsetContours(open, radius - margin, geometry::JoinType::Round, fine);
                for (const Contour& c : area) {
                    sweptPolygons.push_back(verticesOf(c));
                }
                if (!area.empty() && !part.empty()) {
                    part = geometry::booleanOp(geometry::BooleanOp::Difference, part, area, tol,
                                               geometry::FillRule::NonZero)
                               .closed;
                }
            }
            for (Contour& c : part) {
                kept.push_back(std::move(c));
            }
        }
        patches = std::move(kept);
        if (patches.empty()) {
            return out;
        }
    }
    if (sweptPolygons.empty()) {
        return out;
    }
    const EdgeIndex sweptEdges(sweptPolygons);
    // The flat areas: each outer wall ring (counter-clockwise) with the rings inside it. A point
    // belongs to the smallest outer ring round it.
    const std::vector<std::vector<Vec2>>& wallRings = walls.rings();
    std::vector<std::size_t> outers;
    std::vector<double> outerArea;
    std::vector<IndexedRing> outerIndex;
    for (std::size_t i = 0; i < wallRings.size(); ++i) {
        const double a = doubleArea(wallRings[i]);
        if (wallRings[i].size() >= 3 && a > 0.0) {
            outers.push_back(i);
            outerArea.push_back(a);
            outerIndex.emplace_back(wallRings[i]);
        }
    }
    auto flatAreaOf = [&](Vec2 p) -> std::optional<std::size_t> {
        std::optional<std::size_t> best;
        for (std::size_t k = 0; k < outers.size(); ++k) {
            if ((!best || outerArea[k] < outerArea[*best]) && outerIndex[k].contains(p)) {
                best = k;
            }
        }
        return best;
    };
    std::vector<char> sweptArea(outers.size(), 0);
    std::size_t marked = 0;
    for (const SweptPath& p : swept) {
        if (marked == outers.size()) {
            break;
        }
        if (p.points.empty()) {
            continue;
        }
        if (const std::optional<std::size_t> k = flatAreaOf(p.points.front()); k && !sweptArea[*k]) {
            sweptArea[*k] = 1;
            ++marked;
        }
    }

    // The threshold: a patch is sharpened when part of it lies farther than r(chordal) from the
    // wall rings (a grid margin farther, so that part really does).
    // When r(chordal) ≤ s/2, δ is r(chordal) and every patch reaches it (less the grid margin).
    const bool allDeep = !(delta < rc);
    std::vector<std::vector<Vec2>> deepParts;
    if (!allDeep) {
        const std::vector<Contour> deep =
            geometry::offsetContours(rings, -(rc + margin), geometry::JoinType::Round, tol);
        if (!deep.empty()) {
            for (const Contour& c : geometry::booleanOp(geometry::BooleanOp::Intersection, patches, deep, tol,
                                                        geometry::FillRule::NonZero)
                                        .closed) {
                std::vector<Vec2> v = verticesOf(c);
                if (v.size() >= 3 && doubleArea(v) > 0.0) {
                    deepParts.push_back(std::move(v));
                }
            }
        }
    }
    if (!allDeep && deepParts.empty()) {
        return out;
    }

    for (const Contour& patch : patches) {
        std::vector<Vec2> poly = verticesOf(patch);
        if (poly.size() < 3 || !(doubleArea(poly) > 0.0)) {
            continue;  // a hole of the patches (an island the end mills swept inside one)
        }
        Box2 box;
        for (const Vec2 v : poly) {
            box.expand(v);
        }
        const double diag = geometry::distance(box.min, box.max);
        // Only in a flat area the end mills sweep: one they never enter is the carving tool's own
        // rest clearing, not a stretch the end mills could not reach into.
        const std::optional<std::size_t> area = flatAreaOf(poly.front());
        if (!area || !sweptArea[*area]) {
            continue;
        }
        bool deep = allDeep;
        for (std::size_t k = 0; !deep && k < deepParts.size(); ++k) {
            deep = insideOrOn(poly, deepParts[k].front(), kOnPatch);
        }
        if (!deep) {
            continue;  // below the threshold: left alone
        }
        // The outline's greatest distance from the wall rings: at its vertices, and between them
        // where the distance along an edge peaks (golden section; only on edges that can beat the
        // vertices, the distance being 1-Lipschitz). The search box grows from a little beyond δ.
        const double cap = diag + rc + spec.stepover + spec.limitRadius;
        auto f = [&](Vec2 p) {
            for (double r = 2.0 * delta;; r *= 2.0) {
                const double d = walls.distance(p, std::min(r, cap));
                if (d < r || !(r < cap)) {
                    return d;
                }
            }
        };
        const std::size_t n = poly.size();
        std::vector<double> fv(n);
        double farthest = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            fv[i] = f(poly[i]);
            farthest = std::max(farthest, fv[i]);
        }
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = poly[i];
            const Vec2 b = poly[(i + 1) % n];
            if (!(std::max(fv[i], fv[(i + 1) % n]) + 0.5 * geometry::distance(a, b) > farthest)) {
                continue;
            }
            double lo = 0.0;
            double hi = 1.0;
            constexpr double g = 0.381966011250105;
            double m1 = lo + g * (hi - lo);
            double m2 = hi - g * (hi - lo);
            double f1 = f(a + (b - a) * m1);
            double f2 = f(a + (b - a) * m2);
            for (int it = 0; it < kGoldenSteps; ++it) {
                if (f1 < f2) {
                    lo = m1;
                    m1 = m2;
                    f1 = f2;
                    m2 = hi - g * (hi - lo);
                    f2 = f(a + (b - a) * m2);
                } else {
                    hi = m2;
                    m2 = m1;
                    f2 = f1;
                    m1 = lo + g * (hi - lo);
                    f1 = f(a + (b - a) * m1);
                }
            }
            farthest = std::max({farthest, f1, f2});
        }
        const double predicted = std::min(spec.limit, vCarvingDepthForRadius(*spec.tool, farthest));
        // The root: the vertex farthest from the swept area (the wall ring's corner side), among
        // those on the inset rings (the others lie on the swept area's edge).
        Vec2 root = poly.front();
        double deepest = -1.0;
        for (const bool any : {false, true}) {
            for (std::size_t i = 0; i < n; ++i) {
                if (!any && !(fv[i] <= delta + kOnPatch)) {
                    continue;
                }
                const double d = sweptEdges.nearest(poly[i], delta);
                if (d > deepest) {
                    deepest = d;
                    root = poly[i];
                }
            }
            if (deepest >= 0.0) {
                break;
            }
        }
        Vec2 dir = centroidOf(poly) - root;
        const double dl = dir.length();
        dir = dl > 0.0 && std::isfinite(dl) ? dir / dl : Vec2{1.0, 0.0};
        out.push_back(SharpStretch{std::move(poly), root, dir, farthest, predicted});
    }
    return out;
}

std::vector<std::vector<Vec2>> stretchPasses(const SharpStretch& stretch, const RingRegion& walls,
                                             const CornerSpec& spec, double backReach) {
    std::vector<std::vector<Vec2>> cuts;
    const double s = spec.stepover;
    if (!(s > 0.0) || stretch.outline.size() < 3) {
        return cuts;
    }
    const Vec2 u = stretch.direction;
    const Vec2 n{-u.y, u.x};
    const Vec2 o = stretch.root;
    // The patch in the frame of its root: x along the direction, y across it.
    std::vector<Vec2> local;
    local.reserve(stretch.outline.size());
    double ymin = std::numeric_limits<double>::infinity();
    double ymax = -ymin;
    for (const Vec2 v : stretch.outline) {
        const Vec2 d = v - o;
        local.push_back(Vec2{d.dot(u), d.dot(n)});
        ymin = std::min(ymin, local.back().y);
        ymax = std::max(ymax, local.back().y);
    }
    const double half = 0.5 * s;
    const double kLo = std::ceil((ymin - half) / s);
    const double kHi = std::floor((ymax + half) / s);
    if (!(kHi >= kLo) || !(kHi - kLo < kMaxPasses)) {
        return cuts;
    }
    struct Piece {
        Vec2 from;
        Vec2 to;
    };
    std::vector<std::vector<Piece>> passes;
    for (double k = kLo; k <= kHi; k += 1.0) {
        const double y = k * s;
        const double lo = y - half;
        const double hi = y + half;
        // The extent along x of the patch within half a stepover of the line.
        double x0 = std::numeric_limits<double>::infinity();
        double x1 = -x0;
        for (std::size_t i = 0, m = local.size(); i < m; ++i) {
            const Vec2 a = local[i];
            const Vec2 b = local[(i + 1) % m];
            double t0 = 0.0;
            double t1 = 1.0;
            const double dy = b.y - a.y;
            if (dy == 0.0) {
                if (a.y < lo || a.y > hi) {
                    continue;
                }
            } else {
                double ta = (lo - a.y) / dy;
                double tb = (hi - a.y) / dy;
                if (ta > tb) {
                    std::swap(ta, tb);
                }
                t0 = std::max(t0, ta);
                t1 = std::min(t1, tb);
                if (t0 > t1) {
                    continue;
                }
            }
            for (const double t : {t0, t1}) {
                const double x = a.x + (b.x - a.x) * t;
                x0 = std::min(x0, x);
                x1 = std::max(x1, x);
            }
        }
        if (!(x1 >= x0)) {
            continue;
        }
        // From the wall ring (at most backReach back) to the chordal tolerance into the swept area.
        const double xs = x0 - backReach;
        const double xe = x1 + spec.chordal;
        const Vec2 p0 = o + u * xs + n * y;
        const Vec2 p1 = o + u * xe + n * y;
        const double len = xe - xs;
        const double needFrom = (x0 - xs) / len;
        const double needTo = (x1 - xs) / len;
        std::vector<Piece> pieces;
        for (const auto& [t0, t1] : walls.clip(p0, p1)) {
            if (t1 < needFrom || t0 > needTo) {
                continue;  // beyond the patch's extent: back extension past a wall, or not needed
            }
            const Vec2 a = p0 + (p1 - p0) * t0;
            const Vec2 b = p0 + (p1 - p0) * t1;
            if (geometry::distance(a, b) > spec.distance) {
                pieces.push_back(Piece{a, b});
            }
        }
        if (!pieces.empty()) {
            passes.push_back(std::move(pieces));
        }
    }
    // Zig-zag: every other pass runs backwards, so consecutive passes end near each other (both in
    // the swept area, or both on the ring) and join at depth when the link stays inside.
    std::vector<Vec2> cur;
    for (std::size_t i = 0; i < passes.size(); ++i) {
        std::vector<Piece> pieces = passes[i];
        if (i % 2 == 1) {
            std::reverse(pieces.begin(), pieces.end());
            for (Piece& p : pieces) {
                std::swap(p.from, p.to);
            }
        }
        for (const Piece& p : pieces) {
            if (!cur.empty() && !walls.segmentInside(cur.back(), p.from)) {
                cuts.push_back(std::move(cur));
                cur.clear();
            }
            if (cur.empty() || cur.back() != p.from) {
                cur.push_back(p.from);
            }
            cur.push_back(p.to);
        }
    }
    if (!cur.empty()) {
        cuts.push_back(std::move(cur));
    }
    return cuts;
}

}  // namespace carvetto::toolpath::detail
