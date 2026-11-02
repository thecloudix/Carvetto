// T-0018: closed contours offset inwards past their inradius must vanish, not come back inside
// out, and shapes with several lobes keep exactly the lobes that genuinely survive.
//
// The generative test checks `offsetContours` against an oracle that shares no code with it: the
// region boundary is sampled straight from the spans (lines, arcs, Beziers), distances and
// inside tests are brute force over those samples, and the inradius is the refined maximum of
// the sampled distance-to-boundary field (the medial axis' widest point).
#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <random>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/offset.hpp"

using namespace carvetto::geometry;

namespace {

const Tolerance kTol{};  // distance 0.001, chordal 0.01
constexpr double kTwoPi = 6.283185307179586;

// ---------------------------------------------------------------------------------------------
// Oracle: dense boundary samples, brute-force distance and winding number.
// ---------------------------------------------------------------------------------------------

using Ring = std::vector<Vec2>;

// Samples a closed contour straight from its spans: line ends, arcs at a sag of at most 1e-4 mm,
// Beziers at 64 uniform parameters per span (sag far below the offset's chordal tolerance).
Ring sampleRing(const Contour& c) {
    Ring r;
    for (const Span& s : c.spans) {
        if (const auto* l = std::get_if<LineSpan>(&s)) {
            r.push_back(l->start);
        } else if (const auto* a = std::get_if<ArcSpan>(&s)) {
            const double sweep = std::fabs(a->sweepAngle());
            const double step = 2.0 * std::acos(std::max(0.0, 1.0 - 1e-4 / a->radius));
            const int n = std::max(4, static_cast<int>(std::ceil(sweep / step)));
            for (int i = 0; i < n; ++i) {
                r.push_back(a->pointAt(static_cast<double>(i) / n));
            }
        } else if (const auto* b = std::get_if<BezierSpan>(&s)) {
            for (int i = 0; i < 64; ++i) {
                r.push_back(b->pointAt(static_cast<double>(i) / 64.0));
            }
        }
    }
    return r;
}

// Winding number of p about a closed ring (crossing rule with signed edges).
int winding(const Ring& r, Vec2 p) {
    int w = 0;
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        const Vec2 a = r[i];
        const Vec2 b = r[(i + 1) % n];
        const double cross = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
        if (a.y <= p.y) {
            if (b.y > p.y && cross > 0.0) {
                ++w;
            }
        } else if (b.y <= p.y && cross < 0.0) {
            --w;
        }
    }
    return w;
}

double ringArea(const Ring& r) {
    double a = 0.0;
    for (std::size_t i = 0, n = r.size(); i < n; ++i) {
        a += r[i].x * r[(i + 1) % n].y - r[(i + 1) % n].x * r[i].y;
    }
    return 0.5 * a;
}

struct Region {
    std::string name;
    std::vector<Contour> contours;  // outer(s) first, holes wound the other way
};

struct Oracle {
    struct Seg {
        double ax, ay, dx, dy, inv;  // start, direction (end - start), 1 / |direction|^2
    };
    std::vector<Ring> rings;
    std::vector<Seg> segs;
    Vec2 lo{1e300, 1e300};
    Vec2 hi{-1e300, -1e300};

    explicit Oracle(const Region& reg) {
        for (const Contour& c : reg.contours) {
            rings.push_back(sampleRing(c));
            for (const Vec2 p : rings.back()) {
                lo = {std::min(lo.x, p.x), std::min(lo.y, p.y)};
                hi = {std::max(hi.x, p.x), std::max(hi.y, p.y)};
            }
            const Ring& r = rings.back();
            for (std::size_t i = 0, n = r.size(); i < n; ++i) {
                const Vec2 a = r[i];
                const Vec2 d = r[(i + 1) % n] - a;
                const double l2 = d.dot(d);
                segs.push_back(Seg{a.x, a.y, d.x, d.y, l2 > 0.0 ? 1.0 / l2 : 0.0});
            }
        }
    }
    bool inside(Vec2 p) const {
        int w = 0;
        for (const Ring& r : rings) {
            w += winding(r, p);
        }
        return w != 0;
    }
    double dist(Vec2 p) const {
        double best = 1e300;  // squared
        for (const Seg& g : segs) {
            const double px = p.x - g.ax;
            const double py = p.y - g.ay;
            const double t = std::clamp((px * g.dx + py * g.dy) * g.inv, 0.0, 1.0);
            const double ex = px - t * g.dx;
            const double ey = py - t * g.dy;
            best = std::min(best, ex * ex + ey * ey);
        }
        return std::sqrt(best);
    }
    // Signed distance: positive inside the region.
    double sdist(Vec2 p) const { return inside(p) ? dist(p) : -dist(p); }
};

// Distance field on a grid over the region, and the inradius: the largest distance from an
// inside point to the boundary, found on the grid and refined by shrinking pattern searches
// from the best nodes (see makeField). The search can fall a little short of the true maximum,
// never beyond it (every value it reports is a real distance).
struct Field {
    Vec2 origin;
    double h{0.0};
    int nx{0};
    int ny{0};
    std::vector<double> sd;  // signed distance per node
    double inradius{0.0};

    Vec2 node(int i, int j) const { return origin + Vec2{i * h, j * h}; }
    double at(int i, int j) const { return sd[static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i)]; }
};

Field makeField(const Oracle& o, int cells) {
    Field f;
    const double ext = std::max(o.hi.x - o.lo.x, o.hi.y - o.lo.y);
    f.h = ext / cells;
    f.origin = o.lo - Vec2{f.h, f.h};
    f.nx = static_cast<int>((o.hi.x - o.lo.x) / f.h) + 3;
    f.ny = static_cast<int>((o.hi.y - o.lo.y) / f.h) + 3;
    f.sd.resize(static_cast<std::size_t>(f.nx) * static_cast<std::size_t>(f.ny));
    for (int j = 0; j < f.ny; ++j) {
        for (int i = 0; i < f.nx; ++i) {
            f.sd[static_cast<std::size_t>(j) * static_cast<std::size_t>(f.nx) + static_cast<std::size_t>(i)] =
                o.sdist(f.node(i, j));
        }
    }
    // The true maximum lies within h/sqrt(2) of a node, and the field is 1-Lipschitz, so that
    // node is within 0.71 h of the inradius. The nodes that close to the best one, thinned to
    // one per 1.5 h (best first), seed zoom searches: a 5 x 5 pattern over +-step around the
    // best point so far, step starting at 2 h (covering the thinned neighbours) and halving down
    // to 1e-5 mm. A full pattern, unlike a compass search, also climbs ridges of the medial axis
    // that run at any angle.
    double top = 0.0;
    for (const double v : f.sd) {
        top = std::max(top, v);
    }
    std::vector<std::pair<double, Vec2>> candidates;
    for (int j = 0; j < f.ny; ++j) {
        for (int i = 0; i < f.nx; ++i) {
            const double v = f.at(i, j);
            if (v > 0.0 && v >= top - 0.75 * f.h) {
                candidates.emplace_back(v, f.node(i, j));
            }
        }
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const auto& x, const auto& y) { return x.first > y.first; });
    std::vector<std::pair<double, Vec2>> seeds;
    for (const auto& cand : candidates) {
        const bool covered = std::any_of(seeds.begin(), seeds.end(), [&](const auto& s) {
            return distance(s.second, cand.second) < 1.5 * f.h;
        });
        if (!covered) {
            seeds.push_back(cand);
        }
    }
    double best = top;
    for (auto [v, p] : seeds) {
        for (double step = 2.0 * f.h; step > 1e-5; step *= 0.5) {
            const Vec2 centre = p;
            for (int dj = -2; dj <= 2; ++dj) {
                for (int di = -2; di <= 2; ++di) {
                    const Vec2 q = centre + Vec2{di * 0.5 * step, dj * 0.5 * step};
                    const double w = o.sdist(q);
                    if (w > v) {
                        v = w;
                        p = q;
                    }
                }
            }
        }
        best = std::max(best, v);
    }
    f.inradius = best;
    return f;
}

// ---------------------------------------------------------------------------------------------
// Random regions: circles, polygons (stars, needles), arc shapes, Bezier blobs, multi-lobed
// shapes, each optionally with holes and either orientation.
// ---------------------------------------------------------------------------------------------

struct Rng {
    std::mt19937_64 g;
    explicit Rng(std::uint64_t seed) : g(seed) {}
    // Portable uniform double (std::uniform_real_distribution differs between standard libraries).
    double uni(double a, double b) { return a + (b - a) * (static_cast<double>(g() >> 11) / 9007199254740992.0); }
    int pick(int n) { return static_cast<int>(g() % static_cast<std::uint64_t>(n)); }
};

Contour polygon(const std::vector<Vec2>& pts) {
    Contour c;
    c.closed = true;
    for (std::size_t i = 0, n = pts.size(); i < n; ++i) {
        c.spans.emplace_back(LineSpan{pts[i], pts[(i + 1) % n]});
    }
    return c;
}

Contour star(Rng& r, Vec2 c, double rmin, double rmax, int k) {
    std::vector<Vec2> pts;
    for (int i = 0; i < k; ++i) {
        const double a = kTwoPi * (i + r.uni(-0.3, 0.3)) / k;
        pts.push_back(c + Vec2::fromAngle(a, r.uni(rmin, rmax)));
    }
    return polygon(pts);
}

// Star whose every other vertex is a thin needle tip (turns close to a full reversal).
Contour needleStar(Rng& r, Vec2 c, double rin, double rout, int k) {
    std::vector<Vec2> pts;
    for (int i = 0; i < k; ++i) {
        const double a = kTwoPi * i / k;
        const double half = r.uni(0.004, 0.03);
        pts.push_back(c + Vec2::fromAngle(a - half, rin));
        pts.push_back(c + Vec2::fromAngle(a, rout * r.uni(0.7, 1.0)));
        pts.push_back(c + Vec2::fromAngle(a + half, rin));
    }
    return polygon(pts);
}

// Convex-ish polygon whose edges are replaced by arcs bulging out or in (|bulge| small enough
// to keep the contour simple).
Contour arcGon(Rng& r, Vec2 c, double rad, int k) {
    std::vector<Vec2> pts;
    for (int i = 0; i < k; ++i) {
        pts.push_back(c + Vec2::fromAngle(kTwoPi * (i + r.uni(-0.2, 0.2)) / k, rad * r.uni(0.8, 1.0)));
    }
    Contour out;
    out.closed = true;
    for (int i = 0; i < k; ++i) {
        const Vec2 a = pts[static_cast<std::size_t>(i)];
        const Vec2 b = pts[static_cast<std::size_t>((i + 1) % k)];
        const double chord = distance(a, b);
        const double sag = chord * r.uni(-0.15, 0.25);  // + bulges outwards (CCW polygon)
        if (std::fabs(sag) < 1e-3) {
            out.spans.emplace_back(LineSpan{a, b});
            continue;
        }
        // Circle through a, b with the given sag: radius R, centre on the chord's normal.
        const double radius = (chord * chord / 4.0 + sag * sag) / (2.0 * std::fabs(sag));
        const Vec2 mid = (a + b) * 0.5;
        const Vec2 dir = (b - a) * (1.0 / chord);
        const Vec2 leftN{-dir.y, dir.x};
        // Outward bulge of a CCW polygon goes to the right of a->b: centre to the left of the
        // chord by radius - sag; inward bulge mirrors it.
        const double off = radius - std::fabs(sag);
        const Vec2 centre = sag > 0.0 ? mid + leftN * off : mid - leftN * off;
        out.spans.emplace_back(ArcSpan::fromCentre(centre, a, b, sag > 0.0 ? ArcDirection::CCW : ArcDirection::CW));
    }
    return out;
}

// Star-shaped smooth blob of cubic Beziers (radius varies around the centre).
Contour bezierBlob(Rng& r, Vec2 c, double rmin, double rmax, int k) {
    std::vector<Vec2> pts;
    std::vector<Vec2> tans;
    for (int i = 0; i < k; ++i) {
        const double a = kTwoPi * i / k;
        const Vec2 p = c + Vec2::fromAngle(a, r.uni(rmin, rmax));
        pts.push_back(p);
    }
    for (int i = 0; i < k; ++i) {
        const Vec2 prev = pts[static_cast<std::size_t>((i + k - 1) % k)];
        const Vec2 next = pts[static_cast<std::size_t>((i + 1) % k)];
        tans.push_back((next - prev) * (1.0 / 6.0));  // Catmull-Rom tangents
    }
    Contour out;
    out.closed = true;
    for (int i = 0; i < k; ++i) {
        const std::size_t i0 = static_cast<std::size_t>(i);
        const std::size_t i1 = static_cast<std::size_t>((i + 1) % k);
        out.spans.emplace_back(BezierSpan{pts[i0], pts[i0] + tans[i0], pts[i1] - tans[i1], pts[i1]});
    }
    return out;
}

// Two circles joined by a straight neck: big arc about each centre and two neck lines.
Contour dumbbell(Vec2 c1, double r1, Vec2 c2, double r2, double neck) {
    const Vec2 u = (c2 - c1) * (1.0 / distance(c1, c2));
    const Vec2 n{-u.y, u.x};
    const double a1 = std::asin(neck / r1);
    const double a2 = std::asin(neck / r2);
    const Vec2 p1top = c1 + u * (r1 * std::cos(a1)) + n * neck;
    const Vec2 p1bot = c1 + u * (r1 * std::cos(a1)) - n * neck;
    const Vec2 p2top = c2 - u * (r2 * std::cos(a2)) + n * neck;
    const Vec2 p2bot = c2 - u * (r2 * std::cos(a2)) - n * neck;
    Contour out;
    out.closed = true;
    out.spans.emplace_back(ArcSpan::fromCentre(c1, p1top, p1bot, ArcDirection::CCW));
    out.spans.emplace_back(LineSpan{p1bot, p2bot});
    out.spans.emplace_back(ArcSpan::fromCentre(c2, p2bot, p2top, ArcDirection::CCW));
    out.spans.emplace_back(LineSpan{p2top, p1top});
    return out;
}

// Circle as four Beziers (the offset code then sees no ArcSpan at all).
Contour bezierCircle(Vec2 c, double rad) {
    Contour out;
    out.closed = true;
    for (const BezierSpan& b : ArcSpan::fullCircle(c, rad).toBeziers()) {
        out.spans.emplace_back(b);
    }
    return out;
}

Contour outerShape(Rng& r, int kind, std::string& name) {
    const Vec2 c{r.uni(-50.0, 50.0), r.uni(-50.0, 50.0)};
    switch (kind) {
        case 0:
            name = "circle";
            return Contour::circle(c, r.uni(1.0, 25.0));
        case 1:
            name = "bezier circle";
            return bezierCircle(c, r.uni(1.0, 25.0));
        case 2:
            name = "star";
            return star(r, c, r.uni(2.0, 8.0), r.uni(10.0, 25.0), 5 + r.pick(20));
        case 3:
            name = "needle star";
            return needleStar(r, c, r.uni(4.0, 10.0), r.uni(14.0, 25.0), 3 + r.pick(8));
        case 4:
            name = "arc polygon";
            return arcGon(r, c, r.uni(4.0, 25.0), 3 + r.pick(8));
        case 5:
            name = "bezier blob";
            return bezierBlob(r, c, r.uni(3.0, 10.0), r.uni(12.0, 25.0), 4 + r.pick(9));
        case 6: {
            name = "dumbbell";
            const double r1 = r.uni(3.0, 15.0);
            const double r2 = r.uni(3.0, 15.0);
            const double gap = r.uni(1.05, 1.6) * (r1 + r2);  // lobes apart: the contour stays simple
            const double neck = r.uni(0.1, 0.8) * std::min(r1, r2);
            return dumbbell(c, r1, c + Vec2::fromAngle(r.uni(0.0, kTwoPi), gap), r2, neck);
        }
        default: {
            name = "rect";
            const Vec2 sz{r.uni(2.0, 40.0), r.uni(2.0, 40.0)};
            return Contour::rectangle(Box2{c, c + sz});
        }
    }
}

Region randomRegion(Rng& r) {
    Region reg;
    const int kind = r.pick(8);
    reg.contours.push_back(outerShape(r, kind, reg.name));
    // Holes: up to two circles or small polygons, kept clear of the wall and of each other.
    const int holes = r.pick(3);
    if (holes > 0) {
        const Oracle o(reg);
        std::vector<std::pair<Vec2, double>> placed;
        for (int attempt = 0; attempt < 60 && static_cast<int>(placed.size()) < holes; ++attempt) {
            const Vec2 p{r.uni(o.lo.x, o.hi.x), r.uni(o.lo.y, o.hi.y)};
            if (!o.inside(p)) {
                continue;
            }
            double room = o.dist(p);
            for (const auto& [q, rad] : placed) {
                room = std::min(room, distance(p, q) - rad);
            }
            if (room < 1.0) {
                continue;
            }
            const double rad = room * r.uni(0.2, 0.7);
            Contour hole;
            switch (r.pick(3)) {
                case 0:
                    hole = Contour::circle(p, rad, ArcDirection::CW);
                    break;
                case 1:
                    hole = star(r, p, 0.5 * rad, rad, 3 + r.pick(6)).reversed();
                    break;
                default:
                    hole = bezierCircle(p, rad).reversed();
                    break;
            }
            reg.contours.push_back(std::move(hole));
            placed.emplace_back(p, rad);
        }
        reg.name += " +" + std::to_string(placed.size()) + " holes";
    }
    // Either orientation: offsetContours takes "outwards" from the outermost contour.
    if (r.pick(2) == 1) {
        for (Contour& c : reg.contours) {
            c = c.reversed();
        }
        reg.name += " (cw)";
    }
    return reg;
}

const char* joinName(JoinType j) {
    return j == JoinType::Round ? "round" : j == JoinType::Sharp ? "sharp" : "square";
}

// How far beyond d a Sharp/Square inset may still leave a point out (the corner a mitre or
// chamfer cuts off at a reflex vertex of the region lies within this many d of that vertex).
double joinReach(JoinType j, double mitreLimit) {
    return j == JoinType::Round ? 1.0 : j == JoinType::Square ? std::sqrt(2.0) : std::max(mitreLimit, std::sqrt(2.0));
}

struct Checked {
    int failures{0};
    std::string first;  // the first few failures, one per line
    void fail(const std::string& what) {
        if (failures++ < 8) {
            first += what + "\n";
        }
    }
};

// Checks one inset result against the oracle:
//  (a) every output vertex and edge midpoint lies inside the region at distance >= d - slack
//      from its boundary (so nothing survives past the inradius, and no inside-out phantom);
//  (b) every grid node at distance >= reach*d + margin is covered (no genuine lobe lost);
//  (c) output rings nested at even depth have the outermost source's orientation, at odd depth
//      the opposite (only genuine holes are reversed).
void checkInset(const Region& reg, const Oracle& o, const Field& f, const std::vector<Contour>& out, double d,
                JoinType join, double sourceSign, Checked& chk) {
    const double slack = 2.0 * kTol.chordal + 1e-5;
    const double margin = 2.0 * kTol.chordal + 1e-5;
    const std::string tag = reg.name + " " + joinName(join) + " d=" + std::to_string(d) +
                            " inradius=" + std::to_string(f.inradius);
    std::vector<Ring> rings;
    for (const Contour& c : out) {
        Ring ring;
        for (const Span& s : c.spans) {
            ring.push_back(startPoint(s));
        }
        rings.push_back(ring);
    }
    for (const Ring& ring : rings) {
        for (std::size_t i = 0, n = ring.size(); i < n; ++i) {
            for (const Vec2 p : {ring[i], (ring[i] + ring[(i + 1) % n]) * 0.5}) {
                const double sd = o.sdist(p);
                if (sd < d - slack) {
                    chk.fail(tag + ": output point (" + std::to_string(p.x) + "," + std::to_string(p.y) +
                             ") at signed distance " + std::to_string(sd));
                    return;
                }
            }
        }
    }
    const double need = joinReach(join, 2.0) * d + margin;
    for (int j = 0; j < f.ny; ++j) {
        for (int i = 0; i < f.nx; ++i) {
            if (f.at(i, j) < need) {
                continue;
            }
            int w = 0;
            for (const Ring& ring : rings) {
                w += winding(ring, f.node(i, j));
            }
            if (w == 0) {
                chk.fail(tag + ": node (" + std::to_string(f.node(i, j).x) + "," + std::to_string(f.node(i, j).y) +
                         ") at distance " + std::to_string(f.at(i, j)) + " not covered");
                return;
            }
        }
    }
    for (std::size_t k = 0; k < rings.size(); ++k) {
        int depth = 0;
        const Vec2 probe = (rings[k][0] + rings[k][1]) * 0.5;
        for (std::size_t m = 0; m < rings.size(); ++m) {
            if (m != k && winding(rings[m], probe) != 0) {
                ++depth;
            }
        }
        const double want = depth % 2 == 0 ? sourceSign : -sourceSign;
        if (ringArea(rings[k]) * want <= 0.0) {
            chk.fail(tag + ": ring " + std::to_string(k) + " at depth " + std::to_string(depth) +
                     " has the wrong orientation");
            return;
        }
    }
}

}  // namespace

// --- AC1 / AC2: generative test against the brute-force oracle -------------------------------

TEST_CASE("T-0018 AC1/AC2 generative: insets match a brute-force inradius and distance oracle",
          "[geometry][offset][inradius]") {
    Rng rng(0x7018'0018ULL);
    Checked chk;
    int regions = 0;
    int vanishChecks = 0;
    int lobeChecks = 0;
    int nearVanishKept = 0;  // genuine (deep enough) results just past the searched inradius
    for (int n = 0; n < 48; ++n) {
        const Region reg = randomRegion(rng);
        const Oracle o(reg);
        const Field f = makeField(o, 70);
        if (f.inradius < 0.2) {
            continue;
        }
        ++regions;
        const double sourceSign = ringArea(o.rings.front()) > 0.0 ? 1.0 : -1.0;
        const double rho = f.inradius;
        const double m = 3.0 * kTol.chordal;
        // Past the inradius nothing may come back. The searched inradius can fall short of the
        // true one (by up to ~0.05 mm seen, where the medial axis has a sharp, curved ridge), so
        // the verdict is check (a) below, which needs no inradius: anything that comes back must
        // lie at the full distance from the walls, which no point does once d exceeds the true
        // inradius by the slack. Results kept past the searched inradius are counted and must
        // stay rare (they measure the search, not the offset).
        std::vector<double> ds{rho + m,   std::max(1.05 * rho, rho + 0.1), 1.5 * rho,  3.0 * rho,
                               rho - m,   0.97 * rho, 0.8 * rho, 0.5 * rho, 0.2 * rho, rng.uni(0.05, 1.6) * rho};
        for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
            for (const double d : ds) {
                if (d <= 0.0) {
                    continue;
                }
                const std::vector<Contour> out = offsetContours(reg.contours, -d, join, kTol);
                if (d >= rho + m) {
                    ++vanishChecks;
                    nearVanishKept += out.empty() ? 0 : 1;
                }
                if (join == JoinType::Round && d <= rho - m) {
                    ++lobeChecks;
                    if (out.empty()) {
                        chk.fail(reg.name + " round d=" + std::to_string(d) + " < inradius " +
                                 std::to_string(rho) + " but nothing came back");
                        continue;
                    }
                }
                checkInset(reg, o, f, out, d, join, sourceSign, chk);
            }
        }
    }
    INFO("first failures:\n" << chk.first);
    CHECK(chk.failures == 0);
    CHECK(regions >= 40);
    CHECK(vanishChecks >= 400);
    CHECK(lobeChecks >= 150);
    CHECK(nearVanishKept <= vanishChecks / 50);
}

// Lobes of different sizes must each vanish at their own inradius and survive below it, also
// when the inset of one passes the other's (the phantom of a collapsed lobe used to linger).
TEST_CASE("T-0018 AC1: a dumbbell keeps exactly its surviving lobes", "[geometry][offset][inradius]") {
    const Contour shape = dumbbell({0.0, 0.0}, 10.0, {30.0, 0.0}, 4.0, 1.0);
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        INFO(joinName(join));
        // Both lobes survive.
        CHECK(offsetContours({shape}, -3.0, join, kTol).size() == 2);
        // Only the big lobe survives: one contour, around the big centre.
        for (const double d : {4.5, 6.0, 9.0, 9.9}) {
            INFO("d = " << d);
            const auto out = offsetContours({shape}, -d, join, kTol);
            REQUIRE(out.size() == 1);
            const Box2 b = out[0].bounds();
            CHECK(b.min.x > -10.0 + d - 0.05);
            CHECK(b.max.x < 10.0 - d + 0.25);  // the neck opening lets the inset reach a little further
        }
        // Past the big lobe's radius nothing is left.
        for (const double d : {10.1, 12.0, 20.0, 40.0}) {
            INFO("d = " << d);
            CHECK(offsetContours({shape}, -d, join, kTol).empty());
            CHECK(offsetContours({shape.reversed()}, -d, join, kTol).empty());
        }
    }
}

// --- AC2: regression for the reported cases ----------------------------------------------------

TEST_CASE("T-0018 AC2: a 17 mm circle inset by 19.2 mm or 30 mm vanishes", "[geometry][offset][inradius]") {
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        for (const double d : {17.0 + 3.0 * kTol.chordal, 19.2, 30.0, 100.0}) {
            INFO(joinName(join) << " d = " << d);
            CHECK(offsetContours({Contour::circle({0.0, 0.0}, 17.0)}, -d, join, kTol).empty());
            CHECK(offsetContours({Contour::circle({5.0, -3.0}, 17.0, ArcDirection::CW)}, -d, join, kTol).empty());
            CHECK(offsetContours({bezierCircle({0.0, 0.0}, 17.0)}, -d, join, kTol).empty());
            // Finer and coarser polygonisation too (the defect depended on the vertex angle).
            CHECK(offsetContours({Contour::circle({0.0, 0.0}, 17.0)}, -d, join, Tolerance{0.001, 1e-9, 0.001}).empty());
            CHECK(offsetContours({Contour::circle({0.0, 0.0}, 17.0)}, -d, join, Tolerance{0.001, 1e-9, 0.5}).empty());
        }
        // Just inside the radius a small circle remains, wound like its source.
        const auto small = offsetContours({Contour::circle({0.0, 0.0}, 17.0)}, -15.0, join, kTol);
        REQUIRE(small.size() == 1);
        const Ring ring = sampleRing(small[0]);
        CHECK(ringArea(ring) > 0.0);
        for (const Vec2 p : ring) {
            CHECK(std::fabs(p.length() - 2.0) <= 2.0 * kTol.chordal);
        }
        const auto smallCw = offsetContours({Contour::circle({0.0, 0.0}, 17.0, ArcDirection::CW)}, -15.0, join, kTol);
        REQUIRE(smallCw.size() == 1);
        CHECK(ringArea(sampleRing(smallCw[0])) < 0.0);
    }
}

TEST_CASE("T-0018 AC2: holes grow and stay holes while the region shrinks", "[geometry][offset][inradius]") {
    // Annulus 20 / 5: wall 15 thick, inradius 7.5.
    const std::vector<Contour> annulus{Contour::circle({0.0, 0.0}, 20.0),
                                       Contour::circle({0.0, 0.0}, 5.0, ArcDirection::CW)};
    for (const JoinType join : {JoinType::Round, JoinType::Sharp, JoinType::Square}) {
        INFO(joinName(join));
        for (const double d : {1.0, 4.0, 7.0}) {
            INFO("d = " << d);
            const auto out = offsetContours(annulus, -d, join, kTol);
            REQUIRE(out.size() == 2);
            const Ring a = sampleRing(out[0]);
            const Ring b = sampleRing(out[1]);
            const Ring& outer = ringArea(a) > 0.0 ? a : b;
            const Ring& hole = ringArea(a) > 0.0 ? b : a;
            CHECK(ringArea(outer) > 0.0);
            CHECK(ringArea(hole) < 0.0);
            for (const Vec2 p : outer) {
                CHECK(std::fabs(p.length() - (20.0 - d)) <= 2.0 * kTol.chordal);
            }
            for (const Vec2 p : hole) {
                CHECK(p.length() >= 5.0 + d - 2.0 * kTol.chordal);  // Sharp/Square hole corners reach further
            }
        }
        for (const double d : {7.6, 9.0, 15.0, 30.0}) {
            INFO("d = " << d);
            CHECK(offsetContours(annulus, -d, join, kTol).empty());
        }
    }
}

// --- AC3: open paths lying on a clip boundary (ADR-0006 §7) -------------------------------------

TEST_CASE("T-0018 AC3: open paths on a clip boundary: documented Clipper2 behaviour (ADR-0006 §7)",
          "[geometry][boolean][inradius]") {
    const Contour square = Contour::rectangle(Box2{{0.0, 0.0}, {10.0, 10.0}});
    auto length = [](const std::vector<Contour>& cs) {
        double len = 0.0;
        for (const Contour& c : cs) {
            for (const Span& s : c.spans) {
                len += distance(startPoint(s), endPoint(s));
            }
        }
        return len;
    };
    auto line = [](Vec2 a, Vec2 b) {
        Contour c;
        c.append(LineSpan{a, b});
        return c;
    };
    // Each test path runs 5 mm before, 10 mm along and 5 mm after one side of the square.
    const std::array<std::pair<Vec2, Vec2>, 8> paths{{{{-5.0, 0.0}, {15.0, 0.0}},
                                                      {{15.0, 0.0}, {-5.0, 0.0}},
                                                      {{-5.0, 10.0}, {15.0, 10.0}},
                                                      {{15.0, 10.0}, {-5.0, 10.0}},
                                                      {{0.0, -5.0}, {0.0, 15.0}},
                                                      {{0.0, 15.0}, {0.0, -5.0}},
                                                      {{10.0, -5.0}, {10.0, 15.0}},
                                                      {{10.0, 15.0}, {10.0, -5.0}}}};
    int keptInside = 0;
    int keptOutside = 0;
    int keptBoth = 0;
    for (const Contour& clip : {square, square.reversed()}) {
        for (const FillRule rule : {FillRule::NonZero, FillRule::EvenOdd}) {
            for (const auto& [a, b] : paths) {
                const Contour path = line(a, b);
                const auto in = booleanOp(BooleanOp::Intersection, {path}, {clip}, kTol, rule).open;
                const auto outside = booleanOp(BooleanOp::Difference, {path}, {clip}, kTol, rule).open;
                const double lin = length(in);
                const double lout = length(outside);
                INFO("path (" << a.x << "," << a.y << ")->(" << b.x << "," << b.y << ") inside " << lin
                              << " outside " << lout);
                // The 5 mm before and after are outside, every time.
                CHECK(lout >= 10.0 - 1e-6);
                // The 10 mm on the boundary go to the intersection, the difference or both, all
                // or nothing, and never to neither (so a boundary piece is never lost from both).
                CHECK((std::fabs(lin) < 1e-6 || std::fabs(lin - 10.0) < 1e-6));
                CHECK((std::fabs(lout - 10.0) < 1e-6 || std::fabs(lout - 20.0) < 1e-6));
                CHECK(lin + lout >= 20.0 - 1e-6);
                keptInside += lin > 5.0 ? 1 : 0;
                keptOutside += lin < 5.0 ? 1 : 0;
                keptBoth += lin > 5.0 && lout > 15.0 ? 1 : 0;
            }
        }
    }
    // All three outcomes occur, decided by Clipper2's sweep rather than by geometry: callers
    // must not rely on boundary pieces being kept (or dropped) by an intersection or difference.
    // Clipper2 1.4.0 drops e.g. the bottom side of a CCW square from the intersection.
    CHECK(keptInside > 0);
    CHECK(keptOutside > 0);
    CHECK(keptBoth > 0);
    const Contour bottom = line({-5.0, 0.0}, {15.0, 0.0});
    CHECK(booleanOp(BooleanOp::Intersection, {bottom}, {square}, kTol).open.empty());
    // Growing the clip by a tolerance decides the boundary for the caller: inside.
    const auto grown = offsetContours({square}, kTol.distance, JoinType::Sharp, kTol);
    CHECK(std::fabs(length(booleanOp(BooleanOp::Intersection, {bottom}, grown, kTol).open) - 10.002) < 1e-6);
}
