#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "shapes.hpp"
using namespace carvetto::geometry;
using vo::dist;
struct Seg { Vec2 a, b; double ra, rb; };
static double covAt(const std::vector<Seg>& segs, const std::vector<MedialPoint>& iso, Vec2 x) {
    double best = 1e300;
    for (auto& m : iso) best = std::min(best, dist(x, m.point) - m.radius);
    for (auto& s : segs) {
        // quick reject
        double lb = std::min(dist(x, s.a) - s.ra, dist(x, s.b) - s.rb);
        double L = dist(s.a, s.b);
        if (lb - L - std::fabs(s.ra - s.rb) > best) continue;
        double lo = 0, hi = 1;
        for (int it = 0; it < 40; ++it) { double m1 = lo + (hi - lo) / 3, m2 = hi - (hi - lo) / 3;
            auto f = [&](double t) { return dist(x, s.a + (s.b - s.a) * t) - (s.ra + (s.rb - s.ra) * t); };
            if (f(m1) < f(m2)) hi = m2; else lo = m1; }
        double t = 0.5 * (lo + hi); best = std::min(best, dist(x, s.a + (s.b - s.a) * t) - (s.ra + (s.rb - s.ra) * t));
    }
    return best;
}
static double distToSkel(const std::vector<Seg>& segs, const std::vector<MedialPoint>& iso, Vec2 x) {
    double best = 1e300;
    for (auto& m : iso) best = std::min(best, dist(x, m.point));
    for (auto& s : segs) { Vec2 d = s.b - s.a; double L2 = d.dot(d); double t = L2 > 0 ? std::clamp((x - s.a).dot(d) / L2, 0.0, 1.0) : 0; best = std::min(best, dist(x, s.a + d * t)); }
    return best;
}
int main(int argc, char** argv) {
    double chordal = argc > 1 ? std::atof(argv[1]) : 0.01;
    const char* only = argc > 2 ? argv[2] : nullptr;
    Tolerance tol; tol.chordal = chordal;
    int fails = 0;
    for (auto& sh : vs::all()) {
        if (only && sh.name != only) continue;
        auto t0 = std::chrono::steady_clock::now();
        MedialAxis ax = medialAxis(sh.cs, tol);
        double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        vo::Boundary B = vo::makeBoundary(sh.cs, chordal / 4);
        std::vector<MedialPoint> pts; std::vector<Seg> segs; std::vector<MedialPoint> iso;
        for (auto& n : ax.nodes) { pts.push_back({n.point, n.radius}); if (n.branches.empty()) iso.push_back({n.point, n.radius}); }
        double maxRadErr = 0, maxDefect = 0, maxInterp = 0, worstTagDist = 0; Vec2 worstDefP; double worstDefR = 0;
        int outside = 0, nonfinite = 0, tagSideWrong = 0, tagBad = 0, nseg = 0, topo = 0;
        for (size_t bi = 0; bi < ax.branches.size(); ++bi) { auto& br = ax.branches[bi];
            if (br.sides.size() + 1 != br.points.size() || br.points.size() < 2) ++topo;
            if (br.startNode >= ax.nodes.size() || br.endNode >= ax.nodes.size()) { ++topo; continue; }
            if (!(br.points.front().point == ax.nodes[br.startNode].point) || !(br.points.back().point == ax.nodes[br.endNode].point)) ++topo;
            for (auto& p : br.points) pts.push_back(p);
            for (size_t i = 0; i + 1 < br.points.size(); ++i) {
                auto &A = br.points[i], &Bp = br.points[i + 1]; segs.push_back({A.point, Bp.point, A.radius, Bp.radius}); ++nseg;
                Vec2 mid = (A.point + Bp.point) * 0.5; double rm = 0.5 * (A.radius + Bp.radius);
                double dm = vo::exactDist(B, mid).d; maxInterp = std::max(maxInterp, std::fabs(dm - rm));
                if (i < br.sides.size()) for (int k = 0; k < 2; ++k) {
                    auto& ref = br.sides[i][k];
                    if (ref.contour >= sh.cs.size() || ref.span >= sh.cs[ref.contour].spans.size() || !(ref.t >= 0 && ref.t <= 1)) { ++tagBad; continue; }
                    Vec2 q = vo::evalSpan(sh.cs[ref.contour].spans[ref.span], ref.t);
                    // q should be (nearly) equidistant: |q-A| ~ A.radius, |q-B| ~ B.radius (within chordal)
                    double e = std::max(std::fabs(dist(q, A.point) - A.radius), std::fabs(dist(q, Bp.point) - Bp.radius));
                    worstTagDist = std::max(worstTagDist, e);
                    Vec2 dir = Bp.point - A.point; double cr = dir.cross(q - mid);
                    if ((k == 0 && cr < 0) || (k == 1 && cr > 0)) ++tagSideWrong;
                }
            }
        }
        size_t stride = std::max<size_t>(1, pts.size() / 1500);
        for (size_t i = 0; i < pts.size(); ++i) { auto& p = pts[i];
            if (!std::isfinite(p.point.x) || !std::isfinite(p.point.y) || !std::isfinite(p.radius)) { ++nonfinite; continue; }
            double d = vo::exactDist(B, p.point).d; maxRadErr = std::max(maxRadErr, std::fabs(d - p.radius));
            if (!vo::insideEO(B, p.point)) ++outside;
            if (i % stride == 0) { double df = vo::medialDefect2(B, p.point, p.radius, chordal); if (df > maxDefect) { maxDefect = df; worstDefP = p.point; worstDefR = p.radius; } }
        }
        // coverage: grid
        double W = B.maxx - B.minx, H = B.maxy - B.miny; double g = std::max(W, H) / 220.0;
        double maxCov = 0; Vec2 maxCovP; int nGrid = 0, nCov1 = 0, nCov3 = 0; double maxCovDepth = 0;
        double maxMiss = 0; Vec2 maxMissP; int nNear = 0; double maxMissR = 0;
        for (double y = B.miny + g / 2; y < B.maxy; y += g) for (double x = B.minx + g / 2; x < B.maxx; x += g) {
            Vec2 X{x, y}; if (!vo::insideEO(B, X)) continue; ++nGrid;
            double c = covAt(segs, iso, X);
            if (c > chordal) ++nCov1; if (c > 3 * chordal) ++nCov3;
            if (c > maxCov) { maxCov = c; maxCovP = X; maxCovDepth = vo::exactDist(B, X).d; }
        }
        // near-medial completeness on coarser grid
        double g2 = std::max(W, H) / 70.0;
        for (double y = B.miny + g2 / 3; y < B.maxy; y += g2) for (double x = B.minx + g2 / 3; x < B.maxx; x += g2) {
            Vec2 X{x, y}; if (!vo::insideEO(B, X)) continue;
            double r = vo::exactDist(B, X).d; if (r < 3 * chordal) continue;
            if (vo::medialDefect2(B, X, r, chordal) > chordal / 4) continue; ++nNear;
            double m = distToSkel(segs, iso, X); if (m > maxMiss) { maxMiss = m; maxMissP = X; maxMissR = r; }
        }
        double maxTip = 0, maxTipR = 0; Vec2 maxTipP;
        for (auto& n : ax.nodes) if (n.branches.size() == 1) {
            double dv = 1e300; for (auto& c : sh.cs) for (auto& sp : c.spans) dv = std::min(dv, dist(n.point, vo::evalSpan(sp, 0)));
            if (dv < 1.0 && dv > maxTip) { maxTip = dv; maxTipR = n.radius; maxTipP = n.point; } }
        std::printf("   endnode-to-vertex max=%.4f (r=%.4f) at (%.4f,%.4f)\n", maxTip, maxTipR, maxTipP.x, maxTipP.y);
        bool ok = !ax.accuracyCapped && maxRadErr <= chordal && maxDefect <= chordal && maxInterp <= chordal && outside == 0 && nonfinite == 0 && tagSideWrong == 0 && tagBad == 0 && topo == 0 && worstTagDist <= chordal;
        if (!ok) ++fails;
        std::printf("%-18s %s t=%.3fs smp=%zu nodes=%zu br=%zu seg=%d capped=%d ign=%zu | radErr=%.2e defect=%.2e@(%.4f,%.4f r=%.4f) interp=%.2e out=%d nf=%d topo=%d | tags: bad=%d side=%d eqErr=%.2e | cov: max=%.4f@(%.3f,%.3f d=%.4f) >c:%d >3c:%d of %d | nearMed=%d maxMiss=%.4f@(%.3f,%.3f r=%.3f)\n",
            sh.name.c_str(), ok ? "OK  " : "FAIL", secs, ax.boundarySamples, ax.nodes.size(), ax.branches.size(), nseg, (int)ax.accuracyCapped, ax.ignoredContours,
            maxRadErr, maxDefect, worstDefP.x, worstDefP.y, worstDefR, maxInterp, outside, nonfinite, topo, tagBad, tagSideWrong, worstTagDist,
            maxCov, maxCovP.x, maxCovP.y, maxCovDepth, nCov1, nCov3, nGrid, nNear, maxMiss, maxMissP.x, maxMissP.y, maxMissR);
        std::fflush(stdout);
    }
    std::printf("fails=%d\n", fails);
    return fails ? 1 : 0;
}
