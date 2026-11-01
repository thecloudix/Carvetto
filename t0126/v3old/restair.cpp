#define V3_NO_MAIN
#include "v3.cpp"
// Exact re-check of REST-AIR flags: a rest-tool cutting sample q is "air" when no point p within
// r_t of q (0.01 lattice) is pocket material (sd >= allow + 1e-3) left by the earlier tools at final
// depth (exact distance to their final-level cuts > r_u + 1e-4). Grid prefilter at 0.1 as in the sweep.
int main(int argc, char** argv) {
    int bad = 0;
    for (int ai = 1; ai < argc; ++ai) {
        bool r2 = argv[ai][0] == 'r'; uint64_t seed = strtoull(argv[ai] + (r2 ? 1 : 0), 0, 10);
        Scene3 s3; Config cf; int att;
        if (r2) { Rng R(seed); double D = R.u(1, 10); Scene2 s2 = genScene2(R, D); cf = genConfig2(R, s2, D); s3.sc = s2.sc; }
        else makeScene3(seed, s3, cf, att);
        const Scene& sc = s3.sc;
        auto r = pocketToolpath(sc.contours, cf.tools, cf.p);
        double bottom = -(cf.p.startDepth + cf.p.cutDepth);
        double rmax = 0; for (auto& t : cf.tools) rmax = std::max(rmax, t.tool.diameter / 2);
        HMap g = makeHMap(sc, 0.1, rmax + 1, cf.p.startDepth); SimStats st;
        std::vector<std::vector<std::pair<Vec2, Vec2>>> finals(cf.tools.size());
        for (size_t t = 0; t < r.toolpaths.size(); ++t) { Vec3 pos{}; const auto& mv = r.toolpaths[t].moves; for (size_t i = 0; i < mv.size(); ++i) { Vec3 f = i ? pos : mv[i].to; if (mv[i].kind != MoveKind::Rapid && f.z <= bottom + 1e-9 && mv[i].to.z <= bottom + 1e-9) finals[t].push_back({f.xy(), mv[i].to.xy()}); pos = mv[i].to; } }
        long flagged = 0, air = 0; double worst = 0; Vec2 wq{}; size_t wt = 0;
        bool earlier = false;
        for (size_t t = 0; t < r.toolpaths.size(); ++t) {
            const auto& tp = r.toolpaths[t]; double rt = cf.tools[t].tool.diameter / 2;
            if (!tp.moves.empty() && earlier) {
                Grid gg; gg.x0 = g.x0; gg.y0 = g.y0; gg.h = g.h; gg.nx = g.nx; gg.ny = g.ny;
                std::vector<uint8_t> M(g.H.size()); for (size_t k = 0; k < g.H.size(); ++k) M[k] = g.sd[k] >= cf.p.allowance + 1e-3 && g.H[k] > bottom + 1e-6;
                auto dm = edt(gg, M);
                std::vector<int> role; std::string err; classify(tp, r.plans[t], role, err);
                Vec3 pos{};
                for (size_t mi = 0; mi < tp.moves.size() && air < 50; ++mi) {
                    const Move& m = tp.moves[mi]; Vec3 from = mi ? pos : m.to; pos = m.to; if (role[mi] != 1) continue;
                    Vec2 a = from.xy(), b = m.to.xy(); int n = std::max(1, int(std::ceil(std::hypot(b.x - a.x, b.y - a.y) / 0.05)));
                    for (int k = 0; k <= n; ++k) {
                        Vec2 q = a + (b - a) * (double(k) / n);
                        int i = int((q.x - g.x0) / g.h), j = int((q.y - g.y0) / g.h);
                        if (i < 0 || j < 0 || i >= g.nx || j >= g.ny) continue;
                        if (dm[size_t(j) * g.nx + i] <= rt + 0.3) continue;
                        ++flagged;
                        bool found = false; double R = rt + 0.0;
                        for (double dx = -R; dx <= R && !found; dx += 0.01) for (double dy = -R; dy <= R && !found; dy += 0.01) {
                            if (dx * dx + dy * dy > R * R) continue; Vec2 p{q.x + dx, q.y + dy};
                            if (sc.sd(p) < cf.p.allowance + 1e-3) continue;
                            bool cut = false; for (size_t u = 0; u < t && !cut; ++u) for (auto& [s0, s1] : finals[u]) if (segDist(p, s0, s1) <= cf.tools[u].tool.diameter / 2 + 1e-4) { cut = true; break; }
                            if (!cut) found = true;
                        }
                        if (!found) { ++air; if (dm[size_t(j) * g.nx + i] > worst) { worst = dm[size_t(j) * g.nx + i]; wq = q; wt = t; } }
                        break;  // one sample per move
                    }
                }
            }
            simulate(g, tp, r.plans[t], rt, cf.p.allowance, int(t), st, 0.02);
            if (!tp.moves.empty()) earlier = true;
        }
        printf("seed %llu [%s] strat=%d pp=%d: grid-flagged %ld, exact air %ld%s", (unsigned long long)seed, sc.desc.c_str(), int(cf.p.strategy), int(cf.p.profilePass), flagged, air, air ? "" : "\n");
        if (air) printf(" worst: tool %zu at (%.3f,%.3f) grid-dist %.3f\n", wt, wq.x, wq.y, worst);
        if (air) ++bad;
        fflush(stdout);
    }
    return bad;
}
