#define V3_NO_MAIN
#include "v3.cpp"
static long exactAir(const Scene& sc, const Config& cf, uint64_t seed) {
    int bad=0; (void)bad;

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
    return air;
}

static Config mk3(std::vector<PocketTool> ts, PocketStrategy s, ProfilePass pp, double ang) { Config c; c.tools = ts; c.p.cutDepth = 5; c.p.safeZ = 5; c.p.strategy = s; c.p.profilePass = pp; c.p.rasterAngle = ang; c.p.rampPlunge = true; c.p.rampDistance = 6; return c; }
int main() {
    long tot = 0;
    for (int k : {3, 5, 8}) for (double a : {0.5, 0.8}) { Scene s; std::vector<Vec2> p; int n = k * 30; for (int i = 0; i < n; ++i) { double t = 2 * PI * i / n; double rr = 20 * (1 + a * std::cos(k * t)); p.push_back({rr * std::cos(t), rr * std::sin(t)}); } s.addPoly(p); s.desc = "flower";
        for (auto pp : {ProfilePass::None, ProfilePass::Last}) {
            Config c1 = mk3({tool(12, 4.8, 2.5, "T12"), tool(3, 1.2, 2, "T3")}, PocketStrategy::Offset, pp, 0);
            Config c2 = mk3({tool(12, 4.8, 2.5, "T12"), tool(3, 1.2, 2, "T3"), tool(1, 0.4, 1.7, "T1")}, PocketStrategy::Raster, pp, 0.7);
            c2.p.rampPlunge = false;
            printf("k%d a%.1f pp%d ", k, a, int(pp)); tot += exactAir(s, c1, 0); printf("k%d a%.1f pp%d ", k, a, int(pp)); tot += exactAir(s, c2, 0); } }
    printf("total exact air %ld\n", tot);
}
