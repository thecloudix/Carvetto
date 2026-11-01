#define V3_NO_MAIN
#include "v3src.cpp"
int main() {
    int fails = 0;
    PocketParams p; p.cutDepth = 6; p.safeZ = 5;
    // AC1
    { Scene s = rect(0, 0, 50, 30);
      for (auto st : {PocketStrategy::Offset, PocketStrategy::Raster}) for (auto pp : {ProfilePass::None, ProfilePass::First, ProfilePass::Last}) {
        PocketParams q = p; q.strategy = st; q.profilePass = pp; q.rasterAngle = 0.0;
        if (st == PocketStrategy::Raster && pp == ProfilePass::None) continue;
        char lab[64]; snprintf(lab, sizeof lab, "AC1 st%d pp%d", int(st), int(pp));
        fails += acCoverage(s, {tool(6, 2.4, 3)}, q, lab); }
      // exact: every cut point >= 3 from walls, every point at depth >= r from wall within reach (sample 0.05 grid)
      auto r = pocketToolpath(s.contours, {tool(6, 2.4, 3)}, p);
      double worst = 0; Vec3 pos{}; const auto& mv = r.toolpaths[0].moves;
      long miss = 0, tot = 0;
      for (double x = 0.025; x < 50; x += 0.05) for (double y = 0.025; y < 30; y += 0.05) {
        // reachable: within 3 of a centre point in [3,47]x[3,27]
        double cx = std::clamp(x, 3.0, 47.0), cy = std::clamp(y, 3.0, 27.0);
        if (std::hypot(x - cx, y - cy) > 3 - 0.02) continue;  // corner not reachable (minus wall skin)
        ++tot; double best = 1e9; pos = {};
        for (size_t i = 0; i < mv.size(); ++i) { Vec3 f = i ? pos : mv[i].to; if (mv[i].kind != MoveKind::Rapid && f.z <= -6 + 1e-9 && mv[i].to.z <= -6 + 1e-9) best = std::min(best, segDist({x, y}, f.xy(), mv[i].to.xy())); pos = mv[i].to; if (best <= 3) break; }
        if (best > 3 + 1e-9) ++miss; worst = std::max(worst, best - 3);
      }
      printf("AC1 exact 0.05 grid (reach minus 0.02 skin): %ld/%ld missed, worst excess %.4f\n", miss, tot, worst); if (miss) ++fails; }
    // AC2
    { Scene s = rect(0, 0, 50, 30); s.addPoly({{15, 10}, {35, 10}, {35, 20}, {15, 20}});
      for (auto st : {PocketStrategy::Offset, PocketStrategy::Raster}) for (auto pp : {ProfilePass::None, ProfilePass::First, ProfilePass::Last}) for (double so : {1.2, 2.4, 6.0}) {
        PocketParams q = p; q.strategy = st; q.profilePass = pp; q.rasterAngle = 0.5;
        auto r = pocketToolpath(s.contours, {tool(6, so, 3)}, q);
        Check ck; checkToolpath(s, r.toolpaths[0], 3, 0, 5, -6, ck, 1e-9);
        // distance to island exact
        double dmin = 1e9; Vec3 pos{}; for (size_t i = 0; i < r.toolpaths[0].moves.size(); ++i) { auto& m = r.toolpaths[0].moves[i]; Vec3 f = i ? pos : m.to; if (m.kind != MoveKind::Rapid) { Vec2 a = f.xy(), b = m.to.xy(); for (int k = 0; k <= 200; ++k) { Vec2 x = a + (b - a) * (k / 200.0); double dx = std::max({15 - x.x, 0.0, x.x - 35}), dy = std::max({10 - x.y, 0.0, x.y - 20}); dmin = std::min(dmin, std::hypot(dx, dy)); } } pos = m.to; }
        if (!ck.ok || dmin < 3) { ++fails; printf("AC2 FAIL st%d pp%d so%.1f: %s island dist %.6f\n", int(st), int(pp), so, ck.why.c_str(), dmin); }
        else printf("AC2 st%d pp%d so%.1f: island min dist %.6f, wall margin %.6f, retracts %zu\n", int(st), int(pp), so, dmin, ck.minMargin, r.toolpaths[0].count(MoveKind::Retract)); } }
    return fails;
}
