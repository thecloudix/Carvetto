#define V3_NO_MAIN
#include "v3.cpp"
int main(int argc, char** argv) {
    double gap = argc > 1 ? atof(argv[1]) : 5.5; double qx = argc > 2 ? atof(argv[2]) : 17.45, qy = argc > 3 ? atof(argv[3]) : 5.45;
    Scene s; int nx = 5, ny = 4; double iw = 4, W = nx * iw + (nx + 1) * gap, H = ny * iw + (ny + 1) * gap;
    s.addPoly(rectPts({0, 0}, {W, H}));
    for (int a = 0; a < nx; ++a) for (int b = 0; b < ny; ++b) { double x = gap + a * (iw + gap), y = gap + b * (iw + gap); s.addCircle({x + 2, y + 2}, 2); }
    PocketParams p; p.cutDepth = 5; p.safeZ = 5; p.strategy = PocketStrategy::Raster; p.profilePass = ProfilePass::None; p.rasterAngle = 0.7; p.rampDistance = 6;
    std::vector<PocketTool> ts{tool(12, 4.8, 2.5, "T12"), tool(3, 1.2, 2, "T3"), tool(1, 0.4, 1.7, "T1")};
    auto r = pocketToolpath(s.contours, ts, p);
    Vec2 q{qx, qy};
    printf("point sd %.4f\n", s.sd(q));
    for (size_t u = 1; u < 3; ++u) { double best = 1e9, bestPlan = 1e9; Vec3 pos{}; const auto& mv = r.toolpaths[u].moves; for (size_t i = 0; i < mv.size(); ++i) { Vec3 f = i ? pos : mv[i].to; if (mv[i].kind != MoveKind::Rapid && f.z <= -5 + 1e-9 && mv[i].to.z <= -5 + 1e-9) best = std::min(best, segDist(q, f.xy(), mv[i].to.xy())); pos = mv[i].to; }
        for (auto& pa : r.plans[u].paths) for (size_t k = 0; k + 1 < pa.path.points.size() + (pa.path.closed ? 1 : 0); ++k) bestPlan = std::min(bestPlan, segDist(q, pa.path.points[k], pa.path.points[(k + 1) % pa.path.points.size()]));
        printf("tool %zu r=%.2f: nearest final cut %.4f (excess %.4f), nearest plan path %.4f\n", u, ts[u].tool.diameter / 2, best, best - ts[u].tool.diameter / 2, bestPlan); }
    // rest material for T1 near q
    double dr = 1e9; bool inRest = false;
    for (auto& c : r.plans[2].restMaterial) { auto pl = c.polygonise(geometry::Tolerance::defaults()); for (size_t k = 0; k < pl.points.size(); ++k) dr = std::min(dr, segDist(q, pl.points[k], pl.points[(k + 1) % pl.points.size()])); }
    { Scene rs; for (auto& c : r.plans[2].restMaterial) { auto pl = c.polygonise(geometry::Tolerance::defaults()); rs.addPoly(pl.points); } inRest = rs.inside(q); }
    printf("T1 rest material: point inside=%d, distance to its boundary %.4f, rest regions %zu\n", int(inRest), dr, r.plans[2].restMaterial.size());
    // T1 centre region distance
    Scene cs; for (auto& c : r.plans[2].centreRegion) { auto pl = c.polygonise(geometry::Tolerance::defaults()); cs.addPoly(pl.points); }
    // nearest point of T1 centre region to q
    double best = 1e9; Vec2 bp{}; for (double x = q.x - 1; x < q.x + 1; x += 0.002) for (double y = q.y - 1; y < q.y + 1; y += 0.002) if (cs.inside({x, y})) { double d = std::hypot(x - q.x, y - q.y); if (d < best) { best = d; bp = {x, y}; } }
    printf("T1 centre region nearest point to q: %.4f at (%.3f,%.3f)\n", best, bp.x, bp.y);
}
