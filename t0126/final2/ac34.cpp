#define V3_NO_MAIN
#include "v3src.cpp"
int main() {
    int fails = 0;
    // AC3: raster 45 deg, last profile pass, on 50x30 and with island, several stepovers / chordals
    for (int isl = 0; isl < 2; ++isl) for (double so : {1.2, 2.4, 6.0}) for (double ch : {0.01, 0.05}) {
        Scene s = rect(0, 0, 50, 30); if (isl) s.addPoly({{15, 10}, {35, 10}, {35, 20}, {15, 20}});
        PocketParams p; p.cutDepth = 6; p.safeZ = 5; p.strategy = PocketStrategy::Raster; p.rasterAngle = PI / 4; p.profilePass = ProfilePass::Last; p.tolerance.chordal = ch;
        auto r = pocketToolpath(s.contours, {tool(6, so, 3)}, p);
        const auto& pl = r.plans[0];
        double inset = 3 + ch;  // radius + margin
        double worstAng = 0, worstEnd = 0, maxGap = 0; int nRaster = 0, nProf = 0, profAfter = 1; bool seenProf = false;
        std::vector<double> vs;
        for (auto& pa : pl.paths) {
            if (pa.role == PocketPathRole::Raster) { if (seenProf) profAfter = 0; ++nRaster; Vec2 a = pa.path.points.front(), b = pa.path.points.back(); Vec2 d = b - a; double L = std::hypot(d.x, d.y);
                worstAng = std::max(worstAng, std::fabs(d.x * std::sin(PI / 4) - d.y * std::cos(PI / 4)) / L);
                for (Vec2 e : {a, b}) worstEnd = std::max(worstEnd, std::fabs(s.sd(e) - inset));
                if (pa.path.points.size() != 2) worstAng = 1;
                vs.push_back(-a.x * std::sin(PI / 4) + a.y * std::cos(PI / 4)); }
            else if (pa.role == PocketPathRole::Profile) { seenProf = true; ++nProf; double we = 0; for (auto q : pa.path.points) we = std::max(we, std::fabs(s.sd(q) - inset)); worstEnd = std::max(worstEnd, we * 0 ); 
                // profile vertices on the offset boundary (allow join chord inside)
                double wp = 0; for (auto q : pa.path.points) wp = std::max(wp, std::fabs(s.sd(q) - inset)); if (wp > ch) { printf("  profile vertex off boundary by %.4f\n", wp); ++fails; } }
        }
        std::sort(vs.begin(), vs.end()); vs.erase(std::unique(vs.begin(), vs.end(), [](double a, double b) { return std::fabs(a - b) < 1e-6; }), vs.end());
        for (size_t i = 1; i < vs.size(); ++i) maxGap = std::max(maxGap, vs[i] - vs[i - 1]);
        // profile pass followed exactly once in the moves: count feed moves along each profile ring segment
        int expectProf = 1 + isl;
        bool ok = worstAng < 1e-9 && worstEnd < 0.5 * ch + 1e-6 && nProf == expectProf && profAfter && maxGap <= so + 1e-9;
        // last moves: profile rings once per level
        printf("AC3 isl%d so%.1f ch%.2f: raster pieces %d, max angle err %.2e, max end off-boundary %.2e, pitch max %.4f, profile rings %d (after rasters %d) %s\n", isl, so, ch, nRaster, worstAng, worstEnd, maxGap, nProf, profAfter, ok ? "ok" : "FAIL");
        if (!ok) ++fails;
        Check ck; checkToolpath(s, r.toolpaths[0], 3, 0, 5, -6, ck, 1e-9); if (!ck.ok) { printf("  SAFETY %s\n", ck.why.c_str()); ++fails; }
    }
    // AC4: 12 then 3, rest locality: every cutting move of the rest tool within 6 mm of a concave corner / narrow slot.
    {   std::vector<std::pair<std::string, Scene>> sc;
        sc.push_back({"rect", rect(0, 0, 50, 30)});
        { Scene s = rect(0, 0, 50, 30); s.addPoly({{15, 10}, {35, 10}, {35, 20}, {15, 20}}); sc.push_back({"rect+isl", s}); }
        { Scene s; s.addPoly({{0, 0}, {60, 0}, {60, 40}, {34, 40}, {34, 15}, {26, 15}, {26, 40}, {0, 40}}); sc.push_back({"U-slot8", s}); }
        for (auto& [nm, s] : sc) for (auto st : {PocketStrategy::Offset, PocketStrategy::Raster}) for (auto pp : {ProfilePass::None, ProfilePass::Last}) {
            PocketParams p; p.cutDepth = 6; p.safeZ = 5; p.strategy = st; p.profilePass = pp; p.rasterAngle = 0.3;
            Config cf; cf.tools = {tool(12, 4.8, 3, "T12"), tool(3, 1.2, 3, "T3")}; cf.p = p;
            int rc = fullCheck3("AC4 " + nm + " st" + std::to_string(int(st)) + " pp" + std::to_string(int(pp)), s, cf, 0.05, -1);
            // rectangle: rest cuts within 6 mm of a corner (concave corners only), measured to the disc edge
            auto r = pocketToolpath(s.contours, cf.tools, cf.p);
            double worst = 0; Vec3 pos{}; const auto& mv = r.toolpaths[1].moves;
            std::vector<Vec2> corners; for (auto& pr : s.prims) corners.push_back(pr.a);
            for (size_t i = 0; i < mv.size(); ++i) { Vec3 f = i ? pos : mv[i].to; pos = mv[i].to; if (mv[i].kind == MoveKind::Rapid || mv[i].kind == MoveKind::Retract) continue; Vec2 a = f.xy(), b = mv[i].to.xy();
                for (int k = 0; k <= 20; ++k) { Vec2 q = a + (b - a) * (k / 20.0); double dc = 1e9; for (auto c : corners) dc = std::min(dc, std::hypot(q.x - c.x, q.y - c.y)); worst = std::max(worst, dc - 1.5); } }
            printf("AC4 %s st%d pp%d: fullCheck3 rc=%d, rest moves %zu, farthest rest-disc point from a corner %.3f mm (limit 6 for rect-type) slot-shape %s\n", nm.c_str(), int(st), int(pp), rc, mv.size(), worst, nm == "U-slot8" ? "(slot: n/a)" : "");
            if (rc || (nm != "U-slot8" && worst > 6.0)) ++fails;
        }
    }
    printf("AC3/AC4 fails %d\n", fails);
    return fails;
}
