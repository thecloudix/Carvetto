// Round-3 caps: values just inside / just outside each work cap, all strategies, rest tools.
// Each case runs in a forked child with a memory limit and a wall-clock timeout.
#include "/tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0121v2/common.hpp"

static void limitMem(size_t mb) { struct rlimit rl{mb << 20, mb << 20}; setrlimit(RLIMIT_AS, &rl); }

struct Case { std::string nm; std::function<PocketResult()> fn; bool expectRefuse; };

static PocketParams P(double cut = 2) { PocketParams p; p.cutDepth = cut; p.safeZ = 5; return p; }
static PocketParams with(PocketParams p, std::function<void(PocketParams&)> f) { f(p); return p; }
static std::vector<Contour> rectC(double w, double h) { return {Contour::rectangle(Box2{{0, 0}, {w, h}})}; }
static std::vector<Contour> polyC(const std::vector<Vec2>& pts) { return {Contour::fromPolyline(geometry::Polyline{pts, true})}; }
static std::vector<Vec2> ngon(int N, double R) { std::vector<Vec2> p; for (int k = 0; k < N; ++k) { double t = 2 * PI * k / N; p.push_back({R * std::cos(t), R * std::sin(t)}); } return p; }
static std::vector<Contour> comb(int teeth, double pitch, double gap, double hgt) {
    std::vector<Vec2> pts;
    for (int i = 0; i < teeth; ++i) { pts.push_back({i * pitch, 0}); pts.push_back({i * pitch, hgt}); pts.push_back({i * pitch + gap, hgt}); pts.push_back({i * pitch + gap, 0.0}); }
    pts.push_back({teeth * pitch, -5}); pts.push_back({0, -5});
    return polyC(pts);
}
// gear: circle with N rectangular notches of width w and depth dd (rest material in every notch)
static std::vector<Contour> notched(int N, double R, double w, double dd) {
    std::vector<Vec2> pts;
    for (int k = 0; k < N; ++k) {
        double t0 = 2 * PI * k / N, t1 = 2 * PI * (k + 1) / N;
        double tm = 0.5 * (t0 + t1);
        double half = w / 2 / R;
        Vec2 u{std::cos(tm), std::sin(tm)}, v{-u.y, u.x};
        pts.push_back({R * std::cos(t0), R * std::sin(t0)});
        Vec2 a = u * (R * std::cos(half)) - v * (w / 2), b = u * (R * std::cos(half)) + v * (w / 2);
        pts.push_back(a); pts.push_back(a + u * dd); pts.push_back(b + u * dd); pts.push_back(b);
    }
    return polyC(pts);
}

int main(int argc, char** argv) {
    std::string only = argc > 1 ? argv[1] : "";
    int timeout = argc > 2 ? atoi(argv[2]) : 240;
    std::vector<Case> cs;
    auto both = [&](const std::string& nm, std::function<PocketResult(PocketStrategy)> f, bool refuse) {
        cs.push_back({nm + " [offset]", [f] { return f(PocketStrategy::Offset); }, refuse});
        cs.push_back({nm + " [raster]", [f] { return f(PocketStrategy::Raster); }, refuse});
    };
    auto st = [](PocketParams p, PocketStrategy s) { p.strategy = s; p.rasterAngle = 0.3; return p; };
    // chordal / distance
    both("chordal 1e-4 dist 1e-5 (in)", [&](PocketStrategy s) { return pocketToolpath({Contour::circle({0, 0}, 20)}, {tool(6, 2.4, 1)}, st(with(P(), [](auto& q) { q.tolerance.chordal = 1e-4; q.tolerance.distance = 1e-5; }), s)); }, false);
    both("chordal 9.99e-5 (out)", [&](PocketStrategy s) { return pocketToolpath({Contour::circle({0, 0}, 20)}, {tool(6, 2.4, 1)}, st(with(P(), [](auto& q) { q.tolerance.chordal = 9.99e-5; q.tolerance.distance = 1e-5; }), s)); }, true);
    both("chordal 1.0 (in)", [&](PocketStrategy s) { return pocketToolpath({Contour::circle({0, 0}, 20)}, {tool(6, 2.4, 1)}, st(with(P(), [](auto& q) { q.tolerance.chordal = 1.0; }), s)); }, false);
    both("chordal 1.0001 (out)", [&](PocketStrategy s) { return pocketToolpath({Contour::circle({0, 0}, 20)}, {tool(6, 2.4, 1)}, st(with(P(), [](auto& q) { q.tolerance.chordal = 1.0001; }), s)); }, true);
    both("distance = chordal (in)", [&](PocketStrategy s) { return pocketToolpath(rectC(50, 30), {tool(6, 2.4, 1)}, st(with(P(), [](auto& q) { q.tolerance.distance = q.tolerance.chordal; }), s)); }, false);
    both("distance > chordal (out)", [&](PocketStrategy s) { return pocketToolpath(rectC(50, 30), {tool(6, 2.4, 1)}, st(with(P(), [](auto& q) { q.tolerance.distance = q.tolerance.chordal * 1.0001; }), s)); }, true);
    both("distance 0 (out)", [&](PocketStrategy s) { return pocketToolpath(rectC(50, 30), {tool(6, 2.4, 1)}, st(with(P(), [](auto& q) { q.tolerance.distance = 0; }), s)); }, true);
    // vertices (lines: 1 each)
    both("199,999-gon (in)", [&](PocketStrategy s) { auto p = ngon(199999, 40); for (size_t k = 0; k < p.size(); ++k) p[k] = p[k] * (1 + 0.004 * std::sin(k * 0.01)); return pocketToolpath(polyC(p), {tool(6, 2.4, 1)}, st(P(1), s)); }, false);
    both("200,001-gon (out)", [&](PocketStrategy s) { return pocketToolpath(polyC(ngon(200001, 40)), {tool(6, 2.4, 1)}, st(P(1), s)); }, true);
    // stepover vs 10*distance (distance 0.001 default)
    both("stepover 0.01 = 10*dist on 5x5 (in)", [&](PocketStrategy s) { return pocketToolpath(rectC(12, 11), {tool(6, 0.01, 1)}, st(P(1), s)); }, false);
    both("stepover 0.00999 (out)", [&](PocketStrategy s) { return pocketToolpath(rectC(12, 11), {tool(6, 0.00999, 1)}, st(P(1), s)); }, true);
    // levels: ceil(cut/pd)+2 <= 1e4
    both("levels est 10000 (in)", [&](PocketStrategy s) { return pocketToolpath(rectC(50, 30), {tool(6, 2.4, 1.0 / 9998 * (1 + 1e-9))}, st(P(1), s)); }, false);
    both("levels est 10001 (out)", [&](PocketStrategy s) { return pocketToolpath(rectC(50, 30), {tool(6, 2.4, 1.0 / 9999 * (1 + 1e-9))}, st(P(1), s)); }, true);
    // rings / lines per part <= 2e4. Offset: 0.5*min(w,h)/so + 2; Raster: hypot(w,h)/so + 2
    cs.push_back({"rings est 20000 square 405.96 so 0.01 [offset] (in)", [&] { return pocketToolpath(rectC(405.96 - 1e-6, 405.96 - 1e-6), {tool(6, 0.01, 1)}, P(1)); }, false});
    cs.push_back({"rings est >20000 square 406 so 0.01 [offset] (out)", [&] { return pocketToolpath(rectC(406.0, 406.0), {tool(6, 0.01, 1)}, P(1)); }, true});
    { double L = 0.02 * 19998 / std::sqrt(2.0) + 6 - 1e-6;
      cs.push_back({"lines est 20000 so 0.02 [raster] (in)", [&, L] { return pocketToolpath(rectC(L, L), {tool(6, 0.02, 1)}, with(P(1), [](auto& q) { q.strategy = PocketStrategy::Raster; })); }, false});
      cs.push_back({"lines est >20000 so 0.02 [raster] (out)", [&, L] { return pocketToolpath(rectC(L + 0.01, L + 0.01), {tool(6, 0.02, 1)}, with(P(1), [](auto& q) { q.strategy = PocketStrategy::Raster; })); }, true}); }
    // move estimate: levels(3) * passEst * vertices <= 2e7, 2000-gon circle radius R
    { // passEst = (R*cos(pi/2000)-3)/so + 2  (box of the polygon), so=0.03
      double so = 0.03; double need = 2e7 / 3.0 / 2000.0;  // passEst max
      double Rin = ((need - 2) * so - 1e-6 + 3) / std::cos(0.0);
      cs.push_back({"move est just under 2e7 (2000-gon) [offset] (in)", [&, so, Rin] { return pocketToolpath(polyC(ngon(2000, Rin)), {tool(6, so, 1)}, P(1)); }, false});
      cs.push_back({"move est just over 2e7 (2000-gon) [offset] (out)", [&, so, Rin] { return pocketToolpath(polyC(ngon(2000, Rin + 0.1)), {tool(6, so, 1)}, P(1)); }, true}); }
    // actual-move cap (raster comb): pattern size P per level from a 1-level run, then L levels
    // just under / over 2e7/P.
    cs.push_back({"actual-cap probe comb200 raster (1 level)", [&] { return pocketToolpath(comb(200, 1, 0.5, 20), {tool(0.2, 0.02, 9)}, with(P(9), [](auto& q) { q.strategy = PocketStrategy::Raster; q.rasterAngle = 0; })); }, false});
    // rest-tool performance attacks
    cs.push_back({"rest: notched circle 300 notches, T6 then T0.5 so 0.05 [offset]", [&] { return pocketToolpath(notched(300, 50, 3, 3), {tool(6, 2.4, 2, "T6"), tool(0.5, 0.05, 2, "T05")}, P(2)); }, false});
    cs.push_back({"rest: notched circle 300 notches, T6 then T0.5 so 0.02 [offset]", [&] { return pocketToolpath(notched(300, 50, 3, 3), {tool(6, 2.4, 2, "T6"), tool(0.5, 0.02, 2, "T05")}, P(2)); }, false});
    cs.push_back({"rest: notched circle 300 notches, T6 then T0.5 so 0.05 [raster]", [&] { return pocketToolpath(notched(300, 50, 3, 3), {tool(6, 2.4, 2, "T6"), tool(0.5, 0.05, 2, "T05")}, with(P(2), [](auto& q) { q.strategy = PocketStrategy::Raster; q.rasterAngle = 0.2; })); }, false});
    cs.push_back({"rest: notched circle 1000 notches R150, T6 then T0.5 so 0.05 [offset]", [&] { return pocketToolpath(notched(1000, 150, 3, 3), {tool(6, 2.4, 2, "T6"), tool(0.5, 0.05, 2, "T05")}, P(2)); }, false});
    cs.push_back({"raster comb 1000 teeth pitch 0.4 tool 0.1 so 0.03 (many pieces x rows)", [&] { return pocketToolpath(comb(1000, 0.4, 0.2, 400), {tool(0.1, 0.03, 9)}, with(P(9), [](auto& q) { q.strategy = PocketStrategy::Raster; q.rasterAngle = 0; })); }, false});
    cs.push_back({"rest raster comb 300 teeth: T12 then T0.2 so 0.02", [&] { return pocketToolpath(comb(300, 1, 0.5, 60), {tool(12, 4.8, 2, "T12"), tool(0.2, 0.02, 0.5, "T02")}, with(P(4), [](auto& q) { q.strategy = PocketStrategy::Raster; q.rasterAngle = 0; })); }, false});
    cs.push_back({"offset grid 30x30 circle islands, tool 0.5 so 0.05", [&] { std::vector<Contour> c = rectC(310, 310); for (int a = 0; a < 30; ++a) for (int b = 0; b < 30; ++b) c.push_back(Contour::circle({10.0 + a * 10, 10.0 + b * 10}, 4)); return pocketToolpath(c, {tool(0.5, 0.05, 1)}, P(1)); }, false});

    int fails = 0;
    for (auto& c : cs) {
        if (!only.empty() && c.nm.find(only) == std::string::npos) continue;
        auto t0 = std::chrono::steady_clock::now();
        int rc = forked([&] {
            limitMem(4000);
            struct rusage ru0; getrusage(RUSAGE_SELF, &ru0);
            try {
                auto r = c.fn();
                size_t m = 0; for (auto& t : r.toolpaths) m += t.moves.size();
                bool refused = m == 0;
                size_t pts = 0; for (auto& pl : r.plans) for (auto& pa : pl.paths) pts += pa.path.points.size() + 4;
                struct rusage ru; getrusage(RUSAGE_SELF, &ru);
                printf("    moves=%zu levels=%zu planPts+4=%zu paths=%zu cpu=%.1fs maxrss=%ldMB msgs:", m, r.plans.empty() ? 0 : r.plans[0].levels.size(), pts,
                       r.plans.empty() ? 0 : r.plans.back().paths.size(), ru.ru_utime.tv_sec + ru.ru_utime.tv_usec * 1e-6, ru.ru_maxrss / 1024);
                for (auto& s : r.messages) printf(" [%s]", s.c_str());
                printf("\n");
                if (refused != c.expectRefuse) { printf("    UNEXPECTED: %s\n", refused ? "refused" : "ran"); return 4; }
            } catch (std::exception& e) { printf("    EXCEPTION %s\n", e.what()); return 3; }
            return 0;
        }, timeout);
        double el = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        printf("  %s: rc=%d %.2fs%s\n", c.nm.c_str(), rc, el, rc ? "  <-- FAIL (crash/timeout/exception/unexpected)" : "");
        fflush(stdout);
        if (rc) ++fails;
    }
    printf("caps3 fails %d\n", fails);
    return fails;
}
