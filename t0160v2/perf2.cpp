// t0160v2 perf2: super-linear probes of corner sharpening (Release, CPU s; on vs off in one process, interleaved).
// usage: perf2 <mode> <n...>   modes:
//   star   : one star with n spikes (radius 0.05 n mm, inner 0.8 R): n tents in one flat area; 6 mm end mill, V90 F 1
//   starth : same with stepover 0.04 (r(chordal) > s/2: the per-patch threshold test runs)
//   big    : one n x n mm square, 6 mm end mill (swept area of many polylines), V90 F 1
//   comb   : a 200 mm bar with n teeth (2 x 1.2 mm notches: necks/spikes), 3 mm end mill
//   frame  : n letter-shaped L holes in one frame (one part, many corners), 1 mm end mill, F 0.5
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <string>
#include "shapes.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
int main(int argc, char** argv) {
    std::string mode = argv[1];
    for (int ai = 2; ai < argc; ++ai) {
        int n = atoi(argv[ai]); std::vector<Contour> cs; double em = 6, flat = 1.0, so = 0.5;
        if (mode == "star" || mode == "starth") { double R = std::max(15.0, 0.05 * n * 2); cs.push_back(vs::star({0, 0}, R, R * 0.8, n)); if (mode == "starth") so = 0.04; }
        else if (mode == "big") { cs.push_back(vs::poly({{0, 0}, {double(n), 0}, {double(n), double(n)}, {0, double(n)}})); }
        else if (mode == "comb") { std::vector<Vec2> p; double L = 6.0 * n; p.push_back({0, 0}); for (int k = 0; k < n; ++k) { double x = 6.0 * k + 3; p.push_back({x - 0.6, 0}); p.push_back({x - 0.6, 2}); p.push_back({x + 0.6, 2}); p.push_back({x + 0.6, 0}); } p.push_back({L, 0}); p.push_back({L, 10}); p.push_back({0, 10}); cs.push_back(vs::poly(p)); em = 3; }
        else if (mode == "chain") { // n 10 mm squares in a row joined by 1.5 mm wide, 2 mm long necks (W splits into n parts): one region
            std::vector<Vec2> lo, hi; for (int k = 0; k < n; ++k) { double x = 12.0 * k; lo.push_back({x, 0}); lo.push_back({x + 10, 0}); if (k + 1 < n) { lo.push_back({x + 10, 4.25}); lo.push_back({x + 12, 4.25}); } }
            for (int k = n - 1; k >= 0; --k) { double x = 12.0 * k; if (k + 1 < n) { hi.push_back({x + 12, 5.75}); hi.push_back({x + 10, 5.75}); } hi.push_back({x + 10, 10}); hi.push_back({x, 10}); }
            std::vector<Vec2> p = lo; p.insert(p.end(), hi.begin(), hi.end()); cs.push_back(vs::poly(p)); em = 3; }
        else if (mode == "banner") { for (int k = 0; k < n; ++k) { double x = k * 10.0, y = 0; cs.push_back(vs::rev(vs::poly({{x, y}, {x + 6, y}, {x + 6, y + 2}, {x + 2, y + 2}, {x + 2, y + 6}, {x, y + 6}}))); }
            cs.push_back(vs::poly({{-4, -4}, {n * 10.0, -4}, {n * 10.0, 10}, {-4, 10}})); em = 1; flat = 0.5; }
        else if (mode == "frame") { int side = (int)std::ceil(std::sqrt(n)); for (int k = 0; k < n; ++k) { double x = (k % side) * 10.0, y = (k / side) * 10.0; cs.push_back(vs::rev(vs::poly({{x, y}, {x + 6, y}, {x + 6, y + 2}, {x + 2, y + 2}, {x + 2, y + 6}, {x, y + 6}}))); }
            cs.push_back(vs::poly({{-4, -4}, {side * 10.0, -4}, {side * 10.0, side * 10.0}, {-4, side * 10.0}})); em = 1; flat = 0.5; }
        if (getenv("PERF_ROT")) for (auto& c : cs) c = c.transformed(Transform2::rotation(atof(getenv("PERF_ROT"))));
        tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.diameter = 12.7; t.includedAngle = vo::PI / 2;
        t.defaultCuttingData = {1, so, 0, 0, 18000, 1500, 500, 0, 1}; t.defaultCuttingData.stepover = so; t.defaultCuttingData.passDepth = 3;
        tooling::Tool e; e.id = e.name = "EM"; e.type = tooling::ToolType::EndMill; e.diameter = em; e.defaultCuttingData = {2, 0.4 * em, 0, 0, 18000, 1500, 500, 0, 2}; e.defaultCuttingData.passDepth = 3; e.defaultCuttingData.stepover = 0.4 * em;
        VCarvingParams p; p.flatDepth = flat; p.tolerance.chordal = 0.05;
        double tt[2]; size_t mv[2]; bool ok[2];
        for (int rep = 0; rep < 2; ++rep) for (int on = 0; on < 2; ++on) { p.cornerSharpen = on; double t0 = cpu(); auto r = vCarvingToolpath(cs, t, std::nullopt, {PocketTool{e, std::nullopt}}, p); double dt = cpu() - t0; if (rep == 0 || dt < tt[on]) tt[on] = dt; mv[on] = r.toolpath.moves.size(); ok[on] = r.ok(); }
        std::printf("%s n=%d: off %.3f s (%zu carve moves, ok %d)  on %.3f s (%zu, ok %d)  extra %.3f s (%+.0f%%)\n", mode.c_str(), n, tt[0], mv[0], ok[0], tt[1], mv[1], ok[1], tt[1] - tt[0], 100 * (tt[1] - tt[0]) / tt[0]);
    }
}
