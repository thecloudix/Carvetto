// T-0123 verifier (throwaway): CPU-time scaling for profile/pocket on text-like pages and the comb.
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#ifndef NOPOCKET
#include "carvetto/toolpath/pocket.hpp"
#endif
#ifndef NOPROFILE
#include "carvetto/toolpath/profile.hpp"
#endif
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2; using geometry::Box2; using geometry::Polyline;
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
static std::mt19937_64 rng(77);
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
// variant 0: grid like the branch test; 1: jittered random letters/shapes at random sizes, pitch varies
static std::vector<Contour> page(size_t n, int variant, double pitch) {
    std::vector<Contour> cs; size_t cols = size_t(std::ceil(std::sqrt(double(n))));
    for (size_t k = 0; cs.size() < n; ++k) {
        double x = double(k % cols) * pitch, y = double(k / cols) * pitch;
        double s = variant ? U(2.0, 4.5) : 4.0; if (variant) { x += U(-0.5, 0.5); y += U(-0.5, 0.5); }
        switch (variant ? int(U(0, 6)) : int(k % 4)) {
        case 0: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); break;
        case 1: cs.push_back(Contour::fromPolyline(Polyline{{{x, y}, {x + s, y}, {x + s, y + s * 0.375}, {x + s * 0.375, y + s * 0.375}, {x + s * 0.375, y + s}, {x, y + s}}, true})); break;
        case 2: cs.push_back(Contour::circle({x + s / 2, y + s / 2}, s / 2)); break;
        case 3: cs.push_back(Contour::rectangle(Box2{{x, y}, {x + s, y + s}})); if (cs.size() < n) cs.push_back(Contour::rectangle(Box2{{x + 0.3 * s, y + 0.3 * s}, {x + 0.7 * s, y + 0.7 * s}})); break;
        case 4: { std::vector<Vec2> p; int m = 5 + int(U(0, 8)); for (int i = 0; i < m; ++i) { double t = 6.283185307 * i / m, r = (i % 2 ? 0.25 : 0.5) * s; p.push_back({x + s / 2 + r * std::cos(t), y + s / 2 + r * std::sin(t)}); } cs.push_back(Contour::fromPolyline(Polyline{p, true})); break; }
        default: cs.push_back(Contour::fromPolyline(Polyline{{{x, y}, {x + s, y + s}, {x + s, y}}, false})); break;  // open stroke
        }
    }
    return cs;
}
#ifndef NOPOCKET
static PocketTool ptool(const char* nm, int num, double d, double so, double pd) {
    PocketTool t;
#ifdef NEWAPI
    t.tool.id = nm; t.tool.name = nm; t.tool.type = tooling::ToolType::EndMill; t.tool.diameter = d; auto& cd = t.tool.defaultCuttingData; cd.toolNumber = num; cd.feedRate = 1000; cd.plungeRate = 300; cd.stepover = so; cd.passDepth = pd;
#else
    t.tool.name = nm; t.tool.toolNumber = num; t.tool.diameter = d; t.tool.feedRate = 1000; t.tool.plungeRate = 300; t.stepover = so; t.passDepth = pd;
#endif
    return t;
}
#endif
int main(int argc, char** argv) {
    std::string what = argv[1]; size_t n = argc > 2 ? strtoul(argv[2], 0, 10) : 0; int variant = argc > 3 ? atoi(argv[3]) : 0; double pitch = argc > 4 ? atof(argv[4]) : 6.0;
    if (what == "profile" || what == "pocket") {
        auto cs = page(n, variant, pitch);
        double t0 = cpu(); size_t moves = 0, diags = 0, retr = 0;
        if (what == "profile") {
#ifndef NOPROFILE
 tooling::Tool t; t.name = "EM 1"; t.diameter = 1.0; t.defaultCuttingData.passDepth = 3.0; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300;
            ProfileParams p; p.passes.cutDepth = 3.0; if (getenv("SIDE")) p.side = (ProfileSide)atoi(getenv("SIDE")); if (getenv("TABS")) p.tabs.enabled = true; if (getenv("LEADS")) p.leads.type = LeadType::Straight; if (getenv("SEP")) p.separateLastPass = true; if (getenv("ORDER")) p.order.leftToRight = true;
            auto r = profileToolpath(cs, t, p); moves = r.toolpath.moves.size(); diags = r.diagnostics.size(); retr = r.toolpath.count(MoveKind::Retract);
#endif
 }
        else {
#ifndef NOPOCKET
 PocketParams p; p.cutDepth = 3.0; if (getenv("RASTER")) p.strategy = PocketStrategy::Raster; std::vector<PocketTool> tools{ptool("EM 1", 1, 1.0, 0.4, 3.0)}; if (getenv("REST")) tools.push_back(ptool("EM .3", 2, 0.3, 0.12, 3.0));
            auto r = pocketToolpath(cs, tools, p); for (auto& tp : r.toolpaths) { moves += tp.moves.size(); retr += tp.count(MoveKind::Retract); }
#ifdef NEWAPI
            diags = r.diagnostics.size();
#else
            diags = r.messages.size();
#endif
#endif
        }
        std::printf("%s n=%zu var=%d pitch=%g cpu %.3f moves %zu diags %zu retracts %zu\n", what.c_str(), n, variant, pitch, cpu() - t0, moves, diags, retr);
    } else if (what == "comb") {
#ifndef NOPOCKET

        int teeth = n ? int(n) : 200; std::vector<Vec2> pts;
        for (int i = 0; i < teeth; ++i) { pts.push_back({i * 1.0, 0}); pts.push_back({i * 1.0, 20}); pts.push_back({i + 0.5, 20}); pts.push_back({i + 0.5, 0.0}); }
        pts.push_back({double(teeth), -5}); pts.push_back({0, -5});
        Contour comb = Contour::fromPolyline(Polyline{pts, true});
        PocketParams p; p.cutDepth = 9.0; p.strategy = variant ? PocketStrategy::Offset : PocketStrategy::Raster;
        double t0 = cpu(); auto r = pocketToolpath({comb}, {ptool("B", 1, 1.0, 0.4, 9.0), ptool("S", 2, 0.1, 0.04, 9.0)}, p);
        std::printf("comb teeth=%d strat=%s cpu %.3f moves1 %zu retr1 %zu\n", teeth, variant ? "offset" : "raster", cpu() - t0, r.toolpaths[1].moves.size(), r.toolpaths[1].count(MoveKind::Retract));
#endif
    }
}
