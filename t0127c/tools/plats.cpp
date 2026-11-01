// plats scene : prints the raised runs at the tab top (consecutive non-rapid moves at zTab): mid point (by path length) and length
#include <cmath>
#include <cstdio>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main(int argc, char** argv) {
    std::vector<geometry::Contour> cs; ProfileParams p; double D; int vd = 0;
    readScene(argv[1], cs, p, D, vd);
    tooling::Tool t; t.id = "t"; t.name = "t"; t.type = vd ? tooling::ToolType::VBit : tooling::ToolType::EndMill; t.diameter = D;
    if (vd) t.includedAngle = vd * 3.14159265358979323846 / 180;
    t.defaultCuttingData.passDepth = 4; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.spindleSpeed = 18000; t.defaultCuttingData.toolNumber = 1;
    auto res = profileToolpath(cs, t, p);
    const double zTab = p.passes.bottomZ() + std::min(p.tabs.thickness, p.passes.cutDepth);
    const auto& mv = res.toolpath.moves;
    std::vector<Vec2> run; double len = 0;
    auto flush = [&]() { if (run.size() > 1) { double h = len / 2, acc = 0; Vec2 m = run.back(); for (size_t i = 1; i < run.size(); ++i) { double l = std::hypot(run[i].x - run[i-1].x, run[i].y - run[i-1].y); if (acc + l >= h) { double f = l > 0 ? (h - acc) / l : 0; m = {run[i-1].x + (run[i].x - run[i-1].x) * f, run[i-1].y + (run[i].y - run[i-1].y) * f}; break; } acc += l; } std::printf("run %.4f %.4f len %.4f\n", m.x, m.y, len); } run.clear(); len = 0; };
    for (size_t i = 1; i < mv.size(); ++i) {
        bool on = mv[i].kind != MoveKind::Rapid && std::fabs(mv[i].to.z - zTab) < 1e-9 && std::fabs(mv[i - 1].to.z - zTab) < 1e-9;
        if (on) { if (run.empty()) run.push_back(mv[i - 1].to.xy()); run.push_back(mv[i].to.xy()); len += std::hypot(mv[i].to.x - mv[i-1].to.x, mv[i].to.y - mv[i-1].to.y); }
        else flush();
    }
    flush();
    for (auto& d : res.diagnostics) std::printf("diag %s\n", d.message.c_str());
}
