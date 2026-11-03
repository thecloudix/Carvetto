#include <cstdio>
#include <cstdlib>
#include <string>
#include <map>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static tooling::Tool em(double d) { tooling::Tool t; t.type = tooling::ToolType::EndMill; t.diameter = d; return t; }
static Contour beads(int n, double y0) {
    Contour c; c.closed = true; std::vector<Vec2> pts;
    for (int i = 0; i < n; ++i) { double x = 4.0 * i; pts.push_back({x, y0}); pts.push_back({x + 3.0, y0}); if (i + 1 < n) { pts.push_back({x + 3.0, y0 + 1.25}); pts.push_back({x + 4.0, y0 + 1.25}); } }
    for (int i = n - 1; i >= 0; --i) { double x = 4.0 * i; pts.push_back({x + 3.0, y0 + 3.0}); pts.push_back({x, y0 + 3.0}); if (i > 0) { pts.push_back({x, y0 + 1.75}); pts.push_back({x - 1.0, y0 + 1.75}); } }
    for (size_t k = 0; k < pts.size(); ++k) c.append(geometry::LineSpan{pts[k], pts[(k + 1) % pts.size()]});
    return c;
}
int main(int argc, char** argv) {
    std::vector<Contour> cs;
    for (int a = 1; a < argc; ++a) cs.push_back(beads(atoi(argv[a]), 10.0 * (a - 1)));
    ProfileParams p; p.side = ProfileSide::On; p.passes.cutDepth = 3; p.passes.passDepth = 3; p.tabs.enabled = true; p.tabs.count = 0; p.tabs.thickness = 1;
    auto r = profileToolpath(cs, em(1.0), p);
    std::map<int,int> listed; int sum = 0;
    for (auto& d : r.diagnostics) { if (d.message.find("comes free") != std::string::npos) listed[d.contour ? (int)*d.contour : -1]++; if (d.message.find("more loose") != std::string::npos) { ++sum; std::printf("%s\n", d.message.c_str()); } }
    for (auto& [k, v] : listed) std::printf("vector %d listed %d\n", k, v);
    std::printf("summaries %d total diags %zu\n", sum, r.diagnostics.size());
}
