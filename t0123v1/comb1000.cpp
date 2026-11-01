// T-0123 verifier (throwaway): raster piece budget on a big comb; memory/time and message.
#include <cstdio>
#include <ctime>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
int main(int argc, char** argv) {
    int teeth = atoi(argv[1]); double rs = atof(argv[2]);
    std::vector<Vec2> pts; for (int i = 0; i < teeth; ++i) { pts.push_back({i * 1.0, 0}); pts.push_back({i * 1.0, 20}); pts.push_back({i + 0.5, 20}); pts.push_back({i + 0.5, 0.0}); }
    pts.push_back({double(teeth), -5}); pts.push_back({0, -5});
    auto comb = geometry::Contour::fromPolyline(geometry::Polyline{pts, true});
    auto mk = [](const char* n, double d, double so) { PocketTool t; t.tool.name = n; t.tool.diameter = d; t.tool.defaultCuttingData.stepover = so; t.tool.defaultCuttingData.passDepth = 9; return t; };
    PocketParams p; p.cutDepth = 9; p.strategy = PocketStrategy::Raster; if (argc > 3) p.rasterAngle = atof(argv[3]);
    std::vector<PocketTool> tools{mk("B", 0.3, 0.12)}; if (rs > 0) tools = {mk("B", 1.0, 0.4), mk("S", rs, rs * 0.4)};
    double t0 = double(clock()) / CLOCKS_PER_SEC; auto r = pocketToolpath({comb}, tools, p);
    std::printf("teeth %d cpu %.2f moves", teeth, double(clock()) / CLOCKS_PER_SEC - t0); for (auto& tp : r.toolpaths) std::printf(" %zu", tp.moves.size()); for (auto& d : r.diagnostics) std::printf(" | %s", d.message.c_str()); std::printf("\n");
}
