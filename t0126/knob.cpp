#include <cstdio>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto;
int main(int argc, char** argv) {
    std::vector<geometry::Vec2> p; double x, y; FILE* f = fopen(argv[1], "r"); while (fscanf(f, "%lf %lf", &x, &y) == 2) p.push_back({x, y});
    auto c = geometry::Contour::fromPolyline(geometry::Polyline{p, true});
    toolpath::PocketParams P; P.tolerance.chordal = 0.01; P.tolerance.distance = std::min(P.tolerance.distance, 0.01);
    int bad = 0;
    for (int i = 2; i < argc; ++i) { double d = atof(argv[i]); std::vector<std::string> ms; auto C = toolpath::pocketCentreRegion({c}, d, P, &ms);
      printf("d %s parts %zu msgs %zu %s\n", argv[i], C.size(), ms.size(), ms.empty() ? "" : ms[0].c_str()); bad += !ms.empty(); }
    return bad;
}
