#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <variant>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::ifstream f(argv[1]); std::string line; Contour in;
    while (std::getline(f, line)) { double a, b, c, e, g; char dir[8];
        if (std::sscanf(line.c_str(), "  L (%lf,%lf)->(%lf,%lf)", &a, &b, &c, &e) == 4) in.spans.emplace_back(LineSpan{{a, b}, {c, e}});
        else if (std::sscanf(line.c_str(), "  A c(%lf,%lf) r %lf sa %lf ea %lf %7s", &a, &b, &c, &e, &g, dir) == 6) in.spans.emplace_back(ArcSpan{{a, b}, c, e, g, std::string(dir) == "CCW" ? ArcDirection::CCW : ArcDirection::CW}); }
    int j = std::atoi(argv[4]);
    auto out = offsetOpenContour(in, std::atof(argv[2]), std::atoi(argv[3]) ? OffsetSide::Right : OffsetSide::Left, j == 0 ? JoinType::Round : j == 1 ? JoinType::Sharp : JoinType::Square, Tolerance::defaults(), std::atof(argv[5]));
    std::printf("chains %zu\n", out.size());
}
