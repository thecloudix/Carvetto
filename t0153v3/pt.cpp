// distance / inside / per-contour winding for points against a contour file (oracle acc.hpp)
#include <cstdio>
#include <fstream>
#include <sstream>
#include "acc.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::ifstream in(argv[1]); std::string line;
    while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
        else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double x, y, r, s0, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s0 >> e >> dd >> dir; cs.back().append(ArcSpan{{x, y}, r, s0, e, dir ? ArcDirection::CW : ArcDirection::CCW}); } }
    va::Acc A; A.build(cs);
    for (int i = 2; i + 1 < argc; i += 2) { Vec2 p{atof(argv[i]), atof(argv[i + 1])}; std::printf("(%g,%g): inside=%d dist=%.6f", p.x, p.y, (int)A.inside(p), A.dist(p));
        for (size_t c = 0; c < cs.size(); ++c) { std::vector<Contour> one{cs[c]}; va::Acc B; B.build(one); std::printf("  in%zu=%d d%zu=%.6f", c, (int)B.inside(p), c, B.dist(p)); } std::printf("\n"); }
}
