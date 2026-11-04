// Union (even-odd) of the nestedsq_cone repro at chordal/8 (what resolveCrossings does): print each polygon's box, area, vertex count.
#include <cstdio>
#include <fstream>
#include <sstream>
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::ifstream in(argv[1]); std::string line;
    while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t; if (t == "contour") { cs.emplace_back(); cs.back().closed = true; } else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); } }
    Tolerance tol; tol.chordal = atof(argv[2]) / 8; tol.distance = std::min(0.001, tol.chordal);
    auto u = unionContours(cs, tol, FillRule::EvenOdd);
    for (auto& c : u) { auto b = c.bounds(); std::printf("poly spans=%zu box x[%.6f,%.6f] y[%.6f,%.6f] area %.6f\n", c.spans.size(), b.min.x, b.max.x, b.min.y, b.max.y, signedArea(c));
        if (c.spans.size() < 20) for (auto& s : c.spans) { auto p = startPoint(s); std::printf("    %.6f %.6f\n", p.x, p.y); } }
}
