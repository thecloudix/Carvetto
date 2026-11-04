// medialAxis on a contour file at chordal c: ignored contours, capped, and skeleton points inside a given contour (by the oracle).
#include <cstdio>
#include <fstream>
#include <sstream>
#include "acc.hpp"
#include "carvetto/geometry/medial_axis.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::ifstream in(argv[1]); std::string line;
    while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
        else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double x, y, r, s0, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s0 >> e >> dd >> dir; cs.back().append(ArcSpan{{x, y}, r, s0, e, dir ? ArcDirection::CW : ArcDirection::CCW}); } }
    Tolerance tol; tol.chordal = atof(argv[2]); tol.distance = std::min(0.001, tol.chordal);
    MedialAxis ax = medialAxis(cs, tol);
    va::Acc A; A.build(cs); size_t n = 0, outside = 0; double worstR = 0; Vec2 wp{};
    for (auto& b : ax.branches) for (auto& q : b.points) { ++n; if (!A.inside(q.point) && A.dist(q.point) > 1e-4) { ++outside; if (q.radius > worstR) { worstR = q.radius; wp = q.point; } } }
    std::printf("chordal %g: ignored=%zu capped=%d branches=%zu points=%zu outsideRegion=%zu (largest radius there %.4f at (%.4f,%.4f))\n", tol.chordal, ax.ignoredContours, (int)ax.accuracyCapped, ax.branches.size(), n, outside, worstR, wp.x, wp.y);
}
