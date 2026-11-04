// Dumps medial-axis points that lie outside the even-odd region (verifier oracle insideEO) for contours read from stdin.
#include <cstdio>
#include <iostream>
#include <sstream>
#include <string>
#include "oracle.hpp"
#include "carvetto/geometry/medial_axis.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::string line;
    while (std::getline(std::cin, line)) {
        std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
        else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double x, y, r, s, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s >> e >> dd >> dir; cs.back().append(ArcSpan{{x, y}, r, s, e, dir ? ArcDirection::CW : ArcDirection::CCW}); }
        else if (t == "B") { double v[8]; for (double& q : v) is >> q; cs.back().append(BezierSpan{{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}}); }
    }
    Tolerance tol; tol.chordal = atof(argv[1]); tol.distance = std::min(tol.distance, tol.chordal);
    MedialAxis ax = medialAxis(cs, tol);
    vo::Boundary B = vo::makeBoundary(cs, tol.chordal * 0.25);
    int n = 0, out = 0; double worstR = 0;
    for (auto& b : ax.branches) for (auto& p : b.points) { ++n; double d = vo::exactDist(B, p.point).d; bool in = vo::insideEO(B, p.point);
        if (!in && d > 1e-6) { ++out; if (out <= 10) std::printf("OUT (%.5f,%.5f) r=%.5f dist=%.5f\n", p.point.x, p.point.y, p.radius, d); }
        worstR = std::max(worstR, std::fabs(p.radius - d)); }
    int nout = 0;
    for (auto& nd : ax.nodes) { double d = vo::exactDist(B, nd.point).d; if (!vo::insideEO(B, nd.point) && d > 1e-6) { ++nout; if (nout <= 10) std::printf("NODE OUT (%.5f,%.5f) r=%.5f dist=%.5f deg=%zu\n", nd.point.x, nd.point.y, nd.radius, d, nd.branches.size()); } }
    std::printf("nodes=%zu nodesOutside=%d\n", ax.nodes.size(), nout);
    std::printf("points=%d outside=%d branches=%zu capped=%d ignored=%zu worst|r-dist|=%.3g\n", n, out, ax.branches.size(), (int)ax.accuracyCapped, ax.ignoredContours, worstR);
}
