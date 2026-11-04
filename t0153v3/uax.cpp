// Resolves the input like V-carve (even-odd union at chordal/8), then reports each resolved contour and the
// medial axis of all of them: branches, ignored contours, max radius. usage: uax <chordal> [k: dump resolved contour k] < contours
#include <cstdio>
#include <iostream>
#include <sstream>
#include "oracle.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::string line;
    while (std::getline(std::cin, line)) { std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
        else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double x, y, r, s, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s >> e >> dd >> dir; cs.back().append(ArcSpan{{x, y}, r, s, e, dir ? ArcDirection::CW : ArcDirection::CCW}); }
        else if (t == "B") { double v[8]; for (double& q : v) is >> q; cs.back().append(BezierSpan{{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}}); } }
    Tolerance tol; tol.chordal = atof(argv[1]); tol.distance = std::min(0.001, tol.chordal);
    Tolerance fine = tol; fine.chordal *= 0.125; fine.distance = std::min(tol.distance, fine.chordal);
    auto u = unionContours(cs, fine, FillRule::EvenOdd);
    std::printf("resolved contours: %zu\n", u.size());
    for (size_t i = 0; i < u.size(); ++i) { auto b = u[i].bounds(); std::printf("  %zu: spans %zu area %.4f box (%.3f,%.3f)-(%.3f,%.3f)\n", i, u[i].spans.size(), area(u[i]), b.min.x, b.min.y, b.max.x, b.max.y); }
    if (argc > 2) { size_t k = (size_t)atoi(argv[2]); std::printf("contour\n"); for (auto& sp : u[k].spans) { auto l = std::get<LineSpan>(sp); std::printf("    L %.17g %.17g %.17g %.17g\n", l.start.x, l.start.y, l.end.x, l.end.y); } return 0; }
    MedialAxis ax = medialAxis(u, tol); double rmax = 0; size_t pts = 0; for (auto& br : ax.branches) for (auto& p : br.points) { rmax = std::max(rmax, p.radius); ++pts; }
    std::printf("medial axis of all: branches %zu nodes %zu points %zu rmax %.4f ignored %zu capped %d\n", ax.branches.size(), ax.nodes.size(), pts, rmax, ax.ignoredContours, (int)ax.accuracyCapped);
    for (size_t i = 0; i < u.size(); ++i) { MedialAxis a1 = medialAxis({u[i]}, tol); double r1 = 0; for (auto& br : a1.branches) for (auto& p : br.points) r1 = std::max(r1, p.radius); for (auto& nd : a1.nodes) r1 = std::max(r1, nd.radius);
        std::printf("  alone %zu: branches %zu rmax %.4f ignored %zu\n", i, a1.branches.size(), r1, a1.ignoredContours); }
}
