// round 4: medialAxis branches of a contour file at (chordal, distance): per branch its deepest point, radius and
// whether it lies in the even-odd region (oracle), plus isolated nodes; the V-carve net decides only radius > 2c+d.
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
    if (argc > 4) { std::vector<Contour> sub; for (char* p = argv[4]; *p; ++p) sub.push_back(cs[*p - '0']); cs = sub; }  // e.g. "01": only contours 0 and 1 (a V-carve part)
    Tolerance tol; tol.chordal = atof(argv[2]); tol.distance = atof(argv[3]);
    MedialAxis ax = medialAxis(cs, tol); va::Acc A; A.build(cs);
    std::printf("ignored=%zu capped=%d branches=%zu nodes=%zu clear=%.4f\n", ax.ignoredContours, (int)ax.accuracyCapped, ax.branches.size(), ax.nodes.size(), 2 * tol.chordal + tol.distance);
    for (size_t b = 0; b < ax.branches.size(); ++b) { const MedialPoint* d = nullptr; int out = 0; for (auto& m : ax.branches[b].points) { if (!d || m.radius > d->radius) d = &m; if (!A.inside(m.point)) ++out; }
        if (d) std::printf("  branch %zu: %zu pts, %d outside region; deepest (%.4f,%.4f) r=%.4f inside=%d distToBoundary=%.4f\n", b, ax.branches[b].points.size(), out, d->point.x, d->point.y, d->radius, (int)A.inside(d->point), A.dist(d->point)); }
    for (auto& n : ax.nodes) if (n.branches.empty()) std::printf("  isolated node (%.4f,%.4f) r=%.4f inside=%d\n", n.point.x, n.point.y, n.radius, (int)A.inside(n.point));
}
