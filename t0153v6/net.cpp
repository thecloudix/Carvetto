// T-0153 round 6 (bdc932c): drive detail::SkeletonCheck directly (fault injection like the branch's own test).
// usage: net <region file> <chordal> <wrong subset e.g. "01" | "all"> [verbose]  (env OWNLIST=01: own indices override; OWN=0: own={} = measure all rings; default own = the wrong subset, as v_carve passes the part)
// Builds SkeletonCheck over ALL contours of the file at the V-carve internal tolerance (chordal/3, distance min(0.001, c/3)),
// computes medialAxis of the WRONG subset, prints per branch: n, stride, sampled indices/points/radii, whether each
// sampled point is inside the true region (oracle) and the fraction of ALL branch points inside, and the verdict.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <set>
#include "acc.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "v_carve_check.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    std::vector<Contour> cs; std::ifstream in(argv[1]); std::string line;
    while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
        else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double x, y, r, s0, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s0 >> e >> dd >> dir; cs.back().append(ArcSpan{{x, y}, r, s0, e, dir ? ArcDirection::CW : ArcDirection::CCW}); } }
    double ch = atof(argv[2]); Tolerance inner; inner.chordal = std::max(ch / 3.0, 1e-4); inner.distance = std::min(std::min(0.001, ch), inner.chordal);
    std::vector<Contour> wrong;
    if (std::strcmp(argv[3], "all") == 0) wrong = cs; else for (char* p = argv[3]; *p; ++p) wrong.push_back(cs[static_cast<size_t>(*p <= '9' ? *p - '0' : *p - 'a' + 10)]);
    std::vector<size_t> own; if (std::strcmp(argv[3], "all") == 0) { for (size_t i = 0; i < cs.size(); ++i) own.push_back(i); } else for (char* p = argv[3]; *p; ++p) own.push_back(static_cast<size_t>(*p <= '9' ? *p - '0' : *p - 'a' + 10));
    if (std::getenv("OWN") && std::atoi(std::getenv("OWN")) == 0) own.clear();
    if (const char* ol = std::getenv("OWNLIST")) { own.clear(); for (const char* p = ol; *p; ++p) own.push_back(static_cast<size_t>(*p <= '9' ? *p - '0' : *p - 'a' + 10)); }  // e.g. MA of {0} but the part is {0,1}: a silently dropped hole
    bool verbose = argc > 4;
    carvetto::toolpath::detail::SkeletonCheck chk(cs, inner);
    MedialAxis ax = medialAxis(wrong, inner); va::Acc A; A.build(cs);
    // structure: branch end points coincide with their node points; nodes in range
    { int bad = 0; for (auto& br : ax.branches) { if (br.startNode >= ax.nodes.size() || br.endNode >= ax.nodes.size() || br.points.empty()) { ++bad; continue; }
        if (!(br.points.front().point.x == ax.nodes[br.startNode].point.x && br.points.front().point.y == ax.nodes[br.startNode].point.y)) ++bad;
        if (!(br.points.back().point.x == ax.nodes[br.endNode].point.x && br.points.back().point.y == ax.nodes[br.endNode].point.y)) ++bad; }
      for (size_t b = 0; b < ax.branches.size(); ++b) { auto& br = ax.branches[b]; if (br.startNode >= ax.nodes.size() || br.endNode >= ax.nodes.size()) continue;
          auto has = [&](size_t nd) { for (auto x : ax.nodes[nd].branches) if (x == b) return true; return false; }; if (!has(br.startNode) || !has(br.endNode)) ++bad; }
      for (auto& nd : ax.nodes) for (auto x : nd.branches) if (x >= ax.branches.size()) ++bad;
      if (bad) std::printf("STRUCT: %d branch ends not on their node\n", bad); }

    double clear = 2 * inner.chordal + inner.distance;
    int outside = 0, discBad = 0, total = 0;
    for (size_t b = 0; b < ax.branches.size(); ++b) { auto& P = ax.branches[b].points; size_t n = P.size(); if (!n) continue;
        size_t deepest = 0; for (size_t k = 1; k < n; ++k) if (P[k].radius > P[deepest].radius) deepest = k;
        size_t stride = n <= 8 ? 1 : (n + 7) / 8; std::set<size_t> s; for (size_t k = 0; k < n; k += stride) s.insert(k); s.insert(deepest); s.insert(n - 1);
        int bo = 0, bd = 0; for (size_t k = 0; k < n; ++k) { ++total; if (P[k].radius > clear) { bool o = !A.inside(P[k].point); bool d = A.dist(P[k].point) < P[k].radius - clear; bo += o; bd += d; } }
        outside += bo; discBad += bd;
        if (verbose) { std::printf("branch %zu n=%zu stride=%zu outside=%d discBad=%d sampled:", b, n, stride, bo, bd);
            for (size_t k : s) std::printf(" [%zu (%.4f,%.4f) r%.4f in%d d%.4f]", k, P[k].point.x, P[k].point.y, P[k].radius, (int)A.inside(P[k].point), A.dist(P[k].point)); std::printf("\n"); } }
    std::printf("wrong=%s ignored=%zu capped=%d branches=%zu nodes=%zu points=%d bad_points(r>clear): outside=%d disc=%d  consistent=%d\n", argv[3], ax.ignoredContours, (int)ax.accuracyCapped, ax.branches.size(), ax.nodes.size(), total, outside, discBad, (int)chk.consistent(ax, own));
}
