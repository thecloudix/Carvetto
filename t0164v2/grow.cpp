// Cost of the iterative cluster growth in resolveCrossings: M crossing circle pairs, each with a hole
// circle 0.003 mm inside one circle (the polygon of the pair at chordal/8 cuts it, so each cluster absorbs
// its hole one restart at a time), laid out on a page; and a chain of N near-touching circles with a crossing
// pair at one end. GROW_A=<k>: only the last k pairs have the hole. usage: grow pairs <M> | grow chain <N> [chordal]
#include <cstdio>
#include <cstring>
#include <ctime>
#include "gen3.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
int main(int argc, char** argv) {
    int M = atoi(argv[2]); int A = getenv("GROW_A") ? atoi(getenv("GROW_A")) : M; double ch = argc > 3 ? atof(argv[3]) : 0.05; std::vector<Contour> cs;
    if (!strcmp(argv[1], "pairs")) { int side = (int)std::ceil(std::sqrt((double)M));
        for (int i = 0; i < M; ++i) { Vec2 o{(i % side) * 30.0, (i / side) * 20.0}; cs.push_back(g2::circleAt(o, 6, 0)); cs.push_back(g2::circleAt({o.x + 8, o.y}, 4, 0)); if (i >= M - A) cs.push_back(vs::rev(g2::circleAt({o.x, o.y + 4.997}, 1, 0))); } }
    else { double g = ch / 16; double x = 0; for (int i = 0; i < M; ++i) { cs.push_back(g2::circleAt({x, 0}, 1, 0)); x += i == 0 ? 1.5 : 2 + g; } }
    tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.includedAngle = 3.14159265358979 / 2; t.diameter = 12;
    auto& c = t.defaultCuttingData; c.feedRate = 1000; c.plungeRate = 300; c.passDepth = 1; c.stepover = 1; c.toolNumber = 1; c.spindleSpeed = 10000;
    VCarvingParams p; p.tolerance.chordal = ch; p.tolerance.distance = std::min(p.tolerance.distance, ch);
    double t0 = double(std::clock()) / CLOCKS_PER_SEC; auto r = vCarvingToolpath(cs, t, std::nullopt, {}, p); double t1 = double(std::clock()) / CLOCKS_PER_SEC;
    std::printf("%s %d contours=%zu ch=%g cpu=%.2fs ok=%d moves=%zu regions=%zu", argv[1], M, cs.size(), ch, t1 - t0, (int)r.ok(), r.toolpath.moves.size(), r.regions);
    unsigned long long h = 1469598103934665603ull; auto mix = [&](const void* d, size_t n) { auto* b = (const unsigned char*)d; for (size_t q = 0; q < n; ++q) { h ^= b[q]; h *= 1099511628211ull; } };
    for (auto& m : r.toolpath.moves) { int k = (int)m.kind; mix(&k, sizeof k); mix(&m.to.x, 8); mix(&m.to.y, 8); mix(&m.to.z, 8); mix(&m.centre.x, 8); mix(&m.centre.y, 8); }
    for (auto& d : r.diagnostics) { int k = (int)d.issue; mix(&k, sizeof k); mix(d.message.data(), d.message.size()); }
    std::printf(" hash=%016llx", h);  // t0164v1: byte-identity of moves + diagnostics
    for (auto& d : r.diagnostics) std::printf(" [%d %s]", (int)d.issue, d.message.substr(0, 60).c_str()); std::printf("\n");
}
