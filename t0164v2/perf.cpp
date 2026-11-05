// AC2 check with the verifier's own glyphs (A polygon+hole, S Beziers, O Bezier ring, C arcs), 6 mm tall,
// default tolerance, 60 deg 12.7 mm V-bit (also: VPERF_FLAT=1 adds a 0.5 mm flat depth, self-clear).
// VPERF_FRAME=1: one rectangle round the whole page (letters become holes: one part). VPERF_EM=<D>: end-mill clearance tool.
// usage: perf <n1> <n2> ...   prints CPU seconds, moves, peak RSS.
#include <cstdio>
#include <ctime>
#include <sys/resource.h>
#include "oracle.hpp"
#include "shapes.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static double cpu() { return double(std::clock()) / CLOCKS_PER_SEC; }
static Contour xf(const Contour& c, double s, Vec2 off) {
    Contour o; o.closed = c.closed;
    for (auto& sp : c.spans) {
        if (auto* l = std::get_if<LineSpan>(&sp)) o.append(LineSpan{l->start * s + off, l->end * s + off});
        else if (auto* a = std::get_if<ArcSpan>(&sp)) { ArcSpan b = *a; b.centre = a->centre * s + off; b.radius *= s; o.append(b); }
        else { auto b = std::get<BezierSpan>(sp); o.append(BezierSpan{b.p0 * s + off, b.p1 * s + off, b.p2 * s + off, b.p3 * s + off}); }
    }
    return o;
}
int main(int argc, char** argv) {
    auto lib = vs::all(); int ids[4] = {17, 15, 16, 14};
    tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.diameter = 12.7; t.includedAngle = 60 * vo::PI / 180;
    t.defaultCuttingData = {1, 0.3, 0, 0, 18000, 1500, 500, 0, 1};
    for (int ai = 1; ai < argc; ++ai) {
        int n = atoi(argv[ai]); std::vector<Contour> page; size_t spans = 0;
        for (int k = 0; k < n; ++k) { auto& sh = lib[ids[k % 4]]; Box2 bb; for (auto& c : sh.cs) bb.expand(c.bounds()); double s = 6.0 / std::max(bb.height(), bb.width());
            Vec2 off{(k % 100) * 8.0 - bb.min.x * s, (k / 100) * 8.0 - bb.min.y * s};
            for (auto& c : sh.cs) { page.push_back(xf(c, s, off)); spans += c.spans.size(); } }
        if (getenv("VPERF_FRAME")) { Box2 all; for (auto& c : page) all.expand(c.bounds()); page.push_back(vs::poly({{all.min.x - 2, all.min.y - 2}, {all.max.x + 2, all.min.y - 2}, {all.max.x + 2, all.max.y + 2}, {all.min.x - 2, all.max.y + 2}})); }
        std::vector<PocketTool> cl; if (getenv("VPERF_EM")) { tooling::Tool e; e.id = e.name = "EM"; e.type = tooling::ToolType::EndMill; e.diameter = atof(getenv("VPERF_EM")); e.defaultCuttingData = {2, 0.4 * e.diameter, 0, 0, 18000, 1500, 500, 0, 2}; e.defaultCuttingData.passDepth = 1; e.defaultCuttingData.stepover = 0.4 * e.diameter; cl.push_back({e, std::nullopt}); }
        VCarvingParams p; if (getenv("VPERF_FLAT")) p.flatDepth = atof(getenv("VPERF_FLAT")) > 0 ? atof(getenv("VPERF_FLAT")) : 0.5;
        double t0 = cpu(); auto r = vCarvingToolpath(page, t, std::nullopt, cl, p); double dt = cpu() - t0;
        rusage ru; getrusage(RUSAGE_SELF, &ru);
        std::printf("%d regions %zu spans: %.2f s CPU, %zu moves, ok=%d diags=%zu, peak RSS %ld MB\n", n, spans, dt, r.toolpath.moves.size() + (r.clearance.empty() ? 0 : r.clearance[0].moves.size()), (int)r.ok(), r.diagnostics.size(), ru.ru_maxrss / 1024);
        for (size_t i = 0; i < std::min<size_t>(r.diagnostics.size(), 3); ++i) std::printf("   diag %d %s\n", (int)r.diagnostics[i].issue, r.diagnostics[i].message.substr(0, 160).c_str());
    }
}
