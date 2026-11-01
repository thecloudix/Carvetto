// shr case.txt d side join mitre mode(over|under)
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <string>
#include <variant>
#include <algorithm>
#include "carvetto/geometry/offset.hpp"
#include "offset_chain_checks.hpp"
#include "pw_old.hpp"
using namespace carvetto::geometry;
static double D; static int SIDE, JOIN; static double MIT; static bool OVER;
PContour toP(const Contour& c) { PContour p; p.closed = c.closed; for (auto& s : c.spans) { PSpan q{}; if (auto* l = std::get_if<LineSpan>(&s)) { q.kind = 0; q.a[0] = l->start.x; q.a[1] = l->start.y; q.a[2] = l->end.x; q.a[3] = l->end.y; } else { auto& a = std::get<ArcSpan>(s); q.kind = 1; q.a[0] = a.centre.x; q.a[1] = a.centre.y; q.a[2] = a.radius; q.a[3] = a.startAngle; q.a[4] = a.endAngle; q.a[5] = a.direction == ArcDirection::CCW ? 0 : 1; } p.s.push_back(q); } return p; }
Contour fromP(const PContour& p) { Contour c; c.closed = p.closed; for (auto& s : p.s) { if (s.kind == 0) c.spans.emplace_back(LineSpan{{s.a[0], s.a[1]}, {s.a[2], s.a[3]}}); else if (s.kind == 1) c.spans.emplace_back(ArcSpan{{s.a[0], s.a[1]}, s.a[2], s.a[3], s.a[4], s.a[5] == 0 ? ArcDirection::CCW : ArcDirection::CW}); } return c; }
double worst(const Contour& in, const std::vector<Contour>& out) {
    double w = 0;
    for (auto& c : out) for (auto& s : c.spans) for (int k = 0; k <= 2000; ++k) { Vec2 p = pointAt(s, k / 2000.0); double m = 1e300; for (auto& q : in.spans) m = std::min(m, testing::distanceToSpanExact(p, q)); w = std::max(w, OVER ? m - D : D - m); }
    return w;
}
double oldW(const Contour& in) { auto mp = oldOffset(toP(in), D, SIDE, JOIN, 1e-3, MIT); std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p)); return worst(in, mo); }
double newW(const Contour& in) { return worst(in, offsetOpenContour(in, D, SIDE ? OffsetSide::Right : OffsetSide::Left, JOIN == 0 ? JoinType::Round : JOIN == 1 ? JoinType::Sharp : JoinType::Square, Tolerance::defaults(), MIT)); }
int main(int argc, char** argv) {
    std::ifstream f(argv[1]); std::string line; Contour in; bool started = false;
    while (std::getline(f, line)) { double a, b, c, e, g; char dir[8];
        if (line.rfind("branch chain", 0) == 0 || line.rfind("input gap", 0) == 0) { if (started) break; else continue; }
        if (std::sscanf(line.c_str(), "  L (%lf,%lf)->(%lf,%lf)", &a, &b, &c, &e) == 4) { in.spans.emplace_back(LineSpan{{a, b}, {c, e}}); started = true; }
        else if (std::sscanf(line.c_str(), "  A c(%lf,%lf) r %lf sa %lf ea %lf %7s", &a, &b, &c, &e, &g, dir) == 6) { in.spans.emplace_back(ArcSpan{{a, b}, c, e, g, std::string(dir) == "CCW" ? ArcDirection::CCW : ArcDirection::CW}); started = true; } }
    D = std::atof(argv[2]); SIDE = std::atoi(argv[3]); JOIN = std::atoi(argv[4]); MIT = std::atof(argv[5]); OVER = std::string(argv[6]) == "over";
    double thr = std::atof(argv[7]);
    std::string mode = argv[6];
    auto newOut = [&](const Contour& c) { return offsetOpenContour(c, D, SIDE ? OffsetSide::Right : OffsetSide::Left, JOIN == 0 ? JoinType::Round : JOIN == 1 ? JoinType::Sharp : JoinType::Square, Tolerance::defaults(), MIT); };
    auto oldOut = [&](const Contour& c) { auto mp = oldOffset(toP(c), D, SIDE, JOIN, 1e-3, MIT); std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p)); return mo; };
    auto minLen = [](const std::vector<Contour>& o) { double m = 1e300; for (auto& c : o) for (auto& s : c.spans) m = std::min(m, length(s)); return m; };
    auto pred = [&](const Contour& c) {
        if (mode == "chains") return oldOut(c).size() > newOut(c).size() && newOut(c).size() == 1;
        if (mode == "overlap") return testing::countOverlaps(oldOut(c), 1e-6) > 0 && testing::countOverlaps(newOut(c), 1e-6) == 0;
        if (mode == "subtol") { auto endTiny = [](const std::vector<Contour>& o) { for (auto& k : o) if (!k.closed && k.spans.size() > 1 && (length(k.spans.front()) <= 1e-3 || length(k.spans.back()) <= 1e-3)) return true; return false; }; return endTiny(oldOut(c)) && !endTiny(newOut(c)); }
        if (mode == "zerolen") return minLen(oldOut(c)) < 1e-12 && minLen(newOut(c)) >= 1e-12;
        return oldW(c) > thr && newW(c) <= thr; };
    std::fprintf(stderr, "start spans %zu\n", in.spans.size());
    if (!pred(in)) { std::fprintf(stderr, "predicate false at start\n"); return 1; }
    bool progress = true;
    while (progress) { progress = false;
        for (std::size_t chunk : {64u, 16u, 4u, 1u}) {
            while (in.spans.size() > chunk + 1) { Contour t = in; t.spans.erase(t.spans.begin(), t.spans.begin() + (long)chunk); if (pred(t)) { in = t; progress = true; } else break; }
            while (in.spans.size() > chunk + 1) { Contour t = in; t.spans.erase(t.spans.end() - (long)chunk, t.spans.end()); if (pred(t)) { in = t; progress = true; } else break; }
        } }
    std::fprintf(stderr, "end spans %zu\n", in.spans.size());
    std::printf("x\n");
    for (auto& s : in.spans) { if (auto* l = std::get_if<LineSpan>(&s)) std::printf("  L (%.17g,%.17g)->(%.17g,%.17g)\n", l->start.x, l->start.y, l->end.x, l->end.y); else { auto& a = std::get<ArcSpan>(s); std::printf("  A c(%.17g,%.17g) r %.17g sa %.17g ea %.17g %s\n", a.centre.x, a.centre.y, a.radius, a.startAngle, a.endAngle, a.direction == ArcDirection::CCW ? "CCW" : "CW"); } }
}
