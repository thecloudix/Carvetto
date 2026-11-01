#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <fstream>
#include <string>
#include <variant>
#include <algorithm>
#include "carvetto/geometry/offset.hpp"
#include "pw_old.hpp"
using namespace carvetto::geometry;
PContour toP(const Contour& c) { PContour p; p.closed = c.closed; for (auto& s : c.spans) { PSpan q{}; if (auto* l = std::get_if<LineSpan>(&s)) { q.kind = 0; q.a[0] = l->start.x; q.a[1] = l->start.y; q.a[2] = l->end.x; q.a[3] = l->end.y; } else { auto& a = std::get<ArcSpan>(s); q.kind = 1; q.a[0] = a.centre.x; q.a[1] = a.centre.y; q.a[2] = a.radius; q.a[3] = a.startAngle; q.a[4] = a.endAngle; q.a[5] = a.direction == ArcDirection::CCW ? 0 : 1; } p.s.push_back(q); } return p; }
Contour fromP(const PContour& p) { Contour c; c.closed = p.closed; for (auto& s : p.s) { if (s.kind == 0) c.spans.emplace_back(LineSpan{{s.a[0], s.a[1]}, {s.a[2], s.a[3]}}); else if (s.kind == 1) c.spans.emplace_back(ArcSpan{{s.a[0], s.a[1]}, s.a[2], s.a[3], s.a[4], s.a[5] == 0 ? ArcDirection::CCW : ArcDirection::CW}); } return c; }
void report(const char* tag, const std::vector<Contour>& out) {
    for (size_t ci = 0; ci < out.size(); ++ci) { auto& ch = out[ci]; size_t n = ch.spans.size();
        for (size_t j = 0; j + 1 < n || (ch.closed && j < n && n > 1); ++j) { Vec2 P = endPoint(ch.spans[j]), Q = startPoint(ch.spans[(j + 1) % n]); if (!(P == Q)) std::printf("%s ch %zu closed %d j %zu gap %.3g kinds %zu %zu\n", tag, ci, (int)ch.closed, j, distance(P, Q), ch.spans[j].index(), ch.spans[(j + 1) % n].index()); } }
}
int main(int argc, char** argv) {
    std::ifstream f(argv[1]); std::string line; Contour in; bool started = false;
    while (std::getline(f, line)) { double a, b, c, e, g; char dir[8];
        if (line.rfind("branch chain", 0) == 0 || line.rfind("input gap", 0) == 0) { if (started) break; else continue; }
        if (std::sscanf(line.c_str(), "  L (%lf,%lf)->(%lf,%lf)", &a, &b, &c, &e) == 4) { in.spans.emplace_back(LineSpan{{a, b}, {c, e}}); started = true; }
        else if (std::sscanf(line.c_str(), "  A c(%lf,%lf) r %lf sa %lf ea %lf %7s", &a, &b, &c, &e, &g, dir) == 6) { in.spans.emplace_back(ArcSpan{{a, b}, c, e, g, std::string(dir) == "CCW" ? ArcDirection::CCW : ArcDirection::CW}); started = true; } }
    double D = std::atof(argv[2]); int side = std::atoi(argv[3]), j = std::atoi(argv[4]); double mit = std::atof(argv[5]);
    report("NEW", offsetOpenContour(in, D, side ? OffsetSide::Right : OffsetSide::Left, j == 0 ? JoinType::Round : j == 1 ? JoinType::Sharp : JoinType::Square, Tolerance::defaults(), mit));
    auto mp = oldOffset(toP(in), D, side, j, 1e-3, mit); std::vector<Contour> mo; for (auto& p : mp) mo.push_back(fromP(p)); report("OLD", mo);
}
