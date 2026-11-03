#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry; using vo::dist;
static void run(const char* nm, const std::vector<Contour>& cs, const std::vector<Contour>& realCs, FillRule fr) {
    Tolerance tol; MedialAxisOptions o; o.fillRule = fr; MedialAxis ax = medialAxis(cs, tol, o);
    vo::Boundary B = vo::makeBoundary(realCs, tol.chordal / 4);
    size_t bad = 0, n = 0, wrongIdx = 0; double worst = 0, radErr = 0;
    for (auto& br : ax.branches) for (size_t i = 0; i + 1 < br.points.size(); ++i) for (int k = 0; k < 2; ++k) { auto& r = br.sides[i][k]; ++n;
        if (r.contour >= cs.size() || r.span >= cs[r.contour].spans.size()) { ++bad; continue; }
        if (cs[r.contour].spans.empty() || !cs[r.contour].closed) ++wrongIdx;
        Vec2 q = vo::evalSpan(cs[r.contour].spans[r.span], r.t);
        worst = std::max(worst, std::fabs(dist(q, br.points[i].point) - br.points[i].radius)); }
    for (auto& br : ax.branches) for (auto& p : br.points) radErr = std::max(radErr, std::fabs(vo::exactDist(B, p.point).d - p.radius));
    std::printf("%-28s ign=%zu br=%zu refs=%zu bad=%zu refsToIgnored=%zu worstEq=%.3e radErr=%.3e capped=%d\n", nm, ax.ignoredContours, ax.branches.size(), n, bad, wrongIdx, worst, radErr, (int)ax.accuracyCapped);
}
int main() {
    Contour open = vs::poly({{0, 0}, {1, 0}, {1, 1}}); open.closed = false;
    Contour zero = vs::poly({{0, 0}, {5, 0}, {10, 0}});
    Contour empty; empty.closed = true;
    auto outer = vs::poly({{0, 0}, {30, 0}, {30, 30}, {0, 30}});
    auto hole = vs::rev(vs::poly({{5, 5}, {12, 5}, {12, 12}, {5, 12}}));
    run("ignored-first EO", {open, zero, empty, outer, hole}, {outer, hole}, FillRule::EvenOdd);
    run("baseline EO", {outer, hole}, {outer, hole}, FillRule::EvenOdd);
    // NonZero: same-orientation inner contour is NOT a hole -> region is the outer square only
    auto inner_same = vs::poly({{5, 5}, {12, 5}, {12, 12}, {5, 12}});
    run("nonzero same-orient inner", {outer, inner_same}, {outer}, FillRule::NonZero);
    run("evenodd same-orient inner", {outer, inner_same}, {outer, inner_same}, FillRule::EvenOdd);
    // Duplicate contour under even-odd cancels -> empty region expected
    run("duplicate EO", {outer, outer}, {outer}, FillRule::EvenOdd);
    // Two separate regions
    run("two regions", {vs::poly({{0, 0}, {5, 0}, {5, 5}, {0, 5}}), vs::circle({20, 0}, 3)}, {vs::poly({{0, 0}, {5, 0}, {5, 5}, {0, 5}}), vs::circle({20, 0}, 3)}, FillRule::EvenOdd);
}
