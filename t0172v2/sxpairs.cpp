// pairs of grown boxes that meet on the sxbf back-and-forth strand (is selfCrossingCuts output-bound there?)
#include "offset.cpp"
#include <chrono>
#include <cstdio>
#include <random>
using namespace carvetto::geometry;
int main(int, char** argv) {
    int n = atoi(argv[1]); std::mt19937_64 r(1); std::vector<Span> s; double x = 0;
    for (int i = 0; i < n; ++i) { double L = std::exp(std::uniform_real_distribution<double>(std::log(1e-6), std::log(10.0))(r)); double nx = (i % 2 == 0) ? x + L : x - L; s.emplace_back(LineSpan{{x, 0}, {nx, 0}}); x = nx; }
    std::vector<Box2> b; for (auto& q : s) { Box2 bb = bounds(q); b.push_back(bb.inflated(meetReach(q, bb))); }
    detail::BoxTree t(b); size_t pairs = 0;
    for (int i = 2; i < n; ++i) t.queryUntil(b[i], 0, i - 2, [&](size_t) { ++pairs; return false; });
    auto t0 = std::chrono::steady_clock::now(); auto c = selfCrossingCuts(s);
    double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    printf("n %d earlier-pairs %zu (/n %.1f) work %zu cuts %zu time %.3f s (%.1f ns/pair)\n", n, pairs, (double)pairs / n, t.work(), c.size(), sec, 1e9 * sec / std::max<size_t>(pairs, 1));
}
