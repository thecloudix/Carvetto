// T-0172 verifier: selfCrossingCuts on a back-and-forth strand along one line with lengths over many scales (collinear spans:
// carrierIntersections reports nothing for parallel lines, so the strand never "crosses"): build cost of its ClusterGrid.
#include "offset.cpp"
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <random>
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    int n = atoi(argv[1]); double lo = argc > 2 ? atof(argv[2]) : 1e-6; std::mt19937_64 r(1);
    std::vector<Span> s; double x = 0;
    for (int i = 0; i < n; ++i) { double L = std::exp(std::uniform_real_distribution<double>(std::log(lo), std::log(10.0))(r)); double nx = (i % 2 == 0) ? x + L : x - L; s.emplace_back(LineSpan{{x, 0}, {nx, 0}}); x = nx; }
    auto t0 = std::chrono::steady_clock::now();
    auto c = carvetto::geometry::detail::selfCrossingCuts(s);
    double t = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("back-and-forth strand n=%d lengths %.0e..10: cuts %zu, %.3f s\n", n, lo, c.size(), t);
}
