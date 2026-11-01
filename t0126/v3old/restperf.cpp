#include "../t0121v2/common.hpp"
#include <ctime>
static std::vector<Contour> notched(int N, double R, double w, double dd) {
    std::vector<Vec2> pts;
    for (int k = 0; k < N; ++k) { double t0 = 2 * PI * k / N, t1 = 2 * PI * (k + 1) / N, tm = 0.5 * (t0 + t1); double half = w / 2 / R; Vec2 u{std::cos(tm), std::sin(tm)}, v{-u.y, u.x};
        pts.push_back({R * std::cos(t0), R * std::sin(t0)}); Vec2 a = u * (R * std::cos(half)) - v * (w / 2), b = u * (R * std::cos(half)) + v * (w / 2); pts.push_back(a); pts.push_back(a + u * dd); pts.push_back(b + u * dd); pts.push_back(b); }
    return {Contour::fromPolyline(geometry::Polyline{pts, true})};
}
int main(int argc, char** argv) {
    int N = atoi(argv[1]); double R = atof(argv[2]), so = atof(argv[3]); int strat = atoi(argv[4]);
    struct rlimit rl{size_t(4000) << 20, size_t(4000) << 20}; setrlimit(RLIMIT_AS, &rl);
    PocketParams p; p.cutDepth = 2; p.safeZ = 5; p.strategy = strat ? PocketStrategy::Raster : PocketStrategy::Offset; p.rasterAngle = 0.2;
    std::clock_t c0 = std::clock();
    auto r = pocketToolpath(notched(N, R, 3, 3), {tool(6, 2.4, 2, "T6"), tool(0.5, so, 2, "T05")}, p);
    struct rusage ru; getrusage(RUSAGE_SELF, &ru);
    printf("N %d R %.0f so %.3f strat %d: cpu %.1fs maxrss %ld MB moves %zu/%zu rest regions %zu paths %zu msgs %s\n", N, R, so, strat, double(std::clock() - c0) / CLOCKS_PER_SEC, ru.ru_maxrss / 1024, r.toolpaths[0].moves.size(), r.toolpaths[1].moves.size(), r.plans[1].restMaterial.size(), r.plans[1].paths.size(), r.messages.empty() ? "" : r.messages[0].c_str());
}
