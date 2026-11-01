#include "../t0121v2/common.hpp"
#include <ctime>
int main(int argc, char** argv) {
    int teeth = atoi(argv[1]); double W = atof(argv[2]), H = atof(argv[3]), so = atof(argv[4]), d = atof(argv[5]); size_t mb = atoi(argv[6]);
    struct rlimit rl{mb << 20, mb << 20}; setrlimit(RLIMIT_AS, &rl);
    double pitch = W / teeth, gap = pitch / 2;
    std::vector<Vec2> pts;
    for (int i = 0; i < teeth; ++i) { pts.push_back({i * pitch, 0}); pts.push_back({i * pitch, H}); pts.push_back({i * pitch + gap, H}); pts.push_back({i * pitch + gap, 0.0}); }
    pts.push_back({teeth * pitch, -5}); pts.push_back({0, -5});
    PocketParams p; p.cutDepth = 9; p.safeZ = 5; p.strategy = PocketStrategy::Raster; p.rasterAngle = 0;
    std::clock_t c0 = std::clock();
    try {
        auto r = pocketToolpath({Contour::fromPolyline(geometry::Polyline{pts, true})}, {tool(d, so, 9)}, p);
        struct rusage ru; getrusage(RUSAGE_SELF, &ru);
        printf("teeth %d W %.0f H %.0f so %.3f d %.2f: cpu %.1fs maxrss %ld MB moves %zu msgs %s\n", teeth, W, H, so, d, double(std::clock() - c0) / CLOCKS_PER_SEC, ru.ru_maxrss / 1024, r.toolpaths[0].moves.size(), r.messages.empty() ? "" : r.messages[0].c_str());
    } catch (std::exception& e) { struct rusage ru; getrusage(RUSAGE_SELF, &ru); printf("teeth %d: EXCEPTION %s after cpu %.1fs maxrss %ld MB\n", teeth, e.what(), double(std::clock() - c0) / CLOCKS_PER_SEC, ru.ru_maxrss / 1024); }
}
