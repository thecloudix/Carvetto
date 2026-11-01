#include "../t0121v2/common.hpp"
#include <ctime>
int main() {
    auto run = [](const char* nm, std::vector<Contour> c, std::vector<PocketTool> t, PocketParams p) { std::clock_t c0 = std::clock(); auto r = pocketToolpath(c, t, p); double s = double(std::clock() - c0) / CLOCKS_PER_SEC; size_t m = 0; for (auto& tp : r.toolpaths) m += tp.moves.size(); printf("%-60s cpu %.3fs moves %zu\n", nm, s, m); };
    PocketParams p; p.cutDepth = 6; p.safeZ = 5;
    run("AC1 rect 50x30 T6 2.4", {Contour::rectangle(Box2{{0, 0}, {50, 30}})}, {tool(6, 2.4, 3)}, p);
    run("rect 300x200 T6 2.4 offset", {Contour::rectangle(Box2{{0, 0}, {300, 200}})}, {tool(6, 2.4, 3)}, p);
    { PocketParams q = p; q.strategy = PocketStrategy::Raster; q.rasterAngle = 0.5; run("rect 300x200 T6 2.4 raster", {Contour::rectangle(Box2{{0, 0}, {300, 200}})}, {tool(6, 2.4, 3)}, q); }
    { std::vector<Contour> c{Contour::rectangle(Box2{{0, 0}, {300, 200}})}; for (int a = 0; a < 14; ++a) for (int b = 0; b < 9; ++b) c.push_back(Contour::circle({20.0 + a * 20, 20.0 + b * 20}, 5)); run("rect 300x200 + 126 islands T6 2.4 offset", c, {tool(6, 2.4, 3)}, p);
      run("rect 300x200 + 126 islands T12/T3 offset", c, {tool(12, 4.8, 3), tool(3, 1.2, 3)}, p); }
    run("circle r100 T6 so 0.6 cut 6 pd 0.5", {Contour::circle({0, 0}, 100)}, {tool(6, 0.6, 0.5)}, p);
}
