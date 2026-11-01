#define V3_NO_MAIN
#include "v3.cpp"
// Dumbbell / round-neck pinch scan: neck width ~ tool diameter + 2*margin. Each lobe that can hold
// the tool must get cut.
int main() {
    int lost = 0, n = 0;
    for (int shape = 0; shape < 3; ++shape)
    for (double w = 6.000; w <= 6.060; w += 0.0005) {
        Scene s;
        if (shape == 0) s.addPoly({{0, 0}, {25, 0}, {25, 15 - w / 2}, {40, 15 - w / 2}, {40, 5}, {60, 5}, {60, 25}, {40, 25}, {40, 15 + w / 2}, {25, 15 + w / 2}, {25, 30}, {0, 30}});
        else if (shape == 1) { // two circles joined at a waist made by circle arcs: polygonal hourglass
            std::vector<Vec2> top, p; int N = 400; for (int i = 0; i <= N; ++i) { double x = -20 + 40.0 * i / N; double h = std::max({std::sqrt(std::max(0.0, 100 - (x + 10) * (x + 10))), std::sqrt(std::max(0.0, 100 - (x - 10) * (x - 10))), 0.0}); h = std::max(h, w / 2); top.push_back({x, h}); }
            for (auto& t : top) p.push_back({t.x, -t.y}); for (auto it = top.rbegin(); it != top.rend(); ++it) p.push_back(*it);
            std::vector<Vec2> q; for (auto v : p) if (q.empty() || std::hypot(v.x - q.back().x, v.y - q.back().y) > 1e-9) q.push_back(v); if (std::hypot(q.front().x - q.back().x, q.front().y - q.back().y) < 1e-9) q.pop_back(); s.addPoly(q); }
        else { // rect with two islands leaving a gap of width w
            s.addPoly(rectPts({0, 0}, {60, 30})); s.addPoly(rectPts({28, 2}, {32, 15 - w / 2})); s.addPoly(rectPts({28, 15 + w / 2}, {32, 28})); s.addPoly(rectPts({0, 0}, {60, 30})); s.contours.pop_back(); s.prims.resize(s.prims.size() - 4); }
        PocketParams p; p.cutDepth = 1; p.safeZ = 5;
        auto r = pocketToolpath(s.contours, {tool(6, 2.4, 1)}, p);
        ++n;
        // probe centres of both lobes
        std::vector<Vec2> probes = shape == 0 ? std::vector<Vec2>{{12, 15}, {50, 15}} : shape == 1 ? std::vector<Vec2>{{-10, 0}, {10, 0}} : std::vector<Vec2>{{10, 15}, {50, 15}};
        for (auto q : probes) { double dmin = 1e9; for (auto& pa : r.plans[0].paths) for (size_t k = 0; k < pa.path.points.size(); ++k) dmin = std::min(dmin, segDist(q, pa.path.points[k], pa.path.points[(k + 1) % pa.path.points.size()]));
            if (dmin > 3.5) { ++lost; printf("shape %d w=%.4f: lobe at (%.0f,%.0f) not cut (nearest path %.2f), centre parts %zu msgs %zu\n", shape, w, q.x, q.y, dmin, r.plans[0].centreRegion.size(), r.messages.size()); } }
    }
    printf("pinch: %d runs, %d lobes lost\n", n, lost);
}
