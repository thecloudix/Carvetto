// Medial-axis cost and result on inputs aimed at the Gabriel-disc search (exempt samples near flat corners):
// thin bands with near-flat corners and dense vertices, comb of near-flat corners. Prints time, samples, branches, capped.
#include <chrono>
#include <cstdio>
#include <cmath>
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
using namespace carvetto::geometry;
static Contour poly(const std::vector<Vec2>& p) { return Contour::fromPolyline(Polyline{p, true}); }
int main() {
    Tolerance tol; tol.chordal = 0.01 / 3; tol.distance = std::min(tol.distance, tol.chordal);
    for (int n : {100, 1000, 4000, 16000}) for (double w : {0.02, 0.2}) for (double angDeg : {0.05, 0.5}) {
        double L = 30, a = angDeg * M_PI / 180; std::vector<Vec2> p;
        for (int i = 0; i <= n; ++i) { double t = double(i) / n; double y = (t < 0.5 ? t : 1 - t) * L * std::tan(a); p.push_back({L * t, y}); } // near-flat roof, dense
        for (int i = n; i >= 0; --i) { double t = double(i) / n; double y = (t < 0.5 ? t : 1 - t) * L * std::tan(a); p.push_back({L * t, y + w}); }
        p.erase(p.begin() + n + 1); // the two end vertices are shared: keep a simple polygon
        auto t0 = std::chrono::steady_clock::now(); MedialAxis ax = medialAxis({poly(p)}, tol); double dt = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        double rmax = 0; size_t pts = 0; for (auto& b : ax.branches) for (auto& q : b.points) { rmax = std::max(rmax, q.radius); ++pts; }
        std::printf("band n=%5d w=%.2f ang=%.2f: %.3fs samples=%zu branches=%zu pts=%zu rmax=%.5f (w/2=%.5f) capped=%d ign=%zu\n", n, w, angDeg, dt, ax.boundarySamples, ax.branches.size(), pts, rmax, w / 2, (int)ax.accuracyCapped, ax.ignoredContours);
    }
}
