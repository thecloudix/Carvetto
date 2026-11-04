#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry;
int main() {
    Tolerance tol;
    for (double w : {0.05, 0.03, 0.022, 0.021, 0.0205, 0.02, 0.019, 0.015, 0.01, 0.002}) {
        double a = 2.5 - w / 2, b = 2.5 + w / 2;
        std::vector<Contour> cs{vs::poly({{0, 0}, {5, 0}, {5, a}, {7, a}, {7, 0}, {12, 0}, {12, 5}, {7, 5}, {7, b}, {5, b}, {5, 5}, {0, 5}})};
        MedialAxis ax = medialAxis(cs, tol);
        // skeleton coverage of the neck: fraction of x in [5,7] that has a skeleton point within w of (x,2.5)
        int cov = 0, N = 200; double rmin = 1e9, rmax = 0;
        for (int i = 0; i <= N; ++i) { double x = 5 + 2.0 * i / N; bool hit = false;
            for (auto& br : ax.branches) for (size_t k = 0; k + 1 < br.points.size(); ++k) { auto& p = br.points[k].point; auto& q = br.points[k + 1].point;
                if ((p.x - x) * (q.x - x) <= 0 && std::fabs(p.y - 2.5) < w && std::fabs(q.y - 2.5) < w) { hit = true; double r = br.points[k].radius; rmin = std::min(rmin, r); rmax = std::max(rmax, r); } }
            cov += hit; }
        std::printf("neck width=%.4f (%.2f c): skeleton covers %5.1f%% of neck length, r in [%.5f, %.5f], branches=%zu capped=%d samples=%zu\n", w, w / tol.chordal, 100.0 * cov / (N + 1), rmin, rmax, ax.branches.size(), (int)ax.accuracyCapped, ax.boundarySamples);
    }
}
