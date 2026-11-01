#include <cstdio>
#include "carvetto/geometry/intersect.hpp"
using namespace carvetto::geometry;
int main() {
    ArcSpan a{{-6308.8745429246874, 31581.569216038675}, 39785.217135614701, -1.5706767134589792, -1.5706742410393422, ArcDirection::CCW};
    ArcSpan b{{-6304.1211345246902, -8203.558166433495}, 0.089633408917572641, 0.10399344190285414, -1.5101329545047752, ArcDirection::CW};
    for (double t : {1e-3, 1e-9, 1e-12}) {
        Tolerance tol{t, 1e-9, 0.01};
        auto r = intersect(b, a, tol);
        std::printf("tol %g: %zu hits\n", t, r.points.size());
        for (auto& h : r.points) std::printf("  (%.17g, %.17g) tA %.17g tB %.17g tangent %d\n", h.point.x, h.point.y, h.tA, h.tB, h.tangent);
    }
}
