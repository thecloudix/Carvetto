#include <cstdio>
#include "shapes.hpp"
using namespace carvetto::geometry; using vo::dist;
int main() {
    Tolerance tol;
    for (const char* nm : {"rect40x20", "star5", "S_glyph_bez", "C_glyph_arcs", "sq_holes"}) {
        for (double o : {0.0, 1e4, 1e6, 1e8, 1e9, 1e10, 1e11, 1e12}) {
            vs::Shape sh; for (auto& s : vs::all()) if (s.name == nm) sh = s;
            for (auto& c : sh.cs) c.transform(Transform2::translation({o, o * 0.7}));
            MedialAxis ax = medialAxis(sh.cs, tol); vo::Boundary B = vo::makeBoundary(sh.cs, tol.chordal / 4);
            double me = 0, md = 0; size_t n = 0, k = 0;
            for (auto& b : ax.branches) for (auto& p : b.points) { ++n; if (k++ % 5) continue; me = std::max(me, std::fabs(vo::exactDist(B, p.point).d - p.radius)); md = std::max(md, vo::medialDefect2(B, p.point, p.radius, tol.chordal)); }
            std::printf("%-14s offset=%.0e capped=%d samples=%zu pts=%zu radErr=%.3e defect=%.3e\n", nm, o, (int)ax.accuracyCapped, ax.boundarySamples, n, me, md);
        }
    }
}
