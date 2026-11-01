// T-0125 verifier repro: a genuine centre-region lobe (~76 mm^2, ~3 mm deeper than the tool needs)
// is discarded because two Clipper spike vertices overshoot the slack by 5.4e-3 > one join tolerance.
#include <cstdio>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto;
int main() {
    std::vector<geometry::Vec2> pts; FILE* f = fopen("/tmp/claude-0/-home-claude/dd097e91-8a7e-530c-818b-244e9548b908/scratchpad/t0125v/spike_poly.txt", "r");
    double x, y; while (fscanf(f, "%lf %lf", &x, &y) == 2) pts.push_back({x, y}); fclose(f);
    std::vector<geometry::Contour> in{geometry::Contour::fromPolyline(geometry::Polyline{pts, true})};
    for (double d : {5.0, 5.04, 5.0673, 5.09, 5.10}) {
        toolpath::PocketParams p; p.cutDepth = 1;
        std::vector<std::string> msgs;
        auto C = toolpath::pocketCentreRegion(in, d, p, &msgs);
        // (17.3083, 1.5044) lies 5.494 mm from every wall: a tool of d <= 10.9 fits there.
        bool inside = false;
        for (auto& c : C) { auto pl = c.polygonise(geometry::Tolerance::defaults()).points;
            for (size_t i = 0, n = pl.size(), j = n - 1; i < n; j = i++) { auto a = pl[i], b = pl[j];
                if ((a.y > 1.5044) != (b.y > 1.5044) && 17.3083 < (b.x - a.x) * (1.5044 - a.y) / (b.y - a.y) + a.x) inside = !inside; } }
        printf("d=%.4f parts=%zu point(17.31,1.50) in centre region: %s  msgs=%zu %s\n", d, C.size(), inside ? "yes" : "NO", msgs.size(), msgs.empty() ? "" : msgs[0].c_str());
    }
}
