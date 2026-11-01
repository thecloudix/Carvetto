#include <cmath>
#include <cstdio>
#include <exception>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
int main() {
    double R1 = 12, R2 = 9, D = 2.101, x0 = (D * D + R1 * R1 - R2 * R2) / (2 * D), y0 = std::sqrt(R1 * R1 - x0 * x0);
    double a1 = std::atan2(y0, x0), b0 = std::atan2(-y0, x0 - D), b1 = std::atan2(y0, x0 - D);
    printf("x0 %g y0 %g a1 %g\n", x0, y0, a1);
    Contour c; c.append(ArcSpan{{0, 0}, R1, a1, 2 * M_PI - a1, ArcDirection::CCW});
    double e = b1; int guard = 0; while (e <= b0 && guard++ < 10) e += 2 * M_PI;
    c.append(ArcSpan{{D, 0}, R2, b0, e, ArcDirection::CCW}); c.closed = true;
    PocketTool t; t.tool.name = "T"; t.tool.diameter = 6; t.stepover = 2.4; t.passDepth = 2;
    PocketParams p; p.cutDepth = 4;
    try { auto r = pocketToolpath({c}, {t}, p); printf("moves %zu msgs %zu %s\n", r.toolpaths[0].moves.size(), r.messages.size(), r.messages.empty() ? "" : r.messages[0].c_str()); }
    catch (std::exception& ex) { printf("THROWS %s\n", ex.what()); }
    try { std::vector<std::string> m; auto C = pocketCentreRegion({c}, 6, p, &m); printf("centre parts %zu msgs %zu\n", C.size(), m.size()); }
    catch (std::exception& ex) { printf("centre THROWS %s\n", ex.what()); }
}
