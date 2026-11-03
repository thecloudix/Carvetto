#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/path_walk.hpp"
#include "carvetto/toolpath/tab_placement.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::LineSpan;
int main() {
    Contour line; line.append(LineSpan{{0, 0}, {100, 0}});
    const ArcLengthPath lp(line, geometry::Tolerance::defaults());
    auto tp = fitTabs(lp, {29.5, 33.0, 36.0}, 4.0, {30.0}, 3.0);
    std::printf("kept %zu dropped %zu:", tp.centres.size(), tp.dropped); for (double c : tp.centres) std::printf(" %.4f", c); std::printf("\n");
    Contour sq; sq.closed = true; sq.append(LineSpan{{0,0},{40,0}}); sq.append(LineSpan{{40,0},{40,40}}); sq.append(LineSpan{{40,40},{0,40}}); sq.append(LineSpan{{0,40},{0,0}});
    const ArcLengthPath lq(sq, geometry::Tolerance::defaults());
    auto t2 = fitTabs(lq, {29.5, 33.0, 36.0, 100.0}, 4.0, {30.0}, 3.0);
    std::printf("closed kept %zu dropped %zu:", t2.centres.size(), t2.dropped); for (double c : t2.centres) std::printf(" %.4f", c); std::printf("\n");
}
