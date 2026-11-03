#include <cstdio>
#include <cmath>
#include <limits>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/path_walk.hpp"
#include "carvetto/toolpath/tab_placement.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::LineSpan;
static void show(const char* n, const TabPlacement& t) { std::printf("%s kept %zu dropped %zu req %zu:", n, t.centres.size(), t.dropped, t.requested.size()); for (size_t i = 0; i < t.centres.size(); ++i) std::printf(" %.12g(o%zu)", t.centres[i], t.origin[i]); std::printf("\n"); }
int main() {
    Contour line; line.append(LineSpan{{0, 0}, {100, 0}});
    const ArcLengthPath lp(line, geometry::Tolerance::defaults());
    show("basic", fitTabs(lp, {29.5, 33.0, 36.0}, 4.0, {30.0}, 3.0));
    // exactly reach from a keep-clear point: reach = 2 + 1e-3
    double f = 50.0, reach = 2.0 + kMinTabGap;
    show("exact-reach slack0", fitTabs(lp, {f + reach}, 4.0, {f}, 3.0, 0.0, 0.0));
    show("exact-reach default", fitTabs(lp, {f + reach}, 4.0, {f}, 3.0));
    show("moved default", fitTabs(lp, {f + 0.3}, 4.0, {f}, 3.0));
    show("moved slack0", fitTabs(lp, {f + 0.3}, 4.0, {f}, 3.0, 0.0, 0.0));
    show("moved margin 1", fitTabs(lp, {f + 0.3}, 4.0, {f}, 3.0, 1.0));
    show("margin NaN", fitTabs(lp, {f + 0.3}, 4.0, {f}, 3.0, std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::quiet_NaN()));
    show("margin huge", fitTabs(lp, {f + 0.3}, 4.0, {f}, 3.0, 1e9));
    show("empty", fitTabs(lp, {}, 4.0, {f}, 3.0));
    show("two keepclear, margin pushes into other", fitTabs(lp, {50.3}, 4.0, {50.0, 54.5}, 3.0, 0.5));
    // slack edge: centre at reach - 0.5e-9 kept by default, dropped at slack 0 (moved by the loop?)
    show("near-reach default", fitTabs(lp, {f + reach - 0.5e-9}, 4.0, {f}, 3.0));
    show("near-reach slack0", fitTabs(lp, {f + reach - 0.5e-9}, 4.0, {f}, 3.0, 0.0, 0.0));
    Contour sq; sq.closed = true; sq.append(LineSpan{{0,0},{40,0}}); sq.append(LineSpan{{40,0},{40,40}}); sq.append(LineSpan{{40,40},{0,40}}); sq.append(LineSpan{{0,40},{0,0}});
    const ArcLengthPath lq(sq, geometry::Tolerance::defaults());
    show("closed", fitTabs(lq, {29.5, 33.0, 36.0, 100.0, 159.0}, 4.0, {30.0}, 3.0));
    TabOptions o; o.enabled = true; o.count = 4; o.length = 2;
    show("placeTabs", placeTabs(lq, 4.0, o, {20.0}, 1.0));
}
