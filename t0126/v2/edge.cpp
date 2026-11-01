#include <cmath>
#include <cstdio>
#include <limits>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/pocket.hpp"
using namespace carvetto; using geometry::Contour;
static void t(const char* nm, const std::vector<Contour>& in, double d, toolpath::PocketParams P) {
    std::vector<std::string> ms; auto C = toolpath::pocketCentreRegion(in, d, P, &ms);
    printf("%-28s parts %zu msgs %zu %s\n", nm, C.size(), ms.size(), ms.empty() ? "" : ms[0].c_str());
}
int main() {
    toolpath::PocketParams P; auto sq = Contour::rectangle({{0, 0}, {20, 10}});
    t("ok 6mm", {sq}, 6, P);
    t("exact fit 10-2c", {sq}, 10 - 0.02, P);
    t("too big 10", {sq}, 10, P);
    t("d=0", {sq}, 0, P); t("d<0", {sq}, -1, P); t("d=nan", {sq}, std::nan(""), P); t("d=inf", {sq}, INFINITY, P);
    t("empty input", {}, 3, P);
    geometry::Contour open = sq; open.closed = false; t("open only", {open}, 3, P);
    { auto Q = P; Q.allowance = -0.1; t("allow<0", {sq}, 3, Q); }
    { auto Q = P; Q.allowance = 4.9; t("allow 4.9", {sq}, 0.1, Q); }
    { auto Q = P; Q.tolerance.chordal = 1e-5; t("chordal 1e-5", {sq}, 3, Q); }
    { auto Q = P; Q.tolerance.chordal = 2; t("chordal 2", {sq}, 3, Q); }
    { auto Q = P; Q.tolerance.chordal = 1e-4; t("circle r=1e4 chordal 1e-4 (cap)", {Contour::circle({0,0},1e4)}, 3, Q); }
    t("tiny pocket 1e-3", {Contour::rectangle({{0,0},{1e-3,1e-3}})}, 1e-4, P);
    t("huge coords", {Contour::rectangle({{1e6,1e6},{1e6+20,1e6+10}})}, 6, P);
}
