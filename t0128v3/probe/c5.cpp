// Ratio (dist x sine / unit) of the real cusp-lens crossing of fam5 0xc500076c in origin/main's output (main does not cut it).
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <vector>
#include <variant>
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/intersect.hpp"
using namespace carvetto::geometry;
#include "sxcount.inc"
int main() {
    Contour in;
    in.spans.emplace_back(LineSpan{{9973.3181749575033, 4287.5970899958429}, {9978.5020255591153, 4295.935463921991}});
    in.spans.emplace_back(LineSpan{{9978.5020255591153, 4295.935463921991}, {9973.3181749575033, 4287.8970899958431}});
    in.spans.emplace_back(LineSpan{{9973.3181749575033, 4287.8970899958431}, {9973.3181749575033, 4280.3467060387984}});
    in.spans.emplace_back(LineSpan{{9973.3181749575033, 4280.3467060387984}, {9990.4961596835365, 4280.3467060387984}});
    in.spans.emplace_back(LineSpan{{9990.4961596835365, 4280.3467060387984}, {9990.4961596835365, 4288.0707740670941}});
    in.spans.emplace_back(LineSpan{{9990.4961596835365, 4288.0707740670941}, {9985.0524095126821, 4295.935463921991}});
    in.spans.emplace_back(LineSpan{{9985.0524095126821, 4295.935463921991}, {9990.4961596835365, 4287.770774067094}});
    const double d = 3.2751919785224555;
    Tolerance tol{};
    auto chains = offsetOpenContour(in, d, OffsetSide::Right, JoinType::Round, tol, 2.3698544420767025);
    SX r; sxCount(chains, r, true);
    std::printf("chains %zu sxNew %ld sxOld %ld maxNoise %.3g minReal %.4g\n", chains.size(), r.sxNew, r.sxOld, r.maxNoise, r.maxRealLow);
}
