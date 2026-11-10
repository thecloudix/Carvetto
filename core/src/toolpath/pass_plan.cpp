#include "carvetto/toolpath/pass_plan.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::toolpath {

namespace {

// Appends the depths (below `top`) at which each pass over a stretch of `depth` ends, using
// the plan's step rules. The final entry is always exactly `top + depth`.
void planStretch(const PassPlan& plan, double top, double depth, std::vector<double>& outDepths) {
    if (depth <= 0.0) {
        return;
    }
    // Round-off guards: 12 / 4 must count as exactly 3 passes, not 4. `kEps` applies to the
    // dimensionless pass ratio; `eps` is the same tolerance relative to the stretch, in mm, so the
    // guard scales with the depths (an absolute 1e-9 mm swallowed a whole pass at nanometre pass
    // depths: cut 1e-6 mm in 1e-9 mm passes gave 999 levels, not 1000). The step-slack test is
    // relative to the step. For stretches and steps near 1 mm all three are the old 1e-9 mm.
    constexpr double kEps = 1e-9;
    const double eps = kEps * depth;
    if (plan.passDepth <= 0.0 || plan.passDepth >= depth - eps) {
        outDepths.push_back(top + depth);
        return;
    }
    if (plan.maintainExactStep) {
        // T-0184: range-checked cast (a tiny pass depth). Beyond 1e9 passes nothing can be built anyway.
        const double fullD = std::floor(depth / plan.passDepth + kEps);
        const auto full = static_cast<std::size_t>(fullD >= 1.0 ? std::min(fullD, 1e9) : 0.0);
        for (std::size_t i = 1; i <= full; ++i) {
            const double d = static_cast<double>(i) * plan.passDepth;
            if (d < depth - eps) {
                outDepths.push_back(top + d);
            }
        }
        outDepths.push_back(top + depth);
        return;
    }
    // Equalised passes: prefer fewer passes if the step grows by at most kStepSlack.
    const double nD = std::ceil(depth / plan.passDepth - kEps);
    auto n = static_cast<std::size_t>(nD >= 1.0 ? std::min(nD, 1e9) : 0.0);
    if (n > 1 &&
        depth / static_cast<double>(n - 1) <= plan.passDepth * (1.0 + PassPlan::kStepSlack) * (1.0 + kEps)) {
        --n;
    }
    const double step = depth / static_cast<double>(n);
    for (std::size_t i = 1; i < n; ++i) {
        outDepths.push_back(top + static_cast<double>(i) * step);
    }
    outDepths.push_back(top + depth);
}

}  // namespace

std::vector<double> PassPlan::levels() const {
    std::vector<double> depths;
    if (cutDepth <= 0.0) {
        return {};
    }
    const double start = startDepth;
    if (lastPassThickness > 0.0 && lastPassThickness >= cutDepth) {
        depths.push_back(start + cutDepth);
    } else if (lastPassThickness > 0.0) {
        planStretch(*this, start, cutDepth - lastPassThickness, depths);
        depths.push_back(start + cutDepth);
    } else {
        planStretch(*this, start, cutDepth, depths);
    }
    for (double& d : depths) {
        d = -d;
    }
    // The last level is the exact bottom, free of accumulated rounding.
    depths.back() = bottomZ();
    return depths;
}

}  // namespace carvetto::toolpath
