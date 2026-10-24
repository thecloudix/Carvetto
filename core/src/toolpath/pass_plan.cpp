#include "carvetto/toolpath/pass_plan.hpp"

#include <cmath>

namespace carvetto::toolpath {

namespace {

// Appends the depths (below `top`) at which each pass over a stretch of `depth` ends, using
// the plan's step rules. The final entry is always exactly `top + depth`.
void planStretch(const PassPlan& plan, double top, double depth, std::vector<double>& outDepths) {
    if (depth <= 0.0) {
        return;
    }
    // Round-off guard: 12 / 4 must count as exactly 3 passes, not 4.
    constexpr double kEps = 1e-9;
    if (plan.passDepth <= 0.0 || plan.passDepth >= depth - kEps) {
        outDepths.push_back(top + depth);
        return;
    }
    if (plan.maintainExactStep) {
        const auto full = static_cast<std::size_t>(std::floor(depth / plan.passDepth + kEps));
        for (std::size_t i = 1; i <= full; ++i) {
            const double d = static_cast<double>(i) * plan.passDepth;
            if (d < depth - kEps) {
                outDepths.push_back(top + d);
            }
        }
        outDepths.push_back(top + depth);
        return;
    }
    // Equalised passes: prefer fewer passes if the step grows by at most kStepSlack.
    auto n = static_cast<std::size_t>(std::ceil(depth / plan.passDepth - kEps));
    if (n > 1 && depth / static_cast<double>(n - 1) <= plan.passDepth * (1.0 + PassPlan::kStepSlack) + kEps) {
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
