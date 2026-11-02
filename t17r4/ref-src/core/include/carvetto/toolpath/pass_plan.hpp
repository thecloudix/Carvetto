#pragma once

#include <cstddef>
#include <vector>

namespace carvetto::toolpath {

/// Splits a cut into depth passes. Depths are positive distances measured downward from the
/// material surface (Z = 0); the resulting levels are the Z values of each pass, negative
/// downward (feature-inventory §5 "Pass Depth Control").
///
/// Rules:
/// - The cut runs from `startDepth` to `startDepth + cutDepth`; the last level is always exactly
///   the bottom of the cut.
/// - `maintainExactStep` true: every pass steps exactly `passDepth`, and a shorter final pass
///   takes the remainder (0 → 12 by 5 gives −5, −10, −12).
/// - `maintainExactStep` false: the passes are equalised. The planner first takes the smallest
///   pass count whose step does not exceed `passDepth` by more than `kStepSlack` (15 %); when
///   even that over-steps it uses one more pass and shrinks the step instead (12 by 5 gives
///   three passes of 4 mm; 11 by 5 gives two passes of 5.5 mm).
/// - `lastPassThickness` > 0 reserves exactly that depth for the final pass; the passes above
///   it are planned over the remaining depth with the rules above. A thickness ≥ `cutDepth`
///   yields a single pass.
/// - A `passDepth` ≥ `cutDepth`, or a non-positive `passDepth`, yields a single pass to the
///   bottom. A non-positive `cutDepth` yields no passes.
struct PassPlan {
    /// Depth below the surface at which cutting starts (≥ 0).
    double startDepth{0.0};
    /// Total depth to remove below `startDepth` (> 0 for any pass to be produced).
    double cutDepth{0.0};
    /// Nominal depth of each pass (> 0).
    double passDepth{0.0};
    /// Keep the step exactly `passDepth` (final pass takes the remainder) instead of equalising.
    bool maintainExactStep{true};
    /// Depth of the final pass when > 0 ("Set Last Pass Thickness"); 0 disables.
    double lastPassThickness{0.0};

    /// Largest relative increase of the step allowed when equalising passes.
    static constexpr double kStepSlack = 0.15;

    bool operator==(const PassPlan&) const = default;

    /// Z level of each pass, from the first (shallowest) to the last (deepest), each ≤ −startDepth.
    std::vector<double> levels() const;
    /// Number of passes `levels()` would produce.
    std::size_t passCount() const { return levels().size(); }
    /// Z of the bottom of the cut: −(startDepth + cutDepth).
    double bottomZ() const { return -(startDepth + cutDepth); }
};

}  // namespace carvetto::toolpath
