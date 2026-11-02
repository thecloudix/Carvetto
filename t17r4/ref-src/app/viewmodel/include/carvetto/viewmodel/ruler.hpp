#pragma once

#include <string>
#include <vector>

#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

namespace carvetto::viewmodel {

/// Major ruler step `mantissa × 10^exponent` in the display unit, with `mantissa` ∈ {1, 2, 5}.
/// Keeping the step as integers lets labels be produced exactly (no 0.30000000000000004).
struct RulerStep {
    int mantissa{1};
    int exponent{0};

    bool operator==(const RulerStep&) const = default;

    /// Step length in the display unit.
    double value() const;
    /// Minor subdivisions per major step (10 for 1×, 4 for 2×, 5 for 5×).
    int minorDivisions() const;
};

/// Smallest 1/2/5×10ⁿ step whose on-screen length is at least `minSpacingPx`, given how many
/// pixels one display unit covers. Successive candidates differ by at most 2.5×, so the chosen
/// spacing is below 2.5 × `minSpacingPx` (125 px for the default 50 px).
RulerStep chooseRulerStep(double pxPerDisplayUnit, double minSpacingPx = 50.0);

/// Exact decimal text of `units × 10^exponent` ("0.3", "-12.5", "2000", "0").
std::string formatScaledDecimal(long long units, int exponent);

/// Which screen axis a ruler measures.
enum class RulerOrientation { Horizontal, Vertical };

/// One tick on a ruler. `positionPx` is along the ruler (x for horizontal, y for vertical).
struct RulerTick {
    double positionPx{0.0};
    bool major{false};
    /// Label in the display unit; empty for minor ticks.
    std::string label;
};

/// Ruler contents for the current view.
struct RulerModel {
    RulerStep step;
    std::vector<RulerTick> ticks;
};

/// Ticks for the visible extent of one axis of `view`, measuring document coordinates in
/// `units`. Majors fall on integer multiples of the step and carry exact labels; minors divide
/// each major step evenly. Returns no ticks for an empty viewport or an absurd pan distance.
RulerModel rulerTicks(const ViewTransform& view, RulerOrientation orientation, document::Units units,
                      double minSpacingPx = 50.0);

}  // namespace carvetto::viewmodel
