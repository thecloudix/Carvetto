#pragma once

#include <compare>
#include <string>

namespace carvetto::tooling {

/// Feeds and speeds for one tool, valid for one (machine, material) pair or as the tool's default.
/// Lengths are millimetres, feeds mm/min, spindle rev/min. Stepovers are absolute distances;
/// percentage-of-diameter display is a UI concern.
struct CuttingData {
    /// Depth removed per pass (mm).
    double passDepth{0.0};
    /// Lateral step between adjacent passes (mm).
    double stepover{0.0};
    /// Stepover used for the final (finishing) pass (mm). 0 = same as `stepover`.
    double finalPassStepover{0.0};
    /// Stepover for V-bit clearance passes (mm). 0 = same as `stepover`.
    double clearancePassStepover{0.0};
    /// Spindle speed (rev/min).
    double spindleSpeed{0.0};
    /// Feed rate (mm/min).
    double feedRate{0.0};
    /// Plunge rate (mm/min).
    double plungeRate{0.0};
    /// Maximum burn rate for laser tools (mm/min). 0 = not applicable.
    double maxBurnRate{0.0};
    /// Tool number on the machine. Meaningful per machine; the default data carries a fallback.
    int toolNumber{0};

    bool operator==(const CuttingData&) const = default;

    /// Effective final-pass stepover (falls back to `stepover`).
    double effectiveFinalPassStepover() const {
        return finalPassStepover > 0.0 ? finalPassStepover : stepover;
    }
    /// Effective clearance-pass stepover (falls back to `stepover`).
    double effectiveClearancePassStepover() const {
        return clearancePassStepover > 0.0 ? clearancePassStepover : stepover;
    }
};

/// Chip load per flute (mm/tooth) = feed / (spindle · flutes). Returns 0 when undefined.
double chipLoad(const CuttingData& data, int flutes);

/// Identifies which (machine, material) a `CuttingData` entry belongs to. Ids are the stable
/// string ids of `Machine` and `Material`. Ordered so it can key a `std::map` deterministically.
struct CuttingDataKey {
    std::string machineId;
    std::string materialId;

    auto operator<=>(const CuttingDataKey&) const = default;
};

}  // namespace carvetto::tooling
