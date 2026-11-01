#pragma once

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"

// Shared toolpath machinery (T-0120, ADR-0016): lead-in / lead-out geometry that joins a cut path
// tangentially (circular) or at an angle (straight) from the waste side.

namespace carvetto::toolpath {

/// Shape of the lead (feature-inventory §5 "Leads").
enum class LeadType {
    None,
    Straight,  ///< A line of `length` meeting the path at `angle` to its direction.
    Circular,  ///< A tangent arc of `radius` sweeping `sweep`.
};

/// Lead options. Lengths in mm, angles in radians.
struct LeadOptions {
    LeadType type{LeadType::None};
    /// Straight lead: length of the line.
    double length{5.0};
    /// Straight lead: angle between the lead and the path direction, in (0, pi/2]; 0 = tangent.
    double angle{geometry::kPi / 4.0};
    /// Circular lead: arc radius.
    double radius{5.0};
    /// Circular lead: swept angle, 0.1°…90° in the reference behaviour.
    double sweep{geometry::kPi / 2.0};
    /// Also add a lead-out (mirror image of the lead-in) at the end of the cut.
    bool leadOut{false};
    /// Closed paths: distance cut past the start point before leaving (lead-out after it).
    double overcut{0.0};
    bool operator==(const LeadOptions&) const = default;
};

/// Lead-in that ends at `at`, arriving with unit direction `tangent`, approaching from the
/// waste side (`wasteOnLeft` relative to the direction of travel). `scale` multiplies the
/// length/radius (used to shorten a lead that would gouge). Empty for `LeadType::None`.
geometry::Contour leadIn(geometry::Vec2 at, geometry::Vec2 tangent, bool wasteOnLeft, const LeadOptions& options,
                         double scale = 1.0);

/// Lead-out that starts at `at`, leaving with unit direction `tangent`, moving into the waste
/// side. Mirror image of `leadIn`. Empty for `LeadType::None`.
geometry::Contour leadOut(geometry::Vec2 at, geometry::Vec2 tangent, bool wasteOnLeft, const LeadOptions& options,
                          double scale = 1.0);

}  // namespace carvetto::toolpath
