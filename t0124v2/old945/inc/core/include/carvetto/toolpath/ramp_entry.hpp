#pragma once

#include <vector>

#include "carvetto/toolpath/path_walk.hpp"

// Shared toolpath machinery (T-0120, ADR-0016): stepping down to a new pass level along the cut
// path instead of plunging (smooth, zig-zag and spiral ramps).

namespace carvetto::toolpath {

/// How the tool descends to each pass level (feature-inventory §5 "Ramp").
enum class RampType {
    None,    ///< Plunge vertically at the pass start.
    Smooth,  ///< Descend linearly along the path over `distance` (or at `angle`).
    ZigZag,  ///< Walk back and forth over `distance` while descending; returns to the start.
    Spiral,  ///< Closed paths: descend one pass depth per revolution (open paths: Smooth).
};

/// Ramp options. Lengths in mm, angles in radians (from horizontal).
struct RampOptions {
    RampType type{RampType::None};
    /// Smooth: ramp length; ZigZag: length of each leg.
    double distance{10.0};
    /// Smooth: derive the length from `angle` instead of `distance`. ZigZag: limit each leg's
    /// descent to `distance` · tan(`angle`), adding back-and-forth legs as needed.
    bool useAngle{false};
    double angle{0.0872664625997165};  // 5 degrees
    /// Descend along the lead-in on the first pass, when there is one (Smooth/ZigZag/Spiral).
    bool onLeadIn{false};
    bool operator==(const RampOptions&) const = default;
};

/// One traversal of the path from `u0` to `u1` with a height profile in travelled distance.
struct PathLeg {
    double u0{0.0};
    double u1{0.0};
    ZProfile z;
};

/// The legs of one pass that starts at path position `u` with the tool at `zFrom` and cuts at
/// `zTo`, and where the tool ends up.
struct PassLegs {
    std::vector<PathLeg> legs;
    double endU{0.0};
};

/// Plans one pass.
///
/// Closed path of length P (always walked forwards):
/// - None: one revolution at `zTo`.
/// - Smooth (length L ≤ P): from u, descend over L and keep cutting until L past the start, i.e.
///   P + L in all, so the ramped stretch is re-cut at full depth; ends at u + L.
/// - ZigZag: back-and-forth legs over [u, u + L] down to `zTo`, ending at u; then one revolution.
/// - Spiral: one revolution descending linearly from `zFrom` to `zTo`; ends at u. The caller adds
///   a final flat revolution after the last level.
/// Open path: the pass starts at the end `u` (0 or length) and runs to the other end. Smooth and
/// ZigZag ramp over the first L, return to the start at `zTo`, then cut the whole path (so no
/// ramped stretch is left above the level); Spiral behaves as Smooth.
/// No descent (`zFrom` ≤ `zTo`) behaves as None.
PassLegs planPass(const ArcLengthPath& path, double u, double zFrom, double zTo, const RampOptions& ramp);

/// Length over which a Smooth ramp descends `drop` mm (`distance`, or drop / tan(angle)).
double smoothRampLength(const RampOptions& ramp, double drop);

}  // namespace carvetto::toolpath
