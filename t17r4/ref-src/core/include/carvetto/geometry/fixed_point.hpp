#pragma once

namespace carvetto::geometry {

/// Scale applied when converting millimetre coordinates to the 64-bit integer grid used by
/// Clipper2 for offsets and booleans: one integer unit is one nanometre. Rounding error on
/// the way in is therefore 0.5e-6 mm, three orders of magnitude below the default distance
/// tolerance (0.001 mm), and the usable coordinate range is about ±2.3e12 mm. See ADR-0006.
inline constexpr double kFixedPointScale = 1.0e6;

}  // namespace carvetto::geometry
