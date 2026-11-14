#pragma once

#include <vector>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/relief/relief.hpp"

// Modify ops 1 (T-0292, ADR-0068): bake, replace below, reset inside/outside vectors, offset model,
// smooth, emboss. Pure functions on height fields; the caller decides which components feed them
// (bake the selection first) and whether originals are kept. Units are mm. Transparent = NaN.

namespace carvetto::relief {

/// Bakes the components (in order, first against all-transparent, honouring each mode and dynamic
/// properties, hidden ones skipped) into one Add component with neutral properties on the union
/// grid. Does not modify the inputs, so "keep originals" is the caller not deleting them.
Status bakeComponents(const std::vector<Component>& components, Component& out);

/// A neutral Add component (100 %, base 0, no fade/tilt) holding `field`, named/visible like `like`.
Component componentFromField(const Component& like, HeightField field);

enum class ReplaceWith { Transparent, Plane };

struct ReplaceBelowParams {
    double threshold = 0.0;   ///< Cells strictly below this height are replaced.
    ReplaceWith with = ReplaceWith::Transparent;
    double planeHeight = 0.0; ///< Height given to replaced cells for Plane.
    double lowerBy = 0.0;     ///< Subtracted from every opaque cell after the replacement (planes included).
};
Status replaceBelow(const HeightField& in, const ReplaceBelowParams& p, HeightField& out);

enum class ResetRegion { Inside, Outside };

/// Makes cells transparent whose centre is inside (even-odd) or outside the closed vectors.
/// BadParameter for an open/short/non-finite vector or more than 65536 segments in total.
Status resetByVectors(const HeightField& in, const std::vector<geometry::Polyline>& vectors, ResetRegion region,
                      HeightField& out);

struct OffsetParams {
    double distance = 0.0;      ///< > 0 grows the model (ball dilation), < 0 shrinks it (ball erosion).
    bool clipToZero = false;    ///< Heights below 0 become 0 afterwards.
};
Status offsetModel(const HeightField& in, const OffsetParams& p, HeightField& out);

struct SmoothParams {
    double percent = 0.0;               ///< 0..100; Gaussian sigma = percent/100 * radiusMm.
    double radiusMm = 2.0;              ///< Sigma at 100 %.
    bool preserveTransparency = true;
};
Status smooth(const HeightField& in, const SmoothParams& p, HeightField& out);

struct EmbossParams {
    double scale = 1.0;                  ///< >= 0, height scale of the broad shape.
    double detailSmoothnessPercent = 50; ///< 0..100: how coarse the broad shape is.
    double radiusMm = 2.0;
};
/// Result = in - (1 - scale) * smooth(in): scales the broad shape, keeps the detail.
Status emboss(const HeightField& in, const EmbossParams& p, HeightField& out);

}  // namespace carvetto::relief
