#pragma once

#include <atomic>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/relief/relief.hpp"

// Create Shape (T-0283, ADR-0062): a component from closed vectors (even-odd fill, holes respected).
// The surface height follows the distance d from a cell to the nearest vector edge. Units are mm.

namespace carvetto::relief {

enum class ShapeKind { Plane, Round, Angled, Smooth, Custom };

enum class ShapeStatus {
    Ok,
    BadGrid,        ///< Invalid or oversized grid (cell size, explicit grid).
    NoVectors,      ///< No vectors given for a kind that needs them, or no cell lies inside them.
    BadVector,      ///< Open polyline, fewer than 3 vertices or a non-finite coordinate.
    BadParameter,   ///< Angle, limit, smoothing, profile, scale or base out of range / not finite.
    TooMuchInput,   ///< Too many polygons, segments or row crossings for the bounded work.
    Cancelled,      ///< The cancel flag was raised; no result.
};

const char* shapeStatusMessage(ShapeStatus s);

/// One point of a custom profile: depth u = d / dMax in [0,1] to height factor h (raw height h * dMax).
struct ProfilePoint {
    double u = 0.0;
    double h = 0.0;
};

/// Everything needed to (re)generate a shape; kept with the component so it can be re-edited.
struct ShapeParams {
    ShapeKind kind = ShapeKind::Angled;
    double angleRad = 0.7853981633974483;  ///< Edge angle: Angled in (0, pi/2), Round in (0, pi/2].
    bool limitEnabled = false;
    double limit = 0.0;                    ///< Height cap on the raw profile (mm), > 0.
    bool preserveInternalCorners = false;  ///< Mitred (sharp) distance at reflex vertices.
    bool blendToInnerVectors = false;      ///< Smooth/Custom: holes are not edges of the profile.
    double smoothing = 0.0;                ///< Smooth: -1 concave .. 0 linear .. +1 convex.
    std::vector<ProfilePoint> profile;     ///< Custom: u strictly ascending, at least two points.
    double scalePercent = 100.0;           ///< Component shape height %, > 0.
    double baseHeight = 0.0;
    Fade fade;
    Tilt tilt;
    CombineMode mode = CombineMode::Add;
    std::string name;
};

/// A shape = its parameters + the vectors + where to rasterise. `grid` empty: the vector bounds
/// plus a one-cell margin at `cellSize`.
struct Shape {
    ShapeParams params;
    std::vector<geometry::Polyline> vectors;  ///< Closed polygons; nesting parity gives holes.
    double cellSize = 1.0;
    std::optional<Grid> grid;
};

/// Builds the component (field + dynamic properties from the params). Deterministic. `cancel`
/// (may be null) is polled once per row; when it reads true the call returns Cancelled.
/// Cells outside the vectors (and inside holes) are transparent. Plane: height 0 inside.
ShapeStatus buildShape(const Shape& shape, const std::atomic<bool>* cancel, Component& out);

enum class TotalHeightAdjust { Scale, Base };

/// Total height = base + rawMax * scalePercent / 100. Pure form on the three numbers.
double totalHeight(double rawMax, double scalePercent, double baseHeight);
/// Sets the total height by changing the scale (base kept) or the base (scale kept).
/// BadParameter for non-finite input, or rawMax <= 0 / total <= base when adjusting the scale.
ShapeStatus setTotalHeight(double rawMax, double total, TotalHeightAdjust adjust, double& scalePercent,
                           double& baseHeight);

/// Same on a component: rawMax is the stored field's maximum (BadParameter if all transparent).
double totalHeight(const Component& c);
ShapeStatus setTotalHeight(Component& c, double total, TotalHeightAdjust adjust);

}  // namespace carvetto::relief
