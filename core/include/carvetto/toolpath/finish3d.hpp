#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/rough3d.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// 3D finishing of a composed relief model (T-0297, ADR-0072; feature-inventory "3D Finish Toolpath").
// Paths are polylines over the model whose tip height is the *exact* drop height of every straight
// piece: for a piece `ab` the lowest constant tip Z at which the tool, swept from a to b, cuts no model
// cell centre below `model + allowance` (computed with the tool's exact profile). So no piece of any
// path gouges at the grid, whatever the point spacing; the spacing and `tolerance` only control how much
// stock a piece may leave above the drop surface (ADR-0072).

namespace carvetto::toolpath {

enum class Finish3dStrategy {
    /// Parallel lines at `rasterAngle`, `stepover` apart in plan.
    Raster,
    /// Contour-parallel lines at equal distance along the tool-tip surface from the region edge, so the
    /// scallop between lines is constant ("constant scallop"). Distances are found on the grid (16
    /// neighbours), so they are within about 3 % of the true surface distance.
    Offset,
};

/// A toolpath already planned (usually the roughing); the finish only cuts where these left material.
struct Finish3dPrior {
    Toolpath toolpath;
    Rough3dTool tool;
};

struct Finish3dParams {
    std::string name{"3D Finish"};
    Finish3dStrategy strategy{Finish3dStrategy::Raster};

    /// Added to every model height to give machine Z.
    double zOffset{0.0};
    /// Z of the top of the material and the lowest Z the tool tip may reach (the floor).
    double stockTop{0.0};
    double stockBottom{-20.0};
    /// Material left on the model (>= 0), measured vertically at grid nodes.
    double allowance{0.0};
    /// Height of rapid moves; must be above `stockTop`.
    double safeZ{5.0};

    Rough3dBoundary boundary{Rough3dBoundary::Material};
    double boundaryOffset{0.0};
    std::vector<geometry::Polyline> vectors;

    /// Raster: direction of the lines, radians from +X.
    double rasterAngle{0.0};
    RasterTravel travel{RasterTravel::ZigZag};
    /// Climb: the uncut material stays on the right of the travel direction (Raster: lines step towards
    /// the right of the travel direction; Offset: the higher-distance side, i.e. the region's inside, is
    /// on the right). Conventional is the opposite.
    CutDirection direction{CutDirection::Climb};
    /// Raster: reverses the order of the lines. Offset: cuts from the inside out instead of outside in.
    bool reverseSteps{false};

    /// Largest stock a path piece may leave above the drop surface at its ends and middle; a piece
    /// that would leave more is split (down to 1/32 of the cell size). Must be > 0.
    double tolerance{0.01};
    /// Rest tools cut only where the stock left is more than this above what they can reach.
    double restTolerance{0.01};
    /// Earlier toolpaths whose result (by simulation) is the starting stock. With priors even the
    /// first tool is a rest tool.
    std::vector<Finish3dPrior> priors;

    bool operator==(const Finish3dParams&) const = default;
};

using Finish3dResult = Rough3dResult;

/// Largest vertical-free scallop height a ball of radius `toolRadius` leaves between two parallel
/// passes `spacing` apart on a plane perpendicular to the passes (spacing measured on the surface):
/// r - sqrt(r^2 - (spacing/2)^2). A spacing beyond the diameter returns `toolRadius`.
double ballScallopHeight(double toolRadius, double spacing);

/// Finishes `model` with the tool list; cutting data supplies the stepover (limited to the tool
/// diameter, with a `StepoverLimited` diagnostic). Invalid input gives a diagnostic and no moves.
Finish3dResult finish3dToolpath(const relief::HeightField& model, const std::vector<Rough3dTool>& tools,
                                const Finish3dParams& params, const ToolpathRunControl& control = {});

}  // namespace carvetto::toolpath
