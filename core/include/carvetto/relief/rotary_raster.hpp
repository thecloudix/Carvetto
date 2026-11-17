#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>

#include "carvetto/geometry/cylinder_map.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/io/mesh.hpp"
#include "carvetto/relief/relief.hpp"

// Mesh onto the cylinder of a rotary job (T-0321, ADR-0114): the "Full 3D model" import. The mesh lies along the
// cylinder axis (its X axis for a cylinder along X, its Y axis along Y); every cell of the flat design area (length
// along the axis, distance round the circumference) takes the largest distance from the axis at which the mesh
// surface is met on the radial ray at that position. The "Flat model" import needs no new code: it is the ordinary
// top-down raster (`meshToHeightField`) of the mesh onto the flat design area.

namespace carvetto::relief {

struct RotaryRasterOptions {
    /// The cylinder: `alongX`, `radius` (the circumference of a turn is 2 pi radius) and `flip` are used.
    geometry::CylinderMap map;
    /// Flat design area grid: x is the flat X coordinate, y the flat Y coordinate (whichever is along the axis).
    Grid grid;
    /// The two mesh coordinates across the axis through which the axis passes: (Y, Z) for a cylinder along X, (X, Z)
    /// along Y.
    Point axisPoint;
    /// Mesh coordinate along the axis that lands on flat coordinate `alongStart` (flat X or Y).
    double meshAlongOrigin = 0.0;
    double alongStart = 0.0;
    /// Distance from the axis that becomes height 0. Values below it are clamped to 0 when `discardBelowZero`.
    double zeroRadius = 0.0;
    bool discardBelowZero = true;
    /// Ray tests the work may take (bounds the cost); more is an error.
    std::uint64_t maxRayTests = 400'000'000ull;
    /// Polled once per triangle batch; return true to cancel.
    std::function<bool()> cancel;
};

struct RotaryRaster {
    HeightField field;             ///< Height above `zeroRadius`; transparent where no surface is met.
    std::uint64_t rayTests = 0;    ///< Cell/triangle ray tests made (the operation counter).
    std::size_t hitCells = 0;      ///< Cells that met the surface.
    double maxRadius = 0.0;        ///< Largest distance from the axis of any hit.
};

/// Errors (strings): empty mesh, bad index, non-finite vertex or option, bad grid, "too many ray tests", "cancelled".
/// Deterministic. Per triangle only the cells in its along range and angular range are tested, so the cost is
/// the sum over triangles of those cell counts (a triangle that covers the axis is tested against a whole ring).
geometry::ShapeResult<RotaryRaster> meshToRotaryField(const io::TriangleMesh& mesh, const RotaryRasterOptions& options);

}  // namespace carvetto::relief
