#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

#include "carvetto/geometry/shapes.hpp"
#include "carvetto/io/mesh.hpp"
#include "carvetto/relief/relief.hpp"

/// Mesh to height field (T-0286, ADR-0064). Rasterises an oriented triangle mesh top-down.
namespace carvetto::relief {

struct MeshRasterOptions {
    double cellSize = 1.0;           ///< mm, > 0 and finite
    double zeroPlane = 0.0;          ///< mesh Z that becomes height 0
    bool discardBelowZero = true;    ///< heights below the plane are clamped to 0 (data removed)
    bool bothSides = false;          ///< also build the bottom side (mirrored, depth below the plane)
    bool detectUndercuts = true;
    /// Target grid. When empty the grid covers the mesh XY bounds (material fitted to the model):
    /// cells are centred at min + (i + 0.5) * cellSize.
    std::optional<Grid> grid;
    /// Polled once per cell row; return true to cancel.
    std::function<bool()> cancel;
};

struct MeshRaster {
    HeightField top;     ///< highest surface minus zeroPlane; transparent where the mesh has no surface
    HeightField bottom;  ///< bothSides only: zeroPlane minus lowest surface
    /// width*height, 1 where material is hidden below the top surface (row-major, like HeightField).
    std::vector<std::uint8_t> undercut;
    std::size_t undercutCount = 0;
    double maxHeight = 0.0;  ///< largest top height (material thickness above the plane to fit); 0 if none
    double maxDepth = 0.0;   ///< largest bottom depth (bothSides); 0 if none
};

/// Errors (as strings): empty mesh, bad index, non-finite vertex, bad options, grid too large,
/// "cancelled". Deterministic; work is O(triangle-row spans + hits) with a scanline sweep.
geometry::ShapeResult<MeshRaster> meshToHeightField(const io::TriangleMesh& mesh,
                                                    const MeshRasterOptions& options);

}  // namespace carvetto::relief
