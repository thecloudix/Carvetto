#pragma once

#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Toolpath tiling (T-0242, ADR-0043): splits toolpaths that are larger than the machine's work
// area into tiles, one move list per tile. Cut moves are clipped to the tile rectangle (lines and
// arcs are cut at the rectangle edges, Z interpolated); pieces are re-linked inside the tile with
// a retract to safe Z, a rapid and a plunge. Rapids of the source are dropped.

namespace carvetto::toolpath {

enum class TileStrategy {
    Individual,    ///< A grid of width x height tiles.
    FeedThroughX,  ///< Strips `width` wide along X, the full job extent in Y (material fed in X).
    FeedThroughY,  ///< Strips `height` high along Y, the full job extent in X.
};

struct TileSpec {
    TileStrategy strategy{TileStrategy::Individual};
    /// Tile size in mm (the machine work area, usually). `height` is unused for FeedThroughX,
    /// `width` for FeedThroughY.
    double width{0.0};
    double height{0.0};
    /// Overlap of neighbouring tiles in mm, 0 <= overlap < tile size. Cuts in the overlap are
    /// part of both tiles.
    double overlap{0.0};
    /// Order the tiles by ascending area (stable) instead of row by row.
    bool smallestFirst{false};
    /// Keep the original machine coordinates. When false the tile's minimum corner becomes X0 Y0.
    bool originalPosition{false};
    /// Safe Z for retract/rapid links; empty = highest Z of the toolpaths.
    std::optional<double> safeZ;
};

struct Tile {
    std::size_t column{0};
    std::size_t row{0};
    /// The region of the job (original coordinates) this tile covers, trimmed to the job extent.
    geometry::Box2 rect;
    /// Moves per input toolpath (same order as the input; may be empty for a toolpath that has
    /// no cuts in this tile). Never all empty: tiles without any cut are left out.
    std::vector<Toolpath> toolpaths;
};

struct TilingResult {
    std::vector<Tile> tiles;
    /// Non-empty when the spec is invalid (nothing is tiled then).
    std::string error;
    bool ok() const { return error.empty(); }
};

/// Tile rectangles covering `extent` (not yet filtered by content), in row-major order
/// (rows bottom to top, columns left to right) or by ascending area. Empty + `error` when invalid.
std::vector<geometry::Box2> tileRects(const geometry::Box2& extent, const TileSpec& spec, std::string* error = nullptr);

/// Cut moves of `tp` that lie in `rect`, relinked (see above). `safeZ` is the link height.
/// `from` is the position before the first move. Empty toolpath when nothing is cut inside.
Toolpath clipToolpath(const Toolpath& tp, const geometry::Box2& rect, double safeZ, std::optional<Vec3> from = {});

/// Tiles the toolpaths over their common extent (or `extent` when given).
///
/// Limits (T-0268; each refuses with `error`, nothing is tiled):
///  - at most 10000 tiles in total;
///  - every coordinate (move end, arc centre, extent) must be finite and within +-1e7 mm, so
///    non-finite values never reach the output (`clipToolpath` alone skips such moves);
///  - at most 5 million (move, tile) pairs may need clipping: each move is only clipped against
///    the tiles its bounding box touches, so ordinary jobs cost about their move count, but a
///    huge number of moves that each span the whole job is refused;
///  - `cancel` (optional) is polled between tiles and every 1024 moves; true ends with the error
///    "cancelled".
/// Tile numbering: the "tile N" of the save path and preview counts only the tiles that have
/// cuts, 1-based, in tile order (row by row, or ascending area when `smallestFirst`); empty
/// tiles are left out and do not consume a number. `Tile::column`/`row` give the grid position.
/// `originalPosition` keeps job coordinates and does not check that a tile lies inside the
/// machine bed: the tile rectangles are regions of the job and placing the job on the bed is the
/// caller's business (tileFitsWorkArea only checks the tile size).
TilingResult tileToolpaths(const std::vector<Toolpath>& toolpaths, const TileSpec& spec,
                           std::optional<geometry::Box2> extent = {}, const std::function<bool()>& cancel = {});

/// True when the tile's rectangle fits a work area of `areaX` x `areaY` mm (0 = not set = fits).
bool tileFitsWorkArea(const Tile& tile, double areaX, double areaY);

}  // namespace carvetto::toolpath
