#pragma once

#include <cstddef>
#include <optional>
#include <vector>

#include "carvetto/relief/relief.hpp"

// Textured area component (T-0291, ADR-0078): tiles a seed height field inside a closed vector
// boundary. A pure function of TexturedAreaParams, so the parameters are the stored, re-editable
// form of the component. Units are mm.

namespace carvetto::relief {

/// Most tiles (row x column index range, conservative) one textured area may place.
inline constexpr std::size_t kMaxTexturedTiles = 100000;
/// Most (tile, cell) visits one evaluation may perform.
inline constexpr std::size_t kMaxTexturedCellVisits = std::size_t{1} << 28;

enum class TexturedStatus {
    Ok,
    BadSeed,       ///< Empty seed or invalid grid.
    BadBoundary,   ///< No ring, a ring with < 3 vertices, non-finite coordinates, or zero area extent.
    BadParameter,  ///< Non-finite value, spacing <= -100 %.
    TooManyTiles,  ///< More than kMaxTexturedTiles tiles (or cell visits) would be needed.
    TooLarge,      ///< Output grid larger than kMaxCells.
};

const char* texturedStatusMessage(TexturedStatus s);

/// Mirroring of alternate tiles. Horizontal flips odd columns left-right, Vertical flips odd rows
/// top-bottom, Both does both; None repeats the seed unchanged.
enum class Reflection { None, Horizontal, Vertical, Both };

struct TexturedAreaParams {
    HeightField seed;                       ///< One tile; its cell size is the output cell size.
    std::vector<std::vector<Point>> rings;  ///< Closed boundary rings, even-odd fill (holes allowed).
    double spacingXPercent = 0.0;           ///< Gap between tiles as % of tile width; negative overlaps.
    double spacingYPercent = 0.0;           ///< Same for height.
    double shiftXPercent = 0.0;             ///< Odd rows move in X by this % of the column pitch.
    double shiftYPercent = 0.0;             ///< Odd columns move in Y by this % of the row pitch.
    Reflection reflection = Reflection::None;
    /// Lower-left corner of tile (row 0, column 0); the boundary's bounding-box minimum if unset.
    std::optional<Point> anchor;
    /// Where tiles overlap (negative spacing): the later tile (row-major, bottom row first) is the
    /// "above" operand of combine().
    CombineMode overlapMode = CombineMode::MergeHigh;
};

TexturedStatus validateTexturedArea(const TexturedAreaParams& p);

/// Number of tile positions the evaluation will visit (conservative range); 0 on invalid params.
std::size_t texturedTileCount(const TexturedAreaParams& p);

/// Evaluates the textured area on the boundary bounding box at the seed's cell size: cell centres
/// are at bboxMin + (i + 0.5, j + 0.5) * cell. A cell is opaque iff its centre is inside the
/// boundary (even-odd, half-open) and a tile covers it with an opaque seed cell (nearest cell,
/// no interpolation); everything else is transparent.
TexturedStatus texturedArea(const TexturedAreaParams& p, HeightField& out);

/// `texturedArea` wrapped as a component (100 % height, base 0, mode as given).
TexturedStatus makeTexturedComponent(const TexturedAreaParams& p, CombineMode mode, Component& out);

}  // namespace carvetto::relief
