#pragma once

// Settling a block of raster cells from one distance query (T-0167, cost). tabHolds classifies each
// cell of its raster by its distance to the vectors against two thresholds: at least `beyond` (waste
// past the kerf) or below `vectorSide` (the vector side). The distance is 1-Lipschitz, so the
// distance at one cell of a block bounds it at every cell of the block: when the whole block lies on
// one side of a threshold by more than the block's radius, no cell of it needs a query.

#include <algorithm>
#include <cstddef>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::toolpath::detail {

/// Cells per side of a block.
inline constexpr int kSettleBlock = 4;

enum class BlockVerdict : signed char {
    Open = 0,        // not settled: every cell asks for its own distance
    Far = 1,         // every cell is at least `beyond` from the vectors
    VectorSide = 2,  // every cell is less than `vectorSide` from the vectors
};

/// The verdict for the block whose first cell is (bi * kSettleBlock, bj * kSettleBlock) on a raster
/// of nx x ny cells of side `h` whose corner is `origin`. `distance(Vec2)` is the distance to the
/// vectors. The anchor is the block's cell kSettleBlock / 2 along each axis (the last cell of the
/// raster when the block is cut off by it), so no cell of the block is further from it than two
/// cells along either axis; the radius used, 3 cells and 1 µm, exceeds the diagonal of that,
/// 2.83 cells.
template <class Distance>
BlockVerdict settleBlock(int bi, int bj, int nx, int ny, double h, geometry::Vec2 origin, double beyond,
                         double vectorSide, Distance&& distance) {
    const int ci = std::min(nx - 1, bi * kSettleBlock + kSettleBlock / 2);
    const int cj = std::min(ny - 1, bj * kSettleBlock + kSettleBlock / 2);
    const double dc = distance(geometry::Vec2{origin.x + (ci + 0.5) * h, origin.y + (cj + 0.5) * h});
    const double rho = kSettleBlock * h * 0.75 + 1e-6;
    return dc - rho >= beyond ? BlockVerdict::Far : (dc + rho < vectorSide ? BlockVerdict::VectorSide : BlockVerdict::Open);
}

}  // namespace carvetto::toolpath::detail
