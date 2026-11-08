#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "carvetto/simulate/stock.hpp"

// Shaded top-down relief of a simulated heightfield (T-0223): a Qt-free raster the app wraps in
// a QImage. Row 0 is the top of the picture (the high-Y edge of the stock).

namespace carvetto::viewmodel {

struct Raster {
    std::size_t width{0};
    std::size_t height{0};
    /// 0xAARRGGBB, row-major.
    std::vector<std::uint32_t> pixels;
    /// Cells per pixel along each axis (1 = every cell).
    std::size_t stride{1};
};

struct ReliefOptions {
    enum class ColourBy { Depth, Toolpath };
    ColourBy colourBy{ColourBy::Depth};
    /// The raster is at most this many pixels along its longer side (cells are sampled with a stride).
    std::size_t maxPixels{1600};
};

/// Colour of the uncut stock top and of the table where the cutter went through the stock.
inline constexpr std::uint32_t kStockColour = 0xFFDEB887u;
inline constexpr std::uint32_t kTableColour = 0xFF8C8C96u;
/// Colour of the cut with pass id `id` (>= 1) in Toolpath mode.
std::uint32_t toolpathColour(std::uint32_t id);

/// Hill-shaded picture of `field`: the stock colour where uncut, darker with depth (or the
/// toolpath's colour) where cut, the table colour where cut through; light from the upper left.
/// O(pixels). An empty field gives an empty raster.
Raster renderRelief(const simulate::Heightfield& field, const ReliefOptions& options = {});

}  // namespace carvetto::viewmodel
