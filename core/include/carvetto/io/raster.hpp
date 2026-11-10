#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "carvetto/geometry/shapes.hpp"

/// Raw raster images and a tiny in-repo decoder (T-0245, ADR-0038).
///
/// `RasterImage` is the only thing the bitmap tracer consumes: 8-bit RGB, row 0 at the top. Other
/// formats (PNG, JPEG, ...) are decoded by the app layer into this structure; core has no image
/// codec dependency. The one format core reads itself is Netpbm (PBM/PGM/PPM, ASCII and binary).
namespace carvetto::io {

/// Largest accepted image edge and pixel count (hostile-size cap, checked before any allocation).
inline constexpr int kMaxImageEdge = 16384;
inline constexpr std::size_t kMaxImagePixels = std::size_t{16} * 1024 * 1024;

struct RasterImage {
    int width{0};
    int height{0};
    std::vector<std::uint8_t> rgb;  ///< width*height*3 bytes, row-major from the top row

    bool valid() const {
        return width > 0 && height > 0 && width <= kMaxImageEdge && height <= kMaxImageEdge &&
               static_cast<std::size_t>(width) * static_cast<std::size_t>(height) <= kMaxImagePixels &&
               rgb.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3;
    }
};

/// Decodes Netpbm P1..P6 (maxval up to 65535, scaled to 8 bits; PBM 1 = black). Truncated data,
/// bad headers and oversize dimensions return an error; no allocation precedes the size check.
geometry::ShapeResult<RasterImage> decodePnm(std::string_view data);

}  // namespace carvetto::io
