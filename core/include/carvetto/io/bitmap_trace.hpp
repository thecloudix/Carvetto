#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/io/raster.hpp"

/// Bitmap to vector tracing (T-0245, ADR-0038). Clean-room algorithm, in four steps:
///  1. Mask: Black and White mode marks pixels darker than `threshold` (Rec.601 luma; `invert`
///     flips it); Colour mode quantises to `colourCount` (2..16) colours by median cut on a 5-bit
///     histogram and gives each colour its own mask.
///  2. Noise filter: foreground components and background holes of fewer than `noiseFilter`
///     pixels are removed.
///  3. Boundary: the exact pixel-edge loops of each mask (foreground 4-connected, so diagonal
///     pixels stay separate), outer loops CCW and holes CW, in y-up millimetres.
///  4. Fit: corners are kept where two straight runs of at least `cornerRun` pixels meet; between
///     corners the staircase is smoothed (window set by `cornerFit`) and fitted with lines or
///     cubic Beziers within 0.5 px.
namespace carvetto::io {

enum class TraceMode { BlackWhite, Colour };

struct TraceParams {
    TraceMode mode{TraceMode::BlackWhite};
    std::uint8_t threshold{128};   ///< B&W: luma < threshold is foreground
    bool invert{false};            ///< B&W: foreground is the light side
    int colourCount{8};            ///< Colour: 2..16
    double cornerFit{0.5};         ///< 0 loose (smooth, rounded) .. 1 tight (keeps corners)
    int noiseFilter{2};            ///< minimum feature size in pixels (0/1 = off)
    double mmPerPixel{1.0};        ///< > 0
    geometry::Vec2 origin{0, 0};   ///< position of the image's bottom-left corner, mm
};

/// Output limits; a request exceeding them returns an error rather than a partial result.
inline constexpr std::size_t kTraceMaxContours = 50'000;
inline constexpr std::size_t kTraceMaxSpans = 1'000'000;

struct TraceLayer {
    std::uint8_t r{0}, g{0}, b{0};            ///< layer colour (black in B&W mode)
    std::size_t pixelCount{0};                ///< foreground pixels after noise filtering
    std::vector<geometry::Contour> contours;  ///< closed; outer CCW, holes CW
};

/// B&W mode returns one layer; Colour mode one per non-empty palette entry, most populous first.
geometry::ShapeResult<std::vector<TraceLayer>> traceBitmap(const RasterImage& img, const TraceParams& p);

}  // namespace carvetto::io
