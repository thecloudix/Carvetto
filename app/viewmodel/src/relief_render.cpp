#include "carvetto/viewmodel/relief_render.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::viewmodel {

namespace {

std::uint32_t pack(double r, double g, double b) {
    auto c = [](double v) { return static_cast<std::uint32_t>(std::clamp(v, 0.0, 255.0) + 0.5); };
    return 0xFF000000u | (c(r) << 16) | (c(g) << 8) | c(b);
}

double chan(std::uint32_t colour, int shift) {
    return static_cast<double>((colour >> shift) & 0xFFu);
}

// Light direction (towards the light): from the upper left, 45 degrees up.
constexpr double kLx = -0.5;
constexpr double kLy = 0.5;
constexpr double kLz = 0.7071067811865476;

}  // namespace

std::uint32_t toolpathColour(std::uint32_t id) {
    // Golden-angle hue steps give distinct colours for neighbouring ids.
    const double hue = std::fmod(static_cast<double>(id) * 137.50776405, 360.0) / 60.0;
    const double c = 0.55;
    const double x = c * (1.0 - std::fabs(std::fmod(hue, 2.0) - 1.0));
    double r = 0.0;
    double g = 0.0;
    double b = 0.0;
    const int sector = static_cast<int>(hue);
    switch (sector) {
        case 0: r = c; g = x; break;
        case 1: r = x; g = c; break;
        case 2: g = c; b = x; break;
        case 3: g = x; b = c; break;
        case 4: r = x; b = c; break;
        default: r = c; b = x; break;
    }
    const double m = 0.35;
    return pack((r + m) * 255.0, (g + m) * 255.0, (b + m) * 255.0);
}

Raster renderRelief(const simulate::Heightfield& field, const ReliefOptions& options) {
    Raster out;
    const std::size_t nx = field.nx();
    const std::size_t ny = field.ny();
    if (nx == 0 || ny == 0) {
        return out;
    }
    const std::size_t maxPx = std::max<std::size_t>(options.maxPixels, 16);
    const std::size_t stride = std::max<std::size_t>(1, (std::max(nx, ny) + maxPx - 1) / maxPx);
    out.stride = stride;
    out.width = (nx + stride - 1) / stride;
    out.height = (ny + stride - 1) / stride;
    out.pixels.assign(out.width * out.height, 0u);

    const double thickness = std::max(field.topZ() - field.bottomZ(), 1e-9);
    const double flat = kLz;  // shade of a horizontal surface
    const double tableLevel = field.bottomZ() + 1e-6;
    const auto& z = field.heights();
    const auto at = [&](std::size_t i, std::size_t j) {
        return static_cast<double>(z[j * nx + i]);
    };
    for (std::size_t py = 0; py < out.height; ++py) {
        // Picture row 0 is the top of the stock: the highest grid row.
        const std::size_t j = std::min(ny - 1, (out.height - 1 - py) * stride + stride / 2);
        const std::size_t jm = j >= stride ? j - stride : j;
        const std::size_t jp = std::min(ny - 1, j + stride);
        for (std::size_t px = 0; px < out.width; ++px) {
            const std::size_t i = std::min(nx - 1, px * stride + stride / 2);
            const std::size_t im = i >= stride ? i - stride : i;
            const std::size_t ip = std::min(nx - 1, i + stride);
            const double h = at(i, j);
            const double dx = (at(ip, j) - at(im, j)) / (std::max<double>(static_cast<double>(ip - im), 1.0) * field.cellSize());
            const double dy = (at(i, jp) - at(i, jm)) / (std::max<double>(static_cast<double>(jp - jm), 1.0) * field.cellSize());
            const double len = std::sqrt(1.0 + dx * dx + dy * dy);
            const double lit = std::max(0.0, (-dx * kLx - dy * kLy + kLz) / len);
            const double shade = std::clamp(0.45 + 0.55 * (lit / flat), 0.25, 1.35);

            std::uint32_t base = kStockColour;
            const bool cut = field.isCut(i, j);
            if (cut) {
                if (h <= tableLevel) {
                    base = kTableColour;
                } else if (options.colourBy == ReliefOptions::ColourBy::Toolpath && field.tracksPassIds()) {
                    base = toolpathColour(field.passId(i, j));
                } else {
                    const double t = std::clamp((field.topZ() - h) / thickness, 0.0, 1.0);
                    const double k = 1.0 - 0.55 * std::sqrt(t);
                    base = pack(chan(kStockColour, 16) * k, chan(kStockColour, 8) * k, chan(kStockColour, 0) * k);
                }
            }
            out.pixels[py * out.width + px] = pack(chan(base, 16) * shade, chan(base, 8) * shade, chan(base, 0) * shade);
        }
    }
    return out;
}

}  // namespace carvetto::viewmodel
