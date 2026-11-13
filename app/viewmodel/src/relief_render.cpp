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
    // Min-pooling: each pixel stands for the lowest cell of its stride x stride block (the cell
    // index is kept for the cut state and pass id), so a groove narrower than the stride still
    // shows. With stride 1 the block is the cell itself: the picture is unchanged.
    std::vector<std::size_t> cellOf(out.width * out.height, 0);
    std::vector<double> pooled(out.width * out.height, 0.0);
    for (std::size_t py = 0; py < out.height; ++py) {
        // Picture row 0 is the top of the stock: the highest grid row.
        const std::size_t row = out.height - 1 - py;
        const std::size_t j0 = row * stride;
        const std::size_t j1 = std::min(ny, j0 + stride);
        for (std::size_t px = 0; px < out.width; ++px) {
            const std::size_t i0 = px * stride;
            const std::size_t i1 = std::min(nx, i0 + stride);
            std::size_t best = j0 * nx + i0;
            for (std::size_t j = j0; j < j1; ++j) {
                for (std::size_t i = i0; i < i1; ++i) {
                    if (z[j * nx + i] < z[best]) {
                        best = j * nx + i;
                    }
                }
            }
            cellOf[py * out.width + px] = best;
            pooled[py * out.width + px] = static_cast<double>(z[best]);
        }
    }
    const double cell = field.cellSize() * static_cast<double>(stride);
    for (std::size_t py = 0; py < out.height; ++py) {
        // Neighbouring picture rows: a smaller py is a higher grid row.
        const std::size_t pyUp = py > 0 ? py - 1 : py;
        const std::size_t pyDown = std::min(out.height - 1, py + 1);
        for (std::size_t px = 0; px < out.width; ++px) {
            const std::size_t pxm = px > 0 ? px - 1 : px;
            const std::size_t pxp = std::min(out.width - 1, px + 1);
            const std::size_t k = py * out.width + px;
            const double h = pooled[k];
            const double dx = (pooled[py * out.width + pxp] - pooled[py * out.width + pxm]) /
                              (std::max<double>(static_cast<double>(pxp - pxm), 1.0) * cell);
            // Grid y grows as the picture row shrinks.
            const double dy = (pooled[pyUp * out.width + px] - pooled[pyDown * out.width + px]) /
                              (std::max<double>(static_cast<double>(pyDown - pyUp), 1.0) * cell);
            const double len = std::sqrt(1.0 + dx * dx + dy * dy);
            const double lit = std::max(0.0, (-dx * kLx - dy * kLy + kLz) / len);
            const double shade = std::clamp(0.45 + 0.55 * (lit / flat), 0.25, 1.35);

            const std::size_t i = cellOf[k] % nx;
            const std::size_t j = cellOf[k] / nx;
            std::uint32_t base = kStockColour;
            const bool cut = field.isCut(i, j);
            if (cut) {
                if (h <= tableLevel) {
                    base = kTableColour;
                } else if (options.colourBy == ReliefOptions::ColourBy::Toolpath && field.tracksPassIds()) {
                    base = toolpathColour(field.passId(i, j));
                } else {
                    const double t = std::clamp((field.topZ() - h) / thickness, 0.0, 1.0);
                    const double kk = 1.0 - 0.55 * std::sqrt(t);
                    base = pack(chan(kStockColour, 16) * kk, chan(kStockColour, 8) * kk, chan(kStockColour, 0) * kk);
                }
            }
            out.pixels[k] = pack(chan(base, 16) * shade, chan(base, 8) * shade, chan(base, 0) * shade);
        }
    }
    return out;
}

}  // namespace carvetto::viewmodel
