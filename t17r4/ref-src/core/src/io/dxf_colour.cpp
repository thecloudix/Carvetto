#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include "carvetto/io/dxf.hpp"

namespace carvetto::io {

namespace {

std::uint32_t pack(double r, double g, double b) {
    auto channel = [](double v) {
        return static_cast<std::uint32_t>(std::lround(std::fmin(255.0, std::fmax(0.0, v))));
    };
    return (channel(r) << 16) | (channel(g) << 8) | channel(b);
}

}  // namespace

std::uint32_t aciToRgb(int aci) {
    // Indices 1-9 are the named colours; 10-249 are 24 hues in 15-degree steps, each with five
    // brightness levels in a full and a half-saturated variant; 250-255 are a grey ramp. The
    // ramp levels follow the widely published palette; the hue rows are generated from that rule.
    static constexpr std::array<std::uint32_t, 10> kNamed{0x000000, 0xFF0000, 0xFFFF00, 0x00FF00, 0x00FFFF,
                                                          0x0000FF, 0xFF00FF, 0xFFFFFF, 0x808080, 0xC0C0C0};
    static constexpr std::array<std::uint32_t, 6> kGreys{0x333333, 0x505050, 0x696969,
                                                         0x828282, 0xBEBEBE, 0xFFFFFF};
    if (aci >= 1 && aci <= 9) {
        return kNamed[static_cast<std::size_t>(aci)];
    }
    if (aci >= 250 && aci <= 255) {
        return kGreys[static_cast<std::size_t>(aci - 250)];
    }
    if (aci < 10 || aci > 255) {
        return 0xFFFFFF;
    }
    static constexpr std::array<double, 5> kValue{255.0, 204.0, 153.0, 127.0, 76.0};
    const int hueIndex = (aci - 10) / 10;
    const int sub = (aci - 10) % 10;
    const double value = kValue[static_cast<std::size_t>(sub / 2)];
    const double low = (sub % 2 == 0) ? 0.0 : value * 0.5;
    // HSV to RGB with saturation (value - low) / value.
    const double hue = static_cast<double>(hueIndex) * 15.0 / 60.0;  // sector units
    const int sector = static_cast<int>(std::floor(hue)) % 6;
    const double f = hue - std::floor(hue);
    const double rising = low + (value - low) * f;
    const double falling = value - (value - low) * f;
    switch (sector) {
        case 0:
            return pack(value, rising, low);
        case 1:
            return pack(falling, value, low);
        case 2:
            return pack(low, value, rising);
        case 3:
            return pack(low, falling, value);
        case 4:
            return pack(rising, low, value);
        default:
            return pack(value, low, falling);
    }
}

int nearestAci(std::uint32_t rgb) {
    const auto r = static_cast<int>((rgb >> 16) & 0xFFu);
    const auto g = static_cast<int>((rgb >> 8) & 0xFFu);
    const auto b = static_cast<int>(rgb & 0xFFu);
    int best = 7;
    long bestDistance = std::numeric_limits<long>::max();
    for (int aci = 1; aci <= 255; ++aci) {
        const std::uint32_t c = aciToRgb(aci);
        const int dr = static_cast<int>((c >> 16) & 0xFFu) - r;
        const int dg = static_cast<int>((c >> 8) & 0xFFu) - g;
        const int db = static_cast<int>(c & 0xFFu) - b;
        const long d = static_cast<long>(dr) * dr + static_cast<long>(dg) * dg + static_cast<long>(db) * db;
        if (d < bestDistance) {
            bestDistance = d;
            best = aci;
        }
    }
    return best;
}

}  // namespace carvetto::io
