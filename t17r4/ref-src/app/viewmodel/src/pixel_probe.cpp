#include "carvetto/viewmodel/pixel_probe.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::viewmodel {

namespace {

int clampToInt(double v, int lo, int hi) {
    if (!(v > static_cast<double>(lo))) {
        return lo;
    }
    if (v >= static_cast<double>(hi)) {
        return hi;
    }
    return static_cast<int>(v);
}

PixelRect clipped(double left, double top, double right, double bottom, int width, int height) {
    PixelRect r;
    r.left = clampToInt(std::floor(left), 0, width);
    r.top = clampToInt(std::floor(top), 0, height);
    r.right = clampToInt(std::ceil(right), 0, width);
    r.bottom = clampToInt(std::ceil(bottom), 0, height);
    if (r.isEmpty()) {
        return PixelRect{};
    }
    return r;
}

}  // namespace

PixelRect toPixelRect(const geometry::Box2& screenBox, int inflatePx, int width, int height) {
    if (screenBox.isEmpty()) {
        return PixelRect{};
    }
    const double g = static_cast<double>(inflatePx);
    return clipped(screenBox.min.x - g, screenBox.min.y - g, screenBox.max.x + g, screenBox.max.y + g, width,
                   height);
}

PixelRect insetPixelRect(const geometry::Box2& screenBox, int insetPx, int width, int height) {
    if (screenBox.isEmpty()) {
        return PixelRect{};
    }
    const double g = static_cast<double>(insetPx);
    // Round inward so partially covered edge pixels are excluded.
    return clipped(std::ceil(screenBox.min.x + g), std::ceil(screenBox.min.y + g),
                   std::floor(screenBox.max.x - g), std::floor(screenBox.max.y - g), width, height);
}

PixelRect intersect(const PixelRect& a, const PixelRect& b) {
    PixelRect r{std::max(a.left, b.left), std::max(a.top, b.top), std::min(a.right, b.right),
                std::min(a.bottom, b.bottom)};
    return r.isEmpty() ? PixelRect{} : r;
}

std::size_t countPixelsDiffering(const std::uint32_t* pixels, int width, int height,
                                 std::ptrdiff_t strideInPixels, const PixelRect& region,
                                 std::uint32_t reference, std::uint32_t mask) {
    if (pixels == nullptr || width <= 0 || height <= 0) {
        return 0;
    }
    const PixelRect r = intersect(region, PixelRect{0, 0, width, height});
    std::size_t count = 0;
    for (int y = r.top; y < r.bottom; ++y) {
        const std::uint32_t* row = pixels + static_cast<std::ptrdiff_t>(y) * strideInPixels;
        for (int x = r.left; x < r.right; ++x) {
            if ((row[x] & mask) != (reference & mask)) {
                ++count;
            }
        }
    }
    return count;
}

}  // namespace carvetto::viewmodel
