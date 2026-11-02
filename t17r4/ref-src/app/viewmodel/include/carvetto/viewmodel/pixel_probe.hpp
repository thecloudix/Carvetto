#pragma once

#include <cstddef>
#include <cstdint>

#include "carvetto/geometry/box2.hpp"

namespace carvetto::viewmodel {

/// Integer pixel rectangle [left, right) × [top, bottom).
struct PixelRect {
    int left{0};
    int top{0};
    int right{0};
    int bottom{0};

    bool operator==(const PixelRect&) const = default;
    bool isEmpty() const { return right <= left || bottom <= top; }
};

/// Pixels covered by a screen box grown by `inflatePx`, clipped to a `width` × `height` image.
PixelRect toPixelRect(const geometry::Box2& screenBox, int inflatePx, int width, int height);

/// Pixels covered by `screenBox` shrunk by `insetPx` (to stay clear of its outline), clipped.
PixelRect insetPixelRect(const geometry::Box2& screenBox, int insetPx, int width, int height);

/// Overlap of two pixel rectangles (empty when disjoint).
PixelRect intersect(const PixelRect& a, const PixelRect& b);

/// Counts 32-bit pixels in `region` whose bits under `mask` differ from `reference`. `pixels`
/// points at row 0 of an image `width` wide and `height` tall with rows `strideInPixels` apart.
/// Used by the self-test to check whether anything was drawn on the material.
std::size_t countPixelsDiffering(const std::uint32_t* pixels, int width, int height,
                                 std::ptrdiff_t strideInPixels, const PixelRect& region,
                                 std::uint32_t reference, std::uint32_t mask = 0x00FFFFFFu);

}  // namespace carvetto::viewmodel
