#include "carvetto/viewmodel/pixel_probe.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <vector>

using carvetto::geometry::Box2;
using carvetto::viewmodel::countPixelsDiffering;
using carvetto::viewmodel::insetPixelRect;
using carvetto::viewmodel::intersect;
using carvetto::viewmodel::PixelRect;
using carvetto::viewmodel::toPixelRect;

TEST_CASE("pixel rectangles are clipped to the image", "[viewmodel][probe]") {
    CHECK(toPixelRect(Box2{{-5.0, 2.5}, {12.2, 30.0}}, 1, 10, 20) == PixelRect{0, 1, 10, 20});
    CHECK(toPixelRect(Box2{}, 1, 10, 10).isEmpty());
    CHECK(insetPixelRect(Box2{{1.5, 1.5}, {8.5, 8.5}}, 1, 10, 10) == PixelRect{3, 3, 7, 7});
    CHECK(intersect(PixelRect{0, 0, 5, 5}, PixelRect{6, 6, 8, 8}).isEmpty());
    CHECK(intersect(PixelRect{0, 0, 5, 5}, PixelRect{3, 2, 8, 8}) == PixelRect{3, 2, 5, 5});
}

TEST_CASE("countPixelsDiffering counts only inside the region and under the mask", "[viewmodel][probe]") {
    const int w = 8;
    const int h = 6;
    const std::ptrdiff_t stride = 10;  // padded rows
    std::vector<std::uint32_t> img(static_cast<std::size_t>(stride * h), 0xFFFFFFFFu);
    img[static_cast<std::size_t>(2 * stride + 3)] = 0xFF000000u;  // black, inside
    img[static_cast<std::size_t>(5 * stride + 7)] = 0xFF000000u;  // black, outside region
    img[static_cast<std::size_t>(1 * stride + 1)] = 0x00FFFFFFu;  // alpha differs only
    img[static_cast<std::size_t>(0 * stride + 9)] = 0xFF000000u;  // in padding, never read
    const PixelRect region{0, 0, 5, 5};
    CHECK(countPixelsDiffering(img.data(), w, h, stride, region, 0xFFFFFFFFu) == 1);
    CHECK(countPixelsDiffering(img.data(), w, h, stride, PixelRect{0, 0, 100, 100}, 0xFFFFFFFFu) == 2);
    CHECK(countPixelsDiffering(img.data(), w, h, stride, region, 0xFFFFFFFFu, 0xFFFFFFFFu) == 2);
    CHECK(countPixelsDiffering(nullptr, w, h, stride, region, 0) == 0);
}
