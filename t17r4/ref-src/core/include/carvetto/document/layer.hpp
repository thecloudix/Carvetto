#pragma once

#include <cstdint>
#include <string>

#include "carvetto/document/ids.hpp"

namespace carvetto::document {

/// 8-bit RGB colour used for layer swatches (vectors on a layer draw in its colour).
struct Colour {
    std::uint8_t r{0};
    std::uint8_t g{0};
    std::uint8_t b{0};

    constexpr bool operator==(const Colour&) const = default;
};

/// A drawing layer. Stacking order is the layer's position in `Document::layers()`, not a field,
/// so reordering never has to renumber anything.
struct Layer {
    LayerId id{};
    std::string name{"Layer 1"};
    Colour colour{};
    bool visible{true};
    bool locked{false};

    bool operator==(const Layer&) const = default;
};

}  // namespace carvetto::document
