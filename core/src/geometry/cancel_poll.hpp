#pragma once

#include <cstddef>

#include "carvetto/geometry/cancel.hpp"

namespace carvetto::geometry::detail {

/// Thrown by `pollCancel` inside the geometry calls; every public entry point that polls catches it
/// and returns an empty result. Never crosses Clipper2 code (polls sit outside it).
struct GeometryCancelled {};

/// Throws `GeometryCancelled` when the current scope's flag is set.
inline void pollCancel() {
    if (cancelRequested()) {
        throw GeometryCancelled{};
    }
}

/// `pollCancel` on every 32nd call of a loop (i = the loop counter).
inline void pollCancelEvery(std::size_t i) {
    if ((i & 31U) == 0U) {
        pollCancel();
    }
}

}  // namespace carvetto::geometry::detail
