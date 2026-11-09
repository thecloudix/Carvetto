#pragma once

#include <cstddef>

#include "carvetto/geometry/cancel.hpp"

namespace Clipper2Lib {
/// Added to the vendored engine by core/CMakeLists.txt (T-0228): polled inside a sweep.
extern thread_local void (*carvettoPollHook)();
}  // namespace Clipper2Lib

namespace carvetto::geometry::detail {

/// Thrown by `pollCancel` inside the geometry calls; every public entry point that polls catches it
/// and returns an empty result. Never crosses Clipper2 code (polls sit outside it).
struct GeometryCancelled {};

/// Throws `GeometryCancelled` when the current scope's flag is set. (Not the same function as the
/// work meter's `toolpath::detail::pollCancel`, which polls the meter's flag and throws WorkAborted.)
inline void pollCancel() {
    if (cancelRequested()) {
        throw GeometryCancelled{};
    }
}

/// While alive, Clipper2 sweeps on this thread poll the cancel flag and throw `GeometryCancelled`.
/// Installed only inside the entry points that catch it, never around unguarded Clipper2 use.
class ClipperCancelGuard {
   public:
    ClipperCancelGuard() : previous_(Clipper2Lib::carvettoPollHook) { Clipper2Lib::carvettoPollHook = &pollCancel; }
    ~ClipperCancelGuard() { Clipper2Lib::carvettoPollHook = previous_; }
    ClipperCancelGuard(const ClipperCancelGuard&) = delete;
    ClipperCancelGuard& operator=(const ClipperCancelGuard&) = delete;

   private:
    void (*previous_)();
};

/// `pollCancel` on every 32nd call of a loop (i = the loop counter).
inline void pollCancelEvery(std::size_t i) {
    if ((i & 31U) == 0U) {
        pollCancel();
    }
}

}  // namespace carvetto::geometry::detail
