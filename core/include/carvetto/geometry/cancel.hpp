#pragma once

#include <atomic>

namespace carvetto::geometry {

/// Cooperative cancel for the long geometry calls (T-0228). While a `CancelScope` is alive on a
/// thread, `offsetContours`, `offsetOpenContour`, `unionContours` and the other boolean calls made
/// on that thread poll `*flag` (a relaxed load, every few dozen spans or at each stage) and stop
/// within milliseconds once it reads true; the call then returns an EMPTY result, which the caller
/// must discard (it checks the same flag). The flag is never written here and must outlive the
/// scope. A null flag installs nothing. Scopes nest (the previous flag is restored). A call whose
/// flag never reads true returns exactly what it would without any scope.
///
/// The vendored Clipper2 sweep polls too (patched in core/CMakeLists.txt), so even one huge
/// sweep stops in milliseconds. `Contour::polygonise` also checks the flag (every 32 spans) and
/// then returns the polygon built so far: a partial result, valid only to be discarded.
class CancelScope {
   public:
    explicit CancelScope(const std::atomic<bool>* flag);
    ~CancelScope();
    CancelScope(const CancelScope&) = delete;
    CancelScope& operator=(const CancelScope&) = delete;

   private:
    const std::atomic<bool>* previous_;
};

/// True when the flag of the innermost scope on this thread reads true.
bool cancelRequested();

}  // namespace carvetto::geometry
