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
/// Limit: a single Clipper2 call (one scanline sweep over one batch) cannot be interrupted; the
/// poll happens before and after it and between batches.
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
