#pragma once

// Internal to core (pocket.cpp) and its tests: the strict inset guard of the pocket's tool-centre
// region (ADR-0017 §4), callable on a given raw inset so tests can feed it shapes Clipper2 has
// not been seen to produce.

#include <cstddef>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::toolpath::detail {

/// What the strict guard could not keep exactly.
struct InsetReport {
    /// Genuine regions that could not be made valid (not cut).
    std::size_t dropped{0};
    /// Genuine regions kept only after an extra inset larger than the join tolerance, or losing
    /// more area than such an inset would (material near them may be left).
    std::size_t trimmed{0};
};

/// The strict guard applied to `raw`, taken as the inset of the region of `source` by d (with
/// `tol` as the polygon tolerance): phantoms rejected, genuine regions kept, pulled in or
/// repaired so every vertex and every changed edge keeps d less the join tolerance from the
/// walls; what cannot be kept exactly is counted in `*report`.
std::vector<geometry::Contour> strictInset(const std::vector<geometry::Contour>& source,
                                           const std::vector<geometry::Contour>& raw, double d,
                                           const geometry::Tolerance& tol, InsetReport* report);

}  // namespace carvetto::toolpath::detail
