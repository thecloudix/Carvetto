#pragma once

#include <vector>

#include "carvetto/document/object.hpp"
#include "carvetto/geometry/shapes.hpp"

namespace carvetto::document {

/// Contours for a set of creation parameters, via the builders in `geometry/shapes.hpp`.
/// `NoParams` has no parametric origin and yields an error. Deterministic: equal parameters
/// give equal (bit-identical) contours, so an object loaded from a `.carv` file can be rebuilt.
geometry::ShapeResult<std::vector<geometry::Contour>> buildContours(
    const CreationParams& params, const geometry::Tolerance& tol = geometry::Tolerance::defaults());

}  // namespace carvetto::document
