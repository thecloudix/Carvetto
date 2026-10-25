#pragma once

namespace carvetto::geometry {

/// Rule deciding which regions of self-overlapping or nested closed contours count as filled.
/// NonZero treats a hole as a contour wound opposite to its outer (the usual CAD convention);
/// EvenOdd ignores orientation and alternates inside/outside at every boundary. Shared by
/// point-in-contour queries (`contour_ops.hpp`) and boolean operations (`boolean.hpp`); code
/// must compare enumerators by name, never by numeric value.
enum class FillRule { NonZero, EvenOdd };

}  // namespace carvetto::geometry
