#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/contour.hpp"

namespace carvetto::document {

/// A keep-out zone (T-0241, ADR-0036): a closed outline the tool's rapid moves must stay out of.
/// It is a copy of a closed vector's outline, not linked to its source: editing or deleting the
/// vector leaves the zone alone. The region is the outline's interior (non-zero fill); several
/// zones may overlap. Coordinates are job coordinates in mm.
struct KeepOutZone {
    KeepOutZoneId id;
    geometry::Contour outline;

    bool operator==(const KeepOutZone&) const = default;
};

/// Most zones a document holds, and the most spans over all outlines (both enforced by the
/// commands and the file loader).
inline constexpr std::size_t kMaxKeepOutZones = 10000;
inline constexpr std::size_t kMaxKeepOutSpans = 2'000'000;
/// Largest clearance accepted, mm.
inline constexpr double kMaxKeepOutClearanceMm = 1000.0;

}  // namespace carvetto::document
