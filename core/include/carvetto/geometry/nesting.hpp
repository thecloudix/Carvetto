#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/transform2.hpp"

// General polygon nesting (T-0337, ADR-0130). Millimetres and radians; no Qt, no document.

namespace carvetto::geometry {

/// One rigid shape to nest: a closed outline and the closed holes cut out of it.
struct NestShape {
    Polyline outer;
    std::vector<Polyline> holes;
};

/// A shape and how many copies of it to place (>= 1).
struct NestPart {
    NestShape shape;
    std::size_t copies{1};
};

/// The sheet corner the nest starts from; copies fill away from it.
enum class NestCorner { BottomLeft, BottomRight, TopLeft, TopRight };

/// AlongX fills a row along X before starting the next row; AlongY fills a column along Y first.
enum class NestDirection { AlongX, AlongY };

struct NestSheet {
    double lengthMm{2440.0};  ///< Along X.
    double widthMm{1220.0};   ///< Along Y.
};

struct NestingOptions {
    /// Sheet sizes in order (the "nesting sheets" list): sheet `i` uses `sheets[i]`, the last entry
    /// repeats for every later sheet. Empty means one default 2440 x 1220 sheet size.
    std::vector<NestSheet> sheets;
    /// Optional nest boundary: closed polygons, even-odd filled (an inner polygon is an exclusion).
    /// When non-empty every sheet is a copy of this region in its own coordinates and `sheets` is
    /// ignored.
    std::vector<Polyline> boundary;
    /// Spacing between parts (tool diameter plus clearance). Clamped up to 0.001 mm plus the
    /// polygonisation tolerance, so parts never touch.
    double gapMm{6.0};
    /// Distance kept to the sheet edge or the boundary polygons (the border gap).
    double marginMm{10.0};
    /// Rotate parts in steps of this many radians (0 = no rotation; at most 360 steps, steps
    /// below 1 degree are raised to 1 degree).
    double rotationStepRad{0.0};
    /// Also try each part mirrored.
    bool mirror{false};
    /// Let parts sit inside the holes of other parts.
    bool allowInside{false};
    NestCorner corner{NestCorner::BottomLeft};
    NestDirection direction{NestDirection::AlongX};
    std::size_t maxSheets{64};
    /// Deterministic work budget (edge-pair and candidate probes). When it runs out the rest is
    /// reported unplaced with `NestStopReason::Budget`.
    std::uint64_t maxOperations{40'000'000};
    /// Chordal tolerance used when the caller polygonised curves; added to the spacing.
    double chordalMm{0.0};
};

struct NestPlacementResult {
    std::size_t part{0};
    std::size_t copy{0};
    std::size_t sheet{0};
    /// Maps the part's input coordinates to sheet coordinates (rotation, optional mirror, move).
    Transform2 transform;
    double angleRad{0.0};
    bool mirrored{false};
};

enum class NestStopReason {
    TooLarge,   ///< Does not fit an empty sheet (or boundary) in any orientation.
    NoSheet,    ///< Would need more than `maxSheets` sheets.
    Budget,     ///< The operation budget ran out.
    Cancelled,  ///< `cancelRequested()` was true.
    Invalid     ///< Fewer than 3 finite vertices or no area.
};

struct NestUnplaced {
    std::size_t part{0};
    std::size_t copy{0};
    NestStopReason reason{NestStopReason::TooLarge};
};

struct NestingResult {
    std::vector<NestPlacementResult> placements;  ///< In placement order (largest first).
    std::vector<NestUnplaced> unplaced;           ///< Sorted by (part, copy).
    std::size_t sheetCount{0};
    std::uint64_t operations{0};
    bool budgetExhausted{false};
    bool cancelled{false};
};

/// Packs `parts` onto sheets. Greedy bottom-left: copies are taken largest area first (ties by
/// part, copy); for each, every sheet in order is probed with candidate positions taken from the
/// edges of what is already placed (and of the holes when `allowInside`), in every rotation/mirror
/// orientation; the position nearest the start corner (rows along X or columns along Y) is slid
/// toward the corner and the orientation with the smallest far edge wins; no sheet fits opens a
/// new one. Deterministic (no randomness, no clocks, no threads); work is bounded by
/// `maxOperations`; `cancelRequested()` is polled. Every placement keeps the clearance to its
/// neighbours and the margin to the sheet edge or boundary, and holes are only used when
/// `allowInside`. The result is exact for the polygons given (coarse curves: see `chordalMm`).
NestingResult nestParts(const std::vector<NestPart>& parts, const NestingOptions& options);

}  // namespace carvetto::geometry
