#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

#include "carvetto/relief/relief.hpp"

// Extrude and Weave (T-0289, ADR-0066): extrudes a vector cross section, or copies of a component,
// along one or more rail polylines into a height field, optionally weaving crossing rails over and
// under each other. Pure function of ExtrudeParams, so the parameters are the stored, re-editable
// form of the component. Units are mm.

namespace carvetto::relief {

inline constexpr std::size_t kMaxExtrudeRails = 64;
inline constexpr std::size_t kMaxExtrudeSections = 256;
inline constexpr std::size_t kMaxExtrudeProfilePoints = 4096;
inline constexpr std::size_t kMaxExtrudeRailVertices = 100000;
/// Weave looks at every pair of rail segments, so the total segment count is bounded tighter.
inline constexpr std::size_t kMaxWeaveSegments = 2000;
inline constexpr std::size_t kMaxExtrudeCopies = 10000;
/// Most rungs x samples-per-rung one extrusion may build.
inline constexpr std::size_t kMaxExtrudeQuads = std::size_t{1} << 22;

enum class ExtrudeStatus {
    Ok,
    NoRails,        ///< No rails.
    BadRail,        ///< Too many rails/vertices, < 2 (open) / < 3 (closed) distinct vertices, non-finite, zero length.
    NoSections,     ///< Cross-section source without sections.
    BadSection,     ///< Profile u not strictly increasing, non-finite, t outside 0..1 or not strictly increasing.
    BadComponent,   ///< Component source with an empty field, fewer than 2x2 cells or only transparent cells.
    BadParameter,   ///< Non-finite or out-of-range overlap, weave percent or exact height.
    BadGrid,
    TooLarge,
    Cancelled,
    FlatResult,     ///< Scale to exact height requested but the surface has no positive height.
};

const char* extrudeStatusMessage(ExtrudeStatus s);

enum class ExtrudeSource { CrossSections, Component };

struct ExtrudeRail {
    std::vector<Point> points;  ///< Callers polygonise arcs.
    bool closed = false;
};

/// A cross section centred on the rail: (u, z) points, u strictly increasing; the middle of the
/// u range sits on the rail, increasing u is to the left of the travel direction. `t` places it at
/// that fraction (0..1) of every rail's arc length.
struct ExtrudeSection {
    std::vector<Point> profile;
    double t = 0.0;
    bool smooth = false;  ///< Monotone cubic through the points instead of straight segments.
};

enum class WeaveMode {
    ScaleShape,  ///< Height multiplied by the percent.
    AddBase,     ///< Shape kept; lowered/raised by (percent - 100)% of its highest point.
};

struct WeaveParams {
    bool enabled = false;
    double zUnderPercent = 50.0;  ///< Level of the strand that passes under a crossing, >= 0.
    double zOverPercent = 100.0;  ///< Level of the strand that passes over a crossing, >= 0.
    WeaveMode mode = WeaveMode::ScaleShape;
    bool startOver = true;        ///< false swaps over and under at every crossing.
};

struct ExtrudeParams {
    std::vector<ExtrudeRail> rails;
    ExtrudeSource source = ExtrudeSource::CrossSections;

    // CrossSections.
    std::vector<ExtrudeSection> sections;  ///< Ascending t.
    bool sweepBetweenSpans = true;         ///< Blend neighbouring sections; false holds each until the next.

    // Component. The field's x runs along the rail, y to the left of it, centred on the rail.
    // The caller bakes the component's dynamic properties (relief::bake) first.
    HeightField component;
    bool stretchToRail = true;     ///< One copy stretched to the rail length; false: fill with copies.
    double overlapPercent = 0.0;   ///< Copies overlap by this % of their length, 0 <= p < 100.
    bool flipAlternate = false;    ///< Every second copy is mirrored along the rail.

    bool sharpCorners = false;     ///< Mitred (sharp) rail corners; false rounds them.
    WeaveParams weave;
    double exactHeight = 0.0;      ///< > 0: scale so the highest cell equals this.
};

ExtrudeStatus validateExtrude(const ExtrudeParams& p);

/// Extrudes onto `grid`; cells not covered are transparent. Overlaps keep the higher value.
/// `cancel` (may be null) is polled once per rung; on Cancelled `out` is empty.
ExtrudeStatus extrude(const ExtrudeParams& p, const Grid& grid, const std::atomic<bool>* cancel,
                      HeightField& out);

}  // namespace carvetto::relief
