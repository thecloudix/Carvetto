#pragma once

#include <atomic>
#include <cstddef>
#include <vector>

#include "carvetto/relief/relief.hpp"

// Two Rail Sweep (T-0288, ADR-0065): sweeps cross-section profiles along two rail polylines into
// a height field. Pure function of SweepParams, so the parameters are the stored, re-editable
// form of the component. Units are mm.

namespace carvetto::relief {

inline constexpr std::size_t kMaxSweepSections = 256;
inline constexpr std::size_t kMaxSweepRailVertices = 100000;
inline constexpr std::size_t kMaxSweepProfilePoints = 4096;
/// Most rungs x samples-per-rung one sweep may build.
inline constexpr std::size_t kMaxSweepQuads = std::size_t{1} << 22;

enum class SweepStatus {
    Ok,
    BadRail,            ///< < 2 (open) / < 3 (closed) vertices, non-finite, too many, zero length.
    RailKindMismatch,   ///< One rail open, the other closed.
    NoSections,
    BadSection,         ///< Profile u not strictly increasing, non-finite, t outside 0..1 or not monotone.
    NodeCountMismatch,  ///< "Add to all rail nodes" needs equal vertex counts on both rails.
    BadParameter,       ///< Non-finite or negative exact height, bad rail selector.
    BadGrid,
    TooLarge,
    Cancelled,
    FlatResult,         ///< Scale to exact height requested but the swept surface has no positive height.
};

const char* sweepStatusMessage(SweepStatus s);

/// A cross section: (u, z) points, u = distance across the section in mm, strictly increasing;
/// the first point lies on rail 1, the last on rail 2. (t1, t2) place it at that fraction (0..1)
/// of each rail's arc length (proportional positioning is t1 == t2).
struct SweepSection {
    std::vector<Point> profile;
    double t1 = 0.0;
    double t2 = 0.0;
    bool smooth = false;  ///< Monotone cubic through the points instead of straight segments.
};

struct SweepParams {
    std::vector<Point> rail1;
    std::vector<Point> rail2;
    bool closed1 = false;
    bool closed2 = false;
    std::vector<SweepSection> sections;  ///< In order along the rails.
    bool scaleWithWidth = false;         ///< z scaled by (rail gap / section width).
    bool sweepBetweenSpans = true;       ///< Blend neighbouring sections; false holds each until the next.
    bool fillCenter = false;             ///< Closed rails: flat-fill inside the inner rail.
    double exactHeight = 0.0;            ///< > 0: scale so the highest cell equals this.
};

SweepStatus validateSweep(const SweepParams& p);

/// Rail 1 <-> rail 2 (sections keep their u, so the surface is mirrored across the rails).
void swapRails(SweepParams& p);
/// Reverses rail `which` (1 or 2), remapping section positions. Returns BadParameter otherwise.
SweepStatus reverseRail(SweepParams& p, int which);
/// Appends the profile as a section at every node; both rails must have equal vertex counts.
SweepStatus addSectionToAllNodes(SweepParams& p, const std::vector<Point>& profile, bool smooth);

/// Sweeps onto `grid`; cells not between the rails are transparent. `cancel` (may be null) is
/// polled once per rung; on Cancelled `out` is empty.
SweepStatus sweep(const SweepParams& p, const Grid& grid, const std::atomic<bool>* cancel, HeightField& out);

}  // namespace carvetto::relief
