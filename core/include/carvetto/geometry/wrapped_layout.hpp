#pragma once

#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"

// Wrapped Fluting and Spiral layouts (T-0322, ADR-0115; feature-inventory "Wrapping submenu"): vectors on the flat
// design area of a rotary job (ADR-0098: X/Y hold the distance along the axis and round the circumference) that
// wrap onto the cylinder as evenly spaced flutes along the axis, or as helical strands. The vectors are plain
// contours for the Fluting, Profile and Pocket toolpaths. Units are mm; the circumference is pi x diameter.

namespace carvetto::geometry {

/// What a flute is drawn as. CentreLine: an open line along the axis (for the Fluting toolpath). Rectangle: a
/// closed outline of the flute's width with square ends. Stadium: the same with round (cove) ends.
enum class FluteShape { CentreLine, Rectangle, Stadium };

struct WrappedFlutingParams {
    /// The cylinder lies along X (the circumference is Y) when true, along Y when false.
    bool alongX{true};
    /// Cylinder diameter (> 0).
    double diameter{50.0};
    /// Number of flutes, evenly spaced round the circumference (1..kMaxLayoutStrands).
    int flutes{4};
    /// Where the flutes run along the axis (end > start; a Stadium needs room for its two ends).
    double alongStart{0.0};
    double alongEnd{100.0};
    /// Circumference coordinate of the start of a turn, and the offset of the first flute from it (mm; any value,
    /// taken modulo the turn).
    double circStart{0.0};
    double offset{0.0};
    FluteShape shape{FluteShape::CentreLine};
    /// Flute width (mm; > 0 for Rectangle and Stadium, ignored for CentreLine). At most the spacing between flutes.
    double width{4.0};
    /// Flute depth (mm, > 0): checked against the cylinder (it must stay above the axis); the Fluting toolpath
    /// takes it as its flute depth.
    double depth{2.0};
};

struct WrappedFlutingResult {
    /// Flute i (counting from the first) is `contours[i]`.
    std::vector<Contour> contours;
    /// Why nothing was made; empty when it worked.
    std::string error;
    /// Distance between neighbouring flutes round the circumference (mm).
    double spacing{0.0};
    /// Flutes whose outline crosses the edge of the turn (it would leave the flat design area).
    std::vector<int> crossingSeam;
    bool ok() const { return error.empty(); }
};

/// Most flutes or strands, and most vectors a layout makes.
inline constexpr int kMaxLayoutStrands = 1000;
inline constexpr int kMaxLayoutVectors = 100000;

WrappedFlutingResult wrappedFlutingLayout(const WrappedFlutingParams& params);

/// How the spiral's advance is given. Pitch: the distance along the axis for one full turn of a strand.
/// Spacing: the distance along the axis between neighbouring strands (pitch = spacing x strands). Turns: the
/// number of full turns over the whole length (pitch = length / turns).
enum class SpiralPitchMode { Pitch, Spacing, Turns };

/// Direction of the twist of the wrapped helix as seen on the cylinder (as built by `CylinderMap`).
enum class SpiralTwist { RightHand, LeftHand };

struct WrappedSpiralParams {
    bool alongX{true};
    double diameter{50.0};
    /// The job's Flip Design: it mirrors the design round the cylinder, so it swaps the twist in the flat.
    bool flip{false};
    /// Number of strands, evenly spaced round the circumference (1..kMaxLayoutStrands).
    int strands{1};
    double alongStart{0.0};
    double alongEnd{100.0};
    double circStart{0.0};
    /// Offset of the first strand's start from `circStart` (mm; any value, taken modulo the turn).
    double offset{0.0};
    SpiralPitchMode mode{SpiralPitchMode::Pitch};
    /// Pitch, spacing (both mm > 0) or number of turns (> 0), by `mode`.
    double value{25.0};
    SpiralTwist twist{SpiralTwist::RightHand};
};

struct WrappedSpiralResult {
    /// The strands in order; a strand that crosses the edge of the turn is several vectors (each a straight
    /// line in the flat design), listed in order along its length.
    std::vector<Contour> contours;
    std::string error;
    /// The pitch used and the turns made over the length.
    double pitch{0.0};
    double turns{0.0};
    /// Vectors per strand (strand s owns the next `segments[s]` contours).
    std::vector<int> segments;
    bool ok() const { return error.empty(); }
};

/// Flat circumference coordinate change per unit along the axis is `sign x circumference / pitch`, with sign +1
/// when the circumference coordinate increases along the axis. The twist the wrapped helix has for that sign
/// (right-hand, as `CylinderMap` wraps it) needs sign = -1 for a cylinder along X, +1 along Y, negated by flip.
WrappedSpiralResult wrappedSpiralLayout(const WrappedSpiralParams& params);

}  // namespace carvetto::geometry
