#pragma once

#include <optional>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/geometry/vec3.hpp"

namespace carvetto::toolpath {

using geometry::Box2;
using geometry::Vec2;
using geometry::Vec3;

/// What a move does. Mirrors the block families a post-processor emits (ADR-0004).
enum class MoveKind {
    Rapid,    ///< Positioning move at the machine's rapid rate (G0-like), usually at safe Z.
    Feed,     ///< Straight cutting move at the feed rate.
    Plunge,   ///< Straight move into the material at the plunge rate (normally -Z only).
    Retract,  ///< Straight move out of the material back to safe Z (normally +Z only).
    ArcCW,    ///< Clockwise circular arc in the XY plane (Z may change linearly: helix).
    ArcCCW,   ///< Counter-clockwise circular arc in the XY plane (Z may change linearly: helix).
    Dwell,    ///< Pause for `dwellSeconds` at the current position; no motion.
};

/// True for Feed/Plunge/Retract/ArcCW/ArcCCW: moves that travel at a programmed feed rate.
constexpr bool isCutting(MoveKind k) {
    return k != MoveKind::Rapid && k != MoveKind::Dwell;
}
/// True for ArcCW and ArcCCW.
constexpr bool isArc(MoveKind k) {
    return k == MoveKind::ArcCW || k == MoveKind::ArcCCW;
}

/// One toolpath move (ADR-0008). Coordinates are absolute machine coordinates in millimetres,
/// Z up, describing the tool tip. A move starts where the previous move ended: the start point
/// is implicit, so a `Move` on its own is only meaningful inside a move list. The first move of a
/// toolpath is conventionally a `Rapid` whose start is the machine's current position.
///
/// Arcs are given by end point and XY centre; the radius is taken from the start point, the
/// direction from the kind. Coincident start/end (in XY) means a full circle, matching
/// `geometry::ArcSpan` (ADR-0005). A Z change over an arc is interpolated linearly (helix).
///
/// Dwell moves keep `to` equal to the previous end point and carry the pause in `dwellSeconds`.
struct Move {
    MoveKind kind{MoveKind::Rapid};
    /// End point of the move (for Dwell: the position where the tool pauses).
    Vec3 to;
    /// Arc centre in XY; only meaningful for ArcCW/ArcCCW.
    Vec2 centre;
    /// Feed rate for this move in mm/min, overriding the toolpath's default. Ignored for
    /// Rapid and Dwell. Empty = use the toolpath's feed (or plunge) rate.
    std::optional<double> feedOverride;
    /// Pause length in seconds; only meaningful for Dwell.
    double dwellSeconds{0.0};

    bool operator==(const Move&) const = default;

    static Move rapid(Vec3 to) { return {MoveKind::Rapid, to, {}, {}, 0.0}; }
    static Move feed(Vec3 to, std::optional<double> feedOverride = {}) {
        return {MoveKind::Feed, to, {}, feedOverride, 0.0};
    }
    static Move plunge(Vec3 to, std::optional<double> feedOverride = {}) {
        return {MoveKind::Plunge, to, {}, feedOverride, 0.0};
    }
    static Move retract(Vec3 to, std::optional<double> feedOverride = {}) {
        return {MoveKind::Retract, to, {}, feedOverride, 0.0};
    }
    static Move arcCW(Vec3 to, Vec2 centre, std::optional<double> feedOverride = {}) {
        return {MoveKind::ArcCW, to, centre, feedOverride, 0.0};
    }
    static Move arcCCW(Vec3 to, Vec2 centre, std::optional<double> feedOverride = {}) {
        return {MoveKind::ArcCCW, to, centre, feedOverride, 0.0};
    }
    /// Dwell at `at` (the previous move's end point) for `seconds`.
    static Move dwell(Vec3 at, double seconds) { return {MoveKind::Dwell, at, {}, {}, seconds}; }
};

/// Radius of an arc move, measured from `from` (its start point) to the centre.
double arcRadius(const Move& m, Vec2 from);
/// Signed swept angle of an arc move starting at `from`: in (0, 2*pi] for ArcCCW, in
/// [-2*pi, 0) for ArcCW. Coincident start/end gives a full circle. 0 for non-arc moves.
double arcSweep(const Move& m, Vec2 from);
/// Length of travel of a move that starts at `from`. Arcs include the helical Z component;
/// dwells have zero length.
double moveLength(const Move& m, Vec3 from);
/// Exact XY bounding box swept by a move that starts at `from`.
Box2 moveBounds(const Move& m, Vec3 from);

}  // namespace carvetto::toolpath
