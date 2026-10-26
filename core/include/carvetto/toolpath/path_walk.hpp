#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Shared toolpath machinery (T-0120, ADR-0016): walking a cut contour by arc length and emitting
// moves with a height profile. Used by the profile toolpath for ramps, tabs, overcut and leads;
// written to be reused by other operations (pocket boundary passes, ramped entries).

namespace carvetto::toolpath {

/// A contour of line and arc spans indexed by arc length `u` (mm from its start point). Bezier
/// spans are polygonised at `tol.chordal` on construction. A closed path is periodic: positions
/// outside [0, length) wrap; an open path clamps to [0, length].
class ArcLengthPath {
   public:
    ArcLengthPath() = default;
    ArcLengthPath(const geometry::Contour& contour, const geometry::Tolerance& tol);

    /// The flattened contour the path walks (lines and arcs only).
    const geometry::Contour& contour() const { return contour_; }
    bool closed() const { return contour_.closed; }
    bool isEmpty() const { return contour_.spans.empty(); }
    double length() const { return cum_.empty() ? 0.0 : cum_.back(); }
    std::size_t spanCount() const { return contour_.spans.size(); }
    /// Arc length at which span `i` starts.
    double spanStart(std::size_t i) const { return cum_[i]; }
    /// Arc length of span `i`.
    double spanLength(std::size_t i) const { return cum_[i + 1] - cum_[i]; }

    /// Wraps `u` into [0, length) for a closed path; clamps it to [0, length] for an open one.
    double wrap(double u) const;
    /// Point at arc length `u`.
    geometry::Vec2 pointAt(double u) const;
    /// Unit direction of travel at arc length `u` (at a junction: the outgoing span's tangent).
    geometry::Vec2 tangentAt(double u) const;
    /// Spans covering the stretch from `u0` to `u1`; travels backwards (reversed spans) when
    /// `u1 < u0`. Closed paths wrap (the stretch may be longer than one revolution); open paths
    /// clamp to their ends. Lines and arcs are cut exactly; an uncut span is returned unchanged.
    std::vector<geometry::Span> extract(double u0, double u1) const;
    /// Arc length of the path point nearest to `p` (ties: lowest span index, as
    /// `geometry::closestPoint`).
    double project(geometry::Vec2 p) const;

    /// A junction where the direction of travel turns abruptly.
    struct Kink {
        /// Arc length of the junction, in [0, length).
        double u{0.0};
        /// Signed turn angle in radians (positive = turning left).
        double turn{0.0};
    };
    /// Junctions whose turn exceeds `minAngle` (radians). For a closed path the closing junction
    /// counts (at u = 0).
    std::vector<Kink> kinks(double minAngle) const;

   private:
    std::size_t spanAt(double u) const;
    geometry::Contour contour_;
    std::vector<double> cum_;  // cum_[i] = arc length at start of span i; back() = total
};

/// Height knot of a `ZProfile`: the profile may jump at `s` from `zBefore` to `zAfter`.
struct ZKnot {
    double s{0.0};
    double zBefore{0.0};
    double zAfter{0.0};
    bool operator==(const ZKnot&) const = default;
};

/// Piecewise-linear tool height as a function of travelled distance `s` along a leg. Between
/// consecutive knots the height runs linearly from the first knot's `zAfter` to the second's
/// `zBefore`; it is constant before the first and after the last knot. Knots are sorted by `s`.
struct ZProfile {
    std::vector<ZKnot> knots;

    /// Constant height.
    static ZProfile flat(double z) { return ZProfile{{ZKnot{0.0, z, z}}}; }
    /// Linear descent (or climb) from `z0` at s = 0 to `z1` at s = `length`, then constant.
    static ZProfile linear(double z0, double z1, double length);
    /// Height approaching `s` from below.
    double before(double s) const;
    /// Height leaving `s`.
    double after(double s) const;
    /// Point-wise maximum of two profiles (crossings become knots; redundant knots removed).
    static ZProfile max(const ZProfile& a, const ZProfile& b);
};

/// Extra moves spliced into a leg at travelled distance `s`. They must end where they started.
struct LegInsert {
    double s{0.0};
    std::vector<Move> moves;
};

/// Appends the moves that walk `path` from `u0` to `u1` (backwards when `u1 < u0`) with the
/// height `z(s)`, s = distance travelled from `u0`. The tool is expected at `path.pointAt(u0)`;
/// if it is there at another height, a vertical move (Plunge down, Feed up) is emitted first.
/// Lines become Feed moves, arcs ArcCW/ArcCCW moves (helical where the height changes, which is
/// exact because the height is linear in arc length between knots). Jumps in the profile become
/// vertical moves. Arcs whose chord is shorter than `minArcChord` are written as Feed moves (a
/// controller would read such an arc as a full circle after rounding); a whole revolution is
/// one arc move ending where it starts (ADR-0008 full-circle convention).
void appendLeg(const ArcLengthPath& path, double u0, double u1, const ZProfile& z, Toolpath& out,
               const std::vector<LegInsert>& inserts = {}, double minArcChord = 0.01);

}  // namespace carvetto::toolpath
