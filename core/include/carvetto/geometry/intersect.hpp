#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// A single point where two spans meet. `tA`/`tB` are the spans' own parameters (see ADR-0005)
/// clamped to [0,1]; `point` is within `tol.distance` of both A(tA) and B(tB). `tangent` is set when the
/// curves touch without crossing (parallel tangents at the contact point), which includes collinear
/// end-to-end touches.
struct Intersection {
    Vec2 point;
    double tA{0.0};
    double tB{0.0};
    bool tangent{false};
};

/// A stretch along which two spans coincide (collinear line segments or arcs on the same
/// circle). Parameters are clamped to [0,1]; `tA0 < tA1` always, while `tB0`/`tB1` follow B's
/// direction of travel over the same stretch (so `tB0 > tB1` when B runs the other way).
/// Arcs on the same circle may yield two intervals when the overlap wraps across A's start.
struct Overlap {
    double tA0{0.0};
    double tA1{0.0};
    double tB0{0.0};
    double tB1{0.0};
};

/// How completely an intersection routine answered (ADR-0005 A2, T-0180).
enum class IntersectStatus {
    /// Everything was found (the answer of the routine's documented guarantees).
    Complete,
    /// The work budget (`IntersectLimits`) ran out: the hits and overlaps are those found before
    /// it did (best effort, possibly missing some), never wrong ones. Callers that need every
    /// intersection for correctness must refuse; callers that only look for candidates may use them.
    Truncated,
    /// A span has a non-finite coordinate or arc parameter (NaN or infinity): nothing was computed,
    /// the result is empty.
    InvalidInput,
    /// A span has a coordinate or arc radius beyond 1e12 mm in magnitude (`kMaxIntersectMagnitude`):
    /// nothing was computed, the result is empty. No CNC drawing is anywhere near it; past it the
    /// arithmetic of the Bezier routines overflows or loses all precision and their cost cannot be
    /// bounded (ADR-0005 A2).
    TooLarge,
};

/// Everything two spans have in common: isolated points and coincident stretches. Points that
/// lie inside a reported overlap are not repeated in `points`.
struct IntersectionResult {
    std::vector<Intersection> points;
    std::vector<Overlap> overlaps;
    IntersectStatus status{IntersectStatus::Complete};

    bool isEmpty() const { return points.empty() && overlaps.empty(); }
    /// True when `points`/`overlaps` are the complete answer (`status == Complete`).
    bool complete() const { return status == IntersectStatus::Complete; }
};

/// Largest coordinate or arc radius (mm) the intersection routines accept; see `IntersectStatus::TooLarge`.
inline constexpr double kMaxIntersectMagnitude = 1e12;

/// Work budget of the Bezier intersection routines. Work is counted in deterministic units
/// (a chord-pair test costs 1, a curve evaluation or nearest-point step costs tens, a Newton
/// refinement hundreds; about 4 ns of Release CPU per unit on a 2020s desktop core), so a
/// truncated result is reproducible and does not depend on the machine. Line and arc pairs are
/// closed-form and unlimited. The defaults let every input that finishes in a second or two
/// complete unchanged (ADR-0005 A2).
struct IntersectLimits {
    /// Most work one pair of spans may spend (`intersect(Span, Span)`).
    std::uint64_t pairWork{kDefaultPairWork};
    /// Most work one contour-level call (`intersect(Contour, Contour)`, `selfIntersections`,
    /// `hasSelfIntersections`) may spend over all its pairs.
    std::uint64_t totalWork{kDefaultTotalWork};

    static constexpr std::uint64_t kDefaultPairWork = 500'000'000;
    static constexpr std::uint64_t kDefaultTotalWork = 4'000'000'000;
};

/// Exact line–line intersection. Segments are collinear when both end points of the shorter lie
/// within `tol.distance` of the longer one's line; they then give an overlap, or a single tangent
/// point when they only touch end to end. Otherwise the lines are intersected however small the
/// angle between them. Every end point within `tol.distance` of the other segment is a contact
/// too (see `intersect(const Span&, const Span&, const Tolerance&)` for end contacts).
IntersectionResult intersect(const LineSpan& a, const LineSpan& b, const Tolerance& tol);
/// Exact line–arc intersection from the foot of the perpendicular dropped from the centre
/// (hits at ±sqrt(r² − h²) along the line), which stays precise for lines much longer than the
/// circle. Two crossings further apart than `tol.distance` are both reported; closer ones, and a
/// line passing outside the circle within `tol.distance`, give one tangent hit at the foot. Ends
/// within `tol.distance` of the other span are contacts too.
IntersectionResult intersect(const LineSpan& a, const ArcSpan& b, const Tolerance& tol);
/// Exact arc–arc intersection from the circle–circle solution. Two crossings further apart than
/// `tol.distance` are both reported. Circles that touch (closest approach within `tol.distance`,
/// externally or internally) or cross at two points the tolerance cannot separate give one tangent
/// hit at the midpoint of the two circles' facing points on the centre line, so it lies within
/// tolerance of both however close the centres are; this includes nearly concentric circles
/// (centre distance d <= tol.distance) whose closest approach |r1 - r2| - d is within tolerance.
/// Arcs on the same circle (d and |r1 - r2| both within tolerance) give overlaps; other concentric
/// circles give nothing. Ends within `tol.distance` of the other span are contacts too.
IntersectionResult intersect(const ArcSpan& a, const ArcSpan& b, const Tolerance& tol);

/// Intersection of any two spans. Line/arc pairs are solved analytically, in a canonical argument
/// order (so `intersect(b, a)` is exactly `intersect(a, b)` with parameters swapped). Pairs
/// involving a Bezier are cut into pieces over which each Bezier turns by at most 45 degrees (so
/// loops and cusps are handled branch by branch); each pair of pieces is polygonised at `tol.chordal`,
/// chord crossings are refined by Newton on A(t) - B(s) = 0, and the signed distance along every
/// Bezier chord near the other span is scanned for sign changes (crossings) and dips (shallow
/// double crossings, touches), including dips at chord and span ends and next to samples at zero
/// distance (a shared junction). Chord pairs that Newton cannot converge, and the samples near a
/// dip, are touch candidates: each is walked to the local minimum of the distance between the
/// unsplit curves (around the seam of a full circle) and kept when that minimum is within
/// `tol.distance`, flagged `tangent`. A closest approach with gap h <= tol.distance is reported
/// exactly once, wherever it lies along either span; curves running within tolerance of each other
/// report their crossings plus their local closest approaches, nothing in between.
///
/// End contacts (every pair kind): an end of either span within `tol.distance` of the other span
/// (its interior or its end) is reported as one hit, the end paired with its nearest point, unless
/// another hit lies within `tol.distance` of it. It is flagged `tangent` unless the end lies on the
/// other span (exactly, to 1e-6 tolerance), where the tangent rule of any exact contact applies.
/// This holds whether or not the end is a local minimum of the gap: an end inside a stretch where
/// the curves run within tolerance of each other, or on the slope down to a crossing further than
/// the tolerance away, is reported too. A full circle has no ends.
///
/// Degenerate (zero-length) spans are treated as points. Results are ordered by `tA`; hits within
/// `tol.distance` of each other are merged into one, and two distinct crossings merged that way
/// are flagged `tangent` (an end contact never counts as a crossing for this). Bezier pairs are
/// solved in a canonical argument order too. See ADR-0009 (amendments) for the guarantees and the
/// remaining heuristics.
///
/// Limitation: overlap detection is implemented for line/line and arc/arc pairs only. Coincident
/// stretches involving a Bezier are reported as tangent points along the shared part (its ends
/// and local closest approaches, sampled where the curves agree to rounding), not as `Overlap`s
/// (Bezier overlap detection is future work; see ADR-0009).
///
/// Work bounds (ADR-0005 A2): a pair involving a Bezier spends at most `limits.pairWork` units
/// (the overload without `limits` uses the defaults). When that is not enough the result has
/// `status == Truncated` and holds what was found so far. A span with a NaN or infinite
/// coordinate (control point, centre, radius, angle) gives an empty result with `status ==
/// InvalidInput`, and one with a coordinate or radius beyond `kMaxIntersectMagnitude` an empty
/// result with `status == TooLarge`, whatever the other span is.
IntersectionResult intersect(const Span& a, const Span& b, const Tolerance& tol);
IntersectionResult intersect(const Span& a, const Span& b, const Tolerance& tol, const IntersectLimits& limits);

/// An intersection between two contours (or two spans of one contour), tagged with the span
/// indices it belongs to. For self-intersections `spanA <= spanB`.
struct ContourIntersection {
    std::size_t spanA{0};
    std::size_t spanB{0};
    Intersection hit;
};

/// A coincident stretch between span `spanA` of one contour and span `spanB` of another.
struct ContourOverlap {
    std::size_t spanA{0};
    std::size_t spanB{0};
    Overlap overlap;
};

/// Points and overlaps shared by two contours (or by one contour with itself).
struct ContourIntersections {
    std::vector<ContourIntersection> points;
    std::vector<ContourOverlap> overlaps;
    /// Worst status of any span pair (InvalidInput over TooLarge over Truncated over Complete), or Truncated
    /// when `IntersectLimits::totalWork` ran out. Anything but Complete means the lists may miss
    /// intersections (and are empty for InvalidInput and TooLarge).
    IntersectStatus status{IntersectStatus::Complete};

    bool isEmpty() const { return points.empty() && overlaps.empty(); }
    bool complete() const { return status == IntersectStatus::Complete; }
};

/// All intersections between spans of `a` and spans of `b`. A hit at a junction shared by two
/// consecutive spans is reported once (first span wins). Ordered by (spanA, tA). Work is bounded as
/// for the span pairs (check `status`); the cost is near-linear in the spans and hits (box index
/// for the pairs, hash grid for the duplicate test).
ContourIntersections intersect(const Contour& a, const Contour& b, const Tolerance& tol);
ContourIntersections intersect(const Contour& a, const Contour& b, const Tolerance& tol, const IntersectLimits& limits);

/// Points where a contour crosses, touches or overlaps itself. The shared junction of two
/// consecutive spans (including the closing junction) does not count; everything else does,
/// including a crossing next to a junction and a cubic Bezier that loops back through itself
/// (reported with `spanA == spanB`; the double point is computed analytically, so thin sliver
/// loops are found, reported with `tA < tB`; a loop that stays within `tol.distance` of its double
/// point is a cusp and is not reported).
/// In a closed contour of two spans both junctions are ignored, and in a closed contour of one
/// span the span's end meeting its own start is ignored.
ContourIntersections selfIntersections(const Contour& c, const Tolerance& tol);
ContourIntersections selfIntersections(const Contour& c, const Tolerance& tol, const IntersectLimits& limits);

/// True when `selfIntersections(c, tol)` is non-empty (stops at the first hit). Conservative: also
/// true when no hit was found but the answer is not `Complete` (work budget spent, or a
/// non-finite span), because a self-intersection cannot be ruled out. Use
/// `selfIntersections(...).status` to tell the cases apart.
bool hasSelfIntersections(const Contour& c, const Tolerance& tol);

}  // namespace carvetto::geometry
