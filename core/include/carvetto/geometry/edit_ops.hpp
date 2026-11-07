#pragma once

// Geometry edit operations behind the Fillet / Trim / Extend / Join / Close tools (T-0208,
// ADR-0028). Pure functions on contours: nothing here knows about documents or the UI.
// Units mm, angles radians. Every function returns a status instead of throwing; on a non-Ok
// status the result contour(s) are empty and the input is untouched.

#include <cstddef>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

/// Outcome of an edit operation.
enum class EditStatus {
    Ok,
    InvalidArgument,  ///< NaN / infinite / non-positive radius, tolerance or point; any coordinate, radius or
                      ///< tolerance above 1e12 mm (kMaxMagnitude); or, when a Bezier is involved in a trim /
                      ///< extend, a drawing larger than 1e6 x tol.chordal or a tol.chordal within 4 ulps of the
                      ///< coordinates (too costly: scale the tolerance with the drawing; ADR-0028 section 7).
                      ///< `joinContours` only refuses a bad tolerance: unusable contours pass through.
    EmptyContour,     ///< No spans to work on.
    BadNode,          ///< Node index outside the contour (open contours: the two ends are not corners).
    Discontinuous,    ///< The two spans at a node do not meet within the tolerance.
    DegenerateSpan,   ///< A span at the node (or at the end) has no usable length or tangent.
    NoCorner,         ///< Tangent-continuous node (nothing to round) or a contour too short to have one.
    Cusp,             ///< The spans fold back onto each other (turn of ~180 degrees).
    RadiusTooLarge,   ///< The fillet / relief does not fit on the spans (and clamping was not allowed/possible).
    NotClosed,        ///< A boundary contour is open.
    NothingToTrim,    ///< No intersection around the picked point.
    ClosedContour,    ///< Extend needs an open contour.
    NoIntersection,   ///< Extend found no target ahead of the end.
};

/// Human-readable reason for a status.
const char* toString(EditStatus s);

// ------------------------------------------------------------------------------------ fillet

/// WARNING (Dog-Bone / T-Bone): the relief is cut on the OUTER side of the turn at the node, so it
/// is only a relief at an internal corner of a pocket, i.e. a convex vertex of the region (a left
/// turn of a counter-clockwise contour, a right turn of a clockwise one). `filletCorner` applies
/// it at whatever node it is given, so at a reflex vertex it carves a bump into the region (and on
/// a contour of the opposite winding the area change is negative). `filletAll` without an explicit
/// node list therefore uses `convexNodes` for these two kinds; Normal fillets use every corner.
enum class FilletKind {
    Normal,   ///< Tangent circular arc of the given radius, rounding the corner.
    DogBone,  ///< Relief arc centred on the corner node with the given radius; sweep pi + |turn|, i.e. always
              ///< more than pi (exactly 3 pi / 2 for a right-angle corner).
    TBone,    ///< Relief semicircle whose diameter lies on one span, starting at the node.
};

/// Nodes where a relief cut (Dog-Bone / T-Bone) makes sense: corners that turn left on a
/// counter-clockwise closed contour, right on a clockwise one (convex vertices of the region).
/// An open contour has no region; its left turns are returned (it is treated as counter-clockwise).
/// Tangent-continuous nodes, cusps, gaps and contours that are not finite give none. Ascending.
std::vector<std::size_t> convexNodes(const Contour& contour, const Tolerance& tol = Tolerance::defaults());

struct FilletOptions {
    /// When the radius does not fit, use the largest radius that does (reported) instead of refusing.
    bool clampToFit{false};
    /// T-Bone only: put the relief on the incoming span (ending at the node) instead of the
    /// outgoing one (starting at the node).
    bool tBoneAlongIncoming{false};
};

struct FilletResult {
    EditStatus status{EditStatus::Ok};
    Contour contour;
    /// Radius actually used (differs from the request when `clamped`).
    double appliedRadius{0.0};
    bool clamped{false};
    /// Index in `contour.spans` of the inserted arc (valid when status is Ok).
    std::size_t arcSpan{0};
};

/// Rounds / relieves the corner at `nodeIndex`: the node between span nodeIndex-1 and span
/// nodeIndex (node 0 of a closed contour joins the last span and the first). For an open
/// contour valid nodes are 1 .. spanCount-1.
FilletResult filletCorner(const Contour& contour, std::size_t nodeIndex, double radius, FilletKind kind,
                          const FilletOptions& options = {}, const Tolerance& tol = Tolerance::defaults());

struct CornerReport {
    std::size_t node{0};
    EditStatus status{EditStatus::Ok};
    double appliedRadius{0.0};
    bool clamped{false};
};

struct FilletAllResult {
    /// Invalid arguments / empty contour only; per-corner failures are in `corners`.
    EditStatus status{EditStatus::Ok};
    Contour contour;
    /// One entry per requested node, ascending by node.
    std::vector<CornerReport> corners;
    std::size_t applied{0};
};

/// Applies the fillet at every corner of the contour (nodes that are tangent-continuous report
/// NoCorner and are left alone). For DogBone and TBone only the `convexNodes` are used.
FilletAllResult filletAll(const Contour& contour, double radius, FilletKind kind, const FilletOptions& options = {},
                          const Tolerance& tol = Tolerance::defaults());
/// As above, for the given nodes only (duplicates ignored; invalid ones report BadNode).
FilletAllResult filletAll(const Contour& contour, double radius, FilletKind kind, const std::vector<std::size_t>& nodes,
                          const FilletOptions& options = {}, const Tolerance& tol = Tolerance::defaults());

// ------------------------------------------------------------------------------------- trim

enum class TrimSide { ClearInside, ClearOutside };

struct TrimResult {
    EditStatus status{EditStatus::Ok};
    /// Surviving contours (closed ones unchanged when untouched), in input order.
    std::vector<Contour> contours;
    /// For each output contour, the index of the input contour it came from.
    std::vector<std::size_t> sources;
};

/// Cuts every contour at its exact intersections with the (closed) boundary contours and
/// removes the pieces inside / outside the region the boundaries describe under `rule`. Pieces
/// lying on the boundary (within tol.distance) are kept. Curves are cut exactly (no
/// polygonising). `refitCurves` additionally merges collinear neighbouring lines and
/// neighbouring arcs of one circle in the result.
TrimResult trimToBoundary(const std::vector<Contour>& contours, const std::vector<Contour>& boundaries, TrimSide side,
                          bool refitCurves = false, const Tolerance& tol = Tolerance::defaults(),
                          FillRule rule = FillRule::NonZero);

struct TrimAtOptions {
    /// Join the remaining sections again where they meet at the trimmed crossing.
    bool rejoin{false};
};

struct TrimAtResult {
    EditStatus status{EditStatus::Ok};
    std::vector<Contour> contours;
};

/// Interactive trim: removes the run of the contour around `pick` between the nearest
/// intersections on either side (crossings with the contour itself and with `others`). An open
/// contour trims to its end where there is no intersection on that side; a closed contour needs
/// at least two cut points (distinct along the contour). A pick exactly on a crossing removes the
/// run after it.
TrimAtResult trimSpansAt(const Contour& contour, Vec2 pick, const std::vector<Contour>& others = {},
                         const TrimAtOptions& options = {}, const Tolerance& tol = Tolerance::defaults());

// ----------------------------------------------------------------------------------- extend

enum class ContourEnd { Start, End };

struct ExtendResult {
    EditStatus status{EditStatus::Ok};
    Contour contour;
    /// New end point (exactly the intersection) and the target it landed on.
    Vec2 point;
    std::size_t targetContour{0};
    std::size_t targetSpan{0};
    /// Distance travelled along the extension.
    double distance{0.0};
};

/// Extends the open contour's `end` to the nearest intersection ahead with the target contours.
/// A line end is lengthened; an arc end continues round its circle; a Bezier end gets a tangent
/// line appended (ADR-0028). Hits closer than tol.distance to the end are ignored. For an arc end
/// whose remaining circle is longer than 1000 x reach (reach = 2 max(D, 1), D = diagonal of the box
/// of the end point and all targets) the probe is limited to the first `reach` of arc: hits that
/// would need travelling farther (wrap-around hits on the far side of a huge circle, targets
/// behind the end reached by going almost all the way round) are reported as NoIntersection, and
/// within the probe `distance` / `point` may differ from an unbounded probe in the last digits.
ExtendResult extendSpan(const Contour& contour, ContourEnd end, const std::vector<Contour>& targets,
                        const Tolerance& tol = Tolerance::defaults());

// ------------------------------------------------------------------------------- join / close

struct JoinResult {
    EditStatus status{EditStatus::Ok};
    /// All output contours: chains in order of their first input contour; closed inputs pass through.
    std::vector<Contour> contours;
    /// For each output contour, its input contour indices in chain order.
    std::vector<std::vector<std::size_t>> sources;
    /// Output indices of open contours that found no partner (single source, still open).
    std::vector<std::size_t> unmatched;
};

/// Chains open contours whose ends lie within tol.distance of each other (reversing as needed).
/// Closed, empty and unusable contours (non-finite, or a coordinate / radius above 1e12 mm) take no
/// part: they pass through unjoined, in place, with status Ok; only an invalid tolerance gives
/// InvalidArgument.
/// Deterministic: chains are grown from the lowest unused input, nearest partner first (ties to
/// the lowest index, a partner's start before its end). Gaps up to the tolerance are kept (not
/// bridged). A chain whose ends meet within the tolerance becomes closed.
JoinResult joinContours(const std::vector<Contour>& contours, const Tolerance& tol = Tolerance::defaults());

enum class CloseAction { AlreadyClosed, Snapped, LineAdded };

struct CloseResult {
    EditStatus status{EditStatus::Ok};
    Contour contour;
    CloseAction action{CloseAction::AlreadyClosed};
};

/// Closes an open contour: a gap within tol.distance is snapped shut (the end of the last span,
/// or failing that the start of the first, moves onto the other when it is a line or a Bezier),
/// a larger gap gets a closing line.
CloseResult closeContour(const Contour& contour, const Tolerance& tol = Tolerance::defaults());

}  // namespace carvetto::geometry
