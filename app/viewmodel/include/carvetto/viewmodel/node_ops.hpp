#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"

// Pure contour-level node editing (T-0206, ADR-0030). Nothing here knows about documents, the
// view or undo; `NodeEditor` (node_edit.hpp) composes these into undoable operations.
//
// Node numbering: a closed contour of n spans has n nodes, node i is the start of span i (and the
// end of span i-1, node 0 the end of the last span); an open contour has n + 1 nodes, node n being
// the end of the last span. Span indices are the contour's.
//
// Conventions
//   * Every function leaves its input untouched on failure: it returns false / nullopt / an empty
//     list and, when `why` is given, a short reason.
//   * Results are checked with `validContour`: finite coordinates within `kMaxCoordinateMm`, no
//     span shorter than `kMinSpanMm`.
//   * Arcs keep their bulge (tan(sweep / 4), positive counter-clockwise) when an end moves.
//   * A full-circle arc has no nodes of its own; `normalised` splits it into two half arcs and the
//     editor always works on normalised contours.

namespace carvetto::viewmodel::nodeops {

inline constexpr double kMaxCoordinateMm = 1.0e9;
inline constexpr double kMinSpanMm = 1.0e-6;

/// Nodes of the contour (0 for an empty one).
std::size_t nodeCount(const geometry::Contour& c);
/// Position of node `node` (must be < nodeCount).
geometry::Vec2 nodePoint(const geometry::Contour& c, std::size_t node);
/// Copy with every full-circle arc replaced by two half arcs (a no-op copy otherwise).
geometry::Contour normalised(const geometry::Contour& c);
/// True when `normalised(c) == c`.
bool isNormalised(const geometry::Contour& c);
/// Finite coordinates within `kMaxCoordinateMm` and every span at least `kMinSpanMm` long. Gaps
/// between consecutive spans are not checked (imported data may carry them).
bool validContour(const geometry::Contour& c, std::string* why = nullptr);

/// Arc from `a` to `b` with the signed sweep (positive counter-clockwise, |sweep| in (0, 2 pi)).
std::optional<geometry::ArcSpan> arcFromSweep(geometry::Vec2 a, geometry::Vec2 b, double sweep);
/// Arc from `a` through `p` to `b`; nullopt when the three points are (nearly) collinear.
std::optional<geometry::ArcSpan> arcThrough(geometry::Vec2 a, geometry::Vec2 p, geometry::Vec2 b);
/// Arc from `a` to `b` with bulge = tan(sweep / 4) (positive counter-clockwise).
std::optional<geometry::ArcSpan> arcFromBulge(geometry::Vec2 a, geometry::Vec2 b, double bulge);
/// The point at parameter 0.5 of the span (the "virtual midpoint" handle).
geometry::Vec2 midHandle(const geometry::Span& s);

// ------------------------------------------------------------------------------------- moving

/// Moves nodes to absolute positions. Lines and Beziers end exactly there (Bezier handles keep
/// their offset from the node); arcs keep their bulge. False (contour untouched) when a span would
/// degenerate or the result is invalid.
bool moveNodes(geometry::Contour& c, const std::map<std::size_t, geometry::Vec2>& targets, std::string* why = nullptr);
/// Places control point `which` (0 = p1, 1 = p2) of Bezier span `span`. With `keepTangency` the
/// handle on the other side of the node, when that span is a Bezier too, is turned to stay
/// collinear (its length is kept).
bool setControlPoint(geometry::Contour& c, std::size_t span, int which, geometry::Vec2 pos, bool keepTangency,
                     std::string* why = nullptr);
/// Reshapes span `span` by dragging its point at parameter `t` by `delta`: a Bezier moves its
/// control points (the curve at the clamped parameter moves by `delta`), an arc is re-fitted
/// through its end points and the moved point, a line moves with both end nodes.
bool reshapeSpan(geometry::Contour& c, std::size_t span, double t, geometry::Vec2 delta, std::string* why = nullptr);
/// Moves the whole span (both end nodes, hence neighbours follow) by `delta`.
bool moveSpan(geometry::Contour& c, std::size_t span, geometry::Vec2 delta, std::string* why = nullptr);

// ------------------------------------------------------------------------------------ structure

/// Splits `s` at parameter `t` in (0,1) into two spans that together describe exactly the same
/// curve (line, arc by angle, Bezier by de Casteljau).
std::pair<geometry::Span, geometry::Span> splitSpan(const geometry::Span& s, double t);

/// Inserts a node into span `span` at parameter `t`; the new node's index is `span + 1`. Refused
/// when the point is closer than `kMinSpanMm` to an end of the span.
std::optional<geometry::Contour> insertPoint(const geometry::Contour& c, std::size_t span, double t,
                                             std::string* why = nullptr);

/// Removes the nodes (indices in `nodes`); the spans around a removed node are re-joined into one:
/// two lines give a line, anything with a Bezier gives a Bezier keeping the outer handles (a line
/// or arc side contributes a handle a third of the way along the new chord), otherwise a line.
/// On an open contour removing an end node drops the span at that end. Refused when fewer than two
/// nodes (closed: three for a line-only contour) or no span would remain.
std::optional<geometry::Contour> deleteNodes(const geometry::Contour& c, const std::set<std::size_t>& nodes,
                                             std::string* why = nullptr);

/// Removes spans: a closed contour opens up (one open contour starting after the removed run), an
/// open contour splits into the parts before / after. Empty result = every span removed.
std::vector<geometry::Contour> deleteSpans(const geometry::Contour& c, const std::set<std::size_t>& spans);

/// Splits at the nodes into open contours that together cover exactly the original spans. A closed
/// contour cut at k nodes gives k open pieces; an open one gives k + 1 (end nodes cannot be cut).
/// Empty on failure.
std::vector<geometry::Contour> cutAtNodes(const geometry::Contour& c, const std::set<std::size_t>& nodes,
                                          std::string* why = nullptr);

enum class SpanTarget { Line, Bezier, Arc };

struct ConvertSpec {
    SpanTarget target{SpanTarget::Line};
    /// Arc / Bezier: the curve passes through this point (an arc through it at the sweep side; a
    /// Bezier through it at t = 0.5).
    std::optional<geometry::Vec2> through;
    /// Arc: bulge = tan(sweep / 4), positive counter-clockwise. Used when `through` is not given.
    std::optional<double> bulge;
};

/// Default bulge for a line converted to an arc without a point or bulge (a shallow arc, about 56
/// degrees of sweep).
inline constexpr double kDefaultBulge = 0.25;

/// Replaces span `span` by the converted span(s) (an arc to Bezier gives up to four). The end
/// points are kept. Without `through` / `bulge`: line to Bezier = handles a third along the
/// chord, Bezier to arc = through the Bezier's midpoint, line to arc = `kDefaultBulge`.
/// Returns the new contour and how many spans replaced the one (via `produced`).
std::optional<geometry::Contour> convertSpan(const geometry::Contour& c, std::size_t span, const ConvertSpec& spec,
                                             std::size_t* produced = nullptr, std::string* why = nullptr);

/// Makes the tangent continuous at the given nodes (closed: any; open: interior only). Two Beziers
/// share the bisector direction of the incoming and outgoing tangents (handle lengths kept); a
/// Bezier next to a line or arc takes the direction of that side; two lines are converted to
/// Beziers first; an arc next to a line / arc keeps its geometry and the outgoing span is converted
/// and aligned. Nodes that are already smooth are skipped; refused when none changes or a node is
/// a cusp. The span count can change (arcs converted).
std::optional<geometry::Contour> smoothNodes(const geometry::Contour& c, const std::set<std::size_t>& nodes,
                                             std::string* why = nullptr);

/// Closed contour rotated so that `node` becomes node 0 (the start); refused for open contours and
/// for node 0.
std::optional<geometry::Contour> rotateStart(const geometry::Contour& c, std::size_t node, std::string* why = nullptr);

}  // namespace carvetto::viewmodel::nodeops
