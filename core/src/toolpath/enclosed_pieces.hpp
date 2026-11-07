#pragma once

// Internal to core (profile.cpp) and its tests (T-0140 rounds 2 and 3, ADR-0016 §14): the pieces
// of the plane that a union of closed rings encloses, in near-linear time; the band polygons
// Profile builds for them; and the proof that lets most bands go unsearched.
//
// One Clipper2 union over a whole scene of overlapping bands is a single sweep whose active edge
// list holds every band side by side, so it costs about the square of the bands across the scene
// (a row or grid of shapes whose kerfs overlap). Here the plane is cut into leaves instead: a k-d
// subdivision at median vertex coordinates (on the nanometre grid, at least two grid units off
// every vertex) until each leaf holds at most `leafVertices` vertices, or a cut would leave more
// than 4/5 of them on one side or a side narrower than 10 um (10 distance tolerances, when
// larger); every ring reaching within a micrometre of a cut is clipped to both sides a micrometre
// beyond it (Sutherland-Hodgman, which keeps the winding number of every point on that side). A
// ring that surrounds a leaf without an edge in it only adds a constant winding number there and
// is taken out on the way down. Each leaf's material is its rectangle less the union of its rings
// (one small Clipper2 call that drops exact duplicate vertices only); where a ring crosses a leaf
// side Clipper2 truncates the crossing, so output vertices within a grid unit of a side are moved
// onto it (round 3: a part's edge along a side could end a unit off it and the join was lost).
// Parts of material that meet along a shared leaf side over at least 10 nm are joined
// (union-find over the intervals, one sort per cut line); a part on the outer frame is the
// outside. Every other class is a piece: the area of its parts less their islands; and it is
// thick when what is left of it eroded by `clearance` is not empty (each part alone first, whose
// cut edges can only make that smaller, by a point at least `clearance` from its edges or by its
// erosion; when no part alone is thick, the parts together, grown by twice the distance tolerance
// to close the cracks their conversion for Clipper2 can open along leaf sides, and eroded that
// much more).

#include <cstddef>
#include <optional>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::toolpath::detail {

/// A bounded piece of material left by the union of the rings.
struct EnclosedPiece {
    geometry::Box2 bounds;
    double area{0.0};  ///< less its islands
    /// A point of the piece at least `clearance` from the rings' union (a vertex of the eroded
    /// piece) when the piece is thicker than 2 * clearance somewhere; empty when it is not, or when
    /// its area is under `minArea` (not tested).
    std::optional<geometry::Vec2> inner;
    /// The piece's lowest vertex of least x (the same at every leaf size, to the grid): a fixed
    /// point to attribute the piece by.
    geometry::Vec2 anchor{};
};

/// Default most vertices per leaf.
inline constexpr std::size_t kEnclosedLeafVertices = 512;

struct EnclosedOptions {
    double minArea{0.0};    ///< pieces under this area are dropped
    double clearance{0.0};  ///< erosion radius of the thickness test
    geometry::Tolerance tol{};
    double mitreLimit{2.0};
    std::size_t leafVertices{kEnclosedLeafVertices};  ///< most vertices per leaf (tests use few)
};

/// A closed polygon (no repeated closing point).
using PointRing = std::vector<geometry::Vec2>;

/// The pieces the union of `rings` (filled NonZero) encloses with area at least `minArea`, in a
/// deterministic order (by their lowest leaf). See the file comment.
std::vector<EnclosedPiece> enclosedPieces(std::vector<PointRing> rings, const EnclosedOptions& opt);

/// The band of an open path of lines and arcs (every point within `w` of it), as one closed
/// polygon whose winding number is positive exactly on the band (T-0140): the sum of the
/// boundaries of positively wound pieces that make up the band, each span's (a line's rectangle
/// along its normals; an arc's annular sector between radii rho - w and rho + w, or with
/// rho <= w its sector out to rho + w from the centre), a wedge of radius w about each corner on
/// its outer side and a half disc at each end. Every point of the band lies on a normal of its
/// nearest point of the path within w of it, or in a wedge or end disc, so the pieces cover it
/// exactly; pieces meeting at a vertex share their edge along the normal there, which cancels,
/// so the polygon runs along the outer chain of offsets, back along the inner one, and through
/// the vertex at an inner corner (and through the centre of an arc tighter than w). Clipper2's
/// offset puts such a spike at every vertex of the polygonised path, which costs about the
/// square of the vertices on each arc; here arcs stay arcs until the offsets are polygonised,
/// and a gentle inner corner between two lines needs no spike (at most 90 degrees, both lines at
/// least w sin(turn) long: their rectangles cover the triangle the vertex spans). The polygons
/// cover the true band and reach at most `chordal` beyond it (round 3): tangents outside its
/// convex arcs (offsets on the outer side, wedges, end discs), chords inside its concave ones. A closed path (`closedPath`, ending where it
/// starts) has no end discs: its last corner is one like the others (always through the vertex on
/// its inner side), and each chain is closed on itself instead, two rings whose winding numbers
/// add up to the same sum (the edges that would join them lie along the normal at the start, the
/// chains' ends on it, and cancel; as one ring they would leave Clipper2 a zero-width slit, which
/// it can keep in its output as a ring touching itself instead of an outer and a hole). As rings
/// of points; empty when the path holds a Bezier span.
std::vector<PointRing> bandOutline(const geometry::Contour& path, double w, double chordal, bool closedPath);

/// Which paths' bands must be searched for enclosed pieces (round 3), given bands that lie within
/// `reach` of their paths (every point within `reach` of the path): 0 for each path whose band,
/// together with the bands of the other paths, provably encloses nothing, and that lies clear of
/// every band that might; 1 otherwise. Searching the bands of the paths marked 1 alone finds every
/// piece the bands of all paths enclose that is thicker than twice the proof's slack (below). The proof covers each band by convex pieces, one per line and per
/// stretch of an arc whose sagitta is at most `sagitta` (the arc's convex hull grown by `reach`),
/// and adds them in order along the path: a union stays without holes when each new piece meets
/// what is there in one connected set (Mayer-Vietoris), which holds when every earlier piece it
/// meets either holds the path point `reach` back along the path (pieces whose stretches lie at
/// most 2 `reach` apart along it) or meets it at a point one of those also holds. Pieces of
/// different paths must not meet at all. A path that fails (or is closed, `closedPath`, or holds
/// a Bezier span) is searched, with every path whose band reaches into the box of its cluster
/// (paths whose pieces meet): a hole of the pieces lies inside one cluster's box. The proof's
/// pieces reach at most `reach` + `sagitta` from the path, so a hole of the bands they hide lies
/// within `reach` + `sagitta` - (the bands' true radius) of the bands (the proof's slack).
std::vector<char> pathsToSearch(const std::vector<geometry::Contour>& paths, const std::vector<char>& closedPath,
                                double reach, double sagitta);

}  // namespace carvetto::toolpath::detail
