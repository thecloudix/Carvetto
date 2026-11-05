#pragma once

// Internal to the V-Carve toolpath (T-0160, ADR-0025 §6): corner sharpening of flat areas. With a
// flat depth and clearance end mills, each convex corner of a flat area keeps a sloped "tent" of
// floor between the wall ring's corner (where the carving tool runs at the flat depth) and the
// fillet the end mill leaves; these functions find such corners from the region's boundary and
// plan the carving tool's passes that clear them. In a header of its own so the tests can check
// the detection directly.

#include <cstddef>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/box_index.hpp"

namespace carvetto::toolpath::detail {

/// The even-odd region of closed polygons (the flat area's wall rings: the carving tool's centre
/// may go anywhere inside at the flat depth), with its edges indexed by box.
class RingRegion {
   public:
    /// `rings`: the rings' vertices (each ring closed implicitly); `eps`: points within this
    /// distance of an edge count as inside (rounding of computed points on the rings).
    RingRegion(std::vector<std::vector<geometry::Vec2>> rings, double eps);
    bool isEmpty() const { return edges_.empty(); }
    /// Even-odd membership of p (points within eps of an edge count as inside).
    bool inside(geometry::Vec2 p) const;
    /// Parameter intervals [t0, t1] of the segment a → b that lie inside, ascending, disjoint.
    std::vector<std::pair<double, double>> clip(geometry::Vec2 a, geometry::Vec2 b) const;
    /// True when the whole segment a → b lies inside.
    bool segmentInside(geometry::Vec2 a, geometry::Vec2 b) const;

   private:
    struct Edge {
        geometry::Vec2 a;
        geometry::Vec2 b;
    };
    std::vector<Edge> edges_;
    BoxIndex index_;
    geometry::Box2 bounds_;
    double eps_{0.0};
};

/// What corner sharpening needs to know about the carve (ADR-0025 §6). Lengths in mm.
struct CornerSpec {
    const tooling::Tool* tool{nullptr};  ///< the carving tool (V-bit or ball nose)
    double limit{0.0};                   ///< the depth limit F below the start plane
    double limitRadius{0.0};             ///< ρ: the carving tool's footprint radius at F
    double ringInset{0.0};               ///< distance of the wall rings from the walls (ρ + Pocket's chordal)
    double pocketChordal{0.0};           ///< Pocket's chordal tolerance (its safety margin)
    std::vector<double> millRadii;       ///< radii of the clearance end mills
    double stepover{0.0};                ///< the carving tool's stepover (clamped as for its rest clearing)
    double chordal{0.0};                 ///< the user's chordal tolerance (threshold and overlap)
    double distance{0.0};                ///< the user's distance tolerance
};

/// A convex corner of a flat area whose predicted leftover exceeds the chordal tolerance.
struct SharpCorner {
    geometry::Vec2 vertex;    ///< the corner of the boundary
    geometry::Vec2 bisector;  ///< unit, into the region
    double angle{0.0};        ///< interior angle α (radians, < π)
    double millRadius{0.0};   ///< radius of the end mill whose fillet bounds the corner's leftover
    double predicted{0.0};    ///< predicted leftover height without sharpening (mm)
};

/// Predicted height of the material left in a convex corner of interior angle `angle` between
/// the wall ring's corner and the fillet of an end mill of radius `millRadius`, measured from the
/// wall ring's edges: min(F, h(R·(1 − sin(α/2)))), h the carving tool's height above its tip at
/// that distance from its axis (V-bit: d / tan(θ/2)). An upper bound of the real leftover (the
/// skeleton cut along the bisector, when there is one, lowers it).
double predictedCornerLeftover(const CornerSpec& spec, double angle, double millRadius);

/// The convex corners of the region bounded by `boundaries` ([0] the outer, then its holes) that
/// sharpening clears: found from the boundary (junctions of spans whose direction turns towards
/// the inside), whose wall-ring corner lies inside `walls` (a flat-area corner), which some end
/// mill reaches (its centre at the corner of its centre region is clear of every wall: the floor
/// is clean beyond its fillet; the smallest such end mill bounds the leftover), and whose
/// predicted leftover exceeds `spec.chordal`.
std::vector<SharpCorner> sharpCorners(const std::vector<geometry::Contour>& boundaries, const RingRegion& walls,
                                      const CornerSpec& spec);

/// The carving tool's passes at the flat depth that clear `corner` (ADR-0025 §6): a groove along
/// the bisector and parallel passes either side at `spec.stepover`, each from the wall ring into
/// the end mill's swept area (overlap ≥ min(chordal, R/2)), clipped to `walls`. Consecutive passes
/// are joined at depth where the straight link stays inside `walls`; each returned polyline is
/// one cut.
std::vector<std::vector<geometry::Vec2>> cornerPasses(const SharpCorner& corner, const RingRegion& walls,
                                                      const CornerSpec& spec);

}  // namespace carvetto::toolpath::detail
