#pragma once

// Internal to the V-Carve toolpath (T-0160, ADR-0025 §6): corner sharpening of flat areas. With a
// flat depth and clearance end mills, every convex stretch of a flat area's walls that the end
// mills cannot reach into (a sharp corner, a small fillet or chamfer, a junction of curved walls)
// keeps a sloped "tent" of floor between the wall ring (where the carving tool runs at the flat
// depth) and the end mills' swept area. These functions find such stretches from the actual wall
// rings and the actual end-mill moves, and plan the carving tool's passes that clear them. In a
// header of its own so the tests can check the detection directly.

#include <cstddef>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
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
    /// The rings as given.
    const std::vector<std::vector<geometry::Vec2>>& rings() const { return rings_; }
    /// Even-odd membership of p (points within eps of an edge count as inside).
    bool inside(geometry::Vec2 p) const;
    /// Distance from p to the nearest ring edge, or `cap` when none is closer than `cap`.
    double distance(geometry::Vec2 p, double cap) const;
    /// Parameter intervals [t0, t1] of the segment a → b that lie inside, ascending, disjoint.
    std::vector<std::pair<double, double>> clip(geometry::Vec2 a, geometry::Vec2 b) const;
    /// True when the whole segment a → b lies inside.
    bool segmentInside(geometry::Vec2 a, geometry::Vec2 b) const;

   private:
    struct Edge {
        geometry::Vec2 a;
        geometry::Vec2 b;
    };
    std::vector<std::vector<geometry::Vec2>> rings_;
    std::vector<Edge> edges_;
    BoxIndex index_;
    geometry::Box2 bounds_;
    double eps_{0.0};
};

/// Where a clearance end mill's flat bottom ran at the flat depth: a polyline of its centre and
/// the end mill's radius (everything within the radius of the polyline is swept flat).
struct SweptPath {
    std::vector<geometry::Vec2> points;
    double radius{0.0};
};

/// What corner sharpening needs to know about the carve (ADR-0025 §6). Lengths in mm.
struct CornerSpec {
    const tooling::Tool* tool{nullptr};  ///< the carving tool (V-bit or ball nose)
    double limit{0.0};                   ///< the depth limit F below the start plane
    double limitRadius{0.0};             ///< ρ: the carving tool's footprint radius at F
    double pocketChordal{0.0};           ///< Pocket's chordal tolerance c (its safety margin)
    double stepover{0.0};                ///< the carving tool's stepover s (clamped as for its rest clearing)
    double chordal{0.0};                 ///< the user's chordal tolerance (threshold and overlap)
    double distance{0.0};                ///< the user's distance tolerance
};

/// A convex stretch of a flat area's walls whose tent is sharpened: a connected patch of the floor
/// that no end mill sweeps and that lies farther than δ = min(r(chordal), s/2) from the wall
/// rings (r(z) the carving tool's footprint radius at depth z), with a point farther than
/// r(chordal) (predicted leftover above the chordal tolerance).
struct SharpStretch {
    std::vector<geometry::Vec2> outline;  ///< the patch's outer polygon (counter-clockwise)
    geometry::Vec2 root;                  ///< the patch's vertex farthest from the swept area (the corner side)
    geometry::Vec2 direction;             ///< unit, from `root` towards the patch's centroid (the bisector)
    double farthest{0.0};                 ///< greatest distance of the patch's outline from the wall rings
    double predicted{0.0};                ///< min(F, h(farthest)): the leftover at the end mills' fillet
};

/// The stretches to sharpen in a flat area whose wall rings are `walls`, where the end mills
/// swept `swept` at the flat depth (ADR-0025 §6). Patches in a flat area no end mill enters (one
/// outer wall ring with the rings inside it, where no swept path starts: the carving tool's own
/// rest clearing) are left alone.
std::vector<SharpStretch> sharpStretches(const RingRegion& walls, const std::vector<SweptPath>& swept,
                                         const CornerSpec& spec);

/// The carving tool's passes at the flat depth that clear `stretch` (ADR-0025 §6): lines
/// parallel to its direction at multiples of `spec.stepover` from its root (the first is the
/// groove along the bisector), each covering the extent of the patch within half a stepover of
/// it, from the wall ring (at most `backReach` back) to `spec.chordal` beyond the patch into the
/// swept area, clipped to `walls`. Consecutive passes are joined at depth where the straight link
/// stays inside `walls`; each returned polyline is one cut.
std::vector<std::vector<geometry::Vec2>> stretchPasses(const SharpStretch& stretch, const RingRegion& walls,
                                                       const CornerSpec& spec, double backReach);

}  // namespace carvetto::toolpath::detail
