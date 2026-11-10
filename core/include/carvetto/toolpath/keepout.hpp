#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/toolpath.hpp"

namespace carvetto::toolpath {

/// How `KeepOutField::route` found a rapid.
enum class RouteStatus {
    Clear,          ///< The straight line misses every zone: no change.
    Routed,         ///< A detour around the zones was found (`RouteResult::points`).
    EndpointInside, ///< The start or end lies inside a zone (or its clearance band): cannot be routed.
    NoRoute,        ///< Both ends are free but no path exists (a closed-in pocket): cannot be routed.
};

struct RouteResult {
    RouteStatus status{RouteStatus::Clear};
    /// For `Routed`: the waypoints after the start, ending at the destination, in XY mm.
    std::vector<geometry::Vec2> points;
};

/// The keep-out zones of a document, prepared for routing (T-0241, ADR-0035): the closed
/// outlines are merged (union, non-zero fill) and grown by `clearance` mm with round corners, and
/// the result is held as polygons (curves polygonised at `tol.chordal`; every polygon vertex lies
/// on or outside the exact grown outline's chord band). A rapid is *blocked* when its segment
/// passes through the interior of that region; touching the boundary is allowed.
///
/// `route` finds the shortest XY path between two free points that stays out of the region
/// (visibility graph over the polygon vertices, Dijkstra). Visibility between vertices is
/// memoised, so one field serves a whole job. Not thread-safe.
class KeepOutField {
   public:
    /// Open contours and contours with fewer than three vertices are ignored. A non-finite or
    /// negative `clearance` counts as 0.
    KeepOutField(const std::vector<geometry::Contour>& zones, double clearance,
                 const geometry::Tolerance& tol = geometry::Tolerance::defaults());

    /// True when there is no region (nothing can be blocked).
    bool empty() const { return edges_.empty(); }
    /// True when `p` is strictly inside the grown region (the boundary is outside).
    bool contains(geometry::Vec2 p) const;
    /// True when the segment a-b passes through the interior of the grown region.
    bool blocks(geometry::Vec2 a, geometry::Vec2 b) const;
    /// Routes one rapid from `a` to `b` around the region.
    RouteResult route(geometry::Vec2 a, geometry::Vec2 b);

   private:
    struct Edge {
        geometry::Vec2 a, b;
        geometry::Box2 box;
    };
    bool segmentClear(geometry::Vec2 a, geometry::Vec2 b) const;
    bool vertexVisible(std::size_t i, std::size_t j);

    std::vector<Edge> edges_;
    std::vector<geometry::Vec2> vertices_;
    geometry::Box2 bounds_;
    std::unordered_map<std::uint64_t, bool> visibleMemo_;
};

/// What `routeRapids` did to a toolpath.
struct RoutingReport {
    /// Rapids replaced by a detour.
    std::size_t routed{0};
    /// Rapids left straight although they enter a zone (an end point is inside, or no path exists).
    std::size_t unroutable{0};
};

/// Replaces every rapid of `tp` that would pass through a keep-out zone with a detour of rapids
/// around it (flagged `RapidRoute::DetourFirst`/`DetourNext`; `Move::route`). Intermediate legs
/// run at the higher of the two end heights. `from` is the position before the first move (the
/// previous toolpath's end, or the machine's home); when empty the first move is not routed.
/// Feed, plunge, retract, arc and dwell moves are not changed. Rapids that cannot be routed are
/// left as they are and counted. A toolpath is unchanged when `field` is empty.
RoutingReport routeRapids(Toolpath& tp, KeepOutField& field, std::optional<geometry::Vec3> from);

/// True for a rapid move in `tp` whose segment (starting at the previous move's end, or `from`)
/// passes through the grown region of `field`. Used by tests and diagnostics.
std::size_t countBlockedRapids(const Toolpath& tp, const KeepOutField& field, std::optional<geometry::Vec3> from);

}  // namespace carvetto::toolpath
