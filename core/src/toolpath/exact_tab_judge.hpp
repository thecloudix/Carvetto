#pragma once

// The exact judge of the tab layer's "no waste near" answer (T-0279, ADR-0096), apart from
// ProfileBuilder so it can be driven by hand-built fixtures.

#include <cstddef>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::toolpath::detail {

/// A tool position that cuts a disc smaller than the tab-layer radius (a V-bit's corner moves).
struct ToolDisc {
    geometry::Vec2 p{};
    double rho{0.0};
};

/// The tool positions near a tab as the judge sees them: runs of consecutive samples (the capsules'
/// polylines) and corner-move discs, split by what lowering the tab changes. `cut*` is cut with the
/// tab raised (open tools, the other live tabs' raised stretches), `own*` is what lowering the tab adds.
struct ExactTools {
    std::vector<std::vector<geometry::Vec2>> cutRuns;
    std::vector<std::vector<geometry::Vec2>> ownRuns;
    std::vector<ToolDisc> cutX;
    std::vector<ToolDisc> ownX;
};

/// How a tool position near a tab enters the judge's world.
enum class ToolSide { Skip, Cut, Own };

/// Where a sample covered by the raised stretches of the groups `coveredBy` goes when the judge is
/// asked about group `g` (`alive[o]`: group o is still live). A group of another live tab covering a
/// plain sample leaves it cut (that tab's raised stretch counts as cut); a corner move (`isDisc`) such
/// a tab covers is not made at all. Covered by `g` alone: it is what lowering `g` adds.
ToolSide classifyTool(bool isDisc, const std::vector<std::size_t>& coveredBy, std::size_t g,
                      const std::vector<bool>& alive);

/// True when the tab whose raster window is `window` holds: lowering it splits a piece the tools'
/// capsules leave in the window, or the window has no other piece. `rk` is the tab-layer radius.
bool exactNoWasteHolds(const geometry::Box2& window, const ExactTools& ex, double rk,
                       const geometry::Tolerance& tol, double mitreLimit);

}  // namespace carvetto::toolpath::detail
