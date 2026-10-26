#pragma once

#include <cstddef>
#include <vector>

#include "carvetto/toolpath/path_walk.hpp"

// Shared toolpath machinery (T-0120, ADR-0016): automatic tab placement along a cut path and the
// height ceiling the tabs impose on a pass. Reusable by any operation that frees parts.

namespace carvetto::toolpath {

/// How automatic tabs are spread along a path (feature-inventory §5 "Toolpath Tabs").
enum class TabSpacing {
    ConstantNumber,    ///< `count` tabs per path, equally spaced by arc length.
    ConstantDistance,  ///< about one tab every `distance` mm: round(length / distance) tabs (≥ 1).
};

/// Tab options. Lengths in mm.
struct TabOptions {
    bool enabled{false};
    /// Length of material left in each tab, measured along the vector.
    double length{10.0};
    /// Height of the tab above the bottom of the cut.
    double thickness{2.0};
    TabSpacing spacing{TabSpacing::ConstantNumber};
    int count{4};
    double distance{100.0};
    /// Move tabs off arcs and corners: each tab's raised stretch must lie on one straight span.
    bool avoidCorners{false};
    /// Ramp up to and down from each tab at 45° instead of vertical moves.
    bool threeD{false};
    bool operator==(const TabOptions&) const = default;
};

/// Outcome of placing tabs on one path.
struct TabPlacement {
    /// Centres of the raised stretches, as arc length from the path start, ascending.
    std::vector<double> centres;
    /// Number of tabs that could not be moved off a corner/curve (left where they were).
    std::size_t notAvoided{0};
    /// Number of tabs dropped because they overlapped an earlier one or did not fit the path.
    std::size_t dropped{0};
};

/// Places tabs on `path`. `raisedLength` is the length of each raised stretch along the path
/// (typically tab length + tool diameter, so that `length` of material remains). Centres start
/// half a spacing after the path start, so the start point (where the tool plunges) is not in a
/// tab: centre_i = (i + 1/2) · L / n. With `avoidCorners`, each tab moves to the nearest
/// position whose raised stretch lies within one line span (wrapping on closed paths). Tabs
/// whose footprint (`tabFootprint`) does not fit are dropped as `fitTabs` describes, so every
/// pass keeps a stretch at full depth.
TabPlacement placeTabs(const ArcLengthPath& path, double raisedLength, const TabOptions& options);

/// Length of path one tab occupies: the raised stretch, plus (3D tabs) a 45° ramp of
/// `thickness` on each side, as `tabCeiling` draws it.
double tabFootprint(double raisedLength, const TabOptions& options);

/// Smallest stretch of full-depth cut kept between two tab footprints (mm).
inline constexpr double kMinTabGap = 1e-3;

/// Keeps the tab centres whose footprints fit on `path` (sorted ascending): a footprint must not
/// run past an open end, must leave at least `kMinTabGap` to the previous tab (and, on a closed
/// path, to the first tab across the start), and on a closed path no tab is kept when one
/// footprint (plus the gap) is as long as the loop. The others are counted in `dropped`.
TabPlacement fitTabs(const ArcLengthPath& path, std::vector<double> centres, double footprint);

/// Height ceiling imposed by tabs on a leg that walks `path` from `u0` to `u1`, as a profile in
/// travelled distance: `zTab` over each raised stretch, a very low sentinel elsewhere, plus 45°
/// ramps down to `zBottom` on both sides when `threeD`. Closed paths repeat tabs every
/// revolution. Combine with the pass profile with `ZProfile::max`.
ZProfile tabCeiling(const ArcLengthPath& path, double u0, double u1, const std::vector<double>& centres,
                    double raisedLength, double zTab, double zBottom, bool threeD);

/// The "no ceiling" sentinel height used by `tabCeiling`.
inline constexpr double kNoCeiling = -1.0e9;

}  // namespace carvetto::toolpath
