#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/geometry/vec3.hpp"
#include "carvetto/toolpath/keepout.hpp"

// Keep-out zones in the app (T-0256, ADR-0036 + ADR-0046): Qt-free glue over the T-0241 commands
// (Create from Selection, Clear Zones, clearance, show/hide) and the routed-rapids cache shared by
// the toolpath preview and the job summary. The Qt menu and canvas only call these.

namespace carvetto::viewmodel {

struct KeepOutResult {
    bool ok{false};
    /// Status-bar text: what changed, or why nothing did.
    std::string message;
    /// Zones created / removed.
    std::size_t count{0};
};

/// True when the selection holds at least one closed vector of non-zero area.
bool canCreateKeepOutFromSelection(const document::Document& doc);
/// "Create from Selection": ONE undo step. The zones are copies of the selected closed vectors.
KeepOutResult createKeepOutFromSelection(document::Document& doc);
/// "Clear Zones": ONE undo step; refused when there are none.
KeepOutResult clearKeepOutZones(document::Document& doc);
/// Sets the clearance in mm (0..kMaxKeepOutClearanceMm); ONE undo step (consecutive edits merge).
KeepOutResult setKeepOutClearanceMm(document::Document& doc, double mm);
/// Show/hide the zones on the canvas (a view setting: saved, not undoable). True when it changed.
bool setKeepOutVisible(document::Document& doc, bool visible);
/// "3 keep-out zones, clearance 2 mm, shown" style line for the dock / status bar.
std::string describeKeepOut(const document::Document& doc);

/// One enabled, computed toolpath after rapid routing.
struct RoutedToolpath {
    document::ToolpathId id;
    /// The record's moves and the moves with the rapids that cross a zone replaced by detours
    /// (the same pointer when nothing was routed).
    document::SharedMoves original;
    document::SharedMoves routed;
    /// For each routed move the index of the original move it stands for (empty when unrouted).
    std::shared_ptr<const std::vector<std::uint32_t>> originalIndex;
    /// The tool is at `start` before the first move of a routed toolpath whose first rapid was routed.
    std::optional<geometry::Vec3> start;
    toolpath::RoutingReport report;
    bool changed() const { return routed != original; }
};

struct RoutedJob {
    /// Empty when the document has no zones.
    std::vector<RoutedToolpath> toolpaths;
    toolpath::RoutingReport total;
    const RoutedToolpath* find(document::ToolpathId id) const;
};

/// Routes the rapids of toolpaths around the document's zones exactly as Save Toolpaths does (the
/// tool starts at home (0, 0, safe Z of the first toolpath), each toolpath starts where the previous
/// one ended), and caches the results by moves identity, start position and zone set. One thread.
class KeepOutRouter {
   public:
    /// `ids` are the toolpaths to route, in job order (enabled and computed ones).
    const RoutedJob& route(const document::Document& doc, const std::vector<document::ToolpathId>& ids);
    void clear();

   private:
    struct Entry {
        document::SharedMoves original;
        std::optional<geometry::Vec3> from;
        std::uint64_t generation{0};
        RoutedToolpath result;
    };
    std::vector<geometry::Contour> zones_;
    double clearance_{0.0};
    std::uint64_t generation_{0};
    std::unique_ptr<toolpath::KeepOutField> field_;
    std::map<document::ToolpathId, Entry> cache_;
    RoutedJob job_;
};

/// The keep-out zone outlines of `doc` as world contours, in creation order.
std::vector<geometry::Contour> keepOutOutlines(const document::Document& doc);
/// Outline of the zones' union grown by the clearance (round corners): the halo drawn around them.
/// Empty when the clearance is 0.
std::vector<geometry::Contour> keepOutHalo(const document::Document& doc);

}  // namespace carvetto::viewmodel
