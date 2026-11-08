#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "carvetto/document/ids.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/viewmodel/toolpath_session.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

// 2D toolpath preview of the app (T-0223, ADR-0032 amendment, ADR-0014). Qt-free: the scene
// (move polylines per toolpath, classified rapid / feed / plunge / retract / ramp), the visibility
// state, and the per-frame draw list (culled and decimated for the current view) that the canvas
// paints. The scene holds only shared immutable move lists and its own polylines: nothing refers
// to the Document across calls, so a worker or a later frame never sees freed state.
//
// Cost: building a toolpath's polylines is O(moves) and happens only when its move list changes
// (identity of the shared moves); a frame is O(chunks + emitted points), bounded by the screen
// (chunks outside the viewport cost one box test), not by the number of moves.

namespace carvetto::viewmodel {

/// What a move looks like on the canvas. Arcs are Feed (a helical arc, whose Z changes, is Ramp).
enum class MoveClass { Rapid, Feed, Plunge, Retract, Ramp };
inline constexpr std::size_t kMoveClassCount = 5;

/// Colour key of a move class (the same keys are shown in the legend).
struct PreviewColour {
    std::uint8_t r{0};
    std::uint8_t g{0};
    std::uint8_t b{0};
    bool operator==(const PreviewColour&) const = default;
};
PreviewColour colourOf(MoveClass cls);
const char* nameOf(MoveClass cls);

/// Class of a move that starts at `from`. A Plunge/Retract kind is Plunge/Retract; a Feed or arc
/// whose Z changes by more than 1e-9 mm while it also travels in XY is a Ramp (ramp entries and
/// helices); Dwell has no class (nullopt: nothing to draw).
std::optional<MoveClass> classify(const toolpath::Move& move, const toolpath::Vec3& from);

/// Most points of one polyline chunk (the unit of culling); an arc is never split, so a chunk holding
/// one can have up to kPreviewMaxArcPoints + 1 points. Moves without XY travel (vertical feeds and
/// rapids) are counted in `movesByClass` but not drawn; only vertical Plunge/Retract make dots.
inline constexpr std::size_t kPreviewChunkPoints = 256;
/// Chord error in mm used to turn arcs into polylines, and the most points of one arc.
inline constexpr double kPreviewArcTolerance = 0.01;
inline constexpr std::size_t kPreviewMaxArcPoints = 4096;

/// A run of consecutive moves of one class as a polyline in document coordinates (mm).
struct PreviewChunk {
    MoveClass cls{MoveClass::Feed};
    geometry::Box2 bounds;
    /// First and last move (index into the toolpath's moves) the chunk covers.
    std::uint32_t firstMove{0};
    std::uint32_t lastMove{0};
    std::vector<geometry::Vec2> points;
};

/// The polylines of one move list (immutable, shared between scene copies).
struct PreviewGeometry {
    std::vector<PreviewChunk> chunks;
    geometry::Box2 bounds;
    std::size_t points{0};
    std::array<std::size_t, kMoveClassCount> movesByClass{};
};

/// Polylines of `moves`. The tool starts at the end of the first move (zero length), as everywhere.
std::shared_ptr<const PreviewGeometry> buildPreviewGeometry(const std::vector<toolpath::Move>& moves);

struct PreviewToolpath {
    document::ToolpathId id;
    std::string name;
    /// Needs recalculation: drawn dashed and grey, never mistaken for a current toolpath.
    bool stale{false};
    /// Enabled in the job; disabled toolpaths are not drawn.
    bool enabled{true};
    /// Moves computed (otherwise there is nothing to draw).
    bool computed{false};
    std::size_t moveCount{0};
    /// Index of its first move in the job-wide move list (enabled, computed toolpaths in order:
    /// the order the simulation cuts them); only meaningful when `enabled && computed`.
    std::size_t firstMove{0};
    /// The shared moves the geometry was built from (keeps the identity valid).
    document::SharedMoves moves;
    std::shared_ptr<const PreviewGeometry> geometry;
};

struct PreviewScene {
    std::vector<PreviewToolpath> toolpaths;
    /// Moves of the enabled, computed toolpaths.
    std::size_t totalMoves{0};
    geometry::Box2 bounds;
};

/// How a batch of runs is painted.
struct PreviewStyle {
    MoveClass cls{MoveClass::Feed};
    /// Part of the playback already cut (moves before the playback position) vs the rest.
    bool played{true};
    bool highlighted{false};
    bool stale{false};
    bool operator==(const PreviewStyle&) const = default;
    bool operator<(const PreviewStyle& o) const;
};

/// Screen-space polylines of one style. `runLengths` splits `points` into polylines (a run of 1
/// point is a dot).
struct PreviewBatch {
    PreviewStyle style;
    std::vector<geometry::Vec2> points;
    std::vector<std::uint32_t> runLengths;
};

struct PreviewDrawStats {
    std::size_t chunksTotal{0};
    std::size_t chunksDrawn{0};
    std::size_t pointsIn{0};
    std::size_t pointsOut{0};
};

struct PreviewDrawList {
    std::vector<PreviewBatch> batches;
    PreviewDrawStats stats;
};

struct PreviewDrawOptions {
    /// Only chunks whose last move has a job-wide index in [fromMove, toMove) are listed, and they
    /// get the style `played` (the canvas draws the part of the playback already cut in full colour
    /// and the rest faded; it draws a delta range on top of a cached layer while playing).
    std::size_t fromMove{0};
    std::size_t toMove{static_cast<std::size_t>(-1)};
    bool played{true};
    /// Class visibility (legend check boxes).
    std::array<bool, kMoveClassCount> classVisible{true, true, true, true, true};
    /// Points closer than this to the last kept point (pixels, Manhattan) are dropped.
    double minPointSpacingPx{1.5};
};

/// The preview state: on/off, per-toolpath visibility, selection and the scene.
class PreviewModel {
   public:
    /// Rebuilds what changed from `session`'s document (new or recalculated moves; staleness, name
    /// and enabled flag are refreshed every call). Cheap when nothing changed.
    void sync(ToolpathSession& session);
    /// Forgets everything (the document was replaced).
    void clear();

    bool enabled() const { return enabled_; }
    void setEnabled(bool on);

    /// Toolpaths are visible unless hidden here.
    bool isVisible(document::ToolpathId id) const { return hidden_.count(id) == 0; }
    void setVisible(document::ToolpathId id, bool visible);
    /// Draw only the selected toolpath (nothing when none is selected).
    bool selectedOnly() const { return selectedOnly_; }
    void setSelectedOnly(bool on);
    document::ToolpathId selected() const { return selected_; }
    void setSelected(document::ToolpathId id);

    const PreviewScene& scene() const { return scene_; }
    /// Increases whenever something that changes the picture changes (scene, visibility, selection).
    std::uint64_t revision() const { return revision_; }

    /// True when `tp` is drawn now (preview on, enabled, computed, visible, selected if selectedOnly).
    bool drawn(const PreviewToolpath& tp) const;
    /// The draw list for `view`: chunks outside the viewport are culled, the rest decimated.
    PreviewDrawList draw(const ViewTransform& view, const PreviewDrawOptions& options = {}) const;

    /// Position of the tool tip after job-wide move `globalMove` (0 = before the first move, which
    /// is where the first move ends); nullopt when there are no moves. XY only.
    std::optional<geometry::Vec2> positionAfter(std::size_t globalMove) const;
    /// Number of stale toolpaths among the drawn ones.
    std::size_t staleDrawn() const;

   private:
    bool enabled_{false};
    bool selectedOnly_{false};
    document::ToolpathId selected_;
    std::set<document::ToolpathId> hidden_;
    PreviewScene scene_;
    std::uint64_t revision_{1};
};

}  // namespace carvetto::viewmodel
