#pragma once

#include <array>
#include <cstddef>
#include <optional>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/selection.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/box_index.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

// Qt-free picking and selection (T-0201, ADR-0014). Hit tests work in screen pixels through the
// `ViewTransform`; the document stores objects already transformed (no per-object matrix), so
// object transforms are baked into the contours and groups only add structure.
//
// Pickable objects: those on the active sheet whose layer, and every ancestor group's layer, is
// visible and unlocked. Groups resolve to their outermost group: a hit on a child returns the
// top-level object that contains it.

namespace carvetto::viewmodel {

/// How a hit test treats the inside of closed shapes.
enum class FillHit {
    StrokeOnly,    ///< Only the outline (within the tolerance) hits.
    StrokeAndFill  ///< The interior of closed contours hits too (even-odd over the object's contours).
};

struct HitOptions {
    /// Pick radius in screen pixels.
    double tolerancePx{4.0};
    FillHit fill{FillHit::StrokeOnly};
};

/// Result of a point pick.
struct Hit {
    /// Object to select: the outermost group, or the object itself when ungrouped.
    document::ObjectId object{};
    /// The vector or bitmap object that was actually under the cursor.
    document::ObjectId leaf{};
    /// Distance from the cursor to the leaf's outline in pixels (0 for a fill or bitmap hit).
    double distancePx{0.0};
    /// True when the object was hit through its interior and not its outline.
    bool viaFill{false};
};

/// Marquee semantics.
enum class MarqueeMode {
    Window,   ///< Objects entirely inside the rectangle (boundary contact allowed).
    Crossing  ///< Objects whose outline touches or lies inside the rectangle.
};

/// How a set of picked ids combines with the current selection.
enum class SelectMode { Replace, Add, Toggle, Subtract };

/// Spatial index over the pickable objects of a document's active sheet. Build once per document
/// state and reuse for every mouse move; any edit, layer flag change or sheet switch needs a new
/// index. The document must outlive the index and stay unchanged while it is used.
///
/// Point queries touch only entries whose bounds are near the cursor (uniform grid), so a pick
/// among 10 000 objects costs the neighbourhood of the cursor, not the scene.
///
/// Ordering (documented, deterministic):
///   1. Outline hits beat interior hits.
///   2. Among outline hits the nearest wins; distances equal within 1e-9 px tie.
///   3. Ties (and all interior hits) go to the topmost layer (later in the layer order), then to
///      the smaller bounding-box area (a small shape inside a big one stays pickable), then to
///      the later-created object (higher id).
class PickIndex {
   public:
    explicit PickIndex(const document::Document& doc);

    /// Number of pickable leaf objects (vectors and bitmaps) indexed.
    std::size_t size() const { return entries_.size(); }

    /// The pick under screen point `screenPx`, or nullopt.
    std::optional<Hit> hitTest(const ViewTransform& view, geometry::Vec2 screenPx,
                               const HitOptions& options = {}) const;

    /// Top-level objects selected by a marquee given as a screen rectangle (any corner order),
    /// in ascending id order. A group is included when all its pickable leaves are inside
    /// (Window) or at least one touches (Crossing). With `FillHit::StrokeAndFill`, Crossing also
    /// includes closed shapes that enclose the whole rectangle.
    std::vector<document::ObjectId> marquee(const ViewTransform& view, const geometry::Box2& screenRect,
                                            MarqueeMode mode, FillHit fill = FillHit::StrokeOnly) const;

    /// All pickable top-level objects, ascending id order.
    std::vector<document::ObjectId> topLevelObjects() const;

   private:
    struct Entry {
        document::ObjectId leaf{};
        document::ObjectId top{};
        std::size_t layerIndex{0};
        geometry::Box2 bounds;
    };
    const document::Document* doc_;
    std::vector<Entry> entries_;
    toolpath::BoxIndex index_;
};

/// Convenience: builds a `PickIndex` and runs one point query. Prefer a retained `PickIndex`
/// when picking repeatedly.
std::optional<Hit> hitTest(const document::Document& doc, const ViewTransform& view, geometry::Vec2 screenPx,
                           const HitOptions& options = {});

/// Convenience: one marquee query on a fresh index.
std::vector<document::ObjectId> marqueeSelect(const document::Document& doc, const ViewTransform& view,
                                              const geometry::Box2& screenRect, MarqueeMode mode,
                                              FillHit fill = FillHit::StrokeOnly);

/// Combines `ids` with `selection` (the core `Selection`): Replace assigns (empty `ids` clears),
/// Add appends the missing ones, Toggle flips each, Subtract removes them.
void applySelection(document::Selection& selection, const std::vector<document::ObjectId>& ids, SelectMode mode);

/// Click-to-select: hit test at `screenPx` and apply the result. A miss clears the selection in
/// Replace mode and leaves it alone otherwise. Returns the hit.
std::optional<Hit> clickSelect(document::Document& doc, const ViewTransform& view, geometry::Vec2 screenPx,
                               SelectMode mode, const HitOptions& options = {});

/// Marquee-select and apply with `mode`. Returns the ids the marquee picked.
std::vector<document::ObjectId> marqueeApply(document::Document& doc, const ViewTransform& view,
                                             const geometry::Box2& screenRect, MarqueeMode marqueeMode,
                                             SelectMode mode, FillHit fill = FillHit::StrokeOnly);

/// Selects every pickable top-level object on the active sheet.
void selectAll(document::Document& doc);
/// Selects the pickable top-level objects that are not selected, deselects the others.
void invertSelection(document::Document& doc);
/// Selects the pickable top-level objects holding at least one pickable leaf on `layer`
/// (replacing the selection). A hidden or locked layer yields an empty selection.
void selectByLayer(document::Document& doc, document::LayerId layer);

/// Handle positions of a selection box, named by screen orientation (Top is up on screen).
enum class HandleKind {
    TopLeft,
    Top,
    TopRight,
    Left,
    Centre,
    Right,
    BottomLeft,
    Bottom,
    BottomRight
};

struct Handle {
    HandleKind kind{HandleKind::Centre};
    geometry::Vec2 positionPx{};
    geometry::Vec2 positionMm{};
};

/// Bounding box of the selection plus its nine handles (corners, edge midpoints, centre).
struct SelectionBox {
    geometry::Box2 boundsMm;
    std::array<Handle, 9> handles{};
};

/// Union of the bounds of the selected objects (groups via their children); nullopt when nothing
/// with geometry is selected. Unknown ids are skipped.
std::optional<geometry::Box2> selectionBounds(const document::Document& doc);

/// Handles of `boundsMm` through `view`, in the fixed order of `HandleKind`.
SelectionBox selectionBox(const geometry::Box2& boundsMm, const ViewTransform& view);

/// The handle within `tolerancePx` of `screenPx`: the nearest, and on equal distance (coincident
/// handles of a degenerate box) the first in `HandleKind` order; nullopt when none is in range.
std::optional<HandleKind> hitHandle(const SelectionBox& box, geometry::Vec2 screenPx, double tolerancePx);

}  // namespace carvetto::viewmodel
