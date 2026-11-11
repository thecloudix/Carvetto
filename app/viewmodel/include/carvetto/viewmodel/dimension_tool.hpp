#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/dimension.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/vec2.hpp"

// Qt-free dimension tool (T-0246, ADR-0014). Clicks pick the defining points (snapped by the caller;
// a click within `snapToleranceMm` of a vertex or arc centre of a part anchors the point to it so
// the dimension follows the part), then one more click places the annotation and commits. The
// whole dimension is ONE undo step and goes on the layer "Dimensions" (created when missing)
// unless the options name a layer.
//
//   Linear / Aligned   click point 0, click point 1, click the dimension line position.
//   Angular            click the vertex, an arm point, the other arm point, then the arc position.
//   Radial             click the centre (a circle's centre anchors to it), a point on the circle,
//                      then the leader end.
//
// Editing an existing dimension (Shift-click selects it in the canvas): `edit` replaces its
// options (decimals, text height, offset, custom text) as one undo step; the points keep their
// anchors.
//
// T-0255: the tool is `ToolKind::Dimension` of the `ToolController`. Shift-click selects an existing
// dimension (the edit target); its control points are then dragged (`beginHandle`/`dragTo`/`endDrag`,
// one undo step per drag, see `core/document/dimension.hpp`), and `removeSelected` deletes the
// selected dimensions (one undo step). Reflex angular dimensions: placing the arc outside the
// smaller angle measures the reflex angle (negative offset, see dimension.hpp).

namespace carvetto::viewmodel {

/// Persistent options of the dimension tool and the editable options of an existing dimension.
struct DimensionOptions {
    document::DimensionKind kind{document::DimensionKind::Aligned};
    bool vertical{false};
    bool diameter{false};
    int decimals{2};
    int angleDecimals{2};
    double textHeightMm{3.0};
    bool useCustomText{false};
    std::string customText;
    std::string fontKey;
    /// Edit only (`DimensionTool::edit`): the offset of the dimension line / arc radius / leader
    /// length in mm, and whether an angular dimension measures the reflex angle. Nullopt keeps the
    /// current value. `optionsOf` fills both. New dimensions take their offset from the placing click.
    std::optional<double> offsetMm;
    std::optional<bool> reflex;
    /// Layer new dimensions go on; null selects (or creates) the layer "Dimensions".
    document::LayerId layer{};
};

struct DimensionToolResult {
    bool ok{true};
    std::string message;          ///< The reason when something was refused.
    bool committed{false};        ///< The document changed (one undo step).
    std::optional<document::ObjectId> created;
    bool draftChanged{false};     ///< The overlay must be redrawn.
};

class DimensionTool {
   public:
    explicit DimensionTool(document::Document* doc = nullptr) : doc_(doc) {}

    /// Not owned. Drops the draft.
    void setDocument(document::Document* doc);
    document::Document* document() const { return doc_; }
    const DimensionOptions& options() const { return options_; }
    /// Validates and stores the options; refused (nothing changed) when they have a problem. The
    /// draft restarts when the kind changes.
    DimensionToolResult setOptions(const DimensionOptions& options);

    /// Number of points picked so far.
    std::size_t pickedCount() const { return picked_; }
    /// A click at `mm`. Picks the next point or, after the last point, places and commits.
    DimensionToolResult click(geometry::Vec2 mm, double snapToleranceMm);
    /// Pointer movement (for the preview of the placement step).
    void move(geometry::Vec2 mm) { cursor_ = mm; }
    /// Drops the draft without touching the document.
    void cancel() { picked_ = 0; }

    /// The dimension being drawn, as it would be committed at the cursor, for the overlay: the
    /// geometry of a draft that has all its points (placement step), else just the picked points
    /// joined by a line (empty contours when nothing is picked).
    document::DimensionGeometry preview() const;

    /// Replaces the options of dimension `id` (kind and points are kept). One undo step; refused
    /// when `id` is not a dimension or the options have a problem.
    DimensionToolResult edit(document::ObjectId id, const DimensionOptions& options);
    /// The options of an existing dimension (to fill the edit form); nullopt when not a dimension.
    std::optional<DimensionOptions> optionsOf(document::ObjectId id) const;

    /// Drops the last picked point (Backspace); no-op when none.
    void backspace() {
        if (picked_ > 0) {
            --picked_;
        }
    }

    // --- Existing dimensions (T-0255) -----------------------------------------------------------
    /// The dimension whose handles are shown and whose options the form edits; nullopt when none.
    std::optional<document::ObjectId> editTarget() const;
    void setEditTarget(std::optional<document::ObjectId> id) {
        editTarget_ = id;
        drag_.reset();
    }
    /// The dimension nearest `mm` within `toleranceMm` on the active sheet (its lines, arrowheads
    /// or text); ties go to the lower id. Layers that are hidden are skipped.
    std::optional<document::ObjectId> pickAt(geometry::Vec2 mm, double toleranceMm) const;
    /// Shift-click: picks, selects it in the document and makes it the edit target; a miss clears
    /// the edit target (the document selection is untouched).
    DimensionToolResult selectAt(geometry::Vec2 mm, double toleranceMm);
    /// Handles of the edit target (empty when none or the object is no longer a dimension).
    std::vector<document::DimensionHandle> handles() const;
    /// Starts dragging the handle of the edit target nearest `mm` within `toleranceMm`; false when
    /// there is none.
    bool beginHandle(geometry::Vec2 mm, double toleranceMm);
    bool dragging() const { return drag_.has_value(); }
    /// Moves the dragged handle (a point snaps to part vertices within the tolerance of
    /// `beginHandle`, so it re-anchors); positions that would make the dimension invalid or
    /// degenerate are ignored (the last valid state stays).
    DimensionToolResult dragTo(geometry::Vec2 mm);
    /// Final position; commits ONE undo step when anything changed.
    DimensionToolResult endDrag(geometry::Vec2 mm);
    /// Drops the drag without touching the document.
    void cancelDrag() { drag_.reset(); }
    /// Deletes the selected dimensions as one undo step; refused when none is selected.
    DimensionToolResult removeSelected();

    /// What the overlay draws for the draft (placement step) or the dragged dimension: the object
    /// as it would be committed. Nullopt when there is nothing to show.
    std::optional<document::DimensionObject> previewObject() const;

   private:
    document::DimensionObject draft() const;

    struct Drag {
        document::ObjectId id;
        document::DimensionHandle handle;
        document::DimensionObject original;
        document::DimensionObject current;
        double toleranceMm{0.0};
    };

    document::Document* doc_;
    DimensionOptions options_;
    std::array<document::DimensionPoint, 3> points_{};
    std::size_t picked_{0};
    geometry::Vec2 cursor_{};
    std::optional<document::ObjectId> editTarget_;
    std::optional<Drag> drag_;
};

}  // namespace carvetto::viewmodel
