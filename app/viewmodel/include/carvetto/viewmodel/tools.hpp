#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/node_edit.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"
#include "carvetto/viewmodel/snapping.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

// Qt-free drawing tools (T-0204, ADR-0014, ADR-0027). The canvas forwards pointer events in screen
// pixels, modifiers and a few keys to a `ToolController`; the active tool turns them into a
// preview (contours in millimetres, for the overlay) and, on completion, exactly one undoable
// `AddObjectCommand` that carries the creation parameters of the shape, so the object stays
// re-editable. Snapping (T-0203) is applied to every pointer position before the tool sees it.
//
// Interaction model
//   * Two-point shape tools (Rectangle, Circle, Ellipse, Polygon, Star): press sets the first
//     point; releasing after a drag of at least `kDragThresholdPx` completes the shape at the
//     release point; releasing without a drag leaves the tool waiting and the next press completes
//     it (click-click).
//   * Arc and Polyline: every press places a point. Arc completes with its last point; Polyline
//     finishes with Enter or a double-click (open), closes with `ToolKey::Close` (Tab) when it has
//     at least three points, and removes its last point with Backspace.
//   * Select: click selects (Shift adds, Ctrl toggles), a drag draws a marquee (left to right:
//     Window, right to left: Crossing) that combines with the selection by the same modifiers.
//   * Escape cancels whatever is in progress without touching the document.
//   * NodeEdit: pointer events go to the `NodeEditor` (node_edit.hpp); every drag step is snapped
//     with the dragged objects excluded, on the snap index of the drag's start.
//
// Modifiers
//   * Shift disables snapping for that event; in the Polyline tool it also constrains the segment
//     to the nearest multiple of 45 degrees about the previous point.
//   * Alt draws Rectangle and Ellipse from the centre, and a Circle from its diameter; Ctrl makes
//     a Rectangle square and an Ellipse a circle.
//
// Numeric entry: `fields()` lists the values the active tool can take; `setField` validates one
// entry (display units for lengths, degrees for angles) and rejects bad input without any side
// effect; `commitNumeric` (or `ToolKey::Enter`) completes the shape with the typed values
// replacing the pointer-derived ones.

namespace carvetto::viewmodel {

enum class ToolKind { Select, Rectangle, Circle, Ellipse, Polygon, Star, Arc, Polyline, NodeEdit };

/// How the Arc tool defines the arc.
enum class ArcMode {
    ThreePoint,      ///< start, a point on the arc, end
    CentreStartEnd,  ///< centre, start, end (the end point only gives the end ray)
    StartEndRadius,  ///< start, end, then the side (pointer) and the radius (typed, or the circle
                     ///< through the pointer)
    StartEndHeight   ///< start, end, then the height of the apex over the chord (pointer or typed)
};

struct Modifiers {
    bool shift{false};
    bool ctrl{false};
    bool alt{false};
};

enum class ToolKey { Escape, Enter, Backspace, Close };

/// Persistent tool settings (changing them while a shape is in progress re-computes the preview).
struct ToolOptions {
    document::RectangleParams::CornerStyle cornerStyle{document::RectangleParams::CornerStyle::Square};
    /// Corner radius (Rounded) or chamfer leg (Chamfered), millimetres; clamped to half the shorter
    /// side while dragging.
    double cornerSizeMm{0.0};
    int polygonSides{6};
    int starPoints{5};
    /// Inner radius of a star as a percentage of the outer radius, in (0, 100).
    double starInnerPercent{50.0};
    ArcMode arcMode{ArcMode::ThreePoint};
    /// Travel direction for ArcMode::CentreStartEnd.
    geometry::ArcDirection arcDirection{geometry::ArcDirection::CCW};
    /// ArcMode::StartEndRadius: take the arc of more than half a circle.
    bool arcLarge{false};
};

enum class FieldKind { Length, Angle, Count, Percent };

/// One numeric input of the active tool.
struct NumericField {
    std::string id;
    std::string label;
    FieldKind kind{FieldKind::Length};
    bool allowNegative{false};
    bool allowZero{false};
    /// Current effective value, formatted in display units (lengths) / degrees / count / percent;
    /// empty when the tool has no draft yet.
    std::string text;
    /// True when the value was typed (an override), false when it follows the pointer.
    bool typed{false};
};

struct FieldResult {
    bool ok{true};
    std::string message;
};

/// Parsed numeric entry.
struct ParsedNumber {
    bool ok{false};
    double value{0.0};  ///< millimetres for Length, degrees for Angle, count, or percent
    std::string error;
};

/// Parses `text` for a field of `kind`. Lengths are in `units` unless the text carries an explicit
/// `mm` / `in` suffix; angles are in degrees (optional `deg` or degree-sign suffix); percents take
/// an optional `%`. Counts must be whole numbers. Whitespace around the number is ignored; empty
/// text, NaN/infinity, trailing garbage, and magnitudes above 1e9 mm are rejected. Range rules
/// (sign, zero) are applied by `ToolController::setField`.
ParsedNumber parseNumber(std::string_view text, FieldKind kind, document::Units units);

/// What a controller call changed, for the caller to refresh.
struct ToolResult {
    bool committed{false};          ///< an object was added to the document (one undo step)
    bool selectionChanged{false};
    bool previewChanged{true};      ///< overlay must be redrawn
    std::string message;            ///< human-readable reason when the input was refused
};

/// Overlay data. Everything in millimetres except the marquee.
struct ToolPreview {
    std::vector<geometry::Contour> contours;
    /// Points already placed (anchors), for markers.
    std::vector<geometry::Vec2> anchorsMm;
    /// False when the draft cannot be built yet (zero size, collinear points, ...).
    bool valid{false};
    /// Why `valid` is false when the cause is a refused parameter set (empty for "not enough input").
    std::string message;
    /// Marquee rectangle in screen pixels while the Select tool drags one.
    std::optional<geometry::Box2> marqueePx;
};

class ToolController {
   public:
    /// Pointer travel (px) between press and release that counts as a drag.
    static constexpr double kDragThresholdPx = 4.0;

    explicit ToolController(document::Document* doc = nullptr);

    /// Document the tools edit (not owned). Cancels anything in progress and drops the snap index.
    void setDocument(document::Document* doc);
    /// The canvas view used to convert pixels (copied; update it on every zoom / pan / resize).
    void setView(const ViewTransform& view);

    /// Switches tool; cancels any shape in progress.
    void setTool(ToolKind tool);
    ToolKind tool() const { return tool_; }
    /// Display units of the document (millimetres without one); numeric entry is in these.
    document::Units displayUnits() const {
        return doc_ != nullptr ? doc_->job().displayUnits : document::Units::Millimetres;
    }
    ToolOptions& options() { return options_; }
    const ToolOptions& options() const { return options_; }
    /// Snap settings; the controller starts with the grid kind off.
    SnapSettings& snapSettings() { return snapSettings_; }
    const SnapSettings& snapSettings() const { return snapSettings_; }
    /// Call after the document changed without a command (direct layer-flag or object edits).
    /// Edits made through the undo stack are detected via its generation counter.
    void invalidateSnapIndex() { snapIndex_.reset(); }
    /// How many snap indexes were built so far (test hook: pins the reuse during a node-edit drag).
    std::size_t snapIndexBuilds() const { return snapIndexBuilds_; }

    // --- Pointer and key events (screen pixels) ---------------------------------------------
    ToolResult pointerMove(geometry::Vec2 px, Modifiers mods = {});
    ToolResult pointerPress(geometry::Vec2 px, Modifiers mods = {});
    ToolResult pointerRelease(geometry::Vec2 px, Modifiers mods = {});
    /// Second click of a double-click. Polyline: adds the point and finishes. Other tools: a press.
    ToolResult doubleClick(geometry::Vec2 px, Modifiers mods = {});
    ToolResult key(ToolKey k);

    // --- State for the overlay --------------------------------------------------------------
    /// True when the tool would act on Escape / Enter / Backspace / Close now (a shape is in
    /// progress or numeric values are typed); the canvas only consumes those keys then.
    bool wantsKeys() const;
    /// True while a shape, point chain or marquee is in progress.
    bool active() const;
    /// Snap found for the last pointer event (nullopt when none, or snapping was off).
    const std::optional<Snap>& snap() const { return snap_; }
    /// Position the tool received for the last pointer event: the snap point or the raw point.
    geometry::Vec2 pointMm() const { return cursorMm_; }
    ToolPreview preview() const;

    // --- Node editing (T-0206, ADR-0030) ----------------------------------------------------
    /// The node editor behind `ToolKind::NodeEdit`: handles for painting, operations, keys.
    NodeEditor& nodeEditor() { return nodeEditor_; }
    const NodeEditor& nodeEditor() const { return nodeEditor_; }

    // --- Numeric entry ----------------------------------------------------------------------
    /// Fields of the active tool (and arc mode), in display order. Empty for tools with none.
    std::vector<NumericField> fields() const;
    /// Validates and stores a typed value; an empty `text` clears it. A bad value changes nothing.
    FieldResult setField(const std::string& id, const std::string& text);
    /// Completes the shape using the typed values (Polyline: adds a point, or finishes when
    /// nothing was typed). Refused (no side effect) when the draft is invalid.
    ToolResult commitNumeric();

   private:
    struct Draft {
        std::optional<document::CreationParams> params;
        std::vector<geometry::Contour> contours;  ///< shape (committed) or rubber band (preview)
        bool valid{false};
        std::string message;
        std::map<std::string, double> values;  ///< effective field values (mm / deg / count / percent)
    };

    static void finishParams(Draft& d, const document::CreationParams& params);
    Draft makeDraft() const;
    Draft rectangleDraft(geometry::Vec2 a) const;
    Draft circleDraft(geometry::Vec2 a) const;
    Draft ellipseDraft(geometry::Vec2 a) const;
    Draft polygonDraft(geometry::Vec2 a) const;
    Draft starDraft(geometry::Vec2 a) const;
    Draft arcDraft() const;
    Draft polylineDraft(bool includeCursor, bool closeIt) const;

    geometry::Vec2 anchorFor() const;
    std::optional<double> typed(const std::string& id) const;
    ToolResult commitDraft(const Draft& draft);
    ToolResult refuse(std::string message) const;
    void cancel();
    void updatePointer(geometry::Vec2 px, Modifiers mods);
    const SnapIndex& snapIndex();
    ToolResult afterNodeEdit(const NodeEditResult& r);
    ToolResult selectPress(geometry::Vec2 px, Modifiers mods);
    ToolResult selectRelease(geometry::Vec2 px, Modifiers mods);
    ToolResult shapePress(Modifiers mods);
    ToolResult shapeRelease(geometry::Vec2 px);
    ToolResult finishPolyline(bool closeIt);
    bool isTwoPointTool() const;
    std::size_t pointsNeeded() const;
    bool layerWritable(std::string* why) const;

    document::Document* doc_{nullptr};
    NodeEditor nodeEditor_;
    ViewTransform view_;
    ToolKind tool_{ToolKind::Select};
    ToolOptions options_;
    SnapSettings snapSettings_;
    std::optional<SnapIndex> snapIndex_;
    std::array<std::uint64_t, 2> snapSignature_{};
    // NodeEdit drag: the index was current when the drag began and may be kept (ADR-0030).
    bool dragIndexValid_{false};
    std::size_t snapIndexBuilds_{0};
    std::array<std::uint64_t, 2> preSig_{};

    std::optional<Snap> snap_;
    geometry::Vec2 cursorMm_{};
    geometry::Vec2 cursorPx_{};
    Modifiers mods_{};
    bool hasCursor_{false};

    std::vector<geometry::Vec2> pts_;
    bool pressed_{false};
    geometry::Vec2 pressPx_{};
    std::map<std::string, double> typed_;

    // Select tool
    bool selPressed_{false};
    SelectMode selMode_{SelectMode::Replace};
    geometry::Vec2 selStartPx_{};
    geometry::Vec2 selCurPx_{};
    bool selDragging_{false};
};

/// Display name of a tool ("Rectangle", ...).
const char* toolName(ToolKind tool);

}  // namespace carvetto::viewmodel
