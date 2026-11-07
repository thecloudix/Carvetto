#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/node_edit_command.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/node_ops.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

// Qt-free node editing (T-0206, ADR-0030, ADR-0014). The NodeEdit tool of the `ToolController`
// forwards pointer events here; the canvas paints `handles()`; menus and keys call `run()`.
//
// Which objects are edited: the vector leaves of the document selection (groups edit their
// leaves) on the active sheet whose layer, and every ancestor group's layer, is visible and
// unlocked. Everything else is ignored and never modified. A parametric object (rectangle, circle,
// ...) becomes a plain vector (NoParams) at its first edit; one undo step restores it exactly.
//
// Model: nodes (span end points, `nodeops` numbering), Bezier control points and virtual midpoint
// handles (parameter 0.5 of each span), all in millimetres. Full-circle arcs are shown as two half
// arcs (two nodes). Hit testing works in screen pixels: nodes beat control points, those beat
// midpoint handles, and those beat the span itself; the nearest wins inside a class.
//
// Mouse
//   Press on a node: selects it (Shift: toggles it in the selection); dragging moves every selected
//   node. A control point drags alone. A span drag reshapes it (Ctrl: moves the whole span, its
//   end nodes included); dragging a midpoint handle inserts a node there and drags it. A press on
//   empty canvas that is dragged selects the nodes inside the rectangle (Shift adds); a click
//   clears the node selection. A click on another object selects that object.
//   Every pointer position (already snapped by the caller) of one drag is ONE undo step; Escape
//   reverts it exactly and leaves nothing to redo.
//
// Operations (`run`): each is one undo step, all-or-nothing, with `canRun` for the enabling rule and
// a message on refusal. Keys (`opForKey`): I insert point, D delete point, S smooth, C cut vector, B
// convert to Bezier, A convert to arc, L convert to line, P properties (the Qt layer), X / Y match
// the first node's coordinate, H / V mirror mode, K keep Bezier tangency, J join. (The inventory
// lists the letters without their meaning; this mapping is an interpretation, see ADR-0030.)

namespace carvetto::viewmodel {

enum class MirrorMode { Off, Horizontal, Vertical };

enum class HandleType { Node, Control, Midpoint, Span };

struct NodeRef {
    document::ObjectId object;
    std::size_t contour{0};
    std::size_t node{0};
    auto operator<=>(const NodeRef&) const = default;
};

struct SpanRef {
    document::ObjectId object;
    std::size_t contour{0};
    std::size_t span{0};
    auto operator<=>(const SpanRef&) const = default;
};

/// What a pointer position is over.
struct NodeHit {
    HandleType type{HandleType::Node};
    NodeRef node;          ///< Node
    SpanRef span;          ///< Control, Midpoint, Span
    int control{0};        ///< Control: 0 = p1 (start side), 1 = p2 (end side)
    geometry::Vec2 positionMm{};  ///< handle position; for Span the nearest point on the span
    double t{0.0};         ///< Span: parameter of that point
    double distancePx{0.0};
};

/// One handle for painting.
struct NodeHandle {
    HandleType type{HandleType::Node};
    NodeRef node;
    SpanRef span;
    int control{0};
    geometry::Vec2 positionMm{};
    bool selected{false};
    /// Control points: the node they hang on (for drawing the handle line).
    geometry::Vec2 anchorMm{};
};

enum class NodeOp {
    ConvertLine,
    ConvertBezier,
    ConvertArc,
    InsertPoint,
    InsertMidpoint,
    DeletePoint,
    DeleteSpan,
    CutVector,
    Smooth,
    Reverse,
    Close,
    Join,
    StartPoint,
    Extend,
    MatchX,
    MatchY
};

/// Optional inputs of an operation.
struct NodeOpArgs {
    /// InsertPoint / CutVector on one selected span: where (projected onto the span).
    std::optional<geometry::Vec2> positionMm;
    /// ConvertArc / ConvertBezier on one span: pass through this point.
    std::optional<geometry::Vec2> throughMm;
    /// ConvertArc on one span: bulge = tan(sweep / 4), positive counter-clockwise.
    std::optional<double> bulge;
};

struct NodeEditResult {
    bool ok{false};
    /// Empty on success; otherwise the reason, for a status line or message box.
    std::string message;
    /// The document changed (one undo step was recorded, or a gesture was reverted).
    bool changed{false};
    /// The document's object selection changed (a click picked another object).
    bool selectionChanged{false};
    /// `handleKey` only: the key means something to the node editor.
    bool handled{false};
    /// `handleKey` only: the Properties dialog was requested (the Qt layer shows it).
    bool openProperties{false};
};

struct NodeEditOptions {
    /// Dragging a control point keeps the Bezier on the other side of the node collinear.
    bool keepBezierTangency{false};
    MirrorMode mirror{MirrorMode::Off};
    /// Tolerance for close / join / extend.
    geometry::Tolerance tolerance{};
    /// Pick radius in screen pixels.
    double hitTolerancePx{6.0};
};

/// Key that triggers an operation while the NodeEdit tool is active (letters are case-insensitive).
std::optional<NodeOp> opForKey(char key);
const char* opName(NodeOp op);

class NodeEditor {
   public:
    static constexpr double kDragThresholdPx = 4.0;
    /// `handles()` stops after this many.
    static constexpr std::size_t kMaxHandles = 200000;

    explicit NodeEditor(document::Document* doc = nullptr);

    /// Not owned. Drops the node selection and any gesture in progress (without touching either
    /// document).
    void setDocument(document::Document* doc);
    void setView(const ViewTransform& view) { view_ = view; }
    NodeEditOptions& options() { return options_; }
    const NodeEditOptions& options() const { return options_; }

    // --- Model ------------------------------------------------------------------------------
    /// Vector leaves being edited, in selection order.
    std::vector<document::ObjectId> editedObjects() const;
    /// Handles of the edited objects (nodes, Bezier control points, midpoints), ascending by
    /// object; selected ones flagged.
    std::vector<NodeHandle> handles() const;
    std::optional<NodeHit> hitTest(geometry::Vec2 px) const;
    std::optional<geometry::Vec2> nodePosition(const NodeRef& ref) const;

    // --- Selection (node and span selection; the object selection is the document's) --------
    /// Selected nodes in selection order; stale references (removed, locked, out of range) are
    /// dropped on every query.
    std::vector<NodeRef> selectedNodes() const;
    std::vector<SpanRef> selectedSpans() const;
    void clearSelection();
    void selectNode(const NodeRef& ref, bool add);
    void selectSpan(const SpanRef& ref, bool add);
    /// Selects every node of the edited objects.
    void selectAllNodes();

    // --- Pointer events (px: screen pixels; snappedMm: the snapped pointer, millimetres) ----
    NodeEditResult press(geometry::Vec2 px, geometry::Vec2 snappedMm, bool shift, bool ctrl, bool alt);
    NodeEditResult move(geometry::Vec2 px, geometry::Vec2 snappedMm, bool shift, bool ctrl, bool alt);
    NodeEditResult release(geometry::Vec2 px, geometry::Vec2 snappedMm, bool shift, bool ctrl, bool alt);
    /// Cancels a drag in progress (reverting its changes exactly) or a rectangle selection. False
    /// when nothing was in progress.
    bool escape(NodeEditResult* result = nullptr);
    /// Checks that nobody else edited the document during a drag; if so, ends the drag without
    /// reverting. Call at the top of every event.
    void validateGesture();
    /// True while a drag has changed (or is about to change) the document.
    bool gestureActive() const { return gesture_ != Gesture::None; }
    /// A button is down (press seen, release not yet).
    bool pressed() const { return pressed_; }
    /// Leaf objects the current drag edits (to exclude them from snapping).
    const std::vector<document::ObjectId>& gestureObjects() const { return gestureObjects_; }
    std::optional<geometry::Box2> marqueePx() const;

    // --- Operations -------------------------------------------------------------------------
    bool canRun(NodeOp op, std::string* why = nullptr) const;
    NodeEditResult run(NodeOp op, const NodeOpArgs& args = {});
    /// Moves every selected node by `deltaMm` (mirror mode applies).
    NodeEditResult moveSelectedNodes(geometry::Vec2 deltaMm);
    /// Typed coordinates for the selected nodes: relative adds, absolute assigns the given
    /// coordinate(s) to every selected node (a missing one is left alone).
    NodeEditResult setSelectedNodeCoordinates(std::optional<double> xMm, std::optional<double> yMm, bool relative);
    void setMirror(MirrorMode mode) { options_.mirror = mode; }
    /// Runs the operation bound to `key` (see `opForKey`); H / V toggle the mirror mode, K the
    /// Bezier tangency option, P asks for the Properties dialog. `handled` tells whether the key
    /// meant anything.
    NodeEditResult handleKey(char key);

   private:
    enum class Gesture { None, Nodes, Control, SpanReshape, SpanMove };
    struct Snapshot {
        document::ObjectId id;
        std::vector<geometry::Contour> contours;  ///< normalised
    };

    bool eligible(const document::Object& obj) const;
    bool pruneSelection() const;
    bool beginGesture(const geometry::Vec2& snappedMm, bool ctrl);
    bool applyGesture(geometry::Vec2 snappedMm);
    NodeEditResult finishGesture();
    NodeEditResult commitContours(std::vector<document::ContourReplacement> items, const std::vector<document::ObjectId>& removals,
                                  const char* text);

    document::Document* doc_{nullptr};
    ViewTransform view_;
    NodeEditOptions options_;

    // Selection (mutable: pruned lazily on query).
    mutable std::vector<NodeRef> nodes_;
    mutable std::vector<SpanRef> spans_;

    // Pointer state.
    bool pressed_{false};
    bool dragged_{false};
    geometry::Vec2 pressPx_{};
    geometry::Vec2 curPx_{};
    bool pressShift_{false};
    std::optional<NodeHit> pressHit_;
    bool marquee_{false};
    bool pressedNodeWasSelected_{false};

    // Drag.
    Gesture gesture_{Gesture::None};
    std::vector<document::ObjectId> gestureObjects_;
    std::vector<Snapshot> snapshots_;
    std::vector<std::vector<geometry::Contour>> lastApplied_;  ///< contours of the last pushed step
    bool insertedOnBegin_{false};
    bool pushed_{false};
    std::uint64_t generation_{0};
    geometry::Vec2 grabMm_{};      ///< Nodes: position of the grabbed node at the start; Span: grab point
    geometry::Vec2 centreMm_{};    ///< mirror axis centre
    NodeRef grabbedNode_;
    std::vector<NodeRef> dragNodes_;
    std::vector<geometry::Vec2> dragStart_;
    SpanRef dragSpan_;
    int dragControl_{0};
    double dragT_{0.0};
};

}  // namespace carvetto::viewmodel
