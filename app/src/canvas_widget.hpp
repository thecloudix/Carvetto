#pragma once

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>
#include <QSize>
#include <QWidget>
#include <array>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/toolpath_preview.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

class QEvent;
class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QPainter;
class QResizeEvent;
class QWheelEvent;

namespace carvetto::app {

/// Background outside the material.
inline QColor canvasBackground() {
    return QColor(226, 228, 232);
}
/// Fill of the material rectangle.
inline QColor materialFill() {
    return QColor(255, 255, 255);
}

/// What the toolpath preview overlay shows besides the model's own state (T-0223).
struct PreviewPaintState {
    /// While a simulation plays: moves before `playedMoves` (job-wide index) are drawn in full
    /// colour, the rest faded; otherwise every move is drawn in full colour.
    bool playback{false};
    std::size_t playedMoves{0};
    std::array<bool, viewmodel::kMoveClassCount> classVisible{true, true, true, true, true};
    /// Tool tip marker (position in mm, cutter diameter in mm); none when unset.
    std::optional<geometry::Vec2> toolPosMm;
    double toolDiameterMm{0.0};
    bool operator==(const PreviewPaintState&) const = default;
};

/// 2D design view: draws the active sheet's material and the vector contours of a `Document`
/// through a `viewmodel::ViewTransform`. Wheel zooms about the cursor; middle or right drag pans.
/// All geometry decisions live in `carvetto_viewmodel`; this widget only paints the scene.
class CanvasWidget : public QWidget {
   public:
    explicit CanvasWidget(QWidget* parent = nullptr);

    /// Document to draw and whose selection a click edits (not owned; may be null). Re-fits the
    /// view to the material.
    void setDocument(document::Document* doc);
    const viewmodel::ViewTransform& view() const { return view_; }
    /// Fits the active sheet's material into the widget with a 5% margin, and keeps it fitted on
    /// resize until the user zooms or pans.
    void zoomToMaterial();
    /// Click-to-select at a widget position (left click): replaces the selection with the object
    /// under the cursor, or clears it on empty canvas; `additive` (Shift) adds, `toggle` (Ctrl)
    /// flips. All picking is `viewmodel::clickSelect`. Returns true when an object was hit.
    bool selectAt(QPointF widgetPos, bool additive = false, bool toggle = false);
    /// Repaints after a document change.
    void refresh();

    /// Toolpath preview overlay (T-0223): the model is owned by the preview panel (not owned here;
    /// may be null). It is painted over the vectors, from a cached layer image that is redrawn only
    /// when the view, the model or the visible classes change (playback adds the newly played
    /// moves to the layer instead of redrawing it).
    void setPreview(const viewmodel::PreviewModel* model);
    void setPreviewState(const PreviewPaintState& state);
    /// Wall time (ms) and points of the last redraw of the preview layer (0 when it was cached).
    double lastPreviewRenderMs() const { return lastPreviewMs_; }
    std::size_t lastPreviewPoints() const { return lastPreviewPoints_; }

    /// Drawing and selection tools (T-0204). The canvas only forwards pointer and key events to
    /// the controller and paints its preview; the select tool owns click and marquee picking.
    viewmodel::ToolController& tools() { return tools_; }
    const viewmodel::ToolController& tools() const { return tools_; }

    /// Paints one frame into a new `size` image (RGB32) with a view fitted to the material, the
    /// same way `paintEvent` paints the widget. `usedView`, when given, receives that view.
    QImage renderFitted(QSize size, viewmodel::ViewTransform* usedView = nullptr) const;

    /// Called whenever the view transform changes (zoom, pan, resize).
    std::function<void()> onViewChanged;
    /// Called after a click changed (or tried to change) the selection.
    std::function<void()> onSelectionChanged;
    /// Called after a tool added an object to the document (one undo step).
    std::function<void()> onDocumentEdited;
    /// Called after every tool event that may change the numeric fields or the active shape.
    std::function<void()> onToolStateChanged;
    /// Called with the document position under the mouse.
    std::function<void(geometry::Vec2)> onCursorMoved;
    /// NodeEdit tool (T-0206): a right click without drag at a widget position (global position,
    /// widget pixels); the window shows the span / node menu.
    std::function<void(const QPoint& globalPos, geometry::Vec2 px)> onNodeContextMenu;
    /// NodeEdit tool: the P key asked for the Properties dialog.
    std::function<void()> onNodeProperties;
    /// NodeEdit tool: a key or event was refused; text for the status bar.
    std::function<void(const std::string&)> onNodeMessage;

   protected:
    bool event(QEvent* event) override;
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    bool focusNextPrevChild(bool next) override;

   private:
    void paintOverlay(QPainter& painter) const;
    void paintNodes(QPainter& painter) const;
    void paintDimensionHandles(QPainter& painter) const;
    void updateNodeCursor(QPointF pos);
    void paintSelectionBox(QPainter& painter) const;
    void updateCursorHint();
    void afterToolEvent(const viewmodel::ToolResult& result);
    static viewmodel::Modifiers toolModifiers(Qt::KeyboardModifiers mods);

    void paintScene(QPainter& painter, const viewmodel::ViewTransform& view, QSize size) const;
    void paintPreview(QPainter& painter, const viewmodel::ViewTransform& view, QSize size) const;
    void renderPreviewRange(QImage& layer, const viewmodel::ViewTransform& view, std::size_t from, std::size_t to,
                            bool played) const;
    void notifyViewChanged();

    struct PreviewLayerKey {
        std::uint64_t revision{0};
        double scale{0.0};
        geometry::Vec2 origin;
        QSize size;
        bool playback{false};
        std::array<bool, viewmodel::kMoveClassCount> classes{};
        bool operator==(const PreviewLayerKey& o) const {
            return revision == o.revision && scale == o.scale && origin.x == o.origin.x && origin.y == o.origin.y &&
                   size == o.size && playback == o.playback && classes == o.classes;
        }
    };

    document::Document* doc_{nullptr};
    const viewmodel::PreviewModel* preview_{nullptr};
    PreviewPaintState previewState_;
    mutable PreviewLayerKey layerKey_;
    mutable bool layerValid_{false};
    mutable QImage pendingLayer_;
    mutable QImage playedLayer_;
    mutable std::size_t playedUpTo_{0};
    mutable double lastPreviewMs_{0.0};
    mutable std::size_t lastPreviewPoints_{0};
    viewmodel::ViewTransform view_;
    viewmodel::ToolController tools_;
    bool fitPending_{true};
    bool panning_{false};
    QPointF lastPanPos_;
    QPointF rightPressPos_;
    bool rightMoved_{false};
};

}  // namespace carvetto::app
