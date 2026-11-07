#pragma once

#include <QColor>
#include <QImage>
#include <QPoint>
#include <QPointF>
#include <QSize>
#include <QWidget>
#include <functional>
#include <optional>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

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

   protected:
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
    void paintSelectionBox(QPainter& painter) const;
    void updateCursorHint();
    void afterToolEvent(const viewmodel::ToolResult& result);
    static viewmodel::Modifiers toolModifiers(Qt::KeyboardModifiers mods);

    void paintScene(QPainter& painter, const viewmodel::ViewTransform& view, QSize size) const;
    void notifyViewChanged();

    document::Document* doc_{nullptr};
    viewmodel::ViewTransform view_;
    viewmodel::ToolController tools_;
    bool fitPending_{true};
    bool panning_{false};
    QPointF lastPanPos_;
};

}  // namespace carvetto::app
