#include "canvas_widget.hpp"

#include <QBrush>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPolygonF>
#include <QRectF>
#include <QResizeEvent>
#include <QWheelEvent>
#include <cmath>
#include <cstddef>

#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"

namespace carvetto::app {

using geometry::Vec2;
using viewmodel::ViewTransform;

namespace {

QPointF toQt(Vec2 v) {
    return QPointF(v.x, v.y);
}

QRectF toQt(const geometry::Box2& b) {
    return QRectF(QPointF(b.min.x, b.min.y), QPointF(b.max.x, b.max.y));
}

}  // namespace

CanvasWidget::CanvasWidget(QWidget* parent) : QWidget(parent) {
    setMouseTracking(true);
    setFocusPolicy(Qt::WheelFocus);
    setMinimumSize(200, 150);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
}

void CanvasWidget::setDocument(document::Document* doc) {
    doc_ = doc;
    tools_.setDocument(doc);
    fitPending_ = true;
    if (width() > 0 && height() > 0) {
        zoomToMaterial();
    }
    update();
}

void CanvasWidget::zoomToMaterial() {
    view_.setViewport(static_cast<double>(width()), static_cast<double>(height()));
    if (doc_ != nullptr) {
        view_.zoomToFit(viewmodel::activeMaterialMm(*doc_), 0.05);
    }
    // Keep fitting on resize until the user zooms or pans.
    fitPending_ = true;
    notifyViewChanged();
    update();
}

void CanvasWidget::refresh() {
    update();
}

bool CanvasWidget::selectAt(QPointF widgetPos, bool additive, bool toggle) {
    if (doc_ == nullptr) {
        return false;
    }
    const viewmodel::SelectMode mode = toggle     ? viewmodel::SelectMode::Toggle
                                       : additive ? viewmodel::SelectMode::Add
                                                  : viewmodel::SelectMode::Replace;
    const auto hit = viewmodel::clickSelect(*doc_, view_, Vec2{widgetPos.x(), widgetPos.y()}, mode);
    update();
    if (onSelectionChanged) {
        onSelectionChanged();
    }
    return hit.has_value();
}

void CanvasWidget::notifyViewChanged() {
    tools_.setView(view_);
    if (onViewChanged) {
        onViewChanged();
    }
}

QImage CanvasWidget::renderFitted(QSize size, ViewTransform* usedView) const {
    QImage image(size, QImage::Format_RGB32);
    image.fill(canvasBackground());
    ViewTransform view;
    view.setViewport(static_cast<double>(size.width()), static_cast<double>(size.height()));
    if (doc_ != nullptr) {
        view.zoomToFit(viewmodel::activeMaterialMm(*doc_), 0.05);
    }
    {
        QPainter painter(&image);
        paintScene(painter, view, size);
    }
    if (usedView != nullptr) {
        *usedView = view;
    }
    return image;
}

void CanvasWidget::paintScene(QPainter& painter, const ViewTransform& view, QSize size) const {
    painter.fillRect(QRectF(QPointF(0.0, 0.0), QSizeF(size)), canvasBackground());
    if (doc_ == nullptr) {
        return;
    }
    painter.setRenderHint(QPainter::Antialiasing, true);
    const viewmodel::CanvasScene scene = viewmodel::buildCanvasScene(*doc_, view);

    if (!scene.materialPx.isEmpty()) {
        const QRectF material = toQt(scene.materialPx);
        painter.fillRect(material, materialFill());
        QPen outline(QColor(120, 120, 120));
        outline.setWidthF(1.0);
        outline.setCosmetic(true);
        painter.setPen(outline);
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(material);
    }

    for (const viewmodel::DrawPath& path : scene.paths) {
        if (path.pointsPx.size() < 2) {
            continue;
        }
        QPainterPath qp;
        qp.moveTo(toQt(path.pointsPx.front()));
        for (std::size_t i = 1; i < path.pointsPx.size(); ++i) {
            qp.lineTo(toQt(path.pointsPx[i]));
        }
        if (path.closed) {
            qp.closeSubpath();
        }
        QPen pen(QColor(path.colour.r, path.colour.g, path.colour.b));
        pen.setWidthF(1.5);
        pen.setCosmetic(true);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(qp);
    }

    // Selection: dashed bounding box around the selected objects.
    if (const auto sel = viewmodel::selectionBounds(*doc_)) {
        QPen pen(QColor(30, 110, 230));
        pen.setStyle(Qt::DashLine);
        pen.setWidthF(1.0);
        pen.setCosmetic(true);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(toQt(view.worldToScreen(*sel)).adjusted(-3.0, -3.0, 3.0, 3.0));
    }

    // Datum marker: a small filled square at the active sheet's XY datum.
    const QPointF datum = toQt(scene.datumPx);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(QColor(200, 30, 30)));
    painter.drawRect(QRectF(datum.x() - 4.0, datum.y() - 4.0, 8.0, 8.0));
}

void CanvasWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    paintScene(painter, view_, size());
    paintOverlay(painter);
}

void CanvasWidget::paintOverlay(QPainter& painter) const {
    if (doc_ == nullptr) {
        return;
    }
    painter.setRenderHint(QPainter::Antialiasing, true);
    const viewmodel::ToolPreview preview = tools_.preview();

    QPen previewPen(preview.valid ? QColor(30, 110, 230) : QColor(150, 150, 150));
    previewPen.setStyle(Qt::DashLine);
    previewPen.setWidthF(1.5);
    previewPen.setCosmetic(true);
    painter.setPen(previewPen);
    painter.setBrush(Qt::NoBrush);
    for (const geometry::Contour& contour : preview.contours) {
        const std::vector<Vec2> pts = viewmodel::polygoniseForScreen(contour, view_, 0.25);
        if (pts.size() < 2) {
            continue;
        }
        QPainterPath qp;
        qp.moveTo(toQt(pts.front()));
        for (std::size_t i = 1; i < pts.size(); ++i) {
            qp.lineTo(toQt(pts[i]));
        }
        if (contour.closed) {
            qp.closeSubpath();
        }
        painter.drawPath(qp);
    }

    painter.setPen(QPen(QColor(30, 110, 230), 1.0));
    painter.setBrush(QBrush(QColor(255, 255, 255)));
    for (const Vec2& a : preview.anchorsMm) {
        const QPointF p = toQt(view_.worldToScreen(a));
        painter.drawRect(QRectF(p.x() - 3.0, p.y() - 3.0, 6.0, 6.0));
    }

    if (preview.marqueePx.has_value()) {
        QPen pen(QColor(30, 110, 230));
        pen.setStyle(Qt::DashLine);
        pen.setCosmetic(true);
        painter.setPen(pen);
        painter.setBrush(QBrush(QColor(30, 110, 230, 30)));
        painter.drawRect(toQt(*preview.marqueePx));
    }

    paintSelectionBox(painter);

    // Snap glyph at the snapped point; the shape depends on the kind.
    if (tools_.snap().has_value()) {
        const viewmodel::Snap& snap = *tools_.snap();
        const QPointF p = toQt(view_.worldToScreen(snap.pointMm));
        QPen pen(QColor(230, 120, 0));
        pen.setWidthF(2.0);
        pen.setCosmetic(true);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        constexpr double r = 6.0;
        switch (snap.kind) {
            case viewmodel::SnapKind::Endpoint:
                painter.drawRect(QRectF(p.x() - r, p.y() - r, 2 * r, 2 * r));
                break;
            case viewmodel::SnapKind::Midpoint:
                painter.drawPolygon(QPolygonF({QPointF(p.x(), p.y() - r), QPointF(p.x() - r, p.y() + r),
                                               QPointF(p.x() + r, p.y() + r)}));
                break;
            case viewmodel::SnapKind::Centre:
                painter.drawEllipse(p, r, r);
                break;
            case viewmodel::SnapKind::Quadrant:
            case viewmodel::SnapKind::Nearest:
                painter.drawPolygon(QPolygonF({QPointF(p.x(), p.y() - r), QPointF(p.x() + r, p.y()),
                                               QPointF(p.x(), p.y() + r), QPointF(p.x() - r, p.y())}));
                break;
            case viewmodel::SnapKind::Intersection:
                painter.drawLine(QPointF(p.x() - r, p.y() - r), QPointF(p.x() + r, p.y() + r));
                painter.drawLine(QPointF(p.x() - r, p.y() + r), QPointF(p.x() + r, p.y() - r));
                break;
            case viewmodel::SnapKind::Perpendicular:
            case viewmodel::SnapKind::Tangent:
                painter.drawLine(QPointF(p.x() - r, p.y() + r), QPointF(p.x() + r, p.y() + r));
                painter.drawLine(QPointF(p.x() - r, p.y() + r), QPointF(p.x() - r, p.y() - r));
                break;
            case viewmodel::SnapKind::Guide:
            case viewmodel::SnapKind::Grid:
                painter.drawLine(QPointF(p.x() - r, p.y()), QPointF(p.x() + r, p.y()));
                painter.drawLine(QPointF(p.x(), p.y() - r), QPointF(p.x(), p.y() + r));
                break;
        }
    }
}

void CanvasWidget::paintSelectionBox(QPainter& painter) const {
    const std::optional<viewmodel::SelectionBox> box = tools_.selectionHandles();
    if (!box.has_value()) {
        return;
    }
    const bool rotate = tools_.transformMode() == viewmodel::TransformMode::Rotate;
    const geometry::Box2 outline = view_.worldToScreen(box->boundsMm);
    QPen dashed(QColor(30, 110, 230));
    dashed.setStyle(Qt::DashLine);
    dashed.setCosmetic(true);
    painter.setPen(dashed);
    painter.setBrush(Qt::NoBrush);
    painter.drawRect(toQt(outline));
    painter.setPen(QPen(QColor(30, 110, 230), 1.0));
    for (const viewmodel::Handle& h : box->handles) {
        const QPointF p = toQt(h.positionPx);
        const bool corner = h.kind == viewmodel::HandleKind::TopLeft || h.kind == viewmodel::HandleKind::TopRight ||
                            h.kind == viewmodel::HandleKind::BottomLeft || h.kind == viewmodel::HandleKind::BottomRight;
        if (h.kind == viewmodel::HandleKind::Centre) {
            painter.setBrush(QBrush(QColor(255, 255, 255)));
            painter.drawEllipse(p, 4.0, 4.0);
        } else if (rotate) {
            if (corner) {  // black corners rotate
                painter.setBrush(QBrush(QColor(20, 20, 20)));
                painter.drawEllipse(p, 4.5, 4.5);
            }
        } else {
            painter.setBrush(QBrush(QColor(255, 255, 255)));
            painter.drawRect(QRectF(p.x() - 4.0, p.y() - 4.0, 8.0, 8.0));
        }
    }
    if (rotate) {
        if (const std::optional<Vec2> pivot = tools_.pivotMm()) {
            const QPointF p = toQt(view_.worldToScreen(*pivot));
            painter.setPen(QPen(QColor(200, 30, 30), 1.5));
            painter.setBrush(Qt::NoBrush);
            painter.drawEllipse(p, 6.0, 6.0);
            painter.drawLine(QPointF(p.x() - 9.0, p.y()), QPointF(p.x() + 9.0, p.y()));
            painter.drawLine(QPointF(p.x(), p.y() - 9.0), QPointF(p.x(), p.y() + 9.0));
        }
    }
}

void CanvasWidget::updateCursorHint() {
    switch (tools_.cursorHint()) {
        case viewmodel::CursorHint::Default: unsetCursor(); break;
        case viewmodel::CursorHint::Move: setCursor(Qt::SizeAllCursor); break;
        case viewmodel::CursorHint::ScaleHorizontal: setCursor(Qt::SizeHorCursor); break;
        case viewmodel::CursorHint::ScaleVertical: setCursor(Qt::SizeVerCursor); break;
        case viewmodel::CursorHint::ScaleDiagonalUp: setCursor(Qt::SizeBDiagCursor); break;
        case viewmodel::CursorHint::ScaleDiagonalDown: setCursor(Qt::SizeFDiagCursor); break;
        case viewmodel::CursorHint::Rotate:
        case viewmodel::CursorHint::Pivot: setCursor(Qt::CrossCursor); break;
    }
}

viewmodel::Modifiers CanvasWidget::toolModifiers(Qt::KeyboardModifiers mods) {
    viewmodel::Modifiers m;
    m.shift = mods.testFlag(Qt::ShiftModifier);
    m.ctrl = mods.testFlag(Qt::ControlModifier);
    m.alt = mods.testFlag(Qt::AltModifier);
    return m;
}

void CanvasWidget::afterToolEvent(const viewmodel::ToolResult& result) {
    if ((result.committed || result.transformed) && onDocumentEdited) {
        onDocumentEdited();
    }
    if (result.selectionChanged && onSelectionChanged) {
        onSelectionChanged();
    }
    if (onToolStateChanged) {
        onToolStateChanged();
    }
    update();
}

bool CanvasWidget::focusNextPrevChild(bool next) {
    // Tab closes a polyline instead of moving the focus while a tool is using the keys.
    return tools_.wantsKeys() ? false : QWidget::focusNextPrevChild(next);
}

void CanvasWidget::keyPressEvent(QKeyEvent* event) {
    if (!tools_.wantsKeys()) {
        QWidget::keyPressEvent(event);
        return;
    }
    viewmodel::ToolResult result;
    switch (event->key()) {
        case Qt::Key_Escape:
            result = tools_.key(viewmodel::ToolKey::Escape);
            break;
        case Qt::Key_Return:
        case Qt::Key_Enter:
            result = tools_.key(viewmodel::ToolKey::Enter);
            break;
        case Qt::Key_Backspace:
            result = tools_.key(viewmodel::ToolKey::Backspace);
            break;
        case Qt::Key_Tab:
            result = tools_.key(viewmodel::ToolKey::Close);
            break;
        default:
            QWidget::keyPressEvent(event);
            return;
    }
    event->accept();
    afterToolEvent(result);
}

void CanvasWidget::mouseDoubleClickEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        tools_.setView(view_);
        const QPointF pos = event->position();
        afterToolEvent(tools_.doubleClick(Vec2{pos.x(), pos.y()}, toolModifiers(event->modifiers())));
        event->accept();
        return;
    }
    QWidget::mouseDoubleClickEvent(event);
}

void CanvasWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    view_.setViewport(static_cast<double>(width()), static_cast<double>(height()));
    if (fitPending_) {
        zoomToMaterial();
    } else {
        notifyViewChanged();
    }
}

void CanvasWidget::wheelEvent(QWheelEvent* event) {
    const int delta = event->angleDelta().y();
    if (delta == 0) {
        event->ignore();
        return;
    }
    const QPointF pos = event->position();
    // 120 units (one notch) ≈ 20% zoom.
    const double factor = std::pow(1.0015, static_cast<double>(delta));
    view_.zoomAbout(Vec2{pos.x(), pos.y()}, factor);
    fitPending_ = false;
    event->accept();
    notifyViewChanged();
    update();
}

void CanvasWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton) {
        panning_ = true;
        lastPanPos_ = event->position();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton) {
        setFocus(Qt::MouseFocusReason);
        tools_.setView(view_);
        const QPointF pos = event->position();
        afterToolEvent(tools_.pointerPress(Vec2{pos.x(), pos.y()}, toolModifiers(event->modifiers())));
        event->accept();
        return;
    }
    QWidget::mousePressEvent(event);
}

void CanvasWidget::mouseMoveEvent(QMouseEvent* event) {
    const QPointF pos = event->position();
    if (panning_) {
        const QPointF d = pos - lastPanPos_;
        lastPanPos_ = pos;
        view_.pan(Vec2{d.x(), d.y()});
        fitPending_ = false;
        notifyViewChanged();
        update();
    }
    tools_.setView(view_);
    const viewmodel::ToolResult moved = tools_.pointerMove(Vec2{pos.x(), pos.y()}, toolModifiers(event->modifiers()));
    if (onCursorMoved) {
        onCursorMoved(tools_.pointMm());
    }
    if (moved.transformed && onDocumentEdited) {
        onDocumentEdited();  // an interactive transform step changed the document
    }
    if (onToolStateChanged) {
        onToolStateChanged();
    }
    if (!panning_) {
        updateCursorHint();
    }
    update();
    QWidget::mouseMoveEvent(event);
}

void CanvasWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (panning_ && (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton)) {
        panning_ = false;
        unsetCursor();
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton) {
        tools_.setView(view_);
        const QPointF pos = event->position();
        afterToolEvent(tools_.pointerRelease(Vec2{pos.x(), pos.y()}, toolModifiers(event->modifiers())));
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

}  // namespace carvetto::app
