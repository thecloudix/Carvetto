#include "canvas_widget.hpp"

#include <QBrush>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
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
        const Qt::KeyboardModifiers mods = event->modifiers();
        selectAt(event->position(), mods.testFlag(Qt::ShiftModifier), mods.testFlag(Qt::ControlModifier));
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
    if (onCursorMoved) {
        onCursorMoved(view_.screenToWorld(Vec2{pos.x(), pos.y()}));
    }
    QWidget::mouseMoveEvent(event);
}

void CanvasWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (panning_ && (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton)) {
        panning_ = false;
        unsetCursor();
        event->accept();
        return;
    }
    QWidget::mouseReleaseEvent(event);
}

}  // namespace carvetto::app
