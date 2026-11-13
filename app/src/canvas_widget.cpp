#include "canvas_widget.hpp"

#include <QBrush>
#include <QByteArray>
#include <QElapsedTimer>
#include <QEvent>
#include <QFont>
#include <QKeyEvent>
#include <QLineF>
#include <QMouseEvent>
#include <QPaintEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPen>
#include <QPolygonF>
#include <QRectF>
#include <QString>
#include <QResizeEvent>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstddef>
#include <vector>

#include "carvetto/viewmodel/bitmap_trace.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"
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
    bitmapCache_.clear();
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

void CanvasWidget::setPreview(const viewmodel::PreviewModel* model) {
    preview_ = model;
    layerValid_ = false;
    update();
}

void CanvasWidget::setPreviewState(const PreviewPaintState& state) {
    if (!(previewState_ == state)) {
        previewState_ = state;
        update();
    }
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

const QImage& CanvasWidget::bitmapImage(const std::string& key) const {
    static const QImage none;
    if (doc_ == nullptr) {
        return none;
    }
    const auto found = bitmapCache_.find(key);
    if (found != bitmapCache_.end()) {
        return found->second;
    }
    QImage image;
    if (const document::BitmapAsset* asset = doc_->findBitmap(key)) {
        image = QImage::fromData(QByteArray::fromRawData(reinterpret_cast<const char*>(asset->bytes.data()),
                                                         static_cast<qsizetype>(asset->bytes.size())));
        if (!image.isNull() && (image.width() > 1024 || image.height() > 1024)) {
            image = image.scaled(1024, 1024, Qt::KeepAspectRatio, Qt::SmoothTransformation);
        }
    }
    if (bitmapCache_.size() >= 32) {
        bitmapCache_.clear();
    }
    return bitmapCache_.emplace(key, std::move(image)).first->second;
}

void CanvasWidget::paintSides(QPainter& painter, const ViewTransform& view) const {
    if (doc_ == nullptr || doc_->job().type != document::JobType::DoubleSided) {
        return;
    }
    const std::vector<viewmodel::SideOutline> outlines = viewmodel::multiSidedView(*doc_);
    if (outlines.empty()) {
        return;
    }
    const viewmodel::SideOutline& active = outlines.front();
    if (multiSided_ && outlines.size() > 1 && !active.otherSideContent.isEmpty()) {
        QPen pen(QColor(220, 120, 20));
        pen.setStyle(Qt::DashDotLine);
        pen.setWidthF(1.0);
        pen.setCosmetic(true);
        painter.setPen(pen);
        painter.setBrush(QColor(220, 120, 20, 28));
        painter.drawRect(toQt(view.worldToScreen(active.otherSideContent)));
    }
    // Active side and flip direction, in the corner of the material.
    const QRectF material = toQt(view.worldToScreen(active.material));
    const bool back = active.side == document::Side::Back;
    const bool horizontal = doc_->job().flipDirection == document::FlipDirection::Horizontal;
    QString text = back ? QStringLiteral("Bottom side") : QStringLiteral("Top side");
    text += horizontal ? QStringLiteral(" - flip left-right") : QStringLiteral(" - flip top-bottom");
    if (multiSided_) {
        text += QStringLiteral(" - multi-sided view");
    }
    painter.setPen(QColor(90, 90, 90));
    painter.drawText(material.topLeft() + QPointF(6.0, 14.0), text);
}

void CanvasWidget::setBitmapFade(double fade) {
    const double f = viewmodel::clampBitmapFade(fade);
    if (f != bitmapFade_) {
        bitmapFade_ = f;
        update();
    }
}

void CanvasWidget::beginPick(std::function<void(std::optional<geometry::Vec2>)> onPicked) {
    onPicked_ = std::move(onPicked);
    setCursor(Qt::CrossCursor);
}

void CanvasWidget::pickClick(QPointF widgetPos) {
    if (!onPicked_) {
        return;
    }
    auto done = std::move(onPicked_);
    onPicked_ = nullptr;
    unsetCursor();
    done(view_.screenToWorld(Vec2{widgetPos.x(), widgetPos.y()}));
}

void CanvasWidget::cancelPick() {
    if (!onPicked_) {
        return;
    }
    auto done = std::move(onPicked_);
    onPicked_ = nullptr;
    unsetCursor();
    done(std::nullopt);
}

void CanvasWidget::paintScene(QPainter& painter, const ViewTransform& view, QSize size) const {
    painter.fillRect(QRectF(QPointF(0.0, 0.0), QSizeF(size)), canvasBackground());
    if (doc_ == nullptr) {
        return;
    }
    painter.setRenderHint(QPainter::Antialiasing, true);
    viewmodel::SceneOptions sceneOptions;
    sceneOptions.haloCache = &haloCache_;
    const viewmodel::CanvasScene scene = viewmodel::buildCanvasScene(*doc_, view, sceneOptions);

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

    // Bitmaps lie under the vectors: the image, then a frame in the layer colour.
    for (const viewmodel::DrawBitmap& bitmap : scene.bitmaps) {
        const QRectF rect = toQt(bitmap.rectPx);
        const QImage& image = bitmapImage(bitmap.key);
        if (!image.isNull()) {
            painter.save();
            painter.setOpacity(bitmapFade_);
            painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
            painter.drawImage(rect, image);
            painter.restore();
        }
        QPen frame(QColor(bitmap.colour.r, bitmap.colour.g, bitmap.colour.b));
        frame.setWidthF(1.0);
        frame.setCosmetic(true);
        painter.setPen(frame);
        painter.setBrush(Qt::NoBrush);
        painter.drawRect(rect);
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

    // Dimension texts: glyph outlines filled (non-zero), or the toolkit's own text when no font could
    // set them.
    for (const viewmodel::DrawLabel& label : scene.labels) {
        const QColor ink(label.colour.r, label.colour.g, label.colour.b);
        if (!label.outlinesPx.empty()) {
            QPainterPath qp;
            qp.setFillRule(Qt::WindingFill);
            for (const std::vector<Vec2>& poly : label.outlinesPx) {
                if (poly.size() < 3) {
                    continue;
                }
                qp.moveTo(toQt(poly.front()));
                for (std::size_t i = 1; i < poly.size(); ++i) {
                    qp.lineTo(toQt(poly[i]));
                }
                qp.closeSubpath();
            }
            painter.setPen(Qt::NoPen);
            painter.setBrush(ink);
            painter.drawPath(qp);
        } else if (label.heightPx >= 1.0) {
            painter.save();
            painter.translate(toQt(label.centrePx));
            painter.rotate(label.angle * 180.0 / 3.14159265358979323846);
            QFont font = painter.font();
            font.setPixelSize(std::max(1, static_cast<int>(label.heightPx / 0.73)));
            painter.setFont(font);
            painter.setPen(ink);
            const QRectF box(-1.0e4, -1.0e4, 2.0e4, 2.0e4);
            painter.drawText(box, Qt::AlignCenter, QString::fromStdString(label.text));
            painter.restore();
        }
    }

    paintKeepOut(painter, scene);
    paintPreview(painter, view, size);

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

    paintSides(painter, view);

    // Datum marker: a small filled square at the active sheet's XY datum.
    const QPointF datum = toQt(scene.datumPx);
    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(QColor(200, 30, 30)));
    painter.drawRect(QRectF(datum.x() - 4.0, datum.y() - 4.0, 8.0, 8.0));
}

void CanvasWidget::paintKeepOut(QPainter& painter, const viewmodel::CanvasScene& scene) const {
    auto trace = [](const viewmodel::DrawPath& path) {
        QPainterPath qp;
        if (path.pointsPx.size() < 2) {
            return qp;
        }
        qp.moveTo(toQt(path.pointsPx.front()));
        for (std::size_t i = 1; i < path.pointsPx.size(); ++i) {
            qp.lineTo(toQt(path.pointsPx[i]));
        }
        if (path.closed) {
            qp.closeSubpath();
        }
        return qp;
    };
    // Clearance halo: a dashed outline round the zones.
    QPen haloPen(QColor(220, 60, 60, 200));
    haloPen.setStyle(Qt::DashLine);
    haloPen.setWidthF(1.0);
    haloPen.setCosmetic(true);
    painter.setPen(haloPen);
    painter.setBrush(Qt::NoBrush);
    for (const viewmodel::DrawPath& path : scene.keepOutHalo) {
        painter.drawPath(trace(path));
    }
    // The zones: translucent red fill with a solid outline.
    QPen zonePen(QColor(200, 30, 30));
    zonePen.setWidthF(1.5);
    zonePen.setCosmetic(true);
    painter.setPen(zonePen);
    painter.setBrush(QBrush(QColor(220, 60, 60, 60)));
    for (const viewmodel::DrawPath& path : scene.keepOutZones) {
        painter.drawPath(trace(path));
    }
}

namespace {

QColor previewPenColour(const viewmodel::PreviewStyle& style) {
    const viewmodel::PreviewColour c = viewmodel::colourOf(style.cls);
    if (style.stale) {
        return QColor(140, 140, 140, style.played ? 220 : 90);
    }
    return QColor(c.r, c.g, c.b, style.played ? 255 : 80);
}

}  // namespace

void CanvasWidget::renderPreviewRange(QImage& layer, const ViewTransform& view, std::size_t from, std::size_t to,
                                      bool played) const {
    viewmodel::PreviewDrawOptions options;
    options.fromMove = from;
    options.toMove = to;
    options.played = played;
    options.classVisible = previewState_.classVisible;
    options.maxPointsOut = 400000;  // level of detail: a frame stays bounded for pathological jobs
    const viewmodel::PreviewDrawList list = preview_->draw(view, options);
    lastPreviewPoints_ += list.stats.pointsOut;
    QPainter p(&layer);
    p.setRenderHint(QPainter::Antialiasing, list.stats.pointsOut < 150000);
    std::vector<QPointF> buffer;
    for (const viewmodel::PreviewBatch& batch : list.batches) {
        QPen pen(previewPenColour(batch.style));
        pen.setCosmetic(true);
        pen.setWidthF(batch.style.highlighted ? 2.5 : 1.0);
        pen.setCapStyle(Qt::RoundCap);
        pen.setJoinStyle(Qt::RoundJoin);
        if (batch.style.cls == viewmodel::MoveClass::Rapid) {
            pen.setStyle(Qt::DotLine);
        } else if (batch.style.stale) {
            pen.setStyle(Qt::DashLine);
        }
        p.setPen(pen);
        buffer.clear();
        buffer.reserve(batch.points.size());
        for (const Vec2& pt : batch.points) {
            buffer.emplace_back(pt.x, pt.y);
        }
        std::size_t at = 0;
        for (const std::uint32_t len : batch.runLengths) {
            if (len == 1) {
                QPen dot = pen;
                dot.setWidthF(batch.style.highlighted ? 5.0 : 3.5);
                dot.setStyle(Qt::SolidLine);
                p.setPen(dot);
                p.drawPoint(buffer[at]);
                p.setPen(pen);
            } else if (len > 1) {
                p.drawPolyline(buffer.data() + at, static_cast<int>(len));
            }
            at += len;
        }
    }
}

void CanvasWidget::paintPreview(QPainter& painter, const ViewTransform& view, QSize size) const {
    lastPreviewMs_ = 0.0;
    lastPreviewPoints_ = 0;
    if (preview_ == nullptr || !preview_->enabled() || preview_->scene().totalMoves == 0 || size.isEmpty()) {
        return;
    }
    const std::size_t total = preview_->scene().totalMoves;
    const bool playback = previewState_.playback;
    const std::size_t played = playback ? std::min(previewState_.playedMoves, total) : total;
    PreviewLayerKey key;
    key.revision = preview_->revision();
    key.scale = view.scale();
    key.origin = view.origin();
    key.size = size;
    key.playback = playback;
    key.classes = previewState_.classVisible;

    QElapsedTimer timer;
    timer.start();
    const bool rebuild = !layerValid_ || !(key == layerKey_) || playedLayer_.size() != size;
    if (rebuild) {
        layerKey_ = key;
        layerValid_ = true;
        playedLayer_ = QImage(size, QImage::Format_ARGB32_Premultiplied);
        playedLayer_.fill(Qt::transparent);
        playedUpTo_ = 0;
        pendingLayer_ = QImage();
        if (playback) {
            pendingLayer_ = QImage(size, QImage::Format_ARGB32_Premultiplied);
            pendingLayer_.fill(Qt::transparent);
            renderPreviewRange(pendingLayer_, view, 0, total, false);
        }
    }
    if (played < playedUpTo_) {
        playedLayer_.fill(Qt::transparent);
        playedUpTo_ = 0;
    }
    if (played > playedUpTo_) {
        renderPreviewRange(playedLayer_, view, playedUpTo_, played, true);
        playedUpTo_ = played;
    }
    if (rebuild || lastPreviewPoints_ > 0) {
        lastPreviewMs_ = static_cast<double>(timer.nsecsElapsed()) / 1.0e6;
    }
    if (!pendingLayer_.isNull()) {
        painter.drawImage(QPoint(0, 0), pendingLayer_);
    }
    painter.drawImage(QPoint(0, 0), playedLayer_);

    if (previewState_.toolPosMm.has_value()) {
        const QPointF c = toQt(view.worldToScreen(*previewState_.toolPosMm));
        const double r = std::max(5.0, 0.5 * previewState_.toolDiameterMm * view.scale());
        painter.setRenderHint(QPainter::Antialiasing, true);
        QPen outline(QColor(20, 20, 20));
        outline.setCosmetic(true);
        outline.setWidthF(1.5);
        painter.setPen(outline);
        painter.setBrush(QBrush(QColor(255, 220, 0, 90)));
        painter.drawEllipse(c, r, r);
        painter.drawLine(QPointF(c.x() - r - 3.0, c.y()), QPointF(c.x() + r + 3.0, c.y()));
        painter.drawLine(QPointF(c.x(), c.y() - r - 3.0), QPointF(c.x(), c.y() + r + 3.0));
    }
}

void CanvasWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    paintScene(painter, view_, size());
    paintOverlay(painter);
}

// In-canvas text frame, selection highlight and caret of the open text draft (T-0230).
void CanvasWidget::paintTextFrame(QPainter& painter, const viewmodel::ToolPreview& preview) const {
    if (preview.textFrameMm.has_value()) {
        QPen pen(QColor(120, 120, 120));
        pen.setStyle(Qt::DotLine);
        pen.setCosmetic(true);
        painter.setPen(pen);
        painter.setBrush(Qt::NoBrush);
        const QPointF a = toQt(view_.worldToScreen(preview.textFrameMm->min));
        const QPointF b = toQt(view_.worldToScreen(preview.textFrameMm->max));
        painter.drawRect(QRectF(a, b).normalized());
    }
    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(QColor(30, 110, 230, 70)));
    for (const auto& quad : preview.textSelectionMm) {
        QPolygonF poly;
        for (const Vec2& corner : quad) {
            poly << toQt(view_.worldToScreen(corner));
        }
        painter.drawPolygon(poly);
    }
    if (preview.textCaretMm.has_value()) {
        QPen pen(QColor(20, 20, 20));
        pen.setWidthF(1.5);
        pen.setCosmetic(true);
        painter.setPen(pen);
        painter.drawLine(toQt(view_.worldToScreen(preview.textCaretMm->first)),
                         toQt(view_.worldToScreen(preview.textCaretMm->second)));
    }
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

    paintTextFrame(painter, preview);

    if (preview.marqueePx.has_value()) {
        QPen pen(QColor(30, 110, 230));
        pen.setStyle(Qt::DashLine);
        pen.setCosmetic(true);
        painter.setPen(pen);
        painter.setBrush(QBrush(QColor(30, 110, 230, 30)));
        painter.drawRect(toQt(*preview.marqueePx));
    }

    paintNodes(painter);
    paintDimensionHandles(painter);
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

void CanvasWidget::paintDimensionHandles(QPainter& painter) const {
    if (tools_.tool() != viewmodel::ToolKind::Dimension) {
        return;
    }
    // White squares change size or placement, black ones the legs and the text height.
    for (const document::DimensionHandle& h : tools_.dimensionTool().handles()) {
        const QPointF p = toQt(view_.worldToScreen(h.pos));
        painter.setPen(QPen(h.white ? QColor(30, 110, 230) : QColor(255, 255, 255), 1.0));
        painter.setBrush(h.white ? QColor(255, 255, 255) : QColor(0, 0, 0));
        painter.drawRect(QRectF(p.x() - 4.0, p.y() - 4.0, 8.0, 8.0));
    }
}

void CanvasWidget::paintNodes(QPainter& painter) const {
    if (tools_.tool() != viewmodel::ToolKind::NodeEdit) {
        return;
    }
    const std::vector<viewmodel::NodeHandle> handles = tools_.nodeEditor().handles();
    const QColor blue(30, 110, 230);
    const QColor orange(230, 120, 0);
    // Control handle lines first, so the squares sit on top.
    painter.setBrush(Qt::NoBrush);
    painter.setPen(QPen(QColor(140, 140, 160), 1.0, Qt::DashLine));
    for (const viewmodel::NodeHandle& h : handles) {
        if (h.type == viewmodel::HandleType::Control) {
            painter.drawLine(toQt(view_.worldToScreen(h.anchorMm)), toQt(view_.worldToScreen(h.positionMm)));
        }
    }
    for (const viewmodel::NodeHandle& h : handles) {
        const QPointF p = toQt(view_.worldToScreen(h.positionMm));
        switch (h.type) {
            case viewmodel::HandleType::Node:
                painter.setPen(QPen(blue, 1.5));
                painter.setBrush(h.selected ? QBrush(blue) : QBrush(QColor(255, 255, 255)));
                painter.drawRect(QRectF(p.x() - 4.0, p.y() - 4.0, 8.0, 8.0));
                break;
            case viewmodel::HandleType::Control:
                painter.setPen(QPen(QColor(110, 110, 150), 1.0));
                painter.setBrush(QBrush(QColor(255, 255, 255)));
                painter.drawEllipse(p, 3.5, 3.5);
                break;
            case viewmodel::HandleType::Midpoint:
                painter.setPen(QPen(h.selected ? orange : QColor(120, 160, 220), 1.0));
                painter.setBrush(h.selected ? QBrush(orange) : Qt::NoBrush);
                painter.drawPolygon(QPolygonF({QPointF(p.x(), p.y() - 3.5), QPointF(p.x() + 3.5, p.y()),
                                               QPointF(p.x(), p.y() + 3.5), QPointF(p.x() - 3.5, p.y())}));
                break;
            case viewmodel::HandleType::Span:
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

void CanvasWidget::updateNodeCursor(QPointF pos) {
    if (tools_.tool() != viewmodel::ToolKind::NodeEdit || tools_.nodeEditor().pressed()) {
        return;
    }
    tools_.setView(view_);
    const auto hit = tools_.nodeEditor().hitTest(Vec2{pos.x(), pos.y()});
    if (!hit.has_value()) {
        unsetCursor();
        return;
    }
    switch (hit->type) {
        case viewmodel::HandleType::Node:
        case viewmodel::HandleType::Control:
            setCursor(Qt::PointingHandCursor);
            break;
        case viewmodel::HandleType::Midpoint:
        case viewmodel::HandleType::Span:
            setCursor(Qt::SizeAllCursor);
            break;
    }
}

bool CanvasWidget::event(QEvent* e) {
    // While the NodeEdit tool is active its letter keys win over window-level shortcuts.
    if (e->type() == QEvent::ShortcutOverride && tools_.tool() == viewmodel::ToolKind::NodeEdit) {
        auto* ke = static_cast<QKeyEvent*>(e);
        const bool plain = !(ke->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier));
        if (plain && ke->key() >= Qt::Key_A && ke->key() <= Qt::Key_Z) {
            e->accept();
            return true;
        }
    }
    // While a text is being typed every plain key belongs to the text, not to window shortcuts.
    if (e->type() == QEvent::ShortcutOverride && tools_.tool() == viewmodel::ToolKind::Text &&
        tools_.textEditor().hasDraft()) {
        auto* ke = static_cast<QKeyEvent*>(e);
        const bool command = ke->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier);
        if (!command || ke->key() == Qt::Key_Return || ke->key() == Qt::Key_Enter) {
            e->accept();
            return true;
        }
    }
    return QWidget::event(e);
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
    if (!result.message.empty() && tools_.tool() == viewmodel::ToolKind::Dimension && onNodeMessage) {
        onNodeMessage(result.message);
    }
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
    if (picking() && event->key() == Qt::Key_Escape) {
        cancelPick();
        event->accept();
        return;
    }
    if (tools_.tool() == viewmodel::ToolKind::NodeEdit && !tools_.wantsKeys() && event->key() >= Qt::Key_A &&
        event->key() <= Qt::Key_Z &&
        !(event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier))) {
        tools_.setView(view_);
        const viewmodel::NodeEditResult r =
            tools_.nodeEditor().handleKey(static_cast<char>('a' + (event->key() - Qt::Key_A)));
        if (r.handled) {
            event->accept();
            if (r.openProperties && onNodeProperties) {
                onNodeProperties();
            }
            if (!r.ok && onNodeMessage) {
                onNodeMessage(r.message);
            }
            if (r.changed && onDocumentEdited) {
                onDocumentEdited();
            }
            if (onToolStateChanged) {
                onToolStateChanged();
            }
            update();
            return;
        }
    }
    if (tools_.tool() == viewmodel::ToolKind::Text && tools_.textEditor().hasDraft()) {
        const bool command = event->modifiers() & (Qt::ControlModifier | Qt::AltModifier | Qt::MetaModifier);
        viewmodel::ToolResult textResult;
        bool handled = true;
        if (event->key() == Qt::Key_Escape) {
            textResult = tools_.key(viewmodel::ToolKey::Escape);
        } else if (event->key() == Qt::Key_Backspace) {
            textResult = tools_.key(viewmodel::ToolKey::Backspace);
        } else if (event->key() == Qt::Key_Delete) {
            textResult = tools_.key(viewmodel::ToolKey::Delete);
        } else if (event->key() == Qt::Key_Left || event->key() == Qt::Key_Right || event->key() == Qt::Key_Home ||
                   event->key() == Qt::Key_End) {
            const bool ctrl = event->modifiers() & Qt::ControlModifier;
            const bool shift = event->modifiers() & Qt::ShiftModifier;
            viewmodel::CaretMove move = viewmodel::CaretMove::Left;
            if (event->key() == Qt::Key_Right) {
                move = viewmodel::CaretMove::Right;
            } else if (event->key() == Qt::Key_Home) {
                move = ctrl ? viewmodel::CaretMove::TextStart : viewmodel::CaretMove::LineStart;
            } else if (event->key() == Qt::Key_End) {
                move = ctrl ? viewmodel::CaretMove::TextEnd : viewmodel::CaretMove::LineEnd;
            }
            textResult = tools_.moveTextCaret(move, shift);
        } else if (event->key() == Qt::Key_A && (event->modifiers() & Qt::ControlModifier)) {
            textResult = tools_.selectAllText();
        } else if (event->key() == Qt::Key_Return || event->key() == Qt::Key_Enter) {
            // Enter is a new line; Ctrl+Enter places the text.
            textResult = (event->modifiers() & Qt::ControlModifier) ? tools_.key(viewmodel::ToolKey::Enter)
                                                                    : tools_.typeText("\n");
        } else if (!command && !event->text().isEmpty() && QChar::isPrint(event->text().toUcs4().front())) {
            textResult = tools_.typeText(event->text().toUtf8().toStdString());
        } else {
            handled = false;
        }
        if (handled) {
            event->accept();
            afterToolEvent(textResult);
            return;
        }
    }
    if (tools_.tool() == viewmodel::ToolKind::Dimension && event->key() == Qt::Key_Delete &&
        tools_.dimensionTool().pickedCount() == 0 && !tools_.dimensionTool().dragging()) {
        const viewmodel::DimensionToolResult r = tools_.dimensionTool().removeSelected();
        event->accept();
        if (!r.ok && onNodeMessage) {
            onNodeMessage(r.message);
        }
        viewmodel::ToolResult t;
        t.committed = r.committed;
        t.selectionChanged = r.committed;
        afterToolEvent(t);
        return;
    }
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
        rightPressPos_ = event->position();
        rightMoved_ = false;
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }
    if (event->button() == Qt::LeftButton && picking()) {
        pickClick(event->position());
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
        if (QLineF(pos, rightPressPos_).length() >= 4.0) {
            rightMoved_ = true;
        }
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
        if (tools_.tool() == viewmodel::ToolKind::NodeEdit) {
            updateNodeCursor(pos);
        } else {
            updateCursorHint();
        }
    }
    update();
    QWidget::mouseMoveEvent(event);
}

void CanvasWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (panning_ && (event->button() == Qt::MiddleButton || event->button() == Qt::RightButton)) {
        panning_ = false;
        unsetCursor();
        event->accept();
        if (event->button() == Qt::RightButton && !rightMoved_ && tools_.tool() == viewmodel::ToolKind::NodeEdit &&
            onNodeContextMenu) {
            tools_.setView(view_);
            const QPointF pos = event->position();
            onNodeContextMenu(event->globalPosition().toPoint(), Vec2{pos.x(), pos.y()});
        }
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
