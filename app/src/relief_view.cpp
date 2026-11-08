#include "relief_view.hpp"

#include <QFontMetrics>
#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <algorithm>

namespace carvetto::app {

namespace {

constexpr int kMargin = 10;
constexpr int kStatusHeight = 24;

}  // namespace

ReliefView::ReliefView(QWidget* parent) : QWidget(parent) {
    setMinimumSize(200, 150);
    setAttribute(Qt::WA_OpaquePaintEvent, true);
    emptyText_ = QStringLiteral("Simulation view: run Simulate in the Preview panel to see the cut stock.");
}

void ReliefView::setFrame(std::shared_ptr<const viewmodel::SimulationFrame> frame, const geometry::Box2& stockMm) {
    frame_ = std::move(frame);
    stock_ = stockMm;
    image_ = QImage();
    if (frame_ && frame_->relief && frame_->relief->width > 0 && frame_->relief->height > 0) {
        const viewmodel::Raster& r = *frame_->relief;
        // Wrap the shared pixels (no copy); the frame stays alive as long as the image does.
        auto* keep = new std::shared_ptr<const viewmodel::SimulationFrame>(frame_);
        image_ = QImage(reinterpret_cast<const uchar*>(r.pixels.data()), static_cast<int>(r.width),
                        static_cast<int>(r.height), static_cast<qsizetype>(r.width * sizeof(std::uint32_t)),
                        QImage::Format_ARGB32,
                        [](void* info) { delete static_cast<std::shared_ptr<const viewmodel::SimulationFrame>*>(info); },
                        keep);
    }
    update();
}

void ReliefView::setStatus(const QString& text) {
    if (status_ != text) {
        status_ = text;
        update();
    }
}

void ReliefView::setBanner(const QString& text) {
    if (banner_ != text) {
        banner_ = text;
        update();
    }
}

void ReliefView::setTool(std::optional<geometry::Vec2> posMm, double diameterMm) {
    tool_ = posMm;
    toolDiameter_ = diameterMm;
    update();
}

void ReliefView::setEmptyText(const QString& text) {
    emptyText_ = text;
    update();
}

QRectF ReliefView::pictureRect() const {
    const QRectF area(kMargin, kMargin, width() - 2 * kMargin, height() - 2 * kMargin - kStatusHeight);
    if (image_.isNull() || area.width() < 2 || area.height() < 2) {
        return area;
    }
    const double scale = std::min(area.width() / image_.width(), area.height() / image_.height());
    const double w = image_.width() * scale;
    const double h = image_.height() * scale;
    return QRectF(area.left() + (area.width() - w) / 2.0, area.top() + (area.height() - h) / 2.0, w, h);
}

QColor ReliefView::pixelAtStock(geometry::Vec2 mm) const {
    if (image_.isNull() || stock_.isEmpty() || !stock_.contains(mm)) {
        return QColor();
    }
    const double fx = (mm.x - stock_.min.x) / std::max(stock_.width(), 1e-9);
    const double fy = 1.0 - (mm.y - stock_.min.y) / std::max(stock_.height(), 1e-9);
    const int x = std::clamp(static_cast<int>(fx * image_.width()), 0, image_.width() - 1);
    const int y = std::clamp(static_cast<int>(fy * image_.height()), 0, image_.height() - 1);
    return QColor::fromRgb(image_.pixel(x, y));
}

void ReliefView::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter p(this);
    p.fillRect(rect(), QColor(226, 228, 232));
    const QRectF pic = pictureRect();
    if (image_.isNull()) {
        p.setPen(QColor(90, 90, 90));
        p.drawText(pic, Qt::AlignCenter | Qt::TextWordWrap, emptyText_);
    } else {
        p.setRenderHint(QPainter::SmoothPixmapTransform, true);
        p.drawImage(pic, image_);
        p.setPen(QPen(QColor(120, 120, 120), 1.0));
        p.setBrush(Qt::NoBrush);
        p.drawRect(pic);
        if (tool_.has_value() && !stock_.isEmpty() && stock_.contains(*tool_)) {
            const double fx = (tool_->x - stock_.min.x) / std::max(stock_.width(), 1e-9);
            const double fy = 1.0 - (tool_->y - stock_.min.y) / std::max(stock_.height(), 1e-9);
            const QPointF c(pic.left() + fx * pic.width(), pic.top() + fy * pic.height());
            const double r = std::max(4.0, 0.5 * toolDiameter_ * pic.width() / std::max(stock_.width(), 1e-9));
            p.setRenderHint(QPainter::Antialiasing, true);
            p.setPen(QPen(QColor(20, 20, 20), 1.5));
            p.setBrush(QBrush(QColor(255, 220, 0, 110)));
            p.drawEllipse(c, r, r);
            p.drawLine(QPointF(c.x() - r - 3, c.y()), QPointF(c.x() + r + 3, c.y()));
            p.drawLine(QPointF(c.x(), c.y() - r - 3), QPointF(c.x(), c.y() + r + 3));
        }
        if (!banner_.isEmpty()) {
            const QRectF bar(pic.left(), pic.top(), pic.width(), 24);
            p.setRenderHint(QPainter::Antialiasing, false);
            p.fillRect(bar, QColor(255, 170, 40, 230));
            p.setPen(QColor(40, 20, 0));
            p.drawText(bar.adjusted(6, 0, -6, 0), Qt::AlignVCenter | Qt::AlignLeft,
                       QFontMetrics(font()).elidedText(banner_, Qt::ElideRight, static_cast<int>(bar.width()) - 12));
        }
    }
    p.setPen(QColor(50, 50, 50));
    p.drawText(QRect(kMargin, height() - kStatusHeight - 2, width() - 2 * kMargin, kStatusHeight),
               Qt::AlignVCenter | Qt::AlignLeft, status_);
}

}  // namespace carvetto::app
