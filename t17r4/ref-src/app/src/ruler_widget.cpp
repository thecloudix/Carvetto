#include "ruler_widget.hpp"

#include <QColor>
#include <QFont>
#include <QPaintEvent>
#include <QPainter>
#include <QPen>
#include <QPointF>
#include <QRectF>
#include <QString>

#include "canvas_widget.hpp"
#include "carvetto/viewmodel/ruler.hpp"

namespace carvetto::app {

RulerWidget::RulerWidget(Qt::Orientation orientation, const CanvasWidget* canvas, QWidget* parent)
    : QWidget(parent), orientation_(orientation), canvas_(canvas) {
    if (orientation_ == Qt::Horizontal) {
        setFixedHeight(kThickness);
    } else {
        setFixedWidth(kThickness);
    }
}

void RulerWidget::setUnits(document::Units units) {
    units_ = units;
    update();
}

QSize RulerWidget::sizeHint() const {
    return orientation_ == Qt::Horizontal ? QSize(400, kThickness) : QSize(kThickness, 300);
}

void RulerWidget::paintEvent(QPaintEvent* event) {
    Q_UNUSED(event);
    QPainter painter(this);
    painter.fillRect(rect(), QColor(245, 245, 245));
    if (canvas_ == nullptr) {
        return;
    }
    const bool horizontal = orientation_ == Qt::Horizontal;
    const viewmodel::RulerModel model = viewmodel::rulerTicks(
        canvas_->view(),
        horizontal ? viewmodel::RulerOrientation::Horizontal : viewmodel::RulerOrientation::Vertical, units_);

    QFont font = painter.font();
    font.setPointSizeF(7.5);
    painter.setFont(font);
    QPen pen(QColor(70, 70, 70));
    pen.setWidthF(1.0);
    pen.setCosmetic(true);
    painter.setPen(pen);

    const double t = static_cast<double>(kThickness);
    for (const viewmodel::RulerTick& tick : model.ticks) {
        const double len = tick.major ? t * 0.6 : t * 0.25;
        const double p = tick.positionPx + 0.5;
        if (horizontal) {
            painter.drawLine(QPointF(p, t), QPointF(p, t - len));
            if (tick.major) {
                painter.drawText(QPointF(p + 2.0, 9.0), QString::fromStdString(tick.label));
            }
        } else {
            painter.drawLine(QPointF(t, p), QPointF(t - len, p));
            if (tick.major) {
                painter.save();
                painter.translate(9.0, p - 2.0);
                painter.rotate(-90.0);
                painter.drawText(QPointF(0.0, 0.0), QString::fromStdString(tick.label));
                painter.restore();
            }
        }
    }
    // Edge line against the canvas.
    if (horizontal) {
        painter.drawLine(QPointF(0.0, t - 0.5), QPointF(static_cast<double>(width()), t - 0.5));
    } else {
        painter.drawLine(QPointF(t - 0.5, 0.0), QPointF(t - 0.5, static_cast<double>(height())));
    }
}

}  // namespace carvetto::app
