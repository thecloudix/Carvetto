#pragma once

#include <QSize>
#include <QWidget>

#include "carvetto/document/units.hpp"

class QPaintEvent;

namespace carvetto::app {

class CanvasWidget;

/// Ruler along the top or left edge of the 2D canvas. Tick positions, spacing and labels come
/// from `viewmodel::rulerTicks`; this widget only draws them.
class RulerWidget : public QWidget {
   public:
    RulerWidget(Qt::Orientation orientation, const CanvasWidget* canvas, QWidget* parent = nullptr);

    /// Display unit for the labels.
    void setUnits(document::Units units);
    QSize sizeHint() const override;

    /// Ruler thickness in pixels.
    static constexpr int kThickness = 24;

   protected:
    void paintEvent(QPaintEvent* event) override;

   private:
    Qt::Orientation orientation_;
    const CanvasWidget* canvas_;
    document::Units units_{document::Units::Millimetres};
};

}  // namespace carvetto::app
