#pragma once

#include <QImage>
#include <QString>
#include <QWidget>
#include <memory>
#include <optional>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/simulation_controller.hpp"

namespace carvetto::app {

/// The simulation view (T-0223), the content of the "3D View" tab: the shaded top-down relief of the
/// cut stock from the newest simulation frame, the tool position, a one-line status and a banner
/// when the result no longer matches the document. Only paints; the picture is the Qt-free
/// `viewmodel::Raster` of the frame (wrapped, not copied).
class ReliefView : public QWidget {
   public:
    explicit ReliefView(QWidget* parent = nullptr);

    /// Shows `frame` (null: the empty-state text). `stockMm` is the stock rectangle the raster
    /// covers, used to place the tool marker.
    void setFrame(std::shared_ptr<const viewmodel::SimulationFrame> frame, const geometry::Box2& stockMm);
    /// Line under the picture ("Move 12 / 340 - 0:41 / 3:20 - 2 warnings").
    void setStatus(const QString& text);
    /// Orange bar over the picture ("Out of date: ..."); empty hides it.
    void setBanner(const QString& text);
    /// Tool marker in mm and its diameter; unset hides it.
    void setTool(std::optional<geometry::Vec2> posMm, double diameterMm);
    /// Text shown when there is no frame.
    void setEmptyText(const QString& text);

    const QImage& image() const { return image_; }
    const QString& banner() const { return banner_; }
    const QString& status() const { return status_; }
    bool hasPicture() const { return !image_.isNull(); }
    /// Colour of the picture pixel under the stock position `mm` (self-test); invalid when outside.
    QColor pixelAtStock(geometry::Vec2 mm) const;

   protected:
    void paintEvent(QPaintEvent* event) override;

   private:
    QRectF pictureRect() const;

    std::shared_ptr<const viewmodel::SimulationFrame> frame_;
    QImage image_;
    geometry::Box2 stock_;
    QString status_;
    QString banner_;
    QString emptyText_;
    std::optional<geometry::Vec2> tool_;
    double toolDiameter_{0.0};
};

}  // namespace carvetto::app
