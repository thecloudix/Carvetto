#pragma once

#include <QImage>
#include <QString>
#include <QWidget>
#include <memory>
#include <optional>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/cylinder_map.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/model_view.hpp"
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

    /// The Auto-Wrapping View (T-0334): the cut stock of a rotary job drawn round the cylinder `map` from the
    /// frame's wrapped surface, orbitable with the mouse (left-drag orbits, the wheel zooms). Only frames that
    /// carry a wrapped surface (rotary jobs) can show it; others keep the flat picture.
    void setWrapped(bool on);
    void setCylinder(std::optional<geometry::CylinderMap> map);
    bool wrappedRequested() const { return wrapped_; }
    /// True when the picture on screen is the wrapped one.
    bool showingWrapped() const { return wrapped_ && frame_ && frame_->wrapped && cylinder_.has_value(); }
    viewmodel::OrbitCamera& camera() { return camera_; }
    void cameraChanged();
    /// The wrapped picture rendered now at the widget's picture size (null image when not showing it).
    QImage wrappedImage();

    const QImage& image() const { return image_; }
    const QString& banner() const { return banner_; }
    const QString& status() const { return status_; }
    const std::shared_ptr<const viewmodel::SimulationFrame>& frame() const { return frame_; }
    bool hasPicture() const { return !image_.isNull(); }
    /// Colour of the picture pixel under the stock position `mm` (self-test); invalid when outside.
    QColor pixelAtStock(geometry::Vec2 mm) const;

   protected:
    void paintEvent(QPaintEvent* event) override;
    void resizeEvent(QResizeEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

   private:
    QRectF pictureRect() const;
    void rebuildImage();

    std::shared_ptr<const viewmodel::SimulationFrame> frame_;
    QImage image_;
    geometry::Box2 stock_;
    QString status_;
    QString banner_;
    QString emptyText_;
    std::optional<geometry::Vec2> tool_;
    double toolDiameter_{0.0};
    bool wrapped_{true};
    std::optional<geometry::CylinderMap> cylinder_;
    viewmodel::OrbitCamera camera_;
    QPoint last_;
};

}  // namespace carvetto::app
