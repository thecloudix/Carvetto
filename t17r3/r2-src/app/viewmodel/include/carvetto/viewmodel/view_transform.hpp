#pragma once

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::viewmodel {

/// Mapping between document millimetres (world, Y up) and widget pixels (screen, Y down) for
/// the 2D canvas. The map is a uniform scale plus a translation:
///
///     screen.x = origin.x + world.x * scale
///     screen.y = origin.y - world.y * scale
///
/// where `scale` is pixels per millimetre and `origin` is the screen position of world (0,0).
/// Zooming keeps `scale` within [`kMinScale`, `kMaxScale`].
class ViewTransform {
   public:
    /// Smallest zoom, in pixels per millimetre.
    static constexpr double kMinScale = 0.01;
    /// Largest zoom, in pixels per millimetre.
    static constexpr double kMaxScale = 1000.0;

    ViewTransform() = default;

    /// Sets the widget size in pixels (negative values are treated as zero).
    void setViewport(double widthPx, double heightPx);
    double viewportWidth() const { return width_; }
    double viewportHeight() const { return height_; }

    /// Pixels per millimetre.
    double scale() const { return scale_; }
    /// Screen position of world (0,0).
    geometry::Vec2 origin() const { return origin_; }
    /// Sets scale (clamped) and origin directly.
    void set(double scalePxPerMm, geometry::Vec2 originPx);

    /// World millimetres → screen pixels.
    geometry::Vec2 worldToScreen(geometry::Vec2 mm) const {
        return {origin_.x + mm.x * scale_, origin_.y - mm.y * scale_};
    }
    /// Screen pixels → world millimetres.
    geometry::Vec2 screenToWorld(geometry::Vec2 px) const {
        return {(px.x - origin_.x) / scale_, (origin_.y - px.y) / scale_};
    }
    /// Screen rectangle covered by a world box (empty in, empty out).
    geometry::Box2 worldToScreen(const geometry::Box2& mm) const;
    /// World rectangle currently visible in the viewport.
    geometry::Box2 visibleWorld() const;

    /// Multiplies the scale by `factor` (clamped) keeping the world point under `screenPx` fixed.
    void zoomAbout(geometry::Vec2 screenPx, double factor);
    /// Moves the view by a screen-space delta (content follows the mouse).
    void pan(geometry::Vec2 deltaPx);
    /// Centres `worldBox` and scales it to fill the viewport minus `marginFraction` of the
    /// viewport size on *each* side (0.05 → the box spans 90% of the limiting axis). Returns
    /// false (unchanged) for an empty box, a degenerate box or a zero viewport.
    bool zoomToFit(const geometry::Box2& worldBox, double marginFraction = 0.05);

   private:
    double width_{0.0};
    double height_{0.0};
    double scale_{1.0};
    geometry::Vec2 origin_{};
};

/// Clamps a zoom value to [`ViewTransform::kMinScale`, `ViewTransform::kMaxScale`].
double clampScale(double scalePxPerMm);

}  // namespace carvetto::viewmodel
