#include "carvetto/viewmodel/view_transform.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::viewmodel {

using geometry::Box2;
using geometry::Vec2;

double clampScale(double scalePxPerMm) {
    if (!std::isfinite(scalePxPerMm)) {
        return scalePxPerMm > 0.0 ? ViewTransform::kMaxScale : ViewTransform::kMinScale;
    }
    return std::clamp(scalePxPerMm, ViewTransform::kMinScale, ViewTransform::kMaxScale);
}

void ViewTransform::setViewport(double widthPx, double heightPx) {
    width_ = std::max(0.0, widthPx);
    height_ = std::max(0.0, heightPx);
}

void ViewTransform::set(double scalePxPerMm, Vec2 originPx) {
    scale_ = clampScale(scalePxPerMm);
    origin_ = originPx;
}

Box2 ViewTransform::worldToScreen(const Box2& mm) const {
    if (mm.isEmpty()) {
        return Box2{};
    }
    return Box2{worldToScreen(mm.min), worldToScreen(mm.max)};
}

Box2 ViewTransform::visibleWorld() const {
    return Box2{screenToWorld({0.0, 0.0}), screenToWorld({width_, height_})};
}

void ViewTransform::zoomAbout(Vec2 screenPx, double factor) {
    if (!(factor > 0.0) || !std::isfinite(factor)) {
        return;
    }
    const Vec2 anchor = screenToWorld(screenPx);
    scale_ = clampScale(scale_ * factor);
    origin_ = {screenPx.x - anchor.x * scale_, screenPx.y + anchor.y * scale_};
}

void ViewTransform::pan(Vec2 deltaPx) {
    origin_ += deltaPx;
}

bool ViewTransform::zoomToFit(const Box2& worldBox, double marginFraction) {
    if (worldBox.isEmpty() || width_ <= 0.0 || height_ <= 0.0) {
        return false;
    }
    const double margin = std::clamp(marginFraction, 0.0, 0.45);
    const double availableW = width_ * (1.0 - 2.0 * margin);
    const double availableH = height_ * (1.0 - 2.0 * margin);
    const double bw = worldBox.width();
    const double bh = worldBox.height();
    double fit = 0.0;
    if (bw > 0.0 && bh > 0.0) {
        fit = std::min(availableW / bw, availableH / bh);
    } else if (bw > 0.0) {
        fit = availableW / bw;
    } else if (bh > 0.0) {
        fit = availableH / bh;
    } else {
        return false;
    }
    scale_ = clampScale(fit);
    const Vec2 c = worldBox.centre();
    origin_ = {width_ * 0.5 - c.x * scale_, height_ * 0.5 + c.y * scale_};
    return true;
}

}  // namespace carvetto::viewmodel
