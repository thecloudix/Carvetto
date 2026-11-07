#include "carvetto/viewmodel/transform_gesture.hpp"

#include <cmath>
#include <memory>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/viewmodel/transform_ops.hpp"

namespace carvetto::viewmodel {

using geometry::Box2;
using geometry::Transform2;
using geometry::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool isCorner(HandleKind h) {
    return h == HandleKind::TopLeft || h == HandleKind::TopRight || h == HandleKind::BottomLeft ||
           h == HandleKind::BottomRight;
}

HandleKind opposite(HandleKind h) {
    return static_cast<HandleKind>(8 - static_cast<int>(h));
}

double wrapPi(double a) {
    while (a > kPi) {
        a -= 2.0 * kPi;
    }
    while (a <= -kPi) {
        a += 2.0 * kPi;
    }
    return a;
}

bool inRange(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::fabs(p.x) <= kMaxTransformCoordinateMm &&
           std::fabs(p.y) <= kMaxTransformCoordinateMm;
}

}  // namespace

bool TransformGesture::begin(document::Document& doc, GestureKind kind, HandleKind handle, Vec2 startMm,
                             Vec2 pivotMm) {
    kind_ = GestureKind::None;
    targets_ = transformTargets(doc);
    const std::optional<Box2> b = transformBounds(doc);
    if (targets_.empty() || !b.has_value() || kind == GestureKind::None) {
        return false;
    }
    if (kind == GestureKind::Rotate && (!inRange(pivotMm) || geometry::distance(startMm, pivotMm) < 1e-9)) {
        return false;
    }
    bounds_ = *b;
    handle_ = handle;
    start_ = startMm;
    pivot_ = pivotMm;
    applied_ = Transform2::identity();
    pushed_ = false;
    totalAngle_ = 0.0;
    lastAngle_ = std::atan2(startMm.y - pivotMm.y, startMm.x - pivotMm.x);
    generation_ = doc.undoStack().generation();
    kind_ = kind;
    return true;
}

Transform2 TransformGesture::absoluteFor(Vec2 p, bool shift, bool alt, bool* valid) {
    *valid = true;
    switch (kind_) {
        case GestureKind::Move: {
            Vec2 d = p - start_;
            if (alt) {
                if (std::fabs(d.x) >= std::fabs(d.y)) {
                    d.y = 0.0;
                } else {
                    d.x = 0.0;
                }
            }
            return Transform2::translation(d);
        }
        case GestureKind::Rotate: {
            if (geometry::distance(p, pivot_) < 1e-9) {
                *valid = false;
                return applied_;
            }
            const double a = std::atan2(p.y - pivot_.y, p.x - pivot_.x);
            totalAngle_ += wrapPi(a - lastAngle_);
            lastAngle_ = a;
            double angle = totalAngle_;
            if (alt) {
                angle = std::round(angle / kRotateStepRad) * kRotateStepRad;
            }
            return Transform2::rotation(angle, pivot_);
        }
        case GestureKind::Scale: {
            const Vec2 grabbed = anchorPoint(bounds_, handle_);
            Vec2 anchor = anchorPoint(bounds_, opposite(handle_));
            double fx = 1.0;
            double fy = 1.0;
            if (isCorner(handle_)) {
                if (shift) {
                    anchor = bounds_.centre();
                }
                const Vec2 ref = grabbed - anchor;
                const Vec2 cur = p - anchor;
                if (alt) {
                    fx = std::fabs(ref.x) > 1e-12 ? cur.x / ref.x : 1.0;
                    fy = std::fabs(ref.y) > 1e-12 ? cur.y / ref.y : 1.0;
                } else {
                    const double len2 = ref.lengthSquared();
                    if (len2 < 1e-24) {
                        *valid = false;
                        return applied_;
                    }
                    fx = fy = cur.dot(ref) / len2;
                }
            } else {
                const bool horizontal = handle_ == HandleKind::Left || handle_ == HandleKind::Right;
                if (horizontal) {
                    anchor.y = bounds_.centre().y;
                    const double ref = grabbed.x - anchor.x;
                    fx = std::fabs(ref) > 1e-12 ? (p.x - anchor.x) / ref : 1.0;
                    fy = shift ? fx : 1.0;
                } else {
                    anchor.x = bounds_.centre().x;
                    const double ref = grabbed.y - anchor.y;
                    fy = std::fabs(ref) > 1e-12 ? (p.y - anchor.y) / ref : 1.0;
                    fx = shift ? fy : 1.0;
                }
            }
            if (!(fx >= kMinScaleFactor) || !(fy >= kMinScaleFactor) || fx > kMaxScaleFactor || fy > kMaxScaleFactor) {
                *valid = false;
                return applied_;
            }
            return Transform2::scaling(fx, fy, anchor);
        }
        case GestureKind::None:
            break;
    }
    *valid = false;
    return applied_;
}

bool TransformGesture::update(document::Document& doc, Vec2 pointMm, bool shift, bool ctrl, bool alt) {
    (void)ctrl;
    if (!active()) {
        return false;
    }
    if (pushed_ && doc.undoStack().generation() != generation_) {
        kind_ = GestureKind::None;  // something else edited the document: the gesture is over
        return false;
    }
    if (!inRange(pointMm)) {
        return false;
    }
    bool valid = false;
    const Transform2 target = absoluteFor(pointMm, shift, alt, &valid);
    if (!valid) {
        return false;
    }
    const std::optional<Transform2> inv = applied_.inverse();
    if (!inv.has_value()) {
        return false;
    }
    // Keep the corners of the original box in range under the new absolute map.
    const Vec2 corners[4] = {bounds_.min, {bounds_.max.x, bounds_.min.y}, bounds_.max, {bounds_.min.x, bounds_.max.y}};
    for (const Vec2& c : corners) {
        if (!inRange(target.apply(c))) {
            return false;
        }
    }
    const Transform2 delta = target * *inv;
    if (delta == Transform2::identity()) {
        return false;
    }
    if (!pushed_) {
        doc.undoStack().breakCoalescing();
    }
    if (!doc.execute(std::make_unique<document::TransformObjectsCommand>(targets_, delta))) {
        return false;
    }
    pushed_ = true;
    applied_ = target;
    generation_ = doc.undoStack().generation();
    return true;
}

void TransformGesture::end(document::Document& doc) {
    if (pushed_) {
        doc.undoStack().breakCoalescing();
    }
    kind_ = GestureKind::None;
    pushed_ = false;
}

bool TransformGesture::cancel(document::Document& doc) {
    bool reverted = false;
    if (active() && pushed_ && doc.undoStack().generation() == generation_) {
        reverted = doc.undoStack().undoAndDiscard(doc);
    }
    kind_ = GestureKind::None;
    pushed_ = false;
    return reverted;
}

}  // namespace carvetto::viewmodel
