#include "carvetto/toolpath/move.hpp"

#include <cmath>

#include "carvetto/geometry/span.hpp"

namespace carvetto::toolpath {

namespace {
geometry::ArcSpan toArcSpan(const Move& m, Vec2 from) {
    const auto dir = m.kind == MoveKind::ArcCW ? geometry::ArcDirection::CW : geometry::ArcDirection::CCW;
    return geometry::ArcSpan::fromCentre(m.centre, from, m.to.xy(), dir);
}
}  // namespace

double arcRadius(const Move& m, Vec2 from) {
    return geometry::distance(m.centre, from);
}

double arcSweep(const Move& m, Vec2 from) {
    if (!isArc(m.kind)) {
        return 0.0;
    }
    return toArcSpan(m, from).sweepAngle();
}

double moveLength(const Move& m, Vec3 from) {
    switch (m.kind) {
        case MoveKind::Dwell:
            return 0.0;
        case MoveKind::ArcCW:
        case MoveKind::ArcCCW: {
            const double planar = toArcSpan(m, from.xy()).length();
            const double dz = m.to.z - from.z;
            return std::sqrt(planar * planar + dz * dz);
        }
        case MoveKind::Rapid:
        case MoveKind::Feed:
        case MoveKind::Plunge:
        case MoveKind::Retract:
            return geometry::distance(from, m.to);
    }
    return 0.0;
}

Box2 moveBounds(const Move& m, Vec3 from) {
    if (isArc(m.kind)) {
        return toArcSpan(m, from.xy()).bounds();
    }
    Box2 b;
    b.expand(from.xy());
    b.expand(m.to.xy());
    return b;
}

}  // namespace carvetto::toolpath
