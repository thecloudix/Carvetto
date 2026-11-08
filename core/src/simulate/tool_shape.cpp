#include "carvetto/simulate/tool_shape.hpp"

#include <cmath>

#include "det_math.hpp"

namespace carvetto::simulate {

double ProfilePiece::heightAt(double r) const {
    switch (kind) {
        case Kind::Flat:
            return base;
        case Kind::Cone:
            return base + slope * (r - r0);
        case Kind::Sphere: {
            const double rem = sphereRadius * sphereRadius - r * r;
            return base + sphereRadius - std::sqrt(rem > 0.0 ? rem : 0.0);
        }
    }
    return base;
}

double ToolShape::heightAt(double r) const {
    if (!(r <= radius_)) {
        return std::numeric_limits<double>::infinity();
    }
    for (std::size_t i = 0; i < count_; ++i) {
        if (r <= pieces_[i].r1 || i + 1 == count_) {
            return pieces_[i].heightAt(r);
        }
    }
    return 0.0;
}

double ToolShape::reachAtGap(double gap) const {
    double reach = 0.0;
    for (std::size_t i = 0; i < count_; ++i) {
        const ProfilePiece& p = pieces_[i];
        if (!(gap >= p.base)) {
            break;  // heights only grow with r: nothing further out is low enough
        }
        double r = p.r1;
        const double t = gap - p.base;
        if (p.kind == ProfilePiece::Kind::Cone) {
            const double rc = p.r0 + t / p.slope;
            r = rc < p.r1 ? rc : p.r1;
        } else if (p.kind == ProfilePiece::Kind::Sphere) {
            if (t < p.sphereRadius) {
                const double rs = std::sqrt(t * (2.0 * p.sphereRadius - t));
                r = rs < p.r1 ? rs : p.r1;
            }
        }
        reach = r;
        if (r < p.r1) {
            break;
        }
    }
    return reach;
}

ToolShape ToolShape::withFluteLength(double length) const {
    ToolShape s = *this;
    s.fluteLength_ = length;
    return s;
}

ToolShape ToolShape::shank() const {
    ToolShape s;
    s.kind_ = ShapeKind::FlatEndMill;
    s.count_ = 1;
    s.radius_ = radius_;
    s.pieces_[0] = ProfilePiece{ProfilePiece::Kind::Flat, 0.0, radius_, fluteLength_, 0.0, 0.0};
    s.fluteLength_ = fluteLength_;
    return s;
}

namespace {
bool goodDiameter(double d) {
    return std::isfinite(d) && d > 0.0 && d <= ToolShape::kMaxDiameter;
}
bool goodAngle(double a) {
    return std::isfinite(a) && a >= ToolShape::kMinIncludedAngle && a < detail::kPi - ToolShape::kMinIncludedAngle;
}
}  // namespace

std::optional<ToolShape> ToolShape::flatEndMill(double diameter) {
    if (!goodDiameter(diameter)) {
        return std::nullopt;
    }
    ToolShape s;
    s.kind_ = ShapeKind::FlatEndMill;
    s.radius_ = diameter * 0.5;
    s.count_ = 1;
    s.pieces_[0] = ProfilePiece{ProfilePiece::Kind::Flat, 0.0, s.radius_, 0.0, 0.0, 0.0};
    return s;
}

std::optional<ToolShape> ToolShape::ballNose(double diameter) {
    if (!goodDiameter(diameter)) {
        return std::nullopt;
    }
    ToolShape s;
    s.kind_ = ShapeKind::BallNose;
    s.radius_ = diameter * 0.5;
    s.count_ = 1;
    s.pieces_[0] = ProfilePiece{ProfilePiece::Kind::Sphere, 0.0, s.radius_, 0.0, 0.0, s.radius_};
    return s;
}

std::optional<ToolShape> ToolShape::vBit(double diameter, double includedAngle, double tipDiameter) {
    if (!goodDiameter(diameter) || !goodAngle(includedAngle) || !std::isfinite(tipDiameter) || tipDiameter < 0.0 ||
        !(tipDiameter < diameter)) {
        return std::nullopt;
    }
    double sn = 0.0;
    double cs = 1.0;
    detail::sinCosQuarter(includedAngle * 0.5, sn, cs);
    ToolShape s;
    s.kind_ = ShapeKind::VBit;
    s.radius_ = diameter * 0.5;
    const double tipR = tipDiameter * 0.5;
    if (tipR > 0.0) {
        s.pieces_[s.count_++] = ProfilePiece{ProfilePiece::Kind::Flat, 0.0, tipR, 0.0, 0.0, 0.0};
    }
    s.pieces_[s.count_++] = ProfilePiece{ProfilePiece::Kind::Cone, tipR, s.radius_, 0.0, cs / sn, 0.0};
    return s;
}

std::optional<ToolShape> ToolShape::taperedBallNose(double diameter, double includedAngle, double ballDiameter) {
    if (!goodDiameter(diameter) || !goodAngle(includedAngle) || !goodDiameter(ballDiameter) ||
        ballDiameter > diameter) {
        return std::nullopt;
    }
    double sn = 0.0;
    double cs = 1.0;
    detail::sinCosQuarter(includedAngle * 0.5, sn, cs);
    const double rb = ballDiameter * 0.5;
    ToolShape s;
    s.kind_ = ShapeKind::TaperedBallNose;
    s.radius_ = diameter * 0.5;
    // The cone is tangent to the ball at radius rb*cos(a), height rb*(1 - sin(a)) (ADR-0010).
    const double rt = rb * cs;
    const double ht = rb * (1.0 - sn);
    if (rt >= s.radius_) {
        s.pieces_[s.count_++] = ProfilePiece{ProfilePiece::Kind::Sphere, 0.0, s.radius_, 0.0, 0.0, rb};
    } else {
        s.pieces_[s.count_++] = ProfilePiece{ProfilePiece::Kind::Sphere, 0.0, rt, 0.0, 0.0, rb};
        s.pieces_[s.count_++] = ProfilePiece{ProfilePiece::Kind::Cone, rt, s.radius_, ht, cs / sn, 0.0};
    }
    return s;
}

std::optional<ToolShape> ToolShape::fromTool(const tooling::Tool& tool, std::optional<double> fluteLength) {
    if (tooling::validate(tool).has_value()) {
        return std::nullopt;
    }
    std::optional<ToolShape> s;
    switch (tool.type) {
        case tooling::ToolType::EndMill:
        case tooling::ToolType::DiamondDrag:
            s = flatEndMill(tool.diameter);
            break;
        case tooling::ToolType::BallNose:
            s = ballNose(tool.diameter);
            break;
        case tooling::ToolType::VBit:
        case tooling::ToolType::Drill:
            s = vBit(tool.diameter, tool.includedAngle);
            break;
        case tooling::ToolType::Engraving:
            s = vBit(tool.diameter, tool.includedAngle, tool.tipDiameter);
            break;
        case tooling::ToolType::TaperedBallNose:
            s = taperedBallNose(tool.diameter, tool.includedAngle, tool.tipDiameter);
            break;
        case tooling::ToolType::RadiusedEndMill:
        case tooling::ToolType::FormTool:
            return std::nullopt;
    }
    if (s && fluteLength) {
        if (!(*fluteLength > 0.0)) {
            return std::nullopt;
        }
        *s = s->withFluteLength(*fluteLength);
    }
    return s;
}

}  // namespace carvetto::simulate
