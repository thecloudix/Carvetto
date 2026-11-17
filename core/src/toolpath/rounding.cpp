#include "carvetto/toolpath/rounding.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <numbers>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/geometry/vec3.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Vec3;

constexpr double kPi = std::numbers::pi_v<double>;
constexpr double kMaxLength = 100000.0;   // radii, lengths and Z (mm)
constexpr double kMinStepover = 1e-3;
constexpr double kMaxPasses = 1000.0;
constexpr double kMaxStrokes = 200000.0;  // over all passes
constexpr double kEps = 1e-9;
/// The corner stretches of a square blank this narrow (radians) are not material.
constexpr double kMinCornerHalfWidth = 1e-9;

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

/// Half the side of the square bar inscribed in a cylinder of `radius`.
double squareHalfSide(double radius) { return radius / std::numbers::sqrt2; }

/// Radius of the blank in direction `theta`.
double extentAt(RoundingBlank blank, double radius, double theta) {
    if (blank == RoundingBlank::Round) {
        return radius;
    }
    const double quarter = 0.5 * kPi;
    const double phi = theta - quarter * std::round(theta / quarter);  // from the nearest face normal
    return squareHalfSide(radius) / std::cos(phi);
}

std::string check(const tooling::Tool& tool, const RoundingParams& p) {
    const double values[] = {p.blankRadius, p.targetRadius, p.startRadius, p.allowance, p.surfaceZ, p.alongStart,
                             p.alongEnd,    p.circStart,    p.stepover,    p.passDepth, p.safeZ};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (!roundingToolAllowed(tool.type)) {
        return "tool '" + tool.name + "' must be an end mill or a ball nose tool for rounding";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        return "tool '" + tool.name + "': " + *bad;
    }
    if (!(p.blankRadius > 0.0) || p.blankRadius > kMaxLength) {
        return "the blank radius must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (!(p.targetRadius > 0.0) || p.targetRadius > kMaxLength) {
        return "the radius to turn to must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (p.startRadius < 0.0 || p.startRadius > kMaxLength) {
        return "the starting radius must be 0 (none) or up to " + num(kMaxLength) + " mm";
    }
    if (p.allowance < 0.0 || p.allowance > kMaxLength) {
        return "the allowance must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (!(p.alongEnd > p.alongStart) || std::fabs(p.alongStart) > kMaxLength || std::fabs(p.alongEnd) > kMaxLength ||
        std::fabs(p.circStart) > kMaxLength) {
        return "the extent along the axis must run from a start to a larger end, within " + num(kMaxLength) + " mm";
    }
    if (p.stepover < 0.0 || p.stepover > kMaxLength || p.passDepth < 0.0 || p.passDepth > kMaxLength) {
        return "the stepover and pass depth must be 0 (the tool's) or positive";
    }
    if (std::fabs(p.surfaceZ) > kMaxLength || std::fabs(p.safeZ) > kMaxLength) {
        return "a Z value is beyond " + num(kMaxLength) + " mm";
    }
    return {};
}

/// A stroke of one pass: from -> to in (along, circumference), `joins`: the tool stays down from the previous.
struct Stroke {
    double fromAlong{0.0};
    double fromCirc{0.0};
    double toAlong{0.0};
    double toCirc{0.0};
    bool joins{false};
};

}  // namespace

bool roundingToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::EndMill || type == tooling::ToolType::BallNose;
}

std::vector<double> roundingPassRadii(double top, double bottom, double passDepth) {
    std::vector<double> out;
    if (!(top > bottom) || !(passDepth > 0.0) || !std::isfinite(top) || !std::isfinite(bottom)) {
        return out;
    }
    const double n = std::max(1.0, std::ceil((top - bottom) / passDepth - kEps));
    if (!(n <= kMaxPasses)) {
        return out;
    }
    const auto count = static_cast<std::size_t>(n);
    out.reserve(count);
    for (std::size_t k = 1; k < count; ++k) {
        out.push_back(top - (top - bottom) * static_cast<double>(k) / n);
    }
    out.push_back(bottom);
    return out;
}

std::vector<std::pair<double, double>> roundingMaterial(RoundingBlank blank, double radius, double rho, double c0) {
    std::vector<std::pair<double, double>> out;
    const double turn = 2.0 * kPi * radius;
    if (!(rho < radius)) {
        return out;
    }
    if (blank == RoundingBlank::Round || rho <= squareHalfSide(radius)) {
        out.emplace_back(c0, c0 + turn);
        return out;
    }
    const double halfWidth = 0.25 * kPi - std::acos(squareHalfSide(radius) / rho);  // round each corner, radians
    if (!(halfWidth > kMinCornerHalfWidth)) {
        return out;
    }
    const double reach = halfWidth * radius;
    // Corners at angles pi/4 + q pi/2, each repeating every turn; keep the stretches inside [c0, c0 + turn].
    const double first = std::floor(c0 / turn) - 1.0;
    for (double m = first; m <= first + 3.0; m += 1.0) {
        for (int q = 0; q < 4; ++q) {
            const double centre = (0.25 * kPi + 0.5 * kPi * q) * radius + m * turn;
            const double lo = std::max(c0, centre - reach);
            const double hi = std::min(c0 + turn, centre + reach);
            if (hi > lo) {
                out.emplace_back(lo, hi);
            }
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

RoundingResult roundingToolpath(const tooling::Tool& tool, const std::optional<tooling::CuttingData>& cuttingData,
                                const RoundingParams& params, const ToolpathRunControl& control) {
    RoundingResult result;
    result.toolpath.name = params.name;
    const auto say = [&result](ToolpathIssue issue, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, std::nullopt, std::move(message), std::nullopt});
    };
    if (const std::string bad = check(tool, params); !bad.empty()) {
        say(roundingToolAllowed(tool.type) ? ToolpathIssue::InvalidParameters : ToolpathIssue::InvalidTool, bad);
        return result;
    }
    if (control.cancel != nullptr && control.cancel->load()) {
        say(ToolpathIssue::Cancelled, "the calculation was cancelled");
        return result;
    }
    const double radius = params.blankRadius;
    const double top = params.startRadius > 0.0 ? std::min(radius, params.startRadius) : radius;
    const double bottom = params.targetRadius + params.allowance;
    if (!(top > bottom)) {
        say(ToolpathIssue::NoRestMaterial, "nothing to remove: the blank is already at or below the finished radius");
        return result;
    }
    const double axisZ = params.surfaceZ - radius;
    if (!(params.safeZ > axisZ + radius)) {
        say(ToolpathIssue::InvalidParameters, "the safe Z must be above the surface of the cylinder");
        return result;
    }
    const PassTool pt = passTool(tool, effectiveCuttingData(tool, cuttingData), top - bottom);
    result.toolpath.tool = pt.snapshot;
    const double passDepth = params.passDepth > 0.0 ? params.passDepth : pt.passDepth;
    double stepover = params.stepover > 0.0 ? params.stepover : pt.stepover;
    if (!std::isfinite(pt.snapshot.plungeRate) || !(pt.snapshot.plungeRate > 0.0) ||
        !std::isfinite(pt.snapshot.feedRate) || !(pt.snapshot.feedRate > 0.0) || !std::isfinite(passDepth) ||
        !(passDepth > 0.0) || !std::isfinite(stepover) || !(stepover > 0.0)) {
        say(ToolpathIssue::InvalidTool,
            "tool '" + tool.name + "': the feed rate, plunge rate, pass depth and stepover must be greater than zero");
        return result;
    }
    if (stepover > tool.diameter) {
        stepover = tool.diameter;
        say(ToolpathIssue::StepoverLimited, "the stepover is wider than the tool; limited to its diameter");
    }
    if (!(stepover >= kMinStepover)) {
        say(ToolpathIssue::WorkLimitExceeded, "the stepover is too small (at least " + num(kMinStepover) + " mm)");
        return result;
    }
    if (std::ceil((top - bottom) / passDepth - kEps) > kMaxPasses) {
        say(ToolpathIssue::WorkLimitExceeded,
            "more than " + num(kMaxPasses) + " passes (" + num(top - bottom) + " mm deep, " + num(passDepth) +
                " mm per pass)");
        return result;
    }
    const std::vector<double> radii = roundingPassRadii(top, bottom, passDepth);

    const double turn = 2.0 * kPi * radius;
    const double length = params.alongEnd - params.alongStart;
    const bool radial = params.method == RoundingMethod::Radial;
    // Strokes per pass: radial = strokes round the circumference; raster = turns along the axis (each of up to
    // four stretches for a square blank).
    const double lines = std::max(1.0, std::ceil((radial ? turn : length) / stepover - kEps));
    const double perPass = radial ? lines : (lines + 1.0) * (params.blank == RoundingBlank::Square ? 4.0 : 1.0);
    if (!(perPass * static_cast<double>(radii.size()) <= kMaxStrokes)) {
        say(ToolpathIssue::WorkLimitExceeded, "more than " + num(kMaxStrokes) + " strokes (stepover " + num(stepover) +
                                                   " mm over " + std::to_string(radii.size()) + " passes)");
        return result;
    }
    const auto count = static_cast<std::size_t>(lines);

    const auto point = [&](double along, double circ, double z) {
        return params.alongX ? Vec3{along, circ, z} : Vec3{circ, along, z};
    };
    const auto alongAt = [&](std::size_t j) {
        return j >= count ? params.alongEnd
                          : params.alongStart + length * static_cast<double>(j) / static_cast<double>(count);
    };

    std::vector<Move>& moves = result.toolpath.moves;
    bool down = false;
    Vec3 at{};
    for (const double rho : radii) {
        if (control.cancel != nullptr && control.cancel->load()) {
            result.toolpath.moves.clear();
            result.strokes = 0;
            result.radii.clear();
            say(ToolpathIssue::Cancelled, "the calculation was cancelled");
            return result;
        }
        const double z = axisZ + rho;
        std::vector<Stroke> strokes;
        if (radial) {
            bool previous = false;
            for (std::size_t j = 0; j < count; ++j) {
                const double circ = params.circStart + turn * static_cast<double>(j) / static_cast<double>(count);
                const bool present = extentAt(params.blank, radius, circ / radius) > rho + kEps;
                if (present) {
                    const bool forward = j % 2 == 0;
                    strokes.push_back({forward ? params.alongStart : params.alongEnd, circ,
                                       forward ? params.alongEnd : params.alongStart, circ, previous});
                }
                previous = present;
            }
        } else {
            const auto stretches = roundingMaterial(params.blank, radius, rho, params.circStart);
            for (std::size_t j = 0; j <= count; ++j) {
                const double along = alongAt(j);
                const bool reversed = params.method == RoundingMethod::OptimizedRaster && j % 2 == 1;
                for (std::size_t s = 0; s < stretches.size(); ++s) {
                    const auto& stretch = reversed ? stretches[stretches.size() - 1 - s] : stretches[s];
                    strokes.push_back({along, reversed ? stretch.second : stretch.first, along,
                                       reversed ? stretch.first : stretch.second,
                                       params.method == RoundingMethod::OptimizedRaster && j > 0 && s == 0});
                }
            }
        }
        for (const Stroke& s : strokes) {
            const Vec3 start = point(s.fromAlong, s.fromCirc, z);
            if (s.joins && down) {
                moves.push_back(Move::feed(start));
            } else {
                if (down) {
                    moves.push_back(Move::retract(Vec3{at.x, at.y, params.safeZ}));
                }
                moves.push_back(Move::rapid(Vec3{start.x, start.y, params.safeZ}));
                moves.push_back(Move::plunge(start));
            }
            at = point(s.toAlong, s.toCirc, z);
            moves.push_back(Move::feed(at));
            down = true;
            ++result.strokes;
        }
    }
    if (down) {
        moves.push_back(Move::retract(Vec3{at.x, at.y, params.safeZ}));
    }
    result.radii = radii;
    if (result.strokes == 0) {
        result.toolpath.moves.clear();
        say(ToolpathIssue::NoRestMaterial, "nothing to remove: no material is left above the finished radius");
    }
    return result;
}

}  // namespace carvetto::toolpath
