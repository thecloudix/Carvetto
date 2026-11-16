#include "carvetto/toolpath/laser_picture.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <utility>

#include "carvetto/geometry/cancel.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/move.hpp"
#include "laser_raster.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::Vec2;
using namespace laser_detail;

constexpr double kMinChordal = 1e-4;
constexpr double kMaxLength = 100000.0;
constexpr double kMaxOffset = 1000.0;
constexpr int kMaxPasses = 1000;
constexpr double kMaxMoves = 20000000.0;
constexpr std::uint64_t kDefaultWork = 50000000;
constexpr double kPi = 3.14159265358979323846;

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

struct Stop {
    ToolpathIssue issue{ToolpathIssue::Cancelled};
    std::string message;
};

std::string check(const tooling::Tool& tool, const geometry::Box2& placement, const LaserPictureParams& p,
                  ToolpathIssue& issue) {
    issue = ToolpathIssue::InvalidParameters;
    const double values[] = {p.power,          p.minPower,         p.moveSpeed,         p.overscan,
                             p.lineSpacing,    p.lineAngle,        p.safeZ,             p.tolerance.distance,
                             p.tolerance.chordal, p.tolerance.angle, placement.min.x,   placement.min.y,
                             placement.max.x,  placement.max.y};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (placement.isEmpty() || !(placement.width() > 0.0) || !(placement.height() > 0.0) ||
        placement.width() > kMaxLength || placement.height() > kMaxLength) {
        return "the image placement must have a width and height between 0 and " + num(kMaxLength) + " mm";
    }
    if (!(p.power > 0.0) || p.power > 100.0) {
        return "power must be above 0 and at most 100 percent";
    }
    if (p.minPower < 0.0 || p.minPower > p.power) {
        return "minimum power must be between 0 and the power";
    }
    if (p.moveSpeed < 0.0 || p.moveSpeed > 1.0e6) {
        return "move speed must be 0 (the tool's feed rate) or between 0 and 1,000,000 mm/min";
    }
    if (p.passes < 1 || p.passes > kMaxPasses) {
        return "passes must be between 1 and " + std::to_string(kMaxPasses);
    }
    if (p.overscan < 0.0 || p.overscan > kMaxOffset) {
        return "overscan must be between 0 and " + num(kMaxOffset) + " mm";
    }
    if (!(p.lineSpacing > 0.0) || p.lineSpacing > kMaxLength) {
        return "line spacing must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (std::fabs(p.lineAngle) > 1000.0) {
        return "line angle is out of range";
    }
    if (!(p.safeZ > 0.0) || p.safeZ > kMaxLength) {
        return "safe Z must be above the material surface";
    }
    const geometry::Tolerance& t = p.tolerance;
    if (t.chordal < kMinChordal || t.chordal > 1.0 || !(t.distance > 0.0) || t.distance > t.chordal) {
        return "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal";
    }
    if (!laserToolAllowed(tool.type)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "' must be an end mill (its diameter is the beam kerf) for laser toolpaths";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "': " + *bad;
    }
    return {};
}

}  // namespace

GrayImage laserDarkness(const GrayImage& image, bool invert, bool dither) {
    GrayImage out = image;
    if (!image.valid()) {
        return out;
    }
    for (std::uint8_t& v : out.pixels) {
        v = invert ? v : static_cast<std::uint8_t>(255 - v);
    }
    return dither ? laserDither(out) : out;
}

GrayImage laserDither(const GrayImage& darkness) {
    return ditherDarkness(darkness);
}

double laserPicturePower(const LaserPictureParams& params, int darkness) {
    const int g = std::clamp(darkness, 0, 255);
    if (g == 0) {
        return 0.0;
    }
    if (!params.modulatePower) {
        return g >= 128 ? params.power : 0.0;
    }
    return params.minPower + (params.power - params.minPower) * static_cast<double>(g) / 255.0;
}

LaserPictureResult laserPictureToolpath(const GrayImage& image, const geometry::Box2& placement,
                                        const tooling::Tool& tool,
                                        const std::optional<tooling::CuttingData>& cuttingData,
                                        const LaserPictureParams& params, const std::vector<Contour>& contours,
                                        const ToolpathRunControl& control) {
    LaserPictureResult result;
    result.toolpath.name = params.name;
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };
    const auto cancelled = [&control] {
        return control.cancel != nullptr && control.cancel->load(std::memory_order_relaxed);
    };
    if (!image.valid()) {
        say(ToolpathIssue::InvalidParameters, std::nullopt, "the image is empty, too large or inconsistent");
        return result;
    }
    ToolpathIssue issue{};
    if (const std::string bad = check(tool, placement, params, issue); !bad.empty()) {
        say(issue, std::nullopt, bad);
        return result;
    }
    if (const std::string bad = prepareLaserTool(tool, cuttingData, params.moveSpeed, result.toolpath); !bad.empty()) {
        say(ToolpathIssue::InvalidTool, std::nullopt, bad);
        return result;
    }
    const GrayImage darkness = laserDarkness(image, params.invert, params.dither);
    std::array<double, 256> power{};
    for (int g = 0; g < 256; ++g) {
        power[static_cast<std::size_t>(g)] = laserPicturePower(params, g);
    }
    const auto powerOf = [&power](int value) { return power[static_cast<std::size_t>(value)]; };
    const double pixelW = placement.width() / image.width;
    const double pixelH = placement.height() / image.height;
    const double moveLimit = control.moveLimit != 0 ? static_cast<double>(control.moveLimit) : kMaxMoves;
    const double workLimit = static_cast<double>(control.workLimit != 0 ? control.workLimit : kDefaultWork);

    std::vector<Stroke> strokes;
    std::vector<Run> runs;
    std::vector<Span> spans;
    double work = 0.0;
    double pieceCount = 0.0;
    const auto tooMuch = [&](const std::string& what) {
        result.toolpath.moves.clear();
        result.strokes = 0;
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt, what);
        return result;
    };
    const auto checkMoves = [&]() {
        if (!((pieceCount + 4.0 * static_cast<double>(strokes.size())) * static_cast<double>(params.passes) + 1.0 <=
              moveLimit)) {
            throw Stop{ToolpathIssue::WorkLimitExceeded,
                       "the toolpath would have more than " + num(moveLimit) + " moves"};
        }
    };
    const auto countPieces = [&](std::size_t from) {
        for (std::size_t i = from; i < strokes.size(); ++i) {
            pieceCount += static_cast<double>(strokes[i].pieces.size());
        }
    };

    // Raster / hatch passes: parallel lines at `angle` across the placement, as a serpentine.
    const auto rasterPass = [&](double angle) {
        const double ux = std::cos(angle);
        const double uy = std::sin(angle);
        const double nx = -uy;
        const double ny = ux;
        const std::array<Vec2, 4> corners{Vec2{placement.min.x, placement.min.y}, Vec2{placement.max.x, placement.min.y},
                                          Vec2{placement.max.x, placement.max.y}, Vec2{placement.min.x, placement.max.y}};
        double nmin = 1e300;
        double nmax = -1e300;
        for (const Vec2 c : corners) {
            const double d = c.x * nx + c.y * ny;
            nmin = std::min(nmin, d);
            nmax = std::max(nmax, d);
        }
        const double range = nmax - nmin;
        const double countD = std::max(1.0, std::ceil(range / params.lineSpacing - 1e-9));
        const double diagonal = std::hypot(placement.width(), placement.height());
        work += countD * (diagonal / pixelW + diagonal / pixelH + 2.0);
        if (!(work <= workLimit) || !(countD <= moveLimit)) {
            throw Stop{ToolpathIssue::WorkLimitExceeded,
                       "the line spacing is too fine for this image: more than " + num(workLimit) +
                           " pixel crossings of work"};
        }
        const auto count = static_cast<std::size_t>(countD);
        const double pitch = range / countD;
        bool forward = true;
        for (std::size_t k = 0; k < count; ++k) {
            if (cancelled()) {
                throw Stop{ToolpathIssue::Cancelled, {}};
            }
            const double d = nmin + pitch * (static_cast<double>(k) + 0.5);
            Vec2 a{nx * d - ux * 1e6, ny * d - uy * 1e6};
            Vec2 b{nx * d + ux * 1e6, ny * d + uy * 1e6};
            if (!clipSegment(placement, a, b)) {
                continue;
            }
            if (!forward) {
                std::swap(a, b);
            }
            pixelRuns(darkness, placement, a, b, runs);
            spans.clear();
            spansOfLine(a, b, runs, spans);
            const std::size_t before = strokes.size();
            buildStrokes(spans, {forward ? ux : -ux, forward ? uy : -uy}, powerOf, strokes);
            if (strokes.size() != before) {
                forward = !forward;
                countPieces(before);
                checkMoves();
            }
        }
    };

    try {
        if (params.strategy == PhotoStrategy::SelectedVectors) {
            geometry::CancelScope scope(control.cancel);
            bool any = false;
            for (std::size_t i = 0; i < contours.size(); ++i) {
                if (cancelled()) {
                    throw Stop{};
                }
                const Contour& c = contours[i];
                if (c.isEmpty()) {
                    say(ToolpathIssue::EmptyContour, i, "vector " + std::to_string(i) + " has no spans; skipped");
                    continue;
                }
                if (!c.isContinuous(params.tolerance)) {
                    say(ToolpathIssue::DiscontinuousContour, i,
                        "vector " + std::to_string(i) + " has gaps between spans; skipped");
                    continue;
                }
                std::vector<Vec2> pts = c.polygonise(params.tolerance).points;
                if (cancelled()) {
                    throw Stop{};
                }
                if (c.closed && !pts.empty()) {
                    pts.push_back(pts.front());
                }
                bool finite = !pts.empty();
                double length = 0.0;
                for (std::size_t k = 0; k < pts.size(); ++k) {
                    finite = finite && std::isfinite(pts[k].x) && std::isfinite(pts[k].y);
                    if (finite && k > 0) {
                        length += geometry::distance(pts[k - 1], pts[k]);
                    }
                }
                if (!finite) {
                    say(ToolpathIssue::InvalidParameters, i,
                        "vector " + std::to_string(i) + " has a coordinate that is not finite; skipped");
                    continue;
                }
                if (!(length > params.tolerance.distance)) {
                    say(ToolpathIssue::NoRegion, i, "vector " + std::to_string(i) + " has no length to burn; skipped");
                    continue;
                }
                work += length / pixelW + length / pixelH + static_cast<double>(pts.size());
                if (!(work <= workLimit)) {
                    throw Stop{ToolpathIssue::WorkLimitExceeded, "more than " + num(workLimit) + " pixel crossings of work"};
                }
                spans.clear();
                for (std::size_t k = 1; k < pts.size(); ++k) {
                    Vec2 a = pts[k - 1];
                    Vec2 b = pts[k];
                    const Vec2 a0 = a;
                    const Vec2 b0 = b;
                    if (!clipSegment(placement, a, b)) {
                        spans.push_back({a0, b0, -1, static_cast<int>(k)});  // wholly off the bitmap
                        continue;
                    }
                    if (a.x != a0.x || a.y != a0.y) {
                        spans.push_back({a0, a, -1, static_cast<int>(k)});
                    }
                    pixelRuns(darkness, placement, a, b, runs);
                    spansOfLine(a, b, runs, spans, static_cast<int>(k));
                    if (b.x != b0.x || b.y != b0.y) {
                        spans.push_back({b, b0, -1, static_cast<int>(k)});
                    }
                }
                const std::size_t before = strokes.size();
                buildStrokes(spans, {1.0, 0.0}, powerOf, strokes);
                countPieces(before);
                checkMoves();
                any = any || strokes.size() != before;
            }
            if (!any && result.diagnostics.empty()) {
                say(ToolpathIssue::NoRegion, std::nullopt, "no vector crosses a pixel that burns");
            }
        } else {
            rasterPass(params.lineAngle);
            if (params.strategy == PhotoStrategy::Hatch) {
                rasterPass(params.lineAngle + kPi / 2.0);
            }
        }
    } catch (const Stop& stop) {
        if (stop.issue == ToolpathIssue::Cancelled) {
            result.toolpath.moves.clear();
            result.strokes = 0;
            say(ToolpathIssue::Cancelled, std::nullopt, "cancelled");
            return result;
        }
        return tooMuch(stop.message);
    }
    if (strokes.empty()) {
        if (result.diagnostics.empty()) {
            say(ToolpathIssue::NoRegion, std::nullopt, "the image has nothing to burn with these settings");
        }
        return result;
    }
    result.strokes = strokes.size();
    // Hatch with a second pass: the strokes are in order (first pass, second pass); each run-through repeats them.
    const bool overscan = params.overscan > 0.0 && params.strategy != PhotoStrategy::SelectedVectors;
    std::vector<Move>& moves = result.toolpath.moves;
    moves.reserve(static_cast<std::size_t>((pieceCount + 4.0 * static_cast<double>(strokes.size())) *
                                           static_cast<double>(params.passes)) + 1);
    Vec2 last{};
    for (int pass = 0; pass < params.passes; ++pass) {
        last = emitStrokes(strokes, overscan ? params.overscan : 0.0, 0.0, moves, last);
    }
    moves.push_back(Move::rapid({last.x, last.y, params.safeZ}));
    return result;
}

}  // namespace carvetto::toolpath
