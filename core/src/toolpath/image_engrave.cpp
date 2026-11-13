#include "carvetto/toolpath/image_engrave.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <tuple>
#include <utility>

#include "carvetto/geometry/cancel.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::toolpath {

namespace {

using geometry::Contour;
using geometry::Vec2;

constexpr double kMinChordal = 1e-4;
constexpr double kMaxLength = 100000.0;
constexpr double kMaxMoves = 20000000.0;
constexpr std::uint64_t kDefaultWork = 50000000;
constexpr double kMinStep = 1e-3;       // smallest sample step (mm)
constexpr double kCutEps = 1e-6;        // depth below this is "not cutting"
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

std::string check(const tooling::Tool& tool, const geometry::Box2& placement, const ImageEngraveParams& p,
                  ToolpathIssue& issue) {
    issue = ToolpathIssue::InvalidParameters;
    const double values[] = {p.maxDepth,         p.stepoverRetract, p.lineSpacingPercent, p.lineAngle,
                             p.safeZ,            p.depthTolerance,  p.tolerance.distance, p.tolerance.chordal,
                             p.tolerance.angle,  placement.min.x,   placement.min.y,      placement.max.x,
                             placement.max.y};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (placement.isEmpty() || !(placement.width() > 0.0) || !(placement.height() > 0.0) ||
        placement.width() > kMaxLength || placement.height() > kMaxLength) {
        return "the image placement must have a width and height between 0 and " + num(kMaxLength) + " mm";
    }
    if (!(p.maxDepth > 0.0) || p.maxDepth > kMaxLength) {
        return "max carving depth must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (p.stepoverRetract < 0.0 || p.stepoverRetract > kMaxLength) {
        return "stepover retract must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (!(p.lineSpacingPercent > 0.0) || p.lineSpacingPercent > 1000.0) {
        return "line spacing must be greater than 0 and at most 1000 % of the tool diameter";
    }
    if (std::fabs(p.lineAngle) > 1000.0) {
        return "line angle is out of range";
    }
    if (!(p.safeZ > 0.0) || p.safeZ > kMaxLength) {
        return "safe Z must be above the material surface";
    }
    if (!(p.depthTolerance > 0.0) || p.depthTolerance > 1.0) {
        return "depth tolerance must be between 0 and 1 mm";
    }
    const geometry::Tolerance& t = p.tolerance;
    if (t.chordal < kMinChordal || t.chordal > 1.0 || !(t.distance > 0.0) || t.distance > t.chordal) {
        return "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance > 0 and <= chordal";
    }
    if (!imageEngraveToolAllowed(tool.type)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "' must be a V-bit or an engraving bit for an image engrave";
    }
    if (const std::optional<std::string> bad = tooling::validate(tool)) {
        issue = ToolpathIssue::InvalidTool;
        return "tool '" + tool.name + "': " + *bad;
    }
    return {};
}

/// Bilinear gray (0..255) at document position `p`; pixel centres at half-integers, edges clamped.
class Sampler {
   public:
    Sampler(const GrayImage& image, const geometry::Box2& placement, double maxDepth, bool invert, double limit)
        : image_(image), placement_(placement), maxDepth_(maxDepth), invert_(invert), limit_(limit) {
        sx_ = static_cast<double>(image.width) / placement.width();
        sy_ = static_cast<double>(image.height) / placement.height();
    }

    bool inside(Vec2 p) const {
        constexpr double e = 1e-9;
        return p.x >= placement_.min.x - e && p.x <= placement_.max.x + e && p.y >= placement_.min.y - e &&
               p.y <= placement_.max.y + e;
    }

    double gray(Vec2 p) const {
        const double fx = std::clamp((p.x - placement_.min.x) * sx_ - 0.5, 0.0, static_cast<double>(image_.width - 1));
        const double fy = std::clamp((placement_.max.y - p.y) * sy_ - 0.5, 0.0, static_cast<double>(image_.height - 1));
        const int x0 = static_cast<int>(fx);
        const int y0 = static_cast<int>(fy);
        const int x1 = std::min(x0 + 1, image_.width - 1);
        const int y1 = std::min(y0 + 1, image_.height - 1);
        const double tx = fx - x0;
        const double ty = fy - y0;
        const auto px = [&](int x, int y) {
            return static_cast<double>(
                image_.pixels[static_cast<std::size_t>(y) * static_cast<std::size_t>(image_.width) +
                              static_cast<std::size_t>(x)]);
        };
        const double top = px(x0, y0) * (1.0 - tx) + px(x1, y0) * tx;
        const double bot = px(x0, y1) * (1.0 - tx) + px(x1, y1) * tx;
        return top * (1.0 - ty) + bot * ty;
    }

    ImageEngraveParams paramsForDepth() const {
        ImageEngraveParams q;
        q.maxDepth = maxDepth_;
        q.invert = invert_;
        return q;
    }

    double depth(Vec2 p) const {
        if (!inside(p)) {
            return 0.0;
        }
        return imageEngraveDepth(paramsForDepth(), limit_, gray(p));
    }

   private:
    const GrayImage& image_;
    geometry::Box2 placement_;
    double maxDepth_;
    bool invert_;
    double limit_;
    double sx_{1.0};
    double sy_{1.0};
};

struct Sample {
    Vec2 p;
    double depth{0.0};
};

/// Keeps the samples of a straight line (indices into `s`, depth `z`) that a depth tolerance
/// needs: Douglas-Peucker on (arc length, depth), iterative.
std::vector<bool> keepFlags(const std::vector<double>& s, const std::vector<double>& z, double tol) {
    const std::size_t n = s.size();
    std::vector<bool> keep(n, false);
    if (n == 0) {
        return keep;
    }
    keep[0] = true;
    keep[n - 1] = true;
    std::vector<std::pair<std::size_t, std::size_t>> stack;
    if (n > 2) {
        stack.emplace_back(0, n - 1);
    }
    while (!stack.empty()) {
        const auto [a, b] = stack.back();
        stack.pop_back();
        double worst = 0.0;
        std::size_t at = a;
        const double span = s[b] - s[a];
        for (std::size_t i = a + 1; i < b; ++i) {
            const double t = span > 0.0 ? (s[i] - s[a]) / span : 0.0;
            const double d = std::fabs(z[i] - (z[a] + (z[b] - z[a]) * t));
            if (d > worst) {
                worst = d;
                at = i;
            }
        }
        if (worst > tol) {
            keep[at] = true;
            if (at - a > 1) {
                stack.emplace_back(a, at);
            }
            if (b - at > 1) {
                stack.emplace_back(at, b);
            }
        }
    }
    return keep;
}

class Builder {
   public:
    Builder(ImageEngraveResult& result, const ImageEngraveParams& params, double linkDistance, double moveLimit)
        : result_(result), params_(params), linkDistance_(linkDistance), moveLimit_(moveLimit) {}

    /// Emits the cuts of one line: runs of samples that cut, each with a sample of depth ~0 on
    /// each side where there is one.
    bool line(const std::vector<Sample>& v) {
        const std::size_t n = v.size();
        std::size_t i = 0;
        bool any = false;
        while (i < n) {
            if (!(v[i].depth > kCutEps)) {
                ++i;
                continue;
            }
            std::size_t a = i;
            while (i < n && v[i].depth > kCutEps) {
                ++i;
            }
            std::size_t b = i - 1;
            if (a > 0) {
                --a;
            }
            if (b + 1 < n) {
                ++b;
            }
            run(v, a, b);
            any = true;
        }
        if (any) {
            ++result_.lineCount;
        }
        return static_cast<double>(result_.toolpath.moves.size()) <= moveLimit_;
    }

    /// Ends the toolpath: retracts the tool.
    void finish() {
        if (last_) {
            result_.toolpath.moves.push_back(Move::retract({last_->x, last_->y, params_.safeZ}));
            last_.reset();
        }
    }

   private:
    void run(const std::vector<Sample>& v, std::size_t a, std::size_t b) {
        std::vector<Move>& moves = result_.toolpath.moves;
        const Vec3 start{v[a].p.x, v[a].p.y, -v[a].depth};
        bool linked = false;
        if (last_) {
            const double dz = std::fabs(last_->z - start.z);
            const double dxy = geometry::distance(Vec2{last_->x, last_->y}, v[a].p);
            if (params_.stepoverRetract > 0.0 && dz <= params_.stepoverRetract && dxy <= linkDistance_) {
                moves.push_back(Move::feed(start));
                linked = true;
            } else {
                moves.push_back(Move::retract({last_->x, last_->y, params_.safeZ}));
            }
        }
        if (!linked) {
            moves.push_back(Move::rapid({start.x, start.y, params_.safeZ}));
            moves.push_back(Move::plunge(start));
        }
        for (std::size_t k = a + 1; k <= b; ++k) {
            moves.push_back(Move::feed({v[k].p.x, v[k].p.y, -v[k].depth}));
            result_.deepest = std::max(result_.deepest, v[k].depth);
        }
        result_.deepest = std::max(result_.deepest, v[a].depth);
        last_ = Vec3{v[b].p.x, v[b].p.y, -v[b].depth};
    }

    ImageEngraveResult& result_;
    const ImageEngraveParams& params_;
    double linkDistance_;
    double moveLimit_;
    std::optional<Vec3> last_;
};

}  // namespace

GrayImage grayFromRaster(const io::RasterImage& image) {
    GrayImage out;
    if (!image.valid()) {
        return out;
    }
    out.width = image.width;
    out.height = image.height;
    out.pixels.resize(static_cast<std::size_t>(image.width) * static_cast<std::size_t>(image.height));
    for (std::size_t i = 0; i < out.pixels.size(); ++i) {
        const unsigned r = image.rgb[i * 3];
        const unsigned g = image.rgb[i * 3 + 1];
        const unsigned b = image.rgb[i * 3 + 2];
        out.pixels[i] = static_cast<std::uint8_t>((r * 299u + g * 587u + b * 114u + 500u) / 1000u);
    }
    return out;
}

bool imageEngraveToolAllowed(tooling::ToolType type) {
    return type == tooling::ToolType::VBit || type == tooling::ToolType::Engraving;
}

double imageEngraveDepthLimit(const tooling::Tool& tool) {
    if (!imageEngraveToolAllowed(tool.type) || !std::isfinite(tool.diameter) || !std::isfinite(tool.includedAngle) ||
        !(tool.includedAngle > 0.0) || !(tool.includedAngle < kPi)) {
        return 0.0;
    }
    const double tip = tool.type == tooling::ToolType::Engraving ? tool.tipDiameter : 0.0;
    if (!std::isfinite(tip) || !(tool.diameter > tip)) {
        return 0.0;
    }
    return (tool.diameter - tip) / (2.0 * std::tan(tool.includedAngle * 0.5));
}

double imageEngraveDepth(const ImageEngraveParams& params, double depthLimit, double gray) {
    if (!std::isfinite(gray) || !std::isfinite(params.maxDepth)) {
        return 0.0;
    }
    const double v = std::clamp(gray, 0.0, 255.0) / 255.0;
    const double darkness = params.invert ? v : 1.0 - v;
    return std::max(0.0, std::min(params.maxDepth * darkness, depthLimit));
}

ImageEngraveResult imageEngrave(const GrayImage& image, const geometry::Box2& placement, const tooling::Tool& tool,
                              const std::optional<tooling::CuttingData>& cuttingData,
                              const ImageEngraveParams& params, const std::vector<Contour>& contours,
                              const ToolpathRunControl& control) {
    ImageEngraveResult result;
    result.toolpath.name = params.name;
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };
    const auto cancelled = [&control] {
        return control.cancel != nullptr && control.cancel->load(std::memory_order_relaxed);
    };
    const auto cancelOut = [&]() {
        result.toolpath.moves.clear();
        result.lineCount = 0;
        result.deepest = 0.0;
        say(ToolpathIssue::Cancelled, std::nullopt, "cancelled");
        return result;
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
    const PassTool pt = passTool(tool, effectiveCuttingData(tool, cuttingData), params.maxDepth);
    result.toolpath.tool = pt.snapshot;
    if (!std::isfinite(pt.snapshot.plungeRate) || !(pt.snapshot.plungeRate > 0.0) ||
        !std::isfinite(pt.snapshot.feedRate) || !(pt.snapshot.feedRate > 0.0)) {
        say(ToolpathIssue::InvalidTool, std::nullopt,
            "tool '" + tool.name + "': the feed rate and plunge rate must be greater than zero");
        return result;
    }
    const double limit = imageEngraveDepthLimit(tool);
    result.toolDepthLimit = limit;
    if (!(limit > 0.0)) {
        say(ToolpathIssue::InvalidTool, std::nullopt, "tool '" + tool.name + "' has no usable cone angle");
        return result;
    }
    if (params.maxDepth > limit) {
        say(ToolpathIssue::CarveDepthLimited, std::nullopt,
            "max carving depth " + num(params.maxDepth) + " mm is deeper than the tool's full-width depth; limited to " +
                num(limit) + " mm");
    }
    const double spacing = tool.diameter * params.lineSpacingPercent / 100.0;
    const double pixel = std::min(placement.width() / image.width, placement.height() / image.height);
    const double step = std::max(pixel, kMinStep);
    const double moveLimit = control.moveLimit != 0 ? static_cast<double>(control.moveLimit) : kMaxMoves;
    const double workLimit = static_cast<double>(control.workLimit != 0 ? control.workLimit : kDefaultWork);
    const Sampler sampler(image, placement, params.maxDepth, params.invert, limit);
    Builder builder(result, params, std::max(2.0 * spacing, step * 2.0), moveLimit);
    double work = 0.0;
    const auto tooMuch = [&](const std::string& what) {
        result.toolpath.moves.clear();
        result.lineCount = 0;
        result.deepest = 0.0;
        say(ToolpathIssue::WorkLimitExceeded, std::nullopt, what);
        return result;
    };

    // One straight line, given by `a`..`b`, is sampled and cut.
    const auto cutPolyline = [&](const std::vector<Vec2>& pts) {
        std::vector<Sample> samples;
        std::vector<double> s;
        std::vector<double> z;
        double along = 0.0;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            if (i > 0) {
                const double len = geometry::distance(pts[i - 1], pts[i]);
                const auto pieces = static_cast<std::size_t>(std::max(1.0, std::ceil(len / step)));
                for (std::size_t k = 1; k <= pieces; ++k) {
                    const double t = static_cast<double>(k) / static_cast<double>(pieces);
                    const Vec2 q = k == pieces ? pts[i]
                                               : Vec2{pts[i - 1].x + (pts[i].x - pts[i - 1].x) * t,
                                                      pts[i - 1].y + (pts[i].y - pts[i - 1].y) * t};
                    samples.push_back({q, sampler.depth(q)});
                    s.push_back(along + len * t);
                }
                along += len;
            } else {
                samples.push_back({pts[0], sampler.depth(pts[0])});
                s.push_back(0.0);
            }
        }
        z.reserve(samples.size());
        for (const Sample& smp : samples) {
            z.push_back(smp.depth);
        }
        return std::make_tuple(std::move(samples), std::move(s), std::move(z));
    };

    // Raster / hatch passes.
    const auto rasterPass = [&](double angle) -> bool {
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
        const double countD = std::max(1.0, std::ceil(range / spacing - 1e-9));
        const double perLine = std::sqrt(placement.width() * placement.width() + placement.height() * placement.height()) / step;
        work += countD * (perLine + 1.0);
        if (!(work <= workLimit) || !(countD * 2.0 <= moveLimit)) {
            return false;
        }
        const auto count = static_cast<std::size_t>(countD);
        const double pitch = range / countD;
        for (std::size_t k = 0; k < count; ++k) {
            if (cancelled()) {
                throw Stop{ToolpathIssue::Cancelled, {}};
            }
            const double d = nmin + pitch * (static_cast<double>(k) + 0.5);
            // Clip the line {n*d + u*t} to the placement.
            double t0 = -1e300;
            double t1 = 1e300;
            const double lo[2] = {placement.min.x, placement.min.y};
            const double hi[2] = {placement.max.x, placement.max.y};
            const double u[2] = {ux, uy};
            const double base[2] = {nx * d, ny * d};
            bool empty = false;
            for (int axis = 0; axis < 2; ++axis) {
                if (std::fabs(u[axis]) < 1e-12) {
                    if (base[axis] < lo[axis] - 1e-9 || base[axis] > hi[axis] + 1e-9) {
                        empty = true;
                    }
                    continue;
                }
                double ta = (lo[axis] - base[axis]) / u[axis];
                double tb = (hi[axis] - base[axis]) / u[axis];
                if (ta > tb) {
                    std::swap(ta, tb);
                }
                t0 = std::max(t0, ta);
                t1 = std::min(t1, tb);
            }
            if (empty || !(t1 - t0 > 1e-9)) {
                continue;
            }
            Vec2 a{nx * d + ux * t0, ny * d + uy * t0};
            Vec2 b{nx * d + ux * t1, ny * d + uy * t1};
            if (k % 2 == 1) {
                std::swap(a, b);
            }
            auto [samples, s, z] = cutPolyline({a, b});
            const std::vector<bool> keep = keepFlags(s, z, params.depthTolerance);
            std::vector<Sample> kept;
            for (std::size_t i = 0; i < samples.size(); ++i) {
                if (keep[i]) {
                    kept.push_back(samples[i]);
                }
            }
            if (!builder.line(kept)) {
                throw Stop{ToolpathIssue::WorkLimitExceeded, "the toolpath would have more than " + num(moveLimit) + " moves"};
            }
        }
        return true;
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
                    say(ToolpathIssue::NoRegion, i, "vector " + std::to_string(i) + " has no length to carve; skipped");
                    continue;
                }
                work += length / step + static_cast<double>(pts.size());
                if (!(work <= workLimit)) {
                    throw Stop{ToolpathIssue::WorkLimitExceeded,
                               "more than " + num(workLimit) + " samples of work"};
                }
                auto [samples, s, z] = cutPolyline(pts);
                (void)s;
                (void)z;
                if (!builder.line(samples)) {
                    throw Stop{ToolpathIssue::WorkLimitExceeded,
                               "the toolpath would have more than " + num(moveLimit) + " moves"};
                }
                any = true;
            }
            if (!any && result.diagnostics.empty()) {
                say(ToolpathIssue::NoRegion, std::nullopt, "no vectors to carve");
            }
        } else {
            if (!rasterPass(params.lineAngle)) {
                return tooMuch("the line spacing is too fine for this image: more than " + num(workLimit) +
                               " samples of work");
            }
            if (params.strategy == PhotoStrategy::Hatch) {
                builder.finish();
                if (!rasterPass(params.lineAngle + kPi / 2.0)) {
                    return tooMuch("the line spacing is too fine for this image: more than " + num(workLimit) +
                                   " samples of work");
                }
            }
        }
        builder.finish();
    } catch (const Stop& stop) {
        if (stop.issue == ToolpathIssue::Cancelled) {
            return cancelOut();
        }
        return tooMuch(stop.message);
    }
    if (cancelled()) {
        return cancelOut();
    }
    if (result.toolpath.moves.empty() && result.diagnostics.empty()) {
        say(ToolpathIssue::NoRegion, std::nullopt, "the image has nothing to carve at this depth setting");
    }
    return result;
}

}  // namespace carvetto::toolpath
