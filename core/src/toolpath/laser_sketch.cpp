#include "carvetto/toolpath/laser_sketch.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <variant>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
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
constexpr double kMaxMoves = 20000000.0;
constexpr double kMaxLines = 2000000.0;       // hatch lines per angle
constexpr double kMaxCrossings = 50000000.0;  // line/edge crossings per angle
constexpr std::uint64_t kDefaultWork = 50000000;
constexpr std::size_t kMaxTracks = 64;
constexpr std::size_t kPollEvery = 1024;
constexpr double kHalfPi = 1.5707963267948966;

/// cos and sin of `angle`, exact for multiples of a right angle (so a cross hatch is not jittered by rounding).
void direction(double angle, double& c, double& s) {
    c = std::cos(angle);
    s = std::sin(angle);
    for (double* v : {&c, &s}) {
        if (std::fabs(*v) < 1e-12) {
            *v = 0.0;
        } else if (std::fabs(std::fabs(*v) - 1.0) < 1e-12) {
            *v = *v > 0.0 ? 1.0 : -1.0;
        }
    }
}

std::string num(double v) {
    std::array<char, 64> buf{};
    const auto res = std::to_chars(buf.data(), buf.data() + buf.size(), v);
    return std::string(buf.data(), res.ptr);
}

struct Stop {
    ToolpathIssue issue{ToolpathIssue::Cancelled};
    std::string message;
};

std::string check(const tooling::Tool& tool, const geometry::Box2& placement, const LaserSketchParams& p,
                  ToolpathIssue& issue) {
    issue = ToolpathIssue::InvalidParameters;
    const double values[] = {p.power,          p.moveSpeed,        p.overscan,          p.lineThickness,
                             p.stepover,       p.hatchAngle,       p.safeZ,             p.startPosition.x,
                             p.startPosition.y, p.tolerance.distance, p.tolerance.chordal, p.tolerance.angle,
                             placement.min.x,  placement.min.y,    placement.max.x,     placement.max.y};
    for (const double v : values) {
        if (!std::isfinite(v)) {
            return "a parameter is not a finite number";
        }
    }
    if (placement.isEmpty() || !(placement.width() > 0.0) || !(placement.height() > 0.0) ||
        placement.width() > kMaxLength || placement.height() > kMaxLength) {
        return "the source placement must have a width and height between 0 and " + num(kMaxLength) + " mm";
    }
    if (!(p.power > 0.0) || p.power > 100.0) {
        return "power must be above 0 and at most 100 percent";
    }
    if (p.moveSpeed < 0.0 || p.moveSpeed > 1.0e6) {
        return "move speed must be 0 (the tool's feed rate) or between 0 and 1,000,000 mm/min";
    }
    if (p.overscan < 0.0 || p.overscan > 1000.0) {
        return "overscan must be between 0 and 1000 mm";
    }
    if (!(p.lineThickness > 0.0) || p.lineThickness > 1000.0) {
        return "line thickness must be between 0 and 1000 mm";
    }
    if (!(p.stepover > 0.0) || p.stepover > kMaxLength) {
        return "stepover must be between 0 and " + num(kMaxLength) + " mm";
    }
    if (std::fabs(p.hatchAngle) > 1000.0) {
        return "hatch angle is out of range";
    }
    if (!(p.safeZ > 0.0) || p.safeZ > kMaxLength) {
        return "safe Z must be above the material surface";
    }
    const geometry::Tolerance& t = p.tolerance;
    if (t.chordal < kMinChordal || t.chordal > 1.0 || !(t.distance >= 1e-6) || t.distance > t.chordal) {
        return "invalid tolerance: chordal must be between 0.0001 and 1 mm, distance between 1e-6 mm and chordal";
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

/// A closed polygon ring.
struct Ring {
    std::vector<Vec2> pts;
};

double signedArea(const std::vector<Vec2>& p) {
    double a = 0.0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const Vec2& u = p[i];
        const Vec2& v = p[(i + 1) % p.size()];
        a += u.x * v.y - v.x * u.y;
    }
    return 0.5 * a;
}

/// One hatch line: its offset `v` across the hatch direction and its segments (u0 < u1) along it, ascending.
struct HatchLine {
    double v{0.0};
    std::vector<std::pair<double, double>> segs;
};

/// The hatch lines over `rings` (even-odd) at `angle`, on the multiples of `step` across their direction, ascending.
/// Returns false when over the work limits; `cancelled` is set when the cancel flag was seen.
bool hatchLines(const std::vector<const Ring*>& rings, double angle, double step, double minLength,
                std::vector<HatchLine>& out, const ToolpathRunControl& control, bool& cancelled) {
    double ca = 0.0;
    double sa = 0.0;
    direction(angle, ca, sa);
    double vmin = 0.0;
    double vmax = 0.0;
    bool any = false;
    for (const Ring* r : rings) {
        for (const Vec2& p : r->pts) {
            const double v = -sa * p.x + ca * p.y;
            vmin = any ? std::min(vmin, v) : v;
            vmax = any ? std::max(vmax, v) : v;
            any = true;
        }
    }
    if (!any) {
        return true;
    }
    if ((vmax - vmin) / step > kMaxLines) {
        return false;
    }
    struct Crossing {
        long long k;
        double u;
    };
    std::vector<Crossing> xs;
    std::size_t polled = 0;
    for (const Ring* r : rings) {
        const std::size_t n = r->pts.size();
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 a = r->pts[i];
            const Vec2 b = r->pts[(i + 1) % n];
            const double va = -sa * a.x + ca * a.y;
            const double vb = -sa * b.x + ca * b.y;
            if (va == vb) {
                continue;
            }
            const double ua = ca * a.x + sa * a.y;
            const double ub = ca * b.x + sa * b.y;
            const double lo = std::min(va, vb);
            const double hi = std::max(va, vb);
            const auto k0 = static_cast<long long>(std::ceil(lo / step));
            const auto k1 = static_cast<long long>(std::ceil(hi / step)) - 1;  // k * step < hi
            if (k1 >= k0 && static_cast<double>(xs.size()) + static_cast<double>(k1 - k0 + 1) > kMaxCrossings) {
                return false;
            }
            for (long long k = k0; k <= k1; ++k) {
                if (control.cancel != nullptr && ++polled % kPollEvery == 0 && control.cancel->load()) {
                    cancelled = true;
                    return true;
                }
                const double t = (static_cast<double>(k) * step - va) / (vb - va);
                xs.push_back({k, ua + (ub - ua) * t});
            }
        }
    }
    std::sort(xs.begin(), xs.end(), [](const Crossing& l, const Crossing& r) {
        return l.k != r.k ? l.k < r.k : l.u < r.u;
    });
    std::size_t i = 0;
    while (i < xs.size()) {
        std::size_t j = i;
        while (j < xs.size() && xs[j].k == xs[i].k) {
            ++j;
        }
        HatchLine line;
        line.v = static_cast<double>(xs[i].k) * step;
        for (std::size_t q = i; q + 1 < j; q += 2) {
            if (xs[q + 1].u - xs[q].u > minLength) {
                line.segs.emplace_back(xs[q].u, xs[q + 1].u);
            }
        }
        if (!line.segs.empty()) {
            out.push_back(std::move(line));
        }
        i = j;
    }
    return true;
}

}  // namespace

std::size_t laserSketchTracks(double thickness, double kerf) {
    if (!(kerf > 0.0) || !std::isfinite(thickness) || !std::isfinite(kerf)) {
        return 1;
    }
    const double n = std::round(thickness / kerf);
    if (!(n >= 1.0)) {
        return 1;
    }
    return n >= static_cast<double>(kMaxTracks) ? kMaxTracks : static_cast<std::size_t>(n);
}

std::size_t laserSketchLit(int darkness, std::size_t tracks) {
    const int g = std::clamp(darkness, 0, 255);
    return (2 * static_cast<std::size_t>(g) * tracks + 255) / 510;
}

GrayImage toneFromHeightField(const relief::HeightField& field, SketchModelTone tone, geometry::Box2& placement) {
    GrayImage out;
    float lo = 0.0F;
    float hi = 0.0F;
    if (field.empty() || !field.minMax(lo, hi)) {
        return out;
    }
    const std::size_t w = field.width();
    const std::size_t h = field.height();
    const double cs = field.cellSize();
    const relief::Point origin = field.origin();
    std::size_t f = 1;
    while ((w + f - 1) / f > static_cast<std::size_t>(io::kMaxImageEdge) ||
           (h + f - 1) / f > static_cast<std::size_t>(io::kMaxImageEdge) ||
           ((w + f - 1) / f) * ((h + f - 1) / f) > io::kMaxImagePixels) {
        ++f;
    }
    const std::size_t ow = (w + f - 1) / f;
    const std::size_t oh = (h + f - 1) / f;
    constexpr double kLightElevation = 0.7853981633974483;  // 45 degrees
    constexpr double kLightAzimuth = 2.356194490192345;     // 135 degrees: from the upper left
    const double lx = std::cos(kLightElevation) * std::cos(kLightAzimuth);
    const double ly = std::cos(kLightElevation) * std::sin(kLightAzimuth);
    const double lz = std::sin(kLightElevation);
    const auto opaque = [&](std::size_t i, std::size_t j) { return std::isfinite(field.at(i, j)); };
    // Gray (0..255) of one opaque cell.
    const auto cellGray = [&](std::size_t i, std::size_t j) {
        const double z = static_cast<double>(field.at(i, j));
        if (tone == SketchModelTone::Depth) {
            return hi > lo ? 255.0 * (z - static_cast<double>(lo)) / (static_cast<double>(hi) - static_cast<double>(lo))
                           : 255.0;
        }
        const bool hasL = i > 0 && opaque(i - 1, j);
        const bool hasR = i + 1 < w && opaque(i + 1, j);
        const bool hasD = j > 0 && opaque(i, j - 1);
        const bool hasU = j + 1 < h && opaque(i, j + 1);
        const double zl = hasL ? static_cast<double>(field.at(i - 1, j)) : z;
        const double zr = hasR ? static_cast<double>(field.at(i + 1, j)) : z;
        const double zd = hasD ? static_cast<double>(field.at(i, j - 1)) : z;
        const double zu = hasU ? static_cast<double>(field.at(i, j + 1)) : z;
        const double nx = hasL || hasR ? (zr - zl) / (cs * ((hasL ? 1.0 : 0.0) + (hasR ? 1.0 : 0.0))) : 0.0;
        const double ny = hasD || hasU ? (zu - zd) / (cs * ((hasD ? 1.0 : 0.0) + (hasU ? 1.0 : 0.0))) : 0.0;
        const double len = std::sqrt(nx * nx + ny * ny + 1.0);
        const double shade = (-nx * lx - ny * ly + lz) / len;
        return 255.0 * std::clamp(shade / lz, 0.0, 1.0);
    };
    out.width = static_cast<int>(ow);
    out.height = static_cast<int>(oh);
    out.pixels.assign(ow * oh, 255);
    for (std::size_t r = 0; r < oh; ++r) {
        for (std::size_t c = 0; c < ow; ++c) {
            double sum = 0.0;
            std::size_t count = 0;
            for (std::size_t dr = 0; dr < f; ++dr) {
                const std::size_t tr = r * f + dr;  // rows from the top
                if (tr >= h) {
                    break;
                }
                const std::size_t j = h - 1 - tr;
                for (std::size_t dc = 0; dc < f; ++dc) {
                    const std::size_t i = c * f + dc;
                    if (i >= w) {
                        break;
                    }
                    if (opaque(i, j)) {
                        sum += cellGray(i, j);
                        ++count;
                    }
                }
            }
            if (count > 0) {
                out.pixels[r * ow + c] = static_cast<std::uint8_t>(std::clamp(std::lround(sum / static_cast<double>(count)), 0L, 255L));
            }
        }
    }
    placement = geometry::Box2({origin.x - 0.5 * cs, origin.y - 0.5 * cs},
                               {origin.x + (static_cast<double>(w) - 0.5) * cs,
                                origin.y + (static_cast<double>(h) - 0.5) * cs});
    return out;
}

LaserSketchResult laserSketchToolpath(const GrayImage& tone, const geometry::Box2& placement, const tooling::Tool& tool,
                                      const std::optional<tooling::CuttingData>& cuttingData,
                                      const LaserSketchParams& params, const std::vector<Contour>& contours,
                                      const std::optional<geometry::Box2>& machineLimits,
                                      const ToolpathRunControl& control) {
    LaserSketchResult result;
    result.toolpath.name = params.name;
    auto say = [&result](ToolpathIssue issue, std::optional<std::size_t> contour, std::string message) {
        result.diagnostics.push_back(ToolpathDiagnostic{issue, contour, std::move(message), std::nullopt});
    };
    if (!tone.valid()) {
        say(ToolpathIssue::InvalidParameters, std::nullopt, "the source image is empty, too large or inconsistent");
        return result;
    }
    ToolpathIssue issue{};
    if (const std::string bad = check(tool, placement, params, issue); !bad.empty()) {
        say(issue, std::nullopt, bad);
        return result;
    }
    if (params.machineLimitBoundary && machineLimits.has_value()) {
        const geometry::Box2& m = *machineLimits;
        if (!std::isfinite(m.min.x) || !std::isfinite(m.min.y) || !std::isfinite(m.max.x) ||
            !std::isfinite(m.max.y)) {
            say(ToolpathIssue::InvalidParameters, std::nullopt, "the machine limits are not finite numbers");
            return result;
        }
    }
    if (const std::string bad = prepareLaserTool(tool, cuttingData, params.moveSpeed, result.toolpath); !bad.empty()) {
        say(ToolpathIssue::InvalidTool, std::nullopt, bad);
        return result;
    }
    const geometry::Tolerance& tol = params.tolerance;

    // Where beam may burn: the placement, within the machine limits.
    geometry::Box2 clip = placement;
    if (params.machineLimitBoundary && machineLimits.has_value()) {
        clip.min.x = std::max(clip.min.x, machineLimits->min.x);
        clip.min.y = std::max(clip.min.y, machineLimits->min.y);
        clip.max.x = std::min(clip.max.x, machineLimits->max.x);
        clip.max.y = std::min(clip.max.y, machineLimits->max.y);
        if (!(clip.max.x > clip.min.x) || !(clip.max.y > clip.min.y)) {
            say(ToolpathIssue::NoRegion, std::nullopt, "the source lies outside the machine limits");
            return result;
        }
    }

    const double kerf = tool.diameter;
    const std::size_t tracks = laserSketchTracks(params.lineThickness, kerf);
    result.tracks = tracks;
    // Tracks a kerf apart about the hatch line; lit centre-out (the lower side first on a tie).
    std::vector<double> offsets(tracks);
    std::vector<std::size_t> rank(tracks);
    {
        std::vector<std::size_t> order(tracks);
        for (std::size_t j = 0; j < tracks; ++j) {
            offsets[j] = (static_cast<double>(j) - 0.5 * static_cast<double>(tracks - 1)) * kerf;
            order[j] = j;
        }
        std::stable_sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
            return std::fabs(offsets[a]) < std::fabs(offsets[b]) - 1e-12;
        });
        for (std::size_t r = 0; r < tracks; ++r) {
            rank[order[r]] = r;
        }
    }
    // Beam power per pixel value per track.
    std::vector<std::array<double, 256>> powerTable(tracks);
    for (int value = 0; value < 256; ++value) {
        const int darkness = params.invert ? value : 255 - value;
        const std::size_t lit = laserSketchLit(darkness, tracks);
        for (std::size_t j = 0; j < tracks; ++j) {
            powerTable[j][static_cast<std::size_t>(value)] = lit > rank[j] ? params.power : 0.0;
        }
    }

    // The regions to hatch, as groups of rings.
    std::vector<Ring> rings;
    std::vector<std::vector<const Ring*>> groups;
    if (params.boundary == SketchBoundary::Source) {
        Ring ring;
        ring.pts = {{clip.min.x, clip.min.y}, {clip.max.x, clip.min.y}, {clip.max.x, clip.max.y}, {clip.min.x, clip.max.y}};
        rings.push_back(std::move(ring));
        groups.push_back({&rings.front()});
    } else {
        std::vector<Contour> closed;
        for (std::size_t i = 0; i < contours.size(); ++i) {
            const Contour& c = contours[i];
            if (c.isEmpty()) {
                say(ToolpathIssue::EmptyContour, i, "vector " + std::to_string(i) + " has no spans; skipped");
            } else if (!c.closed) {
                say(ToolpathIssue::OpenContourIgnored, i,
                    "vector " + std::to_string(i) + " is open; the sketch needs closed vectors");
            } else if (!c.isContinuous(tol)) {
                say(ToolpathIssue::DiscontinuousContour, i,
                    "vector " + std::to_string(i) + " has gaps between spans; skipped");
            } else if (geometry::refusedByClippers(c, tol)) {
                result.diagnostics.push_back(refusedContourDiagnostic(i, "vector " + std::to_string(i)));
            } else {
                closed.push_back(c);
            }
        }
        if (closed.empty()) {
            if (result.diagnostics.empty()) {
                say(ToolpathIssue::NoRegion, std::nullopt, "no closed vectors to sketch inside");
            }
            return result;
        }
        std::vector<Contour> region = geometry::unionContours(
            closed, tol, params.fill == SketchFill::BetweenVectors ? geometry::FillRule::EvenOdd : geometry::FillRule::NonZero);
        const double inset = 0.5 * kerf;
        if (inset != 0.0) {
            region = geometry::offsetContours(region, -inset, geometry::JoinType::Round, tol);
        }
        if (region.empty()) {
            say(ToolpathIssue::ToolTooLarge, std::nullopt, "the region is too small for the beam");
            return result;
        }
        std::vector<Contour> ringContours;
        for (const Contour& c : region) {
            Ring ring;
            for (const Vec2 q : c.polygonise(tol).points) {
                ring.pts.push_back(q);
            }
            if (ring.pts.size() >= 3) {
                rings.push_back(std::move(ring));
                ringContours.push_back(c);
            }
        }
        std::vector<std::size_t> outers;
        std::vector<std::size_t> holes;
        for (std::size_t i = 0; i < rings.size(); ++i) {
            (signedArea(rings[i].pts) > 0.0 ? outers : holes).push_back(i);
        }
        if (params.fill == SketchFill::Together) {
            std::vector<const Ring*> all;
            for (const Ring& g : rings) {
                all.push_back(&g);
            }
            groups.push_back(std::move(all));
        } else {
            std::vector<std::vector<const Ring*>> byOuter(outers.size());
            std::vector<double> outerArea(outers.size());
            for (std::size_t o = 0; o < outers.size(); ++o) {
                byOuter[o].push_back(&rings[outers[o]]);
                outerArea[o] = signedArea(rings[outers[o]].pts);
            }
            for (const std::size_t h : holes) {
                std::size_t best = outers.size();
                for (std::size_t o = 0; o < outers.size(); ++o) {
                    if ((best == outers.size() || outerArea[o] < outerArea[best]) &&
                        geometry::isInside(ringContours[outers[o]], ringContours[h].startPoint(),
                                           geometry::FillRule::EvenOdd, tol)) {
                        best = o;
                    }
                }
                if (best < outers.size()) {
                    byOuter[best].push_back(&rings[h]);
                }
            }
            std::vector<OrderItem> items;
            for (std::size_t o = 0; o < outers.size(); ++o) {
                OrderItem item;
                item.path = ringContours[outers[o]];
                item.selectionIndex = o;
                item.start = item.end = ringContours[outers[o]].startPoint();
                items.push_back(std::move(item));
            }
            for (const std::size_t o : orderCuts(items, params.order, params.startPosition, tol)) {
                groups.push_back(std::move(byOuter[o]));
            }
        }
    }

    const double pixelW = placement.width() / tone.width;
    const double pixelH = placement.height() / tone.height;
    const double moveLimit = control.moveLimit != 0 ? static_cast<double>(control.moveLimit) : kMaxMoves;
    const double workLimit = static_cast<double>(control.workLimit != 0 ? control.workLimit : kDefaultWork);
    const auto cancelled = [&control] {
        return control.cancel != nullptr && control.cancel->load(std::memory_order_relaxed);
    };
    std::vector<Stroke> strokes;
    std::vector<Run> runs;
    std::vector<Span> spans;
    std::vector<HatchLine> lines;
    double work = 0.0;
    double pieceCount = 0.0;
    try {
        const auto sketch = [&](const std::vector<const Ring*>& group, double angle) {
            lines.clear();
            bool stopped = false;
            if (!hatchLines(group, angle, params.stepover, tol.distance, lines, control, stopped)) {
                throw Stop{ToolpathIssue::WorkLimitExceeded,
                           "the sketch would need more than " + num(kMaxLines) + " hatch lines or " +
                               num(kMaxCrossings) + " crossings (stepover " + num(params.stepover) + " mm)"};
            }
            if (stopped) {
                throw Stop{};
            }
            double length = 0.0;
            for (const HatchLine& l : lines) {
                for (const auto& [u0, u1] : l.segs) {
                    length += u1 - u0;
                }
            }
            work += length * static_cast<double>(tracks) * (1.0 / pixelW + 1.0 / pixelH) +
                    static_cast<double>(lines.size() * tracks);
            if (!(work <= workLimit)) {
                throw Stop{ToolpathIssue::WorkLimitExceeded,
                           "the sketch needs more than " + num(workLimit) + " pixel crossings of work"};
            }
            double ca = 0.0;
            double sa = 0.0;
            direction(angle, ca, sa);
            const auto at = [&](double u, double v) { return Vec2{ca * u - sa * v, sa * u + ca * v}; };
            bool forward = true;
            for (const HatchLine& line : lines) {
                for (std::size_t j = 0; j < tracks; ++j) {
                    if (cancelled()) {
                        throw Stop{};
                    }
                    const auto powerOf = [&](int value) { return powerTable[j][static_cast<std::size_t>(value)]; };
                    const double v = line.v + offsets[j];
                    const std::size_t before = strokes.size();
                    const std::size_t n = line.segs.size();
                    for (std::size_t q = 0; q < n; ++q) {
                        const auto& [u0, u1] = line.segs[forward ? q : n - 1 - q];
                        Vec2 a = at(forward ? u0 : u1, v);
                        Vec2 b = at(forward ? u1 : u0, v);
                        if (!clipSegment(clip, a, b)) {
                            continue;
                        }
                        pixelRuns(tone, placement, a, b, runs);
                        spans.clear();
                        spansOfLine(a, b, runs, spans);
                        buildStrokes(spans, forward ? Vec2{ca, sa} : Vec2{-ca, -sa}, powerOf, strokes);
                    }
                    if (strokes.size() != before) {
                        forward = !forward;
                        for (std::size_t s = before; s < strokes.size(); ++s) {
                            pieceCount += static_cast<double>(strokes[s].pieces.size());
                        }
                        if (!((pieceCount + 4.0 * static_cast<double>(strokes.size())) + 1.0 <= moveLimit)) {
                            throw Stop{ToolpathIssue::WorkLimitExceeded,
                                       "the toolpath would have more than " + num(moveLimit) + " moves"};
                        }
                    }
                }
            }
        };
        for (const auto& group : groups) {
            sketch(group, params.hatchAngle);
            if (params.crossHatch) {
                sketch(group, params.hatchAngle + kHalfPi);
            }
        }
    } catch (const Stop& stop) {
        result.toolpath.moves.clear();
        result.strokes = 0;
        if (stop.issue == ToolpathIssue::Cancelled) {
            say(ToolpathIssue::Cancelled, std::nullopt, "cancelled");
        } else {
            say(stop.issue, std::nullopt, stop.message);
        }
        return result;
    }
    if (strokes.empty()) {
        say(ToolpathIssue::NoRegion, std::nullopt, "the source has nothing to burn with these settings");
        return result;
    }
    result.strokes = strokes.size();
    std::vector<Move>& moves = result.toolpath.moves;
    moves.reserve(static_cast<std::size_t>(pieceCount + 4.0 * static_cast<double>(strokes.size())) + 1);
    // The beam-off overscan runs stay inside the machine limits too (ADR-0113).
    const bool limited = params.machineLimitBoundary && machineLimits.has_value();
    const Vec2 last = emitStrokes(strokes, params.overscan, 0.0, moves, {}, limited ? &*machineLimits : nullptr);
    moves.push_back(Move::rapid({last.x, last.y, params.safeZ}));
    return result;
}

}  // namespace carvetto::toolpath
