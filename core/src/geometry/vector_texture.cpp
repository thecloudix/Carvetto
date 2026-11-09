#include "carvetto/geometry/vector_texture.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

namespace detail {

std::uint64_t splitmix64(std::uint64_t x) {
    x += 0x9E3779B97F4A7C15ULL;
    x = (x ^ (x >> 30)) * 0xBF58476D1CE4E5B9ULL;
    x = (x ^ (x >> 27)) * 0x94D049BB133111EBULL;
    return x ^ (x >> 31);
}

double sinTurns(double t) {
    t -= std::floor(t);  // [0,1); floor and subtraction are exact
    double sign = 1.0;
    if (t >= 0.5) {
        t -= 0.5;
        sign = -1.0;
    }
    if (t > 0.25) t = 0.5 - t;
    const double a = kTwoPi * t;  // [0, pi/2]
    const double a2 = a * a;
    // Taylor series to a^17 (remainder < 1e-13 on [0, pi/2]), Horner form.
    double s = 1.0 / 355687428096000.0;
    s = s * a2 - 1.0 / 1307674368000.0;
    s = s * a2 + 1.0 / 6227020800.0;
    s = s * a2 - 1.0 / 39916800.0;
    s = s * a2 + 1.0 / 362880.0;
    s = s * a2 - 1.0 / 5040.0;
    s = s * a2 + 1.0 / 120.0;
    s = s * a2 - 1.0 / 6.0;
    s = s * a2 + 1.0;
    return sign * a * s;
}

}  // namespace detail

namespace {

constexpr int kSamplesPerWave = 24;
constexpr double kMaxCoord = 1e9;

/// Uniform [0,1) from a hash of three integers (53 significant bits, exact scaling).
double unit(std::uint64_t seed, std::uint64_t line, std::uint64_t wave, std::uint64_t channel) {
    std::uint64_t h = detail::splitmix64(seed ^ (channel * 0xD1B54A32D192ED03ULL));
    h = detail::splitmix64(h ^ (line * 0x9E3779B97F4A7C15ULL));
    h = detail::splitmix64(h ^ (wave * 0xC2B2AE3D27D4EB4FULL));
    return static_cast<double>(h >> 11) * (1.0 / 9007199254740992.0);
}

bool finite(double v) { return std::isfinite(v) && std::fabs(v) <= kMaxCoord; }

/// Liang-Barsky clip of segment a-b to `box`; false when nothing remains.
bool clipSegment(const Box2& box, Vec2 a, Vec2 b, double& t0, double& t1) {
    t0 = 0.0;
    t1 = 1.0;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double p[4] = {-dx, dx, -dy, dy};
    const double q[4] = {a.x - box.min.x, box.max.x - a.x, a.y - box.min.y, box.max.y - a.y};
    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0.0) {
            if (q[i] < 0.0) return false;
        } else {
            const double r = q[i] / p[i];
            if (p[i] < 0.0) {
                if (r > t1) return false;
                t0 = std::max(t0, r);
            } else {
                if (r < t0) return false;
                t1 = std::min(t1, r);
            }
        }
    }
    return true;
}

}  // namespace

ShapeResult<std::vector<Polyline>> generateVectorTexture(const Box2& region,
                                                         const VectorTextureParams& p) {
    using R = ShapeResult<std::vector<Polyline>>;
    if (region.isEmpty() || !finite(region.min.x) || !finite(region.min.y) ||
        !finite(region.max.x) || !finite(region.max.y))
        return R{std::string("texture region must be a finite box")};
    const double w = region.width();
    const double h = region.height();
    if (!(w > 0.0) || !(h > 0.0)) return R{std::string("texture region must have a positive area")};
    if (!std::isfinite(p.angle) || std::fabs(p.angle) > kPi / 2.0 + 1e-9)
        return R{std::string("angle must be within -90 and 90 degrees")};
    if (!finite(p.maxSpacing) || !(p.maxSpacing > 0.0))
        return R{std::string("spacing must be positive")};
    if (!finite(p.wavelength) || !(p.wavelength > 0.0))
        return R{std::string("wavelength must be positive")};
    if (!finite(p.amplitude) || p.amplitude < 0.0)
        return R{std::string("amplitude must not be negative")};
    if (!(p.spacingVariation >= 0.0 && p.spacingVariation <= 1.0))
        return R{std::string("spacing variation must be within 0 and 1")};
    if (!(p.noise >= 0.0 && p.noise <= 1.0)) return R{std::string("noise must be within 0 and 1")};

    const double turns = p.angle / kTwoPi;
    const double sa = detail::sinTurns(turns);
    const double ca = detail::sinTurns(turns + 0.25);
    const Vec2 centre = region.centre();
    // Half extents of the region in the rotated frame.
    const double hx = (std::fabs(w * ca) + std::fabs(h * sa)) * 0.5;
    const double hy = (std::fabs(w * sa) + std::fabs(h * ca)) * 0.5;

    const bool wavy = p.amplitude > 0.0;
    const double step = wavy ? p.wavelength / kSamplesPerWave : 2.0 * hx;
    const double yLimit = hy + p.amplitude;
    // Cheap bound before generating: lines (at the minimum possible pitch) times samples.
    const double minPitch = p.maxSpacing * (1.0 - 0.9 * p.spacingVariation);
    const double estLines = (2.0 * yLimit) / minPitch + 2.0;
    const double estSamples = wavy ? (2.0 * hx) / step + 2.0 : 2.0;
    if (estLines * estSamples > 4.0 * static_cast<double>(kVectorTextureMaxVertices))
        return R{std::string("texture would have too many vertices; increase spacing or wavelength")};

    std::vector<Polyline> out;
    std::size_t total = 0;
    double y = -yLimit;
    for (std::uint64_t line = 0; y <= yLimit; ++line) {
        const double phase = p.noise * unit(p.seed, line, 0, 1);
        std::vector<Vec2> pts;
        if (wavy) {
            const auto n = static_cast<std::uint64_t>(std::ceil((2.0 * hx) / step)) + 1;
            total += static_cast<std::size_t>(n);
            if (total > kVectorTextureMaxVertices)
                return R{std::string("texture would have too many vertices; increase spacing or wavelength")};
            pts.reserve(static_cast<std::size_t>(n));
            for (std::uint64_t k = 0; k < n; ++k) {
                const double x = std::min(-hx + static_cast<double>(k) * step, hx);
                const double cyc = x / p.wavelength + phase;
                const double wave = std::floor(cyc);
                const double amp =
                    p.amplitude *
                    (1.0 - p.noise * unit(p.seed, line,
                                          static_cast<std::uint64_t>(static_cast<std::int64_t>(wave)), 2));
                pts.push_back({x, y + amp * detail::sinTurns(cyc)});
            }
        } else {
            total += 2;
            pts = {{-hx, y}, {hx, y}};
        }
        // Rotate to world and clip, splitting where the line leaves the region.
        Polyline run;
        run.closed = false;
        auto flush = [&] {
            if (run.points.size() >= 2) out.push_back(std::move(run));
            run = Polyline{};
        };
        Vec2 prev{};
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2 cur{centre.x + pts[i].x * ca - pts[i].y * sa,
                           centre.y + pts[i].x * sa + pts[i].y * ca};
            if (i > 0) {
                double t0 = 0.0;
                double t1 = 1.0;
                if (clipSegment(region, prev, cur, t0, t1)) {
                    const Vec2 d{cur.x - prev.x, cur.y - prev.y};
                    // Clipped ends are clamped to the box: rounding must not leave it.
                    auto at = [&](double t) {
                        return Vec2{std::clamp(prev.x + d.x * t, region.min.x, region.max.x),
                                    std::clamp(prev.y + d.y * t, region.min.y, region.max.y)};
                    };
                    if (run.points.empty()) run.points.push_back(t0 == 0.0 ? prev : at(t0));
                    run.points.push_back(t1 == 1.0 ? cur : at(t1));
                    if (t1 < 1.0) flush();
                } else {
                    flush();
                }
            }
            prev = cur;
        }
        flush();
        const double u = unit(p.seed, line, 0, 3);
        y += p.maxSpacing * (1.0 - 0.9 * p.spacingVariation * u);
    }
    return out;
}

ShapeResult<std::vector<Contour>> generateVectorTextureContours(const Box2& region,
                                                                const VectorTextureParams& p) {
    auto r = generateVectorTexture(region, p);
    if (!r.ok()) return ShapeResult<std::vector<Contour>>{r.error()};
    std::vector<Contour> cs;
    cs.reserve(r.value().size());
    for (const Polyline& pl : r.value()) cs.push_back(Contour::fromPolyline(pl));
    return cs;
}

}  // namespace carvetto::geometry
