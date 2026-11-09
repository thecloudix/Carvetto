#include "carvetto/geometry/vector_texture.hpp"

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <cstdint>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"

using namespace carvetto::geometry;
using Catch::Matchers::WithinAbs;

namespace {

const Box2 kRegion{{0.0, 0.0}, {100.0, 60.0}};

std::vector<Polyline> make(const Box2& r, const VectorTextureParams& p) {
    auto res = generateVectorTexture(r, p);
    INFO((res.ok() ? std::string() : res.error()));
    REQUIRE(res.ok());
    return res.value();
}

VectorTextureParams base() {
    VectorTextureParams p;
    p.maxSpacing = 5.0;
    p.wavelength = 10.0;
    p.amplitude = 1.0;
    return p;
}

std::size_t vertexTotal(const std::vector<Polyline>& v) {
    std::size_t n = 0;
    for (const auto& pl : v) n += pl.points.size();
    return n;
}

}  // namespace

TEST_CASE("splitmix64 matches the published reference outputs", "[texture]") {
    // First outputs of SplitMix64 seeded with 0 (state advances by the golden gamma each call).
    REQUIRE(detail::splitmix64(0) == 0xE220A8397B1DCDAFULL);
    REQUIRE(detail::splitmix64(0x9E3779B97F4A7C15ULL) == 0x6E789E6AA1B965F4ULL);
}

TEST_CASE("sinTurns is accurate and exactly periodic", "[texture]") {
    for (int i = -400; i <= 400; ++i) {
        const double t = i / 97.0;
        REQUIRE_THAT(detail::sinTurns(t), WithinAbs(std::sin(kTwoPi * (t - std::floor(t))), 1e-11));
    }
    REQUIRE(detail::sinTurns(0.0) == 0.0);
    REQUIRE_THAT(detail::sinTurns(0.25), WithinAbs(1.0, 1e-12));
    REQUIRE_THAT(detail::sinTurns(0.75), WithinAbs(-1.0, 1e-12));
    REQUIRE(detail::sinTurns(2.375) == detail::sinTurns(0.375));
}

TEST_CASE("same seed gives identical output, different seeds differ", "[texture]") {
    VectorTextureParams p = base();
    p.angle = 0.3;
    p.spacingVariation = 0.5;
    p.noise = 0.7;
    p.seed = 42;
    const auto a = make(kRegion, p);
    const auto b = make(kRegion, p);
    REQUIRE(a == b);
    p.seed = 43;
    REQUIRE_FALSE(a == make(kRegion, p));
}

TEST_CASE("golden vertex pins the platform-stable arithmetic", "[texture]") {
    VectorTextureParams p = base();
    p.noise = 0.5;
    p.spacingVariation = 0.3;
    p.seed = 7;
    const auto v = make(kRegion, p);
    REQUIRE(v.size() > 5);
    // Values (to 1e-12, bit-exact on the recording platform) recorded from this implementation; a change means the output changed.
    const Vec2 q = v[3].points[5];
    REQUIRE_THAT(q.x, WithinAbs(2.0833333333290796, 1e-12));
    REQUIRE_THAT(q.y, WithinAbs(15.099258808269781, 1e-12));
}

TEST_CASE("spacing is exact without variation and bounded with it", "[texture]") {
    VectorTextureParams p = base();
    p.amplitude = 0.0;
    p.maxSpacing = 4.0;
    auto v = make(kRegion, p);
    REQUIRE(v.size() >= 14);
    for (std::size_t i = 1; i < v.size(); ++i)
        REQUIRE_THAT(v[i].points[0].y - v[i - 1].points[0].y, WithinAbs(4.0, 1e-9));
    for (const auto& pl : v) {
        REQUIRE(pl.points.size() == 2);
        REQUIRE_THAT(pl.points[0].x, WithinAbs(0.0, 1e-9));
        REQUIRE_THAT(pl.points[1].x, WithinAbs(100.0, 1e-9));
    }

    p.spacingVariation = 1.0;
    p.seed = 9;
    v = make(kRegion, p);
    double lo = 1e9;
    double hi = 0.0;
    for (std::size_t i = 1; i < v.size(); ++i) {
        const double d = v[i].points[0].y - v[i - 1].points[0].y;
        lo = std::min(lo, d);
        hi = std::max(hi, d);
    }
    REQUIRE(hi <= 4.0 + 1e-9);
    REQUIRE(lo >= 0.4 - 1e-9);
    REQUIRE(hi - lo > 1.0);  // varies
}

TEST_CASE("rotated lines keep their spacing", "[texture]") {
    VectorTextureParams p = base();
    p.amplitude = 0.0;
    p.angle = 0.6;
    p.maxSpacing = 3.0;
    const auto v = make(kRegion, p);
    // Distance from line i's start point to the neighbouring line (normal direction).
    const double nx = -std::sin(0.6);
    const double ny = std::cos(0.6);
    std::vector<double> offs;
    for (const auto& pl : v) offs.push_back(pl.points[0].x * nx + pl.points[0].y * ny);
    std::sort(offs.begin(), offs.end());
    for (std::size_t i = 1; i < offs.size(); ++i)
        REQUIRE_THAT(offs[i] - offs[i - 1], WithinAbs(3.0, 0.03));
    for (const auto& pl : v)
        for (const Vec2& q : pl.points) {
            REQUIRE(q.x >= -1e-9);
            REQUIRE(q.x <= 100.0 + 1e-9);
            REQUIRE(q.y >= -1e-9);
            REQUIRE(q.y <= 60.0 + 1e-9);
        }
}

TEST_CASE("wavelength and amplitude are within 1 percent", "[texture]") {
    VectorTextureParams p = base();
    p.amplitude = 2.0;
    p.wavelength = 12.0;
    p.maxSpacing = 20.0;
    const Box2 big{{0.0, 0.0}, {240.0, 200.0}};
    const auto v = make(big, p);
    REQUIRE(v.size() >= 8);
    // A line fully inside the region: take the one with the most vertices.
    const Polyline* line = &v[0];
    for (const auto& pl : v)
        if (pl.points.size() > line->points.size()) line = &pl;
    REQUIRE(line->points.size() > 60);
    double ymid = 0.0;
    for (const Vec2& q : line->points) ymid += q.y;
    ymid /= static_cast<double>(line->points.size());
    double peak = 0.0;
    std::vector<double> upCross;
    for (std::size_t i = 0; i < line->points.size(); ++i) {
        peak = std::max(peak, std::fabs(line->points[i].y - ymid));
        if (i > 0 && line->points[i - 1].y < ymid && line->points[i].y >= ymid) {
            const Vec2 a = line->points[i - 1];
            const Vec2 b = line->points[i];
            upCross.push_back(a.x + (ymid - a.y) / (b.y - a.y) * (b.x - a.x));
        }
    }
    REQUIRE_THAT(peak, WithinAbs(2.0, 0.02));
    REQUIRE(upCross.size() >= 10);
    const double wl = (upCross.back() - upCross.front()) / static_cast<double>(upCross.size() - 1);
    REQUIRE_THAT(wl, WithinAbs(12.0, 0.12));
}

TEST_CASE("noise keeps amplitude a maximum and changes the lines", "[texture]") {
    VectorTextureParams p = base();
    p.amplitude = 2.0;
    p.maxSpacing = 20.0;
    p.noise = 1.0;
    p.seed = 5;
    const Box2 big{{0.0, 0.0}, {240.0, 200.0}};
    const auto noisy = make(big, p);
    for (const auto& pl : noisy) {
        // y deviation from the line's own mean never exceeds amplitude (plus mean wobble).
        double lo = 1e9;
        double hi = -1e9;
        for (const Vec2& q : pl.points) {
            lo = std::min(lo, q.y);
            hi = std::max(hi, q.y);
        }
        REQUIRE(hi - lo <= 4.0 + 1e-6);
    }
    p.noise = 0.0;
    REQUIRE_FALSE(noisy == make(big, p));
}

TEST_CASE("lines are clipped to the region and split on re-entry", "[texture]") {
    VectorTextureParams p = base();
    p.amplitude = 6.0;  // big waves crossing the region edge
    p.wavelength = 20.0;
    p.maxSpacing = 4.0;
    const Box2 r{{0.0, 0.0}, {60.0, 10.0}};
    const auto v = make(r, p);
    for (const auto& pl : v) {
        REQUIRE(pl.points.size() >= 2);
        REQUIRE_FALSE(pl.closed);
        for (const Vec2& q : pl.points) {
            REQUIRE(q.x >= r.min.x);
            REQUIRE(q.x <= r.max.x);
            REQUIRE(q.y >= r.min.y);
            REQUIRE(q.y <= r.max.y);
        }
    }
    // More polylines than rows would be without splitting: waves leave and re-enter.
    REQUIRE(v.size() > 3);
}

TEST_CASE("invalid parameters and oversize requests are errors", "[texture]") {
    const VectorTextureParams good = base();
    auto bad = [&](auto mutate) {
        VectorTextureParams p = good;
        mutate(p);
        return !generateVectorTexture(kRegion, p).ok();
    };
    REQUIRE(bad([](auto& p) { p.maxSpacing = 0.0; }));
    REQUIRE(bad([](auto& p) { p.maxSpacing = std::nan(""); }));
    REQUIRE(bad([](auto& p) { p.wavelength = -1.0; }));
    REQUIRE(bad([](auto& p) { p.amplitude = -1.0; }));
    REQUIRE(bad([](auto& p) { p.angle = 2.0; }));
    REQUIRE(bad([](auto& p) { p.noise = 1.5; }));
    REQUIRE(bad([](auto& p) { p.spacingVariation = -0.1; }));
    REQUIRE_FALSE(generateVectorTexture(Box2{}, good).ok());
    REQUIRE_FALSE(generateVectorTexture(Box2{{0, 0}, {10, 0}}, good).ok());
    VectorTextureParams huge = good;
    huge.maxSpacing = 0.001;
    huge.wavelength = 0.001;
    REQUIRE_FALSE(generateVectorTexture(Box2{{0, 0}, {1000, 1000}}, huge).ok());
    // Just under the limit succeeds and stays bounded.
    VectorTextureParams dense = good;
    dense.maxSpacing = 1.0;
    dense.wavelength = 2.0;
    const auto v = make(Box2{{0, 0}, {200, 100}}, dense);
    REQUIRE(vertexTotal(v) <= kVectorTextureMaxVertices);
}

TEST_CASE("texture contours add as one undoable object", "[texture]") {
    using namespace carvetto::document;
    Document doc;
    const auto cs = generateVectorTextureContours(kRegion, base());
    REQUIRE(cs.ok());
    REQUIRE_FALSE(cs.value().empty());
    for (const Contour& c : cs.value()) {
        REQUIRE_FALSE(c.closed);
        REQUIRE_FALSE(c.spans.empty());
    }
    const std::size_t before = doc.objectCount();
    doc.execute(std::make_unique<AddObjectCommand>(
        Object::vector(cs.value(), doc.activeLayer(), doc.activeSheet())));
    REQUIRE(doc.objectCount() == before + 1);
    doc.undo();
    REQUIRE(doc.objectCount() == before);
    doc.redo();
    REQUIRE(doc.objectCount() == before + 1);
}
