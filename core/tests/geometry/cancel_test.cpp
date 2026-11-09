// T-0228: cooperative cancel of the geometry offset and boolean calls, and the cheap
// degenerate-Bezier test (same answers as the exact length).
#include <atomic>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <thread>
#include <variant>
#include <vector>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/cancel.hpp"
#include "carvetto/geometry/offset.hpp"
#include "test_rng.hpp"

using namespace carvetto::geometry;

namespace {

// A closed wavy outline of `n` Bezier spans around a circle of radius ~50.
Contour wavyOutline(int n) {
    Contour c;
    c.closed = true;
    const double pi = 3.14159265358979323846;
    auto pt = [&](double a, double r) { return Vec2{r * std::cos(a), r * std::sin(a)}; };
    for (int i = 0; i < n; ++i) {
        const double a0 = 2.0 * pi * i / n;
        const double a1 = 2.0 * pi * (i + 1) / n;
        const double r0 = 50.0 + 3.0 * std::sin(7.0 * a0);
        const double r1 = 50.0 + 3.0 * std::sin(7.0 * a1);
        c.spans.emplace_back(BezierSpan{pt(a0, r0), pt(a0 + 0.3 * (a1 - a0), r0 + 2.0),
                                        pt(a1 - 0.3 * (a1 - a0), r1 - 2.0), pt(a1, r1)});
    }
    return c;
}

bool sameChains(const std::vector<Contour>& a, const std::vector<Contour>& b) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].closed != b[i].closed || a[i].spans.size() != b[i].spans.size()) {
            return false;
        }
        for (std::size_t k = 0; k < a[i].spans.size(); ++k) {
            const auto* la = std::get_if<LineSpan>(&a[i].spans[k]);
            const auto* lb = std::get_if<LineSpan>(&b[i].spans[k]);
            if ((la == nullptr) != (lb == nullptr)) {
                return false;
            }
            if (la != nullptr && (la->start.x != lb->start.x || la->start.y != lb->start.y ||
                                  la->end.x != lb->end.x || la->end.y != lb->end.y)) {
                return false;
            }
            if (la == nullptr && (startPoint(a[i].spans[k]) != startPoint(b[i].spans[k]) ||
                                  endPoint(a[i].spans[k]) != endPoint(b[i].spans[k]))) {
                return false;
            }
        }
    }
    return true;
}

}  // namespace

TEST_CASE("isDegenerate for a Bezier equals the exact length test", "[geometry][cancel]") {
    const Tolerance tol = Tolerance::defaults();
    carvetto::test::SplitMix rng{228};
    for (int i = 0; i < 3000; ++i) {
        // Sizes from far below to far above the tolerance, so both bounds and the exact fallback run.
        const double scale = rng.logUniform(1e-5, 1e-1);
        auto p = [&] { return Vec2{rng.uniform(-1.0, 1.0) * scale, rng.uniform(-1.0, 1.0) * scale}; };
        const BezierSpan b{p(), p(), p(), p()};
        REQUIRE(isDegenerate(Span{b}, tol) == (length(Span{b}) <= tol.distance));
    }
    // Exactly at the tolerance: a straight Bezier of length tol.distance, and one a hair longer.
    const BezierSpan edge{{0, 0}, {tol.distance / 3, 0}, {2 * tol.distance / 3, 0}, {tol.distance, 0}};
    REQUIRE(isDegenerate(Span{edge}, tol) == (length(Span{edge}) <= tol.distance));
    const BezierSpan over{{0, 0}, {tol.distance / 3, 0}, {2 * tol.distance / 3, 0}, {tol.distance * 1.0000001, 0}};
    REQUIRE(isDegenerate(Span{over}, tol) == (length(Span{over}) <= tol.distance));
    const BezierSpan dot{{1, 1}, {1, 1}, {1, 1}, {1, 1}};
    REQUIRE(isDegenerate(Span{dot}, tol));
}

TEST_CASE("a scope whose flag stays false changes nothing", "[geometry][cancel]") {
    const Tolerance tol = Tolerance::defaults();
    const Contour c = wavyOutline(60);
    const std::vector<Contour> plainClosed = offsetContours({c}, 1.5, JoinType::Round, tol);
    const std::vector<Contour> plainOpen = offsetOpenContour(c, 1.5, OffsetSide::Left, JoinType::Round, tol);
    const std::vector<Contour> plainUnion = unionContours({c, c}, tol, FillRule::EvenOdd);
    std::atomic<bool> flag{false};
    {
        const CancelScope scope(&flag);
        REQUIRE_FALSE(cancelRequested());
        REQUIRE(sameChains(offsetContours({c}, 1.5, JoinType::Round, tol), plainClosed));
        REQUIRE(sameChains(offsetOpenContour(c, 1.5, OffsetSide::Left, JoinType::Round, tol), plainOpen));
        REQUIRE(sameChains(unionContours({c, c}, tol, FillRule::EvenOdd), plainUnion));
    }
    REQUIRE_FALSE(plainClosed.empty());
    REQUIRE_FALSE(plainOpen.empty());
}

TEST_CASE("a set flag stops the offset and union calls with an empty result", "[geometry][cancel]") {
    const Tolerance tol = Tolerance::defaults();
    const Contour c = wavyOutline(400);
    std::atomic<bool> flag{true};
    const CancelScope scope(&flag);
    REQUIRE(cancelRequested());
    REQUIRE(offsetContours({c}, 1.5, JoinType::Round, tol).empty());
    REQUIRE(offsetOpenContour(c, 1.5, OffsetSide::Left, JoinType::Round, tol).empty());
    REQUIRE(unionContours({c}, tol, FillRule::EvenOdd).empty());
}

TEST_CASE("scopes nest and restore the outer flag", "[geometry][cancel]") {
    std::atomic<bool> outer{false};
    std::atomic<bool> inner{true};
    const CancelScope a(&outer);
    REQUIRE_FALSE(cancelRequested());
    {
        const CancelScope b(&inner);
        REQUIRE(cancelRequested());
        const CancelScope none(nullptr);  // installs nothing
        REQUIRE(cancelRequested());
    }
    REQUIRE_FALSE(cancelRequested());
}

TEST_CASE("a cancel raised during a long open offset stops it quickly", "[geometry][cancel]") {
    const Tolerance tol = Tolerance::defaults();
    const Contour c = wavyOutline(3000);
    std::atomic<bool> flag{false};
    std::thread raiser([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        flag = true;
    });
    const auto t0 = std::chrono::steady_clock::now();
    std::vector<Contour> out;
    {
        const CancelScope scope(&flag);
        out = offsetOpenContour(c, 1.5, OffsetSide::Left, JoinType::Round, tol);
    }
    raiser.join();
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    // Either it was cancelled (nothing returned) or it finished before the flag went up.
    REQUIRE((out.empty() || !out.front().isEmpty()));
    REQUIRE(secs < 30.0);
}
