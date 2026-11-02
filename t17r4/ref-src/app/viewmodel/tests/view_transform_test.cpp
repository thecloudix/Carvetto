#include "carvetto/viewmodel/view_transform.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <random>

using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;
using carvetto::viewmodel::ViewTransform;
using Catch::Matchers::WithinAbs;

TEST_CASE("AC1: world->screen->world round-trips within 1e-9 mm", "[viewmodel][view]") {
    std::mt19937 rng(20261024);
    std::uniform_real_distribution<double> logScale(std::log(ViewTransform::kMinScale),
                                                    std::log(ViewTransform::kMaxScale));
    std::uniform_real_distribution<double> world(-5000.0, 5000.0);
    std::uniform_real_distribution<double> originPx(-20000.0, 20000.0);
    for (int i = 0; i < 2000; ++i) {
        ViewTransform view;
        view.setViewport(1000.0, 800.0);
        view.set(std::exp(logScale(rng)), {originPx(rng), originPx(rng)});
        const Vec2 p{world(rng), world(rng)};
        const Vec2 back = view.screenToWorld(view.worldToScreen(p));
        REQUIRE(std::fabs(back.x - p.x) <= 1e-9);
        REQUIRE(std::fabs(back.y - p.y) <= 1e-9);
    }
}

TEST_CASE("screen Y points down while world Y points up", "[viewmodel][view]") {
    ViewTransform view;
    view.setViewport(100.0, 100.0);
    view.set(2.0, {10.0, 90.0});
    const Vec2 s = view.worldToScreen({5.0, 5.0});
    CHECK_THAT(s.x, WithinAbs(20.0, 1e-12));
    CHECK_THAT(s.y, WithinAbs(80.0, 1e-12));
}

TEST_CASE("AC1: zooming about a cursor keeps the world point under it", "[viewmodel][view]") {
    std::mt19937 rng(7);
    std::uniform_real_distribution<double> px(0.0, 1000.0);
    std::uniform_real_distribution<double> factor(0.2, 5.0);
    ViewTransform view;
    view.setViewport(1000.0, 800.0);
    view.zoomToFit(Box2{{0.0, 0.0}, {254.0, 254.0}});
    for (int i = 0; i < 500; ++i) {
        const Vec2 cursor{px(rng), px(rng) * 0.8};
        const Vec2 before = view.screenToWorld(cursor);
        view.zoomAbout(cursor, factor(rng));
        const Vec2 after = view.screenToWorld(cursor);
        REQUIRE(std::fabs(after.x - before.x) <= 1e-9);
        REQUIRE(std::fabs(after.y - before.y) <= 1e-9);
        const Vec2 screen = view.worldToScreen(before);
        REQUIRE(std::fabs(screen.x - cursor.x) <= 1e-6);
        REQUIRE(std::fabs(screen.y - cursor.y) <= 1e-6);
        REQUIRE(view.scale() >= ViewTransform::kMinScale);
        REQUIRE(view.scale() <= ViewTransform::kMaxScale);
    }
}

TEST_CASE("zoom is clamped and the anchor still holds at the limit", "[viewmodel][view]") {
    ViewTransform view;
    view.setViewport(400.0, 300.0);
    view.set(500.0, {0.0, 0.0});
    const Vec2 cursor{123.0, 45.0};
    const Vec2 w = view.screenToWorld(cursor);
    view.zoomAbout(cursor, 100.0);
    CHECK(view.scale() == ViewTransform::kMaxScale);
    CHECK_THAT(view.screenToWorld(cursor).x, WithinAbs(w.x, 1e-9));
    view.zoomAbout(cursor, 1e-9);
    CHECK(view.scale() == ViewTransform::kMinScale);
    view.zoomAbout(cursor, 0.0);   // ignored
    view.zoomAbout(cursor, -2.0);  // ignored
    CHECK(view.scale() == ViewTransform::kMinScale);
}

TEST_CASE("panning moves the content with the mouse", "[viewmodel][view]") {
    ViewTransform view;
    view.setViewport(400.0, 300.0);
    view.set(3.0, {50.0, 250.0});
    const Vec2 p{10.0, 20.0};
    const Vec2 s0 = view.worldToScreen(p);
    view.pan({15.0, -7.0});
    const Vec2 s1 = view.worldToScreen(p);
    CHECK_THAT(s1.x - s0.x, WithinAbs(15.0, 1e-12));
    CHECK_THAT(s1.y - s0.y, WithinAbs(-7.0, 1e-12));
    CHECK(view.scale() == 3.0);
}

TEST_CASE("AC1: zoom-to-fit 254x254 mm in 1000x800 px with 5% margin fits exactly on Y",
          "[viewmodel][view]") {
    ViewTransform view;
    view.setViewport(1000.0, 800.0);
    const Box2 job{{0.0, 0.0}, {254.0, 254.0}};
    REQUIRE(view.zoomToFit(job, 0.05));
    // Limiting axis is Y: 800 px minus 5% per side = 720 px for 254 mm.
    CHECK_THAT(view.scale(), WithinAbs(720.0 / 254.0, 1e-12));
    const Box2 s = view.worldToScreen(job);
    CHECK_THAT(s.height(), WithinAbs(720.0, 1e-9));
    CHECK_THAT(s.min.y, WithinAbs(40.0, 1e-9));
    CHECK_THAT(s.max.y, WithinAbs(760.0, 1e-9));
    // The other axis is centred and inside the margin.
    CHECK_THAT(s.width(), WithinAbs(720.0, 1e-9));
    CHECK_THAT(s.min.x, WithinAbs(140.0, 1e-9));
    CHECK_THAT(s.max.x, WithinAbs(860.0, 1e-9));
}

TEST_CASE("zoom-to-fit picks X when the box is wide and handles an offset box", "[viewmodel][view]") {
    ViewTransform view;
    view.setViewport(1000.0, 800.0);
    const Box2 wide{{-600.0, 100.0}, {400.0, 200.0}};
    REQUIRE(view.zoomToFit(wide, 0.05));
    const Box2 s = view.worldToScreen(wide);
    CHECK_THAT(s.min.x, WithinAbs(50.0, 1e-9));
    CHECK_THAT(s.max.x, WithinAbs(950.0, 1e-9));
    CHECK_THAT(s.centre().y, WithinAbs(400.0, 1e-9));
}

TEST_CASE("zoom-to-fit refuses empty input", "[viewmodel][view]") {
    ViewTransform view;
    CHECK_FALSE(view.zoomToFit(Box2{{0.0, 0.0}, {10.0, 10.0}}));  // zero viewport
    view.setViewport(100.0, 100.0);
    CHECK_FALSE(view.zoomToFit(Box2{}));
    CHECK_FALSE(view.zoomToFit(Box2{{5.0, 5.0}, {5.0, 5.0}}));
    CHECK(view.zoomToFit(Box2{{0.0, 5.0}, {10.0, 5.0}}));  // a line still fits on its axis
}

TEST_CASE("visibleWorld covers the viewport", "[viewmodel][view]") {
    ViewTransform view;
    view.setViewport(1000.0, 800.0);
    view.zoomToFit(Box2{{0.0, 0.0}, {254.0, 254.0}});
    const Box2 w = view.visibleWorld();
    CHECK_THAT(w.min.x, WithinAbs(view.screenToWorld({0.0, 0.0}).x, 1e-12));
    CHECK_THAT(w.max.y, WithinAbs(view.screenToWorld({0.0, 0.0}).y, 1e-12));
    CHECK_THAT(w.min.y, WithinAbs(view.screenToWorld({0.0, 800.0}).y, 1e-12));
    CHECK(w.contains(Box2{{0.0, 0.0}, {254.0, 254.0}}));
}
