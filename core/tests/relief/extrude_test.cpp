#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/extrude.hpp"

using namespace carvetto::relief;
using Catch::Approx;

namespace {
Grid g11() { return Grid{11, 11, 1.0, {0.0, 0.0}}; }
Grid g21() { return Grid{21, 21, 0.5, {0.0, 0.0}}; }

ExtrudeRail rail(std::vector<Point> p, bool closed = false) { return ExtrudeRail{std::move(p), closed}; }

ExtrudeSection sec(std::vector<Point> prof, double t = 0.0, bool smooth = false) {
    return ExtrudeSection{std::move(prof), t, smooth};
}

// Rail y=5 from x=1 to x=9, profile of the given points (u 0..2 centred on the rail).
ExtrudeParams straight(std::vector<Point> profile) {
    ExtrudeParams p;
    p.rails.push_back(rail({{1, 5}, {9, 5}}));
    p.sections.push_back(sec(std::move(profile)));
    return p;
}

HeightField run(const ExtrudeParams& p, const Grid& g = g11()) {
    HeightField out;
    REQUIRE(extrude(p, g, nullptr, out) == ExtrudeStatus::Ok);
    return out;
}

bool same(const HeightField& a, const HeightField& b) {
    return a.data().size() == b.data().size() &&
           std::memcmp(a.data().data(), b.data().data(), a.data().size() * sizeof(float)) == 0;
}

// Component of nx x ny cells (cell 1) with z = value(i, j).
template <class F>
HeightField comp(std::size_t nx, std::size_t ny, F value) {
    HeightField f;
    REQUIRE(HeightField::create(Grid{nx, ny, 1.0, {0.0, 0.0}}, 0.0f, f) == Status::Ok);
    for (std::size_t j = 0; j < ny; ++j)
        for (std::size_t i = 0; i < nx; ++i) f.set(i, j, static_cast<float>(value(i, j)));
    return f;
}
}  // namespace

TEST_CASE("extrude: rectangular section on a straight rail is the analytic prism") {
    const HeightField f = run(straight({{0, 2}, {2, 2}}));
    for (std::size_t j = 0; j < 11; ++j)
        for (std::size_t i = 0; i < 11; ++i) {
            INFO("cell " << i << "," << j);
            if (i >= 1 && i <= 9 && j >= 4 && j <= 6)
                CHECK(f.at(i, j) == Approx(2.0f).margin(1e-5));
            else
                CHECK(std::isnan(f.at(i, j)));
        }
}

TEST_CASE("extrude: ramp section: left of travel is higher; u is centred on the rail") {
    const HeightField f = run(straight({{10, 0}, {12, 2}}));  // u offset by 10: only the width matters
    for (std::size_t i = 1; i <= 9; ++i) {
        CHECK(f.at(i, 4) == Approx(0.0f).margin(1e-5));
        CHECK(f.at(i, 5) == Approx(1.0f).margin(1e-5));
        CHECK(f.at(i, 6) == Approx(2.0f).margin(1e-5));
    }
    ExtrudeParams q;
    q.rails.push_back(rail({{9, 5}, {1, 5}}));  // reversed travel: left is -y
    q.sections.push_back(sec({{0, 0}, {2, 2}}));
    const HeightField g = run(q);
    CHECK(g.at(4, 4) == Approx(2.0f).margin(1e-5));
    CHECK(g.at(4, 6) == Approx(0.0f).margin(1e-5));
}

TEST_CASE("extrude: sections blend between spans or hold") {
    ExtrudeParams p;
    p.rails.push_back(rail({{1, 5}, {9, 5}}));
    p.sections.push_back(sec({{0, 1}, {2, 1}}, 0.0));
    p.sections.push_back(sec({{0, 3}, {2, 3}}, 1.0));
    HeightField f = run(p);
    for (std::size_t i = 1; i <= 9; ++i) CHECK(f.at(i, 5) == Approx(1.0 + (static_cast<double>(i) - 1.0) / 4.0).margin(1e-5));

    p.sweepBetweenSpans = false;  // each section holds until the next; the last one only at its own key
    p.sections.insert(p.sections.begin() + 1, sec({{0, 3}, {2, 3}}, 0.5));
    p.sections.back().profile = {{0, 5}, {2, 5}};
    f = run(p);
    for (std::size_t i = 1; i <= 4; ++i) CHECK(f.at(i, 5) == Approx(1.0f).margin(1e-5));
    CHECK(f.at(5, 5) == Approx(3.0f).margin(1e-5));  // the step edge takes the higher side
    for (std::size_t i = 6; i <= 9; ++i) CHECK(f.at(i, 5) == Approx(3.0f).margin(1e-5));

    // Sections away from the ends: anchors hold the first/last profile outside them.
    ExtrudeParams q;
    q.rails.push_back(rail({{1, 5}, {9, 5}}));
    q.sections.push_back(sec({{0, 1}, {2, 1}}, 0.25));
    q.sections.push_back(sec({{0, 3}, {2, 3}}, 0.75));
    f = run(q);
    CHECK(f.at(1, 5) == Approx(1.0f).margin(1e-5));
    CHECK(f.at(3, 5) == Approx(1.0f).margin(1e-5));
    CHECK(f.at(5, 5) == Approx(2.0f).margin(1e-5));
    CHECK(f.at(7, 5) == Approx(3.0f).margin(1e-5));
    CHECK(f.at(9, 5) == Approx(3.0f).margin(1e-5));
}

TEST_CASE("extrude: sharp corners mitre the outside, round corners do not reach it") {
    ExtrudeParams p;
    p.rails.push_back(rail({{2, 2}, {8, 2}, {8, 8}}));
    p.sections.push_back(sec({{0, 1}, {2, 1}}));
    HeightField f = run(p);  // round
    CHECK(std::isnan(f.at(9, 1)));
    CHECK(f.at(8, 2) == Approx(1.0f));
    CHECK(f.at(9, 2) == Approx(1.0f));
    CHECK(f.at(8, 1) == Approx(1.0f));
    CHECK(f.at(7, 3) == Approx(1.0f));  // inner corner
    p.sharpCorners = true;
    f = run(p);
    CHECK(f.at(9, 1) == Approx(1.0f));
    CHECK(f.at(8, 2) == Approx(1.0f));
    CHECK(std::isnan(f.at(10, 1)));
    CHECK(std::isnan(f.at(9, 0)));
}

TEST_CASE("extrude: a very sharp corner is bevelled, not spiked") {
    ExtrudeParams p;
    p.rails.push_back(rail({{1, 5}, {9, 5}, {1, 5.2}}));
    p.sections.push_back(sec({{0, 1}, {2, 1}}));
    p.sharpCorners = true;
    const HeightField f = run(p);
    // A true mitre would reach ~ x = 9 + 1/tan(small angle); the bevel stays near the vertex.
    CHECK(std::isnan(f.at(10, 5)));
}

TEST_CASE("extrude: closed rail ring") {
    ExtrudeParams p;
    p.rails.push_back(rail({{2, 2}, {8, 2}, {8, 8}, {2, 8}}, true));
    p.sections.push_back(sec({{0, 1}, {2, 1}}));
    p.sharpCorners = true;
    const HeightField f = run(p);
    CHECK(std::isnan(f.at(5, 5)));
    CHECK(f.at(2, 5) == Approx(1.0f));
    CHECK(f.at(5, 8) == Approx(1.0f));
    CHECK(f.at(1, 1) == Approx(1.0f));
    CHECK(f.at(9, 9) == Approx(1.0f));
    CHECK(f.at(1, 9) == Approx(1.0f));
    CHECK(std::isnan(f.at(0, 0)));
    CHECK(std::isnan(f.at(4, 5)));
}

TEST_CASE("extrude: component stretched to the rail length") {
    ExtrudeParams p;
    p.source = ExtrudeSource::Component;
    p.rails.push_back(rail({{1, 5}, {9, 5}}));
    p.component = comp(5, 3, [](std::size_t i, std::size_t j) { return static_cast<double>(i) + 10.0 * static_cast<double>(j); });
    const HeightField f = run(p);
    // x along the rail maps 8 mm -> 4 mm; component y (0..2) maps to rail y 4..6.
    for (std::size_t i = 1; i <= 9; ++i)
        for (std::size_t j = 4; j <= 6; ++j)
            CHECK(f.at(i, j) == Approx((static_cast<double>(i) - 1.0) / 2.0 + 10.0 * static_cast<double>(j - 4)).margin(1e-4));
    CHECK(std::isnan(f.at(0, 5)));
    CHECK(std::isnan(f.at(5, 3)));
}

TEST_CASE("extrude: component copies with overlap and flips") {
    ExtrudeParams p;
    p.source = ExtrudeSource::Component;
    p.stretchToRail = false;
    p.rails.push_back(rail({{1, 5}, {9, 5}}));
    p.component = comp(5, 3, [](std::size_t i, std::size_t) { return static_cast<double>(i); });
    HeightField f = run(p);  // two copies of length 4, no overlap
    const double saw[] = {0, 1, 2, 3, 4, 1, 2, 3, 4};
    for (std::size_t i = 1; i <= 9; ++i) {
        INFO("i " << i);
        CHECK(f.at(i, 5) == Approx(i == 5 ? 4.0 : saw[i - 1]).margin(1e-5));
    }
    p.flipAlternate = true;
    f = run(p);  // second copy mirrored: 4 3 2 1 0
    const double flipped[] = {0, 1, 2, 3, 4, 3, 2, 1, 0};
    for (std::size_t i = 1; i <= 9; ++i) CHECK(f.at(i, 5) == Approx(flipped[i - 1]).margin(1e-5));
    p.flipAlternate = false;
    p.overlapPercent = 50.0;  // three copies at 0, 2, 4; higher value wins
    f = run(p);
    const double ov[] = {0, 1, 2, 2, 4, 3, 4, 3, 4};
    // x=1 ->0; x=2 ->1; x=3 -> max(2,0)=2; x=4 -> max(3,1)=3; x=5 -> max(4,2,0)=4; x=6 -> max(3,1)=3?
    (void)ov;
    CHECK(f.at(1, 5) == Approx(0.0f).margin(1e-5));
    CHECK(f.at(2, 5) == Approx(1.0f).margin(1e-5));
    CHECK(f.at(3, 5) == Approx(2.0f).margin(1e-5));
    CHECK(f.at(4, 5) == Approx(3.0f).margin(1e-5));
    CHECK(f.at(5, 5) == Approx(4.0f).margin(1e-5));
    CHECK(f.at(6, 5) == Approx(3.0f).margin(1e-5));
    CHECK(f.at(7, 5) == Approx(4.0f).margin(1e-5));
    CHECK(f.at(8, 5) == Approx(3.0f).margin(1e-5));
    CHECK(f.at(9, 5) == Approx(4.0f).margin(1e-5));
}

TEST_CASE("extrude: transparent component cells stay transparent") {
    ExtrudeParams p;
    p.source = ExtrudeSource::Component;
    p.rails.push_back(rail({{1, 5}, {9, 5}}));
    p.component = comp(5, 3, [](std::size_t, std::size_t j) { return j == 2 ? std::numeric_limits<double>::quiet_NaN() : 1.0; });
    const HeightField f = run(p);
    CHECK(f.at(5, 4) == Approx(1.0f));
    CHECK(std::isnan(f.at(5, 6)));
}

namespace {
// Three rails: A along y=5, B1 along x=3, B2 along x=7 on a 0.5 mm grid; triangle profile
// (0,0) (1,4) (2,0) so scale-shape and add-base differ off the crest.
ExtrudeParams weave3() {
    ExtrudeParams p;
    p.rails.push_back(rail({{0, 5}, {10, 5}}));
    p.rails.push_back(rail({{3, 0}, {3, 10}}));
    p.rails.push_back(rail({{7, 0}, {7, 10}}));
    p.sections.push_back(sec({{0, 0}, {1, 4}, {2, 0}}));
    p.weave.enabled = true;
    return p;
}
float at(const HeightField& f, double x, double y) { return f.at(static_cast<std::size_t>(x * 2), static_cast<std::size_t>(y * 2)); }
}  // namespace

TEST_CASE("weave: crossings alternate over and under") {
    ExtrudeParams p = weave3();
    HeightField f = run(p, g21());
    // A is over B1 (first crossing on A, first on B1) and under B2: at (3, 4.5) both strands read 2
    // (A: profile 2 at full height; B1: crest 4 at 50%), at (7, 4.5) B2 (over, crest 4) wins.
    CHECK(at(f, 3, 4.5) == Approx(2.0f).margin(1e-5));
    CHECK(at(f, 7, 4.5) == Approx(4.0f).margin(1e-5));
    CHECK(at(f, 3, 5.0) == Approx(4.0f).margin(1e-5));
    CHECK(at(f, 7, 5.0) == Approx(4.0f).margin(1e-5));
    // Away from the crossings the strands are untouched.
    CHECK(at(f, 3, 1.0) == Approx(4.0f).margin(1e-5));
    CHECK(at(f, 7, 9.0) == Approx(4.0f).margin(1e-5));
    CHECK(at(f, 0.5, 5.0) == Approx(4.0f).margin(1e-5));
    CHECK(at(f, 9.5, 5.0) == Approx(4.0f).margin(1e-5));
    // Swapping over/under flips every crossing.
    p.weave.startOver = false;
    f = run(p, g21());
    CHECK(at(f, 3, 4.5) == Approx(4.0f).margin(1e-5));
    CHECK(at(f, 7, 4.5) == Approx(2.0f).margin(1e-5));
}

TEST_CASE("weave: ramp between crossings, scale shape vs add base") {
    ExtrudeParams p = weave3();
    // A's level: 1 at x in [2,4] (over B1), 0.5 at [6,8] (under B2), linear between: 0.75 at x=5.
    HeightField f = run(p, g21());
    CHECK(at(f, 5, 4.5) == Approx(1.5f).margin(1e-5));  // 2 * 0.75
    CHECK(at(f, 4.5, 4.5) == Approx(2.0f * 0.875f).margin(1e-5));
    CHECK(at(f, 5, 5.0) == Approx(3.0f).margin(1e-5));  // crest 4 * 0.75
    p.weave.mode = WeaveMode::AddBase;
    f = run(p, g21());
    CHECK(at(f, 5, 4.5) == Approx(1.0f).margin(1e-5));  // 2 + (0.75 - 1) * 4
    CHECK(at(f, 5, 5.0) == Approx(3.0f).margin(1e-5));
    p.weave.zUnderPercent = 0.0;
    p.weave.mode = WeaveMode::ScaleShape;
    f = run(p, g21());
    CHECK(at(f, 5, 5.0) == Approx(2.0f).margin(1e-5));  // factor 0.5 at x=5
    p.weave.zOverPercent = 200.0;
    f = run(p, g21());
    CHECK(at(f, 3, 5.0) == Approx(8.0f).margin(1e-5));
}

TEST_CASE("weave: disabled leaves rails independent, a single rail has no crossings") {
    ExtrudeParams p = weave3();
    p.weave.enabled = false;
    const HeightField off = run(p, g21());
    CHECK(at(off, 3, 4.5) == Approx(4.0f).margin(1e-5));
    CHECK(at(off, 7, 4.5) == Approx(4.0f).margin(1e-5));
    ExtrudeParams one = weave3();
    one.rails.resize(1);
    ExtrudeParams one2 = one;
    one2.weave.enabled = false;
    CHECK(same(run(one, g21()), run(one2, g21())));
}

TEST_CASE("weave: a closed rail crossing itself, on a component source and deterministic") {
    ExtrudeParams p;
    p.rails.push_back(rail({{1, 1}, {9, 9}, {9, 1}, {1, 9}}, true));  // two crossings? one at (5,5)
    p.sections.push_back(sec({{0, 2}, {1, 2}}));
    p.weave.enabled = true;
    p.weave.zUnderPercent = 0.0;
    const HeightField a = run(p);
    const HeightField b = run(p);
    CHECK(same(a, b));
    p.weave.startOver = false;
    CHECK_FALSE(same(a, run(p)));
}

TEST_CASE("weave: alternation follows each rail, closed rail level wraps around the seam") {
    // Ring crossed by a line at (5,2) (ring over) and (5,8) (ring under). Ring level: 1 on [2,4],
    // 0 on [14,16] (period 24), linear between and across the seam.
    ExtrudeParams q;
    q.rails.push_back(rail({{2, 2}, {8, 2}, {8, 8}, {2, 8}}, true));
    q.rails.push_back(rail({{5, 0}, {5, 10}}));
    q.sections.push_back(sec({{0, 2}, {2, 2}}));
    q.weave.enabled = true;
    q.weave.zUnderPercent = 0.0;
    const HeightField w = run(q);
    CHECK(w.at(8, 5) == Approx(1.0f).margin(1e-5));  // s = 9: 1 * (14 - 9) / 10 = 0.5
    CHECK(w.at(2, 5) == Approx(1.0f).margin(1e-5));  // s = 21: (21 - 16) / 10 = 0.5
    CHECK(w.at(3, 9) == Approx(0.2f).margin(1e-5));  // s = 17: 0.1
    CHECK(w.at(3, 2) == Approx(1.8f).margin(1e-5));  // s = 1, across the seam: 0.9
    CHECK(w.at(5, 2) == Approx(2.0f).margin(1e-5));
}

TEST_CASE("weave: crossing exactly at a vertex counts once") {
    ExtrudeParams p = weave3();
    p.rails[0].points = {{0, 5}, {3, 5}, {10, 5}};  // vertex on B1
    const HeightField a = run(p, g21());
    ExtrudeParams q = weave3();
    CHECK(same(a, run(q, g21())));
}

TEST_CASE("weave: oblique crossing widens the window by 1/sin") {
    ExtrudeParams p;
    p.rails.push_back(rail({{0, 5}, {10, 5}}));
    p.rails.push_back(rail({{0, 0}, {10, 10}}));
    p.sections.push_back(sec({{0, 2}, {2, 2}}));
    p.weave.enabled = true;
    p.weave.startOver = false;  // the horizontal rail is under at the crossing (5,5)
    const HeightField f = run(p, g21());
    const double h = std::sqrt(2.0);
    // A's level is 0.5 for |x - 5| <= sqrt(2), then ramps to 1 over sqrt(2).
    CHECK(at(f, 6.5, 5.0) == Approx(2.0 * (0.5 + 0.5 * (6.5 - 5.0 - h) / h)).margin(1e-4));
    CHECK(at(f, 8.5, 5.0) == Approx(2.0f).margin(1e-5));
}

TEST_CASE("weave: windows of close crossings are clipped halfway between them") {
    // Wide strands (6 mm) crossing at x = 3 and 7; short verticals leave the row y = 2.5 to A.
    ExtrudeParams p;
    p.rails.push_back(rail({{0, 5}, {10, 5}}));
    p.rails.push_back(rail({{3, 3}, {3, 7}}));
    p.rails.push_back(rail({{7, 3}, {7, 7}}));
    p.sections.push_back(sec({{0, 2}, {6, 2}}));
    p.weave.enabled = true;
    const HeightField f = run(p, g21());
    CHECK(at(f, 4.5, 2.5) == Approx(2.0f).margin(1e-5));  // over B1: full
    CHECK(at(f, 5.5, 2.5) == Approx(1.0f).margin(1e-5));  // under B2: half, no ramp across the clip
}

TEST_CASE("extrude: round corners follow the arc, not a bevel") {
    ExtrudeParams p;
    p.rails.push_back(rail({{2, 2}, {8, 2}, {8, 8}}));
    p.sections.push_back(sec({{0, 1}, {2, 1}}));
    const Grid g{41, 41, 0.25, {0.0, 0.0}};
    const HeightField f = run(p, g);
    CHECK(f.at(34, 5) == Approx(1.0f));  // (8.5, 1.25): 0.90 from the vertex
    CHECK(std::isnan(f.at(35, 5)));      // (8.75, 1.25): 1.06 from the vertex
}

TEST_CASE("extrude: number of copies rounds to the nearest whole copy") {
    ExtrudeParams p;
    p.source = ExtrudeSource::Component;
    p.stretchToRail = false;
    p.component = comp(5, 3, [](std::size_t i, std::size_t) { return static_cast<double>(i); });
    p.rails.push_back(rail({{1, 5}, {8, 5}}));  // 7 mm: 1.75 copies -> 2 copies of 3.5
    HeightField f = run(p);
    CHECK(f.at(2, 5) == Approx(1.0 / 3.5 * 4.0).margin(1e-4));
    CHECK(f.at(6, 5) == Approx(1.5 / 3.5 * 4.0).margin(1e-4));
    p.rails[0].points = {{1, 5}, {10, 5}};  // 9 mm: 2.25 copies -> 2 copies of 4.5
    f = run(p);
    CHECK(f.at(3, 5) == Approx(2.0 / 4.5 * 4.0).margin(1e-4));
}

TEST_CASE("extrude: a wide profile on a short rail exceeds the size cap through rungs x samples") {
    HeightField out;
    ExtrudeParams p = straight({{0, 1}, {3000, 1}});
    p.rails[0].points = {{0, 0}, {3000, 0}};
    CHECK(extrude(p, Grid{4, 4, 1.0, {0, 0}}, nullptr, out) == ExtrudeStatus::TooLarge);
}

TEST_CASE("weave: lines that only meet beyond a rail end are not crossings") {
    const std::vector<std::vector<Point>> firsts = {
        {{0, 5}, {4, 5}}, {{6, 5}, {10, 5}}};  // horizontal rail stops short of x = 5 / starts after it
    const std::vector<std::vector<Point>> seconds = {
        {{5, 0}, {5, 4}}, {{5, 6}, {5, 10}}};
    auto build = [](const std::vector<Point>& a, const std::vector<Point>& b, bool weave) {
        ExtrudeParams p;
        p.rails.push_back(rail(a));
        p.rails.push_back(rail(b));
        p.sections.push_back(sec({{0, 2}, {2, 2}}));
        p.weave.enabled = weave;
        return p;
    };
    const std::vector<Point> full = {{0, 5}, {10, 5}};
    const std::vector<Point> fullV = {{5, 0}, {5, 10}};
    for (const auto& a : firsts) {
        CHECK(same(run(build(a, fullV, true), g21()), run(build(a, fullV, false), g21())));
    }
    for (const auto& b : seconds) {
        CHECK(same(run(build(full, b, true), g21()), run(build(full, b, false), g21())));
    }
    // Collinear overlapping rails never cross.
    ExtrudeParams c = build({{0, 5}, {6, 5}}, {{4, 5}, {10, 5}}, true);
    CHECK(same(run(c, g21()), run(build({{0, 5}, {6, 5}}, {{4, 5}, {10, 5}}, false), g21())));
}

TEST_CASE("weave: the factor ramps from full height in front of the first crossing") {
    ExtrudeParams p = weave3();
    p.weave.startOver = false;  // A is under B1: level 0.5 on [2,4], ramping up from 1 at x = 1
    const HeightField f = run(p, g21());
    CHECK(at(f, 1.5, 4.5) == Approx(2.0f * 0.75f).margin(1e-5));
    CHECK(at(f, 0.5, 4.5) == Approx(2.0f).margin(1e-5));
}

TEST_CASE("weave: windows of crossings on both sides of a closed rail's seam are clipped") {
    ExtrudeParams p;
    p.rails.push_back(rail({{5, 2}, {8, 2}, {8, 8}, {2, 8}, {2, 2}}, true));  // seam at (5, 2)
    p.rails.push_back(rail({{5.5, 1.9}, {5.5, 2.1}}));                           // crosses at s = 0.5
    p.rails.push_back(rail({{4.5, 1.9}, {4.5, 2.1}}));                           // crosses at s = 23.5
    p.sections.push_back(sec({{0, 2}, {2, 2}}));
    p.weave.enabled = true;
    const Grid g{41, 41, 0.25, {0.0, 0.0}};
    const HeightField f = run(p, g);
    // Ring over the first crossing (level 1 up to the seam), under the second (level 0.5 up to it).
    CHECK(f.at(21, 6) == Approx(2.0f).margin(1e-5));  // (5.25, 1.5): s = 0.25
    CHECK(f.at(19, 6) == Approx(1.0f).margin(1e-5));  // (4.75, 1.5): s = 23.75
}

TEST_CASE("extrude: round corner arc is finer than the 0.2 rad cap") {
    ExtrudeParams p;
    p.rails.push_back(rail({{2, 2}, {8, 2}, {8, 8}}));
    p.sections.push_back(sec({{0, 1}, {2, 1}}));
    // Fan steps are 0.125 rad (0.5 * cell / half width); a cell at radius 0.9975 on the middle of a
    // 0.2 rad chord lies outside that coarser polygon but inside the arc.
    const double a = -1.5707963267948966 + 0.098;
    const Point c{8.0 + 0.9975 * std::cos(a), 2.0 + 0.9975 * std::sin(a)};
    const Grid g{21, 21, 0.25, {c.x - 2.5, c.y - 2.5}};
    CHECK(run(p, g).at(10, 10) == Approx(1.0f));
}

TEST_CASE("extrude: corner joints on both ends of a closed rail cut into spans") {
    ExtrudeParams p;
    p.rails.push_back(rail({{2, 2}, {8, 2}, {8, 8}, {2, 8}}, true));
    p.sections.push_back(sec({{0, 1}, {2, 1}}, 0.25));
    p.sections.push_back(sec({{0, 1}, {2, 1}}, 0.75));
    p.sharpCorners = true;
    const Grid g{41, 41, 0.25, {0.0, 0.0}};
    const HeightField f = run(p, g);
    CHECK(f.at(7, 4) == Approx(1.0f));  // (1.75, 1.0): the first span's half of the seam corner
    CHECK(f.at(4, 7) == Approx(1.0f));  // (1.0, 1.75): the last span's half
}

TEST_CASE("extrude: exact height scales the maximum") {
    ExtrudeParams p = straight({{0, 2}, {1, 4}, {2, 2}});
    p.exactHeight = 10.0;
    float lo = 0.0f;
    float hi = 0.0f;
    REQUIRE(run(p).minMax(lo, hi));
    CHECK(hi == Approx(10.0f));
    CHECK(lo == Approx(5.0f));
    ExtrudeParams z = straight({{0, 0}, {2, 0}});
    z.exactHeight = 1.0;
    HeightField out;
    CHECK(extrude(z, g11(), nullptr, out) == ExtrudeStatus::FlatResult);
    CHECK(out.empty());
}

TEST_CASE("extrude: validation") {
    HeightField out;
    auto status = [&](const ExtrudeParams& p) { return extrude(p, g11(), nullptr, out); };
    ExtrudeParams ok = straight({{0, 1}, {2, 1}});
    CHECK(status(ok) == ExtrudeStatus::Ok);

    ExtrudeParams p = ok;
    p.rails.clear();
    CHECK(status(p) == ExtrudeStatus::NoRails);
    p = ok;
    p.rails[0].points = {{1, 1}};
    CHECK(status(p) == ExtrudeStatus::BadRail);
    p.rails[0].points = {{1, 1}, {1, 1}};
    CHECK(status(p) == ExtrudeStatus::BadRail);
    p.rails[0].points = {{1, 1}, {2, 2}};
    p.rails[0].closed = true;
    CHECK(status(p) == ExtrudeStatus::BadRail);
    p.rails[0].points = {{1, 1}, {std::nan(""), 2}};
    p.rails[0].closed = false;
    CHECK(status(p) == ExtrudeStatus::BadRail);
    p = ok;
    p.rails.assign(kMaxExtrudeRails + 1, p.rails[0]);
    CHECK(status(p) == ExtrudeStatus::BadRail);

    p = ok;
    p.sections.clear();
    CHECK(status(p) == ExtrudeStatus::NoSections);
    p = ok;
    p.sections[0].profile = {{0, 1}};
    CHECK(status(p) == ExtrudeStatus::BadSection);
    p.sections[0].profile = {{0, 1}, {0, 1}};
    CHECK(status(p) == ExtrudeStatus::BadSection);
    p.sections[0].profile = {{0, 1}, {1, std::nan("")}};
    CHECK(status(p) == ExtrudeStatus::BadSection);
    p = ok;
    p.sections[0].t = 1.5;
    CHECK(status(p) == ExtrudeStatus::BadSection);
    p = ok;
    p.sections.push_back(p.sections[0]);  // equal t
    CHECK(status(p) == ExtrudeStatus::BadSection);

    p = ok;
    p.exactHeight = -1.0;
    CHECK(status(p) == ExtrudeStatus::BadParameter);
    p = ok;
    p.weave.enabled = true;
    p.weave.zUnderPercent = -1.0;
    CHECK(status(p) == ExtrudeStatus::BadParameter);
    p.weave.zUnderPercent = 50.0;
    p.weave.zOverPercent = std::nan("");
    CHECK(status(p) == ExtrudeStatus::BadParameter);

    p = ok;
    p.source = ExtrudeSource::Component;
    CHECK(status(p) == ExtrudeStatus::BadComponent);
    p.component = comp(1, 3, [](std::size_t, std::size_t) { return 1.0; });
    CHECK(status(p) == ExtrudeStatus::BadComponent);
    p.component = comp(3, 1, [](std::size_t, std::size_t) { return 1.0; });
    CHECK(status(p) == ExtrudeStatus::BadComponent);
    p.component = comp(3, 3, [](std::size_t, std::size_t) { return std::numeric_limits<double>::quiet_NaN(); });
    CHECK(status(p) == ExtrudeStatus::BadComponent);
    p.component = comp(3, 3, [](std::size_t, std::size_t) { return 1.0; });
    CHECK(status(p) == ExtrudeStatus::Ok);
    p.stretchToRail = false;
    p.overlapPercent = 100.0;
    CHECK(status(p) == ExtrudeStatus::BadParameter);
    p.overlapPercent = -1.0;
    CHECK(status(p) == ExtrudeStatus::BadParameter);
    p.overlapPercent = 99.0;
    CHECK(status(p) == ExtrudeStatus::Ok);

    CHECK(extrude(ok, Grid{0, 3, 1.0, {0, 0}}, nullptr, out) == ExtrudeStatus::BadGrid);
    for (ExtrudeStatus s : {ExtrudeStatus::Ok, ExtrudeStatus::NoRails, ExtrudeStatus::BadRail, ExtrudeStatus::NoSections,
                            ExtrudeStatus::BadSection, ExtrudeStatus::BadComponent, ExtrudeStatus::BadParameter,
                            ExtrudeStatus::BadGrid, ExtrudeStatus::TooLarge, ExtrudeStatus::Cancelled,
                            ExtrudeStatus::FlatResult})
        CHECK(std::strlen(extrudeStatusMessage(s)) > 0);
}

TEST_CASE("extrude: size caps and cancel") {
    HeightField out;
    ExtrudeParams p = straight({{0, 1}, {2, 1}});
    p.rails[0].points = {{0, 0}, {1e7, 0}};
    CHECK(extrude(p, g11(), nullptr, out) == ExtrudeStatus::TooLarge);

    ExtrudeParams q = weave3();
    q.rails.clear();
    for (std::size_t i = 0; i < 40; ++i) {
        ExtrudeRail r;
        for (std::size_t k = 0; k < 60; ++k) r.points.push_back({static_cast<double>(k), static_cast<double>(i)});
        q.rails.push_back(r);
    }
    CHECK(extrude(q, g11(), nullptr, out) == ExtrudeStatus::TooLarge);  // 2360 segments with weave
    q.weave.enabled = false;
    CHECK(extrude(q, g11(), nullptr, out) == ExtrudeStatus::Ok);

    ExtrudeParams c;
    c.source = ExtrudeSource::Component;
    c.stretchToRail = false;
    c.rails.push_back(rail({{0, 0}, {1e6, 0}}));
    c.component = comp(2, 2, [](std::size_t, std::size_t) { return 1.0; });
    CHECK(extrude(c, g11(), nullptr, out) == ExtrudeStatus::TooLarge);  // too many copies

    std::atomic<bool> stop{true};
    CHECK(extrude(straight({{0, 1}, {2, 1}}), g11(), &stop, out) == ExtrudeStatus::Cancelled);
    CHECK(out.empty());
}

TEST_CASE("extrude: smooth section and deterministic output") {
    ExtrudeParams p = straight({{0, 0}, {1, 3}, {2, 3}, {3, 0}});
    p.sections[0].smooth = true;
    const HeightField a = run(p);
    CHECK(same(a, run(p)));
    float lo = 0.0f;
    float hi = 0.0f;
    REQUIRE(a.minMax(lo, hi));
    CHECK(hi <= 3.0f + 1e-5f);  // monotone cubic never overshoots
    CHECK(lo >= -1e-5f);
}
