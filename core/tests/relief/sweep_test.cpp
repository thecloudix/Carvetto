#include <atomic>
#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/sweep.hpp"

using namespace carvetto::relief;
using Catch::Approx;

namespace {
Grid g11() { return Grid{11, 11, 1.0, {0.0, 0.0}}; }

// Straight rails y=1 and y=5 from x=1 to x=9.
SweepParams straight(std::vector<Point> profile) {
    SweepParams p;
    p.rail1 = {{1, 1}, {9, 1}};
    p.rail2 = {{1, 5}, {9, 5}};
    SweepSection s;
    s.profile = std::move(profile);
    p.sections.push_back(s);
    return p;
}

HeightField run(const SweepParams& p, const Grid& g = g11()) {
    HeightField out;
    REQUIRE(sweep(p, g, nullptr, out) == SweepStatus::Ok);
    return out;
}

bool same(const HeightField& a, const HeightField& b) {
    return a.data().size() == b.data().size() &&
           std::memcmp(a.data().data(), b.data().data(), a.data().size() * sizeof(float)) == 0;
}

bool inBand(std::size_t i, std::size_t j) { return i >= 1 && i <= 9 && j >= 1 && j <= 5; }
}  // namespace

TEST_CASE("sweep: straight rails with a rectangular section give the analytic extrusion") {
    const HeightField f = run(straight({{0, 2}, {4, 2}}));
    for (std::size_t j = 0; j < 11; ++j)
        for (std::size_t i = 0; i < 11; ++i) {
            INFO("cell " << i << "," << j);
            if (inBand(i, j))
                CHECK(f.at(i, j) == Approx(2.0f).margin(1e-5));
            else
                CHECK(std::isnan(f.at(i, j)));
        }
}

TEST_CASE("sweep: ramp section, swap rails mirrors, equals reversed-profile sweep") {
    SweepParams p = straight({{0, 0}, {4, 4}});
    const HeightField f = run(p);
    for (std::size_t j = 1; j <= 5; ++j) CHECK(f.at(4, j) == Approx(static_cast<float>(j) - 1.0f).margin(1e-5));
    swapRails(p);
    CHECK(p.rail1[0].y == 5.0);
    const HeightField s = run(p);
    for (std::size_t j = 1; j <= 5; ++j) CHECK(s.at(4, j) == Approx(5.0f - static_cast<float>(j)).margin(1e-5));
    swapRails(p);
    CHECK(same(run(p), f));
}

TEST_CASE("sweep: reversing both rails reproduces the surface; selector validated") {
    SweepParams p = straight({{0, 0}, {4, 4}});
    p.sections.push_back(p.sections.front());
    p.sections[0].t1 = p.sections[0].t2 = 0.0;
    p.sections[0].profile = {{0, 0}, {4, 0}};
    p.sections[1].t1 = p.sections[1].t2 = 1.0;
    p.sections[1].profile = {{0, 4}, {4, 4}};
    const HeightField f = run(p);
    CHECK(f.at(1, 3) == Approx(0.0f).margin(1e-5));
    CHECK(f.at(9, 3) == Approx(4.0f).margin(1e-5));
    CHECK(f.at(5, 3) == Approx(2.0f).margin(1e-5));
    SweepParams q = p;
    CHECK(reverseRail(q, 3) == SweepStatus::BadParameter);
    CHECK(reverseRail(q, 0) == SweepStatus::BadParameter);
    CHECK(reverseRail(q, 1) == SweepStatus::Ok);
    CHECK(q.rail1[0].x == 9.0);
    CHECK(q.sections[0].t1 == 1.0);
    CHECK(q.sections[0].t2 == 0.0);
    CHECK(reverseRail(q, 2) == SweepStatus::Ok);
    CHECK(q.rail2[0].x == 9.0);
    // Sections are now listed high-to-low; reverseRail restored ascending order.
    CHECK(q.sections[0].t1 == 0.0);
    CHECK(q.sections[0].profile[0].y == 4.0);  // the old last section, now first
    // Sections stay attached to the same physical places: the surface is unchanged.
    CHECK(same(run(q), f));
}

TEST_CASE("sweep: unequal node counts are fine for proportional positioning, not for add-to-all-nodes") {
    SweepParams a = straight({{0, 2}, {4, 2}});
    SweepParams b = a;
    b.rail2 = {{1, 5}, {2, 5}, {5, 5}, {9, 5}};
    CHECK(same(run(a), run(b)));
    std::vector<Point> prof = {{0, 2}, {4, 2}};
    const std::size_t before = b.sections.size();
    CHECK(addSectionToAllNodes(b, prof, false) == SweepStatus::NodeCountMismatch);
    CHECK(b.sections.size() == before);
    SweepParams c = a;
    c.rail1 = {{1, 1}, {5, 1}, {9, 1}};
    c.rail2 = {{1, 5}, {5, 5}, {9, 5}};
    c.sections.clear();
    CHECK(addSectionToAllNodes(c, prof, false) == SweepStatus::Ok);
    REQUIRE(c.sections.size() == 3);
    CHECK(c.sections[1].t1 == Approx(0.5));
    CHECK(c.sections[2].t2 == Approx(1.0));
    CHECK(same(run(c), run(a)));
}

TEST_CASE("sweep: add-to-all-nodes follows each rail's own arc length") {
    SweepParams c;
    c.rail1 = {{0, 0}, {2, 0}, {10, 0}};   // node fractions 0, .2, 1
    c.rail2 = {{0, 4}, {5, 4}, {10, 4}};   // 0, .5, 1
    CHECK(addSectionToAllNodes(c, {{0, 1}, {4, 1}}, true) == SweepStatus::Ok);
    REQUIRE(c.sections.size() == 3);
    CHECK(c.sections[1].t1 == Approx(0.2));
    CHECK(c.sections[1].t2 == Approx(0.5));
    CHECK(c.sections[1].smooth);
}

TEST_CASE("sweep: several sections blend or hold between spans") {
    SweepParams p = straight({{0, 0}, {4, 0}});
    SweepSection s2;
    s2.profile = {{0, 2}, {4, 2}};
    s2.t1 = s2.t2 = 0.5;
    p.sections.push_back(s2);
    SweepSection s3 = s2;
    s3.profile = {{0, 4}, {4, 4}};
    s3.t1 = s3.t2 = 1.0;
    p.sections.push_back(s3);
    const HeightField blend = run(p);
    for (std::size_t i = 1; i <= 9; ++i) CHECK(blend.at(i, 3) == Approx((static_cast<float>(i) - 1.0f) / 2.0f).margin(1e-5));
    p.sweepBetweenSpans = false;
    const HeightField hold = run(p);
    for (std::size_t i = 1; i <= 9; ++i) {
        INFO("i " << i);
        const float want = i < 5 ? 0.0f : 2.0f;  // the last section holds to the rail end
        CHECK(hold.at(i, 3) == Approx(want).margin(1e-5));
    }
}

TEST_CASE("sweep: sections away from the ends hold their profile out to the rail ends") {
    SweepParams p = straight({{0, 3}, {4, 3}});
    p.sections[0].t1 = p.sections[0].t2 = 0.5;
    const HeightField f = run(p);
    CHECK(f.at(1, 3) == Approx(3.0f).margin(1e-5));
    CHECK(f.at(9, 3) == Approx(3.0f).margin(1e-5));
}

TEST_CASE("sweep: smooth toggle uses a monotone cubic through the points") {
    SweepParams p = straight({{0, 0}, {2, 1}, {4, 4}});
    const HeightField lin = run(p);
    p.sections[0].smooth = true;
    const HeightField sm = run(p);
    CHECK(lin.at(4, 2) == Approx(0.5f).margin(1e-5));  // u = 1 of 4
    CHECK(sm.at(4, 2) != Approx(0.5f).margin(1e-3));
    CHECK(sm.at(4, 2) >= 0.0f);
    CHECK(sm.at(4, 2) <= 1.0f);
    CHECK(sm.at(4, 3) == Approx(1.0f).margin(1e-5));  // vertices are interpolated
    CHECK(sm.at(4, 5) == Approx(4.0f).margin(1e-5));
    float prev = -1.0f;
    for (std::size_t j = 1; j <= 5; ++j) {
        CHECK(sm.at(4, j) >= prev);
        prev = sm.at(4, j);
    }
    // Two points: smooth is the same as linear.
    SweepParams q = straight({{0, 0}, {4, 4}});
    const HeightField a = run(q);
    q.sections[0].smooth = true;
    CHECK(same(a, run(q)));
}

TEST_CASE("sweep: scale cross sections with width") {
    SweepParams p = straight({{0, 0}, {2, 2}});  // section width 2, rails 4 apart
    const HeightField plain = run(p);
    p.scaleWithWidth = true;
    const HeightField scaled = run(p);
    CHECK(plain.at(4, 5) == Approx(2.0f).margin(1e-5));
    CHECK(scaled.at(4, 5) == Approx(4.0f).margin(1e-5));
    CHECK(scaled.at(4, 3) == Approx(2.0f).margin(1e-5));
}

TEST_CASE("sweep: closed rails, fill centre") {
    SweepParams p;
    p.closed1 = p.closed2 = true;
    p.rail1 = {{0, 0}, {10, 0}, {10, 10}, {0, 10}};
    p.rail2 = {{3, 3}, {7, 3}, {7, 7}, {3, 7}};
    SweepSection s;
    s.profile = {{0, 1}, {3, 3}};
    p.sections.push_back(s);
    HeightField ring = run(p);
    CHECK(ring.at(5, 0) == Approx(1.0f).margin(1e-5));
    CHECK(ring.at(5, 1) == Approx(1.0f + 2.0f / 3.0f).margin(1e-4));
    CHECK(ring.at(5, 2) == Approx(1.0f + 4.0f / 3.0f).margin(1e-4));
    CHECK(ring.at(1, 5) == Approx(1.0f + 2.0f / 3.0f).margin(1e-4));
    CHECK(ring.at(5, 3) == Approx(3.0f).margin(1e-4));
    CHECK(std::isnan(ring.at(5, 5)));
    p.fillCenter = true;
    const HeightField filled = run(p);
    CHECK(filled.at(5, 5) == Approx(3.0f).margin(1e-4));
    CHECK(filled.at(4, 6) == Approx(3.0f).margin(1e-4));
    CHECK(filled.at(5, 1) == ring.at(5, 1));
    // Swapped rails: rail 1 is now the inner one, profile mirrored, fill uses the u=0 end.
    swapRails(p);
    p.sections[0].profile = {{0, 7}, {3, 1}};
    const HeightField sw = run(p);
    CHECK(sw.at(5, 5) == Approx(7.0f).margin(1e-4));
    CHECK(sw.at(5, 0) == Approx(1.0f).margin(1e-4));
}

TEST_CASE("sweep: scale to exact height") {
    SweepParams p = straight({{0, 0}, {4, 2}});
    p.exactHeight = 5.0;
    const HeightField f = run(p);
    float lo = 0, hi = 0;
    REQUIRE(f.minMax(lo, hi));
    CHECK(hi == Approx(5.0f));
    CHECK(lo == Approx(0.0f).margin(1e-6));
    CHECK(f.at(4, 3) == Approx(2.5f).margin(1e-5));
    SweepParams z = straight({{0, 0}, {4, 0}});
    z.exactHeight = 5.0;
    HeightField out;
    CHECK(sweep(z, g11(), nullptr, out) == SweepStatus::FlatResult);
    CHECK(out.empty());
}

TEST_CASE("sweep: validation errors") {
    HeightField out;
    SweepParams base = straight({{0, 2}, {4, 2}});
    auto status = [&](const SweepParams& p) { return sweep(p, g11(), nullptr, out); };
    CHECK(status(base) == SweepStatus::Ok);
    SweepParams p = base;
    p.sections.clear();
    CHECK(status(p) == SweepStatus::NoSections);
    p = base;
    p.rail1 = {{1, 1}};
    CHECK(status(p) == SweepStatus::BadRail);
    p = base;
    p.rail2 = {{1, 5}, {1, 5}};
    CHECK(status(p) == SweepStatus::BadRail);
    p = base;
    p.rail1 = {{1, 1}, {std::numeric_limits<double>::quiet_NaN(), 1}};
    CHECK(status(p) == SweepStatus::BadRail);
    p = base;
    p.closed1 = true;
    CHECK(status(p) == SweepStatus::RailKindMismatch);
    p = base;
    p.closed1 = p.closed2 = true;
    CHECK(status(p) == SweepStatus::BadRail);  // two vertices cannot close
    p = base;
    p.sections[0].profile = {{0, 2}};
    CHECK(status(p) == SweepStatus::BadSection);
    p = base;
    p.sections[0].profile = {{0, 2}, {0, 3}};
    CHECK(status(p) == SweepStatus::BadSection);
    p = base;
    p.sections[0].profile = {{0, 2}, {4, std::numeric_limits<double>::infinity()}};
    CHECK(status(p) == SweepStatus::BadSection);
    p = base;
    p.sections[0].t1 = 1.5;
    CHECK(status(p) == SweepStatus::BadSection);
    p = base;
    p.sections[0].t2 = -0.1;
    CHECK(status(p) == SweepStatus::BadSection);
    p = base;
    p.sections.push_back(p.sections[0]);
    p.sections[0].t1 = 0.6;
    p.sections[1].t1 = 0.4;
    CHECK(status(p) == SweepStatus::BadSection);
    p = base;
    p.sections.push_back(p.sections[0]);
    p.sections[0].t2 = 0.6;
    p.sections[1].t2 = 0.4;
    CHECK(status(p) == SweepStatus::BadSection);
    p = base;
    p.exactHeight = -1.0;
    CHECK(status(p) == SweepStatus::BadParameter);
    p.exactHeight = std::numeric_limits<double>::quiet_NaN();
    CHECK(status(p) == SweepStatus::BadParameter);
    CHECK(sweep(base, Grid{0, 4, 1.0, {0, 0}}, nullptr, out) == SweepStatus::BadGrid);
    p = base;
    p.sections.assign(kMaxSweepSections + 1, base.sections[0]);
    CHECK(status(p) == SweepStatus::BadSection);
    CHECK(std::string(sweepStatusMessage(SweepStatus::NodeCountMismatch)).find("same number") != std::string::npos);
}

TEST_CASE("sweep: bounded by size and cancellable") {
    HeightField out;
    const SweepParams p = straight({{0, 2}, {4, 2}});
    CHECK(sweep(p, Grid{3, 3, 1e-6, {0, 0}}, nullptr, out) == SweepStatus::TooLarge);
    CHECK(out.empty());
    std::atomic<bool> stop{true};
    CHECK(sweep(p, g11(), &stop, out) == SweepStatus::Cancelled);
    CHECK(out.empty());
    stop = false;
    CHECK(sweep(p, g11(), &stop, out) == SweepStatus::Ok);
    CHECK(!out.empty());
}

TEST_CASE("sweep: deterministic and re-editable from stored parameters") {
    SweepParams p = straight({{0, 0}, {2, 1}, {4, 4}});
    p.sections[0].smooth = true;
    const SweepParams copy = p;
    CHECK(same(run(p), run(copy)));
    p.sections[0].profile = {{0, 0}, {4, 1}};
    CHECK(!same(run(p), run(copy)));
    // Rungs follow rail corners: an L-shaped pair of rails.
    SweepParams l;
    l.rail1 = {{1, 1}, {9, 1}, {9, 9}};
    l.rail2 = {{1, 3}, {7, 3}, {7, 9}};
    SweepSection s;
    s.profile = {{0, 1}, {2, 1}};
    l.sections.push_back(s);
    const HeightField f = run(l);
    CHECK(f.at(5, 2) == Approx(1.0f).margin(1e-5));
    CHECK(f.at(8, 5) == Approx(1.0f).margin(1e-5));
    CHECK(std::isnan(f.at(3, 6)));
    CHECK(std::isnan(f.at(5, 5)));
}

TEST_CASE("sweep: smooth never overshoots a steep-then-flat profile") {
    SweepParams p = straight({{0, 0}, {1, 10}, {2, 11}, {4, 11}});
    p.sections[0].smooth = true;
    const HeightField f = run(p);
    float prev = -1.0f;
    for (std::size_t j = 1; j <= 5; ++j) {
        const float v = f.at(4, j);
        CHECK(v >= prev - 1e-5f);
        CHECK(v <= 11.0f + 1e-5f);
        prev = v;
    }
    CHECK(f.at(4, 2) == Approx(10.0f).margin(1e-4));
}
