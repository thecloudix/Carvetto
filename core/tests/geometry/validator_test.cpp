#include "carvetto/geometry/validator.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <limits>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"

using namespace carvetto::geometry;
using namespace carvetto::geometry::validator;
using Catch::Matchers::WithinAbs;

namespace {

Contour poly(std::vector<Vec2> pts, bool closed) {
    Polyline pl;
    pl.points = std::move(pts);
    pl.closed = closed;
    return Contour::fromPolyline(pl);
}

Contour square(double x, double y, double side) {
    return Contour::rectangle(Box2{{x, y}, {x + side, y + side}});
}

Contour cw(Contour c) {
    c.reverse();
    return c;
}

Report check(const std::vector<ObjectContours>& objs, const Options& opt = {}, const std::vector<bool>& skip = {}) {
    std::vector<const ObjectContours*> view;
    for (const ObjectContours& o : objs) {
        view.push_back(&o);
    }
    return validate(view, opt, skip);
}

double totalArea(const std::vector<ObjectContours>& objs) {
    double a = 0.0;
    for (const auto& o : objs) {
        for (const Contour& c : o) {
            if (c.closed && std::isfinite(signedArea(c))) {
                a += std::fabs(signedArea(c));
            }
        }
    }
    return a;
}

std::vector<ObjectContours> corpus() {
    std::vector<ObjectContours> o;
    // 0: clean outer with a clockwise hole.
    o.push_back({square(0, 0, 20), cw(square(5, 5, 10))});
    // 1: clockwise outer: wrong direction.
    o.push_back({cw(square(100, 0, 10))});
    // 2: counter-clockwise hole in an outer: wrong direction.
    o.push_back({square(200, 0, 30), square(205, 5, 10)});
    // 3: open, fixable gap; open, unfixable gap.
    o.push_back({poly({{300, 0}, {310, 0}, {310, 10}, {300, 0.005}}, false),
                 poly({{400, 0}, {410, 0}, {410, 10}, {400, 5}}, false)});
    // 4: square with one zero-length span.
    {
        Contour c = square(500, 0, 10);
        const Vec2 p = endPoint(c.spans[0]);
        c.spans.insert(c.spans.begin() + 1, LineSpan{p, p});
        o.push_back({c});
    }
    // 5: tiny contour.
    o.push_back({square(600, 0, 0.005)});
    // 6, 7: the second repeats the first, reversed and starting elsewhere.
    o.push_back({square(700, 0, 7)});
    {
        Contour d = cw(square(700, 0, 7));
        std::rotate(d.spans.begin(), d.spans.begin() + 2, d.spans.end());
        // Counter-clockwise copy keeps its direction right: reverse back, then rotate.
        d = cw(d);
        o.push_back({d});
    }
    // 8: bow tie.
    o.push_back({poly({{800, 0}, {810, 10}, {810, 0}, {800, 10}}, true)});
    // 9: not a number.
    o.push_back({poly({{900, 0}, {std::numeric_limits<double>::quiet_NaN(), 5}, {910, 10}}, true)});
    return o;
}

}  // namespace

TEST_CASE("validator: a clean drawing reports nothing", "[validator]") {
    const std::vector<ObjectContours> objs{{square(0, 0, 20), cw(square(5, 5, 10)), Contour::circle({50, 50}, 5)},
                                           {Contour::circle({100, 0}, 3, ArcDirection::CCW)}};
    const Report r = check(objs);
    CHECK(r.total() == 0);
    CHECK(r.skipped == 0);
    CHECK_FALSE(r.truncated);
    CHECK(check({}).total() == 0);
    CHECK(check({ObjectContours{}}).total() == 0);
}

TEST_CASE("validator: seeded corpus is detected exactly", "[validator]") {
    const std::vector<ObjectContours> objs = corpus();
    const Report r = check(objs);
    CHECK(r.count(IssueKind::ZeroLengthSpan) == 1);
    CHECK(r.count(IssueKind::TinyContour) == 1);
    CHECK(r.count(IssueKind::OpenContour) == 2);
    CHECK(r.count(IssueKind::DuplicateContour) == 1);
    CHECK(r.count(IssueKind::SelfIntersecting) == 1);
    CHECK(r.count(IssueKind::WrongDirection) == 2);
    CHECK(r.count(IssueKind::InvalidGeometry) == 1);
    CHECK(r.total() == 9);
    CHECK(r.fixableCount() == 6);
    for (const Issue& i : r.issues) {
        switch (i.kind) {
            case IssueKind::ZeroLengthSpan:
                CHECK(i.object == 4);
                CHECK(i.span == 1);
                break;
            case IssueKind::TinyContour:
                CHECK(i.object == 5);
                break;
            case IssueKind::OpenContour:
                CHECK(i.object == 3);
                CHECK(i.fixable == (i.contour == 0));
                break;
            case IssueKind::DuplicateContour:
                CHECK(i.object == 7);
                CHECK(i.otherObject == 6);
                CHECK(i.otherContour == 0);
                break;
            case IssueKind::SelfIntersecting:
                CHECK(i.object == 8);
                CHECK_FALSE(i.fixable);
                break;
            case IssueKind::WrongDirection:
                CHECK((i.object == 1 || (i.object == 2 && i.contour == 1)));
                break;
            case IssueKind::InvalidGeometry:
                CHECK(i.object == 9);
                CHECK_FALSE(i.fixable);
                break;
        }
    }
}

TEST_CASE("validator: fixing repairs what is fixable, keeps area, and is idempotent", "[validator]") {
    std::vector<ObjectContours> objs = corpus();
    const double areaBefore = totalArea(objs);
    const FixSummary s = fix(objs);
    CHECK(s.spansRemoved == 1);
    CHECK(s.tinyRemoved == 1);
    CHECK(s.contoursClosed == 1);
    CHECK(s.duplicatesRemoved == 1);
    CHECK(s.contoursReversed == 2);
    CHECK(s.total() == 6);
    // Only the unfixable ones remain; the invalid contour is untouched.
    const Report after = check(objs);
    CHECK(after.total() == 3);
    CHECK(after.count(IssueKind::OpenContour) == 1);
    CHECK(after.count(IssueKind::SelfIntersecting) == 1);
    CHECK(after.count(IssueKind::InvalidGeometry) == 1);
    // Area changes: the duplicate (49) and tiny square (0.005^2) go; the closed open contour starts
    // to count (a 10 x 10 right triangle with the gap sliver: 50.025). The reversals change nothing.
    CHECK_THAT(totalArea(objs), WithinAbs(areaBefore - 49.0 - 0.005 * 0.005 + 50.025, 1e-3));
    CHECK(objs[5].empty());
    CHECK(objs[7].empty());
    CHECK(objs[4][0].spans.size() == 4);
    CHECK(objs[3][0].closed);
    CHECK_FALSE(objs[3][1].closed);
    CHECK(orientation(objs[1][0], Tolerance{}) == Orientation::CCW);
    CHECK(orientation(objs[2][1], Tolerance{}) == Orientation::CW);
    const FixSummary again = fix(objs);
    CHECK(again.total() == 0);
}

TEST_CASE("validator: fix flags select what is repaired", "[validator]") {
    std::vector<ObjectContours> objs = corpus();
    FixFlags only;
    only = {false, false, false, false, true};
    const FixSummary s = fix(objs, {}, only);
    CHECK(s.contoursReversed == 2);
    CHECK(s.total() == 2);
    CHECK(check(objs).count(IssueKind::WrongDirection) == 0);
    CHECK(check(objs).count(IssueKind::ZeroLengthSpan) == 1);
}

TEST_CASE("validator: zero-length boundary is the distance tolerance", "[validator]") {
    Options opt;
    opt.tol.distance = 0.01;
    auto withSpan = [](double len) {
        Contour c = square(0, 0, 10);
        const Vec2 p = endPoint(c.spans[0]);
        c.spans.insert(c.spans.begin() + 1, LineSpan{p, {p.x + len, p.y}});
        return ObjectContours{c};
    };
    CHECK(check({withSpan(0.01)}, opt).count(IssueKind::ZeroLengthSpan) == 1);
    CHECK(check({withSpan(0.011)}, opt).count(IssueKind::ZeroLengthSpan) == 0);
    // Tiny arcs and Beziers count too.
    Contour c = square(0, 0, 10);
    const Vec2 p = endPoint(c.spans[0]);
    c.spans.insert(c.spans.begin() + 1, BezierSpan{p, p, p, p});
    CHECK(check({ObjectContours{c}}, opt).count(IssueKind::ZeroLengthSpan) == 1);
}

TEST_CASE("validator: tiny boundary needs both dimensions within the size", "[validator]") {
    Options opt;
    opt.tinySizeMm = 0.5;
    CHECK(check({{square(0, 0, 0.5)}}, opt).count(IssueKind::TinyContour) == 1);
    CHECK(check({{square(0, 0, 0.51)}}, opt).count(IssueKind::TinyContour) == 0);
    const Contour thin = Contour::rectangle(Box2{{0, 0}, {0.1, 50}});
    CHECK(check({{thin}}, opt).count(IssueKind::TinyContour) == 0);
    CHECK(check({{Contour{}}}, opt).count(IssueKind::TinyContour) == 1);
}

TEST_CASE("validator: open contours are closed only within the gap limit", "[validator]") {
    Options opt;
    opt.closeGapMm = 0.1;
    auto open = [](double gap) { return poly({{0, 0}, {10, 0}, {10, 10}, {0, gap}}, false); };
    CHECK(check({{open(0.1)}}, opt).issues.at(0).fixable);
    CHECK_FALSE(check({{open(0.2)}}, opt).issues.at(0).fixable);
    std::vector<ObjectContours> objs{{open(0.05)}};
    CHECK(fix(objs, opt).contoursClosed == 1);
    CHECK(objs[0][0].closed);
    CHECK(objs[0][0].isContinuous(opt.tol));
    CHECK(objs[0][0].spans.size() == 4);  // bridging line added
    std::vector<ObjectContours> exact{{poly({{0, 0}, {10, 0}, {10, 10}, {0, 10}, {0, 0}}, false)}};
    CHECK(fix(exact, opt).contoursClosed == 1);
    CHECK(exact[0][0].closed);
    CHECK(exact[0][0].spans.size() == 4);  // no bridge: the end already met the start
    std::vector<ObjectContours> wide{{open(5.0)}};
    CHECK(fix(wide, opt).total() == 0);
    CHECK_FALSE(wide[0][0].closed);
}

TEST_CASE("validator: duplicates across rotation, direction, arcs and objects", "[validator]") {
    const Contour c1 = Contour::circle({0, 0}, 5, ArcDirection::CCW);
    Contour c2 = Contour::circle({0, 0}, 5, ArcDirection::CW);
    const Contour openLine = poly({{0, 20}, {10, 20}, {10, 30}}, false);
    const Contour openRev = openLine.reversed();
    Report r = check({{c1, c2}, {openLine}, {openRev}});
    CHECK(r.count(IssueKind::DuplicateContour) == 2);
    // Three copies: two removed, the first kept.
    r = check({{square(0, 0, 4), square(0, 0, 4)}, {square(0, 0, 4)}});
    CHECK(r.count(IssueKind::DuplicateContour) == 2);
    for (const Issue& i : r.issues) {
        CHECK(i.otherObject == 0);
        CHECK(i.otherContour == 0);
    }
    // Different radius, offset beyond tolerance, or open versus closed are not duplicates.
    CHECK(check({{Contour::circle({0, 0}, 5), Contour::circle({0, 0}, 5.01)}}).count(IssueKind::DuplicateContour) == 0);
    CHECK(check({{square(0, 0, 4), square(0.01, 0, 4)}}).count(IssueKind::DuplicateContour) == 0);
    CHECK(check({{poly({{0, 0}, {4, 0}, {4, 4}, {0, 4}}, true), poly({{0, 0}, {4, 0}, {4, 4}, {0, 4}}, false)}})
              .count(IssueKind::DuplicateContour) == 0);
    // Same bounds but different shape.
    const Contour tri = poly({{0, 0}, {4, 0}, {4, 4}}, true);
    const Contour tri2 = poly({{0, 0}, {4, 0}, {0, 4}}, true);
    CHECK(check({{tri, tri2}}).count(IssueKind::DuplicateContour) == 0);
    // Removal keeps the first.
    std::vector<ObjectContours> objs{{c1}, {c2}};
    CHECK(fix(objs).duplicatesRemoved == 1);
    CHECK(objs[0].size() == 1);
    CHECK(objs[1].empty());
}

TEST_CASE("validator: crossing check can be skipped per object", "[validator]") {
    const std::vector<ObjectContours> objs{{poly({{0, 0}, {10, 10}, {10, 0}, {0, 10}}, true)},
                                           {poly({{20, 0}, {30, 10}, {30, 0}, {20, 10}}, true)}};
    CHECK(check(objs).count(IssueKind::SelfIntersecting) == 2);
    CHECK(check(objs, {}, {false, true}).count(IssueKind::SelfIntersecting) == 1);
    CHECK(check(objs, {}, {true, true}).count(IssueKind::SelfIntersecting) == 0);
    // A contour touching itself at a point also counts; a plain polygon does not.
    CHECK(check({{poly({{0, 0}, {10, 0}, {10, 10}, {5, 0}, {0, 10}}, true)}}).count(IssueKind::SelfIntersecting) == 1);
    // A self-intersecting contour never gets a direction verdict.
    CHECK(check(objs).count(IssueKind::WrongDirection) == 0);
}

TEST_CASE("validator: nesting depth alternates the expected direction", "[validator]") {
    // Island inside a hole inside an outer: CCW, CW, CCW.
    std::vector<ObjectContours> ok{{square(0, 0, 40), cw(square(5, 5, 30)), square(10, 10, 20)}};
    CHECK(check(ok).total() == 0);
    std::vector<ObjectContours> bad{{square(0, 0, 40), cw(square(5, 5, 30)), cw(square(10, 10, 20))}};
    const Report r = check(bad);
    REQUIRE(r.total() == 1);
    CHECK(r.issues[0].contour == 2);
    CHECK(fix(bad).contoursReversed == 1);
    CHECK(check(bad).total() == 0);
    // Contours of different objects do not nest.
    CHECK(check({{square(0, 0, 40)}, {square(5, 5, 10)}}).total() == 0);
    // Side by side: both outers.
    CHECK(check({{square(0, 0, 10), square(20, 0, 10)}}).total() == 0);
    // A contour sharing the boundary of another is not inside it.
    CHECK(check({{square(0, 0, 10), square(0, 0, 5) }}).count(IssueKind::WrongDirection) == 0);
}

TEST_CASE("validator: fixing a zero-length span keeps the shape continuous", "[validator]") {
    Contour c = square(0, 0, 10);
    const Vec2 p = endPoint(c.spans[1]);
    c.spans.insert(c.spans.begin() + 2, LineSpan{p, p});
    c.spans.insert(c.spans.begin() + 1, LineSpan{startPoint(c.spans[1]), startPoint(c.spans[1])});
    std::vector<ObjectContours> objs{{c}};
    const double a = std::fabs(signedArea(c));
    CHECK(fix(objs).spansRemoved == 2);
    CHECK(objs[0][0].spans.size() == 4);
    CHECK(objs[0][0].isContinuous(Tolerance{}));
    CHECK_THAT(std::fabs(signedArea(objs[0][0])), WithinAbs(a, 1e-9));
    // A contour made only of zero-length spans disappears.
    std::vector<ObjectContours> dust{{poly({{1, 1}, {1, 1}, {1, 1}}, true)}};
    fix(dust);
    CHECK(dust[0].empty());
}

TEST_CASE("validator: hostile input is bounded", "[validator]") {
    // Too many spans: nothing is analysed, the report says so.
    ObjectContours big(1);
    big[0].closed = false;
    big[0].spans.assign(kMaxSpans + 1, Span{LineSpan{{0, 0}, {1, 1}}});
    Report r = check({big});
    CHECK(r.truncated);
    CHECK(r.total() == 0);
    // Huge coordinates and infinities are reported and left alone.
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<ObjectContours> odd{{poly({{0, 0}, {1e12, 0}, {0, 5}}, true), poly({{0, 0}, {inf, 0}, {0, 5}}, true)}};
    r = check(odd);
    CHECK(r.count(IssueKind::InvalidGeometry) == 2);
    CHECK(r.total() == 2);
    const std::vector<ObjectContours> before = odd;
    CHECK(fix(odd).total() == 0);
    CHECK(odd == before);
    // A very long contour is not crossing-checked: counted as skipped.
    ObjectContours longC(1);
    longC[0].closed = false;
    for (std::size_t i = 0; i <= kMaxSelfIntersectSpans; ++i) {
        const double x = static_cast<double>(i);
        longC[0].spans.push_back(LineSpan{{x, 0}, {x + 1, 0}});
    }
    r = check({longC});
    CHECK(r.skipped == 1);
    CHECK(r.count(IssueKind::SelfIntersecting) == 0);
    // Many contours in one object: the direction check is skipped for it.
    ObjectContours many;
    for (std::size_t i = 0; i <= kMaxNestingContours; ++i) {
        many.push_back(cw(square(static_cast<double>(i) * 20.0, 0, 5)));
    }
    r = check({many});
    CHECK(r.skipped == 1);
    CHECK(r.count(IssueKind::WrongDirection) == 0);
    // Null object pointers are tolerated.
    CHECK(validate({nullptr}).total() == 0);
}

TEST_CASE("validator: many identical contours finish within the comparison budget", "[validator]") {
    ObjectContours same;
    for (int i = 0; i < 3000; ++i) {
        same.push_back(square(0, 0, 4));
    }
    const Report r = check({same});
    CHECK(r.count(IssueKind::DuplicateContour) + (r.truncated ? 0u : 0u) <= 2999);
    if (!r.truncated) {
        CHECK(r.count(IssueKind::DuplicateContour) == 2999);
    }
}
