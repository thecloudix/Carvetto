#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/derive.hpp"

using namespace carvetto::relief;
using carvetto::geometry::Polyline;
using carvetto::geometry::Vec2;
using Catch::Approx;

namespace {

const double kPi = 3.14159265358979323846;

HeightField makeField(std::size_t w, std::size_t h, double cs, double ox, double oy, float fill = 0.0f) {
    HeightField f;
    REQUIRE(HeightField::create(Grid{w, h, cs, Point{ox, oy}}, fill, f) == Status::Ok);
    return f;
}

Polyline poly(std::vector<Vec2> pts, bool closed) {
    Polyline p;
    p.points = std::move(pts);
    p.closed = closed;
    return p;
}

bool sameBits(float a, float b) { return std::memcmp(&a, &b, sizeof a) == 0; }

HeightField composeOne(const std::vector<Component>& comps, const Grid& grid) {
    Level l;
    l.components = comps;
    HeightField out;
    REQUIRE(compose({l}, grid, out) == Status::Ok);
    return out;
}

}  // namespace

TEST_CASE("opaqueContours: square, hole, saddle, circle", "[relief][derive]") {
    SECTION("filled 3x2 block gives one CCW rectangle through the edge midpoints") {
        HeightField f = makeField(5, 4, 1.0, 10.0, 20.0, kTransparent);
        for (std::size_t j = 1; j <= 2; ++j) for (std::size_t i = 1; i <= 3; ++i) f.set(i, j, 1.0f);
        const auto c = opaqueContours(f);
        REQUIRE(c.size() == 1);
        REQUIRE(c[0].closed);
        REQUIRE(c[0].points.size() == 8);  // corners are cut by the midpoint crossings
        CHECK(c[0].signedArea() > 0.0);
        // lattice is the cell centres, crossings half a cell outside the outermost opaque centres
        CHECK(c[0].signedArea() == Approx(3.0 * 2.0 - 4.0 * 0.125));
        const auto b = c[0].bounds();
        CHECK(b.min.x == Approx(10.5));
        CHECK(b.max.x == Approx(13.5));
        CHECK(b.min.y == Approx(20.5));
        CHECK(b.max.y == Approx(22.5));
    }
    SECTION("a hole is a clockwise contour") {
        HeightField f = makeField(7, 7, 1.0, 0.0, 0.0, 1.0f);
        f.set(3, 3, kTransparent);
        f.set(3, 2, kTransparent);
        const auto c = opaqueContours(f);
        REQUIRE(c.size() == 2);
        int ccw = 0, cw = 0;
        for (const auto& p : c) (p.signedArea() > 0 ? ccw : cw)++;
        CHECK(ccw == 1);
        CHECK(cw == 1);
    }
    SECTION("diagonal cells are joined (one contour)") {
        HeightField f = makeField(3, 3, 1.0, 0.0, 0.0, kTransparent);
        f.set(0, 0, 1.0f);
        f.set(1, 1, 1.0f);
        const auto c = opaqueContours(f);
        REQUIRE(c.size() == 1);
        CHECK(c[0].points.size() == 4);  // a diagonal band; collinear midpoints removed
        CHECK(c[0].signedArea() == Approx(1.5));
    }
    SECTION("empty and all transparent give nothing") {
        CHECK(opaqueContours(HeightField{}).empty());
        CHECK(opaqueContours(makeField(3, 3, 1.0, 0, 0, kTransparent)).empty());
    }
    SECTION("deterministic") {
        HeightField f = makeField(9, 9, 0.5, 0.0, 0.0, kTransparent);
        for (std::size_t j = 1; j < 8; ++j) for (std::size_t i = 1; i < 8; ++i) if ((i + j) % 3 != 0) f.set(i, j, 1.0f);
        CHECK(opaqueContours(f) == opaqueContours(f));
    }
}

TEST_CASE("vectorBoundary: cone silhouette is a circle", "[relief][derive]") {
    const double cs = 0.25, radius = 10.0;
    const std::size_t n = 100;
    Component cone;
    cone.field = makeField(n, n, cs, -12.5, -12.5, kTransparent);
    for (std::size_t j = 0; j < n; ++j) {
        for (std::size_t i = 0; i < n; ++i) {
            const double r = std::hypot(-12.5 + static_cast<double>(i) * cs, -12.5 + static_cast<double>(j) * cs);
            if (r <= radius) cone.field.set(i, j, static_cast<float>(radius - r));
        }
    }
    std::vector<Polyline> out;
    REQUIRE(vectorBoundary({&cone}, out) == DeriveStatus::Ok);
    REQUIRE(out.size() == 1);
    CHECK(out[0].signedArea() > 0.0);
    for (const Vec2& p : out[0].points) CHECK(std::abs(std::hypot(p.x, p.y) - radius) <= cs);
    CHECK(out[0].signedArea() == Approx(kPi * radius * radius).epsilon(0.01));
}

TEST_CASE("vectorBoundary: groups, holes and errors", "[relief][derive]") {
    Component a, b, ring;
    a.field = makeField(4, 4, 1.0, 0.0, 0.0);
    b.field = makeField(4, 4, 1.0, 10.0, 0.0);
    ring.field = makeField(7, 7, 1.0, 0.0, 30.0);
    ring.field.set(3, 3, kTransparent);
    std::vector<Polyline> out;
    REQUIRE(vectorBoundary({&a, &b}, out) == DeriveStatus::Ok);
    CHECK(out.size() == 2);
    REQUIRE(vectorBoundary({&ring}, out) == DeriveStatus::Ok);
    REQUIRE(out.size() == 2);
    CHECK(((out[0].signedArea() > 0) != (out[1].signedArea() > 0)));
    CHECK(vectorBoundary({}, out) == DeriveStatus::EmptyModel);
    CHECK(vectorBoundary({nullptr}, out) == DeriveStatus::EmptyModel);
    Component blank;
    blank.field = makeField(3, 3, 1.0, 0, 0, kTransparent);
    CHECK(vectorBoundary({&blank}, out) == DeriveStatus::EmptyModel);
    CHECK(out.empty());
}

TEST_CASE("sliceModel: recombining slices rebuilds the model", "[relief][derive]") {
    HeightField m = makeField(20, 15, 0.5, 1.0, 2.0);
    for (std::size_t j = 0; j < 15; ++j) {
        for (std::size_t i = 0; i < 20; ++i) {
            m.set(i, j, static_cast<float>(0.3 * static_cast<double>(i) + 0.17 * static_cast<double>(j * j) / 7.0 - 1.0));
        }
    }
    m.set(4, 4, kTransparent);
    float lo = 0, hi = 0;
    REQUIRE(m.minMax(lo, hi));
    for (const bool fromTop : {true, false}) {
        SliceParams p;
        p.thickness = 0.7;
        p.fromTop = fromTop;
        std::vector<Slice> slices;
        REQUIRE(sliceModel(m, p, nullptr, slices) == DeriveStatus::Ok);
        const double need = std::ceil((static_cast<double>(hi) - static_cast<double>(lo)) / 0.7 - 1e-9);
        REQUIRE(static_cast<double>(slices.size()) == need);
        std::vector<Component> comps;
        for (const Slice& s : slices) {
            comps.push_back(s.component);
            CHECK(s.high - s.low == Approx(0.7));
            float flo, fhi;
            if (s.component.field.minMax(flo, fhi)) {
                CHECK(flo >= -1e-6f);
                CHECK(fhi <= 0.7f + 1e-6f);
            }
        }
        const HeightField r = composeOne(comps, m.grid());
        for (std::size_t k = 0; k < m.data().size(); ++k) {
            if (std::isnan(m.data()[k])) CHECK(std::isnan(r.data()[k]));
            else CHECK(r.data()[k] == Approx(m.data()[k]).margin(1e-5));
        }
    }
}

TEST_CASE("sliceModel: bands, order, count and boundaries", "[relief][derive]") {
    // Three plateaus: 0, 1, 2.5 in x strips of two columns.
    HeightField m = makeField(6, 2, 1.0, 0.0, 0.0);
    for (std::size_t j = 0; j < 2; ++j) {
        for (std::size_t i = 0; i < 6; ++i) m.set(i, j, i < 2 ? 0.0f : (i < 4 ? 1.0f : 2.5f));
    }
    SECTION("from top, count limited: bands go down from the top, lower cells are lost") {
        SliceParams p;
        p.thickness = 1.0;
        p.count = 2;
        p.boundaryVectors = true;
        std::vector<Slice> s;
        REQUIRE(sliceModel(m, p, nullptr, s) == DeriveStatus::Ok);
        REQUIRE(s.size() == 2);
        CHECK(s[0].high == Approx(2.5));
        CHECK(s[0].low == Approx(1.5));
        CHECK(s[1].high == Approx(1.5));
        CHECK(s[1].low == Approx(0.5));
        CHECK(s[0].component.baseHeight == Approx(1.5));
        CHECK(s[0].component.mode == CombineMode::MergeHigh);
        CHECK(s[0].component.name == "Slice 1");
        // slice 1: only the top plateau (cols 4,5), height 1.0 above its low
        CHECK(std::isnan(s[0].component.field.at(3, 0)));
        CHECK(s[0].component.field.at(4, 0) == Approx(1.0));
        // slice 2: plateau 1.0 and the top clamped (1.5 - 0.5 = 1.0 -> clamped to 1.0), col 0..1 lost
        CHECK(std::isnan(s[1].component.field.at(1, 0)));
        CHECK(s[1].component.field.at(2, 0) == Approx(0.5));
        CHECK(s[1].component.field.at(5, 1) == Approx(1.0));
        REQUIRE(s[0].boundary.size() == 1);
        CHECK(s[0].boundary[0].signedArea() == Approx(3.5));  // 2x2 cells: 2x2 box minus four 0.125 corners
        REQUIRE(s[1].boundary.size() == 1);
        CHECK(s[1].boundary[0].signedArea() == Approx(7.5));
    }
    SECTION("from bottom: first band floors the model, top clamps") {
        SliceParams p;
        p.thickness = 1.0;
        p.fromTop = false;
        p.count = 2;
        std::vector<Slice> s;
        REQUIRE(sliceModel(m, p, nullptr, s) == DeriveStatus::Ok);
        REQUIRE(s.size() == 2);
        CHECK(s[0].low == Approx(0.0));
        CHECK(s[0].high == Approx(1.0));
        CHECK(s[1].low == Approx(1.0));
        CHECK(s[1].high == Approx(2.0));
        CHECK(s[0].component.field.at(0, 0) == Approx(0.0));  // floor band keeps the lowest cells
        CHECK(s[0].component.field.at(5, 0) == Approx(1.0));
        CHECK(std::isnan(s[1].component.field.at(3, 0)));     // h == low belongs to the band below
        CHECK(s[1].component.field.at(5, 0) == Approx(1.0));  // clamped at high
        CHECK(s[0].boundary.empty());                         // not requested
    }
    SECTION("flat model: one slice; count above need is honoured") {
        HeightField flat = makeField(3, 3, 1.0, 0, 0, 2.0f);
        SliceParams p;
        p.thickness = 5.0;
        std::vector<Slice> s;
        REQUIRE(sliceModel(flat, p, nullptr, s) == DeriveStatus::Ok);
        CHECK(s.size() == 1);
        p.count = 3;
        REQUIRE(sliceModel(flat, p, nullptr, s) == DeriveStatus::Ok);
        CHECK(s.size() == 3);
    }
    SECTION("errors and cancel") {
        std::vector<Slice> s;
        SliceParams p;
        p.thickness = 0.0;
        CHECK(sliceModel(m, p, nullptr, s) == DeriveStatus::BadParameter);
        p.thickness = std::nan("");
        CHECK(sliceModel(m, p, nullptr, s) == DeriveStatus::BadParameter);
        p.thickness = -1.0;
        CHECK(sliceModel(m, p, nullptr, s) == DeriveStatus::BadParameter);
        p.thickness = 1.0;
        p.count = kMaxSlices + 1;
        CHECK(sliceModel(m, p, nullptr, s) == DeriveStatus::TooMany);
        p.count = 0;
        p.thickness = 1e-6;
        CHECK(sliceModel(m, p, nullptr, s) == DeriveStatus::TooMany);
        p.thickness = 1.0;
        CHECK(sliceModel(HeightField{}, p, nullptr, s) == DeriveStatus::EmptyModel);
        CHECK(sliceModel(makeField(2, 2, 1, 0, 0, kTransparent), p, nullptr, s) == DeriveStatus::EmptyModel);
        std::atomic<bool> stop{true};
        CHECK(sliceModel(m, p, &stop, s) == DeriveStatus::Cancelled);
        CHECK(s.empty());
    }
}

TEST_CASE("addDraft: wall test", "[relief][derive]") {
    // Wall along x = 10: left half 0, right half 10; cell size 1.
    const std::size_t n = 31;
    HeightField m = makeField(n, 5, 1.0, 0.0, 0.0);
    for (std::size_t j = 0; j < 5; ++j) for (std::size_t i = 0; i < n; ++i) m.set(i, j, i >= 15 ? 10.0f : 0.0f);
    const double angle = 10.0 * kPi / 180.0;
    const double s = 1.0 / std::tan(angle);
    Component d;
    REQUIRE(addDraft(m, angle, nullptr, d) == DeriveStatus::Ok);
    CHECK(d.name == "Draft");
    CHECK(d.mode == CombineMode::Add);
    for (std::size_t j = 0; j < 5; ++j) {
        for (std::size_t i = 0; i < n; ++i) {
            const double dist = i >= 15 ? 0.0 : 15.0 - static_cast<double>(i);
            const double expect = i >= 15 ? 10.0 : std::max(0.0, 10.0 - s * dist);
            CHECK(d.field.at(i, j) == Approx(expect).margin(1e-4));
        }
    }
    // The wall angle from vertical between neighbouring cells is at least the draft angle.
    const double step = static_cast<double>(d.field.at(14, 2) - d.field.at(13, 2));
    CHECK(std::atan(1.0 / (step / 1.0)) >= angle - 1e-6);  // run per rise
}

TEST_CASE("addDraft: gradient bound, never lowers, preserves transparency", "[relief][derive]") {
    HeightField m = makeField(40, 30, 0.5, 0.0, 0.0);
    for (std::size_t j = 0; j < 30; ++j) {
        for (std::size_t i = 0; i < 40; ++i) {
            const bool tower = (i / 5 + j / 4) % 3 == 0;
            m.set(i, j, tower ? 8.0f + static_cast<float>(i % 3) : 0.5f * static_cast<float>(j % 2));
        }
    }
    m.set(7, 7, kTransparent);
    m.set(30, 20, kTransparent);
    const double angle = 20.0 * kPi / 180.0;
    const double s = 1.0 / std::tan(angle);
    Component d;
    REQUIRE(addDraft(m, angle, nullptr, d) == DeriveStatus::Ok);
    const int offs[16][2] = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}, {-1, -1}, {1, -1}, {-1, 1}, {1, 1},
                             {-2, -1}, {-2, 1}, {2, -1}, {2, 1}, {-1, -2}, {1, -2}, {-1, 2}, {1, 2}};
    for (std::size_t j = 0; j < 30; ++j) {
        for (std::size_t i = 0; i < 40; ++i) {
            const float o = m.at(i, j), v = d.field.at(i, j);
            if (std::isnan(o)) { CHECK(std::isnan(v)); continue; }
            CHECK(v >= o);
            for (const auto& off : offs) {
                const long long ni = static_cast<long long>(i) + off[0], nj = static_cast<long long>(j) + off[1];
                if (ni < 0 || nj < 0 || ni >= 40 || nj >= 30) continue;
                const float nv = d.field.at(static_cast<std::size_t>(ni), static_cast<std::size_t>(nj));
                if (std::isnan(nv)) continue;
                const double len = 0.5 * std::hypot(off[0], off[1]);
                CHECK(static_cast<double>(v) - static_cast<double>(nv) <= s * len + 1e-4);
            }
        }
    }
    float lo, hi, dlo, dhi;
    REQUIRE(m.minMax(lo, hi));
    REQUIRE(d.field.minMax(dlo, dhi));
    CHECK(dhi == hi);
}

TEST_CASE("addDraft: spike follows the Euclidean cone within 3 percent", "[relief][derive]") {
    HeightField m = makeField(61, 61, 1.0, 0.0, 0.0, -1000.0f);
    m.set(30, 30, 100.0f);
    Component d;
    REQUIRE(addDraft(m, kPi / 4.0, nullptr, d) == DeriveStatus::Ok);  // slope 1
    double worst = 1.0;
    for (std::size_t j = 0; j < 61; ++j) {
        for (std::size_t i = 0; i < 61; ++i) {
            if (i == 30 && j == 30) continue;
            const double dist = std::hypot(static_cast<double>(i) - 30.0, static_cast<double>(j) - 30.0);
            const double drop = 100.0 - static_cast<double>(d.field.at(i, j));
            CHECK(drop >= dist * (1.0 - 1e-6));  // never above the exact cone
            worst = std::max(worst, drop / dist);
        }
    }
    CHECK(worst <= 1.03);
}

TEST_CASE("addDraft: zero angle copies, errors, cancel", "[relief][derive]") {
    HeightField m = makeField(4, 4, 1.0, 0, 0);
    m.set(1, 1, 5.0f);
    m.set(2, 2, kTransparent);
    Component d;
    REQUIRE(addDraft(m, 0.0, nullptr, d) == DeriveStatus::Ok);
    for (std::size_t k = 0; k < 16; ++k) CHECK(sameBits(d.field.data()[k], m.data()[k]));
    CHECK(addDraft(m, -0.1, nullptr, d) == DeriveStatus::BadParameter);
    CHECK(addDraft(m, kPi / 2.0, nullptr, d) == DeriveStatus::BadParameter);
    CHECK(addDraft(m, std::nan(""), nullptr, d) == DeriveStatus::BadParameter);
    CHECK(addDraft(HeightField{}, 0.1, nullptr, d) == DeriveStatus::EmptyModel);
    CHECK(addDraft(makeField(2, 2, 1, 0, 0, kTransparent), 0.1, nullptr, d) == DeriveStatus::EmptyModel);
    std::atomic<bool> stop{true};
    CHECK(addDraft(m, 0.3, &stop, d) == DeriveStatus::Cancelled);
    CHECK(d.field.empty());
}

TEST_CASE("splitComponent: closed vector", "[relief][derive]") {
    Component c;
    c.name = "Part";
    c.field = makeField(10, 10, 1.0, 0.0, 0.0);
    c.baseHeight = 2.0;
    c.shapeHeightPercent = 50.0;
    c.field.set(0, 0, kTransparent);
    for (std::size_t j = 0; j < 10; ++j) for (std::size_t i = 0; i < 10; ++i) {
        if (i + j > 0) c.field.set(i, j, static_cast<float>(i + 10 * j));
    }
    Component a, b;
    REQUIRE(splitComponent(c, poly({{2.5, 2.5}, {6.5, 2.5}, {6.5, 6.5}, {2.5, 6.5}}, true), a, b) == DeriveStatus::Ok);
    CHECK(a.name == "Part \xE2\x80\x93 A");
    CHECK(b.name == "Part \xE2\x80\x93 B");
    CHECK(a.baseHeight == 2.0);
    CHECK(b.shapeHeightPercent == 50.0);
    int na = 0, nb = 0;
    for (std::size_t j = 0; j < 10; ++j) {
        for (std::size_t i = 0; i < 10; ++i) {
            const bool inside = i >= 3 && i <= 6 && j >= 3 && j <= 6;
            const float av = a.field.at(i, j), bv = b.field.at(i, j);
            if (i + j == 0) { CHECK(std::isnan(av)); CHECK(std::isnan(bv)); continue; }
            CHECK(std::isnan(av) == !inside);
            CHECK(std::isnan(bv) == inside);
            na += !std::isnan(av);
            nb += !std::isnan(bv);
        }
    }
    CHECK(na == 16);
    CHECK(nb == 99 - 16);
    // Recombining the halves rebuilds the component.
    std::vector<Component> both{a, b};
    const HeightField r = composeOne(both, c.field.grid());
    Component cc = c;
    const HeightField orig = composeOne({cc}, c.field.grid());
    for (std::size_t k = 0; k < 100; ++k) {
        if (std::isnan(orig.data()[k])) CHECK(std::isnan(r.data()[k]));
        else CHECK(r.data()[k] == orig.data()[k]);
    }
}

TEST_CASE("splitComponent: open vector", "[relief][derive]") {
    Component c;
    c.name = "P";
    c.field = makeField(13, 13, 1.0, 0.0, 0.0, 1.0f);
    Component a, b;
    const auto side = [&](std::size_t i, std::size_t j) {  // 'A' 'B'
        return !std::isnan(a.field.at(i, j)) ? 'A' : (!std::isnan(b.field.at(i, j)) ? 'B' : '?');
    };
    SECTION("straight vector inside the field divides the whole plane; left of travel is A") {
        REQUIRE(splitComponent(c, poly({{5.5, 4.0}, {5.5, 8.0}}, false), a, b) == DeriveStatus::Ok);
        for (std::size_t j = 0; j < 13; ++j) {
            for (std::size_t i = 0; i < 13; ++i) CHECK(side(i, j) == (i <= 5 ? 'A' : 'B'));
        }
        // reversed direction swaps the halves
        REQUIRE(splitComponent(c, poly({{5.5, 8.0}, {5.5, 4.0}}, false), a, b) == DeriveStatus::Ok);
        CHECK(side(2, 6) == 'B');
        CHECK(side(9, 6) == 'A');
    }
    SECTION("a cell on the line goes to A") {
        REQUIRE(splitComponent(c, poly({{6.0, 0.0}, {6.0, 12.0}}, false), a, b) == DeriveStatus::Ok);
        CHECK(side(6, 3) == 'A');
        CHECK(side(7, 3) == 'B');
    }
    SECTION("a corner vector uses the vertex normal beyond the corner") {
        REQUIRE(splitComponent(c, poly({{0.5, 3.0}, {9.5, 3.0}, {9.5, 12.5}}, false), a, b) == DeriveStatus::Ok);
        CHECK(side(5, 6) == 'A');    // inside the corner, left of travel
        CHECK(side(12, 1) == 'B');   // beyond the vertex, right of both segments
        CHECK(side(10, 5) == 'B');   // right of the second segment
        CHECK(side(8, 2) == 'B');    // below the first segment
        CHECK(side(12, 12) == 'B');
        CHECK(side(0, 12) == 'A');
    }
}

TEST_CASE("splitComponent: errors", "[relief][derive]") {
    Component c, a, b;
    c.field = makeField(4, 4, 1.0, 0, 0);
    CHECK(splitComponent(c, poly({{0, 0}}, false), a, b) == DeriveStatus::BadVector);
    CHECK(splitComponent(c, poly({{0, 0}, {1, 1}}, true), a, b) == DeriveStatus::BadVector);
    CHECK(splitComponent(c, poly({{1, 1}, {1, 1}}, false), a, b) == DeriveStatus::BadVector);
    CHECK(splitComponent(c, poly({{1, 1}, {std::nan(""), 1}}, false), a, b) == DeriveStatus::BadVector);
    CHECK(splitComponent(Component{}, poly({{1, 1}, {2, 2}}, false), a, b) == DeriveStatus::EmptyModel);
    CHECK(a.field.empty());
    CHECK(b.field.empty());
}

TEST_CASE("crossSection: ramp is a straight line", "[relief][derive]") {
    HeightField m = makeField(21, 21, 0.5, 0.0, 0.0);
    for (std::size_t j = 0; j < 21; ++j) {
        for (std::size_t i = 0; i < 21; ++i) m.set(i, j, static_cast<float>(0.5 * 0.5 * static_cast<double>(i)));  // h = 0.5 x
    }
    std::vector<Polyline> out;
    SECTION("raw samples are collinear with the right slope") {
        REQUIRE(crossSection(m, Point{1.0, 1.0}, Point{9.0, 7.0}, 0.3, 0.0, out) == DeriveStatus::Ok);
        REQUIRE(out.size() == 1);
        CHECK(!out[0].closed);
        const double len = 10.0;
        CHECK(out[0].points.front().x == 0.0);
        CHECK(out[0].points.back().x == Approx(len));
        for (const Vec2& p : out[0].points) {
            const double t = p.x / len;
            CHECK(p.y == Approx(0.5 * (1.0 + 8.0 * t)).margin(1e-5));
        }
        CHECK(out[0].points.size() == 35);  // ceil(10/0.3) = 34 steps + end
    }
    SECTION("simplified ramp has two points") {
        REQUIRE(crossSection(m, Point{1.0, 1.0}, Point{9.0, 7.0}, 0.3, 1e-4, out) == DeriveStatus::Ok);
        REQUIRE(out.size() == 1);
        REQUIRE(out[0].points.size() == 2);
        CHECK(out[0].points[0].y == Approx(0.5));
        CHECK(out[0].points[1].y == Approx(4.5));
    }
    SECTION("simplify keeps a real corner") {
        HeightField tent = makeField(11, 3, 1.0, 0.0, 0.0);
        for (std::size_t i = 0; i < 11; ++i) for (std::size_t j = 0; j < 3; ++j) tent.set(i, j, static_cast<float>(i < 5 ? i : 10 - i));
        REQUIRE(crossSection(tent, Point{0, 1}, Point{10, 1}, 1.0, 0.01, out) == DeriveStatus::Ok);
        REQUIRE(out.size() == 1);
        CHECK(out[0].points.size() == 3);
        CHECK(out[0].points[1].x == Approx(5.0));
        CHECK(out[0].points[1].y == Approx(5.0));
    }
    SECTION("transparent gap splits the profile, outside is trimmed") {
        for (std::size_t j = 0; j < 21; ++j) m.set(10, j, kTransparent);
        m.set(11, 0, kTransparent);
        REQUIRE(crossSection(m, Point{-2.0, 1.0}, Point{8.0, 1.0}, 0.5, 0.0, out) == DeriveStatus::Ok);
        REQUIRE(out.size() == 2);
        CHECK(out[0].points.front().x == Approx(2.0));  // first opaque sample is at model x = 0
        CHECK(out[0].points.back().x == Approx(6.5 - 0.5 + 0.0 + 0.0).margin(0.51));
        CHECK(out[1].points.front().x > out[0].points.back().x + 0.9);
    }
    SECTION("errors") {
        CHECK(crossSection(m, Point{1, 1}, Point{1, 1}, 0.3, 0.0, out) == DeriveStatus::BadParameter);
        CHECK(crossSection(m, Point{1, 1}, Point{2, 1}, 0.0, 0.0, out) == DeriveStatus::BadParameter);
        CHECK(crossSection(m, Point{1, 1}, Point{2, 1}, 0.3, -1.0, out) == DeriveStatus::BadParameter);
        CHECK(crossSection(m, Point{std::nan(""), 1}, Point{2, 1}, 0.3, 0.0, out) == DeriveStatus::BadParameter);
        CHECK(crossSection(m, Point{0, 0}, Point{1e9, 0}, 1e-3, 0.0, out) == DeriveStatus::TooMany);
        CHECK(crossSection(m, Point{100, 100}, Point{200, 100}, 1.0, 0.0, out) == DeriveStatus::EmptyModel);
        CHECK(crossSection(HeightField{}, Point{0, 0}, Point{1, 0}, 0.3, 0.0, out) == DeriveStatus::EmptyModel);
        CHECK(out.empty());
    }
}

TEST_CASE("distortComponent", "[relief][derive]") {
    Component c;
    c.name = "D";
    c.baseHeight = 1.5;
    c.field = makeField(11, 11, 1.0, 0.0, 0.0);
    for (std::size_t j = 0; j < 11; ++j) for (std::size_t i = 0; i < 11; ++i) c.field.set(i, j, static_cast<float>(i + 100 * j));
    c.field.set(3, 3, kTransparent);
    Component out;
    SECTION("identity quad returns the same cells exactly") {
        const Quad q{{0, 0}, {10, 0}, {10, 10}, {0, 10}};
        REQUIRE(distortComponent(c, q, nullptr, out) == DeriveStatus::Ok);
        REQUIRE(out.field.width() == 11);
        REQUIRE(out.field.height() == 11);
        for (std::size_t k = 0; k < 121; ++k) CHECK(sameBits(out.field.data()[k], c.field.data()[k]));
        CHECK(out.name == "D");
        CHECK(out.baseHeight == 1.5);
    }
    SECTION("stretch keeps heights and moves them") {
        const Quad q{{0, 0}, {20, 0}, {20, 10}, {0, 10}};
        REQUIRE(distortComponent(c, q, nullptr, out) == DeriveStatus::Ok);
        REQUIRE(out.field.width() == 21);
        for (std::size_t j = 0; j < 11; ++j) {
            for (std::size_t i = 0; i < 21; ++i) {
                const float v = out.field.at(i, j);
                const double sx = static_cast<double>(i) / 2.0;
                if (i == 6 && j == 3) { CHECK(std::isnan(v)); continue; }  // x = 6 maps from the hole cell
                if (i == 5 || i == 7) { if (j == 3) { CHECK(std::isnan(v)); continue; } }
                CHECK(v == Approx(sx + 100.0 * static_cast<double>(j)).margin(1e-4));
            }
        }
    }
    SECTION("trapezoid maps by the bilinear envelope") {
        const Quad q{{0, 0}, {10, 0}, {8, 10}, {2, 10}};
        REQUIRE(distortComponent(c, q, nullptr, out) == DeriveStatus::Ok);
        REQUIRE(out.field.width() == 11);
        CHECK(std::isnan(out.field.at(0, 5)));  // left of the slanted edge
        CHECK(out.field.at(1, 5) == Approx(0.0 + 500.0).margin(1e-3));
        CHECK(out.field.at(5, 5) == Approx(5.0 + 500.0).margin(1e-3));
        CHECK(out.field.at(3, 5) == Approx(2.5 + 500.0).margin(1e-3));
        CHECK(out.field.at(9, 5) == Approx(10.0 + 500.0).margin(1e-3));
        CHECK(out.field.at(5, 10) == Approx(5.0 + 1000.0).margin(1e-3));
        CHECK(std::isnan(out.field.at(1, 10)));
    }
    SECTION("rotation by 90 degrees") {
        const Quad q{{10, 0}, {10, 10}, {0, 10}, {0, 0}};  // p00 at bottom-right going up
        REQUIRE(distortComponent(c, q, nullptr, out) == DeriveStatus::Ok);
        // Output (x, y) <- source u = y/10, v = (10 - x)/10: value = u*10 + 100 v*10... i + 100 j
        CHECK(out.field.at(10, 0) == Approx(0.0));
        CHECK(out.field.at(10, 4) == Approx(4.0));
        CHECK(out.field.at(7, 0) == Approx(300.0));
    }
    SECTION("errors") {
        CHECK(distortComponent(c, Quad{{0, 0}, {10, 0}, {0, 10}, {10, 10}}, nullptr, out) == DeriveStatus::BadParameter);  // bow-tie
        CHECK(distortComponent(c, Quad{{0, 0}, {10, 0}, {10, 0}, {0, 0}}, nullptr, out) == DeriveStatus::BadParameter);
        CHECK(distortComponent(c, Quad{{0, 0}, {10, 0}, {10, std::nan("")}, {0, 10}}, nullptr, out) == DeriveStatus::BadParameter);
        Component thin;
        thin.field = makeField(1, 5, 1.0, 0, 0);
        CHECK(distortComponent(thin, Quad{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, nullptr, out) == DeriveStatus::BadParameter);
        CHECK(distortComponent(Component{}, Quad{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, nullptr, out) == DeriveStatus::EmptyModel);
        CHECK(distortComponent(c, Quad{{0, 0}, {1e6, 0}, {1e6, 1e6}, {0, 1e6}}, nullptr, out) == DeriveStatus::BadGrid);
        std::atomic<bool> stop{true};
        CHECK(distortComponent(c, Quad{{0, 0}, {10, 0}, {10, 10}, {0, 10}}, &stop, out) == DeriveStatus::Cancelled);
        CHECK(out.field.empty());
    }
}

TEST_CASE("deriveStatusMessage covers every status", "[relief][derive]") {
    for (const DeriveStatus s : {DeriveStatus::Ok, DeriveStatus::EmptyModel, DeriveStatus::BadParameter, DeriveStatus::BadVector,
                                 DeriveStatus::BadGrid, DeriveStatus::TooMany, DeriveStatus::Cancelled}) {
        CHECK(std::strlen(deriveStatusMessage(s)) > 0);
        CHECK(std::strcmp(deriveStatusMessage(s), "unknown") != 0);
    }
}
