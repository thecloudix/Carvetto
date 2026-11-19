#include <algorithm>
#include <atomic>
#include <cmath>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/geometry/cancel.hpp"
#include "carvetto/geometry/nesting.hpp"
#include "carvetto/io/parts_import.hpp"

using namespace carvetto::geometry;

namespace {

Polyline poly(std::initializer_list<Vec2> pts) {
    Polyline p;
    p.points = pts;
    p.closed = true;
    return p;
}

Polyline rect(double w, double h) { return poly({{0, 0}, {w, 0}, {w, h}, {0, h}}); }

Polyline circle(double r, int n = 48) {
    Polyline p;
    p.closed = true;
    for (int i = 0; i < n; ++i) {
        const double a = kTwoPi * i / n;
        p.points.push_back({r * std::cos(a), r * std::sin(a)});
    }
    return p;
}

/// Unequal-armed L (chiral): its mirror image is not a rotation of it.
Polyline ell(double s = 1.0) {
    return poly({{0, 0}, {100 * s, 0}, {100 * s, 20 * s}, {20 * s, 20 * s}, {20 * s, 60 * s}, {0, 60 * s}});
}

NestPart part(Polyline outer, std::size_t copies = 1, std::vector<Polyline> holes = {}) {
    NestPart p;
    p.shape.outer = std::move(outer);
    p.shape.holes = std::move(holes);
    p.copies = copies;
    return p;
}

struct Placed2 {
    std::size_t sheet;
    Polyline outer;
    std::vector<Polyline> holes;
};

std::vector<Placed2> realise(const std::vector<NestPart>& parts, const NestingResult& r) {
    std::vector<Placed2> out;
    for (const NestPlacementResult& pl : r.placements) {
        Placed2 p;
        p.sheet = pl.sheet;
        p.outer = parts[pl.part].shape.outer.transformed(pl.transform);
        for (const Polyline& h : parts[pl.part].shape.holes) p.holes.push_back(h.transformed(pl.transform));
        out.push_back(std::move(p));
    }
    return out;
}

double segDist(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = ab.x * ab.x + ab.y * ab.y;
    const double t = l2 > 0 ? std::clamp(((p.x - a.x) * ab.x + (p.y - a.y) * ab.y) / l2, 0.0, 1.0) : 0.0;
    const Vec2 c = a + ab * t;
    return std::hypot(p.x - c.x, p.y - c.y);
}

double polyDist(const Polyline& a, const Polyline& b) {
    double d = 1e18;
    for (const Vec2& p : a.points) {
        for (std::size_t j = 0; j < b.points.size(); ++j) {
            d = std::min(d, segDist(p, b.points[j], b.points[(j + 1) % b.points.size()]));
        }
    }
    for (const Vec2& p : b.points) {
        for (std::size_t j = 0; j < a.points.size(); ++j) {
            d = std::min(d, segDist(p, a.points[j], a.points[(j + 1) % a.points.size()]));
        }
    }
    return d;
}

bool inPoly(const Polyline& r, Vec2 p) {
    bool in = false;
    const std::size_t n = r.points.size();
    for (std::size_t i = 0, j = n - 1; i < n; j = i++) {
        const Vec2 a = r.points[i];
        const Vec2 b = r.points[j];
        if ((a.y > p.y) != (b.y > p.y) && p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x) in = !in;
    }
    return in;
}

bool inSolid(const Placed2& s, Vec2 p) {
    if (!inPoly(s.outer, p)) return false;
    for (const Polyline& h : s.holes) {
        if (inPoly(h, p)) return false;
    }
    return true;
}

/// Independent check: margins to a rectangular sheet, pairwise clearance, no overlap.
void checkLayout(const std::vector<NestPart>& parts, const NestingOptions& o, const NestingResult& r,
                 double sheetL = 2440, double sheetW = 1220) {
    const std::vector<Placed2> ps = realise(parts, r);
    for (const Placed2& p : ps) {
        const Box2 b = p.outer.bounds();
        CHECK(b.min.x >= o.marginMm - 1e-6);
        CHECK(b.min.y >= o.marginMm - 1e-6);
        CHECK(b.max.x <= sheetL - o.marginMm + 1e-6);
        CHECK(b.max.y <= sheetW - o.marginMm + 1e-6);
    }
    for (std::size_t i = 0; i < ps.size(); ++i) {
        for (std::size_t j = i + 1; j < ps.size(); ++j) {
            if (ps[i].sheet != ps[j].sheet) continue;
            std::vector<const Polyline*> ra{&ps[i].outer};
            std::vector<const Polyline*> rb{&ps[j].outer};
            for (const Polyline& h : ps[i].holes) ra.push_back(&h);
            for (const Polyline& h : ps[j].holes) rb.push_back(&h);
            for (const Polyline* a : ra) {
                for (const Polyline* b : rb) CHECK(polyDist(*a, *b) >= o.gapMm - 1e-6);
            }
            CHECK_FALSE(inSolid(ps[j], ps[i].outer.points[0]));
            CHECK_FALSE(inSolid(ps[i], ps[j].outer.points[0]));
        }
    }
}

std::size_t countUnplaced(const NestingResult& r, NestStopReason why) {
    return static_cast<std::size_t>(
        std::count_if(r.unplaced.begin(), r.unplaced.end(), [&](const NestUnplaced& u) { return u.reason == why; }));
}

}  // namespace

TEST_CASE("nesting: rectangles agree with the shelf nester", "[nesting]") {
    namespace io = carvetto::io;
    const std::vector<std::pair<double, double>> sizes{{600, 300}, {600, 300}, {400, 200}, {400, 200},
                                                       {400, 200}, {300, 300}, {500, 250}, {250, 250}};
    io::NestOptions so;
    so.sheetLengthMm = 1200;
    so.sheetWidthMm = 700;
    so.gapMm = 6;
    so.marginMm = 10;
    std::vector<io::NestItem> items;
    std::vector<NestPart> parts;
    for (const auto& s : sizes) {
        items.push_back({s.first, s.second});
        parts.push_back(part(rect(s.first, s.second)));
    }
    const io::NestResult shelf = io::nestRectangles(items, so);
    NestingOptions o;
    o.sheets = {{1200, 700}};
    o.gapMm = 6;
    o.marginMm = 10;
    const NestingResult r = nestParts(parts, o);
    REQUIRE(r.unplaced.empty());
    CHECK(shelf.unplaced.empty());
    CHECK(r.placements.size() == shelf.placements.size());
    CHECK(r.sheetCount <= shelf.sheetCount);
    checkLayout(parts, o, r, 1200, 700);
}

TEST_CASE("nesting: one part sits at each start corner", "[nesting]") {
    const std::vector<NestPart> parts{part(rect(100, 50))};
    NestingOptions o;
    o.sheets = {{400, 300}};
    o.marginMm = 10;
    const struct {
        NestCorner c;
        double x, y;
    } cases[] = {{NestCorner::BottomLeft, 10, 10},
                 {NestCorner::BottomRight, 290, 10},
                 {NestCorner::TopLeft, 10, 240},
                 {NestCorner::TopRight, 290, 240}};
    for (const auto& c : cases) {
        o.corner = c.c;
        const NestingResult r = nestParts(parts, o);
        REQUIRE(r.placements.size() == 1);
        const Box2 b = parts[0].shape.outer.transformed(r.placements[0].transform).bounds();
        CHECK(b.min.x == Catch::Approx(c.x));
        CHECK(b.min.y == Catch::Approx(c.y));
        CHECK(b.width() == Catch::Approx(100));
    }
}

TEST_CASE("nesting: direction decides whether rows or columns fill first", "[nesting]") {
    const std::vector<NestPart> parts{part(rect(100, 100), 4)};
    NestingOptions o;
    o.sheets = {{400, 400}};
    o.marginMm = 0;
    o.gapMm = 10;
    o.direction = NestDirection::AlongX;
    NestingResult r = nestParts(parts, o);
    REQUIRE(r.placements.size() == 4);
    std::vector<Placed2> ps = realise(parts, r);
    CHECK(ps[1].outer.bounds().min.y == Catch::Approx(ps[0].outer.bounds().min.y));
    CHECK(ps[1].outer.bounds().min.x > ps[0].outer.bounds().min.x);
    o.direction = NestDirection::AlongY;
    r = nestParts(parts, o);
    REQUIRE(r.placements.size() == 4);
    ps = realise(parts, r);
    CHECK(ps[1].outer.bounds().min.x == Catch::Approx(ps[0].outer.bounds().min.x));
    CHECK(ps[1].outer.bounds().min.y > ps[0].outer.bounds().min.y);
}

TEST_CASE("nesting: rotation step finds a fit that none gives", "[nesting]") {
    const std::vector<NestPart> parts{part(rect(100, 20))};
    NestingOptions o;
    o.sheets = {{50, 150}};
    o.marginMm = 5;
    o.gapMm = 2;
    NestingResult r = nestParts(parts, o);
    CHECK(r.placements.empty());
    REQUIRE(r.unplaced.size() == 1);
    CHECK(r.unplaced[0].reason == NestStopReason::TooLarge);
    o.rotationStepRad = kPi / 2.0;
    r = nestParts(parts, o);
    REQUIRE(r.placements.size() == 1);
    CHECK(r.placements[0].angleRad == Catch::Approx(kPi / 2.0));
    checkLayout(parts, o, r, 50, 150);
}

TEST_CASE("nesting: a fine rotation step fits a diagonal", "[nesting]") {
    // A 100 x 10 bar on a 80 x 80 sheet only fits along the diagonal.
    const std::vector<NestPart> parts{part(rect(100, 10))};
    NestingOptions o;
    o.sheets = {{80, 80}};
    o.marginMm = 0;
    o.gapMm = 0;
    o.rotationStepRad = kPi / 4.0;
    const NestingResult r = nestParts(parts, o);
    REQUIRE(r.placements.size() == 1);
    CHECK(r.placements[0].angleRad == Catch::Approx(kPi / 4.0));
    checkLayout(parts, o, r, 80, 80);
}

TEST_CASE("nesting: mirroring fits a chiral part into a mirrored region", "[nesting]") {
    // Boundary: an L with its corner at the bottom right (long arm along X, short arm up the right end).
    const Polyline region = poly({{0, 0}, {140, 0}, {140, 100}, {100, 100}, {100, 40}, {0, 40}});
    // The part is the same shape with its corner at the bottom left: only its mirror image fits.
    const std::vector<NestPart> parts{part(poly({{0, 0}, {120, 0}, {120, 20}, {20, 20}, {20, 80}, {0, 80}}))};
    NestingOptions o;
    o.boundary = {region};
    o.marginMm = 1;
    o.gapMm = 1;
    o.rotationStepRad = kPi / 2.0;
    const NestingResult plain = nestParts(parts, o);
    o.mirror = true;
    const NestingResult mir = nestParts(parts, o);
    REQUIRE(mir.placements.size() == 1);
    CHECK(mir.placements[0].mirrored);
    CHECK(plain.placements.empty());
    // The placed copy lies in the region.
    const Polyline placed = parts[0].shape.outer.transformed(mir.placements[0].transform);
    for (const Vec2& v : placed.points) CHECK(inPoly(region, v));
}

TEST_CASE("nesting: parts inside parts", "[nesting]") {
    const NestShape frameShape{rect(200, 200), {poly({{25, 25}, {175, 25}, {175, 175}, {25, 175}})}};
    std::vector<NestPart> parts;
    NestPart frame;
    frame.shape = frameShape;
    parts.push_back(frame);
    parts.push_back(part(rect(100, 100)));
    NestingOptions o;
    o.sheets = {{220, 220}};
    o.marginMm = 5;
    o.gapMm = 2;
    NestingResult r = nestParts(parts, o);
    CHECK(r.sheetCount == 2);
    o.allowInside = true;
    r = nestParts(parts, o);
    REQUIRE(r.unplaced.empty());
    CHECK(r.sheetCount == 1);
    checkLayout(parts, o, r, 220, 220);
    const std::vector<Placed2> ps = realise(parts, r);
    const Box2 inner = ps[1].outer.bounds();
    CHECK(inner.min.x > 5 + 25);
    CHECK(inner.max.x < 5 + 175);
}

TEST_CASE("nesting: a part cannot straddle another part's hole edge", "[nesting]") {
    // The hole is too small for the part: it must go outside the frame even with allowInside.
    std::vector<NestPart> parts;
    NestPart frame;
    frame.shape = NestShape{rect(200, 200), {poly({{80, 80}, {120, 80}, {120, 120}, {80, 120}})}};
    parts.push_back(frame);
    parts.push_back(part(rect(60, 60)));
    NestingOptions o;
    o.sheets = {{500, 300}};
    o.marginMm = 5;
    o.gapMm = 2;
    o.allowInside = true;
    const NestingResult r = nestParts(parts, o);
    REQUIRE(r.unplaced.empty());
    checkLayout(parts, o, r, 500, 300);
}

TEST_CASE("nesting: copies, sheets list and max sheets", "[nesting]") {
    const std::vector<NestPart> parts{part(rect(200, 100), 5)};
    NestingOptions o;
    o.sheets = {{230, 120}, {230, 230}};  // second and later sheets are taller
    o.marginMm = 5;
    o.gapMm = 5;
    NestingResult r = nestParts(parts, o);
    REQUIRE(r.unplaced.empty());
    CHECK(r.placements.size() == 5);
    // Sheet 0 holds one; each later 230 x 230 sheet holds two stacked parts.
    CHECK(r.sheetCount == 3);
    std::vector<int> per(r.sheetCount, 0);
    for (const auto& p : r.placements) ++per[p.sheet];
    CHECK(per[0] == 1);
    CHECK(per[1] == 2);
    CHECK(per[2] == 2);
    o.maxSheets = 2;
    r = nestParts(parts, o);
    CHECK(r.placements.size() == 3);
    CHECK(countUnplaced(r, NestStopReason::NoSheet) == 2);
}

TEST_CASE("nesting: invalid and oversized parts are reported", "[nesting]") {
    std::vector<NestPart> parts;
    parts.push_back(part(poly({{0, 0}, {1, 0}})));
    parts.push_back(part(rect(5000, 10)));
    parts.push_back(part(rect(10, 10)));
    parts.push_back(part(poly({{0, 0}, {std::nan(""), 0}, {1, 1}})));
    NestingOptions o;
    const NestingResult r = nestParts(parts, o);
    CHECK(r.placements.size() == 1);
    CHECK(countUnplaced(r, NestStopReason::Invalid) == 2);
    CHECK(countUnplaced(r, NestStopReason::TooLarge) == 1);
}

TEST_CASE("nesting: boundary with an exclusion", "[nesting]") {
    const Polyline outerB = poly({{0, 0}, {300, 0}, {300, 300}, {0, 300}});
    const Polyline hole = poly({{100, 100}, {200, 100}, {200, 200}, {100, 200}});
    const std::vector<NestPart> parts{part(rect(80, 80), 6)};
    NestingOptions o;
    o.boundary = {outerB, hole};
    o.marginMm = 5;
    o.gapMm = 5;
    const NestingResult r = nestParts(parts, o);
    for (const Placed2& p : realise(parts, r)) {
        const Box2 b = p.outer.bounds();
        CHECK(b.min.x >= 5 - 1e-6);
        CHECK(b.max.x <= 295 + 1e-6);
        const bool clearOfHole = b.max.x <= 100 - 5 + 1e-6 || b.min.x >= 200 + 5 - 1e-6 || b.max.y <= 100 - 5 + 1e-6 ||
                                 b.min.y >= 200 + 5 - 1e-6;
        CHECK(clearOfHole);
    }
    CHECK(r.placements.size() >= 4);
    // A part bigger than the exclusion cannot wrap around it.
    const NestingResult big = nestParts({part(rect(140, 140))}, o);
    for (const Placed2& p : realise({part(rect(140, 140))}, big)) {
        const Box2 b = p.outer.bounds();
        const bool clearOfHole = b.max.x <= 95 + 1e-6 || b.min.x >= 205 - 1e-6 || b.max.y <= 95 + 1e-6 ||
                                 b.min.y >= 205 - 1e-6;
        CHECK(clearOfHole);
    }
}

TEST_CASE("nesting: curved parts keep their clearance", "[nesting]") {
    std::vector<NestPart> parts{part(circle(40), 6), part(circle(25), 6), part(rect(30, 90), 3)};
    NestingOptions o;
    o.sheets = {{500, 400}};
    o.marginMm = 8;
    o.gapMm = 4;
    o.rotationStepRad = kPi / 6.0;
    const NestingResult r = nestParts(parts, o);
    CHECK(r.unplaced.empty());
    checkLayout(parts, o, r, 500, 400);
}

TEST_CASE("nesting: deterministic and bounded", "[nesting]") {
    std::vector<NestPart> parts{part(ell(), 8), part(circle(30), 5), part(rect(70, 45), 6)};
    NestingOptions o;
    o.sheets = {{600, 400}};
    o.marginMm = 6;
    o.gapMm = 3;
    o.rotationStepRad = kPi / 4.0;
    o.mirror = true;
    const NestingResult a = nestParts(parts, o);
    const NestingResult b = nestParts(parts, o);
    REQUIRE(a.placements.size() == b.placements.size());
    CHECK(a.operations == b.operations);
    for (std::size_t i = 0; i < a.placements.size(); ++i) {
        CHECK(a.placements[i].transform == b.placements[i].transform);
        CHECK(a.placements[i].sheet == b.placements[i].sheet);
    }
    CHECK(a.unplaced.empty());
    CHECK_FALSE(a.budgetExhausted);
    checkLayout(parts, o, a, 600, 400);

    NestingOptions tight = o;
    tight.maxOperations = 5000;
    const NestingResult t = nestParts(parts, tight);
    CHECK(t.budgetExhausted);
    CHECK(t.operations <= tight.maxOperations + 1'000'000);  // one probe may overshoot, never run away
    CHECK(t.placements.size() < a.placements.size());
    CHECK(countUnplaced(t, NestStopReason::Budget) + t.placements.size() == 19);
    const NestingResult t2 = nestParts(parts, tight);
    CHECK(t2.operations == t.operations);
    CHECK(t2.placements.size() == t.placements.size());
}

TEST_CASE("nesting: cancellation stops the nest", "[nesting]") {
    std::atomic<bool> flag{true};
    CancelScope scope(&flag);
    const NestingResult r = nestParts({part(rect(10, 10), 5)}, NestingOptions{});
    CHECK(r.cancelled);
    CHECK(r.placements.size() < 5);
    CHECK(countUnplaced(r, NestStopReason::Cancelled) >= 1);
}

TEST_CASE("nesting: many copies stay within the budget", "[nesting]") {
    const std::vector<NestPart> parts{part(circle(15), 120)};
    NestingOptions o;
    o.sheets = {{1000, 600}};
    o.marginMm = 5;
    o.gapMm = 3;
    const NestingResult r = nestParts(parts, o);
    CHECK(r.unplaced.empty());
    CHECK(r.operations <= o.maxOperations);
    checkLayout(parts, o, r, 1000, 600);
}
