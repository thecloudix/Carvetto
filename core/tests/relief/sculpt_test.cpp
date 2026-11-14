#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/sculpt.hpp"

using namespace carvetto::relief;
using Catch::Approx;

namespace {

HeightField flat(std::size_t w, std::size_t h, float v, double cs = 1.0) {
    HeightField f;
    REQUIRE(HeightField::create(Grid{w, h, cs, Point{0.0, 0.0}}, v, f) == Status::Ok);
    return f;
}

bool sameBits(const HeightField& a, const HeightField& b) {
    return a.data().size() == b.data().size() &&
           std::memcmp(a.data().data(), b.data().data(), a.data().size() * sizeof(float)) == 0;
}

BrushParams brush(BrushKind k, double diameter = 8.0, double strength = 1.0, double smooth = 0.0) {
    BrushParams b;
    b.kind = k;
    b.diameter = diameter;
    b.strength = strength;
    b.smoothness = smooth;
    return b;
}

SculptSession start(const HeightField& f, SculptOptions o = {}) {
    SculptSession s;
    REQUIRE(SculptSession::begin(f, o, s) == Status::Ok);
    return s;
}

Dab dabAt(double x, double y, double w = 1.0, Point delta = {}) { return Dab{Point{x, y}, w, delta}; }

}  // namespace

TEST_CASE("sculpt: kernel shape", "[relief][sculpt]") {
    CHECK(brushKernel(0.0, 10.0, 0.0) == 1.0);
    CHECK(brushKernel(4.999, 10.0, 0.0) == 1.0);
    CHECK(brushKernel(5.0, 10.0, 0.0) == 0.0);
    CHECK(brushKernel(7.0, 10.0, 0.0) == 0.0);
    CHECK(brushKernel(0.0, 10.0, 1.0) == 1.0);
    CHECK(brushKernel(2.5, 10.0, 1.0) == Approx(0.5));  // smoothstep midpoint
    CHECK(brushKernel(5.0, 10.0, 1.0) == 0.0);
    // Half smooth: plateau to half the radius, then smoothstep.
    CHECK(brushKernel(2.5, 10.0, 0.5) == 1.0);
    CHECK(brushKernel(3.75, 10.0, 0.5) == Approx(0.5));
    CHECK(brushKernel(1.0, 0.0, 0.5) == 0.0);
}

TEST_CASE("sculpt: deposit and remove single-dab goldens", "[relief][sculpt]") {
    SculptSession s = start(flat(21, 21, 1.0f));
    REQUIRE(s.applyStroke(brush(BrushKind::Deposit, 8.0, 2.0), {dabAt(10, 10)}) == Status::Ok);
    CHECK(s.working().at(10, 10) == Approx(3.0f));   // centre: +strength
    CHECK(s.working().at(13, 10) == Approx(3.0f));   // inside hard edge
    CHECK(s.working().at(14, 10) == Approx(1.0f));   // on the radius: 0
    CHECK(s.working().at(0, 0) == Approx(1.0f));
    REQUIRE(s.applyStroke(brush(BrushKind::Remove, 8.0, 0.5), {dabAt(10, 10, 2.0)}) == Status::Ok);
    CHECK(s.working().at(10, 10) == Approx(2.0f));   // 3 - 0.5 * weight 2
    CHECK(s.working().at(10, 13) == Approx(2.0f));
}

TEST_CASE("sculpt: smooth golden", "[relief][sculpt]") {
    HeightField f = flat(9, 9, 0.0f);
    f.set(4, 4, 16.0f);
    SculptSession s = start(f);
    // 3x3 binomial blur of a single spike: centre weight 4/16.
    REQUIRE(s.applyStroke(brush(BrushKind::Smooth, 6.0, 1.0), {dabAt(4, 4)}) == Status::Ok);
    CHECK(s.working().at(4, 4) == Approx(4.0f));
    CHECK(s.working().at(5, 4) == Approx(2.0f));
    CHECK(s.working().at(5, 5) == Approx(1.0f));
    CHECK(s.working().at(1, 4) == Approx(0.0f));
    // Strength 0.5 blends halfway.
    SculptSession s2 = start(f);
    REQUIRE(s2.applyStroke(brush(BrushKind::Smooth, 6.0, 0.5), {dabAt(4, 4)}) == Status::Ok);
    CHECK(s2.working().at(4, 4) == Approx(10.0f));
    // Reads the pre-dab values (order within a dab does not matter).
    CHECK(s2.working().at(5, 4) == Approx(1.0f));
}

TEST_CASE("sculpt: smudge drags material along the stroke", "[relief][sculpt]") {
    HeightField f = flat(15, 9, 0.0f);
    f.set(5, 4, 8.0f);
    SculptSession s = start(f);
    BrushParams b = brush(BrushKind::Smudge, 10.0, 1.0);
    REQUIRE(s.applyStroke(b, {dabAt(5, 4, 1.0, Point{2.0, 0.0})}) == Status::Ok);
    CHECK(s.working().at(7, 4) == Approx(8.0f));  // moved +2 in x
    CHECK(s.working().at(5, 4) == Approx(0.0f));
    // Zero movement does nothing.
    SculptSession s2 = start(f);
    REQUIRE(s2.applyStroke(b, {dabAt(5, 4)}) == Status::Ok);
    CHECK(sameBits(s2.working(), f));
}

TEST_CASE("sculpt: modes", "[relief][sculpt]") {
    HeightField f = flat(9, 9, 0.0f);
    f.set(4, 4, 16.0f);
    BrushParams b = brush(BrushKind::Smooth, 6.0, 1.0);
    b.mode = SculptMode::Raise;
    SculptSession r = start(f);
    REQUIRE(r.applyStroke(b, {dabAt(4, 4)}) == Status::Ok);
    CHECK(r.working().at(4, 4) == 16.0f);        // lowering refused
    CHECK(r.working().at(5, 4) == Approx(2.0f)); // raising allowed
    b.mode = SculptMode::Lower;
    SculptSession l = start(f);
    REQUIRE(l.applyStroke(b, {dabAt(4, 4)}) == Status::Ok);
    CHECK(l.working().at(4, 4) == Approx(4.0f));
    CHECK(l.working().at(5, 4) == 0.0f);
}

TEST_CASE("sculpt: transparency", "[relief][sculpt]") {
    HeightField f = flat(9, 9, 1.0f);
    f.set(4, 4, kTransparent);
    BrushParams b = brush(BrushKind::Deposit, 8.0, 1.0);
    SculptSession keepT = start(f);
    REQUIRE(keepT.applyStroke(b, {dabAt(4, 4)}) == Status::Ok);
    CHECK(std::isnan(keepT.working().at(4, 4)));
    CHECK(keepT.working().at(5, 4) == Approx(2.0f));
    b.preserveTransparency = false;
    SculptSession fill = start(f);
    REQUIRE(fill.applyStroke(b, {dabAt(4, 4)}) == Status::Ok);
    CHECK(fill.working().at(4, 4) == Approx(1.0f));  // transparent counts as 0
    // Smooth ignores transparent neighbours and (unpreserved) fills the hole.
    BrushParams sm = brush(BrushKind::Smooth, 6.0, 1.0);
    SculptSession sk = start(f);
    REQUIRE(sk.applyStroke(sm, {dabAt(4, 4)}) == Status::Ok);
    CHECK(std::isnan(sk.working().at(4, 4)));
    CHECK(sk.working().at(5, 4) == Approx(1.0f));
    sm.preserveTransparency = false;
    SculptSession sf = start(f);
    REQUIRE(sf.applyStroke(sm, {dabAt(4, 4)}) == Status::Ok);
    CHECK(sf.working().at(4, 4) == Approx(1.0f));
    // Smudge never reads transparent cells as values.
    HeightField g = flat(9, 3, kTransparent);
    g.set(2, 1, 5.0f);
    BrushParams mu = brush(BrushKind::Smudge, 8.0, 1.0);
    SculptSession sg = start(g);
    REQUIRE(sg.applyStroke(mu, {dabAt(2, 1, 1.0, Point{1.0, 0.0})}) == Status::Ok);
    CHECK(std::isnan(sg.working().at(3, 1)));  // preserved
    mu.preserveTransparency = false;
    SculptSession sh = start(g);
    REQUIRE(sh.applyStroke(mu, {dabAt(2, 1, 1.0, Point{1.0, 0.0})}) == Status::Ok);
    CHECK(sh.working().at(3, 1) == Approx(5.0f));
}

TEST_CASE("sculpt: undo brush", "[relief][sculpt]") {
    HeightField f = flat(21, 21, 1.0f);
    f.set(3, 3, 7.5f);
    SculptSession s = start(f);
    REQUIRE(s.applyStroke(brush(BrushKind::Deposit, 12.0, 3.0), {dabAt(10, 10)}) == Status::Ok);
    REQUIRE(s.applyStroke(brush(BrushKind::Remove, 12.0, 1.0, 1.0), {dabAt(4, 4)}) == Status::Ok);
    CHECK_FALSE(sameBits(s.working(), f));
    // Full strength over the plateau restores exactly; a stroke covering everything restores all.
    BrushParams ub = brush(BrushKind::UndoBrush, 60.0, 1.0, 0.0);
    REQUIRE(s.applyStroke(ub, {dabAt(10, 10)}) == Status::Ok);
    CHECK(sameBits(s.working(), f));
    // Partial strength moves a fraction of the way back.
    SculptSession p = start(f);
    REQUIRE(p.applyStroke(brush(BrushKind::Deposit, 12.0, 4.0), {dabAt(10, 10)}) == Status::Ok);
    BrushParams half = brush(BrushKind::UndoBrush, 12.0, 0.25, 0.0);
    REQUIRE(p.applyStroke(half, {dabAt(10, 10)}) == Status::Ok);
    CHECK(p.working().at(10, 10) == Approx(4.0f));  // 5 + (1-5)*0.25
    // Undo brush restores transparency and handles NaN with a threshold.
    HeightField t = flat(9, 9, 1.0f);
    t.set(4, 4, kTransparent);
    SculptSession q = start(t);
    BrushParams fill = brush(BrushKind::Deposit, 4.0, 1.0);
    fill.preserveTransparency = false;
    REQUIRE(q.applyStroke(fill, {dabAt(4, 4)}) == Status::Ok);
    REQUIRE_FALSE(std::isnan(q.working().at(4, 4)));
    REQUIRE(q.applyStroke(brush(BrushKind::UndoBrush, 4.0, 0.4), {dabAt(4, 4)}) == Status::Ok);
    CHECK_FALSE(std::isnan(q.working().at(4, 4)));  // a = 0.4 < 0.5 keeps
    REQUIRE(q.applyStroke(brush(BrushKind::UndoBrush, 4.0, 0.6), {dabAt(4, 4)}) == Status::Ok);
    CHECK(std::isnan(q.working().at(4, 4)));
}

TEST_CASE("sculpt: stroke undo, keep and discard", "[relief][sculpt]") {
    HeightField f = flat(30, 30, 0.0f);
    SculptSession s = start(f);
    CHECK_FALSE(s.canUndo());
    CHECK_FALSE(s.undoStroke());
    REQUIRE(s.applyStroke(brush(BrushKind::Deposit, 8.0, 1.0), {dabAt(5, 5)}) == Status::Ok);
    HeightField afterFirst = s.working();
    REQUIRE(s.applyStroke(brush(BrushKind::Smooth, 10.0, 1.0), {dabAt(7, 7), dabAt(9, 9)}) == Status::Ok);
    CHECK(s.canUndo());
    CHECK(s.undoStroke());
    CHECK(sameBits(s.working(), afterFirst));
    CHECK(s.undoStroke());
    CHECK(sameBits(s.working(), f));
    CHECK_FALSE(s.canUndo());
    CHECK(s.undoBytes() == 0);
    // Discard goes back to the baseline; Keep moves the baseline.
    REQUIRE(s.applyStroke(brush(BrushKind::Deposit, 8.0, 1.0), {dabAt(5, 5)}) == Status::Ok);
    s.discard();
    CHECK(sameBits(s.working(), f));
    CHECK_FALSE(s.canUndo());
    REQUIRE(s.applyStroke(brush(BrushKind::Deposit, 8.0, 1.0), {dabAt(5, 5)}) == Status::Ok);
    HeightField kept = s.working();
    s.keep();
    CHECK(sameBits(s.baseline(), kept));
    CHECK_FALSE(s.canUndo());
    REQUIRE(s.applyStroke(brush(BrushKind::Deposit, 8.0, 1.0), {dabAt(20, 20)}) == Status::Ok);
    s.discard();
    CHECK(sameBits(s.working(), kept));
}

TEST_CASE("sculpt: undo memory is bounded", "[relief][sculpt]") {
    SculptOptions o;
    o.maxUndoBytes = 3 * 5 * 5 * sizeof(float);  // room for about three small patches
    SculptSession s = start(flat(100, 100, 0.0f), o);
    BrushParams b = brush(BrushKind::Deposit, 4.0, 1.0);  // box of 5x5 cells
    for (int k = 0; k < 10; ++k)
        REQUIRE(s.applyStroke(b, {dabAt(10.0 + 10.0 * k, 50.0)}) == Status::Ok);
    CHECK(s.undoBytes() <= o.maxUndoBytes);
    int undone = 0;
    while (s.undoStroke()) ++undone;
    CHECK(undone >= 1);
    CHECK(undone <= 3);
    // A single stroke bigger than the cap drops history instead of exceeding it.
    SculptSession big = start(flat(100, 100, 0.0f), o);
    REQUIRE(big.applyStroke(b, {dabAt(10, 10)}) == Status::Ok);
    REQUIRE(big.applyStroke(brush(BrushKind::Deposit, 60.0, 1.0), {dabAt(50, 50)}) == Status::Ok);
    CHECK_FALSE(big.canUndo());
    CHECK(big.undoBytes() == 0);
}

TEST_CASE("sculpt: dabs off the field and parameter errors", "[relief][sculpt]") {
    SculptSession s = start(flat(10, 10, 1.0f));
    HeightField before = s.working();
    CHECK(s.applyStroke(brush(BrushKind::Deposit, 4.0, 1.0), {dabAt(-50, -50)}) == Status::Ok);
    CHECK(sameBits(s.working(), before));
    CHECK_FALSE(s.canUndo());
    CHECK(s.applyStroke(brush(BrushKind::Deposit, 0.0, 1.0), {dabAt(5, 5)}) == Status::BadParameter);
    CHECK(s.applyStroke(brush(BrushKind::Deposit, 4.0, -1.0), {dabAt(5, 5)}) == Status::BadParameter);
    CHECK(s.applyStroke(brush(BrushKind::Deposit, 4.0, 1.0, 1.5), {dabAt(5, 5)}) == Status::BadParameter);
    CHECK(s.applyStroke(brush(BrushKind::Deposit, 4.0, 1.0), {dabAt(5, 5, -1.0)}) == Status::BadParameter);
    CHECK(s.applyStroke(brush(BrushKind::Deposit, 4.0, 1.0), {dabAt(std::nan(""), 5)}) == Status::BadParameter);
    CHECK(sameBits(s.working(), before));
    SculptSession none;
    CHECK(none.applyStroke(brush(BrushKind::Deposit), {dabAt(1, 1)}) == Status::ZeroSize);
    CHECK(SculptSession::begin(HeightField{}, {}, none) == Status::ZeroSize);
}

TEST_CASE("sculpt: dabsAlong resampling", "[relief][sculpt]") {
    std::vector<Dab> d;
    CHECK(dabsAlong({}, 1.0, 1.0, d) == Status::BadParameter);
    CHECK(dabsAlong({{0, 0}, {1, 0}}, 0.0, 1.0, d) == Status::BadParameter);
    CHECK(dabsAlong({{0, 0}, {1, 0}}, 1.0, 0.0, d) == Status::BadParameter);
    CHECK(dabsAlong({{0, 0}, {std::nan(""), 0}}, 1.0, 1.0, d) == Status::BadParameter);
    REQUIRE(dabsAlong({{3, 4}, {3, 4}}, 1.0, 5.0, d) == Status::Ok);
    REQUIRE(d.size() == 1);
    CHECK(d[0].weight == 1.0);
    CHECK(d[0].pos.x == 3.0);
    // Length 10, spacing 3 -> 4 pieces of 2.5, dabs at the midpoints.
    REQUIRE(dabsAlong({{0, 0}, {10, 0}}, 3.0, 5.0, d) == Status::Ok);
    REQUIRE(d.size() == 4);
    CHECK(d[0].pos.x == Approx(1.25));
    CHECK(d[3].pos.x == Approx(8.75));
    CHECK(d[1].weight == Approx(0.5));
    CHECK(d[2].delta.x == Approx(2.5));
    double sum = 0;
    for (const Dab& x : d) sum += x.weight;
    CHECK(sum == Approx(2.0));  // length / diameter
    // Exact multiple: no extra piece.
    REQUIRE(dabsAlong({{0, 0}, {9, 0}}, 3.0, 5.0, d) == Status::Ok);
    CHECK(d.size() == 3);
    // Independent of the vertex density (many collinear vertices, a corner).
    std::vector<Point> dense;
    for (int k = 0; k <= 100; ++k) dense.push_back(Point{k * 0.1, 0.0});
    for (int k = 1; k <= 100; ++k) dense.push_back(Point{10.0, k * 0.1});
    std::vector<Dab> a, b;
    REQUIRE(dabsAlong(dense, 0.7, 3.0, a) == Status::Ok);
    REQUIRE(dabsAlong({{0, 0}, {10, 0}, {10, 10}}, 0.7, 3.0, b) == Status::Ok);
    REQUIRE(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
        CHECK(a[i].pos.x == Approx(b[i].pos.x).margin(1e-9));
        CHECK(a[i].pos.y == Approx(b[i].pos.y).margin(1e-9));
    }
    // The corner is followed (a dab past the bend has y > 0).
    CHECK(b.back().pos.x == Approx(10.0).margin(1e-9));
    CHECK(b.back().pos.y > 9.0);
}

TEST_CASE("sculpt: stroke is deterministic and spacing independent", "[relief][sculpt]") {
    const HeightField f = flat(80, 40, 0.0f);
    const std::vector<Point> path = {{10, 20}, {40, 20}, {70, 25}};
    auto run = [&](const BrushParams& b, double spacing) {
        SculptSession s = start(f);
        std::vector<Dab> d;
        REQUIRE(dabsAlong(path, spacing, b.diameter, d) == Status::Ok);
        REQUIRE(s.applyStroke(b, d) == Status::Ok);
        return s.working();
    };
    BrushParams dep = brush(BrushKind::Deposit, 12.0, 1.0, 1.0);
    HeightField a = run(dep, 1.5), a2 = run(dep, 1.5);
    CHECK(sameBits(a, a2));
    // Spacing from diameter/8 down to diameter/64 agrees within 2 % of the full strength.
    HeightField fine = run(dep, 12.0 / 64.0);
    double worst = 0.0;
    for (std::size_t i = 0; i < a.data().size(); ++i)
        worst = std::max(worst, std::fabs(static_cast<double>(a.data()[i]) - static_cast<double>(fine.data()[i])));
    CHECK(worst < 0.02);
    // Total deposited volume is the same to within 0.5 %.
    double va = 0, vf = 0;
    for (float v : a.data()) va += v;
    for (float v : fine.data()) vf += v;
    CHECK(va == Approx(vf).epsilon(0.005));

    // Smooth over a noisy field: spacing independence within 5 % of the field amplitude.
    HeightField noisy = flat(80, 40, 0.0f);
    for (std::size_t j = 0; j < 40; ++j)
        for (std::size_t i = 0; i < 80; ++i) noisy.set(i, j, ((i * 7 + j * 13) % 5 == 0) ? 1.0f : 0.0f);
    BrushParams sm = brush(BrushKind::Smooth, 12.0, 0.3, 1.0);
    auto runSmooth = [&](double spacing) {
        SculptSession s = start(noisy);
        std::vector<Dab> d;
        REQUIRE(dabsAlong(path, spacing, sm.diameter, d) == Status::Ok);
        REQUIRE(s.applyStroke(sm, d) == Status::Ok);
        return s.working();
    };
    HeightField s1 = runSmooth(1.5), s2 = runSmooth(12.0 / 32.0);
    double sworst = 0.0;
    for (std::size_t i = 0; i < s1.data().size(); ++i)
        sworst = std::max(sworst, std::fabs(static_cast<double>(s1.data()[i]) - static_cast<double>(s2.data()[i])));
    CHECK(sworst < 0.05);
}

TEST_CASE("sculpt: custom brush", "[relief][sculpt]") {
    // Stamp: 5 x 3 cells, high on the right half; opaque all over.
    auto cb = std::make_shared<CustomBrush>();
    REQUIRE(HeightField::create(Grid{5, 3, 1.0, Point{0, 0}}, 0.0f, cb->stamp) == Status::Ok);
    for (std::size_t j = 0; j < 3; ++j)
        for (std::size_t i = 3; i < 5; ++i) cb->stamp.set(i, j, 10.0f);
    BrushParams b = brush(BrushKind::Deposit, 10.0, 2.0);  // stamp scaled x2 (5 cells -> 10 mm)
    b.custom = cb;
    SculptSession s = start(flat(30, 30, 0.0f));
    REQUIRE(s.applyStroke(b, {dabAt(15, 15)}) == Status::Ok);
    CHECK(s.working().at(18, 15) == Approx(2.0f));  // right half: profile 1
    CHECK(s.working().at(12, 15) == Approx(0.0f));  // left half: profile 0
    CHECK(s.working().at(25, 15) == 0.0f);           // outside
    // A 90 degree angle turns the high side to the top (+y).
    cb->angleRad = 1.5707963267948966;
    SculptSession r = start(flat(30, 30, 0.0f));
    REQUIRE(r.applyStroke(b, {dabAt(15, 15)}) == Status::Ok);
    CHECK(r.working().at(15, 18) == Approx(2.0f));
    CHECK(r.working().at(15, 12) == Approx(0.0f));
    CHECK(r.working().at(18, 15) == Approx(0.0f).margin(1e-6));
    // Dynamic rotation follows the stroke: moving +y equals a 90 degree angle.
    cb->angleRad = 0.0;
    cb->dynamicRotation = true;
    SculptSession d = start(flat(30, 30, 0.0f));
    REQUIRE(d.applyStroke(b, {dabAt(15, 15, 1.0, Point{0.0, 1.0})}) == Status::Ok);
    CHECK(d.working().at(15, 18) == Approx(2.0f));
    CHECK(d.working().at(15, 12) == Approx(0.0f));
    // Zero movement keeps the base angle.
    SculptSession z = start(flat(30, 30, 0.0f));
    REQUIRE(z.applyStroke(b, {dabAt(15, 15)}) == Status::Ok);
    CHECK(z.working().at(18, 15) == Approx(2.0f));
    // Uniform stamp: profile 1 everywhere inside; transparent stamp cells contribute 0.
    auto u = std::make_shared<CustomBrush>();
    REQUIRE(HeightField::create(Grid{3, 3, 1.0, Point{0, 0}}, 4.0f, u->stamp) == Status::Ok);
    u->stamp.set(0, 0, kTransparent);
    b.custom = u;
    b.diameter = 3.0;
    SculptSession w = start(flat(10, 10, 0.0f));
    REQUIRE(w.applyStroke(b, {dabAt(5, 5)}) == Status::Ok);
    CHECK(w.working().at(5, 5) == Approx(2.0f));
    CHECK(w.working().at(4, 4) == Approx(0.0f));
    // Empty stamp is rejected.
    auto e = std::make_shared<CustomBrush>();
    b.custom = e;
    CHECK(w.applyStroke(b, {dabAt(5, 5)}) == Status::BadParameter);
}

TEST_CASE("sculpt: kernel falloff curve and parameter bounds", "[relief][sculpt]") {
    CHECK(brushKernel(1.25, 10.0, 1.0) == Approx(0.84375));  // u = 0.25 : 1 - (3u^2 - 2u^3)
    CHECK(brushKernel(3.75, 10.0, 1.0) == Approx(0.15625));
    SculptSession s = start(flat(10, 10, 1.0f));
    CHECK(s.applyStroke(brush(BrushKind::Deposit, 4.0, 1.0, -0.1), {dabAt(5, 5)}) == Status::BadParameter);
}

TEST_CASE("sculpt: weight compounds, strength clamps, mode on transparent cells", "[relief][sculpt]") {
    HeightField f = flat(21, 21, 1.0f);
    SculptSession p = start(f);
    REQUIRE(p.applyStroke(brush(BrushKind::Deposit, 12.0, 4.0), {dabAt(10, 10)}) == Status::Ok);
    // Weight 2 at factor 0.5 compounds to 0.75 (not 1.0).
    REQUIRE(p.applyStroke(brush(BrushKind::UndoBrush, 12.0, 0.5), {dabAt(10, 10, 2.0)}) == Status::Ok);
    CHECK(p.working().at(10, 10) == Approx(5.0f - 4.0f * 0.75f));
    // Exactly 0.5 takes the baseline over a transparent baseline cell.
    HeightField t = flat(9, 9, 1.0f);
    t.set(4, 4, kTransparent);
    SculptSession q = start(t);
    BrushParams fill = brush(BrushKind::Deposit, 4.0, 1.0);
    fill.preserveTransparency = false;
    REQUIRE(q.applyStroke(fill, {dabAt(4, 4)}) == Status::Ok);
    REQUIRE(q.applyStroke(brush(BrushKind::UndoBrush, 4.0, 0.5), {dabAt(4, 4)}) == Status::Ok);
    CHECK(std::isnan(q.working().at(4, 4)));
    // Smudge strength above 1 acts as 1.
    HeightField g = flat(15, 9, 0.0f);
    g.set(5, 4, 8.0f);
    SculptSession a = start(g), b = start(g);
    REQUIRE(a.applyStroke(brush(BrushKind::Smudge, 10.0, 1.0), {dabAt(5, 4, 1.0, Point{2.0, 0.0})}) == Status::Ok);
    REQUIRE(b.applyStroke(brush(BrushKind::Smudge, 10.0, 3.0), {dabAt(5, 4, 1.0, Point{2.0, 0.0})}) == Status::Ok);
    CHECK(sameBits(a.working(), b.working()));
    // Raise mode still fills transparent cells when transparency is not preserved.
    BrushParams rm = fill;
    rm.mode = SculptMode::Raise;
    SculptSession r = start(t);
    REQUIRE(r.applyStroke(rm, {dabAt(4, 4)}) == Status::Ok);
    CHECK(r.working().at(4, 4) == Approx(1.0f));
}

TEST_CASE("sculpt: reads just outside the brush box", "[relief][sculpt]") {
    // Smudge pulls a spike from outside the box (source 4 mm behind the box edge cell).
    HeightField g = flat(20, 9, 0.0f);
    g.set(3, 4, 8.0f);
    SculptSession s = start(g);
    BrushParams b = brush(BrushKind::Smudge, 8.0, 1.0);
    REQUIRE(s.applyStroke(b, {dabAt(10, 4, 1.0, Point{4.0, 0.0})}) == Status::Ok);
    CHECK(s.working().at(7, 4) == Approx(8.0f));
    // Smooth edge cell sees the neighbour outside the box.
    HeightField h = flat(20, 9, 0.0f);
    h.set(8, 4, 16.0f);
    SculptSession m = start(h);
    REQUIRE(m.applyStroke(brush(BrushKind::Smooth, 6.0, 1.0), {dabAt(5, 4)}) == Status::Ok);
    CHECK(m.working().at(7, 4) == Approx(2.0f));
}

TEST_CASE("sculpt: custom brush orientation details", "[relief][sculpt]") {
    // Stamp high on its top row (+y).
    auto cb = std::make_shared<CustomBrush>();
    REQUIRE(HeightField::create(Grid{3, 3, 1.0, Point{0, 0}}, 0.0f, cb->stamp) == Status::Ok);
    for (std::size_t i = 0; i < 3; ++i) cb->stamp.set(i, 2, 6.0f);
    BrushParams b = brush(BrushKind::Deposit, 3.0, 1.0);
    b.custom = cb;
    SculptSession s = start(flat(10, 10, 0.0f));
    REQUIRE(s.applyStroke(b, {dabAt(5, 5)}) == Status::Ok);
    CHECK(s.working().at(5, 6) == Approx(1.0f));
    CHECK(s.working().at(5, 4) == Approx(0.0f));
    // Without dynamic rotation a moving dab keeps the base angle.
    SculptSession m = start(flat(10, 10, 0.0f));
    REQUIRE(m.applyStroke(b, {dabAt(5, 5, 1.0, Point{0.0, 1.0})}) == Status::Ok);
    CHECK(sameBits(m.working(), s.working()));
    // Rotated a quarter turn the stamp's +y side points to -x.
    cb->angleRad = 1.5707963267948966;
    SculptSession q = start(flat(10, 10, 0.0f));
    REQUIRE(q.applyStroke(b, {dabAt(5, 5)}) == Status::Ok);
    CHECK(q.working().at(4, 5) == Approx(1.0f));
    CHECK(q.working().at(6, 5) == Approx(0.0f).margin(1e-6));
}

TEST_CASE("sculpt: undo cap exactly reached keeps every stroke", "[relief][sculpt]") {
    SculptOptions o;
    o.maxUndoBytes = 3 * 5 * 5 * sizeof(float);
    SculptSession s = start(flat(100, 100, 0.0f), o);
    BrushParams b = brush(BrushKind::Deposit, 4.0, 1.0);
    for (int k = 0; k < 3; ++k) REQUIRE(s.applyStroke(b, {dabAt(10.0 + 10.0 * k, 50.0)}) == Status::Ok);
    CHECK(s.undoBytes() == o.maxUndoBytes);
    int undone = 0;
    while (s.undoStroke()) ++undone;
    CHECK(undone == 3);
}
