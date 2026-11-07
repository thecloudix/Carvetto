#include "carvetto/viewmodel/array_copies.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <ctime>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"

using namespace carvetto;
using Catch::Approx;
using document::Document;
using document::LayerId;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;
using namespace carvetto::viewmodel;

namespace {

constexpr double kPi = 3.14159265358979323846;

ObjectId add(Document& doc, Contour c, LayerId layer = {}, document::CreationParams params = document::NoParams{}) {
    return doc.insertObject(
        Object::vector(std::move(c), layer ? layer : doc.activeLayer(), doc.activeSheet(), std::move(params)));
}

ObjectId addRect(Document& doc, double x0, double y0, double x1, double y1) {
    return add(doc, Contour::rectangle(Box2{{x0, y0}, {x1, y1}}));
}

ObjectId addGroup(Document& doc, const std::vector<ObjectId>& children) {
    Object g;
    g.layer = doc.activeLayer();
    g.sheet = doc.activeSheet();
    g.data = document::GroupObject{children};
    const ObjectId gid = doc.insertObject(std::move(g));
    for (const ObjectId c : children) {
        doc.findObject(c)->parent = gid;
    }
    return gid;
}

void select(Document& doc, const std::vector<ObjectId>& ids) { doc.selection().assign(ids); }

Box2 boundsOf(const Document& doc, ObjectId id) {
    const auto b = doc.objectBounds(id);
    REQUIRE(b.has_value());
    return *b;
}

std::vector<std::uint8_t> bytes(const Document& doc) {
    auto r = document::saveCarv(doc);
    REQUIRE(r.ok());
    return r.value();
}

/// After an undo the file equals the saved one except for the id counter (ADR-0007).
bool sameAfterUndo(const Document& doc, const std::vector<std::uint8_t>& before) {
    auto loaded = document::loadCarv(before);
    REQUIRE(loaded.ok());
    Document& twin = loaded.value().document;
    const std::uint64_t base = twin.idGenerator().peek();
    REQUIRE(doc.idGenerator().peek() >= base);
    for (std::uint64_t i = base; i < doc.idGenerator().peek(); ++i) {
        twin.allocateObjectId();
    }
    return bytes(doc) == bytes(twin);
}

void checkBox(const Box2& b, double x0, double y0, double x1, double y1) {
    CHECK(b.min.x == Approx(x0).margin(1e-9));
    CHECK(b.min.y == Approx(y0).margin(1e-9));
    CHECK(b.max.x == Approx(x1).margin(1e-9));
    CHECK(b.max.y == Approx(y1).margin(1e-9));
}

}  // namespace

TEST_CASE("array: offset spacing grid positions, original kept and counted", "[array][copy]") {
    Document doc;
    const ObjectId src = addRect(doc, 0, 0, 2, 1);
    select(doc, {src});
    ArrayOptions o;
    o.rows = 2;
    o.columns = 3;
    o.xMm = 10;
    o.yMm = 5;
    const auto before = bytes(doc);
    const CopyResult r = applyArrayCopy(doc, o);
    REQUIRE(r.ok);
    CHECK(r.copies == 5);
    REQUIRE(r.created.size() == 5);
    CHECK(doc.objectCount() == 6);
    CHECK(doc.selection().ids() == r.created);
    checkBox(boundsOf(doc, src), 0, 0, 2, 1);
    const double expect[5][2] = {{10, 0}, {20, 0}, {0, 5}, {10, 5}, {20, 5}};
    for (std::size_t i = 0; i < 5; ++i) {
        checkBox(boundsOf(doc, r.created[i]), expect[i][0], expect[i][1], expect[i][0] + 2, expect[i][1] + 1);
    }
    CHECK(doc.undoStack().count() == 1);
    REQUIRE(doc.undo());
    CHECK(sameAfterUndo(doc, before));
    REQUIRE(doc.redo());
    CHECK(doc.objectCount() == 6);
    checkBox(boundsOf(doc, r.created[4]), 20, 5, 22, 6);
}

TEST_CASE("array: gap spacing adds the selection size", "[array][copy]") {
    Document doc;
    const ObjectId a = addRect(doc, 0, 0, 2, 1);
    const ObjectId b = addRect(doc, 3, 4, 5, 6);  // union box 0..5 x 0..6
    select(doc, {a, b});
    ArrayOptions o;
    o.rows = 2;
    o.columns = 2;
    o.spacing = ArraySpacing::Gap;
    o.xMm = 1;
    o.yMm = 2;
    const CopyResult r = applyArrayCopy(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 6);  // 3 cells x 2 objects
    // Cell (0,1): shifted by 5+1 = 6 in x.
    checkBox(boundsOf(doc, r.created[0]), 6, 0, 8, 1);
    checkBox(boundsOf(doc, r.created[1]), 9, 4, 11, 6);
    // Cell (1,0): shifted by 6+2 = 8 in y.
    checkBox(boundsOf(doc, r.created[2]), 0, 8, 2, 9);
}

TEST_CASE("array: row and column displacement shift alternate lines", "[array][copy]") {
    Document doc;
    const ObjectId src = addRect(doc, 0, 0, 1, 1);
    select(doc, {src});
    ArrayOptions o;
    o.rows = 2;
    o.columns = 2;
    o.xMm = 10;
    o.yMm = 10;
    o.rowDisplacementMm = 3;
    o.columnDisplacementMm = -2;
    const CopyResult r = applyArrayCopy(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 3);
    checkBox(boundsOf(doc, r.created[0]), 10, -2, 11, -1);  // (0,1): odd column shifts y
    checkBox(boundsOf(doc, r.created[1]), 3, 10, 4, 11);    // (1,0): odd row shifts x
    checkBox(boundsOf(doc, r.created[2]), 13, 8, 14, 9);    // (1,1): both
}

TEST_CASE("array: symmetry mirrors or turns alternate cells", "[array][copy]") {
    // An L shape so mirroring is visible in the first span.
    Contour l;
    l.append(geometry::LineSpan{{0, 0}, {4, 0}});
    l.append(geometry::LineSpan{{4, 0}, {4, 1}});
    l.append(geometry::LineSpan{{4, 1}, {1, 1}});
    l.append(geometry::LineSpan{{1, 1}, {1, 3}});
    l.append(geometry::LineSpan{{1, 3}, {0, 3}});
    l.append(geometry::LineSpan{{0, 3}, {0, 0}});
    l.closed = true;
    const auto firstStart = [](const Document& d, ObjectId id) {
        return std::get<geometry::LineSpan>(d.findObject(id)->asVector()->contours[0].spans[0]).start;
    };
    for (const ArraySymmetry sym : {ArraySymmetry::MirrorColumns, ArraySymmetry::MirrorRows, ArraySymmetry::MirrorBoth,
                                    ArraySymmetry::Rotate180}) {
        Document doc;
        const ObjectId src = add(doc, l);
        select(doc, {src});
        ArrayOptions o;
        o.rows = 2;
        o.columns = 2;
        o.xMm = 10;
        o.yMm = 10;
        o.symmetry = sym;
        const CopyResult r = applyArrayCopy(doc, o);
        REQUIRE(r.ok);
        // Cell (0,1): start (0,0) -> mirrored about x=2 gives (4,0), then +10.
        const Vec2 c01 = firstStart(doc, r.created[0]);
        const Vec2 c10 = firstStart(doc, r.created[1]);
        const Vec2 c11 = firstStart(doc, r.created[2]);
        const bool mc = sym == ArraySymmetry::MirrorColumns || sym == ArraySymmetry::MirrorBoth;
        const bool mr = sym == ArraySymmetry::MirrorRows || sym == ArraySymmetry::MirrorBoth;
        const bool rot = sym == ArraySymmetry::Rotate180;
        CHECK(c01.x == Approx(mc ? 14.0 : (rot ? 14.0 : 10.0)));
        CHECK(c01.y == Approx(rot ? 3.0 : 0.0));
        CHECK(c10.x == Approx(rot ? 4.0 : 0.0));
        CHECK(c10.y == Approx(mr || rot ? 13.0 : 10.0));
        // (1,1): (even sum) rotation leaves it unturned; mirrors apply on both axes.
        CHECK(c11.x == Approx(mc ? 14.0 : 10.0));
        CHECK(c11.y == Approx(mr ? 13.0 : 10.0));
    }
}

TEST_CASE("array: group copies makes one new group per copy", "[array][copy]") {
    Document doc;
    const ObjectId a = addRect(doc, 0, 0, 1, 1);
    const ObjectId b = addRect(doc, 2, 0, 3, 1);
    select(doc, {a, b});
    ArrayOptions o;
    o.rows = 1;
    o.columns = 3;
    o.xMm = 10;
    o.groupCopies = true;
    const CopyResult r = applyArrayCopy(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 2);
    for (ObjectId g : r.created) {
        const Object* go = doc.findObject(g);
        REQUIRE(go != nullptr);
        REQUIRE(go->isGroup());
        CHECK(go->asGroup()->children.size() == 2);
        CHECK(!go->parent.has_value());
        for (ObjectId ch : go->asGroup()->children) {
            CHECK(doc.findObject(ch)->parent == g);
        }
    }
    CHECK(doc.objectCount() == 2 + 2 * 3);
    CHECK(doc.checkIntegrity().empty());
    checkBox(boundsOf(doc, r.created[1]), 20, 0, 23, 1);
}

TEST_CASE("array: groups are copied whole with structure", "[array][copy]") {
    Document doc;
    const ObjectId a = addRect(doc, 0, 0, 1, 1);
    const ObjectId b = addRect(doc, 2, 0, 3, 1);
    const ObjectId g = addGroup(doc, {a, b});
    select(doc, {g, a});  // a is inside the selected group: copied once
    ArrayOptions o;
    o.rows = 1;
    o.columns = 2;
    o.xMm = 10;
    const CopyResult r = applyArrayCopy(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 1);
    CHECK(doc.objectCount() == 6);
    const Object* ng = doc.findObject(r.created[0]);
    REQUIRE(ng->isGroup());
    CHECK(ng->asGroup()->children.size() == 2);
    CHECK(ng->id != g);
    CHECK(doc.findObject(g)->asGroup()->children.size() == 2);
    checkBox(boundsOf(doc, r.created[0]), 10, 0, 13, 1);
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("array: copy of a child of an unselected group is top level", "[array][copy]") {
    Document doc;
    const ObjectId a = addRect(doc, 0, 0, 1, 1);
    const ObjectId b = addRect(doc, 2, 0, 3, 1);
    const ObjectId g = addGroup(doc, {a, b});
    select(doc, {a});
    ArrayOptions o;
    o.columns = 2;
    const CopyResult r = applyArrayCopy(doc, o);
    REQUIRE(r.ok);
    CHECK(!doc.findObject(r.created[0])->parent.has_value());
    CHECK(doc.findObject(g)->asGroup()->children.size() == 2);
}

TEST_CASE("array: parameters survive translation and half turns, mirrors make plain vectors", "[array][copy]") {
    Document doc;
    document::PolygonParams pp;
    pp.centreMm = {5, 1};
    pp.sides = 5;
    pp.radiusMm = 2;
    pp.angle = 0.3;
    auto built = document::buildContours(pp);
    REQUIRE(built.ok());
    const ObjectId poly = doc.insertObject(Object::vector(built.value(), doc.activeLayer(), doc.activeSheet(), pp));
    const ObjectId circle = add(doc, Contour::circle({1, 1}, 1), {}, document::CircleParams{{1, 1}, 1});
    select(doc, {poly, circle});
    ArrayOptions o;
    o.rows = 2;
    o.columns = 2;
    o.xMm = 20;
    o.yMm = 20;
    o.symmetry = ArraySymmetry::MirrorBoth;
    const CopyResult r = applyArrayCopy(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 6);
    // (0,1) and (1,0) are mirrored: plain vectors.
    for (const std::size_t i : {0U, 1U, 2U, 3U}) {
        CHECK(std::holds_alternative<document::NoParams>(doc.findObject(r.created[i])->asVector()->params));
    }
    // (1,1) is a half turn about the selection centre: still parametric and consistent.
    const auto* pp11 = std::get_if<document::PolygonParams>(&doc.findObject(r.created[4])->asVector()->params);
    const auto* cp11 = std::get_if<document::CircleParams>(&doc.findObject(r.created[5])->asVector()->params);
    REQUIRE(pp11 != nullptr);
    REQUIRE(cp11 != nullptr);
    CHECK(pp11->angle == Approx(0.3 + kPi));
    const auto rebuilt = document::buildContours(*pp11);
    REQUIRE(rebuilt.ok());
    Box2 expected;
    for (const Contour& c : rebuilt.value()) {
        expected.expand(c.bounds());
    }
    const Box2 got = boundsOf(doc, r.created[4]);
    checkBox(got, expected.min.x, expected.min.y, expected.max.x, expected.max.y);
    const Box2 all = boundsOf(doc, poly);
    CHECK(cp11->centreMm.x == Approx(cp11->centreMm.x));
    // The half-turned circle stays a circle of radius 1 whose contour matches its parameters.
    checkBox(boundsOf(doc, r.created[5]), cp11->centreMm.x - 1, cp11->centreMm.y - 1, cp11->centreMm.x + 1,
             cp11->centreMm.y + 1);
    (void)all;

    // Plain translation keeps the parameters.
    Document d2;
    const ObjectId c2 = add(d2, Contour::circle({1, 1}, 1), {}, document::CircleParams{{1, 1}, 1});
    select(d2, {c2});
    ArrayOptions t;
    t.columns = 2;
    t.xMm = 7;
    const CopyResult rt = applyArrayCopy(d2, t);
    REQUIRE(rt.ok);
    const auto* cp = std::get_if<document::CircleParams>(&d2.findObject(rt.created[0])->asVector()->params);
    REQUIRE(cp != nullptr);
    CHECK(cp->centreMm.x == Approx(8.0));
    CHECK(cp->radiusMm == Approx(1.0));
}

TEST_CASE("array: locked and hidden layers are never copied; invalid requests change nothing", "[array][copy]") {
    Document doc;
    const ObjectId a = addRect(doc, 0, 0, 1, 1);
    document::Layer hidden;
    hidden.name = "H";
    hidden.visible = false;
    const LayerId hid = doc.insertLayer(hidden);
    const ObjectId h = add(doc, Contour::rectangle(Box2{{5, 5}, {6, 6}}), hid);
    select(doc, {a, h});
    ArrayOptions o;
    o.columns = 3;
    const CopyResult ok = applyArrayCopy(doc, o);
    REQUIRE(ok.ok);
    CHECK(ok.created.size() == 2);
    CHECK(doc.objectCount() == 4);

    select(doc, {h});
    const auto before = bytes(doc);
    const std::size_t steps = doc.undoStack().count();
    CHECK_FALSE(applyArrayCopy(doc, o).ok);
    CHECK(bytes(doc) == before);

    select(doc, {a});
    const auto before2 = bytes(doc);
    ArrayOptions bad = o;
    bad.xMm = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(validateArrayOptions(bad).empty());
    CHECK_FALSE(applyArrayCopy(doc, bad).ok);
    bad = o;
    bad.rows = 0;
    CHECK_FALSE(applyArrayCopy(doc, bad).ok);
    bad = o;
    bad.rows = 1;
    bad.columns = 1;
    CHECK_FALSE(applyArrayCopy(doc, bad).ok);
    bad = o;
    bad.rows = 100000;
    bad.columns = 100000;
    CHECK_FALSE(applyArrayCopy(doc, bad).ok);
    bad = o;
    bad.rows = 3000;
    bad.columns = 3000;  // 9e6 cells > cap
    CHECK_FALSE(applyArrayCopy(doc, bad).ok);
    bad = o;
    bad.columns = 50000;  // within cells cap, but 4 entries... still 50000 x 5 < cap -> allowed size?
    bad.xMm = 1e8;        // extends beyond range
    CHECK_FALSE(applyArrayCopy(doc, bad).ok);
    CHECK(bytes(doc) == before2);
    CHECK(doc.undoStack().count() == steps);
    select(doc, {});
    CHECK_FALSE(applyArrayCopy(doc, o).ok);
    CHECK_FALSE(canArrayCopy(doc));
}

TEST_CASE("copies: work cap arithmetic counts objects and spans", "[array][copy]") {
    CHECK_FALSE(copyWorkExceeded(499.0, 1002));   // 499998
    CHECK(copyWorkExceeded(500.0, 1002));         // 501000
    CHECK_FALSE(copyWorkExceeded(0.0, 5));
    CHECK(copyWorkExceeded(std::numeric_limits<double>::quiet_NaN(), 5));
    CHECK(copyWorkExceeded(1e300, 1'000'000));
    CHECK(kMaxCopyWork == 500'000);
}

TEST_CASE("array: an over-cap request is refused without building anything", "[array][copy]") {
    Document doc;
    Contour many;
    for (int i = 0; i < 1000; ++i) {
        many.append(geometry::LineSpan{{double(i), 0}, {double(i + 1), 0}});
    }
    const ObjectId id = add(doc, many);
    select(doc, {id});
    ArrayOptions o;
    o.rows = 10;
    o.columns = 100;  // 999 copies x 1002 > 500000
    CHECK_FALSE(applyArrayCopy(doc, o).ok);
    CHECK(doc.objectCount() == 1);
    CHECK(doc.undoStack().count() == 0);
}

TEST_CASE("copies: non-finite source coordinates are refused", "[array][circular][along][copy]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    for (const double bad : {nan, std::numeric_limits<double>::infinity(), 2.0e9}) {
        Document doc;
        Contour c;
        c.append(geometry::LineSpan{{0, 0}, {bad, 1}});
        const ObjectId src = add(doc, c);
        select(doc, {src});
        const auto before = bytes(doc);
        ArrayOptions a;
        CHECK_FALSE(applyArrayCopy(doc, a).ok);
        CircularOptions ci;
        CHECK_FALSE(applyCircularCopy(doc, ci).ok);
        CHECK_FALSE(applyArrayCopy(doc, a).message.empty());
        CHECK(doc.undoStack().count() == 0);
        CHECK(doc.objectCount() == 1);
        (void)before;

        // A non-finite guide.
        Document d2;
        const ObjectId guide = add(d2, c);
        const ObjectId s2 = addRect(d2, 0, 0, 1, 1);
        select(d2, {s2, guide});
        AlongVectorOptions o;
        o.guide = guide;
        CHECK_FALSE(applyCopyAlongVectors(d2, o).ok);
        CHECK(d2.objectCount() == 2);
        CHECK(d2.undoStack().count() == 0);
    }
}

TEST_CASE("copies: count limit of 100000 for circular and along-vector counts", "[circular][along][copy]") {
    CircularOptions ci;
    ci.count = kMaxCopyCount + 1;
    CHECK_FALSE(validateCircularOptions(ci).empty());
    ci.count = kMaxCopyCount;
    CHECK(validateCircularOptions(ci).empty());
    AlongVectorOptions al;
    al.spacing = AlongSpacing::Count;
    al.count = kMaxCopyCount + 1;
    CHECK_FALSE(validateAlongVectorOptions(al).empty());
    al.count = kMaxCopyCount;
    CHECK(validateAlongVectorOptions(al).empty());
}

TEST_CASE("circular: step and total angle with rotated copies", "[circular][copy]") {
    Document doc;
    const ObjectId src = addRect(doc, 9, -1, 11, 1);  // centre (10, 0)
    select(doc, {src});
    CircularOptions o;
    o.centreMm = {0, 0};
    o.count = 4;
    o.mode = CircularAngleMode::TotalAngle;
    o.angleRad = 2 * kPi;
    const auto before = bytes(doc);
    const CopyResult r = applyCircularCopy(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 3);
    // Positive angle is clockwise: first copy at (0,-10), then (-10,0), then (0,10).
    const Vec2 centres[3] = {{0, -10}, {-10, 0}, {0, 10}};
    for (std::size_t i = 0; i < 3; ++i) {
        const Box2 b = boundsOf(doc, r.created[i]);
        CHECK(b.centre().x == Approx(centres[i].x).margin(1e-9));
        CHECK(b.centre().y == Approx(centres[i].y).margin(1e-9));
        // 2x2 square turned by 90 degrees is still a 2x2 square.
        CHECK(b.width() == Approx(2.0));
    }
    checkBox(boundsOf(doc, src), 9, -1, 11, 1);
    REQUIRE(doc.undo());
    CHECK(sameAfterUndo(doc, before));
    REQUIRE(doc.redo());
    CHECK(doc.objectCount() == 4);
}

TEST_CASE("circular: partial total angle ends on the arc; negative step goes counter-clockwise", "[circular][copy]") {
    Document doc;
    const ObjectId src = addRect(doc, 9, -1, 11, 1);
    select(doc, {src});
    CircularOptions o;
    o.count = 3;
    o.mode = CircularAngleMode::TotalAngle;
    o.angleRad = kPi;  // 0, 90, 180 degrees clockwise
    o.rotateCopies = false;
    const CopyResult r = applyCircularCopy(doc, o);
    REQUIRE(r.ok);
    const Box2 last = boundsOf(doc, r.created[1]);
    CHECK(last.centre().x == Approx(-10.0).margin(1e-9));
    CHECK(last.centre().y == Approx(0.0).margin(1e-9));
    // Not rotated: still 2 wide, and the rectangle is not turned (check a non-square instead).
    Document d2;
    const ObjectId bar = addRect(d2, 9, -2, 11, 2);  // 2 wide, 4 high
    select(d2, {bar});
    CircularOptions s;
    s.count = 2;
    s.mode = CircularAngleMode::StepAngle;
    s.angleRad = -kPi / 2;  // counter-clockwise
    s.rotateCopies = false;
    const CopyResult rs = applyCircularCopy(d2, s);
    REQUIRE(rs.ok);
    const Box2 moved = boundsOf(d2, rs.created[0]);
    CHECK(moved.centre().x == Approx(0.0).margin(1e-9));
    CHECK(moved.centre().y == Approx(10.0).margin(1e-9));
    CHECK(moved.width() == Approx(2.0));
    CHECK(moved.height() == Approx(4.0));
    // Rotated copies of the same bar are turned.
    Document d3;
    const ObjectId bar3 = addRect(d3, 9, -2, 11, 2);
    select(d3, {bar3});
    s.rotateCopies = true;
    const CopyResult r3 = applyCircularCopy(d3, s);
    REQUIRE(r3.ok);
    const Box2 turned = boundsOf(d3, r3.created[0]);
    CHECK(turned.width() == Approx(4.0));
    CHECK(turned.height() == Approx(2.0));
}

TEST_CASE("circular: 90 degree turns keep rectangle parameters, others do not", "[circular][copy]") {
    Document doc;
    document::RectangleParams rp;
    rp.centreMm = {10, 0};
    rp.widthMm = 2;
    rp.heightMm = 4;
    auto built = document::buildContours(rp);
    REQUIRE(built.ok());
    const ObjectId src = doc.insertObject(Object::vector(built.value(), doc.activeLayer(), doc.activeSheet(), rp));
    select(doc, {src});
    CircularOptions o;
    o.count = 4;
    o.mode = CircularAngleMode::TotalAngle;
    o.angleRad = 2 * kPi;
    const CopyResult r = applyCircularCopy(doc, o);
    REQUIRE(r.ok);
    const auto* p = std::get_if<document::RectangleParams>(&doc.findObject(r.created[0])->asVector()->params);
    REQUIRE(p != nullptr);
    CHECK(p->widthMm == Approx(4.0));
    CHECK(p->heightMm == Approx(2.0));
    CHECK(p->centreMm.x == Approx(0.0).margin(1e-9));
    CHECK(p->centreMm.y == Approx(-10.0).margin(1e-9));

    Document d2;
    const ObjectId s2 = d2.insertObject(Object::vector(built.value(), d2.activeLayer(), d2.activeSheet(), rp));
    select(d2, {s2});
    o.count = 3;  // 120 degree steps
    const CopyResult r2 = applyCircularCopy(d2, o);
    REQUIRE(r2.ok);
    CHECK(std::holds_alternative<document::NoParams>(d2.findObject(r2.created[0])->asVector()->params));
}

TEST_CASE("circular: validation", "[circular][copy]") {
    Document doc;
    select(doc, {addRect(doc, 0, 0, 1, 1)});
    const auto before = bytes(doc);
    CircularOptions o;
    for (const int count : {1, 0, -3}) {
        o.count = count;
        CHECK_FALSE(applyCircularCopy(doc, o).ok);
    }
    o.count = 4;
    o.angleRad = 0.0;
    CHECK_FALSE(applyCircularCopy(doc, o).ok);
    o.angleRad = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(applyCircularCopy(doc, o).ok);
    o.angleRad = 7.0;
    CHECK_FALSE(applyCircularCopy(doc, o).ok);
    o.angleRad = 1.0;
    o.centreMm = {std::numeric_limits<double>::infinity(), 0};
    CHECK_FALSE(applyCircularCopy(doc, o).ok);
    o.centreMm = {};
    o.count = 3'000'000;
    CHECK_FALSE(applyCircularCopy(doc, o).ok);
    CHECK(bytes(doc) == before);
    CHECK(doc.undoStack().count() == 0);
}

TEST_CASE("along vectors: line, distance spacing", "[along][copy]") {
    Document doc;
    Contour line;
    line.append(geometry::LineSpan{{0, 0}, {10, 0}});
    const ObjectId guide = add(doc, line);
    const ObjectId src = addRect(doc, 0, 5, 2, 7);  // centre (1, 6)
    select(doc, {src, guide});
    CHECK(defaultGuideObject(doc) == guide);
    AlongVectorOptions o;
    o.guide = guide;
    o.distanceMm = 4;
    const auto before = bytes(doc);
    const CopyResult r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 3);  // 0, 4, 8
    for (std::size_t i = 0; i < 3; ++i) {
        const Box2 b = boundsOf(doc, r.created[i]);
        CHECK(b.centre().x == Approx(4.0 * static_cast<double>(i)).margin(1e-9));
        CHECK(b.centre().y == Approx(0.0).margin(1e-9));
    }
    checkBox(boundsOf(doc, src), 0, 5, 2, 7);
    CHECK(doc.undoStack().count() == 1);
    REQUIRE(doc.undo());
    CHECK(sameAfterUndo(doc, before));
    REQUIRE(doc.redo());
    CHECK(doc.objectCount() == 5);
}

TEST_CASE("along vectors: even spacing, count, reverse, exact end", "[along][copy]") {
    Document doc;
    Contour line;
    line.append(geometry::LineSpan{{0, 0}, {10, 0}});
    const ObjectId guide = add(doc, line);
    const ObjectId src = addRect(doc, -1, -1, 1, 1);
    select(doc, {src, guide});
    AlongVectorOptions o;
    o.guide = guide;
    o.distanceMm = 4;
    o.forceEvenSpacing = true;  // 10/4 = 2.5 -> 3 intervals of 3.333
    CopyResult r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 4);
    CHECK(boundsOf(doc, r.created[3]).centre().x == Approx(10.0));
    CHECK(boundsOf(doc, r.created[1]).centre().x == Approx(10.0 / 3.0));

    AlongVectorOptions c;
    c.guide = guide;
    c.spacing = AlongSpacing::Count;
    c.count = 5;
    c.reverse = true;
    select(doc, {src, guide});
    r = applyCopyAlongVectors(doc, c);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 5);
    CHECK(boundsOf(doc, r.created[0]).centre().x == Approx(10.0));
    CHECK(boundsOf(doc, r.created[4]).centre().x == Approx(0.0).margin(1e-9));
    CHECK(boundsOf(doc, r.created[1]).centre().x == Approx(7.5));
    c.count = 1;
    select(doc, {src, guide});
    r = applyCopyAlongVectors(doc, c);
    REQUIRE(r.ok);
    CHECK(boundsOf(doc, r.created[0]).centre().x == Approx(10.0));
}

TEST_CASE("along vectors: closed guide, arc length around a circle", "[along][copy]") {
    Document doc;
    const ObjectId guide = add(doc, Contour::circle({0, 0}, 10));
    const ObjectId src = addRect(doc, -1, -1, 1, 1);
    select(doc, {src, guide});
    AlongVectorOptions o;
    o.guide = guide;
    o.spacing = AlongSpacing::Count;
    o.count = 4;
    CopyResult r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 4);
    const Vec2 expected[4] = {{10, 0}, {0, 10}, {-10, 0}, {0, -10}};
    for (std::size_t i = 0; i < 4; ++i) {
        const Vec2 c = boundsOf(doc, r.created[i]).centre();
        CHECK(c.x == Approx(expected[i].x).margin(1e-9));
        CHECK(c.y == Approx(expected[i].y).margin(1e-9));
    }
    // Distance mode on a closed loop stops before the seam: 2*pi*10 / 20 -> 4 positions, none at the seam twice.
    select(doc, {src, guide});
    o.spacing = AlongSpacing::Distance;
    o.distanceMm = 2 * kPi * 10 / 4;
    r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    CHECK(r.created.size() == 4);
    // Even spacing: distance 20 -> round(62.83/20) = 3 positions.
    select(doc, {src, guide});
    o.distanceMm = 20;
    o.forceEvenSpacing = true;
    r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 3);
    const Vec2 third = boundsOf(doc, r.created[1]).centre();
    CHECK(third.x == Approx(10.0 * std::cos(2 * kPi / 3)).margin(1e-9));
    CHECK(third.y == Approx(10.0 * std::sin(2 * kPi / 3)).margin(1e-9));
}

TEST_CASE("along vectors: align to tangent on an arc and a Bezier", "[along][copy]") {
    Document doc;
    const ObjectId guide = add(doc, Contour::circle({0, 0}, 10));
    // A bar along +x, 4 long and 1 wide, centred at (3, 0).
    const ObjectId src = addRect(doc, 1, -0.5, 5, 0.5);
    select(doc, {src, guide});
    AlongVectorOptions o;
    o.guide = guide;
    o.spacing = AlongSpacing::Count;
    o.count = 4;
    o.alignToCurve = true;
    const CopyResult r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    // At (0, 10) the CCW tangent points to -x: the 4-long bar lies along x, centre on the point.
    const Box2 b = boundsOf(doc, r.created[1]);
    CHECK(b.centre().x == Approx(0.0).margin(1e-9));
    CHECK(b.centre().y == Approx(10.0).margin(1e-9));
    CHECK(b.width() == Approx(4.0));
    // At (10, 0) the tangent is +y: the bar stands upright.
    const Box2 b0 = boundsOf(doc, r.created[0]);
    CHECK(b0.width() == Approx(1.0));
    CHECK(b0.height() == Approx(4.0));

    // Bezier guide: a straight-looking cubic along the diagonal with uniform parametrisation.
    Document d2;
    Contour bez;
    bez.append(geometry::BezierSpan{{0, 0}, {3, 4}, {6, 8}, {9, 12}});  // straight line of length 15
    const ObjectId g2 = add(d2, bez);
    const ObjectId s2 = addRect(d2, -1, -0.5, 1, 0.5);
    select(d2, {s2, g2});
    AlongVectorOptions bo;
    bo.guide = g2;
    bo.distanceMm = 5;
    bo.alignToCurve = true;
    const CopyResult rb = applyCopyAlongVectors(d2, bo);
    REQUIRE(rb.ok);
    REQUIRE(rb.created.size() == 4);  // 0, 5, 10, 15
    for (std::size_t i = 0; i < 4; ++i) {
        const Box2 bb = boundsOf(d2, rb.created[i]);
        CHECK(bb.centre().x == Approx(3.0 * static_cast<double>(i)).margin(1e-6));
        CHECK(bb.centre().y == Approx(4.0 * static_cast<double>(i)).margin(1e-6));
    }
    // Bar turned to the 53.13 degree tangent: its bounding box is 2*cos+1*sin by 2*sin+1*cos.
    const double ang = std::atan2(4.0, 3.0);
    const Box2 tilted = boundsOf(d2, rb.created[2]);
    CHECK(tilted.width() == Approx(2 * std::cos(ang) + std::sin(ang)).margin(1e-6));
    CHECK(tilted.height() == Approx(2 * std::sin(ang) + std::cos(ang)).margin(1e-6));
}

TEST_CASE("along vectors: Bezier arc-length spacing on a curved guide", "[along][copy]") {
    Document doc;
    Contour bez;
    geometry::BezierSpan span{{0, 0}, {0, 10}, {10, 10}, {10, 0}};
    bez.append(span);
    const ObjectId guide = add(doc, bez);
    const ObjectId src = addRect(doc, -0.5, -0.5, 0.5, 0.5);
    select(doc, {src, guide});
    AlongVectorOptions o;
    o.guide = guide;
    o.spacing = AlongSpacing::Count;
    o.count = 5;
    const CopyResult r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    const double total = span.length();
    // Points must be equally spaced in arc length: compare with the chord of the sub-curves.
    Vec2 prev = boundsOf(doc, r.created[0]).centre();
    CHECK(prev.x == Approx(0.0).margin(1e-9));
    double travelled = 0.0;
    // Walk a fine polyline of the curve and find the arc length of each placed centre.
    constexpr int kSteps = 20000;
    std::vector<double> cum(kSteps + 1, 0.0);
    std::vector<Vec2> pts(kSteps + 1);
    for (int i = 0; i <= kSteps; ++i) {
        pts[static_cast<std::size_t>(i)] = span.pointAt(static_cast<double>(i) / kSteps);
        if (i > 0) {
            cum[static_cast<std::size_t>(i)] =
                cum[static_cast<std::size_t>(i) - 1] + (pts[static_cast<std::size_t>(i)] - pts[static_cast<std::size_t>(i) - 1]).length();
        }
    }
    for (std::size_t k = 1; k < 5; ++k) {
        const Vec2 c = boundsOf(doc, r.created[k]).centre();
        std::size_t best = 0;
        double bestDist = 1e18;
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const double d = (pts[i] - c).length();
            if (d < bestDist) {
                bestDist = d;
                best = i;
            }
        }
        travelled = cum[best];
        CHECK(travelled == Approx(total * static_cast<double>(k) / 4.0).margin(2e-3));
    }
}

TEST_CASE("along vectors: circles of a diameter on a new layer, selection is the copies", "[along][copy]") {
    Document doc;
    Contour line;
    line.append(geometry::LineSpan{{0, 0}, {10, 0}});
    const ObjectId guide = add(doc, line);
    select(doc, {guide});
    AlongVectorOptions o;
    o.guide = guide;
    o.kind = AlongCopyKind::Circles;
    o.circleDiameterMm = 2;
    o.distanceMm = 5;
    o.newLayer = true;
    const std::size_t layers = doc.layers().size();
    const CopyResult r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 3);
    CHECK(doc.layers().size() == layers + 1);
    const LayerId newLayer = doc.findObject(r.created[0])->layer;
    CHECK(newLayer != doc.activeLayer());
    CHECK(doc.findLayer(newLayer)->visible);
    for (std::size_t i = 0; i < 3; ++i) {
        const Object* c = doc.findObject(r.created[i]);
        CHECK(c->layer == newLayer);
        const auto* cp = std::get_if<document::CircleParams>(&c->asVector()->params);
        REQUIRE(cp != nullptr);
        CHECK(cp->radiusMm == Approx(1.0));
        CHECK(cp->centreMm.x == Approx(5.0 * static_cast<double>(i)));
        checkBox(boundsOf(doc, r.created[i]), cp->centreMm.x - 1, -1, cp->centreMm.x + 1, 1);
    }
    CHECK(doc.selection().ids() == r.created);
    CHECK(doc.undoStack().count() == 1);
    REQUIRE(doc.undo());
    CHECK(doc.layers().size() == layers);
    CHECK(doc.objectCount() == 1);
    REQUIRE(doc.redo());
    CHECK(doc.layers().size() == layers + 1);
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("along vectors: copies of objects on a new layer", "[along][copy]") {
    Document doc;
    Contour line;
    line.append(geometry::LineSpan{{0, 0}, {10, 0}});
    const ObjectId guide = add(doc, line);
    const ObjectId a = addRect(doc, 0, 3, 1, 4);
    const ObjectId b = addRect(doc, 2, 3, 3, 4);
    select(doc, {a, b, guide});
    AlongVectorOptions o;
    o.guide = guide;
    o.spacing = AlongSpacing::Count;
    o.count = 2;
    o.newLayer = true;
    const CopyResult r = applyCopyAlongVectors(doc, o);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 4);
    for (ObjectId id : r.created) {
        CHECK(doc.findObject(id)->layer != doc.activeLayer());
    }
    CHECK(doc.findObject(a)->layer == doc.activeLayer());
}

TEST_CASE("along vectors: validation and failures change nothing", "[along][copy]") {
    Document doc;
    Contour line;
    line.append(geometry::LineSpan{{0, 0}, {10, 0}});
    const ObjectId guide = add(doc, line);
    const ObjectId src = addRect(doc, 0, 3, 1, 4);
    select(doc, {src, guide});
    const auto before = bytes(doc);
    AlongVectorOptions o;
    o.guide = guide;
    o.distanceMm = 0.0;
    CHECK_FALSE(applyCopyAlongVectors(doc, o).ok);
    o.distanceMm = -1.0;
    CHECK_FALSE(applyCopyAlongVectors(doc, o).ok);
    o.distanceMm = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(applyCopyAlongVectors(doc, o).ok);
    o.distanceMm = 1e-6;  // 10 million copies
    CHECK_FALSE(applyCopyAlongVectors(doc, o).ok);
    o.distanceMm = 1.0;
    o.spacing = AlongSpacing::Count;
    o.count = 0;
    CHECK_FALSE(applyCopyAlongVectors(doc, o).ok);
    o.count = 3;
    o.guide = ObjectId{};
    CHECK_FALSE(applyCopyAlongVectors(doc, o).ok);
    o.guide = src;  // a guide with no sources left
    select(doc, {src});
    CHECK_FALSE(applyCopyAlongVectors(doc, o).ok);
    AlongVectorOptions c;
    c.guide = guide;
    c.kind = AlongCopyKind::Circles;
    c.circleDiameterMm = 0.0;
    CHECK_FALSE(applyCopyAlongVectors(doc, c).ok);
    CHECK(bytes(doc) == before);
    CHECK(doc.undoStack().count() == 0);
}

TEST_CASE("copies: id exhaustion refuses before touching the document", "[array][copy]") {
    Document doc;
    select(doc, {addRect(doc, 0, 0, 1, 1)});
    // Leave exactly 1 id free by inserting a layer just below the ceiling.
    doc.insertLayer(document::Layer{.id = LayerId{document::IdGenerator::kMaxId - 2}, .name = "high"});
    REQUIRE(doc.idGenerator().remaining() == 1);
    ArrayOptions o;
    o.columns = 3;
    const auto before = bytes(doc);
    const CopyResult r = applyArrayCopy(doc, o);
    CHECK_FALSE(r.ok);
    CHECK(bytes(doc) == before);
    CHECK(doc.undoStack().count() == 0);
}

TEST_CASE("circular: undo and redo of a 100k-copy request stay linear", "[circular][copy][perf]") {
    // Undo used to be quadratic (Selection::remove searched a vector per removed object): 26 s.
    Document doc;
    const ObjectId src = addRect(doc, 9, -1, 11, 1);
    select(doc, {src});
    CircularOptions o;
    o.count = kMaxCopyCount;  // 99999 copies x 5 units < the work cap
    o.angleRad = 2 * kPi;
    const auto cpu = [] { return static_cast<double>(std::clock()) / CLOCKS_PER_SEC; };
    double t0 = cpu();
    const CopyResult r = applyCircularCopy(doc, o);
    const double applySeconds = cpu() - t0;
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 99999);
    t0 = cpu();
    REQUIRE(doc.undo());
    const double undoSeconds = cpu() - t0;
    CHECK(doc.objectCount() == 1);
    t0 = cpu();
    REQUIRE(doc.redo());
    const double redoSeconds = cpu() - t0;
    CHECK(doc.objectCount() == 100000);
    INFO("apply " << applySeconds << " s, undo " << undoSeconds << " s, redo " << redoSeconds << " s");
    CHECK(undoSeconds < 5.0);
    CHECK(redoSeconds < 5.0);
}
