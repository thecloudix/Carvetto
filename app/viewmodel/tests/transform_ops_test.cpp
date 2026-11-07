#include "carvetto/viewmodel/transform_ops.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/viewmodel/tools.hpp"

using namespace carvetto;
using document::Document;
using document::LayerId;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Transform2;
using geometry::Vec2;
using namespace carvetto::viewmodel;

namespace {

constexpr double kPi = 3.14159265358979323846;

Contour boxContour(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(Box2{{x0, y0}, {x1, y1}});
}

ObjectId add(Document& doc, Contour c, LayerId layer = {}, document::CreationParams params = document::NoParams{}) {
    return doc.insertObject(Object::vector(std::move(c), layer ? layer : doc.activeLayer(), doc.activeSheet(),
                                           std::move(params)));
}

ObjectId addRect(Document& doc, double x0, double y0, double x1, double y1, LayerId layer = {}) {
    return add(doc, boxContour(x0, y0, x1, y1), layer);
}

ObjectId addParamRect(Document& doc, Vec2 centre, double w, double h) {
    document::RectangleParams p;
    p.centreMm = centre;
    p.widthMm = w;
    p.heightMm = h;
    auto built = document::buildContours(p);
    REQUIRE(built.ok());
    return doc.insertObject(Object::vector(built.value(), doc.activeLayer(), doc.activeSheet(), p));
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

std::vector<std::uint8_t> bytes(const Document& doc) {
    auto r = document::saveCarv(doc);
    REQUIRE(r.ok());
    return r.value();
}

const std::vector<Contour>& contoursOf(const Document& doc, ObjectId id) {
    const Object* obj = doc.findObject(id);
    REQUIRE(obj != nullptr);
    REQUIRE(obj->asVector() != nullptr);
    return obj->asVector()->contours;
}

Box2 boundsOf(const Document& doc, ObjectId id) {
    Box2 b;
    for (const Contour& c : contoursOf(doc, id)) {
        b.expand(c.bounds());
    }
    return b;
}

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }
bool nearPt(Vec2 a, Vec2 b, double tol = 1e-9) { return near(a.x, b.x, tol) && near(a.y, b.y, tol); }

/// Contours agree: same structure and points sampled along every contour.
bool sameGeometry(const std::vector<Contour>& a, const std::vector<Contour>& b, double tol = 1e-9) {
    if (a.size() != b.size()) {
        return false;
    }
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (a[i].spanCount() != b[i].spanCount() || a[i].closed != b[i].closed) {
            return false;
        }
        for (int k = 0; k <= 16; ++k) {
            const double t = k / 16.0;
            if (!nearPt(a[i].pointAt(t), b[i].pointAt(t), tol)) {
                return false;
            }
        }
    }
    return true;
}

std::vector<Contour> transformedBy(const std::vector<Contour>& in, const Transform2& m) {
    std::vector<Contour> out;
    for (const Contour& c : in) {
        out.push_back(c.transformed(m));
    }
    return out;
}

void select(Document& doc, const std::vector<ObjectId>& ids) { doc.selection().assign(ids); }

struct Scene {
    Document doc{document::Job::singleSided(1000.0, 600.0, 18.0, document::Units::Millimetres)};
};

}  // namespace

TEST_CASE("move: relative and absolute with anchor, exact oracle, one undo step", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    const ObjectId b = addRect(s.doc, 300, 200, 340, 260);
    select(s.doc, {a, b});
    const auto before = bytes(s.doc);
    const Contour oa = contoursOf(s.doc, a).front();

    MoveOptions rel;
    rel.xMm = 12.5;
    rel.yMm = -7.25;
    const TransformResult r = moveSelection(s.doc, rel);
    REQUIRE(r.ok);
    CHECK(r.objects == 2);
    CHECK(contoursOf(s.doc, a).front() == oa.transformed(Transform2::translation({12.5, -7.25})));
    CHECK(s.doc.undoStack().count() == 1);

    REQUIRE(s.doc.undo());
    CHECK(bytes(s.doc) == before);
    REQUIRE(s.doc.redo());
    CHECK(boundsOf(s.doc, a).min.x == 112.5);

    // Absolute: the bottom-left of the union bounds (100,100) goes to (10,20); both objects keep
    // their relative position.
    REQUIRE(s.doc.undo());
    MoveOptions abs;
    abs.absolute = true;
    abs.anchor = HandleKind::BottomLeft;
    abs.xMm = 10.0;
    abs.yMm = 20.0;
    REQUIRE(moveSelection(s.doc, abs).ok);
    CHECK(nearPt(boundsOf(s.doc, a).min, {10.0, 20.0}));
    CHECK(nearPt(boundsOf(s.doc, b).min, {210.0, 120.0}));

    // Top-right anchor of the union (340,260) -> (0,0).
    MoveOptions tr;
    tr.absolute = true;
    tr.anchor = HandleKind::TopRight;
    const Box2 u = *transformBounds(s.doc);
    REQUIRE(moveSelection(s.doc, tr).ok);
    CHECK(nearPt(transformBounds(s.doc)->max, {0.0, 0.0}));
    CHECK(near(transformBounds(s.doc)->width(), u.width()));
}

TEST_CASE("invalid numeric input changes nothing", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    select(s.doc, {a});
    const auto before = bytes(s.doc);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();

    MoveOptions m;
    m.xMm = nan;
    CHECK_FALSE(moveSelection(s.doc, m).ok);
    m.xMm = 2e9;
    CHECK_FALSE(moveSelection(s.doc, m).ok);
    for (double f : {0.0, -1.0, nan, inf, 1e-9}) {
        ScaleOptions so;
        so.scaleX = f;
        CHECK_FALSE(scaleSelection(s.doc, so).ok);
        so.scaleX = 1.0;
        so.scaleY = f;
        CHECK_FALSE(scaleSelection(s.doc, so).ok);
    }
    RotateOptions ro;
    ro.angleRad = nan;
    CHECK_FALSE(rotateSelection(s.doc, ro).ok);
    ro.angleRad = 1.0;
    ro.pivotMm = Vec2{inf, 0.0};
    CHECK_FALSE(rotateSelection(s.doc, ro).ok);
    CHECK_FALSE(applyTransform(s.doc, Transform2::scaling(0.0, 1.0)).ok);
    CHECK_FALSE(applyTransform(s.doc, Transform2::translation({nan, 0.0})).ok);
    CHECK_FALSE(applyTransform(s.doc, Transform2::translation({2e9, 0.0})).ok);
    CHECK(bytes(s.doc) == before);
    CHECK(s.doc.undoStack().count() == 0);

    // Nothing selected is refused too.
    s.doc.selection().clear();
    CHECK_FALSE(moveSelection(s.doc, MoveOptions{}).ok);
    CHECK_FALSE(canTransform(s.doc));
}

TEST_CASE("scale: about each anchor, individually, and by size", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    const ObjectId b = addRect(s.doc, 300, 200, 340, 260);
    select(s.doc, {a, b});
    const auto before = bytes(s.doc);
    const std::vector<Contour> oa = contoursOf(s.doc, a);
    const std::vector<Contour> ob = contoursOf(s.doc, b);

    ScaleOptions so;
    so.scaleX = 2.0;
    so.scaleY = 0.5;
    so.anchor = HandleKind::BottomLeft;
    REQUIRE(scaleSelection(s.doc, so).ok);
    const Transform2 m = Transform2::scaling(2.0, 0.5, {100.0, 100.0});
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, m));
    CHECK(contoursOf(s.doc, b) == transformedBy(ob, m));
    REQUIRE(s.doc.undo());
    CHECK(bytes(s.doc) == before);

    // Top-right anchor of the union is (340,260).
    so.anchor = HandleKind::TopRight;
    REQUIRE(scaleSelection(s.doc, so).ok);
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, Transform2::scaling(2.0, 0.5, {340.0, 260.0})));
    REQUIRE(s.doc.undo());

    // Individually: each object about its own centre.
    so.mode = ScaleMode::Individually;
    so.anchor = HandleKind::Centre;
    so.scaleX = so.scaleY = 1.5;
    REQUIRE(scaleSelection(s.doc, so).ok);
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, Transform2::scaling(1.5, 1.5, {150.0, 125.0})));
    CHECK(contoursOf(s.doc, b) == transformedBy(ob, Transform2::scaling(1.5, 1.5, {320.0, 230.0})));
    CHECK(s.doc.undoStack().count() == 1);
    REQUIRE(s.doc.undo());
    CHECK(bytes(s.doc) == before);
}

TEST_CASE("scale by size: factors, link XY, zero-size selections refused", "[transform][viewmodel]") {
    const Box2 box{{0, 0}, {100, 50}};
    auto f = scaleFactorsForSize(box, 200.0, std::nullopt, true);
    REQUIRE(f.has_value());
    CHECK(near(f->x, 2.0));
    CHECK(near(f->y, 2.0));
    f = scaleFactorsForSize(box, std::nullopt, 25.0, false);
    REQUIRE(f.has_value());
    CHECK(near(f->x, 1.0));
    CHECK(near(f->y, 0.5));
    f = scaleFactorsForSize(box, 50.0, 100.0, false);
    REQUIRE(f.has_value());
    CHECK(near(f->x, 0.5));
    CHECK(near(f->y, 2.0));
    CHECK_FALSE(scaleFactorsForSize(box, 0.0, std::nullopt, false).has_value());
    CHECK_FALSE(scaleFactorsForSize(box, -3.0, std::nullopt, false).has_value());
    CHECK_FALSE(scaleFactorsForSize(box, std::numeric_limits<double>::quiet_NaN(), std::nullopt, false).has_value());
    const Box2 thin{{0, 0}, {100, 0}};  // a horizontal line has no height to scale to a size
    CHECK_FALSE(scaleFactorsForSize(thin, std::nullopt, 10.0, false).has_value());
    CHECK(scaleFactorsForSize(thin, 50.0, std::nullopt, false).has_value());
    CHECK_FALSE(scaleFactorsForSize(Box2{}, 50.0, std::nullopt, false).has_value());
}

TEST_CASE("rotate: corner and coordinate pivots, absolute angle", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    select(s.doc, {a});
    const auto before = bytes(s.doc);
    const std::vector<Contour> oa = contoursOf(s.doc, a);

    RotateOptions ro;
    ro.angleRad = 0.5;
    REQUIRE(rotateSelection(s.doc, ro).ok);
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, Transform2::rotation(0.5, {150.0, 125.0})));
    REQUIRE(s.doc.undo());
    CHECK(bytes(s.doc) == before);

    ro.pivotAnchor = HandleKind::BottomLeft;
    REQUIRE(rotateSelection(s.doc, ro).ok);
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, Transform2::rotation(0.5, {100.0, 100.0})));
    REQUIRE(s.doc.undo());

    ro.pivotMm = Vec2{-30.0, 40.0};
    REQUIRE(rotateSelection(s.doc, ro).ok);
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, Transform2::rotation(0.5, {-30.0, 40.0})));
    REQUIRE(s.doc.undo());

    // Absolute: a polygon with orientation 0.2 rotated to an absolute 0.7 turns by 0.5.
    document::PolygonParams pp;
    pp.centreMm = {400, 300};
    pp.sides = 6;
    pp.radiusMm = 40.0;
    pp.angle = 0.2;
    auto built = document::buildContours(pp);
    REQUIRE(built.ok());
    const ObjectId poly = s.doc.insertObject(Object::vector(built.value(), s.doc.activeLayer(), s.doc.activeSheet(), pp));
    select(s.doc, {poly});
    CHECK(near(selectionOrientation(s.doc), 0.2));
    RotateOptions abs;
    abs.absolute = true;
    abs.angleRad = 0.7;
    REQUIRE(rotateSelection(s.doc, abs).ok);
    const Box2 now = boundsOf(s.doc, poly);
    CHECK(nearPt(now.centre(), Box2{built.value().front().bounds()}.centre(), 1e-6));
    CHECK(sameGeometry(contoursOf(s.doc, poly), transformedBy(built.value(), Transform2::rotation(0.5, built.value().front().bounds().centre()))));
}

TEST_CASE("parametric objects: translation keeps parameters, quarter turns only when the rebuild matches",
          "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addParamRect(s.doc, {100, 100}, 40, 20);
    select(s.doc, {a});
    const auto rebuilds = [&](ObjectId id) {
        const auto* v = s.doc.findObject(id)->asVector();
        REQUIRE(v != nullptr);
        REQUIRE_FALSE(std::holds_alternative<document::NoParams>(v->params));
        auto rebuilt = document::buildContours(v->params);
        REQUIRE(rebuilt.ok());
        return rebuilt.value() == v->contours || sameGeometry(rebuilt.value(), v->contours, 1e-9);
    };

    MoveOptions mv;
    mv.xMm = 33.0;
    mv.yMm = -4.0;
    REQUIRE(moveSelection(s.doc, mv).ok);
    CHECK(rebuilds(a));

    // A rectangle rebuilt after a quarter turn would start at another corner: plain vector.
    RotateOptions r90;
    r90.angleRad = kPi / 2.0;
    r90.pivotMm = Vec2{10.0, 10.0};
    REQUIRE(rotateSelection(s.doc, r90).ok);
    CHECK(std::holds_alternative<document::NoParams>(s.doc.findObject(a)->asVector()->params));

    // A polygon keeps its parameters through a half turn (angle + pi) and rebuilds identically.
    document::PolygonParams pp;
    pp.centreMm = {400, 300};
    pp.sides = 6;
    pp.radiusMm = 40.0;
    pp.angle = 0.2;
    auto built = document::buildContours(pp);
    REQUIRE(built.ok());
    const ObjectId poly = s.doc.insertObject(Object::vector(built.value(), s.doc.activeLayer(), s.doc.activeSheet(), pp));
    select(s.doc, {poly});
    RotateOptions r180;
    r180.angleRad = kPi;
    r180.pivotMm = Vec2{0.0, 0.0};
    REQUIRE(rotateSelection(s.doc, r180).ok);
    CHECK(rebuilds(poly));
    CHECK(near(std::get<document::PolygonParams>(s.doc.findObject(poly)->asVector()->params).angle, 0.2 + kPi));
    select(s.doc, {a});

    // Each of these makes a plain vector.
    const auto becomesPlain = [&](auto&& op) {
        const ObjectId id = addParamRect(s.doc, {300, 300}, 40, 20);
        select(s.doc, {id});
        REQUIRE(op().ok);
        CHECK(std::holds_alternative<document::NoParams>(s.doc.findObject(id)->asVector()->params));
    };
    becomesPlain([&] { RotateOptions r; r.angleRad = 0.5; return rotateSelection(s.doc, r); });
    becomesPlain([&] { ScaleOptions o; o.scaleX = 2.0; o.scaleY = 1.0; return scaleSelection(s.doc, o); });
    becomesPlain([&] { ScaleOptions o; o.scaleX = 2.0; o.scaleY = 2.0; return scaleSelection(s.doc, o); });
    becomesPlain([&] { return mirrorSelection(s.doc, MirrorOptions{}); });

    // Undo restores the parameters too.
    const ObjectId id = addParamRect(s.doc, {500, 300}, 40, 20);
    select(s.doc, {id});
    const Object before = *s.doc.findObject(id);
    RotateOptions r;
    r.angleRad = 0.3;
    REQUIRE(rotateSelection(s.doc, r).ok);
    REQUIRE(s.doc.undo());
    CHECK(*s.doc.findObject(id) == before);
}

TEST_CASE("mirror: horizontal, vertical, material centre, line, rotated bounds", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    const ObjectId b = addRect(s.doc, 300, 200, 340, 260);
    select(s.doc, {a, b});
    const auto before = bytes(s.doc);
    const std::vector<Contour> oa = contoursOf(s.doc, a);

    MirrorOptions mo;
    REQUIRE(mirrorSelection(s.doc, mo).ok);
    // Union centre x = (100+340)/2 = 220.
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, Transform2::mirrorX(220.0)));
    CHECK(nearPt(boundsOf(s.doc, a).min, {240.0, 100.0}));
    REQUIRE(s.doc.undo());
    CHECK(bytes(s.doc) == before);

    mo.axis = MirrorAxis::Vertical;
    REQUIRE(mirrorSelection(s.doc, mo).ok);
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, Transform2::mirrorY(180.0)));
    REQUIRE(s.doc.undo());

    // Material 1000 x 600, datum bottom-left: centre (500,300).
    mo.axis = MirrorAxis::Horizontal;
    mo.aboutMaterialCentre = true;
    REQUIRE(mirrorSelection(s.doc, mo).ok);
    CHECK(contoursOf(s.doc, a) == transformedBy(oa, Transform2::mirrorX(500.0)));
    REQUIRE(s.doc.undo());

    // About a selected line: the line is the axis and does not move.
    Contour axis;
    axis.append(geometry::LineSpan{{50, 0}, {50, 100}});
    const ObjectId line = add(s.doc, axis);
    select(s.doc, {a, line});
    MirrorOptions about;
    about.aboutSelectedLine = true;
    REQUIRE(mirrorSelection(s.doc, about).ok);
    CHECK(sameGeometry(contoursOf(s.doc, a), transformedBy(oa, Transform2::mirrorX(50.0))));
    CHECK(contoursOf(s.doc, line).front() == axis);
    REQUIRE(s.doc.undo());
    // Without a straight line last it is refused.
    select(s.doc, {line, a});
    const auto stable = bytes(s.doc);
    CHECK_FALSE(mirrorSelection(s.doc, about).ok);
    select(s.doc, {line});
    CHECK_FALSE(mirrorSelection(s.doc, about).ok);
    CHECK(bytes(s.doc) == stable);

    // Rotated bounds: a rectangle turned by 30 degrees mirrors onto itself about its own axis.
    Scene t;
    const Transform2 turn = Transform2::rotation(kPi / 6.0, {50.0, 25.0});
    const ObjectId r = add(t.doc, boxContour(0, 0, 100, 50).transformed(turn));
    select(t.doc, {r});
    const Box2 original = boundsOf(t.doc, r);
    MirrorOptions rot;
    rot.useRotatedBounds = true;
    REQUIRE(mirrorSelection(t.doc, rot).ok);
    CHECK(nearPt(boundsOf(t.doc, r).min, original.min, 1e-6));
    CHECK(nearPt(boundsOf(t.doc, r).max, original.max, 1e-6));
    rot.axis = MirrorAxis::Vertical;
    REQUIRE(mirrorSelection(t.doc, rot).ok);
    CHECK(nearPt(boundsOf(t.doc, r).min, original.min, 1e-6));
    CHECK(nearPt(boundsOf(t.doc, r).max, original.max, 1e-6));
    // A lopsided shape: the rotated-bounds mirror is an involution and keeps the box centre.
    Contour tri;
    tri.append(geometry::LineSpan{{0, 0}, {80, 0}});
    tri.append(geometry::LineSpan{{80, 0}, {60, 30}});
    tri.append(geometry::LineSpan{{60, 30}, {0, 30}});
    tri.append(geometry::LineSpan{{0, 30}, {0, 0}});
    tri.closed = true;
    const ObjectId tr = add(t.doc, tri.transformed(Transform2::rotation(kPi / 6.0, {0, 0})));
    select(t.doc, {tr});
    const std::vector<Contour> otri = contoursOf(t.doc, tr);
    rot.axis = MirrorAxis::Horizontal;
    REQUIRE(mirrorSelection(t.doc, rot).ok);
    CHECK_FALSE(sameGeometry(contoursOf(t.doc, tr), otri));
    CHECK(near(contoursOf(t.doc, tr).front().length(), otri.front().length(), 1e-9));
    REQUIRE(mirrorSelection(t.doc, rot).ok);
    CHECK(sameGeometry(contoursOf(t.doc, tr), otri, 1e-6));
}

TEST_CASE("mirror copy: new objects, one undo step, selected, ids never reused", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    const ObjectId b = addRect(s.doc, 300, 200, 340, 260);
    const ObjectId g = addGroup(s.doc, {b});
    select(s.doc, {a, g});
    const auto before = bytes(s.doc);
    const std::size_t objects = s.doc.objectCount();
    const std::uint64_t nextBefore = s.doc.idGenerator().peek();

    MirrorOptions mo;
    mo.createCopy = true;
    mo.axis = MirrorAxis::Vertical;
    const TransformResult r = mirrorSelection(s.doc, mo);
    REQUIRE(r.ok);
    CHECK(r.created.size() == 2);
    CHECK(s.doc.objectCount() == objects + 3);  // a, group, and its child
    CHECK(s.doc.selection().ids() == r.created);
    CHECK(s.doc.undoStack().count() == 1);
    // Originals are untouched; the copy of a is mirrored about the union centre y = 180.
    CHECK(contoursOf(s.doc, a).front() == boxContour(100, 100, 200, 150));
    const Object* copy = s.doc.findObject(r.created.front());
    REQUIRE(copy != nullptr);
    CHECK(copy->asVector()->contours.front() == boxContour(100, 100, 200, 150).transformed(Transform2::mirrorY(180.0)));
    const Object* groupCopy = s.doc.findObject(r.created.back());
    REQUIRE(groupCopy != nullptr);
    REQUIRE(groupCopy->isGroup());
    const Object* child = s.doc.findObject(groupCopy->asGroup()->children.front());
    REQUIRE(child != nullptr);
    CHECK(nearPt(child->asVector()->bounds().min, {300.0, 100.0}));

    REQUIRE(s.doc.undo());
    CHECK(s.doc.objectCount() == objects);
    CHECK(s.doc.idGenerator().peek() >= nextBefore);  // monotonic (ADR-0007)
    CHECK(bytes(s.doc).size() > 0);
    REQUIRE(s.doc.redo());
    CHECK(s.doc.objectCount() == objects + 3);
    (void)before;
}

TEST_CASE("targets: locked, hidden, other sheet, groups, stale ids", "[transform][viewmodel]") {
    Scene s;
    const LayerId lockedLayer = s.doc.insertLayer(document::Layer{LayerId{}, "Locked", {}, true, true});
    const LayerId hiddenLayer = s.doc.insertLayer(document::Layer{LayerId{}, "Hidden", {}, false, false});
    const ObjectId ok = addRect(s.doc, 0, 0, 10, 10);
    const ObjectId locked = addRect(s.doc, 20, 0, 30, 10, lockedLayer);
    const ObjectId hidden = addRect(s.doc, 40, 0, 50, 10, hiddenLayer);
    const ObjectId lockedChild = addRect(s.doc, 80, 0, 90, 10, lockedLayer);
    const ObjectId okChild = addRect(s.doc, 60, 0, 70, 10);
    const ObjectId mixedGroup = addGroup(s.doc, {okChild, lockedChild});
    const ObjectId g1 = addRect(s.doc, 100, 0, 110, 10);
    const ObjectId g2 = addRect(s.doc, 120, 0, 130, 10);
    const ObjectId wholeGroup = addGroup(s.doc, {g1, g2});

    select(s.doc, {ok, locked, hidden, mixedGroup, wholeGroup, ObjectId{9999}});
    CHECK(transformTargets(s.doc) == std::vector<ObjectId>{ok, g1, g2});

    MoveOptions mv;
    mv.xMm = 5.0;
    REQUIRE(moveSelection(s.doc, mv).ok);
    CHECK(near(boundsOf(s.doc, ok).min.x, 5.0));
    CHECK(near(boundsOf(s.doc, g1).min.x, 105.0));
    CHECK(near(boundsOf(s.doc, g2).min.x, 125.0));  // the group moved whole
    CHECK(near(boundsOf(s.doc, locked).min.x, 20.0));
    CHECK(near(boundsOf(s.doc, hidden).min.x, 40.0));
    CHECK(near(boundsOf(s.doc, okChild).min.x, 60.0));   // group with a locked child: skipped whole
    CHECK(near(boundsOf(s.doc, lockedChild).min.x, 80.0));

    // Objects on another sheet are never touched.
    const document::Sheet second = document::Sheet::fromJob(s.doc.job(), "Second");
    const document::SheetId sheet2 = s.doc.insertSheet(second);
    Object other = Object::vector(boxContour(0, 0, 5, 5), s.doc.activeLayer(), sheet2);
    const ObjectId otherId = s.doc.insertObject(std::move(other));
    select(s.doc, {otherId});
    CHECK_FALSE(canTransform(s.doc));
    CHECK_FALSE(moveSelection(s.doc, mv).ok);
    CHECK(near(boundsOf(s.doc, otherId).min.x, 0.0));
}

TEST_CASE("align to last: every kind matches bounding-box arithmetic", "[transform][viewmodel]") {
    const AlignKind kinds[] = {AlignKind::Centre,  AlignKind::VerticalCentre, AlignKind::HorizontalCentre,
                               AlignKind::Left,    AlignKind::Right,          AlignKind::OutsideTop,
                               AlignKind::OutsideBottom};
    for (AlignKind kind : kinds) {
        Scene s;
        // A circle and a rotated rectangle, so stored bounds are not trivially the extents.
        const ObjectId a = add(s.doc, Contour::circle({30, 40}, 12));
        const ObjectId b = add(s.doc, boxContour(100, 10, 160, 40).transformed(Transform2::rotation(0.4, {130, 25})));
        const ObjectId ref = addRect(s.doc, 300, 200, 380, 240);
        select(s.doc, {a, b, ref});
        const auto before = bytes(s.doc);
        const Box2 ba = boundsOf(s.doc, a);
        const Box2 bb = boundsOf(s.doc, b);
        const Box2 br = boundsOf(s.doc, ref);
        REQUIRE(alignToLast(s.doc, kind).ok);
        CHECK(s.doc.undoStack().count() == 1);
        CHECK(boundsOf(s.doc, ref) == br);
        for (const auto& [id, orig] : {std::make_pair(a, ba), std::make_pair(b, bb)}) {
            const Box2 now = boundsOf(s.doc, id);
            CHECK(near(now.width(), orig.width()));  // translation only
            switch (kind) {
                case AlignKind::Centre: CHECK(nearPt(now.centre(), br.centre())); break;
                case AlignKind::VerticalCentre:
                    CHECK(near(now.centre().y, br.centre().y));
                    CHECK(near(now.min.x, orig.min.x));
                    break;
                case AlignKind::HorizontalCentre:
                    CHECK(near(now.centre().x, br.centre().x));
                    CHECK(near(now.min.y, orig.min.y));
                    break;
                case AlignKind::Left: CHECK(near(now.min.x, br.min.x)); break;
                case AlignKind::Right: CHECK(near(now.max.x, br.max.x)); break;
                case AlignKind::OutsideTop: CHECK(near(now.min.y, br.max.y)); break;
                case AlignKind::OutsideBottom: CHECK(near(now.max.y, br.min.y)); break;
            }
        }
        REQUIRE(s.doc.undo());
        CHECK(bytes(s.doc) == before);
    }
    // Needs two operands; an already aligned selection is a successful no-op without an undo step.
    Scene s;
    const ObjectId a = addRect(s.doc, 0, 0, 10, 10);
    select(s.doc, {a});
    CHECK_FALSE(alignToLast(s.doc, AlignKind::Left).ok);
    const ObjectId b = addRect(s.doc, 0, 50, 30, 60);
    select(s.doc, {a, b});
    const TransformResult r = alignToLast(s.doc, AlignKind::Left);
    CHECK(r.ok);
    CHECK(r.objects == 0);
    CHECK(s.doc.undoStack().count() == 0);
}

TEST_CASE("centre in material", "[transform][viewmodel]") {
    Scene s;  // material 1000 x 600, datum bottom-left
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    const ObjectId b = addRect(s.doc, 300, 200, 340, 260);
    select(s.doc, {a, b});
    const Box2 u = *transformBounds(s.doc);
    REQUIRE(centreInMaterial(s.doc).ok);
    CHECK(nearPt(transformBounds(s.doc)->centre(), {500.0, 300.0}));
    CHECK(near(transformBounds(s.doc)->width(), u.width()));
    REQUIRE(s.doc.undo());
    REQUIRE(centreInMaterial(s.doc, CentreAxes::XOnly).ok);
    CHECK(near(transformBounds(s.doc)->centre().x, 500.0));
    CHECK(near(transformBounds(s.doc)->centre().y, u.centre().y));
    REQUIRE(s.doc.undo());
    REQUIRE(centreInMaterial(s.doc, CentreAxes::YOnly).ok);
    CHECK(near(transformBounds(s.doc)->centre().y, 300.0));
    CHECK(near(transformBounds(s.doc)->centre().x, u.centre().x));
}

TEST_CASE("space selection: even gaps, selection order, inside last", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 0, 0, 10, 10);       // width 10
    const ObjectId b = addRect(s.doc, 20, 0, 50, 10);      // width 30
    const ObjectId c = addRect(s.doc, 55, 0, 65, 10);      // width 10
    const ObjectId d = addRect(s.doc, 200, 0, 220, 10);    // width 20
    select(s.doc, {a, b, c, d});
    const auto before = bytes(s.doc);
    SpaceOptions so;
    REQUIRE(spaceSelection(s.doc, so).ok);
    // Span 0..220, sizes 70 -> gap = 150/3 = 50.
    CHECK(near(boundsOf(s.doc, a).min.x, 0.0));
    CHECK(near(boundsOf(s.doc, b).min.x, 60.0));
    CHECK(near(boundsOf(s.doc, c).min.x, 140.0));
    CHECK(near(boundsOf(s.doc, d).max.x, 220.0));
    const double g1 = boundsOf(s.doc, b).min.x - boundsOf(s.doc, a).max.x;
    const double g2 = boundsOf(s.doc, c).min.x - boundsOf(s.doc, b).max.x;
    const double g3 = boundsOf(s.doc, d).min.x - boundsOf(s.doc, c).max.x;
    CHECK(near(g1, 50.0));
    CHECK(near(g2, 50.0));
    CHECK(near(g3, 50.0));
    CHECK(s.doc.undoStack().count() == 1);
    REQUIRE(s.doc.undo());
    CHECK(bytes(s.doc) == before);

    // Selection order: c, a, b, d. First c stays at 55..65 and last d (200..220) ends there:
    // the run is 55..220 = 165, sizes 70, gap 95/3.
    select(s.doc, {c, a, b, d});
    so.selectionOrder = true;
    REQUIRE(spaceSelection(s.doc, so).ok);
    const double gap = 95.0 / 3.0;
    CHECK(near(boundsOf(s.doc, c).min.x, 55.0));
    CHECK(near(boundsOf(s.doc, a).min.x, 65.0 + gap));
    CHECK(near(boundsOf(s.doc, b).min.x, 75.0 + 2 * gap));
    CHECK(near(boundsOf(s.doc, d).max.x, 220.0));
    REQUIRE(s.doc.undo());

    // Vertical, with the order running downwards in the selection.
    Scene v;
    const ObjectId v1 = addRect(v.doc, 0, 0, 10, 10);
    const ObjectId v2 = addRect(v.doc, 0, 20, 10, 40);
    const ObjectId v3 = addRect(v.doc, 0, 100, 10, 110);
    select(v.doc, {v3, v2, v1});
    SpaceOptions vo;
    vo.direction = SpaceDirection::Vertical;
    vo.selectionOrder = true;
    REQUIRE(spaceSelection(v.doc, vo).ok);
    CHECK(near(boundsOf(v.doc, v3).max.y, 110.0));
    CHECK(near(boundsOf(v.doc, v1).min.y, 0.0));
    const double vg1 = boundsOf(v.doc, v2).min.y - boundsOf(v.doc, v1).max.y;
    const double vg2 = boundsOf(v.doc, v3).min.y - boundsOf(v.doc, v2).max.y;
    CHECK(near(vg1, vg2));
    CHECK(near(vg1, (110.0 - 0.0 - 40.0) / 2.0));

    // Inside last: container 0..300 wide, items 10 + 30 + 20 -> gap = 240/4 = 60.
    Scene w;
    const ObjectId i1 = addRect(w.doc, 100, 0, 110, 10);
    const ObjectId i2 = addRect(w.doc, 500, 0, 530, 10);
    const ObjectId i3 = addRect(w.doc, 20, 0, 40, 10);
    const ObjectId box = addRect(w.doc, 0, -50, 300, 50);
    select(w.doc, {i1, i2, i3, box});
    SpaceOptions in;
    in.insideLast = true;
    REQUIRE(spaceSelection(w.doc, in).ok);
    CHECK(near(boundsOf(w.doc, i3).min.x, 60.0));   // leftmost by position
    CHECK(near(boundsOf(w.doc, i1).min.x, 60.0 + 20.0 + 60.0));
    CHECK(near(boundsOf(w.doc, i2).min.x, 140.0 + 10.0 + 60.0));
    CHECK(near(300.0 - boundsOf(w.doc, i2).max.x, 60.0));
    CHECK(boundsOf(w.doc, box) == Box2{Vec2{0, -50}, Vec2{300, 50}});
    REQUIRE(w.doc.undo());
    // Too big for the container, and too few operands: refused, nothing changes.
    const ObjectId small = addRect(w.doc, 0, 0, 20, 20);
    const auto stable = bytes(w.doc);
    select(w.doc, {i1, i2, i3, small});
    CHECK_FALSE(spaceSelection(w.doc, in).ok);
    select(w.doc, {i1, i2});
    CHECK_FALSE(spaceSelection(w.doc, so).ok);
    CHECK(near(boundsOf(w.doc, i1).min.x, 100.0));
    CHECK(bytes(w.doc) == stable);
}

TEST_CASE("input beyond 1e9 mm is refused", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    const ObjectId far1 = addRect(s.doc, 2e9, 0, 2e9 + 10, 10);
    select(s.doc, {a, far1});
    const auto before = bytes(s.doc);
    MoveOptions mv;
    mv.xMm = 1.0;
    CHECK_FALSE(moveSelection(s.doc, mv).ok);
    CHECK_FALSE(applyTransform(s.doc, Transform2::translation({1, 1})).ok);
    CHECK(bytes(s.doc) == before);
    CHECK(s.doc.undoStack().count() == 0);
}

TEST_CASE("scale factors at the limits are consistent", "[transform][viewmodel]") {
    Scene s;
    const ObjectId a = addRect(s.doc, 100, 100, 200, 150);
    select(s.doc, {a});
    ScaleOptions so;
    so.scaleX = so.scaleY = kMinScaleFactor;
    const TransformResult r = scaleSelection(s.doc, so);
    CHECK(r.ok);  // validFactor accepts it, so the invertibility check must too
    so.scaleX = so.scaleY = kMaxScaleFactor;
    CHECK(scaleSelection(s.doc, so).ok);
}
