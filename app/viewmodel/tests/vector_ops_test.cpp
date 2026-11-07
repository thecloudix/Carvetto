#include "carvetto/viewmodel/vector_ops.hpp"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <utility>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
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
using viewmodel::BooleanKind;
using viewmodel::OffsetDirection;
using viewmodel::OffsetOptions;

namespace {

Contour rect(double x0, double y0, double x1, double y1, bool ccw = true) {
    Contour c = Contour::rectangle(Box2{{x0, y0}, {x1, y1}});
    if (!ccw) {
        c.reverse();
    }
    return c;
}

Contour circleAt(Vec2 c, double r) { return Contour::circle(c, r); }

/// A closed shape with a Bezier side and an arc side.
Contour curvy(double x, double y, double s) {
    Contour c;
    c.append(geometry::BezierSpan{{x, y}, {x + s, y - s}, {x + 2 * s, y + s}, {x + 3 * s, y}});
    c.append(geometry::LineSpan{{x + 3 * s, y}, {x + 3 * s, y + 2 * s}});
    c.append(geometry::ArcSpan::fromCentre({x + 1.5 * s, y + 2 * s}, {x + 3 * s, y + 2 * s}, {x, y + 2 * s},
                                           geometry::ArcDirection::CCW));
    c.append(geometry::LineSpan{{x, y + 2 * s}, {x, y}});
    c.closed = true;
    return c;
}

Contour openLine(Vec2 a, Vec2 b) {
    Contour c;
    c.append(geometry::LineSpan{a, b});
    return c;
}

ObjectId add(Document& doc, std::vector<Contour> cs, LayerId layer = {}) {
    return doc.insertObject(Object::vector(std::move(cs), layer ? layer : doc.activeLayer(), doc.activeSheet()));
}
ObjectId add(Document& doc, Contour c, LayerId layer = {}) { return add(doc, std::vector<Contour>{std::move(c)}, layer); }

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

bool sameBytes(const Document& doc, const std::vector<std::uint8_t>& expected) { return bytes(doc) == expected; }

/// After an undo the file equals the one saved before the operation, except for `next_id`: ids
/// are never reused (ADR-0007), so the counter stays advanced. The expected file is the earlier
/// one re-loaded with its counter advanced by the same number of ids.
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

double regionArea(const std::vector<Contour>& cs) {
    double a = 0.0;
    for (const Contour& c : cs) {
        if (c.closed) {
            a += geometry::signedArea(c);
        }
    }
    return a;
}

/// Sum of signed areas over every vector object in `ids`, and their contour count.
std::pair<double, std::size_t> measure(const Document& doc, const std::vector<ObjectId>& ids) {
    double a = 0.0;
    std::size_t n = 0;
    for (ObjectId id : ids) {
        const Object* o = doc.findObject(id);
        REQUIRE(o != nullptr);
        REQUIRE(o->isVector());
        a += regionArea(o->asVector()->contours);
        n += o->asVector()->contours.size();
    }
    return {a, n};
}

std::pair<double, std::size_t> measure(const std::vector<Contour>& cs) {
    return {regionArea(cs), cs.size()};
}

void select(Document& doc, const std::vector<ObjectId>& ids) { doc.selection().assign(ids); }

/// Deterministic scene of shapes, some with holes, arcs and Beziers, overlapping in clusters.
std::vector<std::vector<Contour>> scene(unsigned seed, int n) {
    std::uint32_t s = seed * 2654435761u + 12345u;
    const auto rnd = [&s]() {
        s = s * 1664525u + 1013904223u;
        return static_cast<double>((s >> 8) & 0xFFFF) / 65536.0;
    };
    std::vector<std::vector<Contour>> out;
    for (int i = 0; i < n; ++i) {
        const double x = rnd() * 30.0;
        const double y = rnd() * 30.0;
        const double size = 6.0 + rnd() * 10.0;
        switch (i % 4) {
            case 0:
                out.push_back({rect(x, y, x + size, y + size)});
                break;
            case 1:
                out.push_back({circleAt({x, y}, size * 0.5)});
                break;
            case 2:
                out.push_back({rect(x, y, x + size, y + size), rect(x + size * 0.25, y + size * 0.25, x + size * 0.75, y + size * 0.75, false)});
                break;
            default:
                out.push_back({curvy(x, y, size / 4.0)});
                break;
        }
    }
    return out;
}

}  // namespace

TEST_CASE("vector_ops: weld of overlapping rectangles", "[vector_ops]") {
    Document doc;
    const ObjectId a = add(doc, rect(0, 0, 10, 10));
    const ObjectId b = add(doc, rect(5, 5, 15, 15));
    select(doc, {a, b});
    REQUIRE(viewmodel::canWeld(doc));
    const auto before = bytes(doc);

    const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    REQUIRE(r.ok);
    REQUIRE(r.message.empty());
    REQUIRE(r.created.size() == 1);
    CHECK(r.removed == 2);
    CHECK_FALSE(doc.hasObject(a));
    CHECK_FALSE(doc.hasObject(b));
    CHECK(doc.selection().ids() == r.created);
    CHECK(measure(doc, r.created).first == Approx(175.0).margin(1e-6));
    CHECK(std::get<document::NoParams>(doc.findObject(r.created[0])->asVector()->params) == document::NoParams{});
    CHECK(doc.findObject(r.created[0])->layer == doc.activeLayer());

    REQUIRE(doc.undoStack().count() == 1);
    REQUIRE(doc.undo());
    CHECK(sameAfterUndo(doc, before));
    REQUIRE(doc.redo());
    CHECK(measure(doc, r.created).first == Approx(175.0).margin(1e-6));
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("vector_ops: holes stay holes and disjoint regions become separate objects", "[vector_ops]") {
    Document doc;
    const ObjectId frame = add(doc, {rect(0, 0, 20, 20), rect(5, 5, 15, 15, false)});
    const ObjectId bar = add(doc, rect(18, 8, 30, 12));  // crosses the frame's right side
    const ObjectId island = add(doc, circleAt({100, 100}, 5));
    select(doc, {frame, bar, island});
    const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    REQUIRE(r.ok);
    REQUIRE(r.created.size() == 2);
    // The welded frame keeps its hole: one object with 2 contours; the circle has 1.
    std::size_t withHole = 0;
    for (ObjectId id : r.created) {
        const auto& cs = doc.findObject(id)->asVector()->contours;
        if (cs.size() == 2) {
            ++withHole;
            CHECK(geometry::signedArea(cs[0]) > 0.0);
            CHECK(geometry::signedArea(cs[1]) < 0.0);
            CHECK(regionArea(cs) == Approx(400.0 - 100.0 + 10.0 * 4.0).margin(1e-4));
        }
    }
    CHECK(withHole == 1);
}

TEST_CASE("vector_ops: results equal the core functions on generated scenes", "[vector_ops]") {
    for (unsigned seed = 1; seed <= 6; ++seed) {
        const auto shapes = scene(seed, 6);
        const BooleanKind kinds[] = {BooleanKind::Weld, BooleanKind::Subtract, BooleanKind::Intersect,
                                     BooleanKind::Exclude};
        for (const BooleanKind kind : kinds) {
            CAPTURE(seed, static_cast<int>(kind));
            Document doc;
            std::vector<ObjectId> ids;
            for (const auto& s : shapes) {
                ids.push_back(add(doc, s));
            }
            select(doc, ids);
            const auto before = bytes(doc);

            // Oracle: the core calls on each shape's even-odd region.
            const auto tol = geometry::Tolerance::defaults();
            std::vector<std::vector<Contour>> regions;
            for (const auto& s : shapes) {
                regions.push_back(geometry::unionContours(s, tol, geometry::FillRule::EvenOdd));
            }
            std::vector<Contour> expect;
            if (kind == BooleanKind::Weld) {
                std::vector<Contour> all;
                for (auto& r : regions) {
                    all.insert(all.end(), r.begin(), r.end());
                }
                expect = geometry::unionContours(all, tol);
            } else if (kind == BooleanKind::Exclude) {
                expect = regions[0];
                for (std::size_t i = 1; i < regions.size(); ++i) {
                    expect = geometry::booleanOp(geometry::BooleanOp::Xor, expect, regions[i], tol).closed;
                }
            } else {
                const auto op = kind == BooleanKind::Subtract ? geometry::BooleanOp::Difference
                                                              : geometry::BooleanOp::Intersection;
                for (std::size_t i = 0; i + 1 < regions.size(); ++i) {
                    auto part = geometry::booleanOp(op, regions[i], regions.back(), tol).closed;
                    expect.insert(expect.end(), part.begin(), part.end());
                }
            }
            const auto r = viewmodel::applyBoolean(doc, kind);
            if (expect.empty()) {
                CHECK_FALSE(r.ok);
                CHECK(sameBytes(doc, before));
                continue;
            }
            REQUIRE(r.ok);
            const auto got = measure(doc, r.created);
            const auto want = measure(expect);
            CHECK(got.first == Approx(want.first).margin(1e-6));
            CHECK(got.second == want.second);  // contour count (outers + holes)
            // Hole nesting: every hole sits in the object of an outer that contains it.
            for (ObjectId id : r.created) {
                const auto& cs = doc.findObject(id)->asVector()->contours;
                REQUIRE_FALSE(cs.empty());
                CHECK(geometry::signedArea(cs[0]) > 0.0);
                for (std::size_t i = 1; i < cs.size(); ++i) {
                    CHECK(geometry::signedArea(cs[i]) < 0.0);
                    CHECK(geometry::locatePoint(cs[0], cs[i].startPoint(), geometry::FillRule::NonZero, tol) !=
                          geometry::PointLocation::Outside);
                }
            }
            CHECK(doc.undoStack().count() == 1);
            REQUIRE(doc.undo());
            CHECK(sameAfterUndo(doc, before));
        }
    }
}

TEST_CASE("vector_ops: subtract and intersect use the last-selected shape", "[vector_ops]") {
    Document doc;
    const ObjectId a = add(doc, rect(0, 0, 10, 10));
    const ObjectId b = add(doc, rect(20, 0, 30, 10));
    const ObjectId cutter = add(doc, rect(5, -5, 25, 15));
    select(doc, {a, b, cutter});
    const auto sub = viewmodel::applyBoolean(doc, BooleanKind::Subtract);
    REQUIRE(sub.ok);
    CHECK(sub.created.size() == 2);
    CHECK(measure(doc, sub.created).first == Approx(50.0 + 50.0).margin(1e-6));
    REQUIRE(doc.undo());

    select(doc, {a, b, cutter});
    const auto isect = viewmodel::applyBoolean(doc, BooleanKind::Intersect);
    REQUIRE(isect.ok);
    CHECK(measure(doc, isect.created).first == Approx(50.0 + 50.0).margin(1e-6));
    REQUIRE(doc.undo());

    // Selection order matters: the cutter first means a and b are subtracted from it... no: last wins.
    select(doc, {cutter, a});
    const auto rev = viewmodel::applyBoolean(doc, BooleanKind::Subtract);
    REQUIRE(rev.ok);
    CHECK(measure(doc, rev.created).first == Approx(20.0 * 20.0 - 50.0).margin(1e-6));
}

TEST_CASE("vector_ops: errors never change the document", "[vector_ops]") {
    Document doc;
    const ObjectId a = add(doc, rect(0, 0, 10, 10));
    const ObjectId b = add(doc, rect(50, 50, 60, 60));
    const ObjectId line = add(doc, openLine({0, 0}, {5, 5}));
    const auto before = bytes(doc);
    const std::size_t steps = doc.undoStack().count();

    select(doc, {});
    CHECK_FALSE(viewmodel::canWeld(doc));
    CHECK_FALSE(viewmodel::canOffset(doc));
    CHECK_FALSE(viewmodel::applyBoolean(doc, BooleanKind::Weld).ok);

    select(doc, {a});
    CHECK_FALSE(viewmodel::canWeld(doc));
    CHECK(viewmodel::applyBoolean(doc, BooleanKind::Weld).message == "Select at least two closed vectors.");

    select(doc, {a, line});  // the open vector is not an operand
    CHECK_FALSE(viewmodel::canSubtract(doc));
    select(doc, {line});
    CHECK(viewmodel::applyBoolean(doc, BooleanKind::Weld).message == "Select closed vectors first.");

    select(doc, {a, b});
    CHECK(viewmodel::canIntersect(doc));
    const auto none = viewmodel::applyBoolean(doc, BooleanKind::Intersect);  // disjoint
    CHECK_FALSE(none.ok);
    CHECK(none.message == "Nothing is left after the operation.");
    CHECK(none.created.empty());

    CHECK(sameBytes(doc, before));
    CHECK(doc.undoStack().count() == steps);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{a, b});
}

TEST_CASE("vector_ops: hidden and locked layers are never touched", "[vector_ops]") {
    Document doc;
    const LayerId base = doc.activeLayer();
    const LayerId hidden = doc.insertLayer(document::Layer{{}, "Hidden", {}, false, false});
    const LayerId locked = doc.insertLayer(document::Layer{{}, "Locked", {}, true, true});
    const ObjectId a = add(doc, rect(0, 0, 10, 10), base);
    const ObjectId b = add(doc, rect(5, 5, 15, 15), base);
    const ObjectId h = add(doc, rect(2, 2, 12, 12), hidden);
    const ObjectId l = add(doc, rect(3, 3, 13, 13), locked);
    select(doc, {a, h, l, b});
    const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    REQUIRE(r.ok);
    CHECK(doc.hasObject(h));
    CHECK(doc.hasObject(l));
    CHECK_FALSE(doc.hasObject(a));
    CHECK(measure(doc, r.created).first == Approx(175.0).margin(1e-6));

    // Only hidden/locked objects selected: not enabled, offset reports nothing to do.
    select(doc, {h, l});
    CHECK_FALSE(viewmodel::canWeld(doc));
    CHECK_FALSE(viewmodel::canOffset(doc));
    OffsetOptions o;
    CHECK_FALSE(viewmodel::applyOffset(doc, o).ok);
}

TEST_CASE("vector_ops: a group is operated on through its leaves, on their layers", "[vector_ops]") {
    Document doc;
    const LayerId other = doc.insertLayer(document::Layer{{}, "Other", {}, true, false});
    const ObjectId a = add(doc, rect(0, 0, 10, 10));
    const ObjectId b = add(doc, rect(8, 0, 18, 10));
    const ObjectId g = addGroup(doc, {a, b});
    const ObjectId c = add(doc, rect(0, 8, 18, 14), other);

    select(doc, {g, c});
    REQUIRE(viewmodel::canWeld(doc));
    auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    REQUIRE(r.ok);
    CHECK(r.removed == 3);
    CHECK_FALSE(doc.hasObject(g));  // the emptied group goes with its leaves
    CHECK(measure(doc, r.created).first == Approx(180.0 + 108.0 - 36.0).margin(1e-6));
    CHECK(doc.findObject(r.created[0])->layer == doc.activeLayer());  // first operand's layer
    CHECK(doc.checkIntegrity().empty());
    REQUIRE(doc.undo());

    // Subtract with a group as the last operand: the group acts as the union of its leaves.
    select(doc, {c, g});
    r = viewmodel::applyBoolean(doc, BooleanKind::Subtract);
    REQUIRE(r.ok);
    CHECK(measure(doc, r.created).first == Approx(18.0 * 6.0 - 18.0 * 2.0).margin(1e-6));
    CHECK(doc.findObject(r.created[0])->layer == other);
    REQUIRE(doc.undo());

    // A leaf selected together with its group counts once.
    select(doc, {a, g, c});
    r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    REQUIRE(r.ok);
    CHECK(r.removed == 3);
}

TEST_CASE("vector_ops: offset of closed vectors matches the core", "[vector_ops]") {
    for (const auto join : {geometry::JoinType::Round, geometry::JoinType::Sharp, geometry::JoinType::Square}) {
        for (const auto dir : {OffsetDirection::Outwards, OffsetDirection::Inwards}) {
            Document doc;
            const std::vector<Contour> frame = {rect(0, 0, 40, 30), rect(10, 10, 30, 20, false)};
            const ObjectId a = add(doc, frame);
            const ObjectId b = add(doc, curvy(60, 0, 8));
            select(doc, {a, b});
            const auto before = bytes(doc);

            OffsetOptions o;
            o.distanceMm = 2.0;
            o.direction = dir;
            o.join = join;
            o.deleteOriginals = false;
            const double d = dir == OffsetDirection::Outwards ? 2.0 : -2.0;
            const auto tol = geometry::Tolerance::defaults();
            const auto wantA = measure(geometry::offsetContours(frame, d, join, tol));
            const auto wantB = measure(geometry::offsetContours({curvy(60, 0, 8)}, d, join, tol));

            const auto r = viewmodel::applyOffset(doc, o);
            REQUIRE(r.ok);
            REQUIRE(r.created.size() == 2);
            CHECK(r.removed == 0);
            CHECK(doc.hasObject(a));
            CHECK(doc.hasObject(b));
            const auto gotA = measure(doc, {r.created[0]});
            const auto gotB = measure(doc, {r.created[1]});
            CHECK(gotA.first == Approx(wantA.first).margin(1e-9));
            CHECK(gotA.second == wantA.second);
            CHECK(gotB.first == Approx(wantB.first).margin(1e-9));
            CHECK(gotB.second == wantB.second);
            CHECK(doc.selection().ids() == r.created);
            REQUIRE(doc.undo());
            CHECK(sameAfterUndo(doc, before));
        }
    }
}

TEST_CASE("vector_ops: offset of open vectors, sides and delete originals", "[vector_ops]") {
    Document doc;
    Contour poly;
    poly.append(geometry::LineSpan{{0, 0}, {20, 0}});
    poly.append(geometry::LineSpan{{20, 0}, {20, 20}});
    const ObjectId a = add(doc, poly);
    select(doc, {a});

    OffsetOptions o;
    o.distanceMm = 3.0;
    o.join = geometry::JoinType::Sharp;
    const auto tol = geometry::Tolerance::defaults();
    for (const auto dir : {OffsetDirection::Outwards, OffsetDirection::Inwards}) {
        o.direction = dir;
        o.deleteOriginals = dir == OffsetDirection::Inwards;
        const auto side = dir == OffsetDirection::Outwards ? geometry::OffsetSide::Right : geometry::OffsetSide::Left;
        const auto want = geometry::offsetOpenContour(poly, 3.0, side, o.join, tol);
        const auto r = viewmodel::applyOffset(doc, o);
        REQUIRE(r.ok);
        REQUIRE(r.created.size() == 1);
        const auto& got = doc.findObject(r.created[0])->asVector()->contours;
        REQUIRE(got.size() == want.size());
        for (std::size_t i = 0; i < got.size(); ++i) {
            CHECK(got[i] == want[i]);
        }
        CHECK(doc.hasObject(a) == (dir == OffsetDirection::Outwards));
        // Right of travel (+x then +y) is down/right: the chain lies below the first leg.
        CHECK(doc.findObject(r.created[0])->asVector()->contours[0].bounds().min.y ==
              Approx(dir == OffsetDirection::Outwards ? -3.0 : 3.0).margin(1e-9));
        if (dir == OffsetDirection::Inwards) {
            REQUIRE(doc.undo());
            CHECK(doc.hasObject(a));
        }
        select(doc, {a});
        doc.undo();  // drop the outwards result too (first iteration) so ids stay simple
        select(doc, {a});
    }
}

TEST_CASE("vector_ops: offset delete originals keeps sources that vanish", "[vector_ops]") {
    Document doc;
    const ObjectId big = add(doc, rect(0, 0, 40, 40));
    const ObjectId tiny = add(doc, rect(100, 0, 103, 3));
    select(doc, {big, tiny});
    OffsetOptions o;
    o.distanceMm = 5.0;
    o.direction = OffsetDirection::Inwards;
    o.deleteOriginals = true;
    const auto r = viewmodel::applyOffset(doc, o);
    REQUIRE(r.ok);
    CHECK(r.created.size() == 1);
    CHECK(measure(doc, r.created).first == Approx(30.0 * 30.0).margin(1e-6));
    CHECK_FALSE(doc.hasObject(big));
    CHECK(doc.hasObject(tiny));  // collapsed: nothing produced, so the original stays

    select(doc, {tiny});
    const auto before = bytes(doc);
    const auto none = viewmodel::applyOffset(doc, o);
    CHECK_FALSE(none.ok);
    CHECK(none.message == "Nothing is left after the offset.");
    CHECK(sameBytes(doc, before));
}

TEST_CASE("vector_ops: offset option validation", "[vector_ops]") {
    OffsetOptions o;
    CHECK(viewmodel::validateOffsetOptions(o).empty());
    o.distanceMm = 0.0;
    CHECK_FALSE(viewmodel::validateOffsetOptions(o).empty());
    o.distanceMm = -1.0;
    CHECK_FALSE(viewmodel::validateOffsetOptions(o).empty());
    o.distanceMm = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(viewmodel::validateOffsetOptions(o).empty());
    o.distanceMm = std::numeric_limits<double>::infinity();
    CHECK_FALSE(viewmodel::validateOffsetOptions(o).empty());
    o.distanceMm = 2e6;
    CHECK_FALSE(viewmodel::validateOffsetOptions(o).empty());
    o.distanceMm = 1.0;
    o.mitreLimit = 0.5;
    CHECK_FALSE(viewmodel::validateOffsetOptions(o).empty());

    Document doc;
    const ObjectId a = add(doc, rect(0, 0, 10, 10));
    select(doc, {a});
    const auto before = bytes(doc);
    o.mitreLimit = 2.0;
    o.distanceMm = -3.0;
    CHECK_FALSE(viewmodel::applyOffset(doc, o).ok);
    CHECK(sameAfterUndo(doc, before));
}

TEST_CASE("vector_ops: degenerate, tiny and huge inputs neither crash nor hang", "[vector_ops]") {
    Document doc;
    // Zero-area, collinear, sliver, tiny and huge rectangles.
    Contour collinear;
    collinear.append(geometry::LineSpan{{0, 0}, {10, 0}});
    collinear.append(geometry::LineSpan{{10, 0}, {20, 0}});
    collinear.append(geometry::LineSpan{{20, 0}, {0, 0}});
    collinear.closed = true;
    std::vector<ObjectId> ids = {add(doc, collinear),
                                 add(doc, rect(0, 0, 1e-9, 1e-9)),
                                 add(doc, rect(0, 0, 1000, 1e-7)),
                                 add(doc, rect(-1e7, -1e7, 1e7, 1e7)),
                                 add(doc, rect(5, 5, 6, 6)),
                                 add(doc, std::vector<Contour>{})};
    select(doc, ids);
    for (const auto kind : {BooleanKind::Weld, BooleanKind::Subtract, BooleanKind::Intersect, BooleanKind::Exclude}) {
        const auto before = bytes(doc);
        const auto r = viewmodel::applyBoolean(doc, kind);
        if (r.ok) {
            CHECK(doc.checkIntegrity().empty());
            REQUIRE(doc.undo());
        } else {
            CHECK_FALSE(r.message.empty());
        }
        CHECK(sameAfterUndo(doc, before));
        select(doc, ids);
    }
    OffsetOptions o;
    for (const double d : {1e-9, 1e-3, 5.0, 1e5}) {
        o.distanceMm = d;
        for (const auto dir : {OffsetDirection::Outwards, OffsetDirection::Inwards}) {
            o.direction = dir;
            const auto r = viewmodel::applyOffset(doc, o);
            if (r.ok) {
                REQUIRE(doc.undo());
            }
            select(doc, ids);
        }
    }
}

TEST_CASE("vector_ops: non-finite and out-of-range coordinates are refused", "[vector_ops]") {
    Document doc;
    const ObjectId a = add(doc, rect(0, 0, 10, 10));
    Contour nanShape;
    const double nanv = std::numeric_limits<double>::quiet_NaN();
    nanShape.append(geometry::LineSpan{{0, 0}, {nanv, 0}});
    nanShape.append(geometry::LineSpan{{nanv, 0}, {5, 5}});
    nanShape.append(geometry::LineSpan{{5, 5}, {0, 0}});
    nanShape.closed = true;
    const ObjectId nan = add(doc, nanShape);
    const ObjectId huge = add(doc, rect(0, 0, 5e9, 5));
    const auto before = bytes(doc);
    for (const ObjectId bad : {nan, huge}) {
        select(doc, {a, bad});
        const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
        CHECK_FALSE(r.ok);
        CHECK_FALSE(r.message.empty());
        OffsetOptions o;
        const auto ro = viewmodel::applyOffset(doc, o);
        CHECK_FALSE(ro.ok);
        CHECK_FALSE(ro.message.empty());
    }
    CHECK(doc.undoStack().count() == 0);
    CHECK(sameBytes(doc, before));
}

TEST_CASE("vector_ops: welding 2000 objects is fast", "[vector_ops]") {
    Document doc;
    std::vector<ObjectId> ids;
    // 50 x 40 grid of 12 mm squares at 10 mm pitch: neighbours overlap into one region with holes
    // left by nothing; every other square is a circle so arcs take part.
    for (int j = 0; j < 40; ++j) {
        for (int i = 0; i < 50; ++i) {
            const double x = i * 10.0;
            const double y = j * 10.0;
            ids.push_back(add(doc, ((i + j) % 2 == 0) ? rect(x, y, x + 12, y + 12) : circleAt({x + 6, y + 6}, 6.5)));
        }
    }
    select(doc, ids);
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "[vector_ops] 2000-object weld: " << sec << " s\n";
    REQUIRE(r.ok);
    CHECK(r.removed == 2000);
    CHECK(doc.objectCount() == r.created.size());
    CHECK(sec < 60.0);  // documented bound (header of this test): generous, loaded CI machines
}

TEST_CASE("vector_ops: 2000 disjoint objects weld into 2000 objects", "[vector_ops]") {
    Document doc;
    std::vector<ObjectId> ids;
    for (int i = 0; i < 2000; ++i) {
        ids.push_back(add(doc, std::vector<Contour>{rect(i * 20.0, 0, i * 20.0 + 10, 10), rect(i * 20.0 + 3, 3, i * 20.0 + 7, 7, false)}));
    }
    select(doc, ids);
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "[vector_ops] 2000 disjoint objects with holes weld: " << sec << " s\n";
    REQUIRE(r.ok);
    CHECK(r.created.size() == 2000);
    CHECK(measure(doc, r.created).first == Approx(2000 * 84.0).margin(1e-3));
    CHECK(sec < 60.0);
}

TEST_CASE("vector_ops: welding 2000 heavily overlapping objects in one cluster", "[vector_ops]") {
    Document doc;
    std::vector<ObjectId> ids;
    std::uint32_t seed = 99;
    const auto rnd = [&seed]() {
        seed = seed * 1664525u + 1013904223u;
        return static_cast<double>((seed >> 8) & 0xFFFF) / 65536.0;
    };
    for (int i = 0; i < 2000; ++i) {
        const double x = rnd() * 300.0;
        const double y = rnd() * 300.0;
        ids.push_back(add(doc, (i % 3 == 0) ? circleAt({x, y}, 3.0 + rnd() * 4.0) : rect(x, y, x + 4 + rnd() * 6, y + 4 + rnd() * 6)));
    }
    select(doc, ids);
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = viewmodel::applyBoolean(doc, BooleanKind::Weld);
    const double sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::cout << "[vector_ops] 2000 overlapping objects weld: " << sec << " s\n";
    REQUIRE(r.ok);
    CHECK(r.removed == 2000);
    CHECK(sec < 60.0);
    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 2000);
}
