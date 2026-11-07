#include "carvetto/viewmodel/node_edit.hpp"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/node_edit_command.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/viewmodel/node_ops.hpp"
#include "carvetto/viewmodel/tools.hpp"

using namespace carvetto::document;
using namespace carvetto::geometry;
using namespace carvetto::viewmodel;
using Catch::Matchers::WithinAbs;

namespace {

struct Fix {
    Document doc;
    ViewTransform view;
    ToolController tool;

    Fix() : doc(Job::singleSided(1000.0, 1000.0, 18.0, Units::Millimetres)) {
        view.setViewport(1000.0, 1000.0);
        view.set(2.0, {100.0, 900.0});  // 2 px per mm
        tool.setDocument(&doc);
        tool.setView(view);
        tool.setTool(ToolKind::NodeEdit);
        tool.snapSettings().disableAll();
    }

    NodeEditor& ed() { return tool.nodeEditor(); }
    Vec2 px(Vec2 mm) const { return view.worldToScreen(mm); }

    ObjectId add(Contour c, CreationParams params = NoParams{}) {
        auto cmd = std::make_unique<AddObjectCommand>(
            Object::vector(std::move(c), doc.activeLayer(), doc.activeSheet(), std::move(params)));
        AddObjectCommand* raw = cmd.get();
        REQUIRE(doc.execute(std::move(cmd)));
        return raw->id();
    }
    ObjectId addRect(double x0, double y0, double x1, double y1) {
        RectangleParams rp;
        rp.centreMm = {(x0 + x1) / 2, (y0 + y1) / 2};
        rp.widthMm = x1 - x0;
        rp.heightMm = y1 - y0;
        return add(Contour::rectangle(Box2{{x0, y0}, {x1, y1}}), rp);
    }
    void select(ObjectId id) { doc.selection().assign({id}); }
    const Object& obj(ObjectId id) const { return *doc.findObject(id); }
    const std::vector<Contour>& contours(ObjectId id) const { return obj(id).asVector()->contours; }
    const Contour& contour(ObjectId id, std::size_t i = 0) const { return contours(id).at(i); }

    /// Press at `from`, move through `via`, release at the last point.
    NodeEditResult drag(Vec2 from, const std::vector<Vec2>& via, Modifiers m = {}) {
        tool.pointerMove(px(from), m);
        tool.pointerPress(px(from), m);
        for (const Vec2& p : via) {
            tool.pointerMove(px(p), m);
        }
        const ToolResult r = tool.pointerRelease(px(via.back()), m);
        NodeEditResult out;
        out.changed = r.committed;
        return out;
    }
    void click(Vec2 mm, Modifiers m = {}) {
        tool.pointerMove(px(mm), m);
        tool.pointerPress(px(mm), m);
        tool.pointerRelease(px(mm), m);
    }
    NodeRef node(ObjectId id, std::size_t n, std::size_t contourIndex = 0) const { return {id, contourIndex, n}; }
};

bool closeTo(Vec2 a, Vec2 b, double tol = 1e-9) {
    return distance(a, b) <= tol;
}

double maxDeviation(const Contour& a, const Contour& b, int samples = 100) {
    double worst = 0.0;
    for (const Span& s : a.spans) {
        for (int i = 0; i <= samples; ++i) {
            worst = std::max(worst, distanceTo(b, pointAt(s, static_cast<double>(i) / samples)));
        }
    }
    return worst;
}

Contour waveContour() {
    Contour c;
    c.append(BezierSpan{{20, 20}, {30, 50}, {50, -10}, {60, 20}});
    return c;
}

}  // namespace

TEST_CASE("only the selected editable objects have handles", "[viewmodel][node][edit]") {
    Fix f;
    const ObjectId a = f.addRect(20, 20, 60, 40);
    const ObjectId b = f.addRect(100, 20, 140, 40);
    CHECK(f.ed().handles().empty());  // nothing selected
    f.select(a);
    const auto handles = f.ed().handles();
    std::size_t nodes = 0;
    std::size_t mids = 0;
    for (const NodeHandle& h : handles) {
        CHECK((h.type == HandleType::Node ? h.node.object : h.span.object) == a);
        nodes += h.type == HandleType::Node ? 1 : 0;
        mids += h.type == HandleType::Midpoint ? 1 : 0;
    }
    CHECK(nodes == 4);
    CHECK(mids == 4);
    CHECK(f.ed().editedObjects() == std::vector<ObjectId>{a});
    f.doc.selection().assign({a, b});
    CHECK(f.ed().editedObjects() == std::vector<ObjectId>{a, b});
}

TEST_CASE("hit testing: nodes beat control points beat midpoints beat spans", "[viewmodel][node][edit]") {
    Fix f;
    Contour tight;
    tight.append(BezierSpan{{20, 20}, {21, 20}, {50, 60}, {60, 20}});
    const ObjectId t = f.add(tight);
    f.select(t);
    // Node 0 (20,20) with its control point 1 mm away: a pointer nearer to the control point (but
    // inside the node's radius) still gets the node.
    auto hit = f.ed().hitTest(f.px({20.9, 20.0}));
    REQUIRE(hit.has_value());
    CHECK(hit->type == HandleType::Node);
    CHECK(hit->node.node == 0);
    const ObjectId w = f.add(waveContour());
    f.select(w);
    // Control point 2 (50,-10) alone.
    hit = f.ed().hitTest(f.px({50.5, -10.0}));
    REQUIRE(hit.has_value());
    CHECK(hit->type == HandleType::Control);
    CHECK(hit->control == 1);
    // The midpoint handle sits on the curve and beats the span there.
    const Vec2 mid = std::get<BezierSpan>(f.contour(w).spans[0]).pointAt(0.5);
    hit = f.ed().hitTest(f.px(mid + Vec2{0.5, 0.0}));
    REQUIRE(hit.has_value());
    CHECK(hit->type == HandleType::Midpoint);
    // Elsewhere on the curve: the span, with its parameter.
    const Vec2 q = std::get<BezierSpan>(f.contour(w).spans[0]).pointAt(0.2);
    hit = f.ed().hitTest(f.px(q));
    REQUIRE(hit.has_value());
    CHECK(hit->type == HandleType::Span);
    CHECK_THAT(hit->t, WithinAbs(0.2, 0.02));
    CHECK(closeTo(hit->positionMm, q, 0.05));
    // Out of tolerance: nothing.
    CHECK_FALSE(f.ed().hitTest(f.px({300, 300})).has_value());
    // Tolerance is in pixels: 5.9 px hits, 6.5 px does not.
    CHECK(f.ed().hitTest(f.px({20.0 - 5.9 / 2.0, 20.0})).has_value());
    CHECK_FALSE(f.ed().hitTest(f.px({20.0 - 6.5 / 2.0, 20.0})).has_value());
}

TEST_CASE("dragging a node is one undo step, equals the numeric edit, and undo is exact", "[viewmodel][node][edit]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    const auto objectsBefore = f.doc.objects();
    const std::size_t steps = f.doc.undoStack().count();

    const NodeEditResult res = f.drag({60, 40}, {{61, 41}, {65, 45}, {70, 52}, {72.5, 55.25}});
    CHECK(res.changed);
    CHECK(f.doc.undoStack().count() == steps + 1);
    Contour expected = Contour::rectangle(Box2{{20, 20}, {60, 40}});
    REQUIRE(nodeops::moveNodes(expected, {{2, {72.5, 55.25}}}));
    CHECK(f.contour(r) == expected);
    CHECK(std::holds_alternative<NoParams>(f.obj(r).asVector()->params));
    CHECK(f.doc.selection().ids() == std::vector<ObjectId>{r});

    REQUIRE(f.doc.undoStack().undo(f.doc));
    CHECK(f.doc.objects() == objectsBefore);  // byte-identical, rectangle params included
    CHECK(std::holds_alternative<RectangleParams>(f.obj(r).asVector()->params));
    REQUIRE(f.doc.undoStack().redo(f.doc));
    CHECK(f.contour(r) == expected);
}

TEST_CASE("a click on a node selects it without editing", "[viewmodel][node][edit]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    const auto before = f.doc.objects();
    const std::size_t steps = f.doc.undoStack().count();
    f.click({60, 40});
    CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(r, 2)});
    CHECK(f.doc.objects() == before);
    CHECK(f.doc.undoStack().count() == steps);
    // A sub-threshold jitter is a click too.
    f.drag({20, 20}, {{20.5, 20.5}});
    CHECK(f.doc.objects() == before);
    CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(r, 0)});
}

TEST_CASE("Escape reverts a drag exactly and leaves nothing to redo", "[viewmodel][node][edit]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    const auto before = f.doc.objects();
    const std::size_t steps = f.doc.undoStack().count();

    f.tool.pointerMove(f.px({60, 40}));
    f.tool.pointerPress(f.px({60, 40}));
    f.tool.pointerMove(f.px({70, 50}));
    f.tool.pointerMove(f.px({80, 60}));
    CHECK(f.doc.undoStack().count() == steps + 1);
    CHECK(f.tool.wantsKeys());
    const ToolResult esc = f.tool.key(ToolKey::Escape);
    CHECK(esc.committed);  // the document changed back
    CHECK(f.doc.objects() == before);
    CHECK(f.doc.undoStack().count() == steps);
    CHECK_FALSE(f.doc.undoStack().canRedo());
    CHECK_FALSE(f.tool.wantsKeys());
    // The late release does nothing.
    f.tool.pointerRelease(f.px({80, 60}));
    CHECK(f.doc.objects() == before);
    CHECK(f.doc.undoStack().count() == steps);

    // Escape before the first move (no step pushed) is harmless.
    f.tool.pointerMove(f.px({60, 40}));
    f.tool.pointerPress(f.px({60, 40}));
    f.tool.key(ToolKey::Escape);
    f.tool.pointerRelease(f.px({60, 40}));
    CHECK(f.doc.objects() == before);
}

TEST_CASE("Shift-click and a box select several nodes; dragging moves them all", "[viewmodel][node][edit]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    Modifiers shift;
    shift.shift = true;
    f.click({20, 20});
    f.click({60, 20}, shift);
    CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(r, 0), f.node(r, 1)});
    f.click({60, 20}, shift);  // toggles off
    CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(r, 0)});

    // Box: from empty canvas, enclosing the two top nodes.
    f.tool.pointerMove(f.px({15, 45}));
    f.tool.pointerPress(f.px({15, 45}));
    f.tool.pointerMove(f.px({40, 38}));
    CHECK(f.tool.preview().marqueePx.has_value());
    f.tool.pointerMove(f.px({65, 35}));
    f.tool.pointerRelease(f.px({65, 35}));
    CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(r, 2), f.node(r, 3)});
    // A Shift box adds.
    f.tool.pointerMove(f.px({15, 15}), shift);
    f.tool.pointerPress(f.px({15, 15}), shift);
    f.tool.pointerMove(f.px({25, 25}), shift);
    f.tool.pointerRelease(f.px({25, 25}), shift);
    CHECK(f.ed().selectedNodes().size() == 3);

    // Drag the two top nodes together.
    f.click({60, 40});
    f.click({20, 40}, shift);
    const std::size_t steps = f.doc.undoStack().count();
    f.drag({60, 40}, {{61, 44}, {62, 48}});
    CHECK(f.doc.undoStack().count() == steps + 1);
    CHECK(closeTo(nodeops::nodePoint(f.contour(r), 2), {62, 48}, 1e-12));
    CHECK(closeTo(nodeops::nodePoint(f.contour(r), 3), {22, 48}, 1e-12));
    CHECK(closeTo(nodeops::nodePoint(f.contour(r), 0), {20, 20}, 1e-12));

    // A plain click on empty canvas clears the node selection.
    f.click({400, 400});
    CHECK(f.ed().selectedNodes().empty());
}

TEST_CASE("clicking another object selects it for editing", "[viewmodel][node][edit]") {
    Fix f;
    const ObjectId a = f.addRect(20, 20, 60, 40);
    const ObjectId b = f.addRect(100, 20, 140, 40);
    f.select(a);
    f.click({20, 20});
    REQUIRE(f.ed().selectedNodes().size() == 1);
    const auto before = f.doc.objects();
    f.click({120, 20});  // on b's bottom edge
    CHECK(f.doc.selection().ids() == std::vector<ObjectId>{b});
    CHECK(f.ed().selectedNodes().empty());
    CHECK(f.doc.objects() == before);
}

TEST_CASE("dragging a control point, with and without keeping tangency", "[viewmodel][node][edit]") {
    Fix f;
    Contour c;
    c.append(BezierSpan{{20, 20}, {30, 40}, {50, 40}, {60, 20}});
    c.append(BezierSpan{{60, 20}, {70, 0}, {90, 0}, {100, 20}});
    const ObjectId w = f.add(c);
    f.select(w);
    const std::size_t steps = f.doc.undoStack().count();
    f.drag({50, 40}, {{52, 42}, {55, 46}});
    CHECK(f.doc.undoStack().count() == steps + 1);
    CHECK(std::get<BezierSpan>(f.contour(w).spans[0]).p2 == Vec2{55, 46});
    CHECK(std::get<BezierSpan>(f.contour(w).spans[1]).p1 == Vec2{70, 0});
    REQUIRE(f.doc.undoStack().undo(f.doc));

    f.ed().options().keepBezierTangency = true;
    f.drag({50, 40}, {{52, 42}, {55, 46}});
    const Vec2 node{60, 20};
    const Vec2 in = std::get<BezierSpan>(f.contour(w).spans[0]).p2 - node;
    const Vec2 out = std::get<BezierSpan>(f.contour(w).spans[1]).p1 - node;
    CHECK_THAT(in.cross(out), WithinAbs(0.0, 1e-9));
    CHECK(in.dot(out) < 0.0);
    CHECK_THAT(out.length(), WithinAbs(Vec2(10, -20).length(), 1e-9));
    CHECK(f.doc.undoStack().count() == steps + 1);
}

TEST_CASE("dragging a span reshapes it; Ctrl moves the whole span", "[viewmodel][node][edit]") {
    Fix f;
    SECTION("bezier reshape") {
        const ObjectId w = f.add(waveContour());
        f.select(w);
        const BezierSpan orig = std::get<BezierSpan>(f.contour(w).spans[0]);
        const Vec2 grab = orig.pointAt(0.25);
        const std::size_t steps = f.doc.undoStack().count();
        f.drag(grab, {grab + Vec2{1, 1}, grab + Vec2{2, 3}, grab + Vec2{3, 4}});
        CHECK(f.doc.undoStack().count() == steps + 1);
        const BezierSpan now = std::get<BezierSpan>(f.contour(w).spans[0]);
        CHECK(now.p0 == orig.p0);
        CHECK(now.p3 == orig.p3);
        CHECK(distance(now.pointAt(0.25), grab + Vec2{3, 4}) < 0.3);  // the grab hit is within the pick radius
        CHECK(f.ed().selectedSpans().size() == 1);
    }
    SECTION("arc reshape keeps the end points and passes through the moved point") {
        Contour c;
        c.append(*nodeops::arcFromBulge({20, 20}, {60, 20}, 0.5));
        const ObjectId a = f.add(c);
        f.select(a);
        const Vec2 grab = std::get<ArcSpan>(c.spans[0]).pointAt(0.3);
        f.drag(grab, {grab + Vec2{0, -1}, grab + Vec2{0, -4}});
        const auto& arc = std::get<ArcSpan>(f.contour(a).spans[0]);
        CHECK(closeTo(arc.startPoint(), {20, 20}, 1e-8));
        CHECK(closeTo(arc.endPoint(), {60, 20}, 1e-8));
        CHECK(distanceTo(f.contour(a), grab + Vec2{0, -4}) < 0.35);
    }
    SECTION("Ctrl+drag moves a Bezier span with its end nodes; neighbours follow") {
        Contour c;
        c.append(LineSpan{{0, 20}, {20, 20}});
        c.append(BezierSpan{{20, 20}, {30, 50}, {50, -10}, {60, 20}});
        c.append(LineSpan{{60, 20}, {80, 20}});
        const ObjectId w = f.add(c);
        f.select(w);
        const Vec2 grab = std::get<BezierSpan>(c.spans[1]).pointAt(0.4);
        Modifiers ctrl;
        ctrl.ctrl = true;
        const std::size_t steps = f.doc.undoStack().count();
        // The span is picked at the press point; its offset to the curve is below the pick radius.
        const Vec2 d{4, 6};
        f.drag(grab, {grab + d * 0.5, grab + d}, ctrl);
        CHECK(f.doc.undoStack().count() == steps + 1);
        const auto& moved = std::get<BezierSpan>(f.contour(w).spans[1]);
        const auto& orig = std::get<BezierSpan>(c.spans[1]);
        const Vec2 shift = moved.p0 - orig.p0;
        CHECK(closeTo(shift, d, 0.2));
        CHECK(closeTo(moved.p3 - orig.p3, shift, 1e-12));
        CHECK(closeTo(moved.p1 - orig.p1, shift, 1e-12));
        CHECK(closeTo(moved.p2 - orig.p2, shift, 1e-12));
        CHECK(endPoint(f.contour(w).spans[0]) == moved.p0);
        CHECK(startPoint(f.contour(w).spans[2]) == moved.p3);
        CHECK(startPoint(f.contour(w).spans[0]) == Vec2{0, 20});
    }
}

TEST_CASE("dragging a midpoint handle inserts a node and drags it", "[viewmodel][node][edit]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    const std::size_t steps = f.doc.undoStack().count();
    f.drag({40, 20}, {{41, 18}, {42, 14}, {44, 10}});
    CHECK(f.doc.undoStack().count() == steps + 1);
    const Contour& c = f.contour(r);
    REQUIRE(c.spans.size() == 5);
    CHECK(closeTo(nodeops::nodePoint(c, 1), {44, 10}, 1e-12));
    CHECK(closeTo(nodeops::nodePoint(c, 0), {20, 20}));
    CHECK(closeTo(nodeops::nodePoint(c, 2), {60, 20}));
    CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(r, 1)});
    REQUIRE(f.doc.undoStack().undo(f.doc));
    CHECK(f.contour(r) == Contour::rectangle(Box2{{20, 20}, {60, 40}}));
}

TEST_CASE("dragging a circle node: the circle is normalised and the params dropped", "[viewmodel][node][edit]") {
    Fix f;
    CircleParams cp;
    cp.centreMm = {50, 50};
    cp.radiusMm = 20;
    const ObjectId c = f.add(Contour::circle({50, 50}, 20), cp);
    f.select(c);
    CHECK(f.ed().selectedNodes().empty());
    std::size_t nodes = 0;
    for (const NodeHandle& h : f.ed().handles()) {
        nodes += h.type == HandleType::Node ? 1 : 0;
    }
    CHECK(nodes == 2);
    const auto before = f.doc.objects();
    f.drag({70, 50}, {{72, 50}, {75, 50}});
    const Contour& now = f.contour(c);
    REQUIRE(now.spans.size() == 2);
    CHECK(closeTo(nodeops::nodePoint(now, 0), {75, 50}, 1e-9));
    CHECK(std::holds_alternative<NoParams>(f.obj(c).asVector()->params));
    REQUIRE(f.doc.undoStack().undo(f.doc));
    CHECK(f.doc.objects() == before);
}

TEST_CASE("mirror mode moves nodes symmetrically", "[viewmodel][node][edit]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    f.ed().selectAllNodes();
    f.ed().options().mirror = MirrorMode::Horizontal;  // left / right symmetry about x = 40
    f.drag({60, 40}, {{62, 42}, {65, 47}});
    const Contour& c = f.contour(r);
    const Vec2 n2 = nodeops::nodePoint(c, 2);   // grabbed (top right)
    const Vec2 n3 = nodeops::nodePoint(c, 3);   // top left, mirrored
    const Vec2 n1 = nodeops::nodePoint(c, 1);   // bottom right moves with the grabbed one
    const Vec2 n0 = nodeops::nodePoint(c, 0);   // bottom left mirrors n1
    CHECK(closeTo(n2, {65, 47}, 1e-12));
    CHECK_THAT(n2.x - 40.0, WithinAbs(40.0 - n3.x, 1e-12));
    CHECK_THAT(n2.y, WithinAbs(n3.y, 1e-12));
    CHECK_THAT(n1.x - 40.0, WithinAbs(40.0 - n0.x, 1e-12));
    CHECK_THAT(n1.y, WithinAbs(n0.y, 1e-12));
    // Vertical mode through the typed move: nodes above the centre go up, below go down.
    f.doc.undoStack().undo(f.doc);
    f.ed().selectAllNodes();
    f.ed().options().mirror = MirrorMode::Vertical;
    REQUIRE(f.ed().setSelectedNodeCoordinates(0.0, 3.0, true).ok);
    const Contour& v = f.contour(r);
    // The first selected node (0, below the axis y = 30) moves by +3, node 2 above it mirrors to -3.
    CHECK_THAT(nodeops::nodePoint(v, 0).y, WithinAbs(23.0, 1e-12));
    CHECK_THAT(nodeops::nodePoint(v, 2).y, WithinAbs(37.0, 1e-12));
    CHECK_THAT(nodeops::nodePoint(v, 1).y, WithinAbs(23.0, 1e-12));
    CHECK_THAT(nodeops::nodePoint(v, 3).y, WithinAbs(37.0, 1e-12));
}

TEST_CASE("snapping: drag steps snap with the dragged geometry excluded", "[viewmodel][node][edit][snap]") {
    Fix f;
    f.tool.snapSettings().setEnabled(SnapKind::Endpoint, true);
    const ObjectId a = f.addRect(20, 20, 60, 40);
    const ObjectId other = f.addRect(100, 60, 140, 80);
    f.select(a);
    // Close to the other rectangle's corner (140,80): lands exactly on it.
    f.drag({60, 40}, {{100, 60}, {139, 79}, {139.6, 79.7}});
    CHECK(closeTo(nodeops::nodePoint(f.contour(a), 2), {140, 80}, 1e-12));
    f.doc.undoStack().undo(f.doc);
    // Next to the dragged object's own corner (20,40): no snap to itself.
    f.drag({60, 40}, {{40, 40}, {22.0, 40.0}});
    CHECK(closeTo(nodeops::nodePoint(f.contour(a), 2), {22, 40}, 1e-12));
    CHECK(f.doc.undoStack().canUndo());
    (void)other;
}

TEST_CASE("snapping after the document changed uses fresh geometry", "[viewmodel][node][edit][snap]") {
    Fix f;
    f.tool.snapSettings().setEnabled(SnapKind::Endpoint, true);
    const ObjectId a = f.addRect(20, 20, 60, 40);
    const ObjectId other = f.addRect(100, 60, 140, 80);
    f.select(a);
    f.drag({60, 40}, {{100, 60}, {139.5, 79.5}});
    CHECK(closeTo(nodeops::nodePoint(f.contour(a), 2), {140, 80}, 1e-12));
    // Move the other rectangle with a command; a stale snap index would still offer (140,80).
    REQUIRE(f.doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{other},
                                                                     Transform2::translation({0, 100}))));
    f.drag({140, 80}, {{130, 70}, {139.0, 79.0}});
    CHECK(closeTo(nodeops::nodePoint(f.contour(a), 2), {139, 79}, 1e-12));
    f.drag({139, 79}, {{139, 150}, {140.2, 179.4}});
    CHECK(closeTo(nodeops::nodePoint(f.contour(a), 2), {140, 180}, 1e-12));
}

TEST_CASE("a drag survives unrelated edits and a replaced document without crashing", "[viewmodel][node][edit]") {
    SECTION("undo during the drag ends it; later steps edit nothing") {
        Fix f;
        const ObjectId r = f.addRect(20, 20, 60, 40);
        const ObjectId s = f.addRect(100, 20, 140, 40);
        f.select(r);
        const std::size_t steps = f.doc.undoStack().count();
        f.tool.pointerMove(f.px({60, 40}));
        f.tool.pointerPress(f.px({60, 40}));
        f.tool.pointerMove(f.px({70, 50}));
        REQUIRE(f.doc.undoStack().count() == steps + 1);
        REQUIRE(f.doc.undoStack().undo(f.doc));  // reverts the drag step itself
        f.tool.pointerMove(f.px({80, 60}));
        f.tool.pointerRelease(f.px({80, 60}));
        CHECK_FALSE(f.tool.active());
        CHECK(f.doc.undoStack().count() == steps + 1);  // nothing new was recorded
        CHECK(f.doc.undoStack().canRedo());              // and the redo entry was not discarded
        CHECK(f.contour(r) == Contour::rectangle(Box2{{20, 20}, {60, 40}}));
        CHECK(f.doc.hasObject(s));
    }
    SECTION("document replaced mid-drag") {
        auto doc1 = std::make_unique<Document>(Job::singleSided(1000.0, 1000.0, 18.0, Units::Millimetres));
        ToolController tool;
        ViewTransform view;
        view.setViewport(1000, 1000);
        view.set(2.0, {100.0, 900.0});
        tool.setView(view);
        tool.setDocument(doc1.get());
        tool.setTool(ToolKind::NodeEdit);
        tool.snapSettings().disableAll();
        auto cmd = std::make_unique<AddObjectCommand>(Object::vector(Contour::rectangle(Box2{{20, 20}, {60, 40}}),
                                                                      doc1->activeLayer(), doc1->activeSheet()));
        REQUIRE(doc1->execute(std::move(cmd)));
        doc1->selection().assign(doc1->objectIds());
        tool.pointerMove(view.worldToScreen({60, 40}));
        tool.pointerPress(view.worldToScreen({60, 40}));
        tool.pointerMove(view.worldToScreen({70, 50}));
        auto doc2 = std::make_unique<Document>(Job::singleSided(500.0, 500.0, 18.0, Units::Millimetres));
        tool.setDocument(doc2.get());
        doc1.reset();  // the old document is gone
        tool.pointerMove(view.worldToScreen({80, 60}));
        tool.pointerRelease(view.worldToScreen({80, 60}));
        CHECK(doc2->undoStack().count() == 0);
        CHECK_FALSE(tool.active());
    }
    SECTION("the object vanishes mid-drag") {
        Fix f;
        const ObjectId r = f.addRect(20, 20, 60, 40);
        f.select(r);
        f.tool.pointerMove(f.px({60, 40}));
        f.tool.pointerPress(f.px({60, 40}));
        f.tool.pointerMove(f.px({70, 50}));
        REQUIRE(f.doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{r})));
        f.tool.pointerMove(f.px({80, 60}));
        f.tool.pointerRelease(f.px({80, 60}));
        CHECK(f.doc.objectCount() == 0);
        CHECK(f.ed().selectedNodes().empty());
    }
}

// ----------------------------------------------------------------------------- operations

TEST_CASE("convert span to line / bezier / arc", "[viewmodel][node][edit][ops]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    const auto before = f.doc.objects();
    const Contour original = f.contour(r);

    SECTION("to arc through a point") {
        f.ed().selectSpan({r, 0, 0}, false);
        NodeOpArgs args;
        args.throughMm = Vec2{40, 12};
        const NodeEditResult res = f.ed().run(NodeOp::ConvertArc, args);
        REQUIRE(res.ok);
        const auto* arc = std::get_if<ArcSpan>(&f.contour(r).spans[0]);
        REQUIRE(arc != nullptr);
        CHECK(closeTo(arc->startPoint(), {20, 20}, 1e-9));
        CHECK(closeTo(arc->endPoint(), {60, 20}, 1e-9));
        CHECK(distanceTo(Contour{{*arc}, false}, {40, 12}) < 1e-9);
        CHECK(f.contour(r).spans[1] == original.spans[1]);
        CHECK(f.doc.undoStack().count() == 2);
        REQUIRE(f.doc.undoStack().undo(f.doc));
        CHECK(f.doc.objects() == before);
    }
    SECTION("to bezier then back to line") {
        f.ed().selectSpan({r, 0, 2}, false);
        REQUIRE(f.ed().run(NodeOp::ConvertBezier).ok);
        REQUIRE(std::holds_alternative<BezierSpan>(f.contour(r).spans[2]));
        CHECK(maxDeviation(original, f.contour(r)) < 1e-12);
        REQUIRE(f.ed().run(NodeOp::ConvertLine).ok);
        CHECK(f.contour(r) == original);
        // Already a line: refused, nothing recorded.
        const std::size_t steps = f.doc.undoStack().count();
        const NodeEditResult again = f.ed().run(NodeOp::ConvertLine);
        CHECK_FALSE(again.ok);
        CHECK_FALSE(again.message.empty());
        CHECK(f.doc.undoStack().count() == steps);
    }
    SECTION("two selected neighbouring nodes select the span between them") {
        f.ed().selectNode(f.node(r, 1), false);
        f.ed().selectNode(f.node(r, 2), true);
        REQUIRE(f.ed().run(NodeOp::ConvertBezier).ok);
        CHECK(std::holds_alternative<BezierSpan>(f.contour(r).spans[1]));
        CHECK(std::holds_alternative<LineSpan>(f.contour(r).spans[0]));
    }
    SECTION("a bad bulge or point refuses without change") {
        f.ed().selectSpan({r, 0, 0}, false);
        NodeOpArgs args;
        args.bulge = std::numeric_limits<double>::quiet_NaN();
        CHECK_FALSE(f.ed().run(NodeOp::ConvertArc, args).ok);
        args.bulge.reset();
        args.throughMm = Vec2{40, 20};  // collinear
        CHECK_FALSE(f.ed().run(NodeOp::ConvertArc, args).ok);
        CHECK(f.doc.objects() == before);
    }
}

TEST_CASE("insert point and insert midpoint", "[viewmodel][node][edit][ops]") {
    Fix f;
    const ObjectId w = f.add(waveContour());
    f.select(w);
    const Contour original = f.contour(w);
    f.ed().selectSpan({w, 0, 0}, false);
    NodeOpArgs args;
    args.positionMm = std::get<BezierSpan>(original.spans[0]).pointAt(0.3) + Vec2{0.2, 0.1};
    REQUIRE(f.ed().run(NodeOp::InsertPoint, args).ok);
    const Contour c = f.contour(w);
    REQUIRE(c.spans.size() == 2);
    const Vec2 inserted = nodeops::nodePoint(c, 1);
    CHECK(distanceTo(original, inserted) < 1e-9);
    CHECK(maxDeviation(original, c) < 1e-9);
    CHECK(maxDeviation(c, original) < 1e-9);
    CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(w, 1)});

    // Midpoint of the first half at parameter 0.5.
    f.ed().selectSpan({w, 0, 0}, false);
    REQUIRE(f.ed().run(NodeOp::InsertMidpoint).ok);
    CHECK(f.contour(w).spans.size() == 3);
    CHECK(maxDeviation(original, f.contour(w)) < 1e-9);
    CHECK(closeTo(nodeops::nodePoint(f.contour(w), 1), std::get<BezierSpan>(c.spans[0]).pointAt(0.5), 1e-9));

    // Several spans at once, one undo step.
    Fix g;
    const ObjectId r = g.addRect(20, 20, 60, 40);
    g.select(r);
    g.ed().selectSpan({r, 0, 0}, false);
    g.ed().selectSpan({r, 0, 2}, true);
    const std::size_t steps = g.doc.undoStack().count();
    REQUIRE(g.ed().run(NodeOp::InsertMidpoint).ok);
    CHECK(g.doc.undoStack().count() == steps + 1);
    CHECK(g.contour(r).spans.size() == 6);
    CHECK(closeTo(nodeops::nodePoint(g.contour(r), 1), {40, 20}));
    CHECK(closeTo(nodeops::nodePoint(g.contour(r), 4), {40, 40}));
    CHECK(g.ed().selectedNodes() == std::vector<NodeRef>{g.node(r, 1), g.node(r, 4)});
    CHECK(maxDeviation(Contour::rectangle(Box2{{20, 20}, {60, 40}}), g.contour(r)) < 1e-12);
}

TEST_CASE("delete point, delete span and cut vector", "[viewmodel][node][edit][ops]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    const auto before = f.doc.objects();

    SECTION("delete point re-joins the spans") {
        f.ed().selectNode(f.node(r, 2), false);
        REQUIRE(f.ed().run(NodeOp::DeletePoint).ok);
        const Contour& c = f.contour(r);
        REQUIRE(c.spans.size() == 3);
        CHECK(c.spans[1] == Span{LineSpan{{60, 20}, {20, 40}}});
        CHECK(f.ed().selectedNodes().empty());
        REQUIRE(f.doc.undoStack().undo(f.doc));
        CHECK(f.doc.objects() == before);
    }
    SECTION("delete span opens a closed vector") {
        f.ed().selectSpan({r, 0, 1}, false);
        REQUIRE(f.ed().run(NodeOp::DeleteSpan).ok);
        REQUIRE(f.contours(r).size() == 1);
        const Contour& c = f.contour(r);
        CHECK_FALSE(c.closed);
        REQUIRE(c.spans.size() == 3);
        CHECK(c.spans[0] == Span{LineSpan{{60, 40}, {20, 40}}});
    }
    SECTION("cut vector at two nodes gives two open contours covering the original spans") {
        f.ed().selectNode(f.node(r, 1), false);
        f.ed().selectNode(f.node(r, 3), true);
        REQUIRE(f.ed().run(NodeOp::CutVector).ok);
        REQUIRE(f.contours(r).size() == 2);
        const Contour orig = Contour::rectangle(Box2{{20, 20}, {60, 40}});
        CHECK(f.contours(r)[0].spans == std::vector<Span>{orig.spans[1], orig.spans[2]});
        CHECK(f.contours(r)[1].spans == std::vector<Span>{orig.spans[3], orig.spans[0]});
        CHECK_FALSE(f.contours(r)[0].closed);
        CHECK(f.doc.undoStack().count() == 2);
    }
    SECTION("cut at a span splits it at the stated position first") {
        f.ed().selectSpan({r, 0, 0}, false);
        NodeOpArgs args;
        args.positionMm = Vec2{30, 20.4};
        REQUIRE(f.ed().run(NodeOp::CutVector, args).ok);
        REQUIRE(f.contours(r).size() == 1);
        const Contour& c = f.contour(r);
        CHECK_FALSE(c.closed);
        REQUIRE(c.spans.size() == 5);
        CHECK(closeTo(startPoint(c.spans[0]), {30, 20}, 1e-12));
        CHECK(closeTo(endPoint(c.spans[4]), {30, 20}, 1e-12));
    }
    SECTION("deleting every span, or too many nodes, is refused") {
        f.ed().selectNode(f.node(r, 0), false);
        f.ed().selectNode(f.node(r, 1), true);
        f.ed().selectNode(f.node(r, 2), true);
        f.ed().selectNode(f.node(r, 3), true);
        CHECK_FALSE(f.ed().run(NodeOp::DeletePoint).ok);
        CHECK(f.doc.objects() == before);
    }
}

TEST_CASE("smooth, reverse, start point", "[viewmodel][node][edit][ops]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    const Contour original = f.contour(r);

    SECTION("smooth gives continuous tangents") {
        f.ed().selectNode(f.node(r, 1), false);
        REQUIRE(f.ed().run(NodeOp::Smooth).ok);
        const Contour& c = f.contour(r);
        const Vec2 in = tangentAt(c.spans[0], 1.0);
        const Vec2 out = tangentAt(c.spans[1], 0.0);
        CHECK(std::atan2(std::fabs(in.cross(out)), in.dot(out)) < 1e-9);
        CHECK(closeTo(endPoint(c.spans[0]), {60, 20}, 1e-12));
        // Smoothing again: nothing to do, refused with a message.
        const NodeEditResult again = f.ed().run(NodeOp::Smooth);
        CHECK_FALSE(again.ok);
        CHECK_FALSE(again.message.empty());
    }
    SECTION("reverse gives exactly the reversed spans") {
        f.ed().selectSpan({r, 0, 2}, false);
        REQUIRE(f.ed().run(NodeOp::Reverse).ok);
        CHECK(f.contour(r) == original.reversed());
        REQUIRE(f.doc.undoStack().undo(f.doc));
        CHECK(f.contour(r) == original);
    }
    SECTION("start point rotation preserves the geometry") {
        f.ed().selectNode(f.node(r, 2), false);
        REQUIRE(f.ed().run(NodeOp::StartPoint).ok);
        const Contour& c = f.contour(r);
        CHECK(closeTo(startPoint(c.spans[0]), {60, 40}));
        CHECK(c.closed);
        CHECK(maxDeviation(original, c) < 1e-12);
        CHECK(c.spans.size() == original.spans.size());
        CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(r, 0)});
        // Node 0 is already the start: refused.
        CHECK_FALSE(f.ed().run(NodeOp::StartPoint).ok);
    }
}

TEST_CASE("close, join and extend", "[viewmodel][node][edit][ops]") {
    Fix f;
    SECTION("close adds a line or snaps a small gap") {
        Contour c;
        c.append(LineSpan{{20, 20}, {60, 20}});
        c.append(LineSpan{{60, 20}, {60, 40}});
        c.append(LineSpan{{60, 40}, {20.0005, 20.0}});
        const ObjectId o = f.add(c);
        f.select(o);
        f.ed().selectNode(f.node(o, 1), false);
        REQUIRE(f.ed().run(NodeOp::Close).ok);
        CHECK(f.contour(o).closed);
        CHECK(f.contour(o).spans.size() == 3);  // snapped shut
        CHECK_FALSE(f.ed().run(NodeOp::Close).ok);

        Contour d;
        d.append(LineSpan{{100, 20}, {160, 20}});
        d.append(LineSpan{{160, 20}, {160, 40}});
        const ObjectId o2 = f.add(d);
        f.select(o2);
        f.ed().selectSpan({o2, 0, 0}, false);
        REQUIRE(f.ed().run(NodeOp::Close).ok);
        CHECK(f.contour(o2).closed);
        CHECK(f.contour(o2).spans.size() == 3);
        CHECK(f.contour(o2).spans[2] == Span{LineSpan{{160, 40}, {100, 20}}});
    }
    SECTION("join within one object and across objects (one undo step)") {
        Contour a;
        a.append(LineSpan{{20, 20}, {60, 20}});
        Contour b;
        b.append(LineSpan{{60, 20}, {60, 40}});
        Contour c;
        c.append(LineSpan{{60, 40}, {20, 40}});
        const ObjectId o1 = f.add(a);
        const ObjectId o2 = f.add(b);
        const ObjectId o3 = f.add(c);
        f.doc.selection().assign({o1, o2, o3});
        const auto before = f.doc.objects();
        const std::size_t steps = f.doc.undoStack().count();
        REQUIRE(f.ed().run(NodeOp::Join).ok);
        CHECK(f.doc.undoStack().count() == steps + 1);
        CHECK(f.doc.objectCount() == 1);
        REQUIRE(f.doc.hasObject(o1));
        REQUIRE(f.contours(o1).size() == 1);
        CHECK(f.contour(o1).spans.size() == 3);
        CHECK(f.contour(o1).isContinuous(Tolerance::defaults()));
        CHECK(f.doc.selection().ids() == std::vector<ObjectId>{o1});
        REQUIRE(f.doc.undoStack().undo(f.doc));
        CHECK(f.doc.objects() == before);

        // Nothing within the tolerance: refused.
        Contour distant;
        distant.append(LineSpan{{500, 500}, {520, 500}});
        const ObjectId o4 = f.add(distant);
        f.doc.selection().assign({o1, o4});
        const std::size_t count = f.doc.undoStack().count();
        CHECK_FALSE(f.ed().run(NodeOp::Join).ok);
        CHECK(f.doc.undoStack().count() == count);
    }
    SECTION("join closes a loop") {
        Contour a;
        a.append(LineSpan{{20, 20}, {60, 20}});
        a.append(LineSpan{{60, 20}, {60, 40}});
        Contour b;
        b.append(LineSpan{{60, 40}, {20, 20}});
        const ObjectId o = f.add(a);
        const ObjectId p = f.add(b);
        f.doc.selection().assign({o, p});
        REQUIRE(f.ed().run(NodeOp::Join).ok);
        REQUIRE(f.doc.hasObject(o));
        CHECK(f.contour(o).closed);
        CHECK(f.doc.objectCount() == 1);
    }
    SECTION("extend an open end to the next object") {
        Contour a;
        a.append(LineSpan{{20, 20}, {40, 20}});
        const ObjectId o = f.add(a);
        Contour wall;
        wall.append(LineSpan{{100, 0}, {100, 50}});
        f.add(wall);
        f.select(o);
        f.ed().selectNode(f.node(o, 1), false);
        REQUIRE(f.ed().run(NodeOp::Extend).ok);
        CHECK(closeTo(endPoint(f.contour(o).spans.back()), {100, 20}, 1e-9));
        // Nothing ahead of the start end: refused.
        f.ed().selectNode(f.node(o, 0), false);
        CHECK_FALSE(f.ed().run(NodeOp::Extend).ok);
    }
}

TEST_CASE("typed values: relative and absolute moves, Properties, match coordinates", "[viewmodel][node][edit][ops]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    f.ed().selectNode(f.node(r, 2), false);

    REQUIRE(f.ed().setSelectedNodeCoordinates(12.5, std::nullopt, false).ok);  // exact X
    CHECK(nodeops::nodePoint(f.contour(r), 2).x == 12.5);
    CHECK(nodeops::nodePoint(f.contour(r), 2).y == 40.0);
    REQUIRE(f.ed().setSelectedNodeCoordinates(1.5, -2.25, true).ok);
    CHECK(nodeops::nodePoint(f.contour(r), 2) == Vec2{14.0, 37.75});
    REQUIRE(f.ed().moveSelectedNodes({1, 1}).ok);
    CHECK(nodeops::nodePoint(f.contour(r), 2) == Vec2{15.0, 38.75});
    CHECK(f.doc.undoStack().count() == 4);  // add + three separate edits, never merged

    // Match X to the first selected node.
    f.ed().selectNode(f.node(r, 2), false);
    f.ed().selectNode(f.node(r, 1), true);
    REQUIRE(f.ed().run(NodeOp::MatchX).ok);
    CHECK(nodeops::nodePoint(f.contour(r), 1).x == nodeops::nodePoint(f.contour(r), 2).x);
    f.ed().selectNode(f.node(r, 3), false);
    f.ed().selectNode(f.node(r, 1), true);
    REQUIRE(f.ed().run(NodeOp::MatchY).ok);
    CHECK(nodeops::nodePoint(f.contour(r), 1).y == nodeops::nodePoint(f.contour(r), 3).y);

    // Invalid input changes nothing.
    const auto before = f.doc.objects();
    const std::size_t steps = f.doc.undoStack().count();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    CHECK_FALSE(f.ed().setSelectedNodeCoordinates(nan, std::nullopt, false).ok);
    CHECK_FALSE(f.ed().setSelectedNodeCoordinates(inf, 0.0, true).ok);
    CHECK_FALSE(f.ed().setSelectedNodeCoordinates(2e9, std::nullopt, false).ok);
    CHECK_FALSE(f.ed().setSelectedNodeCoordinates(std::nullopt, std::nullopt, true).ok);
    CHECK_FALSE(f.ed().moveSelectedNodes({nan, 0}).ok);
    CHECK(f.doc.objects() == before);
    CHECK(f.doc.undoStack().count() == steps);
}

TEST_CASE("keys map to operations", "[viewmodel][node][edit][ops]") {
    CHECK(opForKey('i') == NodeOp::InsertPoint);
    CHECK(opForKey('D') == NodeOp::DeletePoint);
    CHECK(opForKey('s') == NodeOp::Smooth);
    CHECK(opForKey('c') == NodeOp::CutVector);
    CHECK(opForKey('b') == NodeOp::ConvertBezier);
    CHECK(opForKey('a') == NodeOp::ConvertArc);
    CHECK(opForKey('l') == NodeOp::ConvertLine);
    CHECK(opForKey('x') == NodeOp::MatchX);
    CHECK(opForKey('y') == NodeOp::MatchY);
    CHECK(opForKey('j') == NodeOp::Join);
    CHECK_FALSE(opForKey('q').has_value());

    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    f.ed().selectSpan({r, 0, 0}, false);
    NodeEditResult res = f.ed().handleKey('b');
    CHECK(res.handled);
    CHECK(res.ok);
    CHECK(std::holds_alternative<BezierSpan>(f.contour(r).spans[0]));
    res = f.ed().handleKey('h');
    CHECK(res.handled);
    CHECK(f.ed().options().mirror == MirrorMode::Horizontal);
    f.ed().handleKey('h');
    CHECK(f.ed().options().mirror == MirrorMode::Off);
    f.ed().handleKey('v');
    CHECK(f.ed().options().mirror == MirrorMode::Vertical);
    f.ed().handleKey('k');
    CHECK(f.ed().options().keepBezierTangency);
    CHECK(f.ed().handleKey('p').openProperties);
    CHECK_FALSE(f.ed().handleKey('q').handled);
    // A key whose operation cannot run reports why.
    f.ed().clearSelection();
    res = f.ed().handleKey('d');
    CHECK(res.handled);
    CHECK_FALSE(res.ok);
    CHECK_FALSE(res.message.empty());
}

TEST_CASE("only visible, unlocked, active-sheet vectors are edited; groups edit their leaves", "[viewmodel][node][edit][safety]") {
    Fix f;
    const ObjectId a = f.addRect(20, 20, 60, 40);
    const ObjectId b = f.addRect(100, 20, 140, 40);
    f.doc.selection().assign({a, b});
    f.ed().selectNode(f.node(a, 0), false);
    f.ed().selectNode(f.node(b, 0), true);
    REQUIRE(f.ed().selectedNodes().size() == 2);

    SECTION("locked layer") {
        REQUIRE(f.doc.execute(std::make_unique<AddLayerCommand>(Layer{.name = "L2"}, false)));
        // Move b to a second layer and lock it.
        const LayerId l2 = f.doc.layers().back().id;
        REQUIRE(f.doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{b}, l2)));
        Layer lk = *f.doc.findLayer(l2);
        lk.locked = true;
        REQUIRE(f.doc.execute(std::make_unique<SetLayerPropertiesCommand>(lk)));
        CHECK(f.ed().editedObjects() == std::vector<ObjectId>{a});
        CHECK(f.ed().selectedNodes() == std::vector<NodeRef>{f.node(a, 0)});  // the stale one is dropped
        const auto before = f.doc.objects();
        REQUIRE(f.ed().setSelectedNodeCoordinates(5.0, std::nullopt, true).ok);
        CHECK(f.obj(b) == before.at(b));
        CHECK(closeTo(nodeops::nodePoint(f.contour(a), 0), {25, 20}));
    }
    SECTION("hidden layer") {
        REQUIRE(f.doc.execute(std::make_unique<AddLayerCommand>(Layer{.name = "L2"}, false)));
        const LayerId l2 = f.doc.layers().back().id;
        REQUIRE(f.doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{a}, l2)));
        Layer hidden = *f.doc.findLayer(l2);
        hidden.visible = false;
        REQUIRE(f.doc.execute(std::make_unique<SetLayerPropertiesCommand>(hidden)));
        CHECK(f.ed().editedObjects() == std::vector<ObjectId>{b});
        CHECK(f.ed().handles().size() == 8);
        const auto before = f.doc.objects();
        f.ed().selectAllNodes();
        REQUIRE(f.ed().run(NodeOp::Reverse).ok);
        CHECK(f.obj(a) == before.at(a));
    }
    SECTION("other sheet") {
        Sheet second;
        second.name = "Second";
        REQUIRE(f.doc.execute(std::make_unique<AddSheetCommand>(second, false)));
        const SheetId s2 = f.doc.sheets().back().id;
        REQUIRE(f.doc.execute(std::make_unique<MoveObjectsToSheetCommand>(std::vector<ObjectId>{b}, s2)));
        CHECK(f.ed().editedObjects() == std::vector<ObjectId>{a});
    }
    SECTION("groups edit their leaves") {
        auto cmd = std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a, b});
        auto* raw = cmd.get();
        REQUIRE(f.doc.execute(std::move(cmd)));
        f.doc.selection().assign({raw->groupId()});
        CHECK(f.ed().editedObjects() == std::vector<ObjectId>{a, b});
        f.ed().selectNode(f.node(a, 1), false);
        f.ed().selectNode(f.node(b, 1), true);
        REQUIRE(f.ed().moveSelectedNodes({0, 2}).ok);
        CHECK(closeTo(nodeops::nodePoint(f.contour(a), 1), {60, 22}));
        CHECK(closeTo(nodeops::nodePoint(f.contour(b), 1), {140, 22}));
        CHECK(f.doc.undoStack().count() >= 1);
    }
}

TEST_CASE("empty and stale selections neither crash nor change anything", "[viewmodel][node][edit][safety]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    const auto before = f.doc.objects();
    const std::size_t steps = f.doc.undoStack().count();
    // No object selected.
    for (const NodeOp op : {NodeOp::ConvertArc, NodeOp::InsertPoint, NodeOp::DeletePoint, NodeOp::Smooth, NodeOp::Join,
                            NodeOp::Close, NodeOp::CutVector, NodeOp::StartPoint, NodeOp::Extend}) {
        std::string why;
        CHECK_FALSE(f.ed().canRun(op, &why));
        CHECK_FALSE(why.empty());
        CHECK_FALSE(f.ed().run(op).ok);
    }
    CHECK_FALSE(f.ed().moveSelectedNodes({1, 1}).ok);
    // Object selected, no nodes.
    f.select(r);
    CHECK_FALSE(f.ed().run(NodeOp::DeletePoint).ok);
    // Stale node and span references (contour / node out of range, removed object).
    f.ed().selectNode({r, 7, 0}, false);
    f.ed().selectNode({r, 0, 99}, true);
    f.ed().selectNode({ObjectId{12345}, 0, 0}, true);
    f.ed().selectSpan({r, 0, 99}, false);
    CHECK(f.ed().selectedNodes().empty());
    CHECK(f.ed().selectedSpans().empty());
    CHECK_FALSE(f.ed().run(NodeOp::ConvertLine).ok);
    // The object is removed after its nodes were selected.
    f.ed().selectNode(f.node(r, 1), false);
    REQUIRE(f.doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{r})));
    CHECK(f.ed().selectedNodes().empty());
    CHECK_FALSE(f.ed().run(NodeOp::DeletePoint).ok);
    f.doc.undoStack().undo(f.doc);
    CHECK(f.doc.objects() == before);
    CHECK(f.doc.undoStack().count() == steps + 1);
}

TEST_CASE("degenerate and out-of-range edits are refused as a whole", "[viewmodel][node][edit][safety]") {
    Fix f;
    const ObjectId r = f.addRect(20, 20, 60, 40);
    f.select(r);
    const auto before = f.doc.objects();
    f.ed().selectNode(f.node(r, 2), false);
    f.ed().selectNode(f.node(r, 3), true);
    // Both top nodes onto node 0: spans collapse.
    CHECK_FALSE(f.ed().setSelectedNodeCoordinates(20.0, 20.0, false).ok);
    CHECK_FALSE(f.ed().moveSelectedNodes({2e9, 0}).ok);
    CHECK(f.doc.objects() == before);
    // A drag out of range keeps the last valid step and never produces an invalid contour.
    f.drag({60, 40}, {{70, 50}, {1e12, 1e12}, {80, 60}});
    for (const Span& s : f.contour(r).spans) {
        const Vec2 a = startPoint(s);
        CHECK(std::isfinite(a.x));
        CHECK(std::fabs(a.x) <= 1e9);
    }
    CHECK(closeTo(nodeops::nodePoint(f.contour(r), 2), {80, 60}, 1e-12));
}
