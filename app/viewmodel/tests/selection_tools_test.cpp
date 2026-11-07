#include "carvetto/viewmodel/selection_tools.hpp"

#include <algorithm>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <utility>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"

using namespace carvetto;
using document::Document;
using document::LayerId;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;
using viewmodel::FillHit;
using viewmodel::HitOptions;
using viewmodel::MarqueeMode;
using viewmodel::PickIndex;
using viewmodel::SelectMode;
using viewmodel::ViewTransform;

namespace {

ViewTransform makeView(double scale = 2.0) {
    ViewTransform v;
    v.setViewport(1000.0, 800.0);
    v.set(scale, {100.0, 700.0});
    return v;
}

ObjectId addVector(Document& doc, Contour c, LayerId layer = {}) {
    return doc.insertObject(
        Object::vector(std::move(c), layer ? layer : doc.activeLayer(), doc.activeSheet()));
}

ObjectId addVector(Document& doc, std::vector<Contour> cs, LayerId layer = {}) {
    return doc.insertObject(
        Object::vector(std::move(cs), layer ? layer : doc.activeLayer(), doc.activeSheet()));
}

ObjectId addGroup(Document& doc, const std::vector<ObjectId>& children, LayerId layer = {}) {
    Object g;
    g.layer = layer ? layer : doc.activeLayer();
    g.sheet = doc.activeSheet();
    g.data = document::GroupObject{children};
    const ObjectId gid = doc.insertObject(std::move(g));
    for (const ObjectId c : children) {
        doc.findObject(c)->parent = gid;
    }
    return gid;
}

Contour line(Vec2 a, Vec2 b) {
    Contour c;
    c.append(geometry::LineSpan{a, b});
    return c;
}

Contour bezier(Vec2 a, Vec2 b, Vec2 c, Vec2 d) {
    Contour k;
    k.append(geometry::BezierSpan{a, b, c, d});
    return k;
}

Contour arc(Vec2 centre, double r, double a0, double a1) {
    Contour k;
    k.append(geometry::ArcSpan{centre, r, a0, a1, geometry::ArcDirection::CCW});
    return k;
}

/// Screen point of a world point.
Vec2 px(const ViewTransform& v, Vec2 mm) {
    return v.worldToScreen(mm);
}

/// Reference distance: dense sampling of each span in its own parameter.
double oracleDistance(const std::vector<Contour>& contours, Vec2 p) {
    double best = std::numeric_limits<double>::infinity();
    for (const Contour& c : contours) {
        for (const geometry::Span& s : c.spans) {
            const int n = 4000;
            for (int i = 0; i <= n; ++i) {
                best = std::min(best, geometry::distance(geometry::pointAt(s, static_cast<double>(i) / n), p));
            }
        }
    }
    return best;
}

struct Lcg {
    std::uint64_t s;
    double next() {  // [0,1)
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return static_cast<double>(s >> 11) / 9007199254740992.0;
    }
    double range(double a, double b) { return a + (b - a) * next(); }
};

}  // namespace

TEST_CASE("hit test: strokes of open and closed vectors within tolerance", "[viewmodel][selection]") {
    Document doc;
    const ViewTransform view = makeView();
    const ObjectId open = addVector(doc, line({10, 10}, {60, 10}));
    const ObjectId circle = addVector(doc, Contour::circle({100, 50}, 20));
    const PickIndex index(doc);
    HitOptions opt;
    opt.tolerancePx = 4.0;  // 2 mm at scale 2

    auto h = index.hitTest(view, px(view, {30, 11.5}), opt);
    REQUIRE(h.has_value());
    CHECK(h->object == open);
    CHECK(h->distancePx == Catch::Approx(3.0));
    CHECK_FALSE(h->viaFill);
    CHECK_FALSE(index.hitTest(view, px(view, {30, 12.5}), opt).has_value());
    // Beyond the end of the segment the distance is to the end point.
    CHECK_FALSE(index.hitTest(view, px(view, {63, 10}), opt).has_value());
    CHECK(index.hitTest(view, px(view, {61, 10}), opt).has_value());

    h = index.hitTest(view, px(view, {120.5, 50}), opt);
    REQUIRE(h.has_value());
    CHECK(h->object == circle);
    // Inside the circle but off the outline: no hit in stroke mode, hit in fill mode.
    CHECK_FALSE(index.hitTest(view, px(view, {100, 50}), opt).has_value());
    opt.fill = FillHit::StrokeAndFill;
    h = index.hitTest(view, px(view, {100, 50}), opt);
    REQUIRE(h.has_value());
    CHECK(h->object == circle);
    CHECK(h->viaFill);
    // Fill never hits an open contour's "inside".
    CHECK_FALSE(index.hitTest(view, px(view, {30, 20}), opt).has_value());
}

TEST_CASE("hit test: even-odd fill leaves holes unpicked", "[viewmodel][selection]") {
    Document doc;
    const ViewTransform view = makeView();
    const ObjectId ring = addVector(doc, {Contour::circle({50, 50}, 30), Contour::circle({50, 50}, 10)});
    HitOptions opt;
    opt.tolerancePx = 2.0;
    opt.fill = FillHit::StrokeAndFill;
    const PickIndex index(doc);
    CHECK(index.hitTest(view, px(view, {50 + 20, 50}), opt).has_value());
    CHECK_FALSE(index.hitTest(view, px(view, {50, 50}), opt).has_value());
    CHECK(index.hitTest(view, px(view, {50, 50 + 10.2}), opt)->object == ring);
}

TEST_CASE("hit test: hidden and locked layers never hit", "[viewmodel][selection]") {
    Document doc;
    document::Layer second;
    second.name = "L2";
    const LayerId l2 = doc.insertLayer(second);
    document::Layer third;
    third.name = "L3";
    const LayerId l3 = doc.insertLayer(third);
    const ViewTransform view = makeView();
    const ObjectId a = addVector(doc, line({0, 0}, {50, 0}), doc.defaultLayer());
    addVector(doc, line({0, 0}, {50, 0}), l2);
    addVector(doc, line({0, 0}, {50, 0}), l3);
    const Vec2 p = px(view, {25, 0});

    CHECK(hitTest(doc, view, p)->object.value > a.value);  // topmost layer wins
    doc.findLayer(l3)->locked = true;
    doc.findLayer(l2)->visible = false;
    CHECK(hitTest(doc, view, p)->object == a);
    doc.findLayer(doc.defaultLayer())->visible = false;
    CHECK_FALSE(hitTest(doc, view, p).has_value());
    CHECK(PickIndex(doc).size() == 0);
}

TEST_CASE("hit test: objects on other sheets are ignored", "[viewmodel][selection]") {
    Document doc;
    document::Sheet s;
    s.name = "Back";
    const document::SheetId other = doc.insertSheet(s);
    const ViewTransform view = makeView();
    Object o = Object::vector(line({0, 0}, {50, 0}), doc.activeLayer(), other);
    doc.insertObject(std::move(o));
    CHECK_FALSE(hitTest(doc, view, px(view, {25, 0})).has_value());
}

TEST_CASE("hit test: groups resolve to the outermost group", "[viewmodel][selection]") {
    Document doc;
    const ViewTransform view = makeView();
    const ObjectId a = addVector(doc, line({0, 0}, {50, 0}));
    const ObjectId b = addVector(doc, line({0, 30}, {50, 30}));
    const ObjectId inner = addGroup(doc, {a});
    const ObjectId outer = addGroup(doc, {inner, b});
    const ObjectId loose = addVector(doc, line({0, 60}, {50, 60}));
    HitOptions opt;
    auto h = hitTest(doc, view, px(view, {25, 0}), opt);
    REQUIRE(h.has_value());
    CHECK(h->object == outer);
    CHECK(h->leaf == a);
    CHECK(hitTest(doc, view, px(view, {25, 30}), opt)->object == outer);
    CHECK(hitTest(doc, view, px(view, {25, 60}), opt)->object == loose);

    // A locked layer on the group hides the whole group from picking.
    document::Layer l;
    l.name = "locked";
    l.locked = true;
    const LayerId lk = doc.insertLayer(l);
    doc.findObject(outer)->layer = lk;
    CHECK_FALSE(hitTest(doc, view, px(view, {25, 0}), opt).has_value());
}

TEST_CASE("hit test: ties go to the topmost layer, then smaller, then later", "[viewmodel][selection]") {
    Document doc;
    document::Layer l;
    l.name = "top";
    const LayerId top = doc.insertLayer(l);
    const ViewTransform view = makeView();
    // Same stroke point on three objects at equal distance.
    const ObjectId big = addVector(doc, Contour::rectangle(Box2{{0, 0}, {100, 100}}), doc.defaultLayer());
    const ObjectId narrow = addVector(doc, line({50, 0}, {60, 0}), doc.defaultLayer());
    const Vec2 p = px(view, {55, 0});
    CHECK(hitTest(doc, view, p)->object == narrow);
    const ObjectId onTop = addVector(doc, line({0, 0}, {100, 0}), top);
    CHECK(hitTest(doc, view, p)->object == onTop);
    (void)big;
    // Equal size on the same layer: the later object wins.
    const ObjectId twin = addVector(doc, line({50, 0}, {60, 0}), doc.defaultLayer());
    doc.findObject(onTop)->layer = doc.defaultLayer();
    CHECK(hitTest(doc, view, p)->object == twin);
    // Nearer outline beats a topmost-but-farther one.
    const ObjectId farther = addVector(doc, line({50, 1}, {60, 1}), top);
    CHECK(hitTest(doc, view, px(view, {55, 0.4}))->object != farther);
}

TEST_CASE("hit test: stroke hit beats interior hit", "[viewmodel][selection]") {
    Document doc;
    document::Layer l;
    l.name = "top";
    const LayerId top = doc.insertLayer(l);
    const ViewTransform view = makeView();
    // Big circle outline on the lower layer; a small filled rectangle on the top layer whose
    // interior contains the point (90,50) on that outline. The rectangle is topmost AND smaller,
    // so only the "outline beats interior" rule lets the circle win.
    const ObjectId big = addVector(doc, Contour::circle({50, 50}, 40), doc.defaultLayer());
    const ObjectId pad = addVector(doc, Contour::rectangle(Box2{{80, 40}, {100, 60}}), top);
    HitOptions opt;
    opt.tolerancePx = 2.0;
    opt.fill = FillHit::StrokeAndFill;
    const auto h = hitTest(doc, view, px(view, {90, 50}), opt);
    REQUIRE(h.has_value());
    CHECK(h->object == big);
    CHECK_FALSE(h->viaFill);
    // Inside the rectangle but away from the circle's outline: the fill hit is the only one.
    const auto f = hitTest(doc, view, px(view, {96, 50}), opt);
    REQUIRE(f.has_value());
    CHECK(f->object == pad);
    CHECK(f->viaFill);
}

TEST_CASE("hit test agrees with a distance oracle on generated scenes", "[viewmodel][selection]") {
    Lcg rng{12345};
    Document doc;
    document::Layer l2;
    l2.name = "hidden";
    l2.visible = false;
    const LayerId hidden = doc.insertLayer(l2);
    document::Layer l3;
    l3.name = "locked";
    l3.locked = true;
    const LayerId locked = doc.insertLayer(l3);

    struct Rec {
        ObjectId top;
        std::vector<Contour> contours;
        bool pickable;
    };
    std::vector<Rec> recs;
    std::vector<ObjectId> pending;  // for grouping pairs
    for (int i = 0; i < 60; ++i) {
        Contour c;
        const Vec2 a{rng.range(0, 300), rng.range(0, 200)};
        switch (i % 5) {
            case 0:
                c = line(a, {a.x + rng.range(-40, 40), a.y + rng.range(-40, 40)});
                break;
            case 1:
                c = arc(a, rng.range(3, 30), rng.range(0, 6), rng.range(0, 6));
                break;
            case 2:
                c = bezier(a, {a.x + rng.range(-30, 30), a.y + rng.range(-30, 30)},
                           {a.x + rng.range(-30, 30), a.y + rng.range(-30, 30)},
                           {a.x + rng.range(-30, 30), a.y + rng.range(-30, 30)});
                break;
            case 3:
                // Non-uniformly scaled and rotated circle becomes Beziers.
                c = Contour::circle(a, rng.range(3, 20))
                        .transformed(geometry::Transform2::rotation(rng.range(0, 3), a) *
                                     geometry::Transform2::scaling(1.0, 0.5, a));
                break;
            default:
                c = Contour::rectangle(Box2{a, {a.x + rng.range(3, 40), a.y + rng.range(3, 40)}});
                c = c.transformed(geometry::Transform2::rotation(rng.range(0, 3), a));
                break;
        }
        const LayerId layer = (i % 11 == 7) ? hidden : (i % 13 == 5) ? locked : doc.defaultLayer();
        const ObjectId id = addVector(doc, c, layer);
        recs.push_back({id, {c}, layer == doc.defaultLayer()});
        if (i % 7 == 3) {
            pending.push_back(id);
        }
    }
    // Group pairs of the pending objects (all on varying layers; group layer = default).
    for (std::size_t k = 0; k + 1 < pending.size(); k += 2) {
        const ObjectId g = addGroup(doc, {pending[k], pending[k + 1]}, doc.defaultLayer());
        for (Rec& r : recs) {
            if (r.top == pending[k] || r.top == pending[k + 1]) {
                r.top = g;
            }
        }
    }

    const ViewTransform view = makeView(3.0);
    const PickIndex index(doc);
    HitOptions opt;
    opt.tolerancePx = 5.0;
    int hits = 0;
    int misses = 0;
    for (int q = 0; q < 600; ++q) {
        const Vec2 pw{rng.range(-10, 310), rng.range(-10, 210)};
        // Oracle: nearest pickable object by sampled distance (px).
        double best = std::numeric_limits<double>::infinity();
        double second = std::numeric_limits<double>::infinity();
        ObjectId bestTop;
        for (const Rec& r : recs) {
            if (!r.pickable) {
                continue;
            }
            const double d = oracleDistance(r.contours, pw) * view.scale();
            if (d < best) {
                second = best;
                best = d;
                bestTop = r.top;
            } else if (d < second) {
                second = d;
            }
        }
        const auto h = index.hitTest(view, view.worldToScreen(pw), opt);
        if (best < opt.tolerancePx - 0.05) {
            REQUIRE(h.has_value());
            ++hits;
            // Members of one group share `top`, so the runner-up may be the same group.
            const bool unambiguous = second - best > 0.1 || second == std::numeric_limits<double>::infinity();
            if (unambiguous) {
                CHECK(h->object == bestTop);
                CHECK(h->distancePx == Catch::Approx(best).margin(0.05));
            }
        } else if (best > opt.tolerancePx + 0.05) {
            CHECK_FALSE(h.has_value());
            ++misses;
        }
    }
    CHECK(hits > 20);
    CHECK(misses > 20);
}

TEST_CASE("marquee: window requires containment, crossing requires contact", "[viewmodel][selection]") {
    Document doc;
    const ViewTransform view = makeView();
    const ObjectId inside = addVector(doc, Contour::rectangle(Box2{{20, 20}, {30, 30}}));
    const ObjectId straddle = addVector(doc, line({40, 50}, {90, 50}));
    const ObjectId outside = addVector(doc, line({200, 200}, {210, 210}));
    const ObjectId containing = addVector(doc, Contour::rectangle(Box2{{-100, -100}, {400, 400}}));
    const ObjectId touchingEdge = addVector(doc, line({60, 60}, {80, 60}));  // meets the corner (60,60)

    // Marquee rectangle in world: [0,0]-[60,60]; screen box is the corner pair.
    const Box2 rectMm{{0, 0}, {60, 60}};
    const Box2 rectPx{px(view, rectMm.min), px(view, rectMm.max)};

    auto win = viewmodel::marqueeSelect(doc, view, rectPx, MarqueeMode::Window);
    CHECK(win == std::vector<ObjectId>{inside});
    auto cross = viewmodel::marqueeSelect(doc, view, rectPx, MarqueeMode::Crossing);
    // `containing` encloses the rectangle without touching it: stroke crossing skips it.
    CHECK(cross == std::vector<ObjectId>{inside, straddle, touchingEdge});
    cross = viewmodel::marqueeSelect(doc, view, rectPx, MarqueeMode::Crossing, FillHit::StrokeAndFill);
    CHECK(cross == std::vector<ObjectId>{inside, straddle, containing, touchingEdge});
    (void)outside;

    // A closed shape enclosing the rectangle: only fill-crossing picks it.
    Document doc2;
    const ObjectId frame = addVector(doc2, Contour::rectangle(Box2{{0, 0}, {300, 300}}));
    const Box2 inner{px(view, {100, 100}), px(view, {110, 110})};
    CHECK(viewmodel::marqueeSelect(doc2, view, inner, MarqueeMode::Crossing).empty());
    CHECK(viewmodel::marqueeSelect(doc2, view, inner, MarqueeMode::Crossing, FillHit::StrokeAndFill) ==
          std::vector<ObjectId>{frame});
}

TEST_CASE("marquee: exact boundary contact and open contours", "[viewmodel][selection]") {
    Document doc;
    const ViewTransform view = makeView();
    const ObjectId open = addVector(doc, line({10, 10}, {50, 10}));
    const Box2 exact{px(view, {10, 10}), px(view, {50, 10.0})};  // zero-height rectangle on the line
    CHECK(viewmodel::marqueeSelect(doc, view, exact, MarqueeMode::Window) == std::vector<ObjectId>{open});
    // Rectangle touching only the end point.
    const Box2 endOnly{px(view, {50, 0}), px(view, {70, 10})};
    CHECK(viewmodel::marqueeSelect(doc, view, endOnly, MarqueeMode::Crossing) == std::vector<ObjectId>{open});
    CHECK(viewmodel::marqueeSelect(doc, view, endOnly, MarqueeMode::Window).empty());
    // Just short of the end point: no contact.
    const Box2 miss{px(view, {50.01, 0}), px(view, {70, 10})};
    CHECK(viewmodel::marqueeSelect(doc, view, miss, MarqueeMode::Crossing).empty());
    // Arc whose bounds meet the rectangle but whose curve does not.
    Document doc2;
    const ObjectId a = addVector(doc2, arc({0, 0}, 50, 0.0, 1.5707963267948966));  // quarter arc, x,y >= 0
    const Box2 bounds{px(view, {40, 40}), px(view, {50, 50})};  // inside the bbox, outside the arc (r>50)
    CHECK(viewmodel::marqueeSelect(doc2, view, bounds, MarqueeMode::Crossing).empty());
    const Box2 onArc{px(view, {34, 34}), px(view, {38, 38})};  // arc passes (35.36, 35.36)
    CHECK(viewmodel::marqueeSelect(doc2, view, onArc, MarqueeMode::Crossing) == std::vector<ObjectId>{a});
    // Any corner order is accepted.
    const Box2 flipped{Vec2{onArc.max.x, onArc.min.y}, Vec2{onArc.min.x, onArc.max.y}};
    CHECK(viewmodel::marqueeSelect(doc2, view, flipped, MarqueeMode::Crossing) == std::vector<ObjectId>{a});
}

TEST_CASE("marquee: groups need all (window) or any (crossing) member", "[viewmodel][selection]") {
    Document doc;
    const ViewTransform view = makeView();
    const ObjectId a = addVector(doc, line({10, 10}, {20, 10}));
    const ObjectId b = addVector(doc, line({100, 10}, {110, 10}));
    const ObjectId g = addGroup(doc, {a, b});
    const Box2 r{px(view, {0, 0}), px(view, {30, 30})};
    CHECK(viewmodel::marqueeSelect(doc, view, r, MarqueeMode::Window).empty());
    CHECK(viewmodel::marqueeSelect(doc, view, r, MarqueeMode::Crossing) == std::vector<ObjectId>{g});
    const Box2 all{px(view, {0, 0}), px(view, {120, 30})};
    CHECK(viewmodel::marqueeSelect(doc, view, all, MarqueeMode::Window) == std::vector<ObjectId>{g});
}

TEST_CASE("selection modes go through the core Selection", "[viewmodel][selection]") {
    Document doc;
    const ViewTransform view = makeView();
    const ObjectId a = addVector(doc, line({0, 0}, {50, 0}));
    const ObjectId b = addVector(doc, line({0, 30}, {50, 30}));
    const ObjectId c = addVector(doc, line({0, 60}, {50, 60}));

    CHECK(viewmodel::clickSelect(doc, view, px(view, {25, 0}), SelectMode::Replace).has_value());
    CHECK(doc.selection().ids() == std::vector<ObjectId>{a});
    viewmodel::clickSelect(doc, view, px(view, {25, 30}), SelectMode::Add);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{a, b});
    viewmodel::clickSelect(doc, view, px(view, {25, 0}), SelectMode::Toggle);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{b});
    // A miss leaves the selection alone in Add/Toggle/Subtract, clears it in Replace.
    CHECK_FALSE(viewmodel::clickSelect(doc, view, px(view, {25, 15}), SelectMode::Add).has_value());
    CHECK(doc.selection().ids() == std::vector<ObjectId>{b});
    viewmodel::clickSelect(doc, view, px(view, {25, 15}), SelectMode::Subtract);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{b});
    viewmodel::clickSelect(doc, view, px(view, {25, 30}), SelectMode::Subtract);
    CHECK(doc.selection().empty());
    viewmodel::clickSelect(doc, view, px(view, {25, 60}), SelectMode::Replace);
    viewmodel::clickSelect(doc, view, px(view, {25, 15}), SelectMode::Replace);
    CHECK(doc.selection().empty());

    const Box2 r{px(view, {-5, -5}), px(view, {60, 40})};
    const auto picked = viewmodel::marqueeApply(doc, view, r, MarqueeMode::Window, SelectMode::Replace);
    CHECK(picked == std::vector<ObjectId>{a, b});
    CHECK(doc.selection().ids() == picked);
    viewmodel::marqueeApply(doc, view, r, MarqueeMode::Window, SelectMode::Subtract);
    CHECK(doc.selection().empty());
    (void)c;
}

TEST_CASE("select all, invert and by layer respect hidden and locked layers", "[viewmodel][selection]") {
    Document doc;
    document::Layer l2;
    l2.name = "L2";
    const LayerId layer2 = doc.insertLayer(l2);
    document::Layer l3;
    l3.name = "L3";
    l3.locked = true;
    const LayerId layer3 = doc.insertLayer(l3);
    const ObjectId a = addVector(doc, line({0, 0}, {5, 0}), doc.defaultLayer());
    const ObjectId b = addVector(doc, line({0, 1}, {5, 1}), layer2);
    const ObjectId c = addVector(doc, line({0, 2}, {5, 2}), layer2);
    addVector(doc, line({0, 3}, {5, 3}), layer3);
    const ObjectId g = addGroup(doc, {c});

    viewmodel::selectAll(doc);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{a, b, g});
    doc.selection().assign({a});
    viewmodel::invertSelection(doc);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{b, g});
    viewmodel::selectByLayer(doc, layer2);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{b, g});
    viewmodel::selectByLayer(doc, layer3);
    CHECK(doc.selection().empty());
    viewmodel::selectByLayer(doc, doc.defaultLayer());
    CHECK(doc.selection().ids() == std::vector<ObjectId>{a});
    doc.findLayer(doc.defaultLayer())->visible = false;
    viewmodel::selectByLayer(doc, doc.defaultLayer());
    CHECK(doc.selection().empty());
}

TEST_CASE("selection bounding box and handles", "[viewmodel][selection]") {
    Document doc;
    const ViewTransform view = makeView();
    CHECK_FALSE(viewmodel::selectionBounds(doc).has_value());
    const ObjectId a = addVector(doc, Contour::rectangle(Box2{{10, 20}, {30, 40}}));
    const ObjectId b = addVector(doc, line({50, 5}, {60, 15}));
    const ObjectId g = addGroup(doc, {a, b});
    doc.selection().assign({g});
    const auto bounds = viewmodel::selectionBounds(doc);
    REQUIRE(bounds.has_value());
    CHECK(geometry::approxEqual(*bounds, Box2{{10, 5}, {60, 40}}));

    const viewmodel::SelectionBox box = viewmodel::selectionBox(*bounds, view);
    using K = viewmodel::HandleKind;
    CHECK(box.handles[0].kind == K::TopLeft);
    CHECK(box.handles[8].kind == K::BottomRight);
    // Top is up on screen: the top-left handle sits at world (min.x, max.y).
    CHECK(box.handles[0].positionMm.x == Catch::Approx(10));
    CHECK(box.handles[0].positionMm.y == Catch::Approx(40));
    CHECK(box.handles[0].positionPx.x == Catch::Approx(view.worldToScreen({10, 40}).x));
    CHECK(box.handles[0].positionPx.y == Catch::Approx(view.worldToScreen({10, 40}).y));
    CHECK(box.handles[4].kind == K::Centre);
    CHECK(box.handles[4].positionMm.x == Catch::Approx(35));
    CHECK(box.handles[4].positionMm.y == Catch::Approx(22.5));
    CHECK(box.handles[5].kind == K::Right);
    CHECK(box.handles[5].positionMm.x == Catch::Approx(60));

    CHECK(viewmodel::hitHandle(box, box.handles[2].positionPx + Vec2{2, -2}, 4.0) == K::TopRight);
    CHECK(viewmodel::hitHandle(box, box.handles[7].positionPx, 4.0) == K::Bottom);
    CHECK_FALSE(viewmodel::hitHandle(box, box.handles[2].positionPx + Vec2{30, 30}, 4.0).has_value());
    // Degenerate box: coincident handles resolve to the first in order.
    const viewmodel::SelectionBox pt = viewmodel::selectionBox(Box2{{5, 5}, {5, 5}}, view);
    CHECK(viewmodel::hitHandle(pt, view.worldToScreen({5, 5}), 3.0) == K::TopLeft);
    // Empty bounds: nothing to hit.
    CHECK_FALSE(viewmodel::hitHandle(viewmodel::selectionBox(Box2{}, view), {0, 0}, 1e9).has_value());
}

TEST_CASE("picking 10k objects uses the spatial index", "[viewmodel][selection][perf]") {
    Document doc;
    Lcg rng{777};
    for (int i = 0; i < 10000; ++i) {
        const Vec2 c{rng.range(0, 1000), rng.range(0, 1000)};
        if (i % 2 == 0) {
            addVector(doc, Contour::circle(c, rng.range(0.5, 3.0)));
        } else {
            addVector(doc, line(c, {c.x + rng.range(-5, 5), c.y + rng.range(-5, 5)}));
        }
    }
    ViewTransform view;
    view.setViewport(1000.0, 1000.0);
    view.set(1.0, {0.0, 1000.0});

    const auto t0 = std::chrono::steady_clock::now();
    const PickIndex index(doc);
    const auto t1 = std::chrono::steady_clock::now();
    REQUIRE(index.size() == 10000);
    int hits = 0;
    const int queries = 2000;
    for (int q = 0; q < queries; ++q) {
        if (index.hitTest(view, {rng.range(0, 1000), rng.range(0, 1000)}).has_value()) {
            ++hits;
        }
    }
    const auto t2 = std::chrono::steady_clock::now();
    const double buildMs = std::chrono::duration<double, std::milli>(t1 - t0).count();
    const double queryMs = std::chrono::duration<double, std::milli>(t2 - t1).count();
    CHECK(hits > 0);
    // Documented bound (generous, Debug build, shared CI machines): index build under 5 s and
    // 2000 picks under 2 s in total, i.e. a millisecond per pick at most; in practice a pick
    // examines a handful of entries.
    CHECK(buildMs < 5000.0);
    CHECK(queryMs < 2000.0);
    // The retained index must agree with a fresh one-shot hit test.
    const Vec2 probe{500.0, 500.0};
    const auto a = index.hitTest(view, probe);
    const auto b = viewmodel::hitTest(doc, view, probe);
    CHECK(a.has_value() == b.has_value());
    if (a.has_value() && b.has_value()) {
        CHECK(a->object == b->object);
    }
}
