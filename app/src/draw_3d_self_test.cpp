// Self-test of drawing in the 3D view (T-0345): drives the real Model view with mouse events offscreen.

#include "draw_3d_self_test.hpp"

#include <QComboBox>
#include <QCoreApplication>
#include <QMouseEvent>
#include <cmath>
#include <memory>
#include <string>
#include <variant>

#include "carvetto/document/document.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/geometry/box2.hpp"
#include "main_window.hpp"
#include "relief_panels.hpp"

namespace carvetto::app {

namespace {

#define D3_REQUIRE(cond, text)                         \
    do {                                               \
        if (!(cond)) {                                 \
            return std::string("3D drawing: ") + (text); \
        }                                              \
    } while (false)

void mouse(ModelView3D* view, QEvent::Type type, QPointF at, Qt::MouseButton button, Qt::MouseButtons buttons) {
    QMouseEvent e(type, at, view->mapToGlobal(at), button, buttons, Qt::NoModifier);
    QCoreApplication::sendEvent(view, &e);
}

bool near(double a, double b) {
    return std::abs(a - b) < 2.0;  // the pointer is a whole pixel: a couple of mm at this zoom
}

}  // namespace

std::string draw3dSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(200.0, 100.0, 18.0, document::Units::Millimetres)));
    window.documentChanged();
    ModelView3D* view = window.modelView();
    view->resize(640, 480);
    D3_REQUIRE(view->findChild<QComboBox*>(QStringLiteral("drawShapeBox")) != nullptr, "the Draw shape box is missing");
    view->setDrawShape(viewmodel::Draw3DShape::Rectangle);
    view->camera().viewTop();
    view->cameraChanged();
    document::Document& doc = window.currentDocument();

    // A rectangle by dragging between two plane points.
    const auto a = view->planePixel(40.0, 20.0);
    const auto b = view->planePixel(120.0, 70.0);
    D3_REQUIRE(a && b, "the plane has no screen position");
    mouse(view, QEvent::MouseMove, {a->x, a->y}, Qt::NoButton, Qt::NoButton);
    mouse(view, QEvent::MouseButtonPress, {a->x, a->y}, Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseMove, {0.5 * (a->x + b->x), 0.5 * (a->y + b->y)}, Qt::NoButton, Qt::LeftButton);
    mouse(view, QEvent::MouseMove, {b->x, b->y}, Qt::NoButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, {b->x, b->y}, Qt::LeftButton, Qt::NoButton);
    D3_REQUIRE(doc.objects().size() == 1 && doc.undoStack().count() == 1, "the rectangle was not added in one step");
    {
        const document::Object& o = doc.objects().rbegin()->second;
        const auto* p = o.asVector() ? std::get_if<document::RectangleParams>(&o.asVector()->params) : nullptr;
        D3_REQUIRE(p != nullptr, "the new object is not a rectangle");
        D3_REQUIRE(near(p->centreMm.x, 80.0) && near(p->centreMm.y, 45.0) && near(p->widthMm, 80.0) &&
                       near(p->heightMm, 50.0),
                   "the rectangle is not at the picked plane coordinates");
    }

    // A circle by click-click, tilted view.
    view->setDrawShape(viewmodel::Draw3DShape::Circle);
    view->camera() = viewmodel::OrbitCamera{};
    view->cameraChanged();
    const auto c = view->planePixel(100.0, 50.0);
    const auto e = view->planePixel(130.0, 50.0);
    D3_REQUIRE(c && e, "the tilted plane has no screen position");
    for (const auto& q : {*c, *e}) {
        mouse(view, QEvent::MouseButtonPress, {q.x, q.y}, Qt::LeftButton, Qt::LeftButton);
        mouse(view, QEvent::MouseButtonRelease, {q.x, q.y}, Qt::LeftButton, Qt::NoButton);
    }
    D3_REQUIRE(doc.objects().size() == 2 && doc.undoStack().count() == 2, "the circle was not added in one step");
    {
        const document::Object& o = doc.objects().rbegin()->second;
        const auto* p = o.asVector() ? std::get_if<document::CircleParams>(&o.asVector()->params) : nullptr;
        D3_REQUIRE(p != nullptr && near(p->centreMm.x, 100.0) && near(p->centreMm.y, 50.0) && near(p->radiusMm, 30.0),
                   "the circle is not at the picked plane coordinates");
    }

    // Editing from the 3D view (T-0346): drop the circle, then select, move and node-edit the rectangle.
    D3_REQUIRE(doc.undo() && doc.objects().size() == 1, "undo did not remove the circle");
    window.documentChanged();
    view->camera().viewTop();
    view->cameraChanged();
    const auto bounds = [&doc]() {
        const document::Object& o = doc.objects().begin()->second;
        geometry::Box2 box = o.asVector()->contours.front().bounds();
        for (const geometry::Contour& contour : o.asVector()->contours) {
            box.expand(contour.bounds());
        }
        return box;
    };
    const auto dragPlane = [&](double x0, double y0, double x1, double y1) -> bool {
        const auto p0 = view->planePixel(x0, y0);
        const auto p1 = view->planePixel(x1, y1);
        if (!p0 || !p1) {
            return false;
        }
        mouse(view, QEvent::MouseMove, {p0->x, p0->y}, Qt::NoButton, Qt::NoButton);
        mouse(view, QEvent::MouseButtonPress, {p0->x, p0->y}, Qt::LeftButton, Qt::LeftButton);
        mouse(view, QEvent::MouseMove, {0.5 * (p0->x + p1->x), 0.5 * (p0->y + p1->y)}, Qt::NoButton, Qt::LeftButton);
        mouse(view, QEvent::MouseMove, {p1->x, p1->y}, Qt::NoButton, Qt::LeftButton);
        mouse(view, QEvent::MouseButtonRelease, {p1->x, p1->y}, Qt::LeftButton, Qt::NoButton);
        return true;
    };
    view->setDrawShape(viewmodel::Draw3DShape::Select);
    doc.selection().clear();
    const geometry::Box2 b0 = bounds();
    D3_REQUIRE(dragPlane(b0.min.x, 0.5 * (b0.min.y + b0.max.y), b0.min.x, 0.5 * (b0.min.y + b0.max.y)),
               "the rectangle edge has no screen position");
    D3_REQUIRE(doc.selection().size() == 1, "a click in the 3D view did not select the vector");
    const std::size_t stepsBefore = doc.undoStack().index();
    const double cx = 0.5 * (b0.min.x + b0.max.x), cy = 0.5 * (b0.min.y + b0.max.y);
    D3_REQUIRE(dragPlane(cx, cy, cx + 30.0, cy + 10.0), "the move drag has no screen position");
    const geometry::Box2 b1 = bounds();
    D3_REQUIRE(near(b1.min.x, b0.min.x + 30.0) && near(b1.min.y, b0.min.y + 10.0) &&
                   near(b1.max.x - b1.min.x, b0.max.x - b0.min.x),
               "dragging the selection in the 3D view did not move it by the pointer delta");
    D3_REQUIRE(doc.undoStack().index() == stepsBefore + 1, "the move drag was not one undo step");
    D3_REQUIRE(doc.undo() && near(bounds().min.x, b0.min.x), "undo did not restore the position");

    view->setDrawShape(viewmodel::Draw3DShape::NodeEdit);
    D3_REQUIRE(dragPlane(b0.min.x, 0.5 * (b0.min.y + b0.max.y), b0.min.x, 0.5 * (b0.min.y + b0.max.y)) &&
                   doc.selection().size() == 1,
               "node edit did not pick the vector");
    const std::size_t nodeBefore = doc.undoStack().index();
    D3_REQUIRE(dragPlane(b0.max.x, b0.max.y, b0.max.x + 20.0, b0.max.y + 15.0), "the node drag has no screen position");
    const geometry::Box2 b2 = bounds();
    D3_REQUIRE(near(b2.max.x, b0.max.x + 20.0) && near(b2.max.y, b0.max.y + 15.0) && near(b2.min.x, b0.min.x),
               "node edit in the 3D view did not move the corner node");
    D3_REQUIRE(doc.undoStack().index() == nodeBefore + 1, "the node drag was not one undo step");
    D3_REQUIRE(doc.undo() && near(bounds().max.x, b0.max.x), "undo did not restore the node");

    // Undo removes the rectangle.
    D3_REQUIRE(doc.undo() && doc.objects().empty(), "undo did not remove the rectangle");
    window.documentChanged();

    // An edge-on view refuses the pick.
    view->setDrawShape(viewmodel::Draw3DShape::Rectangle);
    view->camera().viewFront();
    view->cameraChanged();
    mouse(view, QEvent::MouseButtonPress, {300, 240}, Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, {300, 240}, Qt::LeftButton, Qt::NoButton);
    D3_REQUIRE(doc.objects().empty() && !view->drawMessage().isEmpty(), "an edge-on pick was not refused");

    // The wrapped rotary view refuses too.
    window.setDocument(std::make_unique<document::Document>(
        document::Job::rotary(60.0, 20.0, document::Units::Millimetres, document::RotaryAxis::X,
                              document::ZZero::MaterialSurface)));
    window.documentChanged();
    view->setDrawShape(viewmodel::Draw3DShape::Rectangle);
    view->camera().viewTop();
    view->cameraChanged();
    D3_REQUIRE(view->showingWrapped(), "the rotary Model view is not wrapped");
    mouse(view, QEvent::MouseButtonPress, {300, 240}, Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, {330, 260}, Qt::LeftButton, Qt::NoButton);
    D3_REQUIRE(window.currentDocument().objects().empty() && !view->drawMessage().isEmpty(),
               "a pick in the wrapped view was not refused");

    view->setDrawShape(std::nullopt);
    return {};
}

}  // namespace carvetto::app
