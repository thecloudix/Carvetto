#include "carvetto/viewmodel/vector_validator.hpp"

#include <catch2/catch_test_macros.hpp>
#include <utility>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto;
using document::Document;
using document::LayerId;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::validator::IssueKind;
using viewmodel::ValidatorOptions;
using viewmodel::ValidatorScope;

namespace {

Contour square(double x, double y, double s, bool ccw = true) {
    Contour c = Contour::rectangle(Box2{{x, y}, {x + s, y + s}});
    if (!ccw) {
        c.reverse();
    }
    return c;
}

ObjectId add(Document& doc, std::vector<Contour> cs, LayerId layer = {}) {
    return doc.insertObject(Object::vector(std::move(cs), layer ? layer : doc.activeLayer(), doc.activeSheet()));
}

}  // namespace

TEST_CASE("vector_validator: reports the visible vectors and selects offenders", "[vector_validator]") {
    Document doc;
    const ObjectId good = add(doc, {square(0, 0, 10)});
    const ObjectId cwObj = add(doc, {square(20, 0, 10, false)});
    const ObjectId tiny = add(doc, {square(40, 0, 0.002)});
    const ObjectId dupA = add(doc, {square(60, 0, 5)});
    const ObjectId dupB = add(doc, {square(60, 0, 5)});
    const LayerId hidden = doc.insertLayer(document::Layer{{}, "Hidden", {}, false, false});
    add(doc, {square(80, 0, 10, false)}, hidden);

    ValidatorOptions opt;
    const viewmodel::ValidatorReport r = viewmodel::validateVectors(doc, opt);
    CHECK(r.count(IssueKind::WrongDirection) == 1);
    CHECK(r.count(IssueKind::TinyContour) == 1);
    CHECK(r.count(IssueKind::DuplicateContour) == 1);
    CHECK(r.report.total() == 3);
    CHECK_FALSE(r.clean());
    CHECK(r.offenders() == std::vector<ObjectId>{cwObj, tiny, dupB});
    CHECK(r.summary().find("3 problems found") != std::string::npos);

    CHECK(viewmodel::selectOffenders(doc, r, IssueKind::WrongDirection) == 1);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{cwObj});
    CHECK(viewmodel::selectDuplicateVectors(doc) == 1);
    CHECK(doc.selection().ids() == std::vector<ObjectId>{dupB});
    CHECK(viewmodel::selectOffenders(doc, r) == 3);
    CHECK(doc.selection().size() == 3);
    (void)good;
    (void)dupA;
}

TEST_CASE("vector_validator: selection scope and default scope", "[vector_validator]") {
    Document doc;
    const ObjectId a = add(doc, {square(0, 0, 10, false)});
    add(doc, {square(20, 0, 10, false)});
    CHECK(viewmodel::defaultValidatorScope(doc) == ValidatorScope::AllVisible);
    doc.selection().assign({a});
    CHECK(viewmodel::defaultValidatorScope(doc) == ValidatorScope::Selection);
    ValidatorOptions opt;
    opt.scope = ValidatorScope::Selection;
    const viewmodel::ValidatorReport r = viewmodel::validateVectors(doc, opt);
    CHECK(r.objects == std::vector<ObjectId>{a});
    CHECK(r.count(IssueKind::WrongDirection) == 1);
    // Empty selection under Selection scope: nothing to check, nothing found.
    doc.selection().clear();
    CHECK(viewmodel::validateVectors(doc, opt).clean());
}

TEST_CASE("vector_validator: fix is one undo step and removes emptied objects", "[vector_validator]") {
    Document doc;
    const ObjectId cwObj = add(doc, {square(20, 0, 10, false)});
    const ObjectId tiny = add(doc, {square(40, 0, 0.002)});
    const ObjectId dupA = add(doc, {square(60, 0, 5)});
    const ObjectId dupB = add(doc, {square(60, 0, 5)});
    const std::size_t undoBefore = doc.undoStack().count();
    const viewmodel::ValidatorFixResult f = viewmodel::fixVectors(doc);
    REQUIRE(f.ok);
    CHECK(f.fixed.contoursReversed == 1);
    CHECK(f.fixed.tinyRemoved == 1);
    CHECK(f.fixed.duplicatesRemoved == 1);
    CHECK(f.objectsChanged == 1);
    CHECK(f.objectsRemoved == 2);
    CHECK(doc.undoStack().count() == undoBefore + 1);
    CHECK(doc.hasObject(cwObj));
    CHECK_FALSE(doc.hasObject(tiny));
    CHECK(doc.hasObject(dupA));
    CHECK_FALSE(doc.hasObject(dupB));
    CHECK(viewmodel::validateVectors(doc).clean());
    CHECK(doc.checkIntegrity().empty());

    REQUIRE(doc.undo());
    CHECK(doc.hasObject(tiny));
    CHECK(doc.hasObject(dupB));
    CHECK(viewmodel::validateVectors(doc).report.total() == 3);
    REQUIRE(doc.redo());
    CHECK(viewmodel::validateVectors(doc).clean());

    // Nothing left to fix: no undo entry.
    const std::size_t n = doc.undoStack().count();
    const viewmodel::ValidatorFixResult again = viewmodel::fixVectors(doc);
    CHECK(again.ok);
    CHECK(again.fixed.total() == 0);
    CHECK(doc.undoStack().count() == n);
}

TEST_CASE("vector_validator: hidden and locked layers are not touched", "[vector_validator]") {
    Document doc;
    const LayerId locked = doc.insertLayer(document::Layer{{}, "Locked", {}, true, true});
    const ObjectId l = add(doc, {square(0, 0, 10, false)}, locked);
    const viewmodel::ValidatorFixResult f = viewmodel::fixVectors(doc);
    CHECK(f.fixed.total() == 0);
    CHECK(doc.findObject(l)->asVector()->contours[0] == square(0, 0, 10, false));
}

TEST_CASE("vector_validator: V-carving mode skips crossings in text only", "[vector_validator]") {
    Document doc;
    geometry::Polyline bow;
    bow.points = {{0, 0}, {10, 10}, {10, 0}, {0, 10}};
    bow.closed = true;
    const ObjectId plain = add(doc, {Contour::fromPolyline(bow)});
    geometry::Polyline bow2;
    bow2.points = {{50, 0}, {60, 10}, {60, 0}, {50, 10}};
    bow2.closed = true;
    const ObjectId text = add(doc, {Contour::fromPolyline(bow2)});
    doc.findObject(text)->asVector()->params = document::TextParams{};
    ValidatorOptions opt;
    CHECK(viewmodel::validateVectors(doc, opt).count(IssueKind::SelfIntersecting) == 2);
    opt.vCarvingMode = true;
    const viewmodel::ValidatorReport r = viewmodel::validateVectors(doc, opt);
    CHECK(r.count(IssueKind::SelfIntersecting) == 1);
    CHECK(r.offenders() == std::vector<ObjectId>{plain});
}
