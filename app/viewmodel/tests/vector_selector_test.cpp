// T-0220: the Vector Selector (resolution rules, filters, validation, fingerprint).

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>

#include "carvetto/document/commands.hpp"
#include "carvetto/viewmodel/vector_selector.hpp"
#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using document::Document;
using document::ObjectId;

namespace {

geometry::Contour openLine(double x) {
    geometry::Contour c;
    c.append(geometry::LineSpan{{x, 0}, {x + 5, 0}});
    c.closed = false;
    return c;
}

geometry::Contour twoArcCircle(double cx, double cy, double r) {
    geometry::Contour c;
    c.append(geometry::ArcSpan{{cx, cy}, r, 0.0, geometry::kPi, geometry::ArcDirection::CCW});
    c.append(geometry::ArcSpan{{cx, cy}, r, geometry::kPi, 2.0 * geometry::kPi, geometry::ArcDirection::CCW});
    c.closed = true;
    return c;
}

}  // namespace

TEST_CASE("selector: every vector, in document order", "[selector]") {
    Document doc;
    const ObjectId a = addVector(doc, square(0, 0, 5));
    const ObjectId b = addVector(doc, openLine(10));
    const ObjectId c = addVector(doc, square(20, 0, 5));
    const ResolvedVectors r = resolveVectors(doc, VectorSelector::allVectors());
    REQUIRE(r.objects == std::vector<ObjectId>{a, b, c});
    REQUIRE(r.contours.size() == 3);
    CHECK(r.ownerOf(1) == b);
    CHECK(r.ownerOf(9).isNull());
    CHECK(r.spanCount == 4 + 1 + 4);
    CHECK(r.copyContours().size() == 3);
}

TEST_CASE("selector: layers and explicit objects keep the core's order", "[selector]") {
    Document doc;
    const document::LayerId l2 = addLayer(doc, "Two");
    const ObjectId a = addVector(doc, square(0, 0, 5), l2);
    const ObjectId b = addVector(doc, square(10, 0, 5));
    const ObjectId c = addVector(doc, square(20, 0, 5), l2);
    CHECK(resolveVectors(doc, VectorSelector::fromLayers({l2})).objects == std::vector<ObjectId>{a, c});
    // Objects first in list order, then layer vectors not yet taken.
    VectorSelector both;
    both.base = document::VectorSelection{false, {l2}, {c, b}};
    CHECK(resolveVectors(doc, both).objects == std::vector<ObjectId>{c, b, a});
    CHECK(resolveVectors(doc, VectorSelector::fromObjects({b})).objects == std::vector<ObjectId>{b});
    // An empty selection selects nothing.
    CHECK(resolveVectors(doc, VectorSelector::fromObjects({})).objects.empty());
}

TEST_CASE("selector: hidden and locked layers and other sheets never contribute", "[selector]") {
    Document doc;
    const document::LayerId hidden = addLayer(doc, "Hidden");
    const document::LayerId locked = addLayer(doc, "Locked");
    const ObjectId ok = addVector(doc, square(0, 0, 5));
    addVector(doc, square(10, 0, 5), hidden);
    addVector(doc, square(20, 0, 5), locked);
    doc.findLayer(hidden)->visible = false;
    doc.findLayer(locked)->locked = true;
    // An object on another sheet.
    document::Sheet other;
    other.name = "Second";
    const document::SheetId s2 = doc.insertSheet(other);
    document::Object foreign = document::Object::vector(square(30, 0, 5), doc.defaultLayer(), s2);
    doc.insertObject(foreign);

    const ResolvedVectors r = resolveVectors(doc, VectorSelector::allVectors());
    CHECK(r.objects == std::vector<ObjectId>{ok});
    CHECK(r.skippedHidden == 1);
    CHECK(r.skippedLocked == 1);
    CHECK(r.skippedOtherSheet == 1);

    doc.setActiveSheet(s2);
    CHECK(resolveVectors(doc, VectorSelector::allVectors()).objects.size() == 1);
}

TEST_CASE("selector: a group on a hidden layer hides its children", "[selector]") {
    Document doc;
    const document::LayerId hidden = addLayer(doc, "Hidden");
    const ObjectId child = addVector(doc, square(0, 0, 5));
    document::Object group;
    group.layer = hidden;
    group.sheet = doc.activeSheet();
    group.data = document::GroupObject{{child}};
    const ObjectId g = doc.insertObject(group);
    doc.findObject(child)->parent = g;
    doc.findLayer(hidden)->visible = false;
    CHECK(resolveVectors(doc, VectorSelector::allVectors()).objects.empty());
}

TEST_CASE("selector: open, closed and circle filters apply per contour", "[selector]") {
    Document doc;
    const ObjectId sq = addVector(doc, square(0, 0, 5));
    const ObjectId line = addVector(doc, openLine(10));
    const ObjectId circle = addVector(doc, geometry::Contour::circle({30, 0}, 3.0));
    const ObjectId split = addVector(doc, twoArcCircle(50, 0, 2.0));
    const ObjectId mixed = addVector(doc, square(70, 0, 4));
    doc.findObject(mixed)->asVector()->contours.push_back(openLine(80));

    VectorSelector s;
    s.includeOpen = false;
    CHECK(resolveVectors(doc, s).objects == std::vector<ObjectId>{sq, circle, split, mixed});
    CHECK(resolveVectors(doc, s).filteredOut == 2);
    s.includeOpen = true;
    s.includeClosed = false;
    const ResolvedVectors openOnly = resolveVectors(doc, s);
    CHECK(openOnly.objects == std::vector<ObjectId>{line, mixed});
    CHECK(openOnly.contours.size() == 2);

    VectorSelector circles;
    circles.onlyCircles = true;
    CHECK(resolveVectors(doc, circles).objects == std::vector<ObjectId>{circle, split});
    circles.circleDiameterMm = 6.0;
    CHECK(resolveVectors(doc, circles).objects == std::vector<ObjectId>{circle});
    circles.circleDiameterMm = 4.0;
    circles.circleToleranceMm = 0.001;
    CHECK(resolveVectors(doc, circles).objects == std::vector<ObjectId>{split});
    circles.circleDiameterMm = 5.0;
    CHECK(resolveVectors(doc, circles).objects.empty());
    CHECK(VectorSelector{}.hasFilters() == false);
    CHECK(circles.hasFilters());
}

TEST_CASE("selector: ids that name nothing are skipped", "[selector]") {
    Document doc;
    const ObjectId a = addVector(doc, square(0, 0, 5));
    const ObjectId b = addVector(doc, square(10, 0, 5));
    VectorSelector s = VectorSelector::fromObjects({a, b});
    REQUIRE(doc.execute(std::make_unique<document::RemoveObjectsCommand>(std::vector<ObjectId>{a})));
    CHECK(resolveVectors(doc, s).objects == std::vector<ObjectId>{b});
    doc.undo();
    CHECK(resolveVectors(doc, s).objects == std::vector<ObjectId>{a, b});
    // The selection helper drops ids that no longer exist.
    doc.selection().assign({a, b});
    REQUIRE(doc.execute(std::make_unique<document::RemoveObjectsCommand>(std::vector<ObjectId>{b})));
    CHECK(VectorSelector::fromSelection(doc).base.objects == std::vector<ObjectId>{a});
}

TEST_CASE("selector: validation names unusable selections", "[selector]") {
    Document doc;
    const ObjectId a = addVector(doc, square(0, 0, 5));
    const document::LayerId layer = doc.defaultLayer();
    CHECK(validateSelector(doc, VectorSelector::allVectors()).empty());
    CHECK(validateSelector(doc, VectorSelector::fromObjects({a})).empty());
    CHECK(validateSelector(doc, VectorSelector::fromLayers({layer})).empty());
    CHECK_FALSE(validateSelector(doc, VectorSelector::fromObjects({a, a})).empty());
    CHECK_FALSE(validateSelector(doc, VectorSelector::fromObjects({ObjectId{layer.value}})).empty());
    CHECK_FALSE(validateSelector(doc, VectorSelector::fromObjects({ObjectId{99999}})).empty());
    CHECK_FALSE(validateSelector(doc, VectorSelector::fromObjects({ObjectId{}})).empty());
    CHECK_FALSE(validateSelector(doc, VectorSelector::fromLayers({document::LayerId{a.value}})).empty());
    VectorSelector mixed;
    mixed.base = document::VectorSelection{true, {layer}, {}};
    CHECK_FALSE(validateSelector(doc, mixed).empty());
    VectorSelector bad;
    bad.onlyCircles = true;
    bad.circleDiameterMm = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(validateSelector(doc, bad).empty());
    bad.circleDiameterMm = -1.0;
    CHECK_FALSE(validateSelector(doc, bad).empty());
}

TEST_CASE("selector: what a record stores", "[selector]") {
    Document doc;
    const ObjectId a = addVector(doc, square(0, 0, 5));
    addVector(doc, openLine(10));
    VectorSelector plain = VectorSelector::fromObjects({a, a});
    const ResolvedVectors r = resolveVectors(doc, plain);
    CHECK(storedSelection(plain, r).objects == std::vector<ObjectId>{a});  // repeated ids dropped
    CHECK(storedSelection(VectorSelector::allVectors(), r).all);
    VectorSelector filtered;
    filtered.includeOpen = false;
    const ResolvedVectors rf = resolveVectors(doc, filtered);
    const document::VectorSelection stored = storedSelection(filtered, rf);
    CHECK_FALSE(stored.all);
    CHECK(stored.objects == std::vector<ObjectId>{a});
    VectorSelector everything;
    everything.base = document::VectorSelection{true, {document::LayerId{1}}, {ObjectId{2}}};
    CHECK(normalisedSelector(everything).base == document::VectorSelection::allVectors());
}

TEST_CASE("selector: the fingerprint follows the geometry", "[selector]") {
    Document doc;
    const ObjectId a = addVector(doc, square(0, 0, 5));
    const std::uint64_t f0 = fingerprint(resolveVectors(doc, VectorSelector::allVectors()));
    CHECK(f0 == fingerprint(resolveVectors(doc, VectorSelector::allVectors())));
    addVector(doc, square(10, 0, 5));
    const std::uint64_t f1 = fingerprint(resolveVectors(doc, VectorSelector::allVectors()));
    CHECK(f1 != f0);
    doc.undo();
    CHECK(fingerprint(resolveVectors(doc, VectorSelector::allVectors())) == f0);
    // A moved corner changes it; -0 and 0 do not.
    doc.findObject(a)->asVector()->contours[0] = square(0.001, 0, 5);
    CHECK(fingerprint(resolveVectors(doc, VectorSelector::allVectors())) != f0);
    Fingerprint x;
    Fingerprint y;
    x.add(0.0);
    y.add(-0.0);
    CHECK(x.value() == y.value());
    Fingerprint s1;
    Fingerprint s2;
    s1.add(std::string_view("ab"));
    s2.add(std::string_view("ba"));
    CHECK(s1.value() != s2.value());
}
