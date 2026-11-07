#include "carvetto/viewmodel/transform_form.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto;
using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;
using namespace carvetto::viewmodel;

namespace {

bool near(double a, double b, double tol = 1e-9) { return std::fabs(a - b) <= tol; }

struct Form {
    Document doc{document::Job::singleSided(1000.0, 600.0, 18.0, document::Units::Inches)};
    ObjectId rect;
    Form() {
        rect = doc.insertObject(Object::vector(Contour::rectangle(Box2{{100, 100}, {200, 150}}), doc.activeLayer(),
                                               doc.activeSheet()));
        doc.selection().assign({rect});
    }
    Box2 bounds() const { return *transformBounds(doc); }
};

std::string textOf(const std::vector<NumericField>& fields, const std::string& id) {
    for (const NumericField& f : fields) {
        if (f.id == id) {
            return f.text;
        }
    }
    return "<missing>";
}

}  // namespace

TEST_CASE("move form: units, relative and absolute live values, bad input refused", "[transform][form]") {
    Form f;
    TransformForm form(TransformFormKind::Move, document::Units::Inches);
    CHECK(textOf(form.fields(f.doc), "x") == "0");
    CHECK(form.setField("x", "1").ok);  // inches
    CHECK(form.setField("y", "-5mm").ok);
    REQUIRE(form.apply(f.doc).ok);
    CHECK(near(f.bounds().min.x, 100.0 + 25.4));
    CHECK(near(f.bounds().min.y, 95.0));

    const auto before = f.bounds();
    CHECK_FALSE(form.setField("x", "abc").ok);
    CHECK_FALSE(form.setField("x", "nan").ok);
    CHECK_FALSE(form.setField("x", "1e12").ok);
    CHECK_FALSE(form.setField("nope", "1").ok);
    CHECK(textOf(form.fields(f.doc), "x") == "1");  // a bad entry changed nothing
    CHECK(f.bounds().min.x == before.min.x);

    // Absolute: live text is where the anchor is now; typing one field keeps the other.
    TransformForm abs(TransformFormKind::Move, document::Units::Millimetres);
    abs.options().absolute = true;
    abs.options().anchor = HandleKind::BottomLeft;
    CHECK(textOf(abs.fields(f.doc), "x") == "125.40");
    CHECK(abs.setField("x", "10").ok);
    REQUIRE(abs.apply(f.doc).ok);
    CHECK(near(f.bounds().min.x, 10.0));
    CHECK(near(f.bounds().min.y, 95.0));
    // Clearing a field restores the live value.
    CHECK(abs.setField("x", "").ok);
    CHECK(textOf(abs.fields(f.doc), "x") == "10.00");
}

TEST_CASE("scale form: sizes, factors typed as value S, link XY, refusals", "[transform][form]") {
    Form f;
    TransformForm form(TransformFormKind::Scale, document::Units::Millimetres);
    CHECK(textOf(form.fields(f.doc), "width") == "100.00");
    CHECK_FALSE(form.apply(f.doc).ok);  // nothing typed
    CHECK(form.setField("width", "200").ok);
    REQUIRE(form.apply(f.doc).ok);  // linked: height follows
    CHECK(near(f.bounds().width(), 200.0));
    CHECK(near(f.bounds().height(), 100.0));
    CHECK(near(f.bounds().centre().x, 150.0));  // about the centre

    TransformForm unlinked(TransformFormKind::Scale, document::Units::Millimetres);
    unlinked.options().linkXY = false;
    unlinked.options().anchor = HandleKind::BottomLeft;
    CHECK(unlinked.setField("height", "2 S").ok);
    REQUIRE(unlinked.apply(f.doc).ok);
    CHECK(near(f.bounds().height(), 200.0));
    CHECK(near(f.bounds().width(), 200.0));
    CHECK(near(f.bounds().min.y, 75.0));

    for (const char* bad : {"0", "-5", "0 S", "-1 S", "x S", "nan", "inf", "1e12 S", "S"}) {
        TransformForm t(TransformFormKind::Scale, document::Units::Millimetres);
        CHECK_FALSE(t.setField("width", bad).ok);
        CHECK_FALSE(t.apply(f.doc).ok);
    }
    // A line has no height: scale-to-height is refused, no change.
    Form line;
    line.doc.selection().clear();
    Contour c;
    c.append(geometry::LineSpan{{0, 0}, {100, 0}});
    const ObjectId lid = line.doc.insertObject(Object::vector(c, line.doc.activeLayer(), line.doc.activeSheet()));
    line.doc.selection().assign({lid});
    TransformForm h(TransformFormKind::Scale, document::Units::Millimetres);
    h.options().linkXY = false;
    CHECK(h.setField("height", "20").ok);
    CHECK_FALSE(h.apply(line.doc).ok);
    CHECK(line.doc.undoStack().count() == 0);
}

TEST_CASE("rotate form: degrees, relative and absolute, pivot coordinates", "[transform][form]") {
    Form f;
    TransformForm form(TransformFormKind::Rotate, document::Units::Millimetres);
    CHECK(form.fields(f.doc).size() == 1);
    CHECK(form.setField("angle", "90").ok);
    CHECK(form.setField("angle", "-45 deg").ok);
    CHECK_FALSE(form.setField("angle", "1e9").ok);
    CHECK_FALSE(form.setField("angle", "x").ok);
    CHECK(form.setField("angle", "90").ok);
    form.options().useCoordinates = true;
    CHECK(form.fields(f.doc).size() == 3);
    CHECK(textOf(form.fields(f.doc), "pivotX") == "150.00");
    CHECK(form.setField("pivotX", "100").ok);
    CHECK(form.setField("pivotY", "100").ok);
    REQUIRE(form.apply(f.doc).ok);
    // The rectangle (100..200 x 100..150) turned 90 degrees about (100,100): x in [50,100], y in [100,200].
    CHECK(near(f.bounds().min.x, 50.0, 1e-9));
    CHECK(near(f.bounds().max.x, 100.0, 1e-9));
    CHECK(near(f.bounds().max.y, 200.0, 1e-9));
    CHECK(f.doc.undoStack().count() == 1);

    // Absolute with no orientation to read: target 0 on a plain vector is "rotate by the angle".
    TransformForm abs(TransformFormKind::Rotate, document::Units::Millimetres);
    abs.options().absolute = true;
    CHECK(textOf(abs.fields(f.doc), "angle") == "0.00");
    CHECK(abs.apply(f.doc).ok);  // nothing typed: orientation unchanged, no undo step
    CHECK(f.doc.undoStack().count() == 1);
}

TEST_CASE("mirror form: options map onto the operation", "[transform][form]") {
    Form f;
    TransformForm form(TransformFormKind::Mirror, document::Units::Millimetres);
    CHECK(form.fields(f.doc).empty());
    CHECK_FALSE(form.setField("x", "1").ok);
    form.options().createCopy = true;
    form.options().axis = MirrorAxis::Vertical;
    const TransformResult r = form.apply(f.doc);
    REQUIRE(r.ok);
    CHECK(r.created.size() == 1);
    CHECK(f.doc.objectCount() == 2);
    form.options().createCopy = false;
    form.options().aboutSelectedLine = true;
    CHECK_FALSE(form.apply(f.doc).ok);  // the selected copy is not a single line
}
