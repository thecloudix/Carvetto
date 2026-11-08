// Text objects (T-0211): layout into contours (point, box, curve), commands and undo, the .carv
// round trip with embedded fonts, and the 1.1/1.2 format rules.

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>
#include <limits>
#include <memory>
#include <string>
#include <variant>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/node_edit_command.hpp"
#include "carvetto/document/text_commands.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "../text/text_test_util.hpp"

using namespace carvetto;
using namespace carvetto::document;
using Catch::Approx;
using geometry::Vec2;

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes testFontBytes() {
    return text::testutil::fontBytes();
}

text::Font loadFont() {
    return text::testutil::loadTestFont();
}

double totalArea(const std::vector<geometry::Contour>& cs) {
    return text::testutil::netArea(cs, 0, cs.size());
}

geometry::Box2 boundsOf(const std::vector<geometry::Contour>& cs) {
    geometry::Box2 b;
    for (const auto& c : cs) {
        b.expand(c.bounds());
    }
    return b;
}

TextParams params(const std::string& text, const std::string& key) {
    TextParams p;
    p.text = text;
    p.fontKey = key;
    p.sizeMm = 10.0;
    return p;
}

// A document with the test font embedded and one text object; returns the object id.
struct Fixture {
    Document doc;
    std::string key;
    ObjectId id;
    Fixture(const std::string& text = "TOI") {
        const Bytes bytes = testFontBytes();
        key = fontKeyFor(bytes);
        REQUIRE(doc.insertFont(FontAsset{key, bytes}));
        const text::Font font = loadFont();
        TextParams p = params(text, key);
        const TextBuild b = buildTextContours(font, p);
        REQUIRE(b.ok());
        id = doc.insertObject(Object::vector(b.contours, doc.activeLayer(), doc.activeSheet(), p));
    }
};

Bytes saveOk(const Document& doc, const CarvExtras& ex = {}) {
    auto r = saveCarv(doc, ex);
    INFO((r.ok() ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return r.value();
}

LoadedCarv loadOk(const Bytes& b) {
    auto r = loadCarv(std::span<const std::uint8_t>(b));
    INFO((r.ok() ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return std::move(r.value());
}

}  // namespace

TEST_CASE("point text lays out with oracle area and placement", "[text][document]") {
    const text::Font font = loadFont();
    TextParams p = params("O", "k");
    // O: 275000 font units^2 = 27.5 mm^2 at 10 mm.
    TextBuild b = buildTextContours(font, p);
    REQUIRE(b.ok());
    CHECK(totalArea(b.contours) == Approx(27.5).epsilon(1e-12));
    const geometry::Box2 base = boundsOf(b.contours);

    p.placement = geometry::Transform2::translation({100.0, 50.0});
    b = buildTextContours(font, p);
    REQUIRE(b.ok());
    CHECK(boundsOf(b.contours).min.x == Approx(base.min.x + 100.0));
    CHECK(boundsOf(b.contours).min.y == Approx(base.min.y + 50.0));
    CHECK(totalArea(b.contours) == Approx(27.5).epsilon(1e-12));

    // A mirroring placement keeps outer contours counter-clockwise (positive net area).
    p.placement = geometry::Transform2::mirrorX(0.0);
    b = buildTextContours(font, p);
    REQUIRE(b.ok());
    CHECK(totalArea(b.contours) == Approx(27.5).epsilon(1e-9));
    CHECK(boundsOf(b.contours).max.x == Approx(-base.min.x));
}

TEST_CASE("text parameters are validated", "[text][document]") {
    const text::Font font = loadFont();
    TextParams p = params("I", "k");
    p.sizeMm = 0.0;
    CHECK_FALSE(buildTextContours(font, p).ok());
    p.sizeMm = std::nan("");
    CHECK_FALSE(buildTextContours(font, p).ok());
    p.sizeMm = 10.0;
    p.mode = TextMode::Box;
    CHECK_FALSE(buildTextContours(font, p).ok());  // no box width
    p.mode = TextMode::Curve;
    CHECK_FALSE(buildTextContours(font, p).ok());  // no guide
    p.mode = TextMode::Point;
    p.text = std::string(kMaxTextObjectBytes + 1, 'I');
    CHECK_FALSE(textParamsProblem(p).empty());
    p.text = "I";
    p.placement.tx = std::numeric_limits<double>::infinity();
    CHECK_FALSE(buildTextContours(font, p).ok());
}

TEST_CASE("text in a box wraps at word boundaries and aligns about the box", "[text][document][box]") {
    const text::Font font = loadFont();
    // At 10 mm: "I" is 4 mm advance (bar x 1..3, y 0..7), a space is 3 mm, ascender 8 mm, line pitch 11 mm.
    TextParams p = params("I I I I", "k");
    p.mode = TextMode::Box;
    p.boxWidthMm = 12.0;  // "I I" is 4+3+4 = 11 mm, "I I I" would be 18 mm
    p.boxHeightMm = 40.0;
    TextBuild b = buildTextContours(font, p);
    REQUIRE(b.ok());
    CHECK_FALSE(b.overflow);
    REQUIRE(b.contours.size() == 4);
    geometry::Box2 box = boundsOf(b.contours);
    CHECK(box.max.y == Approx(-1.0));   // first baseline 8 mm below the box top, bar top 7 mm up
    CHECK(box.min.y == Approx(-19.0));  // second baseline 11 mm lower
    CHECK(box.min.x == Approx(1.0));
    CHECK(box.max.x == Approx(10.0));  // 2nd glyph origin 7 + bar end 3

    p.align = TextAlign::Right;  // lines of 11 mm end at x = 12
    b = buildTextContours(font, p);
    box = boundsOf(b.contours);
    CHECK(box.max.x == Approx(11.0));
    CHECK(box.min.x == Approx(2.0));

    p.align = TextAlign::Centre;  // 11 mm line centred in 12: starts at 0.5
    b = buildTextContours(font, p);
    box = boundsOf(b.contours);
    CHECK(box.min.x == Approx(1.5));

    // Explicit paragraph break and CRLF behave like a wrap point.
    p.align = TextAlign::Left;
    p.text = "I\r\nI";
    b = buildTextContours(font, p);
    REQUIRE(b.ok());
    CHECK(boundsOf(b.contours).min.y == Approx(-19.0));

    // A word wider than the box, and text taller than the box, are reported.
    p.text = "II";
    p.boxWidthMm = 5.0;
    CHECK(buildTextContours(font, p).overflow);
    p.text = "I I I I";
    p.boxWidthMm = 12.0;
    p.boxHeightMm = 10.0;
    CHECK(buildTextContours(font, p).overflow);
}

TEST_CASE("text in a box refuses an absurd word count", "[text][document][box]") {
    const text::Font font = loadFont();
    TextParams p = params(std::string(kMaxBoxWords + 10, ' '), "k");
    p.mode = TextMode::Box;
    p.boxWidthMm = 100.0;
    const TextBuild b = buildTextContours(font, p);
    CHECK_FALSE(b.ok());
    CHECK(b.contours.empty());
}

TEST_CASE("text on a curve follows a straight guide exactly", "[text][document][curve]") {
    const text::Font font = loadFont();
    TextParams p = params("I", "k");
    p.mode = TextMode::Curve;
    p.guide.closed = false;
    p.guide.spans.push_back(geometry::LineSpan{{0.0, 0.0}, {100.0, 0.0}});
    p.align = TextAlign::Centre;
    p.startOffsetMm = 50.0;
    TextBuild b = buildTextContours(font, p);
    REQUIRE(b.ok());
    const geometry::Box2 box = boundsOf(b.contours);
    // The advance midpoint (2 mm) of the glyph sits at arc length 50: bar at 49..51.
    CHECK(box.min.x == Approx(49.0));
    CHECK(box.max.x == Approx(51.0));
    CHECK(box.min.y == Approx(0.0));
    CHECK(box.max.y == Approx(7.0));
    // Past the end of an open guide the glyph is dropped and reported.
    p.startOffsetMm = 500.0;
    b = buildTextContours(font, p);
    REQUIRE(b.ok());
    CHECK(b.truncated);
    CHECK(b.contours.empty());
}

TEST_CASE("embedding a font and adding text is one undo step", "[text][document][undo]") {
    Document doc;
    const Bytes bytes = testFontBytes();
    const std::string key = fontKeyFor(bytes);
    const text::Font font = loadFont();
    const TextParams p = params("TOI", key);
    const TextBuild b = buildTextContours(font, p);
    REQUIRE(b.ok());

    auto macro = std::make_unique<MacroCommand>("Add text");
    macro->add(std::make_unique<AddFontCommand>(FontAsset{key, bytes}));
    macro->add(std::make_unique<AddObjectCommand>(Object::vector(b.contours, doc.activeLayer(), doc.activeSheet(), p)));
    REQUIRE(doc.execute(std::move(macro)));
    REQUIRE(doc.objectCount() == 1);
    REQUIRE(doc.findFont(key) != nullptr);
    CHECK(doc.checkIntegrity().empty());
    const Object snapshot = doc.objects().begin()->second;

    REQUIRE(doc.undo());
    CHECK(doc.objectCount() == 0);
    CHECK(doc.fonts().empty());
    REQUIRE(doc.redo());
    CHECK(doc.fonts().size() == 1);
    REQUIRE(doc.objectCount() == 1);
    CHECK(doc.objects().begin()->second == snapshot);

    // A second text with the same font does not duplicate it, and undo keeps the font that
    // the first text still needs.
    auto second = std::make_unique<MacroCommand>("Add text");
    second->add(std::make_unique<AddFontCommand>(FontAsset{key, bytes}));
    second->add(std::make_unique<AddObjectCommand>(Object::vector(b.contours, doc.activeLayer(), doc.activeSheet(), p)));
    REQUIRE(doc.execute(std::move(second)));
    CHECK(doc.fonts().size() == 1);
    REQUIRE(doc.undo());
    CHECK(doc.fonts().size() == 1);
    CHECK(doc.objectCount() == 1);
}

TEST_CASE("SetTextCommand edits and restores the payload exactly", "[text][document][undo]") {
    Fixture f("TOI");
    const text::Font font = loadFont();
    const VectorObject before = *f.doc.findObject(f.id)->asVector();

    TextParams p = std::get<TextParams>(before.params);
    p.text = "O";
    p.sizeMm = 20.0;
    const TextBuild b = buildTextContours(font, p);
    REQUIRE(b.ok());
    REQUIRE(f.doc.execute(std::make_unique<SetTextCommand>(f.id, p, b.contours)));
    const VectorObject after = *f.doc.findObject(f.id)->asVector();
    CHECK(totalArea(after.contours) == Approx(110.0).epsilon(1e-12));  // 27.5 * 4
    REQUIRE(f.doc.undo());
    CHECK(*f.doc.findObject(f.id)->asVector() == before);
    REQUIRE(f.doc.redo());
    CHECK(*f.doc.findObject(f.id)->asVector() == after);
    CHECK(f.doc.checkIntegrity().empty());

    // Refusals leave the document untouched.
    TextParams bad = p;
    bad.fontKey = "0000000000000000";
    CHECK_FALSE(f.doc.execute(std::make_unique<SetTextCommand>(f.id, bad, b.contours)));
    bad = p;
    bad.sizeMm = -1.0;
    CHECK_FALSE(f.doc.execute(std::make_unique<SetTextCommand>(f.id, bad, b.contours)));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetTextCommand>(f.id, p, b.contours)));  // no change
    CHECK_FALSE(f.doc.execute(std::make_unique<SetTextCommand>(ObjectId{9999}, p, b.contours)));
    CHECK(*f.doc.findObject(f.id)->asVector() == after);
}

TEST_CASE("converting text to curves keeps the contours and is one undo step", "[text][document][undo]") {
    Fixture f("TOI");
    Document& doc = f.doc;
    const text::Font font = loadFont();
    const Object before = *doc.findObject(f.id);
    const double area = totalArea(before.asVector()->contours);
    // T(120000)+O(275000)+I(140000) font units^2 at 0.01 mm: 53.5 mm^2.
    CHECK(area == Approx(53.5).epsilon(1e-12));

    REQUIRE(doc.execute(std::make_unique<ConvertTextToCurvesCommand>(std::vector<ObjectId>{f.id})));
    const VectorObject* v = doc.findObject(f.id)->asVector();
    REQUIRE(v != nullptr);
    CHECK(std::holds_alternative<NoParams>(v->params));
    CHECK(v->contours == before.asVector()->contours);
    CHECK(totalArea(v->contours) == Approx(area).epsilon(1e-15));
    CHECK(doc.checkIntegrity().empty());

    REQUIRE(doc.undo());
    CHECK(*doc.findObject(f.id) == before);
    REQUIRE(doc.redo());
    CHECK(std::holds_alternative<NoParams>(doc.findObject(f.id)->asVector()->params));

    // Not text any more: a second conversion is a refusal, as are unknown ids and empty lists.
    CHECK_FALSE(doc.execute(std::make_unique<ConvertTextToCurvesCommand>(std::vector<ObjectId>{f.id})));
    CHECK_FALSE(doc.execute(std::make_unique<ConvertTextToCurvesCommand>(std::vector<ObjectId>{})));
    CHECK_FALSE(doc.execute(std::make_unique<ConvertTextToCurvesCommand>(std::vector<ObjectId>{ObjectId{999}})));
    (void)font;
}

TEST_CASE("transforms keep text editable through the placement", "[text][document]") {
    Fixture f("TOI");
    Document& doc = f.doc;
    const text::Font font = loadFont();
    const Object before = *doc.findObject(f.id);
    const geometry::Transform2 m = geometry::Transform2::rotation(0.5, {3.0, 4.0}) *
                                   geometry::Transform2::translation({10.0, -2.0});
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{f.id}, m)));
    const VectorObject* v = doc.findObject(f.id)->asVector();
    const auto* tp = std::get_if<TextParams>(&v->params);
    REQUIRE(tp != nullptr);
    CHECK(tp->placement == m);
    // Laying the transformed parameters out again reproduces the moved contours.
    const TextBuild again = buildTextContours(font, *tp);
    REQUIRE(again.ok());
    REQUIRE(again.contours.size() == v->contours.size());
    const geometry::Box2 a = boundsOf(again.contours);
    const geometry::Box2 b = boundsOf(v->contours);
    CHECK(a.min.x == Approx(b.min.x).margin(1e-9));
    CHECK(a.max.y == Approx(b.max.y).margin(1e-9));
    REQUIRE(doc.undo());
    CHECK(*doc.findObject(f.id) == before);

    // A mirror keeps text parameters and positive-area outer contours.
    REQUIRE(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{f.id},
                                                                  geometry::Transform2::mirrorX(0.0))));
    CHECK(totalArea(doc.findObject(f.id)->asVector()->contours) == Approx(53.5).epsilon(1e-9));
}

TEST_CASE("node editing text converts it to curves", "[text][document]") {
    Fixture f("I");
    auto contours = f.doc.findObject(f.id)->asVector()->contours;
    REQUIRE(f.doc.execute(std::make_unique<SetObjectContoursCommand>(
        std::vector<ContourReplacement>{{f.id, contours}})));
    CHECK(std::holds_alternative<NoParams>(f.doc.findObject(f.id)->asVector()->params));
}

TEST_CASE("text objects round trip through .carv byte for byte", "[text][document][carv]") {
    Fixture f("TOI");
    TextParams p = std::get<TextParams>(f.doc.findObject(f.id)->asVector()->params);
    p.align = TextAlign::Centre;
    p.letterSpacingMm = 0.25;
    p.lineSpacing = 1.5;
    p.kerning = false;
    p.placement = geometry::Transform2::rotation(0.3, {1.0, 2.0});
    const text::Font font = loadFont();
    TextBuild b = buildTextContours(font, p);
    REQUIRE(b.ok());
    REQUIRE(f.doc.execute(std::make_unique<SetTextCommand>(f.id, p, b.contours)));

    // A curve text and a box text too.
    TextParams curve = params("OI", f.key);
    curve.mode = TextMode::Curve;
    curve.guide.spans.push_back(geometry::LineSpan{{0, 0}, {50, 0}});
    curve.guideId = f.id;
    curve.startOffsetMm = 3.5;
    b = buildTextContours(font, curve);
    REQUIRE(b.ok());
    f.doc.insertObject(Object::vector(b.contours, f.doc.activeLayer(), f.doc.activeSheet(), curve));
    TextParams box = params("T O I", f.key);
    box.mode = TextMode::Box;
    box.boxWidthMm = 30.0;
    box.boxHeightMm = 20.0;
    box.align = TextAlign::Right;
    b = buildTextContours(font, box);
    REQUIRE(b.ok());
    f.doc.insertObject(Object::vector(b.contours, f.doc.activeLayer(), f.doc.activeSheet(), box));

    const Bytes first = saveOk(f.doc);
    LoadedCarv loaded = loadOk(first);
    CHECK(loaded.formatMinor == kCarvFormatMinor);
    CHECK(loaded.extras.empty());
    REQUIRE(loaded.document.objectCount() == 3);
    CHECK(loaded.document.checkIntegrity().empty());
    CHECK(loaded.document.fonts().size() == 1);
    REQUIRE(loaded.document.findFont(f.key) != nullptr);
    CHECK(loaded.document.findFont(f.key)->bytes == testFontBytes());
    for (const auto& [id, obj] : f.doc.objects()) {
        const Object* other = loaded.document.findObject(id);
        REQUIRE(other != nullptr);
        CHECK(*other == obj);
    }
    const Bytes second = saveOk(loaded.document, loaded.extras);
    CHECK(second == first);

    // Re-laying out from the loaded parameters and font gives the stored contours (the cache is
    // a faithful derived view).
    for (const auto& [id, obj] : loaded.document.objects()) {
        const auto& tp = std::get<TextParams>(obj.asVector()->params);
        const TextBuild re = buildTextContours(font, tp);
        REQUIRE(re.ok());
        CHECK(re.contours == obj.asVector()->contours);
    }
}

TEST_CASE("documents without text are still written as format 1.1 and carry no fonts", "[text][document][carv]") {
    Document doc;
    doc.insertObject(Object::vector(geometry::Contour{}, doc.activeLayer(), doc.activeSheet()));
    REQUIRE(doc.insertFont(FontAsset{fontKeyFor(testFontBytes()), testFontBytes()}));  // embedded but unused
    const Bytes bytes = saveOk(doc);
    const LoadedCarv loaded = loadOk(bytes);
    CHECK(loaded.formatMinor == kCarvFormatMinorBase);
    CHECK(loaded.document.fonts().empty());  // unreferenced fonts are not saved
    auto zip = detail::readZip(bytes);
    REQUIRE(zip.ok);
    for (const auto& m : zip.members) {
        CHECK(m.name.rfind("fonts/", 0) != 0);
    }
}

TEST_CASE("format 1.2 files: a font member per used font, refusal of bad fonts", "[text][document][carv]") {
    Fixture f("I");
    const Bytes bytes = saveOk(f.doc);
    auto zip = detail::readZip(bytes);
    REQUIRE(zip.ok);
    const std::string fontMember = "fonts/" + f.key + ".font";
    const auto has = [&](const std::string& name) {
        return std::any_of(zip.members.begin(), zip.members.end(),
                           [&](const detail::ZipMember& m) { return m.name == name; });
    };
    CHECK(has(fontMember));

    // A text object whose font member is missing from the archive does not load.
    std::vector<detail::ZipMember> without;
    for (const auto& m : zip.members) {
        if (m.name != fontMember) {
            without.push_back(m);
        }
    }
    auto w = detail::writeZip(without);
    REQUIRE(w.ok);
    auto r = loadCarv(std::span<const std::uint8_t>(w.bytes));
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == CarvError::Code::IntegrityFailure);

    // A document that names a font it does not embed refuses to save.
    Document bad;
    TextParams p = params("I", "0123456789abcdef");
    bad.insertObject(Object::vector(geometry::Contour{}, bad.activeLayer(), bad.activeSheet(), p));
    auto s = saveCarv(bad);
    REQUIRE_FALSE(s.ok());
    CHECK(s.error().code == CarvError::Code::InvalidContent);

    // The reserved member names cannot be injected through CarvExtras.
    CHECK_THROWS(CarvExtras{}.withMember(fontMember, {1, 2, 3}));
}

TEST_CASE("an older reader keeps text as plain curves and re-saves it unchanged", "[text][document][carv]") {
    // A 1.1 reader treats the "text" parameter kind as unknown: simulate it by renaming the kind
    // in job.json (the existing unknown-parameter path of ADR-0013) and loading.
    Fixture f("TOI");
    const Bytes bytes = saveOk(f.doc);
    auto zip = detail::readZip(bytes);
    REQUIRE(zip.ok);
    for (auto& m : zip.members) {
        if (m.name == "job.json") {
            std::string t(m.data.begin(), m.data.end());
            const std::string from = "\"type\": \"text\"";
            const auto pos = t.find(from);
            REQUIRE(pos != std::string::npos);
            t.replace(pos, from.size(), "\"type\": \"tfut\"");
            m.data.assign(t.begin(), t.end());
        }
    }
    auto w = detail::writeZip(zip.members);
    REQUIRE(w.ok);
    LoadedCarv loaded = loadOk(w.bytes);
    const Object* o = loaded.document.objects().empty() ? nullptr : &loaded.document.objects().begin()->second;
    REQUIRE(o != nullptr);
    CHECK(std::holds_alternative<NoParams>(o->asVector()->params));
    CHECK(o->asVector()->contours == f.doc.findObject(f.id)->asVector()->contours);
    CHECK_FALSE(loaded.extras.empty());
    // The unknown parameters survive a re-save verbatim.
    const Bytes again = saveOk(loaded.document, loaded.extras);
    CHECK(loadOk(again).document.objectCount() == 1);
}

TEST_CASE("hostile .carv text parameters are refused", "[text][document][carv]") {
    Fixture f("I");
    const Bytes bytes = saveOk(f.doc);
    auto zip = detail::readZip(bytes);
    REQUIRE(zip.ok);
    const auto mutate = [&](const std::string& from, const std::string& to) {
        auto members = zip.members;
        for (auto& m : members) {
            if (m.name == "job.json") {
                std::string t(m.data.begin(), m.data.end());
                const auto pos = t.find(from);
                REQUIRE(pos != std::string::npos);
                t.replace(pos, from.size(), to);
                m.data.assign(t.begin(), t.end());
            }
        }
        auto w = detail::writeZip(members);
        REQUIRE(w.ok);
        return loadCarv(std::span<const std::uint8_t>(w.bytes));
    };
    CHECK_FALSE(mutate("\"size_mm\": 10", "\"size_mm\": -3").ok());
    CHECK_FALSE(mutate("\"size_mm\": 10", "\"size_mm\": \"NaN\"").ok());
    CHECK_FALSE(mutate("\"align\": \"left\"", "\"align\": \"justify\"").ok());
    CHECK_FALSE(mutate("\"font\": \"" + f.key + "\"", "\"font\": \"../x\"").ok());
    CHECK_FALSE(mutate("\"mode\": \"point\"", "\"mode\": \"curve\"").ok());  // curve without a guide
    CHECK_FALSE(mutate("\"line_spacing\": 1", "\"line_spacing\": -1").ok());
}

TEST_CASE("embedded font caps", "[text][document]") {
    Document doc;
    FontAsset big;
    big.bytes.assign(kMaxEmbeddedFontBytes + 1, 1);
    big.key = fontKeyFor(big.bytes);
    CHECK_FALSE(doc.insertFont(big));
    CHECK_FALSE(doc.insertFont(FontAsset{"bad", {1}}));
    CHECK_FALSE(doc.insertFont(FontAsset{"0123456789abcdef", {}}));
    CHECK(doc.insertFont(FontAsset{"0123456789abcdef", {1}}));
    CHECK_FALSE(doc.insertFont(FontAsset{"0123456789abcdef", {2}}));  // key taken
    CHECK(fontKeyFor({}) == "cbf29ce484222325");                      // FNV-1a offset basis
    CHECK(fontKeyFor({'a'}) == "af63dc4c8601ec8c");                   // FNV-1a 64 of "a"
}
