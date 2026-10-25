#include "carvetto/document/carv_io.hpp"

#include <algorithm>
#include <bit>
#include <catch2/catch_test_macros.hpp>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "carv_json_reader.hpp"
#include "carv_zip.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"

#if defined(__linux__)
#include <malloc.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

using namespace carvetto::document;
using carvetto::geometry::ArcDirection;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::BezierSpan;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Transform2;
using carvetto::geometry::Vec2;
using Bytes = std::vector<std::uint8_t>;
using Code = CarvError::Code;

namespace {

// --- Helpers ----------------------------------------------------------------------------------

Bytes save(const Document& doc, const CarvExtras& extras = {}) {
    auto r = saveCarv(doc, extras);
    INFO((r ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return r.value();
}

LoadedCarv load(const Bytes& bytes) {
    auto r = loadCarv(std::span<const std::uint8_t>(bytes));
    INFO((r ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return std::move(r.value());
}

CarvError loadError(const Bytes& bytes) {
    auto r = loadCarv(std::span<const std::uint8_t>(bytes));
    REQUIRE_FALSE(r.ok());
    return r.error();
}

std::vector<detail::ZipMember> members(const Bytes& archive) {
    auto r = detail::readZip(archive);
    INFO(r.error);
    REQUIRE(r.ok);
    return r.members;
}

std::string memberText(const Bytes& archive, const std::string& name) {
    for (const auto& m : members(archive)) {
        if (m.name == name) {
            return {m.data.begin(), m.data.end()};
        }
    }
    FAIL("member " << name << " not found");
    return {};
}

Bytes zipOf(const std::vector<std::pair<std::string, std::string>>& files) {
    std::vector<detail::ZipMember> ms;
    for (const auto& [name, text] : files) {
        ms.push_back({name, Bytes(text.begin(), text.end())});
    }
    auto r = detail::writeZip(ms);
    REQUIRE(r.ok);
    return r.bytes;
}

/// Replaces the archive member `name` with `text` (keeping member order).
Bytes withMember(const Bytes& archive, const std::string& name, const std::string& text) {
    auto ms = members(archive);
    bool found = false;
    for (auto& m : ms) {
        if (m.name == name) {
            m.data.assign(text.begin(), text.end());
            found = true;
        }
    }
    if (!found) {
        ms.push_back({name, Bytes(text.begin(), text.end())});
    }
    auto r = detail::writeZip(ms);
    REQUIRE(r.ok);
    return r.bytes;
}

/// `s` with the first occurrence of `from` replaced; the test fails if `from` is absent.
std::string replaced(std::string s, const std::string& from, const std::string& to) {
    const auto pos = s.find(from);
    INFO("looking for: " << from);
    REQUIRE(pos != std::string::npos);
    s.replace(pos, from.size(), to);
    return s;
}

bool contains(const std::string& haystack, const std::string& needle) {
    return haystack.find(needle) != std::string::npos;
}

/// Everything persisted in a `.carv` file (selection and undo history are not).
void requireSameDocument(const Document& a, const Document& b) {
    CHECK(a.job() == b.job());
    CHECK(a.sheets() == b.sheets());
    CHECK(a.layers() == b.layers());
    CHECK(a.objects() == b.objects());
    CHECK(a.variables().all() == b.variables().all());
    CHECK(a.activeLayer() == b.activeLayer());
    CHECK(a.activeSheet() == b.activeSheet());
    CHECK(a.idGenerator().peek() == b.idGenerator().peek());
}

void requireIntact(const Document& doc) {
    const auto issues = doc.checkIntegrity();
    std::string text;
    for (const auto& i : issues) {
        text += i.message + "; ";
    }
    INFO(text);
    REQUIRE(issues.empty());
}

/// save -> load -> save: the two archives are byte-identical and the loaded document equals `doc`.
void requireRoundTrip(const Document& doc) {
    const Bytes first = save(doc);
    LoadedCarv loaded = load(first);
    requireIntact(loaded.document);
    requireSameDocument(doc, loaded.document);
    const Bytes second = save(loaded.document, loaded.extras);
    REQUIRE(first == second);
}

LayerId addLayer(Document& doc, const std::string& name, Colour colour) {
    auto cmd = std::make_unique<AddLayerCommand>(Layer{.name = name, .colour = colour});
    const AddLayerCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

ObjectId addObject(Document& doc, Object obj) {
    auto cmd = std::make_unique<AddObjectCommand>(std::move(obj));
    const AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

ObjectId group(Document& doc, std::vector<ObjectId> ids) {
    auto cmd = std::make_unique<GroupObjectsCommand>(std::move(ids));
    const GroupObjectsCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->groupId();
}

CreationParams paramsOfKind(int k, double s) {
    switch (k % 6) {
        case 0:
            return NoParams{};
        case 1:
            return RectangleParams{Vec2{s, -s}, 10.0 + s, 5.25, RectangleParams::CornerStyle::Rounded, 1.5};
        case 2:
            return CircleParams{Vec2{s, 0.1}, 3.0 + s / 7.0};
        case 3:
            return EllipseParams{Vec2{-s, s}, 8.0, 4.0 / 3.0, 0.3};
        case 4:
            return PolygonParams{Vec2{s, s}, 7, 12.5, -0.25};
        default:
            return StarParams{Vec2{0.0, s}, 9, 20.0, 1.0 / 3.0, 1e-7};
    }
}

/// A vector contour mixing all three span kinds.
Contour mixedContour(double s) {
    Contour c;
    const Vec2 a{s, 0.1 * s};
    const Vec2 b{s + 10.0, 0.1 * s};
    c.append(LineSpan{a, b});
    const ArcSpan arc = ArcSpan::fromCentre(Vec2{s + 10.0, 0.1 * s + 5.0}, b, Vec2{s + 15.0, 0.1 * s + 5.0},
                                            ArcDirection::CCW);
    c.append(arc);
    c.append(BezierSpan{arc.endPoint(), Vec2{s + 12.0, 20.0 / 3.0}, Vec2{s + 3.0, std::sqrt(2.0)}, a});
    c.closed = true;
    return c;
}

/// The acceptance-criterion-1 document: 3 layers, 50 vector objects with lines, arcs and Beziers,
/// every creation-parameter kind, and document variables.
Document makeAcceptanceDocument() {
    Document doc(Job::singleSided(24.0, 18.0, 0.75, Units::Inches, Datum::Centre));
    std::vector<LayerId> layers{doc.defaultLayer(), addLayer(doc, "Profile", Colour{255, 0, 16}),
                                addLayer(doc, "Pocket \"deep\" \xC3\xA9", Colour{1, 2, 3})};
    for (int i = 0; i < 50; ++i) {
        const double s = 0.1 * i + 1.0 / 3.0;
        std::vector<Contour> contours{mixedContour(s)};
        if (i % 4 == 0) {
            contours.push_back(Contour::circle(Vec2{s, s}, 2.5 + s, ArcDirection::CW));
        }
        if (i % 5 == 0) {
            Contour open;
            open.append(LineSpan{Vec2{0, 0}, Vec2{s, 1e-300}});
            contours.push_back(open);
        }
        Object obj = Object::vector(std::move(contours), layers[static_cast<std::size_t>(i) % 3],
                                    doc.activeSheet(), paramsOfKind(i, s));
        obj.name = "Object " + std::to_string(i);
        addObject(doc, std::move(obj));
    }
    REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Depth", 12.7)));
    REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Width", 0.1 + 0.2)));
    REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("tab_h", -1e-12)));
    return doc;
}

// Minimal hand-written files for error tests. OBJECTS / NEXT are substituted.
const std::string kManifest =
    R"({"app_version": "test", "format": "carvetto", "format_version": {"major": 1, "minor": 0}})";
const std::string kJobTemplate = R"({
  "active_layer": 2, "active_sheet": 1,
  "job": {"type": "single_sided", "size_mm": [300.0, 200.0, 18.0], "display_units": "mm",
          "z_zero": "material_surface", "datum": "bottom_left", "datum_offset_mm": [0.0, 0.0],
          "flip_direction": "horizontal", "zero_off_same_side": false, "diameter_mm": 0.0,
          "rotary_axis": "x", "flip_design": false},
  "layers": [{"id": 2, "name": "Layer 1", "colour": "#000000", "visible": true, "locked": false},
             {"id": 3, "name": "Cut", "colour": "#FF8000", "visible": false, "locked": true}],
  "next_id": NEXT,
  "objects": OBJECTS,
  "sheets": [{"id": 1, "name": "Sheet 1", "size_mm": [300, 200, 18], "z_zero": "material_surface",
              "datum": "bottom_left", "datum_offset_mm": [0, 0], "visible": true}],
  "variables": {"Depth": 3}
})";

std::string vec(std::uint64_t id, const std::string& extra = "") {
    return R"({"id": )" + std::to_string(id) +
           R"(, "layer": 2, "sheet": 1, "name": "v", "type": "vector", "contours": [{"closed": false, )"
           R"("spans": [{"type": "line", "start": [0, 0], "end": [1, 2]}]}], "params": {"type": "none"})" +
           extra + "}";
}

std::string jobWith(const std::string& objects, const std::string& next = "100") {
    return replaced(replaced(kJobTemplate, "OBJECTS", objects), "NEXT", next);
}

Bytes fileWith(const std::string& objects, const std::string& next = "100") {
    return zipOf({{"manifest.json", kManifest}, {"job.json", jobWith(objects, next)}});
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// Acceptance criteria
// ------------------------------------------------------------------------------------------------

TEST_CASE("AC1: save -> load -> save gives byte-identical job.json and archive", "[document][carv]") {
    const Document doc = makeAcceptanceDocument();
    REQUIRE(doc.layers().size() == 3);
    REQUIRE(doc.objectCount() == 50);

    const Bytes first = save(doc);
    LoadedCarv loaded = load(first);
    requireIntact(loaded.document);
    requireSameDocument(doc, loaded.document);
    CHECK(loaded.extras.empty());
    CHECK(loaded.formatMajor == kCarvFormatMajor);
    CHECK(loaded.formatMinor == kCarvFormatMinor);
    const Bytes second = save(loaded.document, loaded.extras);

    CHECK(memberText(first, "job.json") == memberText(second, "job.json"));
    CHECK(memberText(first, "manifest.json") == memberText(second, "manifest.json"));
    CHECK(first == second);  // the whole archive, not just job.json

    // And a third generation from a fresh load, for good measure.
    LoadedCarv again = load(second);
    CHECK(save(again.document, again.extras) == first);
}

TEST_CASE("AC2: an unknown top-level JSON field is kept on re-save", "[document][carv]") {
    const Document doc = makeAcceptanceDocument();
    const Bytes original = save(doc);
    const std::string job = memberText(original, "job.json");
    const std::string field = R"("zz_future_feature": {"enabled": true, "list": [1, 2.5, "x"]})";
    const Bytes edited = withMember(original, "job.json", replaced(job, "{\n", "{\n  " + field + ",\n"));

    LoadedCarv loaded = load(edited);
    requireSameDocument(doc, loaded.document);
    CHECK_FALSE(loaded.extras.empty());

    const Bytes resaved = save(loaded.document, loaded.extras);
    const std::string resavedJob = memberText(resaved, "job.json");
    CHECK(contains(resavedJob, "\"zz_future_feature\": {"));
    CHECK(contains(resavedJob, "\"enabled\": true"));
    CHECK(contains(resavedJob, "\"list\": [1, 2.5, \"x\"]"));
    // Keys stay sorted: the unknown field comes after every known root key.
    CHECK(resavedJob.find("\"zz_future_feature\"") > resavedJob.find("\"variables\""));

    // Stable from here on.
    LoadedCarv again = load(resaved);
    CHECK(save(again.document, again.extras) == resaved);

    // Without the extras the field is dropped (it is not part of the Document).
    CHECK_FALSE(contains(memberText(save(loaded.document), "job.json"), "zz_future_feature"));
}

TEST_CASE("AC3: a file with a higher major version fails with a clear error", "[document][carv]") {
    const Bytes original = save(Document{});
    const std::string manifest = memberText(original, "manifest.json");

    SECTION("major 2 is rejected as too new") {
        const Bytes newer =
            withMember(original, "manifest.json", replaced(manifest, "\"major\": 1", "\"major\": 2"));
        const CarvError err = loadError(newer);
        CHECK(err.code == Code::UnsupportedVersion);
        CHECK(contains(err.message, "2.0"));
        CHECK(contains(err.message, "newer"));
    }
    SECTION("the version is checked before job.json is read") {
        const Bytes newer = withMember(
            withMember(original, "manifest.json", replaced(manifest, "\"major\": 1", "\"major\": 99")),
            "job.json", R"({"completely": "different schema"})");
        CHECK(loadError(newer).code == Code::UnsupportedVersion);
    }
    SECTION("major 0 is not supported either") {
        const Bytes older =
            withMember(original, "manifest.json", replaced(manifest, "\"major\": 1", "\"major\": 0"));
        CHECK(loadError(older).code == Code::UnsupportedVersion);
    }
    SECTION("a higher minor version of the same major loads") {
        const Bytes minor =
            withMember(original, "manifest.json", replaced(manifest, "\"minor\": 0", "\"minor\": 7"));
        LoadedCarv loaded = load(minor);
        CHECK(loaded.formatMinor == 7);
        // Re-saved with this build's version.
        CHECK(contains(memberText(save(loaded.document, loaded.extras), "manifest.json"), "\"minor\": 0"));
    }
}

// ------------------------------------------------------------------------------------------------
// Forward compatibility
// ------------------------------------------------------------------------------------------------

TEST_CASE("unknown fields on every record and unknown archive members are preserved", "[document][carv]") {
    Document doc;
    Object obj = Object::vector(Contour::rectangle(Box2{Vec2{0, 0}, Vec2{4, 3}}), doc.defaultLayer(),
                                doc.activeSheet());
    obj.name = "keep-me";
    const ObjectId kept = addObject(doc, std::move(obj));
    Object other = Object::vector(Contour::circle(Vec2{9, 9}, 1), doc.defaultLayer(), doc.activeSheet());
    other.name = "drop-me";
    const ObjectId dropped = addObject(doc, std::move(other));

    const Bytes original = save(doc);
    std::string job = memberText(original, "job.json");
    job = replaced(job, "\"name\": \"keep-me\"", "\"name\": \"keep-me\", \"zz_object\": \"o\"");
    job = replaced(job, "\"name\": \"drop-me\"", "\"name\": \"drop-me\", \"zz_object2\": 2");
    job = replaced(job, "\"name\": \"Layer 1\"", "\"name\": \"Layer 1\", \"zz_layer\": [true, null]");
    job = replaced(job, "\"name\": \"Sheet 1\"", "\"name\": \"Sheet 1\", \"zz_sheet\": {\"k\": -0.5}");
    job = replaced(job, "\"flip_design\": false", "\"flip_design\": false, \"zz_job\": 42");
    std::string manifest = memberText(original, "manifest.json");
    manifest =
        replaced(manifest, "\"format\": \"carvetto\"", "\"format\": \"carvetto\", \"zz_manifest\": \"m\"");
    const std::string blob("\x00\x01\x02\xff binary", 11);
    Bytes edited = withMember(withMember(original, "job.json", job), "manifest.json", manifest);
    edited = withMember(edited, "toolpaths/0001.bin", blob);
    edited = withMember(edited, "bitmaps/a.png", "");

    LoadedCarv loaded = load(edited);
    requireSameDocument(doc, loaded.document);
    CHECK(loaded.extras.memberNames() == std::vector<std::string>{"bitmaps/a.png", "toolpaths/0001.bin"});

    const Bytes resaved = save(loaded.document, loaded.extras);
    const std::string resavedJob = memberText(resaved, "job.json");
    CHECK(contains(resavedJob, "\"zz_object\": \"o\""));
    CHECK(contains(resavedJob, "\"zz_object2\": 2"));
    CHECK(contains(resavedJob, "\"zz_layer\": [true, null]"));
    CHECK(contains(resavedJob, "\"zz_sheet\": {"));
    CHECK(contains(resavedJob, "\"k\": -0.5"));
    CHECK(contains(resavedJob, "\"zz_job\": 42"));
    CHECK(contains(memberText(resaved, "manifest.json"), "\"zz_manifest\": \"m\""));
    CHECK(memberText(resaved, "toolpaths/0001.bin") == blob);
    CHECK(memberText(resaved, "bitmaps/a.png").empty());
    // Member order: manifest, job, then preserved members sorted by name.
    const auto ms = members(resaved);
    REQUIRE(ms.size() == 4);
    CHECK(ms[0].name == "manifest.json");
    CHECK(ms[1].name == "job.json");
    CHECK(ms[2].name == "bitmaps/a.png");
    CHECK(ms[3].name == "toolpaths/0001.bin");
    LoadedCarv again = load(resaved);
    CHECK(save(again.document, again.extras) == resaved);

    // Extras follow their entity: deleting the object drops its unknown fields only.
    REQUIRE(loaded.document.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{dropped})));
    const std::string afterDelete = memberText(save(loaded.document, loaded.extras), "job.json");
    CHECK_FALSE(contains(afterDelete, "zz_object2"));
    CHECK(contains(afterDelete, "\"zz_object\": \"o\""));
    CHECK(loaded.document.hasObject(kept));
}

TEST_CASE("creation parameters of an unknown kind (e.g. text) are kept verbatim", "[document][carv]") {
    Document doc;
    const ObjectId id =
        addObject(doc, Object::vector(mixedContour(1.0), doc.defaultLayer(), doc.activeSheet()));
    const Bytes original = save(doc);
    const std::string job = replaced(memberText(original, "job.json"), "\"type\": \"none\"",
                                     R"("type": "text", "text": "Hello", "font": "Sans", "height_mm": 12.5)");
    LoadedCarv loaded = load(withMember(original, "job.json", job));
    const auto* v = loaded.document.findObject(id)->asVector();
    REQUIRE(v);
    CHECK(std::holds_alternative<NoParams>(v->params));  // modelled as plain geometry
    const Bytes resaved = save(loaded.document, loaded.extras);
    const std::string resavedJob = memberText(resaved, "job.json");
    CHECK(contains(resavedJob, "\"type\": \"text\""));
    CHECK(contains(resavedJob, "\"text\": \"Hello\""));
    CHECK(contains(resavedJob, "\"height_mm\": 12.5"));
    LoadedCarv again = load(resaved);
    CHECK(save(again.document, again.extras) == resaved);
}

// ------------------------------------------------------------------------------------------------
// Round trips
// ------------------------------------------------------------------------------------------------

TEST_CASE("an empty document round-trips", "[document][carv]") {
    const Document doc;
    requireRoundTrip(doc);
    const std::string job = memberText(save(doc), "job.json");
    CHECK(contains(job, "\"objects\": []"));
    CHECK(contains(job, "\"variables\": {}"));
}

TEST_CASE("every job type, span, object and creation-parameter kind round-trips", "[document][carv]") {
    for (const Job& job : {Job::singleSided(300, 200, 18, Units::Millimetres),
                           Job::doubleSided(12, 8, 0.5, Units::Inches, Datum::TopRight, ZZero::MachineBed,
                                            FlipDirection::Vertical),
                           Job::rotary(400, 50, Units::Millimetres, RotaryAxis::Y)}) {
        Document doc(job);
        Job edited = job;
        edited.datumOffsetMm = Vec2{-3.25, 1e-9};
        edited.zeroOffSameSide = true;
        edited.flipDesign = true;
        REQUIRE(doc.execute(std::make_unique<SetJobCommand>(edited)));
        const LayerId hidden = addLayer(doc, "Hidden", Colour{0xAB, 0xCD, 0xEF});
        Layer h = *doc.findLayer(hidden);
        h.visible = false;
        h.locked = true;
        REQUIRE(doc.execute(std::make_unique<SetLayerPropertiesCommand>(h)));
        Sheet sheet2 = Sheet::fromJob(job, "Second");
        sheet2.datum = Datum::TopLeft;
        sheet2.zZero = ZZero::CylinderAxis;
        sheet2.visible = false;
        auto addSheet = std::make_unique<AddSheetCommand>(sheet2, true);
        const AddSheetCommand* rawSheet = addSheet.get();
        REQUIRE(doc.execute(std::move(addSheet)));
        doc.setActiveLayer(hidden);

        for (int k = 0; k < 6; ++k) {
            Contour c;
            c.append(LineSpan{Vec2{0, 0}, Vec2{static_cast<double>(k), 1}});
            c.append(ArcSpan{Vec2{static_cast<double>(k), 0}, 1.0, 0.5, 2.5, ArcDirection::CW});
            c.append(ArcSpan::fullCircle(Vec2{1, 1}, 0.75, ArcDirection::CCW));
            c.append(BezierSpan{Vec2{0, 0}, Vec2{1, 2}, Vec2{3, 4}, Vec2{5, 6}});
            addObject(doc, Object::vector(c, k % 2 ? hidden : doc.defaultLayer(), rawSheet->id(),
                                          paramsOfKind(k, 1.0 + k)));
        }
        addObject(doc, Object::vector(std::vector<Contour>{}, hidden, rawSheet->id()));  // no contours
        addObject(doc, Object::vector(Contour{}, hidden, rawSheet->id()));               // empty contour
        Object bitmap;
        bitmap.layer = hidden;
        bitmap.sheet = doc.sheets().front().id;
        bitmap.name = "photo";
        bitmap.data = BitmapObject{640, 480, Box2{Vec2{1, 2}, Vec2{65, 50}}, "bitmaps/photo.png"};
        addObject(doc, bitmap);
        Object emptyBitmap = bitmap;
        emptyBitmap.data = BitmapObject{};  // default (empty, infinite) placement box
        addObject(doc, emptyBitmap);
        REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Huge", 1.7976931348623157e308)));
        REQUIRE(doc.execute(
            std::make_unique<SetDocumentVariableCommand>("Inf", std::numeric_limits<double>::infinity())));
        REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("Tiny", 4.9e-324)));

        requireIntact(doc);
        requireRoundTrip(doc);
    }
}

TEST_CASE("doubles round-trip bit for bit", "[document][carv]") {
    std::mt19937_64 rng(20261024);
    std::vector<double> values{0.0,
                               -0.0,
                               0.1,
                               1.0 / 3.0,
                               1e22,
                               1e-7,
                               123456789.0,
                               2.5e-308,
                               4.9e-324,
                               -1.7976931348623157e308,
                               9007199254740993.0,
                               100.0};
    while (values.size() < 4000) {
        const double d = std::bit_cast<double>(rng());
        if (std::isfinite(d)) {
            values.push_back(d);
        }
    }
    Document doc;
    Contour c;
    for (std::size_t i = 0; i + 4 <= values.size(); i += 4) {
        c.append(
            BezierSpan{Vec2{values[i], values[i + 1]}, Vec2{values[i + 2], values[i + 3]}, Vec2{}, Vec2{}});
    }
    const ObjectId id = addObject(doc, Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
    REQUIRE(doc.execute(std::make_unique<SetDocumentVariableCommand>("NaNVar", std::nan(""))));

    const Bytes first = save(doc);
    LoadedCarv loaded = load(first);
    const auto& spans = loaded.document.findObject(id)->asVector()->contours.front().spans;
    REQUIRE(spans.size() == values.size() / 4);
    for (std::size_t i = 0; i < spans.size(); ++i) {
        const auto& b = std::get<BezierSpan>(spans[i]);
        INFO("span " << i);
        CHECK(std::bit_cast<std::uint64_t>(b.p0.x) == std::bit_cast<std::uint64_t>(values[4 * i]));
        CHECK(std::bit_cast<std::uint64_t>(b.p0.y) == std::bit_cast<std::uint64_t>(values[4 * i + 1]));
        CHECK(std::bit_cast<std::uint64_t>(b.p1.x) == std::bit_cast<std::uint64_t>(values[4 * i + 2]));
        CHECK(std::bit_cast<std::uint64_t>(b.p1.y) == std::bit_cast<std::uint64_t>(values[4 * i + 3]));
    }
    CHECK(std::signbit(std::get<BezierSpan>(spans[0]).p0.y));  // -0.0 keeps its sign
    CHECK(std::isnan(*loaded.document.variables().get("NaNVar")));
    CHECK(save(loaded.document, loaded.extras) == first);

    const std::string job = memberText(first, "job.json");
    CHECK(contains(job, "[0.0, -0.0]"));
    CHECK(contains(job, "0.1"));
    CHECK(contains(job, "\"NaNVar\": \"NaN\""));
}

TEST_CASE("groups and nested groups round-trip", "[document][carv]") {
    Document doc;
    const LayerId l0 = doc.defaultLayer();
    const LayerId l1 = addLayer(doc, "Other", Colour{9, 9, 9});
    std::vector<ObjectId> ids;
    for (int i = 0; i < 6; ++i) {
        ids.push_back(
            addObject(doc, Object::vector(Contour::rectangle(Box2{Vec2{i * 10.0, 0}, Vec2{i * 10.0 + 5, 5}}),
                                          i % 2 ? l1 : l0, doc.activeSheet())));
    }
    const ObjectId inner = group(doc, {ids[0], ids[1]});
    const ObjectId middle = group(doc, {inner, ids[2]});
    const ObjectId outer = group(doc, {middle, ids[3], ids[4]});
    const ObjectId solo = group(doc, {ids[5]});  // one-child group
    requireIntact(doc);
    REQUIRE(doc.findObject(ids[0])->parent == inner);
    REQUIRE(doc.findObject(middle)->parent == outer);

    requireRoundTrip(doc);
    LoadedCarv loaded = load(save(doc));
    CHECK(loaded.document.findObject(outer)->asGroup()->children ==
          std::vector<ObjectId>{middle, ids[3], ids[4]});
    CHECK(loaded.document.findObject(solo)->asGroup()->children == std::vector<ObjectId>{ids[5]});
    // The loaded document is fully editable: ungrouping works and stays consistent.
    REQUIRE(loaded.document.execute(std::make_unique<UngroupCommand>(middle)));
    requireIntact(loaded.document);
    CHECK(loaded.document.findObject(outer)->asGroup()->children ==
          std::vector<ObjectId>{inner, ids[2], ids[3], ids[4]});
}

TEST_CASE("the id high-water mark, active layer/sheet and a clean history are restored", "[document][carv]") {
    Document doc;
    const LayerId extra = addLayer(doc, "Extra", Colour{});
    std::vector<ObjectId> ids;
    for (int i = 0; i < 5; ++i) {
        ids.push_back(
            addObject(doc, Object::vector(Contour::circle(Vec2{}, 1.0 + i), extra, doc.activeSheet())));
    }
    REQUIRE(doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{ids[3], ids[4]})));
    REQUIRE(doc.execute(std::make_unique<DeleteLayerCommand>(doc.defaultLayer(), false)));  // drops id 2
    doc.setActiveLayer(extra);
    doc.selection().add(ids[0]);
    const std::uint64_t next = doc.idGenerator().peek();
    REQUIRE(next > ids[2].value + 1);  // removed objects consumed ids beyond the largest survivor

    LoadedCarv loaded = load(save(doc));
    Document& d = loaded.document;
    requireSameDocument(doc, d);
    CHECK(d.idGenerator().peek() == next);
    CHECK(d.selection().empty());
    CHECK_FALSE(d.undoStack().canUndo());
    CHECK(d.undoStack().isClean());
    CHECK(d.layers().size() == 1);
    const ObjectId fresh = addObject(d, Object::vector(Contour::circle(Vec2{}, 9), extra, d.activeSheet()));
    CHECK(fresh.value == next);
}

namespace {

/// Random edits through the public commands (a lighter variant of the integrity fuzzer).
class EditFuzzer {
   public:
    explicit EditFuzzer(std::uint32_t seed) : rng_(seed) {}

    std::size_t pick(std::size_t n) { return n == 0 ? 0 : static_cast<std::size_t>(rng_() % n); }
    bool chance(std::uint32_t percent) { return rng_() % 100 < percent; }
    template <class T>
    T pickFrom(const std::vector<T>& v) {
        return v[pick(v.size())];
    }
    std::vector<ObjectId> some(const Document& doc, std::size_t max) {
        const std::vector<ObjectId> all = doc.objectIds();
        std::vector<ObjectId> out;
        for (std::size_t i = 0, n = 1 + pick(max); i < n && !all.empty(); ++i) {
            out.push_back(pickFrom(all));
        }
        return out;
    }

    void step(Document& doc) {
        std::vector<LayerId> layers;
        for (const Layer& l : doc.layers()) layers.push_back(l.id);
        std::vector<SheetId> sheets;
        for (const Sheet& s : doc.sheets()) sheets.push_back(s.id);
        std::vector<ObjectId> groups;
        for (const auto& [id, obj] : doc.objects()) {
            if (obj.isGroup()) groups.push_back(id);
        }
        const double x = static_cast<double>(pick(1000)) / 7.0;
        switch (pick(18)) {
            case 0:
            case 1:
            case 2: {
                Object obj = Object::vector(mixedContour(x), pickFrom(layers), pickFrom(sheets),
                                            paramsOfKind(static_cast<int>(pick(6)), x));
                obj.name = "o" + std::to_string(pick(100));
                if (!groups.empty() && chance(30)) obj.parent = pickFrom(groups);
                doc.execute(std::make_unique<AddObjectCommand>(std::move(obj)));
                break;
            }
            case 3:
                doc.execute(std::make_unique<RemoveObjectsCommand>(some(doc, 3)));
                break;
            case 4:
            case 5:
                doc.execute(std::make_unique<GroupObjectsCommand>(some(doc, 4)));
                break;
            case 6:
                if (!groups.empty())
                    doc.execute(std::make_unique<UngroupCommand>(pickFrom(groups), chance(50)));
                break;
            case 7:
                doc.execute(std::make_unique<MoveObjectsToLayerCommand>(some(doc, 3), pickFrom(layers)));
                break;
            case 8:
                doc.execute(std::make_unique<MoveObjectsToSheetCommand>(some(doc, 3), pickFrom(sheets)));
                break;
            case 9:
                doc.execute(std::make_unique<TransformObjectsCommand>(
                    some(doc, 3),
                    chance(50) ? Transform2::translation({x, 1.0 / 3.0}) : Transform2::scaling(1.5, 0.7)));
                break;
            case 10:
                if (!doc.objectIds().empty()) {
                    doc.execute(
                        std::make_unique<PasteObjectsCommand>(doc.copyObjects(some(doc, 3)), Vec2{1, 1}));
                }
                break;
            case 11:
                doc.execute(std::make_unique<AddLayerCommand>(
                    Layer{.name = "L", .colour = Colour{static_cast<std::uint8_t>(pick(256)), 7, 9}},
                    chance(50), std::optional<std::size_t>(pick(layers.size() + 1))));
                break;
            case 12:
                doc.execute(std::make_unique<DeleteLayerCommand>(pickFrom(layers), chance(50)));
                break;
            case 13:
                doc.execute(std::make_unique<MoveLayerCommand>(pickFrom(layers), pick(layers.size())));
                break;
            case 14:
                doc.execute(std::make_unique<AddSheetCommand>(Sheet{.name = "S"}, chance(50)));
                break;
            case 15:
                doc.execute(std::make_unique<DeleteSheetCommand>(pickFrom(sheets)));
                break;
            case 16:
                doc.execute(std::make_unique<SetDocumentVariableCommand>("v" + std::to_string(pick(5)), x));
                doc.setActiveLayer(pickFrom(layers));
                doc.setActiveSheet(pickFrom(sheets));
                break;
            default:
                if (chance(50)) {
                    doc.undo();
                } else {
                    doc.redo();
                }
                break;
        }
        doc.undoStack().breakCoalescing();
    }

   private:
    std::mt19937 rng_;
};

}  // namespace

TEST_CASE("documents after randomized command sequences round-trip", "[document][carv][fuzz]") {
    for (std::uint32_t seed = 1; seed <= 40; ++seed) {
        Document doc;
        EditFuzzer fuzz(seed);
        for (int i = 0; i < 80; ++i) {
            fuzz.step(doc);
        }
        INFO("seed " << seed);
        requireIntact(doc);
        requireRoundTrip(doc);
    }
}

TEST_CASE("output is deterministic with zeroed zip timestamps", "[document][carv]") {
    const Bytes a = save(makeAcceptanceDocument());
    const Bytes b = save(makeAcceptanceDocument());
    CHECK(a == b);
    // First local file header: signature PK\3\4, then DOS time (offset 10) and date (offset 12).
    REQUIRE(a.size() > 30);
    CHECK(a[0] == 'P');
    CHECK(a[1] == 'K');
    CHECK(a[2] == 3);
    CHECK(a[3] == 4);
    CHECK(a[10] == 0);
    CHECK(a[11] == 0);
    CHECK(a[12] == 0);
    CHECK(a[13] == 0);
    const auto ms = members(a);
    REQUIRE(ms.size() == 2);
    CHECK(ms[0].name == "manifest.json");
    CHECK(ms[1].name == "job.json");
}

TEST_CASE("saving to and loading from a file", "[document][carv]") {
    const auto dir = std::filesystem::temp_directory_path() / "carvetto_carv_io_test";
    std::filesystem::create_directories(dir);
    const auto path = dir / "design.carv";
    const Document doc = makeAcceptanceDocument();

    const CarvStatus status = saveCarv(doc, path);
    INFO((status ? std::string() : status.error().message));
    REQUIRE(status.ok());
    CHECK_FALSE(std::filesystem::exists(dir / "design.carv.tmp"));
    auto loaded = loadCarv(path);
    REQUIRE(loaded.ok());
    requireSameDocument(doc, loaded.value().document);
    // Overwriting an existing file works too.
    REQUIRE(saveCarv(loaded.value().document, path, loaded.value().extras).ok());

    const auto missing = loadCarv(dir / "does-not-exist.carv");
    REQUIRE_FALSE(missing.ok());
    CHECK(missing.error().code == Code::Io);
    const CarvStatus badDir = saveCarv(doc, dir / "no-such-dir" / "x.carv");
    REQUIRE_FALSE(badDir.ok());
    CHECK(badDir.error().code == Code::Io);
    std::filesystem::remove_all(dir);
}

// ------------------------------------------------------------------------------------------------
// Errors: each defect gives a clear error, never a crash
// ------------------------------------------------------------------------------------------------

TEST_CASE("the hand-written minimal file loads", "[document][carv]") {
    LoadedCarv loaded = load(fileWith("[" + vec(10) + "]"));
    const Document& d = loaded.document;
    REQUIRE(d.layers().size() == 2);
    CHECK(d.layers()[1].colour == Colour{0xFF, 0x80, 0x00});
    CHECK(d.layers()[1].locked);
    CHECK(d.sheets().size() == 1);
    CHECK(d.hasObject(ObjectId{10}));
    CHECK(d.idGenerator().peek() == 100);
    CHECK(*d.variables().get("Depth") == 3.0);
    requireIntact(d);
}

TEST_CASE("archive-level defects are reported", "[document][carv][errors]") {
    const Bytes good = save(makeAcceptanceDocument());
    CHECK(loadError(Bytes{}).code == Code::InvalidArchive);
    CHECK(loadError(Bytes{'n', 'o', 't', ' ', 'a', ' ', 'z', 'i', 'p'}).code == Code::InvalidArchive);
    for (const std::size_t keep : {std::size_t{4}, std::size_t{30}, good.size() / 2, good.size() - 1}) {
        INFO("truncated to " << keep);
        CHECK(loadError(Bytes(good.begin(), good.begin() + static_cast<std::ptrdiff_t>(keep))).code ==
              Code::InvalidArchive);
    }
    // A corrupted byte inside job.json's compressed data fails its CRC / inflate check.
    Bytes corrupt = good;
    corrupt[corrupt.size() / 2] ^= 0x5A;
    CHECK(loadError(corrupt).code == Code::InvalidArchive);

    const CarvError noManifest = loadError(zipOf({{"job.json", jobWith("[]")}}));
    CHECK(noManifest.code == Code::MissingMember);
    CHECK(contains(noManifest.message, "manifest.json"));
    const CarvError noJob = loadError(zipOf({{"manifest.json", kManifest}}));
    CHECK(noJob.code == Code::MissingMember);
    CHECK(contains(noJob.message, "job.json"));
}

TEST_CASE("JSON-level defects are reported with the member and item", "[document][carv][errors]") {
    struct Case {
        const char* label;
        Bytes file;
        Code code;
        const char* mention;
    };
    const std::string good = jobWith("[" + vec(10) + "]");
    auto job = [](const std::string& text) {
        return zipOf({{"manifest.json", kManifest}, {"job.json", text}});
    };
    std::string deep(200, '[');
    deep += std::string(200, ']');
    const std::vector<Case> cases{
        {"malformed job.json", job(good.substr(0, good.size() / 2)), Code::InvalidJson, "job.json"},
        {"malformed manifest", zipOf({{"manifest.json", "{\"format\": "}, {"job.json", good}}),
         Code::InvalidJson, "manifest.json"},
        {"not an object", job("[1, 2]"), Code::InvalidContent, "job.json"},
        {"too deep", job(R"({"zz": )" + deep + "}"), Code::ResourceLimit, "deeper"},
        {"wrong format",
         zipOf({{"manifest.json", replaced(kManifest, "\"carvetto\"", "\"other\"")}, {"job.json", good}}),
         Code::InvalidContent, "format"},
        {"missing field", job(replaced(good, "\"active_sheet\": 1,", "")), Code::InvalidContent,
         "active_sheet"},
        {"wrong type",
         job(replaced(good, "\"visible\": true, \"locked\": false",
                      "\"visible\": \"yes\", \"locked\": false")),
         Code::InvalidContent, "visible"},
        {"bad colour", job(replaced(good, "#FF8000", "red")), Code::InvalidContent, "colour"},
        {"unknown enum", job(replaced(good, "\"single_sided\"", "\"hexagonal\"")), Code::InvalidContent,
         "hexagonal"},
        {"unknown span", job(replaced(good, "\"type\": \"line\"", "\"type\": \"nurbs\"")),
         Code::InvalidContent, "nurbs"},
        {"unknown object type", job(replaced(good, "\"type\": \"vector\"", "\"type\": \"relief\"")),
         Code::InvalidContent, "relief"},
        {"bad point", job(replaced(good, "\"end\": [1, 2]", "\"end\": [1]")), Code::InvalidContent, "end"},
        {"no sheets", job(replaced(good, "\"sheets\": [", "\"sheets\": [], \"zz\": [")), Code::InvalidContent,
         "sheet"},
        {"zero id", fileWith("[" + vec(0) + "]"), Code::InvalidContent, "id"},
        {"negative id", fileWith("[" + vec(10) + "]", "-5"), Code::InvalidContent, "next_id"},
        {"float id", fileWith("[" + vec(10) + "]", "100.5"), Code::InvalidContent, "next_id"},
        {"duplicate object id", fileWith("[" + vec(10) + ", " + vec(10) + "]"), Code::InvalidContent,
         "duplicate id 10"},
        {"object id equals a layer id", fileWith("[" + vec(3) + "]"), Code::InvalidContent, "duplicate id 3"},
        {"next_id too small", fileWith("[" + vec(10) + "]", "10"), Code::InvalidContent, "next_id"},
        {"missing layer", fileWith(replaced("[" + vec(10) + "]", "\"layer\": 2", "\"layer\": 77")),
         Code::InvalidContent, "layer 77"},
        {"missing sheet", fileWith(replaced("[" + vec(10) + "]", "\"sheet\": 1", "\"sheet\": 77")),
         Code::InvalidContent, "sheet 77"},
        {"bad active layer", job(replaced(good, "\"active_layer\": 2", "\"active_layer\": 10")),
         Code::InvalidContent, "active_layer"},
        {"bad variable name", job(replaced(good, "\"Depth\"", "\"9lives\"")), Code::InvalidContent, "9lives"},
        {"bad variable value", job(replaced(good, "\"Depth\": 3", "\"Depth\": \"deep\"")),
         Code::InvalidContent, "Depth"},
        {"polygon sides out of range",
         fileWith(
             "[" +
             replaced(
                 vec(10), "{\"type\": \"none\"}",
                 R"({"type": "polygon", "centre_mm": [0, 0], "sides": 9999999999, "radius_mm": 1, "angle_rad": 0})") +
             "]"),
         Code::InvalidContent, "sides"},
        {"dangling child",
         fileWith(
             "[" + vec(10, ", \"parent\": 20") +
             R"(, {"id": 20, "layer": 2, "sheet": 1, "name": "g", "type": "group", "children": [10, 99]}])"),
         Code::IntegrityFailure, "#99"},
        {"parent not listing child",
         fileWith("[" + vec(10, ", \"parent\": 20") +
                  R"(, {"id": 20, "layer": 2, "sheet": 1, "name": "g", "type": "group", "children": []}])"),
         Code::IntegrityFailure, "#10"},
        {"parent is not a group", fileWith("[" + vec(10, ", \"parent\": 11") + ", " + vec(11) + "]"),
         Code::IntegrityFailure, "#11"},
        {"group cycle",
         fileWith(
             R"([{"id": 20, "layer": 2, "sheet": 1, "parent": 21, "name": "a", "type": "group", "children": [21]},
                      {"id": 21, "layer": 2, "sheet": 1, "parent": 20, "name": "b", "type": "group", "children": [20]}])"),
         Code::IntegrityFailure, "loops"},
    };
    for (const Case& c : cases) {
        INFO(c.label);
        const CarvError err = loadError(c.file);
        INFO(err.message);
        CHECK(err.code == c.code);
        CHECK(contains(err.message, c.mention));
    }
}

TEST_CASE("fuzz: flipped and truncated archive bytes never crash the loader", "[document][carv][fuzz]") {
    Document doc = makeAcceptanceDocument();
    const ObjectId g = group(doc, {ObjectId{doc.objectIds()[0]}, ObjectId{doc.objectIds()[1]}});
    (void)g;
    const Bytes good = save(doc);
    std::mt19937 rng(12345);
    int loadedOk = 0;
    for (int i = 0; i < 400; ++i) {
        Bytes bad = good;
        const auto mode = static_cast<unsigned>(rng() % 3);
        if (mode != 1) {  // flip 1..8 bytes
            const auto flips = static_cast<unsigned>(1 + rng() % 8);
            for (unsigned f = 0; f < flips; ++f) {
                bad[rng() % bad.size()] ^= static_cast<std::uint8_t>(1 + rng() % 255);
            }
        }
        if (mode != 0) {  // truncate
            bad.resize(rng() % bad.size());
        }
        const auto r = loadCarv(std::span<const std::uint8_t>(bad));
        if (r.ok()) {
            ++loadedOk;
            INFO("case " << i);
            requireIntact(r.value().document);
            CHECK(saveCarv(r.value().document, r.value().extras).ok());
        } else {
            CHECK_FALSE(r.error().message.empty());
        }
    }
    // Flips in the zip headers' unchecked fields (e.g. timestamps, attributes) may still load.
    CHECK(loadedOk < 400);
}

TEST_CASE("fuzz: mutated job.json text never crashes the loader", "[document][carv][fuzz]") {
    Document doc = makeAcceptanceDocument();
    group(doc, {doc.objectIds()[0], doc.objectIds()[1]});
    const Bytes good = save(doc);
    const std::string text = memberText(good, "job.json");
    const std::string alphabet = "{}[]\",:.-+eE0123456789 truefalsnul#";
    std::mt19937 rng(777);
    int loadedOk = 0;
    for (int i = 0; i < 300; ++i) {
        std::string t = text;
        const auto edits = static_cast<unsigned>(1 + rng() % 4);
        for (unsigned e = 0; e < edits; ++e) {
            const std::size_t pos = rng() % t.size();
            switch (rng() % 4) {
                case 0:
                    t[pos] = alphabet[rng() % alphabet.size()];
                    break;
                case 1:
                    t.erase(pos, 1 + rng() % 16);
                    break;
                case 2:
                    t.insert(pos, t.substr(rng() % t.size(), 1 + rng() % 40));
                    break;
                default: {
                    // Swap one digit for another: keeps the JSON valid but perturbs ids and refs.
                    const std::size_t d = t.find_first_of("0123456789", pos);
                    if (d != std::string::npos) t[d] = static_cast<char>('0' + rng() % 10);
                    break;
                }
            }
        }
        const auto r = loadCarv(std::span<const std::uint8_t>(withMember(good, "job.json", t)));
        if (r.ok()) {
            ++loadedOk;
            INFO("case " << i);
            requireIntact(r.value().document);
            const Bytes once = save(r.value().document, r.value().extras);
            LoadedCarv again = load(once);
            CHECK(save(again.document, again.extras) == once);
        } else {
            CHECK_FALSE(r.error().message.empty());
        }
    }
    CHECK(loadedOk < 300);
}

// ------------------------------------------------------------------------------------------------
// Resource limits: decompression bombs, JSON memory budget, archive caps, id range
// ------------------------------------------------------------------------------------------------

namespace {

std::uint32_t readLe32(const Bytes& b, std::size_t at) {
    return static_cast<std::uint32_t>(b[at]) | static_cast<std::uint32_t>(b[at + 1]) << 8 |
           static_cast<std::uint32_t>(b[at + 2]) << 16 | static_cast<std::uint32_t>(b[at + 3]) << 24;
}

std::uint16_t readLe16(const Bytes& b, std::size_t at) {
    return static_cast<std::uint16_t>(b[at] | b[at + 1] << 8);
}

void writeLe32(Bytes& b, std::size_t at, std::uint32_t v) {
    for (int k = 0; k < 4; ++k) {
        b[at + static_cast<std::size_t>(k)] = static_cast<std::uint8_t>(v >> (8 * k));
    }
}

/// Rewrites the uncompressed size declared in the central directory for member `name`. The data
/// itself is unchanged, so only the up-front limit checks can see the claimed size.
Bytes withDeclaredSize(Bytes zip, const std::string& name, std::uint32_t size) {
    REQUIRE(zip.size() >= 22);
    const std::size_t eocd = zip.size() - 22;  // no archive comment
    REQUIRE(readLe32(zip, eocd) == 0x06054b50u);
    const std::uint16_t entries = readLe16(zip, eocd + 10);
    std::size_t at = readLe32(zip, eocd + 16);
    bool patched = false;
    for (std::uint16_t e = 0; e < entries; ++e) {
        REQUIRE(readLe32(zip, at) == 0x02014b50u);
        const std::size_t nameLen = readLe16(zip, at + 28);
        const std::size_t extraLen = readLe16(zip, at + 30);
        const std::size_t commentLen = readLe16(zip, at + 32);
        const std::string entryName(zip.begin() + static_cast<std::ptrdiff_t>(at + 46),
                                    zip.begin() + static_cast<std::ptrdiff_t>(at + 46 + nameLen));
        if (entryName == name) {
            writeLe32(zip, at + 24, size);
            patched = true;
        }
        at += 46 + nameLen + extraLen + commentLen;
    }
    REQUIRE(patched);
    return zip;
}

double secondsToLoad(const Bytes& bytes, CarvError& error) {
    const auto t0 = std::chrono::steady_clock::now();
    auto r = loadCarv(std::span<const std::uint8_t>(bytes));
    const double s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    REQUIRE_FALSE(r.ok());
    error = r.error();
    return s;
}

std::string idFile(const std::string& objectId, const std::string& nextId) {
    return jobWith("[" + replaced(vec(10), "\"id\": 10", "\"id\": " + objectId) + "]", nextId);
}

}  // namespace

namespace {

/// Appends `unit` repeated (comma separated) to about `bytes`, in 64 KiB blocks.
void appendRepeated(Bytes& out, std::string_view unit, std::size_t bytes) {
    std::string block;
    while (block.size() < (std::size_t{1} << 16)) {
        block += unit;
        block += ',';
    }
    const std::size_t start = out.size();
    while (out.size() - start + block.size() < bytes) {
        out.insert(out.end(), block.begin(), block.end());
    }
    out.insert(out.end(), unit.begin(), unit.end());
}

void appendText(Bytes& out, std::string_view text) {
    out.insert(out.end(), text.begin(), text.end());
}

using BombBody = std::function<void(Bytes&)>;

/// `"x": <body>` followed by `"pad": "<random text>"`, sized like the verification report's bombs:
/// the random pad keeps the member under the 200:1 JSON ratio check so the parser sees the bomb.
void appendBomb(Bytes& out, const BombBody& body, std::mt19937_64& rng) {
    const std::size_t start = out.size();
    appendText(out, ", \"x\": ");
    body(out);
    const std::size_t bombBytes = out.size() - start;
    appendText(out, ", \"pad\": \"");
    static constexpr char kAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    for (std::size_t i = 0; i < bombBytes / 140; ++i) {  // ~0.75 compressed: keeps the ratio near 190:1
        out.push_back(static_cast<std::uint8_t>(kAlphabet[rng() % 64]));
    }
    out.push_back('"');
}

/// A .carv file whose job.json (and optionally manifest.json) carries an unknown bomb field.
Bytes bombFile(const BombBody& job, const BombBody& manifest = {}, int level = 1) {
    std::mt19937_64 rng(5);
    Bytes m;
    appendText(m,
               R"({"app_version": "test", "format": "carvetto", "format_version": {"major": 1, "minor": 0})");
    if (manifest) {
        appendBomb(m, manifest, rng);
    }
    m.push_back('}');
    std::string head = jobWith("[]");
    head.resize(head.rfind('}'));  // reopen the root object
    Bytes j;
    appendText(j, head);
    if (job) {
        appendBomb(j, job, rng);
    }
    j.push_back('}');
    std::vector<detail::ZipMember> ms;
    ms.push_back({"manifest.json", std::move(m)});
    ms.push_back({"job.json", std::move(j)});
    auto r = detail::writeZip(ms, level);  // level 1 by default: fast for big fixtures
    REQUIRE(r.ok);
    return r.bytes;
}

BombBody array(std::string unit, std::size_t bytes) {
    return [unit, bytes](Bytes& out) {
        out.push_back('[');
        appendRepeated(out, unit, bytes);
        out.push_back(']');
    };
}

#if defined(__linux__)
std::int64_t statusKb(const char* field) {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind(field, 0) == 0) {
            return std::stoll(line.substr(std::string(field).size()));
        }
    }
    return -1;
}

/// Peak RSS growth (bytes) of loading `file` in a forked child: the child trims the heap, resets
/// its high-water mark (/proc/self/clear_refs = 5), loads, and reports VmHWM - VmRSS(before).
/// -1 if unsupported.
std::int64_t childPeakGrowth(const Bytes& file) {
    int fds[2];
    if (pipe(fds) != 0) {
        return -1;
    }
    const pid_t pid = fork();
    if (pid == 0) {
        close(fds[0]);
        std::int64_t result = -1;
#if defined(__GLIBC__)
        malloc_trim(0);  // return memory the parent freed, so the load cannot hide in reused pages
#endif
        {
            std::ofstream clear("/proc/self/clear_refs");
            clear << "5";
        }
        const std::int64_t before = statusKb("VmRSS:");
        if (before >= 0 && statusKb("VmHWM:") <= before + 1024) {  // the reset took effect
            {
                auto r = loadCarv(std::span<const std::uint8_t>(file));
                (void)r;
            }
            result = (statusKb("VmHWM:") - before) * 1024;
        }
        const ssize_t written = write(fds[1], &result, sizeof result);
        (void)written;
        _exit(0);
    }
    close(fds[1]);
    std::int64_t result = -1;
    if (pid > 0) {
        if (read(fds[0], &result, sizeof result) != static_cast<ssize_t>(sizeof result)) {
            result = -1;
        }
        int status = 0;
        waitpid(pid, &status, 0);
    }
    close(fds[0]);
    return result;
}
#endif

/// A job.json with `objects` (pre-rendered) and the given next_id.
Bytes structureFile(const std::string& objects, std::uint64_t nextId) {
    return zipOf(
        {{"manifest.json", kManifest}, {"job.json", jobWith("[" + objects + "]", std::to_string(nextId))}});
}

std::string groupRecord(std::uint64_t id, const std::string& children, std::uint64_t parent) {
    return R"({"id": )" + std::to_string(id) + R"(, "layer": 2, "sheet": 1, "name": "", "type": "group", )" +
           (parent ? R"("parent": )" + std::to_string(parent) + ", " : std::string()) + R"("children": [)" +
           children + "]}";
}

std::string vectorRecord(std::uint64_t id, std::uint64_t parent) {
    return R"({"id": )" + std::to_string(id) + R"(, "layer": 2, "sheet": 1, "name": "", "type": "vector", )" +
           R"("parent": )" + std::to_string(parent) + R"(, "contours": [], "params": {"type": "none"}})";
}

}  // namespace

TEST_CASE("bombs from the verification reports are refused in bounded time and memory",
          "[document][carv][limits][bombs]") {
    constexpr std::size_t MiB = std::size_t{1} << 20;
    struct Bomb {
        const char* name;
        std::function<Bytes()> make;
        bool smallFile{true};  // a decompression bomb (<= 2 MiB file); false for breadth attacks
    };
    const std::vector<Bomb> bombs{
        {"combo_str", [] { return bombFile(array("\"\"", 90 * MiB), array("\"\"", 90 * MiB)); }},
        {"manifest_str", [] { return bombFile({}, array("\"\"", 90 * MiB)); }},
        {"arr_str_60M", [] { return bombFile(array("\"\"", 90 * MiB)); }},
        {"arr_arr_40M", [] { return bombFile(array("[]", 45 * MiB)); }},
        {"arr_zero_60M", [] { return bombFile(array("0", 60 * MiB)); }},
        {"arr_true_150M", [] { return bombFile(array("true", 150 * MiB)); }},
        {"arr_obj_44M", [] { return bombFile(array("{}", 44 * MiB)); }},
        // Distinct keys barely compress, so this one is a breadth attack (a ~5 MB file), not a bomb.
        {"many_keys_2M",
         [] {
             return bombFile(
                 [](Bytes& out) {
                     out.push_back('{');
                     std::array<char, 16> key{'"', 'k', '0', '0', '0', '0', '0', '0',
                                              '0', '0', '0', '"', ':', '0', ','};
                     for (int i = 0; i < 2'000'000; ++i) {
                         out.insert(out.end(), key.begin(), key.begin() + 15);
                         for (int d = 10; d >= 2; --d) {  // increment the decimal counter in place
                             if (++key[static_cast<std::size_t>(d)] <= '9') break;
                             key[static_cast<std::size_t>(d)] = '0';
                         }
                     }
                     out.back() = '}';
                 },
                 {}, 6);
         },
         false},
        {"dup_keys_60M",
         [] {
             return bombFile([](Bytes& out) {
                 out.push_back('{');
                 appendRepeated(out, "\"a\":0", 60 * MiB);
                 out.push_back('}');
             });
         }},
        {"long_string_250M",
         [] {
             return bombFile([](Bytes& out) {
                 out.push_back('"');
                 out.insert(out.end(), 250 * MiB, static_cast<std::uint8_t>('a'));
                 out.push_back('"');
             });
         }},
        {"long_number_250M",
         [] {
             return bombFile([](Bytes& out) {
                 out.push_back('1');
                 out.insert(out.end(), 250 * MiB - 10, static_cast<std::uint8_t>('0'));
             });
         }},
        {"deep_wide",
         [] { return bombFile(array(std::string(60, '[') + "0" + std::string(60, ']'), 100 * MiB)); }},
        {"medium_objs",
         [] {
             return bombFile(
                 array(R"({"a":1,"b":[1,2,3],"c":"0123456789abcdefXYZ","d":{"e":null}})", 100 * MiB));
         }},
    };
    for (const Bomb& bomb : bombs) {
        INFO(bomb.name);
        const Bytes file = bomb.make();
        if (bomb.smallFile) {
            CHECK(file.size() <= 2 * MiB);
        }
        CarvError err;
        const double seconds = secondsToLoad(file, err);
        INFO(err.message);
        CHECK(err.code == Code::ResourceLimit);
        CHECK(seconds < 2.0);
        std::int64_t peak = -1;
#if defined(__linux__)
        peak = childPeakGrowth(file);
        if (peak >= 0) {
            CHECK(peak < std::int64_t{512} << 20);
        } else {
            WARN("peak RSS not measurable here (no /proc/self/clear_refs)");
        }
#endif
        WARN(bomb.name << ": " << file.size() / 1024 << " KiB file, refused in " << seconds
                       << " s, peak RSS growth " << peak / (std::int64_t{1} << 20)
                       << " MiB: " << err.message);
    }
}

TEST_CASE("group structure is pre-checked linearly", "[document][carv][limits]") {
    SECTION("a 60,000-deep group chain is refused quickly") {
        constexpr std::uint64_t kDepth = 60000;
        std::string objects;
        for (std::uint64_t i = 0; i < kDepth; ++i) {
            const std::uint64_t id = 10 + i;
            objects += groupRecord(id, std::to_string(id + 1), i ? id - 1 : 0) + ",";
        }
        objects += vectorRecord(10 + kDepth, 10 + kDepth - 1);
        const Bytes file = structureFile(objects, 20 + kDepth);
        CarvError err;
        const double seconds = secondsToLoad(file, err);
        INFO(err.message);
        CHECK(err.code == Code::ResourceLimit);
        CHECK(contains(err.message, "deeper than 1024"));
        CHECK(seconds < 2.0);
    }
    SECTION("a 1,000-deep chain (under the limit) loads") {
        std::string objects;
        for (std::uint64_t i = 0; i < 1000; ++i) {
            objects += groupRecord(10 + i, std::to_string(11 + i), i ? 9 + i : 0) + ",";
        }
        objects += vectorRecord(1010, 1009);
        LoadedCarv loaded = load(structureFile(objects, 2000));
        CHECK(loaded.document.objectCount() == 1001);
        requireIntact(loaded.document);
    }
    SECTION("a group with 200,000 children loads quickly") {
        constexpr std::uint64_t kWidth = 200000;
        std::string objects;
        std::string children;
        for (std::uint64_t i = 0; i < kWidth; ++i) {
            objects += vectorRecord(10 + i, 5) + ",";
            children += (i ? "," : "") + std::to_string(10 + i);
        }
        objects += groupRecord(5, children, 0);
        const Bytes file = structureFile(objects, 10 + kWidth);
        const auto t0 = std::chrono::steady_clock::now();
        auto r = loadCarv(std::span<const std::uint8_t>(file));
        const double seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        INFO((r ? std::string() : r.error().message));
        REQUIRE(r.ok());
        CHECK(r.value().document.objectCount() == kWidth + 1);
        CHECK(r.value().document.findObject(ObjectId{5})->asGroup()->children.size() == kWidth);
        CHECK(seconds < 10.0);  // Debug; the pre-check and checkIntegrity (T-0014) are both linear
    }
    SECTION("a cycle is reported without walking it forever") {
        const std::string objects = groupRecord(10, "11", 11) + "," + groupRecord(11, "10", 10);
        const CarvError err = loadError(structureFile(objects, 100));
        CHECK(err.code == Code::IntegrityFailure);
        CHECK(contains(err.message, "loops"));
    }
}

TEST_CASE("each load limit fires with ResourceLimit", "[document][carv][limits]") {
    const auto canonical = [](std::string_view text, detail::LoadLimits limits) {
        detail::LoadBudget budget(limits);
        return detail::canonicalJson(text, budget);
    };
    const auto codeOf = [&](std::string_view text, detail::LoadLimits limits) {
        try {
            canonical(text, limits);
        } catch (const detail::LoadFailure& f) {
            return f.code;
        }
        return Code::Io;  // no failure
    };
    detail::LoadLimits l;
    CHECK(canonical(R"({"b": [1, 2.50, -0, 1e2], "a": "\u00e9"})", l) ==
          R"({"a": "é", "b": [1, 2.5, 0, 100.0]})");
    l.maxNumberChars = 8;
    CHECK(codeOf("123456789", l) == Code::ResourceLimit);
    l = {};
    l.maxUnknownStringBytes = 10;
    CHECK(codeOf(R"("01234567890")", l) == Code::ResourceLimit);
    l = {};
    l.maxUnknownContainerElements = 3;
    CHECK(codeOf("[1, 2, 3, 4]", l) == Code::ResourceLimit);
    CHECK(codeOf(R"({"a": 1, "b": 2, "c": 3, "d": 4})", l) == Code::ResourceLimit);
    l = {};
    l.maxValues = 5;
    CHECK(codeOf("[[1], [2], [3]]", l) == Code::ResourceLimit);
    l = {};
    l.maxUnknownValues = 5;
    CHECK(codeOf("[[1], [2], [3]]", l) == Code::ResourceLimit);
    l = {};
    l.maxExtrasBytes = 100;
    CHECK(
        codeOf(
            R"({"a": "0123456789012345678901234567890123456789", "b": "0123456789012345678901234567890123456789"})",
            l) == Code::ResourceLimit);
    l = {};
    l.maxDepth = 3;
    CHECK(codeOf("[[[[0]]]]", l) == Code::ResourceLimit);
    // Syntax errors stay InvalidJson.
    CHECK(codeOf(R"({"a": 1,})", {}) == Code::InvalidJson);
    CHECK(codeOf(R"({"a": 1, "a": 2})", {}) == Code::InvalidJson);
    CHECK(codeOf("[01]", {}) == Code::InvalidJson);
    CHECK(codeOf("\"\xC3\x28\"", {}) == Code::InvalidJson);  // invalid UTF-8
    CHECK(codeOf("\"\\ud800\"", {}) == Code::InvalidJson);   // lone surrogate
    CHECK(codeOf("[1] x", {}) == Code::InvalidJson);
    CHECK(codeOf("1e999", {}) == Code::InvalidContent);  // out of double range

    // Known fields: a name longer than 1 MiB is refused.
    const std::string longName(std::size_t{1} << 21, 'n');
    const CarvError err = loadError(
        fileWith("[" + replaced(vec(10), "\"name\": \"v\"", "\"name\": \"" + longName + "\"") + "]"));
    CHECK(err.code == Code::ResourceLimit);
}
TEST_CASE("archive size caps are enforced before decompression", "[document][carv][limits]") {
    const Bytes good = save(makeAcceptanceDocument());
    SECTION("a JSON member declared over 256 MiB") {
        const CarvError err = loadError(withDeclaredSize(good, "job.json", 300u << 20));
        CHECK(err.code == Code::ResourceLimit);
        CHECK(contains(err.message, "too large"));
    }
    SECTION("a binary member declared over 256 MiB") {
        const Bytes withBlob = withMember(good, "components/a.hf", std::string(1000, 'x'));
        const CarvError err = loadError(withDeclaredSize(withBlob, "components/a.hf", 600u << 20));
        CHECK(err.code == Code::ResourceLimit);
        CHECK(contains(err.message, "components/a.hf"));
    }
    SECTION("many large unknown members: refused before anything is decompressed") {
        Bytes zip = good;
        for (int i = 0; i < 5; ++i) {
            zip = withMember(zip, "bitmaps/" + std::to_string(i) + ".png",
                             std::string(4096, static_cast<char>('a' + i)));
        }
        for (int i = 0; i < 5; ++i) {
            zip = withDeclaredSize(zip, "bitmaps/" + std::to_string(i) + ".png", 250u << 20);  // 5 x 250 MiB
        }
        CarvError err;
        const double seconds = secondsToLoad(zip, err);
        CHECK(err.code == Code::ResourceLimit);  // the binary ratio check (20:1) fires first
        CHECK(contains(err.message, "more than"));
        CHECK(seconds < 2.0);
        // With the ratio check out of the way, the 1 GiB total cap still refuses them up front.
        detail::ZipLimits noRatio;
        noRatio.maxBinaryRatio = std::uint64_t{1} << 40;
        const auto r = detail::readZip(zip, noRatio);
        CHECK_FALSE(r.ok);
        CHECK(r.limitExceeded);
        CHECK(contains(r.error, "more than"));
    }
    SECTION("many small members that add up to a bomb (each under the per-member ratio threshold)") {
        std::vector<detail::ZipMember> ms = members(good);
        for (int i = 0; i < 600; ++i) {
            ms.push_back({"m" + std::to_string(i) + ".bin", Bytes(100000, 0)});  // 60 MB of zeros in total
        }
        const auto zip = detail::writeZip(ms, 1);
        REQUIRE(zip.ok);
        CarvError err;
        const double seconds = secondsToLoad(zip.bytes, err);
        INFO(err.message);
        CHECK(err.code == Code::ResourceLimit);
        CHECK(contains(err.message, "overall"));
        CHECK(seconds < 2.0);
        // Incompressible members of the same total size are fine.
        std::mt19937 rng(3);
        std::vector<detail::ZipMember> real = members(good);
        for (int i = 0; i < 30; ++i) {
            Bytes data(100000);
            for (auto& b : data) {
                b = static_cast<std::uint8_t>(rng());
            }
            real.push_back({"r" + std::to_string(i) + ".bin", std::move(data)});
        }
        const auto okZip = detail::writeZip(real, 1);
        REQUIRE(okZip.ok);
        CHECK(load(okZip.bytes).extras.memberNames().size() == 30);
    }
    SECTION("the total cap with real data (reduced limits)") {
        std::vector<detail::ZipMember> ms;
        for (int i = 0; i < 3; ++i) {
            Bytes data(512u << 10);
            std::mt19937 rng(static_cast<std::uint32_t>(i));
            for (auto& b : data) {
                b = static_cast<std::uint8_t>(rng());
            }
            ms.push_back({"blob" + std::to_string(i), std::move(data)});
        }
        const auto zip = detail::writeZip(ms);
        REQUIRE(zip.ok);
        detail::ZipLimits limits;
        limits.maxTotalBytes = std::uint64_t{1} << 20;
        const auto r = detail::readZip(zip.bytes, limits);
        CHECK_FALSE(r.ok);
        CHECK(r.limitExceeded);
        CHECK(r.members.empty());  // nothing was extracted
        CHECK(detail::readZip(zip.bytes).ok);
    }
}

TEST_CASE("ids at or beyond 2^62 are rejected; next_id may equal 2^62", "[document][carv][limits]") {
    const auto file = [](const std::string& objectId, const std::string& nextId) {
        return zipOf({{"manifest.json", kManifest}, {"job.json", idFile(objectId, nextId)}});
    };
    const CarvError maxNext = loadError(file("10", "18446744073709551615"));  // UINT64_MAX
    CHECK(maxNext.code == Code::InvalidContent);
    CHECK(contains(maxNext.message, "next_id"));
    const CarvError bigId = loadError(file("9223372036854775808", "100"));  // 2^63
    CHECK(bigId.code == Code::InvalidContent);
    CHECK(contains(bigId.message, "2^62"));
    CHECK(loadError(file("4611686018427387904", "100")).code == Code::InvalidContent);  // 2^62
    CHECK(loadError(file("10", "4611686018427387905")).code == Code::InvalidContent);   // 2^62 + 1
    const std::string badChild = jobWith(
        "[" + vec(10, ", \"parent\": 20") +
        R"(, {"id": 20, "layer": 2, "sheet": 1, "name": "g", "type": "group", "children": [10, 4611686018427387904]}])");
    CHECK(loadError(zipOf({{"manifest.json", kManifest}, {"job.json", badChild}})).code ==
          Code::InvalidContent);

    // The largest valid id with an exhausted generator loads, and the generator is restored exactly.
    LoadedCarv loaded = load(file("4611686018427387903", "4611686018427387904"));  // 2^62 - 1, 2^62
    CHECK(loaded.document.hasObject(ObjectId{(std::uint64_t{1} << 62) - 1}));
    CHECK(loaded.document.idGenerator().peek() == std::uint64_t{1} << 62);
    requireIntact(loaded.document);
}

TEST_CASE("a legitimate 100k-object document loads within the limits", "[document][carv][limits]") {
    Document doc;
    for (int i = 0; i < 100000; ++i) {
        const double s = 0.37 * i + 1.0 / 3.0;
        doc.insertObject(Object::vector(Contour::circle(Vec2{s, -s}, 2.5 + s), doc.defaultLayer(),
                                        doc.activeSheet(), CircleParams{Vec2{s, -s}, 2.5 + s}));
    }
    const Bytes file = save(doc);
    LoadedCarv loaded = load(file);
    CHECK(loaded.document.objectCount() == 100000);
    CHECK(loaded.document.objects() == doc.objects());
}
