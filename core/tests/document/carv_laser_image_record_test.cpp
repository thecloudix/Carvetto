// T-0324: Laser Picture and Laser Sketch records (ADR-0104, ADR-0105) in .carv: the LaserPictureParams and
// LaserSketchParams alternatives, their job.json params, minor 1.16, exact round trips and hostile input.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/laser_picture.hpp"
#include "carvetto/toolpath/laser_sketch.hpp"

using namespace carvetto;
using namespace carvetto::document;
using toolpath::Move;
using Bytes = std::vector<std::uint8_t>;

namespace {

toolpath::LaserPictureParams fullPicture() {
    toolpath::LaserPictureParams p;
    p.name = "Picture \"a\" \xC3\xA9";
    p.strategy = toolpath::PhotoStrategy::Hatch;
    p.power = 72.5;
    p.modulatePower = false;
    p.minPower = 12.5;
    p.dither = true;
    p.invert = true;
    p.moveSpeed = 1500.5;
    p.passes = 3;
    p.overscan = 1.25;
    p.lineSpacing = 0.075;
    p.lineAngle = 0.5235987755982988;
    p.safeZ = 6.5;
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

toolpath::LaserSketchParams fullSketch() {
    toolpath::LaserSketchParams p;
    p.name = "Sketch \xC3\xA9";
    p.source = toolpath::SketchSource::Model;
    p.modelTone = toolpath::SketchModelTone::Shaded;
    p.boundary = toolpath::SketchBoundary::Vectors;
    p.invert = true;
    p.power = 55.5;
    p.moveSpeed = 2200.0;
    p.overscan = 2.5;
    p.machineLimitBoundary = true;
    p.lineThickness = 0.45;
    p.stepover = 0.65;
    p.hatchAngle = 0.7853981633974483;
    p.crossHatch = true;
    p.fill = toolpath::SketchFill::BetweenVectors;
    p.safeZ = 8.5;
    p.order.leftToRight = true;
    p.startPosition = {-3.5, 2.25};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool beam() {
    tooling::Tool t;
    t.id = "laser";
    t.name = "Laser";
    t.diameter = 0.2;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 0.0, 900.0, 900.0, 2000.0, 1};
    return t;
}

toolpath::GrayImage image() {
    toolpath::GrayImage g;
    g.width = 4;
    g.height = 4;
    g.pixels = {0, 40, 80, 120, 160, 200, 240, 255, 0, 0, 0, 0, 90, 90, 200, 10};
    return g;
}

/// A document with one record of `params` whose moves were computed for real.
Document documentWith(const ToolpathParams& params) {
    Document doc;
    const geometry::Contour c = geometry::Contour::rectangle(geometry::Box2({0, 0}, {4, 4}));
    const ObjectId a = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
    toolpath::Toolpath tp;
    if (const auto* pic = std::get_if<toolpath::LaserPictureParams>(&params)) {
        toolpath::LaserPictureParams p = *pic;
        p.lineSpacing = 0.5;
        auto r = toolpath::laserPictureToolpath(image(), geometry::Box2({0, 0}, {4, 4}), beam(), std::nullopt, p,
                                                {c});
        INFO((r.diagnostics.empty() ? std::string() : r.diagnostics.front().message));
        REQUIRE(r.ok());
        tp = std::move(r.toolpath);
    } else {
        toolpath::LaserSketchParams p = std::get<toolpath::LaserSketchParams>(params);
        p.boundary = toolpath::SketchBoundary::Source;
        p.stepover = 0.5;
        auto r = toolpath::laserSketchToolpath(image(), geometry::Box2({0, 0}, {4, 4}), beam(), std::nullopt, p);
        INFO((r.diagnostics.empty() ? std::string() : r.diagnostics.front().message));
        REQUIRE(r.ok());
        tp = std::move(r.toolpath);
    }
    ToolpathRecord rec;
    rec.name = tp.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.tool = tp.tool;
    rec.toolSource = {"laser", "", ""};
    rec.moves = shareMoves(std::move(tp.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

std::vector<detail::ZipMember> members(const Bytes& archive) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    return r.members;
}

Bytes member(const Bytes& archive, const std::string& name) {
    for (const auto& m : members(archive)) {
        if (m.name == name) {
            return m.data;
        }
    }
    FAIL("member " << name << " not found");
    return {};
}

Bytes withMember(const Bytes& archive, const std::string& name, const Bytes& data) {
    auto ms = members(archive);
    for (auto& m : ms) {
        if (m.name == name) {
            m.data = data;
        }
    }
    auto r = detail::writeZip(ms);
    REQUIRE(r.ok);
    return r.bytes;
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

}  // namespace

TEST_CASE("T-0324: a laser picture saves and loads exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][laser][picture]") {
    const Document doc = documentWith(fullPicture());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::LaserPicture);
    CHECK(doc.toolpaths()[0].safeZ() == 6.5);
    CHECK(doc.toolpaths()[0].tool.laser);
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 1);
    CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.value().formatMinor == kCarvFormatMinorLaserImages);
    CHECK(std::get<toolpath::LaserPictureParams>(back.value().document.toolpaths()[0].params) == fullPicture());
    bool varied = false;
    for (const Move& m : *back.value().document.toolpaths()[0].moves) {
        varied = varied || (m.jetPower > 0.0 && m.jetPower != 72.5);
    }
    CHECK_FALSE(varied);  // a dithered picture burns at one power
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0324: a laser sketch saves and loads exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][laser][sketch]") {
    const Document doc = documentWith(fullSketch());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::LaserSketch);
    CHECK(doc.toolpaths()[0].tool.laser);
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.value().formatMinor == kCarvFormatMinorLaserImages);
    CHECK(std::get<toolpath::LaserSketchParams>(back.value().document.toolpaths()[0].params) == fullSketch());
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0324: every strategy, source, tone, boundary and fill survives a save and load",
          "[document][toolpath][carv][laser]") {
    for (const auto s : {toolpath::PhotoStrategy::Raster, toolpath::PhotoStrategy::Hatch,
                         toolpath::PhotoStrategy::SelectedVectors}) {
        toolpath::LaserPictureParams p = fullPicture();
        p.strategy = s;
        // The saved moves do not depend on the strategy being valid for the vector; compute with raster.
        const Document doc = documentWith(p);
        auto saved = saveCarv(doc, CarvExtras{});
        REQUIRE(saved.ok());
        auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
        REQUIRE(back.ok());
        CHECK(std::get<toolpath::LaserPictureParams>(back.value().document.toolpaths()[0].params) == p);
    }
    for (const auto src : {toolpath::SketchSource::Bitmap, toolpath::SketchSource::Model}) {
        for (const auto tone : {toolpath::SketchModelTone::Depth, toolpath::SketchModelTone::Shaded}) {
            for (const auto b : {toolpath::SketchBoundary::Source, toolpath::SketchBoundary::Vectors}) {
                for (const auto f : {toolpath::SketchFill::Together, toolpath::SketchFill::Regionally,
                                     toolpath::SketchFill::BetweenVectors}) {
                    toolpath::LaserSketchParams p = fullSketch();
                    p.source = src;
                    p.modelTone = tone;
                    p.boundary = b;
                    p.fill = f;
                    const Document doc = documentWith(p);
                    auto saved = saveCarv(doc, CarvExtras{});
                    REQUIRE(saved.ok());
                    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
                    REQUIRE(back.ok());
                    CHECK(std::get<toolpath::LaserSketchParams>(back.value().document.toolpaths()[0].params) == p);
                }
            }
        }
    }
}

TEST_CASE("T-0324: the file is 1.16 only when a laser picture or sketch is saved", "[document][toolpath][carv][laser]") {
    Document plain;
    auto saved = saveCarv(plain, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    CHECK(back.value().formatMinor < kCarvFormatMinorLaserImages);
    for (const ToolpathParams& p : {ToolpathParams{fullPicture()}, ToolpathParams{fullSketch()}}) {
        auto file = saveCarv(documentWith(p), CarvExtras{});
        REQUIRE(file.ok());
        const Bytes manifestBytes = member(file.value(), "manifest.json");
        const std::string manifest(manifestBytes.begin(), manifestBytes.end());
        CHECK((contains(manifest, "\"minor\":16") || contains(manifest, "\"minor\": 16")));
    }
}

TEST_CASE("T-0324: the laser marking is accepted on the laser image operations only", "[document][toolpath][laser]") {
    Document doc = documentWith(fullPicture());
    ToolpathRecord sketch = doc.toolpaths()[0];
    sketch.params = fullSketch();
    CHECK(spindleOffProblem(sketch).empty());
    ToolpathRecord other = doc.toolpaths()[0];
    other.params = toolpath::ProfileParams{};
    CHECK_FALSE(spindleOffProblem(other).empty());
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(other)));
    ToolpathRecord mixed = doc.toolpaths()[0];
    mixed.tool.spindleOff = true;
    CHECK_FALSE(spindleOffProblem(mixed).empty());
    // Unmarked, not yet computed: fine.
    ToolpathRecord fresh = doc.toolpaths()[0];
    fresh.tool = {};
    fresh.moves = nullptr;
    CHECK(spindleOffProblem(fresh).empty());
    CHECK(isLaserOperation(ToolpathOperation::LaserPicture));
    CHECK(isLaserOperation(ToolpathOperation::LaserSketch));
    CHECK(isLaserOperation(ToolpathOperation::LaserCutFill));
    CHECK_FALSE(isLaserOperation(ToolpathOperation::ImageEngrave));
}

TEST_CASE("T-0324: non-finite laser picture and sketch parameters are refused by the commands",
          "[document][toolpath][laser]") {
    const double inf = std::numeric_limits<double>::infinity();
    {
        Document doc = documentWith(fullPicture());
        ToolpathRecord edit = doc.toolpaths()[0];
        auto& p = std::get<toolpath::LaserPictureParams>(edit.params);
        struct Field {
            double toolpath::LaserPictureParams::*member;
            const char* key;
        };
        const Field fields[] = {{&toolpath::LaserPictureParams::power, "power"},
                                {&toolpath::LaserPictureParams::minPower, "min_power"},
                                {&toolpath::LaserPictureParams::moveSpeed, "move_speed"},
                                {&toolpath::LaserPictureParams::overscan, "overscan"},
                                {&toolpath::LaserPictureParams::lineSpacing, "line_spacing"},
                                {&toolpath::LaserPictureParams::lineAngle, "line_angle_rad"},
                                {&toolpath::LaserPictureParams::safeZ, "safe_z"}};
        for (const Field& f : fields) {
            p = fullPicture();
            p.*(f.member) = inf;
            CHECK(nonFiniteParam(edit.params) == f.key);
            CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
        }
        p = fullPicture();
        p.tolerance.chordal = inf;
        CHECK_FALSE(nonFiniteParam(edit.params).empty());
        p = fullPicture();
        CHECK(nonFiniteParam(edit.params).empty());
    }
    {
        Document doc = documentWith(fullSketch());
        ToolpathRecord edit = doc.toolpaths()[0];
        auto& p = std::get<toolpath::LaserSketchParams>(edit.params);
        struct Field {
            double toolpath::LaserSketchParams::*member;
            const char* key;
        };
        const Field fields[] = {{&toolpath::LaserSketchParams::power, "power"},
                                {&toolpath::LaserSketchParams::moveSpeed, "move_speed"},
                                {&toolpath::LaserSketchParams::overscan, "overscan"},
                                {&toolpath::LaserSketchParams::lineThickness, "line_thickness"},
                                {&toolpath::LaserSketchParams::stepover, "stepover"},
                                {&toolpath::LaserSketchParams::hatchAngle, "hatch_angle_rad"},
                                {&toolpath::LaserSketchParams::safeZ, "safe_z"}};
        for (const Field& f : fields) {
            p = fullSketch();
            p.*(f.member) = inf;
            CHECK(nonFiniteParam(edit.params) == f.key);
            CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
        }
        p = fullSketch();
        p.startPosition.x = inf;
        CHECK(nonFiniteParam(edit.params) == "start_position[0]");
        p = fullSketch();
        CHECK(nonFiniteParam(edit.params).empty());
    }
}

TEST_CASE("T-0324: hostile laser picture and sketch records never crash the loader",
          "[document][toolpath][carv][laser][fuzz]") {
    for (const ToolpathParams& params : {ToolpathParams{fullPicture()}, ToolpathParams{fullSketch()}}) {
        const Bytes good = [&] {
            auto r = saveCarv(documentWith(params), CarvExtras{});
            REQUIRE(r.ok());
            return r.value();
        }();
        const Bytes jobBytes = member(good, "job.json");
        const std::string job(jobBytes.begin(), jobBytes.end());
        std::mt19937 rng(324);
        int loaded = 0;
        for (int i = 0; i < 200; ++i) {
            std::string t = job;
            const std::size_t from = t.find("\"toolpaths\"");
            REQUIRE(from != std::string::npos);
            const unsigned edits = 1 + static_cast<unsigned>(rng() % 3);
            for (unsigned e = 0; e < edits; ++e) {
                const std::size_t pos = from + rng() % (t.size() - from);
                t[pos] = "{}[]\",:.-+eE0123456789 truefalsnul#"[rng() % 35];
            }
            const Bytes file = withMember(good, "job.json", Bytes(t.begin(), t.end()));
            const auto r = loadCarv(std::span<const std::uint8_t>(file));
            if (r.ok()) {
                ++loaded;
                REQUIRE(r.value().document.checkIntegrity().empty());
                auto once = saveCarv(r.value().document, r.value().extras);
                REQUIRE(once.ok());
            } else {
                CHECK_FALSE(r.error().message.empty());
            }
        }
        CHECK(loaded < 200);
    }
    // Bad enum names and numbers.
    const Bytes goodPicture = [] {
        auto r = saveCarv(documentWith(fullPicture()), CarvExtras{});
        REQUIRE(r.ok());
        return r.value();
    }();
    const Bytes goodSketch = [] {
        auto r = saveCarv(documentWith(fullSketch()), CarvExtras{});
        REQUIRE(r.ok());
        return r.value();
    }();
    const auto loadWith = [](const Bytes& good, const std::string& from, const std::string& to) {
        const Bytes jobBytes = member(good, "job.json");
        std::string t(jobBytes.begin(), jobBytes.end());
        const std::size_t pos = t.find(from);
        REQUIRE(pos != std::string::npos);
        t.replace(pos, from.size(), to);
        return loadCarv(std::span<const std::uint8_t>(withMember(good, "job.json", Bytes(t.begin(), t.end()))));
    };
    CHECK_FALSE(loadWith(goodPicture, "\"hatch\"", "\"burn\"").ok());
    CHECK_FALSE(loadWith(goodPicture, "\"passes\": 3", "\"passes\": 3.5").ok());
    CHECK_FALSE(loadWith(goodPicture, "\"power\": 72.5", "\"power\": \"NaN\"").ok());
    CHECK_FALSE(loadWith(goodPicture, "\"laser_picture\"", "\"profile\"").ok());  // laser marking on another operation
    CHECK_FALSE(loadWith(goodSketch, "\"model\"", "\"photo\"").ok());
    CHECK_FALSE(loadWith(goodSketch, "\"shaded\"", "\"glossy\"").ok());
    CHECK_FALSE(loadWith(goodSketch, "\"between_vectors\"", "\"between\"").ok());
    CHECK_FALSE(loadWith(goodSketch, "\"vectors\"", "\"everything\"").ok());
    CHECK_FALSE(loadWith(goodSketch, "\"cross_hatch\": true", "\"cross_hatch\": 1").ok());
    CHECK_FALSE(loadWith(goodSketch, "\"laser_sketch\"", "\"profile\"").ok());
}
