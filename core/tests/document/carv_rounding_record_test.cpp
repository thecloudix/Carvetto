// T-0322: Rounding records (ADR-0115) in .carv: the RoundingParams alternative, its job.json params, minor 1.19
// only when one is saved (every other file is byte-identical to before), exact round trips, hostile input.

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
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/rounding.hpp"

using namespace carvetto;
using namespace carvetto::document;
using Bytes = std::vector<std::uint8_t>;

namespace {

toolpath::RoundingParams fullRounding() {
    toolpath::RoundingParams p;
    p.name = "Round \"a\" \xC3\xA9";
    p.method = toolpath::RoundingMethod::Radial;
    p.blank = toolpath::RoundingBlank::Square;
    p.alongX = false;
    p.blankRadius = 31.75;
    p.targetRadius = 12.125;
    p.startRadius = 20.5;
    p.allowance = 0.25;
    p.surfaceZ = -3.5;
    p.alongStart = 2.5;
    p.alongEnd = 140.0;
    p.circStart = -7.25;
    p.stepover = 2.5;
    p.passDepth = 1.5;
    p.safeZ = 40.0;
    return p;
}

tooling::Tool mill() {
    tooling::Tool t;
    t.id = "mill";
    t.name = "Mill";
    t.diameter = 6.0;
    t.defaultCuttingData = {2.0, 3.0, 0.0, 0.0, 12000.0, 1000.0, 300.0, 0.0, 1};
    return t;
}

Document roundingDocument(const toolpath::RoundingParams& params) {
    Document doc(Job::rotary(150.0, 63.5, Units::Millimetres, RotaryAxis::Y));
    toolpath::RoundingResult r = toolpath::roundingToolpath(mill(), std::nullopt, params);
    INFO((r.diagnostics.empty() ? std::string() : r.diagnostics.front().message));
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"mill", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
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

TEST_CASE("T-0322: rounding records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][rounding]") {
    const Document doc = roundingDocument(fullRounding());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Rounding);
    CHECK(doc.toolpaths()[0].selection.objects.empty());
    CHECK(doc.toolpaths()[0].moveCount() > 10);
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 1);
    CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.value().formatMinor == kCarvFormatMinorRounding);
    CHECK(std::get<toolpath::RoundingParams>(back.value().document.toolpaths()[0].params) == fullRounding());
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0322: every method and blank survives a save and load", "[document][toolpath][carv][rounding]") {
    for (const auto method : {toolpath::RoundingMethod::Radial, toolpath::RoundingMethod::Raster,
                              toolpath::RoundingMethod::OptimizedRaster}) {
        for (const auto blank : {toolpath::RoundingBlank::Round, toolpath::RoundingBlank::Square}) {
            toolpath::RoundingParams p = fullRounding();
            p.method = method;
            p.blank = blank;
            auto saved = saveCarv(roundingDocument(p), CarvExtras{});
            REQUIRE(saved.ok());
            auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
            REQUIRE(back.ok());
            const auto& q = std::get<toolpath::RoundingParams>(back.value().document.toolpaths()[0].params);
            CHECK(q.method == method);
            CHECK(q.blank == blank);
        }
    }
}

TEST_CASE("T-0322: the file is 1.19 only when a rounding record is saved", "[document][toolpath][carv][rounding]") {
    CHECK(kCarvFormatMinor == kCarvFormatMinorRounding);
    CHECK(kCarvFormatMinorRounding == 19);
    Document plain(Job::rotary(150.0, 63.5, Units::Millimetres));
    auto saved = saveCarv(plain, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    CHECK(back.value().formatMinor < kCarvFormatMinorRounding);
    auto rounding = saveCarv(roundingDocument(fullRounding()), CarvExtras{});
    REQUIRE(rounding.ok());
    const Bytes manifestBytes = member(rounding.value(), "manifest.json");
    const std::string manifest(manifestBytes.begin(), manifestBytes.end());
    CHECK((contains(manifest, "\"minor\":19") || contains(manifest, "\"minor\": 19")));
    // Deleting the toolpath goes back to the older minor.
    Document doc = roundingDocument(fullRounding());
    REQUIRE(doc.undo());
    auto after = saveCarv(doc, CarvExtras{});
    REQUIRE(after.ok());
    auto reloaded = loadCarv(std::span<const std::uint8_t>(after.value()));
    REQUIRE(reloaded.ok());
    CHECK(reloaded.value().formatMinor < kCarvFormatMinorRounding);
}

TEST_CASE("T-0322: non-finite rounding parameters are refused by the commands", "[document][toolpath][rounding]") {
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double inf = std::numeric_limits<double>::infinity();
    for (int field = 0; field < 11; ++field) {
        toolpath::RoundingParams p = fullRounding();
        double* slots[] = {&p.blankRadius, &p.targetRadius, &p.startRadius, &p.allowance, &p.surfaceZ, &p.alongStart,
                           &p.alongEnd,    &p.circStart,    &p.stepover,    &p.passDepth, &p.safeZ};
        *slots[field] = field % 2 == 0 ? nan : inf;
        Document doc(Job::rotary(150.0, 63.5, Units::Millimetres));
        ToolpathRecord rec;
        rec.name = "bad";
        rec.params = p;
        rec.tool.name = "Mill";
        rec.tool.toolNumber = 1;
        rec.tool.diameter = 6.0;
        rec.tool.feedRate = 1000.0;
        rec.tool.plungeRate = 300.0;
        rec.tool.spindleSpeed = 12000.0;
        rec.toolSource = {"mill", "", ""};
        rec.moves = shareMoves({toolpath::Move::rapid({0.0, 0.0, 5.0})});
        INFO("field " << field);
        CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
        CHECK(doc.toolpaths().empty());
    }
}

TEST_CASE("T-0322: hostile rounding records never crash the loader", "[document][toolpath][carv][rounding][fuzz]") {
    const Bytes good = [] {
        auto r = saveCarv(roundingDocument(fullRounding()), CarvExtras{});
        REQUIRE(r.ok());
        return r.value();
    }();
    const Bytes jobBytes = member(good, "job.json");
    const std::string job(jobBytes.begin(), jobBytes.end());
    std::mt19937 rng(322);
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
    const auto loadWith = [&](const std::string& from, const std::string& to) {
        std::string t = job;
        const std::size_t pos = t.find(from);
        REQUIRE(pos != std::string::npos);
        t.replace(pos, from.size(), to);
        return loadCarv(std::span<const std::uint8_t>(withMember(good, "job.json", Bytes(t.begin(), t.end()))));
    };
    CHECK_FALSE(loadWith("\"radial\"", "\"spiral\"").ok());
    CHECK_FALSE(loadWith("\"square\"", "\"hexagon\"").ok());
    CHECK_FALSE(loadWith("\"blank_radius\": 31.75", "\"blank_radius\": \"NaN\"").ok());
    CHECK_FALSE(loadWith("\"along_x\": false", "\"along_x\": 3").ok());
    CHECK_FALSE(loadWith("\"rounding\"", "\"turning\"").ok());
}
