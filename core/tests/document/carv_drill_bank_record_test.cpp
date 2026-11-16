// T-0328: drill bank toolpaths as native toolpath records (ADR-0106; ADR-0021 schema): the DrillBankParams variant
// alternative, its job.json params, exact round trips, the 1.17 format minor, and refusal of damaged params.

#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/drill_bank.hpp"

using namespace carvetto;
using namespace carvetto::document;
using Bytes = std::vector<std::uint8_t>;

namespace {

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

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

std::string jobText(const Bytes& archive) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (const auto& m : r.members) {
        if (m.name == "job.json") {
            return {m.data.begin(), m.data.end()};
        }
    }
    FAIL("no job.json");
    return {};
}

Bytes withJob(const Bytes& archive, const std::string& job) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (auto& m : r.members) {
        if (m.name == "job.json") {
            m.data.assign(job.begin(), job.end());
        }
    }
    auto w = detail::writeZip(r.members);
    REQUIRE(w.ok);
    return w.bytes;
}

std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const std::size_t pos = text.find(from);
    INFO("looking for: " << from);
    REQUIRE(pos != std::string::npos);
    text.replace(pos, from.size(), to);
    return text;
}

/// Drill bank parameters with every value away from its default, with awkward doubles.
toolpath::DrillBankParams fullBank() {
    toolpath::DrillBankParams p;
    p.name = "Hinge bank \"A\" \xC3\xA9";
    p.startDepth = 0.30000000000000004;
    p.cutDepth = 9.7;
    p.safeZ = 12.000000000000002;
    p.peck = true;
    p.peckRetract = toolpath::PeckRetract::AbovePreviousPass;
    p.retractGap = 0.75;
    p.dwellTime = 0.1;
    p.order = {true, false, true, true};
    p.startPosition = {-10.5, -0.0};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    p.horizontalDrills = 3;
    p.verticalDrills = 2;
    p.pitch = 32.00000000000001;
    p.rowOffset = -4.5;
    p.columnOffset = 7.25;
    p.referenceDrill = 4;
    p.inactiveSpindles = {2, 5};
    p.spindleDiameters = {5.0, 5.0, 8.0, 8.0, 3.0000000000000004};
    p.spindleDepths = {12.0, 0.0, 0.1, 9.5, 11.0};
    p.matchDiameters = true;
    p.pitchTolerance = 0.30000000000000004;
    p.diameterTolerance = 0.05;
    p.minHoles = 3;
    p.unmatched = toolpath::BankUnmatched::Skip;
    return p;
}

tooling::Tool drill() {
    tooling::Tool t;
    t.id = "drill-3mm";
    t.name = "Drill 3 mm";
    t.type = tooling::ToolType::Drill;
    t.diameter = 3.0;
    t.includedAngle = 2.0594885173533086;
    t.defaultCuttingData = {2.5, 0.0, 0.0, 0.0, 10000.0, 600.0, 250.0, 0.0, 2};
    return t;
}

/// A document with a computed drill bank record: two rows of three holes at the pitch.
Document bankDocument(const toolpath::DrillBankParams& params) {
    Document doc;
    ObjectId first{};
    std::vector<geometry::Contour> contours;
    for (int r = 0; r < 2; ++r) {
        for (int i = 0; i < 3; ++i) {
            geometry::Contour c = geometry::Contour::circle({10.0 + 32.0 * i, 10.0 + 100.0 * r}, 1.5);
            const ObjectId id = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
            if (r == 0 && i == 0) {
                first = id;
            }
            contours.push_back(c);
        }
    }
    toolpath::DrillBankParams used = params;
    used.horizontalDrills = 3;
    used.verticalDrills = 0;
    used.pitch = 32.0;
    used.inactiveSpindles.clear();
    used.spindleDiameters.clear();
    used.spindleDepths.clear();
    used.referenceDrill = 1;
    used.matchDiameters = false;
    used.minHoles = 3;
    used.pitchTolerance = 0.1;
    toolpath::DrillBankResult r = toolpath::drillBankToolpath(contours, drill(), std::nullopt, used);
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {first};
    rec.selection.layers = {doc.defaultLayer()};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"drill-3mm", "machine-generic-router", "material-hardwood-mdf"};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

std::string paramsObject(const std::string& job) {
    const std::size_t from = job.find("\"params\": {", job.find("\"operation\": \"drill_bank\""));
    REQUIRE(from != std::string::npos);
    return job.substr(from, job.find("\n      },", from) + 8 - from);
}

}  // namespace

TEST_CASE("T-0328: drill bank records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][drill_bank]") {
    const Document doc = bankDocument(fullBank());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::DrillBank);
    CHECK(doc.toolpaths()[0].safeZ() == 12.000000000000002);
    CHECK(doc.toolpaths()[0].moveCount() > 6);  // two pecked strokes
    const Bytes once = save(doc);
    const std::string job = jobText(once);
    CHECK(contains(job, "\"operation\": \"drill_bank\""));
    for (const char* line : {
             "\"horizontal_drills\": 3,", "\"vertical_drills\": 2", "\"pitch\": 32.00000000000001,",
             "\"row_offset\": -4.5,", "\"column_offset\": 7.25,", "\"reference_drill\": 4,",
             "\"match_diameters\": true,", "\"pitch_tolerance\": 0.30000000000000004,",
             "\"diameter_tolerance\": 0.05,", "\"min_holes\": 3,", "\"unmatched\": \"skip\",",
             "\"peck_retract\": \"above_previous_pass\",", "\"cut_depth\": 9.7,"}) {
        INFO(line);
        CHECK(contains(job, std::string("\n        ") + line));
    }
    LoadedCarv back = load(once);
    REQUIRE(back.document.toolpaths().size() == 1);
    CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(std::get<toolpath::DrillBankParams>(back.document.toolpaths()[0].params) == fullBank());
    CHECK(back.document.checkIntegrity().empty());
    CHECK(back.formatMinor == kCarvFormatMinorBankSpindles);  // T-0335: the strokes name spindles
    CHECK(kCarvFormatMinorDrillBank == 17);
    CHECK(kCarvFormatMinor >= kCarvFormatMinorBankSpindles);  // newer minors (T-0321: 1.19) are for other features
    CHECK(save(back.document, back.extras) == once);
}

TEST_CASE("T-0328: every bank option round-trips, defaults included, and the minor is 1.17 only for banks",
          "[document][toolpath][carv][drill_bank]") {
    for (const auto unmatched : {toolpath::BankUnmatched::Refuse, toolpath::BankUnmatched::Skip}) {
        for (const bool match : {false, true}) {
            toolpath::DrillBankParams p;
            p.cutDepth = 4.0;
            p.unmatched = unmatched;
            p.matchDiameters = match;
            const LoadedCarv back = load(save(bankDocument(p)));
            CHECK(std::get<toolpath::DrillBankParams>(back.document.toolpaths()[0].params) == p);
        }
    }
    // Empty lists are written as empty arrays and read back empty.
    toolpath::DrillBankParams plain;
    plain.cutDepth = 4.0;
    const std::string job = jobText(save(bankDocument(plain)));
    CHECK(contains(job, "\"inactive_spindles\": []"));
    CHECK(contains(job, "\"spindle_diameters\": []"));
    CHECK(contains(job, "\"spindle_depths\": []"));
    // A record without moves holding the default parameters.
    Document doc;
    ToolpathRecord pending;
    pending.name = "Not computed";
    pending.params = toolpath::DrillBankParams{};
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(pending)));
    const LoadedCarv back = load(save(doc));
    CHECK(back.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.document.toolpaths()[0].operation() == ToolpathOperation::DrillBank);
    // Plain drilling and an empty document do not need the new minor.
    CHECK(load(save(Document{})).formatMinor < kCarvFormatMinorDrillBank);
    toolpath::DrillParams dp;
    dp.cutDepth = 4.0;
    Document drillDoc;
    ToolpathRecord d;
    d.name = "Drill";
    d.params = dp;
    REQUIRE(drillDoc.execute(std::make_unique<AddToolpathCommand>(d)));
    CHECK(load(save(drillDoc)).formatMinor < kCarvFormatMinorDrillBank);
    // A bank record is not a drilling record.
    CHECK(std::holds_alternative<toolpath::DrillBankParams>(back.document.toolpaths()[0].params));
    CHECK_FALSE(std::holds_alternative<toolpath::DrillParams>(back.document.toolpaths()[0].params));
}

TEST_CASE("T-0328: bank params keys are optional, unknown keys dropped, bad values refused",
          "[document][toolpath][carv][drill_bank][errors]") {
    toolpath::DrillBankParams p;
    p.cutDepth = 4.0;
    p.inactiveSpindles = {2};
    p.spindleDiameters = {3, 3, 3, 3};
    const Document doc = bankDocument(p);
    const Bytes good = save(doc);
    const std::string job = jobText(good);
    const std::string object = paramsObject(job);

    const LoadedCarv empty = load(withJob(good, replaced(job, object, "\"params\": {}")));
    CHECK(std::get<toolpath::DrillBankParams>(empty.document.toolpaths()[0].params) == toolpath::DrillBankParams{});

    const LoadedCarv extra = load(withJob(good, replaced(job, "\"min_holes\": 2", "\"min_holes\": 2, \"zz_future\": [1]")));
    CHECK(extra.document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK_FALSE(contains(jobText(save(extra.document, extra.extras)), "zz_"));

    struct Bad {
        std::string from;
        std::string to;
        std::string message;
    };
    const std::vector<Bad> bad{
        {"\"unmatched\": \"refuse\"", "\"unmatched\": \"ignore\"", "job.json.toolpaths[0].params.unmatched"},
        {"\"match_diameters\": false", "\"match_diameters\": 0", "job.json.toolpaths[0].params.match_diameters"},
        {"\"pitch\": 32.0", "\"pitch\": \"NaN\"", "job.json.toolpaths[0].params.pitch: must be finite"},
        {"\"pitch_tolerance\": 0.1", "\"pitch_tolerance\": \"Infinity\"",
         "job.json.toolpaths[0].params.pitch_tolerance: must be finite"},
        {"\"horizontal_drills\": 4", "\"horizontal_drills\": 1.5", "job.json.toolpaths[0].params.horizontal_drills"},
        {"\"horizontal_drills\": 4", "\"horizontal_drills\": 5000", "job.json.toolpaths[0].params.horizontal_drills"},
        {"\"vertical_drills\": 0", "\"vertical_drills\": -1", "job.json.toolpaths[0].params.vertical_drills"},
        {"\"min_holes\": 2", "\"min_holes\": \"2\"", "job.json.toolpaths[0].params.min_holes"},
    };
    for (const Bad& b : bad) {
        INFO(b.to);
        const CarvError e = loadError(withJob(good, replaced(job, b.from, b.to)));
        CHECK(e.code == CarvError::Code::InvalidContent);
        CHECK(contains(e.message, b.message));
    }
}

TEST_CASE("T-0328: hostile spindle lists are refused", "[document][toolpath][carv][drill_bank][errors]") {
    toolpath::DrillBankParams p;
    p.cutDepth = 4.0;
    const Document doc = bankDocument(p);
    const Bytes good = save(doc);
    const std::string job = jobText(good);
    const std::string object = paramsObject(job);
    const auto with = [&](const std::string& key, const std::string& value) {
        // Replace the whole array member by `key: value`.
        const std::size_t at = object.find("\"" + key + "\": []");
        REQUIRE(at != std::string::npos);
        std::string changed = object;
        changed.replace(at, key.size() + 6, "\"" + key + "\": " + value);
        return withJob(good, replaced(job, object, changed));
    };
    // Valid lists load.
    CHECK(load(with("inactive_spindles", "[1, 3]")).document.toolpaths()[0].params ==
          [&] {
              toolpath::DrillBankParams q = p;
              q.inactiveSpindles = {1, 3};
              return ToolpathParams{q};
          }());
    const std::string big = [] {
        std::string s = "[";
        for (int i = 0; i < 65; ++i) {
            s += i == 0 ? "1" : ", 1";
        }
        return s + "]";
    }();
    for (const std::string key : {"inactive_spindles", "spindle_diameters", "spindle_depths"}) {
        INFO(key);
        const CarvError tooMany = loadError(with(key, big));
        CHECK(tooMany.code == CarvError::Code::InvalidContent);
        CHECK(contains(tooMany.message, "params." + key));
        CHECK(contains(tooMany.message, "too many spindles"));
        const CarvError notArray = loadError(with(key, "7"));
        CHECK(contains(notArray.message, "params." + key));
        CHECK(contains(notArray.message, "must be a JSON array"));
        const CarvError text = loadError(with(key, "[\"a\"]"));
        CHECK(contains(text.message, "params." + key + "[0]"));
    }
    CHECK(contains(loadError(with("spindle_diameters", "[3, \"NaN\"]")).message, "spindle_diameters[1]: must be finite"));
    CHECK(contains(loadError(with("spindle_depths", "[\"Infinity\"]")).message, "spindle_depths[0]: must be finite"));
    CHECK(contains(loadError(with("inactive_spindles", "[1.5]")).message, "inactive_spindles[0]"));
    CHECK(contains(loadError(with("inactive_spindles", "[5000]")).message, "inactive_spindles[0]"));
    // Exactly 64 entries are accepted by the reader (the toolpath checks fit the head).
    std::string ok64 = "[";
    for (int i = 0; i < 64; ++i) {
        ok64 += i == 0 ? "3" : ", 3";
    }
    ok64 += "]";
    const LoadedCarv sixtyFour = load(with("spindle_diameters", ok64));
    CHECK(std::get<toolpath::DrillBankParams>(sixtyFour.document.toolpaths()[0].params).spindleDiameters.size() == 64);
}

TEST_CASE("T-0328: bank records go through the toolpath commands with undo", "[document][toolpath][drill_bank][undo]") {
    Document doc = bankDocument(fullBank());
    ToolpathRecord edit = doc.toolpaths()[0];
    std::get<toolpath::DrillBankParams>(edit.params).minHoles = 2;
    REQUIRE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    CHECK(std::get<toolpath::DrillBankParams>(doc.toolpaths()[0].params).minHoles == 2);
    REQUIRE(doc.undo());
    CHECK(std::get<toolpath::DrillBankParams>(doc.toolpaths()[0].params) == fullBank());
    // Non-finite parameters are refused by the commands (nonFiniteParam names them).
    std::get<toolpath::DrillBankParams>(edit.params).pitch = std::numeric_limits<double>::infinity();
    CHECK(nonFiniteParam(edit.params) == "pitch");
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    edit = doc.toolpaths()[0];
    std::get<toolpath::DrillBankParams>(edit.params).spindleDiameters[3] = std::numeric_limits<double>::quiet_NaN();
    CHECK(nonFiniteParam(edit.params) == "spindle_diameters[3]");
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    edit = doc.toolpaths()[0];
    std::get<toolpath::DrillBankParams>(edit.params).spindleDepths[0] = std::numeric_limits<double>::infinity();
    CHECK(nonFiniteParam(edit.params) == "spindle_depths[0]");
    edit = doc.toolpaths()[0];
    std::get<toolpath::DrillBankParams>(edit.params).diameterTolerance = std::numeric_limits<double>::infinity();
    CHECK(nonFiniteParam(edit.params) == "diameter_tolerance");
    edit = doc.toolpaths()[0];
    std::get<toolpath::DrillBankParams>(edit.params).tolerance.chordal = std::numeric_limits<double>::infinity();
    CHECK_FALSE(nonFiniteParam(edit.params).empty());
}
