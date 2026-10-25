// Tests for the shipped starter tool database data/tools/starter_tools.json (T-0110 AC3).
// The path comes from CMake (CARVETTO_STARTER_TOOLS) so the tests run from any build directory.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/json_io.hpp"

using namespace carvetto::tooling;

namespace {

constexpr const char* kMachine = "machine-generic-router";
constexpr const char* kSoftwood = "material-softwood";
constexpr const char* kHardwood = "material-hardwood-mdf";
constexpr const char* kAcrylic = "material-acrylic";
constexpr const char* kAluminium = "material-aluminium";

std::string starterText() {
    std::ifstream in(CARVETTO_STARTER_TOOLS, std::ios::binary);
    if (!in) {
        FAIL("cannot open " << CARVETTO_STARTER_TOOLS);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

ToolDatabase starter() {
    ImportResult r = importToolDatabaseJson(starterText());
    if (!r.ok) {
        FAIL("starter_tools.json failed to import: " << r.error);
    }
    return std::move(r.database);
}

/// Sane chip-load band (mm per tooth) per material for router cutters: below it the cutter rubs
/// and burns, above it a light machine or small cutter is overloaded.
struct Band {
    double minChip;
    double maxChip;
    double maxFeed;  ///< mm/min, a conservative ceiling for a hobby router
};

const std::map<std::string, Band>& bands() {
    static const std::map<std::string, Band> b{
        {kSoftwood, {0.01, 0.20, 3000.0}},
        {kHardwood, {0.01, 0.18, 2500.0}},
        {kAcrylic, {0.01, 0.20, 2000.0}},
        {kAluminium, {0.005, 0.08, 1000.0}},
    };
    return b;
}

bool approxEq(double a, double b) {
    return std::fabs(a - b) < 1e-6;
}

/// The tool of the given type and diameter (mm), or nullptr.
const Tool* findBy(const ToolDatabase& db, ToolType type, double diameter, double angleDeg = -1.0) {
    for (const auto& [id, t] : db.tools()) {
        if (t.type == type && approxEq(t.diameter, diameter) &&
            (angleDeg < 0.0 || approxEq(t.includedAngle * 180.0 / carvetto::geometry::kPi, angleDeg))) {
            return &t;
        }
    }
    return nullptr;
}

}  // namespace

TEST_CASE("AC3: starter tool database loads and every tool validates", "[tooling][starter]") {
    const ToolDatabase db = starter();
    CHECK(db.tools().size() >= 25);
    CHECK(db.tools().size() <= 40);
    REQUIRE(db.findMachine(kMachine) != nullptr);
    CHECK(db.findMachine(kMachine)->name == "Generic Router");
    for (const char* m : {kSoftwood, kHardwood, kAcrylic, kAluminium}) {
        INFO(m);
        CHECK(db.findMaterial(m) != nullptr);
    }
    for (const auto& [id, t] : db.tools()) {
        INFO(id);
        CHECK(!validate(t).has_value());
        CHECK(!t.name.empty());
        CHECK(db.findGroup(t.groupId) != nullptr);
    }
}

TEST_CASE("AC3: starter database exports byte-identically", "[tooling][starter]") {
    const std::string text = starterText();
    const ToolDatabase db = starter();
    CHECK(exportToolDatabaseJson(db) == text);
    const ImportResult again = importToolDatabaseJson(exportToolDatabaseJson(db));
    REQUIRE(again.ok);
    CHECK(again.database == db);
}

TEST_CASE("AC3: every starter tool has cutting data for at least one material", "[tooling][starter]") {
    const ToolDatabase db = starter();
    std::set<int> toolNumbers;
    for (const auto& [id, t] : db.tools()) {
        INFO(id);
        REQUIRE(!t.cuttingData.empty());
        const int number = t.defaultCuttingData.toolNumber;
        CHECK(number > 0);
        CHECK(toolNumbers.insert(number).second);  // unique tool numbers
        bool defaultMatchesAnEntry = false;
        for (const auto& [key, data] : t.cuttingData) {
            INFO(key.materialId);
            CHECK(key.machineId == kMachine);
            CHECK(db.findMaterial(key.materialId) != nullptr);
            CHECK(data.toolNumber == number);
            defaultMatchesAnEntry = defaultMatchesAnEntry || data == t.defaultCuttingData;
            const CuttingDataLookup lookup = t.cuttingDataFor(kMachine, key.materialId);
            CHECK(!lookup.isFallback);
        }
        CHECK(defaultMatchesAnEntry);
    }
}

TEST_CASE("Starter cutting data is conservative and plausible", "[tooling][starter]") {
    const ToolDatabase db = starter();
    for (const auto& [id, t] : db.tools()) {
        for (const auto& [key, d] : t.cuttingData) {
            INFO(id << " / " << key.materialId);
            const Band& band = bands().at(key.materialId);
            // Chip load = feed / (rpm x flutes), within the material's band.
            const double chip = chipLoad(d, t.flutes);
            INFO("chip load " << chip);
            CHECK(chip >= band.minChip);
            CHECK(chip <= band.maxChip);
            CHECK(d.feedRate <= band.maxFeed);
            CHECK(d.spindleSpeed >= 8000.0);
            CHECK(d.spindleSpeed <= 24000.0);
            CHECK(d.plungeRate > 0.0);
            CHECK(d.plungeRate <= d.feedRate);
            CHECK(d.passDepth > 0.0);
            CHECK(d.passDepth <= t.diameter + 1e-9);
            CHECK(d.passDepth <= 4.0 + 1e-9);
            CHECK(d.maxBurnRate == 0.0);
            if (t.type == ToolType::Drill) {
                CHECK(d.plungeRate == d.feedRate);
            } else {
                CHECK(d.stepover > 0.0);
                CHECK(d.stepover <= 0.5 * t.diameter + 1e-9);
            }
            if (t.type == ToolType::BallNose) {
                CHECK(d.stepover <= 0.15 * t.diameter + 1e-9);  // finishing stepover
            }
            if (key.materialId == kAluminium) {
                CHECK(d.passDepth <= 0.5 + 1e-9);
                CHECK(t.type == ToolType::EndMill);
                CHECK(chip >= 0.013);  // >= 0.0005 in: below that a small cutter rubs
            }
            if (key.materialId == kAcrylic && t.type == ToolType::EndMill) {
                CHECK(chip >= 0.025);  // >= 0.001 in: below that the cutter rubs and melts acrylic
            }
        }
    }
}

TEST_CASE("Starter database covers the tools the card asks for", "[tooling][starter]") {
    const ToolDatabase db = starter();
    const double in = 25.4;
    for (double d : {in / 8, in / 4, 3 * in / 8, in / 2, 2.0, 3.0, 4.0, 6.0, 8.0, 10.0, 12.0}) {
        INFO("end mill " << d);
        const Tool* t = findBy(db, ToolType::EndMill, d);
        REQUIRE(t != nullptr);
        // Flat end mills carry data for every starter material.
        for (const char* m : {kSoftwood, kHardwood, kAcrylic, kAluminium}) {
            INFO(m);
            CHECK(t->findCuttingData(kMachine, m) != nullptr);
        }
    }
    for (double d : {in / 8, in / 4, 3.0, 6.0}) {
        INFO("ball nose " << d);
        CHECK(findBy(db, ToolType::BallNose, d) != nullptr);
    }
    for (double angle : {60.0, 90.0, 120.0}) {
        for (double d : {in / 4, in / 2}) {
            INFO("V-bit " << angle << " " << d);
            CHECK(findBy(db, ToolType::VBit, d, angle) != nullptr);
        }
    }
    bool engraving = false;
    bool tapered = false;
    int drills = 0;
    for (const auto& [id, t] : db.tools()) {
        engraving = engraving || t.type == ToolType::Engraving;
        tapered = tapered || t.type == ToolType::TaperedBallNose;
        drills += t.type == ToolType::Drill ? 1 : 0;
    }
    CHECK(engraving);
    CHECK(tapered);
    CHECK(drills >= 1);
}

// Hidden test (tag [.regen], never run by ctest): rewrites data/tools/starter_tools.json in the
// canonical form produced by the exporter (sorted keys, id order, rounded numbers), so hand edits
// to the file become byte-stable. Run through tools/regen_golden.py and review the diff.
TEST_CASE("Regenerate canonical starter tool database", "[.regen]") {
    const std::string text = exportToolDatabaseJson(starter());
    std::ofstream out(CARVETTO_STARTER_TOOLS, std::ios::binary);
    REQUIRE(out);
    out << text;
}
