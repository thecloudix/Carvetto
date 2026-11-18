#include "carvetto/viewmodel/preferences.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <filesystem>
#include <set>

using namespace carvetto::viewmodel;
using carvetto::document::Units;
using carvetto::simulate::Quality;

namespace {

bool hasIssue(const std::vector<PrefIssue>& issues, const std::string& key) {
    for (const PrefIssue& i : issues) {
        if (i.key == key) {
            return true;
        }
    }
    return false;
}

Preferences custom() {
    Preferences p;
    p.defaultUnits = Units::Inches;
    p.language = "en";
    p.undoDepth = 50;
    p.autosaveEnabled = false;
    p.autosaveMinutes = 17;
    p.showGrid = false;
    p.gridSpacingMm = 2.5;
    p.snapEnabled = false;
    p.snapTolerancePx = 7.5;
    p.canvasBackground = 0x102030;
    p.materialFill = 0xF0E0D0;
    p.antialiasing = false;
    p.showRulers = false;
    p.openFolder = "/tmp/open";
    p.saveFolder = "/tmp/save";
    p.exportFolder = "/tmp/export";
    p.toolDatabasePath = "/tmp/tools.json";
    p.simulationQuality = Quality::Fine;
    return p;
}

}  // namespace

TEST_CASE("defaults are valid", "[viewmodel][preferences]") {
    CHECK(validate(Preferences{}).empty());
    CHECK(PreferencesModel::defaults() == Preferences{});
}

TEST_CASE("every preference round-trips through the key/value form", "[viewmodel][preferences]") {
    const Preferences p = custom();
    REQUIRE(validate(p).empty());
    std::vector<PrefIssue> issues;
    CHECK(deserialize(serialize(p), &issues) == p);
    CHECK(issues.empty());
    CHECK(deserialize(serialize(Preferences{})) == Preferences{});
}

TEST_CASE("the field list matches the serialised keys exactly", "[viewmodel][preferences]") {
    std::set<std::string> listed;
    for (const PrefField& f : preferenceFields()) {
        CHECK(listed.insert(f.key).second);
    }
    std::set<std::string> stored;
    for (const auto& kv : serialize(Preferences{})) {
        if (kv.first != "version") {
            stored.insert(kv.first);
        }
    }
    CHECK(listed == stored);
}

TEST_CASE("out-of-range values are refused", "[viewmodel][preferences]") {
    Preferences p;
    p.undoDepth = 5;
    p.autosaveMinutes = 0;
    p.gridSpacingMm = 0.0;
    p.snapTolerancePx = 99.0;
    p.language = "klingon";
    p.canvasBackground = 0x1000000;
    auto issues = validate(p);
    CHECK(hasIssue(issues, "editing/undoDepth"));
    CHECK(hasIssue(issues, "editing/autosaveMinutes"));
    CHECK(hasIssue(issues, "grid/spacingMm"));
    CHECK(hasIssue(issues, "grid/snapTolerancePx"));
    CHECK(hasIssue(issues, "general/language"));
    CHECK(hasIssue(issues, "colours/canvasBackground"));
    Preferences q;
    q.gridSpacingMm = std::nan("");
    CHECK(hasIssue(validate(q), "grid/spacingMm"));
    Preferences same;
    same.materialFill = same.canvasBackground;
    CHECK(hasIssue(validate(same), "colours/materialFill"));
    Preferences bad;
    bad.openFolder = "a\nb";
    CHECK(hasIssue(validate(bad), "folders/open"));
}

TEST_CASE("paths are checked for existence only on request", "[viewmodel][preferences]") {
    Preferences p;
    p.openFolder = (std::filesystem::temp_directory_path() / "carvetto-no-such-dir-348").string();
    CHECK(validate(p).empty());
    PrefValidation opts;
    opts.checkPaths = true;
    CHECK(hasIssue(validate(p, opts), "folders/open"));
    p.openFolder = std::filesystem::temp_directory_path().string();
    CHECK(validate(p, opts).empty());
    p.toolDatabasePath = (std::filesystem::temp_directory_path() / "carvetto-no-such-dir-348" / "t.json").string();
    CHECK(hasIssue(validate(p, opts), "tools/databasePath"));
}

TEST_CASE("the model refuses invalid values and keeps the old ones", "[viewmodel][preferences]") {
    PreferencesModel model;
    int fired = 0;
    model.onChanged = [&](const Preferences&) { ++fired; };
    Preferences ok = custom();
    CHECK(model.apply(ok).empty());
    CHECK(model.current() == ok);
    CHECK(fired == 1);
    CHECK(model.apply(ok).empty());  // unchanged: no notification
    CHECK(fired == 1);
    Preferences bad = ok;
    bad.undoDepth = 1;
    CHECK_FALSE(model.apply(bad).empty());
    CHECK(model.current() == ok);
    CHECK(fired == 1);
}

TEST_CASE("restore defaults resets everything", "[viewmodel][preferences]") {
    PreferencesModel model;
    REQUIRE(model.apply(custom()).empty());
    int fired = 0;
    model.onChanged = [&](const Preferences&) { ++fired; };
    model.restoreDefaults();
    CHECK(model.current() == Preferences{});
    CHECK(fired == 1);
    model.restoreDefaults();
    CHECK(fired == 1);
}

TEST_CASE("save then load restores the model", "[viewmodel][preferences]") {
    MemoryPreferencesStore store;
    PreferencesModel a;
    REQUIRE(a.apply(custom()).empty());
    CHECK(a.save(store));
    PreferencesModel b;
    CHECK(b.load(store).empty());
    CHECK(b.current() == custom());
}

TEST_CASE("bad stored values fall back field by field", "[viewmodel][preferences]") {
    PrefKeyValues kv = serialize(custom());
    for (auto& e : kv) {
        if (e.first == "editing/undoDepth") {
            e.second = "lots";
        } else if (e.first == "grid/spacingMm") {
            e.second = "-4";
        } else if (e.first == "colours/materialFill") {
            e.second = "red";
        } else if (e.first == "simulation/quality") {
            e.second = "ultra";
        }
    }
    std::vector<PrefIssue> issues;
    const Preferences p = deserialize(kv, &issues);
    CHECK(p.undoDepth == Preferences{}.undoDepth);
    CHECK(p.gridSpacingMm == Preferences{}.gridSpacingMm);
    CHECK(p.materialFill == Preferences{}.materialFill);
    CHECK(p.simulationQuality == Preferences{}.simulationQuality);
    CHECK(p.autosaveMinutes == 17);  // untouched fields survive
    CHECK(p.canvasBackground == 0x102030);
    CHECK(issues.size() == 4);
}

TEST_CASE("missing, older and newer versions load safely", "[viewmodel][preferences]") {
    CHECK(deserialize({}) == Preferences{});
    // No "version" key (version 0): keys are read as they are.
    PrefKeyValues v0 = {{"editing/undoDepth", "40"}};
    CHECK(deserialize(v0).undoDepth == 40);
    // A newer version is reported, known keys still read, unknown ones ignored.
    PrefKeyValues newer = {{"version", "99"}, {"editing/undoDepth", "40"}, {"future/key", "x"}};
    std::vector<PrefIssue> issues;
    CHECK(deserialize(newer, &issues).undoDepth == 40);
    CHECK(hasIssue(issues, "version"));
    PrefKeyValues junk = {{"version", "x"}};
    issues.clear();
    deserialize(junk, &issues);
    CHECK(hasIssue(issues, "version"));
}

TEST_CASE("colour strings", "[viewmodel][preferences]") {
    CHECK(colourToString(0x0A1B2C) == "#0a1b2c");
    std::uint32_t c = 0;
    CHECK(colourFromString("#FFaa00", c));
    CHECK(c == 0xFFAA00);
    CHECK_FALSE(colourFromString("FFaa00", c));
    CHECK_FALSE(colourFromString("#ffaa0", c));
    CHECK_FALSE(colourFromString("#gggggg", c));
}
