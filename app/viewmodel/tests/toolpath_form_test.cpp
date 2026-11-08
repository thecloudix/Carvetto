// T-0220: the form contracts of the five toolpath operations (fields, parsing, units, rejection
// without side effect, tool choice with the tool database).

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "toolpath_test_support.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;
using namespace carvetto::viewmodel::testing;
using document::ToolpathOperation;
using document::Units;

namespace {

const ToolpathOperation kAllOps[] = {ToolpathOperation::Profile, ToolpathOperation::Pocket, ToolpathOperation::Drill,
                                     ToolpathOperation::Engrave, ToolpathOperation::VCarving};

const ToolpathField* find(const std::vector<ToolpathField>& fields, const std::string& id) {
    for (const ToolpathField& f : fields) {
        if (f.id == id) {
            return &f;
        }
    }
    return nullptr;
}

const ToolpathChoice* findChoice(const std::vector<ToolpathChoice>& choices, const std::string& id) {
    for (const ToolpathChoice& c : choices) {
        if (c.id == id) {
            return &c;
        }
    }
    return nullptr;
}

}  // namespace

TEST_CASE("form: every operation has fields and choices with unique ids", "[toolpath][form]") {
    for (const ToolpathOperation op : kAllOps) {
        ToolpathForm form(op, Units::Millimetres);
        const auto ids = form.allFieldIds();
        const auto cids = form.allChoiceIds();
        CHECK(ids.size() >= 6);
        CHECK(cids.size() >= 2);
        std::set<std::string> seen;
        for (const auto& id : ids) {
            CHECK(seen.insert(id).second);
        }
        for (const auto& id : cids) {
            CHECK(seen.insert(id).second);
        }
        // Visible fields are a subset of all fields.
        for (const ToolpathField& f : form.fields()) {
            CHECK(std::find(ids.begin(), ids.end(), f.id) != ids.end());
            CHECK_FALSE(f.label.empty());
            CHECK_FALSE(f.text.empty());
        }
    }
}

TEST_CASE("form: defaults show the core's defaults and map 1:1 onto the params", "[toolpath][form]") {
    ToolpathForm form(ToolpathOperation::Profile, Units::Millimetres);
    const toolpath::ProfileParams defaults;
    CHECK(form.definition().params == document::ToolpathParams{defaults});
    CHECK(find(form.fields(), "safeZ")->text == "5");
    CHECK_FALSE(find(form.fields(), "safeZ")->typed);
    REQUIRE(form.setField("safeZ", "7.5").ok);
    CHECK(find(form.fields(), "safeZ")->typed);
    CHECK(std::get<toolpath::ProfileParams>(form.definition().params).safeZ == 7.5);
    // An empty entry resets the field.
    REQUIRE(form.setField("safeZ", "  ").ok);
    CHECK(std::get<toolpath::ProfileParams>(form.definition().params).safeZ == 5.0);
    CHECK_FALSE(find(form.fields(), "safeZ")->typed);
}

TEST_CASE("form: fields appear and disappear with the choices", "[toolpath][form]") {
    ToolpathForm form(ToolpathOperation::Profile, Units::Millimetres);
    CHECK(find(form.fields(), "tabLength") == nullptr);
    CHECK(findChoice(form.choices(), "tabSpacing") == nullptr);
    REQUIRE(form.setChoice("tabsEnabled", 1).ok);
    CHECK(find(form.fields(), "tabLength") != nullptr);
    CHECK(find(form.fields(), "tabCount") != nullptr);
    CHECK(find(form.fields(), "tabDistance") == nullptr);
    REQUIRE(form.setChoice("tabSpacing", 1).ok);
    CHECK(find(form.fields(), "tabCount") == nullptr);
    CHECK(find(form.fields(), "tabDistance") != nullptr);
    CHECK(find(form.fields(), "leadLength") == nullptr);
    REQUIRE(form.setChoice("leadType", 1).ok);
    CHECK(find(form.fields(), "leadLength") != nullptr);
    CHECK(find(form.fields(), "leadRadius") == nullptr);
    REQUIRE(form.setChoice("leadType", 2).ok);
    CHECK(find(form.fields(), "leadRadius") != nullptr);
    CHECK(find(form.fields(), "leadSweep") != nullptr);
    CHECK(find(form.fields(), "rampDistance") == nullptr);
    REQUIRE(form.setChoice("rampType", 2).ok);
    CHECK(find(form.fields(), "rampDistance") != nullptr);
    CHECK(find(form.fields(), "lastPassAllowance") == nullptr);
    REQUIRE(form.setChoice("separateLastPass", 1).ok);
    CHECK(find(form.fields(), "lastPassAllowance") != nullptr);

    ToolpathForm pocket(ToolpathOperation::Pocket, Units::Millimetres);
    CHECK(find(pocket.fields(), "rasterAngle") == nullptr);
    REQUIRE(pocket.setChoice("strategy", 1).ok);
    CHECK(find(pocket.fields(), "rasterAngle") != nullptr);

    ToolpathForm engrave(ToolpathOperation::Engrave, Units::Millimetres);
    CHECK(find(engrave.fields(), "stepover") == nullptr);
    CHECK(find(engrave.fields(), "startX") != nullptr);
    REQUIRE(engrave.setChoice("strategy", 1).ok);
    CHECK(find(engrave.fields(), "stepover") != nullptr);
    CHECK(find(engrave.fields(), "startX") == nullptr);
}

TEST_CASE("form: units, suffixes and angles", "[toolpath][form]") {
    ToolpathForm form(ToolpathOperation::Profile, Units::Inches);
    REQUIRE(form.setField("cutDepth", "0.25").ok);
    CHECK(std::get<toolpath::ProfileParams>(form.definition().params).passes.cutDepth == 0.25 * 25.4);
    CHECK(find(form.fields(), "cutDepth")->text == "0.25");
    REQUIRE(form.setField("safeZ", "10mm").ok);  // an explicit suffix wins
    CHECK(std::get<toolpath::ProfileParams>(form.definition().params).safeZ == 10.0);
    CHECK(find(form.fields(), "safeZ")->text == "0.3937");
    form.setUnits(Units::Millimetres);
    CHECK(find(form.fields(), "safeZ")->text == "10");

    REQUIRE(form.setChoice("rampType", 1).ok);
    REQUIRE(form.setChoice("rampUseAngle", 1).ok);
    REQUIRE(form.setField("rampAngle", "10").ok);
    CHECK(std::get<toolpath::ProfileParams>(form.definition().params).ramp.angle == 10.0 * (geometry::kPi / 180.0));
    CHECK(find(form.fields(), "rampAngle")->text == "10");
    REQUIRE(form.setField("rampAngle", "12.5deg").ok);
    CHECK(find(form.fields(), "rampAngle")->text == "12.5");
}

TEST_CASE("form: bad entries are rejected with a message and no side effect", "[toolpath][form]") {
    for (const ToolpathOperation op : kAllOps) {
        ToolpathForm form(op, Units::Millimetres);
        form.setChoice(op == ToolpathOperation::Engrave ? "strategy" : "orderGrid", 1);
        const document::ToolpathParams before = form.definition().params;
        const char* bad[] = {"abc", "nan", "NaN", "inf", "-inf", "1e999", "1e10", "--1", "0x10", "1,5", "1 2", "mm"};
        for (const std::string& id : form.allFieldIds()) {
            for (const char* text : bad) {
                const FieldResult r = form.setField(id, text);
                INFO(id << " <- " << text);
                CHECK_FALSE(r.ok);
                CHECK_FALSE(r.message.empty());
                CHECK(form.definition().params == before);
            }
        }
        // Negative and zero where the field forbids them.
        for (const ToolpathField& f : form.fields()) {
            if (!f.allowNegative) {
                INFO(f.id);
                CHECK_FALSE(form.setField(f.id, "-1").ok);
                CHECK(form.definition().params == before);
            }
            if (!f.allowZero) {
                INFO(f.id);
                CHECK_FALSE(form.setField(f.id, "0").ok);
                CHECK(form.definition().params == before);
            }
        }
        CHECK_FALSE(form.setField("noSuchField", "1").ok);
        CHECK_FALSE(form.setChoice("noSuchChoice", 0).ok);
        for (const std::string& id : form.allChoiceIds()) {
            CHECK_FALSE(form.setChoice(id, -1).ok);
            CHECK_FALSE(form.setChoice(id, 99).ok);
            CHECK(form.definition().params == before);
        }
    }
}

TEST_CASE("form: ranges of the angle, count and tolerance fields", "[toolpath][form]") {
    ToolpathForm profile(ToolpathOperation::Profile, Units::Millimetres);
    REQUIRE(profile.setChoice("leadType", 2).ok);
    // The core and the CLI bound neither the sweep nor the angles; only sign and magnitude are checked.
    CHECK(profile.setField("leadSweep", "0.05").ok);
    CHECK(profile.setField("leadSweep", "120").ok);
    CHECK_FALSE(profile.setField("leadSweep", "0").ok);
    CHECK_FALSE(profile.setField("leadSweep", "-5").ok);
    CHECK_FALSE(profile.setField("chordal", "0.00001").ok);
    CHECK_FALSE(profile.setField("chordal", "1.5").ok);
    CHECK(profile.setField("chordal", "0.02").ok);
    REQUIRE(profile.setChoice("tabsEnabled", 1).ok);
    CHECK_FALSE(profile.setField("tabCount", "0").ok);
    CHECK_FALSE(profile.setField("tabCount", "2.5").ok);
    CHECK_FALSE(profile.setField("tabCount", "1001").ok);
    CHECK(profile.setField("tabCount", "12").ok);
    CHECK(std::get<toolpath::ProfileParams>(profile.definition().params).tabs.count == 12);

    ToolpathForm engrave(ToolpathOperation::Engrave, Units::Millimetres);
    CHECK_FALSE(engrave.setField("passes", "0").ok);
    CHECK_FALSE(engrave.setField("passes", "1001").ok);
    CHECK(engrave.setField("passes", "1000").ok);

    ToolpathForm pocket(ToolpathOperation::Pocket, Units::Millimetres);
    REQUIRE(pocket.setChoice("strategy", 1).ok);
    CHECK(pocket.setField("rasterAngle", "-30").ok);
    CHECK_FALSE(pocket.setField("rasterAngle", "400000").ok);

    ToolpathForm drill(ToolpathOperation::Drill, Units::Millimetres);
    CHECK(drill.setField("dwellTime", "0.5").ok);
    CHECK_FALSE(drill.setField("dwellTime", "-1").ok);
    CHECK_FALSE(drill.setField("dwellTime", "1 s").ok);
    CHECK_FALSE(drill.setField("dwellTime", "+-1").ok);
    CHECK_FALSE(drill.setField("startX", "+-1").ok);  // the shared length parser, a field allowing negatives
    CHECK_FALSE(drill.setField("startX", "++1").ok);
    CHECK_FALSE(drill.setField("startX", "+").ok);
    CHECK(drill.setField("startX", "+1").ok);
    CHECK(drill.setField("startX", "-1").ok);
    CHECK_FALSE(drill.setField("dwellTime", "++1").ok);
    CHECK(drill.setField("dwellTime", "+0.5").ok);
    CHECK(drill.setField("dwellTime", "0.5").ok);
    CHECK(find(drill.fields(), "dwellTime")->text == "0.5");
}

TEST_CASE("form: every field and choice can be set, with extreme values, without breaking the params", "[toolpath][form]") {
    for (const ToolpathOperation op : kAllOps) {
        ToolpathForm form(op, Units::Millimetres);
        for (const std::string& id : form.allChoiceIds()) {
            for (int i = 0; i < 12; ++i) {
                if (form.setChoice(id, i).ok) {
                    CHECK(form.setChoice(id, i).ok);
                }
            }
        }
        for (const std::string& id : form.allFieldIds()) {
            for (const char* text : {"1", "0.5", "1e8", "999999999", "-999999999", "1e-9", "0"}) {
                form.setField(id, text);
                // Whatever was accepted keeps every parameter a finite number.
                CHECK(document::nonFiniteParam(form.definition().params).empty());
            }
        }
        // And everything resets.
        form.resetValues();
        CHECK(form.definition().params == defaultDefinition(op).params);
    }
}

TEST_CASE("form: the name is checked", "[toolpath][form]") {
    ToolpathForm form(ToolpathOperation::Pocket, Units::Millimetres);
    CHECK(form.setName("Pocket 2").ok);
    CHECK(form.name() == "Pocket 2");
    CHECK_FALSE(form.setName("").ok);
    CHECK_FALSE(form.setName("a\nG0 X0").ok);
    CHECK_FALSE(form.setName(std::string("a\0b", 3)).ok);
    CHECK_FALSE(form.setName("tab\there").ok);
    CHECK_FALSE(form.setName(std::string(kMaxToolpathNameBytes + 1, 'x')).ok);
    CHECK(form.setName(std::string(kMaxToolpathNameBytes, 'x')).ok);
    CHECK(form.setName("Plaque \xC3\xA9").ok);  // UTF-8 is fine
    CHECK(form.setName("\xE2\x82\xAC 5").ok);
    CHECK_FALSE(form.setName("bad \xFF").ok);              // not UTF-8
    CHECK_FALSE(form.setName("cut \xC3").ok);              // cut short
    CHECK_FALSE(form.setName("over \xC0\x80").ok);         // overlong
    CHECK_FALSE(form.setName("sur \xED\xA0\x80").ok);     // a surrogate
    CHECK_FALSE(form.setName("big \xF4\x90\x80\x80").ok);  // beyond U+10FFFF
    CHECK_FALSE(form.setName("c1 \xC2\x85").ok);          // a C1 control (next line)
    CHECK_FALSE(form.setName("lone \x80").ok);
    CHECK(form.name() == "\xE2\x82\xAC 5");
    CHECK(validateToolpathName("ok").empty());
}

TEST_CASE("form: picking a tool from the tool database", "[toolpath][form]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Pocket, Units::Millimetres);
    CHECK(form.problems().size() >= 2);  // no tool, no depth
    REQUIRE(form.setTool(db, source("end-mill-6mm", "machine-generic-router", "material-hardwood-mdf")).ok);
    const ToolSummary& t = form.toolSummary();
    REQUIRE(t.valid);
    CHECK(t.name == db.findTool("end-mill-6mm")->name);
    CHECK(t.diameterMm == 6.0);
    const auto* entry = db.findTool("end-mill-6mm")->findCuttingData("machine-generic-router", "material-hardwood-mdf");
    REQUIRE(entry != nullptr);
    CHECK(t.data == *entry);
    CHECK_FALSE(t.fallback);

    // Rejected picks change nothing.
    const document::ToolSource before = form.tool();
    CHECK_FALSE(form.setTool(db, source("nope")).ok);
    CHECK_FALSE(form.setTool(db, source("end-mill-6mm", "machine-generic-router")).ok);
    CHECK_FALSE(form.setTool(db, source("end-mill-6mm", "no-machine", "material-hardwood-mdf")).ok);
    CHECK_FALSE(form.setTool(db, source("end-mill-6mm", "machine-generic-router", "no-material")).ok);
    CHECK_FALSE(form.setTool(db, source("")).ok);
    CHECK(form.tool() == before);
    CHECK(form.toolSummary().valid);

    // A pair without cutting data falls back to the tool's defaults, and says so.
    REQUIRE(form.setTool(db, source("v-bit-90", "machine-generic-router", "material-acrylic")).ok);
    CHECK(form.toolSummary().data == db.findTool("v-bit-90")->defaultCuttingData);
    CHECK(form.toolSummary().fallback);
}

TEST_CASE("form: the tool must suit the operation", "[toolpath][form]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm drill(ToolpathOperation::Drill, Units::Millimetres);
    CHECK_FALSE(drill.setTool(db, source("v-bit-90")).ok);
    CHECK(drill.setTool(db, source("drill-3mm")).ok);
    CHECK(drill.setTool(db, source("end-mill-3mm")).ok);

    ToolpathForm vcarve(ToolpathOperation::VCarving, Units::Millimetres);
    CHECK_FALSE(vcarve.setTool(db, source("end-mill-3mm")).ok);
    CHECK(vcarve.setTool(db, source("v-bit-90")).ok);
    CHECK_FALSE(vcarve.setClearanceTools(db, {source("v-bit-90")}).ok);
    CHECK_FALSE(vcarve.setClearanceTools(db, {source("end-mill-3mm"), source("missing")}).ok);
    CHECK(vcarve.clearanceTools().empty());  // all or nothing
    CHECK(vcarve.setClearanceTools(db, {source("end-mill-6mm"), source("end-mill-3mm")}).ok);
    CHECK(vcarve.clearanceTools().size() == 2);

    ToolpathForm pocket(ToolpathOperation::Pocket, Units::Millimetres);
    CHECK_FALSE(pocket.setClearanceTools(db, {source("end-mill-3mm")}).ok);
    CHECK(pocket.setTool(db, source("v-bit-90")).ok);  // the core judges geometry; any tool is offered

    CHECK(toolAllowedFor(ToolpathOperation::Engrave, tooling::ToolType::DiamondDrag));
    CHECK_FALSE(toolAllowedFor(ToolpathOperation::Engrave, tooling::ToolType::Drill));
}

TEST_CASE("form: values the tool database provides are shown for fields left at 0", "[toolpath][form]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Profile, Units::Millimetres);
    REQUIRE(form.setTool(db, source("end-mill-3mm")).ok);
    const ToolpathField* pass = find(form.fields(), "passDepth");
    REQUIRE(pass != nullptr);
    CHECK(pass->fromTool);
    CHECK_FALSE(pass->typed);
    const double toolDepth = db.findTool("end-mill-3mm")->defaultCuttingData.passDepth;
    CHECK(std::stod(pass->text) == toolDepth);
    REQUIRE(form.setField("passDepth", "0.75").ok);
    CHECK(find(form.fields(), "passDepth")->text == "0.75");
    CHECK(find(form.fields(), "passDepth")->typed);
    CHECK(std::get<toolpath::ProfileParams>(form.definition().params).passes.passDepth == 0.75);
}

TEST_CASE("form: problems lists what blocks a calculation", "[toolpath][form]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    ToolpathForm form(ToolpathOperation::Engrave, Units::Millimetres);
    auto problems = form.problems();
    CHECK(std::any_of(problems.begin(), problems.end(), [](const std::string& p) { return p.find("tool") != std::string::npos; }));
    CHECK(std::any_of(problems.begin(), problems.end(), [](const std::string& p) { return p.find("depth") != std::string::npos; }));
    REQUIRE(form.setTool(db, source("v-bit-90")).ok);
    REQUIRE(form.setField("depth", "0.4").ok);
    CHECK(form.problems().empty());
    // A form built from a definition shows no tool until the database has been read.
    ToolpathForm loaded(form.definition(), Units::Millimetres);
    CHECK_FALSE(loaded.problems().empty());
    loaded.refreshTools(db);
    CHECK(loaded.problems().empty());
    loaded.refreshTools(tooling::ToolDatabase{});
    CHECK_FALSE(loaded.problems().empty());
    // V-carve needs no depth.
    ToolpathForm v(ToolpathOperation::VCarving, Units::Millimetres);
    REQUIRE(v.setTool(db, source("v-bit-90")).ok);
    CHECK(v.problems().empty());
}

TEST_CASE("form: tools an operation accepts and their cutting data choices", "[toolpath][form]") {
    const tooling::ToolDatabase db = cliToolDatabase();
    const auto ids = [](const std::vector<const tooling::Tool*>& tools) {
        std::vector<std::string> out;
        for (const tooling::Tool* t : tools) {
            out.push_back(t->id);
        }
        return out;
    };
    CHECK(ids(toolsFor(db, ToolpathOperation::Pocket)) ==
          std::vector<std::string>{"drill-3mm", "end-mill-3mm", "end-mill-6mm", "v-bit-90"});
    CHECK(ids(toolsFor(db, ToolpathOperation::Drill)) ==
          std::vector<std::string>{"drill-3mm", "end-mill-3mm", "end-mill-6mm"});
    CHECK(ids(toolsFor(db, ToolpathOperation::VCarving)) == std::vector<std::string>{"v-bit-90"});
    CHECK(ids(toolsFor(db, ToolpathOperation::VCarving, true)) ==
          std::vector<std::string>{"end-mill-3mm", "end-mill-6mm"});
    CHECK(toolsFor(tooling::ToolDatabase{}, ToolpathOperation::Pocket).empty());

    const auto choices = cuttingDataChoices(*db.findTool("end-mill-6mm"));
    REQUIRE(choices.size() == 5);  // the default data and four materials
    CHECK(choices[0] == source("end-mill-6mm"));
    CHECK(choices[1].machine == "machine-generic-router");
    CHECK(choices[1].material == "material-acrylic");
    for (const document::ToolSource& c : choices) {
        ToolpathForm form(ToolpathOperation::Pocket, Units::Millimetres);
        CHECK(form.setTool(db, c).ok);
    }
    CHECK(cuttingDataChoices(*db.findTool("v-bit-90")).size() == 1);
}
