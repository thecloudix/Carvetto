// T-0314: toolpath groups (a calculation that makes several toolpaths, one per tool) and a 3D finish's prior
// toolpaths in the .carv record (ADR-0094): exact round trips, byte-identical re-save, the minor-version bump
// only when used, old files (no group) unchanged, and hostile group/prior content refused.

#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"

using namespace carvetto;
using namespace carvetto::document;

namespace {

using Bytes = std::vector<std::uint8_t>;

ToolpathRecord recordWith(ToolpathParams params, const std::string& name = "t") {
    ToolpathRecord rec;
    rec.name = name;
    rec.params = std::move(params);
    rec.selection = VectorSelection::allVectors();
    rec.tool.name = "em";
    rec.tool.diameter = 6.0;
    rec.tool.toolNumber = 1;
    rec.tool.feedRate = 1500.0;
    rec.tool.plungeRate = 500.0;
    rec.tool.spindleSpeed = 18000.0;
    rec.toolSource = {"em", "", ""};
    std::vector<toolpath::Move> moves;
    moves.push_back(toolpath::Move::rapid({0, 0, 5}));
    moves.push_back(toolpath::Move::plunge({0, 0, -1.5}));
    moves.push_back(toolpath::Move::feed({10, 0, -1.5}));
    rec.moves = shareMoves(std::move(moves));
    return rec;
}

ToolGroup groupOf(std::uint64_t id, std::uint32_t role) {
    ToolGroup g;
    g.id = id;
    g.role = role;
    g.tools = {ToolSource{"big", "m1", "wood"}, ToolSource{"mid", "", ""}, ToolSource{"small", "m1", "mdf"}};
    return g;
}

Bytes save(const Document& doc) {
    auto saved = saveCarv(doc, CarvExtras{});
    const std::string why = saved.ok() ? std::string() : saved.error().message;
    INFO(why);
    REQUIRE(saved.ok());
    return saved.value();
}

std::string memberText(const Bytes& archive, const std::string& name) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (const auto& m : r.members) {
        if (m.name == name) {
            return {m.data.begin(), m.data.end()};
        }
    }
    FAIL("member " << name << " not found");
    return {};
}

Bytes withJob(const Bytes& archive, const std::string& job) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (auto& m : r.members) {
        if (m.name == "job.json") {
            m.data = Bytes(job.begin(), job.end());
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

Document threeToolDoc() {
    Document doc;
    toolpath::Rough3dParams rough;
    for (std::uint32_t role = 0; role < 3; ++role) {
        ToolpathRecord rec = recordWith(rough, "Rough [t" + std::to_string(role) + "]");
        rec.group = groupOf(7, role);
        rec.toolSource = rec.group.tools[role];
        REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(std::move(rec))));
    }
    return doc;
}

}  // namespace

TEST_CASE("T-0314: grouped records and a finish's priors round trip exactly; re-save is byte-identical",
          "[document][toolpath][carv][3d][group]") {
    Document doc = threeToolDoc();
    toolpath::Finish3dParams finish;
    finish.priorToolpaths = {1, 3, 2};  // order is kept
    ToolpathRecord fin = recordWith(finish, "Finish");
    fin.group = groupOf(9, 2);
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(std::move(fin))));
    toolpath::MouldingParams moulding;
    ToolpathRecord mould = recordWith(moulding, "Moulding [Clear]");
    mould.group = ToolGroup{11, 0, {ToolSource{"clear", "", ""}, ToolSource{"main", "", ""}}};
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(std::move(mould))));

    const Bytes saved = save(doc);
    auto back = loadCarv(std::span<const std::uint8_t>(saved));
    REQUIRE(back.ok());
    const auto& recs = back.value().document.toolpaths();
    REQUIRE(recs.size() == 5);
    CHECK(recs == doc.toolpaths());
    CHECK(recs[1].group == groupOf(7, 1));
    CHECK(recs[2].group.role == 2);
    CHECK(recs[2].group.tools[0] == ToolSource{"big", "m1", "wood"});
    CHECK(std::get<toolpath::Finish3dParams>(recs[3].params).priorToolpaths == std::vector<std::uint64_t>{1, 3, 2});
    CHECK(recs[4].group.tools.size() == 2);
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved);
}

TEST_CASE("T-0314: the minor version is bumped only when a group or a prior list is stored",
          "[document][toolpath][carv][group]") {
    Document plain;
    REQUIRE(plain.execute(std::make_unique<AddToolpathCommand>(recordWith(toolpath::Rough3dParams{}))));
    const Bytes old = save(plain);
    CHECK(memberText(old, "manifest.json").find("\"minor\": 11") == std::string::npos);
    CHECK(memberText(old, "job.json").find("\"group\"") == std::string::npos);
    CHECK(memberText(old, "job.json").find("prior_toolpaths") == std::string::npos);
    auto loadedOld = loadCarv(std::span<const std::uint8_t>(old));  // an old file: no group, defaults
    REQUIRE(loadedOld.ok());
    CHECK(loadedOld.value().document.toolpaths()[0].group == ToolGroup{});
    CHECK(loadedOld.value().formatMinor < kCarvFormatMinorToolGroups);

    const Document grouped = threeToolDoc();
    const Bytes withGroup = save(grouped);
    CHECK(memberText(withGroup, "manifest.json").find("\"minor\": 11") != std::string::npos);
    auto loaded = loadCarv(std::span<const std::uint8_t>(withGroup));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorToolGroups);

    Document priors;
    toolpath::Finish3dParams finish;
    finish.priorToolpaths = {5};
    REQUIRE(priors.execute(std::make_unique<AddToolpathCommand>(recordWith(finish))));
    CHECK(memberText(save(priors), "manifest.json").find("\"minor\": 11") != std::string::npos);
}

TEST_CASE("T-0314: the commands and the writer refuse an inconsistent group or too many priors",
          "[document][toolpath][carv][group]") {
    Document doc;
    ToolpathRecord badRole = recordWith(toolpath::Rough3dParams{});
    badRole.group = groupOf(1, 3);  // three tools: roles 0..2
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(badRole)));
    ToolpathRecord noTools = recordWith(toolpath::Rough3dParams{});
    noTools.group = ToolGroup{1, 0, {}};
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(noTools)));
    ToolpathRecord stray = recordWith(toolpath::Rough3dParams{});
    stray.group.tools = {ToolSource{"a", "", ""}};  // tools without an id
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(stray)));
    ToolpathRecord tooMany = recordWith(toolpath::Rough3dParams{});
    tooMany.group.id = 1;
    tooMany.group.tools.assign(kMaxGroupTools + 1, ToolSource{"a", "", ""});
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(tooMany)));
    toolpath::Finish3dParams finish;
    finish.priorToolpaths.assign(257, 1);
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(recordWith(finish))));
    CHECK(doc.toolpaths().empty());
    ToolpathRecord ok = recordWith(toolpath::Rough3dParams{});
    ok.group = groupOf(1, 2);
    CHECK(doc.execute(std::make_unique<AddToolpathCommand>(ok)));
    CHECK(groupProblem(ok).empty());
    CHECK_FALSE(groupProblem(badRole).empty());
}

TEST_CASE("T-0314: a hostile group or prior list in job.json is refused by the loader",
          "[document][toolpath][carv][group]") {
    Document doc;
    ToolpathRecord rec = recordWith(toolpath::Finish3dParams{});
    rec.group = groupOf(4, 1);
    toolpath::Finish3dParams f;
    f.priorToolpaths = {2};
    rec.params = f;
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    const Bytes saved = save(doc);
    const std::string job = memberText(saved, "job.json");
    const auto loadWith = [&](const std::string& edited) {
        return loadCarv(std::span<const std::uint8_t>(withJob(saved, edited)));
    };
    REQUIRE(loadWith(job).ok());  // the unedited text loads

    CHECK_FALSE(loadWith(replaced(job, "\"role\": 1", "\"role\": 3")).ok());          // role beyond the tools
    CHECK_FALSE(loadWith(replaced(job, "\"role\": 1", "\"role\": -1")).ok());         // negative
    CHECK_FALSE(loadWith(replaced(job, "\"role\": 1", "\"role\": 1.5")).ok());        // not an integer
    CHECK_FALSE(loadWith(replaced(job, "\"role\": 1", "\"role\": 64")).ok());         // beyond the cap
    CHECK_FALSE(loadWith(replaced(job, "\"id\": 4", "\"id\": 0")).ok());              // a group needs an id
    CHECK_FALSE(loadWith(replaced(job, "\"id\": 4", "\"id\": \"x\"")).ok());          // wrong type
    CHECK_FALSE(loadWith(replaced(job, "\"role\": 1", "\"role\": 1, \"role\": 2")).ok());  // duplicate key
    CHECK_FALSE(loadWith(replaced(job, "\"prior_toolpaths\": [2]", "\"prior_toolpaths\": 5")).ok());     // not an array
    CHECK_FALSE(loadWith(replaced(job, "\"prior_toolpaths\": [2]", "\"prior_toolpaths\": [\"a\"]")).ok());  // wrong type
    CHECK_FALSE(loadWith(replaced(job, "\"prior_toolpaths\": [2]", "\"prior_toolpaths\": [0.5]")).ok());
    CHECK_FALSE(loadWith(replaced(job, "\"prior_toolpaths\": [2]", "\"prior_toolpaths\": [-1]")).ok());
    // tools: not an array / an element that is not an object / an empty list
    std::string tools = job;
    const std::size_t at = tools.find("\"tools\": [");
    REQUIRE(at != std::string::npos);
    const std::size_t close = tools.find(']', at);  // no tool entry contains a bracket
    REQUIRE(close != std::string::npos);
    const auto withTools = [&](const std::string& value) {
        std::string t = job;
        t.replace(at, close + 1 - at, "\"tools\": " + value);
        return t;
    };
    CHECK(loadWith(withTools("[{\"id\": \"a\"}, {\"id\": \"b\"}]")).ok());
    CHECK_FALSE(loadWith(withTools("5")).ok());                  // not an array
    CHECK_FALSE(loadWith(withTools("[]")).ok());                 // empty
    CHECK_FALSE(loadWith(withTools("[1]")).ok());                // not an object
    CHECK_FALSE(loadWith(withTools("[{\"id\": 3}]")).ok());      // wrong type
    CHECK_FALSE(loadWith(withTools("[{\"id\": \"a\", \"id\": \"b\"}, {}]")).ok());  // duplicate key
    // a list of 257 priors
    std::string many = "[";
    for (int i = 0; i < 257; ++i) {
        many += i == 0 ? "1" : ",1";
    }
    many += "]";
    CHECK_FALSE(loadWith(replaced(job, "\"prior_toolpaths\": [2]", "\"prior_toolpaths\": " + many)).ok());
    CHECK(loadWith(replaced(job, "\"prior_toolpaths\": [2]", "\"prior_toolpaths\": [2, 3, 4]")).ok());
}
