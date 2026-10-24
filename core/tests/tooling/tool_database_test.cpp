#include "carvetto/tooling/tool_database.hpp"

#include <catch2/catch_test_macros.hpp>

using namespace carvetto::tooling;
using carvetto::geometry::kPi;

namespace {
Tool endMill(std::string id, std::string name, double diameter, std::string groupId = "") {
    Tool t;
    t.id = std::move(id);
    t.name = std::move(name);
    t.type = ToolType::EndMill;
    t.diameter = diameter;
    t.groupId = std::move(groupId);
    t.defaultCuttingData.feedRate = 1000.0;
    return t;
}

CuttingData feed(double f) {
    CuttingData d;
    d.feedRate = f;
    return d;
}

ToolGroup group(std::string id, std::string name, std::string parent = "") {
    return ToolGroup{std::move(id), std::move(name), std::move(parent)};
}
}  // namespace

// ------------------------------------------------------------------- tools

TEST_CASE("Tool CRUD", "[tooling][database]") {
    ToolDatabase db;
    CHECK(db.isEmpty());
    CHECK(db.addTool(endMill("em6", "6 mm end mill", 6.0)));
    CHECK_FALSE(db.addTool(endMill("em6", "dup", 6.0)));
    CHECK_FALSE(db.addTool(endMill("", "no id", 6.0)));
    CHECK_FALSE(db.addTool(endMill("em3", "bad group", 3.0, "missing")));
    REQUIRE(db.findTool("em6") != nullptr);
    CHECK(db.findTool("em6")->name == "6 mm end mill");
    CHECK(db.findTool("nope") == nullptr);

    Tool updated = endMill("em6", "renamed", 6.35);
    CHECK(db.updateTool(updated));
    CHECK(db.findTool("em6")->name == "renamed");
    CHECK(db.findTool("em6")->diameter == 6.35);
    CHECK_FALSE(db.updateTool(endMill("ghost", "x", 1.0)));
    updated.groupId = "missing";
    CHECK_FALSE(db.updateTool(updated));

    CHECK(db.removeTool("em6"));
    CHECK_FALSE(db.removeTool("em6"));
    CHECK(db.isEmpty());
}

TEST_CASE("copyTool duplicates geometry and cutting data under a new id", "[tooling][database]") {
    ToolDatabase db;
    Tool t = endMill("a", "A", 6.0);
    t.setCuttingData("m", "x", feed(1234.0));
    db.addTool(t);
    CHECK(db.copyTool("a", "b", "B"));
    CHECK_FALSE(db.copyTool("a", "b", "again"));
    CHECK_FALSE(db.copyTool("zzz", "c", "C"));
    CHECK_FALSE(db.copyTool("a", "", "C"));
    const Tool* b = db.findTool("b");
    REQUIRE(b != nullptr);
    CHECK(b->name == "B");
    CHECK(b->diameter == 6.0);
    REQUIRE(b->findCuttingData("m", "x") != nullptr);
    CHECK(b->findCuttingData("m", "x")->feedRate == 1234.0);
    CHECK(db.tools().size() == 2);
}

// ------------------------------------------------------------------ groups

TEST_CASE("Groups nest, list children and remove by re-homing contents", "[tooling][database]") {
    ToolDatabase db;
    CHECK(db.addGroup(group("g1", "Roughing")));
    CHECK(db.addGroup(group("g2", "Fine", "g1")));
    CHECK_FALSE(db.addGroup(group("g2", "dup")));
    CHECK_FALSE(db.addGroup(group("", "no id")));
    CHECK_FALSE(db.addGroup(group("g3", "orphan", "nope")));
    CHECK(db.addTool(endMill("t1", "root tool", 6.0)));
    CHECK(db.addTool(endMill("t2", "in g1", 6.0, "g1")));
    CHECK(db.addTool(endMill("t3", "in g2", 6.0, "g2")));

    CHECK(db.toolsInGroup("").size() == 1);
    CHECK(db.toolsInGroup("g1").size() == 1);
    CHECK(db.toolsInGroup("g2").size() == 1);
    REQUIRE(db.childGroups("").size() == 1);
    CHECK(db.childGroups("")[0]->id == "g1");
    REQUIRE(db.childGroups("g1").size() == 1);
    CHECK(db.childGroups("g1")[0]->id == "g2");

    CHECK(db.renameGroup("g2", "Finishing"));
    CHECK_FALSE(db.renameGroup("zz", "x"));
    CHECK(db.findGroup("g2")->name == "Finishing");

    // Removing g1 moves t2 to the root and g2 under the root.
    CHECK(db.removeGroup("g1"));
    CHECK_FALSE(db.removeGroup("g1"));
    CHECK(db.findTool("t2")->groupId.empty());
    CHECK(db.findGroup("g2")->parentId.empty());
    CHECK(db.findTool("t3")->groupId == "g2");
    CHECK(db.toolsInGroup("").size() == 2);
}

// --------------------------------------------------------------- materials

TEST_CASE("Materials: add, rename, duplicate with/without cutting data, remove", "[tooling][database]") {
    ToolDatabase db;
    CHECK(db.addMaterial({"oak", "Oak"}));
    CHECK_FALSE(db.addMaterial({"oak", "Oak again"}));
    CHECK_FALSE(db.addMaterial({"", "x"}));
    CHECK(db.renameMaterial("oak", "Red Oak"));
    CHECK_FALSE(db.renameMaterial("ash", "x"));
    CHECK(db.findMaterial("oak")->name == "Red Oak");

    Tool t = endMill("t", "T", 6.0);
    t.setCuttingData("m", "oak", feed(500.0));
    t.setCuttingData("m", "mdf", feed(900.0));
    db.addTool(t);

    CHECK(db.duplicateMaterial("oak", "oak2", "Oak copy", true));
    CHECK(db.duplicateMaterial("oak", "oak3", "Oak empty", false));
    CHECK_FALSE(db.duplicateMaterial("oak", "oak2", "dup", true));
    CHECK_FALSE(db.duplicateMaterial("none", "x", "x", true));
    const Tool* stored = db.findTool("t");
    REQUIRE(stored->findCuttingData("m", "oak2") != nullptr);
    CHECK(stored->findCuttingData("m", "oak2")->feedRate == 500.0);
    CHECK(stored->findCuttingData("m", "oak3") == nullptr);
    CHECK(db.materials().size() == 3);

    CHECK(db.removeMaterial("oak"));
    CHECK_FALSE(db.removeMaterial("oak"));
    stored = db.findTool("t");
    CHECK(stored->findCuttingData("m", "oak") == nullptr);
    CHECK(stored->findCuttingData("m", "oak2") != nullptr);
    CHECK(stored->findCuttingData("m", "mdf") != nullptr);
}

// ---------------------------------------------------------------- machines

TEST_CASE("Machines: add, update, duplicate, remove keeping or deleting cutting data",
          "[tooling][database]") {
    ToolDatabase db;
    Machine m;
    m.id = "shapeoko";
    m.name = "Shapeoko";
    m.accelerationX = 500.0;
    m.accelerationAngular = kPi;
    m.associatedPosts = {"grbl_mm", "grbl_inch"};
    m.defaultPost = "grbl_mm";
    CHECK(db.addMachine(m));
    CHECK_FALSE(db.addMachine(m));
    Machine blank;
    CHECK_FALSE(db.addMachine(blank));

    m.name = "Shapeoko 4";
    CHECK(db.updateMachine(m));
    CHECK(db.findMachine("shapeoko")->name == "Shapeoko 4");
    Machine ghost;
    ghost.id = "ghost";
    CHECK_FALSE(db.updateMachine(ghost));

    Tool t = endMill("t", "T", 6.0);
    t.setCuttingData("shapeoko", "oak", feed(700.0));
    t.setCuttingData("other", "oak", feed(800.0));
    db.addTool(t);

    CHECK(db.duplicateMachine("shapeoko", "s2", "Shapeoko copy", true));
    CHECK(db.duplicateMachine("shapeoko", "s3", "Shapeoko bare", false));
    CHECK_FALSE(db.duplicateMachine("shapeoko", "s2", "dup", true));
    CHECK_FALSE(db.duplicateMachine("nope", "x", "x", true));
    const Machine* copy = db.findMachine("s2");
    REQUIRE(copy != nullptr);
    CHECK(copy->accelerationX == 500.0);
    CHECK(copy->associatedPosts == m.associatedPosts);
    CHECK(copy->defaultPost == "grbl_mm");
    CHECK(db.findTool("t")->findCuttingData("s2", "oak")->feedRate == 700.0);
    CHECK(db.findTool("t")->findCuttingData("s3", "oak") == nullptr);

    SECTION("remove deleting cutting data") {
        CHECK(db.removeMachine("shapeoko", true));
        CHECK(db.findMachine("shapeoko") == nullptr);
        CHECK(db.findTool("t")->findCuttingData("shapeoko", "oak") == nullptr);
        CHECK(db.findTool("t")->findCuttingData("other", "oak") != nullptr);
    }
    SECTION("remove keeping cutting data") {
        CHECK(db.removeMachine("shapeoko", false));
        CHECK(db.findMachine("shapeoko") == nullptr);
        CHECK(db.findTool("t")->findCuttingData("shapeoko", "oak") != nullptr);
    }
    CHECK_FALSE(db.removeMachine("shapeoko", true));
}

// ------------------------------------------------------------------- merge

namespace {
ToolDatabase existingDb() {
    ToolDatabase db;
    db.addGroup(group("g", "Group"));
    db.addMaterial({"oak", "Oak"});
    Machine m;
    m.id = "m1";
    m.name = "Mill";
    db.addMachine(m);
    Tool t = endMill("shared", "Existing shared", 6.0, "g");
    t.setCuttingData("m1", "oak", feed(111.0));
    db.addTool(t);
    db.addTool(endMill("onlyMine", "Only mine", 3.0));
    return db;
}

ToolDatabase incomingDb() {
    ToolDatabase db;
    db.addGroup(group("g", "Group renamed"));
    db.addGroup(group("g2", "New group"));
    db.addMaterial({"oak", "Oak (incoming)"});
    db.addMaterial({"mdf", "MDF"});
    Machine m;
    m.id = "m2";
    m.name = "Router";
    db.addMachine(m);
    Tool t = endMill("shared", "Incoming shared", 6.35, "g2");
    t.setCuttingData("m1", "oak", feed(999.0));
    t.setCuttingData("m2", "mdf", feed(222.0));
    db.addTool(t);
    db.addTool(endMill("onlyTheirs", "Only theirs", 12.0));
    return db;
}
}  // namespace

TEST_CASE("Merge without overwrite keeps existing cutting data and adds new tools (AC3)",
          "[tooling][database]") {
    ToolDatabase db = existingDb();
    const MergeReport report = db.merge(incomingDb(), MergePolicy::KeepExisting);

    CHECK(report.tools == MergeCounts{1, 0, 1});
    CHECK(report.groups == MergeCounts{1, 0, 1});
    CHECK(report.materials == MergeCounts{1, 0, 1});
    CHECK(report.machines == MergeCounts{1, 0, 0});

    const Tool* shared = db.findTool("shared");
    REQUIRE(shared != nullptr);
    CHECK(shared->name == "Existing shared");
    CHECK(shared->diameter == 6.0);
    CHECK(shared->groupId == "g");
    REQUIRE(shared->findCuttingData("m1", "oak") != nullptr);
    CHECK(shared->findCuttingData("m1", "oak")->feedRate == 111.0);
    CHECK(shared->findCuttingData("m2", "mdf") == nullptr);

    CHECK(db.findTool("onlyMine") != nullptr);
    REQUIRE(db.findTool("onlyTheirs") != nullptr);
    CHECK(db.findTool("onlyTheirs")->diameter == 12.0);
    CHECK(db.findGroup("g")->name == "Group");
    CHECK(db.findGroup("g2") != nullptr);
    CHECK(db.findMaterial("oak")->name == "Oak");
    CHECK(db.findMaterial("mdf") != nullptr);
    CHECK(db.findMachine("m2") != nullptr);
    CHECK(db.tools().size() == 3);
}

TEST_CASE("Merge with overwrite replaces existing items (AC3)", "[tooling][database]") {
    ToolDatabase db = existingDb();
    const MergeReport report = db.merge(incomingDb(), MergePolicy::Overwrite);

    CHECK(report.tools == MergeCounts{1, 1, 0});
    CHECK(report.groups == MergeCounts{1, 1, 0});
    CHECK(report.materials == MergeCounts{1, 1, 0});
    CHECK(report.machines == MergeCounts{1, 0, 0});

    const Tool* shared = db.findTool("shared");
    REQUIRE(shared != nullptr);
    CHECK(shared->name == "Incoming shared");
    CHECK(shared->diameter == 6.35);
    CHECK(shared->groupId == "g2");
    REQUIRE(shared->findCuttingData("m1", "oak") != nullptr);
    CHECK(shared->findCuttingData("m1", "oak")->feedRate == 999.0);
    CHECK(shared->findCuttingData("m2", "mdf") != nullptr);

    CHECK(db.findTool("onlyMine") != nullptr);
    CHECK(db.findTool("onlyTheirs") != nullptr);
    CHECK(db.findGroup("g")->name == "Group renamed");
    CHECK(db.findMaterial("oak")->name == "Oak (incoming)");
    CHECK(db.tools().size() == 3);
}

TEST_CASE("Merge is idempotent and merging an empty database changes nothing", "[tooling][database]") {
    ToolDatabase db = existingDb();
    const ToolDatabase before = db;
    const MergeReport none = db.merge(ToolDatabase{}, MergePolicy::Overwrite);
    CHECK(none == MergeReport{});
    CHECK(db == before);

    db.merge(incomingDb(), MergePolicy::Overwrite);
    const ToolDatabase once = db;
    db.merge(incomingDb(), MergePolicy::Overwrite);
    CHECK(db == once);
    db.merge(incomingDb(), MergePolicy::KeepExisting);
    CHECK(db == once);
}

TEST_CASE("Merge brings a tool's group along so the tool stays in it", "[tooling][database]") {
    ToolDatabase incoming;
    incoming.addGroup(group("ghost", "Ghost"));
    incoming.addTool(endMill("x", "X", 6.0, "ghost"));

    ToolDatabase target;
    target.merge(incoming, MergePolicy::KeepExisting);
    REQUIRE(target.findTool("x") != nullptr);
    CHECK(target.findTool("x")->groupId == "ghost");
    CHECK(target.findGroup("ghost") != nullptr);
    CHECK(target.toolsInGroup("ghost").size() == 1);
}

TEST_CASE("Merge breaks group parent cycles introduced by the incoming data", "[tooling][database]") {
    ToolDatabase a;
    a.addGroup(group("p", "P"));
    a.addGroup(group("c", "C", "p"));
    ToolDatabase b;
    b.addGroup(group("c", "C"));
    b.addGroup(group("p", "P", "c"));  // reversed: p under c
    a.merge(b, MergePolicy::Overwrite);
    // Both groups exist and no group's parent chain loops back to itself.
    for (const auto& [id, g] : a.groups()) {
        std::string cursor = g.parentId;
        int steps = 0;
        while (!cursor.empty() && steps++ < 10) {
            CHECK(cursor != id);
            cursor = a.findGroup(cursor)->parentId;
        }
        CHECK(steps < 10);
    }
}
