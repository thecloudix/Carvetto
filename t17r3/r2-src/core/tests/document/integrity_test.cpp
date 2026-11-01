#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstdint>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Transform2;
using carvetto::geometry::Vec2;
using Kind = IntegrityIssue::Kind;

namespace {

std::string describe(const std::vector<IntegrityIssue>& issues) {
    std::string out;
    for (const auto& issue : issues) {
        out += issue.message;
        out += "; ";
    }
    return out;
}

// Fails the current test with every violation listed when `doc` is inconsistent.
void requireIntact(const Document& doc, const std::string& where) {
    const auto issues = doc.checkIntegrity();
    INFO(where << ": " << describe(issues));
    REQUIRE(issues.empty());
}

bool hasKind(const std::vector<IntegrityIssue>& issues, Kind kind) {
    return std::any_of(issues.begin(), issues.end(),
                       [kind](const IntegrityIssue& i) { return i.kind == kind; });
}

ObjectId addSquareOn(Document& doc, LayerId layer, double x = 0.0) {
    Object obj =
        Object::vector(Contour::rectangle(Box2{Vec2{x, 0}, Vec2{x + 10, 10}}), layer, doc.activeSheet());
    auto cmd = std::make_unique<AddObjectCommand>(std::move(obj));
    const AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

LayerId addLayer(Document& doc, const std::string& name) {
    auto cmd = std::make_unique<AddLayerCommand>(Layer{.name = name});
    const AddLayerCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->id();
}

ObjectId group(Document& doc, std::vector<ObjectId> ids) {
    auto cmd = std::make_unique<GroupObjectsCommand>(std::move(ids));
    const GroupObjectsCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    return raw->groupId();
}

const std::vector<ObjectId>& childrenOf(const Document& doc, ObjectId g) {
    return doc.findObject(g)->asGroup()->children;
}

// Everything undo/redo must restore (selection and active layer/sheet are not undoable).
struct Snapshot {
    Job job;
    std::vector<Sheet> sheets;
    std::vector<Layer> layers;
    std::map<ObjectId, Object> objects;
    std::map<std::string, double, std::less<>> variables;

    explicit Snapshot(const Document& doc)
        : job(doc.job()),
          sheets(doc.sheets()),
          layers(doc.layers()),
          objects(doc.objects()),
          variables(doc.variables().all()) {}
    bool operator==(const Snapshot&) const = default;
};

}  // namespace

// ------------------------------------------------------------------------------------------------
// Regression tests for the verification report
// ------------------------------------------------------------------------------------------------

TEST_CASE("regression: ungrouping a nested group splices its children into the outer group",
          "[document][integrity][regression]") {
    Document doc;
    const LayerId layer = doc.defaultLayer();
    const ObjectId a = addSquareOn(doc, layer, 0);
    const ObjectId b = addSquareOn(doc, layer, 20);
    const ObjectId c = addSquareOn(doc, layer, 40);
    const ObjectId inner = group(doc, {a, b});
    const ObjectId outer = group(doc, {inner, c});
    REQUIRE(childrenOf(doc, outer) == std::vector<ObjectId>{inner, c});
    const Snapshot before(doc);

    REQUIRE(doc.execute(std::make_unique<UngroupCommand>(inner)));
    requireIntact(doc, "after ungroup");
    CHECK_FALSE(doc.hasObject(inner));
    CHECK(childrenOf(doc, outer) == std::vector<ObjectId>{a, b, c});  // a,b at inner's position
    CHECK(doc.findObject(a)->parent == outer);
    CHECK(doc.findObject(b)->parent == outer);
    CHECK(doc.findObject(c)->parent == outer);
    const Snapshot after(doc);

    REQUIRE(doc.undo());
    requireIntact(doc, "after undo");
    CHECK(Snapshot(doc) == before);
    REQUIRE(doc.redo());
    requireIntact(doc, "after redo");
    CHECK(Snapshot(doc) == after);
}

TEST_CASE("regression: delete layer with contents drops children of groups on other layers",
          "[document][integrity][regression]") {
    Document doc;
    const LayerId l0 = doc.defaultLayer();
    const LayerId x = addLayer(doc, "X");
    const ObjectId a = addSquareOn(doc, l0, 0);
    const ObjectId b = addSquareOn(doc, x, 20);
    const ObjectId g = group(doc, {a, b});
    REQUIRE(doc.findObject(g)->layer == l0);
    const Snapshot before(doc);

    REQUIRE(doc.execute(std::make_unique<DeleteLayerCommand>(x, /*deleteContents=*/true)));
    requireIntact(doc, "after delete layer");
    CHECK_FALSE(doc.hasObject(b));
    REQUIRE(doc.hasObject(g));
    CHECK(childrenOf(doc, g) == std::vector<ObjectId>{a});  // a one-child group is kept
    CHECK(doc.findObject(a)->parent == g);
    const Snapshot after(doc);

    REQUIRE(doc.undo());
    requireIntact(doc, "after undo");
    CHECK(Snapshot(doc) == before);
    REQUIRE(doc.redo());
    requireIntact(doc, "after redo");
    CHECK(Snapshot(doc) == after);
}

TEST_CASE("regression: delete layer with contents dissolves a group whose child is elsewhere",
          "[document][integrity][regression]") {
    Document doc;
    const LayerId e0 = doc.defaultLayer();
    const LayerId y = addLayer(doc, "Y");
    const ObjectId p = addSquareOn(doc, y, 0);
    const ObjectId q = addSquareOn(doc, e0, 20);
    const ObjectId g = group(doc, {p, q});
    REQUIRE(doc.findObject(g)->layer == y);
    const Snapshot before(doc);

    REQUIRE(doc.execute(std::make_unique<DeleteLayerCommand>(y, /*deleteContents=*/true)));
    requireIntact(doc, "after delete layer");
    CHECK_FALSE(doc.hasObject(g));
    CHECK_FALSE(doc.hasObject(p));
    REQUIRE(doc.hasObject(q));
    CHECK_FALSE(doc.findObject(q)->parent.has_value());  // q survives as a top-level object
    // ...and can be grouped again.
    const ObjectId r = addSquareOn(doc, e0, 40);
    const ObjectId regrouped = group(doc, {q, r});
    CHECK(doc.findObject(q)->parent == regrouped);
    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    const Snapshot after(doc);

    REQUIRE(doc.undo());
    requireIntact(doc, "after undo");
    CHECK(Snapshot(doc) == before);
    REQUIRE(doc.redo());
    requireIntact(doc, "after redo");
    CHECK(Snapshot(doc) == after);
}

// ------------------------------------------------------------------------------------------------
// Group rules
// ------------------------------------------------------------------------------------------------

TEST_CASE("a group emptied by a removal is removed with it, recursively", "[document][integrity]") {
    Document doc;
    const LayerId l0 = doc.defaultLayer();
    const LayerId x = addLayer(doc, "X");
    const ObjectId a = addSquareOn(doc, x, 0);
    const ObjectId b = addSquareOn(doc, x, 20);
    const ObjectId keep = addSquareOn(doc, l0, 40);
    const ObjectId inner = group(doc, {a, b});
    doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{inner}, x));
    const ObjectId outer = group(doc, {inner});
    doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{outer}, l0));
    // outer (L0) -> inner (L0 after the move) -> a, b (L0 too: moving a group moves descendants).
    CHECK(doc.findObject(a)->layer == l0);
    doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{a, b}, x));
    const Snapshot before(doc);

    REQUIRE(doc.execute(std::make_unique<DeleteLayerCommand>(x, true)));
    requireIntact(doc, "after delete");
    CHECK_FALSE(doc.hasObject(inner));
    CHECK_FALSE(doc.hasObject(outer));
    CHECK(doc.hasObject(keep));
    REQUIRE(doc.undo());
    requireIntact(doc, "after undo");
    CHECK(Snapshot(doc) == before);

    // Removing the last child directly prunes the same way.
    REQUIRE(doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{a, b})));
    requireIntact(doc, "after remove");
    CHECK_FALSE(doc.hasObject(outer));
    REQUIRE(doc.undo());
    CHECK(Snapshot(doc) == before);
}

TEST_CASE("moving a group to a layer moves its descendants", "[document][integrity]") {
    Document doc;
    const LayerId l0 = doc.defaultLayer();
    const LayerId other = addLayer(doc, "Other");
    const ObjectId a = addSquareOn(doc, l0, 0);
    const ObjectId b = addSquareOn(doc, l0, 20);
    const ObjectId inner = group(doc, {a});
    const ObjectId outer = group(doc, {inner, b});

    REQUIRE(doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{outer}, other)));
    for (ObjectId id : {outer, inner, a, b}) {
        CHECK(doc.findObject(id)->layer == other);
    }
    REQUIRE(doc.undo());
    for (ObjectId id : {outer, inner, a, b}) {
        CHECK(doc.findObject(id)->layer == l0);
    }
    // A child can still be moved on its own; it stays in its group.
    REQUIRE(doc.execute(std::make_unique<MoveObjectsToLayerCommand>(std::vector<ObjectId>{a}, other)));
    CHECK(doc.findObject(a)->layer == other);
    CHECK(doc.findObject(a)->parent == inner);
    requireIntact(doc, "after moving a child");
}

TEST_CASE("moving a child to another sheet detaches it from its group", "[document][integrity]") {
    Document doc;
    const LayerId l0 = doc.defaultLayer();
    auto addSheet = std::make_unique<AddSheetCommand>(Sheet{.name = "Back"});
    const AddSheetCommand* sheetRaw = addSheet.get();
    REQUIRE(doc.execute(std::move(addSheet)));
    const SheetId back = sheetRaw->id();
    const ObjectId a = addSquareOn(doc, l0, 0);
    const ObjectId b = addSquareOn(doc, l0, 20);
    const ObjectId g = group(doc, {a, b});
    const Snapshot before(doc);

    REQUIRE(doc.execute(std::make_unique<MoveObjectsToSheetCommand>(std::vector<ObjectId>{a}, back)));
    requireIntact(doc, "after moving a");
    CHECK(doc.findObject(a)->sheet == back);
    CHECK_FALSE(doc.findObject(a)->parent.has_value());
    CHECK(childrenOf(doc, g) == std::vector<ObjectId>{b});

    REQUIRE(doc.execute(std::make_unique<MoveObjectsToSheetCommand>(std::vector<ObjectId>{b}, back)));
    requireIntact(doc, "after moving b");
    CHECK_FALSE(doc.hasObject(g));  // emptied group pruned

    REQUIRE(doc.undo());
    REQUIRE(doc.undo());
    CHECK(Snapshot(doc) == before);

    // Moving the whole group keeps it intact.
    REQUIRE(doc.execute(std::make_unique<MoveObjectsToSheetCommand>(std::vector<ObjectId>{g}, back)));
    requireIntact(doc, "after moving the group");
    CHECK(childrenOf(doc, g) == std::vector<ObjectId>{a, b});
    CHECK(doc.findObject(b)->sheet == back);
}

TEST_CASE("adding an object into a group links both ways", "[document][integrity]") {
    Document doc;
    const LayerId l0 = doc.defaultLayer();
    const ObjectId a = addSquareOn(doc, l0, 0);
    const ObjectId g = group(doc, {a});
    Object child = Object::vector(Contour::circle({0, 0}, 1), l0, doc.activeSheet());
    child.parent = g;
    auto cmd = std::make_unique<AddObjectCommand>(child);
    const AddObjectCommand* raw = cmd.get();
    REQUIRE(doc.execute(std::move(cmd)));
    requireIntact(doc, "after add");
    CHECK(childrenOf(doc, g) == std::vector<ObjectId>{a, raw->id()});
    REQUIRE(doc.undo());
    requireIntact(doc, "after undo");
    CHECK(childrenOf(doc, g) == std::vector<ObjectId>{a});

    Object orphan = Object::vector(Contour::circle({0, 0}, 1), l0, doc.activeSheet());
    orphan.parent = ObjectId{999};  // not a group: parent is cleared
    REQUIRE(doc.execute(std::make_unique<AddObjectCommand>(orphan)));
    requireIntact(doc, "after orphan add");
}

// ------------------------------------------------------------------------------------------------
// The checker itself
// ------------------------------------------------------------------------------------------------

TEST_CASE("checkIntegrity reports each kind of violation", "[document][integrity]") {
    Document doc;
    const LayerId l0 = doc.defaultLayer();
    const SheetId s0 = doc.activeSheet();
    requireIntact(doc, "fresh document");

    const ObjectId a = doc.insertObject(Object::vector(Contour::circle({0, 0}, 1), l0, s0));
    const ObjectId b = doc.insertObject(Object::vector(Contour::circle({0, 0}, 1), l0, s0));
    Object g1;
    g1.layer = l0;
    g1.sheet = s0;
    g1.data = GroupObject{{a, ObjectId{500}}};
    const ObjectId g = doc.insertObject(g1);

    auto issues = doc.checkIntegrity();
    CHECK(hasKind(issues, Kind::DanglingChild));        // #500
    CHECK(hasKind(issues, Kind::ChildParentMismatch));  // a.parent not set

    doc.findObject(a)->parent = g;
    doc.findObject(g)->asGroup()->children = {a};
    requireIntact(doc, "repaired");

    doc.findObject(b)->parent = g;  // g does not list b
    CHECK(hasKind(doc.checkIntegrity(), Kind::ChildNotListed));
    doc.findObject(b)->parent = a;  // a is not a group
    CHECK(hasKind(doc.checkIntegrity(), Kind::BadParent));
    doc.findObject(b)->parent = ObjectId{777};  // missing
    CHECK(hasKind(doc.checkIntegrity(), Kind::BadParent));
    doc.findObject(b)->parent.reset();

    Object g2;
    g2.layer = l0;
    g2.sheet = s0;
    g2.data = GroupObject{{a}};
    const ObjectId h = doc.insertObject(g2);
    CHECK(hasKind(doc.checkIntegrity(), Kind::DuplicateChild));  // a listed by g and h
    doc.takeObject(h);
    requireIntact(doc, "duplicate removed");

    // Cycle: g -> a(group) -> g.
    Object* aObj = doc.findObject(a);
    aObj->data = GroupObject{{g}};
    doc.findObject(g)->parent = a;
    CHECK(hasKind(doc.checkIntegrity(), Kind::Cycle));
    aObj->data = VectorObject{};
    doc.findObject(g)->parent.reset();
    requireIntact(doc, "cycle broken");

    doc.findObject(b)->layer = LayerId{888};
    doc.findObject(b)->sheet = SheetId{889};
    issues = doc.checkIntegrity();
    CHECK(hasKind(issues, Kind::MissingLayer));
    CHECK(hasKind(issues, Kind::MissingSheet));
    doc.findObject(b)->layer = l0;
    doc.findObject(b)->sheet = s0;

    const SheetId s1 = doc.insertSheet(Sheet{.name = "Other"});
    doc.findObject(a)->sheet = s1;
    CHECK(hasKind(doc.checkIntegrity(), Kind::SheetMismatch));
    doc.findObject(a)->sheet = s0;
    requireIntact(doc, "all repaired");
}

// ------------------------------------------------------------------------------------------------
// Randomized: every command, nested groups, delete-layer both modes, undo/redo interleaved
// ------------------------------------------------------------------------------------------------

namespace {

class Fuzzer {
   public:
    explicit Fuzzer(std::uint32_t seed) : rng_(seed) {}

    std::size_t pick(std::size_t n) { return n == 0 ? 0 : static_cast<std::size_t>(rng_() % n); }
    bool chance(std::uint32_t percent) { return rng_() % 100 < percent; }

    template <class T>
    T pickFrom(const std::vector<T>& v) {
        return v[pick(v.size())];
    }

    std::vector<ObjectId> someObjects(const Document& doc, std::size_t max) {
        std::vector<ObjectId> all = doc.objectIds();
        std::vector<ObjectId> out;
        const std::size_t n = 1 + pick(max);
        for (std::size_t i = 0; i < n && !all.empty(); ++i) {
            out.push_back(pickFrom(all));
        }
        return out;
    }

    std::vector<ObjectId> groups(const Document& doc) {
        std::vector<ObjectId> out;
        for (const auto& [id, obj] : doc.objects()) {
            if (obj.isGroup()) {
                out.push_back(id);
            }
        }
        return out;
    }

    // Performs one random step. Returns a label for failure messages.
    std::string step(Document& doc) {
        std::vector<LayerId> layers;
        for (const Layer& l : doc.layers()) {
            layers.push_back(l.id);
        }
        std::vector<SheetId> sheets;
        for (const Sheet& s : doc.sheets()) {
            sheets.push_back(s.id);
        }
        const std::vector<ObjectId> objects = doc.objectIds();
        const std::vector<ObjectId> groupIds = groups(doc);

        switch (pick(22)) {
            case 0:
            case 1:
            case 2: {
                const double x = static_cast<double>(pick(200));
                Object obj = Object::vector(Contour::rectangle(Box2{Vec2{x, 0}, Vec2{x + 5, 5}}),
                                            pickFrom(layers), pickFrom(sheets));
                if (!groupIds.empty() && chance(30)) {
                    obj.parent = pickFrom(groupIds);
                }
                doc.execute(std::make_unique<AddObjectCommand>(std::move(obj)));
                return "add object";
            }
            case 3:
                doc.execute(std::make_unique<RemoveObjectsCommand>(someObjects(doc, 3)));
                return "remove objects";
            case 4:
            case 5:
                doc.execute(std::make_unique<GroupObjectsCommand>(someObjects(doc, 4)));
                return "group";
            case 6:
                if (!groupIds.empty()) {
                    doc.execute(std::make_unique<UngroupCommand>(pickFrom(groupIds), chance(50)));
                }
                return "ungroup";
            case 7:
                doc.execute(
                    std::make_unique<MoveObjectsToLayerCommand>(someObjects(doc, 3), pickFrom(layers)));
                return "move to layer";
            case 8:
                doc.execute(
                    std::make_unique<MoveObjectsToSheetCommand>(someObjects(doc, 3), pickFrom(sheets)));
                return "move to sheet";
            case 9:
                doc.execute(std::make_unique<TransformObjectsCommand>(
                    someObjects(doc, 3), Transform2::translation({static_cast<double>(pick(5)), 1.0})));
                return "transform";
            case 10:
                if (!objects.empty()) {
                    doc.execute(std::make_unique<PasteObjectsCommand>(
                        doc.copyObjects(someObjects(doc, 3)), Vec2{1, 1},
                        chance(50) ? pickFrom(layers) : LayerId{},
                        chance(50) ? pickFrom(sheets) : SheetId{}));
                }
                return "paste";
            case 11:
                doc.execute(std::make_unique<AddLayerCommand>(
                    Layer{.name = "L"}, chance(50), std::optional<std::size_t>(pick(layers.size() + 1))));
                return "add layer";
            case 12:
            case 13:
                doc.execute(std::make_unique<DeleteLayerCommand>(pickFrom(layers), chance(50)));
                return "delete layer";
            case 14: {
                Layer l = *doc.findLayer(pickFrom(layers));
                l.visible = !l.visible;
                l.colour = Colour{static_cast<std::uint8_t>(pick(256)), 0, 0};
                doc.execute(std::make_unique<SetLayerPropertiesCommand>(l));
                return "layer properties";
            }
            case 15:
                doc.execute(std::make_unique<MoveLayerCommand>(pickFrom(layers), pick(layers.size())));
                return "move layer";
            case 16:
                doc.execute(std::make_unique<AddSheetCommand>(Sheet{.name = "S"}, chance(50)));
                return "add sheet";
            case 17:
                doc.execute(std::make_unique<DeleteSheetCommand>(pickFrom(sheets)));
                return "delete sheet";
            case 18:
                doc.setActiveLayer(pickFrom(layers));
                doc.setActiveSheet(pickFrom(sheets));
                if (!objects.empty()) {
                    doc.selection().add(pickFrom(objects));
                }
                doc.undoStack().breakCoalescing();
                return "activate/select";
            case 19:
                doc.undoStack().beginMacro("macro");
                doc.execute(std::make_unique<GroupObjectsCommand>(someObjects(doc, 3)));
                doc.execute(std::make_unique<DeleteLayerCommand>(pickFrom(layers), chance(50)));
                doc.undoStack().endMacro();
                return "macro";
            case 20:
                doc.undo();
                return "undo";
            default:
                doc.redo();
                return "redo";
        }
    }

   private:
    std::mt19937 rng_;
};

}  // namespace

TEST_CASE("randomized edit sequences keep the document consistent through undo and redo",
          "[document][integrity][fuzz]") {
    constexpr std::uint32_t kSeeds = 120;
    constexpr int kSteps = 120;
    for (std::uint32_t seed = 1; seed <= kSeeds; ++seed) {
        Document doc;
        Fuzzer fuzz(seed);
        const Snapshot initial(doc);
        for (int i = 0; i < kSteps; ++i) {
            const std::string label = fuzz.step(doc);
            requireIntact(doc, std::string("seed ") + std::to_string(seed) + " step " + std::to_string(i) +
                                   " (" + label + ")");
        }
        int redone = 0;
        while (doc.redo()) {
            requireIntact(
                doc, std::string("seed ") + std::to_string(seed) + " redo-tail " + std::to_string(redone++));
        }
        const Snapshot finalState(doc);
        int undone = 0;
        while (doc.undo()) {
            requireIntact(doc,
                          std::string("seed ") + std::to_string(seed) + " undo " + std::to_string(undone++));
        }
        {
            INFO("seed " << seed << ": undo all returns to the initial document");
            REQUIRE(Snapshot(doc) == initial);
        }
        redone = 0;
        while (doc.redo()) {
            requireIntact(doc,
                          std::string("seed ") + std::to_string(seed) + " redo " + std::to_string(redone++));
        }
        INFO("seed " << seed << ": redo all returns to the final document");
        REQUIRE(Snapshot(doc) == finalState);
    }
}
