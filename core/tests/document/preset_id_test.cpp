// T-0013: commands refuse caller-supplied (preset) ids that are already used by any sheet, layer or
// object, or that lie below the id generator's next value; checkIntegrity reports an id shared by
// several items (ADR-0007, amendment A2).

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <map>
#include <memory>
#include <optional>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
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

void requireIntact(const Document& doc, const std::string& where) {
    const auto issues = doc.checkIntegrity();
    INFO(where << ": " << describe(issues));
    REQUIRE(issues.empty());
}

std::size_t countKind(const std::vector<IntegrityIssue>& issues, Kind kind) {
    return static_cast<std::size_t>(std::count_if(
        issues.begin(), issues.end(), [kind](const IntegrityIssue& i) { return i.kind == kind; }));
}

Object squareAt(const Document& doc, double x) {
    return Object::vector(Contour::rectangle(Box2{Vec2{x, 0}, Vec2{x + 5, 5}}), doc.defaultLayer(),
                          doc.activeSheet());
}

// Every sheet, layer and object id value currently in the document (with repeats, if any).
std::vector<std::uint64_t> allIds(const Document& doc) {
    std::vector<std::uint64_t> out;
    for (const Sheet& s : doc.sheets()) {
        out.push_back(s.id.value);
    }
    for (const Layer& l : doc.layers()) {
        out.push_back(l.id.value);
    }
    for (const auto& [id, obj] : doc.objects()) {
        out.push_back(id.value);
    }
    return out;
}

void requireUniqueIds(const Document& doc, const std::string& where) {
    const std::vector<std::uint64_t> ids = allIds(doc);
    const std::set<std::uint64_t> unique(ids.begin(), ids.end());
    INFO(where);
    REQUIRE(unique.size() == ids.size());
}

// Everything a refused command must leave untouched.
struct State {
    explicit State(const Document& doc)
        : sheets(doc.sheets()),
          layers(doc.layers()),
          objects(doc.objects()),
          activeSheet(doc.activeSheet()),
          activeLayer(doc.activeLayer()),
          nextId(doc.idGenerator().peek()),
          undoCount(doc.undoStack().count()),
          undoIndex(doc.undoStack().index()),
          selection(doc.selection()) {}
    bool operator==(const State&) const = default;

    std::vector<Sheet> sheets;
    std::vector<Layer> layers;
    std::map<ObjectId, Object> objects;
    SheetId activeSheet;
    LayerId activeLayer;
    std::uint64_t nextId;
    std::size_t undoCount;
    std::size_t undoIndex;
    Selection selection;
};

// The three commands that accept a preset id, each given the id `value`.
enum class AddKind { Object, Layer, Sheet };

std::unique_ptr<Command> addWithId(const Document& doc, AddKind kind, std::uint64_t value) {
    switch (kind) {
        case AddKind::Object: {
            Object o = squareAt(doc, 0);
            o.id = ObjectId{value};
            return std::make_unique<AddObjectCommand>(std::move(o));
        }
        case AddKind::Layer:
            return std::make_unique<AddLayerCommand>(Layer{.id = LayerId{value}, .name = "preset"});
        case AddKind::Sheet:
            return std::make_unique<AddSheetCommand>(Sheet{.id = SheetId{value}, .name = "preset"});
    }
    return nullptr;
}

const char* kindName(AddKind kind) {
    switch (kind) {
        case AddKind::Object:
            return "object";
        case AddKind::Layer:
            return "layer";
        case AddKind::Sheet:
            return "sheet";
    }
    return "?";
}

constexpr AddKind kAllKinds[] = {AddKind::Object, AddKind::Layer, AddKind::Sheet};

}  // namespace

TEST_CASE("AC3 regression: an object cannot take the default layer's id", "[document][ids]") {
    Document d;
    const State before(d);
    Object o = squareAt(d, 0);
    o.id = ObjectId{d.defaultLayer().value};
    REQUIRE_FALSE(d.execute(std::make_unique<AddObjectCommand>(o)));
    REQUIRE(State(d) == before);
    REQUIRE_FALSE(d.undoStack().canUndo());
    requireIntact(d, "after refused add");
}

TEST_CASE("every preset-id command refuses an id held by any sheet, layer or object", "[document][ids]") {
    Document d;
    const ObjectId obj = ObjectId{d.insertObject(squareAt(d, 0))};
    const std::uint64_t taken[] = {d.activeSheet().value, d.defaultLayer().value, obj.value};
    const char* takenBy[] = {"sheet", "layer", "object"};
    for (AddKind kind : kAllKinds) {
        for (std::size_t i = 0; i < 3; ++i) {
            INFO(kindName(kind) << " taking the id of a " << takenBy[i]);
            const State before(d);
            REQUIRE_FALSE(d.execute(addWithId(d, kind, taken[i])));
            REQUIRE(State(d) == before);
            requireIntact(d, "after refused add");
        }
    }
}

TEST_CASE("every preset-id command refuses an id below the generator's next id", "[document][ids]") {
    for (AddKind kind : kAllKinds) {
        INFO(kindName(kind));
        Document d;
        // An id handed out and then freed by undoing its add must not be taken again.
        REQUIRE(d.execute(std::make_unique<AddObjectCommand>(squareAt(d, 0))));
        const std::uint64_t freedObject = d.objectIds().front().value;
        REQUIRE(d.execute(std::make_unique<AddLayerCommand>(Layer{.name = "L"})));
        const std::uint64_t freedLayer = d.layers().back().id.value;
        REQUIRE(d.execute(std::make_unique<AddSheetCommand>(Sheet{.name = "S"})));
        const std::uint64_t freedSheet = d.sheets().back().id.value;
        REQUIRE(d.undo());
        REQUIRE(d.undo());
        REQUIRE(d.undo());
        REQUIRE(d.objectCount() == 0);
        for (std::uint64_t freed : {freedObject, freedLayer, freedSheet}) {
            REQUIRE(freed < d.idGenerator().peek());
            const State before(d);
            REQUIRE_FALSE(d.execute(addWithId(d, kind, freed)));
            REQUIRE(State(d) == before);
            // The redo history survives a refused command.
            REQUIRE(d.undoStack().canRedo());
        }
        // A fresh id at or above the next id is still accepted, and redo can bring the freed ids
        // back with their original identity.
        REQUIRE(d.redo());
        REQUIRE(d.hasObject(ObjectId{freedObject}));
        requireIntact(d, "after redo");
    }
}

TEST_CASE("a fresh preset id is accepted, reserves the generator and survives undo/redo", "[document][ids]") {
    for (AddKind kind : kAllKinds) {
        INFO(kindName(kind));
        Document d;
        const std::uint64_t preset = d.idGenerator().peek() + 10;
        REQUIRE(d.execute(addWithId(d, kind, preset)));
        REQUIRE(d.idGenerator().peek() == preset + 1);
        requireIntact(d, "after add");
        const State added(d);
        REQUIRE(d.undo());
        requireIntact(d, "after undo");
        // The freed preset id is now below the next id: no command may take it again.
        for (AddKind other : kAllKinds) {
            REQUIRE_FALSE(d.execute(addWithId(d, other, preset)));
        }
        REQUIRE(d.redo());
        REQUIRE(State(d) == added);
        requireIntact(d, "after redo");
        REQUIRE(d.undo());
        REQUIRE(d.redo());
        REQUIRE(State(d) == added);
    }
}

TEST_CASE("checkIntegrity reports an id shared across kinds", "[document][ids][integrity]") {
    SECTION("object with a layer's id") {
        Document d;
        d.insertObject(squareAt(d, 0));
        Object o = squareAt(d, 10);
        o.id = ObjectId{d.defaultLayer().value};
        d.insertObject(o);  // raw insert: only same-kind duplicates are asserted
        const auto issues = d.checkIntegrity();
        REQUIRE(countKind(issues, Kind::DuplicateId) == 1);
        const auto it = std::find_if(issues.begin(), issues.end(),
                                     [](const IntegrityIssue& i) { return i.kind == Kind::DuplicateId; });
        REQUIRE(it->object == o.id);
        REQUIRE(it->message.find("layer") != std::string::npos);
        REQUIRE(it->message.find("object") != std::string::npos);
    }
    SECTION("layer with a sheet's id") {
        Document d;
        d.insertLayer(Layer{.id = LayerId{d.activeSheet().value}, .name = "clash"});
        const auto issues = d.checkIntegrity();
        REQUIRE(countKind(issues, Kind::DuplicateId) == 1);
        REQUIRE(issues.size() == 1);
    }
    SECTION("sheet, layer and object sharing one id is one issue") {
        Document d;
        const std::uint64_t v = d.idGenerator().peek() + 5;
        d.insertSheet(Sheet{.id = SheetId{v}});
        d.insertLayer(Layer{.id = LayerId{v}});
        Object o = squareAt(d, 0);
        o.id = ObjectId{v};
        d.insertObject(o);
        REQUIRE(countKind(d.checkIntegrity(), Kind::DuplicateId) == 1);
    }
    SECTION("a consistent document reports nothing") {
        Document d;
        d.insertObject(squareAt(d, 0));
        d.insertLayer(Layer{.name = "L2"});
        d.insertSheet(Sheet{.name = "S2"});
        REQUIRE(d.checkIntegrity().empty());
    }
}

namespace {

// Random walk over every command, with the preset-id adds fed ids drawn from every value the
// document has ever used (live or freed by undo/delete), small offsets around the generator's
// next id, and random values below it.
class PresetFuzzer {
   public:
    explicit PresetFuzzer(std::uint32_t seed) : rng_(seed) {}

    std::size_t pick(std::size_t n) { return std::uniform_int_distribution<std::size_t>(0, n - 1)(rng_); }
    bool chance(int percent) { return static_cast<int>(pick(100)) < percent; }

    void remember(const Document& doc) {
        for (std::uint64_t v : allIds(doc)) {
            seen_.insert(v);
        }
    }

    std::uint64_t presetId(const Document& doc) {
        const std::uint64_t next = doc.idGenerator().peek();
        switch (pick(4)) {
            case 0:
                if (!seen_.empty()) {
                    auto it = seen_.begin();
                    std::advance(it, static_cast<std::ptrdiff_t>(pick(seen_.size())));
                    return *it;
                }
                return next;
            case 1:
                return 1 + static_cast<std::uint64_t>(pick(static_cast<std::size_t>(next)));
            case 2:
                return next - 1 + static_cast<std::uint64_t>(pick(4));  // next-1 .. next+2
            default:
                return next;
        }
    }

    // Performs one random step; returns a label.
    std::string step(Document& doc) {
        remember(doc);
        std::vector<LayerId> layers;
        for (const Layer& l : doc.layers()) {
            layers.push_back(l.id);
        }
        std::vector<SheetId> sheets;
        for (const Sheet& s : doc.sheets()) {
            sheets.push_back(s.id);
        }
        const std::vector<ObjectId> objects = doc.objectIds();
        const auto someObjects = [&]() {
            std::vector<ObjectId> out;
            const std::size_t n = 1 + pick(3);
            for (std::size_t i = 0; i < n && !objects.empty(); ++i) {
                out.push_back(objects[pick(objects.size())]);
            }
            return out;
        };

        switch (pick(14)) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4: {
                const AddKind kind = kAllKinds[pick(3)];
                const std::uint64_t value = presetId(doc);
                const std::vector<std::uint64_t> ids = allIds(doc);
                const bool fresh = value >= doc.idGenerator().peek() && value < IdGenerator::kMaxId &&
                                   std::find(ids.begin(), ids.end(), value) == ids.end();
                const State before(doc);
                std::unique_ptr<Command> cmd;
                if (kind == AddKind::Object) {
                    Object o = Object::vector(Contour::rectangle(Box2{Vec2{0, 0}, Vec2{5, 5}}),
                                              layers[pick(layers.size())], sheets[pick(sheets.size())]);
                    o.id = ObjectId{value};
                    cmd = std::make_unique<AddObjectCommand>(std::move(o));
                } else {
                    cmd = addWithId(doc, kind, value);
                }
                const bool done = doc.execute(std::move(cmd));
                const std::string label =
                    std::string("preset ") + kindName(kind) + " #" + std::to_string(value);
                INFO(label);
                REQUIRE(done == fresh);
                if (!done) {
                    REQUIRE(State(doc) == before);
                }
                return label;
            }
            case 5:
                doc.execute(std::make_unique<AddObjectCommand>(
                    Object::vector(Contour::rectangle(Box2{Vec2{0, 0}, Vec2{5, 5}}),
                                   layers[pick(layers.size())], sheets[pick(sheets.size())])));
                return "add object";
            case 6:
                doc.execute(std::make_unique<RemoveObjectsCommand>(someObjects()));
                return "remove objects";
            case 7:
                doc.execute(std::make_unique<GroupObjectsCommand>(someObjects()));
                return "group";
            case 8:
                if (!objects.empty()) {
                    doc.execute(
                        std::make_unique<PasteObjectsCommand>(doc.copyObjects(someObjects()), Vec2{1, 1}));
                }
                return "paste";
            case 9:
                doc.execute(std::make_unique<DeleteLayerCommand>(layers[pick(layers.size())], chance(50)));
                return "delete layer";
            case 10:
                doc.execute(std::make_unique<DeleteSheetCommand>(sheets[pick(sheets.size())]));
                return "delete sheet";
            case 11:
            case 12:
                doc.undo();
                return "undo";
            default:
                doc.redo();
                return "redo";
        }
    }

   private:
    std::mt19937 rng_;
    std::set<std::uint64_t> seen_;
};

}  // namespace

TEST_CASE("randomized preset-id commands never produce a shared id, through undo and redo",
          "[document][ids][fuzz]") {
    constexpr std::uint32_t kSeeds = 60;
    constexpr int kSteps = 150;
    for (std::uint32_t seed = 1; seed <= kSeeds; ++seed) {
        Document doc;
        PresetFuzzer fuzz(seed);
        for (int i = 0; i < kSteps; ++i) {
            const std::string where =
                "seed " + std::to_string(seed) + " step " + std::to_string(i) + " (" + fuzz.step(doc) + ")";
            requireIntact(doc, where);
            requireUniqueIds(doc, where);
        }
        int n = 0;
        while (doc.undo()) {
            const std::string where = "seed " + std::to_string(seed) + " undo " + std::to_string(n++);
            requireIntact(doc, where);
            requireUniqueIds(doc, where);
        }
        // With everything undone, a preset command may still not take any id ever handed out.
        fuzz.remember(doc);
        for (AddKind kind : kAllKinds) {
            for (std::uint64_t v = 1; v < doc.idGenerator().peek(); ++v) {
                INFO("seed " << seed << ": " << kindName(kind) << " #" << v << " after undo-all");
                REQUIRE_FALSE(addWithId(doc, kind, v)->apply(doc));
            }
        }
        requireIntact(doc, "seed " + std::to_string(seed) + " after refused re-takes");
        n = 0;
        while (doc.redo()) {
            const std::string where = "seed " + std::to_string(seed) + " redo " + std::to_string(n++);
            requireIntact(doc, where);
            requireUniqueIds(doc, where);
        }
    }
}
