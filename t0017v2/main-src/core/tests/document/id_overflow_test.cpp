// T-0011: the id generator never wraps; allocation at the ceiling is refused, integrity flags
// out-of-range ids, and no public command sequence near the ceiling yields a duplicate id.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <limits>
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
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::Transform2;
using carvetto::geometry::Vec2;
using Kind = IntegrityIssue::Kind;

namespace {

constexpr std::uint64_t kMax = IdGenerator::kMaxId;
static_assert(IdGenerator::kMaxId == (std::uint64_t{1} << 62), "documented ceiling is 2^62");

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

bool hasKind(const std::vector<IntegrityIssue>& issues, Kind kind) {
    return std::any_of(issues.begin(), issues.end(),
                       [kind](const IntegrityIssue& i) { return i.kind == kind; });
}

Object squareAt(const Document& doc, double x) {
    return Object::vector(Contour::rectangle(Box2{Vec2{x, 0}, Vec2{x + 5, 5}}), doc.defaultLayer(),
                          doc.activeSheet());
}

// Leaves exactly `remaining` ids free by inserting a layer just below the ceiling (the generator
// reserves through it). Returns that layer's id.
LayerId leaveRemaining(Document& doc, std::uint64_t remaining) {
    const LayerId high = doc.insertLayer(Layer{.id = LayerId{kMax - 1 - remaining}, .name = "high"});
    REQUIRE(doc.idGenerator().remaining() == remaining);
    return high;
}

// Every sheet, layer and object id currently in the document (with repeats, if any).
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
        if (obj.id != id) {
            out.push_back(obj.id.value);
        }
    }
    return out;
}

// No id appears twice across kinds, every id is in range and below the generator's next id.
void requireIdsSound(const Document& doc, const std::string& where) {
    std::vector<std::uint64_t> ids = allIds(doc);
    std::sort(ids.begin(), ids.end());
    INFO(where);
    REQUIRE(std::adjacent_find(ids.begin(), ids.end()) == ids.end());
    REQUIRE(doc.idGenerator().peek() <= kMax);
    for (std::uint64_t v : ids) {
        REQUIRE(v >= 1);
        REQUIRE(v < kMax);
        REQUIRE(v < doc.idGenerator().peek());
    }
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// AC1: the generator reports exhaustion, never wraps, never reuses
// ------------------------------------------------------------------------------------------------

TEST_CASE("AC1: the id generator stops at the documented ceiling", "[document][ids][overflow]") {
    IdGenerator gen;
    CHECK(gen.peek() == 1);
    CHECK(gen.remaining() == kMax - 1);
    CHECK_FALSE(gen.exhausted());

    gen.reserveThrough(kMax - 3);  // next = kMax - 2: two ids left
    CHECK(gen.remaining() == 2);
    CHECK(gen.canAllocate(2));
    CHECK_FALSE(gen.canAllocate(3));
    CHECK(gen.allocate() == kMax - 2);
    CHECK(gen.tryAllocate() == std::optional<std::uint64_t>(kMax - 1));

    CHECK(gen.exhausted());
    CHECK(gen.remaining() == 0);
    CHECK(gen.canAllocate(0));
    CHECK_FALSE(gen.canAllocate(1));
    CHECK(gen.tryAllocate() == std::nullopt);
    CHECK(gen.tryAllocate() == std::nullopt);
    CHECK(gen.peek() == kMax);  // did not advance past the ceiling

    gen.reserveThrough(5);  // never goes backwards
    CHECK(gen.peek() == kMax);
}

TEST_CASE("AC1: reserving at or past the ceiling exhausts the generator without wrapping",
          "[document][ids][overflow]") {
    for (const std::uint64_t value : {kMax - 1, kMax, kMax + 1, std::numeric_limits<std::uint64_t>::max() - 1,
                                      std::numeric_limits<std::uint64_t>::max()}) {
        IdGenerator gen;
        gen.reserveThrough(value);
        INFO("reserveThrough(" << value << ")");
        CHECK(gen.exhausted());
        CHECK(gen.peek() == kMax);
        CHECK(gen.tryAllocate() == std::nullopt);
    }
}

TEST_CASE("AC1: allocating commands are no-ops once ids run out", "[document][ids][overflow]") {
    Document doc;
    const ObjectId a = doc.insertObject(squareAt(doc, 0));
    const ObjectId b = doc.insertObject(squareAt(doc, 10));
    leaveRemaining(doc, 0);
    REQUIRE(doc.idGenerator().exhausted());
    requireIntact(doc, "exhausted but consistent");

    const std::size_t objects = doc.objectCount();
    const std::size_t layers = doc.layers().size();
    const std::size_t sheets = doc.sheets().size();
    const std::size_t steps = doc.undoStack().count();

    SECTION("add object") {
        CHECK_FALSE(doc.execute(std::make_unique<AddObjectCommand>(squareAt(doc, 20))));
    }
    SECTION("add object with a preset id at the ceiling") {
        Object obj = squareAt(doc, 20);
        obj.id = ObjectId{kMax};
        CHECK_FALSE(doc.execute(std::make_unique<AddObjectCommand>(std::move(obj))));
    }
    SECTION("paste") {
        CHECK_FALSE(doc.execute(std::make_unique<PasteObjectsCommand>(doc.copyObjects({a}), Vec2{1, 1})));
    }
    SECTION("group") {
        CHECK_FALSE(doc.execute(std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a, b})));
        CHECK_FALSE(doc.findObject(a)->parent);
    }
    SECTION("add layer") {
        CHECK_FALSE(doc.execute(std::make_unique<AddLayerCommand>(Layer{.name = "L"}, true)));
    }
    SECTION("add sheet") {
        CHECK_FALSE(doc.execute(std::make_unique<AddSheetCommand>(Sheet{.name = "S"}, true)));
    }
    SECTION("inside a macro the refused command is not recorded (empty macro records nothing)") {
        doc.undoStack().beginMacro("m");
        CHECK_FALSE(doc.execute(std::make_unique<AddObjectCommand>(squareAt(doc, 20))));
        CHECK_FALSE(doc.execute(std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a, b})));
        doc.undoStack().endMacro();
    }

    CHECK(doc.objectCount() == objects);
    CHECK(doc.layers().size() == layers);
    CHECK(doc.sheets().size() == sheets);
    CHECK(doc.undoStack().count() == steps);
    CHECK(doc.idGenerator().peek() == kMax);
    requireIntact(doc, "after refused command");
}

TEST_CASE("AC1: commands that need no new id keep working when exhausted", "[document][ids][overflow]") {
    Document doc;
    const ObjectId a = doc.insertObject(squareAt(doc, 0));
    const ObjectId b = doc.insertObject(squareAt(doc, 10));
    leaveRemaining(doc, 1);
    auto grp = std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a, b});
    const GroupObjectsCommand* raw = grp.get();
    REQUIRE(doc.execute(std::move(grp)));  // takes the last id
    const ObjectId g = raw->groupId();
    CHECK(g.value == kMax - 1);
    REQUIRE(doc.idGenerator().exhausted());

    CHECK(doc.execute(std::make_unique<UngroupCommand>(g)));
    CHECK(doc.execute(std::make_unique<TransformObjectsCommand>(std::vector<ObjectId>{a},
                                                                Transform2::translation({1, 0}))));
    CHECK(doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{b})));
    // Undo and redo restore recorded ids and never allocate.
    while (doc.undo()) {
        requireIntact(doc, "undo");
    }
    while (doc.redo()) {
        requireIntact(doc, "redo");
    }
    CHECK_FALSE(doc.hasObject(g));
    CHECK(doc.idGenerator().peek() == kMax);
}

TEST_CASE("AC1: paste is all-or-nothing near the ceiling", "[document][ids][overflow]") {
    Document doc;
    const ObjectId a = doc.insertObject(squareAt(doc, 0));
    const ObjectId b = doc.insertObject(squareAt(doc, 10));
    const ObjectId c = doc.insertObject(squareAt(doc, 20));
    leaveRemaining(doc, 2);
    const std::size_t before = doc.objectCount();

    CHECK_FALSE(doc.execute(std::make_unique<PasteObjectsCommand>(doc.copyObjects({a, b, c}), Vec2{1, 1})));
    CHECK(doc.objectCount() == before);
    CHECK(doc.idGenerator().remaining() == 2);

    auto paste = std::make_unique<PasteObjectsCommand>(doc.copyObjects({a, b}), Vec2{1, 1});
    const PasteObjectsCommand* raw = paste.get();
    REQUIRE(doc.execute(std::move(paste)));
    CHECK(raw->pastedIds() == std::vector<ObjectId>{ObjectId{kMax - 2}, ObjectId{kMax - 1}});
    CHECK(doc.idGenerator().exhausted());
    requireIntact(doc, "after paste");
}

// ------------------------------------------------------------------------------------------------
// AC2: checkIntegrity flags out-of-range ids and a next id that is not above every id
// ------------------------------------------------------------------------------------------------

TEST_CASE("AC2: checkIntegrity flags ids at or above the ceiling", "[document][ids][integrity]") {
    Document doc;
    requireIntact(doc, "fresh");

    SECTION("object") {
        Object obj = squareAt(doc, 0);
        obj.id = ObjectId{kMax + 7};  // raw insert, as a corrupt load would do
        doc.insertObject(std::move(obj));
        CHECK(doc.idGenerator().peek() == kMax);  // clamped, not wrapped
        const auto issues = doc.checkIntegrity();
        CHECK(hasKind(issues, Kind::IdOutOfRange));
        CHECK(hasKind(issues, Kind::IdNotBelowNext));
    }
    SECTION("layer exactly at the ceiling") {
        doc.insertLayer(Layer{.id = LayerId{kMax}, .name = "bad"});
        const auto issues = doc.checkIntegrity();
        CHECK(hasKind(issues, Kind::IdOutOfRange));
    }
    SECTION("sheet") {
        doc.insertSheet(Sheet{.id = SheetId{std::numeric_limits<std::uint64_t>::max()}, .name = "bad"});
        CHECK(hasKind(doc.checkIntegrity(), Kind::IdOutOfRange));
    }
    SECTION("the largest valid id is fine") {
        doc.insertLayer(Layer{.id = LayerId{kMax - 1}, .name = "top"});
        requireIntact(doc, "id kMax - 1 with exhausted generator");
    }
}

TEST_CASE("AC2: checkIntegrity flags a next id at or below an existing id", "[document][ids][integrity]") {
    Document doc;
    const LayerId extra = doc.insertLayer(Layer{.name = "extra"});
    const SheetId extraSheet = doc.insertSheet(Sheet{.name = "extra"});
    const ObjectId obj = doc.insertObject(squareAt(doc, 0));
    requireIntact(doc, "before tampering");
    const std::uint64_t next = doc.idGenerator().peek();

    SECTION("layer id equal to next") {
        doc.findLayer(extra)->id = LayerId{next};
        const auto issues = doc.checkIntegrity();
        CHECK(hasKind(issues, Kind::IdNotBelowNext));
        CHECK_FALSE(hasKind(issues, Kind::IdOutOfRange));
    }
    SECTION("sheet id above next") {
        doc.findSheet(extraSheet)->id = SheetId{next + 100};
        CHECK(hasKind(doc.checkIntegrity(), Kind::IdNotBelowNext));
    }
    SECTION("object's stored id above next") {
        doc.findObject(obj)->id = ObjectId{next + 1};
        CHECK(hasKind(doc.checkIntegrity(), Kind::IdNotBelowNext));
    }
    SECTION("null id") {
        doc.findLayer(extra)->id = LayerId{};
        CHECK(hasKind(doc.checkIntegrity(), Kind::IdOutOfRange));
    }
}

// ------------------------------------------------------------------------------------------------
// AC3: randomized regression near the ceiling
// ------------------------------------------------------------------------------------------------

namespace {

class NearCeilingFuzzer {
   public:
    explicit NearCeilingFuzzer(std::uint32_t seed) : rng_(seed) {}

    std::size_t pick(std::size_t n) { return std::uniform_int_distribution<std::size_t>(0, n - 1)(rng_); }
    bool chance(int percent) { return static_cast<int>(pick(100)) < percent; }
    template <class T>
    T pickFrom(const std::vector<T>& v) {
        return v[pick(v.size())];
    }

    std::vector<ObjectId> someObjects(const Document& doc, std::size_t max) {
        const std::vector<ObjectId> all = doc.objectIds();
        std::vector<ObjectId> out;
        const std::size_t n = 1 + pick(max);
        for (std::size_t i = 0; i < n && !all.empty(); ++i) {
            out.push_back(pickFrom(all));
        }
        return out;
    }

    // One random step. `fresh` receives every id a command claims to have created; the caller
    // checks they were never seen before.
    std::string step(Document& doc, std::vector<std::uint64_t>& fresh) {
        std::vector<LayerId> layers;
        for (const Layer& l : doc.layers()) {
            layers.push_back(l.id);
        }
        std::vector<SheetId> sheets;
        for (const Sheet& s : doc.sheets()) {
            sheets.push_back(s.id);
        }
        std::vector<ObjectId> groups;
        for (const auto& [id, obj] : doc.objects()) {
            if (obj.isGroup()) {
                groups.push_back(id);
            }
        }
        switch (pick(14)) {
            case 0:
            case 1:
            case 2: {
                const double x = static_cast<double>(pick(200));
                Object obj = Object::vector(Contour::rectangle(Box2{Vec2{x, 0}, Vec2{x + 5, 5}}),
                                            pickFrom(layers), pickFrom(sheets));
                if (!groups.empty() && chance(30)) {
                    obj.parent = pickFrom(groups);
                }
                auto cmd = std::make_unique<AddObjectCommand>(std::move(obj));
                const AddObjectCommand* raw = cmd.get();
                if (doc.execute(std::move(cmd))) {
                    fresh.push_back(raw->id().value);
                }
                return "add object";
            }
            case 3: {
                // A caller-supplied id at or past the ceiling must be refused.
                Object obj = Object::vector(Contour::circle({0, 0}, 1), pickFrom(layers), pickFrom(sheets));
                obj.id = ObjectId{kMax + pick(3)};
                REQUIRE_FALSE(doc.execute(std::make_unique<AddObjectCommand>(std::move(obj))));
                return "add object with out-of-range id";
            }
            case 4:
            case 5:
                if (!doc.objectIds().empty()) {
                    auto cmd = std::make_unique<PasteObjectsCommand>(doc.copyObjects(someObjects(doc, 4)),
                                                                     Vec2{1, 1});
                    const PasteObjectsCommand* raw = cmd.get();
                    if (doc.execute(std::move(cmd))) {
                        for (ObjectId id : raw->pastedIds()) {
                            fresh.push_back(id.value);
                        }
                    }
                }
                return "paste";
            case 6:
            case 7: {
                auto cmd = std::make_unique<GroupObjectsCommand>(someObjects(doc, 4));
                const GroupObjectsCommand* raw = cmd.get();
                if (doc.execute(std::move(cmd))) {
                    fresh.push_back(raw->groupId().value);
                }
                return "group";
            }
            case 8:
                if (!groups.empty()) {
                    doc.execute(std::make_unique<UngroupCommand>(pickFrom(groups), chance(50)));
                }
                return "ungroup";
            case 9:
                doc.execute(std::make_unique<RemoveObjectsCommand>(someObjects(doc, 2)));
                return "remove";
            case 10: {
                auto cmd = std::make_unique<AddLayerCommand>(Layer{.name = "L"}, chance(50));
                const AddLayerCommand* raw = cmd.get();
                if (doc.execute(std::move(cmd))) {
                    fresh.push_back(raw->id().value);
                }
                return "add layer";
            }
            case 11: {
                auto cmd = std::make_unique<AddSheetCommand>(Sheet{.name = "S"}, chance(50));
                const AddSheetCommand* raw = cmd.get();
                if (doc.execute(std::move(cmd))) {
                    fresh.push_back(raw->id().value);
                }
                return "add sheet";
            }
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
};

}  // namespace

TEST_CASE("AC3: no public command sequence near the ceiling yields a duplicate or out-of-range id",
          "[document][ids][overflow][fuzz]") {
    constexpr std::uint32_t kSeeds = 60;
    constexpr int kSteps = 150;
    int seedsExhausted = 0;
    for (std::uint32_t seed = 1; seed <= kSeeds; ++seed) {
        Document doc;
        // A few ordinary objects, then jump the generator to within 0..40 ids of the ceiling.
        for (int i = 0; i < 3; ++i) {
            doc.insertObject(squareAt(doc, 10.0 * i));
        }
        NearCeilingFuzzer fuzz(seed);
        leaveRemaining(doc, fuzz.pick(41));
        requireIntact(doc, "seed " + std::to_string(seed) + " start");

        std::set<std::uint64_t> everSeen;
        for (std::uint64_t v : allIds(doc)) {
            everSeen.insert(v);
        }
        std::uint64_t lastNext = doc.idGenerator().peek();
        bool exhausted = false;
        for (int i = 0; i < kSteps; ++i) {
            std::vector<std::uint64_t> fresh;
            const std::string label = fuzz.step(doc, fresh);
            const std::string where =
                "seed " + std::to_string(seed) + " step " + std::to_string(i) + " (" + label + ")";
            requireIntact(doc, where);
            requireIdsSound(doc, where);
            INFO(where);
            REQUIRE(doc.idGenerator().peek() >= lastNext);  // monotonic
            lastNext = doc.idGenerator().peek();
            for (std::uint64_t v : fresh) {
                REQUIRE(v != 0);
                REQUIRE(v < kMax);
                REQUIRE(everSeen.insert(v).second);  // a newly created id was never used before
            }
            for (std::uint64_t v : allIds(doc)) {
                everSeen.insert(v);
            }
            exhausted = exhausted || doc.idGenerator().exhausted();
        }
        while (doc.undo()) {
            requireIntact(doc, "seed " + std::to_string(seed) + " undo-all");
            requireIdsSound(doc, "seed " + std::to_string(seed) + " undo-all");
        }
        while (doc.redo()) {
            requireIntact(doc, "seed " + std::to_string(seed) + " redo-all");
            requireIdsSound(doc, "seed " + std::to_string(seed) + " redo-all");
        }
        if (exhausted) {
            ++seedsExhausted;
        }
    }
    // The test is only meaningful if most runs actually hit the ceiling.
    CHECK(seedsExhausted >= static_cast<int>(kSeeds) * 3 / 4);
}
