// T-0305: per-component and per-level appearance: commands and `.carv` 1.12 persistence (ADR-0095).

#include <catch2/catch_test_macros.hpp>
#include <memory>
#include <string>
#include <vector>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"

using namespace carvetto::document;
namespace relief = carvetto::relief;
using Bytes = std::vector<std::uint8_t>;

namespace {

relief::Component comp(const std::string& name) {
    std::vector<float> d(16, 1.0F);
    relief::Component c;
    c.name = name;
    REQUIRE(relief::HeightField::fromData(relief::Grid{4, 4, 1.0, {0, 0}}, std::move(d), c.field) == relief::Status::Ok);
    return c;
}

relief::Appearance solid(std::uint32_t rgb) {
    relief::Appearance a;
    a.kind = relief::AppearanceKind::SolidColour;
    a.rgb = rgb;
    return a;
}
relief::Appearance material(relief::AppearanceMaterial m) {
    relief::Appearance a;
    a.kind = relief::AppearanceKind::Material;
    a.material = m;
    return a;
}
relief::Appearance children() {
    relief::Appearance a;
    a.kind = relief::AppearanceKind::ColourFromChildren;
    return a;
}

struct Fixture {
    Document doc;
    LevelId level;
    ComponentId a, b;
    Fixture() {
        auto add = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "Lv");
        auto* p = add.get();
        REQUIRE(doc.execute(std::move(add)));
        level = p->createdId();
        a = addComp("A");
        b = addComp("B");
    }
    ComponentId addComp(const std::string& n) {
        auto cmd = std::make_unique<AddReliefComponentCommand>(level, comp(n));
        auto* p = cmd.get();
        REQUIRE(doc.execute(std::move(cmd)));
        return p->createdId();
    }
};

Bytes save(const Document& doc) {
    auto r = saveCarv(doc);
    if (!r.ok()) FAIL(r.error().message);
    return r.value();
}

std::string jobJson(const Bytes& bytes) {
    auto z = carvetto::document::detail::readZip(bytes);
    REQUIRE(z.ok);
    for (const auto& m : z.members) {
        if (m.name == "job.json") return {m.data.begin(), m.data.end()};
    }
    FAIL("no job.json");
    return {};
}

carvetto::document::CarvResult<LoadedCarv> mutated(const Bytes& bytes, const std::string& from, const std::string& to) {
    auto z = carvetto::document::detail::readZip(bytes);
    REQUIRE(z.ok);
    for (auto& m : z.members) {
        if (m.name != "job.json") continue;
        std::string job(m.data.begin(), m.data.end());
        const auto at = job.find(from);
        REQUIRE(at != std::string::npos);
        job.replace(at, from.size(), to);
        m.data.assign(job.begin(), job.end());
    }
    auto packed = carvetto::document::detail::writeZip(z.members, 1);
    REQUIRE(packed.ok);
    return loadCarv(packed.bytes);
}

}  // namespace

TEST_CASE("appearance: commands set, refuse, undo and coalesce", "[document][relief][appearance]") {
    Fixture f;
    CHECK(f.doc.findReliefComponent(f.a)->data.appearance.kind == relief::AppearanceKind::SameAsParent);
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.a, relief::Appearance{})));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.a, children())));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.a, solid(0x1000000u))));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(ComponentId{9999}, solid(1))));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.a, solid(0x336699u))));
    CHECK(f.doc.findReliefComponent(f.a)->data.appearance == solid(0x336699u));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.a, solid(0x336699u))));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefComponent(f.a)->data.appearance.kind == relief::AppearanceKind::SameAsParent);
    REQUIRE(f.doc.redo());
    CHECK(f.doc.findReliefComponent(f.a)->data.appearance == solid(0x336699u));
    // Consecutive edits of the same component coalesce into one undo step (the run opened by the first).
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(
        f.a, material(relief::AppearanceMaterial::Stone))));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(
        f.a, material(relief::AppearanceMaterial::Metal))));
    CHECK(f.doc.findReliefComponent(f.a)->data.appearance == material(relief::AppearanceMaterial::Metal));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefComponent(f.a)->data.appearance == solid(0x336699u));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefComponent(f.a)->data.appearance.kind == relief::AppearanceKind::SameAsParent);
    // An edit of another component does not merge.
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.a, solid(5))));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.b, solid(6))));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefComponent(f.a)->data.appearance == solid(5));
    REQUIRE(f.doc.undo());

    // Levels accept all four kinds.
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelAppearanceCommand>(f.level, relief::Appearance{})));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelAppearanceCommand>(f.level, children())));
    CHECK(f.doc.findReliefLevel(f.level)->appearance.kind == relief::AppearanceKind::ColourFromChildren);
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelAppearanceCommand>(LevelId{9999}, solid(1))));
    REQUIRE(f.doc.undo());
    CHECK(f.doc.findReliefLevel(f.level)->appearance.kind == relief::AppearanceKind::SameAsParent);

    // The model view sees it, and a duplicate keeps it.
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.b, solid(0xABCDEFu))));
    auto dup = std::make_unique<DuplicateReliefComponentCommand>(f.b);
    auto* dp = dup.get();
    REQUIRE(f.doc.execute(std::move(dup)));
    CHECK(f.doc.findReliefComponent(dp->createdId())->data.appearance == solid(0xABCDEFu));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelAppearanceCommand>(f.level, solid(0x010203u))));
    const auto model = f.doc.reliefModel(f.doc.activeSheet());
    REQUIRE(model.size() == 1);
    CHECK(model[0].appearance == solid(0x010203u));
    CHECK(model[0].components[1].appearance == solid(0xABCDEFu));
}

TEST_CASE("appearance: integrity rejects a bad stored appearance", "[document][relief][appearance]") {
    Fixture f;
    CHECK(f.doc.checkIntegrity().empty());
    f.doc.findReliefComponent(f.a)->data.appearance = children();
    CHECK_FALSE(f.doc.checkIntegrity().empty());
    f.doc.findReliefComponent(f.a)->data.appearance = relief::Appearance{};
    f.doc.findReliefLevel(f.level)->appearance = solid(0xFF000000u);
    CHECK_FALSE(f.doc.checkIntegrity().empty());
    CHECK_FALSE(saveCarv(f.doc).ok());
}

TEST_CASE("appearance: persists in .carv 1.12, minor only when used, byte-stable", "[document][relief][appearance][carv]") {
    Fixture f;
    const Bytes plain = save(f.doc);
    CHECK(loadCarv(plain).value().formatMinor == kCarvFormatMinorRelief);
    CHECK(jobJson(plain).find("appearance") == std::string::npos);

    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.a, solid(0x336699u))));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(
        f.b, material(relief::AppearanceMaterial::Metal))));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelAppearanceCommand>(f.level, children())));
    const Bytes first = save(f.doc);
    auto loaded = loadCarv(first);
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorAppearance);
    const Document& d = loaded.value().document;
    REQUIRE(d.checkIntegrity().empty());
    CHECK(d.reliefLevels()[0].appearance.kind == relief::AppearanceKind::ColourFromChildren);
    CHECK(d.reliefLevels()[0].components[0].data.appearance == solid(0x336699u));
    CHECK(d.reliefLevels()[0].components[1].data.appearance == material(relief::AppearanceMaterial::Metal));
    CHECK(save(d) == first);

    // Only the level carries one: still 1.12.
    Fixture g;
    REQUIRE(g.doc.execute(std::make_unique<SetReliefLevelAppearanceCommand>(g.level, solid(0))));
    auto only = loadCarv(save(g.doc));
    REQUIRE(only.ok());
    CHECK(only.value().formatMinor == kCarvFormatMinorAppearance);
    CHECK(only.value().document.reliefLevels()[0].appearance == solid(0));

    // Undoing back to "same as parent" drops the key and the minor again.
    REQUIRE(g.doc.undo());
    CHECK(save(g.doc) == plain);
}

TEST_CASE("appearance: malformed or hostile appearance records are refused", "[document][relief][appearance][carv][hostile]") {
    Fixture f;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefComponentAppearanceCommand>(f.a, solid(255))));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelAppearanceCommand>(f.level, children())));
    const Bytes bytes = save(f.doc);
    const std::string comp = "{\"kind\": \"solid\", \"rgb\": 255}";
    REQUIRE(jobJson(bytes).find(comp) != std::string::npos);
    CHECK(mutated(bytes, comp, comp).ok());
    CHECK_FALSE(mutated(bytes, "\"rgb\": 255", "\"rgb\": 16777216").ok());
    CHECK_FALSE(mutated(bytes, "\"rgb\": 255", "\"rgb\": -1").ok());
    CHECK_FALSE(mutated(bytes, "\"rgb\": 255", "\"rgb\": 1.5").ok());
    CHECK_FALSE(mutated(bytes, "\"solid\"", "\"glossy\"").ok());
    CHECK_FALSE(mutated(bytes, "\"solid\"", "\"same_as_parent\"").ok());
    CHECK_FALSE(mutated(bytes, "\"solid\"", "\"children\"").ok());       // a component has no children
    CHECK_FALSE(mutated(bytes, "\"solid\", \"rgb\": 255", "\"solid\"").ok());  // solid without rgb
    CHECK_FALSE(mutated(bytes, "\"rgb\": 255", "\"rgb\": 255, \"material\": \"wood\"").ok());
    CHECK_FALSE(mutated(bytes, "\"children\"", "\"children\", \"rgb\": 1").ok());
    CHECK_FALSE(mutated(bytes, "\"children\"", "\"material\"").ok());  // material without a material name
    CHECK_FALSE(mutated(bytes, "{\"kind\": \"children\"}", "{}").ok());
    CHECK_FALSE(mutated(bytes, "{\"kind\": \"children\"}", "7").ok());
}
