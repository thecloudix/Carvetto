// T-0321: the Wrapping level effect in the document: undoable command, bake, `.carv` 1.19 (ADR-0114).

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "carvetto/relief/effects.hpp"

using namespace carvetto::document;
namespace relief = carvetto::relief;
using Bytes = std::vector<std::uint8_t>;

namespace {

relief::Component tall(const std::string& name) {
    std::vector<float> d(4 * 14);
    for (std::size_t i = 0; i < d.size(); ++i) d[i] = 1.0F + static_cast<float>((i * 5) % 17);
    relief::Component c;
    c.name = name;
    REQUIRE(relief::HeightField::fromData(relief::Grid{4, 14, 1.0, {0, 0}}, std::move(d), c.field) == relief::Status::Ok);
    return c;
}

relief::Wrapping wrapY(double c, double start = 0.0) {
    relief::Wrapping w;
    w.enabled = true;
    w.circumference = c;
    w.start = start;
    return w;
}

struct Fixture {
    Document doc;
    LevelId level;
    ComponentId comp;
    Fixture() {
        auto add = std::make_unique<AddReliefLevelCommand>(doc.activeSheet(), "Lv");
        auto* p = add.get();
        REQUIRE(doc.execute(std::move(add)));
        level = p->createdId();
        auto cmd = std::make_unique<AddReliefComponentCommand>(level, tall("A"));
        auto* q = cmd.get();
        REQUIRE(doc.execute(std::move(cmd)));
        comp = q->createdId();
    }
    const ReliefLevel& lv() const { return *doc.findReliefLevel(level); }
    relief::HeightField composed() const {
        relief::HeightField out;
        REQUIRE(relief::compose(doc.reliefModel(doc.activeSheet()), relief::Grid{4, 14, 1.0, {0, 0}}, out) ==
                relief::Status::Ok);
        return out;
    }
};

Bytes save(const Document& doc) {
    auto r = saveCarv(doc);
    if (!r.ok()) FAIL(r.error().message);
    return r.value();
}

std::string textOf(const carvetto::document::detail::ZipMember& m) { return {m.data.begin(), m.data.end()}; }

bool same(float a, float b) { return (std::isnan(a) && std::isnan(b)) || a == b; }

}  // namespace

TEST_CASE("wrapping: the command sets, changes, removes and undoes", "[document][relief][effects][wrapping]") {
    Fixture f;
    const relief::HeightField flat = f.composed();
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(LevelId{}, wrapY(6.0))));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, wrapY(0.0))));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, wrapY(std::nan("")))));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, relief::Wrapping{})));  // already off
    CHECK_FALSE(f.lv().effects.any());
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, wrapY(6.0))));
    CHECK(f.lv().effects.wrapping.enabled);
    CHECK(f.lv().effects.any());
    CHECK(f.doc.checkIntegrity().empty());
    const relief::HeightField wrapped = f.composed();
    CHECK(std::isnan(wrapped.at(0, 8)));  // outside the window
    CHECK_FALSE(std::isnan(wrapped.at(0, 5)));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, wrapY(6.0))));  // unchanged
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, wrapY(6.0, 2.0))));  // Update
    CHECK(f.lv().effects.wrapping.start == 2.0);
    relief::Wrapping off = wrapY(99.0, 7.0);
    off.enabled = false;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, off)));
    CHECK_FALSE(f.lv().effects.wrapping.enabled);
    CHECK_FALSE(f.lv().effects.any());
    REQUIRE(f.doc.undo());
    CHECK(f.lv().effects.wrapping.start == 2.0);
    REQUIRE(f.doc.undo());
    REQUIRE(f.doc.undo());
    CHECK_FALSE(f.lv().effects.any());
    const relief::HeightField back = f.composed();
    for (std::size_t k = 0; k < flat.data().size(); ++k) CHECK(same(back.data()[k], flat.data()[k]));  // field untouched
    REQUIRE(f.doc.redo());
    CHECK(f.lv().effects.wrapping.circumference == 6.0);
}

TEST_CASE("wrapping: bake equals the composition and undo restores the effect", "[document][relief][effects][wrapping]") {
    Fixture f;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(f.level, relief::CombineMode::Add, 3.0, true)));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, wrapY(6.0, 1.0))));
    const relief::HeightField expect = f.composed();
    auto bake = std::make_unique<BakeReliefLevelEffectsCommand>(f.level);
    auto* p = bake.get();
    REQUIRE(f.doc.execute(std::move(bake)));
    CHECK_FALSE(p->createdId().isNull());
    CHECK_FALSE(f.lv().effects.any());
    CHECK(f.lv().baseHeight == 3.0);
    CHECK(f.doc.checkIntegrity().empty());
    const relief::HeightField after = f.composed();
    for (std::size_t k = 0; k < expect.data().size(); ++k) CHECK(same(after.data()[k], expect.data()[k]));
    REQUIRE(f.doc.undo());
    CHECK(f.lv().effects.wrapping == wrapY(6.0, 1.0));
    CHECK(f.lv().components[0].id == f.comp);
    const relief::HeightField restored = f.composed();
    for (std::size_t k = 0; k < expect.data().size(); ++k) CHECK(same(restored.data()[k], expect.data()[k]));
}

TEST_CASE("wrapping: persists in .carv 1.20, minor only when used, byte-stable", "[document][relief][effects][wrapping][carv]") {
    Fixture f;
    CHECK(loadCarv(save(f.doc)).value().formatMinor == kCarvFormatMinorRelief);
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(f.level, relief::MirrorMode::LeftToRight,
                                                                        relief::Point{1.5, 1.5})));
    const Bytes tenBytes = save(f.doc);
    CHECK(loadCarv(tenBytes).value().formatMinor == kCarvFormatMinorLevelEffects);
    {  // without wrapping the effects record is the 1.10 one
        auto ms = carvetto::document::detail::readZip(tenBytes);
        REQUIRE(ms.ok);
        for (const auto& m : ms.members) {
            if (m.name == "job.json") CHECK(textOf(m).find("wrapping") == std::string::npos);
        }
    }
    relief::Wrapping w = wrapY(31.5, -2.25);
    w.circumferenceIsY = false;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, w)));
    const Bytes first = save(f.doc);
    auto loaded = loadCarv(first);
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == 20);
    CHECK(kCarvFormatMinor == 20);
    const Document& d = loaded.value().document;
    REQUIRE(d.checkIntegrity().empty());
    CHECK(d.reliefLevels()[0].effects.wrapping == w);
    CHECK(d.reliefLevels()[0].effects.mirror == relief::MirrorMode::LeftToRight);
    CHECK(save(d) == first);
    // Wrapping alone (no mirror, no clip) round-trips too.
    Fixture g;
    REQUIRE(g.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(g.level, wrapY(8.0))));
    auto r = loadCarv(save(g.doc));
    REQUIRE(r.ok());
    CHECK(r.value().formatMinor == kCarvFormatMinorWrapping);
    CHECK(r.value().document.reliefLevels()[0].effects.wrapping == wrapY(8.0));
}

TEST_CASE("wrapping: hostile or malformed wrapping records are refused", "[document][relief][effects][wrapping][carv][hostile]") {
    Fixture f;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelWrappingCommand>(f.level, wrapY(8.0))));
    const Bytes bytes = save(f.doc);
    const auto mutated = [&](const std::string& from, const std::string& to) {
        auto z = carvetto::document::detail::readZip(bytes);
        REQUIRE(z.ok);
        for (auto& m : z.members) {
            if (m.name != "job.json") continue;
            std::string job = textOf(m);
            const auto at = job.find(from);
            REQUIRE(at != std::string::npos);
            job.replace(at, from.size(), to);
            m.data.assign(job.begin(), job.end());
        }
        auto packed = carvetto::document::detail::writeZip(z.members, 1);
        REQUIRE(packed.ok);
        return loadCarv(packed.bytes);
    };
    CHECK(mutated("\"circumference_axis\": \"y\"", "\"circumference_axis\": \"x\"").ok());
    CHECK_FALSE(mutated("\"circumference_axis\": \"y\"", "\"circumference_axis\": \"z\"").ok());
    CHECK_FALSE(mutated("\"circumference_axis\": \"y\"", "\"circumference_axis\": 1").ok());
    CHECK_FALSE(mutated("\"circumference\": 8.0", "\"circumference\": 0").ok());
    CHECK_FALSE(mutated("\"circumference\": 8.0", "\"circumference\": -4").ok());
    CHECK_FALSE(mutated("\"circumference\": 8.0", "\"circumference\": 1e-9").ok());  // below the floor
    CHECK(mutated("\"circumference\": 8.0", "\"circumference\": 1e-6").ok());
    CHECK_FALSE(mutated("\"circumference\": 8.0", "\"circumference\": 1e999").ok());
    CHECK_FALSE(mutated("\"circumference\": 8.0", "\"circumference\": \"8\"").ok());
    CHECK_FALSE(mutated("\"start\": 0.0", "\"start\": 1e999").ok());
    CHECK_FALSE(mutated("\"circumference_axis\": \"y\"", "\"circumference_axis\": \"y\", \"circumference_axis\": \"x\"").ok());
    CHECK_FALSE(mutated("\"start\": 0.0", "\"start\": 0.0, \"start\": 1").ok());
    // Missing keys are refused; unknown keys are skipped like in the other relief records.
    CHECK_FALSE(mutated("\"start\": 0.0", "\"startx\": 0").ok());
    CHECK(mutated("\"start\": 0.0", "\"start\": 0.0, \"future\": [1]").ok());
}
