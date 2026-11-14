// T-0295: level effects in the document: undoable commands and `.carv` 1.10 persistence (ADR-0079).

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

relief::Component comp(const std::string& name, float base) {
    std::vector<float> d(16);
    for (std::size_t i = 0; i < d.size(); ++i) d[i] = base + static_cast<float>(i);
    relief::Component c;
    c.name = name;
    REQUIRE(relief::HeightField::fromData(relief::Grid{4, 4, 1.0, {0, 0}}, std::move(d), c.field) == relief::Status::Ok);
    return c;
}

relief::Clip square(double x0, double y0, double x1, double y1) {
    relief::Clip c;
    c.polygons.push_back({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}});
    return c;
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
        a = addComp("A", 1.0F);
        b = addComp("B", 100.0F);
    }
    ComponentId addComp(const std::string& n, float base) {
        auto cmd = std::make_unique<AddReliefComponentCommand>(level, comp(n, base));
        auto* p = cmd.get();
        REQUIRE(doc.execute(std::move(cmd)));
        return p->createdId();
    }
    const ReliefLevel& lv() const { return *doc.findReliefLevel(level); }
    relief::HeightField composed() const {
        relief::HeightField out;
        REQUIRE(relief::compose(doc.reliefModel(doc.activeSheet()), relief::Grid{4, 4, 1.0, {0, 0}}, out) ==
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

}  // namespace

TEST_CASE("level effects: clip command applies, updates, removes and undoes", "[document][relief][effects]") {
    Fixture f;
    const relief::HeightField before = f.composed();
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, square(0.5, 0.5, 2.5, 2.5))));
    CHECK(f.lv().effects.clip.polygons.size() == 1);
    CHECK(std::isnan(f.composed().at(0, 0)));
    CHECK(f.doc.checkIntegrity().empty());
    // Update with another outline.
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, square(-0.5, -0.5, 0.5, 0.5))));
    CHECK_FALSE(std::isnan(f.composed().at(0, 0)));
    CHECK(std::isnan(f.composed().at(1, 1)));
    // Same clip again: no-op.
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, square(-0.5, -0.5, 0.5, 0.5))));
    // Remove.
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, relief::Clip{})));
    CHECK(f.composed().data() == before.data());
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, relief::Clip{})));
    for (int i = 0; i < 3; ++i) REQUIRE(f.doc.undo());
    CHECK(f.lv().effects.clip.polygons.empty());
    CHECK(f.composed().data() == before.data());
    REQUIRE(f.doc.redo());
    CHECK(f.lv().effects.clip.polygons.size() == 1);
    CHECK(f.doc.checkIntegrity().empty());
}

TEST_CASE("level effects: clip command refuses unknown levels and invalid polygons", "[document][relief][effects]") {
    Fixture f;
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(LevelId{99999}, square(0, 0, 1, 1))));
    relief::Clip bad;
    bad.polygons.push_back({{0, 0}, {1, 1}});
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, bad)));
    bad = square(0, 0, 1, 1);
    bad.polygons[0][1].y = std::nan("");
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, bad)));
    relief::Clip many;
    for (std::size_t i = 0; i <= relief::kMaxClipPolygons; ++i) many.polygons.push_back(square(0, 0, 1, 1).polygons[0]);
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, many)));
    CHECK_FALSE(f.lv().effects.any());
}

TEST_CASE("level effects: mirror command changes the render, not the stored field", "[document][relief][effects]") {
    Fixture f;
    const std::vector<float> stored = f.doc.findReliefComponent(f.a)->data.field.data();
    const relief::HeightField before = f.composed();
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(f.level, relief::MirrorMode::LeftToRight,
                                                                        relief::Point{1.5, 1.5})));
    const relief::HeightField m = f.composed();
    CHECK(m.at(3, 2) == before.at(0, 2));
    CHECK(m.at(2, 2) == before.at(1, 2));
    CHECK(f.doc.findReliefComponent(f.a)->data.field.data() == stored);
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(f.level, relief::MirrorMode::LeftToRight,
                                                                            relief::Point{1.5, 1.5})));
    CHECK_FALSE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(
        f.level, relief::MirrorMode::TopToBottom, relief::Point{std::nan(""), 0.0})));
    REQUIRE(f.doc.undo());
    CHECK(f.composed().data() == before.data());
    REQUIRE(f.doc.redo());
    CHECK(f.composed().data() == m.data());
}

TEST_CASE("level effects: consecutive mirror edits coalesce into one undo step", "[document][relief][effects]") {
    Fixture f;
    f.doc.undoStack().breakCoalescing();
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(f.level, relief::MirrorMode::LeftToRight,
                                                                        relief::Point{1.0, 1.0})));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(f.level, relief::MirrorMode::RightToLeft,
                                                                        relief::Point{2.0, 1.0})));
    CHECK(f.lv().effects.mirror == relief::MirrorMode::RightToLeft);
    REQUIRE(f.doc.undo());
    CHECK(f.lv().effects.mirror == relief::MirrorMode::None);
    CHECK(f.lv().effects.mirrorCenter.x == 0.0);
}

TEST_CASE("level effects: baking makes them permanent and is undoable", "[document][relief][effects]") {
    Fixture f;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelPropertiesCommand>(f.level, relief::CombineMode::Add, 5.0, true)));
    CHECK_FALSE(f.doc.execute(std::make_unique<BakeReliefLevelEffectsCommand>(f.level)));  // no effects yet
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(f.level, relief::MirrorMode::LeftToRight,
                                                                        relief::Point{1.5, 1.5})));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, square(-0.5, -0.5, 2.5, 3.5))));
    f.doc.componentSelection().assign({f.a});
    const relief::HeightField expect = f.composed();
    auto bake = std::make_unique<BakeReliefLevelEffectsCommand>(f.level);
    auto* p = bake.get();
    REQUIRE(f.doc.execute(std::move(bake)));
    const ComponentId baked = p->createdId();
    CHECK_FALSE(baked.isNull());
    REQUIRE(f.lv().components.size() == 1);
    CHECK(f.lv().components[0].id == baked);
    CHECK_FALSE(f.lv().effects.any());
    CHECK(f.lv().baseHeight == 5.0);
    CHECK(f.doc.componentSelection().empty());
    CHECK(f.doc.checkIntegrity().empty());
    const relief::HeightField after = f.composed();
    for (std::size_t k = 0; k < expect.data().size(); ++k) {
        if (std::isnan(expect.data()[k])) CHECK(std::isnan(after.data()[k]));
        else CHECK(after.data()[k] == expect.data()[k]);
    }
    REQUIRE(f.doc.undo());
    CHECK(f.lv().components.size() == 2);
    CHECK(f.lv().components[0].id == f.a);
    CHECK(f.lv().effects.mirror == relief::MirrorMode::LeftToRight);
    CHECK(f.lv().effects.clip.polygons.size() == 1);
    CHECK(f.doc.componentSelection().contains(f.a));
    REQUIRE(f.doc.redo());
    CHECK(f.lv().components[0].id == baked);
    CHECK(f.doc.checkIntegrity().empty());
}

TEST_CASE("level effects: persist in .carv 1.10, byte-stable, minor only when used", "[document][relief][effects][carv]") {
    Fixture f;
    CHECK(loadCarv(save(f.doc)).value().formatMinor == kCarvFormatMinorRelief);
    {
        auto ms = carvetto::document::detail::readZip(save(f.doc));
        REQUIRE(ms.ok);
        for (const auto& m : ms.members) {
            if (m.name == "job.json") CHECK(textOf(m).find("effects") == std::string::npos);
        }
    }
    relief::Clip clip = square(0.25, 0.5, 2.5, 3.0);
    clip.polygons.push_back({{1.0, 1.0}, {2.0, 1.0}, {1.5, 2.0}});
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, clip)));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(f.level, relief::MirrorMode::QuadTopLeft,
                                                                        relief::Point{1.5, 0.75})));
    const Bytes first = save(f.doc);
    auto loaded = loadCarv(first);
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorLevelEffects);
    const Document& d = loaded.value().document;
    REQUIRE(d.checkIntegrity().empty());
    const relief::LevelEffects& e = d.reliefLevels()[0].effects;
    CHECK(e.mirror == relief::MirrorMode::QuadTopLeft);
    CHECK(e.mirrorCenter.x == 1.5);
    CHECK(e.mirrorCenter.y == 0.75);
    REQUIRE(e.clip.polygons.size() == 2);
    CHECK(e.clip.polygons[0].size() == 4);
    CHECK(e.clip.polygons[1].size() == 3);
    CHECK(e.clip.polygons[0][0].x == 0.25);
    CHECK(e.clip.polygons[1][2].y == 2.0);
    CHECK(save(d) == first);
    // Mirror alone and every mode round-trips.
    for (int m = 0; m <= 8; ++m) {
        Fixture g;
        REQUIRE(g.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(
            g.level, static_cast<relief::MirrorMode>(m), relief::Point{static_cast<double>(m), -1.0})));
        auto r = loadCarv(save(g.doc));
        REQUIRE(r.ok());
        CHECK(r.value().document.reliefLevels()[0].effects.mirror == static_cast<relief::MirrorMode>(m));
    }
}

TEST_CASE("level effects: hostile or malformed effects records are refused", "[document][relief][effects][carv][hostile]") {
    Fixture f;
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelClipCommand>(f.level, square(0, 0, 2, 2))));
    REQUIRE(f.doc.execute(std::make_unique<SetReliefLevelMirrorCommand>(f.level, relief::MirrorMode::LeftToRight,
                                                                        relief::Point{1.0, 1.0})));
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
    CHECK(mutated("\"left_to_right\"", "\"left_to_right\"").ok());
    CHECK_FALSE(mutated("\"left_to_right\"", "\"sideways\"").ok());
    CHECK_FALSE(mutated("\"mirror_center\": [", "\"mirror_center\": [1e999, ").ok());
    // Unknown keys inside the effects record are skipped, as for other relief records.
    CHECK(mutated("\"mirror\": \"left", "\"future\": 1, \"mirror\": \"left").ok());
    // A polygon with two points.
    const auto r = mutated("\"clip\": [", "\"clip\": [[[0, 0], [1, 1]], ");
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().message.find("at least 3") != std::string::npos);
    CHECK_FALSE(mutated("\"clip\": [", "\"clip\": [7, ").ok());
    {  // the vertex cap: exactly the cap loads, one more is refused
        const auto polygon = [](std::size_t n) {
            std::string t = "\"clip\": [[";
            for (std::size_t i = 0; i < n; ++i) t += std::string(i ? ", " : "") + "[0, " + std::to_string(i % 7) + "]";
            return t + "], ";
        };
        CHECK(mutated("\"clip\": [", polygon(relief::kMaxClipVertices - 4)).ok());  // plus the 4-point original
        const auto over = mutated("\"clip\": [", polygon(relief::kMaxClipVertices - 3));
        REQUIRE_FALSE(over.ok());
        CHECK(over.error().message.find("clip vertices") != std::string::npos);
    }
    CHECK_FALSE(mutated("\"clip\": [", "\"clip\": [[[0, \"a\"], [1, 1], [2, 2]], ").ok());
}

TEST_CASE("level effects: a 1.9 file without effects still loads", "[document][relief][effects][carv]") {
    Fixture f;
    const Bytes bytes = save(f.doc);  // 1.9, no effects
    auto r = loadCarv(bytes);
    REQUIRE(r.ok());
    CHECK(r.value().formatMinor == kCarvFormatMinorRelief);
    CHECK_FALSE(r.value().document.reliefLevels()[0].effects.any());
}
