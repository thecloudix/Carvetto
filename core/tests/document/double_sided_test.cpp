#include <memory>
#include <span>
#include <string>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/keepout_commands.hpp"
#include "carvetto/document/double_sided.hpp"

using namespace carvetto::document;
using carvetto::geometry::Box2;
using carvetto::geometry::Vec2;
using carvetto::toolpath::Move;
using Catch::Approx;

TEST_CASE("flipPoint mirrors about the material centre line and is an involution", "[document][double-sided]") {
    const Box2 m{{10, 20}, {110, 70}};  // centre (60, 45)
    const Vec2 p{25, 30};
    const Vec2 h = flipPoint(m, FlipDirection::Horizontal, p);
    CHECK(h.x == Approx(95));
    CHECK(h.y == Approx(30));
    const Vec2 v = flipPoint(m, FlipDirection::Vertical, p);
    CHECK(v.x == Approx(25));
    CHECK(v.y == Approx(60));
    for (const FlipDirection d : {FlipDirection::Horizontal, FlipDirection::Vertical}) {
        const Vec2 back = flipPoint(m, d, flipPoint(m, d, p));
        CHECK(back.x == Approx(p.x));
        CHECK(back.y == Approx(p.y));
    }
    // The centre and the edge lines map onto the material.
    CHECK(flipPoint(m, FlipDirection::Horizontal, {10, 20}).x == Approx(110));
    CHECK(flipPoint(m, FlipDirection::Vertical, {10, 20}).y == Approx(70));
}

TEST_CASE("flipBox mirrors a box and keeps it normalised", "[document][double-sided]") {
    const Box2 m{{0, 0}, {100, 50}};
    const Box2 b = flipBox(m, FlipDirection::Horizontal, Box2{{10, 5}, {30, 20}});
    CHECK(b.min.x == Approx(70));
    CHECK(b.max.x == Approx(90));
    CHECK(b.min.y == Approx(5));
    CHECK(b.max.y == Approx(20));
    const Box2 v = flipBox(m, FlipDirection::Vertical, Box2{{10, 5}, {30, 20}});
    CHECK(v.min.x == Approx(10));
    CHECK(v.min.y == Approx(30));
    CHECK(v.max.y == Approx(45));
}

TEST_CASE("materialTopZ places Z zero per side", "[document][double-sided]") {
    constexpr double t = 18.0;
    // Each side zeroes on its own cut face / its own bed.
    CHECK(materialTopZ(ZZero::MaterialSurface, false, t, Side::Front) == 0.0);
    CHECK(materialTopZ(ZZero::MaterialSurface, false, t, Side::Back) == 0.0);
    CHECK(materialTopZ(ZZero::MachineBed, false, t, Side::Front) == t);
    CHECK(materialTopZ(ZZero::MachineBed, false, t, Side::Back) == t);
    // Zero off the same physical face: the back side sees it at the bed (surface zero) or on top.
    CHECK(materialTopZ(ZZero::MaterialSurface, true, t, Side::Front) == 0.0);
    CHECK(materialTopZ(ZZero::MaterialSurface, true, t, Side::Back) == t);
    CHECK(materialTopZ(ZZero::MachineBed, true, t, Side::Front) == t);
    CHECK(materialTopZ(ZZero::MachineBed, true, t, Side::Back) == 0.0);
    CHECK(otherSide(Side::Front) == Side::Back);
    CHECK(otherSide(Side::Back) == Side::Front);
}

TEST_CASE("shiftMovesZ moves only Z", "[document][double-sided]") {
    const std::vector<Move> in{Move::rapid({1, 2, 3}), Move::feed({4, 5, -1})};
    const auto out = shiftMovesZ(in, 18.0);
    REQUIRE(out.size() == 2);
    CHECK(out[0].to.x == 1.0);
    CHECK(out[0].to.z == Approx(21));
    CHECK(out[1].to.y == 5.0);
    CHECK(out[1].to.z == Approx(17));
    CHECK(in[0].to.z == 3.0);
}

namespace {

Document pairedDocument() {
    Document doc(Job::doubleSided(200, 100, 18, Units::Millimetres, Datum::BottomLeft, ZZero::MachineBed,
                                  FlipDirection::Vertical));
    Sheet front = *doc.findSheet(doc.activeSheet());
    Sheet back = front;
    back.id = {};
    back.name = "Sheet 1 (back)";
    back.side = Side::Back;
    back.partner = front.id;
    const SheetId backId = doc.insertSheet(back);
    doc.findSheet(front.id)->partner = backId;
    return doc;
}

std::vector<std::uint8_t> save(const Document& d, const CarvExtras& ex = {}) {
    auto r = saveCarv(d, ex);
    REQUIRE(r.ok());
    return r.value();
}

}  // namespace

TEST_CASE("paired sheets round-trip byte-stably as format 1.5", "[document][double-sided][carv]") {
    const Document doc = pairedDocument();
    const auto first = save(doc);
    auto loaded = loadCarv(std::span<const std::uint8_t>(first));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorDoubleSided);
    CHECK(kCarvFormatMinor == 5);
    CHECK(loaded.value().extras.empty());
    const Document& back = loaded.value().document;
    REQUIRE(back.sheets().size() == 2);
    CHECK(back.sheets()[0].side == Side::Front);
    CHECK(back.sheets()[1].side == Side::Back);
    CHECK(back.sheets()[0].partner == back.sheets()[1].id);
    CHECK(back.sheets()[1].partner == back.sheets()[0].id);
    CHECK(back.job().type == JobType::DoubleSided);
    CHECK(back.job().flipDirection == FlipDirection::Vertical);
    CHECK(save(back, loaded.value().extras) == first);
}

TEST_CASE("a single-sided document writes no side keys", "[document][double-sided][carv]") {
    const auto bytes = save(Document{});
    const std::string text(bytes.begin(), bytes.end());
    CHECK(text.find("\"partner\"") == std::string::npos);
    CHECK(text.find("\"side\"") == std::string::npos);
}

TEST_CASE("a non-mutual or same-side pairing is an integrity error and cannot be saved",
          "[document][double-sided][carv]") {
    CHECK(pairedDocument().checkIntegrity().empty());

    Document nonMutual = pairedDocument();
    nonMutual.findSheet(nonMutual.sheets()[1].id)->partner = SheetId{};
    Document sameSide = pairedDocument();
    sameSide.findSheet(sameSide.sheets()[1].id)->side = Side::Front;
    Document dangling = pairedDocument();
    dangling.findSheet(dangling.sheets()[0].id)->partner = SheetId{9999};
    for (const Document* d : {&nonMutual, &sameSide, &dangling}) {
        const auto issues = d->checkIntegrity();
        REQUIRE_FALSE(issues.empty());
        CHECK(issues.front().kind == IntegrityIssue::Kind::BadSheetPartner);
        CHECK_FALSE(saveCarv(*d).ok());
    }
}

TEST_CASE("paired sheets plus keep-out zones or dimensions write the highest minor", "[document][double-sided][carv]") {
    Document doc = pairedDocument();
    REQUIRE(doc.execute(std::make_unique<SetKeepOutClearanceCommand>(2.0)));
    const auto bytes = save(doc);
    auto loaded = loadCarv(std::span<const std::uint8_t>(bytes));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorDoubleSided);
    CHECK(loaded.value().document.keepOutClearance() == 2.0);
    CHECK(save(loaded.value().document, loaded.value().extras) == bytes);
}

TEST_CASE("a document without paired sheets keeps its previous minor", "[document][double-sided][carv]") {
    Document plain;
    auto loaded = loadCarv(std::span<const std::uint8_t>(save(plain)));
    REQUIRE(loaded.ok());
    CHECK(loaded.value().formatMinor == kCarvFormatMinorBase);
    Document zones;
    REQUIRE(zones.execute(std::make_unique<SetKeepOutClearanceCommand>(1.0)));
    auto l2 = loadCarv(std::span<const std::uint8_t>(save(zones)));
    REQUIRE(l2.ok());
    CHECK(l2.value().formatMinor == kCarvFormatMinorKeepOut);
}
