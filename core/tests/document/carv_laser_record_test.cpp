// T-0323: Laser Cut / Fill records (ADR-0099) in .carv: the LaserCutFillParams alternative, its job.json
// params, the `laser` marking of the tool, minor 1.13, move caches of version 2 (beam power), exact round
// trips and hostile input.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <string>
#include <variant>
#include <vector>

#include "carv_json_reader.hpp"
#include "carv_moves.hpp"
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/laser.hpp"

using namespace carvetto;
using namespace carvetto::document;
using toolpath::Move;
using Bytes = std::vector<std::uint8_t>;

namespace {

toolpath::LaserCutFillParams fullLaser() {
    toolpath::LaserCutFillParams p;
    p.name = "Laser \"a\" \xC3\xA9";
    p.strategy = toolpath::LaserStrategy::HatchFill;
    p.power = 62.5;
    p.moveSpeed = 1234.5;
    p.cutDepth = 3.25;
    p.passDepth = 1.5;
    p.repeats = 3;
    p.overscan = 2.5;
    p.allowance = -0.125;
    p.direction = toolpath::CutDirection::Conventional;
    p.stepover = 0.35;
    p.hatchAngle = 0.5235987755982988;
    p.crossHatch = true;
    p.fillMode = toolpath::LaserFillMode::Regionally;
    p.outline = true;
    p.safeZ = 7.5;
    p.order.leftToRight = true;
    p.startAt = toolpath::StartAt::ClosestOnBoundingBox;
    p.startAnchor = toolpath::BoxAnchor::TopRight;
    p.startPosition = {-3.5, 2.25};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

tooling::Tool beam() {
    tooling::Tool t;
    t.id = "laser";
    t.name = "Laser";
    t.diameter = 0.2;
    t.defaultCuttingData = {1.0, 0.2, 0.0, 0.0, 0.0, 900.0, 900.0, 2000.0, 1};
    return t;
}

Document laserDocument(const toolpath::LaserCutFillParams& params) {
    Document doc;
    const geometry::Contour c = geometry::Contour::rectangle(geometry::Box2({0, 0}, {10, 10}));
    const ObjectId a = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
    toolpath::LaserCutFillResult r = toolpath::laserCutFillToolpath({c}, beam(), std::nullopt, params);
    INFO((r.diagnostics.empty() ? std::string() : r.diagnostics.front().message));
    REQUIRE(r.ok());
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {a};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"laser", "", ""};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

std::vector<detail::ZipMember> members(const Bytes& archive) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    return r.members;
}

Bytes member(const Bytes& archive, const std::string& name) {
    for (const auto& m : members(archive)) {
        if (m.name == name) {
            return m.data;
        }
    }
    FAIL("member " << name << " not found");
    return {};
}

Bytes withMember(const Bytes& archive, const std::string& name, const Bytes& data) {
    auto ms = members(archive);
    for (auto& m : ms) {
        if (m.name == name) {
            m.data = data;
        }
    }
    auto r = detail::writeZip(ms);
    REQUIRE(r.ok);
    return r.bytes;
}

std::string decodeError(const Bytes& cache, std::uint64_t expected) {
    const std::string text(cache.begin(), cache.end());
    detail::StringSource src(text);
    std::vector<Move> out;
    return detail::decodeMoves(src, expected, out);
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

std::uint32_t versionOf(const Bytes& cache) {
    return static_cast<std::uint32_t>(cache[4]) | (static_cast<std::uint32_t>(cache[5]) << 8);
}

}  // namespace

TEST_CASE("T-0323: laser records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv][laser]") {
    const Document doc = laserDocument(fullLaser());
    REQUIRE(doc.toolpaths().size() == 1);
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::LaserCutFill);
    CHECK(doc.toolpaths()[0].safeZ() == 7.5);
    CHECK(doc.toolpaths()[0].tool.laser);
    CHECK(doc.toolpaths()[0].moveCount() > 4);
    auto saved = saveCarv(doc, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    REQUIRE(back.value().document.toolpaths().size() == 1);
    CHECK(back.value().document.toolpaths()[0] == doc.toolpaths()[0]);
    CHECK(back.value().formatMinor == kCarvFormatMinorLaser);
    CHECK(std::get<toolpath::LaserCutFillParams>(back.value().document.toolpaths()[0].params) == fullLaser());
    // Beam power survives on the moves.
    bool any = false;
    for (const Move& m : *back.value().document.toolpaths()[0].moves) {
        any = any || m.jetPower == 62.5;
    }
    CHECK(any);
    auto again = saveCarv(back.value().document, back.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
}

TEST_CASE("T-0323: every strategy and fill mode survives a save and load", "[document][toolpath][carv][laser]") {
    for (const auto s : {toolpath::LaserStrategy::CutOutside, toolpath::LaserStrategy::CutInside,
                         toolpath::LaserStrategy::CutOn, toolpath::LaserStrategy::HatchFill}) {
        for (const auto f : {toolpath::LaserFillMode::Together, toolpath::LaserFillMode::Regionally}) {
            toolpath::LaserCutFillParams p = fullLaser();
            p.strategy = s;
            p.fillMode = f;
            p.allowance = s == toolpath::LaserStrategy::HatchFill ? -0.125 : 0.05;
            const Document doc = laserDocument(p);
            auto saved = saveCarv(doc, CarvExtras{});
            REQUIRE(saved.ok());
            auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
            REQUIRE(back.ok());
            CHECK(std::get<toolpath::LaserCutFillParams>(back.value().document.toolpaths()[0].params) == p);
        }
    }
}

TEST_CASE("T-0323: the move cache is version 1 without beam power and version 2 with it", "[document][toolpath][moves][laser]") {
    Bytes plain;
    REQUIRE(detail::encodeMoves({Move::rapid({0, 0, 5}), Move::feed({1, 0, 0})}, plain).empty());
    CHECK(versionOf(plain) == 1);
    Bytes laser;
    const std::vector<Move> moves{Move::rapid({0, 0, 0}), Move::feed({1, 0, 0}).withJet(40.0),
                                  Move::feed({2, 0, 0}).withJet(40.0), Move::feed({3, 0, 0}),
                                  Move::arcCW({3, 1, 0}, {3, 0.5}).withJet(80.0)};
    REQUIRE(detail::encodeMoves(moves, laser).empty());
    CHECK(versionOf(laser) == 2);
    const std::string text(laser.begin(), laser.end());
    detail::StringSource src(text);
    std::vector<Move> back;
    REQUIRE(detail::decodeMoves(src, moves.size(), back).empty());
    CHECK(back == moves);
    // A repeated power is stored once: the cache is shorter than one with two distinct powers.
    std::vector<Move> varied = moves;
    varied[2] = Move::feed({2, 0, 0}).withJet(41.0);
    Bytes other;
    REQUIRE(detail::encodeMoves(varied, other).empty());
    CHECK(other.size() == laser.size() + 8);
}

TEST_CASE("T-0323: move cache defects of the laser version are reported, never crash",
          "[document][toolpath][moves][laser][errors]") {
    const auto encoded = [](const std::vector<Move>& ms) {
        Bytes b;
        REQUIRE(detail::encodeMoves(ms, b).empty());
        return b;
    };
    const Bytes good = encoded({Move::rapid({0, 0, 0}), Move::feed({1, 0, 0}).withJet(40.0)});
    CHECK(decodeError(good, 2).empty());
    // Hand-made second move: tag, flags, then the values.
    const auto withTail = [&](std::vector<std::uint8_t> tail) {
        Bytes b = encoded({Move::rapid({0, 0, 0}), Move::feed({1, 0, 0}).withJet(40.0)});
        b.resize(detail::kMovesHeaderBytes + 2 + 0);  // header + the first move (tag + flag byte); same x/y/z
        b.insert(b.end(), tail.begin(), tail.end());
        return b;
    };
    // kind feed (1) with same x/y/z and a flag byte of 0x04 (unknown bit).
    CHECK(contains(decodeError(withTail({0x01 | 0x70, 0x04}), 2), "invalid laser flags"));
    // "same power" without a power flag.
    CHECK(contains(decodeError(withTail({0x01 | 0x70, 0x02}), 2), "invalid laser flags"));
    // "same power" with no earlier power.
    CHECK(contains(decodeError(withTail({0x01 | 0x70, 0x03}), 2), "previous beam power"));
    // Power on a rapid.
    CHECK(contains(decodeError(withTail({0x00 | 0x70, 0x01, 0, 0, 0, 0, 0, 0, 0x44, 0x40}), 2), "rapid or dwell"));
    // Power 0, 101, NaN and -5 on a feed (doubles written little endian).
    const auto bitsOfDouble = [](double v) {
        std::uint64_t u = 0;
        static_assert(sizeof(u) == sizeof(v));
        std::memcpy(&u, &v, sizeof(u));
        std::vector<std::uint8_t> b;
        for (int i = 0; i < 8; ++i) {
            b.push_back(static_cast<std::uint8_t>(u >> (8 * i)));
        }
        return b;
    };
    for (const double bad : {0.0, 101.0, -5.0, std::numeric_limits<double>::quiet_NaN(),
                             std::numeric_limits<double>::infinity()}) {
        std::vector<std::uint8_t> tail{0x01 | 0x70, 0x01};
        const auto v = bitsOfDouble(bad);
        tail.insert(tail.end(), v.begin(), v.end());
        CHECK(contains(decodeError(withTail(tail), 2), "beam power"));
    }
    // Encoding refuses bad power too.
    for (const double bad : {101.0, -1.0, std::numeric_limits<double>::quiet_NaN()}) {
        Bytes out;
        CHECK(contains(detail::encodeMoves({Move::feed({1, 0, 0}).withJet(bad)}, out), "beam power"));
    }
    Bytes out;
    CHECK(contains(detail::encodeMoves({Move::rapid({1, 0, 0}).withJet(10.0)}, out), "beam power"));
    // Truncation anywhere is reported.
    for (std::size_t cut = detail::kMovesHeaderBytes; cut + 1 < good.size(); ++cut) {
        CHECK_FALSE(decodeError(Bytes(good.begin(), good.begin() + static_cast<std::ptrdiff_t>(cut)), 2).empty());
    }
}

TEST_CASE("T-0323: the laser marking and operation are only accepted together", "[document][toolpath][laser]") {
    Document doc = laserDocument(fullLaser());
    // A laser tool on another operation is refused by the commands.
    ToolpathRecord other = doc.toolpaths()[0];
    other.params = toolpath::ProfileParams{};
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(other)));
    CHECK_FALSE(spindleOffProblem(other).empty());
    // A laser snapshot cannot also be spindle-off or carry a tool type.
    ToolpathRecord mixed = doc.toolpaths()[0];
    mixed.tool.spindleOff = true;
    CHECK_FALSE(spindleOffProblem(mixed).empty());
    mixed = doc.toolpaths()[0];
    mixed.tool.toolType = "diamond_drag";
    CHECK_FALSE(spindleOffProblem(mixed).empty());
    // A laser operation with an unmarked (not yet computed) snapshot is fine.
    ToolpathRecord fresh = doc.toolpaths()[0];
    fresh.tool = {};
    fresh.moves = nullptr;
    CHECK(spindleOffProblem(fresh).empty());
    CHECK(doc.execute(std::make_unique<SetToolpathCommand>(fresh)));
}

TEST_CASE("T-0323: the file is 1.13 only when a laser record is saved", "[document][toolpath][carv][laser]") {
    Document plain;
    auto saved = saveCarv(plain, CarvExtras{});
    REQUIRE(saved.ok());
    auto back = loadCarv(std::span<const std::uint8_t>(saved.value()));
    REQUIRE(back.ok());
    CHECK(back.value().formatMinor < kCarvFormatMinorLaser);
    auto laser = saveCarv(laserDocument(fullLaser()), CarvExtras{});
    REQUIRE(laser.ok());
    const Bytes manifestBytes = member(laser.value(), "manifest.json");
    const std::string manifest(manifestBytes.begin(), manifestBytes.end());
    CHECK((contains(manifest, "\"minor\":13") || contains(manifest, "\"minor\": 13")));
}

TEST_CASE("T-0323: non-finite laser parameters are refused by the commands", "[document][toolpath][laser]") {
    Document doc = laserDocument(fullLaser());
    ToolpathRecord edit = doc.toolpaths()[0];
    const double inf = std::numeric_limits<double>::infinity();
    auto& p = std::get<toolpath::LaserCutFillParams>(edit.params);
    struct Field {
        double toolpath::LaserCutFillParams::*member;
        const char* key;
    };
    const Field fields[] = {{&toolpath::LaserCutFillParams::power, "power"},
                            {&toolpath::LaserCutFillParams::moveSpeed, "move_speed"},
                            {&toolpath::LaserCutFillParams::cutDepth, "cut_depth"},
                            {&toolpath::LaserCutFillParams::passDepth, "pass_depth"},
                            {&toolpath::LaserCutFillParams::overscan, "overscan"},
                            {&toolpath::LaserCutFillParams::allowance, "allowance"},
                            {&toolpath::LaserCutFillParams::stepover, "stepover"},
                            {&toolpath::LaserCutFillParams::hatchAngle, "hatch_angle_rad"},
                            {&toolpath::LaserCutFillParams::safeZ, "safe_z"}};
    for (const Field& f : fields) {
        p = fullLaser();
        p.*(f.member) = inf;
        CHECK(nonFiniteParam(edit.params) == f.key);
        CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    }
    p = fullLaser();
    p.startPosition.y = inf;
    CHECK(nonFiniteParam(edit.params) == "start_position[1]");
    p = fullLaser();
    CHECK(nonFiniteParam(edit.params).empty());
}

TEST_CASE("T-0323: hostile laser records and caches never crash the loader", "[document][toolpath][carv][laser][fuzz]") {
    const Bytes good = [] {
        auto r = saveCarv(laserDocument(fullLaser()), CarvExtras{});
        REQUIRE(r.ok());
        return r.value();
    }();
    const Bytes cache = member(good, "toolpaths/0001.bin");
    const Bytes jobBytes = member(good, "job.json");
    const std::string job(jobBytes.begin(), jobBytes.end());
    std::mt19937 rng(323);
    int loaded = 0;
    for (int i = 0; i < 300; ++i) {
        Bytes file;
        if (i % 2 == 0) {
            Bytes c = cache;
            const unsigned edits = 1 + static_cast<unsigned>(rng() % 4);
            for (unsigned e = 0; e < edits && !c.empty(); ++e) {
                const std::size_t pos = detail::kMovesHeaderBytes + rng() % (c.size() - detail::kMovesHeaderBytes);
                c[pos] ^= static_cast<std::uint8_t>(1 + rng() % 255);
            }
            file = withMember(good, "toolpaths/0001.bin", c);
        } else {
            std::string t = job;
            const std::size_t from = t.find("\"toolpaths\"");
            REQUIRE(from != std::string::npos);
            const unsigned edits = 1 + static_cast<unsigned>(rng() % 3);
            for (unsigned e = 0; e < edits; ++e) {
                const std::size_t pos = from + rng() % (t.size() - from);
                t[pos] = "{}[]\",:.-+eE0123456789 truefalsnul#"[rng() % 35];
            }
            file = withMember(good, "job.json", Bytes(t.begin(), t.end()));
        }
        const auto r = loadCarv(std::span<const std::uint8_t>(file));
        if (r.ok()) {
            ++loaded;
            REQUIRE(r.value().document.checkIntegrity().empty());
            auto once = saveCarv(r.value().document, r.value().extras);
            REQUIRE(once.ok());
        } else {
            CHECK_FALSE(r.error().message.empty());
        }
    }
    CHECK(loaded < 300);
    // Specific hostile spellings of the records.
    const auto loadWith = [&](const std::string& from, const std::string& to) {
        std::string t = job;
        const std::size_t pos = t.find(from);
        REQUIRE(pos != std::string::npos);
        t.replace(pos, from.size(), to);
        return loadCarv(std::span<const std::uint8_t>(withMember(good, "job.json", Bytes(t.begin(), t.end()))));
    };
    CHECK_FALSE(loadWith("\"hatch_fill\"", "\"burn\"").ok());
    CHECK_FALSE(loadWith("\"power\": 62.5", "\"power\": \"NaN\"").ok());
    CHECK_FALSE(loadWith("\"laser_cut_fill\"", "\"profile\"").ok());  // laser marking on another operation
}
