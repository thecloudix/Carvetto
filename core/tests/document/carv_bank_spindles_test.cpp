// T-0335: per-stroke spindle sets of drill bank toolpaths in .carv (ADR-0111): the move cache version 4 trailer (exact
// layout, canonical runs, hostile defects), the 1.18 format minor written only when a move names spindles, the
// record checks (only a drill bank's rapid/plunge/retract/dwell moves, only spindles of its head) on save and load, and
// byte-identical files for banks without spindle sets.

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <utility>
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
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/drill_bank.hpp"
#include "carvetto/toolpath/profile.hpp"

using namespace carvetto;
using namespace carvetto::document;
using toolpath::Move;
using toolpath::MoveKind;
using Bytes = std::vector<std::uint8_t>;

namespace {

Bytes save(const Document& doc) {
    auto r = saveCarv(doc);
    INFO((r ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return r.value();
}

LoadedCarv load(const Bytes& bytes) {
    auto r = loadCarv(std::span<const std::uint8_t>(bytes));
    INFO((r ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return std::move(r.value());
}

CarvError loadError(const Bytes& bytes) {
    auto r = loadCarv(std::span<const std::uint8_t>(bytes));
    REQUIRE_FALSE(r.ok());
    return r.error();
}

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

std::string member(const Bytes& archive, const std::string& name) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (const auto& m : r.members) {
        if (m.name == name) {
            return {m.data.begin(), m.data.end()};
        }
    }
    FAIL("no member " << name);
    return {};
}

Bytes withMember(const Bytes& archive, const std::string& name, const std::string& text) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (auto& m : r.members) {
        if (m.name == name) {
            m.data.assign(text.begin(), text.end());
        }
    }
    auto w = detail::writeZip(r.members);
    REQUIRE(w.ok);
    return w.bytes;
}

std::string cacheName(const Bytes& archive) {
    auto r = detail::readZip(archive);
    REQUIRE(r.ok);
    for (const auto& m : r.members) {
        if (m.name.rfind("toolpaths/", 0) == 0) {
            return m.name;
        }
    }
    FAIL("no move cache");
    return {};
}

tooling::Tool drill() {
    tooling::Tool t;
    t.id = "drill-3mm";
    t.name = "Drill 3 mm";
    t.type = tooling::ToolType::Drill;
    t.diameter = 3.0;
    t.includedAngle = 2.0594885173533086;
    t.defaultCuttingData = {2.5, 0.0, 0.0, 0.0, 10000.0, 600.0, 250.0, 0.0, 2};
    return t;
}

toolpath::DrillBankParams threeSpindles() {
    toolpath::DrillBankParams p;
    p.horizontalDrills = 3;
    p.pitch = 32.0;
    p.cutDepth = 6.0;
    p.safeZ = 5.0;
    p.minHoles = 3;
    return p;
}

/// A document with a computed record of `params` over two rows of three holes at the pitch; `strip` clears the
/// spindle sets (a record as T-0328 made it).
Document bankDocument(const toolpath::DrillBankParams& params, bool strip = false) {
    Document doc;
    ObjectId first{};
    std::vector<geometry::Contour> contours;
    for (int r = 0; r < 2; ++r) {
        for (int i = 0; i < 3; ++i) {
            geometry::Contour c = geometry::Contour::circle({10.0 + 32.0 * i, 10.0 + 100.0 * r}, 1.5);
            const ObjectId id = doc.insertObject(Object::vector(c, doc.defaultLayer(), doc.activeSheet()));
            if (r == 0 && i == 0) {
                first = id;
            }
            contours.push_back(c);
        }
    }
    toolpath::DrillBankResult r = toolpath::drillBankToolpath(contours, drill(), std::nullopt, params);
    REQUIRE(r.ok());
    if (strip) {
        for (Move& m : r.toolpath.moves) {
            m.bank = 0;
        }
    }
    ToolpathRecord rec;
    rec.name = r.toolpath.name;
    rec.params = params;
    rec.selection.objects = {first};
    rec.selection.layers = {doc.defaultLayer()};
    rec.tool = r.toolpath.tool;
    rec.toolSource = {"drill-3mm", "machine-generic-router", "material-hardwood-mdf"};
    rec.moves = shareMoves(std::move(r.toolpath.moves));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
    return doc;
}

Bytes cacheOf(const std::vector<Move>& moves) {
    Bytes out;
    REQUIRE(detail::encodeMoves(moves, out).empty());
    return out;
}

std::string decodeError(const Bytes& cache, std::uint64_t expected) {
    const std::string text(cache.begin(), cache.end());
    detail::StringSource src(text);
    std::vector<Move> out;
    return detail::decodeMoves(src, expected, out);
}

void putU64(Bytes& out, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
}

Move banked(Move m, std::uint64_t set) {
    m.bank = set;
    return m;
}

/// Two strokes (sets 0b101 and 0b101) and a loose rapid before them.
std::vector<Move> strokes() {
    return {Move::rapid({0, 0, 5}),
            banked(Move::rapid({10, 10, 5}), 5),
            banked(Move::plunge({10, 10, -3}), 5),
            banked(Move::dwell({10, 10, -3}, 0.5), 5),
            banked(Move::retract({10, 10, 5}), 5),
            banked(Move::rapid({40, 10, 5}), 5),
            banked(Move::plunge({40, 10, -3}), 5),
            banked(Move::retract({40, 10, 5}), 5),
            banked(Move::rapid({70, 10, 5}), 2),
            banked(Move::plunge({70, 10, -3}), 2),
            banked(Move::retract({70, 10, 5}), 2)};
}

/// The cache with its trailer replaced by `runs` (count, then first / length / set per run).
Bytes withTrailer(const std::vector<Move>& moves, const std::vector<std::array<std::uint64_t, 3>>& runs,
                  std::optional<std::uint64_t> declared = std::nullopt) {
    Bytes cache = cacheOf(moves);
    // The good trailer is 8 + 24 * (runs of the moves) bytes.
    std::size_t good = 0;
    {
        std::size_t i = 0;
        while (i < moves.size()) {
            if (moves[i].bank == 0) {
                ++i;
                continue;
            }
            std::size_t j = i;
            while (j < moves.size() && moves[j].bank == moves[i].bank) {
                ++j;
            }
            ++good;
            i = j;
        }
    }
    cache.resize(cache.size() - (8 + 24 * good));
    putU64(cache, declared.value_or(runs.size()));
    for (const auto& r : runs) {
        for (const std::uint64_t v : r) {
            putU64(cache, v);
        }
    }
    return cache;
}

}  // namespace

TEST_CASE("T-0335: a drill bank record round-trips with its spindle sets, as format 1.18",
          "[document][toolpath][carv][drill_bank]") {
    const Document doc = bankDocument(threeSpindles());
    const ToolpathRecord& rec = doc.toolpaths()[0];
    REQUIRE(rec.hasMoves());
    for (const Move& m : *rec.moves) {
        CHECK(m.bank == 0b111);  // every stroke drills all three spindles
    }
    const Bytes once = save(doc);
    CHECK(contains(member(once, "manifest.json"), "\"minor\": 18"));
    CHECK(kCarvFormatMinorBankSpindles == 18);
    const std::string cache = member(once, cacheName(once));
    CHECK(cache.substr(0, 4) == "CVTP");
    CHECK(static_cast<std::uint8_t>(cache[4]) == 4);  // move cache version 4
    LoadedCarv back = load(once);
    CHECK(back.formatMinor == 18);
    REQUIRE(back.document.toolpaths().size() == 1);
    CHECK(back.document.toolpaths()[0] == rec);
    CHECK(*back.document.toolpaths()[0].moves == *rec.moves);
    CHECK(back.document.checkIntegrity().empty());
    // The head comes from the parameters and the tool snapshot.
    const std::vector<toolpath::BankSpindleSpec> head = back.document.toolpaths()[0].bankHead();
    REQUIRE(head.size() == 3);
    CHECK(head[0].offset == geometry::Vec2{0.0, 0.0});  // the reference spindle (1)
    CHECK(head[2].offset == geometry::Vec2{64.0, 0.0});
    CHECK(head[1].diameter == 3.0);
    CHECK(head[1].depth == 6.0);
    CHECK(back.document.toolpaths()[0].toolpath().bankHead == head);
    CHECK(save(back.document) == once);
}

TEST_CASE("T-0335: a drill bank record without spindle sets is written as before (minor 17, cache version 1)",
          "[document][toolpath][carv][drill_bank]") {
    const Document doc = bankDocument(threeSpindles(), true);
    const Bytes bytes = save(doc);
    CHECK(contains(member(bytes, "manifest.json"), "\"minor\": 17"));
    const std::string cache = member(bytes, cacheName(bytes));
    CHECK(static_cast<std::uint8_t>(cache[4]) == 1);
    CHECK(doc.toolpaths()[0].bankHead().empty());
    CHECK(doc.toolpaths()[0].toolpath().bankHead.empty());
    CHECK(load(bytes).formatMinor == 17);
    // A plain document still writes its old minor.
    CHECK(contains(member(save(Document{}), "manifest.json"), "\"minor\": 1"));
}

TEST_CASE("T-0335: the version 4 cache layout is exact and canonical", "[document][toolpath][carv][drill_bank]") {
    const std::vector<Move> moves = strokes();
    const Bytes cache = cacheOf(moves);
    CHECK(cache[4] == 4);
    // Runs: the two strokes of set 5 form one run (first move 1, 7 moves), the last stroke another.
    const std::size_t trailer = 8 + 24 * 2;
    REQUIRE(cache.size() > trailer);
    Bytes expected;
    putU64(expected, 2);
    putU64(expected, 1);
    putU64(expected, 7);
    putU64(expected, 5);
    putU64(expected, 8);
    putU64(expected, 3);
    putU64(expected, 2);
    CHECK(Bytes(cache.end() - static_cast<std::ptrdiff_t>(trailer), cache.end()) == expected);
    // Decoding restores every set; equal input, equal bytes.
    const std::string text(cache.begin(), cache.end());
    detail::StringSource src(text);
    std::vector<Move> back;
    REQUIRE(detail::decodeMoves(src, moves.size(), back).empty());
    REQUIRE(back.size() == moves.size());
    for (std::size_t i = 0; i < moves.size(); ++i) {
        CHECK(back[i].bank == moves[i].bank);
        CHECK(back[i].kind == moves[i].kind);
    }
    CHECK(cacheOf(back) == cache);
    // Without sets the cache is version 1 and has no trailer.
    std::vector<Move> plain = moves;
    for (Move& m : plain) {
        m.bank = 0;
    }
    CHECK(cacheOf(plain)[4] == 1);
}

TEST_CASE("T-0335: damaged spindle-set runs are refused with their own messages", "[document][toolpath][carv][drill_bank]") {
    const std::vector<Move> moves = strokes();
    const std::uint64_t n = moves.size();
    CHECK(decodeError(cacheOf(moves), n).empty());
    using Run = std::array<std::uint64_t, 3>;
    CHECK(decodeError(withTrailer(moves, {}), n).find("without drill bank runs") != std::string::npos);
    CHECK(contains(decodeError(withTrailer(moves, {Run{1, 7, 0}, Run{8, 3, 2}}), n), "an empty spindle set"));
    CHECK(contains(decodeError(withTrailer(moves, {Run{1, 0, 5}}), n), "outside the moves"));
    CHECK(contains(decodeError(withTrailer(moves, {Run{1, 11, 5}}), n), "outside the moves"));  // past the end
    CHECK(contains(decodeError(withTrailer(moves, {Run{12, 1, 5}}), n), "outside the moves"));
    CHECK(contains(decodeError(withTrailer(moves, {Run{1, 7, 5}, Run{7, 3, 2}}), n), "overlapping"));
    CHECK(contains(decodeError(withTrailer(moves, {Run{8, 3, 2}, Run{1, 7, 5}}), n), "overlapping"));  // out of order
    CHECK(contains(decodeError(withTrailer(moves, {Run{1, 4, 5}, Run{5, 3, 5}}), n), "without a gap"));
    // A run may not cover a move that is not a rapid, plunge, retract or dwell.
    std::vector<Move> withFeed = moves;
    withFeed[2] = Move::feed({10, 10, -3});
    CHECK(contains(decodeError(withTrailer(withFeed, {Run{1, 7, 5}, Run{8, 3, 2}}), n),
                   "not a rapid, plunge, retract or dwell"));
    // Truncated or over-long trailer.
    Bytes cut = cacheOf(moves);
    cut.resize(cut.size() - 5);
    CHECK(contains(decodeError(cut, n), "ends inside its drill bank runs"));
    Bytes noTrailer = cacheOf(moves);
    noTrailer.resize(noTrailer.size() - (8 + 48));
    CHECK(contains(decodeError(noTrailer, n), "before its drill bank runs"));
    Bytes extra = cacheOf(moves);
    extra.push_back(0);
    CHECK(contains(decodeError(extra, n), "follows the last move"));
    // A run count that promises more than the data holds.
    CHECK(contains(decodeError(withTrailer(moves, {Run{1, 7, 5}}, 2), n), "ends inside its drill bank runs"));
}

TEST_CASE("T-0335: encoding refuses spindle sets that cannot be stored", "[document][toolpath][carv][drill_bank]") {
    Bytes out;
    CHECK(contains(detail::encodeMoves({Move::rapid({0, 0, 5}), banked(Move::feed({1, 0, 0}), 1)}, out),
                   "only a rapid, plunge, retract or dwell"));
    out.clear();
    CHECK(contains(detail::encodeMoves({Move::rapid({0, 0, 5}), banked(Move::arcCW({1, 0, 0}, {0.5, 0}), 1)}, out),
                   "only a rapid, plunge, retract or dwell"));
    out.clear();
    Move laser = banked(Move::plunge({0, 0, -1}), 1);
    laser.jetPower = 50.0;
    CHECK(contains(detail::encodeMoves({laser}, out), "no beam power or cutter compensation"));
    out.clear();
    Move comp = banked(Move::plunge({0, 0, -1}), 1);
    comp.comp = toolpath::CutterComp::Left;
    CHECK(contains(detail::encodeMoves({comp}, out), "no beam power or cutter compensation"));
}

TEST_CASE("T-0335: spindle sets are checked against the record on save and load", "[document][toolpath][carv][drill_bank]") {
    // A head of two spindles cannot have a move naming spindle 3.
    const Document doc = bankDocument(threeSpindles());
    const Bytes good = save(doc);
    const std::string job = member(good, "job.json");
    const std::size_t at = job.find("\"horizontal_drills\": 3");
    REQUIRE(at != std::string::npos);
    std::string narrow = job;
    narrow.replace(at, std::string("\"horizontal_drills\": 3").size(), "\"horizontal_drills\": 2");
    const CarvError err = loadError(withMember(good, "job.json", narrow));
    CHECK(contains(err.message, "names a spindle beyond the head's 2"));
    CHECK(contains(err.message, "toolpaths[0]"));
    // A drill bank is the only record whose moves may name spindles.
    // Saving: the same moves on a Profile record are refused, naming the toolpath.
    Document other;
    ToolpathRecord rec = doc.toolpaths()[0];
    rec.params = toolpath::ProfileParams{};
    rec.selection = {};
    REQUIRE(other.execute(std::make_unique<AddToolpathCommand>(rec)));
    auto saved = saveCarv(other);
    REQUIRE_FALSE(saved.ok());
    CHECK(contains(saved.error().message, "names drill bank spindles, but the toolpath is not a drill bank"));
    // A drill bank move that is not a rapid, plunge, retract or dwell.
    ToolpathRecord bad = doc.toolpaths()[0];
    std::vector<Move> moves = *bad.moves;
    moves[2] = banked(Move::feed({10, 10, -3}), 7);
    bad.moves = shareMoves(std::move(moves));
    CHECK(contains(drillBankMovesProblem(bad), "not a rapid, plunge, retract or dwell"));
    // A head with no spindles to name (the parameters are damaged).
    ToolpathRecord empty = doc.toolpaths()[0];
    std::get<toolpath::DrillBankParams>(empty.params).horizontalDrills = 0;
    CHECK(contains(drillBankMovesProblem(empty), "no spindle set can name"));
    // A record without moves, or with unmarked moves, has nothing to check.
    ToolpathRecord none = doc.toolpaths()[0];
    none.moves = nullptr;
    CHECK(drillBankMovesProblem(none).empty());
    CHECK(drillBankMovesProblem(doc.toolpaths()[0]).empty());
}
