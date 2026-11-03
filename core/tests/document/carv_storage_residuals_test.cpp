// T-0136: native toolpath storage residuals (ADR-0021, ADR-0020; T-0132/T-0134 verifier findings).
//
// (a) saveCarv verifies its output without decoding a second copy of the moves.
// (b) clear messages for a computed toolpath past position 9,999 and for the member limit.
// (c) over-long names are reported as "name too long" with their field.
// (d) the interim counting pass shares the load budget (no over-reservation).
// (f) non-finite toolpath parameters are refused by the loader, saveCarv and the commands.
// (e) is tested in core/tests/post/post_safety_test.cpp.
// T-0139: (a) names measured as stored in job.json, (d) member-limit wording, RSS test skipped
// under AddressSanitizer.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
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
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"

#if defined(__linux__)
#include <fstream>
#if defined(__GLIBC__)
#include <malloc.h>
#endif
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

// AddressSanitizer inflates RSS (shadow memory, freed-memory quarantine): RSS bounds are skipped
// under it (T-0139). GCC and MSVC define __SANITIZE_ADDRESS__; Clang has __has_feature.
#if defined(__SANITIZE_ADDRESS__)
#define CARVETTO_TEST_ASAN 1
#elif defined(__has_feature)
#if __has_feature(address_sanitizer)
#define CARVETTO_TEST_ASAN 1
#endif
#endif
#ifndef CARVETTO_TEST_ASAN
#define CARVETTO_TEST_ASAN 0
#endif

using namespace carvetto;
using namespace carvetto::document;
using carvetto::toolpath::Move;
using Bytes = std::vector<std::uint8_t>;
using Code = CarvError::Code;

namespace {

const toolpath::ToolSnapshot kTool{"tool", 1, 6.0, 1000.0, 300.0, 18000.0};
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

bool contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

ToolpathRecord computed(std::vector<Move> moves) {
    ToolpathRecord r;
    r.name = "t";
    r.selection = VectorSelection::allVectors();
    r.tool = kTool;
    r.moves = shareMoves(std::move(moves));
    return r;
}

std::string saveError(const Document& doc, const CarvExtras& extras = {}) {
    const auto r = saveCarv(doc, extras);
    REQUIRE_FALSE(r.ok());
    return r.error().message;
}

Bytes save(const Document& doc) {
    auto r = saveCarv(doc);
    INFO((r ? std::string() : r.error().message));
    REQUIRE(r.ok());
    return r.value();
}

std::vector<detail::ZipMember> members(const Bytes& archive) {
    auto r = detail::readZip(archive);
    INFO(r.error);
    REQUIRE(r.ok);
    return r.members;
}

std::string memberText(const Bytes& archive, const std::string& name) {
    for (const auto& m : members(archive)) {
        if (m.name == name) {
            return {m.data.begin(), m.data.end()};
        }
    }
    FAIL("member " << name << " not found");
    return {};
}

Bytes withText(const Bytes& archive, const std::string& name, const std::string& text) {
    auto ms = members(archive);
    for (auto& m : ms) {
        if (m.name == name) {
            m.data.assign(text.begin(), text.end());
        }
    }
    auto r = detail::writeZip(ms);
    REQUIRE(r.ok);
    return r.bytes;
}

std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const std::size_t pos = text.find(from);
    INFO("looking for: " << from);
    REQUIRE(pos != std::string::npos);
    REQUIRE(text.find(from, pos + 1) == std::string::npos);  // unique
    text.replace(pos, from.size(), to);
    return text;
}

CarvError loadError(const Bytes& bytes) {
    auto r = loadCarv(std::span<const std::uint8_t>(bytes));
    REQUIRE_FALSE(r.ok());
    return r.error();
}

/// Raster-like moves: long feeds along x, a step in y, few repeated coordinates.
std::vector<Move> rasterMoves(std::size_t count) {
    std::vector<Move> v;
    v.reserve(count);
    double y = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
        const double x = (i % 2 == 0) ? 0.25 : 300.5 + static_cast<double>(i % 7) * 0.001;
        if (i % 2 == 0) {
            y += 0.4;
        }
        v.push_back(Move::feed({x, y, -1.5 - static_cast<double>(i % 3) * 0.125}));
    }
    return v;
}

const std::string kManifest10 =
    R"({"app_version": "0.1.0", "format": "carvetto", "format_version": {"major": 1, "minor": 0}})";
const std::string kJob10 = R"({
  "active_layer": 2, "active_sheet": 1,
  "job": {"type": "single_sided", "size_mm": [300.0, 200.0, 18.0], "display_units": "mm",
          "z_zero": "material_surface", "datum": "bottom_left", "datum_offset_mm": [0.0, 0.0],
          "flip_direction": "horizontal", "zero_off_same_side": false, "diameter_mm": 0.0,
          "rotary_axis": "x", "flip_design": false},
  "layers": [{"id": 2, "name": "Layer 1", "colour": "#000000", "visible": true, "locked": false}],
  "next_id": 10,
  "objects": [],
  "sheets": [{"id": 1, "name": "Sheet 1", "size_mm": [300, 200, 18], "z_zero": "material_surface",
              "datum": "bottom_left", "datum_offset_mm": [0, 0], "visible": true}],
  "variables": {}
})";

std::string interimMember(const std::string& params, const std::string& moves) {
    return R"({"format":"carvetto-toolpath","format_version":1,"moves":)" + moves +
           R"(,"name":"Old","operation":"pocket","params":)" + params +
           R"(,"safe_z":5.0,"tool":{"diameter":6.0,"feed_rate":2300.0,"name":"End Mill 6 mm","plunge_rate":800.0,"spindle_speed":16000.0,"tool_number":1}})";
}

Bytes legacyFile(const std::vector<std::pair<std::string, std::string>>& interim) {
    std::vector<detail::ZipMember> ms{{"manifest.json", Bytes(kManifest10.begin(), kManifest10.end())},
                                      {"job.json", Bytes(kJob10.begin(), kJob10.end())}};
    for (const auto& [name, text] : interim) {
        ms.push_back({name, Bytes(text.begin(), text.end())});
    }
    auto r = detail::writeZip(ms);
    REQUIRE(r.ok);
    return r.bytes;
}

#if defined(__linux__)
std::int64_t statusKb(const char* field) {
    std::ifstream status("/proc/self/status");
    std::string line;
    while (std::getline(status, line)) {
        if (line.rfind(field, 0) == 0) {
            return std::stoll(line.substr(std::string(field).size()));
        }
    }
    return -1;
}

/// Runs `fn` in a forked child and returns what it returns (-1 when the child fails).
std::int64_t inChild(const std::function<std::int64_t()>& fn) {
    int fds[2];
    if (pipe(fds) != 0) {
        return -1;
    }
    const pid_t pid = fork();
    if (pid == 0) {
        close(fds[0]);
        const std::int64_t result = fn();
        const ssize_t written = write(fds[1], &result, sizeof result);
        (void)written;
        _exit(0);
    }
    close(fds[1]);
    std::int64_t result = -1;
    if (pid > 0) {
        if (read(fds[0], &result, sizeof result) != static_cast<ssize_t>(sizeof result)) {
            result = -1;
        }
        int status = 0;
        waitpid(pid, &status, 0);
    }
    close(fds[0]);
    return result;
}

/// Peak RSS growth (bytes) while `fn` runs, measured as in carv_toolpath_test (heap trimmed,
/// high-water mark reset through /proc/self/clear_refs). -1 if unsupported or `fn` fails.
std::int64_t peakGrowth(const std::function<bool()>& fn) {
    return inChild([&fn]() -> std::int64_t {
#if defined(__GLIBC__)
        malloc_trim(0);
#endif
        {
            std::ofstream clear("/proc/self/clear_refs");
            clear << "5";
        }
        const std::int64_t before = statusKb("VmRSS:");
        if (before < 0 || statusKb("VmHWM:") > before + 1024) {
            return -1;
        }
        if (!fn()) {
            return -1;
        }
        return (statusKb("VmHWM:") - before) * 1024;
    });
}
#endif

}  // namespace

// ------------------------------------------------------------------------------------------------
// (a) verification without a second decoded copy
// ------------------------------------------------------------------------------------------------

TEST_CASE("T-0136 a: validateMoves accepts and refuses exactly what decodeMoves does", "[document][toolpath][moves]") {
    const std::vector<Move> moves{Move::rapid({0, 0, 5}),           Move::plunge({1, 2, -1}, 300.0),
                                  Move::feed({4, 2, -1}, 300.0),    Move::arcCW({6, 2, -1}, {5, 2}),
                                  Move::dwell({6, 2, -1}, 0.25),    Move::retract({6, 2, 5})};
    Bytes cache;
    REQUIRE(detail::encodeMoves(moves, cache).empty());
    std::mt19937 rng(136);
    int refused = 0;
    for (int trial = 0; trial < 3000; ++trial) {
        Bytes b = cache;
        const int edits = 1 + trial % 3;
        for (int e = 0; e < edits; ++e) {
            b[rng() % b.size()] = static_cast<std::uint8_t>(rng());
        }
        if (trial % 7 == 0) {
            b.resize(rng() % b.size());
        }
        const std::uint64_t expected = trial % 11 == 0 ? moves.size() + 1 : moves.size();
        const std::string text(b.begin(), b.end());
        detail::StringSource a(text);
        detail::StringSource v(text);
        std::vector<Move> out;
        const std::string decoded = detail::decodeMoves(a, expected, out);
        const std::string validated = detail::validateMoves(v, expected);
        INFO("trial " << trial);
        CHECK(decoded == validated);
        refused += decoded.empty() ? 0 : 1;
    }
    CHECK(refused > 1000);
    const std::string text(cache.begin(), cache.end());
    detail::StringSource ok(text);
    CHECK(detail::validateMoves(ok, moves.size()).empty());
}

TEST_CASE("T-0136 a: saveCarv's verification load holds no second copy of the moves",
          "[document][toolpath][carv][scale]") {
    constexpr std::size_t kMoves = 1'000'000;  // 72 MB decoded
    Document doc;
    doc.insertToolpath(computed(rasterMoves(kMoves)));
    // The saved file still loads with every move.
    const Bytes file = save(doc);
    {
        auto back = loadCarv(std::span<const std::uint8_t>(file));
        REQUIRE(back.ok());
        CHECK(back.value().document.toolpaths().front() == doc.toolpaths().front());
    }
#if defined(__linux__) && !CARVETTO_TEST_ASAN
    const std::int64_t peak = peakGrowth([&doc]() { return saveCarv(doc).ok(); });
    INFO("save peak growth " << (peak >> 20) << " MiB for " << kMoves << " moves");
    REQUIRE(peak >= 0);
    // Before T-0136 the verification decoded every move again (72 bytes each, 69 MiB here) on
    // top of the encoded cache and the archive. Now only those remain (about 10 + 6 MB here).
    CHECK(peak < static_cast<std::int64_t>(kMoves) * 72 / 2);
#elif CARVETTO_TEST_ASAN
    WARN("peak RSS not checked under AddressSanitizer (its shadow memory and quarantine inflate it)");
#endif
}

// ------------------------------------------------------------------------------------------------
// (b) position and member-count messages
// ------------------------------------------------------------------------------------------------

TEST_CASE("T-0136 b: a computed toolpath past position 9999 is named with its position",
          "[document][toolpath][carv][limits]") {
    Document doc;
    doc.insertToolpath(computed({Move::rapid({0, 0, 5})}));
    for (std::size_t i = 1; i < kCarvMaxCachedToolpaths; ++i) {
        ToolpathRecord r;
        r.selection = VectorSelection::allVectors();
        doc.insertToolpath(r);
    }
    CHECK(saveCarv(doc).ok());  // 9,999 records, one computed
    ToolpathRecord late = computed({Move::rapid({0, 0, 5})});
    late.name = "Late";
    const ToolpathId lateId = doc.insertToolpath(late);
    const std::string why = saveError(doc);
    INFO(why);
    CHECK(contains(why, "toolpath 'Late' (#" + std::to_string(lateId.value) + ") is at position 10000"));
    CHECK(contains(why, "only toolpaths at positions 1 to 9999 can keep computed moves"));
    CHECK_FALSE(contains(why, "more than 9999"));
    // Moved up the job order, it saves: only two toolpaths are computed.
    REQUIRE(doc.execute(std::make_unique<MoveToolpathCommand>(lateId, 1)));
    CHECK(saveCarv(doc).ok());
}

TEST_CASE("T-0136 b: the member limit message says how many computed toolpaths fit",
          "[document][toolpath][carv][limits]") {
    const SharedMoves one = shareMoves({Move::rapid({0, 0, 5})});
    Document doc;
    for (std::size_t i = 0; i < 9999; ++i) {
        ToolpathRecord r = computed({});
        r.moves = one;
        doc.insertToolpath(r);
    }
    const std::string why = saveError(doc);
    INFO(why);
    CHECK(contains(why, "10001 archive members (9999 toolpath move caches among them)"));
    CHECK(contains(why, "at most 9998 toolpaths with computed moves fit"));
    doc.takeToolpath(doc.toolpaths().back().id);
    const CarvExtras extra = CarvExtras{}.withMember("notes.txt", Bytes{'x'});
    const std::string withExtra = saveError(doc, extra);
    INFO(withExtra);
    CHECK(contains(withExtra, "at most 9997 toolpaths with computed moves fit next to the 1 preserved member "));
}

TEST_CASE("T-0139 d: the member limit message reads well with no caches or no room for one",
          "[document][toolpath][carv][limits]") {
    // A file with 9,998 preserved members: manifest + job + 9,998 = the 10,000-member limit.
    std::vector<detail::ZipMember> ms = members(save(Document{}));
    for (int i = 0; i < 9998; ++i) {
        ms.push_back({"notes/" + std::to_string(10000 + i) + ".txt", Bytes{'x'}});
    }
    const auto zip = detail::writeZip(ms);
    REQUIRE(zip.ok);
    auto loaded = loadCarv(std::span<const std::uint8_t>(zip.bytes));
    REQUIRE(loaded.ok());
    const CarvExtras full = loaded.value().extras;
    // One computed toolpath does not fit next to them.
    {
        Document d;
        d.insertToolpath(computed({Move::rapid({0, 0, 5})}));
        const std::string why = saveError(d, full);
        INFO(why);
        CHECK(contains(why, "10001 archive members (1 toolpath move cache among them)"));
        CHECK(contains(why, "no toolpath with computed moves fits next to the 9998 preserved members"));
        CHECK_FALSE(contains(why, "at most 0"));
    }
    // No caches at all: one preserved member too many.
    {
        const std::string why = saveError(Document{}, full.withMember("notes/extra.txt", Bytes{'y'}));
        INFO(why);
        CHECK(contains(why, "the file would have 10001 archive members; a .carv file holds at most 10000"));
        CHECK(contains(why, "so the file has too many preserved members: at most 9998 fit, not 9999"));
        CHECK_FALSE(contains(why, "move cache"));
        // With a computed toolpath too, clearing moves would not be enough: said so.
        Document d;
        d.insertToolpath(computed({Move::rapid({0, 0, 5})}));
        const std::string both = saveError(d, full.withMember("notes/extra.txt", Bytes{'y'}));
        INFO(both);
        CHECK(contains(both, "10002 archive members (1 toolpath move cache among them)"));
        CHECK(contains(both, "too many preserved members: at most 9998 fit, not 9999 (even without toolpath "
                             "move caches)"));
        CHECK_FALSE(contains(both, "clear the moves"));
    }
}

// ------------------------------------------------------------------------------------------------
// (c) over-long names
// ------------------------------------------------------------------------------------------------

TEST_CASE("T-0139 a: names are measured as decoded for the cap and as stored for the ratio",
          "[document][toolpath][carv][limits]") {
    const auto nameError = [](const std::string& name) {
        Document d;
        ToolpathRecord r;
        r.name = name;
        r.selection = VectorSelection::allVectors();
        d.insertToolpath(r);
        return saveError(d);
    };
    const std::string prefix = "the file would not load again: name too long: job.json.toolpaths[0].name holds ";
    // Escapes decode to one byte each: under the 1 MiB cap as loaded, these are refused only by
    // the 200:1 ratio of job.json, and reported with their stored size.
    struct Repetitive {
        const char* what;
        std::string name;
        std::size_t stored;
    };
    const std::vector<Repetitive> repetitive{
        {"control characters (sixfold)", std::string(200'000, '\x01'), 1'200'000},
        {"control characters (just over 1 MiB stored)", std::string(180'000, '\x01'), 1'080'000},
        {"control characters (below 1 MiB stored)", std::string(174'700, '\x01'), 1'048'200},
        {"control characters (above 1 MiB stored)", std::string(174'800, '\x01'), 1'048'800},
        {"quotes (twofold)", std::string(600'000, '"'), 1'200'000},
        {"backslashes (twofold)", std::string(600'000, '\\'), 1'200'000},
    };
    for (const Repetitive& c : repetitive) {
        INFO(c.what);
        const std::string why = nameError(c.name);
        INFO(why.substr(0, 400));
        CHECK(contains(why, prefix + std::to_string(c.stored) + " bytes of repetitive text when stored (" +
                                std::to_string(c.name.size()) + " in memory"));
        CHECK(contains(why, "200:1"));
        CHECK_FALSE(contains(why, "holds at most"));
        CHECK(why.size() < 400);
    }
    // Under 1 MiB as stored, but repetitive enough together with another field.
    {
        Document d;
        ToolpathRecord r;
        r.name = std::string(100'000, '\x02');  // 600,000 bytes stored
        r.toolSource.toolId = std::string(500'000, 'b');
        r.selection = VectorSelection::allVectors();
        d.insertToolpath(r);
        const std::string why = saveError(d);
        INFO(why);
        CHECK(contains(why, prefix + "600000 bytes of repetitive text when stored (100000 in memory"));
        CHECK(contains(why, "200:1"));
    }
    // Invalid UTF-8 is saved as U+FFFD and grows when loaded: the cap message, with the size the
    // loader checks (escapes counted once, not as stored).
    struct OverCap {
        const char* what;
        std::string name;
        std::size_t decoded;
    };
    std::string mixed;
    for (int i = 0; i < 200'000; ++i) {
        mixed += "\xC3\xA9\x80\"";  // e-acute (2 bytes), an invalid byte (3 when loaded), a quote (1)
    }
    const std::vector<OverCap> overCap{
        {"invalid UTF-8 (threefold)", std::string(400'000, '\xFF'), 1'200'000},
        {"invalid UTF-8 (just over)", std::string(350'000, '\xFF'), 1'050'000},
        {"valid and invalid UTF-8 with quotes", mixed, 1'200'000},  // 1,400,000 stored
    };
    for (const OverCap& c : overCap) {
        INFO(c.what);
        const std::string why = nameError(c.name);
        INFO(why.substr(0, 400));
        CHECK(contains(why, prefix + std::to_string(c.decoded) + " bytes when loaded (" +
                                std::to_string(c.name.size()) + " in memory"));
        CHECK(contains(why, "; a name or text holds at most 1048576"));
        CHECK_FALSE(contains(why, "decompression bomb"));
        CHECK(why.size() < 400);
    }
    // Names far over 1 MiB as stored but under it as loaded, and varied enough to compress within
    // the ratio, save and load back unchanged.
    std::mt19937 rng(139);
    for (const bool all : {false, true}) {
        INFO((all ? "control characters, quotes and backslashes" : "control characters"));
        std::string name;
        const int count = all ? 300'000 : 150'000;
        for (int i = 0; i < count; ++i) {
            const unsigned pick = static_cast<unsigned>(rng() % 33);
            name += pick == 31 && all ? '"' : pick == 32 && all ? '\\' : static_cast<char>(1 + pick % 31);
        }
        Document d;
        ToolpathRecord r;
        r.name = name;
        r.selection = VectorSelection::allVectors();
        d.insertToolpath(r);
        const Bytes file = save(d);
        CHECK(memberText(file, "job.json").size() > (all ? 1'500'000U : 800'000U));
        auto back = loadCarv(std::span<const std::uint8_t>(file));
        REQUIRE(back.ok());
        CHECK(back.value().document.toolpaths().front().name == name);
    }
}

TEST_CASE("T-0136 c: over-long names are reported as such, with the field", "[document][toolpath][carv][limits]") {
    const std::string repetitive(1'100'000, 'n');  // over the 1 MiB string cap, compresses > 1000:1
    std::string noiseText(1'100'000, ' ');
    std::mt19937 rng(7);
    for (char& c : noiseText) {
        c = static_cast<char>('a' + rng() % 26);
    }
    const std::string noise = noiseText;
    for (const std::string* text : {&repetitive, &noise}) {
        INFO((text == &noise ? "random" : "repetitive"));
        {
            Document d;
            ToolpathRecord r = computed({Move::rapid({1, 2, 30})});
            r.name = *text;
            d.insertToolpath(r);
            const std::string why = saveError(d);
            CHECK(contains(why, "the file would not load again: name too long: job.json.toolpaths[0].name holds "
                                "1100000 bytes; a name or text holds at most 1048576"));
            CHECK_FALSE(contains(why, "decompression bomb"));
        }
        {
            Document d;
            Layer l;
            l.name = *text;
            d.insertLayer(l);
            const std::string why = saveError(d);
            CHECK(contains(why, "name too long: job.json.layers[1].name holds 1100000 bytes"));
        }
        {
            Document d;
            ToolpathRecord r = computed({Move::rapid({1, 2, 30})});
            r.tool.name = *text;
            d.insertToolpath(r);
            CHECK(contains(saveError(d), "name too long: job.json.toolpaths[0].tool.name"));
        }
    }
    // Under the cap each, but repetitive enough together to trip the 200:1 ratio of job.json.
    {
        Document d;
        ToolpathRecord r = computed({Move::rapid({1, 2, 30})});
        r.name = std::string(700'000, 'a');
        r.toolSource.toolId = std::string(650'000, 'b');
        d.insertToolpath(r);
        const std::string why = saveError(d);
        INFO(why);
        CHECK(contains(why, "name too long: job.json.toolpaths[0].name holds 700000 bytes of repetitive text"));
        CHECK(contains(why, "200:1"));
    }
    // A long key (a variable name) is cut in the path, so the message stays short.
    {
        Document d;
        REQUIRE(d.variables().set(std::string(2'000'000, 'v'), 1.0));
        const std::string why = saveError(d);
        INFO(why.substr(0, 300));
        CHECK(contains(why, "name too long: job.json.variables." + std::string(64, 'v') + "... (key) holds 2000000 bytes"));
        CHECK(why.size() < 400);
    }
    // A long name under the cap that compresses within the ratio still saves.
    {
        Document d;
        ToolpathRecord r = computed({Move::rapid({1, 2, 30})});
        r.name = std::string(900'000, 'a');
        d.insertToolpath(r);
        CHECK(saveCarv(d).ok());
    }
}

// ------------------------------------------------------------------------------------------------
// (d) the interim counting pass shares the budget
// ------------------------------------------------------------------------------------------------

TEST_CASE("T-0136 d: the interim counting pass is bounded by what the shared budget has left",
          "[document][toolpath][carv][limits][migration]") {
    constexpr std::size_t kMoves = 600'000;  // 43 MB if reserved
    std::string big = "[";
    big.reserve(kMoves * 34);
    std::mt19937 rng(42);
    for (std::size_t i = 0; i < kMoves; ++i) {
        big += (i ? ",[\"feed\"," : "[\"feed\",") + std::to_string(rng() % 100000) + ".5," +
               std::to_string(rng() % 100000) + ".25,-1.0]";
    }
    big += "]";
    const std::string small = R"([["rapid",0.0,0.0,5.0],["rapid",1.0,0.0,5.0]])";
    const Bytes file =
        legacyFile({{"toolpaths/0001.json", interimMember("{}", small)}, {"toolpaths/0002.json", interimMember("{}", big)}});
    big.clear();
    big.shrink_to_fit();
    detail::LoadLimits limits;
    limits.movesPerArchiveByte = 1000;
    limits.maxMoves = 2 + kMoves;
    CHECK(detail::loadCarvWithLimits(file, limits, detail::ZipLimits{}).ok());
    limits.maxMoves = 2 + kMoves - 1;  // the second member does not fit after the first
    const auto refused = detail::loadCarvWithLimits(file, limits, detail::ZipLimits{});
    REQUIRE_FALSE(refused.ok());
    CHECK(refused.error().code == Code::ResourceLimit);
    CHECK(contains(refused.error().message, "more than " + std::to_string(2 + kMoves - 1) + " toolpath moves"));
#if defined(__linux__)
    // Before T-0136 the counting pass used a fresh budget, so the real pass reserved all 600,000
    // moves (43 MB of address space) before the shared budget refused the member. With the
    // address space capped 24 MiB above the current size, that reservation fails ("out of
    // memory"); now the counting pass itself stops at the shared budget.
    const std::int64_t outcome = inChild([&file, &limits]() -> std::int64_t {
        const std::int64_t sizeKb = statusKb("VmSize:");
        if (sizeKb < 0) {
            return -1;
        }
        rlimit lim{};
        lim.rlim_cur = lim.rlim_max = static_cast<rlim_t>(sizeKb * 1024 + (std::int64_t{24} << 20));
        if (setrlimit(RLIMIT_AS, &lim) != 0) {
            return -1;
        }
        const auto r = detail::loadCarvWithLimits(file, limits, detail::ZipLimits{});
        if (r.ok()) {
            return 0;
        }
        return contains(r.error().message, "toolpath moves") ? 1 : 2;  // 2: out of memory
    });
    CHECK(outcome == 1);
#endif
}

// ------------------------------------------------------------------------------------------------
// (f) non-finite parameters
// ------------------------------------------------------------------------------------------------

namespace {

/// Every number of a params variant, with its job.json key path as `nonFiniteParam` names it.
using Setter = std::function<void(ToolpathParams&, double)>;
struct ParamField {
    const char* key;
    Setter set;
};

std::vector<ParamField> profileFields() {
    const auto P = [](ToolpathParams& p) -> toolpath::ProfileParams& { return std::get<toolpath::ProfileParams>(p); };
    return {
        {"allowance", [P](ToolpathParams& p, double v) { P(p).allowance = v; }},
        {"passes.start_depth", [P](ToolpathParams& p, double v) { P(p).passes.startDepth = v; }},
        {"passes.cut_depth", [P](ToolpathParams& p, double v) { P(p).passes.cutDepth = v; }},
        {"passes.pass_depth", [P](ToolpathParams& p, double v) { P(p).passes.passDepth = v; }},
        {"passes.last_pass_thickness", [P](ToolpathParams& p, double v) { P(p).passes.lastPassThickness = v; }},
        {"safe_z", [P](ToolpathParams& p, double v) { P(p).safeZ = v; }},
        {"last_pass_allowance", [P](ToolpathParams& p, double v) { P(p).lastPassAllowance = v; }},
        {"tabs.length", [P](ToolpathParams& p, double v) { P(p).tabs.length = v; }},
        {"tabs.thickness", [P](ToolpathParams& p, double v) { P(p).tabs.thickness = v; }},
        {"tabs.distance", [P](ToolpathParams& p, double v) { P(p).tabs.distance = v; }},
        {"ramp.distance", [P](ToolpathParams& p, double v) { P(p).ramp.distance = v; }},
        {"ramp.angle_rad", [P](ToolpathParams& p, double v) { P(p).ramp.angle = v; }},
        {"leads.length", [P](ToolpathParams& p, double v) { P(p).leads.length = v; }},
        {"leads.angle_rad", [P](ToolpathParams& p, double v) { P(p).leads.angle = v; }},
        {"leads.radius", [P](ToolpathParams& p, double v) { P(p).leads.radius = v; }},
        {"leads.sweep_rad", [P](ToolpathParams& p, double v) { P(p).leads.sweep = v; }},
        {"leads.overcut", [P](ToolpathParams& p, double v) { P(p).leads.overcut = v; }},
        {"start_position[0]", [P](ToolpathParams& p, double v) { P(p).startPosition.x = v; }},
        {"start_position[1]", [P](ToolpathParams& p, double v) { P(p).startPosition.y = v; }},
        {"corners.mitre_limit", [P](ToolpathParams& p, double v) { P(p).corners.mitreLimit = v; }},
        {"tolerance.distance", [P](ToolpathParams& p, double v) { P(p).tolerance.distance = v; }},
        {"tolerance.angle_rad", [P](ToolpathParams& p, double v) { P(p).tolerance.angle = v; }},
        {"tolerance.chordal", [P](ToolpathParams& p, double v) { P(p).tolerance.chordal = v; }},
    };
}

std::vector<ParamField> pocketFields() {
    const auto P = [](ToolpathParams& p) -> toolpath::PocketParams& { return std::get<toolpath::PocketParams>(p); };
    return {
        {"start_depth", [P](ToolpathParams& p, double v) { P(p).startDepth = v; }},
        {"cut_depth", [P](ToolpathParams& p, double v) { P(p).cutDepth = v; }},
        {"last_pass_thickness", [P](ToolpathParams& p, double v) { P(p).lastPassThickness = v; }},
        {"safe_z", [P](ToolpathParams& p, double v) { P(p).safeZ = v; }},
        {"raster_angle_rad", [P](ToolpathParams& p, double v) { P(p).rasterAngle = v; }},
        {"allowance", [P](ToolpathParams& p, double v) { P(p).allowance = v; }},
        {"ramp_distance", [P](ToolpathParams& p, double v) { P(p).rampDistance = v; }},
        {"tolerance.distance", [P](ToolpathParams& p, double v) { P(p).tolerance.distance = v; }},
        {"tolerance.angle_rad", [P](ToolpathParams& p, double v) { P(p).tolerance.angle = v; }},
        {"tolerance.chordal", [P](ToolpathParams& p, double v) { P(p).tolerance.chordal = v; }},
    };
}

std::vector<ParamField> drillFields() {
    const auto P = [](ToolpathParams& p) -> toolpath::DrillParams& { return std::get<toolpath::DrillParams>(p); };
    return {
        {"start_depth", [P](ToolpathParams& p, double v) { P(p).startDepth = v; }},
        {"cut_depth", [P](ToolpathParams& p, double v) { P(p).cutDepth = v; }},
        {"safe_z", [P](ToolpathParams& p, double v) { P(p).safeZ = v; }},
        {"retract_gap", [P](ToolpathParams& p, double v) { P(p).retractGap = v; }},
        {"dwell_time", [P](ToolpathParams& p, double v) { P(p).dwellTime = v; }},
        {"start_position[0]", [P](ToolpathParams& p, double v) { P(p).startPosition.x = v; }},
        {"start_position[1]", [P](ToolpathParams& p, double v) { P(p).startPosition.y = v; }},
        {"tolerance.distance", [P](ToolpathParams& p, double v) { P(p).tolerance.distance = v; }},
        {"tolerance.angle_rad", [P](ToolpathParams& p, double v) { P(p).tolerance.angle = v; }},
        {"tolerance.chordal", [P](ToolpathParams& p, double v) { P(p).tolerance.chordal = v; }},
    };
}

std::vector<ParamField> engraveFields() {
    const auto P = [](ToolpathParams& p) -> toolpath::EngraveParams& { return std::get<toolpath::EngraveParams>(p); };
    return {
        {"depth", [P](ToolpathParams& p, double v) { P(p).depth = v; }},
        {"safe_z", [P](ToolpathParams& p, double v) { P(p).safeZ = v; }},
        {"stepover", [P](ToolpathParams& p, double v) { P(p).stepover = v; }},
        {"raster_angle_rad", [P](ToolpathParams& p, double v) { P(p).rasterAngle = v; }},
        {"start_position[0]", [P](ToolpathParams& p, double v) { P(p).startPosition.x = v; }},
        {"start_position[1]", [P](ToolpathParams& p, double v) { P(p).startPosition.y = v; }},
        {"tolerance.distance", [P](ToolpathParams& p, double v) { P(p).tolerance.distance = v; }},
        {"tolerance.angle_rad", [P](ToolpathParams& p, double v) { P(p).tolerance.angle = v; }},
        {"tolerance.chordal", [P](ToolpathParams& p, double v) { P(p).tolerance.chordal = v; }},
    };
}

/// Params with every number set to a distinct value (777.015625, 778.015625, ...).
ToolpathParams distinctParams(const ToolpathParams& base, const std::vector<ParamField>& fields) {
    ToolpathParams p = base;
    for (std::size_t i = 0; i < fields.size(); ++i) {
        fields[i].set(p, 777.015625 + static_cast<double>(i));
    }
    return p;
}

std::string distinctText(std::size_t i) {
    return std::to_string(777 + i) + ".015625";
}

}  // namespace

TEST_CASE("T-0136 f: non-finite toolpath parameters are refused by the loader, saveCarv and the commands",
          "[document][toolpath][carv][errors]") {
    const std::vector<std::pair<ToolpathParams, std::vector<ParamField>>> operations{
        {toolpath::ProfileParams{}, profileFields()},
        {toolpath::PocketParams{}, pocketFields()},
        {toolpath::DrillParams{}, drillFields()},
        {toolpath::EngraveParams{}, engraveFields()}};
    for (const auto& [base, fields] : operations) {
        INFO("operation " << base.index());
        CHECK(nonFiniteParam(base).empty());
        const ToolpathParams distinct = distinctParams(base, fields);
        CHECK(nonFiniteParam(distinct).empty());

        // A saved file with every parameter distinct, then each one replaced by NaN / ±Infinity.
        Document doc;
        ToolpathRecord rec = computed({Move::rapid({0, 0, 5})});
        rec.params = distinct;
        REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(rec)));
        const Bytes good = save(doc);
        const std::string job = memberText(good, "job.json");
        const ToolpathId id = doc.toolpaths().front().id;

        for (std::size_t i = 0; i < fields.size(); ++i) {
            const ParamField& f = fields[i];
            INFO("field " << f.key);
            for (const double bad : {kNaN, kInf, -kInf}) {
                // nonFiniteParam names the field.
                ToolpathParams p = distinct;
                f.set(p, bad);
                CHECK(nonFiniteParam(p) == f.key);

                // The commands refuse it (no-op).
                ToolpathRecord edit = doc.toolpaths().front();
                edit.params = p;
                CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
                CHECK(doc.toolpaths().front().params == distinct);
                ToolpathRecord added = edit;
                added.id = {};
                const std::size_t before = doc.toolpaths().size();
                CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(added)));
                CHECK(doc.toolpaths().size() == before);

                // saveCarv refuses a record inserted raw, naming the toolpath and the field.
                Document raw;
                ToolpathRecord r = computed({Move::rapid({0, 0, 5})});
                r.name = "Raw";
                r.params = p;
                const ToolpathId rawId = raw.insertToolpath(r);
                const auto refused = saveCarv(raw);
                REQUIRE_FALSE(refused.ok());
                CHECK(refused.error().code == Code::InvalidContent);
                CHECK(refused.error().message == "toolpath 'Raw' (#" + std::to_string(rawId.value) + "): parameter " +
                                                     f.key + " is not a finite number");
            }
            // The loader refuses the ADR-0013 spellings in params.
            for (const char* spelled : {"\"NaN\"", "\"Infinity\"", "\"-Infinity\""}) {
                INFO(spelled);
                const CarvError e = loadError(withText(good, "job.json", replaced(job, distinctText(i), spelled)));
                CHECK(e.code == Code::InvalidContent);
                CHECK(e.message == "job.json.toolpaths[0].params." + std::string(f.key) + ": must be finite");
            }
        }
        CHECK(doc.toolpaths().front().id == id);
    }

    // Interim members (params.json dialect, degrees) are refused on migration too.
    const std::string moves = R"([["rapid",0.0,0.0,5.0]])";
    CHECK(loadCarv(legacyFile({{"toolpaths/0001.json", interimMember(R"({"safe_z": 4.0})", moves)}})).ok());
    for (const std::string params : {R"({"safe_z": "NaN"})", R"({"raster_angle": "Infinity"})",
                                     R"({"tolerance": {"chordal": "-Infinity"}})"}) {
        INFO(params);
        const CarvError e = loadError(legacyFile({{"toolpaths/0001.json", interimMember(params, moves)}}));
        CHECK(e.code == Code::InvalidContent);
        CHECK(contains(e.message, "toolpaths/0001.json.params."));
        CHECK(contains(e.message, ": must be finite"));
    }

    // Other non-finite values keep their ADR-0013 behaviour: a record without moves may keep a
    // non-finite tool snapshot (filled in when the toolpath is calculated).
    Document pending;
    ToolpathRecord uncomputed;
    uncomputed.tool.spindleSpeed = kNaN;
    pending.insertToolpath(uncomputed);
    CHECK(saveCarv(pending).ok());
}
