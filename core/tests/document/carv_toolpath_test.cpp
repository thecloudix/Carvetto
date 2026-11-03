// T-0132: toolpath records in the document and in .carv files (ADR-0021): job.json definitions,
// the binary move cache toolpaths/NNNN.bin, migration of T-0122's interim toolpaths/NNNN.json
// members (ADR-0019), the move budget, and fuzzing of all of it.

#include <algorithm>
#include <array>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstring>
#include <csignal>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <tuple>
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
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "../geometry/cpu_time.hpp"

#if defined(__linux__)
#include <fstream>
#if defined(__GLIBC__)
#include <malloc.h>
#endif
#include <sys/wait.h>
#include <unistd.h>
#endif

using namespace carvetto;
using namespace carvetto::document;
using carvetto::toolpath::Move;
using carvetto::toolpath::MoveKind;
using Bytes = std::vector<std::uint8_t>;
using Code = CarvError::Code;

namespace {

// --- Helpers ----------------------------------------------------------------------------------

/// A tool snapshot a toolpath with computed moves needs: diameter, feed and plunge rate above zero
/// (T-0134; uncomputed records may keep the default snapshot).
const toolpath::ToolSnapshot kValidTool{"tool", 1, 6.0, 1000.0, 300.0, 18000.0};

Bytes save(const Document& doc, const CarvExtras& extras = {}) {
    auto r = saveCarv(doc, extras);
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

std::vector<detail::ZipMember> members(const Bytes& archive) {
    auto r = detail::readZip(archive);
    INFO(r.error);
    REQUIRE(r.ok);
    return r.members;
}

std::vector<std::string> memberNames(const Bytes& archive) {
    std::vector<std::string> out;
    for (const auto& m : members(archive)) {
        out.push_back(m.name);
    }
    return out;
}

Bytes memberBytes(const Bytes& archive, const std::string& name) {
    for (const auto& m : members(archive)) {
        if (m.name == name) {
            return m.data;
        }
    }
    FAIL("member " << name << " not found");
    return {};
}

std::string memberText(const Bytes& archive, const std::string& name) {
    const Bytes b = memberBytes(archive, name);
    return {b.begin(), b.end()};
}

/// Replaces (or adds) member `name`; an empty optional removes it.
Bytes withMember(const Bytes& archive, const std::string& name, const std::optional<Bytes>& data) {
    auto ms = members(archive);
    bool found = false;
    for (auto it = ms.begin(); it != ms.end();) {
        if (it->name == name) {
            found = true;
            if (!data) {
                it = ms.erase(it);
                continue;
            }
            it->data = *data;
        }
        ++it;
    }
    if (!found && data) {
        ms.push_back({name, *data});
    }
    auto r = detail::writeZip(ms);
    REQUIRE(r.ok);
    return r.bytes;
}

Bytes withText(const Bytes& archive, const std::string& name, const std::string& text) {
    return withMember(archive, name, Bytes(text.begin(), text.end()));
}

std::string replaced(std::string text, const std::string& from, const std::string& to) {
    const std::size_t pos = text.find(from);
    INFO("looking for: " << from);
    REQUIRE(pos != std::string::npos);
    text.replace(pos, from.size(), to);
    return text;
}

/// A profile with every parameter away from its default, with awkward doubles.
toolpath::ProfileParams fullProfile() {
    using namespace toolpath;
    ProfileParams p;
    p.name = "Cut \"out\" \xC3\xA9";
    p.side = ProfileSide::Inside;
    p.direction = CutDirection::Conventional;
    p.allowance = -0.1;
    p.passes = {0.3, 9.7, 2.5, false, 0.25};
    p.safeZ = 12.000000000000002;
    p.separateLastPass = true;
    p.lastPassAllowance = 0.3;
    p.lastPassReverse = true;
    p.tabs = {true, 7.0, 1.5, TabSpacing::ConstantDistance, 6, 80.0, true, true};
    p.ramp = {RampType::ZigZag, 15.0, true, 0.17453292519943295, true};
    p.leads = {LeadType::Circular, 4.0, 0.5235987755982988, 6.0, 1.0471975511965976, true, 1e-300};
    p.order = {true, true, true, true};
    p.startAt = StartAt::ClosestOnBoundingBox;
    p.startAnchor = BoxAnchor::TopRight;
    p.startPosition = {10.0, -0.0};
    p.corners = {true, true, 3.0};
    p.tolerance = {0.002, 1.7453292519943296e-05, 0.02};
    return p;
}

toolpath::PocketParams fullPocket() {
    using namespace toolpath;
    PocketParams p;
    p.name = "Pocket";
    p.startDepth = 0.5;
    p.cutDepth = 6.0;
    p.maintainExactStep = false;
    p.lastPassThickness = 0.4;
    p.safeZ = 8.0;
    p.strategy = PocketStrategy::Raster;
    p.direction = CutDirection::Conventional;
    p.rasterAngle = 0.7853981633974483;
    p.profilePass = ProfilePass::First;
    p.allowance = 0.1;
    p.rampPlunge = true;
    p.rampDistance = 12.0;
    p.tolerance = {0.003, 1e-8, 0.05};
    return p;
}

/// A short move list using every kind, feed overrides and the "same coordinate" shortcuts.
std::vector<Move> sampleMoves() {
    return {
        Move::rapid({0.0, 0.0, 5.0}),
        Move::rapid({10.5, -3.25, 5.0}),
        Move::plunge({10.5, -3.25, -1.0}, 300.0),
        Move::feed({20.0, -3.25, -1.0}),
        Move::feed({20.0, 4.0, -1.0}, 1200.0),
        Move::feed({20.0, 8.0, -1.0}, 1200.0),
        Move::arcCW({25.0, 3.0, -1.0}, {20.0, 3.0}),
        Move::arcCCW({25.0, 3.0, -2.0}, {22.5, 3.0}, 800.0),  // full circle with a helix
        Move::dwell({25.0, 3.0, -2.0}, 0.5),
        Move::feed({-0.0, 1e-310, -2.0}),
        Move::retract({-0.0, 1e-310, 5.0}, 300.0),
    };
}

/// A document with vectors on two layers, a group, and three toolpath records (profile and
/// pocket with moves, a pocket without).
struct Sample {
    Document doc;
    ObjectId square;
    ObjectId circle;
    ObjectId group;
    LayerId cut;
};

Sample makeSample() {
    Sample s;
    Document& doc = s.doc;
    Layer cut;
    cut.name = "Cut";
    s.cut = doc.insertLayer(cut);
    s.square = doc.insertObject(Object::vector(geometry::Contour::rectangle(geometry::Box2{{0, 0}, {40, 30}}),
                                               doc.defaultLayer(), doc.activeSheet()));
    s.circle =
        doc.insertObject(Object::vector(geometry::Contour::circle({60, 20}, 8), s.cut, doc.activeSheet()));
    const ObjectId a = doc.insertObject(
        Object::vector(geometry::Contour::circle({90, 20}, 4), doc.defaultLayer(), doc.activeSheet()));
    REQUIRE(doc.execute(std::make_unique<GroupObjectsCommand>(std::vector<ObjectId>{a})));
    s.group = *doc.findObject(a)->parent;

    ToolpathRecord profile;
    profile.name = "Cut out";
    profile.params = fullProfile();
    profile.selection.layers = {s.cut};
    profile.selection.objects = {s.square, s.group};
    profile.tool = {"End Mill 3 mm", 5, 3.0, 1500.0, 500.0, 18000.0};
    profile.toolSource = {"end-mill-3mm", "machine-generic-router", "material-hardwood-mdf"};
    profile.moves = shareMoves(sampleMoves());
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(profile)));

    ToolpathRecord pocket;
    pocket.name = "Pockets [End Mill 6 mm]";
    pocket.params = fullPocket();
    pocket.selection = VectorSelection::allVectors();
    pocket.tool = {"End Mill 6 mm", 1, 6.0, 2300.0, 800.0, 16000.0};
    pocket.moves = shareMoves({Move::rapid({1, 2, 3}), Move::plunge({1, 2, -4})});
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(pocket, std::size_t{0})));  // inserted first

    ToolpathRecord pending;
    pending.name = "Not computed";
    pending.params = toolpath::PocketParams{};
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(pending)));
    return s;
}

void requireSameToolpaths(const Document& a, const Document& b) {
    REQUIRE(a.toolpaths().size() == b.toolpaths().size());
    for (std::size_t i = 0; i < a.toolpaths().size(); ++i) {
        INFO("toolpath " << i);
        CHECK(a.toolpaths()[i] == b.toolpaths()[i]);
    }
    CHECK(a.idGenerator().peek() == b.idGenerator().peek());
}

const std::string kManifest10 =
    R"({"app_version": "0.1.0", "format": "carvetto", "format_version": {"major": 1, "minor": 0}})";
const std::string kJob10 = R"({
  "active_layer": 2, "active_sheet": 1,
  "job": {"type": "single_sided", "size_mm": [300.0, 200.0, 18.0], "display_units": "mm",
          "z_zero": "material_surface", "datum": "bottom_left", "datum_offset_mm": [0.0, 0.0],
          "flip_direction": "horizontal", "zero_off_same_side": false, "diameter_mm": 0.0,
          "rotary_axis": "x", "flip_design": false},
  "layers": [{"id": 2, "name": "Layer 1", "colour": "#000000", "visible": true, "locked": false},
             {"id": 3, "name": "Outline", "colour": "#FF8000", "visible": true, "locked": false},
             {"id": 4, "name": "Outline", "colour": "#FF8000", "visible": true, "locked": false}],
  "next_id": 10,
  "objects": [{"id": 5, "layer": 3, "sheet": 1, "name": "v", "type": "vector", "contours": [], "params": {"type": "none"}}],
  "sheets": [{"id": 1, "name": "Sheet 1", "size_mm": [300, 200, 18], "z_zero": "material_surface",
              "datum": "bottom_left", "datum_offset_mm": [0, 0], "visible": true}],
  "variables": {}
})";

/// An interim member as carvetto-cli wrote it before T-0132 (ADR-0019).
std::string interimMember(const std::string& operation, const std::string& params, const std::string& moves) {
    return R"({"format":"carvetto-toolpath","format_version":1,"moves":)" + moves + R"(,"name":"Old )" + operation +
           R"(","operation":")" + operation + R"(","params":)" + params +
           R"(,"safe_z":5.0,"tool":{"diameter":6.0,"feed_rate":2300.0,"name":"End Mill 6 mm","plunge_rate":800.0,"spindle_speed":16000.0,"tool_number":1}})";
}

const std::string kInterimMoves =
    R"([["rapid",0.0,0.0,5.0],["plunge",1.0,2.0,-3.0,{"feed":300.0}],["arc_cw",3.0,2.0,-3.0,2.0,2.0],["dwell",3.0,2.0,-3.0,0.25],["feed",4.0,2.0,-3.0],["retract",4.0,2.0,5.0]])";

/// A 1.0 file with interim members (no toolpaths in job.json).
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

/// Bytes of a move cache for `moves`.
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

/// A deterministic large toolpath: raster-like feeds with ramps, arcs, plunges and retracts.
std::vector<Move> bigToolpath(std::size_t count) {
    std::vector<Move> moves;
    moves.reserve(count);
    std::mt19937_64 rng(20261101);
    std::uniform_real_distribution<double> step(0.05, 2.0);
    double x = 0.0;
    double y = 0.0;
    double z = 5.0;
    moves.push_back(Move::rapid({x, y, z}));
    while (moves.size() < count) {
        switch (moves.size() % 50) {
            case 0:
                z = -1.0 - static_cast<double>((moves.size() / 50) % 7) * 0.5;
                moves.push_back(Move::plunge({x, y, z}, 300.0));
                break;
            case 17:
                moves.push_back(Move::arcCCW({x + 1.0, y + 1.0, z}, {x + 1.0, y}, 1100.0));
                x += 1.0;
                y += 1.0;
                break;
            case 49:
                moves.push_back(Move::retract({x, y, 5.0}));
                z = 5.0;
                break;
            default:
                if (moves.size() % 2 == 0) {
                    x += step(rng);
                } else {
                    y += step(rng) * 0.1;
                }
                moves.push_back(Move::feed({x, y, z}));
                break;
        }
    }
    return moves;
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

/// Peak RSS growth (bytes) of loading `file` in a forked child, measured as in carv_io_test
/// (heap trimmed, high-water mark reset through /proc/self/clear_refs). -1 if unsupported.
std::int64_t childPeakGrowth(const Bytes& file) {
    int fds[2];
    if (pipe(fds) != 0) {
        return -1;
    }
    const pid_t pid = fork();
    if (pid == 0) {
        close(fds[0]);
        std::int64_t result = -1;
#if defined(__GLIBC__)
        malloc_trim(0);
#endif
        {
            std::ofstream clear("/proc/self/clear_refs");
            clear << "5";
        }
        const std::int64_t before = statusKb("VmRSS:");
        if (before >= 0 && statusKb("VmHWM:") <= before + 1024) {
            {
                auto r = loadCarv(std::span<const std::uint8_t>(file));
                result = r.ok() ? 0 : -1;
            }
            if (result == 0) {
                result = (statusKb("VmHWM:") - before) * 1024;
            }
        }
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
#endif

}  // namespace

// ------------------------------------------------------------------------------------------------
// Document model, commands, integrity
// ------------------------------------------------------------------------------------------------

TEST_CASE("toolpath records live in the document in job order", "[document][toolpath]") {
    Sample s = makeSample();
    const Document& doc = s.doc;
    REQUIRE(doc.toolpaths().size() == 3);
    CHECK(doc.toolpaths()[0].name == "Pockets [End Mill 6 mm]");
    CHECK(doc.toolpaths()[1].name == "Cut out");
    CHECK(doc.toolpaths()[0].operation() == ToolpathOperation::Pocket);
    CHECK(doc.toolpaths()[1].operation() == ToolpathOperation::Profile);
    CHECK(doc.toolpaths()[1].safeZ() == fullProfile().safeZ);
    CHECK(doc.toolpaths()[1].moveCount() == sampleMoves().size());
    CHECK_FALSE(doc.toolpaths()[2].hasMoves());
    const toolpath::Toolpath tp = doc.toolpaths()[1].toolpath();
    CHECK(tp.name == "Cut out");
    CHECK(tp.tool.name == "End Mill 3 mm");
    CHECK(tp.moves == sampleMoves());
    CHECK(doc.toolpathIndex(doc.toolpaths()[2].id) == std::size_t{2});
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("selectedVectors resolves layers, objects and groups", "[document][toolpath]") {
    Sample s = makeSample();
    const Document& doc = s.doc;
    const ObjectId child = doc.findObject(s.group)->asGroup()->children.front();
    CHECK(doc.selectedVectors(VectorSelection::allVectors()) == std::vector<ObjectId>{s.square, s.circle, child});
    VectorSelection sel;
    CHECK(doc.selectedVectors(sel).empty());
    sel.objects = {s.group, s.square, s.square};
    sel.layers = {s.cut};
    CHECK(doc.selectedVectors(sel) == std::vector<ObjectId>{child, s.square, s.circle});
    sel.objects = {ObjectId{999}};  // names nothing
    sel.layers = {LayerId{998}};
    CHECK(doc.selectedVectors(sel).empty());
}

TEST_CASE("toolpath commands undo and redo exactly", "[document][toolpath][undo]") {
    Sample s = makeSample();
    Document& doc = s.doc;
    const auto snapshot = [&doc]() { return doc.toolpaths(); };
    const auto start = snapshot();
    const ToolpathId first = doc.toolpaths()[0].id;
    const ToolpathId second = doc.toolpaths()[1].id;

    ToolpathRecord edited = doc.toolpaths()[1];
    edited.name = "Renamed";
    std::get<toolpath::ProfileParams>(edited.params).tabs.count = 9;
    edited.moves = shareMoves({Move::rapid({0, 0, 9})});
    REQUIRE(doc.execute(std::make_unique<SetToolpathCommand>(edited)));
    ToolpathRecord edited2 = edited;
    edited2.name = "Renamed again";
    REQUIRE(doc.execute(std::make_unique<SetToolpathCommand>(edited2)));  // coalesces
    CHECK(doc.findToolpath(second)->name == "Renamed again");
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edited2)));  // no change: no-op
    REQUIRE(doc.undo());
    CHECK(snapshot() == start);
    REQUIRE(doc.redo());
    CHECK(*doc.findToolpath(second) == edited2);
    REQUIRE(doc.undo());

    REQUIRE(doc.execute(std::make_unique<MoveToolpathCommand>(first, 2)));
    CHECK(doc.toolpaths()[2].id == first);
    CHECK_FALSE(doc.execute(std::make_unique<MoveToolpathCommand>(first, 2)));
    CHECK_FALSE(doc.execute(std::make_unique<MoveToolpathCommand>(first, 3)));
    REQUIRE(doc.undo());
    CHECK(snapshot() == start);

    REQUIRE(doc.execute(std::make_unique<RemoveToolpathsCommand>(std::vector<ToolpathId>{second, first, first})));
    CHECK(doc.toolpaths().size() == 1);
    CHECK_FALSE(doc.execute(std::make_unique<RemoveToolpathsCommand>(std::vector<ToolpathId>{second})));
    REQUIRE(doc.undo());
    CHECK(snapshot() == start);
    REQUIRE(doc.redo());
    CHECK(doc.toolpaths().size() == 1);
    REQUIRE(doc.undo());

    // Undoing the adds removes the records; redo brings them back with the same ids.
    while (doc.undo()) {
    }
    CHECK(doc.toolpaths().empty());
    while (doc.redo()) {
    }
    CHECK(doc.toolpaths().size() == 1);  // the removal was the last step
    REQUIRE(doc.undo());
    CHECK(snapshot() == start);
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("toolpath ids share the document's id generator", "[document][toolpath][ids]") {
    Document doc;
    ToolpathRecord r;
    r.name = "a";
    auto add = std::make_unique<AddToolpathCommand>(r);
    AddToolpathCommand* addPtr = add.get();
    REQUIRE(doc.execute(std::move(add)));
    const ToolpathId id = addPtr->id();
    CHECK(!id.isNull());
    // A preset id must be fresh: not the toolpath's, not below the next id.
    Object obj = Object::vector(geometry::Contour::circle({0, 0}, 1), doc.defaultLayer(), doc.activeSheet());
    obj.id = ObjectId{id.value};
    CHECK_FALSE(doc.execute(std::make_unique<AddObjectCommand>(obj)));
    ToolpathRecord preset = r;
    preset.id = ToolpathId{doc.idGenerator().peek() + 5};
    CHECK(doc.execute(std::make_unique<AddToolpathCommand>(preset)));
    ToolpathRecord stale = r;
    stale.id = id;
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(stale)));
    CHECK(doc.checkIntegrity().empty());
}

TEST_CASE("toolpath commands refuse invalid selections and normalise records", "[document][toolpath][undo]") {
    Sample s = makeSample();
    Document& doc = s.doc;
    const std::size_t before = doc.toolpaths().size();
    ToolpathRecord r;
    r.name = "x";
    r.selection.objects = {ObjectId{doc.idGenerator().peek() + 3}};  // never allocated
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(r)));
    r.selection.objects = {ObjectId{s.cut.value}};  // a layer
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(r)));
    r.selection.objects.clear();
    r.selection.layers = {LayerId{s.square.value}};  // an object
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(r)));
    r.selection.layers = {LayerId{doc.toolpaths()[0].id.value}};  // a toolpath
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(r)));
    r.selection = VectorSelection::allVectors();
    r.selection.layers = {s.cut};
    CHECK_FALSE(doc.execute(std::make_unique<AddToolpathCommand>(r)));
    CHECK(doc.toolpaths().size() == before);

    ToolpathRecord edit = doc.toolpaths()[1];
    edit.selection.objects.push_back(ObjectId{doc.idGenerator().peek()});
    CHECK_FALSE(doc.execute(std::make_unique<SetToolpathCommand>(edit)));
    CHECK(doc.checkIntegrity().empty());

    // cuttingData is cleared and ignored move fields are zeroed, so the record equals its saved
    // and reloaded copy.
    ToolpathRecord n;
    n.tool = kValidTool;  // computed moves need a real tool (T-0134)
    n.name = "normalised";
    n.selection = VectorSelection::allVectors();
    auto profile = fullProfile();
    profile.cuttingData = tooling::CuttingData{};
    n.params = profile;
    Move rapid = Move::rapid({1, 2, 3});
    rapid.feedOverride = 99.0;
    rapid.centre = {4, 5};
    Move feed = Move::feed({4, 5, 6});
    feed.dwellSeconds = 2.0;
    n.moves = shareMoves({rapid, feed});
    CHECK((*n.moves)[0] == Move::rapid({1, 2, 3}));
    CHECK((*n.moves)[1] == Move::feed({4, 5, 6}));
    REQUIRE(doc.execute(std::make_unique<AddToolpathCommand>(n)));
    CHECK_FALSE(std::get<toolpath::ProfileParams>(doc.toolpaths().back().params).cuttingData);
    LoadedCarv back = load(save(doc));
    requireSameToolpaths(doc, back.document);
}

TEST_CASE("integrity checks cover toolpath ids and selections", "[document][toolpath][integrity]") {
    using Kind = IntegrityIssue::Kind;
    const auto kinds = [](const Document& doc) {
        std::vector<Kind> out;
        for (const auto& i : doc.checkIntegrity()) {
            out.push_back(i.kind);
        }
        return out;
    };
    const auto has = [&](const Document& doc, Kind k) {
        const auto ks = kinds(doc);
        return std::find(ks.begin(), ks.end(), k) != ks.end();
    };
    SECTION("a deleted object or layer may stay selected") {
        Sample s = makeSample();
        REQUIRE(s.doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{s.square})));
        REQUIRE(s.doc.execute(std::make_unique<DeleteLayerCommand>(s.cut, true)));
        CHECK(s.doc.checkIntegrity().empty());
        // The square is no longer machined; undo brings it back into the selection.
        const VectorSelection sel = s.doc.toolpaths()[1].selection;
        CHECK(std::find(sel.objects.begin(), sel.objects.end(), s.square) != sel.objects.end());
        const auto selected = s.doc.selectedVectors(sel);
        CHECK(std::find(selected.begin(), selected.end(), s.square) == selected.end());
        REQUIRE(s.doc.undo());
        REQUIRE(s.doc.undo());
        const auto again = s.doc.selectedVectors(sel);
        CHECK(std::find(again.begin(), again.end(), s.square) != again.end());
    }
    SECTION("an unallocated id or an item of another kind is reported") {
        Sample s = makeSample();
        ToolpathRecord* tp = s.doc.findToolpath(s.doc.toolpaths()[1].id);
        tp->selection.objects.push_back(ObjectId{s.doc.idGenerator().peek()});
        CHECK(has(s.doc, Kind::BadToolpathSelection));
        tp->selection.objects.pop_back();
        tp->selection.layers.push_back(LayerId{s.square.value});  // an object, not a layer
        CHECK(has(s.doc, Kind::BadToolpathSelection));
        tp->selection.layers.pop_back();
        tp->selection.objects.push_back(ObjectId{s.cut.value});  // a layer, not an object
        CHECK(has(s.doc, Kind::BadToolpathSelection));
        tp->selection.objects.pop_back();
        tp->selection.objects.push_back(ObjectId{s.doc.toolpaths()[0].id.value});  // a toolpath
        CHECK(has(s.doc, Kind::BadToolpathSelection));
        tp->selection.objects.pop_back();
        CHECK(s.doc.checkIntegrity().empty());
        tp->selection.all = true;  // with explicit layers and objects
        CHECK(has(s.doc, Kind::BadToolpathSelection));
    }
    SECTION("toolpath ids are range-checked and unique across kinds") {
        Sample s = makeSample();
        ToolpathRecord* tp = s.doc.findToolpath(s.doc.toolpaths()[0].id);
        tp->id = ToolpathId{s.square.value};
        CHECK(has(s.doc, Kind::DuplicateId));
        tp->id = ToolpathId{s.doc.idGenerator().peek()};
        CHECK(has(s.doc, Kind::IdNotBelowNext));
        tp->id = ToolpathId{};
        CHECK(has(s.doc, Kind::IdOutOfRange));
    }
}

TEST_CASE("random command sequences with toolpaths keep the document consistent and round-trip",
          "[document][toolpath][fuzz]") {
    for (std::uint32_t seed = 1; seed <= 40; ++seed) {
        INFO("seed " << seed);
        Sample s = makeSample();
        Document& doc = s.doc;
        std::mt19937 rng(seed);
        for (int step = 0; step < 60; ++step) {
            const auto pickToolpath = [&]() -> std::optional<ToolpathId> {
                if (doc.toolpaths().empty()) {
                    return std::nullopt;
                }
                return doc.toolpaths()[rng() % doc.toolpaths().size()].id;
            };
            switch (rng() % 9) {
                case 0: {
                    ToolpathRecord r;
                    r.name = "T" + std::to_string(step);
                    r.params = (rng() % 2) ? toolpath::ProfileParams{} : toolpath::ProfileParams{fullProfile()};
                    const auto ids = doc.objectIds();
                    if (!ids.empty()) {
                        r.selection.objects.push_back(ids[rng() % ids.size()]);
                    }
                    r.selection.layers.push_back(doc.layers()[rng() % doc.layers().size()].id);
                    if (rng() % 2) {
                        r.moves = shareMoves(sampleMoves());
                        r.tool = kValidTool;  // computed moves need a real tool (T-0134)
                    }
                    doc.execute(std::make_unique<AddToolpathCommand>(r, rng() % 4));
                    break;
                }
                case 1:
                    if (const auto id = pickToolpath()) {
                        doc.execute(std::make_unique<RemoveToolpathsCommand>(std::vector<ToolpathId>{*id}));
                    }
                    break;
                case 2:
                    if (const auto id = pickToolpath()) {
                        ToolpathRecord r = *doc.findToolpath(*id);
                        r.name += "+";
                        r.moves = (rng() % 2) ? shareMoves({Move::rapid({1.0 * step, 0, 5})}) : nullptr;
                        if (r.moves) {
                            r.tool = kValidTool;  // computed moves need a real tool (T-0134)
                        }
                        doc.execute(std::make_unique<SetToolpathCommand>(r));
                    }
                    break;
                case 3:
                    if (const auto id = pickToolpath()) {
                        doc.execute(std::make_unique<MoveToolpathCommand>(*id, rng() % doc.toolpaths().size()));
                    }
                    break;
                case 4: {
                    const auto ids = doc.objectIds();
                    if (!ids.empty()) {
                        doc.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{ids[rng() % ids.size()]}));
                    }
                    break;
                }
                case 5:
                    if (doc.layers().size() > 1) {
                        doc.execute(std::make_unique<DeleteLayerCommand>(doc.layers()[rng() % doc.layers().size()].id,
                                                                         rng() % 2 == 0));
                    }
                    break;
                case 6:
                    doc.undo();
                    break;
                case 7:
                    doc.redo();
                    break;
                default: {
                    Layer l;
                    l.name = "L" + std::to_string(step);
                    doc.execute(std::make_unique<AddLayerCommand>(l));
                    break;
                }
            }
            const auto issues = doc.checkIntegrity();
            INFO("step " << step << ": " << (issues.empty() ? std::string() : issues.front().message));
            REQUIRE(issues.empty());
        }
        const Bytes once = save(doc);
        LoadedCarv back = load(once);
        requireSameToolpaths(doc, back.document);
        CHECK(save(back.document, back.extras) == once);
    }
}

// ------------------------------------------------------------------------------------------------
// The move cache
// ------------------------------------------------------------------------------------------------

TEST_CASE("move cache: exact round trip of every kind, compact encoding", "[document][toolpath][moves]") {
    const std::vector<Move> moves = sampleMoves();
    const Bytes cache = cacheOf(moves);
    std::vector<Move> back;
    const std::string text(cache.begin(), cache.end());
    detail::StringSource src(text);
    REQUIRE(detail::decodeMoves(src, moves.size(), back).empty());
    CHECK(back == moves);
    CHECK(std::signbit(back[9].to.x));  // -0.0 kept
    CHECK(back[9].to.y == 1e-310);      // subnormal kept
    // Header + tags; the second 1200 feed and repeated coordinates are not written again.
    CHECK(cache.size() < detail::kMovesHeaderBytes + moves.size() * 25);
    CHECK(std::string(cache.begin(), cache.begin() + 4) == "CVTP");
    // Equal input, equal bytes.
    CHECK(cacheOf(moves) == cache);
    // An empty list is just the header.
    CHECK(cacheOf({}).size() == detail::kMovesHeaderBytes);
}

TEST_CASE("move cache: fields a kind ignores are not stored", "[document][toolpath][moves]") {
    Move rapid = Move::rapid({1, 2, 3});
    rapid.feedOverride = 99.0;
    rapid.centre = {4, 5};
    rapid.dwellSeconds = 7.0;
    Move feed = Move::feed({4, 5, 6});
    feed.centre = {1, 1};
    const Bytes cache = cacheOf({rapid, feed});
    std::vector<Move> back;
    const std::string text(cache.begin(), cache.end());
    detail::StringSource src(text);
    REQUIRE(detail::decodeMoves(src, 2, back).empty());
    CHECK(back[0] == Move::rapid({1, 2, 3}));
    CHECK(back[1] == Move::feed({4, 5, 6}));
}

TEST_CASE("move cache: defects are reported, never crash", "[document][toolpath][moves][errors]") {
    const std::vector<Move> moves = sampleMoves();
    const Bytes good = cacheOf(moves);
    CHECK(decodeError(good, moves.size()).empty());
    CHECK(contains(decodeError(good, moves.size() + 1), "job.json says"));
    CHECK(contains(decodeError(Bytes(good.begin(), good.begin() + 10), moves.size()), "shorter than its header"));
    Bytes badMagic = good;
    badMagic[0] = 'X';
    CHECK(contains(decodeError(badMagic, moves.size()), "signature"));
    Bytes badVersion = good;
    badVersion[4] = 2;
    CHECK(contains(decodeError(badVersion, moves.size()), "version 2"));
    CHECK(contains(decodeError(Bytes(good.begin(), good.end() - 3), moves.size()), "ends early"));
    Bytes trailing = good;
    trailing.push_back(0);
    CHECK(contains(decodeError(trailing, moves.size()), "follows the last move"));
    // Hand-made single moves after a valid header.
    const auto oneMove = [](std::vector<std::uint8_t> body) {
        Bytes b = cacheOf({Move::rapid({0, 0, 0})});
        b.resize(detail::kMovesHeaderBytes);
        b.insert(b.end(), body.begin(), body.end());
        return b;
    };
    CHECK(contains(decodeError(oneMove({0x07}), 1), "invalid move kind"));
    CHECK(contains(decodeError(oneMove({0x08 | 0x70, 0, 0, 0, 0, 0, 0, 0, 0}), 1), "feed override on a rapid"));
    CHECK(contains(decodeError(oneMove({0x01 | 0x08 | 0x80 | 0x70}), 1), "previous feed override"));
    // x = +infinity
    CHECK(contains(decodeError(oneMove({0x01 | 0x60, 0, 0, 0, 0, 0, 0, 0xF0, 0x7F}), 1), "non-finite"));
    // Encoding refuses non-finite values too.
    Bytes out;
    CHECK(contains(detail::encodeMoves({Move::feed({std::nan(""), 0, 0})}, out), "non-finite"));
    out.clear();
    CHECK(contains(detail::encodeMoves({Move::dwell({0, 0, 0}, std::numeric_limits<double>::infinity())}, out), "non-finite"));
    out.clear();
    Move badKind;
    badKind.kind = static_cast<MoveKind>(9);
    CHECK(contains(detail::encodeMoves({badKind}, out), "invalid kind"));
}

// ------------------------------------------------------------------------------------------------
// .carv files
// ------------------------------------------------------------------------------------------------

TEST_CASE("AC1: toolpath records save and load exactly; save -> load -> save is byte-identical",
          "[document][toolpath][carv]") {
    const Sample s = makeSample();
    const Bytes once = save(s.doc);
    // Records in job.json, moves in one cache per computed toolpath, named by position.
    CHECK(memberNames(once) ==
          std::vector<std::string>{"manifest.json", "job.json", "toolpaths/0001.bin", "toolpaths/0002.bin"});
    const std::string job = memberText(once, "job.json");
    CHECK(contains(job, "\"toolpaths\": ["));
    CHECK(contains(job, "\"operation\": \"profile\""));
    CHECK(contains(job, "\"moves\": {\"count\": 11, \"member\": \"toolpaths/0002.bin\"}"));
    CHECK(contains(job, "\"angle_rad\": 0.5235987755982988"));
    CHECK(contains(job, "\"raster_angle_rad\": 0.7853981633974483"));
    CHECK(contains(job, "\"start_position\": [10.0, -0.0]"));
    CHECK(contains(memberText(once, "manifest.json"), "\"minor\": 1"));

    LoadedCarv back = load(once);
    CHECK(back.formatMinor == 1);
    requireSameToolpaths(s.doc, back.document);
    CHECK(back.document.checkIntegrity().empty());
    CHECK(back.extras.empty());
    CHECK(save(back.document, back.extras) == once);
    // cuttingData is tool-database input, not part of the record.
    Document withData;
    ToolpathRecord r = s.doc.toolpaths()[1];
    r.id = {};
    r.selection = VectorSelection::allVectors();
    std::get<toolpath::ProfileParams>(r.params).cuttingData = tooling::CuttingData{};
    withData.insertToolpath(r);
    LoadedCarv noData = load(save(withData));
    CHECK_FALSE(std::get<toolpath::ProfileParams>(noData.document.toolpaths()[0].params).cuttingData);
}

TEST_CASE("unknown keys of a toolpath record are preserved; nested ones are dropped",
          "[document][toolpath][carv]") {
    const Sample s = makeSample();
    const Bytes original = save(s.doc);
    std::string job = memberText(original, "job.json");
    job = replaced(job, "\"name\": \"Cut out\"", "\"name\": \"Cut out\", \"zz_toolpath\": [1, {\"a\": true}]");
    job = replaced(job, "\"safe_z\": 12.000000000000002", "\"safe_z\": 12.000000000000002, \"zz_param\": 3");
    LoadedCarv back = load(withText(original, "job.json", job));
    requireSameToolpaths(s.doc, back.document);
    const std::string resaved = memberText(save(back.document, back.extras), "job.json");
    CHECK(contains(resaved, "\"zz_toolpath\": [1, {\"a\": true}]"));
    CHECK_FALSE(contains(resaved, "zz_param"));
    // Deleting the toolpath drops its unknown keys.
    REQUIRE(back.document.execute(
        std::make_unique<RemoveToolpathsCommand>(std::vector<ToolpathId>{back.document.toolpaths()[1].id})));
    CHECK_FALSE(contains(memberText(save(back.document, back.extras), "job.json"), "zz_toolpath"));
}

TEST_CASE("a 1.0 file without toolpaths still loads", "[document][toolpath][carv][migration]") {
    LoadedCarv old = load(legacyFile({}));
    CHECK(old.formatMinor == 0);
    CHECK(old.document.toolpaths().empty());
    CHECK(old.document.objectCount() == 1);
    // Re-saved as 1.1, still without toolpath members.
    const Bytes resaved = save(old.document, old.extras);
    CHECK(memberNames(resaved) == std::vector<std::string>{"manifest.json", "job.json"});
    CHECK(contains(memberText(resaved, "job.json"), "\"toolpaths\": []"));
}

TEST_CASE("T-0122 interim toolpath members are migrated to records", "[document][toolpath][carv][migration]") {
    const std::string profileParams =
        R"({"layers":["Outline"],"leads":{"angle":30,"sweep":60,"type":"circular"},"name":"P","passes":{"cut_depth":6},"ramp":{"angle":10},"side":"right","tolerance":{"angle":0.001},"tool":{"database":"tools.json","id":"end-mill-6mm","machine":"m","material":"w"}})";
    const std::string pocketParams =
        R"({"cut_depth":3,"raster_angle":45,"strategy":"raster","tool":{"id":"end-mill-6mm"}})";
    const Bytes file = legacyFile({{"toolpaths/0002.json", interimMember("pocket", pocketParams, kInterimMoves)},
                                   {"toolpaths/0001.json", interimMember("profile", profileParams, "[]")},
                                   {"toolpaths/0003.json", interimMember("pocket",
                                                                         R"({"layers":["Nope"],"tool":{"id":"x"}})", "[]")},
                                   {"other.json", "{\"keep\": 1}"}});
    LoadedCarv back = load(file);
    CHECK(back.extras.memberNames() == std::vector<std::string>{"other.json"});
    const auto& tps = back.document.toolpaths();
    REQUIRE(tps.size() == 3);
    // Member-name (creation) order, fresh ids above next_id.
    CHECK(tps[0].name == "Old profile");
    CHECK(tps[1].name == "Old pocket");
    CHECK(tps[0].id.value >= 10);
    CHECK(tps[1].id.value > tps[0].id.value);
    CHECK(back.document.checkIntegrity().empty());

    constexpr double kDeg = geometry::kPi / 180.0;
    const auto& profile = std::get<toolpath::ProfileParams>(tps[0].params);
    CHECK(profile.name == "P");
    CHECK(profile.side == toolpath::ProfileSide::Outside);  // "right" is an alias
    CHECK(profile.passes.cutDepth == 6.0);
    CHECK(profile.leads.type == toolpath::LeadType::Circular);
    CHECK(profile.leads.angle == 30 * kDeg);  // the same expression the CLI used
    CHECK(profile.leads.sweep == 60 * kDeg);
    CHECK(profile.ramp.angle == 10 * kDeg);
    CHECK(profile.tolerance.angle == 0.001 * kDeg);
    CHECK(tps[0].toolSource == ToolSource{"end-mill-6mm", "m", "w"});
    // "layers" names every layer of that name.
    CHECK(tps[0].selection.layers == std::vector<LayerId>{LayerId{3}, LayerId{4}});
    CHECK_FALSE(tps[0].selection.all);
    CHECK(tps[0].moveCount() == 0);
    CHECK(tps[0].hasMoves());

    const auto& pocket = std::get<toolpath::PocketParams>(tps[1].params);
    CHECK(pocket.cutDepth == 3.0);
    CHECK(pocket.rasterAngle == 45 * kDeg);
    CHECK(pocket.strategy == toolpath::PocketStrategy::Raster);
    CHECK(tps[1].selection == VectorSelection::allVectors());  // no "layers": every vector
    CHECK(tps[1].tool == toolpath::ToolSnapshot{"End Mill 6 mm", 1, 6.0, 2300.0, 800.0, 16000.0});
    const std::vector<Move> expected{Move::rapid({0, 0, 5}),        Move::plunge({1, 2, -3}, 300.0),
                                     Move::arcCW({3, 2, -3}, {2, 2}), Move::dwell({3, 2, -3}, 0.25),
                                     Move::feed({4, 2, -3}),         Move::retract({4, 2, 5})};
    CHECK(*tps[1].moves == expected);
    // Layer order is canonical, as carvetto-cli writes it: names as listed, each name's layers in
    // stacking order.
    {
        const std::string ordered = R"({"layers":["Layer 1","Outline"],"tool":{"id":"t"}})";
        const std::string reversed = R"({"layers":["Outline","Layer 1"],"tool":{"id":"t"}})";
        LoadedCarv a = load(legacyFile({{"toolpaths/0001.json", interimMember("pocket", ordered, "[]")}}));
        LoadedCarv b = load(legacyFile({{"toolpaths/0001.json", interimMember("pocket", reversed, "[]")}}));
        CHECK(a.document.toolpaths()[0].selection.layers == std::vector<LayerId>{LayerId{2}, LayerId{3}, LayerId{4}});
        CHECK(b.document.toolpaths()[0].selection.layers == std::vector<LayerId>{LayerId{3}, LayerId{4}, LayerId{2}});
    }
    // A name that matches no layer selects nothing (not everything).
    CHECK(tps[2].selection == VectorSelection{});

    // Re-saved natively: no interim members any more, and stable from then on.
    const Bytes resaved = save(back.document, back.extras);
    CHECK(memberNames(resaved) == std::vector<std::string>{"manifest.json", "job.json", "other.json",
                                                           "toolpaths/0001.bin", "toolpaths/0002.bin",
                                                           "toolpaths/0003.bin"});
    LoadedCarv again = load(resaved);
    requireSameToolpaths(back.document, again.document);
    CHECK(save(again.document, again.extras) == resaved);
}

TEST_CASE("damaged interim members and toolpath records are reported", "[document][toolpath][carv][errors]") {
    struct Case {
        const char* label;
        Bytes file;
        Code code;
        std::string mention;
    };
    const Bytes good = save(makeSample().doc);
    const std::string job = memberText(good, "job.json");
    const std::vector<Case> cases{
        {"interim: bad move kind",
         legacyFile({{"toolpaths/0001.json", interimMember("pocket", "{}", R"([["warp",0,0,0]])")}}),
         Code::InvalidContent, "toolpaths/0001.json.moves[0][0]: unknown move kind 'warp'"},
        {"interim: feed on a rapid",
         legacyFile({{"toolpaths/0001.json", interimMember("pocket", "{}", R"([["rapid",0,0,0,{"feed":1}]])")}}),
         Code::InvalidContent, "only valid on a cutting move"},
        {"interim: too many elements",
         legacyFile({{"toolpaths/0001.json", interimMember("pocket", "{}", R"([["feed",0,0,0,{"feed":1},2]])")}}),
         Code::InvalidContent, "too many elements"},
        {"interim: unknown operation",
         legacyFile({{"toolpaths/0001.json", interimMember("drill", "{}", "[]")}}), Code::InvalidContent,
         "unknown operation 'drill'"},
        {"interim: missing format",
         legacyFile({{"toolpaths/0001.json", R"({"moves": []})"}}), Code::InvalidContent, "missing 'format'"},
        {"interim: not JSON", legacyFile({{"toolpaths/0001.json", "{oops"}}), Code::InvalidJson,
         "toolpaths/0001.json"},
        {"record: unknown operation", withText(good, "job.json", replaced(job, "\"operation\": \"profile\"", "\"operation\": \"fluting\"")),
         Code::InvalidContent, "unsupported toolpath operation 'fluting'"},
        {"record: bad enum in params", withText(good, "job.json", replaced(job, "\"side\": \"inside\"", "\"side\": \"middle\"")),
         Code::InvalidContent, "params.side: unknown value 'middle'"},
        {"record: missing tool name", withText(good, "job.json", replaced(job, "\"name\": \"End Mill 3 mm\", ", "")),
         Code::InvalidContent, "missing 'name'"},
        {"record: all with layers",
         withText(good, "job.json", replaced(job, "\"all\": false", "\"all\": true")), Code::InvalidContent,
         "'all' cannot be combined"},
        {"record: cache missing", withMember(good, "toolpaths/0002.bin", std::nullopt), Code::MissingMember,
         "the archive has no 'toolpaths/0002.bin'"},
        {"record: bad cache name",
         withText(good, "job.json", replaced(job, "\"member\": \"toolpaths/0002.bin\"", "\"member\": \"x.bin\"")),
         Code::InvalidContent, "must name a move cache"},
        {"record: two records, one cache",
         withText(good, "job.json", replaced(job, "\"member\": \"toolpaths/0002.bin\"", "\"member\": \"toolpaths/0001.bin\"")),
         Code::InvalidContent, "named by two toolpaths"},
        {"record: count differs from the cache",
         withText(good, "job.json", replaced(job, "\"count\": 11", "\"count\": 12")), Code::InvalidContent,
         "toolpaths/0002.bin"},
        {"record: declared size cannot hold the count",
         withText(good, "job.json", replaced(job, "\"count\": 11", "\"count\": 1500")), Code::InvalidContent,
         "cannot hold 1500 moves"},
        {"cache: damaged content",
         withMember(good, "toolpaths/0002.bin", Bytes{'C', 'V', 'T', 'P', 1, 0, 0, 0, 11, 0, 0, 0, 0, 0, 0, 0, 0x07}),
         Code::InvalidContent, ""},
    };
    for (const Case& c : cases) {
        INFO(c.label);
        auto r = loadCarv(std::span<const std::uint8_t>(c.file));
        REQUIRE_FALSE(r.ok());
        INFO(r.error().message);
        CHECK(r.error().code == c.code);
        if (!c.mention.empty()) {
            CHECK(contains(r.error().message, c.mention));
        }
    }
    // A toolpath id that an object uses too.
    const std::string firstId = std::to_string(makeSample().doc.toolpaths()[0].id.value);
    const std::string dup = replaced(job, "\"id\": " + firstId + ",", "\"id\": " + std::to_string(makeSample().square.value) + ",");
    const CarvError err = loadError(withText(good, "job.json", dup));
    CHECK(err.code == Code::InvalidContent);
    CHECK(contains(err.message, "duplicate id"));
}

TEST_CASE("a move cache no record names is dropped; deflate ratio does not apply to caches",
          "[document][toolpath][carv]") {
    const Sample s = makeSample();
    const Bytes good = save(s.doc);
    LoadedCarv back = load(withMember(good, "toolpaths/0007.bin", Bytes{1, 2, 3}));
    CHECK(back.extras.empty());
    CHECK(save(back.document, back.extras) == good);

    // 1,200,000 identical moves (1 byte each in the cache) compress far beyond the 20:1 binary
    // ratio. With enough other content in the file (random padding) they are within one move per
    // byte of the file and load; on their own the file is far too small for them and saveCarv
    // refuses, as the loader would.
    Document doc;
    ToolpathRecord r;
    r.tool = kValidTool;  // computed moves need a real tool (T-0134)
    r.name = "same";
    r.selection = VectorSelection::allVectors();
    r.moves = shareMoves(std::vector<Move>(1'200'000, Move::feed({1, 2, 3}, 500.0)));
    doc.insertToolpath(r);
    const auto alone = saveCarv(doc);
    REQUIRE_FALSE(alone.ok());
    CHECK(alone.error().code == Code::ResourceLimit);
    CHECK(contains(alone.error().message, "move per byte"));
    std::mt19937 rng(7);
    Bytes pad(1'250'000);
    for (auto& b : pad) {
        b = static_cast<std::uint8_t>(rng());
    }
    const Bytes file = save(doc, CarvExtras{}.withMember("pad.bin", pad));
    CHECK(memberBytes(file, "toolpaths/0001.bin").size() > 1'200'000);
    LoadedCarv loaded = load(file);
    requireSameToolpaths(doc, loaded.document);
}

TEST_CASE("moves per byte of the file: the exact boundary, and small crafted files",
          "[document][toolpath][carv][limits]") {
    // A file of S bytes holds at most S moves (owner decision 2026-11-02). Build a file whose size
    // equals its move count exactly by padding it, then add one move.
    constexpr std::size_t kMoves = 50'000;
    Document doc;
    ToolpathRecord r;
    r.tool = kValidTool;  // computed moves need a real tool (T-0134)
    r.name = "boundary";
    r.selection = VectorSelection::allVectors();
    r.moves = shareMoves(std::vector<Move>(kMoves, Move::rapid({0, 0, 5})));
    doc.insertToolpath(r);
    std::mt19937 rng(11);
    Bytes pad(kMoves * 2);
    for (auto& b : pad) {
        b = static_cast<std::uint8_t>(rng());
    }
    const Bytes reference = save(doc, CarvExtras{}.withMember("pad.bin", pad));
    const auto zipWithPad = [&](std::size_t padBytes, const Bytes& cache, const std::string& job) {
        std::vector<detail::ZipMember> ms;
        for (const auto& m : members(reference)) {
            if (m.name == "pad.bin") {
                ms.push_back({m.name, Bytes(pad.begin(), pad.begin() + static_cast<std::ptrdiff_t>(padBytes))});
            } else if (m.name == "toolpaths/0001.bin") {
                ms.push_back({m.name, cache});
            } else if (m.name == "job.json") {
                ms.push_back({m.name, Bytes(job.begin(), job.end())});
            } else {
                ms.push_back(m);
            }
        }
        auto z = detail::writeZip(ms);
        REQUIRE(z.ok);
        return z.bytes;
    };
    const Bytes cache = memberBytes(reference, "toolpaths/0001.bin");
    const std::string job = memberText(reference, "job.json");
    // Binary search for the padding that makes the file exactly kMoves bytes long.
    std::size_t lo = 0;
    std::size_t hi = pad.size();
    while (lo < hi) {
        const std::size_t mid = (lo + hi) / 2;
        if (zipWithPad(mid, cache, job).size() < kMoves) {
            lo = mid + 1;
        } else {
            hi = mid;
        }
    }
    const Bytes exact = zipWithPad(lo, cache, job);
    REQUIRE(exact.size() == kMoves);
    CHECK(load(exact).document.toolpaths()[0].moveCount() == kMoves);
    // The same file one move longer (cache and count) is one move over.
    const Bytes cache1 = cacheOf(std::vector<Move>(kMoves + 1, Move::rapid({0, 0, 5})));
    const std::string job1 =
        replaced(job, "\"count\": " + std::to_string(kMoves), "\"count\": " + std::to_string(kMoves + 1));
    const Bytes over = zipWithPad(lo, cache1, job1);
    REQUIRE(over.size() == kMoves);
    const CarvError err = loadError(over);
    CHECK(err.code == Code::ResourceLimit);
    CHECK(contains(err.message, "more than 50000 toolpath moves"));
    CHECK(contains(err.message, "per byte of its size"));
    // saveCarv draws the same line: with the padding that gave the exact file it saves kMoves...
    const CarvExtras atPad = CarvExtras{}.withMember("pad.bin", Bytes(pad.begin(), pad.begin() + static_cast<std::ptrdiff_t>(lo)));
    CHECK(saveCarv(doc, atPad).ok());
    // ...and refuses kMoves + 1.
    Document more;
    r.moves = shareMoves(std::vector<Move>(kMoves + 1, Move::rapid({0, 0, 5})));
    more.insertToolpath(r);
    const auto refused = saveCarv(more, atPad);
    REQUIRE_FALSE(refused.ok());
    CHECK(refused.error().code == Code::ResourceLimit);

    // A small crafted file claiming the whole 8M budget is refused from job.json, before any
    // cache is read (the T-0132 verifier's 9 KB file loaded 8M moves at 581 MiB).
    const Bytes small = save(makeSample().doc);
    REQUIRE(small.size() < 20'000);
    const std::string smallJob = memberText(small, "job.json");
    const CarvError claim =
        loadError(withText(small, "job.json", replaced(smallJob, "\"count\": 11", "\"count\": 8000000")));
    CHECK(claim.code == Code::ResourceLimit);
    CHECK(contains(claim.message, "per byte of its size"));
#if defined(__linux__)
    // Worst case from a file of S bytes: S moves of 72 bytes, reserved exactly.
    const std::int64_t peak = childPeakGrowth(exact);
    WARN("a " << exact.size() << "-byte file at the move boundary: peak RSS growth " << peak / 1024 << " KiB");
    if (peak >= 0) {
        CHECK(peak < static_cast<std::int64_t>(kMoves) * 72 * 2 + (std::int64_t{16} << 20));
    }
#endif
}

TEST_CASE("saveCarv counts every archive member against the loader's member limit (T-0132 B1)",
          "[document][toolpath][carv][limits]") {
    // manifest + job + one cache per computed toolpath (+ preserved members) <= 10,000.
    const SharedMoves one = shareMoves({Move::rapid({0, 0, 5})});
    const auto docWith = [&one](std::size_t computed) {
        auto d = std::make_unique<Document>();
        for (std::size_t i = 0; i < computed; ++i) {
            ToolpathRecord t;
            t.tool = kValidTool;  // computed moves need a real tool (T-0134)
            t.moves = one;
            d->insertToolpath(t);
        }
        return d;
    };
    const auto at9998 = docWith(9998);
    const Bytes ok = save(*at9998);
    CHECK(load(ok).document.toolpaths().size() == 9998);
    const auto at9999 = docWith(9999);
    const auto refused = saveCarv(*at9999);
    REQUIRE_FALSE(refused.ok());
    CHECK(refused.error().code == Code::ResourceLimit);
    CHECK(contains(refused.error().message, "10001 archive members"));
    // Preserved members count too.
    const CarvExtras extra = CarvExtras{}.withMember("notes.txt", Bytes{'x'});
    const auto at9997 = docWith(9997);
    CHECK(load(save(*at9997, extra)).document.toolpaths().size() == 9997);
    const auto withExtra = saveCarv(*at9998, extra);
    REQUIRE_FALSE(withExtra.ok());
    CHECK(withExtra.error().code == Code::ResourceLimit);
    // Records without moves write no member.
    auto mixed = docWith(9998);
    mixed->insertToolpath(ToolpathRecord{});
    CHECK(saveCarv(*mixed).ok());
}

TEST_CASE("the move budget is enforced on load and on save", "[document][toolpath][carv][limits]") {
    const Sample s = makeSample();
    const Bytes good = save(s.doc);
    detail::LoadLimits limits;
    limits.maxMoves = 12;  // the file holds 2 + 11 moves
    auto r = detail::loadCarvWithLimits(good, limits, detail::ZipLimits{});
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == Code::ResourceLimit);
    CHECK(contains(r.error().message, "more than 12 toolpath moves"));
    limits.maxMoves = 13;
    CHECK(detail::loadCarvWithLimits(good, limits, detail::ZipLimits{}).ok());
    // Interim members count too.
    limits.maxMoves = 5;
    const Bytes legacy = legacyFile({{"toolpaths/0001.json", interimMember("pocket", "{}", kInterimMoves)}});
    auto lr = detail::loadCarvWithLimits(legacy, limits, detail::ZipLimits{});
    REQUIRE_FALSE(lr.ok());
    CHECK(lr.error().code == Code::ResourceLimit);
    // A count beyond the budget is refused from job.json, before the cache is inflated.
    const std::string job = memberText(good, "job.json");
    const CarvError huge = loadError(
        withText(good, "job.json", replaced(job, "\"count\": 11", "\"count\": 18446744073709551615")));
    CHECK(huge.code == Code::ResourceLimit);

    // Record counts: at most 10,000 records, and no computed toolpath past position 9,999 (its
    // cache would have no name), also when an interim member is appended after 9,999 records.
    const auto manyRecords = [](std::size_t n) {
        std::string records;
        for (std::size_t i = 0; i < n; ++i) {
            records += (i == 0 ? "" : ",") + std::string(R"({"id": )") + std::to_string(100 + i) +
                       R"(, "name": "", "operation": "pocket", "params": {}, "selection": {}, "tool": {"name": "", )"
                       R"("tool_number": 0, "diameter": 0, "feed_rate": 0, "plunge_rate": 0, "spindle_speed": 0}})";
        }
        return replaced(kJob10, "\"next_id\": 10,", "\"next_id\": 20000, \"toolpaths\": [" + records + "],");
    };
    {
        const std::string many = manyRecords(kCarvMaxCachedToolpaths);
        std::vector<detail::ZipMember> ms{{"manifest.json", Bytes(kManifest10.begin(), kManifest10.end())},
                                          {"job.json", Bytes(many.begin(), many.end())}};
        const Bytes noMoves = detail::writeZip(ms).bytes;
        CHECK(load(noMoves).document.toolpaths().size() == kCarvMaxCachedToolpaths);
        const std::string member = interimMember("pocket", "{}", kInterimMoves);
        ms.push_back({"toolpaths/0001.json", Bytes(member.begin(), member.end())});
        const CarvError err = loadError(detail::writeZip(ms).bytes);
        CHECK(err.code == Code::ResourceLimit);
        CHECK(contains(err.message, "the toolpath at position 10000 (#"));
        CHECK(contains(err.message, "only toolpaths at positions 1 to 9999 can keep computed moves"));
    }
    {
        const std::string many = manyRecords(10001);
        const std::vector<detail::ZipMember> ms{{"manifest.json", Bytes(kManifest10.begin(), kManifest10.end())},
                                                {"job.json", Bytes(many.begin(), many.end())}};
        const CarvError err = loadError(detail::writeZip(ms).bytes);
        CHECK(err.code == Code::ResourceLimit);
        CHECK(contains(err.message, "more than 10000 toolpaths"));
    }

    // saveCarv refuses to write more than the loader accepts (the moves are shared, so this is cheap).
    Document doc;
    const SharedMoves block = shareMoves(std::vector<Move>(kCarvMaxToolpathMoves / 4 + 1, Move::rapid({0, 0, 5})));
    for (int i = 0; i < 4; ++i) {
        ToolpathRecord t;
        t.tool = kValidTool;  // computed moves need a real tool (T-0134)
        t.moves = block;
        doc.insertToolpath(t);
    }
    const auto saved = saveCarv(doc);
    REQUIRE_FALSE(saved.ok());
    CHECK(saved.error().code == Code::ResourceLimit);
    CHECK(contains(saved.error().message, std::to_string(kCarvMaxToolpathMoves)));
    // And a non-finite move value.
    Document nan;
    ToolpathRecord t;
    t.tool = kValidTool;  // computed moves need a real tool (T-0134)
    t.name = "bad";
    t.moves = shareMoves({Move::feed({0, std::numeric_limits<double>::infinity(), 0})});
    nan.insertToolpath(t);
    const auto bad = saveCarv(nan);
    REQUIRE_FALSE(bad.ok());
    CHECK(bad.error().code == Code::InvalidContent);
    CHECK(contains(bad.error().message, "toolpath 'bad'"));
}

TEST_CASE("AC2: a 2,000,000-move toolpath saves and reloads within budget and time", "[document][toolpath][carv][scale]") {
    Document doc;
    doc.insertObject(Object::vector(geometry::Contour::rectangle(geometry::Box2{{0, 0}, {100, 100}}),
                                    doc.defaultLayer(), doc.activeSheet()));
    ToolpathRecord r;
    r.tool = kValidTool;  // computed moves need a real tool (T-0134)
    r.name = "Big pocket";
    r.params = fullPocket();
    r.selection = VectorSelection::allVectors();
    r.moves = shareMoves(bigToolpath(2'000'000));
    doc.insertToolpath(r);

    const double t0 = test::processCpuSeconds();
    const Bytes file = save(doc);
    const double t1 = test::processCpuSeconds();
    LoadedCarv back = load(file);
    const double t2 = test::processCpuSeconds();
    REQUIRE(back.document.toolpaths().size() == 1);
    CHECK(*back.document.toolpaths()[0].moves == *r.moves);
    CHECK(save(back.document, back.extras) == file);
    // The default limits hold it with room to spare (it is 25% of the move budget).
    CHECK(back.document.toolpaths()[0].moveCount() * 4 <= kCarvMaxToolpathMoves);
    const std::size_t cache = memberBytes(file, "toolpaths/0001.bin").size();
    WARN("2M moves: file " << file.size() / 1000 << " kB, cache " << cache / 1000 << " kB, save " << (t1 - t0)
                           << " s, load " << (t2 - t1) << " s CPU");
    CHECK(cache < 2'000'000 * 14);  // compact: well under the 49-byte worst case
#ifdef NDEBUG
    CHECK(t1 - t0 < 6.0);
    CHECK(t2 - t1 < 3.0);
#endif
#if defined(__linux__)
    // Memory: the 2M decoded moves are 144 MB; the load streams the cache (never holds it whole).
    const std::int64_t peak = childPeakGrowth(file);
    WARN("2M moves: peak RSS growth of the load " << peak / (1 << 20) << " MiB");
    if (peak >= 0) {
        CHECK(peak < (std::int64_t{400} << 20));
    }
#endif
}

// ------------------------------------------------------------------------------------------------
// Fuzzing (forked, under a timeout, like the T-0015 fuzz test)
// ------------------------------------------------------------------------------------------------

TEST_CASE("fuzz: mutated move caches and toolpath records never crash the loader", "[document][toolpath][fuzz]") {
    const Sample s = makeSample();
    const Bytes good = save(s.doc);
    const Bytes cache = memberBytes(good, "toolpaths/0002.bin");
    const std::string job = memberText(good, "job.json");
    const std::string interim = interimMember("pocket", R"({"cut_depth":3,"layers":["Outline"]})", kInterimMoves);
    std::mt19937 rng(1132);
    int loaded = 0;
    for (int i = 0; i < 600; ++i) {
        Bytes file;
        switch (i % 3) {
            case 0: {  // flip, insert or drop bytes inside the (uncompressed) cache
                Bytes c = cache;
                const unsigned edits = 1 + static_cast<unsigned>(rng() % 4);
                for (unsigned e = 0; e < edits && !c.empty(); ++e) {
                    const std::size_t pos = rng() % c.size();
                    switch (rng() % 3) {
                        case 0:
                            c[pos] ^= static_cast<std::uint8_t>(1 + rng() % 255);
                            break;
                        case 1:
                            c.insert(c.begin() + static_cast<std::ptrdiff_t>(pos), static_cast<std::uint8_t>(rng()));
                            break;
                        default:
                            c.erase(c.begin() + static_cast<std::ptrdiff_t>(pos));
                            break;
                    }
                }
                file = withMember(good, "toolpaths/0002.bin", c);
                break;
            }
            case 1: {  // mutate the toolpath part of job.json
                std::string t = job;
                const std::size_t from = t.find("\"toolpaths\"");
                const std::string alphabet = "{}[]\",:.-+eE0123456789 truefalsnul#";
                const unsigned edits = 1 + static_cast<unsigned>(rng() % 3);
                for (unsigned e = 0; e < edits; ++e) {
                    const std::size_t pos = from + rng() % (t.size() - from);
                    if (rng() % 2) {
                        t[pos] = alphabet[rng() % alphabet.size()];
                    } else {
                        const std::size_t d = t.find_first_of("0123456789", pos);
                        if (d != std::string::npos) {
                            t[d] = static_cast<char>('0' + rng() % 10);
                        }
                    }
                }
                file = withText(good, "job.json", t);
                break;
            }
            default: {  // mutate an interim member
                std::string t = interim;
                const unsigned edits = 1 + static_cast<unsigned>(rng() % 3);
                for (unsigned e = 0; e < edits; ++e) {
                    const std::size_t pos = rng() % t.size();
                    if (rng() % 2) {
                        t[pos] = "[]{},\"0123456789.e-"[rng() % 19];
                    } else {
                        t.erase(pos, 1 + rng() % 8);
                    }
                }
                file = legacyFile({{"toolpaths/0001.json", t}});
                break;
            }
        }
        const auto r = loadCarv(std::span<const std::uint8_t>(file));
        if (r.ok()) {
            ++loaded;
            INFO("case " << i);
            REQUIRE(r.value().document.checkIntegrity().empty());
            const Bytes once = save(r.value().document, r.value().extras);
            LoadedCarv again = load(once);
            CHECK(save(again.document, again.extras) == once);
        } else {
            CHECK_FALSE(r.error().message.empty());
        }
    }
    CHECK(loaded < 600);
}

#if defined(__linux__)
namespace {

/// Loads `file` in a forked child under `alarm(timeoutSeconds)` and re-saves on success. Returns
/// 0 for a normal return, the signal number on a crash or timeout, or a negative exit code for an
/// inconsistent result (empty error message, re-save failure).
int loadInChild(const Bytes& file, unsigned timeoutSeconds) {
    const pid_t pid = fork();
    if (pid == 0) {
        alarm(timeoutSeconds);
        auto r = loadCarv(std::span<const std::uint8_t>(file));
        if (r.ok()) {
            auto again = saveCarv(r.value().document, r.value().extras);
            _exit(again.ok() ? 0 : 3);
        }
        _exit(r.error().message.empty() ? 2 : 0);
    }
    if (pid < 0) {
        return -1;
    }
    int status = 0;
    waitpid(pid, &status, 0);
    if (WIFSIGNALED(status)) {
        return WTERMSIG(status);
    }
    return WEXITSTATUS(status) == 0 ? 0 : -WEXITSTATUS(status);
}

/// Offset and length of member `name`'s compressed data in a zip written by `writeZip`.
std::pair<std::size_t, std::size_t> compressedRange(const Bytes& zip, const std::string& name) {
    detail::ZipArchive archive(zip);
    std::string error;
    bool limit = false;
    REQUIRE(archive.open(detail::ZipLimits{}, error, limit));
    const detail::ZipEntry* e = archive.find(name);
    REQUIRE(e != nullptr);
    return {static_cast<std::size_t>(e->dataOffset), static_cast<std::size_t>(e->dataEnd - e->dataOffset)};
}

}  // namespace

TEST_CASE("fuzz under a per-load timeout: toolpath caches, records and interim members",
          "[document][toolpath][fuzz][ci]") {
    // Forked loads under alarm(), so a hang shows as SIGALRM and a crash as its signal: bit flips
    // in a cache's compressed data, truncation, central-directory size lies for the caches, and
    // byte edits in uncompressed caches and interim members. A cache of 20,000 moves keeps each
    // load meaningful but quick.
    Sample s = makeSample();
    ToolpathRecord big = s.doc.toolpaths()[1];
    big.moves = shareMoves(bigToolpath(20'000));
    REQUIRE(s.doc.execute(std::make_unique<SetToolpathCommand>(big)));
    const Bytes good = save(s.doc);
    const auto [start, length] = compressedRange(good, "toolpaths/0002.bin");
    const Bytes cache = memberBytes(good, "toolpaths/0002.bin");
    const std::string interim = interimMember("profile", R"({"layers":["Outline"]})", kInterimMoves);
    int hangs = 0;
    int crashes = 0;
    int inconsistent = 0;
    for (const std::uint32_t seed : {std::uint32_t{20261101}, std::uint32_t{132}}) {
        std::mt19937 rng(seed);
        for (int i = 0; i < 600; ++i) {
            Bytes file = good;
            switch (rng() % 5) {
                case 0: {
                    const unsigned flips = 1 + static_cast<unsigned>(rng() % 5);
                    for (unsigned f = 0; f < flips; ++f) {
                        file[start + rng() % length] ^= static_cast<std::uint8_t>(1u << (rng() % 8));
                    }
                    break;
                }
                case 1:
                    file.resize(start + rng() % length);
                    break;
                case 2: {
                    // Lie about the cache's sizes in the central directory (compressed 20,
                    // uncompressed 24).
                    const std::size_t eocd = file.size() - 22;
                    std::size_t at = static_cast<std::size_t>(file[eocd + 16]) |
                                     (static_cast<std::size_t>(file[eocd + 17]) << 8) |
                                     (static_cast<std::size_t>(file[eocd + 18]) << 16) |
                                     (static_cast<std::size_t>(file[eocd + 19]) << 24);
                    const std::size_t entries =
                        static_cast<std::size_t>(file[eocd + 10]) | (static_cast<std::size_t>(file[eocd + 11]) << 8);
                    for (std::size_t e = 0; e < entries; ++e) {
                        const std::size_t nameLen = file[at + 28] | (static_cast<std::size_t>(file[at + 29]) << 8);
                        const std::string entryName(file.begin() + static_cast<std::ptrdiff_t>(at + 46),
                                                    file.begin() + static_cast<std::ptrdiff_t>(at + 46 + nameLen));
                        if (entryName == "toolpaths/0002.bin") {
                            const std::size_t field = at + ((rng() % 2) ? 20 : 24);
                            const auto value = static_cast<std::uint32_t>(rng() % (std::uint64_t{1} << (rng() % 30)));
                            for (int b = 0; b < 4; ++b) {
                                file[field + static_cast<std::size_t>(b)] = static_cast<std::uint8_t>(value >> (8 * b));
                            }
                        }
                        at += 46 + nameLen + (file[at + 30] | (static_cast<std::size_t>(file[at + 31]) << 8)) +
                              (file[at + 32] | (static_cast<std::size_t>(file[at + 33]) << 8));
                    }
                    break;
                }
                case 3: {
                    Bytes c = cache;
                    const unsigned edits = 1 + static_cast<unsigned>(rng() % 6);
                    for (unsigned e = 0; e < edits; ++e) {
                        c[rng() % c.size()] ^= static_cast<std::uint8_t>(1 + rng() % 255);
                    }
                    if (rng() % 3 == 0) {
                        c.resize(rng() % c.size());
                    }
                    file = withMember(good, "toolpaths/0002.bin", c);
                    break;
                }
                default: {
                    std::string t = interim;
                    const unsigned edits = 1 + static_cast<unsigned>(rng() % 4);
                    for (unsigned e = 0; e < edits; ++e) {
                        const std::size_t pos = rng() % t.size();
                        switch (rng() % 3) {
                            case 0:
                                t[pos] = "[]{},\"0123456789.e-:"[rng() % 20];
                                break;
                            case 1:
                                t.erase(pos, 1 + rng() % 12);
                                break;
                            default:
                                t.insert(pos, t.substr(rng() % t.size(), 1 + rng() % 30));
                                break;
                        }
                    }
                    file = withText(good, "toolpaths/0001.json", t);
                    break;
                }
            }
            const int outcome = loadInChild(file, 5);
            if (outcome == SIGALRM) {
                ++hangs;
            } else if (outcome > 0) {
                ++crashes;
            } else if (outcome < 0) {
                ++inconsistent;
            }
            if (outcome != 0 && hangs + crashes + inconsistent <= 3) {
                WARN("seed " << seed << " iteration " << i << ": outcome " << outcome);
            }
        }
    }
    CHECK(hangs == 0);
    CHECK(crashes == 0);
    CHECK(inconsistent == 0);
}
#endif

// ------------------------------------------------------------------------------------------------
// saveCarv never writes a file loadCarv refuses (T-0132 verifier rounds 1-2): saveCarv runs the
// loader on its own output.
// ------------------------------------------------------------------------------------------------

namespace {

ToolpathRecord plainRecord(std::vector<Move> moves, bool withMoves = true) {
    ToolpathRecord r;
    r.name = "t";
    r.selection = VectorSelection::allVectors();
    r.tool = {"tool", 1, 6.0, 1000.0, 300.0, 18000.0};
    if (withMoves) {
        r.moves = shareMoves(std::move(moves));
    }
    return r;
}

std::vector<Move> randomMoves(std::size_t n, unsigned seed) {
    std::mt19937_64 g(seed);
    std::uniform_real_distribution<double> d(-500, 500);
    std::vector<Move> v;
    v.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        v.push_back(Move::feed({d(g), d(g), d(g)}));
    }
    return v;
}

/// Saves; when that succeeds the file must load, and re-save to the same bytes. Returns whether
/// the save succeeded (and the refusal message when not).
bool consistent(const Document& doc, const CarvExtras& extras = {}, std::string* why = nullptr) {
    const auto saved = saveCarv(doc, extras);
    if (!saved.ok()) {
        CHECK_FALSE(saved.error().message.empty());
        if (why) {
            *why = saved.error().message;
        }
        return false;
    }
    const auto loaded = loadCarv(std::span<const std::uint8_t>(saved.value()));
    INFO((loaded.ok() ? std::string() : loaded.error().message));
    REQUIRE(loaded.ok());
    const auto again = saveCarv(loaded.value().document, loaded.value().extras);
    REQUIRE(again.ok());
    CHECK(again.value() == saved.value());
    return true;
}

}  // namespace

TEST_CASE("consistency: toolpath record counts (nomoves, count)", "[document][toolpath][carv][limits]") {
    std::string why;
    Document atLimit;
    for (int i = 0; i < 10000; ++i) {
        atLimit.insertToolpath(plainRecord({}, false));
    }
    CHECK(consistent(atLimit));
    atLimit.insertToolpath(plainRecord({}, false));  // 10,001 records, none computed
    CHECK_FALSE(consistent(atLimit, {}, &why));
    CHECK(contains(why, "would not load again"));
    CHECK(contains(why, "more than 10000 toolpaths"));

    Document mixed;
    const SharedMoves two = shareMoves(randomMoves(2, 1));
    for (int i = 0; i < 9998; ++i) {
        ToolpathRecord r = plainRecord({}, false);
        r.moves = two;
        mixed.insertToolpath(r);
    }
    CHECK(consistent(mixed));
    for (int i = 0; i < 3000; ++i) {
        mixed.insertToolpath(plainRecord({}, false));
    }
    CHECK_FALSE(consistent(mixed, {}, &why));
    CHECK(contains(why, "more than 10000 toolpaths"));
}

TEST_CASE("consistency: selections with repeated or unusual ids (dupsel, zero)", "[document][toolpath][carv][limits]") {
    Document doc;
    Layer l;
    l.name = "a";
    const LayerId layer = doc.insertLayer(l);
    ToolpathRecord r = plainRecord({Move::rapid({1, 2, 30})});
    r.selection = {};
    r.selection.layers.assign(400'000, layer);
    auto add = std::make_unique<AddToolpathCommand>(r);
    REQUIRE(doc.execute(std::move(add)));
    CHECK(doc.toolpaths().back().selection.layers == std::vector<LayerId>{layer});  // deduplicated
    CHECK(doc.checkIntegrity().empty());
    CHECK(consistent(doc));
    // A raw insert can still hold repeats: checkIntegrity reports them once, and saveCarv refuses.
    ToolpathRecord raw = r;
    raw.id = {};
    doc.insertToolpath(raw);
    const auto issues = doc.checkIntegrity();
    REQUIRE(issues.size() == 1);
    CHECK(issues[0].kind == IntegrityIssue::Kind::BadToolpathSelection);
    CHECK(contains(issues[0].message, "more than once"));
    std::string why;
    CHECK_FALSE(consistent(doc, {}, &why));
    CHECK(contains(why, "would not load again"));
    // With few repeats the file is small enough for every other limit: the integrity check names it.
    raw.selection.layers.assign(3, layer);
    doc.takeToolpath(doc.toolpaths().back().id);
    doc.insertToolpath(raw);
    CHECK_FALSE(consistent(doc, {}, &why));
    CHECK(contains(why, "more than once"));

    // Distinct, allocated but unused ids are valid. 13M of them (the verifier's case) exceed the
    // loader's document budget and are refused by saveCarv's verification; that needs ~1 GB, so
    // this test only checks that a large valid selection round-trips.
    Document many;
    {
        Layer marker;
        marker.id = LayerId{100'000};
        many.insertLayer(marker);
        many.takeLayer(marker.id);
    }
    ToolpathRecord unused = plainRecord({});
    unused.selection = {};
    for (std::uint64_t i = 0; i < 50'000; ++i) {
        unused.selection.objects.push_back(ObjectId{100 + i});
    }
    REQUIRE(many.execute(std::make_unique<AddToolpathCommand>(unused)));
    CHECK(consistent(many));

    // zero: empty but computed moves, no moves.
    Document z;
    z.insertToolpath(plainRecord({}));
    z.insertToolpath(plainRecord({}, false));
    CHECK(consistent(z));
    // A dwell of -0.0 seconds is refused since T-0134 (no zero or negative pause).
    z.insertToolpath(plainRecord({Move::dwell({0, 0, 0}, -0.0)}));
    std::string zeroWhy;
    CHECK_FALSE(consistent(z, {}, &zeroWhy));
    CHECK(zeroWhy.find("move 1: dwell seconds must be greater than zero") != std::string::npos);
}

TEST_CASE("consistency: long strings and interim extras (strings, interimextra)", "[document][toolpath][carv][limits]") {
    const std::string longText(1'100'000, 'n');  // over the loader's 1 MiB string cap
    std::string why;
    {
        Document d;
        ToolpathRecord r = plainRecord({Move::rapid({1, 2, 30})});
        r.name = longText;
        d.insertToolpath(r);
        CHECK_FALSE(consistent(d, {}, &why));
        CHECK(contains(why, "would not load again"));
    }
    {
        Document d;
        ToolpathRecord r = plainRecord({Move::rapid({1, 2, 30})});
        r.tool.name = longText;
        d.insertToolpath(r);
        CHECK_FALSE(consistent(d));
    }
    {
        Document d;
        ToolpathRecord r = plainRecord({Move::rapid({1, 2, 30})});
        r.toolSource.toolId = longText;
        d.insertToolpath(r);
        CHECK_FALSE(consistent(d));
    }
    {
        Document d;
        Layer l;
        l.name = longText;
        d.insertLayer(l);
        CHECK_FALSE(consistent(d));
        Document ok;
        Layer shortName;
        shortName.name = std::string(1000, 'n');
        ok.insertLayer(shortName);
        CHECK(consistent(ok));
    }
    // Interim members can no longer be added through the API; a preserved .json member that is
    // not valid JSON is refused on save.
    CHECK_THROWS_AS(CarvExtras{}.withMember("toolpaths/0001.json", Bytes{'{', '}'}), std::invalid_argument);
    const CarvExtras broken = CarvExtras{}.withMember("notes.json", Bytes{'{', 'x'});
    CHECK_FALSE(consistent(Document{}, broken, &why));
    CHECK(contains(why, "notes.json"));
}

TEST_CASE("consistency: the move budget sweep (movebudget)", "[document][toolpath][carv][limits]") {
    int saved = 0;
    int refused = 0;
    for (std::size_t n = 2000; n <= 200'000; n = n * 13 / 10) {
        for (const std::size_t other : {std::size_t{0}, std::size_t{50}, std::size_t{2500}}) {
            Document d;
            d.insertToolpath(plainRecord(std::vector<Move>(n, Move::rapid({1, 2, 3}))));
            d.insertToolpath(plainRecord(randomMoves(other + 1, 3)));
            (consistent(d) ? saved : refused) += 1;
        }
    }
    CHECK(saved > 0);
    CHECK(refused > 0);
}

TEST_CASE("generative: documents near every limit either save and load or are refused on save",
          "[document][toolpath][carv][limits][fuzz]") {
    std::mt19937 rng(20261102);
    int saved = 0;
    int refused = 0;
    for (int round = 0; round < 24; ++round) {
        INFO("round " << round);
        Document d;
        CarvExtras extras;
        const bool manyRecords = rng() % 4 == 0;
        const std::size_t records = manyRecords ? 9995 + rng() % 8 : 1 + rng() % 5;
        const SharedMoves small = shareMoves(randomMoves(1 + rng() % 3, static_cast<unsigned>(round)));
        for (std::size_t i = 0; i < records; ++i) {
            ToolpathRecord r = plainRecord({}, false);
            if (rng() % 3 != 0) {
                r.moves = small;
            }
            d.insertToolpath(r);
        }
        if (rng() % 3 == 0) {  // repetitive moves near one move per file byte
            const std::size_t n = 20'000 + rng() % 60'000;
            ToolpathRecord r = plainRecord(std::vector<Move>(n, Move::rapid({0, 0, 5})));
            d.insertToolpath(r);
            Bytes pad(n - n / 8 + rng() % (n / 4));
            for (auto& b : pad) {
                b = static_cast<std::uint8_t>(rng());
            }
            extras = extras.withMember("pad.bin", pad);
        }
        for (unsigned e = 0; e < rng() % 4; ++e) {
            extras = extras.withMember("extra/" + std::to_string(e) + ".bin", Bytes(10, 1));
        }
        if (rng() % 5 == 0) {  // a name at the string cap (1 MiB) or just over it
            ToolpathRecord& r = *d.findToolpath(d.toolpaths()[0].id);
            r.name = std::string((std::size_t{1} << 20) - 3 + rng() % 7, 'q');
        }
        if (rng() % 6 == 0) {  // a raw repeated selection id
            ToolpathRecord& r = *d.findToolpath(d.toolpaths()[0].id);
            r.selection = {};
            r.selection.layers = {d.defaultLayer(), d.defaultLayer()};
        }
        (consistent(d, extras) ? saved : refused) += 1;
    }
    WARN("generative near-limit documents: " << saved << " saved and loaded, " << refused << " refused on save");
    CHECK(saved > 0);
    CHECK(refused > 0);
}

// --- T-0134 (h): zero or negative tool sizes, feeds and dwell are refused on load and save -----

namespace {

/// A one-move cache written by hand (the encoder refuses these values): `kind` with x, y, z
/// stored, then the dwell seconds or the feed override.
Bytes handCache(MoveKind kind, double value) {
    Bytes out{'C', 'V', 'T', 'P', 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0};
    const bool dwell = kind == MoveKind::Dwell;
    out.push_back(static_cast<std::uint8_t>(static_cast<int>(kind) | (dwell ? 0 : 0x08)));
    for (double v : {1.0, 2.0, -3.0, value}) {
        std::uint64_t bits = 0;
        std::memcpy(&bits, &v, sizeof bits);
        for (int i = 0; i < 8; ++i) {
            out.push_back(static_cast<std::uint8_t>(bits >> (8 * i)));
        }
    }
    return out;
}

}  // namespace

TEST_CASE("T-0134: loading refuses zero or negative tool sizes, feeds, feed overrides and dwell",
          "[document][toolpath][carv][errors]") {
    const Bytes good = save(makeSample().doc);
    const std::string job = memberText(good, "job.json");
    for (const char* bad : {"0.0", "-0.0", "-3.0", "\"NaN\"", "\"-Infinity\""}) {
        INFO(bad);
        // Native record: the tool snapshot in job.json.
        for (const auto& [key, value] : {std::pair{"diameter", "3.0"}, std::pair{"feed_rate", "1500.0"},
                                         std::pair{"plunge_rate", "500.0"}}) {
            INFO(key);
            const std::string from = std::string("\"") + key + "\": " + value;
            auto r = loadCarv(std::span<const std::uint8_t>(withText(good, "job.json", replaced(job, from, std::string("\"") + key + "\": " + bad))));
            REQUIRE_FALSE(r.ok());
            INFO(r.error().message);
            CHECK(r.error().code == Code::InvalidContent);
            CHECK(contains(r.error().message, std::string("tool.") + key + ": must be a finite number greater than zero"));
        }
        // Interim member being migrated: tool snapshot, dwell and feed override.
        const std::string tool = interimMember("pocket", "{}", kInterimMoves);
        for (const auto& [from, to, mention] :
             {std::tuple{std::string("\"diameter\":6.0"), std::string("\"diameter\":") + bad, std::string("tool.diameter: must be a finite number greater than zero")},
              std::tuple{std::string("\"feed_rate\":2300.0"), std::string("\"feed_rate\":") + bad, std::string("tool.feed_rate: must be a finite number greater than zero")},
              std::tuple{std::string("\"plunge_rate\":800.0"), std::string("\"plunge_rate\":") + bad, std::string("tool.plunge_rate: must be a finite number greater than zero")},
              std::tuple{std::string("-3.0,0.25]"), std::string("-3.0,") + bad + "]", std::string("moves[3][4]: ")},
              std::tuple{std::string("{\"feed\":300.0}"), std::string("{\"feed\":") + bad + "}", std::string("moves[1][4].feed: must be a finite number greater than zero")}}) {
            INFO(from);
            auto r = loadCarv(std::span<const std::uint8_t>(legacyFile({{"toolpaths/0001.json", replaced(tool, from, to)}})));
            REQUIRE_FALSE(r.ok());
            INFO(r.error().message);
            CHECK(r.error().code == Code::InvalidContent);
            CHECK(contains(r.error().message, "toolpaths/0001.json"));
            CHECK(contains(r.error().message, mention));
        }
    }

    // Binary move cache: dwell seconds and feed overrides, moves numbered from 1.
    for (double bad : {0.0, -0.0, -1.0, -std::numeric_limits<double>::denorm_min()}) {
        INFO(bad);
        CHECK(contains(decodeError(handCache(MoveKind::Dwell, bad), 1), "move 1: dwell seconds must be greater than zero"));
        CHECK(contains(decodeError(handCache(MoveKind::Feed, bad), 1), "move 1: feed override must be greater than zero"));
        // In a whole file: a record with one computed dwell, its cache swapped for the bad one.
        Document d;
        d.insertToolpath(plainRecord({Move::dwell({1, 2, -3}, 0.5)}));
        const Bytes file = withMember(save(d), "toolpaths/0001.bin", handCache(MoveKind::Dwell, bad));
        const CarvError e = loadError(file);
        CHECK(e.code == Code::InvalidContent);
        CHECK(contains(e.message, "toolpaths/0001.bin: move 1: dwell seconds must be greater than zero"));
    }
    CHECK(decodeError(handCache(MoveKind::Dwell, 0.25), 1).empty());
    CHECK(decodeError(handCache(MoveKind::Feed, 300.0), 1).empty());
}

TEST_CASE("T-0134: saving refuses zero or negative tool sizes, feeds, feed overrides and dwell",
          "[document][toolpath][carv][errors]") {
    const auto saveError = [](const ToolpathRecord& r) {
        Document d;
        d.insertToolpath(r);
        const auto saved = saveCarv(d, CarvExtras{});
        REQUIRE_FALSE(saved.ok());
        CHECK(saved.error().code == Code::InvalidContent);
        return saved.error().message;
    };
    for (double bad : {0.0, -0.0, -2.0, std::numeric_limits<double>::quiet_NaN()}) {
        INFO(bad);
        ToolpathRecord r = plainRecord({Move::rapid({0, 0, 5})});
        r.tool.diameter = bad;
        CHECK(contains(saveError(r), "toolpath 't' (#"));
        CHECK(contains(saveError(r), "tool diameter must be a finite number greater than zero"));
        r = plainRecord({Move::rapid({0, 0, 5})});
        r.tool.feedRate = bad;
        CHECK(contains(saveError(r), "tool feed rate must be a finite number greater than zero"));
        r = plainRecord({Move::rapid({0, 0, 5})});
        r.tool.plungeRate = bad;
        CHECK(contains(saveError(r), "tool plunge rate must be a finite number greater than zero"));
        if (std::isfinite(bad)) {
            CHECK(contains(saveError(plainRecord({Move::rapid({0, 0, 5}), Move::dwell({0, 0, 5}, bad)})),
                           "move 2: dwell seconds must be greater than zero"));
            CHECK(contains(saveError(plainRecord({Move::rapid({0, 0, 5}), Move::feed({1, 0, 5}, bad)})),
                           "move 2: feed override must be greater than zero"));
        }
    }
    // Computed but empty (zero moves) still counts as computed.
    ToolpathRecord empty = plainRecord({});
    empty.tool = {};
    CHECK(contains(saveError(empty), "tool diameter must be a finite number greater than zero"));
}

TEST_CASE("T-0134: a record without computed moves keeps a default (zero) tool snapshot",
          "[document][toolpath][carv]") {
    // T-0132 behaviour: an uncomputed record may have the default snapshot; it saves and loads.
    Document d;
    d.insertToolpath(ToolpathRecord{});
    ToolpathRecord negative = plainRecord({}, false);
    negative.tool.diameter = -1.0;
    negative.tool.feedRate = 0.0;
    d.insertToolpath(negative);
    const Bytes file = save(d);
    const LoadedCarv back = load(file);
    REQUIRE(back.document.toolpaths().size() == 2);
    CHECK(back.document.toolpaths()[0].tool == toolpath::ToolSnapshot{});
    CHECK(back.document.toolpaths()[1].tool.diameter == -1.0);
    CHECK(consistent(d));

    // The same zero tool on a computed record is refused on save and on load.
    Document computed;
    computed.insertToolpath(plainRecord({Move::rapid({0, 0, 5})}));
    const Bytes good = save(computed);
    const std::string job = memberText(good, "job.json");
    const CarvError e = loadError(withText(good, "job.json", replaced(job, "\"diameter\": 6.0", "\"diameter\": 0.0")));
    CHECK(e.code == Code::InvalidContent);
    CHECK(contains(e.message, "tool.diameter: must be a finite number greater than zero for a toolpath with computed moves"));
    ToolpathRecord zero = plainRecord({Move::rapid({0, 0, 5})});
    zero.tool = {};
    Document z;
    z.insertToolpath(zero);
    const auto saved = saveCarv(z, CarvExtras{});
    REQUIRE_FALSE(saved.ok());
    CHECK(contains(saved.error().message, "tool diameter must be a finite number greater than zero for a toolpath with computed moves"));
}
