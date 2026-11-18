// T-0354: end-to-end acceptance scenarios (PLAN.md section 5), run headless through the library the way the
// application drives it. Three scenarios:
//   1. 2D sign: open a DXF, put text on a curve, pocket, V-carve the text with a clearance tool, profile with
//      tabs, save/load the document, simulate, post to the shipped Grbl post.
//   2. 3D relief: read an STL, raster it, rough + finish, simulate, post to the shipped Grbl post.
//   3. Rotary: a mesh wrapped onto a cylinder, rough + finish, simulate in the flat design space, post with the
//      wrapped rotary axis.
// Each scenario checks the G-code against a golden file (core/tests/golden/e2e, see its README), re-reads the
// G-code and compares it move by move with the toolpath, and checks the simulated stock. The goldens are written
// only by the hidden [.regen] test (tools/regen_golden.py) and must be hand-reviewed.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <limits>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <sstream>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/text_object.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/io/dxf.hpp"
#include "carvetto/io/mesh.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/relief/mesh_raster.hpp"
#include "carvetto/relief/relief.hpp"
#include "carvetto/relief/rotary_raster.hpp"
#include "carvetto/simulate/simulator.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/tooling/tool_database.hpp"
#include "carvetto/toolpath/finish3d.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/rough3d.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "../text/text_test_util.hpp"

using namespace carvetto;
using Catch::Approx;
using geometry::Vec2;
using geometry::Vec3;

namespace {

constexpr double kPi = std::numbers::pi;

// ---------------------------------------------------------------------------------------------
// Files and the tool database.
// ---------------------------------------------------------------------------------------------

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        FAIL("cannot open " << path);
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

void writeFile(const std::string& path, const std::string& text) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    REQUIRE(out);
    out << text;
}

std::string goldenPath(const std::string& name) {
    return std::string(CARVETTO_GOLDEN_DIR) + "/e2e/" + name;
}

const tooling::ToolDatabase& starterDb() {
    static const tooling::ToolDatabase db = [] {
        tooling::ImportResult r = tooling::importToolDatabaseJson(readFile(CARVETTO_STARTER_TOOLS));
        if (!r.ok) {
            FAIL("starter tools: " << r.error);
        }
        return std::move(r.database);
    }();
    return db;
}

const tooling::Tool& starterTool(const char* id) {
    const tooling::Tool* t = starterDb().findTool(id);
    if (t == nullptr) {
        FAIL("starter tool missing: " << id);
    }
    return *t;
}

toolpath::Rough3dTool rough3dTool(const char* id) {
    toolpath::Rough3dTool t;
    t.tool = starterTool(id);
    return t;
}

post::PostProcessor shippedPost(const std::string& stem, const std::string& prefix = {},
                                const std::vector<std::pair<std::string, std::string>>& replace = {}) {
    std::string text = readFile(std::string(CARVETTO_POSTS_DIR) + "/" + stem + ".pp");
    for (const auto& [from, to] : replace) {
        const std::size_t at = text.find(from);
        REQUIRE(at != std::string::npos);
        text.replace(at, from.size(), to);
    }
    auto r = post::PostProcessor::parse(prefix + text);
    if (!r.ok()) {
        FAIL(stem << ": " << r.error().describe());
    }
    return r.value();
}

std::string runPost(const post::PostProcessor& pp, const std::vector<toolpath::Toolpath>& toolpaths,
                    const post::PostContext& ctx) {
    auto r = pp.run(toolpaths, ctx);
    if (!r.ok()) {
        FAIL("post run failed: " << r.error().describe());
    }
    CHECK(r.value().warnings.empty());
    return r.value().text;
}

// ---------------------------------------------------------------------------------------------
// G-code checks shared by the scenarios.
// ---------------------------------------------------------------------------------------------

struct Pos {
    int g{0};  ///< 0 rapid, 1 line, 2/3 arcs.
    double x{0}, y{0}, z{0}, a{0};
    bool hasA{false};
};

/// Motion lines of a program with modal X/Y/Z/A, skipping comments. Arcs keep their end point.
std::vector<Pos> motionOf(const std::string& gcode) {
    std::vector<Pos> out;
    std::istringstream in(gcode);
    std::string line;
    Pos cur;
    bool any = false;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '(' || line[0] == ';') {
            continue;
        }
        std::istringstream ls(line);
        std::string tok;
        int g = -1;
        Pos next = cur;
        bool axis = false;
        while (ls >> tok) {
            const double v = std::atof(tok.c_str() + 1);
            switch (tok[0]) {
                case 'G':
                    if (tok == "G0" || tok == "G1" || tok == "G2" || tok == "G3") {
                        g = static_cast<int>(v);
                    }
                    break;
                case 'X': next.x = v; axis = true; break;
                case 'Y': next.y = v; axis = true; break;
                case 'Z': next.z = v; axis = true; break;
                case 'A': next.a = v; next.hasA = true; axis = true; break;
                default: break;
            }
        }
        (void)axis;
        if (g >= 0) {
            next.g = g;
            out.push_back(next);
            cur = next;
            any = true;
        }
    }
    CHECK(any);
    return out;
}

/// Program-level safety rules (posts/README.md checklist) on a Grbl-style program.
void checkProgram(const std::string& gcode, double safeZ, double deepestZ, std::size_t toolPauses) {
    CHECK(gcode.find("G21") != std::string::npos);
    CHECK(gcode.find("G90") != std::string::npos);
    CHECK(gcode.find("nan") == std::string::npos);
    CHECK(gcode.find("inf") == std::string::npos);
    for (const char c : gcode) {
        CHECK(((c >= 32 && c < 127) || c == '\n'));
    }
    CHECK(gcode.rfind("M30") != std::string::npos);
    // The spindle is on before the first cutting move.
    const std::size_t m3 = gcode.find("\nM3 S");
    REQUIRE(m3 != std::string::npos);
    const std::size_t firstFeed = gcode.find("\nG1 ");
    REQUIRE(firstFeed != std::string::npos);
    CHECK(m3 < firstFeed);
    std::size_t pauses = 0;
    for (std::size_t at = gcode.find("\nM0\n"); at != std::string::npos; at = gcode.find("\nM0\n", at + 1)) {
        ++pauses;
    }
    CHECK(pauses == toolPauses);
    for (const Pos& p : motionOf(gcode)) {
        CHECK(p.z >= deepestZ - 1e-6);
        CHECK(p.z <= safeZ + 1e-6);
        if (p.g == 0) {
            CHECK(p.z == Approx(safeZ).margin(1e-6));  // rapids only at safe Z
        }
    }
}

/// Every non-dwell move of `toolpaths` is one motion line of the program, same end point (X/Y/Z, or X/A for a
/// wrapped axis `y -> a` with the given radius).
void checkProgramMatchesMoves(const std::string& gcode, const std::vector<toolpath::Toolpath>& toolpaths,
                              double wrapRadius = 0.0) {
    std::vector<toolpath::Move> moves;
    for (const toolpath::Toolpath& tp : toolpaths) {
        for (const toolpath::Move& m : tp.moves) {
            if (m.kind != toolpath::MoveKind::Dwell) {
                moves.push_back(m);
            }
        }
    }
    const std::vector<Pos> lines = motionOf(gcode);
    // Every move is one line, in order. The post's own safe-Z lifts (header, tool changes, footer) and the
    // home move are the only extra motion lines.
    auto matches = [&](const Pos& l, const toolpath::Move& m) {
        const bool y = wrapRadius > 0.0
                           ? std::fabs(l.a - m.to.y / wrapRadius * 180.0 / kPi) <= 2e-3
                           : std::fabs(l.y - m.to.y) <= 6e-4;
        return std::fabs(l.x - m.to.x) <= 6e-4 && std::fabs(l.z - m.to.z) <= 6e-4 && y;
    };
    std::size_t k = 0;
    for (std::size_t i = 0; i < lines.size(); ++i) {
        if (k < moves.size() && matches(lines[i], moves[k])) {
            ++k;
        } else {
            INFO("line " << i << " G" << lines[i].g << " X" << lines[i].x << " Z" << lines[i].z);
            CHECK(lines[i].g == 0);  // a post lift or the home move, not a cutting move
        }
    }
    CHECK(k == moves.size());
}

simulate::Simulator runSim(const simulate::Stock& stock,
                             const std::vector<std::pair<toolpath::Toolpath, simulate::ToolShape>>& runs) {
    simulate::Simulator sim(stock);
    REQUIRE(sim.ok());
    std::uint32_t id = 1;
    for (const auto& [tp, shape] : runs) {
        REQUIRE(sim.addToolpath(tp, shape, id++));
    }
    REQUIRE(sim.advanceAll());
    return sim;
}

simulate::ToolShape shapeOf(const tooling::Tool& tool) {
    auto s = simulate::ToolShape::fromTool(tool);
    REQUIRE(s);
    return *s;
}

double heightAt(const simulate::Heightfield& hf, double x, double y) {
    std::size_t i = 0, j = 0;
    REQUIRE(hf.cellAt({x, y}, i, j));
    return hf.height(i, j);
}

// ---------------------------------------------------------------------------------------------
// Scenario 1: 2D sign.
// ---------------------------------------------------------------------------------------------

constexpr double kThickness = 10.0;
constexpr double kSafeZ = 5.0;
constexpr double kPocketDepth = 4.0;
constexpr double kCarveFlat = 0.6;
constexpr double kTabThickness = 2.0;

struct SignJob {
    document::Document doc;
    document::ObjectId textId;
    std::vector<toolpath::Toolpath> toolpaths;  ///< Pocket, V-carve clearance, V-carve, profile (cutting order).
    std::vector<tooling::Tool> tools;           ///< The tool of each toolpath.
    std::vector<std::string> diagnostics;
    bool textTruncated{false};
};

void addRecord(document::Document& doc, const toolpath::Toolpath& tp, document::ToolpathParams params,
               const document::VectorSelection& selection, const std::string& toolId) {
    document::ToolpathRecord r;
    r.name = tp.name;
    r.params = std::move(params);
    r.selection = selection;
    r.tool = tp.tool;
    r.toolSource = {toolId, "", ""};
    r.moves = document::shareMoves(tp.moves);
    REQUIRE(doc.execute(std::make_unique<document::AddToolpathCommand>(std::move(r))));
}

std::vector<geometry::Contour> contoursOf(const document::Document& doc, const document::VectorSelection& sel) {
    std::vector<geometry::Contour> out;
    for (const document::ObjectId id : doc.selectedVectors(sel)) {
        const auto& cs = doc.findObject(id)->asVector()->contours;
        out.insert(out.end(), cs.begin(), cs.end());
    }
    return out;
}

SignJob buildSign() {
    SignJob job;
    document::Document& doc = job.doc;

    // 1. Open the DXF into a document (layers, one vector object per contour).
    const std::string dxfText = readFile(std::string(CARVETTO_E2E_FIXTURE_DIR) + "/bracket.dxf");
    auto imported = io::importDxf(std::span<const std::uint8_t>(
        reinterpret_cast<const std::uint8_t*>(dxfText.data()), dxfText.size()));
    REQUIRE(imported.ok());
    const io::DxfDrawing& dxf = imported.value();
    CHECK(dxf.warnings.empty());
    REQUIRE(dxf.layers.size() == 3);
    doc.setJob(document::Job::singleSided(120.0, 70.0, kThickness, document::Units::Millimetres));
    const document::LayerId placeholder = doc.defaultLayer();
    std::map<std::string, document::LayerId> layer;
    for (const io::DxfLayer& l : dxf.layers) {
        document::Layer nl;
        nl.name = l.name;
        layer[l.name] = doc.insertLayer(nl);
    }
    doc.setActiveLayer(layer["Outline"]);
    doc.takeLayer(placeholder);
    geometry::Contour guide;
    for (const io::DxfContour& c : dxf.contours) {
        doc.insertObject(document::Object::vector(c.contour, layer[dxf.layers.at(c.layer).name], doc.activeSheet()));
        if (dxf.layers.at(c.layer).name == "Guide") {
            guide = c.contour;
        }
    }
    REQUIRE(doc.objectCount() == 3);
    REQUIRE_FALSE(guide.spans.empty());

    // 2. Text on the curve: the DXF arc runs right to left, reversed so that the text reads left to right.
    const std::vector<std::uint8_t> fontBytes = text::testutil::fontBytes();
    document::TextParams tp;
    tp.text = "TOIT";
    tp.fontKey = document::fontKeyFor(fontBytes);
    tp.sizeMm = 16.0;
    tp.mode = document::TextMode::Curve;
    tp.guide = guide.reversed();
    tp.startOffsetMm = 5.0;
    REQUIRE(doc.insertFont(document::FontAsset{tp.fontKey, fontBytes}));
    const text::Font font = text::testutil::loadTestFont();
    const document::TextBuild built = document::buildTextContours(font, tp);
    REQUIRE(built.ok());
    job.textTruncated = built.truncated;
    CHECK_FALSE(built.truncated);
    CHECK_FALSE(built.missingGlyphs);
    REQUIRE(built.contours.size() == 5);  // T, O (outer + counter), I, T
    doc.setActiveLayer(layer["Guide"]);
    job.textId = doc.insertObject(document::Object::vector(built.contours, layer["Guide"], doc.activeSheet(), tp));

    // 3. Pocket (6 mm end mill, 4 mm deep).
    const tooling::Tool& end6 = starterTool("end-mill-6mm");
    document::VectorSelection pocketSel;
    pocketSel.layers = {layer["Pocket"]};
    toolpath::PocketParams pp;
    pp.name = "Pocket";
    pp.cutDepth = kPocketDepth;
    pp.safeZ = kSafeZ;
    const toolpath::PocketResult pr =
        toolpath::pocketToolpath(contoursOf(doc, pocketSel), {toolpath::PocketTool{end6, std::nullopt}}, pp);
    REQUIRE(pr.toolpaths.size() == 1);
    REQUIRE_FALSE(pr.toolpaths[0].isEmpty());
    for (const auto& d : pr.diagnostics) job.diagnostics.push_back(d.message);
    addRecord(doc, pr.toolpaths[0], pp, pocketSel, end6.id);
    job.toolpaths.push_back(pr.toolpaths[0]);
    job.tools.push_back(end6);

    // 4. V-carve the text with a V-bit, flat-bottomed 0.6 mm, the flat area cleared by a 1 mm end mill.
    const tooling::Tool& vbit = starterTool("v-bit-60-1-4in");
    const tooling::Tool& end1 = starterTool("end-mill-1mm");
    document::VectorSelection textSel;
    textSel.objects = {job.textId};
    toolpath::VCarvingParams vp;
    vp.name = "Sign text";
    vp.flatDepth = kCarveFlat;
    vp.safeZ = kSafeZ;
    vp.tolerance.chordal = 0.02;
    tooling::CuttingData carveData = vbit.defaultCuttingData;
    carveData.stepover = 0.5;  // at most what the V-bit cuts at the flat bottom (0.69 mm)
    const toolpath::VCarvingResult vr = toolpath::vCarvingToolpath(
        contoursOf(doc, textSel), vbit, carveData,
        {toolpath::PocketTool{end1, std::nullopt}}, vp);
    REQUIRE(vr.ok());
    for (const auto& d : vr.diagnostics) job.diagnostics.push_back(d.message);
    REQUIRE(vr.clearance.size() == 1);
    REQUIRE_FALSE(vr.clearance[0].isEmpty());
    addRecord(doc, vr.clearance[0], vp, textSel, end1.id);
    addRecord(doc, vr.toolpath, vp, textSel, vbit.id);
    job.toolpaths.push_back(vr.clearance[0]);
    job.tools.push_back(end1);
    job.toolpaths.push_back(vr.toolpath);
    job.tools.push_back(vbit);

    // 5. Profile outside the outline with four tabs, through the 10 mm stock.
    const tooling::Tool& end3 = starterTool("end-mill-3mm");
    document::VectorSelection outlineSel;
    outlineSel.layers = {layer["Outline"]};
    toolpath::ProfileParams pf;
    pf.name = "Cut out";
    pf.side = toolpath::ProfileSide::Outside;
    pf.passes.cutDepth = kThickness;
    pf.passes.passDepth = 2.5;
    pf.safeZ = kSafeZ;
    pf.tabs.enabled = true;
    pf.tabs.count = 4;
    pf.tabs.length = 8.0;
    pf.tabs.thickness = kTabThickness;
    const toolpath::ProfileResult fr = toolpath::profileToolpath(contoursOf(doc, outlineSel), end3, pf);
    REQUIRE_FALSE(fr.toolpath.isEmpty());
    for (const auto& d : fr.diagnostics) job.diagnostics.push_back(d.message);
    addRecord(doc, fr.toolpath, pf, outlineSel, end3.id);
    job.toolpaths.push_back(fr.toolpath);
    job.tools.push_back(end3);
    return job;
}

post::PostContext signContext() {
    post::PostContext ctx;
    ctx.fileName = "sign";
    ctx.fileExt = "nc";
    ctx.product = "Carvetto";
    ctx.safeZ = kSafeZ;
    ctx.home = {0, 0, kSafeZ};
    ctx.materialMin = {0, 0, -kThickness};
    ctx.materialMax = {120, 70, 0};
    return ctx;
}

std::string signGcode(const SignJob& job) {
    return runPost(shippedPost("grbl_mm"), job.toolpaths, signContext());
}

}  // namespace

TEST_CASE("e2e 2D: DXF, text on a curve, pocket, V-carve with clearance, profile with tabs, Grbl", "[e2e]") {
    const SignJob job = buildSign();
    // No diagnostics from any operation: every part was placed and nothing was trimmed or refused.
    for (const std::string& d : job.diagnostics) {
        INFO(d);
        CHECK(d.empty());
    }
    REQUIRE(job.toolpaths.size() == 4);
    // Each toolpath has its own tool number (so the post stops for a change between all of them).
    std::vector<int> numbers;
    for (const toolpath::Toolpath& t : job.toolpaths) {
        numbers.push_back(t.tool.toolNumber);
    }
    std::sort(numbers.begin(), numbers.end());
    CHECK(std::adjacent_find(numbers.begin(), numbers.end()) == numbers.end());

    // The text sits on the guide: inside the outline, clear of the pocket, above the arc's chord.
    const geometry::Box2 tb = job.doc.findObject(job.textId)->asVector()->bounds();
    CHECK(tb.min.x > 32.0);
    CHECK(tb.max.x < 110.0);
    CHECK(tb.min.y > 25.0);
    CHECK(tb.max.y < 60.0);

    const std::string gcode = signGcode(job);
    checkProgram(gcode, kSafeZ, -kThickness, 3);
    checkProgramMatchesMoves(gcode, job.toolpaths);
    CHECK(gcode == readFile(goldenPath("sign_grbl.nc")));
    CHECK(gcode == signGcode(job));  // deterministic
}

TEST_CASE("e2e 2D: the document round trip keeps the toolpaths and the G-code", "[e2e]") {
    const SignJob job = buildSign();
    auto bytes = document::saveCarv(job.doc);
    REQUIRE(bytes.ok());
    auto loaded = document::loadCarv(bytes.value());
    REQUIRE(loaded.ok());
    const document::Document& doc = loaded.value().document;
    CHECK(doc.checkIntegrity().empty());
    CHECK(doc.objectCount() == 4);
    REQUIRE(doc.toolpaths().size() == 4);
    // Cutting order: the records were added in the order pocket, clearance, V-carve, profile.
    std::vector<toolpath::Toolpath> fromRecords;
    for (const document::ToolpathRecord& r : doc.toolpaths()) {
        REQUIRE(r.hasMoves());
        fromRecords.push_back(r.toolpath());
    }
    CHECK(runPost(shippedPost("grbl_mm"), fromRecords, signContext()) == signGcode(job));
    // The text object keeps its parameters and the embedded font.
    const document::Object* t = doc.findObject(job.textId);
    REQUIRE(t != nullptr);
    REQUIRE(std::holds_alternative<document::TextParams>(t->asVector()->params));
    CHECK(std::get<document::TextParams>(t->asVector()->params).text == "TOIT");
    CHECK(doc.fonts().size() == 1);
}

TEST_CASE("e2e 2D: the simulation shows the pocket, the carved text and the tabbed cut-out", "[e2e][simulate]") {
    const SignJob job = buildSign();
    simulate::Stock stock;
    stock.width = 140.0;
    stock.height = 90.0;
    stock.thickness = kThickness;
    stock.origin = {-10.0, -10.0};
    stock.cellSize = 0.25;
    std::vector<std::pair<toolpath::Toolpath, simulate::ToolShape>> runs;
    for (std::size_t i = 0; i < job.toolpaths.size(); ++i) {
        runs.emplace_back(job.toolpaths[i], shapeOf(job.tools[i]));
    }
    simulate::Simulator sim = runSim(stock, runs);
    CHECK(sim.diagnostics().empty());
    const simulate::Heightfield& hf = sim.heightfield();

    // Untouched surface between the features.
    CHECK(heightAt(hf, 60.0, 10.0) == Approx(0.0).margin(1e-6));
    CHECK(heightAt(hf, 100.0, 60.0) == Approx(0.0).margin(1e-6));
    // The pocket floor (20 x 30 mm pocket, the 6 mm cutter reaches every part of it).
    for (const Vec2 p : {Vec2{20.0, 27.0}, Vec2{12.0, 14.0}, Vec2{28.0, 40.0}, Vec2{12.0, 40.0}}) {
        INFO(p.x << "," << p.y);
        CHECK(heightAt(hf, p.x, p.y) == Approx(-kPocketDepth).margin(1e-3));
    }
    // Pocket walls stay put just outside it.
    CHECK(heightAt(hf, 8.5, 27.0) == Approx(0.0).margin(1e-6));
    // The carved text: some cells down to the flat depth and none deeper, only inside the text's box.
    const geometry::Box2 tb = job.doc.findObject(job.textId)->asVector()->bounds();
    std::size_t carved = 0;
    double deepest = 0.0;
    for (std::size_t j = 0; j < hf.ny(); ++j) {
        for (std::size_t i = 0; i < hf.nx(); ++i) {
            const Vec2 c = hf.centreOf(i, j);
            const bool inText = c.x >= tb.min.x - 1.0 && c.x <= tb.max.x + 1.0 && c.y >= tb.min.y - 1.0 &&
                                c.y <= tb.max.y + 1.0;
            if (inText) {
                deepest = std::min(deepest, static_cast<double>(hf.height(i, j)));
                carved += hf.height(i, j) < -0.05f ? 1u : 0u;
            }
        }
    }
    CHECK(carved > 1500);                          // the four letters are carved over a real area
    CHECK(deepest == Approx(-kCarveFlat).margin(0.02));  // the flat depth, never deeper
    // Cut-out: outside the outline the stock is gone, except in the tabs (left at the tab thickness).
    std::size_t ringBottom = 0, ringTab = 0, ringOther = 0;
    for (double x = 8.0; x <= 112.0; x += 1.0) {  // straight stretches along the bottom and top sides
        for (const double y : {-1.5, 71.5}) {
            const double z = heightAt(hf, x, y);
            if (std::fabs(z + kThickness) < 1e-3) {
                ++ringBottom;
            } else if (std::fabs(z + (kThickness - kTabThickness)) < 1e-3) {
                ++ringTab;
            } else {
                ++ringOther;
            }
        }
    }
    CHECK(ringOther == 0);
    CHECK(ringTab > 0);
    CHECK(ringBottom > ringTab);
    // The part itself is still there inside the cut: its outline edge stays at the surface.
    CHECK(heightAt(hf, 1.0, 35.0) == Approx(0.0).margin(1e-6));
}

// ---------------------------------------------------------------------------------------------
// Scenario 2: 3D relief from an STL.
// ---------------------------------------------------------------------------------------------

namespace {

/// A closed solid (top surface z = f(x, y) over [0, w] x [0, h], flat bottom at z = 0) as a triangle mesh.
io::TriangleMesh domeMesh(int nx, int ny, double step) {
    io::TriangleMesh m;
    const double w = (nx - 1) * step, h = (ny - 1) * step;
    auto f = [&](double x, double y) {
        const double r2 = (x - 0.5 * w) * (x - 0.5 * w) + (y - 0.5 * h) * (y - 0.5 * h);
        return 3.0 + 5.0 * std::exp(-r2 / 60.0) + 1.0 * std::sin(x * 0.6) * std::cos(y * 0.5);
    };
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            m.vertices.push_back({i * step, j * step, f(i * step, j * step)});
        }
    }
    auto idx = [&](int i, int j) { return static_cast<std::uint32_t>(j * nx + i); };
    for (int j = 0; j + 1 < ny; ++j) {
        for (int i = 0; i + 1 < nx; ++i) {
            m.triangles.push_back({idx(i, j), idx(i + 1, j), idx(i + 1, j + 1)});
            m.triangles.push_back({idx(i, j), idx(i + 1, j + 1), idx(i, j + 1)});
        }
    }
    // Flat bottom, two triangles.
    const auto b = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.push_back({0, 0, 0});
    m.vertices.push_back({w, 0, 0});
    m.vertices.push_back({w, h, 0});
    m.vertices.push_back({0, h, 0});
    m.triangles.push_back({b, b + 2, b + 1});
    m.triangles.push_back({b, b + 3, b + 2});
    return m;
}

std::string binaryStl(const io::TriangleMesh& m) {
    std::string out(80, ' ');
    const auto count = static_cast<std::uint32_t>(m.triangles.size());
    out.append(reinterpret_cast<const char*>(&count), 4);  // little-endian hosts only (CI: x86-64, ARM64 LE)
    for (const auto& t : m.triangles) {
        const float zero[3] = {0, 0, 0};
        out.append(reinterpret_cast<const char*>(zero), 12);
        for (const std::uint32_t v : t) {
            const float c[3] = {static_cast<float>(m.vertices[v].x), static_cast<float>(m.vertices[v].y),
                                static_cast<float>(m.vertices[v].z)};
            out.append(reinterpret_cast<const char*>(c), 12);
        }
        out.append(2, '\0');
    }
    return out;
}

struct ReliefJob {
    relief::HeightField model;  ///< Heights above the stock bottom (0..max).
    toolpath::Rough3dParams roughParams;
    toolpath::Finish3dParams finishParams;
    std::vector<toolpath::Toolpath> toolpaths;  ///< Rough, finish.
    std::vector<toolpath::Rough3dTool> tools;
};

ReliefJob buildRelief() {
    ReliefJob job;
    // Read an STL (the reader parses what a user would open), size it and raster it top down.
    const std::string stl = binaryStl(domeMesh(21, 16, 1.0));
    auto read = io::readStl(stl);
    REQUIRE(read.ok());
    io::TriangleMesh mesh = std::move(read.value());
    REQUIRE(mesh.triangles.size() == 2u * 20u * 15u + 2u);
    const io::MeshBounds b = io::meshBounds(mesh);
    CHECK(b.size().x == Approx(20.0));
    CHECK(b.size().y == Approx(15.0));
    relief::MeshRasterOptions ro;
    ro.cellSize = 0.5;
    ro.zeroPlane = 0.0;
    auto raster = relief::meshToHeightField(mesh, ro);
    REQUIRE(raster.ok());
    job.model = raster.value().top;
    CHECK(raster.value().undercutCount == 0);
    const double maxHeight = raster.value().maxHeight;
    CHECK(maxHeight > 8.0);
    CHECK(maxHeight < 10.0);

    // Material 12 mm thick, surface at Z 0: the model's top at the surface.
    toolpath::Rough3dParams rp;
    rp.name = "Rough";
    rp.stockTop = 0.0;
    rp.stockBottom = -12.0;
    rp.zOffset = -maxHeight;
    rp.allowance = 0.3;
    rp.safeZ = kSafeZ;
    rp.strategy = toolpath::Rough3dStrategy::ZLevel;
    rp.clearing = toolpath::Rough3dClearing::Raster;
    job.roughParams = rp;
    toolpath::Rough3dTool rough = rough3dTool("end-mill-6mm");
    rough.cuttingData = rough.tool.defaultCuttingData;
    rough.cuttingData->passDepth = 2.0;
    rough.cuttingData->stepover = 3.0;
    const toolpath::Rough3dResult rr = toolpath::rough3dToolpath(job.model, {rough}, rp);
    for (const auto& d : rr.diagnostics) {
        INFO(d.message);
        CHECK(d.message.empty());
    }
    REQUIRE(rr.toolpaths.size() == 1);

    toolpath::Finish3dParams fp;
    fp.name = "Finish";
    fp.stockTop = rp.stockTop;
    fp.stockBottom = rp.stockBottom;
    fp.zOffset = rp.zOffset;
    fp.safeZ = kSafeZ;
    fp.strategy = toolpath::Finish3dStrategy::Raster;
    fp.tolerance = 0.1;
    toolpath::Rough3dTool finish = rough3dTool("ball-nose-3mm");
    finish.cuttingData = finish.tool.defaultCuttingData;
    finish.cuttingData->stepover = 0.8;
    const toolpath::Finish3dResult fr = toolpath::finish3dToolpath(job.model, {finish}, fp);
    for (const auto& d : fr.diagnostics) {
        INFO(d.message);
        CHECK(d.message.empty());
    }
    REQUIRE(fr.toolpaths.size() == 1);
    job.finishParams = fp;
    job.toolpaths = {rr.toolpaths[0], fr.toolpaths[0]};
    job.tools = {rough, finish};
    return job;
}

post::PostContext reliefContext() {
    post::PostContext ctx;
    ctx.fileName = "relief";
    ctx.fileExt = "nc";
    ctx.safeZ = kSafeZ;
    ctx.home = {0, 0, kSafeZ};
    ctx.materialMin = {0, 0, -12.0};
    ctx.materialMax = {20, 15, 0};
    return ctx;
}

std::string reliefGcode(const ReliefJob& job) {
    return runPost(shippedPost("grbl_mm"), job.toolpaths, reliefContext());
}

}  // namespace

TEST_CASE("e2e 3D: STL, raster, rough and finish, simulate, Grbl", "[e2e][relief]") {
    const ReliefJob job = buildRelief();
    for (const toolpath::Toolpath& tp : job.toolpaths) {
        CHECK(tp.moveCount() > 40);
    }
    const std::string gcode = reliefGcode(job);
    checkProgram(gcode, kSafeZ, -12.0, 1);
    checkProgramMatchesMoves(gcode, job.toolpaths);
    CHECK(gcode == readFile(goldenPath("relief_grbl.nc")));
    CHECK(gcode == reliefGcode(job));

    // Simulate on the model's grid (cell centres coincide with the model nodes).
    const relief::HeightField& m = job.model;
    const double cs = m.cellSize();
    simulate::Stock st;
    st.width = (static_cast<double>(m.width()) - 0.25) * cs;
    st.height = (static_cast<double>(m.height()) - 0.25) * cs;
    st.thickness = 12.0;
    st.cellSize = cs;
    st.origin = {m.origin().x - 0.5 * cs, m.origin().y - 0.5 * cs};
    st.topZ = 0.0;
    st.trackPassIds = false;
    std::vector<std::pair<toolpath::Toolpath, simulate::ToolShape>> runs;
    for (std::size_t i = 0; i < job.toolpaths.size(); ++i) {
        runs.emplace_back(job.toolpaths[i], shapeOf(job.tools[i].tool));
    }
    simulate::Simulator sim = runSim(st, runs);
    CHECK(sim.diagnostics().empty());
    const simulate::Heightfield& hf = sim.heightfield();
    double worstGouge = -1e9, worstLeft = -1e9;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            const float mv = m.at(i, j);
            if (std::isnan(mv)) {
                continue;
            }
            const double target = std::max(static_cast<double>(mv) + job.roughParams.zOffset, -12.0);
            const double got = hf.height(i, j);
            worstGouge = std::max(worstGouge, target - got);  // > 0: cut below the model
            // Material left above it, away from the border: the last raster row stops up to one stepover short
            // of the edge, so the border band (here 1.25 mm) can keep a thicker scallop by design.
            const bool border = i < 3 || j < 3 || i + 3 >= m.width() || j + 3 >= m.height();
            if (!border) {
                worstLeft = std::max(worstLeft, got - target);
            }
        }
    }
    CHECK(worstGouge < 1e-3);  // never below the model
    CHECK(worstLeft < 0.25);   // the 0.8 mm raster with a 3 mm ball leaves only a small scallop
    CHECK(worstLeft > 0.0);    // ... which is there (the model is not simply cut away)
}

// ---------------------------------------------------------------------------------------------
// Scenario 3: rotary.
// ---------------------------------------------------------------------------------------------

namespace {

constexpr double kRotLength = 30.0;
constexpr double kRotRadius = 10.0;

/// A solid of revolution about the X axis: radius 5.5 + 3 sin^2(pi x / L) with a 1 mm groove at x = 15.
io::TriangleMesh vaseMesh() {
    io::TriangleMesh m;
    const int rings = 61, around = 48;
    for (int i = 0; i < rings; ++i) {
        const double x = kRotLength * i / (rings - 1);
        const double s = std::sin(kPi * x / kRotLength);
        double r = 5.5 + 3.0 * s * s;
        r -= 1.0 * std::exp(-(x - 15.0) * (x - 15.0) / 4.0);
        for (int k = 0; k < around; ++k) {
            const double a = 2.0 * kPi * k / around;
            m.vertices.push_back({x, r * std::sin(a), r * std::cos(a)});
        }
    }
    for (int i = 0; i + 1 < rings; ++i) {
        for (int k = 0; k < around; ++k) {
            const auto a = static_cast<std::uint32_t>(i * around + k);
            const auto b = static_cast<std::uint32_t>(i * around + (k + 1) % around);
            const std::uint32_t c = a + around, d = b + around;
            m.triangles.push_back({a, b, d});
            m.triangles.push_back({a, d, c});
        }
    }
    const auto c0 = static_cast<std::uint32_t>(m.vertices.size());
    m.vertices.push_back({0, 0, 0});
    m.vertices.push_back({kRotLength, 0, 0});
    for (int k = 0; k < around; ++k) {
        const auto a = static_cast<std::uint32_t>(k), b = static_cast<std::uint32_t>((k + 1) % around);
        m.triangles.push_back({c0, b, a});
        const auto e = static_cast<std::uint32_t>((rings - 1) * around);
        m.triangles.push_back({c0 + 1, e + a, e + b});
    }
    return m;
}

struct RotaryJob {
    document::Job job;
    relief::HeightField model;  ///< Radius (mm) over the flat design area (along x circumference).
    toolpath::Rough3dParams roughParams;
    std::vector<toolpath::Toolpath> toolpaths;
    std::vector<toolpath::Rough3dTool> tools;
};

RotaryJob buildRotary() {
    RotaryJob r;
    r.job = document::Job::rotary(kRotLength, 2.0 * kRotRadius, document::Units::Millimetres, document::RotaryAxis::X);
    CHECK(r.job.circumferenceMm() == Approx(2.0 * kPi * kRotRadius));

    const io::TriangleMesh mesh = vaseMesh();
    const double cell = 1.0;
    relief::RotaryRasterOptions o;
    o.map.alongX = true;
    o.map.radius = kRotRadius;
    const auto circ = static_cast<std::size_t>(std::ceil(r.job.circumferenceMm() / cell));
    o.grid = relief::Grid{static_cast<std::size_t>(kRotLength / cell), circ, cell, {0.5 * cell, 0.5 * cell}};
    o.axisPoint = {0.0, 0.0};
    o.meshAlongOrigin = 0.0;
    o.alongStart = 0.0;
    o.zeroRadius = 0.0;
    auto raster = relief::meshToRotaryField(mesh, o);
    REQUIRE(raster.ok());
    CHECK(raster.value().hitCells == o.grid.width * o.grid.height);
    CHECK(raster.value().maxRadius > 8.0);
    CHECK(raster.value().maxRadius < 8.6);
    r.model = raster.value().field;

    // Machine Z has the cylinder axis at 0: the model radius is the surface Z, the stock surface is at Z = the radius.
    toolpath::Rough3dParams rp;
    rp.name = "Rough";
    rp.stockTop = r.job.rotarySurfaceZ();
    rp.stockBottom = r.job.rotarySurfaceZ() - r.job.radiusMm();
    rp.zOffset = 0.0;
    rp.allowance = 0.3;
    rp.safeZ = rp.stockTop + kSafeZ;
    rp.strategy = toolpath::Rough3dStrategy::ZLevel;
    rp.clearing = toolpath::Rough3dClearing::Raster;
    r.roughParams = rp;
    toolpath::Rough3dTool rough = rough3dTool("end-mill-6mm");
    rough.cuttingData = rough.tool.defaultCuttingData;
    rough.cuttingData->passDepth = 2.0;
    rough.cuttingData->stepover = 3.0;
    const toolpath::Rough3dResult rr = toolpath::rough3dToolpath(r.model, {rough}, rp);
    for (const auto& d : rr.diagnostics) {
        INFO(d.message);
        CHECK(d.message.empty());
    }
    REQUIRE(rr.toolpaths.size() == 1);

    toolpath::Finish3dParams fp;
    fp.name = "Finish";
    fp.stockTop = rp.stockTop;
    fp.stockBottom = rp.stockBottom;
    fp.zOffset = 0.0;
    fp.safeZ = rp.safeZ;
    fp.strategy = toolpath::Finish3dStrategy::Raster;
    fp.rasterAngle = 0.0;
    fp.tolerance = 0.1;
    toolpath::Rough3dTool finish = rough3dTool("ball-nose-3mm");
    finish.cuttingData = finish.tool.defaultCuttingData;
    finish.cuttingData->stepover = 1.0;
    const toolpath::Finish3dResult fr = toolpath::finish3dToolpath(r.model, {finish}, fp);
    for (const auto& d : fr.diagnostics) {
        INFO(d.message);
        CHECK(d.message.empty());
    }
    REQUIRE(fr.toolpaths.size() == 1);
    r.toolpaths = {rr.toolpaths[0], fr.toolpaths[0]};
    r.tools = {rough, finish};
    return r;
}

post::PostProcessor rotaryPost() {
    // The shipped Grbl post with the circumference axis written as A: the circumference word is replaced by the
    // rotary axis word in the move blocks.
    std::string text;
    {
        const std::string src = readFile(std::string(CARVETTO_POSTS_DIR) + "/grbl_mm.pp");
        text = "ROTARY_WRAP_Y = \"A\"\nVAR A_POSITION = [A|C| A|1.3]\n" + src;
    }
    for (const char* block : {"G0[X][Y][Z]", "G1[X][Y][Z][F]"}) {
        const std::string from = std::string("\"") + block + "\"";
        std::string to = from;
        to.insert(to.find("[Z]") + 3, "[A]");
        for (std::size_t at = text.find(from); at != std::string::npos; at = text.find(from, at + to.size())) {
            text.replace(at, from.size(), to);
        }
    }
    auto r = post::PostProcessor::parse(text);
    if (!r.ok()) {
        FAIL("rotary post: " << r.error().describe());
    }
    return r.value();
}

post::PostContext rotaryContext(const RotaryJob& r) {
    post::PostContext ctx;
    ctx.fileName = "rotary";
    ctx.fileExt = "nc";
    ctx.safeZ = r.roughParams.safeZ;
    ctx.home = {0, 0, ctx.safeZ};
    ctx.rotaryDiameter = r.job.diameterMm;
    ctx.rotaryAlong = 'X';
    ctx.rotaryFlip = r.job.flipDesign;
    ctx.rotaryAxisZ = r.job.rotarySurfaceZ() - r.job.radiusMm();
    return ctx;
}

std::string rotaryGcode(const RotaryJob& r) {
    return runPost(rotaryPost(), r.toolpaths, rotaryContext(r));
}

}  // namespace

TEST_CASE("e2e rotary: mesh on a cylinder, rough and finish, simulate, wrapped Grbl", "[e2e][rotary]") {
    const RotaryJob job = buildRotary();
    for (const toolpath::Toolpath& tp : job.toolpaths) {
        CHECK(tp.moveCount() > 40);
    }
    const std::string gcode = rotaryGcode(job);
    checkProgram(gcode, job.roughParams.safeZ, 0.0, 1);
    // The circumference axis never appears; the rotary axis carries it, as the angle of the flat distance.
    // (The footer's home move `G0 X.. Y..` comes after the last M5 and is the post's own.)
    CHECK(gcode.substr(0, gcode.rfind("M5")).find(" Y") == std::string::npos);
    checkProgramMatchesMoves(gcode, job.toolpaths, kRotRadius);
    double maxA = 0.0;
    std::vector<Pos> lines = motionOf(gcode);
    lines.pop_back();  // the home move
    std::size_t withA = 0;
    for (const Pos& p : lines) {
        if (p.hasA) {
            ++withA;
            maxA = std::max(maxA, std::fabs(p.a));
        }
    }
    CHECK(withA > 100);
    // The design area spans the circumference once (rounded up to whole cells): about one turn.
    CHECK(maxA > 330.0);
    CHECK(maxA < 380.0);
    CHECK(gcode == readFile(goldenPath("rotary_grbl.nc")));
    CHECK(gcode == rotaryGcode(job));

    // Simulate in the flat design space (X along, Y round, Z machine height with the axis at 0).
    const relief::HeightField& m = job.model;
    const double cs = m.cellSize();
    simulate::Stock st;
    st.width = (static_cast<double>(m.width()) - 0.25) * cs;
    st.height = (static_cast<double>(m.height()) - 0.25) * cs;
    st.thickness = job.roughParams.stockTop - job.roughParams.stockBottom;
    st.cellSize = cs;
    st.origin = {m.origin().x - 0.5 * cs, m.origin().y - 0.5 * cs};
    st.topZ = job.roughParams.stockTop;
    st.trackPassIds = false;
    std::vector<std::pair<toolpath::Toolpath, simulate::ToolShape>> runs;
    for (std::size_t i = 0; i < job.toolpaths.size(); ++i) {
        runs.emplace_back(job.toolpaths[i], shapeOf(job.tools[i].tool));
    }
    simulate::Simulator sim = runSim(st, runs);
    CHECK(sim.diagnostics().empty());
    const simulate::Heightfield& hf = sim.heightfield();
    double worstGouge = -1e9, worstLeft = -1e9;
    for (std::size_t j = 0; j < m.height(); ++j) {
        for (std::size_t i = 0; i < m.width(); ++i) {
            const double target = static_cast<double>(m.at(i, j));
            const double got = hf.height(i, j);
            worstGouge = std::max(worstGouge, target - got);
            worstLeft = std::max(worstLeft, got - target);
        }
    }
    CHECK(worstGouge < 1e-3);
    CHECK(worstLeft < 0.5);
    CHECK(worstLeft > 0.0);
}

// Hidden (tag [.regen], never run by ctest): rewrites the goldens. Run via tools/regen_golden.py and hand-review
// the diff against posts/README.md and core/tests/golden/e2e/README.md before committing.
TEST_CASE("Regenerate the end-to-end goldens", "[.regen]") {
    writeFile(goldenPath("sign_grbl.nc"), signGcode(buildSign()));
    writeFile(goldenPath("relief_grbl.nc"), reliefGcode(buildRelief()));
    writeFile(goldenPath("rotary_grbl.nc"), rotaryGcode(buildRotary()));
}
