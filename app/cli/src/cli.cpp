#include "cli.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <ios>
#include <iterator>
#include <iomanip>
#include <map>
#include <memory>
#include <new>
#include <optional>
#include <span>
#include <sstream>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/document/layer.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/io/dxf.hpp"
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "carvetto/toolpath/v_carve.hpp"
#include "carvetto/document/toolpath_record.hpp"
#include "params.hpp"

#ifndef CARVETTO_VERSION_STRING
#define CARVETTO_VERSION_STRING "0.0.0"
#endif

namespace carvetto::cli {

namespace {

constexpr const char* kProgram = "carvetto-cli";

/// A wrong command line: reported with a pointer to the help and exit code 2.
struct UsageError {
    std::string message;
    std::string command;  // empty = top level
};

// ------------------------------------------------------------------------------------- help

constexpr std::string_view kImportHelp = R"(Usage: carvetto-cli import <in.dxf> <out.carv>

Reads a DXF drawing (R12 to 2018, ASCII or binary) and writes a new .carv document:
one layer per DXF layer (name, colour, visibility), one vector object per contour, in
millimetres ($INSUNITS / $MEASUREMENT honoured; unitless files are read as mm). Blocks are
flattened. Text entities are not imported yet: they are skipped with a warning. The job is
the default single-sided job (300 x 300 x 18 mm, Z zero on the material surface).
)";

constexpr std::string_view kToolpathHelp = R"(Usage: carvetto-cli toolpath profile <params.json> <in.carv> <out.carv>
       carvetto-cli toolpath pocket  <params.json> <in.carv> <out.carv>
       carvetto-cli toolpath drill   <params.json> <in.carv> <out.carv>
       carvetto-cli toolpath engrave <params.json> <in.carv> <out.carv>
       carvetto-cli toolpath v_carve  <params.json> <in.carv> <out.carv>

Computes a 2D profile, pocket, drilling, quick engrave or V-carve toolpath for the vectors of <in.carv> (all
of them, or those on the layers named in the params file) and writes <out.carv> (which may be <in.carv>)
with the toolpath added after the ones already there (a V-carve with clearance tools adds one
toolpath per clearance tool, then the carving tool's, in machining order). Each toolpath is stored as a toolpath record
of the document: its name, the params it came from, the layers it machines, the tool, and
its computed moves. Diagnostics (e.g. tabs that could not be placed) are printed as warnings;
when no toolpath can be produced at all, the command fails and nothing is written.

)";

constexpr std::string_view kPostHelp = R"(Usage: carvetto-cli post <post.pp> <in.carv> <out.nc>

Runs every toolpath of <in.carv>, in the order they were added, through the
post-processor <post.pp> (Carvetto ships posts in posts/, e.g. posts/grbl_mm.pp) and writes
the program to <out.nc>. [TP_FILENAME] is the file name of <out.nc> without its extension;
[DATE] and [TIME] are left empty so the output is reproducible. Safe Z and the home position
(X0 Y0 at safe Z) use the highest safe Z of the toolpaths; material extents come from the job.
)";

constexpr std::string_view kInfoHelp = R"(Usage: carvetto-cli info <file.carv>

Prints a summary of a .carv document: format version, job setup, layers with their vector
counts, the extent of the vectors and every toolpath (operation, tool, moves, Z range,
length).
)";

std::string commandHelp(std::string_view command) {
    if (command == "import") {
        return std::string(kImportHelp);
    }
    if (command == "toolpath") {
        return std::string(kToolpathHelp) + paramsHelp();
    }
    if (command == "post") {
        return std::string(kPostHelp);
    }
    if (command == "info") {
        return std::string(kInfoHelp);
    }
    return helpText();
}

// ------------------------------------------------------------------------------------- utils

std::string readFile(const std::string& path, const std::string& what) {
    std::ifstream in(std::filesystem::path(path), std::ios::binary);
    if (!in) {
        throw CliError("cannot read " + what + " '" + path + "'");
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    if (in.bad()) {
        throw CliError("cannot read " + what + " '" + path + "'");
    }
    return ss.str();
}

void writeFile(const std::string& path, const std::string& text) {
    std::ofstream out(std::filesystem::path(path), std::ios::binary | std::ios::trunc);
    if (!out) {
        throw CliError("cannot write '" + path + "'");
    }
    out.write(text.data(), static_cast<std::streamsize>(text.size()));
    out.close();
    if (!out) {
        throw CliError("cannot write '" + path + "'");
    }
}

/// A length for display: up to three decimals, trailing zeros dropped.
std::string fmt(double v) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << v;
    std::string s = ss.str();
    while (!s.empty() && s.back() == '0') {
        s.pop_back();
    }
    if (!s.empty() && s.back() == '.') {
        s.pop_back();
    }
    return s == "-0" ? "0" : s;
}

document::LoadedCarv loadDocument(const std::string& path) {
    auto loaded = document::loadCarv(std::filesystem::path(path));
    if (!loaded) {
        throw CliError("cannot load '" + path + "': " + loaded.error().message);
    }
    return std::move(loaded.value());
}

void saveDocument(const document::Document& doc, const std::string& path, const document::CarvExtras& extras) {
    const auto saved = document::saveCarv(doc, std::filesystem::path(path), extras);
    if (!saved) {
        throw CliError("cannot save '" + path + "': " + saved.error().message);
    }
}

void warn(std::ostream& err, const std::string& message) {
    err << kProgram << ": warning: " << message << '\n';
}

// ------------------------------------------------------------------------------------ import

int cmdImport(const std::vector<std::string>& a, std::ostream& out, std::ostream& err) {
    const std::string& inPath = a[0];
    const std::string& outPath = a[1];
    const std::string text = readFile(inPath, "DXF file");
    const auto* data = reinterpret_cast<const std::uint8_t*>(text.data());
    auto imported = io::importDxf(std::span<const std::uint8_t>(data, text.size()));
    if (!imported) {
        throw CliError("cannot import '" + inPath + "': " + imported.error().describe());
    }
    const io::DxfDrawing& dxf = imported.value();
    for (const io::DxfWarning& w : dxf.warnings) {
        warn(err, inPath + ": " + w.describe());
    }
    if (!dxf.texts.empty()) {
        warn(err, inPath + ": " + std::to_string(dxf.texts.size()) +
                      " text item(s) skipped: text import is not supported yet");
    }

    document::Document doc;
    const document::LayerId placeholder = doc.defaultLayer();
    std::vector<document::LayerId> layerIds;
    for (const io::DxfLayer& l : dxf.layers) {
        document::Layer layer;
        layer.name = l.name;
        layer.colour = {static_cast<std::uint8_t>((l.rgb >> 16) & 0xFF), static_cast<std::uint8_t>((l.rgb >> 8) & 0xFF),
                        static_cast<std::uint8_t>(l.rgb & 0xFF)};
        layer.visible = l.visible;
        layerIds.push_back(doc.insertLayer(layer));
    }
    if (!layerIds.empty()) {
        doc.setActiveLayer(layerIds.front());
        doc.takeLayer(placeholder);
    }
    for (const io::DxfContour& c : dxf.contours) {
        doc.insertObject(document::Object::vector(c.contour, layerIds.at(c.layer), doc.activeSheet()));
    }
    if (dxf.contours.empty()) {
        warn(err, inPath + ": the drawing holds no vectors");
    }
    saveDocument(doc, outPath, {});
    out << "imported " << dxf.contours.size() << " vector(s) on " << dxf.layers.size() << " layer(s) from '"
        << inPath << "' into '" << outPath << "'\n";
    return kExitOk;
}

// ---------------------------------------------------------------------------------- toolpath

/// The selection of the vector objects on the named layers (every vector object when `layers` is
/// empty); a name selects every layer of that name.
document::VectorSelection selectionFor(const document::Document& doc, const std::vector<std::string>& layers,
                                       const std::string& docPath) {
    if (layers.empty()) {
        return document::VectorSelection::allVectors();
    }
    document::VectorSelection selection;
    for (const std::string& name : layers) {
        bool found = false;
        for (const document::Layer& l : doc.layers()) {
            if (l.name == name) {
                if (std::find(selection.layers.begin(), selection.layers.end(), l.id) == selection.layers.end()) {
                    selection.layers.push_back(l.id);
                }
                found = true;
            }
        }
        if (!found) {
            std::string names;
            for (const document::Layer& l : doc.layers()) {
                names += names.empty() ? "'" : ", '";
                names += l.name + "'";
            }
            throw CliError("'" + docPath + "' has no layer named '" + name + "' (layers: " + names + ")");
        }
    }
    return selection;
}

/// Contours of the selected vector objects, in object (creation) order.
std::vector<geometry::Contour> selectedContours(const document::Document& doc,
                                                const document::VectorSelection& selection,
                                                const std::string& docPath, bool someLayers) {
    std::vector<geometry::Contour> contours;
    for (const document::ObjectId id : doc.selectedVectors(selection)) {
        const document::VectorObject* v = doc.findObject(id)->asVector();
        contours.insert(contours.end(), v->contours.begin(), v->contours.end());
    }
    if (contours.empty()) {
        throw CliError("'" + docPath + "' has no vectors to machine" + (someLayers ? " on the chosen layers" : ""));
    }
    return contours;
}

std::string diagnosticsText(const std::vector<toolpath::ToolpathDiagnostic>& diagnostics) {
    std::string s;
    for (const auto& d : diagnostics) {
        s += s.empty() ? "" : "; ";
        s += d.message;
    }
    return s.empty() ? "no reason given" : s;
}

int cmdToolpath(const std::vector<std::string>& a, std::ostream& out, std::ostream& err) {
    const std::string& op = a[0];
    const std::string& paramsPath = a[1];
    const std::string& inPath = a[2];
    const std::string& outPath = a[3];

    const std::string paramsText = readFile(paramsPath, "params file");
    std::optional<ProfileJob> profile;
    std::optional<PocketJob> pocket;
    std::optional<DrillJob> drill;
    std::optional<EngraveJob> engrave;
    std::optional<VCarvingJob> vCarving;
    try {
        if (op == "profile") {
            profile = parseProfileParams(paramsText);
        } else if (op == "drill") {
            drill = parseDrillParams(paramsText);
        } else if (op == "engrave") {
            engrave = parseEngraveParams(paramsText);
        } else if (op == "v_carve") {
            vCarving = parseVCarvingParams(paramsText);
        } else {
            pocket = parsePocketParams(paramsText);
        }
    } catch (const CliError& e) {
        throw CliError(paramsPath + ": " + e.what());
    }
    const CommonParams& common = profile   ? profile->common
                                 : drill   ? drill->common
                                 : engrave ? engrave->common
                                 : vCarving  ? vCarving->common
                                           : pocket->common;

    document::LoadedCarv loaded = loadDocument(inPath);
    const document::VectorSelection selection = selectionFor(loaded.document, common.layers, inPath);
    const std::vector<geometry::Contour> contours =
        selectedContours(loaded.document, selection, inPath, !common.layers.empty());
    const ResolvedTool tool = resolveTool(common.tool, std::filesystem::path(paramsPath).parent_path());
    for (const std::string& w : tool.warnings) {
        warn(err, w);
    }

    std::vector<toolpath::Toolpath> results;
    std::vector<document::ToolSource> sources;  // per result; empty = the params file's tool
    std::vector<toolpath::ToolpathDiagnostic> diagnostics;
    document::ToolpathParams recordParams;
    if (profile) {
        profile->params.cuttingData = tool.cuttingData;
        toolpath::ProfileResult r = toolpath::profileToolpath(contours, tool.tool, profile->params);
        diagnostics = r.diagnostics;
        if (r.ok()) {
            results.push_back(std::move(r.toolpath));
        }
        profile->params.cuttingData.reset();  // tool-database input, not part of the record
        recordParams = profile->params;
    } else if (drill) {
        toolpath::DrillResult r = toolpath::drillToolpath(contours, tool.tool, tool.cuttingData, drill->params);
        diagnostics = r.diagnostics;
        if (r.ok()) {
            results.push_back(std::move(r.toolpath));
        }
        recordParams = drill->params;
    } else if (engrave) {
        toolpath::EngraveResult r = toolpath::engraveToolpath(contours, tool.tool, tool.cuttingData, engrave->params);
        diagnostics = r.diagnostics;
        if (r.ok()) {
            results.push_back(std::move(r.toolpath));
        }
        recordParams = engrave->params;
    } else if (vCarving) {
        std::vector<toolpath::PocketTool> clearance;
        for (const ToolChoice& c : vCarving->clearanceTools) {
            const ResolvedTool ct = resolveTool(c, std::filesystem::path(paramsPath).parent_path());
            for (const std::string& w : ct.warnings) {
                warn(err, w);
            }
            clearance.push_back(toolpath::PocketTool{ct.tool, ct.cuttingData});
        }
        toolpath::VCarvingResult r =
            toolpath::vCarvingToolpath(contours, tool.tool, tool.cuttingData, clearance, vCarving->params);
        diagnostics = r.diagnostics;
        if (r.ok()) {
            for (std::size_t i = 0; i < r.clearance.size(); ++i) {
                if (!r.clearance[i].isEmpty()) {
                    const ToolChoice& c = vCarving->clearanceTools[i];
                    results.push_back(std::move(r.clearance[i]));
                    sources.push_back({c.id, c.machine, c.material});
                }
            }
            results.push_back(std::move(r.toolpath));
            sources.push_back({common.tool.id, common.tool.machine, common.tool.material});
        }
        recordParams = vCarving->params;
    } else {
        toolpath::PocketResult r =
            toolpath::pocketToolpath(contours, {toolpath::PocketTool{tool.tool, tool.cuttingData}}, pocket->params);
        diagnostics = r.diagnostics;
        for (toolpath::Toolpath& tp : r.toolpaths) {
            if (!tp.isEmpty()) {
                results.push_back(std::move(tp));
            }
        }
        recordParams = pocket->params;
    }
    if (results.empty()) {
        throw CliError("no " + op + " toolpath was produced: " + diagnosticsText(diagnostics));
    }
    for (const auto& d : diagnostics) {
        warn(err, d.message);
    }

    document::Document& doc = loaded.document;
    std::string added;  // reported only once the file is written
    for (std::size_t i = 0; i < results.size(); ++i) {
        toolpath::Toolpath& tp = results[i];
        document::ToolpathRecord record;
        record.name = tp.name;
        record.params = recordParams;
        record.selection = selection;
        record.tool = tp.tool;
        record.toolSource = i < sources.size() ? sources[i]
                                               : document::ToolSource{common.tool.id, common.tool.machine,
                                                                      common.tool.material};
        added += "added " + op + " toolpath '" + tp.name + "' (" + std::to_string(tp.moveCount()) + " moves, tool '" +
                 tp.tool.name + "') as toolpath " + std::to_string(doc.toolpaths().size() + 1) + '\n';
        record.moves = document::shareMoves(std::move(tp.moves));
        if (!doc.execute(std::make_unique<document::AddToolpathCommand>(std::move(record)))) {
            throw CliError("'" + inPath + "' has no ids left for another toolpath");
        }
    }
    saveDocument(doc, outPath, loaded.extras);
    out << added << "wrote '" << outPath << "'\n";
    return kExitOk;
}

// -------------------------------------------------------------------------------------- post

int cmdPost(const std::vector<std::string>& a, std::ostream& out, std::ostream& err) {
    const std::string& ppPath = a[0];
    const std::string& inPath = a[1];
    const std::string& outPath = a[2];

    const std::string ppText = readFile(ppPath, "post-processor");
    auto pp = post::PostProcessor::parse(ppText);
    if (!pp) {
        throw CliError("post-processor '" + ppPath + "': " + pp.error().describe());
    }
    for (const post::Diagnostic& d : pp.value().warnings()) {
        warn(err, ppPath + ": " + d.describe());
    }

    const document::LoadedCarv loaded = loadDocument(inPath);
    const std::vector<document::ToolpathRecord>& records = loaded.document.toolpaths();
    if (records.empty()) {
        throw CliError("'" + inPath + "' holds no toolpaths (add some with 'carvetto-cli toolpath')");
    }
    std::vector<toolpath::Toolpath> toolpaths;
    double safeZ = records.front().safeZ();
    for (const document::ToolpathRecord& r : records) {
        if (!r.hasMoves()) {
            throw CliError("toolpath '" + r.name + "' in '" + inPath +
                           "' has no computed moves (the file holds only its definition)");
        }
        safeZ = std::max(safeZ, r.safeZ());
        toolpaths.push_back(r.toolpath());
    }

    const std::filesystem::path outFs(outPath);
    post::PostContext ctx;
    ctx.fileName = outFs.stem().string();
    ctx.fileExt = outFs.has_extension() ? outFs.extension().string().substr(1) : std::string();
    ctx.fileDir = outFs.parent_path().generic_string();
    ctx.pathName = outFs.generic_string();
    ctx.safeZ = safeZ;
    ctx.home = {0.0, 0.0, safeZ};
    const document::Job& job = loaded.document.job();
    const geometry::Box2 material = job.materialBoundsMm();
    const bool bedZero = job.zZero == document::ZZero::MachineBed;
    ctx.materialMin = {material.min.x, material.min.y, bedZero ? 0.0 : -job.sizeMm.z};
    ctx.materialMax = {material.max.x, material.max.y, bedZero ? job.sizeMm.z : 0.0};
    ctx.rotaryDiameter = job.diameterMm;

    auto program = pp.value().run(toolpaths, ctx);
    if (!program) {
        throw CliError("post-processor '" + ppPath + "' failed: " + program.error().describe());
    }
    for (const post::Diagnostic& d : program.value().warnings) {
        warn(err, ppPath + ": " + d.describe());
    }
    writeFile(outPath, program.value().text);
    out << "wrote " << toolpaths.size() << " toolpath(s) to '" << outPath << "'\n";
    return kExitOk;
}

// -------------------------------------------------------------------------------------- info

const char* operationName(document::ToolpathOperation op) {
    switch (op) {
        case document::ToolpathOperation::Profile: return "profile";
        case document::ToolpathOperation::Pocket: return "pocket";
        case document::ToolpathOperation::Drill: return "drill";
        case document::ToolpathOperation::Engrave: return "engrave";
        case document::ToolpathOperation::VCarving: return "v_carve";
        case document::ToolpathOperation::Chamfer: return "chamfer";
        case document::ToolpathOperation::ThreadMill: return "thread_mill";
        case document::ToolpathOperation::Inlay: return "inlay";
    }
    return "?";
}

const char* jobTypeName(document::JobType t) {
    switch (t) {
        case document::JobType::SingleSided: return "single-sided";
        case document::JobType::DoubleSided: return "double-sided";
        case document::JobType::Rotary: return "rotary";
    }
    return "?";
}

int cmdInfo(const std::vector<std::string>& a, std::ostream& out, std::ostream& /*err*/) {
    const std::string& path = a[0];
    const document::LoadedCarv loaded = loadDocument(path);
    const document::Document& doc = loaded.document;
    const document::Job& job = doc.job();

    out << "file: " << path << '\n';
    out << "format: " << loaded.formatMajor << '.' << loaded.formatMinor << " (written by Carvetto "
        << loaded.appVersion << ")\n";
    out << "job: " << jobTypeName(job.type) << ", " << fmt(job.sizeMm.x) << " x " << fmt(job.sizeMm.y) << " x "
        << fmt(job.sizeMm.z) << " mm, display units "
        << (job.displayUnits == document::Units::Inches ? "inches" : "mm") << ", Z zero on the "
        << (job.zZero == document::ZZero::MaterialSurface ? "material surface"
            : job.zZero == document::ZZero::MachineBed ? "machine bed"
                                                         : "cylinder axis")
        << '\n';

    geometry::Box2 extent;
    out << "layers: " << doc.layers().size() << '\n';
    for (const document::Layer& l : doc.layers()) {
        std::size_t objects = 0;
        std::size_t closed = 0;
        std::size_t open = 0;
        for (const document::ObjectId id : doc.objectsOnLayer(l.id)) {
            ++objects;
            if (const document::VectorObject* v = doc.findObject(id)->asVector()) {
                for (const geometry::Contour& c : v->contours) {
                    (c.closed ? closed : open) += 1;
                    extent.expand(c.bounds());
                }
            }
        }
        out << "  '" << l.name << "': " << objects << " object(s), " << closed << " closed and " << open
            << " open vector(s)" << (l.visible ? "" : ", hidden") << '\n';
    }
    if (extent.isEmpty()) {
        out << "vectors: none\n";
    } else {
        out << "vectors extent: X " << fmt(extent.min.x) << " .. " << fmt(extent.max.x) << ", Y " << fmt(extent.min.y)
            << " .. " << fmt(extent.max.y) << " mm\n";
    }

    const std::vector<document::ToolpathRecord>& records = doc.toolpaths();
    out << "toolpaths: " << records.size() << '\n';
    for (std::size_t i = 0; i < records.size(); ++i) {
        const document::ToolpathRecord& r = records[i];
        const toolpath::Toolpath tp = r.toolpath();
        const toolpath::LengthByKind len = tp.lengthByKind();
        out << "  " << i + 1 << ": " << operationName(r.operation())
            << " '" << tp.name << "', tool T" << tp.tool.toolNumber << " '" << tp.tool.name << "' D"
            << fmt(tp.tool.diameter) << " mm, ";
        if (r.hasMoves()) {
            out << tp.moveCount() << " moves";
        } else {
            out << "not computed";
        }
        if (!tp.isEmpty()) {
            out << ", Z " << fmt(*tp.minZ()) << " .. " << fmt(*tp.maxZ()) << " mm, cutting " << fmt(len.feed + len.plunge)
                << " mm, rapid " << fmt(len.rapid + len.retract) << " mm";
        }
        out << '\n';
    }
    const std::size_t others = loaded.extras.memberNames().size();
    if (others > 0) {
        out << "other preserved archive members: " << others << '\n';
    }
    return kExitOk;
}

// ---------------------------------------------------------------------------------- dispatch

struct Command {
    std::string_view name;
    std::size_t argCount;  // positional arguments after the command (toolpath: incl. the operation)
    int (*fn)(const std::vector<std::string>&, std::ostream&, std::ostream&);
};

constexpr Command kCommands[] = {
    {"import", 2, cmdImport},
    {"toolpath", 4, cmdToolpath},
    {"post", 3, cmdPost},
    {"info", 1, cmdInfo},
};

std::string usageLine(std::string_view command) {
    const std::string help = commandHelp(command);
    const std::size_t end = help.find("\n\n");
    return help.substr(0, end);
}

int dispatch(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    if (args.empty()) {
        throw UsageError{"no command given", ""};
    }
    const std::string& first = args[0];
    if (first == "--help" || first == "-h") {
        out << helpText();
        return kExitOk;
    }
    if (first == "help") {
        if (args.size() > 2) {
            throw UsageError{"'help' takes at most one command name", ""};
        }
        if (args.size() == 2) {
            const auto it = std::find_if(std::begin(kCommands), std::end(kCommands),
                                         [&](const Command& c) { return c.name == args[1]; });
            if (it == std::end(kCommands)) {
                throw UsageError{"unknown command '" + args[1] + "'", ""};
            }
            out << commandHelp(args[1]);
        } else {
            out << helpText();
        }
        return kExitOk;
    }
    if (first == "--version") {
        out << kProgram << ' ' << CARVETTO_VERSION_STRING << '\n';
        return kExitOk;
    }
    const auto it = std::find_if(std::begin(kCommands), std::end(kCommands),
                                 [&](const Command& c) { return c.name == first; });
    if (it == std::end(kCommands)) {
        throw UsageError{first.starts_with("-") ? "unknown option '" + first + "'" : "unknown command '" + first + "'",
                         ""};
    }
    std::vector<std::string> rest(args.begin() + 1, args.end());
    for (const std::string& r : rest) {
        if (r == "--help" || r == "-h") {
            out << commandHelp(it->name);
            return kExitOk;
        }
    }
    for (const std::string& r : rest) {
        if (r.size() > 1 && r.starts_with("-")) {
            throw UsageError{"unknown option '" + r + "'", std::string(it->name)};
        }
    }
    if (rest.size() != it->argCount) {
        throw UsageError{"'" + std::string(it->name) + "' takes " + std::to_string(it->argCount) + " argument(s), got " +
                             std::to_string(rest.size()),
                         std::string(it->name)};
    }
    if (it->name == "toolpath" && rest[0] != "profile" && rest[0] != "pocket" && rest[0] != "drill" &&
        rest[0] != "engrave" && rest[0] != "v_carve") {
        throw UsageError{
            "unknown toolpath operation '" + rest[0] + "' (expected profile, pocket, drill, engrave or v_carve)",
            "toolpath"};
    }
    return it->fn(rest, out, err);
}

}  // namespace

std::string helpText() {
    return R"(carvetto-cli - headless Carvetto: DXF import, toolpaths and G-code from the command line

Usage: carvetto-cli <command> [arguments]
       carvetto-cli help <command>      detailed help for one command
       carvetto-cli --help | --version

Commands:
  import <in.dxf> <out.carv>
      Import a DXF drawing into a new .carv document (one layer per DXF layer).
  toolpath profile <params.json> <in.carv> <out.carv>
      Add a 2D profile toolpath (outside/inside/on, tabs, ramps, leads, ...).
  toolpath pocket <params.json> <in.carv> <out.carv>
      Add a pocket toolpath (offset or raster clearing, boundary pass, ramp plunge).
  toolpath drill <params.json> <in.carv> <out.carv>
      Add a drilling toolpath (a hole at the centre of every closed vector, peck, dwell).
  toolpath engrave <params.json> <in.carv> <out.carv>
      Add a quick engrave toolpath (outline or fill at a shallow depth, passes).
  toolpath v_carve <params.json> <in.carv> <out.carv>
      Add a V-carve toolpath (depth from the medial axis, flat depth, clearance tools).
  post <post.pp> <in.carv> <out.nc>
      Write every toolpath through a post-processor (e.g. posts/grbl_mm.pp).
  info <file.carv>
      Summarise a .carv document: job, layers, vectors and toolpaths.

params.json holds the toolpath settings (snake_case keys, mm and degrees); see
'carvetto-cli help toolpath' for every key.

Example (DXF to Grbl G-code):
  carvetto-cli import part.dxf part.carv
  carvetto-cli toolpath pocket pocket.json part.carv part.carv
  carvetto-cli toolpath profile profile.json part.carv part.carv
  carvetto-cli post posts/grbl_mm.pp part.carv part.nc

Exit status: 0 on success, 1 when a command fails (unreadable or invalid input, no
toolpath produced), 2 for a wrong command line. Errors and warnings go to stderr.
)";
}

int run(const std::vector<std::string>& args, std::ostream& out, std::ostream& err) {
    try {
        return dispatch(args, out, err);
    } catch (const UsageError& e) {
        err << kProgram << ": error: " << e.message << '\n';
        if (!e.command.empty()) {
            err << usageLine(e.command) << '\n';
        }
        err << "Run '" << kProgram << " " << (e.command.empty() ? "--help" : "help " + e.command)
            << "' for usage.\n";
        return kExitUsage;
    } catch (const CliError& e) {
        err << kProgram << ": error: " << e.what() << '\n';
        return kExitFailure;
    } catch (const std::bad_alloc&) {
        err << kProgram << ": error: out of memory\n";
        return kExitFailure;
    } catch (const std::exception& e) {
        err << kProgram << ": error: " << e.what() << '\n';
        return kExitFailure;
    }
}

}  // namespace carvetto::cli
