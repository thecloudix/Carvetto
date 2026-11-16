#include "carvetto/viewmodel/toolpath_calc.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <exception>
#include <new>
#include <set>
#include <span>
#include <string>
#include <type_traits>
#include <utility>
#include <variant>

#include "carvetto/relief/relief.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/finish3d.hpp"
#include "carvetto/toolpath/moulding.hpp"
#include "carvetto/toolpath/project3d.hpp"
#include "carvetto/toolpath/rough3d.hpp"
#include "carvetto/toolpath/chamfer.hpp"
#include "carvetto/toolpath/fluting.hpp"
#include "carvetto/toolpath/laser.hpp"
#include "carvetto/toolpath/thread_mill.hpp"
#include "carvetto/toolpath/inlay.hpp"
#include "carvetto/toolpath/v_carve_inlay.hpp"
#include "carvetto/toolpath/image_engrave.hpp"
#include "carvetto/toolpath/prism.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/v_carve.hpp"

namespace carvetto::viewmodel {

using document::ObjectId;
using document::ToolpathOperation;
using toolpath::ToolpathIssue;

namespace {

template <class... Ts>
struct Overloaded : Ts... {
    using Ts::operator()...;
};
template <class... Ts>
Overloaded(Ts...) -> Overloaded<Ts...>;

void addCuttingData(Fingerprint& fp, const tooling::CuttingData& d) {
    fp.add(d.passDepth);
    fp.add(d.stepover);
    fp.add(d.finalPassStepover);
    fp.add(d.clearancePassStepover);
    fp.add(d.spindleSpeed);
    fp.add(d.feedRate);
    fp.add(d.plungeRate);
    fp.add(d.maxBurnRate);
    fp.add(static_cast<std::uint64_t>(static_cast<std::int64_t>(d.toolNumber)));
}

bool spanFinite(const geometry::Span& span) {
    const auto ok = [](double v) { return std::isfinite(v) && std::fabs(v) <= kMaxToolpathCoordinateMm; };
    return std::visit(
        Overloaded{[&](const geometry::LineSpan& s) { return ok(s.start.x) && ok(s.start.y) && ok(s.end.x) && ok(s.end.y); },
                   [&](const geometry::ArcSpan& s) {
                       return ok(s.centre.x) && ok(s.centre.y) && ok(s.radius) && std::isfinite(s.startAngle) &&
                              std::isfinite(s.endAngle) && std::fabs(s.startAngle) < 1e6 && std::fabs(s.endAngle) < 1e6;
                   },
                   [&](const geometry::BezierSpan& s) {
                       return ok(s.p0.x) && ok(s.p0.y) && ok(s.p1.x) && ok(s.p1.y) && ok(s.p2.x) && ok(s.p2.y) &&
                              ok(s.p3.x) && ok(s.p3.y);
                   }},
        span);
}

bool isInfoIssue(ToolpathIssue issue) {
    return issue == ToolpathIssue::DuplicateHole || issue == ToolpathIssue::MergedWithNeighbour ||
           issue == ToolpathIssue::CrossingVectorsResolved;
}

bool isRefusalIssue(ToolpathIssue issue) {
    switch (issue) {
        case ToolpathIssue::InvalidParameters:
        case ToolpathIssue::NoPasses:
        case ToolpathIssue::NoTool:
        case ToolpathIssue::InvalidTool:
        case ToolpathIssue::WorkLimitExceeded:
        case ToolpathIssue::MedialAxisCapped:
        case ToolpathIssue::ContourRefused:
        case ToolpathIssue::CutterCompRefused:
            return true;
        default:
            return false;
    }
}

std::vector<ToolpathMessage> convertDiagnostics(const std::vector<toolpath::ToolpathDiagnostic>& diagnostics,
                                                bool produced, const std::vector<ObjectId>& owners) {
    std::vector<ToolpathMessage> out;
    out.reserve(diagnostics.size());
    for (const toolpath::ToolpathDiagnostic& d : diagnostics) {
        ToolpathMessage m;
        m.issue = d.issue;
        m.text = d.message;
        m.severity = (!produced || isRefusalIssue(d.issue)) ? MessageSeverity::Error
                     : isInfoIssue(d.issue)                 ? MessageSeverity::Info
                                                            : MessageSeverity::Warning;
        if (d.contour.has_value()) {
            m.contour = *d.contour;
            if (*d.contour < owners.size()) {
                m.vector = owners[*d.contour];
            }
        }
        out.push_back(std::move(m));
    }
    return out;
}

std::string joinDiagnostics(const std::vector<toolpath::ToolpathDiagnostic>& diagnostics) {
    std::string s;
    for (const auto& d : diagnostics) {
        s += s.empty() ? "" : "; ";
        s += d.message;
    }
    return s.empty() ? "no reason given" : s;
}

ToolpathMessage note(MessageSeverity severity, std::string text) {
    ToolpathMessage m;
    m.severity = severity;
    m.text = std::move(text);
    return m;
}

constexpr double kMaxWorkLevels = 10000.0;     // as Pocket and Drill
constexpr double kMaxWorkMoves = 20000000.0;   // as Pocket and Drill

/// Segments a circular arc of `sweep` radians and `radius` mm is cut into at the chordal tolerance.
double arcSegments(double radius, double sweep, double chordal) {
    const double c = std::max(chordal, 1e-4);
    const double r = std::max(std::fabs(radius), c);
    const double step = std::acos(1.0 - std::min(1.0, c / r));
    return 1.0 + std::fabs(sweep) / std::max(step, 1e-6);
}

/// The profile has no work limit of its own (the pocket and drilling have: 10000 levels, 20 M moves),
/// so a depth in micrometre steps, an allowance of a million millimetres, an overcut of a million
/// millimetres or a tab every nanometre would make it allocate gigabytes. This refuses what would
/// need more than those limits, by a coarse upper estimate of the moves.
std::string profileWorkProblem(const toolpath::ProfileParams& p, const ResolvedTool& tool,
                               const ResolvedVectors& vectors) {
    double step = p.passes.passDepth;
    if (!(step > 0.0)) {
        step = toolpath::effectiveCuttingData(tool.tool, tool.data).passDepth;
    }
    double levels = 1.0;
    if (p.passes.cutDepth > 0.0 && step > 0.0) {
        levels = std::ceil(p.passes.cutDepth / step) + 2.0;
    }
    if (!(levels <= kMaxWorkLevels)) {
        return "more than " + std::to_string(static_cast<long long>(kMaxWorkLevels)) + " passes (cut depth " +
               std::to_string(p.passes.cutDepth) + " mm in steps of " + std::to_string(step) + " mm)";
    }
    const double chordal = p.tolerance.chordal;
    const double reach = tool.tool.diameter * 0.5 + std::fabs(p.allowance) +
                         (p.separateLastPass ? std::fabs(p.lastPassAllowance) : 0.0);
    const double contours = static_cast<double>(vectors.contours.size());
    double perCut = static_cast<double>(vectors.spanCount) * arcSegments(reach, 1.5707963267948966, chordal);
    if (p.leads.type == toolpath::LeadType::Circular) {
        perCut += contours * 2.0 * arcSegments(p.leads.radius, p.leads.sweep, chordal);
    } else if (p.leads.type == toolpath::LeadType::Straight) {
        perCut += contours * 4.0;
    }
    if (p.leads.type != toolpath::LeadType::None) {
        perCut += contours * std::fabs(p.leads.overcut) / std::max(chordal, 1e-4);
    }
    if (p.tabs.enabled) {
        double length = 0.0;
        for (const ContourRef& c : vectors.contours) {
            length += c.contour->length();
        }
        const double tabs = p.tabs.spacing == toolpath::TabSpacing::ConstantNumber
                                ? contours * static_cast<double>(std::max(p.tabs.count, 0))
                                : length / std::max(p.tabs.distance, 1e-9) + contours;
        perCut += tabs * 8.0;
    }
    if (p.ramp.type == toolpath::RampType::ZigZag && p.ramp.useAngle) {
        const double legs = step / std::max(p.ramp.distance * std::tan(std::max(p.ramp.angle, 1e-9)), 1e-12);
        perCut += contours * 2.0 * legs;
    }
    const double moves = levels * perCut * 2.0;
    if (!(moves <= kMaxWorkMoves)) {
        return "the toolpath would have more than " + std::to_string(static_cast<long long>(kMaxWorkMoves)) +
               " moves (too many passes, vectors, tabs or too large an offset, lead or overcut)";
    }
    return {};
}

/// Quick Engrave outlines cut every vector `passes` times.
std::string engraveWorkProblem(const toolpath::EngraveParams& p, const ResolvedVectors& vectors) {
    if (p.strategy == toolpath::EngraveStrategy::Outline &&
        !(static_cast<double>(std::max(p.passes, 0)) * static_cast<double>(vectors.spanCount) * 2.0 <= kMaxWorkMoves)) {
        return "the toolpath would have more than " + std::to_string(static_cast<long long>(kMaxWorkMoves)) +
               " moves (too many passes or vectors)";
    }
    return {};
}

}  // namespace

// --- Definition ----------------------------------------------------------------------------------

const std::string& ToolpathDefinition::name() const {
    return std::visit([](const auto& p) -> const std::string& { return p.name; }, params);
}

void ToolpathDefinition::setName(std::string name) {
    std::visit([&](auto& p) { p.name = std::move(name); }, params);
}

ToolpathDefinition defaultDefinition(ToolpathOperation operation) {
    ToolpathDefinition def;
    switch (operation) {
        case ToolpathOperation::Profile: def.params = toolpath::ProfileParams{}; break;
        case ToolpathOperation::Pocket: def.params = toolpath::PocketParams{}; break;
        case ToolpathOperation::Drill: def.params = toolpath::DrillParams{}; break;
        case ToolpathOperation::Engrave: def.params = toolpath::EngraveParams{}; break;
        case ToolpathOperation::VCarving: def.params = toolpath::VCarvingParams{}; break;
        case ToolpathOperation::Chamfer: def.params = toolpath::ChamferParams{}; break;
        case ToolpathOperation::ThreadMill: def.params = toolpath::ThreadMillParams{}; break;
        case ToolpathOperation::Inlay: def.params = toolpath::InlayParams{}; break;
        case ToolpathOperation::Fluting: def.params = toolpath::FlutingParams{}; break;
        case ToolpathOperation::LaserCutFill: def.params = toolpath::LaserCutFillParams{}; break;
        case ToolpathOperation::Prism: def.params = toolpath::PrismParams{}; break;
        case ToolpathOperation::ImageEngrave: def.params = toolpath::ImageEngraveParams{}; break;
        case ToolpathOperation::VCarveInlay: def.params = toolpath::VCarveInlayParams{}; break;
        case ToolpathOperation::Rough3d: def.params = toolpath::Rough3dParams{}; break;
        case ToolpathOperation::Finish3d: def.params = toolpath::Finish3dParams{}; break;
        case ToolpathOperation::Moulding: def.params = toolpath::MouldingParams{}; break;
        case ToolpathOperation::Project3d: {
            toolpath::Project3dParams p;
            p.name = "Project to Model";  // the core's empty name would be refused as a toolpath name
            def.params = std::move(p);
            break;
        }
    }
    return def;
}

namespace {

/// True when `text` is well-formed UTF-8 without control characters (C0, DEL, C1).
bool cleanUtf8(const std::string& text) {
    std::size_t i = 0;
    while (i < text.size()) {
        const auto lead = static_cast<unsigned char>(text[i]);
        std::size_t extra = 0;
        std::uint32_t cp = 0;
        if (lead < 0x80) {
            cp = lead;
        } else if (lead >= 0xC2 && lead <= 0xDF) {
            extra = 1;
            cp = lead & 0x1FU;
        } else if (lead >= 0xE0 && lead <= 0xEF) {
            extra = 2;
            cp = lead & 0x0FU;
        } else if (lead >= 0xF0 && lead <= 0xF4) {
            extra = 3;
            cp = lead & 0x07U;
        } else {
            return false;
        }
        if (extra > 0 && i + extra >= text.size()) {
            return false;  // cut short
        }
        for (std::size_t k = 1; k <= extra; ++k) {
            const auto next = static_cast<unsigned char>(text[i + k]);
            if ((next & 0xC0U) != 0x80U) {
                return false;
            }
            cp = (cp << 6) | (next & 0x3FU);
        }
        if ((extra == 2 && cp < 0x800) || (extra == 3 && cp < 0x10000) || cp > 0x10FFFF ||
            (cp >= 0xD800 && cp <= 0xDFFF) || cp < 0x20 || cp == 0x7F || (cp >= 0x80 && cp <= 0x9F)) {
            return false;
        }
        i += extra + 1;
    }
    return true;
}

}  // namespace

std::string validateToolpathName(const std::string& name) {
    if (name.empty()) {
        return "the toolpath needs a name";
    }
    if (name.size() > kMaxToolpathNameBytes) {
        return "the name is too long (at most " + std::to_string(kMaxToolpathNameBytes) + " bytes)";
    }
    if (!cleanUtf8(name)) {
        return "the name must be valid text without control characters or line breaks";
    }
    return {};
}

const char* operationName(ToolpathOperation operation) {
    switch (operation) {
        case ToolpathOperation::Profile: return "profile";
        case ToolpathOperation::Pocket: return "pocket";
        case ToolpathOperation::Drill: return "drill";
        case ToolpathOperation::Engrave: return "engrave";
        case ToolpathOperation::VCarving: return "v_carve";
        case ToolpathOperation::Chamfer: return "chamfer";
        case ToolpathOperation::ThreadMill: return "thread_mill";
        case ToolpathOperation::Inlay: return "inlay";
        case ToolpathOperation::Fluting: return "fluting";
        case ToolpathOperation::LaserCutFill: return "laser_cut_fill";
        case ToolpathOperation::Prism: return "prism";
        case ToolpathOperation::ImageEngrave: return "image_engrave";
        case ToolpathOperation::VCarveInlay: return "v_carve_inlay";
        case ToolpathOperation::Rough3d: return "rough_3d";
        case ToolpathOperation::Finish3d: return "finish_3d";
        case ToolpathOperation::Moulding: return "moulding";
        case ToolpathOperation::Project3d: return "project_3d";
    }
    return "?";
}

bool toolAllowedFor(ToolpathOperation operation, tooling::ToolType type, bool clearance) {
    switch (operation) {
        case ToolpathOperation::Profile:
        case ToolpathOperation::Pocket: return true;
        case ToolpathOperation::Drill: return toolpath::drillToolAllowed(type);
        case ToolpathOperation::Engrave: return toolpath::engraveToolAllowed(type);
        case ToolpathOperation::Chamfer: return toolpath::chamferToolAllowed(type);
        case ToolpathOperation::ThreadMill: return toolpath::threadMillToolAllowed(type);
        case ToolpathOperation::Inlay: return toolpath::inlayToolAllowed(type);
        case ToolpathOperation::Fluting: return toolpath::flutingToolAllowed(type);
        case ToolpathOperation::LaserCutFill: return toolpath::laserToolAllowed(type);
        case ToolpathOperation::Prism: return toolpath::prismToolAllowed(type);
        case ToolpathOperation::ImageEngrave: return toolpath::imageEngraveToolAllowed(type);
        case ToolpathOperation::Rough3d:
        case ToolpathOperation::Finish3d:
        case ToolpathOperation::Moulding:
        case ToolpathOperation::Project3d:
            // Every tool with a cutting profile the drop-tool surface understands.
            return type != tooling::ToolType::DiamondDrag && type != tooling::ToolType::Drill &&
                   type != tooling::ToolType::ThreadMill;
        case ToolpathOperation::VCarveInlay:
            return clearance ? toolpath::vCarvingClearanceToolAllowed(type) : toolpath::vCarveInlayToolAllowed(type);
        case ToolpathOperation::VCarving:
            return clearance ? toolpath::vCarvingClearanceToolAllowed(type) : toolpath::vCarvingToolAllowed(type);
    }
    return false;
}

bool hasClearanceTools(ToolpathOperation operation) {
    return operation == ToolpathOperation::VCarving || operation == ToolpathOperation::VCarveInlay ||
           hasRestTools(operation);
}

bool hasRestTools(ToolpathOperation operation) {
    return operation == ToolpathOperation::Rough3d || operation == ToolpathOperation::Finish3d ||
           operation == ToolpathOperation::Moulding;
}

bool usesModel(ToolpathOperation operation) {
    return operation == ToolpathOperation::Rough3d || operation == ToolpathOperation::Finish3d ||
           operation == ToolpathOperation::Project3d;
}

namespace {
/// True when a 3D toolpath's boundary is the selected vectors.
bool boundaryUsesVectors(const document::ToolpathParams& params) {
    if (const auto* r = std::get_if<toolpath::Rough3dParams>(&params)) {
        return r->boundary == toolpath::Rough3dBoundary::Vectors;
    }
    if (const auto* f = std::get_if<toolpath::Finish3dParams>(&params)) {
        return f->boundary == toolpath::Rough3dBoundary::Vectors;
    }
    return false;
}

std::uint64_t mixWord(std::uint64_t h, std::uint64_t w) {
    h ^= w + 0x9E3779B97F4A7C15ULL + (h << 6) + (h >> 2);
    return h * 1099511628211ULL;
}
}  // namespace

std::uint64_t modelFingerprint(const document::Document& doc, document::SheetId sheet) {
    const std::vector<relief::Level> levels = doc.reliefModel(sheet);
    Fingerprint fp;
    fp.add(static_cast<std::uint64_t>(levels.size()));
    for (const relief::Level& level : levels) {
        fp.add(static_cast<std::uint64_t>(level.mode));
        fp.add(level.baseHeight);
        fp.add(static_cast<std::uint64_t>(level.visible));
        fp.add(static_cast<std::uint64_t>(level.effects.mirror));
        fp.add(level.effects.mirrorCenter.x);
        fp.add(level.effects.mirrorCenter.y);
        for (const auto& polygon : level.effects.clip.polygons) {
            fp.add(static_cast<std::uint64_t>(polygon.size()));
            for (const relief::Point& p : polygon) {
                fp.add(p.x);
                fp.add(p.y);
            }
        }
        fp.add(static_cast<std::uint64_t>(level.components.size()));
        for (const relief::Component& c : level.components) {
            fp.add(static_cast<std::uint64_t>(c.mode));
            fp.add(c.shapeHeightPercent);
            fp.add(c.baseHeight);
            fp.add(static_cast<std::uint64_t>(c.visible));
            fp.add(static_cast<std::uint64_t>(c.fade.enabled));
            fp.add(c.fade.anchor.x);
            fp.add(c.fade.anchor.y);
            fp.add(c.fade.second.x);
            fp.add(c.fade.second.y);
            fp.add(c.fade.percent);
            fp.add(static_cast<std::uint64_t>(c.tilt.enabled));
            fp.add(c.tilt.pivot.x);
            fp.add(c.tilt.pivot.y);
            fp.add(c.tilt.raise.x);
            fp.add(c.tilt.raise.y);
            fp.add(c.tilt.angleRad);
            const relief::Grid& g = c.field.grid();
            fp.add(static_cast<std::uint64_t>(g.width));
            fp.add(static_cast<std::uint64_t>(g.height));
            fp.add(g.cellSize);
            fp.add(g.origin.x);
            fp.add(g.origin.y);
            std::uint64_t h = 1469598103934665603ULL;  // one multiply per value: the model can be large
            for (const float v : c.field.data()) {
                h = mixWord(h, static_cast<std::uint64_t>(std::bit_cast<std::uint32_t>(v)));
            }
            fp.add(h);
        }
    }
    return fp.value();
}

std::string modelProblem(const document::Document& doc) {
    const std::vector<relief::Level> levels = doc.reliefModel(doc.activeSheet());
    if (levels.empty()) {
        return "this sheet has no relief model: create or import a component first";
    }
    relief::Grid grid;
    if (const relief::Status s = relief::unionGrid(levels, grid); s != relief::Status::Ok) {
        return relief::statusMessage(s);
    }
    return {};
}

void attachModel(ToolpathJob& job, const document::Document& doc) {
    if (!usesModel(job.definition.operation())) {
        return;
    }
    const auto refuse = [&](std::string why) {
        if (job.refusal.empty()) {
            job.refusal = std::move(why);
        }
    };
    const std::vector<relief::Level> levels = doc.reliefModel(doc.activeSheet());
    if (levels.empty()) {
        refuse("this sheet has no relief model: create or import a component first");
        return;
    }
    relief::Grid grid;
    if (const relief::Status s = relief::unionGrid(levels, grid); s != relief::Status::Ok) {
        refuse(std::string("the relief model cannot be machined: ") + relief::statusMessage(s));
        return;
    }
    auto field = std::make_shared<relief::HeightField>();
    if (const relief::Status s = relief::compose(levels, grid, *field); s != relief::Status::Ok) {
        refuse(std::string("the relief model cannot be composed: ") + relief::statusMessage(s));
        return;
    }
    job.model = std::move(field);
}

void attachSource(ToolpathJob& job, const document::Document& doc) {
    const auto* params = std::get_if<toolpath::Project3dParams>(&job.definition.params);
    if (params == nullptr) {
        return;
    }
    const document::ToolpathRecord* rec =
        params->sourceToolpath == 0 ? nullptr : doc.findToolpath(document::ToolpathId{params->sourceToolpath});
    if (rec == nullptr) {
        if (job.refusal.empty()) {
            job.refusal = "choose the toolpath to project: the source toolpath does not exist";
        }
        return;
    }
    if (!rec->hasMoves() || rec->moves->empty()) {
        if (job.refusal.empty()) {
            job.refusal = "the source toolpath '" + rec->name + "' has no moves to project";
        }
        return;
    }
    if (rec->operation() == document::ToolpathOperation::Project3d && rec->id == job.target) {
        if (job.refusal.empty()) {
            job.refusal = "a toolpath cannot be projected onto the model from itself";
        }
        return;
    }
    ProjectSource src;
    src.id = rec->id;
    src.name = rec->name;
    src.tool = rec->tool;
    src.moves = rec->moves;
    job.source = std::move(src);
}

ModelDefaults modelDefaults(const document::Document& doc) {
    ModelDefaults out;
    const document::Job& job = doc.job();
    if (job.zZero == document::ZZero::MachineBed) {
        out.stockTop = job.sizeMm.z;
        out.stockBottom = 0.0;
    } else {
        out.stockTop = 0.0;
        out.stockBottom = -job.sizeMm.z;
    }
    out.safeZ = out.stockTop + 5.0;
    const std::vector<relief::Level> levels = doc.reliefModel(doc.activeSheet());
    relief::Grid grid;
    relief::HeightField field;
    if (levels.empty() || relief::unionGrid(levels, grid) != relief::Status::Ok) {
        return out;
    }
    // A coarse composition is enough to find the top of the model.
    const std::size_t longer = std::max(grid.width, grid.height);
    if (longer > 256) {
        const std::size_t stride = (longer + 255) / 256;
        grid.cellSize *= static_cast<double>(stride);
        grid.width = (grid.width - 1) / stride + 1;
        grid.height = (grid.height - 1) / stride + 1;
    }
    float lo = 0.0f, hi = 0.0f;
    if (relief::compose(levels, grid, field) == relief::Status::Ok && field.minMax(lo, hi)) {
        out.zOffset = out.stockTop - static_cast<double>(hi);
        out.valid = true;
    }
    return out;
}

// --- Tools ---------------------------------------------------------------------------------------

ToolLookup resolveTool(const tooling::ToolDatabase& db, const document::ToolSource& source) {
    ToolLookup out;
    if (source.toolId.empty()) {
        out.error = "no tool chosen";
        return out;
    }
    const tooling::Tool* tool = db.findTool(source.toolId);
    if (tool == nullptr) {
        out.error = "the tool database has no tool with id '" + source.toolId + "'";
        return out;
    }
    if (source.machine.empty() != source.material.empty()) {
        out.error = "machine and material must be chosen together";
        return out;
    }
    ResolvedTool resolved;
    resolved.tool = *tool;
    if (!source.machine.empty()) {
        if (db.findMachine(source.machine) == nullptr) {
            out.error = "the tool database has no machine with id '" + source.machine + "'";
            return out;
        }
        if (db.findMaterial(source.material) == nullptr) {
            out.error = "the tool database has no material with id '" + source.material + "'";
            return out;
        }
        const tooling::CuttingDataLookup lookup = tool->cuttingDataFor(source.machine, source.material);
        resolved.fallback = lookup.isFallback;
        resolved.data = *lookup.data;
    }
    out.tool = std::move(resolved);
    return out;
}

std::uint64_t toolFingerprint(const ResolvedTool& r) {
    Fingerprint fp;
    const tooling::Tool& t = r.tool;
    fp.add(std::string_view(t.id));
    fp.add(std::string_view(t.name));
    fp.add(static_cast<std::uint64_t>(t.type));
    fp.add(t.diameter);
    fp.add(t.includedAngle);
    fp.add(t.tipDiameter);
    fp.add(t.cornerRadius);
    fp.add(t.shankDiameter);
    fp.add(t.toothHeight);
    fp.add(t.toothOffset);
    fp.add(t.threadedLength);
    fp.add(static_cast<std::uint64_t>(t.profile.size()));
    for (const geometry::Vec2 p : t.profile) {
        fp.add(p.x);
        fp.add(p.y);
    }
    fp.add(static_cast<std::uint64_t>(static_cast<std::int64_t>(t.flutes)));
    addCuttingData(fp, t.defaultCuttingData);
    fp.add(static_cast<std::uint64_t>(r.data.has_value() ? 1 : 0));
    if (r.data.has_value()) {
        addCuttingData(fp, *r.data);
    }
    return fp.value();
}

std::uint64_t toolsFingerprintOf(const ToolpathDefinition& definition, const tooling::ToolDatabase& db) {
    Fingerprint fp;
    const auto one = [&](const document::ToolSource& s) {
        const ToolLookup l = resolveTool(db, s);
        if (!l.tool.has_value()) {
            return false;
        }
        fp.add(toolFingerprint(*l.tool));
        return true;
    };
    if (!one(definition.tool)) {
        return 0;
    }
    if (hasClearanceTools(definition.operation())) {
        for (const document::ToolSource& c : definition.clearanceTools) {
            if (!one(c)) {
                return 0;
            }
        }
    }
    const std::uint64_t v = fp.value();
    return v == 0 ? 1 : v;
}

// --- Calculation ---------------------------------------------------------------------------------

bool ToolpathCalculation::hasError() const {
    return std::any_of(messages.begin(), messages.end(),
                       [](const ToolpathMessage& m) { return m.severity == MessageSeverity::Error; });
}

std::string ToolpathCalculation::errorText() const {
    std::string s;
    for (const ToolpathMessage& m : messages) {
        if (m.severity == MessageSeverity::Error) {
            s += s.empty() ? "" : "; ";
            s += m.text;
        }
    }
    return s;
}

ToolpathJob makeToolpathJob(const ToolpathDefinition& definition, const ResolvedVectors& vectors,
                            const tooling::ToolDatabase& db, const document::VectorSelection& selection) {
    ToolpathJob job;
    job.definition = definition;
    job.definition.selector = normalisedSelector(job.definition.selector);
    if (!hasClearanceTools(job.definition.operation())) {
        job.definition.clearanceTools.clear();
    }
    if (auto* profile = std::get_if<toolpath::ProfileParams>(&job.definition.params)) {
        profile->cuttingData.reset();  // cutting data comes from the tool database, not the definition
    }
    job.selection = selection;
    job.vectorsFingerprint = fingerprint(vectors);

    const auto refuse = [&](std::string why) {
        if (job.refusal.empty()) {
            job.refusal = std::move(why);
        }
    };
    if (const std::string bad = validateToolpathName(job.definition.name()); !bad.empty()) {
        refuse(bad);
    }
    if (const std::string key = document::nonFiniteParam(job.definition.params); !key.empty()) {
        refuse("parameter " + key + " is not a finite number");
    } else if (const std::string range = validateParams(job.definition.params); !range.empty()) {
        refuse(range);
    }

    // Tools.
    const ToolLookup main = resolveTool(db, job.definition.tool);
    if (!main.tool.has_value()) {
        refuse("tool: " + main.error);
    } else {
        job.tool = *main.tool;
        if (main.tool->fallback) {
            job.notes.push_back(note(MessageSeverity::Info, "tool '" + main.tool->tool.name +
                                                                "' has no cutting data for the chosen machine and "
                                                                "material; its default cutting data is used"));
        }
    }
    if (job.definition.operation() == ToolpathOperation::Moulding && job.definition.clearanceTools.size() > 1) {
        refuse("a moulding takes at most one larger-area clearance tool");
    }
    for (std::size_t i = 0; i < job.definition.clearanceTools.size(); ++i) {
        const ToolLookup l = resolveTool(db, job.definition.clearanceTools[i]);
        if (!l.tool.has_value()) {
            refuse("clearance tool " + std::to_string(i + 1) + ": " + l.error);
            continue;
        }
        job.clearance.push_back(*l.tool);
        if (l.tool->fallback) {
            job.notes.push_back(note(MessageSeverity::Info, "clearance tool '" + l.tool->tool.name +
                                                                "' has no cutting data for the chosen machine and "
                                                                "material; its default cutting data is used"));
        }
    }
    job.toolsFingerprint = toolsFingerprintOf(job.definition, db);

    // Vectors.
    if (vectors.skippedHidden > 0) {
        job.notes.push_back(note(MessageSeverity::Info, std::to_string(vectors.skippedHidden) +
                                                            " vector(s) on hidden layers are not machined"));
    }
    if (vectors.skippedLocked > 0) {
        job.notes.push_back(note(MessageSeverity::Info, std::to_string(vectors.skippedLocked) +
                                                            " vector(s) on locked layers are not machined"));
    }
    if (vectors.skippedOtherSheet > 0) {
        job.notes.push_back(note(MessageSeverity::Info, std::to_string(vectors.skippedOtherSheet) +
                                                            " vector(s) on other sheets are not machined"));
    }
    const auto* photoParams = std::get_if<toolpath::ImageEngraveParams>(&job.definition.params);
    const bool photoWithoutVectors = photoParams != nullptr && photoParams->strategy != toolpath::PhotoStrategy::SelectedVectors;
    if (photoWithoutVectors) {
        return job;  // reads the bitmap only (see attachPhotoInput)
    }
    const bool modelOnly = usesModel(job.definition.operation()) && !boundaryUsesVectors(job.definition.params);
    if (modelOnly) {
        return job;  // reads the relief model only (see attachModel)
    }
    if (vectors.contours.empty()) {
        refuse(photoParams != nullptr
                   ? "Selected Vectors needs vectors to follow: select some besides the bitmap"
                   : usesModel(job.definition.operation())
                       ? "the Vectors boundary needs closed vectors: select some"
                   : "there are no vectors to machine (the selection is empty, or hidden, locked or on another sheet)");
        return job;
    }
    if (vectors.spanCount > kMaxToolpathSpans) {
        refuse("too many vectors to machine in one toolpath (" + std::to_string(vectors.spanCount) +
               " spans; at most " + std::to_string(kMaxToolpathSpans) + ")");
        return job;
    }
    for (const ContourRef& c : vectors.contours) {
        for (const geometry::Span& s : c.contour->spans) {
            if (!spanFinite(s)) {
                refuse("vector #" + std::to_string(c.object.value) + " has coordinates that are not finite or beyond " +
                       std::to_string(static_cast<long long>(kMaxToolpathCoordinateMm)) + " mm");
                return job;
            }
        }
    }
    std::string work;
    if (const auto* profile = std::get_if<toolpath::ProfileParams>(&job.definition.params);
        profile != nullptr && job.tool.has_value()) {
        work = profileWorkProblem(*profile, *job.tool, vectors);
    } else if (const auto* engrave = std::get_if<toolpath::EngraveParams>(&job.definition.params)) {
        work = engraveWorkProblem(*engrave, vectors);
    }
    if (!work.empty()) {
        refuse(work);
        return job;
    }
    double totalLength = 0.0;
    for (const ContourRef& c : vectors.contours) {
        totalLength += c.contour->length();
        if (!(totalLength <= kMaxToolpathLengthMm)) {
            refuse("the vectors are too long to machine in one toolpath (more than " +
                   std::to_string(static_cast<long long>(kMaxToolpathLengthMm)) + " mm in all)");
            return job;
        }
    }
    job.contours = vectors.copyContours();
    job.owners.reserve(vectors.contours.size());
    for (const ContourRef& c : vectors.contours) {
        job.owners.push_back(c.object);
    }
    return job;
}

void attachPlugSheet(ToolpathJob& job, const document::Document& doc) {
    const auto* params = std::get_if<toolpath::VCarveInlayParams>(&job.definition.params);
    if (params == nullptr || params->piece != toolpath::VCarveInlayPiece::Plug) {
        return;
    }
    const document::Sheet* sheet = nullptr;
    if (params->plugSheet != 0) {
        sheet = doc.findSheet(document::SheetId{params->plugSheet});
        if (sheet == nullptr) {
            if (job.refusal.empty()) {
                job.refusal = "the plug's sheet (id " + std::to_string(params->plugSheet) + ") does not exist";
            }
            return;
        }
        if (doc.job().type == document::JobType::DoubleSided) {
            job.sheet = sheet->id;  // the plug's toolpath belongs to the sheet it is cut on
        }
    } else {
        sheet = doc.findSheet(doc.activeSheet());
    }
    if (sheet != nullptr) {
        job.plugSheetBounds = sheet->materialBoundsMm();
    }
}

void attachPhotoInput(ToolpathJob& job, const document::Document& doc, const ResolvedBitmap& bitmap,
                      const document::BitmapDecoder& decoder) {
    if (!std::holds_alternative<toolpath::ImageEngraveParams>(job.definition.params)) {
        return;
    }
    const auto refuse = [&](std::string why) {
        if (job.refusal.empty()) {
            job.refusal = std::move(why);
        }
    };
    if (bitmap.bitmap == nullptr) {
        refuse(bitmap.problem.empty() ? std::string("select a bitmap") : bitmap.problem);
        return;
    }
    const document::BitmapAsset* asset = doc.findBitmap(bitmap.bitmap->sourceRef);
    if (asset == nullptr) {
        refuse("the bitmap has no embedded image data");
        return;
    }
    const geometry::Box2& box = bitmap.bitmap->placementMm;
    const auto ok = [](double v) { return std::isfinite(v) && std::fabs(v) <= kMaxToolpathCoordinateMm; };
    if (!(ok(box.min.x) && ok(box.min.y) && ok(box.max.x) && ok(box.max.y)) || box.isEmpty()) {
        refuse("the bitmap's placement is empty or beyond " +
               std::to_string(static_cast<long long>(kMaxToolpathCoordinateMm)) + " mm");
        return;
    }
    PhotoInput in;
    in.bitmap = bitmap.object;
    in.placement = box;
    in.bytes = asset->bytes;
    in.decoder = decoder;
    job.photo = std::move(in);
}

void attachPriors(ToolpathJob& job, const document::Document& doc, const tooling::ToolDatabase& db,
                  const std::vector<document::ToolpathId>& own) {
    const auto* params = std::get_if<toolpath::Finish3dParams>(&job.definition.params);
    if (params == nullptr || params->priorToolpaths.empty()) {
        return;
    }
    const auto refuse = [&](std::string why) {
        if (job.refusal.empty()) {
            job.refusal = std::move(why);
        }
    };
    std::set<std::uint64_t> seen;
    for (const std::uint64_t id : params->priorToolpaths) {
        const std::string label = "prior toolpath #" + std::to_string(id);
        const document::ToolpathRecord* rec = id == 0 ? nullptr : doc.findToolpath(document::ToolpathId{id});
        if (rec == nullptr) {
            refuse(label + " does not exist");
            return;
        }
        if (!seen.insert(id).second) {
            refuse(label + " is listed twice");
            return;
        }
        if (std::find(own.begin(), own.end(), rec->id) != own.end()) {
            refuse("a 3D finish cannot use its own toolpath as a prior");
            return;
        }
        if (!rec->hasMoves()) {
            refuse("prior toolpath '" + rec->name + "' is not calculated");
            return;
        }
        const ToolLookup l = resolveTool(db, rec->toolSource);
        if (!l.tool.has_value()) {
            refuse("prior toolpath '" + rec->name + "': tool: " + l.error);
            return;
        }
        ProjectSource src;
        src.id = rec->id;
        src.name = rec->name;
        src.tool = rec->tool;
        src.moves = rec->moves;
        job.priors.push_back(PriorInput{std::move(src), *l.tool});
    }
}

std::uint64_t sourceFingerprint(const document::Document& doc, std::uint64_t sourceToolpath) {
    const document::ToolpathRecord* rec = sourceToolpath == 0 ? nullptr : doc.findToolpath(document::ToolpathId{sourceToolpath});
    Fingerprint fp;
    if (rec == nullptr || !rec->hasMoves()) {
        fp.add(std::uint64_t{0});
        return fp.value();
    }
    fp.add(std::uint64_t{1});
    fp.add(static_cast<std::uint64_t>(rec->moves->size()));
    for (const toolpath::Move& mv : *rec->moves) {
        fp.add(static_cast<std::uint64_t>(mv.kind));
        fp.add(mv.to.x);
        fp.add(mv.to.y);
        fp.add(mv.to.z);
        fp.add(mv.centre.x);
        fp.add(mv.centre.y);
    }
    return fp.value();
}

std::uint64_t inputsFingerprint(const document::Document& doc, const VectorSelector& selector,
                                const document::ToolpathParams& params) {
    if (const ToolpathOperation op = static_cast<ToolpathOperation>(params.index()); usesModel(op)) {
        Fingerprint m;
        m.add(modelFingerprint(doc, doc.activeSheet()));
        if (const auto* project = std::get_if<toolpath::Project3dParams>(&params)) {
            m.add(sourceFingerprint(doc, project->sourceToolpath));
        }
        if (const auto* finish = std::get_if<toolpath::Finish3dParams>(&params)) {
            for (const std::uint64_t prior : finish->priorToolpaths) {
                m.add(sourceFingerprint(doc, prior));
            }
        }
        if (boundaryUsesVectors(params)) {
            m.add(fingerprint(resolveVectors(doc, selector)));
        }
        return m.value();
    }
    const auto* photo = std::get_if<toolpath::ImageEngraveParams>(&params);
    if (photo == nullptr) {
        return fingerprint(resolveVectors(doc, selector));
    }
    Fingerprint fp;
    fp.add(fingerprint(resolveBitmap(doc, selector)));
    if (photo->strategy == toolpath::PhotoStrategy::SelectedVectors) {
        fp.add(fingerprint(resolveVectors(doc, selector)));
    }
    return fp.value();
}

namespace {

document::ToolpathRecord makeRecord(const ToolpathJob& job, toolpath::Toolpath tp, const document::ToolSource& source) {
    document::ToolpathRecord record;
    record.name = tp.name;
    record.params = job.definition.params;
    if (auto* profile = std::get_if<toolpath::ProfileParams>(&record.params)) {
        profile->cuttingData.reset();  // tool-database input, not part of the record
    }
    if (auto* rough = std::get_if<toolpath::Rough3dParams>(&record.params)) {
        rough->vectors.clear();  // the boundary vectors come from the selection
    }
    if (auto* finish = std::get_if<toolpath::Finish3dParams>(&record.params)) {
        finish->vectors.clear();
        finish->priors.clear();
    }
    record.selection = job.selection;
    record.sheet = job.sheet;
    record.expressions = job.definition.expressions;
    record.tool = tp.tool;
    record.toolSource = source;
    record.moves = document::shareMoves(std::move(tp.moves));
    return record;
}

}  // namespace

ToolpathCalculation ToolpathJob::run(const std::atomic<bool>* cancel) const {
    ToolpathCalculation out;
    out.definition = definition;
    out.vectorCount = contours.size();
    out.vectorsFingerprint = vectorsFingerprint;
    out.toolsFingerprint = toolsFingerprint;
    out.target = target;
    out.targetRecords = targetRecords;
    out.messages = notes;
    if (!refusal.empty()) {
        out.messages.push_back(note(MessageSeverity::Error, refusal));
        return out;
    }
    const auto* photoParams = std::get_if<toolpath::ImageEngraveParams>(&definition.params);
    const bool modelOp = usesModel(definition.operation());
    const bool projectOp = definition.operation() == ToolpathOperation::Project3d;
    const bool needsContours = modelOp ? boundaryUsesVectors(definition.params)
                                       : photoParams == nullptr ||
                                             photoParams->strategy == toolpath::PhotoStrategy::SelectedVectors;
    if (!tool.has_value() || (needsContours && contours.empty()) || (photoParams != nullptr && !photo.has_value()) ||
        (modelOp && model == nullptr) || (projectOp && !source.has_value()) ||
        (std::holds_alternative<toolpath::Finish3dParams>(definition.params) &&
         std::get<toolpath::Finish3dParams>(definition.params).priorToolpaths.size() != priors.size()) ||
        (hasRestTools(definition.operation()) && clearance.size() != definition.clearanceTools.size())) {
        // a job not made by makeToolpathJob
        out.messages.push_back(note(MessageSeverity::Error, "the job has no tool or no vectors"));
        return out;
    }
    const auto cancelled = [&] { return cancel != nullptr && cancel->load(); };
    if (cancelled()) {
        out.cancelled = true;
        out.messages.push_back(note(MessageSeverity::Info, "the calculation was cancelled"));
        return out;
    }

    const ToolpathOperation op = definition.operation();
    std::vector<toolpath::ToolpathDiagnostic> diagnostics;
    std::vector<ToolpathCalculation::Item> items;
    try {
        if (op == ToolpathOperation::Profile) {
            toolpath::ProfileParams params = std::get<toolpath::ProfileParams>(definition.params);
            params.cuttingData = tool->data;
            toolpath::ProfileResult r = toolpath::profileToolpath(contours, tool->tool, params);
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::Drill) {
            toolpath::DrillResult r = toolpath::drillToolpath(contours, tool->tool, tool->data,
                                                              std::get<toolpath::DrillParams>(definition.params));
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::Engrave) {
            toolpath::EngraveResult r = toolpath::engraveToolpath(contours, tool->tool, tool->data,
                                                                  std::get<toolpath::EngraveParams>(definition.params));
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::Chamfer) {
            toolpath::ChamferResult r = toolpath::chamferToolpath(contours, tool->tool, tool->data,
                                                                  std::get<toolpath::ChamferParams>(definition.params));
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::ThreadMill) {
            toolpath::ThreadMillResult r = toolpath::threadMillToolpath(
                contours, tool->tool, tool->data, std::get<toolpath::ThreadMillParams>(definition.params));
            diagnostics = r.diagnostics;
            if (r.ok()) {
                const auto& tm = std::get<toolpath::ThreadMillParams>(definition.params);
                if (tm.createCircles && tm.type == toolpath::ThreadType::Internal) {
                    for (const toolpath::ThreadMillThread& t : r.threads) {
                        const std::vector<geometry::Contour> c = toolpath::threadBoreCircles(tm, {t.centre});
                        if (!c.empty() && t.contour < owners.size()) {
                            out.boreCircles.push_back({c.front(), owners[t.contour]});
                        }
                    }
                }
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::Inlay) {
            toolpath::InlayResult r = toolpath::inlayToolpath(contours, tool->tool, tool->data,
                                                              std::get<toolpath::InlayParams>(definition.params));
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::Fluting) {
            toolpath::FlutingResult r = toolpath::flutingToolpath(
                contours, tool->tool, tool->data, std::get<toolpath::FlutingParams>(definition.params));
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::LaserCutFill) {
            toolpath::ToolpathRunControl control;
            control.cancel = cancel;
            toolpath::LaserCutFillResult r = toolpath::laserCutFillToolpath(
                contours, tool->tool, tool->data, std::get<toolpath::LaserCutFillParams>(definition.params), control);
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::Prism) {
            toolpath::PrismResult r = toolpath::prismToolpath(contours, tool->tool, tool->data,
                                                              std::get<toolpath::PrismParams>(definition.params));
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::ImageEngrave) {
            const document::BitmapDecoder& decode = photo->decoder;
            geometry::ShapeResult<io::RasterImage> raster =
                document::decodeBitmapBytes(std::span<const std::uint8_t>(photo->bytes), decode);
            if (!raster.ok()) {
                out.messages.push_back(note(MessageSeverity::Error, "the bitmap cannot be read: " + raster.error()));
                return out;
            }
            if (cancelled()) {
                out.cancelled = true;
                out.messages.push_back(note(MessageSeverity::Info, "the calculation was cancelled"));
                return out;
            }
            const toolpath::GrayImage gray = toolpath::grayFromRaster(raster.value());
            toolpath::ToolpathRunControl control;
            control.cancel = cancel;
            toolpath::ImageEngraveResult r = toolpath::imageEngrave(gray, photo->placement, tool->tool, tool->data,
                                                                  *photoParams, contours, control);
            diagnostics = r.diagnostics;
            if (r.ok()) {
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::Rough3d || op == ToolpathOperation::Finish3d) {
            std::vector<geometry::Polyline> vectors;
            if (needsContours) {
                for (const geometry::Contour& c : contours) {
                    vectors.push_back(c.polygonise(geometry::Tolerance{}));
                }
            }
            toolpath::ToolpathRunControl control;
            control.cancel = cancel;
            // The tool list: the earlier (rest) tools first, the operation's own tool last (T-0314).
            std::vector<toolpath::Rough3dTool> tools;
            for (const ResolvedTool& c : clearance) {
                tools.push_back(toolpath::Rough3dTool{c.tool, c.data, std::nullopt});
            }
            tools.push_back(toolpath::Rough3dTool{tool->tool, tool->data, std::nullopt});
            toolpath::Rough3dResult r;
            if (op == ToolpathOperation::Rough3d) {
                toolpath::Rough3dParams params = std::get<toolpath::Rough3dParams>(definition.params);
                params.vectors = std::move(vectors);
                r = toolpath::rough3dToolpath(*model, tools, params, control);
            } else {
                toolpath::Finish3dParams params = std::get<toolpath::Finish3dParams>(definition.params);
                params.vectors = std::move(vectors);
                params.priors.clear();
                for (const PriorInput& prior : priors) {
                    toolpath::Finish3dPrior p;
                    p.toolpath.name = prior.source.name;
                    p.toolpath.tool = prior.source.tool;
                    p.toolpath.moves = *prior.source.moves;
                    p.tool = toolpath::Rough3dTool{prior.tool.tool, prior.tool.data, std::nullopt};
                    params.priors.push_back(std::move(p));
                }
                r = toolpath::finish3dToolpath(*model, tools, params, control);
            }
            diagnostics = r.diagnostics;
            for (std::size_t i = 0; i < r.toolpaths.size(); ++i) {
                toolpath::Toolpath& tp = r.toolpaths[i];
                if (tp.isEmpty()) {
                    continue;
                }
                const bool last = i + 1 == r.toolpaths.size();
                const std::size_t rest = clearance.size();
                items.push_back({last ? -1 : static_cast<int>(i),
                                 makeRecord(*this, std::move(tp),
                                            last || i >= rest ? definition.tool : definition.clearanceTools[i])});
            }
        } else if (op == ToolpathOperation::Moulding) {
            toolpath::MouldingParams params = std::get<toolpath::MouldingParams>(definition.params);
            std::vector<geometry::Polyline> lines;
            lines.reserve(contours.size());
            for (const geometry::Contour& c : contours) {
                lines.push_back(c.polygonise(geometry::Tolerance{}));
            }
            std::size_t profile = lines.size();
            for (std::size_t i = 0; i < lines.size() && profile == lines.size(); ++i) {
                const bool named = params.profileObject != 0 && i < owners.size() && owners[i].value == params.profileObject;
                if ((params.profileObject != 0 && named) || (params.profileObject == 0 && !lines[i].closed)) {
                    profile = i;
                }
            }
            if (profile == lines.size()) {
                out.messages.push_back(note(MessageSeverity::Error, params.profileObject != 0
                                                                        ? "the moulding's profile vector is not among the selected vectors"
                                                                        : "the moulding needs a profile: select an open vector (the section)"));
                return out;
            }
            if (lines[profile].closed) {
                out.messages.push_back(note(MessageSeverity::Error, "the moulding's profile must be an open vector"));
                return out;
            }
            params.profile = std::move(lines[profile]);
            params.rails.clear();
            for (std::size_t i = 0; i < lines.size(); ++i) {
                if (i != profile) {
                    params.rails.push_back(std::move(lines[i]));
                }
            }
            if (params.rails.empty()) {
                out.messages.push_back(note(MessageSeverity::Error, "the moulding needs at least one rail: select another vector besides the profile"));
                return out;
            }
            params.clearTool.reset();
            if (!clearance.empty()) {
                params.clearTool = toolpath::Rough3dTool{clearance.front().tool, clearance.front().data, std::nullopt};
            }
            toolpath::ToolpathRunControl control;
            control.cancel = cancel;
            const toolpath::Rough3dTool mouldingTool{tool->tool, tool->data, std::nullopt};
            toolpath::MouldingResult r = toolpath::mouldingToolpath(mouldingTool, params, control);
            diagnostics = r.diagnostics;
            // With a clearance tool the result is [Clear, main] (T-0314); otherwise just the main one.
            for (std::size_t i = 0; i < r.toolpaths.size(); ++i) {
                toolpath::Toolpath& tp = r.toolpaths[i];
                if (tp.isEmpty()) {
                    continue;
                }
                const bool last = i + 1 == r.toolpaths.size();
                items.push_back({last ? -1 : static_cast<int>(i),
                                 makeRecord(*this, std::move(tp), last ? definition.tool : definition.clearanceTools[0])});
            }
        } else if (op == ToolpathOperation::Project3d) {
            toolpath::Toolpath input;
            input.name = source->name;
            input.tool = source->tool;
            input.moves = *source->moves;
            toolpath::ToolpathRunControl control;
            control.cancel = cancel;
            toolpath::Project3dResult r = toolpath::projectToolpath(
                input, *model, tool->tool, std::get<toolpath::Project3dParams>(definition.params), control);
            diagnostics = r.diagnostics;
            if (!r.path.isEmpty()) {
                items.push_back({-1, makeRecord(*this, std::move(r.path), definition.tool)});
            }
        } else if (op == ToolpathOperation::VCarveInlay) {
            std::vector<toolpath::PocketTool> clearanceTools;
            for (const ResolvedTool& c : clearance) {
                clearanceTools.push_back(toolpath::PocketTool{c.tool, c.data});
            }
            toolpath::VCarvingResult r = toolpath::vCarveInlayToolpath(
                contours, tool->tool, tool->data, clearanceTools, std::get<toolpath::VCarveInlayParams>(definition.params),
                plugSheetBounds);
            diagnostics = r.diagnostics;
            if (r.ok()) {
                for (std::size_t i = 0; i < r.clearance.size(); ++i) {
                    if (!r.clearance[i].isEmpty() && i < definition.clearanceTools.size()) {
                        items.push_back({static_cast<int>(i),
                                         makeRecord(*this, std::move(r.clearance[i]), definition.clearanceTools[i])});
                    }
                }
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else if (op == ToolpathOperation::VCarving) {
            std::vector<toolpath::PocketTool> clearanceTools;
            for (const ResolvedTool& c : clearance) {
                clearanceTools.push_back(toolpath::PocketTool{c.tool, c.data});
            }
            toolpath::VCarvingResult r =
                toolpath::vCarvingToolpath(contours, tool->tool, tool->data, clearanceTools,
                                           std::get<toolpath::VCarvingParams>(definition.params));
            diagnostics = r.diagnostics;
            if (r.ok()) {
                for (std::size_t i = 0; i < r.clearance.size(); ++i) {
                    if (!r.clearance[i].isEmpty() && i < definition.clearanceTools.size()) {
                        items.push_back({static_cast<int>(i),
                                         makeRecord(*this, std::move(r.clearance[i]), definition.clearanceTools[i])});
                    }
                }
                items.push_back({-1, makeRecord(*this, std::move(r.toolpath), definition.tool)});
            }
        } else {
            toolpath::PocketResult r = toolpath::pocketToolpath(
                contours, {toolpath::PocketTool{tool->tool, tool->data}},
                std::get<toolpath::PocketParams>(definition.params));
            diagnostics = r.diagnostics;
            for (toolpath::Toolpath& tp : r.toolpaths) {
                if (!tp.isEmpty()) {
                    items.push_back({-1, makeRecord(*this, std::move(tp), definition.tool)});
                }
            }
        }
    } catch (const std::bad_alloc&) {
        out.messages.push_back(note(MessageSeverity::Error, "out of memory while calculating the toolpath"));
        return out;
    } catch (const std::exception& e) {
        out.messages.push_back(
            note(MessageSeverity::Error, std::string("the toolpath calculation failed: ") + e.what()));
        return out;
    }

    if (cancelled()) {
        out.cancelled = true;
        out.messages.push_back(note(MessageSeverity::Info, "the calculation was cancelled"));
        return out;
    }
    const bool produced = !items.empty();
    for (ToolpathMessage& m : convertDiagnostics(diagnostics, produced, owners)) {
        out.messages.push_back(std::move(m));
    }
    if (!produced) {
        out.messages.push_back(note(MessageSeverity::Error, std::string("no ") + operationName(op) +
                                                                " toolpath was produced: " +
                                                                joinDiagnostics(diagnostics)));
        return out;
    }
    out.items = std::move(items);
    out.ok = true;
    return out;
}

}  // namespace carvetto::viewmodel
