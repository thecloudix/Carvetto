#include "carvetto/viewmodel/relief_build.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <sstream>
#include <utility>

#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/relief/extrude.hpp"
#include "carvetto/relief/modify.hpp"
#include "carvetto/relief/revolve.hpp"
#include "carvetto/relief/sweep.hpp"
#include "carvetto/relief/textured_area.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"
#include "carvetto/viewmodel/vector_selector.hpp"

namespace carvetto::viewmodel {

namespace {

using Kind = ModifyField::Kind;
using document::Document;

ModifyField num(const char* label, const char* def, const char* tip) {
    return {label, Kind::Number, def, {}, tip};
}
ModifyField chk(const char* label, bool def, const char* tip) {
    return {label, Kind::Check, def ? "true" : "false", {}, tip};
}
ModifyField choice(const char* label, std::vector<std::string> choices, const char* tip) {
    ModifyField f{label, Kind::Choice, choices.front(), std::move(choices), tip};
    return f;
}
ModifyField text(const char* label, const char* def, const char* tip) {
    return {label, Kind::Number, def, {}, tip};   // a line edit; read as text by the tool
}

std::vector<std::string> modeNames() {
    std::vector<std::string> names;
    for (const relief::CombineMode m : allCombineModes()) {
        names.emplace_back(combineModeName(m));
    }
    return names;
}

relief::CombineMode modeFromName(const std::string& name) {
    for (const relief::CombineMode m : allCombineModes()) {
        if (name == combineModeName(m)) {
            return m;
        }
    }
    return relief::CombineMode::Add;
}

bool parseFinite(const std::string& s, double& out) {
    const char* begin = s.c_str();
    while (*begin == ' ' || *begin == '\t') {
        ++begin;
    }
    if (*begin == '\0') {
        return false;
    }
    char* end = nullptr;
    const double v = std::strtod(begin, &end);
    if (end == begin) {
        return false;
    }
    while (*end == ' ' || *end == '\t') {
        ++end;
    }
    if (*end != '\0' || !std::isfinite(v) || std::fabs(v) > 1.0e9) {
        return false;
    }
    out = v;
    return true;
}

ModellingPreview fail(std::string message) {
    ModellingPreview p;
    p.message = std::move(message);
    return p;
}

/// Field text: the typed value, else the default.
struct Reader {
    const BuildForm& form;
    std::string error;

    std::string raw(const char* label) const {
        const auto it = form.values.find(label);
        if (it != form.values.end()) {
            return it->second;
        }
        for (const ModifyField& f : buildFields(form.tool)) {
            if (f.label == label) {
                return f.defaultValue;
            }
        }
        return {};
    }
    bool num(const char* label, double& out) {
        if (!error.empty()) {
            return false;
        }
        if (!parseFinite(raw(label), out)) {
            error = std::string(label) + " is not a valid number";
            return false;
        }
        return true;
    }
    bool flag(const char* label) const { return raw(label) == "true"; }
};

std::vector<geometry::Polyline> selectedPolylines(const Document& doc) {
    const ResolvedVectors rv = resolveVectors(doc, VectorSelector::fromSelection(doc));
    std::vector<geometry::Polyline> out;
    for (const ContourRef& ref : rv.contours) {
        if (ref.contour == nullptr) {
            continue;
        }
        geometry::Polyline pl = ref.contour->polygonise(geometry::Tolerance::defaults());
        pl.closed = ref.contour->closed;
        if (pl.vertexCount() >= (pl.closed ? 3U : 2U)) {
            out.push_back(std::move(pl));
        }
    }
    return out;
}

std::vector<relief::Point> toPoints(const geometry::Polyline& pl) {
    std::vector<relief::Point> pts;
    pts.reserve(pl.points.size());
    for (const geometry::Vec2& v : pl.points) {
        pts.push_back({v.x, v.y});
    }
    return pts;
}

/// Grid covering `[min, max]` grown by `margin`, at `cell`; false when the size is not usable.
bool gridFor(relief::Point lo, relief::Point hi, double margin, double cell, relief::Grid& g, std::string& error) {
    if (!(cell > 0.0)) {
        error = "Cell size must be above zero";
        return false;
    }
    lo.x -= margin;
    lo.y -= margin;
    hi.x += margin;
    hi.y += margin;
    const double w = std::ceil((hi.x - lo.x) / cell) + 1.0;
    const double h = std::ceil((hi.y - lo.y) / cell) + 1.0;
    if (!(w < 1.0e6 && h < 1.0e6)) {
        error = "The model is too large for this cell size";
        return false;
    }
    g.width = static_cast<std::size_t>(w);
    g.height = static_cast<std::size_t>(h);
    g.cellSize = cell;
    g.origin = lo;
    const relief::Status st = relief::validate(g);
    if (st != relief::Status::Ok) {
        error = relief::statusMessage(st);
        return false;
    }
    return true;
}

void railBounds(const std::vector<std::vector<relief::Point>>& rails, relief::Point& lo, relief::Point& hi) {
    lo = rails.front().front();
    hi = lo;
    for (const auto& r : rails) {
        for (const relief::Point& p : r) {
            lo.x = std::min(lo.x, p.x);
            lo.y = std::min(lo.y, p.y);
            hi.x = std::max(hi.x, p.x);
            hi.y = std::max(hi.y, p.y);
        }
    }
}

std::string describe(const relief::Component& c) {
    std::ostringstream os;
    os.precision(6);
    os << c.field.width() << " x " << c.field.height() << " cells, " << c.field.cellSize() << " mm";
    float lo = 0.0F;
    float hi = 0.0F;
    if (c.field.minMax(lo, hi)) {
        os << ", height " << static_cast<double>(lo) << " to " << static_cast<double>(hi) << " mm";
    }
    return os.str();
}

/// The first selected component baked onto its own grid.
bool selectedComponent(const Document& doc, relief::Component& out, std::string& error) {
    for (const document::ComponentId id : doc.componentSelection().ids()) {
        if (const document::ReliefComponent* c = doc.findReliefComponent(id)) {
            const relief::Status st = relief::bakeComponents({c->data}, out);
            if (st != relief::Status::Ok) {
                error = relief::statusMessage(st);
                return false;
            }
            return true;
        }
    }
    error = "Select a component in the model tree first";
    return false;
}

// --- tools --------------------------------------------------------------------------------------------

ModellingPreview previewSweep(const Document& doc, Reader& r) {
    double cell = 0.0;
    relief::SweepParams p;
    double exact = 0.0;
    std::vector<relief::Point> profile;
    std::string perr;
    if (!r.num("Cell size", cell) || !r.num("Exact height", exact)) {
        return fail(r.error);
    }
    if (!parsePointList(r.raw("Section profile"), profile, perr)) {
        return fail(perr);
    }
    const std::vector<geometry::Polyline> rails = selectedPolylines(doc);
    if (rails.size() < 2) {
        return fail("Select two vectors as the rails first");
    }
    p.rail1 = toPoints(rails[0]);
    p.rail2 = toPoints(rails[1]);
    p.closed1 = rails[0].closed;
    p.closed2 = rails[1].closed;
    p.scaleWithWidth = r.flag("Scale with width");
    p.sweepBetweenSpans = r.flag("Blend between sections");
    p.fillCenter = r.flag("Fill centre");
    p.exactHeight = exact;
    if (r.flag("Swap rails")) {
        relief::swapRails(p);
    }
    for (int which = 1; which <= 2; ++which) {
        if (r.flag(which == 1 ? "Reverse rail 1" : "Reverse rail 2")) {
            const relief::SweepStatus st = relief::reverseRail(p, which);
            if (st != relief::SweepStatus::Ok) {
                return fail(relief::sweepStatusMessage(st));
            }
        }
    }
    const bool smooth = r.flag("Smooth profile");
    if (r.flag("Section at every node")) {
        const relief::SweepStatus st = relief::addSectionToAllNodes(p, profile, smooth);
        if (st != relief::SweepStatus::Ok) {
            return fail(relief::sweepStatusMessage(st));
        }
    } else {
        for (const double t : {0.0, 1.0}) {
            relief::SweepSection s;
            s.profile = profile;
            s.t1 = t;
            s.t2 = t;
            s.smooth = smooth;
            p.sections.push_back(std::move(s));
        }
    }
    relief::Point lo;
    relief::Point hi;
    railBounds({p.rail1, p.rail2}, lo, hi);
    relief::Grid grid;
    std::string error;
    if (!gridFor(lo, hi, cell, cell, grid, error)) {
        return fail(error);
    }
    ModellingPreview out;
    const relief::SweepStatus st = relief::sweep(p, grid, nullptr, out.component.field);
    if (st != relief::SweepStatus::Ok) {
        return fail(relief::sweepStatusMessage(st));
    }
    out.component.name = r.raw("Name");
    out.component.mode = modeFromName(r.raw("Combine mode"));
    out.ok = true;
    out.info = describe(out.component);
    return out;
}

ModellingPreview previewExtrude(const Document& doc, Reader& r) {
    double cell = 0.0;
    relief::ExtrudeParams p;
    std::string error;
    if (!r.num("Cell size", cell) || !r.num("Exact height", p.exactHeight) ||
        !r.num("Weave under %", p.weave.zUnderPercent) || !r.num("Weave over %", p.weave.zOverPercent) ||
        !r.num("Overlap %", p.overlapPercent)) {
        return fail(r.error);
    }
    const std::vector<geometry::Polyline> lines = selectedPolylines(doc);
    if (lines.empty()) {
        return fail("Select one or more vectors as the rails first");
    }
    std::vector<std::vector<relief::Point>> railPoints;
    for (const geometry::Polyline& pl : lines) {
        relief::ExtrudeRail rail;
        rail.points = toPoints(pl);
        rail.closed = pl.closed;
        railPoints.push_back(rail.points);
        p.rails.push_back(std::move(rail));
    }
    p.sharpCorners = r.flag("Sharp corners");
    p.weave.enabled = r.flag("Weave");
    p.weave.startOver = r.flag("Start over");
    p.weave.mode = r.raw("Weave mode") == "Add base" ? relief::WeaveMode::AddBase : relief::WeaveMode::ScaleShape;
    double halfWidth = 0.0;
    if (r.raw("Source") == "Component") {
        relief::Component seed;
        if (!selectedComponent(doc, seed, error)) {
            return fail(error);
        }
        p.source = relief::ExtrudeSource::Component;
        p.component = std::move(seed.field);
        p.stretchToRail = r.flag("Stretch to rail");
        p.flipAlternate = r.flag("Flip alternate copies");
        halfWidth = 0.5 * static_cast<double>(p.component.height()) * p.component.cellSize();
    } else {
        std::vector<relief::Point> profile;
        if (!parsePointList(r.raw("Section profile"), profile, error)) {
            return fail(error);
        }
        relief::ExtrudeSection s;
        s.profile = profile;
        s.t = 0.0;
        s.smooth = r.flag("Smooth profile");
        p.sections.push_back(std::move(s));
        for (const relief::Point& pt : profile) {
            halfWidth = std::max(halfWidth, std::fabs(pt.x));
        }
        const double width = profile.back().x - profile.front().x;
        halfWidth = std::max(halfWidth, 0.5 * width);
    }
    relief::Point lo;
    relief::Point hi;
    railBounds(railPoints, lo, hi);
    relief::Grid grid;
    if (!gridFor(lo, hi, halfWidth + cell, cell, grid, error)) {
        return fail(error);
    }
    ModellingPreview out;
    const relief::ExtrudeStatus st = relief::extrude(p, grid, nullptr, out.component.field);
    if (st != relief::ExtrudeStatus::Ok) {
        return fail(relief::extrudeStatusMessage(st));
    }
    out.component.name = r.raw("Name");
    out.component.mode = modeFromName(r.raw("Combine mode"));
    out.ok = true;
    out.info = describe(out.component);
    return out;
}

ModellingPreview previewRevolve(const Document& doc, Reader& r) {
    relief::Revolve rev;
    double exact = 0.0;
    if (!r.num("Cell size", rev.params.cellSize) || !r.num("Centre X", rev.params.center.x) ||
        !r.num("Centre Y", rev.params.center.y) || !r.num("Base height", rev.params.baseHeight) ||
        !r.num("Exact height", exact)) {
        return fail(r.error);
    }
    rev.params.kind = r.raw("Kind") == "Turn" ? relief::RevolveKind::Turn : relief::RevolveKind::Spin;
    if (r.flag("Scale to exact height")) {
        rev.params.exactHeight = exact;
    }
    rev.params.mode = modeFromName(r.raw("Combine mode"));
    rev.params.name = r.raw("Name");
    std::string error;
    const std::string profileText = r.raw("Profile");
    if (profileText.find_first_not_of(" \t") != std::string::npos) {
        if (!parsePointList(profileText, rev.profile, error)) {
            return fail(error);
        }
    } else {
        bool found = false;
        for (const geometry::Polyline& pl : selectedPolylines(doc)) {
            if (!pl.closed) {
                rev.profile = toPoints(pl);
                found = true;
                break;
            }
        }
        if (!found) {
            return fail("Type a Profile, or select an open vector as the profile");
        }
    }
    ModellingPreview out;
    const relief::RevolveStatus st = relief::buildRevolve(rev, nullptr, out.component);
    if (st != relief::RevolveStatus::Ok) {
        return fail(relief::revolveStatusMessage(st));
    }
    out.ok = true;
    out.info = describe(out.component);
    return out;
}

ModellingPreview previewTextured(const Document& doc, Reader& r) {
    relief::TexturedAreaParams p;
    if (!r.num("Spacing X %", p.spacingXPercent) || !r.num("Spacing Y %", p.spacingYPercent) ||
        !r.num("Shift X %", p.shiftXPercent) || !r.num("Shift Y %", p.shiftYPercent)) {
        return fail(r.error);
    }
    relief::Component seed;
    std::string error;
    if (!selectedComponent(doc, seed, error)) {
        return fail(error);
    }
    p.seed = std::move(seed.field);
    for (const geometry::Polyline& pl : selectedPolylines(doc)) {
        if (pl.closed) {
            p.rings.push_back(toPoints(pl));
        }
    }
    if (p.rings.empty()) {
        return fail("Select one or more closed vectors as the boundary first");
    }
    const std::string refl = r.raw("Reflection");
    p.reflection = refl == "Horizontal" ? relief::Reflection::Horizontal
                   : refl == "Vertical" ? relief::Reflection::Vertical
                   : refl == "Both"     ? relief::Reflection::Both
                                        : relief::Reflection::None;
    p.overlapMode = modeFromName(r.raw("Overlap mode"));
    if (r.flag("Set anchor")) {
        relief::Point a;
        if (!r.num("Anchor X", a.x) || !r.num("Anchor Y", a.y)) {
            return fail(r.error);
        }
        p.anchor = a;
    }
    ModellingPreview out;
    const relief::TexturedStatus st =
        relief::makeTexturedComponent(p, modeFromName(r.raw("Combine mode")), out.component);
    if (st != relief::TexturedStatus::Ok) {
        return fail(relief::texturedStatusMessage(st));
    }
    out.component.name = r.raw("Name");
    out.ok = true;
    out.info = describe(out.component);
    return out;
}

}  // namespace

const char* buildToolName(BuildTool tool) {
    switch (tool) {
        case BuildTool::TwoRailSweep: return "Two Rail Sweep";
        case BuildTool::ExtrudeWeave: return "Extrude and Weave";
        case BuildTool::TurnSpin: return "Turn and Spin";
        case BuildTool::TexturedArea: return "Textured Area";
    }
    return "";
}

const char* buildHelp(BuildTool tool) {
    switch (tool) {
        case BuildTool::TwoRailSweep:
            return "Select two vectors (rail 1, then rail 2), both open or both closed. The section profile "
                   "(across, height) is placed at the start and end, or at every node.";
        case BuildTool::ExtrudeWeave:
            return "Select the rail vectors. The section profile is centred on the rail (negative across is "
                   "right of travel); the Component source copies the selected component along the rails.";
        case BuildTool::TurnSpin:
            return "Spin: profile is (radius from the left end, height). Turn: profile is the outline in model "
                   "coordinates, revolved about the line from its first to its last point. Leave Profile empty "
                   "to use the selected open vector.";
        case BuildTool::TexturedArea:
            return "Select a component (the seed tile) and the closed vectors to fill with it.";
    }
    return "";
}

const std::vector<ModifyField>& buildFields(BuildTool tool) {
    static const std::vector<ModifyField> sweep = {
        text("Name", "Sweep", "Name of the new component"),
        text("Section profile", "0,0; 5,4; 10,0", "\"across,height; across,height; ...\" in mm, across rising"),
        chk("Smooth profile", false, "Smooth curve through the profile points"),
        chk("Section at every node", false, "Needs rails with equal vertex counts"),
        chk("Scale with width", false, "Height follows the gap between the rails"),
        chk("Blend between sections", true, "Blend neighbouring sections; off holds each section"),
        chk("Fill centre", false, "Closed rails: flat fill inside the inner rail"),
        chk("Swap rails", false, "Rail 1 <-> rail 2"),
        chk("Reverse rail 1", false, "Reverse the direction of rail 1"),
        chk("Reverse rail 2", false, "Reverse the direction of rail 2"),
        num("Exact height", "0", "Above 0: scale so the top equals this (mm)"),
        num("Cell size", "0.5", "Resolution of the surface in mm"),
        choice("Combine mode", modeNames(), "How the component combines with those below"),
    };
    static const std::vector<ModifyField> extrude = {
        text("Name", "Extrusion", "Name of the new component"),
        choice("Source", {"Cross section", "Component"}, "Extrude a profile, or copies of the selected component"),
        text("Section profile", "-4,0; 0,3; 4,0", "\"across,height; ...\", centred on the rail"),
        chk("Smooth profile", false, "Smooth curve through the profile points"),
        chk("Stretch to rail", true, "Component: one copy stretched to the rail; off fills with copies"),
        num("Overlap %", "0", "Component copies overlap by this much, 0 to under 100"),
        chk("Flip alternate copies", false, "Component: mirror every second copy"),
        chk("Sharp corners", false, "Mitred rail corners; off rounds them"),
        chk("Weave", false, "Crossing rails pass over and under each other"),
        num("Weave under %", "50", "Level of the strand passing under"),
        num("Weave over %", "100", "Level of the strand passing over"),
        choice("Weave mode", {"Scale shape", "Add base"}, "Scale the shape, or lower it"),
        chk("Start over", true, "Off swaps over and under at every crossing"),
        num("Exact height", "0", "Above 0: scale so the top equals this (mm)"),
        num("Cell size", "0.5", "Resolution of the surface in mm"),
        choice("Combine mode", modeNames(), "How the component combines with those below"),
    };
    static const std::vector<ModifyField> revolve = {
        text("Name", "Turned", "Name of the new component"),
        choice("Kind", {"Spin", "Turn"}, "Spin: circular component; Turn: lathe part"),
        text("Profile", "0,5; 10,5; 20,0", "\"x,y; x,y; ...\"; empty uses the selected open vector"),
        num("Centre X", "0", "Spin: model position of the axis (mm)"),
        num("Centre Y", "0", "Spin: model position of the axis (mm)"),
        chk("Scale to exact height", false, "Total height equals Exact height"),
        num("Exact height", "5", "mm"),
        num("Base height", "0", "Z offset in mm"),
        num("Cell size", "0.5", "Resolution of the surface in mm"),
        choice("Combine mode", modeNames(), "How the component combines with those below"),
    };
    static const std::vector<ModifyField> textured = {
        text("Name", "Textured", "Name of the new component"),
        num("Spacing X %", "0", "Gap between tiles as % of the tile width; negative overlaps"),
        num("Spacing Y %", "0", "Gap between tiles as % of the tile height"),
        num("Shift X %", "0", "Odd rows move along X by this % of the column pitch"),
        num("Shift Y %", "0", "Odd columns move along Y by this % of the row pitch"),
        choice("Reflection", {"None", "Horizontal", "Vertical", "Both"}, "Mirror alternate tiles"),
        choice("Overlap mode", modeNames(), "Where tiles overlap, how the later tile combines"),
        chk("Set anchor", false, "Place tile (0,0) at the anchor instead of the boundary corner"),
        num("Anchor X", "0", "mm"),
        num("Anchor Y", "0", "mm"),
        choice("Combine mode", modeNames(), "How the component combines with those below"),
    };
    switch (tool) {
        case BuildTool::TwoRailSweep: return sweep;
        case BuildTool::ExtrudeWeave: return extrude;
        case BuildTool::TurnSpin: return revolve;
        case BuildTool::TexturedArea: return textured;
    }
    return sweep;
}

BuildForm defaultBuildForm(BuildTool tool) {
    BuildForm f;
    f.tool = tool;
    for (const ModifyField& spec : buildFields(tool)) {
        f.values[spec.label] = spec.defaultValue;
    }
    return f;
}

bool parsePointList(const std::string& textIn, std::vector<relief::Point>& out, std::string& error,
                    std::size_t minPoints) {
    out.clear();
    std::istringstream in(textIn);
    std::string item;
    while (std::getline(in, item, ';')) {
        if (item.find_first_not_of(" \t") == std::string::npos) {
            continue;   // empty item / trailing separator
        }
        const std::size_t comma = item.find(',');
        relief::Point pt;
        if (comma == std::string::npos || !parseFinite(item.substr(0, comma), pt.x) ||
            !parseFinite(item.substr(comma + 1), pt.y)) {
            error = "Point \"" + item + "\" is not \"x,y\"";
            return false;
        }
        out.push_back(pt);
    }
    if (out.size() < minPoints) {
        error = "At least " + std::to_string(minPoints) + " points are needed";
        return false;
    }
    return true;
}

ModellingPreview previewBuild(const Document& doc, const BuildForm& form) {
    Reader r{form, {}};
    switch (form.tool) {
        case BuildTool::TwoRailSweep: return previewSweep(doc, r);
        case BuildTool::ExtrudeWeave: return previewExtrude(doc, r);
        case BuildTool::TurnSpin: return previewRevolve(doc, r);
        case BuildTool::TexturedArea: return previewTextured(doc, r);
    }
    return fail("Unknown tool");
}

}  // namespace carvetto::viewmodel
