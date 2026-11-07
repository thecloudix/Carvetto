#include "carvetto/viewmodel/toolpath_form.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <functional>
#include <limits>
#include <type_traits>
#include <utility>
#include <variant>

#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/units_format.hpp"

namespace carvetto::viewmodel {

using document::ToolpathOperation;
using document::Units;
using toolpath::CutDirection;

namespace {

using Kind = ToolpathFieldKind;

constexpr double kRadPerDeg = geometry::kPi / 180.0;  // as carvetto-cli converts degrees
constexpr double kMaxLength = 1e9;
constexpr double kInf = std::numeric_limits<double>::infinity();

enum class FromTool { No, PassDepth, Stepover };

template <class P>
struct NumDef {
    std::string id;
    std::string label;
    std::string group;
    Kind kind{Kind::Length};
    bool neg{false};
    bool zero{false};
    /// Inclusive bounds in the field's own unit (mm, degrees, seconds, count, factor).
    double lo{-kInf};
    double hi{kInf};
    std::function<double&(P&)> ref;   ///< for Length/Angle/Seconds/Factor
    std::function<int&(P&)> iref;     ///< for Count
    std::function<bool(const P&)> visible;
    FromTool fromTool{FromTool::No};
};

template <class P>
struct ChoiceDef {
    std::string id;
    std::string label;
    std::string group;
    std::vector<std::string> options;
    std::function<int(const P&)> get;
    std::function<void(P&, int)> set;
    std::function<bool(const P&)> visible;
};

template <class P>
struct Table {
    std::vector<NumDef<P>> nums;
    std::vector<ChoiceDef<P>> choices;

    using Dref = std::function<double&(P&)>;
    using Iref = std::function<int&(P&)>;
    using Vis = std::function<bool(const P&)>;

    NumDef<P>& num(std::string id, std::string label, std::string group, Kind kind, bool neg, bool zero, Dref ref,
                   Vis vis = {}) {
        NumDef<P> d;
        d.id = std::move(id);
        d.label = std::move(label);
        d.group = std::move(group);
        d.kind = kind;
        d.neg = neg;
        d.zero = zero;
        d.ref = std::move(ref);
        d.visible = std::move(vis);
        nums.push_back(std::move(d));
        return nums.back();
    }
    NumDef<P>& count(std::string id, std::string label, std::string group, double lo, double hi, Iref ref, Vis vis = {}) {
        NumDef<P> d;
        d.id = std::move(id);
        d.label = std::move(label);
        d.group = std::move(group);
        d.kind = Kind::Count;
        d.lo = lo;
        d.hi = hi;
        d.iref = std::move(ref);
        d.visible = std::move(vis);
        nums.push_back(std::move(d));
        return nums.back();
    }
    void flag(std::string id, std::string label, std::string group, std::function<bool&(P&)> ref, Vis vis = {}) {
        ChoiceDef<P> c;
        c.id = std::move(id);
        c.label = std::move(label);
        c.group = std::move(group);
        c.options = {"No", "Yes"};
        c.get = [ref](const P& p) { return ref(const_cast<P&>(p)) ? 1 : 0; };
        c.set = [ref](P& p, int i) { ref(p) = i != 0; };
        c.visible = std::move(vis);
        choices.push_back(std::move(c));
    }
    template <class E>
    void option(std::string id, std::string label, std::string group, std::vector<std::string> options,
                std::function<E&(P&)> ref, Vis vis = {}) {
        ChoiceDef<P> c;
        c.id = std::move(id);
        c.label = std::move(label);
        c.group = std::move(group);
        c.options = std::move(options);
        c.get = [ref](const P& p) { return static_cast<int>(ref(const_cast<P&>(p))); };
        c.set = [ref](P& p, int i) { ref(p) = static_cast<E>(i); };
        c.visible = std::move(vis);
        choices.push_back(std::move(c));
    }
};

// --- Shared pieces -------------------------------------------------------------------------------

template <class P>
void addOrder(Table<P>& t, std::function<toolpath::OrderOptions&(P&)> order) {
    t.flag("orderSelection", "Vector selection order", "Order", [order](P& p) -> bool& { return order(p).selectionOrder; });
    t.flag("orderLeftToRight", "Left to right", "Order", [order](P& p) -> bool& { return order(p).leftToRight; });
    t.flag("orderBottomToTop", "Bottom to top", "Order", [order](P& p) -> bool& { return order(p).bottomToTop; });
    t.flag("orderGrid", "Grid", "Order", [order](P& p) -> bool& { return order(p).grid; });
}

template <class P>
void addStartPosition(Table<P>& t, std::function<geometry::Vec2&(P&)> pos, std::function<bool(const P&)> vis = {}) {
    t.num("startX", "Start position X", "Start", Kind::Length, true, true, [pos](P& p) -> double& { return pos(p).x; }, vis);
    t.num("startY", "Start position Y", "Start", Kind::Length, true, true, [pos](P& p) -> double& { return pos(p).y; }, vis);
}

template <class P>
void addChordal(Table<P>& t, std::function<geometry::Tolerance&(P&)> tol, double lo) {
    NumDef<P>& d = t.num("chordal", "Chordal tolerance", "Tolerance", Kind::Length, false, false,
                         [tol](P& p) -> double& { return tol(p).chordal; });
    d.lo = lo;
    d.hi = 1.0;
}

// --- Tables --------------------------------------------------------------------------------------

Table<toolpath::ProfileParams> makeProfileTable() {
    using P = toolpath::ProfileParams;
    Table<P> t;
    t.num("startDepth", "Start depth", "Depth", Kind::Length, false, true, [](P& p) -> double& { return p.passes.startDepth; });
    t.num("cutDepth", "Cut depth", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.passes.cutDepth; });
    t.num("passDepth", "Pass depth", "Depth", Kind::Length, false, true, [](P& p) -> double& { return p.passes.passDepth; })
        .fromTool = FromTool::PassDepth;
    t.num("lastPassThickness", "Last pass thickness", "Depth", Kind::Length, false, true,
          [](P& p) -> double& { return p.passes.lastPassThickness; });
    t.num("safeZ", "Safe Z", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.safeZ; });
    t.num("allowance", "Allowance offset", "Cut", Kind::Length, true, true, [](P& p) -> double& { return p.allowance; });
    t.num("lastPassAllowance", "Last pass allowance", "Cut", Kind::Length, false, true,
          [](P& p) -> double& { return p.lastPassAllowance; }, [](const P& p) { return p.separateLastPass; });
    t.num("tabLength", "Tab length", "Tabs", Kind::Length, false, false, [](P& p) -> double& { return p.tabs.length; },
          [](const P& p) { return p.tabs.enabled; });
    t.num("tabThickness", "Tab thickness", "Tabs", Kind::Length, false, false,
          [](P& p) -> double& { return p.tabs.thickness; }, [](const P& p) { return p.tabs.enabled; });
    t.count("tabCount", "Number of tabs", "Tabs", 1, 1000, [](P& p) -> int& { return p.tabs.count; },
            [](const P& p) { return p.tabs.enabled && p.tabs.spacing == toolpath::TabSpacing::ConstantNumber; });
    t.num("tabDistance", "Distance between tabs", "Tabs", Kind::Length, false, false,
          [](P& p) -> double& { return p.tabs.distance; },
          [](const P& p) { return p.tabs.enabled && p.tabs.spacing == toolpath::TabSpacing::ConstantDistance; });
    t.num("rampDistance", "Ramp distance", "Ramp", Kind::Length, false, false, [](P& p) -> double& { return p.ramp.distance; },
          [](const P& p) { return p.ramp.type != toolpath::RampType::None; });
    {
        NumDef<P>& d = t.num("rampAngle", "Ramp angle", "Ramp", Kind::Angle, false, false,
                             [](P& p) -> double& { return p.ramp.angle; },
                             [](const P& p) { return p.ramp.type != toolpath::RampType::None && p.ramp.useAngle; });
        d.hi = 90.0;
    }
    t.num("leadLength", "Lead length", "Leads", Kind::Length, false, false, [](P& p) -> double& { return p.leads.length; },
          [](const P& p) { return p.leads.type == toolpath::LeadType::Straight; });
    {
        NumDef<P>& d = t.num("leadAngle", "Lead angle", "Leads", Kind::Angle, false, true,
                             [](P& p) -> double& { return p.leads.angle; },
                             [](const P& p) { return p.leads.type == toolpath::LeadType::Straight; });
        d.hi = 90.0;
    }
    t.num("leadRadius", "Lead radius", "Leads", Kind::Length, false, false, [](P& p) -> double& { return p.leads.radius; },
          [](const P& p) { return p.leads.type == toolpath::LeadType::Circular; });
    {
        NumDef<P>& d = t.num("leadSweep", "Lead sweep", "Leads", Kind::Angle, false, false,
                             [](P& p) -> double& { return p.leads.sweep; },
                             [](const P& p) { return p.leads.type == toolpath::LeadType::Circular; });
        d.lo = 0.1;
        d.hi = 90.0;
    }
    t.num("overcut", "Overcut distance", "Leads", Kind::Length, false, true, [](P& p) -> double& { return p.leads.overcut; },
          [](const P& p) { return p.leads.type != toolpath::LeadType::None; });
    addStartPosition<P>(t, [](P& p) -> geometry::Vec2& { return p.startPosition; });
    {
        NumDef<P>& d = t.num("mitreLimit", "Mitre limit", "Corners", Kind::Factor, false, false,
                             [](P& p) -> double& { return p.corners.mitreLimit; },
                             [](const P& p) { return p.corners.sharpExternal; });
        d.hi = 1000.0;
    }
    addChordal<P>(t, [](P& p) -> geometry::Tolerance& { return p.tolerance; }, 1e-4);

    t.option<toolpath::ProfileSide>("side", "Machine vectors", "Cut", {"Outside / right", "Inside / left", "On"},
                                    [](P& p) -> toolpath::ProfileSide& { return p.side; });
    t.option<CutDirection>("direction", "Direction", "Cut", {"Climb", "Conventional"},
                           [](P& p) -> CutDirection& { return p.direction; });
    t.flag("maintainExactStep", "Maintain exact pass depth", "Depth", [](P& p) -> bool& { return p.passes.maintainExactStep; });
    t.flag("separateLastPass", "Separate last pass", "Cut", [](P& p) -> bool& { return p.separateLastPass; });
    t.flag("lastPassReverse", "Reverse direction of the last pass", "Cut", [](P& p) -> bool& { return p.lastPassReverse; },
           [](const P& p) { return p.separateLastPass; });
    t.flag("tabsEnabled", "Add tabs", "Tabs", [](P& p) -> bool& { return p.tabs.enabled; });
    t.option<toolpath::TabSpacing>("tabSpacing", "Tab spacing", "Tabs", {"Constant number", "Constant distance"},
                                   [](P& p) -> toolpath::TabSpacing& { return p.tabs.spacing; },
                                   [](const P& p) { return p.tabs.enabled; });
    t.flag("tabAvoidCorners", "Avoid corners and curves", "Tabs", [](P& p) -> bool& { return p.tabs.avoidCorners; },
           [](const P& p) { return p.tabs.enabled; });
    t.flag("tab3D", "3D tabs", "Tabs", [](P& p) -> bool& { return p.tabs.threeD; }, [](const P& p) { return p.tabs.enabled; });
    t.option<toolpath::RampType>("rampType", "Ramp", "Ramp", {"None", "Smooth", "Zig zag", "Spiral"},
                                 [](P& p) -> toolpath::RampType& { return p.ramp.type; });
    t.flag("rampUseAngle", "Ramp by angle", "Ramp", [](P& p) -> bool& { return p.ramp.useAngle; },
           [](const P& p) { return p.ramp.type == toolpath::RampType::Smooth || p.ramp.type == toolpath::RampType::ZigZag; });
    t.flag("rampOnLeadIn", "Ramp on lead in", "Ramp", [](P& p) -> bool& { return p.ramp.onLeadIn; },
           [](const P& p) { return p.ramp.type != toolpath::RampType::None; });
    t.option<toolpath::LeadType>("leadType", "Leads", "Leads", {"None", "Straight line", "Circular"},
                                 [](P& p) -> toolpath::LeadType& { return p.leads.type; });
    t.flag("leadOut", "Add lead out", "Leads", [](P& p) -> bool& { return p.leads.leadOut; },
           [](const P& p) { return p.leads.type != toolpath::LeadType::None; });
    addOrder<P>(t, [](P& p) -> toolpath::OrderOptions& { return p.order; });
    t.option<toolpath::StartAt>("startAt", "Start at", "Start", {"Vector start points", "Optimise start points", "Closest on bounding box"},
                                [](P& p) -> toolpath::StartAt& { return p.startAt; });
    t.option<toolpath::BoxAnchor>("startAnchor", "Bounding box anchor", "Start",
                                  {"Bottom left", "Bottom centre", "Bottom right", "Centre left", "Centre", "Centre right",
                                   "Top left", "Top centre", "Top right"},
                                  [](P& p) -> toolpath::BoxAnchor& { return p.startAnchor; },
                                  [](const P& p) { return p.startAt == toolpath::StartAt::ClosestOnBoundingBox; });
    t.flag("sharpExternal", "Sharp external corners", "Corners", [](P& p) -> bool& { return p.corners.sharpExternal; });
    t.flag("sharpInternal", "Sharp internal corners", "Corners", [](P& p) -> bool& { return p.corners.sharpInternal; });
    return t;
}

Table<toolpath::PocketParams> makePocketTable() {
    using P = toolpath::PocketParams;
    Table<P> t;
    t.num("startDepth", "Start depth", "Depth", Kind::Length, false, true, [](P& p) -> double& { return p.startDepth; });
    t.num("cutDepth", "Cut depth", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.cutDepth; });
    t.num("lastPassThickness", "Last pass thickness", "Depth", Kind::Length, false, true,
          [](P& p) -> double& { return p.lastPassThickness; });
    t.num("safeZ", "Safe Z", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.safeZ; });
    t.num("allowance", "Pocket allowance", "Cut", Kind::Length, false, true, [](P& p) -> double& { return p.allowance; });
    t.num("rasterAngle", "Raster angle", "Cut", Kind::Angle, true, true, [](P& p) -> double& { return p.rasterAngle; },
          [](const P& p) { return p.strategy == toolpath::PocketStrategy::Raster; });
    t.num("rampDistance", "Ramp distance", "Ramp", Kind::Length, false, false, [](P& p) -> double& { return p.rampDistance; },
          [](const P& p) { return p.rampPlunge; });
    addChordal<P>(t, [](P& p) -> geometry::Tolerance& { return p.tolerance; }, 1e-4);
    t.option<toolpath::PocketStrategy>("strategy", "Strategy", "Cut", {"Offset", "Raster"},
                                       [](P& p) -> toolpath::PocketStrategy& { return p.strategy; });
    t.option<CutDirection>("direction", "Direction", "Cut", {"Climb", "Conventional"},
                           [](P& p) -> CutDirection& { return p.direction; });
    t.option<toolpath::ProfilePass>("profilePass", "Profile pass", "Cut", {"None", "First", "Last"},
                                    [](P& p) -> toolpath::ProfilePass& { return p.profilePass; });
    t.flag("maintainExactStep", "Maintain exact pass depth", "Depth", [](P& p) -> bool& { return p.maintainExactStep; });
    t.flag("rampPlunge", "Ramp plunge moves", "Ramp", [](P& p) -> bool& { return p.rampPlunge; });
    return t;
}

Table<toolpath::DrillParams> makeDrillTable() {
    using P = toolpath::DrillParams;
    Table<P> t;
    t.num("startDepth", "Start depth", "Depth", Kind::Length, false, true, [](P& p) -> double& { return p.startDepth; });
    t.num("cutDepth", "Cut depth", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.cutDepth; });
    t.num("safeZ", "Safe Z", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.safeZ; });
    t.num("retractGap", "Retract gap", "Peck", Kind::Length, false, false, [](P& p) -> double& { return p.retractGap; },
          [](const P& p) { return p.peck; });
    t.num("dwellTime", "Dwell time", "Peck", Kind::Seconds, false, true, [](P& p) -> double& { return p.dwellTime; });
    addStartPosition<P>(t, [](P& p) -> geometry::Vec2& { return p.startPosition; });
    {
        NumDef<P>& d = t.num("distance", "Merge holes closer than", "Tolerance", Kind::Length, false, false,
                             [](P& p) -> double& { return p.tolerance.distance; });
        d.lo = 1e-6;
        d.hi = 1.0;
    }
    addChordal<P>(t, [](P& p) -> geometry::Tolerance& { return p.tolerance; }, 1e-4);
    t.flag("peck", "Peck drilling", "Peck", [](P& p) -> bool& { return p.peck; });
    t.option<toolpath::PeckRetract>("peckRetract", "Retract between passes", "Peck", {"Above start", "Above previous pass"},
                                    [](P& p) -> toolpath::PeckRetract& { return p.peckRetract; },
                                    [](const P& p) { return p.peck; });
    addOrder<P>(t, [](P& p) -> toolpath::OrderOptions& { return p.order; });
    return t;
}

Table<toolpath::EngraveParams> makeEngraveTable() {
    using P = toolpath::EngraveParams;
    Table<P> t;
    t.num("depth", "Engrave depth", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.depth; });
    t.count("passes", "Passes", "Depth", 1, 1000, [](P& p) -> int& { return p.passes; });
    t.num("safeZ", "Safe Z", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.safeZ; });
    t.num("stepover", "Fill stepover", "Fill", Kind::Length, false, true, [](P& p) -> double& { return p.stepover; },
          [](const P& p) { return p.strategy == toolpath::EngraveStrategy::Fill; })
        .fromTool = FromTool::Stepover;
    t.num("rasterAngle", "Raster angle", "Fill", Kind::Angle, true, true, [](P& p) -> double& { return p.rasterAngle; },
          [](const P& p) {
              return p.strategy == toolpath::EngraveStrategy::Fill && p.fillPattern == toolpath::PocketStrategy::Raster;
          });
    addStartPosition<P>(t, [](P& p) -> geometry::Vec2& { return p.startPosition; },
                        [](const P& p) { return p.strategy == toolpath::EngraveStrategy::Outline; });
    addChordal<P>(t, [](P& p) -> geometry::Tolerance& { return p.tolerance; }, 1e-4);
    t.option<toolpath::EngraveStrategy>("strategy", "Strategy", "Cut", {"Outline", "Fill"},
                                        [](P& p) -> toolpath::EngraveStrategy& { return p.strategy; });
    t.option<toolpath::PocketStrategy>("fillPattern", "Fill pattern", "Fill", {"Offset", "Raster"},
                                       [](P& p) -> toolpath::PocketStrategy& { return p.fillPattern; },
                                       [](const P& p) { return p.strategy == toolpath::EngraveStrategy::Fill; });
    return t;
}

Table<toolpath::VCarvingParams> makeVCarvingTable() {
    using P = toolpath::VCarvingParams;
    Table<P> t;
    t.num("startDepth", "Start depth", "Depth", Kind::Length, false, true, [](P& p) -> double& { return p.startDepth; });
    t.num("flatDepth", "Flat depth", "Depth", Kind::Length, false, true, [](P& p) -> double& { return p.flatDepth; });
    t.num("safeZ", "Safe Z", "Depth", Kind::Length, false, false, [](P& p) -> double& { return p.safeZ; });
    t.num("clearanceRasterAngle", "Clearance raster angle", "Clearance", Kind::Angle, true, true,
          [](P& p) -> double& { return p.clearanceRasterAngle; },
          [](const P& p) { return p.clearanceStrategy == toolpath::PocketStrategy::Raster; });
    t.num("rampDistance", "Ramp distance", "Clearance", Kind::Length, false, false,
          [](P& p) -> double& { return p.rampDistance; }, [](const P& p) { return p.rampPlunge; });
    addStartPosition<P>(t, [](P& p) -> geometry::Vec2& { return p.startPosition; });
    addChordal<P>(t, [](P& p) -> geometry::Tolerance& { return p.tolerance; }, 3e-4);
    t.option<toolpath::PocketStrategy>("clearanceStrategy", "Clearance strategy", "Clearance", {"Offset", "Raster"},
                                       [](P& p) -> toolpath::PocketStrategy& { return p.clearanceStrategy; });
    t.option<CutDirection>("clearanceDirection", "Clearance direction", "Clearance", {"Climb", "Conventional"},
                           [](P& p) -> CutDirection& { return p.clearanceDirection; });
    t.flag("rampPlunge", "Ramp plunge moves", "Clearance", [](P& p) -> bool& { return p.rampPlunge; });
    t.flag("cornerSharpen", "Corner sharpen", "Clearance", [](P& p) -> bool& { return p.cornerSharpen; });
    addOrder<P>(t, [](P& p) -> toolpath::OrderOptions& { return p.order; });
    return t;
}

template <class P>
const Table<P>& tableOf();
template <>
const Table<toolpath::ProfileParams>& tableOf() {
    static const Table<toolpath::ProfileParams> t = makeProfileTable();
    return t;
}
template <>
const Table<toolpath::PocketParams>& tableOf() {
    static const Table<toolpath::PocketParams> t = makePocketTable();
    return t;
}
template <>
const Table<toolpath::DrillParams>& tableOf() {
    static const Table<toolpath::DrillParams> t = makeDrillTable();
    return t;
}
template <>
const Table<toolpath::EngraveParams>& tableOf() {
    static const Table<toolpath::EngraveParams> t = makeEngraveTable();
    return t;
}
template <>
const Table<toolpath::VCarvingParams>& tableOf() {
    static const Table<toolpath::VCarvingParams> t = makeVCarvingTable();
    return t;
}

// --- Values --------------------------------------------------------------------------------------

std::string trimmed(const std::string& s) {
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) {
        return {};
    }
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

std::string trimZeros(std::string s) {
    if (s.find('.') != std::string::npos) {
        while (!s.empty() && s.back() == '0') {
            s.pop_back();
        }
        if (!s.empty() && s.back() == '.') {
            s.pop_back();
        }
    }
    return s.empty() || s == "-0" ? "0" : s;
}

std::string lengthText(double mm, Units units) {
    return trimZeros(formatFixed(document::toDisplay(mm, units), units == Units::Inches ? 5 : 4));
}

std::string valueText(Kind kind, double value, Units units) {
    switch (kind) {
        case Kind::Length: return lengthText(value, units);
        case Kind::Angle: return trimZeros(formatFixed(value / kRadPerDeg, 4));
        case Kind::Count: return trimZeros(formatFixed(value, 0));
        case Kind::Seconds:
        case Kind::Factor: return trimZeros(formatFixed(value, 4));
    }
    return {};
}

/// A plain decimal number (no unit suffix, no inf/nan/hex), as `parseNumber` accepts for lengths.
ParsedNumber parsePlain(const std::string& text) {
    ParsedNumber out;
    std::string t = trimmed(text);
    if (!t.empty() && t.front() == '+') {
        t.erase(0, 1);
    }
    if (t.empty() || t.size() > 64) {
        out.error = t.empty() ? "enter a number" : "not a number";
        return out;
    }
    const char first = t.front();
    if (!((first >= '0' && first <= '9') || first == '.' || first == '-')) {
        out.error = "not a number";
        return out;
    }
    double v = 0.0;
    const auto res = std::from_chars(t.data(), t.data() + t.size(), v);
    if (res.ec != std::errc() || res.ptr != t.data() + t.size() || !std::isfinite(v)) {
        out.error = "not a number";
        return out;
    }
    if (std::fabs(v) > kMaxLength) {
        out.error = "value is too large";
        return out;
    }
    out.ok = true;
    out.value = v;
    return out;
}

template <class P>
double readValue(const NumDef<P>& d, const P& p) {
    P copy = p;  // the accessors take a mutable reference; the params are small
    if (d.iref) {
        return static_cast<double>(d.iref(copy));
    }
    return d.ref(copy);
}

template <class P>
bool isVisible(const NumDef<P>& d, const P& p) {
    return !d.visible || d.visible(p);
}

template <class P>
std::vector<ToolpathField> buildFields(const P& p, Units units, const ToolSummary& tool) {
    const P defaults{};
    std::vector<ToolpathField> out;
    for (const NumDef<P>& d : tableOf<P>().nums) {
        if (!isVisible(d, p)) {
            continue;
        }
        ToolpathField f;
        f.id = d.id;
        f.label = d.label;
        f.group = d.group;
        f.kind = d.kind;
        f.allowNegative = d.neg;
        f.allowZero = d.zero;
        const double value = readValue(d, p);
        f.typed = value != readValue(d, defaults);
        f.fromTool = d.fromTool != FromTool::No;
        if (f.fromTool && value == 0.0 && tool.valid) {
            f.text = lengthText(d.fromTool == FromTool::PassDepth ? tool.data.passDepth : tool.data.stepover, units);
        } else {
            f.text = valueText(d.kind, value, units);
        }
        out.push_back(std::move(f));
    }
    return out;
}

template <class P>
std::vector<ToolpathChoice> buildChoices(const P& p) {
    std::vector<ToolpathChoice> out;
    for (const ChoiceDef<P>& c : tableOf<P>().choices) {
        if (c.visible && !c.visible(p)) {
            continue;
        }
        ToolpathChoice ch;
        ch.id = c.id;
        ch.label = c.label;
        ch.group = c.group;
        ch.options = c.options;
        ch.current = c.get(p);
        ch.isBool = c.options.size() == 2 && c.options[0] == "No" && c.options[1] == "Yes";
        out.push_back(std::move(ch));
    }
    return out;
}

template <class P>
FieldResult storeField(P& p, const std::string& id, const std::string& text, Units units) {
    FieldResult res;
    const auto fail = [&](std::string message) {
        res.ok = false;
        res.message = std::move(message);
        return res;
    };
    const NumDef<P>* def = nullptr;
    for (const NumDef<P>& d : tableOf<P>().nums) {
        if (d.id == id) {
            def = &d;
        }
    }
    if (def == nullptr) {
        return fail("unknown field");
    }
    const std::string t = trimmed(text);
    if (t.empty()) {  // reset to the default
        P defaults{};
        if (def->iref) {
            def->iref(p) = def->iref(defaults);
        } else {
            def->ref(p) = def->ref(defaults);
        }
        return res;
    }
    ParsedNumber n;
    switch (def->kind) {
        case Kind::Length: n = parseNumber(t, FieldKind::Length, units); break;
        case Kind::Angle: n = parseNumber(t, FieldKind::Angle, units); break;
        case Kind::Count: n = parseNumber(t, FieldKind::Count, units); break;
        case Kind::Seconds:
        case Kind::Factor: n = parsePlain(t); break;
    }
    if (!n.ok) {
        return fail(n.error);
    }
    if (!def->neg && n.value < 0.0) {
        return fail("must not be negative");
    }
    if (!def->zero && n.value == 0.0) {
        return fail("must not be zero");
    }
    if (n.value < def->lo || n.value > def->hi) {
        const auto num = [](double v) { return trimZeros(formatFixed(v, 4)); };
        const std::string unit = def->kind == Kind::Length ? " mm" : def->kind == Kind::Angle ? " degrees" : "";
        if (std::isfinite(def->lo) && std::isfinite(def->hi)) {
            return fail("must be between " + num(def->lo) + " and " + num(def->hi) + unit);
        }
        return fail(std::isfinite(def->hi) ? "must be at most " + num(def->hi) + unit
                                           : "must be at least " + num(def->lo) + unit);
    }
    if (def->kind == Kind::Angle && std::fabs(n.value) > 360000.0) {
        return fail("the angle is too large");
    }
    if (def->kind == Kind::Count) {
        def->iref(p) = static_cast<int>(n.value);
    } else if (def->kind == Kind::Angle) {
        def->ref(p) = n.value * kRadPerDeg;
    } else {
        def->ref(p) = n.value;
    }
    return res;
}

template <class P>
FieldResult storeChoice(P& p, const std::string& id, int option) {
    FieldResult res;
    for (const ChoiceDef<P>& c : tableOf<P>().choices) {
        if (c.id == id) {
            if (option < 0 || static_cast<std::size_t>(option) >= c.options.size()) {
                res.ok = false;
                res.message = "not one of the options";
                return res;
            }
            c.set(p, option);
            return res;
        }
    }
    res.ok = false;
    res.message = "unknown option";
    return res;
}

template <class P>
std::vector<std::string> numIds() {
    std::vector<std::string> ids;
    for (const NumDef<P>& d : tableOf<P>().nums) {
        ids.push_back(d.id);
    }
    return ids;
}

template <class P>
std::vector<std::string> choiceIds() {
    std::vector<std::string> ids;
    for (const ChoiceDef<P>& c : tableOf<P>().choices) {
        ids.push_back(c.id);
    }
    return ids;
}

std::string toolTypeLabel(tooling::ToolType type) { return std::string(tooling::toString(type)); }

}  // namespace

// --- ToolpathForm --------------------------------------------------------------------------------

ToolpathForm::ToolpathForm(ToolpathOperation operation, Units units)
    : definition_(defaultDefinition(operation)), units_(units) {}

ToolpathForm::ToolpathForm(const ToolpathDefinition& definition, Units units) : definition_(definition), units_(units) {
    if (definition_.operation() != ToolpathOperation::VCarving) {
        definition_.clearanceTools.clear();
    }
}

FieldResult ToolpathForm::setName(const std::string& name) {
    FieldResult res;
    const std::string bad = validateToolpathName(name);
    if (!bad.empty()) {
        res.ok = false;
        res.message = bad;
        return res;
    }
    definition_.setName(name);
    return res;
}

std::vector<ToolpathField> ToolpathForm::fields() const {
    return std::visit([&](const auto& p) { return buildFields(p, units_, toolSummary_); }, definition_.params);
}

std::vector<ToolpathChoice> ToolpathForm::choices() const {
    return std::visit([&](const auto& p) { return buildChoices(p); }, definition_.params);
}

FieldResult ToolpathForm::setField(const std::string& id, const std::string& text) {
    // Work on a copy: a rejected entry leaves the form untouched.
    return std::visit(
        [&](auto& p) {
            auto copy = p;
            FieldResult res = storeField(copy, id, text, units_);
            if (res.ok) {
                p = copy;
            }
            return res;
        },
        definition_.params);
}

FieldResult ToolpathForm::setChoice(const std::string& id, int option) {
    return std::visit(
        [&](auto& p) {
            auto copy = p;
            FieldResult res = storeChoice(copy, id, option);
            if (res.ok) {
                p = copy;
            }
            return res;
        },
        definition_.params);
}

std::vector<std::string> ToolpathForm::allFieldIds() const {
    return std::visit([](const auto& p) { return numIds<std::decay_t<decltype(p)>>(); }, definition_.params);
}

std::vector<std::string> ToolpathForm::allChoiceIds() const {
    return std::visit([](const auto& p) { return choiceIds<std::decay_t<decltype(p)>>(); }, definition_.params);
}

FieldResult ToolpathForm::checkTool(const tooling::ToolDatabase& db, const document::ToolSource& source, bool clearance,
                                    ToolSummary* summary) const {
    FieldResult res;
    const ToolLookup lookup = resolveTool(db, source);
    if (!lookup.tool.has_value()) {
        res.ok = false;
        res.message = lookup.error;
        return res;
    }
    const tooling::Tool& tool = lookup.tool->tool;
    if (!toolAllowedFor(operation(), tool.type, clearance)) {
        res.ok = false;
        res.message = "tool '" + tool.name + "' (" + toolTypeLabel(tool.type) + ") cannot be used " +
                      (clearance ? "as a clearance tool of a " : "for a ") + operationName(operation()) + " toolpath";
        return res;
    }
    if (summary != nullptr) {
        summary->valid = true;
        summary->id = tool.id;
        summary->name = tool.name;
        summary->type = tool.type;
        summary->diameterMm = tool.diameter;
        summary->data = toolpath::effectiveCuttingData(tool, lookup.tool->data);
        summary->fallback = lookup.tool->fallback;
    }
    return res;
}

FieldResult ToolpathForm::setTool(const tooling::ToolDatabase& db, const document::ToolSource& source) {
    ToolSummary summary;
    FieldResult res = checkTool(db, source, false, &summary);
    if (res.ok) {
        definition_.tool = source;
        toolSummary_ = std::move(summary);
    }
    return res;
}

FieldResult ToolpathForm::setClearanceTools(const tooling::ToolDatabase& db,
                                            const std::vector<document::ToolSource>& sources) {
    FieldResult res;
    if (operation() != ToolpathOperation::VCarving) {
        res.ok = false;
        res.message = "only a V-carve has clearance tools";
        return res;
    }
    for (std::size_t i = 0; i < sources.size(); ++i) {
        res = checkTool(db, sources[i], true, nullptr);
        if (!res.ok) {
            res.message = "clearance tool " + std::to_string(i + 1) + ": " + res.message;
            return res;
        }
    }
    definition_.clearanceTools = sources;
    return res;
}

void ToolpathForm::refreshTools(const tooling::ToolDatabase& db) {
    toolSummary_ = ToolSummary{};
    if (!definition_.tool.toolId.empty()) {
        ToolSummary summary;
        if (checkTool(db, definition_.tool, false, &summary).ok) {
            toolSummary_ = std::move(summary);
        }
    }
}

std::vector<std::string> ToolpathForm::problems() const {
    std::vector<std::string> out;
    if (const std::string bad = validateToolpathName(definition_.name()); !bad.empty()) {
        out.push_back(bad);
    }
    if (definition_.tool.toolId.empty()) {
        out.emplace_back("choose a tool");
    } else if (!toolSummary_.valid) {
        out.emplace_back("the chosen tool is not available in the tool database");
    }
    for (std::size_t i = 0; i < definition_.clearanceTools.size(); ++i) {
        if (definition_.clearanceTools[i].toolId.empty()) {
            out.push_back("clearance tool " + std::to_string(i + 1) + " has no tool chosen");
        }
    }
    bool needsDepth = false;
    double depth = 0.0;
    std::visit(
        [&](const auto& p) {
            using P = std::decay_t<decltype(p)>;
            if constexpr (std::is_same_v<P, toolpath::ProfileParams>) {
                needsDepth = true;
                depth = p.passes.cutDepth;
            } else if constexpr (std::is_same_v<P, toolpath::PocketParams> || std::is_same_v<P, toolpath::DrillParams>) {
                needsDepth = true;
                depth = p.cutDepth;
            } else if constexpr (std::is_same_v<P, toolpath::EngraveParams>) {
                needsDepth = true;
                depth = p.depth;
            }
        },
        definition_.params);
    if (needsDepth && !(depth > 0.0)) {
        out.emplace_back("set the cut depth");
    }
    if (const std::string key = document::nonFiniteParam(definition_.params); !key.empty()) {
        out.push_back("parameter " + key + " is not a finite number");
    }
    return out;
}

void ToolpathForm::resetValues() {
    const std::string name = definition_.name();
    definition_.params = defaultDefinition(operation()).params;
    definition_.setName(name);
}

}  // namespace carvetto::viewmodel
