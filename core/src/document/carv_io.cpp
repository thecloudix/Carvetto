#include "carvetto/document/carv_io.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <exception>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <ios>
#include <limits>
#include <map>
#include <memory>
#include <new>
#include <nlohmann/json.hpp>
#include <optional>
#include <set>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "carv_json_reader.hpp"
#include "carv_moves.hpp"
#include "carv_zip.hpp"
#include "carvetto/geometry/vec2.hpp"

#ifndef CARVETTO_VERSION_STRING
#define CARVETTO_VERSION_STRING "0.0.0"
#endif

namespace carvetto::document {

using json = nlohmann::json;
using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Span;
using geometry::Vec2;

// ------------------------------------------------------------------------------------------------
// Preserved unknown content
// ------------------------------------------------------------------------------------------------

namespace {
/// Unknown members of one record: key -> canonical compact JSON text of the value.
using ExtraMap = std::map<std::string, std::string>;
}  // namespace

struct CarvExtras::Impl {
    /// Unknown keys of manifest.json, the job.json root and the `job` record.
    ExtraMap manifest;
    ExtraMap root;
    ExtraMap job;
    /// Unknown keys of each sheet / layer / object record, by id.
    std::map<std::uint64_t, ExtraMap> sheets;
    std::map<std::uint64_t, ExtraMap> layers;
    std::map<std::uint64_t, ExtraMap> objects;
    /// Unknown keys of each toolpath record, by id.
    std::map<std::uint64_t, ExtraMap> toolpaths;
    /// Creation parameters of a kind this version does not know (canonical text), by object id.
    /// Loaded as `NoParams` and written back while the object still has `NoParams`.
    std::map<std::uint64_t, std::string> params;
    /// Archive members other than manifest.json and job.json.
    std::map<std::string, std::vector<std::uint8_t>> members;

    bool empty() const {
        return manifest.empty() && root.empty() && job.empty() && sheets.empty() && layers.empty() &&
               objects.empty() && toolpaths.empty() && params.empty() && members.empty();
    }
};

bool CarvExtras::empty() const {
    return !impl_ || impl_->empty();
}

std::vector<std::string> CarvExtras::memberNames() const {
    std::vector<std::string> out;
    if (impl_) {
        for (const auto& [name, data] : impl_->members) {
            out.push_back(name);
        }
    }
    return out;
}

const std::vector<std::uint8_t>* CarvExtras::member(std::string_view name) const {
    if (!impl_) {
        return nullptr;
    }
    const auto it = impl_->members.find(std::string(name));
    return it == impl_->members.end() ? nullptr : &it->second;
}

CarvExtras CarvExtras::withMember(std::string name, std::vector<std::uint8_t> bytes) const {
    const auto bad = [&](const char* why) {
        throw std::invalid_argument("invalid .carv member name '" + name + "': " + why);
    };
    if (name.empty()) {
        bad("empty");
    }
    if (name.size() > 255) {
        bad("longer than 255 bytes");
    }
    if (name == "manifest.json" || name == "job.json") {
        bad("reserved");
    }
    if (detail::isToolpathCacheName(name)) {
        bad("reserved for toolpath move caches, which saveCarv writes from the toolpath records");
    }
    if (detail::isInterimToolpathName(name)) {
        bad("an interim toolpath member (ADR-0019); toolpaths are stored as toolpath records now");
    }
    if (name.front() == '/' || name.find('\\') != std::string::npos || name.find(':') != std::string::npos) {
        bad("absolute paths, backslashes and drive letters are not allowed");
    }
    std::size_t start = 0;
    while (start <= name.size()) {
        std::size_t end = name.find('/', start);
        if (end == std::string::npos) {
            end = name.size();
        }
        const std::string_view seg(name.data() + start, end - start);
        if (seg.empty() || seg == "." || seg == "..") {
            bad("empty, '.' and '..' path segments are not allowed");
        }
        start = end + 1;
    }
    auto impl = impl_ ? std::make_shared<Impl>(*impl_) : std::make_shared<Impl>();
    impl->members[std::move(name)] = std::move(bytes);
    return CarvExtras(std::move(impl));
}

namespace {

constexpr const char* kManifestName = "manifest.json";
constexpr const char* kJobName = "job.json";
constexpr const char* kFormatName = "carvetto";

// ------------------------------------------------------------------------------------------------
// Canonical JSON text: sorted keys (nlohmann's default std::map objects), 2-space indent, arrays of
// scalars on one line, shortest round-trip doubles that always carry a '.' or exponent so they
// re-parse as doubles (-0.0 stays -0.0), and non-finite doubles as the strings "NaN", "Infinity",
// "-Infinity".
// ------------------------------------------------------------------------------------------------

void writeDouble(double v, std::string& out) {
    detail::appendDouble(out, v);
}

void writeString(const std::string& s, std::string& out) {
    // Invalid UTF-8 (only possible from in-memory strings, never from a parsed file) is replaced
    // by U+FFFD rather than failing the save.
    detail::appendJsonString(out, s);
}

/// A preserved unknown value: canonical JSON text, carried as a json binary value and written
/// verbatim by `writeJson`.
json rawJson(const std::string& text) {
    return json::binary(json::binary_t::container_type(text.begin(), text.end()));
}

bool isScalar(const json& j) {
    return !j.is_array() && !j.is_object();
}

/// True for an object whose values are all scalars or arrays of scalars.
bool isFlat(const json& obj) {
    for (const auto& [key, value] : obj.items()) {
        if (value.is_object()) {
            return false;
        }
        if (value.is_array()) {
            for (const json& e : value) {
                if (!isScalar(e)) {
                    return false;
                }
            }
        }
    }
    return true;
}

void writeIndent(int indent, std::string& out) {
    out.append(static_cast<std::size_t>(indent), ' ');
}

void writeJson(const json& j, int indent, std::string& out) {
    switch (j.type()) {
        case json::value_t::binary: {
            const auto& raw = j.get_binary();
            out.append(raw.begin(), raw.end());
            return;
        }
        case json::value_t::null:
        case json::value_t::discarded:
            out += "null";
            return;
        case json::value_t::boolean:
            out += j.get<bool>() ? "true" : "false";
            return;
        case json::value_t::number_integer:
            out += std::to_string(j.get<std::int64_t>());
            return;
        case json::value_t::number_unsigned:
            out += std::to_string(j.get<std::uint64_t>());
            return;
        case json::value_t::number_float:
            writeDouble(j.get<double>(), out);
            return;
        case json::value_t::string:
            writeString(j.get_ref<const std::string&>(), out);
            return;
        case json::value_t::array: {
            if (j.empty()) {
                out += "[]";
                return;
            }
            bool allScalar = true;
            for (const json& e : j) {
                allScalar = allScalar && isScalar(e);
            }
            if (allScalar) {
                out += '[';
                bool first = true;
                for (const json& e : j) {
                    if (!first) {
                        out += ", ";
                    }
                    first = false;
                    writeJson(e, indent, out);
                }
                out += ']';
                return;
            }
            out += "[\n";
            bool first = true;
            for (const json& e : j) {
                if (!first) {
                    out += ",\n";
                }
                first = false;
                writeIndent(indent + 2, out);
                writeJson(e, indent + 2, out);
            }
            out += '\n';
            writeIndent(indent, out);
            out += ']';
            return;
        }
        case json::value_t::object: {
            if (j.empty()) {
                out += "{}";
                return;
            }
            if (isFlat(j)) {
                // One line per leaf record (span, params, layer, sheet, job): compact and still
                // one entity per line for diffs.
                out += '{';
                bool first = true;
                for (const auto& [key, value] : j.items()) {
                    if (!first) {
                        out += ", ";
                    }
                    first = false;
                    writeString(key, out);
                    out += ": ";
                    writeJson(value, indent, out);
                }
                out += '}';
                return;
            }
            out += "{\n";
            bool first = true;
            for (const auto& [key, value] : j.items()) {
                if (!first) {
                    out += ",\n";
                }
                first = false;
                writeIndent(indent + 2, out);
                writeString(key, out);
                out += ": ";
                writeJson(value, indent + 2, out);
            }
            out += '\n';
            writeIndent(indent, out);
            out += '}';
            return;
        }
    }
}

std::vector<std::uint8_t> canonicalBytes(const json& j) {
    std::string text;
    writeJson(j, 0, text);
    text += '\n';
    return {text.begin(), text.end()};
}

// ------------------------------------------------------------------------------------------------
// Enum names
// ------------------------------------------------------------------------------------------------

template <class E>
struct EnumName {
    E value;
    const char* name;
};

constexpr EnumName<JobType> kJobTypes[] = {{JobType::SingleSided, "single_sided"},
                                           {JobType::DoubleSided, "double_sided"},
                                           {JobType::Rotary, "rotary"}};
constexpr EnumName<Units> kUnits[] = {{Units::Millimetres, "mm"}, {Units::Inches, "in"}};
constexpr EnumName<ZZero> kZZeros[] = {{ZZero::MaterialSurface, "material_surface"},
                                       {ZZero::MachineBed, "machine_bed"},
                                       {ZZero::CylinderAxis, "cylinder_axis"}};
constexpr EnumName<Datum> kDatums[] = {{Datum::BottomLeft, "bottom_left"},
                                       {Datum::BottomRight, "bottom_right"},
                                       {Datum::TopLeft, "top_left"},
                                       {Datum::TopRight, "top_right"},
                                       {Datum::Centre, "centre"}};
constexpr EnumName<FlipDirection> kFlips[] = {{FlipDirection::Horizontal, "horizontal"},
                                              {FlipDirection::Vertical, "vertical"}};
constexpr EnumName<RotaryAxis> kAxes[] = {{RotaryAxis::X, "x"}, {RotaryAxis::Y, "y"}};
constexpr EnumName<ArcDirection> kArcDirs[] = {{ArcDirection::CCW, "ccw"}, {ArcDirection::CW, "cw"}};
constexpr EnumName<RectangleParams::CornerStyle> kCorners[] = {
    {RectangleParams::CornerStyle::Square, "square"},
    {RectangleParams::CornerStyle::Rounded, "rounded"},
    {RectangleParams::CornerStyle::Chamfered, "chamfered"}};

// Toolpath parameter enums (ADR-0021: the params.json names of T-0122, ADR-0019). Aliases come
// after the canonical name, which is the one written.
constexpr EnumName<toolpath::ProfileSide> kProfileSides[] = {{toolpath::ProfileSide::Outside, "outside"},
                                                            {toolpath::ProfileSide::Inside, "inside"},
                                                            {toolpath::ProfileSide::On, "on"},
                                                            {toolpath::ProfileSide::Right, "right"},
                                                            {toolpath::ProfileSide::Left, "left"}};
constexpr EnumName<toolpath::CutDirection> kCutDirections[] = {{toolpath::CutDirection::Climb, "climb"},
                                                              {toolpath::CutDirection::Conventional, "conventional"}};
constexpr EnumName<toolpath::TabSpacing> kTabSpacings[] = {
    {toolpath::TabSpacing::ConstantNumber, "constant_number"},
    {toolpath::TabSpacing::ConstantDistance, "constant_distance"}};
constexpr EnumName<toolpath::RampType> kRampTypes[] = {{toolpath::RampType::None, "none"},
                                                      {toolpath::RampType::Smooth, "smooth"},
                                                      {toolpath::RampType::ZigZag, "zig_zag"},
                                                      {toolpath::RampType::Spiral, "spiral"}};
constexpr EnumName<toolpath::LeadType> kLeadTypes[] = {{toolpath::LeadType::None, "none"},
                                                      {toolpath::LeadType::Straight, "straight"},
                                                      {toolpath::LeadType::Circular, "circular"}};
constexpr EnumName<toolpath::StartAt> kStartAts[] = {{toolpath::StartAt::VectorStart, "vector_start"},
                                                    {toolpath::StartAt::Optimise, "optimise"},
                                                    {toolpath::StartAt::ClosestOnBoundingBox, "closest_on_bounding_box"}};
constexpr EnumName<toolpath::BoxAnchor> kBoxAnchors[] = {{toolpath::BoxAnchor::BottomLeft, "bottom_left"},
                                                        {toolpath::BoxAnchor::BottomCentre, "bottom_centre"},
                                                        {toolpath::BoxAnchor::BottomRight, "bottom_right"},
                                                        {toolpath::BoxAnchor::CentreLeft, "centre_left"},
                                                        {toolpath::BoxAnchor::Centre, "centre"},
                                                        {toolpath::BoxAnchor::CentreRight, "centre_right"},
                                                        {toolpath::BoxAnchor::TopLeft, "top_left"},
                                                        {toolpath::BoxAnchor::TopCentre, "top_centre"},
                                                        {toolpath::BoxAnchor::TopRight, "top_right"}};
constexpr EnumName<toolpath::PocketStrategy> kPocketStrategies[] = {
    {toolpath::PocketStrategy::Offset, "offset"}, {toolpath::PocketStrategy::Raster, "raster"}};
constexpr EnumName<toolpath::ProfilePass> kProfilePasses[] = {{toolpath::ProfilePass::None, "none"},
                                                             {toolpath::ProfilePass::First, "first"},
                                                             {toolpath::ProfilePass::Last, "last"}};
constexpr EnumName<ToolpathOperation> kOperations[] = {{ToolpathOperation::Profile, "profile"},
                                                      {ToolpathOperation::Pocket, "pocket"}};

template <class E, std::size_t N>
const char* nameOf(const EnumName<E> (&table)[N], E value) {
    for (const auto& item : table) {
        if (item.value == value) {
            return item.name;
        }
    }
    return "invalid";  // only reachable for an out-of-range value cast into the enum
}

// ------------------------------------------------------------------------------------------------
// Encoding
// ------------------------------------------------------------------------------------------------

json encodePoint(Vec2 p) {
    return json::array({p.x, p.y});
}

json encodeSize(const JobSize& s) {
    return json::array({s.x, s.y, s.z});
}

std::string encodeColour(Colour c) {
    constexpr char kHex[] = "0123456789abcdef";
    std::string s = "#";
    for (const std::uint8_t v : {c.r, c.g, c.b}) {
        s += kHex[v >> 4];
        s += kHex[v & 0x0F];
    }
    return s;
}

void mergeUnknown(json& target, const ExtraMap* extra) {
    if (!extra) {
        return;
    }
    for (const auto& [key, text] : *extra) {
        if (!target.contains(key)) {
            target[key] = rawJson(text);
        }
    }
}

const ExtraMap* findExtra(const CarvExtras::Impl* impl,
                          std::map<std::uint64_t, ExtraMap> CarvExtras::Impl::*member, std::uint64_t id) {
    if (!impl) {
        return nullptr;
    }
    const auto& map = impl->*member;
    const auto it = map.find(id);
    return it == map.end() ? nullptr : &it->second;
}

json encodeJob(const Job& job) {
    json j = json::object();
    j["type"] = nameOf(kJobTypes, job.type);
    j["size_mm"] = encodeSize(job.sizeMm);
    j["display_units"] = nameOf(kUnits, job.displayUnits);
    j["z_zero"] = nameOf(kZZeros, job.zZero);
    j["datum"] = nameOf(kDatums, job.datum);
    j["datum_offset_mm"] = encodePoint(job.datumOffsetMm);
    j["flip_direction"] = nameOf(kFlips, job.flipDirection);
    j["zero_off_same_side"] = job.zeroOffSameSide;
    j["diameter_mm"] = job.diameterMm;
    j["rotary_axis"] = nameOf(kAxes, job.rotaryAxis);
    j["flip_design"] = job.flipDesign;
    return j;
}

json encodeSheet(const Sheet& s) {
    json j = json::object();
    j["id"] = s.id.value;
    j["name"] = s.name;
    j["size_mm"] = encodeSize(s.sizeMm);
    j["z_zero"] = nameOf(kZZeros, s.zZero);
    j["datum"] = nameOf(kDatums, s.datum);
    j["datum_offset_mm"] = encodePoint(s.datumOffsetMm);
    j["visible"] = s.visible;
    return j;
}

json encodeLayer(const Layer& l) {
    json j = json::object();
    j["id"] = l.id.value;
    j["name"] = l.name;
    j["colour"] = encodeColour(l.colour);
    j["visible"] = l.visible;
    j["locked"] = l.locked;
    return j;
}

json encodeSpan(const Span& span) {
    json j = json::object();
    if (const auto* line = std::get_if<LineSpan>(&span)) {
        j["type"] = "line";
        j["start"] = encodePoint(line->start);
        j["end"] = encodePoint(line->end);
    } else if (const auto* arc = std::get_if<ArcSpan>(&span)) {
        j["type"] = "arc";
        j["centre"] = encodePoint(arc->centre);
        j["radius"] = arc->radius;
        j["start_angle"] = arc->startAngle;
        j["end_angle"] = arc->endAngle;
        j["direction"] = nameOf(kArcDirs, arc->direction);
    } else {
        const auto& bez = std::get<BezierSpan>(span);
        j["type"] = "bezier";
        j["p0"] = encodePoint(bez.p0);
        j["p1"] = encodePoint(bez.p1);
        j["p2"] = encodePoint(bez.p2);
        j["p3"] = encodePoint(bez.p3);
    }
    return j;
}

json encodeContour(const Contour& c) {
    json j = json::object();
    j["closed"] = c.closed;
    json spans = json::array();
    for (const Span& s : c.spans) {
        spans.push_back(encodeSpan(s));
    }
    j["spans"] = std::move(spans);
    return j;
}

json encodeParams(const CreationParams& params) {
    json j = json::object();
    if (std::holds_alternative<NoParams>(params)) {
        j["type"] = "none";
    } else if (const auto* r = std::get_if<RectangleParams>(&params)) {
        j["type"] = "rectangle";
        j["centre_mm"] = encodePoint(r->centreMm);
        j["width_mm"] = r->widthMm;
        j["height_mm"] = r->heightMm;
        j["corners"] = nameOf(kCorners, r->corners);
        j["corner_radius_mm"] = r->cornerRadiusMm;
    } else if (const auto* c = std::get_if<CircleParams>(&params)) {
        j["type"] = "circle";
        j["centre_mm"] = encodePoint(c->centreMm);
        j["radius_mm"] = c->radiusMm;
    } else if (const auto* e = std::get_if<EllipseParams>(&params)) {
        j["type"] = "ellipse";
        j["centre_mm"] = encodePoint(e->centreMm);
        j["width_mm"] = e->widthMm;
        j["height_mm"] = e->heightMm;
        j["angle_rad"] = e->angle;
    } else if (const auto* p = std::get_if<PolygonParams>(&params)) {
        j["type"] = "polygon";
        j["centre_mm"] = encodePoint(p->centreMm);
        j["sides"] = p->sides;
        j["radius_mm"] = p->radiusMm;
        j["angle_rad"] = p->angle;
    } else {
        const auto& s = std::get<StarParams>(params);
        j["type"] = "star";
        j["centre_mm"] = encodePoint(s.centreMm);
        j["points"] = s.points;
        j["outer_radius_mm"] = s.outerRadiusMm;
        j["inner_radius_mm"] = s.innerRadiusMm;
        j["angle_rad"] = s.angle;
    }
    return j;
}

json encodeObject(const Object& obj, const CarvExtras::Impl* extras) {
    json j = json::object();
    j["id"] = obj.id.value;
    j["layer"] = obj.layer.value;
    j["sheet"] = obj.sheet.value;
    if (obj.parent) {
        j["parent"] = obj.parent->value;
    }
    j["name"] = obj.name;
    if (const auto* v = obj.asVector()) {
        j["type"] = "vector";
        json contours = json::array();
        for (const Contour& c : v->contours) {
            contours.push_back(encodeContour(c));
        }
        j["contours"] = std::move(contours);
        const std::string* unknownParams = nullptr;
        if (extras) {
            const auto it = extras->params.find(obj.id.value);
            unknownParams = it == extras->params.end() ? nullptr : &it->second;
        }
        j["params"] = (unknownParams && std::holds_alternative<NoParams>(v->params))
                          ? rawJson(*unknownParams)
                          : encodeParams(v->params);
    } else if (const auto* b = obj.asBitmap()) {
        j["type"] = "bitmap";
        j["width_px"] = b->widthPx;
        j["height_px"] = b->heightPx;
        json placement = json::object();
        placement["min"] = encodePoint(b->placementMm.min);
        placement["max"] = encodePoint(b->placementMm.max);
        j["placement_mm"] = std::move(placement);
        j["source_ref"] = b->sourceRef;
    } else {
        j["type"] = "group";
        json children = json::array();
        for (const ObjectId child : obj.asGroup()->children) {
            children.push_back(child.value);
        }
        j["children"] = std::move(children);
    }
    return j;
}

// --- Toolpath records (ADR-0021). Parameters use the params.json keys of carvetto-cli (T-0122,
// ADR-0019): lengths in mm without a suffix; angles in radians under "<key>_rad" (params.json
// gives them in degrees under "<key>"), so they round-trip exactly. `cuttingData` is not saved.

json encodeTolerance(const geometry::Tolerance& t) {
    json j = json::object();
    j["distance"] = t.distance;
    j["angle_rad"] = t.angle;
    j["chordal"] = t.chordal;
    return j;
}

json encodeProfileParams(const toolpath::ProfileParams& p) {
    json j = json::object();
    j["name"] = p.name;
    j["side"] = nameOf(kProfileSides, p.side);
    j["direction"] = nameOf(kCutDirections, p.direction);
    j["allowance"] = p.allowance;
    json passes = json::object();
    passes["start_depth"] = p.passes.startDepth;
    passes["cut_depth"] = p.passes.cutDepth;
    passes["pass_depth"] = p.passes.passDepth;
    passes["maintain_exact_step"] = p.passes.maintainExactStep;
    passes["last_pass_thickness"] = p.passes.lastPassThickness;
    j["passes"] = std::move(passes);
    j["safe_z"] = p.safeZ;
    j["separate_last_pass"] = p.separateLastPass;
    j["last_pass_allowance"] = p.lastPassAllowance;
    j["last_pass_reverse"] = p.lastPassReverse;
    json tabs = json::object();
    tabs["enabled"] = p.tabs.enabled;
    tabs["length"] = p.tabs.length;
    tabs["thickness"] = p.tabs.thickness;
    tabs["spacing"] = nameOf(kTabSpacings, p.tabs.spacing);
    tabs["count"] = p.tabs.count;
    tabs["distance"] = p.tabs.distance;
    tabs["avoid_corners"] = p.tabs.avoidCorners;
    tabs["three_d"] = p.tabs.threeD;
    j["tabs"] = std::move(tabs);
    json ramp = json::object();
    ramp["type"] = nameOf(kRampTypes, p.ramp.type);
    ramp["distance"] = p.ramp.distance;
    ramp["use_angle"] = p.ramp.useAngle;
    ramp["angle_rad"] = p.ramp.angle;
    ramp["on_lead_in"] = p.ramp.onLeadIn;
    j["ramp"] = std::move(ramp);
    json leads = json::object();
    leads["type"] = nameOf(kLeadTypes, p.leads.type);
    leads["length"] = p.leads.length;
    leads["angle_rad"] = p.leads.angle;
    leads["radius"] = p.leads.radius;
    leads["sweep_rad"] = p.leads.sweep;
    leads["lead_out"] = p.leads.leadOut;
    leads["overcut"] = p.leads.overcut;
    j["leads"] = std::move(leads);
    json order = json::object();
    order["selection_order"] = p.order.selectionOrder;
    order["left_to_right"] = p.order.leftToRight;
    order["bottom_to_top"] = p.order.bottomToTop;
    order["grid"] = p.order.grid;
    j["order"] = std::move(order);
    j["start_at"] = nameOf(kStartAts, p.startAt);
    j["start_anchor"] = nameOf(kBoxAnchors, p.startAnchor);
    j["start_position"] = encodePoint(p.startPosition);
    json corners = json::object();
    corners["sharp_external"] = p.corners.sharpExternal;
    corners["sharp_internal"] = p.corners.sharpInternal;
    corners["mitre_limit"] = p.corners.mitreLimit;
    j["corners"] = std::move(corners);
    j["tolerance"] = encodeTolerance(p.tolerance);
    return j;
}

json encodePocketParams(const toolpath::PocketParams& p) {
    json j = json::object();
    j["name"] = p.name;
    j["start_depth"] = p.startDepth;
    j["cut_depth"] = p.cutDepth;
    j["maintain_exact_step"] = p.maintainExactStep;
    j["last_pass_thickness"] = p.lastPassThickness;
    j["safe_z"] = p.safeZ;
    j["strategy"] = nameOf(kPocketStrategies, p.strategy);
    j["direction"] = nameOf(kCutDirections, p.direction);
    j["raster_angle_rad"] = p.rasterAngle;
    j["profile_pass"] = nameOf(kProfilePasses, p.profilePass);
    j["allowance"] = p.allowance;
    j["ramp_plunge"] = p.rampPlunge;
    j["ramp_distance"] = p.rampDistance;
    j["tolerance"] = encodeTolerance(p.tolerance);
    return j;
}

/// Name of the move cache of the toolpath at `index` (0-based) in the job order.
std::string cacheMemberName(std::size_t index) {
    std::string digits = std::to_string(index + 1);
    digits.insert(0, digits.size() < 4 ? 4 - digits.size() : 0, '0');
    return "toolpaths/" + digits + ".bin";
}

json encodeToolpath(const ToolpathRecord& tp, const std::string* cacheMember, const CarvExtras::Impl* extras) {
    json j = json::object();
    j["id"] = tp.id.value;
    j["name"] = tp.name;
    j["operation"] = nameOf(kOperations, tp.operation());
    if (const auto* profile = std::get_if<toolpath::ProfileParams>(&tp.params)) {
        j["params"] = encodeProfileParams(*profile);
    } else {
        j["params"] = encodePocketParams(std::get<toolpath::PocketParams>(tp.params));
    }
    json selection = json::object();
    selection["all"] = tp.selection.all;
    json layers = json::array();
    for (const LayerId id : tp.selection.layers) {
        layers.push_back(id.value);
    }
    selection["layers"] = std::move(layers);
    json objects = json::array();
    for (const ObjectId id : tp.selection.objects) {
        objects.push_back(id.value);
    }
    selection["objects"] = std::move(objects);
    j["selection"] = std::move(selection);
    json tool = json::object();
    tool["id"] = tp.toolSource.toolId;
    tool["machine"] = tp.toolSource.machine;
    tool["material"] = tp.toolSource.material;
    tool["name"] = tp.tool.name;
    tool["tool_number"] = tp.tool.toolNumber;
    tool["diameter"] = tp.tool.diameter;
    tool["feed_rate"] = tp.tool.feedRate;
    tool["plunge_rate"] = tp.tool.plungeRate;
    tool["spindle_speed"] = tp.tool.spindleSpeed;
    j["tool"] = std::move(tool);
    if (cacheMember) {
        json moves = json::object();
        moves["count"] = static_cast<std::uint64_t>(tp.moveCount());
        moves["member"] = *cacheMember;
        j["moves"] = std::move(moves);
    }
    mergeUnknown(j, findExtra(extras, &CarvExtras::Impl::toolpaths, tp.id.value));
    return j;
}

// ------------------------------------------------------------------------------------------------
// Decoding: untrusted JSON is streamed through `detail::JsonReader` straight into records; no DOM
// is ever built. Every allocation is charged to the archive-wide `detail::LoadBudget`. Any defect
// throws `detail::LoadFailure`, caught once in `loadCarv`.
// ------------------------------------------------------------------------------------------------

using detail::JsonReader;
using detail::LoadBudget;
using detail::LoadFailure;
using JKind = JsonReader::Kind;

// Estimated bytes of decoded records charged to `LoadBudget::chargeDocument` (staging plus final
// storage, allocator and vector-growth overhead included; ADR-0013 §9).
constexpr std::uint64_t kCostObject = 448;
constexpr std::uint64_t kCostContour = 64;
constexpr std::uint64_t kCostSpan = 144;
constexpr std::uint64_t kCostChild = 16;
constexpr std::uint64_t kCostRecord = 256;  // sheet, layer, variable
constexpr std::uint64_t kCostString = 32;   // plus the length
constexpr std::size_t kMaxLayers = 1000;
constexpr std::size_t kMaxSheets = 1000;
constexpr std::int64_t kMaxGroupDepth = 1024;
constexpr std::size_t kMaxToolpaths = 10000;
/// After a parse error, members up to this size are drained to tell damage (CRC) from bad JSON.
constexpr std::uint64_t kDrainForCrcBytes = std::uint64_t{16} << 20;

/// Location of a value, rendered only when an error is reported.
struct Path {
    const Path* parent{nullptr};
    std::string_view key{};
    std::size_t index{0};
    bool isIndex{false};

    std::string str() const {
        std::string s = parent ? parent->str() : std::string();
        if (isIndex) {
            s += '[' + std::to_string(index) + ']';
        } else {
            if (!s.empty()) {
                s += '.';
            }
            s += key;
        }
        return s;
    }
    Path field(std::string_view k) const { return Path{this, k, 0, false}; }
    Path at(std::size_t i) const { return Path{this, {}, i, true}; }
};

[[noreturn]] void fail(const std::string& where, const std::string& what,
                       CarvError::Code code = CarvError::Code::InvalidContent) {
    throw LoadFailure{code, where + ": " + what};
}

[[noreturn]] void fail(const Path& where, const std::string& what,
                       CarvError::Code code = CarvError::Code::InvalidContent) {
    fail(where.str(), what, code);
}

std::string idText(std::uint64_t id) {
    return "#" + std::to_string(id);
}

double readDouble(JsonReader& r, const Path& p) {
    const JKind k = r.peek();
    if (k == JKind::Number) {
        const std::string_view t = r.readNumber();
        double v = 0.0;
        const auto res = std::from_chars(t.data(), t.data() + t.size(), v);
        if (res.ec != std::errc{} || res.ptr != t.data() + t.size()) {
            fail(p, "number " + std::string(t) + " is out of range");
        }
        return v;
    }
    if (k == JKind::String) {
        const std::string s = r.readString();
        if (s == "NaN") {
            return std::numeric_limits<double>::quiet_NaN();
        }
        if (s == "Infinity") {
            return std::numeric_limits<double>::infinity();
        }
        if (s == "-Infinity") {
            return -std::numeric_limits<double>::infinity();
        }
    }
    fail(p, "must be a number");
}

std::int64_t readInteger(JsonReader& r, const Path& p, std::int64_t lo, std::int64_t hi) {
    if (r.peek() != JKind::Number) {
        fail(p, "must be an integer");
    }
    const std::string_view t = r.readNumber();
    if (t.find_first_of(".eE") != std::string_view::npos) {
        fail(p, "must be an integer");
    }
    std::int64_t v = 0;
    const auto res = std::from_chars(t.data(), t.data() + t.size(), v);
    if (res.ec != std::errc{} || res.ptr != t.data() + t.size() || v < lo || v > hi) {
        fail(p, std::string(t) + " is out of range");
    }
    return v;
}

int readInt(JsonReader& r, const Path& p, int lo, int hi) {
    return static_cast<int>(readInteger(r, p, lo, hi));
}

/// A positive id (or next_id): an unsigned integer token.
std::uint64_t readId(JsonReader& r, const Path& p) {
    if (r.peek() != JKind::Number) {
        fail(p, "must be a positive integer id");
    }
    const std::string_view t = r.readNumber();
    std::uint64_t v = 0;
    const auto res = std::from_chars(t.data(), t.data() + t.size(), v);
    if (res.ec != std::errc{} || res.ptr != t.data() + t.size() || v == 0) {
        fail(p, "must be a positive integer id");
    }
    return v;
}

/// An object/layer/sheet id: 1 .. 2^62 - 1.
std::uint64_t readEntityId(JsonReader& r, const Path& p) {
    const std::uint64_t v = readId(r, p);
    if (v >= IdGenerator::kMaxId) {
        fail(p, std::to_string(v) + " is out of range (ids must be below 2^62)");
    }
    return v;
}

bool readBool(JsonReader& r, const Path& p) {
    if (r.peek() != JKind::Bool) {
        fail(p, "must be true or false");
    }
    return r.readBool();
}

std::string readString(JsonReader& r, const Path& p) {
    if (r.peek() != JKind::String) {
        fail(p, "must be a string");
    }
    return r.readString();
}

/// A string kept in the document (charged to the budget).
std::string readStoredString(JsonReader& r, const Path& p) {
    std::string s = readString(r, p);
    r.budget().chargeDocument(kCostString + s.size());
    return s;
}

template <std::size_t N>
std::array<double, N> readTuple(JsonReader& r, const Path& p, const char* shape) {
    if (r.peek() != JKind::Array) {
        fail(p, std::string("must be an ") + shape);
    }
    r.beginArray();
    std::array<double, N> v{};
    for (std::size_t i = 0; i < N; ++i) {
        if (!r.nextElement()) {
            fail(p, std::string("must be an ") + shape);
        }
        v[i] = readDouble(r, p.at(i));
    }
    if (r.nextElement()) {
        fail(p, std::string("must be an ") + shape);
    }
    return v;
}

Vec2 readPoint(JsonReader& r, const Path& p) {
    const auto v = readTuple<2>(r, p, "[x, y] pair");
    return {v[0], v[1]};
}

JobSize readSize(JsonReader& r, const Path& p) {
    const auto v = readTuple<3>(r, p, "[x, y, z] triple");
    return {v[0], v[1], v[2]};
}

template <class E, std::size_t N>
E readEnum(JsonReader& r, const Path& p, const EnumName<E> (&table)[N]) {
    const std::string s = readString(r, p);
    for (const auto& item : table) {
        if (s == item.name) {
            return item.value;
        }
    }
    fail(p, "unknown value '" + s + "'");
}

Colour readColour(JsonReader& r, const Path& p) {
    const std::string s = readString(r, p);
    auto hex = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        if (c >= 'A' && c <= 'F') return c - 'A' + 10;
        return -1;
    };
    std::array<std::uint8_t, 3> rgb{};
    bool ok = s.size() == 7 && s[0] == '#';
    for (std::size_t i = 0; ok && i < 3; ++i) {
        const int hi = hex(s[1 + 2 * i]);
        const int lo = hex(s[2 + 2 * i]);
        ok = hi >= 0 && lo >= 0;
        rgb[i] = static_cast<std::uint8_t>(ok ? hi * 16 + lo : 0);
    }
    if (!ok) {
        fail(p, "must be a colour like \"#rrggbb\"");
    }
    return {rgb[0], rgb[1], rgb[2]};
}

/// Members of one JSON object record: known keys (at most 64) are tracked for duplicates and
/// presence; unknown members are either kept as canonical text or skipped.
class Record {
   public:
    Record(JsonReader& r, const Path& p, std::span<const std::string_view> known)
        : r_(r), path_(p), known_(known) {
        if (r.peek() != JKind::Object) {
            fail(p, "must be a JSON object");
        }
        r.beginObject();
    }

    bool next() {
        if (!r_.nextKey(key_)) {
            return false;
        }
        index_ = indexOf(key_);
        if (index_ != kUnknown) {
            const std::uint64_t bit = std::uint64_t{1} << index_;
            if (seen_ & bit) {
                fail(path_, "duplicate key '" + key_ + "'");
            }
            seen_ |= bit;
        }
        return true;
    }
    const std::string& key() const { return key_; }
    bool known() const { return index_ != kUnknown; }
    Path field() const { return path_.field(key_); }
    bool has(std::string_view k) const {
        const std::size_t i = indexOf(k);
        return i != kUnknown && ((seen_ >> i) & 1U);
    }
    void require(std::string_view k) const {
        if (!has(k)) {
            fail(path_, "missing '" + std::string(k) + "'");
        }
    }
    void requireAll() const {
        for (const std::string_view k : known_) {
            require(k);
        }
    }
    /// Keeps the current (unknown) member's value as canonical text.
    void keep(ExtraMap& out) {
        std::string value;
        r_.readCanonical(&value);
        r_.budget().chargeExtras(key_.size() + 64);
        if (!out.emplace(key_, std::move(value)).second) {
            fail(path_, "duplicate key '" + key_ + "'");
        }
    }
    /// Validates and drops the current member's value.
    void skip() { r_.readCanonical(nullptr); }

   private:
    static constexpr std::size_t kUnknown = std::numeric_limits<std::size_t>::max();

    std::size_t indexOf(std::string_view k) const {
        const char* data = k.data();
        const std::size_t size = k.size();
        for (std::size_t i = 0; i < known_.size(); ++i) {
            const std::string_view& candidate = known_[i];
            if (candidate.size() == size && std::memcmp(candidate.data(), data, size) == 0) {
                return i;
            }
        }
        return kUnknown;
    }

    JsonReader& r_;
    const Path& path_;
    std::span<const std::string_view> known_;
    std::string key_;
    std::size_t index_{kUnknown};
    std::uint64_t seen_{0};
};

constexpr std::array<std::string_view, 11> kJobKeys{
    "type",        "size_mm",         "display_units",  "z_zero",
    "datum",       "datum_offset_mm", "flip_direction", "zero_off_same_side",
    "diameter_mm", "rotary_axis",     "flip_design"};

Job decodeJob(JsonReader& r, const Path& p, ExtraMap& unknown) {
    Record rec(r, p, kJobKeys);
    Job job;
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "type") {
            job.type = readEnum(r, f, kJobTypes);
        } else if (k == "size_mm") {
            job.sizeMm = readSize(r, f);
        } else if (k == "display_units") {
            job.displayUnits = readEnum(r, f, kUnits);
        } else if (k == "z_zero") {
            job.zZero = readEnum(r, f, kZZeros);
        } else if (k == "datum") {
            job.datum = readEnum(r, f, kDatums);
        } else if (k == "datum_offset_mm") {
            job.datumOffsetMm = readPoint(r, f);
        } else if (k == "flip_direction") {
            job.flipDirection = readEnum(r, f, kFlips);
        } else if (k == "zero_off_same_side") {
            job.zeroOffSameSide = readBool(r, f);
        } else if (k == "diameter_mm") {
            job.diameterMm = readDouble(r, f);
        } else if (k == "rotary_axis") {
            job.rotaryAxis = readEnum(r, f, kAxes);
        } else if (k == "flip_design") {
            job.flipDesign = readBool(r, f);
        } else {
            rec.keep(unknown);
        }
    }
    rec.requireAll();
    return job;
}

constexpr std::array<std::string_view, 7> kSheetKeys{
    "id", "name", "size_mm", "z_zero", "datum", "datum_offset_mm", "visible"};

Sheet decodeSheet(JsonReader& r, const Path& p, ExtraMap& unknown) {
    Record rec(r, p, kSheetKeys);
    Sheet s;
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "id") {
            s.id = SheetId{readEntityId(r, f)};
        } else if (k == "name") {
            s.name = readStoredString(r, f);
        } else if (k == "size_mm") {
            s.sizeMm = readSize(r, f);
        } else if (k == "z_zero") {
            s.zZero = readEnum(r, f, kZZeros);
        } else if (k == "datum") {
            s.datum = readEnum(r, f, kDatums);
        } else if (k == "datum_offset_mm") {
            s.datumOffsetMm = readPoint(r, f);
        } else if (k == "visible") {
            s.visible = readBool(r, f);
        } else {
            rec.keep(unknown);
        }
    }
    rec.requireAll();
    return s;
}

constexpr std::array<std::string_view, 5> kLayerKeys{"id", "name", "colour", "visible", "locked"};

Layer decodeLayer(JsonReader& r, const Path& p, ExtraMap& unknown) {
    Record rec(r, p, kLayerKeys);
    Layer l;
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "id") {
            l.id = LayerId{readEntityId(r, f)};
        } else if (k == "name") {
            l.name = readStoredString(r, f);
        } else if (k == "colour") {
            l.colour = readColour(r, f);
        } else if (k == "visible") {
            l.visible = readBool(r, f);
        } else if (k == "locked") {
            l.locked = readBool(r, f);
        } else {
            rec.keep(unknown);
        }
    }
    rec.requireAll();
    return l;
}

constexpr std::array<std::string_view, 12> kSpanKeys{"type",   "start",       "end",       "centre",
                                                     "radius", "start_angle", "end_angle", "direction",
                                                     "p0",     "p1",          "p2",        "p3"};

Span decodeSpan(JsonReader& r, const Path& p) {
    // Keys arrive in any order (canonical files put "type" last), so fields are staged first.
    Record rec(r, p, kSpanKeys);
    std::string type;
    std::array<Vec2, 4> pts{};  // start/end or p0..p3
    ArcSpan arc;
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "type") {
            type = readString(r, f);
        } else if (k == "start" || k == "p0") {
            pts[0] = readPoint(r, f);
        } else if (k == "end") {
            pts[3] = readPoint(r, f);
        } else if (k == "p1") {
            pts[1] = readPoint(r, f);
        } else if (k == "p2") {
            pts[2] = readPoint(r, f);
        } else if (k == "p3") {
            pts[3] = readPoint(r, f);
        } else if (k == "centre") {
            arc.centre = readPoint(r, f);
        } else if (k == "radius") {
            arc.radius = readDouble(r, f);
        } else if (k == "start_angle") {
            arc.startAngle = readDouble(r, f);
        } else if (k == "end_angle") {
            arc.endAngle = readDouble(r, f);
        } else if (k == "direction") {
            arc.direction = readEnum(r, f, kArcDirs);
        } else {
            rec.skip();  // unknown keys nested in a span are not preserved (ADR-0013 §7)
        }
    }
    rec.require("type");
    if (type == "line") {
        if (rec.has("p0") || rec.has("p3")) {
            fail(p, "a line span takes 'start' and 'end'");
        }
        rec.require("start");
        rec.require("end");
        return LineSpan{pts[0], pts[3]};
    }
    if (type == "arc") {
        for (const char* k : {"centre", "radius", "start_angle", "end_angle", "direction"}) {
            rec.require(k);
        }
        return arc;
    }
    if (type == "bezier") {
        if (rec.has("start") || rec.has("end")) {
            fail(p, "a bezier span takes 'p0' .. 'p3'");
        }
        for (const char* k : {"p0", "p1", "p2", "p3"}) {
            rec.require(k);
        }
        return BezierSpan{pts[0], pts[1], pts[2], pts[3]};
    }
    fail(p.field("type"), "unknown span type '" + type + "'");
}

constexpr std::array<std::string_view, 2> kContourKeys{"closed", "spans"};

Contour decodeContour(JsonReader& r, const Path& p) {
    Record rec(r, p, kContourKeys);
    Contour c;
    while (rec.next()) {
        const Path f = rec.field();
        if (rec.key() == "closed") {
            c.closed = readBool(r, f);
        } else if (rec.key() == "spans") {
            if (r.peek() != JKind::Array) {
                fail(f, "must be a JSON array");
            }
            r.beginArray();
            for (std::size_t i = 0; r.nextElement(); ++i) {
                r.budget().chargeDocument(kCostSpan);
                c.spans.push_back(decodeSpan(r, f.at(i)));
            }
        } else {
            rec.skip();
        }
    }
    rec.requireAll();
    return c;
}

constexpr std::array<std::string_view, 12> kParamKeys{
    "type",      "centre_mm", "width_mm", "height_mm", "corners",         "corner_radius_mm",
    "radius_mm", "angle_rad", "sides",    "points",    "outer_radius_mm", "inner_radius_mm"};

/// Decodes canonical params text (already bounded by the extras budget). An unknown `type`
/// yields `NoParams` and sets `unknown`.
CreationParams decodeParamsText(const std::string& text, const Path& p, bool& unknown) {
    unknown = false;
    if (text == R"({"type": "none"})") {
        return NoParams{};  // the common case, without a re-read
    }
    LoadBudget local;  // the text is already charged; this only bounds the re-read
    std::string type;
    {
        detail::StringSource src(text);
        JsonReader r(src, local, "params", text.size() + 1);
        Record rec(r, p, kParamKeys);
        while (rec.next()) {
            if (rec.key() == "type") {
                type = readString(r, rec.field());
            } else {
                rec.skip();
            }
        }
        rec.require("type");
    }
    if (type == "none") {
        return NoParams{};
    }
    constexpr std::array<std::string_view, 6> kKnown{"rectangle", "circle", "ellipse",
                                                     "polygon",   "star",   "none"};
    if (std::find(kKnown.begin(), kKnown.end(), type) == kKnown.end()) {
        unknown = true;  // a parameter kind from a newer minor version: kept verbatim
        return NoParams{};
    }
    detail::StringSource src(text);
    JsonReader r(src, local, "params", text.size() + 1);
    Record rec(r, p, kParamKeys);
    Vec2 centre;
    double width = 0, height = 0, cornerRadius = 0, radius = 0, angle = 0, outer = 0, inner = 0;
    int sides = 0, points = 0;
    auto corners = RectangleParams::CornerStyle::Square;
    constexpr int kIntMax = std::numeric_limits<int>::max();
    constexpr int kIntMin = std::numeric_limits<int>::min();
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "type") {
            readString(r, f);
        } else if (k == "centre_mm") {
            centre = readPoint(r, f);
        } else if (k == "width_mm") {
            width = readDouble(r, f);
        } else if (k == "height_mm") {
            height = readDouble(r, f);
        } else if (k == "corners") {
            corners = readEnum(r, f, kCorners);
        } else if (k == "corner_radius_mm") {
            cornerRadius = readDouble(r, f);
        } else if (k == "radius_mm") {
            radius = readDouble(r, f);
        } else if (k == "angle_rad") {
            angle = readDouble(r, f);
        } else if (k == "sides") {
            sides = readInt(r, f, kIntMin, kIntMax);
        } else if (k == "points") {
            points = readInt(r, f, kIntMin, kIntMax);
        } else if (k == "outer_radius_mm") {
            outer = readDouble(r, f);
        } else if (k == "inner_radius_mm") {
            inner = readDouble(r, f);
        } else {
            rec.skip();
        }
    }
    const auto need = [&rec](std::initializer_list<const char*> keys) {
        for (const char* k : keys) {
            rec.require(k);
        }
    };
    if (type == "rectangle") {
        need({"centre_mm", "width_mm", "height_mm", "corners", "corner_radius_mm"});
        return RectangleParams{centre, width, height, corners, cornerRadius};
    }
    if (type == "circle") {
        need({"centre_mm", "radius_mm"});
        return CircleParams{centre, radius};
    }
    if (type == "ellipse") {
        need({"centre_mm", "width_mm", "height_mm", "angle_rad"});
        return EllipseParams{centre, width, height, angle};
    }
    if (type == "polygon") {
        need({"centre_mm", "sides", "radius_mm", "angle_rad"});
        return PolygonParams{centre, sides, radius, angle};
    }
    need({"centre_mm", "points", "outer_radius_mm", "inner_radius_mm", "angle_rad"});
    return StarParams{centre, points, outer, inner, angle};
}

struct DecodedObject {
    Object object;
    ExtraMap unknown;
    std::optional<std::string> unknownParams;
};

constexpr std::array<std::string_view, 13> kObjectKeys{
    "id",     "layer",    "sheet",     "parent",       "name",       "type",    "contours",
    "params", "width_px", "height_px", "placement_mm", "source_ref", "children"};
constexpr std::array<std::string_view, 2> kPlacementKeys{"min", "max"};

DecodedObject decodeObject(JsonReader& r, const Path& p) {
    Record rec(r, p, kObjectKeys);
    DecodedObject out;
    Object& obj = out.object;
    std::string type;
    std::vector<Contour> contours;
    std::string paramsText;
    std::uint64_t paramsExtras = 0;
    std::uint64_t paramsUnknownValues = 0;
    BitmapObject bitmap;
    std::vector<ObjectId> children;
    constexpr int kIntMax = std::numeric_limits<int>::max();
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "id") {
            obj.id = ObjectId{readEntityId(r, f)};
        } else if (k == "layer") {
            obj.layer = LayerId{readEntityId(r, f)};
        } else if (k == "sheet") {
            obj.sheet = SheetId{readEntityId(r, f)};
        } else if (k == "parent") {
            obj.parent = ObjectId{readEntityId(r, f)};
        } else if (k == "name") {
            obj.name = readStoredString(r, f);
        } else if (k == "type") {
            type = readString(r, f);
        } else if (k == "contours") {
            if (r.peek() != JKind::Array) {
                fail(f, "must be a JSON array");
            }
            r.beginArray();
            for (std::size_t i = 0; r.nextElement(); ++i) {
                r.budget().chargeDocument(kCostContour);
                contours.push_back(decodeContour(r, f.at(i)));
            }
        } else if (k == "params") {
            if (r.peek() != JKind::Object) {
                fail(f, "must be a JSON object");
            }
            // Charged as unknown content until decoded; refunded below for known kinds.
            const std::uint64_t extrasBefore = r.budget().extrasBytes();
            const std::uint64_t unknownBefore = r.budget().unknownValues();
            r.readCanonical(&paramsText);
            paramsExtras = r.budget().extrasBytes() - extrasBefore;
            paramsUnknownValues = r.budget().unknownValues() - unknownBefore;
        } else if (k == "width_px") {
            bitmap.widthPx = readInt(r, f, 0, kIntMax);
        } else if (k == "height_px") {
            bitmap.heightPx = readInt(r, f, 0, kIntMax);
        } else if (k == "placement_mm") {
            Record placement(r, f, kPlacementKeys);
            while (placement.next()) {
                const Path pf = placement.field();
                if (placement.key() == "min") {
                    bitmap.placementMm.min = readPoint(r, pf);
                } else if (placement.key() == "max") {
                    bitmap.placementMm.max = readPoint(r, pf);
                } else {
                    placement.skip();
                }
            }
            placement.requireAll();
        } else if (k == "source_ref") {
            bitmap.sourceRef = readStoredString(r, f);
        } else if (k == "children") {
            if (r.peek() != JKind::Array) {
                fail(f, "must be a JSON array");
            }
            r.beginArray();
            for (std::size_t i = 0; r.nextElement(); ++i) {
                r.budget().chargeDocument(kCostChild);
                children.push_back(ObjectId{readEntityId(r, f.at(i))});
            }
        } else {
            rec.keep(out.unknown);
        }
    }
    for (const char* k : {"id", "layer", "sheet", "name", "type"}) {
        rec.require(k);
    }
    const auto where = [&p, &obj]() { return p.str() + " (id " + std::to_string(obj.id.value) + ")"; };
    const auto only = [&](std::initializer_list<const char*> allowed,
                          std::initializer_list<const char*> required) {
        for (const char* k : required) {
            if (!rec.has(k)) {
                fail(where(), std::string("missing '") + k + "'");
            }
        }
        for (const char* k :
             {"contours", "params", "width_px", "height_px", "placement_mm", "source_ref", "children"}) {
            if (rec.has(k) && std::find_if(allowed.begin(), allowed.end(), [k](const char* a) {
                                  return std::string_view(a) == k;
                              }) == allowed.end()) {
                fail(where(), std::string("'") + k + "' is not valid for a " + type + " object");
            }
        }
    };
    if (type == "vector") {
        only({"contours", "params"}, {"contours", "params"});
        VectorObject v;
        v.contours = std::move(contours);
        bool unknownParams = false;
        v.params = decodeParamsText(paramsText, p.field("params"), unknownParams);
        if (unknownParams) {
            out.unknownParams = std::move(paramsText);
        } else {
            r.budget().refundExtras(paramsExtras);
            r.budget().refundUnknownValues(paramsUnknownValues);
        }
        obj.data = std::move(v);
    } else if (type == "bitmap") {
        only({"width_px", "height_px", "placement_mm", "source_ref"},
             {"width_px", "height_px", "placement_mm", "source_ref"});
        obj.data = std::move(bitmap);
    } else if (type == "group") {
        only({"children"}, {"children"});
        obj.data = GroupObject{std::move(children)};
    } else {
        fail(where() + ".type", "unsupported object type '" + type + "' (written by a newer Carvetto?)");
    }
    return out;
}

// --- Toolpath records (ADR-0021) ---------------------------------------------------------------

/// Which params schema is read: job.json (angles in radians under "<key>_rad") or a params.json of
/// an interim toolpath member (angles in degrees under "<key>", plus "tool" and "layers").
enum class ParamsDialect { JobJson, ParamsJson };

/// What an interim member's params.json says besides the operation parameters.
struct LegacyParamsExtras {
    ToolSource tool;
    std::optional<std::vector<std::string>> layers;
};

constexpr double kRadPerDeg = geometry::kPi / 180.0;

/// Reads the members of one params object. Every key is optional (the core default stays);
/// unknown keys, and keys of the other dialect, are validated and skipped (ADR-0013 §7).
class ParamsRecord {
   public:
    ParamsRecord(JsonReader& r, const Path& p, std::span<const std::string_view> keys, ParamsDialect dialect)
        : r_(r), rec_(r, p, keys), dialect_(dialect) {}

    /// Next known member; skips unknown ones. Returns false at the end of the object.
    bool next() {
        while (rec_.next()) {
            if (rec_.known()) {
                return true;
            }
            rec_.skip();
        }
        return false;
    }
    const std::string& key() const { return rec_.key(); }
    Path field() const { return rec_.field(); }
    /// True when the current key is `base` in this dialect's spelling of an angle.
    bool isAngle(std::string_view base) const {
        const std::string& k = rec_.key();
        if (dialect_ == ParamsDialect::ParamsJson) {
            return k == base;
        }
        return k.size() == base.size() + 4 && k.compare(0, base.size(), base) == 0 &&
               k.compare(base.size(), 4, "_rad") == 0;
    }
    double number() { return readDouble(r_, field()); }
    double angle() {
        const double v = readDouble(r_, field());
        return dialect_ == ParamsDialect::ParamsJson ? v * kRadPerDeg : v;
    }
    bool boolean() { return readBool(r_, field()); }
    int integer() {
        return readInt(r_, field(), std::numeric_limits<int>::min(), std::numeric_limits<int>::max());
    }
    std::string string() { return readStoredString(r_, field()); }
    template <class E, std::size_t N>
    E choice(const EnumName<E> (&table)[N]) {
        return readEnum(r_, field(), table);
    }
    JsonReader& reader() { return r_; }
    void skip() { rec_.skip(); }

   private:
    JsonReader& r_;
    Record rec_;
    ParamsDialect dialect_;
};

constexpr std::array<std::string_view, 3> kToleranceKeysJob{"distance", "angle_rad", "chordal"};
constexpr std::array<std::string_view, 3> kToleranceKeysParams{"distance", "angle", "chordal"};

geometry::Tolerance decodeTolerance(JsonReader& r, const Path& p, ParamsDialect d) {
    geometry::Tolerance t;
    ParamsRecord rec(r, p, d == ParamsDialect::JobJson ? std::span<const std::string_view>(kToleranceKeysJob)
                                                       : std::span<const std::string_view>(kToleranceKeysParams),
                     d);
    while (rec.next()) {
        if (rec.key() == "distance") {
            t.distance = rec.number();
        } else if (rec.isAngle("angle")) {
            t.angle = rec.angle();
        } else {
            t.chordal = rec.number();
        }
    }
    return t;
}

constexpr std::array<std::string_view, 4> kLegacyToolKeys{"id", "database", "machine", "material"};

/// The "tool" and "layers" members of an interim params.json (T-0122's schema).
void decodeLegacyCommon(ParamsRecord& rec, LegacyParamsExtras& out) {
    JsonReader& r = rec.reader();
    const Path f = rec.field();
    if (rec.key() == "tool") {
        Record tool(r, f, kLegacyToolKeys);
        while (tool.next()) {
            const Path tf = tool.field();
            if (tool.key() == "id") {
                out.tool.toolId = readStoredString(r, tf);
            } else if (tool.key() == "machine") {
                out.tool.machine = readStoredString(r, tf);
            } else if (tool.key() == "material") {
                out.tool.material = readStoredString(r, tf);
            } else {
                tool.skip();  // "database" (a path relative to the params file) and unknown keys
            }
        }
        return;
    }
    if (r.peek() != JKind::Array) {
        fail(f, "must be an array of layer names");
    }
    std::vector<std::string> names;
    r.beginArray();
    for (std::size_t i = 0; r.nextElement(); ++i) {
        names.push_back(readStoredString(r, f.at(i)));
    }
    out.layers = std::move(names);
}

constexpr std::array<std::string_view, 5> kPassKeys{"start_depth", "cut_depth", "pass_depth", "maintain_exact_step",
                                                    "last_pass_thickness"};
constexpr std::array<std::string_view, 8> kTabKeys{"enabled", "length",   "thickness",     "spacing",
                                                   "count",   "distance", "avoid_corners", "three_d"};
constexpr std::array<std::string_view, 5> kRampKeysJob{"type", "distance", "use_angle", "angle_rad", "on_lead_in"};
constexpr std::array<std::string_view, 5> kRampKeysParams{"type", "distance", "use_angle", "angle", "on_lead_in"};
constexpr std::array<std::string_view, 7> kLeadKeysJob{"type",      "length",   "angle_rad", "radius",
                                                       "sweep_rad", "lead_out", "overcut"};
constexpr std::array<std::string_view, 7> kLeadKeysParams{"type",  "length",   "angle",  "radius",
                                                          "sweep", "lead_out", "overcut"};
constexpr std::array<std::string_view, 4> kOrderKeys{"selection_order", "left_to_right", "bottom_to_top", "grid"};
constexpr std::array<std::string_view, 3> kCornerKeys{"sharp_external", "sharp_internal", "mitre_limit"};
constexpr std::array<std::string_view, 20> kProfileKeys{
    "name",         "side",           "direction",   "allowance",          "passes",
    "safe_z",       "separate_last_pass", "last_pass_allowance", "last_pass_reverse", "tabs",
    "ramp",         "leads",          "order",       "start_at",           "start_anchor",
    "start_position", "corners",      "tolerance",   "tool",               "layers"};
constexpr std::array<std::string_view, 16> kPocketKeysJob{
    "name",     "start_depth",      "cut_depth",    "maintain_exact_step", "last_pass_thickness", "safe_z",
    "strategy", "direction",        "raster_angle_rad", "profile_pass",    "allowance",           "ramp_plunge",
    "ramp_distance", "tolerance",   "tool",         "layers"};
constexpr std::array<std::string_view, 16> kPocketKeysParams{
    "name",     "start_depth",      "cut_depth",    "maintain_exact_step", "last_pass_thickness", "safe_z",
    "strategy", "direction",        "raster_angle", "profile_pass",        "allowance",           "ramp_plunge",
    "ramp_distance", "tolerance",   "tool",         "layers"};

template <std::size_t N>
std::span<const std::string_view> keysFor(const std::array<std::string_view, N>& keys, ParamsDialect d) {
    // "tool" and "layers" (always the last two) belong to params.json only.
    return d == ParamsDialect::ParamsJson ? std::span<const std::string_view>(keys)
                                          : std::span<const std::string_view>(keys.data(), N - 2);
}

toolpath::ProfileParams decodeProfileParams(JsonReader& r, const Path& p, ParamsDialect d,
                                            LegacyParamsExtras* legacy) {
    toolpath::ProfileParams out;
    ParamsRecord rec(r, p, keysFor(kProfileKeys, d), d);
    while (rec.next()) {
        const std::string& k = rec.key();
        const Path f = rec.field();
        if (k == "name") {
            out.name = rec.string();
        } else if (k == "side") {
            out.side = rec.choice(kProfileSides);
        } else if (k == "direction") {
            out.direction = rec.choice(kCutDirections);
        } else if (k == "allowance") {
            out.allowance = rec.number();
        } else if (k == "passes") {
            ParamsRecord g(r, f, kPassKeys, d);
            while (g.next()) {
                const std::string& gk = g.key();
                if (gk == "start_depth") {
                    out.passes.startDepth = g.number();
                } else if (gk == "cut_depth") {
                    out.passes.cutDepth = g.number();
                } else if (gk == "pass_depth") {
                    out.passes.passDepth = g.number();
                } else if (gk == "maintain_exact_step") {
                    out.passes.maintainExactStep = g.boolean();
                } else {
                    out.passes.lastPassThickness = g.number();
                }
            }
        } else if (k == "safe_z") {
            out.safeZ = rec.number();
        } else if (k == "separate_last_pass") {
            out.separateLastPass = rec.boolean();
        } else if (k == "last_pass_allowance") {
            out.lastPassAllowance = rec.number();
        } else if (k == "last_pass_reverse") {
            out.lastPassReverse = rec.boolean();
        } else if (k == "tabs") {
            ParamsRecord g(r, f, kTabKeys, d);
            while (g.next()) {
                const std::string& gk = g.key();
                if (gk == "enabled") {
                    out.tabs.enabled = g.boolean();
                } else if (gk == "length") {
                    out.tabs.length = g.number();
                } else if (gk == "thickness") {
                    out.tabs.thickness = g.number();
                } else if (gk == "spacing") {
                    out.tabs.spacing = g.choice(kTabSpacings);
                } else if (gk == "count") {
                    out.tabs.count = g.integer();
                } else if (gk == "distance") {
                    out.tabs.distance = g.number();
                } else if (gk == "avoid_corners") {
                    out.tabs.avoidCorners = g.boolean();
                } else {
                    out.tabs.threeD = g.boolean();
                }
            }
        } else if (k == "ramp") {
            ParamsRecord g(r, f, d == ParamsDialect::JobJson ? kRampKeysJob : kRampKeysParams, d);
            while (g.next()) {
                const std::string& gk = g.key();
                if (gk == "type") {
                    out.ramp.type = g.choice(kRampTypes);
                } else if (gk == "distance") {
                    out.ramp.distance = g.number();
                } else if (gk == "use_angle") {
                    out.ramp.useAngle = g.boolean();
                } else if (g.isAngle("angle")) {
                    out.ramp.angle = g.angle();
                } else {
                    out.ramp.onLeadIn = g.boolean();
                }
            }
        } else if (k == "leads") {
            ParamsRecord g(r, f, d == ParamsDialect::JobJson ? kLeadKeysJob : kLeadKeysParams, d);
            while (g.next()) {
                const std::string& gk = g.key();
                if (gk == "type") {
                    out.leads.type = g.choice(kLeadTypes);
                } else if (gk == "length") {
                    out.leads.length = g.number();
                } else if (g.isAngle("angle")) {
                    out.leads.angle = g.angle();
                } else if (gk == "radius") {
                    out.leads.radius = g.number();
                } else if (g.isAngle("sweep")) {
                    out.leads.sweep = g.angle();
                } else if (gk == "lead_out") {
                    out.leads.leadOut = g.boolean();
                } else {
                    out.leads.overcut = g.number();
                }
            }
        } else if (k == "order") {
            ParamsRecord g(r, f, kOrderKeys, d);
            while (g.next()) {
                const std::string& gk = g.key();
                if (gk == "selection_order") {
                    out.order.selectionOrder = g.boolean();
                } else if (gk == "left_to_right") {
                    out.order.leftToRight = g.boolean();
                } else if (gk == "bottom_to_top") {
                    out.order.bottomToTop = g.boolean();
                } else {
                    out.order.grid = g.boolean();
                }
            }
        } else if (k == "start_at") {
            out.startAt = rec.choice(kStartAts);
        } else if (k == "start_anchor") {
            out.startAnchor = rec.choice(kBoxAnchors);
        } else if (k == "start_position") {
            out.startPosition = readPoint(r, f);
        } else if (k == "corners") {
            ParamsRecord g(r, f, kCornerKeys, d);
            while (g.next()) {
                const std::string& gk = g.key();
                if (gk == "sharp_external") {
                    out.corners.sharpExternal = g.boolean();
                } else if (gk == "sharp_internal") {
                    out.corners.sharpInternal = g.boolean();
                } else {
                    out.corners.mitreLimit = g.number();
                }
            }
        } else if (k == "tolerance") {
            out.tolerance = decodeTolerance(r, f, d);
        } else if (legacy) {  // "tool" or "layers" (params.json only)
            decodeLegacyCommon(rec, *legacy);
        } else {
            rec.skip();
        }
    }
    return out;
}

toolpath::PocketParams decodePocketParams(JsonReader& r, const Path& p, ParamsDialect d,
                                          LegacyParamsExtras* legacy) {
    toolpath::PocketParams out;
    ParamsRecord rec(r, p, keysFor(d == ParamsDialect::JobJson ? kPocketKeysJob : kPocketKeysParams, d), d);
    while (rec.next()) {
        const std::string& k = rec.key();
        if (k == "name") {
            out.name = rec.string();
        } else if (k == "start_depth") {
            out.startDepth = rec.number();
        } else if (k == "cut_depth") {
            out.cutDepth = rec.number();
        } else if (k == "maintain_exact_step") {
            out.maintainExactStep = rec.boolean();
        } else if (k == "last_pass_thickness") {
            out.lastPassThickness = rec.number();
        } else if (k == "safe_z") {
            out.safeZ = rec.number();
        } else if (k == "strategy") {
            out.strategy = rec.choice(kPocketStrategies);
        } else if (k == "direction") {
            out.direction = rec.choice(kCutDirections);
        } else if (rec.isAngle("raster_angle")) {
            out.rasterAngle = rec.angle();
        } else if (k == "profile_pass") {
            out.profilePass = rec.choice(kProfilePasses);
        } else if (k == "allowance") {
            out.allowance = rec.number();
        } else if (k == "ramp_plunge") {
            out.rampPlunge = rec.boolean();
        } else if (k == "ramp_distance") {
            out.rampDistance = rec.number();
        } else if (k == "tolerance") {
            out.tolerance = decodeTolerance(r, rec.field(), d);
        } else if (legacy) {
            decodeLegacyCommon(rec, *legacy);
        } else {
            rec.skip();
        }
    }
    return out;
}

/// Decodes params captured as canonical text (already charged to the budget) for `operation`.
ToolpathParams decodeParamsOf(ToolpathOperation operation, const std::string& text, const Path& p,
                              ParamsDialect d, LegacyParamsExtras* legacy) {
    LoadBudget local;  // the text is already charged; this only bounds the re-read
    detail::StringSource src(text);
    JsonReader r(src, local, "params", text.size() + 1);
    if (operation == ToolpathOperation::Profile) {
        return decodeProfileParams(r, p, d, legacy);
    }
    return decodePocketParams(r, p, d, legacy);
}

/// Captures the value at `r` as canonical text; it is charged as unknown content only while held.
std::string captureParams(JsonReader& r, const Path& f) {
    if (r.peek() != JKind::Object) {
        fail(f, "must be a JSON object");
    }
    const std::uint64_t extrasBefore = r.budget().extrasBytes();
    const std::uint64_t unknownBefore = r.budget().unknownValues();
    std::string text;
    r.readCanonical(&text);
    r.budget().refundExtras(r.budget().extrasBytes() - extrasBefore);
    r.budget().refundUnknownValues(r.budget().unknownValues() - unknownBefore);
    // Charged as document content instead: one value per byte is a generous upper bound.
    r.budget().chargeDocument(kCostString + text.size());
    return text;
}

struct DecodedToolpath {
    ToolpathRecord record;
    ExtraMap unknown;
    /// The move cache named in job.json, and its move count.
    std::optional<std::string> cacheMember;
    std::uint64_t moveCount{0};
};

constexpr std::array<std::string_view, 7> kToolpathKeys{"id",        "name", "operation", "params",
                                                        "selection", "tool", "moves"};
constexpr std::array<std::string_view, 3> kSelectionKeys{"all", "layers", "objects"};
constexpr std::array<std::string_view, 9> kToolKeys{"id",        "machine",     "material",    "name",         "tool_number",
                                                    "diameter",  "feed_rate",   "plunge_rate", "spindle_speed"};
constexpr std::array<std::string_view, 2> kMovesKeys{"count", "member"};

DecodedToolpath decodeToolpath(JsonReader& r, const Path& p) {
    Record rec(r, p, kToolpathKeys);
    DecodedToolpath out;
    ToolpathRecord& tp = out.record;
    std::string paramsText;
    std::string operation;
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "id") {
            tp.id = ToolpathId{readEntityId(r, f)};
        } else if (k == "name") {
            tp.name = readStoredString(r, f);
        } else if (k == "operation") {
            operation = readString(r, f);
        } else if (k == "params") {
            paramsText = captureParams(r, f);
        } else if (k == "selection") {
            Record sel(r, f, kSelectionKeys);
            while (sel.next()) {
                const Path sf = sel.field();
                if (sel.key() == "all") {
                    tp.selection.all = readBool(r, sf);
                } else if (sel.key() == "layers" || sel.key() == "objects") {
                    const bool layers = sel.key() == "layers";
                    if (r.peek() != JKind::Array) {
                        fail(sf, "must be a JSON array of ids");
                    }
                    r.beginArray();
                    for (std::size_t i = 0; r.nextElement(); ++i) {
                        r.budget().chargeDocument(kCostChild);
                        const std::uint64_t id = readEntityId(r, sf.at(i));
                        if (layers) {
                            tp.selection.layers.push_back(LayerId{id});
                        } else {
                            tp.selection.objects.push_back(ObjectId{id});
                        }
                    }
                } else {
                    sel.skip();
                }
            }
        } else if (k == "tool") {
            Record tool(r, f, kToolKeys);
            constexpr int kIntMax = std::numeric_limits<int>::max();
            constexpr int kIntMin = std::numeric_limits<int>::min();
            while (tool.next()) {
                const Path tf = tool.field();
                const std::string& tk = tool.key();
                if (tk == "id") {
                    tp.toolSource.toolId = readStoredString(r, tf);
                } else if (tk == "machine") {
                    tp.toolSource.machine = readStoredString(r, tf);
                } else if (tk == "material") {
                    tp.toolSource.material = readStoredString(r, tf);
                } else if (tk == "name") {
                    tp.tool.name = readStoredString(r, tf);
                } else if (tk == "tool_number") {
                    tp.tool.toolNumber = readInt(r, tf, kIntMin, kIntMax);
                } else if (tk == "diameter") {
                    tp.tool.diameter = readDouble(r, tf);
                } else if (tk == "feed_rate") {
                    tp.tool.feedRate = readDouble(r, tf);
                } else if (tk == "plunge_rate") {
                    tp.tool.plungeRate = readDouble(r, tf);
                } else if (tk == "spindle_speed") {
                    tp.tool.spindleSpeed = readDouble(r, tf);
                } else {
                    tool.skip();
                }
            }
            for (const char* key : {"name", "tool_number", "diameter", "feed_rate", "plunge_rate", "spindle_speed"}) {
                tool.require(key);
            }
        } else if (k == "moves") {
            Record moves(r, f, kMovesKeys);
            while (moves.next()) {
                const Path mf = moves.field();
                if (moves.key() == "count") {
                    if (r.peek() != JKind::Number) {
                        fail(mf, "must be a move count");
                    }
                    const std::string_view t = r.readNumber();
                    std::uint64_t n = 0;
                    const auto res = std::from_chars(t.data(), t.data() + t.size(), n);
                    if (res.ec != std::errc{} || res.ptr != t.data() + t.size()) {
                        fail(mf, "must be a move count");
                    }
                    r.budget().chargeMoves(n);  // refused here, before the cache is read
                    out.moveCount = n;
                } else if (moves.key() == "member") {
                    out.cacheMember = readString(r, mf);
                    if (!detail::isToolpathCacheName(*out.cacheMember)) {
                        fail(mf, "must name a move cache \"toolpaths/NNNN.bin\"");
                    }
                } else {
                    moves.skip();
                }
            }
            moves.requireAll();
        } else {
            rec.keep(out.unknown);
        }
    }
    for (const char* key : {"id", "name", "operation", "params", "selection", "tool"}) {
        rec.require(key);
    }
    const auto where = [&p, &tp]() { return p.str() + " (id " + std::to_string(tp.id.value) + ")"; };
    ToolpathOperation op = ToolpathOperation::Profile;
    bool knownOp = false;
    for (const auto& item : kOperations) {
        if (operation == item.name) {
            op = item.value;
            knownOp = true;
        }
    }
    if (!knownOp) {
        fail(where() + ".operation",
             "unsupported toolpath operation '" + operation + "' (written by a newer Carvetto?)");
    }
    tp.params = decodeParamsOf(op, paramsText, p.field("params"), ParamsDialect::JobJson, nullptr);
    if (tp.selection.all && (!tp.selection.layers.empty() || !tp.selection.objects.empty())) {
        fail(where() + ".selection", "'all' cannot be combined with layers or objects");
    }
    return out;
}

/// A T-0122 interim toolpath member `toolpaths/NNNN.json` (ADR-0019), decoded for migration.
struct InterimToolpath {
    ToolpathRecord record;  // id null; selection filled in once the layers exist
    std::optional<std::vector<std::string>> layers;
};

constexpr std::array<std::string_view, 8> kInterimKeys{"format", "format_version", "operation", "name",
                                                       "params", "safe_z",         "tool",      "moves"};
constexpr std::array<std::string_view, 6> kInterimToolKeys{"name",      "tool_number", "diameter",
                                                           "feed_rate", "plunge_rate", "spindle_speed"};
constexpr std::array<std::string_view, 7> kInterimKinds{"rapid", "feed", "plunge", "retract", "arc_cw", "arc_ccw", "dwell"};
constexpr std::array<std::string_view, 1> kFeedKeys{"feed"};

double readFinite(JsonReader& r, const Path& p) {
    const double v = readDouble(r, p);
    if (!std::isfinite(v)) {
        fail(p, "must be finite");
    }
    return v;
}

toolpath::Move decodeInterimMove(JsonReader& r, const Path& p) {
    if (r.peek() != JKind::Array) {
        fail(p, "must be an array [\"kind\", x, y, z, ...]");
    }
    r.beginArray();
    if (!r.nextElement()) {
        fail(p, "must not be empty");
    }
    const std::string kind = readString(r, p.at(0));
    const auto it = std::find(kInterimKinds.begin(), kInterimKinds.end(), kind);
    if (it == kInterimKinds.end()) {
        fail(p.at(0), "unknown move kind '" + kind + "'");
    }
    toolpath::Move m;
    m.kind = static_cast<toolpath::MoveKind>(it - kInterimKinds.begin());
    const std::size_t fixed = toolpath::isArc(m.kind) ? 5 : m.kind == toolpath::MoveKind::Dwell ? 4 : 3;
    std::array<double, 5> v{};
    for (std::size_t i = 0; i < fixed; ++i) {
        if (!r.nextElement()) {
            fail(p, "has too few elements");
        }
        v[i] = readFinite(r, p.at(i + 1));
    }
    m.to = {v[0], v[1], v[2]};
    if (toolpath::isArc(m.kind)) {
        m.centre = {v[3], v[4]};
    } else if (m.kind == toolpath::MoveKind::Dwell) {
        m.dwellSeconds = v[3];
    }
    if (r.nextElement()) {
        const Path f = p.at(fixed + 1);
        if (!toolpath::isCutting(m.kind)) {
            fail(f, "a feed override is only valid on a cutting move");
        }
        Record feed(r, f, kFeedKeys);
        while (feed.next()) {
            if (feed.known()) {
                m.feedOverride = readFinite(r, feed.field());
            } else {
                fail(f, "unknown key '" + feed.key() + "'");
            }
        }
        feed.requireAll();
        if (r.nextElement()) {
            fail(p, "has too many elements");
        }
    }
    return m;
}

/// Decodes an interim member. With `countOnly` the moves are validated and counted (in
/// `moveCount`) but not kept; otherwise exactly `reserve` moves are reserved first, so a counting
/// pass beforehand gives an exact allocation (no geometric growth, ADR-0021 §4).
InterimToolpath decodeInterim(JsonReader& r, bool countOnly = false, std::uint64_t reserve = 0,
                              std::uint64_t* moveCount = nullptr) {
    const Path root{nullptr, r.member(), 0, false};
    Record rec(r, root, kInterimKeys);
    InterimToolpath out;
    ToolpathRecord& tp = out.record;
    std::string format;
    std::string operation;
    std::string paramsText;
    std::vector<toolpath::Move> moves;
    moves.reserve(static_cast<std::size_t>(reserve));
    std::uint64_t counted = 0;
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "format") {
            format = readString(r, f);
        } else if (k == "format_version") {
            if (readInteger(r, f, 0, std::numeric_limits<std::int64_t>::max()) != 1) {
                fail(f, "unsupported interim toolpath version (this build migrates version 1)");
            }
        } else if (k == "operation") {
            operation = readString(r, f);
        } else if (k == "name") {
            tp.name = readStoredString(r, f);
        } else if (k == "params") {
            paramsText = captureParams(r, f);
        } else if (k == "safe_z") {
            readFinite(r, f);  // the params' safe_z, repeated
        } else if (k == "tool") {
            Record tool(r, f, kInterimToolKeys);
            while (tool.next()) {
                const Path tf = tool.field();
                const std::string& tk = tool.key();
                if (tk == "name") {
                    tp.tool.name = readStoredString(r, tf);
                } else if (tk == "tool_number") {
                    tp.tool.toolNumber = readInt(r, tf, 0, 1'000'000);
                } else if (tk == "diameter") {
                    tp.tool.diameter = readFinite(r, tf);
                } else if (tk == "feed_rate") {
                    tp.tool.feedRate = readFinite(r, tf);
                } else if (tk == "plunge_rate") {
                    tp.tool.plungeRate = readFinite(r, tf);
                } else if (tk == "spindle_speed") {
                    tp.tool.spindleSpeed = readFinite(r, tf);
                } else {
                    tool.skip();
                }
            }
            tool.requireAll();
        } else if (k == "moves") {
            if (r.peek() != JKind::Array) {
                fail(f, "must be a JSON array");
            }
            r.beginArray();
            for (std::size_t i = 0; r.nextElement(); ++i) {
                r.budget().chargeMoves(1);
                const toolpath::Move m = decodeInterimMove(r, f.at(i));
                ++counted;
                if (!countOnly) {
                    moves.push_back(m);
                }
            }
        } else {
            rec.skip();
        }
    }
    rec.requireAll();
    if (format != "carvetto-toolpath") {
        fail(root.field("format"), "is not \"carvetto-toolpath\"");
    }
    ToolpathOperation op = ToolpathOperation::Profile;
    if (operation == "pocket") {
        op = ToolpathOperation::Pocket;
    } else if (operation != "profile") {
        fail(root.field("operation"), "unknown operation '" + operation + "'");
    }
    LegacyParamsExtras legacy;
    tp.params = decodeParamsOf(op, paramsText, root.field("params"), ParamsDialect::ParamsJson, &legacy);
    tp.toolSource = std::move(legacy.tool);
    out.layers = std::move(legacy.layers);
    if (moveCount) {
        *moveCount = counted;
    }
    tp.moves = shareMoves(std::move(moves));
    return out;
}

/// The parsed manifest.json.
struct ManifestFile {
    std::string format;
    int major{0};
    int minor{0};
    std::string appVersion;
    ExtraMap unknown;
};

constexpr std::array<std::string_view, 3> kManifestKeys{"format", "format_version", "app_version"};
constexpr std::array<std::string_view, 2> kVersionKeys{"major", "minor"};

ManifestFile decodeManifest(JsonReader& r) {
    const Path root{nullptr, kManifestName, 0, false};
    ManifestFile m;
    Record rec(r, root, kManifestKeys);
    constexpr int kIntMax = std::numeric_limits<int>::max();
    while (rec.next()) {
        const Path f = rec.field();
        if (rec.key() == "format") {
            m.format = readString(r, f);
        } else if (rec.key() == "format_version") {
            Record version(r, f, kVersionKeys);
            while (version.next()) {
                const Path vf = version.field();
                if (version.key() == "major") {
                    m.major = readInt(r, vf, 0, kIntMax);
                } else if (version.key() == "minor") {
                    m.minor = readInt(r, vf, 0, kIntMax);
                } else {
                    version.skip();
                }
            }
            version.requireAll();
        } else if (rec.key() == "app_version") {
            m.appVersion = readStoredString(r, f);
        } else {
            rec.keep(m.unknown);
        }
    }
    rec.requireAll();
    return m;
}

/// The parsed job.json, before it becomes a Document.
struct JobFile {
    Job job;
    ExtraMap jobUnknown;
    ExtraMap rootUnknown;
    std::vector<std::pair<Sheet, ExtraMap>> sheets;
    std::vector<std::pair<Layer, ExtraMap>> layers;
    std::vector<DecodedObject> objects;
    std::vector<DecodedToolpath> toolpaths;
    std::vector<std::pair<std::string, double>> variables;
    std::uint64_t nextId{0};
    std::uint64_t activeSheet{0};
    std::uint64_t activeLayer{0};
};

constexpr std::array<std::string_view, 9> kRootKeys{"active_layer", "active_sheet", "job",
                                                    "layers",       "next_id",      "objects",
                                                    "sheets",       "toolpaths",    "variables"};

JobFile decodeJobFile(JsonReader& r) {
    const Path root{nullptr, kJobName, 0, false};
    JobFile jf;
    Record rec(r, root, kRootKeys);
    while (rec.next()) {
        const Path f = rec.field();
        const std::string& k = rec.key();
        if (k == "active_layer") {
            jf.activeLayer = readEntityId(r, f);
        } else if (k == "active_sheet") {
            jf.activeSheet = readEntityId(r, f);
        } else if (k == "next_id") {
            jf.nextId = readId(r, f);
            if (jf.nextId > IdGenerator::kMaxId) {
                fail(f, std::to_string(jf.nextId) + " is out of range (at most 2^62)");
            }
        } else if (k == "job") {
            jf.job = decodeJob(r, f, jf.jobUnknown);
        } else if (k == "sheets" || k == "layers") {
            const bool sheets = k == "sheets";
            if (r.peek() != JKind::Array) {
                fail(f, "must be a JSON array");
            }
            r.beginArray();
            for (std::size_t i = 0; r.nextElement(); ++i) {
                if (i >= (sheets ? kMaxSheets : kMaxLayers)) {
                    fail(f,
                         std::string("more than ") + std::to_string(sheets ? kMaxSheets : kMaxLayers) +
                             (sheets ? " sheets" : " layers"),
                         CarvError::Code::ResourceLimit);
                }
                r.budget().chargeDocument(kCostRecord);
                ExtraMap unknown;
                if (sheets) {
                    Sheet s = decodeSheet(r, f.at(i), unknown);
                    jf.sheets.emplace_back(std::move(s), std::move(unknown));
                } else {
                    Layer l = decodeLayer(r, f.at(i), unknown);
                    jf.layers.emplace_back(std::move(l), std::move(unknown));
                }
            }
            if ((sheets && jf.sheets.empty()) || (!sheets && jf.layers.empty())) {
                fail(f, sheets ? "must list at least one sheet" : "must list at least one layer");
            }
        } else if (k == "objects") {
            if (r.peek() != JKind::Array) {
                fail(f, "must be a JSON array");
            }
            r.beginArray();
            for (std::size_t i = 0; r.nextElement(); ++i) {
                r.budget().chargeDocument(kCostObject);
                jf.objects.push_back(decodeObject(r, f.at(i)));
            }
        } else if (k == "toolpaths") {
            if (r.peek() != JKind::Array) {
                fail(f, "must be a JSON array");
            }
            r.beginArray();
            for (std::size_t i = 0; r.nextElement(); ++i) {
                if (i >= kMaxToolpaths) {
                    fail(f, "more than " + std::to_string(kMaxToolpaths) + " toolpaths",
                         CarvError::Code::ResourceLimit);
                }
                r.budget().chargeDocument(kCostRecord);
                jf.toolpaths.push_back(decodeToolpath(r, f.at(i)));
            }
        } else if (k == "variables") {
            if (r.peek() != JKind::Object) {
                fail(f, "must be a JSON object");
            }
            r.beginObject();
            std::string name;
            std::set<std::string> seen;
            while (r.nextKey(name)) {
                if (!DocumentVariables::isValidName(name)) {
                    fail(f, "invalid variable name '" + name + "'");
                }
                const double value = readDouble(r, f.field(name));
                r.budget().chargeDocument(kCostRecord + name.size());
                if (!seen.insert(name).second) {
                    fail(f, "duplicate variable '" + name + "'");
                }
                jf.variables.emplace_back(name, value);
            }
        } else {
            rec.keep(jf.rootUnknown);
        }
    }
    for (const std::string_view key : kRootKeys) {
        if (key != "toolpaths") {  // optional: added in format 1.1
            rec.require(key);
        }
    }
    return jf;
}

/// Linear check of the group structure before a Document is built (ADR-0013 §9): every parent
/// exists and is a group that lists the child exactly once, every listed child exists, points
/// back and shares the group's sheet, there is no cycle, and nesting is at most kMaxGroupDepth.
/// `Document::checkIntegrity()` repeats the link checks afterwards; this pass exists so that a
/// pathological shape (cycle, absurd depth) is refused before any Document is built.
void checkGroupStructure(const std::vector<DecodedObject>& objects) {
    using Code = CarvError::Code;
    const std::size_t n = objects.size();
    std::unordered_map<std::uint64_t, std::size_t> index;
    index.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        index.emplace(objects[i].object.id.value, i);
    }
    const std::string where = kJobName;
    std::vector<std::size_t> parentOf(n, n);
    for (std::size_t i = 0; i < n; ++i) {
        const Object& o = objects[i].object;
        if (!o.parent) {
            continue;
        }
        const auto it = index.find(o.parent->value);
        if (it == index.end() || !objects[it->second].object.isGroup()) {
            fail(where, idText(o.id.value) + " has missing or non-group parent " + idText(o.parent->value),
                 Code::IntegrityFailure);
        }
        parentOf[i] = it->second;
    }
    std::vector<std::uint64_t> listedBy(n, 0);
    for (std::size_t g = 0; g < n; ++g) {
        const Object& group = objects[g].object;
        const GroupObject* data = group.asGroup();
        if (!data) {
            continue;
        }
        for (const ObjectId child : data->children) {
            const auto it = index.find(child.value);
            if (it == index.end()) {
                fail(where, "group " + idText(group.id.value) + " lists missing child " + idText(child.value),
                     Code::IntegrityFailure);
            }
            const std::size_t c = it->second;
            if (listedBy[c] != 0) {
                fail(where,
                     idText(child.value) + " is listed by " + idText(listedBy[c]) + " and " +
                         idText(group.id.value),
                     Code::IntegrityFailure);
            }
            listedBy[c] = group.id.value;
            if (objects[c].object.parent != group.id) {
                fail(where,
                     "group " + idText(group.id.value) + " lists " + idText(child.value) +
                         " whose parent differs",
                     Code::IntegrityFailure);
            }
            if (objects[c].object.sheet != group.sheet) {
                fail(where,
                     "child " + idText(child.value) + " is on another sheet than group " +
                         idText(group.id.value),
                     Code::IntegrityFailure);
            }
        }
    }
    for (std::size_t i = 0; i < n; ++i) {
        const Object& o = objects[i].object;
        if (o.parent && listedBy[i] != o.parent->value) {
            fail(where, idText(o.id.value) + " is not listed by its parent " + idText(o.parent->value),
                 Code::IntegrityFailure);
        }
    }
    // Depth by memoised iterative walks up the parent links: each object is visited once.
    constexpr std::int64_t kUnseen = -1;
    constexpr std::int64_t kOnPath = -2;
    std::vector<std::int64_t> depth(n, kUnseen);
    std::vector<std::size_t> chain;
    for (std::size_t i = 0; i < n; ++i) {
        if (depth[i] >= 0) {
            continue;
        }
        chain.clear();
        std::size_t cur = i;
        std::int64_t base = -1;
        for (;;) {
            if (depth[cur] >= 0) {
                base = depth[cur];
                break;
            }
            if (depth[cur] == kOnPath) {
                fail(where, "parent chain of " + idText(objects[cur].object.id.value) + " loops",
                     Code::IntegrityFailure);
            }
            depth[cur] = kOnPath;
            chain.push_back(cur);
            if (parentOf[cur] == n) {
                break;
            }
            cur = parentOf[cur];
        }
        for (std::size_t k = chain.size(); k-- > 0;) {
            depth[chain[k]] = ++base;
            if (base > kMaxGroupDepth) {
                fail(where, "groups nest deeper than " + std::to_string(kMaxGroupDepth) + " levels",
                     Code::ResourceLimit);
            }
        }
    }
}

std::string versionText(std::int64_t major, std::int64_t minor) {
    return std::to_string(major) + "." + std::to_string(minor);
}

/// Streams one JSON member through `decode`, then checks the member's size and CRC.
template <class Decode>
auto decodeMember(detail::ZipArchive& zip, const detail::ZipEntry& entry, LoadBudget& budget, Decode decode) {
    std::string error;
    auto stream = zip.stream(entry, error);
    if (!stream) {
        throw LoadFailure{CarvError::Code::InvalidArchive, error};
    }
    JsonReader reader(stream->source(), budget, entry.name);
    try {
        auto result = decode(reader);
        reader.finish();
        if (!stream->finish(error)) {
            throw LoadFailure{CarvError::Code::InvalidArchive, error};
        }
        return result;
    } catch (const LoadFailure& f) {
        // A parse error may just be the visible symptom of a damaged member: for a member small
        // enough to drain cheaply, check its CRC and report damage instead. Limit breaches are
        // reported as they are (draining a bomb is exactly what the limits avoid).
        if (f.code != CarvError::Code::ResourceLimit && f.code != CarvError::Code::InvalidArchive &&
            entry.size <= kDrainForCrcBytes && !stream->finish(error)) {
            throw LoadFailure{CarvError::Code::InvalidArchive, error};
        }
        throw;
    }
}

LoadedCarv loadImpl(std::span<const std::uint8_t> bytes, const detail::LoadLimits& limits,
                    const detail::ZipLimits& zipLimits) {
    detail::ZipArchive zip(bytes);
    {
        std::string error;
        bool limit = false;
        if (!zip.open(zipLimits, error, limit)) {
            throw LoadFailure{limit ? CarvError::Code::ResourceLimit : CarvError::Code::InvalidArchive,
                              error};
        }
    }
    const detail::ZipEntry* manifestEntry = zip.find(kManifestName);
    if (!manifestEntry) {
        throw LoadFailure{CarvError::Code::MissingMember,
                          "the archive has no manifest.json (not a .carv file?)"};
    }
    LoadBudget budget(limits);  // one budget for every member of the archive
    budget.setArchiveBytes(bytes.size());  // toolpath moves: at most one per byte of the file
    auto extras = std::make_shared<CarvExtras::Impl>();
    LoadedCarv out{};

    // --- manifest.json: checked before job.json is read so a newer major version fails cleanly.
    ManifestFile manifest = decodeMember(zip, *manifestEntry, budget, decodeManifest);
    if (manifest.format != kFormatName) {
        fail(std::string(kManifestName) + ".format", "is not \"carvetto\" (not a Carvetto file)");
    }
    if (manifest.major > kCarvFormatMajor) {
        throw LoadFailure{CarvError::Code::UnsupportedVersion,
                          "file format version " + versionText(manifest.major, manifest.minor) +
                              " is newer than this Carvetto can read (" +
                              versionText(kCarvFormatMajor, kCarvFormatMinor) + "); please update Carvetto"};
    }
    if (manifest.major < kCarvFormatMajor) {
        throw LoadFailure{
            CarvError::Code::UnsupportedVersion,
            "file format version " + versionText(manifest.major, manifest.minor) + " is not supported"};
    }
    out.formatMajor = manifest.major;
    out.formatMinor = manifest.minor;
    out.appVersion = std::move(manifest.appVersion);
    extras->manifest = std::move(manifest.unknown);

    // --- job.json
    const detail::ZipEntry* jobEntry = zip.find(kJobName);
    if (!jobEntry) {
        throw LoadFailure{CarvError::Code::MissingMember, "the archive has no job.json"};
    }
    JobFile jf = decodeMember(zip, *jobEntry, budget, decodeJobFile);
    extras->root = std::move(jf.rootUnknown);
    extras->job = std::move(jf.jobUnknown);

    // --- other members, kept verbatim under the held-bytes cap: max(64 MiB, 32 x archive size).
    // Binary members are read incrementally; an unknown .json member is first streamed through
    // the strict reader (validated and charged to the budget, never buffered whole) and only then
    // copied if it fits.
    const std::uint64_t heldCap =
        std::max(budget.limits().minHeldBytes, budget.limits().heldBytesPerArchiveByte * bytes.size());
    std::uint64_t held = 0;
    // Toolpath move caches named by job.json: member -> index in jf.toolpaths (ADR-0021).
    std::map<std::string, std::size_t> caches;
    for (std::size_t i = 0; i < jf.toolpaths.size(); ++i) {
        if (const auto& member = jf.toolpaths[i].cacheMember) {
            if (!caches.emplace(*member, i).second) {
                fail(std::string(kJobName) + ".toolpaths[" + std::to_string(i) + "].moves.member",
                     "'" + *member + "' is named by two toolpaths");
            }
        }
    }
    std::set<std::string> cachesRead;
    std::map<std::string, InterimToolpath> interim;  // by member name, i.e. creation order
    const auto hold = [&](const detail::ZipEntry& e) {
        held += e.size;
        if (held > heldCap) {
            throw LoadFailure{CarvError::Code::ResourceLimit,
                              "preserved members would hold more than " + std::to_string(heldCap >> 20) +
                                  " MiB in memory (" + std::to_string(heldCap) + " bytes allowed for a " +
                                  std::to_string(bytes.size()) + "-byte file); member '" + e.name + "'"};
        }
    };
    for (const detail::ZipEntry& e : zip.entries()) {
        if (e.name == kManifestName || e.name == kJobName) {
            continue;
        }
        if (e.toolpathCache) {
            const auto it = caches.find(e.name);
            if (it == caches.end()) {
                continue;  // a cache no toolpath names: stale, dropped unread (ADR-0021)
            }
            DecodedToolpath& d = jf.toolpaths[it->second];
            // The declared size must fit the declared count before anything is inflated
            // (1 to kMaxMoveBytes bytes per move), so the move budget bounds the work.
            const std::uint64_t count = d.moveCount;
            if (e.size < detail::kMovesHeaderBytes + count ||
                (e.size - detail::kMovesHeaderBytes) / detail::kMaxMoveBytes > count) {
                throw LoadFailure{CarvError::Code::InvalidContent,
                                  e.name + ": " + std::to_string(e.size) + " bytes cannot hold " +
                                      std::to_string(count) + " moves"};
            }
            std::string error;
            auto stream = zip.stream(e, error);
            if (!stream) {
                throw LoadFailure{CarvError::Code::InvalidArchive, error};
            }
            std::vector<toolpath::Move> moves;
            const std::string defect = detail::decodeMoves(stream->source(), count, moves);
            if (!stream->finish(error)) {
                throw LoadFailure{CarvError::Code::InvalidArchive, error};  // damage, not bad content
            }
            if (!defect.empty()) {
                throw LoadFailure{CarvError::Code::InvalidContent, e.name + ": " + defect};
            }
            d.record.moves = shareMoves(std::move(moves));
            cachesRead.insert(e.name);
            continue;
        }
        if (detail::isInterimToolpathName(e.name)) {
            // carvetto-cli's interim storage before format 1.1 (ADR-0019): becomes a record.
            // Two passes: count the moves under a throwaway budget with the same limits, then
            // decode for real with exactly that many reserved. Interim members come from the
            // T-0122 CLI, whose files held at most ~350,000 moves, so the extra pass is cheap.
            std::uint64_t count = 0;
            {
                LoadBudget probe(budget.limits());
                probe.setArchiveBytes(bytes.size());
                decodeMember(zip, e, probe, [&count](JsonReader& r) { return decodeInterim(r, true, 0, &count); });
            }
            interim.emplace(e.name, decodeMember(zip, e, budget, [count](JsonReader& r) {
                                return decodeInterim(r, false, count, nullptr);
                            }));
            continue;
        }
        hold(e);
        if (e.json) {
            // Validate as JSON while streaming; the canonical text is discarded (the member is
            // kept byte for byte below), but the pass bounds the work and refuses non-JSON.
            decodeMember(zip, e, budget, [](JsonReader& r) {
                r.readCanonical(nullptr);
                return 0;
            });
        }
        std::vector<std::uint8_t> data;
        std::string error;
        if (!zip.readAll(e, data, error)) {
            throw LoadFailure{CarvError::Code::InvalidArchive, error};
        }
        extras->members.emplace(e.name, std::move(data));
    }

    for (const auto& [member, index] : caches) {
        if (!cachesRead.contains(member)) {
            throw LoadFailure{CarvError::Code::MissingMember,
                              std::string(kJobName) + ".toolpaths[" + std::to_string(index) + "].moves.member: the archive has no '" +
                                  member + "'"};
        }
    }

    // --- references and ids
    const std::string where = kJobName;
    std::set<std::uint64_t> allIds;
    std::uint64_t maxId = 0;
    auto claimId = [&](std::uint64_t id, const std::string& w) {
        if (!allIds.insert(id).second) {
            fail(w, "duplicate id " + std::to_string(id));
        }
        maxId = std::max(maxId, id);
    };
    std::set<std::uint64_t> sheetIds;
    for (std::size_t i = 0; i < jf.sheets.size(); ++i) {
        claimId(jf.sheets[i].first.id.value, where + ".sheets[" + std::to_string(i) + "]");
        sheetIds.insert(jf.sheets[i].first.id.value);
    }
    std::set<std::uint64_t> layerIds;
    for (std::size_t i = 0; i < jf.layers.size(); ++i) {
        claimId(jf.layers[i].first.id.value, where + ".layers[" + std::to_string(i) + "]");
        layerIds.insert(jf.layers[i].first.id.value);
    }
    for (std::size_t i = 0; i < jf.objects.size(); ++i) {
        const Object& o = jf.objects[i].object;
        const auto ow = [&]() {
            return where + ".objects[" + std::to_string(i) + "] (id " + std::to_string(o.id.value) + ")";
        };
        if (!allIds.insert(o.id.value).second) {
            fail(ow(), "duplicate id " + std::to_string(o.id.value));
        }
        maxId = std::max(maxId, o.id.value);
        if (!layerIds.contains(o.layer.value)) {
            fail(ow(), "layer " + std::to_string(o.layer.value) + " does not exist");
        }
        if (!sheetIds.contains(o.sheet.value)) {
            fail(ow(), "sheet " + std::to_string(o.sheet.value) + " does not exist");
        }
    }
    for (std::size_t i = 0; i < jf.toolpaths.size(); ++i) {
        claimId(jf.toolpaths[i].record.id.value, where + ".toolpaths[" + std::to_string(i) + "]");
    }
    if (jf.nextId <= maxId) {
        fail(where + ".next_id", std::to_string(jf.nextId) + " must be greater than every id in the file (" +
                                     std::to_string(maxId) + ")");
    }
    if (!sheetIds.contains(jf.activeSheet)) {
        fail(where + ".active_sheet", "sheet " + std::to_string(jf.activeSheet) + " does not exist");
    }
    if (!layerIds.contains(jf.activeLayer)) {
        fail(where + ".active_layer", "layer " + std::to_string(jf.activeLayer) + " does not exist");
    }
    checkGroupStructure(jf.objects);

    // --- Build the document. Everything referenced below was validated above, so none of the
    // Document's programmer-error asserts can fire. The constructor's default sheet and layer are
    // overwritten when the file uses their ids, otherwise removed once the file's ones exist.
    Document doc(jf.job);
    const SheetId defaultSheet = doc.sheets().front().id;
    const LayerId defaultLayer = doc.layers().front().id;
    bool keepDefaultSheet = false;
    for (std::size_t i = 0; i < jf.sheets.size(); ++i) {
        Sheet& s = jf.sheets[i].first;
        if (!jf.sheets[i].second.empty()) {
            extras->sheets[s.id.value] = std::move(jf.sheets[i].second);
        }
        if (s.id == defaultSheet) {
            *doc.findSheet(defaultSheet) = std::move(s);
            keepDefaultSheet = true;
        } else {
            doc.insertSheet(std::move(s), i);
        }
    }
    if (!keepDefaultSheet) {
        doc.takeSheet(defaultSheet);
    }
    bool keepDefaultLayer = false;
    for (std::size_t i = 0; i < jf.layers.size(); ++i) {
        Layer& l = jf.layers[i].first;
        if (!jf.layers[i].second.empty()) {
            extras->layers[l.id.value] = std::move(jf.layers[i].second);
        }
        if (l.id == defaultLayer) {
            *doc.findLayer(defaultLayer) = std::move(l);
            keepDefaultLayer = true;
        } else {
            doc.insertLayer(std::move(l), i);
        }
    }
    if (!keepDefaultLayer) {
        doc.takeLayer(defaultLayer);
    }
    // Restore the id high-water mark: inserting a layer with an explicit id reserves through it,
    // so a temporary layer with id next_id - 1 (greater than every id in use) advances the
    // generator in O(1). It is removed straight away.
    if (doc.idGenerator().peek() < jf.nextId) {
        Layer marker;
        marker.id = LayerId{jf.nextId - 1};
        doc.insertLayer(marker);
        doc.takeLayer(marker.id);
    }
    for (DecodedObject& d : jf.objects) {
        const std::uint64_t id = d.object.id.value;
        if (!d.unknown.empty()) {
            extras->objects[id] = std::move(d.unknown);
        }
        if (d.unknownParams) {
            extras->params[id] = std::move(*d.unknownParams);
        }
        doc.insertObject(std::move(d.object));
    }
    jf.objects.clear();
    jf.objects.shrink_to_fit();
    for (DecodedToolpath& d : jf.toolpaths) {
        if (!d.unknown.empty()) {
            extras->toolpaths[d.record.id.value] = std::move(d.unknown);
        }
        doc.insertToolpath(std::move(d.record));
    }
    // Interim members become records after the file's own, in member-name (creation) order, with
    // fresh ids. Their "layers" names select every layer of that name; names that match no layer
    // select nothing.
    if (!interim.empty() && !doc.idGenerator().canAllocate(interim.size())) {
        throw LoadFailure{CarvError::Code::InvalidContent,
                          "no ids are left for the toolpaths in toolpaths/NNNN.json members"};
    }
    for (auto& [member, legacy] : interim) {
        ToolpathRecord& record = legacy.record;
        if (!legacy.layers || legacy.layers->empty()) {
            record.selection = VectorSelection::allVectors();
        } else {
            // Canonical order, the same as carvetto-cli's: names as listed, each name's layers in
            // stacking order, every layer once.
            for (const std::string& name : *legacy.layers) {
                for (const Layer& l : doc.layers()) {
                    auto& ids = record.selection.layers;
                    if (l.name == name && std::find(ids.begin(), ids.end(), l.id) == ids.end()) {
                        ids.push_back(l.id);
                    }
                }
            }
        }
        doc.insertToolpath(std::move(record));
    }
    // saveCarv names caches by position, so a computed toolpath past the last name could be
    // loaded but never saved again.
    for (std::size_t i = kCarvMaxCachedToolpaths; i < doc.toolpaths().size(); ++i) {
        if (doc.toolpaths()[i].hasMoves()) {
            throw LoadFailure{CarvError::Code::ResourceLimit,
                              "more than " + std::to_string(kCarvMaxCachedToolpaths) +
                                  " toolpaths with computed moves"};
        }
    }
    doc.setActiveSheet(SheetId{jf.activeSheet});
    doc.setActiveLayer(LayerId{jf.activeLayer});
    for (const auto& [name, value] : jf.variables) {
        doc.variables().set(name, value);
    }

    const auto issues = doc.checkIntegrity();
    if (!issues.empty()) {
        std::string msg = "job.json: the document fails its integrity check: " + issues.front().message;
        if (issues.size() > 1) {
            msg += " (and " + std::to_string(issues.size() - 1) + " more issue" +
                   (issues.size() > 2 ? "s" : "") + ")";
        }
        throw LoadFailure{CarvError::Code::IntegrityFailure, msg};
    }

    out.document = std::move(doc);
    if (!extras->empty()) {
        out.extras = CarvExtras(std::move(extras));
    }
    return out;
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// Public API
// ------------------------------------------------------------------------------------------------

CarvResult<std::vector<std::uint8_t>> saveCarv(const Document& doc, const CarvExtras& extras) {
    try {
        const CarvExtras::Impl* ex = extras.impl();

        json manifest = json::object();
        manifest["format"] = kFormatName;
        json version = json::object();
        version["major"] = kCarvFormatMajor;
        version["minor"] = kCarvFormatMinor;
        manifest["format_version"] = std::move(version);
        manifest["app_version"] = CARVETTO_VERSION_STRING;
        mergeUnknown(manifest, ex ? &ex->manifest : nullptr);

        json root = json::object();
        root["active_layer"] = doc.activeLayer().value;
        root["active_sheet"] = doc.activeSheet().value;
        root["next_id"] = doc.idGenerator().peek();
        json job = encodeJob(doc.job());
        mergeUnknown(job, ex ? &ex->job : nullptr);
        root["job"] = std::move(job);

        json sheets = json::array();
        for (const Sheet& s : doc.sheets()) {
            json j = encodeSheet(s);
            mergeUnknown(j, findExtra(ex, &CarvExtras::Impl::sheets, s.id.value));
            sheets.push_back(std::move(j));
        }
        root["sheets"] = std::move(sheets);

        json layers = json::array();
        for (const Layer& l : doc.layers()) {
            json j = encodeLayer(l);
            mergeUnknown(j, findExtra(ex, &CarvExtras::Impl::layers, l.id.value));
            layers.push_back(std::move(j));
        }
        root["layers"] = std::move(layers);

        json objects = json::array();
        for (const auto& [id, obj] : doc.objects()) {  // id order = creation order
            json j = encodeObject(obj, ex);
            mergeUnknown(j, findExtra(ex, &CarvExtras::Impl::objects, id.value));
            objects.push_back(std::move(j));
        }
        root["objects"] = std::move(objects);

        // Toolpath records in job order; computed moves go to toolpaths/NNNN.bin, NNNN being the
        // 1-based position (ADR-0021). Checked first so no file the loader refuses is written.
        std::uint64_t totalMoves = 0;
        for (std::size_t i = 0; i < doc.toolpaths().size(); ++i) {
            const ToolpathRecord& tp = doc.toolpaths()[i];
            if (tp.hasMoves() && i >= kCarvMaxCachedToolpaths) {
                return CarvError{CarvError::Code::ResourceLimit,
                                 "cannot save more than " + std::to_string(kCarvMaxCachedToolpaths) +
                                     " toolpaths with computed moves"};
            }
            totalMoves += tp.moveCount();
        }
        if (totalMoves > kCarvMaxToolpathMoves) {
            return CarvError{CarvError::Code::ResourceLimit,
                             "the toolpaths hold " + std::to_string(totalMoves) + " moves; a .carv file holds at most " +
                                 std::to_string(kCarvMaxToolpathMoves) +
                                 " (use fewer moves: larger stepover or pass depth, coarser tolerance, or split the job)"};
        }
        // Every member counts against the loader's member limit: manifest, job, caches, and the
        // preserved members a cache does not replace.
        const detail::ZipLimits zipLimits;
        {
            std::set<std::string> names;
            for (std::size_t i = 0; i < doc.toolpaths().size(); ++i) {
                if (doc.toolpaths()[i].hasMoves()) {
                    names.insert(cacheMemberName(i));
                }
            }
            if (ex) {
                for (const auto& [name, data] : ex->members) {
                    names.insert(name);
                }
            }
            const std::uint64_t count = 2 + names.size();
            if (count > zipLimits.maxMembers) {
                return CarvError{CarvError::Code::ResourceLimit,
                                 "the file would have " + std::to_string(count) + " archive members (" +
                                     std::to_string(names.size() - (ex ? ex->members.size() : 0)) +
                                     " toolpath move caches among them); a .carv file holds at most " +
                                     std::to_string(zipLimits.maxMembers)};
            }
        }
        std::map<std::string, std::vector<std::uint8_t>> caches;
        json toolpaths = json::array();
        for (std::size_t i = 0; i < doc.toolpaths().size(); ++i) {
            const ToolpathRecord& tp = doc.toolpaths()[i];
            std::string member;
            if (tp.hasMoves()) {
                member = cacheMemberName(i);
                std::vector<std::uint8_t> bytes;
                bytes.reserve(detail::kMovesHeaderBytes + tp.moveCount() * 26);
                const std::string error = detail::encodeMoves(*tp.moves, bytes);
                if (!error.empty()) {
                    return CarvError{CarvError::Code::InvalidContent,
                                     "toolpath '" + tp.name + "' (#" + std::to_string(tp.id.value) + "): " + error};
                }
                caches.emplace(member, std::move(bytes));
            }
            toolpaths.push_back(encodeToolpath(tp, tp.hasMoves() ? &member : nullptr, ex));
        }
        root["toolpaths"] = std::move(toolpaths);

        json variables = json::object();
        for (const auto& [name, value] : doc.variables().all()) {
            variables[name] = value;
        }
        root["variables"] = std::move(variables);
        mergeUnknown(root, ex ? &ex->root : nullptr);

        std::vector<detail::ZipMember> members;
        members.push_back({kManifestName, canonicalBytes(manifest)});
        members.push_back({kJobName, canonicalBytes(root)});
        // Move caches and preserved members, merged in name order (a cache wins over a preserved
        // member of the same name, which `CarvExtras::withMember` never creates).
        auto cache = caches.begin();
        const auto flushCachesBefore = [&](const std::string* name) {
            while (cache != caches.end() && (!name || cache->first <= *name)) {
                members.push_back({cache->first, std::move(cache->second)});
                ++cache;
            }
        };
        if (ex) {
            for (const auto& [name, data] : ex->members) {  // sorted by name
                const bool shadowed = caches.contains(name);
                flushCachesBefore(&name);
                if (!shadowed) {
                    members.push_back({name, data});
                }
            }
        }
        flushCachesBefore(nullptr);
        // The loader's archive caps (ADR-0013 §9), checked on what is about to be written.
        std::uint64_t totalBytes = 0;
        for (const detail::ZipMember& m : members) {
            const bool isJson = m.name.size() >= 5 && m.name.compare(m.name.size() - 5, 5, ".json") == 0;
            const std::uint64_t cap = isJson ? zipLimits.maxJsonMemberBytes : zipLimits.maxBinaryMemberBytes;
            if (m.data.size() > cap) {
                return CarvError{CarvError::Code::ResourceLimit,
                                 "member '" + m.name + "' would hold " + std::to_string(m.data.size()) +
                                     " bytes; the limit is " + std::to_string(cap)};
            }
            totalBytes += m.data.size();
        }
        if (totalBytes > zipLimits.maxTotalBytes) {
            return CarvError{CarvError::Code::ResourceLimit,
                             "the file would expand to " + std::to_string(totalBytes) + " bytes; the limit is " +
                                 std::to_string(zipLimits.maxTotalBytes)};
        }
        detail::ZipWriteResult zip = detail::writeZip(members);
        if (!zip.ok) {
            return CarvError{CarvError::Code::InvalidArchive, zip.error};
        }
        // At most kCarvMovesPerFileByte moves per byte of the file (owner decision 2026-11-02);
        // only known now that the file is compressed.
        if (totalMoves > zip.bytes.size() * kCarvMovesPerFileByte) {
            return CarvError{CarvError::Code::ResourceLimit,
                             "the toolpaths hold " + std::to_string(totalMoves) + " moves but the file would be only " +
                                 std::to_string(zip.bytes.size()) + " bytes; a .carv file holds at most " +
                                 std::to_string(kCarvMovesPerFileByte) +
                                 " move per byte of its size (are the toolpaths highly repetitive?)"};
        }
        // The promise "saveCarv never writes a file loadCarv refuses" is made structurally true:
        // the finished archive goes through the real load path, with the loader's own limits.
        // Cost: about one load (2M moves: ~0.2 s Release, plus the decoded copy in memory while
        // it runs). The checks above only give clearer messages for the common cases.
        try {
            (void)loadImpl(zip.bytes, detail::LoadLimits{}, detail::ZipLimits{});
        } catch (const LoadFailure& f) {
            return CarvError{f.code, "the file would not load again: " + f.message};
        } catch (const std::bad_alloc&) {
            return CarvError{CarvError::Code::ResourceLimit,
                             "the file would not load again: out of memory while checking it"};
        }
        return std::move(zip.bytes);
    } catch (const std::bad_alloc&) {
        return CarvError{CarvError::Code::Io, "out of memory while saving"};
    } catch (const std::exception& e) {
        return CarvError{CarvError::Code::Io, std::string("cannot encode the document: ") + e.what()};
    }
}

CarvStatus saveCarv(const Document& doc, const std::filesystem::path& path, const CarvExtras& extras) {
    auto bytes = saveCarv(doc, extras);
    if (!bytes) {
        return bytes.error();
    }
    std::filesystem::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream file(tmp, std::ios::binary | std::ios::trunc);
        if (!file) {
            return CarvError{CarvError::Code::Io, "cannot open '" + tmp.string() + "' for writing"};
        }
        const auto& data = bytes.value();
        file.write(reinterpret_cast<const char*>(data.data()), static_cast<std::streamsize>(data.size()));
        file.close();
        if (!file) {
            std::error_code ignored;
            std::filesystem::remove(tmp, ignored);
            return CarvError{CarvError::Code::Io, "cannot write '" + tmp.string() + "'"};
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, path, ec);
    if (ec) {
        std::error_code ignored;
        std::filesystem::remove(tmp, ignored);
        return CarvError{CarvError::Code::Io, "cannot replace '" + path.string() + "': " + ec.message()};
    }
    return std::monostate{};
}

namespace detail {

CarvResult<LoadedCarv> loadCarvWithLimits(std::span<const std::uint8_t> bytes, const LoadLimits& limits,
                                          const ZipLimits& zipLimits) {
    try {
        return loadImpl(bytes, limits, zipLimits);
    } catch (const LoadFailure& f) {
        return CarvError{f.code, f.message};
    } catch (const std::bad_alloc&) {
        return CarvError{CarvError::Code::ResourceLimit, "out of memory while loading"};
    } catch (const std::exception& e) {
        return CarvError{CarvError::Code::InvalidContent, std::string("cannot load: ") + e.what()};
    }
}

}  // namespace detail

CarvResult<LoadedCarv> loadCarv(std::span<const std::uint8_t> bytes) {
    try {
        return loadImpl(bytes, detail::LoadLimits{}, detail::ZipLimits{});
    } catch (const LoadFailure& f) {
        return CarvError{f.code, f.message};
    } catch (const std::bad_alloc&) {
        return CarvError{CarvError::Code::ResourceLimit, "out of memory while loading"};
    } catch (const std::exception& e) {
        return CarvError{CarvError::Code::InvalidContent, std::string("cannot load: ") + e.what()};
    }
}

CarvResult<LoadedCarv> loadCarv(const std::filesystem::path& path) {
    std::vector<std::uint8_t> data;
    try {
        std::ifstream file(path, std::ios::binary | std::ios::ate);
        if (!file) {
            return CarvError{CarvError::Code::Io, "cannot open '" + path.string() + "'"};
        }
        const std::streamoff size = file.tellg();
        if (size < 0) {
            return CarvError{CarvError::Code::Io, "cannot read '" + path.string() + "'"};
        }
        data.resize(static_cast<std::size_t>(size));
        file.seekg(0);
        file.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(size));
        if (!file) {
            return CarvError{CarvError::Code::Io, "cannot read '" + path.string() + "'"};
        }
    } catch (const std::exception& e) {
        return CarvError{CarvError::Code::Io, "cannot read '" + path.string() + "': " + e.what()};
    }
    auto result = loadCarv(std::span<const std::uint8_t>(data));
    if (!result) {
        CarvError err = result.error();
        err.message = path.filename().string() + ": " + err.message;
        return err;
    }
    return result;
}

}  // namespace carvetto::document
