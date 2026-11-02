#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/io/dxf.hpp"
#include "dxf_curves.hpp"
#include "dxf_pairs.hpp"
#include "dxf_text.hpp"

// DXF import (ADR-0015). Three stages:
//  1. `PairReader` streams group-code/value pairs (ASCII or binary) without copying strings.
//  2. The section parser collects the header variables, LAYER/STYLE tables, block definitions and
//     model-space entities, decoding each entity into a small struct in its own coordinates.
//  3. The emitter walks model space, expands INSERTs recursively (cycle and depth checks,
//     instance budget), maps everything through OCS -> block -> unit transforms and converts it
//     to contours (exact lines/arcs/Beziers where possible, chordal-tolerance Beziers otherwise)
//     and text items, under span and text budgets.
namespace carvetto::io {

std::string DxfError::describe() const {
    return line > 0 ? "line " + std::to_string(line) + ": " + message : message;
}

std::string DxfWarning::describe() const {
    std::string s = line > 0 ? "line " + std::to_string(line) + ": " + message : message;
    if (count > 1) {
        s += " (x" + std::to_string(count) + ")";
    }
    return s;
}

std::optional<double> insunitsToMillimetres(int code) {
    switch (code) {
        case 1:
            return 25.4;  // inches
        case 2:
            return 304.8;  // feet
        case 3:
            return 1609344.0;  // miles
        case 4:
            return 1.0;  // millimetres
        case 5:
            return 10.0;  // centimetres
        case 6:
            return 1000.0;  // metres
        case 7:
            return 1.0e6;  // kilometres
        case 8:
            return 25.4e-6;  // microinches
        case 9:
            return 0.0254;  // mils
        case 10:
            return 914.4;  // yards
        case 11:
            return 1.0e-7;  // angstroms
        case 12:
            return 1.0e-6;  // nanometres
        case 13:
            return 1.0e-3;  // microns
        case 14:
            return 100.0;  // decimetres
        case 15:
            return 1.0e4;  // decametres
        case 16:
            return 1.0e5;  // hectometres
        case 21:
            return 1200000.0 / 3937.0;  // US survey feet
        default:
            return std::nullopt;
    }
}

namespace {

using detail::Pair;
using detail::PairReader;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::kPi;
using geometry::kTwoPi;
using geometry::LineSpan;
using geometry::Span;
using geometry::Transform2;
using geometry::Vec2;

constexpr double kDegToRad = kPi / 180.0;
/// Largest coordinate magnitude (mm) accepted in the result; larger geometry is skipped.
constexpr double kMaxCoordinate = 1.0e12;

/// True when both components are finite and within kMaxCoordinate.
bool inRange(Vec2 p) {
    return std::fabs(p.x) <= kMaxCoordinate && std::fabs(p.y) <= kMaxCoordinate;
}
/// Largest angle or curve parameter magnitude (radians) accepted. Beyond it a double cannot
/// place a point on the curve precisely (spacing 1.5e-11 rad at 1e5), so the entity is skipped.
constexpr double kMaxAngle = 1.0e5;
/// Distinct warning messages kept; further distinct ones are counted in a final warning.
constexpr std::size_t kMaxDistinctWarnings = 256;

// ------------------------------------------------------------------------- warnings

class Warnings {
   public:
    void add(const std::string& message, std::size_t line) {
        const auto it = index_.find(message);
        if (it != index_.end()) {
            ++list_[it->second].count;
            return;
        }
        if (index_.size() >= kMaxDistinctWarnings) {
            ++suppressed_;
            return;
        }
        index_.emplace(message, list_.size());
        list_.push_back(DxfWarning{line, 1, message});
    }
    std::vector<DxfWarning> take() {
        if (suppressed_ > 0) {
            list_.push_back(DxfWarning{0, suppressed_, "further distinct warnings suppressed"});
        }
        return std::move(list_);
    }

   private:
    std::vector<DxfWarning> list_;
    std::map<std::string, std::size_t> index_;
    std::size_t suppressed_{0};
};

// ------------------------------------------------------------------ decoded entities

struct LineE {
    Vec2 a;
    Vec2 b;
};
/// ARC or CIRCLE (angles in radians, counter-clockwise in the OCS).
struct ArcE {
    Vec2 centre;
    double radius{0.0};
    double start{0.0};
    double sweep{kTwoPi};
    bool circle{false};
};
struct EllipseE {
    Vec2 centre;
    Vec2 major;
    double ratio{1.0};
    double start{0.0};
    double end{kTwoPi};
    double normalSign{1.0};
};
struct Vertex {
    Vec2 p;
    double bulge{0.0};
};
struct PolyE {
    std::vector<Vertex> vertices;
    bool closed{false};
};
struct SplineE {
    detail::NurbsCurve nurbs;
    std::vector<Vec2> fit;
    std::optional<Vec2> startTangent;
    std::optional<Vec2> endTangent;
    bool closedFlag{false};
};
struct TextE {
    DxfTextKind kind{DxfTextKind::Text};
    std::string raw;
    std::string_view style;
    double height{0.0};
    double widthFactor{1.0};
    double rotation{0.0};
    Vec2 p;
    std::optional<Vec2> p2;
    DxfHAlign h{DxfHAlign::Left};
    DxfVAlign v{DxfVAlign::Baseline};
    double refWidth{0.0};
    bool mirrored{false};
};
struct InsertE {
    std::string_view block;
    Vec2 p;
    double sx{1.0};
    double sy{1.0};
    double rotation{0.0};
    int cols{1};
    int rows{1};
    double dx{0.0};
    double dy{0.0};
};

/// Carvetto XDATA written by `exportDxf`: which contour a piece belongs to.
struct PieceTag {
    std::int64_t contour{0};
    std::int64_t piece{0};
    std::int64_t count{1};
    bool closed{false};
    std::optional<double> circleStart;
    bool circleClockwise{false};
    /// LWPOLYLINE arcs written as several bulge segments (sweep > pi), restored exactly.
    struct SplitArc {
        std::size_t firstSegment{0};
        std::size_t segments{0};
        ArcSpan arc;
    };
    std::vector<SplitArc> splitArcs;
};

struct Entity {
    std::string_view layer{"0"};
    std::size_t line{0};
    Transform2 ocs{};
    std::variant<LineE, ArcE, EllipseE, PolyE, SplineE, TextE, InsertE> geom;
    std::optional<PieceTag> tag;
};

struct Block {
    Vec2 base;
    bool external{false};
    std::string_view name;
    std::vector<Entity> entities;
};

struct LayerRec {
    std::string_view name;
    int colour{7};
    std::optional<std::uint32_t> rgb;
    std::int64_t flags{0};
};

struct RawEntity {
    std::string_view type;
    std::size_t line{0};
    std::vector<Pair> pairs;
};

// ------------------------------------------------------------------------- parser

class Parser {
   public:
    Parser(std::span<const std::uint8_t> bytes, Warnings& warnings) : reader_(bytes), warnings_(warnings) {}

    bool parse();
    const std::optional<DxfError>& error() const { return error_; }

    bool binary() const { return reader_.binary(); }
    std::string_view version;
    std::string_view codepage;
    std::optional<std::int64_t> insunits;
    std::optional<std::int64_t> measurement;
    std::vector<LayerRec> layers;
    std::map<std::string, std::string_view> styleFonts;  // upper-case style name -> font file
    std::map<std::string, Block> blocks;                 // upper-case block name -> block
    std::vector<Entity> model;

   private:
    bool fail(DxfError::Code code, std::string message, std::size_t line) {
        if (!error_) {
            error_ = DxfError{code, std::move(message), line};
        }
        return false;
    }
    bool readerFailed() {
        if (reader_.error()) {
            error_ = reader_.error();
            return true;
        }
        return false;
    }
    bool readRaw(RawEntity& raw);
    bool peekType(std::string_view& type);
    bool parseHeader();
    bool parseTables();
    bool parseBlocks();
    bool skipSection();
    /// Reads entities until ENDBLK/ENDSEC/EOF (not consumed); returns false on error.
    bool parseEntityList(std::vector<Entity>& out, bool modelSpace, std::string_view sectionName);
    void decode(const RawEntity& raw, std::vector<Entity>& out, bool modelSpace);
    void decodePolyline(const RawEntity& head, std::vector<Entity>& out);
    std::optional<Entity> decodeText(const RawEntity& raw);
    bool readChildren(std::string_view childType, std::vector<RawEntity>& children);
    void warn(const std::string& message, std::size_t line) { warnings_.add(message, line); }

    PairReader reader_;
    Warnings& warnings_;
    std::optional<DxfError> error_;
};

bool Parser::peekType(std::string_view& type) {
    Pair p;
    if (!reader_.peek(p)) {
        return false;
    }
    if (p.code != 0) {
        return fail(DxfError::Code::InvalidFormat, "expected group code 0, found " + std::to_string(p.code),
                    p.line);
    }
    type = detail::trim(p.text);
    return true;
}

bool Parser::readRaw(RawEntity& raw) {
    Pair p;
    if (!reader_.next(p)) {
        return false;
    }
    if (p.code != 0) {
        return fail(DxfError::Code::InvalidFormat, "expected group code 0, found " + std::to_string(p.code),
                    p.line);
    }
    raw.type = detail::trim(p.text);
    raw.line = p.line;
    raw.pairs.clear();
    while (reader_.peek(p) && p.code != 0) {
        reader_.next(p);
        raw.pairs.push_back(p);
    }
    return !readerFailed();
}

bool Parser::parse() {
    bool sawSection = false;
    bool sawEof = false;
    Pair p;
    while (reader_.next(p)) {
        if (p.code != 0) {
            return fail(DxfError::Code::InvalidFormat,
                        sawSection ? "expected SECTION or EOF, found group code " + std::to_string(p.code)
                                   : "not a DXF file (expected '0 SECTION')",
                        p.line);
        }
        const std::string_view name = detail::trim(p.text);
        if (detail::iequals(name, "EOF")) {
            sawEof = true;
            break;
        }
        if (!detail::iequals(name, "SECTION")) {
            return fail(DxfError::Code::InvalidFormat,
                        "expected SECTION, found '" + detail::printable(name) + "'", p.line);
        }
        const std::size_t sectionLine = p.line;
        if (!reader_.next(p)) {
            if (readerFailed()) {
                return false;
            }
            return fail(DxfError::Code::Truncated, "data ends after SECTION", sectionLine);
        }
        if (p.code != 2) {
            return fail(DxfError::Code::InvalidFormat, "SECTION without a name (group 2)", p.line);
        }
        sawSection = true;
        const std::string_view section = detail::trim(p.text);
        bool ok = false;
        if (detail::iequals(section, "HEADER")) {
            ok = parseHeader();
        } else if (detail::iequals(section, "TABLES")) {
            ok = parseTables();
        } else if (detail::iequals(section, "BLOCKS")) {
            ok = parseBlocks();
        } else if (detail::iequals(section, "ENTITIES")) {
            ok = parseEntityList(model, true, "ENTITIES");
            if (ok) {
                RawEntity end;
                ok = readRaw(end);
                if (ok && !detail::iequals(end.type, "ENDSEC")) {
                    return fail(DxfError::Code::InvalidFormat,
                                "ENTITIES section ended by " + detail::printable(end.type), end.line);
                }
            }
        } else {
            ok = skipSection();
        }
        if (!ok) {
            if (!error_ && !readerFailed()) {
                fail(DxfError::Code::Truncated,
                     "data ends inside the " + detail::printable(section) + " section", sectionLine);
            }
            return false;
        }
    }
    if (readerFailed()) {
        return false;
    }
    if (!sawSection) {
        return fail(DxfError::Code::InvalidFormat, "not a DXF file (no sections)", 0);
    }
    if (!sawEof) {
        warn("file has no EOF marker", reader_.position());
    }
    return true;
}

bool Parser::skipSection() {
    Pair p;
    while (reader_.next(p)) {
        if (p.code == 0 && detail::iequals(detail::trim(p.text), "ENDSEC")) {
            return true;
        }
    }
    return false;
}

bool Parser::parseHeader() {
    Pair p;
    std::string_view variable;
    while (reader_.next(p)) {
        if (p.code == 0) {
            if (detail::iequals(detail::trim(p.text), "ENDSEC")) {
                return true;
            }
            continue;
        }
        if (p.code == 9) {
            variable = detail::trim(p.text);
            continue;
        }
        if (variable == "$ACADVER" && p.code == 1) {
            version = detail::trim(p.text);
        } else if (variable == "$INSUNITS" && p.code == 70) {
            insunits = p.integer;
        } else if (variable == "$MEASUREMENT" && p.code == 70) {
            measurement = p.integer;
        } else if (variable == "$DWGCODEPAGE" && p.code == 3) {
            codepage = detail::trim(p.text);
        }
    }
    return false;
}

bool Parser::parseTables() {
    RawEntity raw;
    std::string_view type;
    while (peekType(type)) {
        if (!readRaw(raw)) {
            return false;
        }
        if (detail::iequals(raw.type, "ENDSEC")) {
            return true;
        }
        if (detail::iequals(raw.type, "EOF")) {
            return fail(DxfError::Code::Truncated, "EOF inside the TABLES section", raw.line);
        }
        if (detail::iequals(raw.type, "LAYER")) {
            LayerRec rec;
            for (const Pair& q : raw.pairs) {
                switch (q.code) {
                    case 2:
                        rec.name = detail::trim(q.text);
                        break;
                    case 62:
                        rec.colour = static_cast<int>(std::clamp<std::int64_t>(q.integer, -32768, 32767));
                        break;
                    case 70:
                        rec.flags = q.integer;
                        break;
                    case 420:
                        rec.rgb = static_cast<std::uint32_t>(q.integer) & 0xFFFFFFu;
                        break;
                    default:
                        break;
                }
            }
            if (!rec.name.empty()) {
                layers.push_back(rec);
            }
        } else if (detail::iequals(raw.type, "STYLE")) {
            std::string_view name;
            std::string_view font;
            for (const Pair& q : raw.pairs) {
                if (q.code == 2) {
                    name = detail::trim(q.text);
                } else if (q.code == 3) {
                    font = detail::trim(q.text);
                }
            }
            if (!name.empty()) {
                styleFonts.emplace(detail::upper(name), font);
            }
        }
    }
    return false;
}

bool Parser::parseBlocks() {
    RawEntity raw;
    std::string_view type;
    while (peekType(type)) {
        if (!readRaw(raw)) {
            return false;
        }
        if (detail::iequals(raw.type, "ENDSEC")) {
            return true;
        }
        if (detail::iequals(raw.type, "EOF")) {
            return fail(DxfError::Code::Truncated, "EOF inside the BLOCKS section", raw.line);
        }
        if (!detail::iequals(raw.type, "BLOCK")) {
            continue;  // stray ENDBLK or unknown records
        }
        Block block;
        std::int64_t flags = 0;
        for (const Pair& q : raw.pairs) {
            switch (q.code) {
                case 2:
                    block.name = detail::trim(q.text);
                    break;
                case 10:
                    block.base.x = q.real;
                    break;
                case 20:
                    block.base.y = q.real;
                    break;
                case 70:
                    flags = q.integer;
                    break;
                default:
                    break;
            }
        }
        block.external = (flags & 4) != 0;
        const std::size_t blockLine = raw.line;
        if (!parseEntityList(block.entities, false, "BLOCK")) {
            return false;
        }
        if (!peekType(type)) {
            return false;
        }
        if (detail::iequals(type, "ENDBLK")) {
            if (!readRaw(raw)) {
                return false;
            }
        } else {
            warn("block without ENDBLK", blockLine);
        }
        const std::string key = detail::upper(block.name);
        if (blocks.find(key) != blocks.end()) {
            warn("duplicate block definition '" + detail::printable(block.name) + "' ignored", blockLine);
        } else {
            blocks.emplace(key, std::move(block));
        }
    }
    return false;
}

bool Parser::readChildren(std::string_view childType, std::vector<RawEntity>& children) {
    std::string_view type;
    while (peekType(type)) {
        if (detail::iequals(type, childType)) {
            RawEntity child;
            if (!readRaw(child)) {
                return false;
            }
            children.push_back(std::move(child));
            continue;
        }
        if (detail::iequals(type, "SEQEND")) {
            RawEntity end;
            return readRaw(end);
        }
        return true;  // missing SEQEND: tolerate
    }
    return !error_ && !readerFailed();
}

bool Parser::parseEntityList(std::vector<Entity>& out, bool modelSpace, std::string_view sectionName) {
    RawEntity raw;
    std::string_view type;
    while (true) {
        if (!peekType(type)) {
            if (!error_ && !readerFailed()) {
                fail(DxfError::Code::Truncated,
                     "data ends inside the " + std::string(sectionName) + " section", reader_.position());
            }
            return false;
        }
        if (detail::iequals(type, "ENDSEC") || detail::iequals(type, "ENDBLK") ||
            detail::iequals(type, "EOF")) {
            if (detail::iequals(type, "EOF")) {
                return fail(DxfError::Code::Truncated,
                            "EOF inside the " + std::string(sectionName) + " section", reader_.position());
            }
            return true;
        }
        if (!readRaw(raw)) {
            return false;
        }
        bool paperSpace = false;
        for (const Pair& q : raw.pairs) {
            if (q.code == 67 && q.integer == 1) {
                paperSpace = true;
            }
        }
        if (modelSpace && paperSpace) {
            warn("paper-space entities skipped", raw.line);
            std::vector<RawEntity> children;
            if (detail::iequals(raw.type, "POLYLINE") && !readChildren("VERTEX", children)) {
                return false;
            }
            continue;
        }
        if (detail::iequals(raw.type, "POLYLINE")) {
            decodePolyline(raw, out);
            if (error_) {
                return false;
            }
            continue;
        }
        decode(raw, out, modelSpace);
        if (detail::iequals(raw.type, "INSERT")) {
            bool attribsFollow = false;
            for (const Pair& q : raw.pairs) {
                if (q.code == 66 && q.integer != 0) {
                    attribsFollow = true;
                }
            }
            if (attribsFollow) {
                std::vector<RawEntity> attribs;
                if (!readChildren("ATTRIB", attribs)) {
                    return false;
                }
                for (const RawEntity& a : attribs) {
                    if (std::optional<Entity> t = decodeText(a)) {
                        out.push_back(std::move(*t));
                    }
                }
            }
        }
    }
}

/// OCS of an entity from its extrusion direction: identity for +Z, a mirror in X for -Z
/// (the arbitrary-axis algorithm gives X axis = (-1, 0, 0) there); nullopt when tilted.
std::optional<Transform2> ocsFor(const std::vector<Pair>& pairs) {
    double nx = 0.0;
    double ny = 0.0;
    double nz = 1.0;
    for (const Pair& q : pairs) {
        if (q.code == 210) nx = q.real;
        if (q.code == 220) ny = q.real;
        if (q.code == 230) nz = q.real;
    }
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (!(len > 0.0) || !std::isfinite(len)) {
        return Transform2{};
    }
    if (std::fabs(nx) > 1e-9 * len || std::fabs(ny) > 1e-9 * len) {
        return std::nullopt;
    }
    return nz > 0.0 ? Transform2{} : Transform2::mirrorX();
}

std::string_view layerOf(const std::vector<Pair>& pairs) {
    for (const Pair& q : pairs) {
        if (q.code == 8) {
            const std::string_view name = detail::trim(q.text);
            return name.empty() ? std::string_view{"0"} : name;
        }
    }
    return "0";
}

std::optional<PieceTag> carvettoTag(const std::vector<Pair>& pairs) {
    bool inApp = false;
    std::vector<std::int64_t> ints;
    std::vector<double> reals;
    for (const Pair& q : pairs) {
        if (q.code == 1001) {
            inApp = detail::trim(q.text) == "CARVETTO";
            continue;
        }
        if (!inApp) {
            continue;
        }
        if (q.code == 1071) {
            ints.push_back(q.integer);
        } else if (q.code == 1040) {
            reals.push_back(q.real);
        }
    }
    if (ints.size() < 5 || ints[0] != 1 || ints[3] < 1 || ints[2] < 0 || ints[2] >= ints[3]) {
        return std::nullopt;
    }
    PieceTag tag;
    tag.contour = ints[1];
    tag.piece = ints[2];
    tag.count = ints[3];
    tag.closed = ints[4] != 0;
    const std::size_t extra = ints.size() - 5;
    if (extra == 1 && reals.size() == 1) {
        // CIRCLE: direction and start angle.
        tag.circleStart = reals[0];
        tag.circleClockwise = ints[5] != 0;
    } else if (extra >= 1 && ints[5] >= 1) {
        // LWPOLYLINE: split arcs (see exportDxf / ADR-0015).
        const auto n = static_cast<std::size_t>(std::min<std::int64_t>(ints[5], 1'000'000));
        if (extra == 1 + 3 * n && reals.size() == 5 * n) {
            for (std::size_t i = 0; i < n; ++i) {
                const std::int64_t first = ints[6 + 3 * i];
                const std::int64_t count = ints[7 + 3 * i];
                PieceTag::SplitArc a;
                a.arc.centre = {reals[5 * i], reals[5 * i + 1]};
                a.arc.radius = reals[5 * i + 2];
                a.arc.startAngle = reals[5 * i + 3];
                a.arc.endAngle = reals[5 * i + 4];
                a.arc.direction =
                    ints[8 + 3 * i] != 0 ? geometry::ArcDirection::CW : geometry::ArcDirection::CCW;
                if (first < 0 || count < 2 || !(a.arc.radius > 0.0)) {
                    tag.splitArcs.clear();
                    break;
                }
                a.firstSegment = static_cast<std::size_t>(first);
                a.segments = static_cast<std::size_t>(count);
                tag.splitArcs.push_back(a);
            }
        }
    }
    return tag;
}

std::optional<Entity> Parser::decodeText(const RawEntity& raw) {
    TextE t;
    const bool attrib = detail::iequals(raw.type, "ATTRIB");
    const bool mtext = detail::iequals(raw.type, "MTEXT");
    t.kind = mtext ? DxfTextKind::MText : (attrib ? DxfTextKind::Attrib : DxfTextKind::Text);
    std::optional<Vec2> direction;
    double nz = 1.0;
    std::int64_t hcode = 0;
    std::int64_t vcode = 0;
    std::int64_t attachment = 1;
    bool hasP2x = false;
    for (const Pair& q : raw.pairs) {
        if (q.code == 101) {
            break;  // embedded object data (multi-line attributes, MTEXT columns) is not text
        }
        switch (q.code) {
            case 1:
            case 3:
                if (q.code == 1 || mtext) {
                    t.raw.append(q.text.data(), q.text.size());
                }
                break;
            case 7:
                t.style = detail::trim(q.text);
                break;
            case 10:
                t.p.x = q.real;
                break;
            case 20:
                t.p.y = q.real;
                break;
            case 11:
                if (mtext) {
                    direction = Vec2{q.real, direction ? direction->y : 0.0};
                } else {
                    t.p2 = Vec2{q.real, t.p2 ? t.p2->y : 0.0};
                    hasP2x = true;
                }
                break;
            case 21:
                if (mtext) {
                    direction = Vec2{direction ? direction->x : 0.0, q.real};
                } else if (hasP2x) {
                    t.p2 = Vec2{t.p2->x, q.real};
                }
                break;
            case 40:
                t.height = q.real;
                break;
            case 41:
                if (mtext) {
                    t.refWidth = q.real;
                } else {
                    t.widthFactor = q.real;
                }
                break;
            case 50:
                t.rotation = mtext ? q.real : q.real * kDegToRad;
                break;
            case 70:
                if (attrib && (q.integer & 1) != 0) {
                    return std::nullopt;  // invisible attribute
                }
                break;
            case 71:
                if (mtext) {
                    attachment = q.integer;
                } else {
                    t.mirrored = (q.integer & 2) != 0;
                }
                break;
            case 72:
                if (!mtext) hcode = q.integer;
                break;
            case 73:
                if (!mtext && !attrib) vcode = q.integer;
                break;
            case 74:
                if (attrib) vcode = q.integer;
                break;
            case 230:
                nz = q.real;
                break;
            default:
                break;
        }
    }
    if (mtext) {
        if (direction && direction->length() > 0.0) {
            t.rotation = direction->angle();
        }
        const std::int64_t a = std::clamp<std::int64_t>(attachment, 1, 9) - 1;
        static constexpr DxfHAlign kH[3] = {DxfHAlign::Left, DxfHAlign::Center, DxfHAlign::Right};
        static constexpr DxfVAlign kV[3] = {DxfVAlign::Top, DxfVAlign::Middle, DxfVAlign::Bottom};
        t.h = kH[a % 3];
        t.v = kV[a / 3];
        t.mirrored = nz < 0.0;
    } else {
        static constexpr DxfHAlign kH[6] = {DxfHAlign::Left,    DxfHAlign::Center, DxfHAlign::Right,
                                            DxfHAlign::Aligned, DxfHAlign::Middle, DxfHAlign::Fit};
        static constexpr DxfVAlign kV[4] = {DxfVAlign::Baseline, DxfVAlign::Bottom, DxfVAlign::Middle,
                                            DxfVAlign::Top};
        t.h = kH[std::clamp<std::int64_t>(hcode, 0, 5)];
        t.v = kV[std::clamp<std::int64_t>(vcode, 0, 3)];
        if (hcode == 0 && vcode == 0) {
            t.p2.reset();  // left/baseline text is placed by its first point only
        }
    }
    Entity e;
    e.layer = layerOf(raw.pairs);
    e.line = raw.line;
    if (mtext) {
        e.ocs = Transform2{};  // MTEXT points are in world coordinates
    } else {
        const std::optional<Transform2> ocs = ocsFor(raw.pairs);
        if (!ocs) {
            warn("text not parallel to the XY plane skipped", raw.line);
            return std::nullopt;
        }
        e.ocs = *ocs;
    }
    e.geom = std::move(t);
    return e;
}

void Parser::decodePolyline(const RawEntity& head, std::vector<Entity>& out) {
    std::vector<RawEntity> vertices;
    if (!readChildren("VERTEX", vertices)) {
        return;
    }
    std::int64_t flags = 0;
    bool widths = false;
    for (const Pair& q : head.pairs) {
        if (q.code == 70) flags = q.integer;
        if ((q.code == 40 || q.code == 41) && q.real != 0.0) widths = true;
    }
    if ((flags & (16 | 64)) != 0) {
        warn("polygon mesh / polyface POLYLINE skipped (3D)", head.line);
        return;
    }
    Entity e;
    e.layer = layerOf(head.pairs);
    e.line = head.line;
    if ((flags & 8) != 0) {
        warn("3D POLYLINE projected onto the XY plane", head.line);
    } else {
        const std::optional<Transform2> ocs = ocsFor(head.pairs);
        if (!ocs) {
            warn("POLYLINE not parallel to the XY plane skipped", head.line);
            return;
        }
        e.ocs = *ocs;
    }
    PolyE poly;
    poly.closed = (flags & 1) != 0;
    for (const RawEntity& v : vertices) {
        Vertex vx;
        std::int64_t vflags = 0;
        for (const Pair& q : v.pairs) {
            switch (q.code) {
                case 10:
                    vx.p.x = q.real;
                    break;
                case 20:
                    vx.p.y = q.real;
                    break;
                case 42:
                    vx.bulge = q.real;
                    break;
                case 70:
                    vflags = q.integer;
                    break;
                case 40:
                case 41:
                    widths = widths || q.real != 0.0;
                    break;
                default:
                    break;
            }
        }
        if ((vflags & 16) != 0) {
            continue;  // spline frame control point, not on the curve
        }
        poly.vertices.push_back(vx);
    }
    if (widths) {
        warn("polyline widths ignored", head.line);
    }
    e.geom = std::move(poly);
    out.push_back(std::move(e));
}

void Parser::decode(const RawEntity& raw, std::vector<Entity>& out, bool modelSpace) {
    const std::string type = detail::upper(raw.type);
    const std::vector<Pair>& pairs = raw.pairs;
    Entity e;
    e.layer = layerOf(pairs);
    e.line = raw.line;
    if (modelSpace) {
        e.tag = carvettoTag(pairs);
    }
    auto withOcs = [&]() {
        const std::optional<Transform2> ocs = ocsFor(pairs);
        if (!ocs) {
            warn(type + " not parallel to the XY plane skipped", raw.line);
            return false;
        }
        e.ocs = *ocs;
        return true;
    };
    if (type == "LINE") {
        LineE g;
        for (const Pair& q : pairs) {
            if (q.code == 10) g.a.x = q.real;
            if (q.code == 20) g.a.y = q.real;
            if (q.code == 11) g.b.x = q.real;
            if (q.code == 21) g.b.y = q.real;
        }
        e.geom = g;
    } else if (type == "ARC" || type == "CIRCLE") {
        if (!withOcs()) {
            return;
        }
        ArcE g;
        g.circle = type == "CIRCLE";
        double a0 = 0.0;
        double a1 = 360.0;
        for (const Pair& q : pairs) {
            if (q.code == 10) g.centre.x = q.real;
            if (q.code == 20) g.centre.y = q.real;
            if (q.code == 40) g.radius = q.real;
            if (q.code == 50) a0 = q.real;
            if (q.code == 51) a1 = q.real;
        }
        if (!(g.radius > 0.0)) {
            warn(type + " with non-positive radius skipped", raw.line);
            return;
        }
        if (!g.circle) {
            if (!(std::fabs(a0 * kDegToRad) <= kMaxAngle) || !(std::fabs(a1 * kDegToRad) <= kMaxAngle)) {
                warn("ARC with start/end angle out of range skipped", raw.line);
                return;
            }
            g.start = a0 * kDegToRad;
            double sweep = geometry::normalizeAngle(a1 * kDegToRad - g.start);
            g.sweep = sweep > 0.0 ? sweep : kTwoPi;
        }
        e.geom = g;
    } else if (type == "ELLIPSE") {
        EllipseE g;
        double nx = 0.0;
        double ny = 0.0;
        double nz = 1.0;
        for (const Pair& q : pairs) {
            switch (q.code) {
                case 10:
                    g.centre.x = q.real;
                    break;
                case 20:
                    g.centre.y = q.real;
                    break;
                case 11:
                    g.major.x = q.real;
                    break;
                case 21:
                    g.major.y = q.real;
                    break;
                case 40:
                    g.ratio = q.real;
                    break;
                case 41:
                    g.start = q.real;
                    break;
                case 42:
                    g.end = q.real;
                    break;
                case 210:
                    nx = q.real;
                    break;
                case 220:
                    ny = q.real;
                    break;
                case 230:
                    nz = q.real;
                    break;
                default:
                    break;
            }
        }
        const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (std::fabs(nx) > 1e-9 * len || std::fabs(ny) > 1e-9 * len) {
            warn("ELLIPSE not parallel to the XY plane skipped", raw.line);
            return;
        }
        if (!(g.ratio > 0.0) || g.major.length() <= 0.0) {
            warn("degenerate ELLIPSE skipped", raw.line);
            return;
        }
        if (!(std::fabs(g.start) <= kMaxAngle) || !(std::fabs(g.end) <= kMaxAngle)) {
            warn("ELLIPSE with start/end parameter out of range skipped", raw.line);
            return;
        }
        g.normalSign = nz < 0.0 ? -1.0 : 1.0;
        e.geom = g;
    } else if (type == "LWPOLYLINE") {
        if (!withOcs()) {
            return;
        }
        PolyE g;
        bool widths = false;
        for (const Pair& q : pairs) {
            switch (q.code) {
                case 70:
                    g.closed = (q.integer & 1) != 0;
                    break;
                case 10:
                    g.vertices.push_back(Vertex{{q.real, 0.0}, 0.0});
                    break;
                case 20:
                    if (!g.vertices.empty()) g.vertices.back().p.y = q.real;
                    break;
                case 42:
                    if (!g.vertices.empty()) g.vertices.back().bulge = q.real;
                    break;
                case 40:
                case 41:
                case 43:
                    widths = widths || q.real != 0.0;
                    break;
                default:
                    break;
            }
        }
        if (widths) {
            warn("polyline widths ignored", raw.line);
        }
        e.geom = std::move(g);
    } else if (type == "SPLINE") {
        SplineE g;
        std::int64_t flags = 0;
        for (const Pair& q : pairs) {
            switch (q.code) {
                case 70:
                    flags = q.integer;
                    break;
                case 71:
                    g.nurbs.degree = static_cast<int>(std::clamp<std::int64_t>(q.integer, -1, 1000));
                    break;
                case 40:
                    g.nurbs.knots.push_back(q.real);
                    break;
                case 41:
                    g.nurbs.weights.push_back(q.real);
                    break;
                case 10:
                    g.nurbs.controls.push_back({q.real, 0.0});
                    break;
                case 20:
                    if (!g.nurbs.controls.empty()) g.nurbs.controls.back().y = q.real;
                    break;
                case 11:
                    g.fit.push_back({q.real, 0.0});
                    break;
                case 21:
                    if (!g.fit.empty()) g.fit.back().y = q.real;
                    break;
                case 12:
                    g.startTangent = Vec2{q.real, g.startTangent ? g.startTangent->y : 0.0};
                    break;
                case 22:
                    if (g.startTangent) g.startTangent->y = q.real;
                    break;
                case 13:
                    g.endTangent = Vec2{q.real, g.endTangent ? g.endTangent->y : 0.0};
                    break;
                case 23:
                    if (g.endTangent) g.endTangent->y = q.real;
                    break;
                default:
                    break;
            }
        }
        g.closedFlag = (flags & 1) != 0;
        e.geom = std::move(g);
    } else if (type == "TEXT" || type == "MTEXT" || type == "ATTRIB") {
        if (std::optional<Entity> t = decodeText(raw)) {
            t->tag.reset();
            out.push_back(std::move(*t));
        }
        return;
    } else if (type == "INSERT") {
        if (!withOcs()) {
            return;
        }
        InsertE g;
        for (const Pair& q : pairs) {
            switch (q.code) {
                case 2:
                    g.block = detail::trim(q.text);
                    break;
                case 10:
                    g.p.x = q.real;
                    break;
                case 20:
                    g.p.y = q.real;
                    break;
                case 41:
                    g.sx = q.real;
                    break;
                case 42:
                    g.sy = q.real;
                    break;
                case 50:
                    g.rotation = q.real * kDegToRad;
                    break;
                case 70:
                    g.cols = static_cast<int>(std::clamp<std::int64_t>(q.integer, 1, 32767));
                    break;
                case 71:
                    g.rows = static_cast<int>(std::clamp<std::int64_t>(q.integer, 1, 32767));
                    break;
                case 44:
                    g.dx = q.real;
                    break;
                case 45:
                    g.dy = q.real;
                    break;
                default:
                    break;
            }
        }
        e.geom = g;
    } else if (type == "VERTEX" || type == "SEQEND" || type == "ATTDEF") {
        return;  // orphan sequence members; attribute definitions are templates, not drawn
    } else {
        warn("unsupported entity " + detail::printable(raw.type, 32) + " skipped", raw.line);
        return;
    }
    out.push_back(std::move(e));
}

// ------------------------------------------------------------------------- emitter

class Emitter {
   public:
    Emitter(const Parser& parsed, const DxfImportOptions& options, Warnings& warnings, DxfDrawing& out,
            bool utf8)
        : p_(parsed), opt_(options), warnings_(warnings), out_(out), utf8_(utf8) {
        for (const LayerRec& rec : p_.layers) {
            const std::string key = detail::upper(rec.name);
            if (layerIndex_.count(key) != 0) {
                continue;
            }
            DxfLayer layer;
            layer.name = detail::decodeDxfString(rec.name, utf8_);
            const int aci = rec.colour < 0 ? -rec.colour : rec.colour;
            layer.aci = (aci >= 1 && aci <= 255) ? aci : 7;
            layer.rgb = rec.rgb ? *rec.rgb : aciToRgb(layer.aci);
            layer.visible = rec.colour >= 0 && (rec.flags & 1) == 0;
            layerIndex_.emplace(key, out_.layers.size());
            out_.layers.push_back(std::move(layer));
        }
    }

    void run(const Transform2& root) {
        emitList(p_.model, root, std::nullopt, 0, true);
        if (fatal_) {
            return;
        }
        rejoinPieces();
        if (opt_.joinContiguous) {
            joinContiguous();
        }
    }

    const std::optional<DxfError>& fatal() const { return fatal_; }

   private:
    std::size_t layerFor(std::string_view name) {
        const std::string key = detail::upper(name);
        const auto it = layerIndex_.find(key);
        if (it != layerIndex_.end()) {
            return it->second;
        }
        DxfLayer layer;
        layer.name = detail::decodeDxfString(name, utf8_);
        layer.aci = 7;
        layer.rgb = aciToRgb(7);
        layerIndex_.emplace(key, out_.layers.size());
        out_.layers.push_back(std::move(layer));
        return out_.layers.size() - 1;
    }

    void setFatal(DxfError::Code code, std::string message, std::size_t line) {
        if (!fatal_) {
            fatal_ = DxfError{code, std::move(message), line};
        }
    }

    std::size_t spanBudget() const { return opt_.maxSpans > spans_ ? opt_.maxSpans - spans_ : 0; }

    void spanOverflow(std::size_t line) {
        setFatal(DxfError::Code::ResourceLimit,
                 "import would create more than " + std::to_string(opt_.maxSpans) + " spans", line);
    }

    /// Appends an arc (local centre/radius/start/signed sweep) mapped through `xf`.
    bool appendArc(Vec2 c, double r, double start, double sweep, const Transform2& xf,
                   std::vector<Span>& spans, std::size_t line) {
        const Vec2 u = xf.applyVector({r, 0.0});
        const Vec2 v = xf.applyVector({0.0, r});
        const Vec2 centre = xf.apply(c);
        if (!inRange(centre) || !inRange(u) || !inRange(v)) {
            // Checked before span generation: a huge radius would need a huge piece count.
            warnings_.add("geometry with coordinates out of range skipped", line);
            return false;
        }
        const std::size_t budget = spanBudget() > spans.size() ? spanBudget() - spans.size() : 0;
        const detail::CurveStatus s =
            detail::appendEllipticalArc(centre, u, v, start, sweep, opt_.tolerance.chordal, budget, spans);
        if (s == detail::CurveStatus::TooManySpans) {
            spanOverflow(line);
            return false;
        }
        if (s == detail::CurveStatus::Invalid) {
            warnings_.add("arc with invalid geometry skipped", line);
            return false;
        }
        return true;
    }

    void addContour(std::size_t layer, Contour&& contour, std::size_t line,
                    const std::optional<PieceTag>& tag, bool topLevel) {
        if (contour.spans.empty() || fatal_) {
            return;
        }
        for (const Span& s : contour.spans) {
            bool ok = true;
            std::visit(
                [&ok](const auto& v) {
                    using T = std::decay_t<decltype(v)>;
                    auto good = [](Vec2 p) {
                        return std::isfinite(p.x) && std::isfinite(p.y) && std::fabs(p.x) <= kMaxCoordinate &&
                               std::fabs(p.y) <= kMaxCoordinate;
                    };
                    if constexpr (std::is_same_v<T, LineSpan>) {
                        ok = good(v.start) && good(v.end);
                    } else if constexpr (std::is_same_v<T, ArcSpan>) {
                        ok = good(v.centre) && std::isfinite(v.radius) && v.radius <= kMaxCoordinate &&
                             std::isfinite(v.startAngle) && std::isfinite(v.endAngle);
                    } else {
                        ok = good(v.p0) && good(v.p1) && good(v.p2) && good(v.p3);
                    }
                },
                s);
            if (!ok) {
                warnings_.add("geometry with coordinates out of range skipped", line);
                return;
            }
        }
        if (contour.spans.size() > spanBudget()) {
            spanOverflow(line);
            return;
        }
        spans_ += contour.spans.size();
        out_.contours.push_back(DxfContour{layer, std::move(contour)});
        tags_.push_back(topLevel ? tag : std::nullopt);
    }

    void emitList(const std::vector<Entity>& list, const Transform2& xf, std::optional<std::size_t> inherited,
                  std::size_t depth, bool topLevel) {
        for (const Entity& e : list) {
            if (fatal_) {
                return;
            }
            const bool byBlock = inherited && e.layer == "0";
            const std::size_t layer = byBlock ? *inherited : layerFor(e.layer);
            const Transform2 m = xf * e.ocs;
            std::visit([&](const auto& g) { emit(g, e, m, layer, depth, topLevel); }, e.geom);
        }
    }

    void emit(const LineE& g, const Entity& e, const Transform2& m, std::size_t layer, std::size_t,
              bool top) {
        const Vec2 a = m.apply(g.a);
        const Vec2 b = m.apply(g.b);
        if (!inRange(a) || !inRange(b)) {
            warnings_.add("geometry with coordinates out of range skipped", e.line);
            return;
        }
        if (a == b) {
            warnings_.add("zero-length LINE skipped", e.line);
            return;
        }
        Contour c;
        c.spans.emplace_back(LineSpan{a, b});
        addContour(layer, std::move(c), e.line, e.tag, top);
    }

    void emit(const ArcE& g, const Entity& e, const Transform2& m, std::size_t layer, std::size_t, bool top) {
        Contour c;
        if (!appendArc(g.centre, g.radius, g.start, g.sweep, m, c.spans, e.line)) {
            return;
        }
        c.closed = g.circle;
        if (g.circle && top && e.tag && e.tag->circleStart && c.spans.size() == 1 &&
            std::holds_alternative<ArcSpan>(c.spans.front())) {
            // Carvetto export: restore the full circle's start angle and direction.
            auto& arc = std::get<ArcSpan>(c.spans.front());
            arc.startAngle = arc.endAngle = *e.tag->circleStart;
            arc.direction = e.tag->circleClockwise ? geometry::ArcDirection::CW : geometry::ArcDirection::CCW;
        }
        addContour(layer, std::move(c), e.line, e.tag, top);
    }

    void emit(const EllipseE& g, const Entity& e, const Transform2& m, std::size_t layer, std::size_t,
              bool top) {
        const Vec2 minor = g.major.perp() * (g.ratio * g.normalSign);
        double sweep = g.end - g.start;
        if (!std::isfinite(sweep)) {
            return;
        }
        if (sweep > kTwoPi) {
            sweep = kTwoPi;
        }
        if (sweep <= 1e-12) {
            sweep = geometry::normalizeAngle(sweep);
            if (sweep <= 1e-12) {
                sweep = kTwoPi;
            }
        }
        // Parameters are bounded by kMaxAngle (parser); reduce the start so that the piece
        // parameters start + k * step keep full precision.
        const double start = std::fabs(g.start) > kTwoPi ? std::fmod(g.start, kTwoPi) : g.start;
        Contour c;
        const std::size_t budget = spanBudget();
        const Vec2 centre = m.apply(g.centre);
        const Vec2 u = m.applyVector(g.major);
        const Vec2 v = m.applyVector(minor);
        if (!inRange(centre) || !inRange(u) || !inRange(v)) {
            warnings_.add("geometry with coordinates out of range skipped", e.line);
            return;
        }
        const detail::CurveStatus s =
            detail::appendEllipticalArc(centre, u, v, start, sweep, opt_.tolerance.chordal, budget, c.spans);
        if (s == detail::CurveStatus::TooManySpans) {
            spanOverflow(e.line);
            return;
        }
        if (s == detail::CurveStatus::Invalid) {
            warnings_.add("ELLIPSE with invalid geometry skipped", e.line);
            return;
        }
        c.closed = sweep >= kTwoPi - 1e-12;
        addContour(layer, std::move(c), e.line, e.tag, top);
    }

    bool appendSegment(const Vertex& a, const Vertex& b, const Transform2& m, std::vector<Span>& spans,
                       std::size_t line) {
        if (a.p == b.p) {
            return true;
        }
        const double bulge = a.bulge;
        if (!(std::fabs(bulge) > 1e-12) || !std::isfinite(bulge)) {
            if (spans.size() >= spanBudget()) {
                spanOverflow(line);
                return false;
            }
            spans.emplace_back(LineSpan{m.apply(a.p), m.apply(b.p)});
            return true;
        }
        // Bulge = tan(sweep / 4), positive counter-clockwise. The centre lies on the chord's
        // perpendicular bisector at signed distance c * (1 - b^2) / (4 b) to the left.
        const Vec2 chord = b.p - a.p;
        const double len = chord.length();
        const Vec2 mid = (a.p + b.p) * 0.5;
        const Vec2 left = chord.perp() / len;
        const Vec2 centre = mid + left * (len * (1.0 - bulge * bulge) / (4.0 * bulge));
        const double sweep = 4.0 * std::atan(bulge);
        const Vec2 radial = a.p - centre;
        return appendArc(centre, radial.length(), radial.angle(), sweep, m, spans, line);
    }

    void emit(const PolyE& g, const Entity& e, const Transform2& m, std::size_t layer, std::size_t,
              bool top) {
        if (g.vertices.size() < 2) {
            return;
        }
        Contour c;
        // First span and span count of every segment (zero-length segments produce none).
        std::vector<std::pair<std::size_t, std::size_t>> segments;
        bool failed = false;  // a segment was skipped (warned): drop the whole polyline, not leave a gap
        auto segment = [&](const Vertex& a, const Vertex& b) {
            const std::size_t before = c.spans.size();
            failed = !appendSegment(a, b, m, c.spans, e.line) || failed;
            segments.emplace_back(before, c.spans.size() - before);
        };
        for (std::size_t i = 0; i + 1 < g.vertices.size() && !fatal_; ++i) {
            segment(g.vertices[i], g.vertices[i + 1]);
        }
        if (g.closed && !fatal_) {
            segment(g.vertices.back(), g.vertices.front());
        }
        if (fatal_ || failed) {
            return;
        }
        if (c.spans.empty()) {
            warnings_.add("zero-length polyline skipped", e.line);
            return;
        }
        if (top && e.tag && !e.tag->splitArcs.empty()) {
            restoreSplitArcs(*e.tag, segments, m, c.spans);
        }
        c.closed = g.closed;
        addContour(layer, std::move(c), e.line, e.tag, top);
    }

    /// True when `arc` traces the same path as the bulge arcs spans[from, to): same turning
    /// direction, same total sweep, and points at five parameters of every piece within a tight
    /// tolerance (1e-9 mm, relaxed by 1e-14 relative for far-out coordinates). The XDATA is only
    /// trusted to restore exact values, never to change the geometry the bulges describe.
    static bool matchesPieces(const ArcSpan& arc, const std::vector<Span>& spans, std::size_t from,
                              std::size_t to) {
        const double total = arc.sweepAngle();
        if (!(arc.radius > 0.0) || !std::isfinite(arc.radius) || !std::isfinite(total) || total == 0.0) {
            return false;
        }
        const double tol = std::max(1e-9, 1e-14 * (arc.centre.length() + arc.radius));
        double sum = 0.0;
        for (std::size_t i = from; i < to; ++i) {
            const auto& piece = std::get<ArcSpan>(spans[i]);
            const double sweep = piece.sweepAngle();
            if ((sweep > 0.0) != (total > 0.0)) {
                return false;  // opposite turning direction
            }
            for (int k = 0; k <= 4; ++k) {
                const double t = k / 4.0;
                const Vec2 expected = arc.pointAt((sum + t * sweep) / total);
                if (!(geometry::distance(piece.pointAt(t), expected) <= tol)) {
                    return false;
                }
            }
            sum += sweep;
        }
        return std::fabs(sum - total) * arc.radius <= tol;
    }

    /// Carvetto export: arcs sweeping more than pi were written as several bulge segments. Put
    /// the original arc back (exact values from the XDATA) when the segments are as expected.
    void restoreSplitArcs(const PieceTag& tag,
                          const std::vector<std::pair<std::size_t, std::size_t>>& segments,
                          const Transform2& m, std::vector<Span>& spans) {
        std::vector<Span> out;
        std::size_t next = 0;  // next span of `spans` not yet copied
        for (const PieceTag::SplitArc& a : tag.splitArcs) {
            if (a.firstSegment + a.segments > segments.size() || segments[a.firstSegment].first < next) {
                return;  // unexpected layout: keep the pieces
            }
            for (std::size_t k = 0; k < a.segments; ++k) {
                const auto& [first, count] = segments[a.firstSegment + k];
                if (count != 1 || !std::holds_alternative<ArcSpan>(spans[first])) {
                    return;
                }
            }
            const std::size_t from = segments[a.firstSegment].first;
            const std::size_t to = from + a.segments;  // one span per segment, consecutive
            std::vector<Span> restored{Span{a.arc}};
            if (!(m == Transform2{})) {
                restored = geometry::transformed(Span{a.arc}, m);
            }
            if (restored.size() != 1 || !std::holds_alternative<ArcSpan>(restored[0]) ||
                !matchesPieces(std::get<ArcSpan>(restored[0]), spans, from, to)) {
                return;
            }
            out.insert(out.end(), spans.begin() + static_cast<std::ptrdiff_t>(next),
                       spans.begin() + static_cast<std::ptrdiff_t>(from));
            out.push_back(restored[0]);
            next = to;
        }
        out.insert(out.end(), spans.begin() + static_cast<std::ptrdiff_t>(next), spans.end());
        spans = std::move(out);
    }

    void emit(const SplineE& g, const Entity& e, const Transform2& m, std::size_t layer, std::size_t,
              bool top) {
        Contour c;
        detail::CurveStatus s = detail::CurveStatus::Ok;
        std::string why;
        if (!g.nurbs.controls.empty()) {
            // NURBS are affine invariant: transform the control points, keep the weights.
            detail::NurbsCurve world = g.nurbs;
            for (Vec2& p : world.controls) {
                p = m.apply(p);
                // The curve lies in the control hull: out-of-range controls would only produce
                // geometry that is discarded, after a costly approximation.
                if (!inRange(p)) {
                    warnings_.add("geometry with coordinates out of range skipped", e.line);
                    return;
                }
            }
            s = detail::nurbsToSpans(world, opt_.tolerance.chordal, spanBudget(), c.spans, why);
        } else if (g.fit.size() >= 2) {
            warnings_.add(
                "SPLINE defined only by fit points interpolated (C2 cubic, chord-length parameters)", e.line);
            for (const Vec2& p : g.fit) {
                if (!inRange(m.apply(p))) {
                    warnings_.add("geometry with coordinates out of range skipped", e.line);
                    return;
                }
            }
            std::vector<Span> local;
            s = detail::interpolateFitPoints(g.fit, g.startTangent, g.endTangent, spanBudget(), local);
            for (const Span& sp : local) {
                const auto& b = std::get<BezierSpan>(sp);
                c.spans.emplace_back(b.transformed(m));
            }
            why = "fewer than two distinct fit points";
        } else {
            s = detail::CurveStatus::Invalid;
            why = "no control or fit points";
        }
        if (s == detail::CurveStatus::TooManySpans) {
            spanOverflow(e.line);
            return;
        }
        if (s == detail::CurveStatus::Invalid) {
            warnings_.add("invalid SPLINE skipped: " + why, e.line);
            return;
        }
        if (c.spans.empty()) {
            warnings_.add("zero-length SPLINE skipped", e.line);
            return;
        }
        if (!c.spans.empty() && g.closedFlag &&
            geometry::distance(geometry::startPoint(c.spans.front()), geometry::endPoint(c.spans.back())) <=
                opt_.tolerance.distance) {
            c.closed = true;
        }
        addContour(layer, std::move(c), e.line, e.tag, top);
    }

    void emit(const TextE& g, const Entity& e, const Transform2& m, std::size_t layer, std::size_t, bool) {
        if (out_.texts.size() >= opt_.maxTexts) {
            setFatal(DxfError::Code::ResourceLimit,
                     "import would create more than " + std::to_string(opt_.maxTexts) + " text items",
                     e.line);
            return;
        }
        DxfText t;
        t.layer = layer;
        t.kind = g.kind;
        std::string decoded = detail::decodeDxfString(g.raw, utf8_);
        if (g.kind == DxfTextKind::MText) {
            decoded = stripMTextFormatting(decoded);
        }
        t.text = detail::decodeControlCodes(decoded);
        const std::string_view style = g.style.empty() ? std::string_view{"Standard"} : g.style;
        t.style = detail::decodeDxfString(style, utf8_);
        const auto font = p_.styleFonts.find(detail::upper(style));
        if (font != p_.styleFonts.end()) {
            t.font = detail::decodeDxfString(font->second, utf8_);
        }
        // Map the baseline direction and the character box through the transform.
        const Vec2 dir = m.applyVector(Vec2::fromAngle(g.rotation));
        const double dirLen = dir.length();
        const double det = m.determinant();
        if (!(dirLen > 0.0) || !std::isfinite(dirLen) || det == 0.0 || !std::isfinite(det)) {
            warnings_.add("text with degenerate transform skipped", e.line);
            return;
        }
        t.insertion = m.apply(g.p);
        if (g.p2) {
            t.alignmentPoint = m.apply(*g.p2);
        }
        t.rotation = dir.angle();
        t.height = g.height * std::fabs(det) / dirLen;
        t.widthFactor = g.widthFactor * dirLen * dirLen / std::fabs(det);
        t.referenceWidth = g.refWidth * dirLen;
        t.mirrored = g.mirrored != (det < 0.0);
        t.hAlign = g.h;
        t.vAlign = g.v;
        const bool finite = std::isfinite(t.insertion.x) && std::isfinite(t.insertion.y) &&
                            std::fabs(t.insertion.x) <= kMaxCoordinate &&
                            std::fabs(t.insertion.y) <= kMaxCoordinate && std::isfinite(t.height) &&
                            std::isfinite(t.widthFactor);
        if (!finite) {
            warnings_.add("text with coordinates out of range skipped", e.line);
            return;
        }
        out_.texts.push_back(std::move(t));
    }

    void emit(const InsertE& g, const Entity& e, const Transform2& m, std::size_t layer, std::size_t depth,
              bool) {
        const std::string key = detail::upper(g.block);
        const auto it = p_.blocks.find(key);
        if (it == p_.blocks.end()) {
            warnings_.add("INSERT of undefined block '" + detail::printable(g.block) + "' skipped", e.line);
            return;
        }
        if (std::find(stack_.begin(), stack_.end(), key) != stack_.end()) {
            warnings_.add(
                "block '" + detail::printable(g.block) + "' inserts itself; recursive INSERT skipped",
                e.line);
            return;
        }
        if (depth + 1 > opt_.maxBlockDepth) {
            warnings_.add(
                "blocks nested deeper than " + std::to_string(opt_.maxBlockDepth) + " levels skipped",
                e.line);
            return;
        }
        const Block& block = it->second;
        if (block.external) {
            warnings_.add("external reference '" + detail::printable(g.block) + "' not loaded", e.line);
            return;
        }
        const auto cells = static_cast<std::size_t>(g.cols) * static_cast<std::size_t>(g.rows);
        if (cells > opt_.maxInstances || instances_ > opt_.maxInstances - cells) {
            setFatal(
                DxfError::Code::ResourceLimit,
                "import would expand more than " + std::to_string(opt_.maxInstances) + " block instances",
                e.line);
            return;
        }
        instances_ += cells;
        // Insert transform: translate(P) * rotate * [array offset] * scale * translate(-base).
        const Transform2 scale = Transform2::scaling(g.sx, g.sy) * Transform2::translation(-block.base);
        const Transform2 place = m * Transform2::translation(g.p) * Transform2::rotation(g.rotation);
        stack_.push_back(key);
        for (int r = 0; r < g.rows && !fatal_; ++r) {
            for (int col = 0; col < g.cols && !fatal_; ++col) {
                const Vec2 offset{static_cast<double>(col) * g.dx, static_cast<double>(r) * g.dy};
                const Transform2 xf = place * Transform2::translation(offset) * scale;
                const double det = xf.determinant();
                if (!std::isfinite(det) || std::fabs(det) < 1e-300 || !std::isfinite(xf.tx) ||
                    !std::isfinite(xf.ty)) {
                    warnings_.add("INSERT with zero or invalid scale skipped", e.line);
                    continue;
                }
                emitList(block.entities, xf, layer, depth + 1, false);
            }
        }
        stack_.pop_back();
    }

    /// Joins consecutive pieces that Carvetto's exporter split from one contour (XDATA tags).
    void rejoinPieces() {
        std::vector<DxfContour> result;
        result.reserve(out_.contours.size());
        const double tol = opt_.tolerance.distance;
        std::size_t i = 0;
        while (i < out_.contours.size()) {
            const std::optional<PieceTag>& tag = tags_[i];
            if (!tag || tag->piece != 0) {
                result.push_back(std::move(out_.contours[i]));
                ++i;
                continue;
            }
            const auto count = static_cast<std::size_t>(tag->count);
            bool ok = count <= out_.contours.size() - i;
            for (std::size_t k = 1; ok && k < count; ++k) {
                const std::optional<PieceTag>& t = tags_[i + k];
                ok = t && t->contour == tag->contour && t->piece == static_cast<std::int64_t>(k) &&
                     t->count == tag->count && out_.contours[i + k].layer == out_.contours[i].layer &&
                     geometry::distance(out_.contours[i + k - 1].contour.endPoint(),
                                        out_.contours[i + k].contour.startPoint()) <= tol;
            }
            if (!ok) {
                result.push_back(std::move(out_.contours[i]));
                ++i;
                continue;
            }
            DxfContour joined = std::move(out_.contours[i]);
            for (std::size_t k = 1; k < count; ++k) {
                for (Span& s : out_.contours[i + k].contour.spans) {
                    joined.contour.spans.push_back(std::move(s));
                }
            }
            joined.contour.closed = tag->closed && geometry::distance(joined.contour.startPoint(),
                                                                      joined.contour.endPoint()) <= tol;
            result.push_back(std::move(joined));
            i += count;
        }
        out_.contours = std::move(result);
    }

    /// Optional: joins consecutive open contours on one layer whose ends meet.
    void joinContiguous() {
        std::vector<DxfContour> result;
        const double tol = opt_.tolerance.distance;
        for (DxfContour& c : out_.contours) {
            if (!result.empty()) {
                DxfContour& last = result.back();
                if (!last.contour.closed && !c.contour.closed && last.layer == c.layer) {
                    bool reverse = false;
                    bool join = geometry::distance(last.contour.endPoint(), c.contour.startPoint()) <= tol;
                    if (!join && geometry::distance(last.contour.endPoint(), c.contour.endPoint()) <= tol) {
                        join = true;
                        reverse = true;
                    }
                    if (join) {
                        if (reverse) {
                            c.contour.reverse();
                        }
                        for (Span& s : c.contour.spans) {
                            last.contour.spans.push_back(std::move(s));
                        }
                        if (last.contour.spans.size() > 1 &&
                            geometry::distance(last.contour.startPoint(), last.contour.endPoint()) <= tol) {
                            last.contour.closed = true;
                        }
                        continue;
                    }
                }
            }
            result.push_back(std::move(c));
        }
        out_.contours = std::move(result);
    }

    const Parser& p_;
    const DxfImportOptions& opt_;
    Warnings& warnings_;
    DxfDrawing& out_;
    bool utf8_;
    std::map<std::string, std::size_t> layerIndex_;
    std::vector<std::optional<PieceTag>> tags_;
    std::vector<std::string> stack_;
    std::size_t spans_{0};
    std::size_t instances_{0};
    std::optional<DxfError> fatal_;
};

}  // namespace

DxfResult<DxfDrawing> importDxf(std::span<const std::uint8_t> bytes, const DxfImportOptions& options) {
    if (!(options.tolerance.chordal > 0.0) || !(options.tolerance.distance >= 0.0)) {
        return DxfError{DxfError::Code::InvalidArgument, "tolerances must be positive", 0};
    }
    Warnings warnings;
    Parser parser(bytes, warnings);
    if (!parser.parse()) {
        return *parser.error();
    }
    DxfDrawing drawing;
    drawing.binary = parser.binary();
    drawing.version = std::string(parser.version);
    drawing.insunits =
        parser.insunits ? static_cast<int>(std::clamp<std::int64_t>(*parser.insunits, -1, 1000)) : 0;
    const bool utf8 = parser.version.empty() || parser.version >= std::string_view{"AC1021"};
    if (!utf8 && !parser.codepage.empty() && !detail::iequals(parser.codepage, "ANSI_1252")) {
        warnings.add("code page " + detail::printable(parser.codepage) + " decoded as Windows-1252", 0);
    }

    // Units: forced > $INSUNITS > $MEASUREMENT > option default.
    if (options.forcedScale && std::isfinite(*options.forcedScale) && *options.forcedScale > 0.0) {
        drawing.unitScale = *options.forcedScale;
        drawing.unitSource = DxfUnitSource::Forced;
    } else if (const std::optional<double> s = insunitsToMillimetres(drawing.insunits)) {
        drawing.unitScale = *s;
        drawing.unitSource = DxfUnitSource::Insunits;
    } else {
        if (drawing.insunits != 0) {
            warnings.add("unsupported $INSUNITS " + std::to_string(drawing.insunits) + " treated as unitless",
                         0);
        }
        if (parser.measurement && (*parser.measurement == 0 || *parser.measurement == 1)) {
            drawing.unitScale = *parser.measurement == 0 ? 25.4 : 1.0;
            drawing.unitSource = DxfUnitSource::Measurement;
        } else {
            drawing.unitScale = std::isfinite(options.unitlessScale) && options.unitlessScale > 0.0
                                    ? options.unitlessScale
                                    : 1.0;
            drawing.unitSource = DxfUnitSource::Default;
        }
    }

    Emitter emitter(parser, options, warnings, drawing, utf8);
    emitter.run(Transform2::scaling(drawing.unitScale));
    if (emitter.fatal()) {
        return *emitter.fatal();
    }
    drawing.warnings = warnings.take();
    return drawing;
}

}  // namespace carvetto::io
