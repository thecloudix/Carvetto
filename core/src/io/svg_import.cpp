#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/io/svg.hpp"
#include "svg_path.hpp"
#include "svg_xml.hpp"

// SVG import (ADR-0026): bounded XML subset -> element walk with an explicit stack (no
// recursion, so nesting and <use> chains cannot exhaust the call stack) -> PathBuilder.
namespace carvetto::io {

std::string SvgError::describe() const {
    return line > 0 ? "line " + std::to_string(line) + ": " + message : message;
}

std::string SvgWarning::describe() const {
    std::string s = line > 0 ? "line " + std::to_string(line) + ": " + message : message;
    if (count > 1) {
        s += " (x" + std::to_string(count) + ")";
    }
    return s;
}

namespace {

using detail::kNoNode;
using detail::PathBuilder;
using detail::XmlDoc;
using detail::XmlNode;
using geometry::Transform2;
using geometry::Vec2;

constexpr double kMaxLength = 1.0e12;

bool isWs(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f';
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && isWs(s.front())) {
        s.remove_prefix(1);
    }
    while (!s.empty() && isWs(s.back())) {
        s.remove_suffix(1);
    }
    return s;
}

std::string lower(std::string_view s) {
    std::string out(s);
    for (char& c : out) {
        if (c >= 'A' && c <= 'Z') {
            c = static_cast<char>(c - 'A' + 'a');
        }
    }
    return out;
}

/// Value of a presentation property: the inline `style` wins over the attribute.
std::optional<std::string> presentation(const XmlNode& n, std::string_view name) {
    if (const std::string* style = n.attr("style")) {
        std::string_view rest = *style;
        std::optional<std::string> found;
        while (!rest.empty()) {
            const std::size_t semi = rest.find(';');
            const std::string_view decl = rest.substr(0, semi);
            rest = semi == std::string_view::npos ? std::string_view{} : rest.substr(semi + 1);
            const std::size_t colon = decl.find(':');
            if (colon == std::string_view::npos) {
                continue;
            }
            if (lower(trim(decl.substr(0, colon))) == name) {
                std::string v = lower(trim(decl.substr(colon + 1)));
                const std::size_t imp = v.find('!');
                if (imp != std::string::npos) {
                    v = std::string(trim(std::string_view(v).substr(0, imp)));
                }
                found = v;
            }
        }
        if (found) {
            return found;
        }
    }
    if (const std::string* a = n.attr(name)) {
        return lower(trim(*a));
    }
    return std::nullopt;
}

enum class Unit { None, Px, Mm, Cm, In, Pt, Pc, Percent };

struct Length {
    double value{0.0};
    Unit unit{Unit::None};
};

std::optional<Length> parseLength(std::string_view s) {
    s = trim(s);
    std::size_t pos = 0;
    double v = 0.0;
    if (!detail::scanSvgNumber(s, pos, v)) {
        return std::nullopt;
    }
    const std::string_view u = s.substr(pos);
    Length l;
    l.value = v;
    if (u.empty()) {
        l.unit = Unit::None;
    } else if (u == "px") {
        l.unit = Unit::Px;
    } else if (u == "mm") {
        l.unit = Unit::Mm;
    } else if (u == "cm") {
        l.unit = Unit::Cm;
    } else if (u == "in") {
        l.unit = Unit::In;
    } else if (u == "pt") {
        l.unit = Unit::Pt;
    } else if (u == "pc") {
        l.unit = Unit::Pc;
    } else if (u == "%") {
        l.unit = Unit::Percent;
    } else {
        return std::nullopt;
    }
    return l;
}

/// Millimetres per unit for absolute units (px follows `dpi`).
double unitMm(Unit u, double dpi) {
    switch (u) {
        case Unit::Mm: return 1.0;
        case Unit::Cm: return 10.0;
        case Unit::In: return 25.4;
        case Unit::Pt: return 25.4 / 72.0;
        case Unit::Pc: return 25.4 / 6.0;
        default: return 25.4 / dpi;
    }
}

/// Reads `count` numbers separated by whitespace and/or commas, all of `s`.
bool parseNumberList(std::string_view s, std::vector<double>& out, std::size_t maxCount) {
    std::size_t pos = 0;
    for (;;) {
        while (pos < s.size() && (isWs(s[pos]) || s[pos] == ',')) {
            ++pos;
        }
        if (pos >= s.size()) {
            return true;
        }
        double v = 0.0;
        if (out.size() >= maxCount || !detail::scanSvgNumber(s, pos, v)) {
            return false;
        }
        out.push_back(v);
    }
}

bool sane(const Transform2& t) {
    for (const double v : {t.a, t.b, t.c, t.d, t.tx, t.ty}) {
        if (!std::isfinite(v) || std::fabs(v) > kMaxLength) {
            return false;
        }
    }
    return true;
}

/// Parses an SVG transform list; nullopt when malformed.
std::optional<Transform2> parseTransform(std::string_view s) {
    Transform2 result;
    std::size_t pos = 0;
    for (;;) {
        while (pos < s.size() && (isWs(s[pos]) || s[pos] == ',')) {
            ++pos;
        }
        if (pos >= s.size()) {
            return result;
        }
        const std::size_t nameStart = pos;
        while (pos < s.size() && ((s[pos] >= 'a' && s[pos] <= 'z') || (s[pos] >= 'A' && s[pos] <= 'Z'))) {
            ++pos;
        }
        const std::string_view name = s.substr(nameStart, pos - nameStart);
        while (pos < s.size() && isWs(s[pos])) {
            ++pos;
        }
        if (name.empty() || pos >= s.size() || s[pos] != '(') {
            return std::nullopt;
        }
        const std::size_t close = s.find(')', pos);
        if (close == std::string_view::npos) {
            return std::nullopt;
        }
        std::vector<double> a;
        if (!parseNumberList(s.substr(pos + 1, close - pos - 1), a, 6)) {
            return std::nullopt;
        }
        pos = close + 1;
        Transform2 t;
        constexpr double kDeg = geometry::kPi / 180.0;
        if (name == "matrix" && a.size() == 6) {
            t = Transform2(a[0], a[2], a[1], a[3], a[4], a[5]);
        } else if (name == "translate" && (a.size() == 1 || a.size() == 2)) {
            t = Transform2::translation(Vec2{a[0], a.size() == 2 ? a[1] : 0.0});
        } else if (name == "scale" && (a.size() == 1 || a.size() == 2)) {
            t = Transform2::scaling(a[0], a.size() == 2 ? a[1] : a[0]);
        } else if (name == "rotate" && (a.size() == 1 || a.size() == 3)) {
            const double c = std::cos(a[0] * kDeg);
            const double sn = std::sin(a[0] * kDeg);
            t = Transform2(c, -sn, sn, c, 0.0, 0.0);
            if (a.size() == 3) {
                t = Transform2::translation(Vec2{a[1], a[2]}) * t * Transform2::translation(Vec2{-a[1], -a[2]});
            }
        } else if (name == "skewX" && a.size() == 1) {
            t = Transform2(1.0, std::tan(a[0] * kDeg), 0.0, 1.0, 0.0, 0.0);
        } else if (name == "skewY" && a.size() == 1) {
            t = Transform2(1.0, 0.0, std::tan(a[0] * kDeg), 1.0, 0.0, 0.0);
        } else {
            return std::nullopt;
        }
        result = result * t;
        if (!sane(result)) {
            return std::nullopt;
        }
    }
}

std::string_view localName(const std::string& name, bool& foreign) {
    foreign = false;
    const std::size_t colon = name.find(':');
    if (colon == std::string::npos) {
        return name;
    }
    if (std::string_view(name).substr(0, colon) == "svg") {
        return std::string_view(name).substr(colon + 1);
    }
    foreign = true;
    return name;
}

bool isSkippedType(std::string_view n) {
    static constexpr std::array<std::string_view, 17> kSkipped = {
        "text",       "tspan",   "textPath",      "image",  "linearGradient", "radialGradient",
        "meshgradient", "pattern", "clipPath",    "mask",   "filter",         "marker",
        "foreignObject", "switch", "style",       "script", "animate"};
    return std::find(kSkipped.begin(), kSkipped.end(), n) != kSkipped.end();
}

struct Ctx {
    Transform2 ctm;
    std::size_t layer{kNoNode};  // kNoNode: the default layer, created on first use
    bool visible{true};
    std::size_t useDepth{0};
};

struct Frame {
    std::size_t node{kNoNode};  // for diagnostics and the on-stack marker
    std::size_t next{kNoNode};  // next child to visit
    Ctx ctx;
};

class Importer {
   public:
    Importer(const XmlDoc& doc, const SvgImportOptions& o) : doc_(doc), opt_(o), onStack_(doc.nodes.size(), 0) {}

    SvgResult<SvgDrawing> run() {
        const XmlNode& root = doc_.nodes[0];
        bool foreign = false;
        if (localName(root.name, foreign) != "svg" || foreign) {
            return SvgError{SvgError::Code::InvalidFormat, "the root element is <" + root.name + ">, not <svg>",
                            root.line};
        }
        for (std::size_t i = 0; i < doc_.nodes.size(); ++i) {
            if (const std::string* id = doc_.nodes[i].attr("id")) {
                ids_.emplace(*id, i);  // first occurrence wins
            }
        }
        Ctx ctx;
        ctx.ctm = rootTransform(root);
        if (isHidden(root)) {
            return std::move(out_);
        }
        std::vector<Frame> stack;
        push(stack, 0, doc_.nodes[0].firstChild, ctx);
        while (!stack.empty()) {
            if (error_) {
                return *error_;
            }
            Frame& f = stack.back();
            const std::size_t child = f.next;
            if (child == kNoNode) {
                onStack_[f.node] = 0;
                stack.pop_back();
                continue;
            }
            f.next = doc_.nodes[child].nextSibling;
            const Ctx parentCtx = f.ctx;
            enter(stack, child, parentCtx, false);
        }
        if (error_) {
            return *error_;
        }
        return std::move(out_);
    }

   private:
    void push(std::vector<Frame>& stack, std::size_t node, std::size_t first, const Ctx& ctx) {
        onStack_[node] = 1;
        Frame f;
        f.node = node;
        f.next = first;
        f.ctx = ctx;
        stack.push_back(f);
    }

    void warn(std::size_t line, const std::string& message) {
        const auto it = warnIndex_.find(message);
        if (it != warnIndex_.end()) {
            ++out_.warnings[it->second].count;
            return;
        }
        warnIndex_.emplace(message, out_.warnings.size());
        SvgWarning w;
        w.line = line;
        w.message = message;
        out_.warnings.push_back(std::move(w));
    }

    void fail(SvgError::Code code, const std::string& message, std::size_t line) {
        if (!error_) {
            error_ = SvgError{code, message, line};
        }
    }

    bool isHidden(const XmlNode& n) {
        const auto d = presentation(n, "display");
        return d && *d == "none";
    }

    Transform2 rootTransform(const XmlNode& root) {
        const double pxMm = 25.4 / opt_.dpi;
        double w = 0.0;
        double h = 0.0;
        bool haveW = false;
        bool haveH = false;
        auto size = [&](const char* name, double& dst, bool& have) {
            const std::string* a = root.attr(name);
            if (a == nullptr) {
                return;
            }
            const auto l = parseLength(*a);
            if (!l || l->unit == Unit::Percent) {
                if (!l) {
                    warn(root.line, std::string("invalid ") + name + " on <svg> ignored");
                }
                return;
            }
            const double mm = l->value * unitMm(l->unit, opt_.dpi);
            if (!(mm > 0.0) || mm > kMaxLength) {
                warn(root.line, std::string("invalid ") + name + " on <svg> ignored");
                return;
            }
            dst = mm;
            have = true;
        };
        size("width", w, haveW);
        size("height", h, haveH);
        std::array<double, 4> vb{};
        bool haveVb = false;
        if (const std::string* a = root.attr("viewBox")) {
            std::vector<double> n;
            if (parseNumberList(*a, n, 4) && n.size() == 4 && n[2] > 0.0 && n[3] > 0.0 &&
                std::isfinite(n[2] * n[3])) {
                vb = {n[0], n[1], n[2], n[3]};
                haveVb = true;
            } else {
                warn(root.line, "invalid viewBox on <svg> ignored");
            }
        }
        Transform2 down;
        if (haveVb) {
            if (!haveW && !haveH) {
                w = vb[2] * pxMm;
                h = vb[3] * pxMm;
            } else if (!haveH) {
                h = w * vb[3] / vb[2];
            } else if (!haveW) {
                w = h * vb[2] / vb[3];
            }
            double sx = w / vb[2];
            double sy = h / vb[3];
            double ax = 0.5;
            double ay = 0.5;
            if (const std::string* par = root.attr("preserveAspectRatio")) {
                const std::string p = lower(trim(*par));
                if (p.find("none") != std::string::npos) {
                    ax = -1.0;
                } else {
                    ax = p.find("xmin") != std::string::npos ? 0.0 : (p.find("xmax") != std::string::npos ? 1.0 : 0.5);
                    ay = p.find("ymin") != std::string::npos ? 0.0 : (p.find("ymax") != std::string::npos ? 1.0 : 0.5);
                    const double s = p.find("slice") != std::string::npos ? std::max(sx, sy) : std::min(sx, sy);
                    sx = s;
                    sy = s;
                }
            } else {
                const double s = std::min(sx, sy);
                sx = s;
                sy = s;
            }
            double tx = -vb[0] * sx;
            double ty = -vb[1] * sy;
            if (ax >= 0.0) {
                tx += (w - vb[2] * sx) * ax;
                ty += (h - vb[3] * sy) * ay;
            }
            down = Transform2(sx, 0.0, 0.0, sy, tx, ty);
        } else {
            down = Transform2::scaling(pxMm);
            if (!haveW && !haveH) {
                warn(root.line, "the <svg> element has neither a size nor a viewBox; the page size is unknown");
            }
        }
        out_.widthMm = w;
        out_.heightMm = h;
        Transform2 result = Transform2(1.0, 0.0, 0.0, -1.0, 0.0, h) * down;
        if (const std::string* o = root.attr("data-origin-mm")) {
            std::vector<double> n;
            if (parseNumberList(*o, n, 2) && n.size() == 2 && std::fabs(n[0]) <= kMaxLength &&
                std::fabs(n[1]) <= kMaxLength) {
                result = Transform2::translation(Vec2{n[0], n[1]}) * result;
            } else {
                warn(root.line, "invalid data-origin-mm on <svg> ignored");
            }
        }
        viewW_ = haveVb ? vb[2] : (haveW ? w / pxMm : 100.0);
        viewH_ = haveVb ? vb[3] : (haveH ? h / pxMm : 100.0);
        return result;
    }

    /// A length attribute in user units; nullopt when absent (`present` false) or invalid.
    std::optional<double> lengthAttr(const XmlNode& n, const char* name, char axis, bool& present, bool& valid) {
        present = false;
        const std::string* a = n.attr(name);
        if (a == nullptr) {
            return std::nullopt;
        }
        present = true;
        const auto l = parseLength(*a);
        if (!l) {
            valid = false;
            return std::nullopt;
        }
        double v = l->value;
        if (l->unit == Unit::Percent) {
            const double basis = axis == 'x' ? viewW_ : (axis == 'y' ? viewH_ : std::sqrt((viewW_ * viewW_ + viewH_ * viewH_) / 2.0));
            v = v * basis / 100.0;
        } else if (l->unit != Unit::None && l->unit != Unit::Px) {
            v = v * unitMm(l->unit, opt_.dpi) / (25.4 / opt_.dpi);
        }
        if (!std::isfinite(v) || std::fabs(v) > kMaxLength) {
            valid = false;
            return std::nullopt;
        }
        return v;
    }

    double num(const XmlNode& n, const char* name, char axis, bool& valid, double fallback = 0.0) {
        bool present = false;
        const auto v = lengthAttr(n, name, axis, present, valid);
        return v ? *v : fallback;
    }

    std::size_t layerIndex(const std::string& name) {
        const auto it = layerByName_.find(name);
        if (it != layerByName_.end()) {
            return it->second;
        }
        if (out_.layers.size() >= opt_.maxLayers) {
            fail(SvgError::Code::ResourceLimit, "too many layers", 0);
            return 0;
        }
        out_.layers.push_back(SvgLayer{name});
        layerByName_.emplace(name, out_.layers.size() - 1);
        return out_.layers.size() - 1;
    }

    void emit(PathBuilder& b, const XmlNode& n, const Ctx& ctx) {
        if (b.overflowed()) {
            fail(SvgError::Code::ResourceLimit, "the drawing exceeds the span limit", n.line);
            return;
        }
        if (b.droppedOutOfRange() > 0) {
            warn(n.line, "geometry with out-of-range coordinates skipped");
        }
        if (b.arcFallbacks() > 0) {
            warn(n.line, "arcs with unusable radii replaced by lines");
        }
        spans_ += b.spansUsed();
        if (b.contours().empty()) {
            return;
        }
        const std::size_t layer = ctx.layer != kNoNode ? ctx.layer : layerIndex("0");
        for (geometry::Contour& c : b.contours()) {
            out_.contours.push_back(SvgContour{layer, std::move(c)});
        }
    }

    void shape(std::string_view name, const XmlNode& n, const Ctx& ctx) {
        const std::size_t budget = opt_.maxSpans > spans_ ? opt_.maxSpans - spans_ : 0;
        PathBuilder b(ctx.ctm, opt_.tolerance.chordal, opt_.tolerance.distance, budget);
        bool valid = true;
        if (name == "path") {
            const std::string* d = n.attr("d");
            if (d == nullptr) {
                return;
            }
            std::string err;
            detail::parsePathData(*d, b, err);
            if (!err.empty()) {
                warn(n.line, "path data error (" + err + "); the path is kept up to the error");
            }
        } else if (name == "rect") {
            rect(n, b, valid);
        } else if (name == "circle") {
            const double cx = num(n, "cx", 'x', valid);
            const double cy = num(n, "cy", 'y', valid);
            const double r = num(n, "r", 'r', valid);
            if (!valid || r < 0.0) {
                warn(n.line, "invalid <circle> skipped");
                return;
            }
            if (r > 0.0) {
                b.ellipse(Vec2{cx, cy}, r, r);
            }
        } else if (name == "ellipse") {
            const double cx = num(n, "cx", 'x', valid);
            const double cy = num(n, "cy", 'y', valid);
            const double rx = num(n, "rx", 'x', valid);
            const double ry = num(n, "ry", 'y', valid);
            if (!valid || rx < 0.0 || ry < 0.0) {
                warn(n.line, "invalid <ellipse> skipped");
                return;
            }
            if (rx > 0.0 && ry > 0.0) {
                b.ellipse(Vec2{cx, cy}, rx, ry);
            } else if ((n.attr("rx") != nullptr) != (n.attr("ry") != nullptr)) {
                warn(n.line, "<ellipse> with only one of rx and ry skipped");
                return;
            }
        } else if (name == "line") {
            const double x1 = num(n, "x1", 'x', valid);
            const double y1 = num(n, "y1", 'y', valid);
            const double x2 = num(n, "x2", 'x', valid);
            const double y2 = num(n, "y2", 'y', valid);
            if (!valid) {
                warn(n.line, "invalid <line> skipped");
                return;
            }
            b.moveTo(Vec2{x1, y1});
            b.lineTo(Vec2{x2, y2});
            b.finish();
        } else {  // polyline, polygon
            const std::string* pts = n.attr("points");
            if (pts == nullptr) {
                return;
            }
            std::vector<double> v;
            const bool whole = parseNumberList(*pts, v, pts->size());
            if (!whole || v.size() % 2 != 0) {
                warn(n.line, "points list has an error or an odd number of values; the rest is ignored");
            }
            const std::size_t count = v.size() / 2;
            if (count >= 2) {
                b.moveTo(Vec2{v[0], v[1]});
                for (std::size_t i = 1; i < count; ++i) {
                    b.lineTo(Vec2{v[2 * i], v[2 * i + 1]});
                }
                if (name == "polygon") {
                    b.close();
                } else {
                    b.finish();
                }
            }
        }
        emit(b, n, ctx);
    }

    void rect(const XmlNode& n, PathBuilder& b, bool& valid) {
        const double x = num(n, "x", 'x', valid);
        const double y = num(n, "y", 'y', valid);
        const double w = num(n, "width", 'x', valid);
        const double h = num(n, "height", 'y', valid);
        bool hasRx = false;
        bool hasRy = false;
        bool v1 = true;
        bool v2 = true;
        std::optional<double> rxv = lengthAttr(n, "rx", 'x', hasRx, v1);
        std::optional<double> ryv = lengthAttr(n, "ry", 'y', hasRy, v2);
        if (rxv && *rxv < 0.0) {
            rxv.reset();
        }
        if (ryv && *ryv < 0.0) {
            ryv.reset();
        }
        if (!valid || w < 0.0 || h < 0.0) {
            warn(n.line, "invalid <rect> skipped");
            valid = false;
            return;
        }
        if (!(w > 0.0) || !(h > 0.0)) {
            return;
        }
        if (!rxv && ryv) {
            rxv = ryv;
        } else if (rxv && !ryv) {
            ryv = rxv;
        }
        double rx = rxv ? *rxv : 0.0;
        double ry = ryv ? *ryv : 0.0;
        rx = std::min(rx, w / 2.0);
        ry = std::min(ry, h / 2.0);
        if (rx <= 0.0 || ry <= 0.0) {
            b.moveTo(Vec2{x, y});
            b.lineTo(Vec2{x + w, y});
            b.lineTo(Vec2{x + w, y + h});
            b.lineTo(Vec2{x, y + h});
            b.close();
            return;
        }
        b.moveTo(Vec2{x + rx, y});
        b.lineTo(Vec2{x + w - rx, y});
        b.arcTo(rx, ry, 0.0, false, true, Vec2{x + w, y + ry});
        b.lineTo(Vec2{x + w, y + h - ry});
        b.arcTo(rx, ry, 0.0, false, true, Vec2{x + w - rx, y + h});
        b.lineTo(Vec2{x + rx, y + h});
        b.arcTo(rx, ry, 0.0, false, true, Vec2{x, y + h - ry});
        b.lineTo(Vec2{x, y + ry});
        b.arcTo(rx, ry, 0.0, false, true, Vec2{x + rx, y});
        b.close();
    }

    void enter(std::vector<Frame>& stack, std::size_t idx, const Ctx& parent, bool viaUse) {
        const XmlNode& n = doc_.nodes[idx];
        if (++visits_ > opt_.maxVisits) {
            fail(SvgError::Code::ResourceLimit, "too many element visits (is a <use> expansion exploding?)", n.line);
            return;
        }
        bool foreign = false;
        const std::string_view name = localName(n.name, foreign);
        if (foreign) {
            return;  // other namespaces (inkscape:, sodipodi:, ...) carry no geometry
        }
        if (isHidden(n)) {
            return;
        }
        Ctx ctx = parent;
        if (const auto vis = presentation(n, "visibility")) {
            if (*vis == "hidden" || *vis == "collapse") {
                ctx.visible = false;
            } else if (*vis == "visible") {
                ctx.visible = true;
            }
        }
        if (const std::string* t = n.attr("transform")) {
            const auto m = parseTransform(*t);
            if (!m) {
                warn(n.line, "<" + std::string(name) + "> with an invalid transform skipped");
                return;
            }
            ctx.ctm = ctx.ctm * *m;
            if (!ctx.ctm.isInvertible(1e-24)) {
                warn(n.line, "<" + std::string(name) + "> with a non-invertible transform (zero scale) skipped");
                return;
            }
            if (!sane(ctx.ctm)) {
                warn(n.line, "<" + std::string(name) + "> with an out-of-range transform skipped");
                return;
            }
        }
        if (isSkippedType(name)) {
            warn(n.line, "<" + std::string(name) + "> element skipped");
            return;
        }
        if (name == "defs") {
            for (std::size_t c = n.firstChild; c != kNoNode; c = doc_.nodes[c].nextSibling) {
                bool f = false;
                const std::string_view cn = localName(doc_.nodes[c].name, f);
                if (!f && isSkippedType(cn)) {
                    warn(doc_.nodes[c].line, "<" + std::string(cn) + "> element skipped");
                }
            }
            return;
        }
        if (name == "title" || name == "desc" || name == "metadata") {
            return;
        }
        if (name == "symbol" && !viaUse) {
            return;
        }
        const bool isShape = name == "path" || name == "rect" || name == "circle" || name == "ellipse" ||
                             name == "line" || name == "polyline" || name == "polygon";
        const bool isGroup = name == "g" || name == "a" || name == "svg" || name == "symbol";
        if (!isShape && !isGroup && name != "use") {
            warn(n.line, "unknown element <" + std::string(name) + "> skipped");
            return;
        }
        for (const char* attr : {"clip-path", "mask", "filter"}) {
            if (const auto v = presentation(n, attr)) {
                if (*v != "none" && !v->empty()) {
                    warn(n.line, std::string(attr) + " attribute ignored");
                }
            }
        }
        if (isShape) {
            if (ctx.visible) {
                shape(name, n, ctx);
            }
            return;
        }
        if (name == "use") {
            use(stack, n, ctx);
            return;
        }
        if (name == "svg") {
            ctx.ctm = ctx.ctm * Transform2::translation(Vec2{num2(n, "x", 'x'), num2(n, "y", 'y')});
            if (n.attr("viewBox") != nullptr || n.attr("width") != nullptr || n.attr("height") != nullptr) {
                warn(n.line, "nested <svg> viewport (size, viewBox) ignored");
            }
        } else if (name == "symbol" && n.attr("viewBox") != nullptr) {
            warn(n.line, "<symbol> viewBox ignored");
        }
        if (name == "g") {
            const std::string* mode = n.attr("inkscape:groupmode");
            const bool layerMode = mode != nullptr && *mode == "layer";
            std::string label;
            if (const std::string* l = n.attr("inkscape:label")) {
                label = *l;
            }
            if (label.empty()) {
                if (const std::string* id = n.attr("id")) {
                    label = *id;
                }
            }
            if (label.empty() && layerMode) {
                label = "Layer " + std::to_string(out_.layers.size() + 1);
            }
            if ((layerMode || n.parent == 0) && !label.empty()) {
                ctx.layer = layerIndex(label);
            }
        }
        push(stack, idx, n.firstChild, ctx);
    }

    double num2(const XmlNode& n, const char* name, char axis) {
        bool valid = true;
        return num(n, name, axis, valid);
    }

    void use(std::vector<Frame>& stack, const XmlNode& n, Ctx ctx) {
        const std::string* href = n.attr("href");
        if (href == nullptr) {
            href = n.attr("xlink:href");
        }
        if (href == nullptr) {
            return;
        }
        const std::string_view ref = trim(*href);
        if (ref.size() < 2 || ref.front() != '#') {
            warn(n.line, "<use> of an external or empty reference skipped");
            return;
        }
        const auto it = ids_.find(std::string(ref.substr(1)));
        if (it == ids_.end()) {
            warn(n.line, "<use> of an unknown id skipped");
            return;
        }
        if (onStack_[it->second] != 0 || ctx.useDepth >= opt_.maxUseDepth) {
            warn(n.line, "<use> that refers to itself or nests too deeply skipped");
            return;
        }
        ctx.ctm = ctx.ctm * Transform2::translation(Vec2{num2(n, "x", 'x'), num2(n, "y", 'y')});
        ++ctx.useDepth;
        enter(stack, it->second, ctx, true);
    }

    const XmlDoc& doc_;
    SvgImportOptions opt_;
    std::vector<char> onStack_;
    std::map<std::string, std::size_t> ids_;
    std::map<std::string, std::size_t> layerByName_;
    std::map<std::string, std::size_t> warnIndex_;
    SvgDrawing out_;
    std::optional<SvgError> error_;
    std::size_t visits_{0};
    std::size_t spans_{0};
    double viewW_{100.0};
    double viewH_{100.0};
};

}  // namespace

SvgResult<SvgDrawing> importSvg(std::span<const std::uint8_t> bytes, const SvgImportOptions& options) {
    detail::XmlLimits lim;
    lim.maxInputBytes = options.maxInputBytes;
    lim.maxDepth = options.maxDepth;
    lim.maxElements = options.maxElements;
    lim.maxAttributes = options.maxAttributes;
    lim.maxAttributeLength = options.maxAttributeLength;
    lim.maxLongAttributeLength = options.maxPathLength;
    if (!(options.dpi > 0.0) || !std::isfinite(options.dpi)) {
        return SvgError{SvgError::Code::InvalidArgument, "dpi must be positive", 0};
    }
    const std::string_view text(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    auto parsed = detail::parseXml(text, lim);
    if (!parsed.ok()) {
        return parsed.error();
    }
    return Importer(parsed.value(), options).run();
}

}  // namespace carvetto::io
