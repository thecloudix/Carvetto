#include "carvetto/scripting/gadget_api.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <set>
#include <utility>

#include "carvetto/document/command.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/transform2.hpp"

// Binding of the gadget API (ADR-0102, ADR-0108). Every function validates its arguments and
// raises a ScriptError with a message naming the function; nothing here can crash on a bad id or
// value (stale ids are errors, ADR-0102 principle 3). Edits are commands executed inside one
// UndoStack macro opened by `runGadget`.
namespace carvetto::scripting {

namespace {

using document::Document;
using document::LayerId;
using document::ObjectId;
using document::ToolpathId;
using geometry::Contour;
using geometry::Vec2;

constexpr double kPi = 3.14159265358979323846;
constexpr std::size_t kMaxFormFields = 64;
constexpr std::size_t kMaxNameBytes = 256;
constexpr std::size_t kMaxTextBytes = 16 * 1024;

[[noreturn]] void fail(const std::string& fn, const std::string& what) {
    throw ScriptError{fn + ": " + what};
}

ScriptValues one(ScriptValue v) {
    ScriptValues r;
    r.push_back(std::move(v));
    return r;
}

ScriptValue::Array pointValue(Vec2 p) {
    ScriptValue::Array a;
    a.emplace_back(p.x);
    a.emplace_back(p.y);
    return a;
}

/// State of one run: the document, the services, caps, counters and the pausable clock.
struct Session {
    Document& doc;
    const GadgetServices& sv;
    const GadgetLimits& gl;
    std::size_t edits{0};
    std::size_t createdCount{0};
    std::size_t toolpathCalls{0};
    std::vector<ObjectId> createdObjects;
    std::vector<ToolpathId> createdToolpaths;
    std::vector<std::string> log;
    std::size_t logBytes{0};

    Session(Document& d, const GadgetServices& se, const GadgetLimits& g) : doc(d), sv(se), gl(g) {}

    std::function<std::uint64_t()> base;
    std::uint64_t pausedNs{0};

    /// The deadline clock: the time source minus the time spent paused. (The script thread is the
    /// only reader and it is inside the host callback while paused, so no read sees a pause in
    /// progress.)
    std::uint64_t now() const { return base() - pausedNs; }
    /// Stops the deadline clock while host code the script waits for (a dialog, a toolpath
    /// calculation) runs.
    struct Pause {
        Session& s;
        std::uint64_t start;
        explicit Pause(Session& se) : s(se), start(se.base()) {}
        ~Pause() { s.pausedNs += s.base() - start; }
        Pause(const Pause&) = delete;
        Pause& operator=(const Pause&) = delete;
    };
};

// ---- argument helpers -------------------------------------------------------------------------

const ScriptValue& arg(const ScriptValues& a, std::size_t i) {
    static const ScriptValue nil;
    return i < a.size() ? a[i] : nil;
}

double number(const std::string& fn, const ScriptValue& v, const char* what) {
    if (!v.isNumber()) {
        fail(fn, std::string(what) + " must be a number");
    }
    const double d = v.asNumber();
    if (!std::isfinite(d)) {
        fail(fn, std::string(what) + " must be a finite number");
    }
    return d;
}

double numberArg(const std::string& fn, const ScriptValues& a, std::size_t i, const char* what) {
    return number(fn, arg(a, i), what);
}

double numberArgOr(const std::string& fn, const ScriptValues& a, std::size_t i, const char* what, double dflt) {
    return arg(a, i).isNil() ? dflt : number(fn, arg(a, i), what);
}

std::int64_t integer(const std::string& fn, const ScriptValue& v, const char* what) {
    if (v.isInt()) {
        return v.asInt();
    }
    if (v.isNumber()) {
        const double d = v.asNumber();
        if (std::isfinite(d) && d == std::floor(d) && std::fabs(d) < 9.0e15) {
            return static_cast<std::int64_t>(d);
        }
    }
    fail(fn, std::string(what) + " must be an integer");
}

const std::string& stringArg(const std::string& fn, const ScriptValues& a, std::size_t i, const char* what) {
    const ScriptValue& v = arg(a, i);
    if (!v.isString()) {
        fail(fn, std::string(what) + " must be a string");
    }
    return v.asString();
}

bool hasControl(const std::string& s) {
    return std::any_of(s.begin(), s.end(), [](char c) { return static_cast<unsigned char>(c) < 0x20; });
}

void checkName(const std::string& fn, const std::string& name, const char* what) {
    if (name.empty() || name.size() > kMaxNameBytes || hasControl(name)) {
        fail(fn, std::string(what) + " must be 1 to 256 characters without control characters");
    }
}

Vec2 point(const std::string& fn, const ScriptValue& v, const char* what) {
    if (!v.isArray() || v.asArray().size() != 2) {
        fail(fn, std::string(what) + " must be a point {x, y}");
    }
    return Vec2{number(fn, v.asArray()[0], what), number(fn, v.asArray()[1], what)};
}

ObjectId objectId(Session& s, const std::string& fn, const ScriptValue& v) {
    const std::int64_t n = integer(fn, v, "object id");
    if (n <= 0 || !s.doc.hasObject(ObjectId{static_cast<std::uint64_t>(n)})) {
        fail(fn, "no such object (id " + std::to_string(n) + ")");
    }
    return ObjectId{static_cast<std::uint64_t>(n)};
}

/// One id or an array of ids; each must exist; duplicates dropped, first occurrence wins.
std::vector<ObjectId> objectList(Session& s, const std::string& fn, const ScriptValue& v) {
    std::vector<ObjectId> out;
    if (v.isArray()) {
        std::set<ObjectId> seen;
        for (const ScriptValue& e : v.asArray()) {
            const ObjectId id = objectId(s, fn, e);
            if (seen.insert(id).second) {
                out.push_back(id);
            }
        }
    } else {
        out.push_back(objectId(s, fn, v));
    }
    if (out.empty()) {
        fail(fn, "no objects given");
    }
    return out;
}

LayerId layerId(Session& s, const std::string& fn, const ScriptValue& v) {
    const std::int64_t n = integer(fn, v, "layer id");
    if (n <= 0 || s.doc.findLayer(LayerId{static_cast<std::uint64_t>(n)}) == nullptr) {
        fail(fn, "no such layer (id " + std::to_string(n) + ")");
    }
    return LayerId{static_cast<std::uint64_t>(n)};
}

/// The layer to draw on: nil = the active layer. A locked layer cannot be drawn on.
LayerId writableLayer(Session& s, const std::string& fn, const ScriptValue& v) {
    const LayerId id = v.isNil() ? s.doc.activeLayer() : layerId(s, fn, v);
    if (s.doc.findLayer(id)->locked) {
        fail(fn, "layer \"" + s.doc.findLayer(id)->name + "\" is locked");
    }
    return id;
}

/// Reads the keys of a parameter table; `finish` rejects the keys nobody asked for (a typo must
/// not be silently ignored).
class Params {
   public:
    Params(std::string fn, const ScriptValue& v) : fn_(std::move(fn)) {
        if (v.isMap()) {
            map_ = &v.asMap();
        } else if (!(v.isArray() && v.asArray().empty()) && !v.isNil()) {
            fail(fn_, "parameters must be a table with string keys");
        }
    }
    bool has(const std::string& key) const { return find(key) != nullptr; }
    double num(const std::string& key, std::optional<double> dflt = std::nullopt) {
        const ScriptValue* v = find(key);
        if (v == nullptr) {
            if (!dflt) {
                fail(fn_, "missing parameter '" + key + "'");
            }
            return *dflt;
        }
        return number(fn_, *v, ("parameter '" + key + "'").c_str());
    }
    int count(const std::string& key, int dflt) {
        const double d = num(key, static_cast<double>(dflt));
        if (d != std::floor(d) || d < 0.0 || d > 1.0e6) {
            fail(fn_, "parameter '" + key + "' must be a whole number");
        }
        return static_cast<int>(d);
    }
    std::string text(const std::string& key, const std::string& dflt) {
        const ScriptValue* v = find(key);
        if (v == nullptr) {
            return dflt;
        }
        if (!v->isString()) {
            fail(fn_, "parameter '" + key + "' must be a string");
        }
        return v->asString();
    }
    void finish() const {
        if (map_ == nullptr) {
            return;
        }
        for (const auto& kv : map_->items) {
            if (!used_.contains(kv.first)) {
                fail(fn_, "unknown parameter '" + kv.first + "'");
            }
        }
    }

   private:
    const ScriptValue* find(const std::string& key) const {
        used_.insert(key);
        return map_ != nullptr ? map_->find(key) : nullptr;
    }
    std::string fn_;
    const ScriptValue::Map* map_{nullptr};
    mutable std::set<std::string> used_;
};

// ---- geometry conversion ----------------------------------------------------------------------

/// Contour -> {points = {{x,y},...}, bulges = {...}, closed = bool}. Arcs become bulge vertices
/// (bulge = tan(sweep / 4), positive counter-clockwise; an arc over half a turn is cut into
/// equal pieces so no bulge is infinite), Beziers are flattened to the default tolerance.
ScriptValue contourValue(const Contour& c) {
    ScriptValue::Array points;
    ScriptValue::Array bulges;
    const auto add = [&](Vec2 p, double bulge) {
        points.emplace_back(pointValue(p));
        bulges.emplace_back(bulge);
    };
    for (const geometry::Span& span : c.spans) {
        if (const auto* line = std::get_if<geometry::LineSpan>(&span)) {
            add(line->start, 0.0);
        } else if (const auto* arc = std::get_if<geometry::ArcSpan>(&span)) {
            const double sweep = arc->sweepAngle();
            const int n = std::max(1, static_cast<int>(std::ceil(std::fabs(sweep) / kPi - 1e-9)));
            const double bulge = std::tan(sweep / (4.0 * static_cast<double>(n)));
            add(arc->startPoint(), bulge);
            for (int i = 1; i < n; ++i) {
                add(arc->pointAt(static_cast<double>(i) / static_cast<double>(n)), bulge);
            }
        } else if (const auto* bez = std::get_if<geometry::BezierSpan>(&span)) {
            Contour one;
            one.spans.push_back(*bez);
            const geometry::Polyline pl = one.polygonise(geometry::Tolerance::defaults());
            if (pl.points.empty()) {
                add(bez->p0, 0.0);
            }
            for (std::size_t i = 0; i + 1 < pl.points.size(); ++i) {
                add(pl.points[i], 0.0);
            }
        }
    }
    if (!c.closed && !c.spans.empty()) {
        add(geometry::endPoint(c.spans.back()), 0.0);
    }
    ScriptValue::Map m;
    m.items.emplace_back("bulges", ScriptValue(std::move(bulges)));
    m.items.emplace_back("closed", ScriptValue(c.closed));
    m.items.emplace_back("points", ScriptValue(std::move(points)));
    return ScriptValue(std::move(m));
}

/// Contour of line and arc spans through `pts` (a bulge on a vertex bends the segment to the next
/// vertex into an arc, tan(sweep / 4), positive counter-clockwise). Repeated points are dropped.
Contour buildPath(const std::string& fn, const ScriptValue::Array& ptsIn, const ScriptValue* bulgesIn, bool closed,
                  std::size_t maxPoints) {
    if (ptsIn.size() < 2) {
        fail(fn, "a contour needs at least two points");
    }
    if (ptsIn.size() > maxPoints) {
        fail(fn, "too many points");
    }
    std::vector<Vec2> pts;
    std::vector<double> bul;
    pts.reserve(ptsIn.size());
    if (bulgesIn != nullptr && !bulgesIn->isNil()) {
        if (!bulgesIn->isArray() || bulgesIn->asArray().size() != ptsIn.size()) {
            fail(fn, "bulges must be an array with one number per point");
        }
    }
    for (std::size_t i = 0; i < ptsIn.size(); ++i) {
        pts.push_back(point(fn, ptsIn[i], "a point"));
        double b = 0.0;
        if (bulgesIn != nullptr && !bulgesIn->isNil()) {
            b = number(fn, bulgesIn->asArray()[i], "a bulge");
            if (std::fabs(b) > 1.0e6) {
                fail(fn, "a bulge is out of range");
            }
        }
        bul.push_back(b);
    }
    const std::size_t n = pts.size();
    const std::size_t segs = closed ? n : n - 1;
    Contour out;
    out.closed = closed;
    for (std::size_t i = 0; i < segs; ++i) {
        const Vec2 a = pts[i];
        const Vec2 b = pts[(i + 1) % n];
        const double chord = geometry::distance(a, b);
        if (bul[i] == 0.0) {
            if (chord < 1e-9) {
                continue;  // a repeated point adds nothing
            }
            out.spans.emplace_back(geometry::LineSpan{a, b});
            continue;
        }
        if (chord < 1e-9) {
            fail(fn, "a vertex with a bulge needs a distinct next point");
        }
        const double theta = 4.0 * std::atan(bul[i]);
        const Vec2 dir = (b - a) * (1.0 / chord);
        const Vec2 left{-dir.y, dir.x};
        const double h = (chord * 0.5) / std::tan(theta * 0.5);
        const Vec2 centre = (a + b) * 0.5 + left * h;
        out.spans.emplace_back(geometry::ArcSpan::fromCentre(
            centre, a, b, theta > 0.0 ? geometry::ArcDirection::CCW : geometry::ArcDirection::CW));
    }
    if (out.spans.empty()) {
        fail(fn, "all points coincide");
    }
    return out;
}

// ---- object creation ---------------------------------------------------------------------------

void countCreated(Session& s, const std::string& fn, std::size_t n = 1) {
    if (s.createdCount + n > s.gl.maxCreatedObjects) {
        fail(fn, "this gadget created too many objects (limit " + std::to_string(s.gl.maxCreatedObjects) + ")");
    }
    s.createdCount += n;
}

ObjectId addObject(Session& s, const std::string& fn, document::Object obj) {
    if (!s.doc.idGenerator().canAllocate(1)) {
        fail(fn, "the document has no free ids left");
    }
    auto cmd = std::make_unique<document::AddObjectCommand>(std::move(obj));
    const document::AddObjectCommand* raw = cmd.get();
    if (!s.doc.execute(std::move(cmd))) {
        fail(fn, "the object was refused");
    }
    ++s.edits;
    s.createdObjects.push_back(raw->id());
    return raw->id();
}

void execute(Session& s, const std::string& fn, std::unique_ptr<document::Command> cmd) {
    if (!s.doc.execute(std::move(cmd))) {
        fail(fn, "the edit was refused");
    }
    ++s.edits;
}

document::CreationParams shapeParams(const std::string& fn, const std::string& kind, Params& p) {
    const Vec2 centre{p.num("cx", 0.0), p.num("cy", 0.0)};
    if (kind == "rect") {
        document::RectangleParams r;
        r.centreMm = centre;
        r.widthMm = p.num("width");
        r.heightMm = p.num("height");
        const std::string corner = p.text("corner", "square");
        if (corner == "square") {
            r.corners = document::RectangleParams::CornerStyle::Square;
        } else if (corner == "rounded") {
            r.corners = document::RectangleParams::CornerStyle::Rounded;
        } else if (corner == "chamfered") {
            r.corners = document::RectangleParams::CornerStyle::Chamfered;
        } else {
            fail(fn, "corner must be \"square\", \"rounded\" or \"chamfered\"");
        }
        r.cornerRadiusMm = p.num("corner_size", 0.0);
        return r;
    }
    if (kind == "circle") {
        document::CircleParams c;
        c.centreMm = centre;
        c.radiusMm = p.num("radius");
        return c;
    }
    if (kind == "ellipse") {
        document::EllipseParams e;
        e.centreMm = centre;
        e.widthMm = p.num("width");
        e.heightMm = p.num("height");
        e.angle = p.num("angle", 0.0);
        return e;
    }
    if (kind == "polygon") {
        document::PolygonParams g;
        g.centreMm = centre;
        g.sides = p.count("sides", 0);
        g.radiusMm = p.num("radius");
        g.angle = p.num("angle", 0.0);
        return g;
    }
    if (kind == "star") {
        document::StarParams t;
        t.centreMm = centre;
        t.points = p.count("points", 0);
        t.outerRadiusMm = p.num("outer");
        t.innerRadiusMm = p.num("inner");
        t.angle = p.num("angle", 0.0);
        return t;
    }
    fail(fn, "unknown shape \"" + kind + "\" (rect, circle, ellipse, polygon, star)");
}

// ---- form specification ------------------------------------------------------------------------

FormFieldKind fieldKind(const std::string& fn, const std::string& k) {
    if (k == "number") return FormFieldKind::Number;
    if (k == "text") return FormFieldKind::Text;
    if (k == "choice") return FormFieldKind::Choice;
    if (k == "bool") return FormFieldKind::Bool;
    if (k == "layer") return FormFieldKind::Layer;
    if (k == "tool") return FormFieldKind::Tool;
    fail(fn, "unknown field kind \"" + k + "\" (number, text, choice, bool, layer, tool)");
}

FormSpec parseForm(Session& s, const std::string& fn, const ScriptValue& v) {
    if (!v.isMap()) {
        fail(fn, "the form specification must be a table {title=..., fields={...}}");
    }
    Params top(fn, v);
    FormSpec spec;
    spec.title = top.text("title", "Gadget");
    if (spec.title.size() > kMaxNameBytes || hasControl(spec.title)) {
        fail(fn, "bad title");
    }
    const ScriptValue* fields = v.asMap().find("fields");
    (void)top.has("fields");  // known key, read below
    top.finish();
    if (fields == nullptr || !(fields->isArray())) {
        fail(fn, "fields must be an array of field tables");
    }
    if (fields->asArray().size() > kMaxFormFields) {
        fail(fn, "too many fields (limit 64)");
    }
    std::set<std::string> names;
    for (const ScriptValue& fv : fields->asArray()) {
        if (!fv.isMap()) {
            fail(fn, "each field must be a table {name=..., kind=...}");
        }
        const ScriptValue::Map& fm = fv.asMap();
        static const std::set<std::string> known{"name", "label", "kind", "default", "choices", "min", "max"};
        for (const auto& kv : fm.items) {
            if (!known.contains(kv.first)) {
                fail(fn, "unknown field key '" + kv.first + "'");
            }
        }
        FormField f;
        const ScriptValue* nm = fm.find("name");
        if (nm == nullptr || !nm->isString()) {
            fail(fn, "each field needs a name");
        }
        f.name = nm->asString();
        checkName(fn, f.name, "a field name");
        if (!names.insert(f.name).second) {
            fail(fn, "duplicate field name '" + f.name + "'");
        }
        const ScriptValue* lb = fm.find("label");
        f.label = (lb != nullptr && lb->isString()) ? lb->asString() : f.name;
        if (lb != nullptr && !lb->isString()) {
            fail(fn, "label must be a string");
        }
        if (f.label.size() > kMaxNameBytes || hasControl(f.label)) {
            fail(fn, "bad label");
        }
        const ScriptValue* kd = fm.find("kind");
        f.kind = fieldKind(fn, kd != nullptr && kd->isString() ? kd->asString() : std::string("number"));
        const ScriptValue* df = fm.find("default");
        if (const ScriptValue* ch = fm.find("choices")) {
            if (!ch->isArray()) {
                fail(fn, "choices must be an array of strings");
            }
            for (const ScriptValue& c : ch->asArray()) {
                if (!c.isString() || c.asString().size() > kMaxNameBytes) {
                    fail(fn, "choices must be an array of strings");
                }
                f.choices.push_back(c.asString());
            }
        }
        if (const ScriptValue* mn = fm.find("min")) {
            f.min = number(fn, *mn, "min");
        }
        if (const ScriptValue* mx = fm.find("max")) {
            f.max = number(fn, *mx, "max");
        }
        switch (f.kind) {
            case FormFieldKind::Number:
                f.defaultValue = df != nullptr ? ScriptValue(number(fn, *df, "a number default")) : ScriptValue(0.0);
                break;
            case FormFieldKind::Text:
                if (df != nullptr && !df->isString()) {
                    fail(fn, "a text default must be a string");
                }
                f.defaultValue = df != nullptr ? *df : ScriptValue(std::string());
                break;
            case FormFieldKind::Choice:
                if (f.choices.empty()) {
                    fail(fn, "a choice field needs choices");
                }
                if (df != nullptr && (!df->isString() ||
                                      std::find(f.choices.begin(), f.choices.end(), df->asString()) == f.choices.end())) {
                    fail(fn, "a choice default must be one of the choices");
                }
                f.defaultValue = df != nullptr ? *df : ScriptValue(f.choices.front());
                break;
            case FormFieldKind::Bool:
                if (df != nullptr && !df->isBool()) {
                    fail(fn, "a bool default must be true or false");
                }
                f.defaultValue = df != nullptr ? *df : ScriptValue(false);
                break;
            case FormFieldKind::Layer:
                f.defaultValue = df != nullptr ? ScriptValue(static_cast<std::int64_t>(layerId(s, fn, *df).value))
                                               : ScriptValue(static_cast<std::int64_t>(s.doc.activeLayer().value));
                break;
            case FormFieldKind::Tool:
                if (df != nullptr && !df->isString()) {
                    fail(fn, "a tool default must be a tool id string");
                }
                f.defaultValue = df != nullptr ? *df : ScriptValue(std::string());
                break;
        }
        spec.fields.push_back(std::move(f));
    }
    return spec;
}

/// The values a form callback returned, checked against the spec; a host bug must not hand the
/// script a value of the wrong type.
ScriptValue::Map checkedFormValues(Session& s, const std::string& fn, const FormSpec& spec, ScriptValue::Map in) {
    ScriptValue::Map out;
    for (const FormField& f : spec.fields) {
        const ScriptValue* v = in.find(f.name);
        const std::string bad = "the form returned an invalid value for '" + f.name + "'";
        if (v == nullptr) {
            fail(fn, bad);
        }
        switch (f.kind) {
            case FormFieldKind::Number:
                if (!v->isNumber() || !std::isfinite(v->asNumber()) || (f.min && v->asNumber() < *f.min) ||
                    (f.max && v->asNumber() > *f.max)) {
                    fail(fn, bad);
                }
                break;
            case FormFieldKind::Text:
            case FormFieldKind::Tool:
                if (!v->isString() || v->asString().size() > kMaxTextBytes) {
                    fail(fn, bad);
                }
                break;
            case FormFieldKind::Choice:
                if (!v->isString() ||
                    std::find(f.choices.begin(), f.choices.end(), v->asString()) == f.choices.end()) {
                    fail(fn, bad);
                }
                break;
            case FormFieldKind::Bool:
                if (!v->isBool()) {
                    fail(fn, bad);
                }
                break;
            case FormFieldKind::Layer:
                if (!v->isInt() || v->asInt() <= 0 ||
                    s.doc.findLayer(LayerId{static_cast<std::uint64_t>(v->asInt())}) == nullptr) {
                    fail(fn, bad);
                }
                break;
        }
        out.items.emplace_back(f.name, *v);
    }
    return out;
}

// ---- toolpath moves ----------------------------------------------------------------------------

std::vector<toolpath::Move> parseMoves(const std::string& fn, const ScriptValue& v, std::size_t maxMoves) {
    if (!v.isArray() || v.asArray().empty()) {
        fail(fn, "moves must be a non-empty array of move tables");
    }
    if (v.asArray().size() > maxMoves) {
        fail(fn, "too many moves");
    }
    std::vector<toolpath::Move> out;
    out.reserve(v.asArray().size());
    geometry::Vec3 cur{};
    bool have = false;
    for (const ScriptValue& mv : v.asArray()) {
        if (!mv.isMap()) {
            fail(fn, "each move must be a table {kind=..., x=..., y=..., z=...}");
        }
        Params p(fn, mv);
        const std::string kind = p.text("kind", "");
        static const std::set<std::string> kinds{"rapid", "feed", "plunge", "retract", "arc_cw", "arc_ccw", "dwell"};
        if (!kinds.contains(kind)) {
            fail(fn, "unknown move kind \"" + kind + "\" (rapid, feed, plunge, retract, arc_cw, arc_ccw, dwell)");
        }
        const bool dwell = kind == "dwell";
        geometry::Vec3 to = cur;
        if (!dwell || p.has("x") || p.has("y") || p.has("z")) {
            to.x = p.num("x", have ? std::optional<double>(cur.x) : std::nullopt);
            to.y = p.num("y", have ? std::optional<double>(cur.y) : std::nullopt);
            to.z = p.num("z", have ? std::optional<double>(cur.z) : std::nullopt);
        } else if (!have) {
            fail(fn, "a dwell first in the path needs x, y and z");
        }
        std::optional<double> feed;
        if (p.has("feed")) {
            feed = p.num("feed");
            if (*feed <= 0.0) {
                fail(fn, "feed must be positive");
            }
        }
        toolpath::Move m;
        if (kind == "rapid") {
            m = toolpath::Move::rapid(to);
        } else if (kind == "feed") {
            m = toolpath::Move::feed(to, feed);
        } else if (kind == "plunge") {
            m = toolpath::Move::plunge(to, feed);
        } else if (kind == "retract") {
            m = toolpath::Move::retract(to, feed);
        } else if (kind == "arc_cw" || kind == "arc_ccw") {
            const Vec2 c{p.num("cx"), p.num("cy")};
            m = kind == "arc_cw" ? toolpath::Move::arcCW(to, c, feed) : toolpath::Move::arcCCW(to, c, feed);
        } else if (dwell) {
            const double secs = p.num("seconds");
            if (secs < 0.0 || secs > 1.0e6) {
                fail(fn, "seconds must be between 0 and 1000000");
            }
            m = toolpath::Move::dwell(to, secs);
        }
        p.finish();
        cur = to;
        have = true;
        out.push_back(m);
    }
    return out;
}

std::string toText(const ScriptValue& v) {
    if (v.isNil()) return "nil";
    if (v.isBool()) return v.asBool() ? "true" : "false";
    if (v.isInt()) return std::to_string(v.asInt());
    if (v.isNumber()) {
        char buf[48];
        std::snprintf(buf, sizeof buf, "%.14g", v.asNumber());
        return buf;
    }
    if (v.isString()) return v.asString();
    return "table";
}

}  // namespace

const std::vector<std::string>& toolpathKinds() {
    static const std::vector<std::string> kinds{"profile", "pocket", "drill", "engrave"};
    return kinds;
}

GadgetRunResult runGadget(document::Document& doc, const GadgetServices& services, std::string_view source,
                          std::string_view chunkName, const GadgetRunOptions& options) {
    GadgetRunResult result;
    document::UndoStack& undo = doc.undoStack();
    if (undo.inMacro()) {
        result.script.status = ScriptStatus::RuntimeError;
        result.script.chunkName = std::string(chunkName);
        result.script.message = "a gadget cannot run while another edit sequence is open";
        return result;
    }

    Session s(doc, services, options.gadgetLimits);
    ScriptLimits limits = options.limits;
    if (limits.clockNanos) {
        s.base = limits.clockNanos;
    } else {
        s.base = [] {
            return static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
                    .count());
        };
    }
    limits.clockNanos = [&s] { return s.now(); };

    ScriptHost host(limits);
    const auto reg = [&host](const char* module, const char* name, HostFunction fn) {
        host.registerFunction(module, name, std::move(fn));
    };
    host.setValue("carvetto", "api_version", ScriptValue(kGadgetApiVersion));

    // ---- carvetto.log -----------------------------------------------------------------------
    reg("carvetto", "log", [&s](const ScriptValues& a) {
        std::string line;
        for (std::size_t i = 0; i < a.size(); ++i) {
            if (i > 0) {
                line += '\t';
            }
            line += toText(a[i]);
        }
        if (s.log.size() < s.gl.maxLogLines && s.logBytes < s.gl.maxLogBytes) {
            if (line.size() > 4096) {
                line.resize(4096);
            }
            s.logBytes += line.size() + 1;
            s.log.push_back(std::move(line));
        }
        return ScriptValues{};
    });

    // ---- document (read) --------------------------------------------------------------------
    reg("carvetto.doc", "units", [&s](const ScriptValues&) {
        return one(ScriptValue(s.doc.job().displayUnits == document::Units::Inches ? "inch" : "mm"));
    });
    reg("carvetto.doc", "job_size", [&s](const ScriptValues&) {
        const document::JobSize& z = s.doc.job().sizeMm;
        ScriptValues r;
        r.emplace_back(z.x);
        r.emplace_back(z.y);
        r.emplace_back(z.z);
        return r;
    });
    reg("carvetto.doc", "layers", [&s](const ScriptValues&) {
        ScriptValue::Array ids;
        for (const document::Layer& l : s.doc.layers()) {
            ids.emplace_back(static_cast<std::int64_t>(l.id.value));
        }
        return one(ScriptValue(std::move(ids)));
    });
    reg("carvetto.doc", "active_layer", [&s](const ScriptValues&) {
        return one(ScriptValue(static_cast<std::int64_t>(s.doc.activeLayer().value)));
    });
    reg("carvetto.doc", "variable", [&s](const ScriptValues& a) {
        const std::string name = stringArg("carvetto.doc.variable", a, 0, "name");
        const std::optional<double> v = s.doc.variables().get(name);
        return v ? one(ScriptValue(*v)) : one(ScriptValue());
    });
    reg("carvetto.doc", "variables", [&s](const ScriptValues&) {
        ScriptValue::Map m;
        for (const auto& kv : s.doc.variables().all()) {
            m.items.emplace_back(kv.first, ScriptValue(kv.second));
        }
        if (m.items.empty()) {
            return one(ScriptValue(ScriptValue::Array{}));
        }
        return one(ScriptValue(std::move(m)));
    });
    reg("carvetto.doc", "set_variable", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.doc.set_variable";
        const std::string name = stringArg(fn, a, 0, "name");
        if (!document::DocumentVariables::isValidName(name) || name.size() > kMaxNameBytes) {
            fail(fn, "a variable name starts with a letter and continues with letters, digits or _");
        }
        const double value = numberArg(fn, a, 1, "value");
        if (s.doc.variables().get(name) == value) {
            return ScriptValues{};
        }
        execute(s, fn, std::make_unique<document::SetDocumentVariableCommand>(name, value));
        return ScriptValues{};
    });

    // ---- layers -----------------------------------------------------------------------------
    reg("carvetto.layer", "name", [&s](const ScriptValues& a) {
        return one(ScriptValue(s.doc.findLayer(layerId(s, "carvetto.layer.name", arg(a, 0)))->name));
    });
    reg("carvetto.layer", "visible", [&s](const ScriptValues& a) {
        return one(ScriptValue(s.doc.findLayer(layerId(s, "carvetto.layer.visible", arg(a, 0)))->visible));
    });
    reg("carvetto.layer", "locked", [&s](const ScriptValues& a) {
        return one(ScriptValue(s.doc.findLayer(layerId(s, "carvetto.layer.locked", arg(a, 0)))->locked));
    });
    reg("carvetto.layer", "colour", [&s](const ScriptValues& a) {
        const document::Colour c = s.doc.findLayer(layerId(s, "carvetto.layer.colour", arg(a, 0)))->colour;
        char buf[16];
        std::snprintf(buf, sizeof buf, "#%02x%02x%02x", c.r, c.g, c.b);
        return one(ScriptValue(std::string(buf)));
    });
    const auto parseColour = [](const std::string& fn, const ScriptValue& v) {
        if (!v.isString() || v.asString().size() != 7 || v.asString()[0] != '#') {
            fail(fn, "a colour is a string like \"#ff8800\"");
        }
        std::uint32_t rgb = 0;
        for (std::size_t i = 1; i < 7; ++i) {
            const char ch = v.asString()[i];
            std::uint32_t d = 0;
            if (ch >= '0' && ch <= '9') {
                d = static_cast<std::uint32_t>(ch - '0');
            } else if (ch >= 'a' && ch <= 'f') {
                d = static_cast<std::uint32_t>(ch - 'a' + 10);
            } else if (ch >= 'A' && ch <= 'F') {
                d = static_cast<std::uint32_t>(ch - 'A' + 10);
            } else {
                fail(fn, "a colour is a string like \"#ff8800\"");
            }
            rgb = rgb * 16 + d;
        }
        return document::Colour{static_cast<std::uint8_t>(rgb >> 16), static_cast<std::uint8_t>((rgb >> 8) & 0xff),
                                static_cast<std::uint8_t>(rgb & 0xff)};
    };
    reg("carvetto.layer", "create", [&s, parseColour](const ScriptValues& a) {
        const std::string fn = "carvetto.layer.create";
        const std::string name = stringArg(fn, a, 0, "name");
        checkName(fn, name, "a layer name");
        document::Layer l;
        l.name = name;
        if (!arg(a, 1).isNil()) {
            l.colour = parseColour(fn, arg(a, 1));
        }
        countCreated(s, fn);
        auto cmd = std::make_unique<document::AddLayerCommand>(l);
        const document::AddLayerCommand* raw = cmd.get();
        execute(s, fn, std::move(cmd));
        return one(ScriptValue(static_cast<std::int64_t>(raw->id().value)));
    });
    reg("carvetto.layer", "set", [&s, parseColour](const ScriptValues& a) {
        const std::string fn = "carvetto.layer.set";
        const LayerId id = layerId(s, fn, arg(a, 0));
        Params p(fn, arg(a, 1));
        document::Layer l = *s.doc.findLayer(id);
        if (p.has("name")) {
            l.name = p.text("name", "");
            checkName(fn, l.name, "a layer name");
        }
        if (arg(a, 1).isMap()) {
            if (const ScriptValue* c = arg(a, 1).asMap().find("colour")) {
                l.colour = parseColour(fn, *c);
            }
            if (const ScriptValue* v = arg(a, 1).asMap().find("visible")) {
                if (!v->isBool()) {
                    fail(fn, "visible must be true or false");
                }
                l.visible = v->asBool();
            }
            if (const ScriptValue* v = arg(a, 1).asMap().find("locked")) {
                if (!v->isBool()) {
                    fail(fn, "locked must be true or false");
                }
                l.locked = v->asBool();
            }
        }
        for (const char* key : {"colour", "visible", "locked"}) {
            (void)p.has(key);  // marks the keys read above as known
        }
        p.finish();
        if (!(l == *s.doc.findLayer(id))) {
            execute(s, fn, std::make_unique<document::SetLayerPropertiesCommand>(l));
        }
        return ScriptValues{};
    });

    // ---- selection --------------------------------------------------------------------------
    reg("carvetto.selection", "objects", [&s](const ScriptValues&) {
        ScriptValue::Array ids;
        for (const ObjectId id : s.doc.selection().ids()) {
            ids.emplace_back(static_cast<std::int64_t>(id.value));
        }
        return one(ScriptValue(std::move(ids)));
    });
    reg("carvetto.selection", "set", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.selection.set";
        if (arg(a, 0).isArray() && arg(a, 0).asArray().empty()) {
            s.doc.selection().clear();
            return ScriptValues{};
        }
        s.doc.selection().assign(objectList(s, fn, arg(a, 0)));
        return ScriptValues{};
    });

    // ---- objects (read) ---------------------------------------------------------------------
    reg("carvetto.object", "kind", [&s](const ScriptValues& a) {
        const document::Object& o = *s.doc.findObject(objectId(s, "carvetto.object.kind", arg(a, 0)));
        const char* k = "contour";
        switch (o.kind()) {
            case document::ObjectKind::Vector:
                k = std::holds_alternative<document::TextParams>(o.asVector()->params) ? "text" : "contour";
                break;
            case document::ObjectKind::Bitmap: k = "bitmap"; break;
            case document::ObjectKind::Group: k = "group"; break;
            case document::ObjectKind::Dimension: k = "dimension"; break;
        }
        return one(ScriptValue(k));
    });
    reg("carvetto.object", "name", [&s](const ScriptValues& a) {
        return one(ScriptValue(s.doc.findObject(objectId(s, "carvetto.object.name", arg(a, 0)))->name));
    });
    reg("carvetto.object", "layer", [&s](const ScriptValues& a) {
        const document::Object& o = *s.doc.findObject(objectId(s, "carvetto.object.layer", arg(a, 0)));
        return one(ScriptValue(static_cast<std::int64_t>(o.layer.value)));
    });
    reg("carvetto.object", "bounds", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.object.bounds";
        const ObjectId id = objectId(s, fn, arg(a, 0));
        const auto b = s.doc.objectBounds(id);
        if (!b || b->isEmpty()) {
            fail(fn, "the object has no extent");
        }
        ScriptValues r;
        r.emplace_back(b->min.x);
        r.emplace_back(b->min.y);
        r.emplace_back(b->max.x);
        r.emplace_back(b->max.y);
        return r;
    });
    reg("carvetto.object", "contours", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.object.contours";
        const document::Object& o = *s.doc.findObject(objectId(s, fn, arg(a, 0)));
        const document::VectorObject* v = o.asVector();
        if (v == nullptr) {
            fail(fn, "the object is not a vector");
        }
        ScriptValue::Array out;
        out.reserve(v->contours.size());
        for (const Contour& c : v->contours) {
            out.push_back(contourValue(c));
        }
        return one(ScriptValue(std::move(out)));
    });

    // ---- objects (create / modify) -----------------------------------------------------------
    reg("carvetto.object", "create_contour", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.object.create_contour";
        const LayerId layer = writableLayer(s, fn, arg(a, 0));
        if (!arg(a, 1).isArray()) {
            fail(fn, "points must be an array of {x, y}");
        }
        if (!arg(a, 2).isNil() && !arg(a, 2).isBool()) {
            fail(fn, "closed must be true or false");
        }
        const bool closed = arg(a, 2).isBool() && arg(a, 2).asBool();
        countCreated(s, fn);
        Contour c = buildPath(fn, arg(a, 1).asArray(), &arg(a, 3), closed, 1u << 20);
        const ObjectId id = addObject(s, fn, document::Object::vector(std::move(c), layer, s.doc.activeSheet()));
        return one(ScriptValue(static_cast<std::int64_t>(id.value)));
    });
    reg("carvetto.object", "create_shape", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.object.create_shape";
        const LayerId layer = writableLayer(s, fn, arg(a, 0));
        const std::string kind = stringArg(fn, a, 1, "shape kind");
        Params p(fn, arg(a, 2));
        const document::CreationParams params = shapeParams(fn, kind, p);
        p.finish();
        countCreated(s, fn);
        auto built = document::buildContours(params);
        if (!built.ok()) {
            fail(fn, built.error());
        }
        const ObjectId id =
            addObject(s, fn, document::Object::vector(built.value(), layer, s.doc.activeSheet(), params));
        return one(ScriptValue(static_cast<std::int64_t>(id.value)));
    });
    reg("carvetto.object", "move", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.object.move";
        const auto ids = objectList(s, fn, arg(a, 0));
        const double dx = numberArg(fn, a, 1, "dx");
        const double dy = numberArg(fn, a, 2, "dy");
        execute(s, fn,
                std::make_unique<document::TransformObjectsCommand>(ids, geometry::Transform2::translation({dx, dy})));
        return ScriptValues{};
    });
    const auto centreOf = [](Session& se, const std::string& fn, const std::vector<ObjectId>& ids,
                             const ScriptValues& a, std::size_t at) {
        if (!arg(a, at).isNil()) {
            return Vec2{numberArg(fn, a, at, "cx"), numberArg(fn, a, at + 1, "cy")};
        }
        geometry::Box2 box;
        for (const ObjectId id : ids) {
            if (const auto b = se.doc.objectBounds(id)) {
                box.expand(*b);
            }
        }
        return box.centre();
    };
    reg("carvetto.object", "rotate", [&s, centreOf](const ScriptValues& a) {
        const std::string fn = "carvetto.object.rotate";
        const auto ids = objectList(s, fn, arg(a, 0));
        const double angle = numberArg(fn, a, 1, "angle (radians)");
        const Vec2 c = centreOf(s, fn, ids, a, 2);
        execute(s, fn, std::make_unique<document::TransformObjectsCommand>(ids, geometry::Transform2::rotation(angle, c)));
        return ScriptValues{};
    });
    reg("carvetto.object", "scale", [&s, centreOf](const ScriptValues& a) {
        const std::string fn = "carvetto.object.scale";
        const auto ids = objectList(s, fn, arg(a, 0));
        const double sx = numberArg(fn, a, 1, "sx");
        const double sy = numberArgOr(fn, a, 2, "sy", sx);
        if (sx == 0.0 || sy == 0.0 || std::fabs(sx) > 1.0e6 || std::fabs(sy) > 1.0e6) {
            fail(fn, "a scale factor must be non-zero and at most 1e6");
        }
        const Vec2 c = centreOf(s, fn, ids, a, 3);
        execute(s, fn,
                std::make_unique<document::TransformObjectsCommand>(ids, geometry::Transform2::scaling(sx, sy, c)));
        return ScriptValues{};
    });
    reg("carvetto.object", "delete", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.object.delete";
        execute(s, fn, std::make_unique<document::RemoveObjectsCommand>(objectList(s, fn, arg(a, 0))));
        return ScriptValues{};
    });
    reg("carvetto.object", "set_layer", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.object.set_layer";
        const auto ids = objectList(s, fn, arg(a, 0));
        const LayerId layer = writableLayer(s, fn, arg(a, 1));
        execute(s, fn, std::make_unique<document::MoveObjectsToLayerCommand>(ids, layer));
        return ScriptValues{};
    });

    // ---- tools and toolpaths ----------------------------------------------------------------
    const auto toolList = [&s]() {
        return s.sv.tools ? s.sv.tools() : std::vector<GadgetTool>{};
    };
    reg("carvetto.tool", "list", [toolList](const ScriptValues&) {
        ScriptValue::Array out;
        for (const GadgetTool& t : toolList()) {
            ScriptValue::Map m;
            m.items.emplace_back("diameter", ScriptValue(t.diameterMm));
            m.items.emplace_back("id", ScriptValue(t.id));
            m.items.emplace_back("name", ScriptValue(t.name));
            m.items.emplace_back("type", ScriptValue(t.type));
            out.emplace_back(std::move(m));
        }
        return one(ScriptValue(std::move(out)));
    });
    reg("carvetto.tool", "find", [toolList](const ScriptValues& a) {
        const std::string key = stringArg("carvetto.tool.find", a, 0, "name");
        const std::vector<GadgetTool> tools = toolList();
        for (const GadgetTool& t : tools) {
            if (t.name == key) {
                return one(ScriptValue(t.id));
            }
        }
        for (const GadgetTool& t : tools) {
            if (t.id == key) {
                return one(ScriptValue(t.id));
            }
        }
        return one(ScriptValue());
    });
    reg("carvetto.toolpath", "create", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.toolpath.create";
        ToolpathRequest req;
        req.kind = stringArg(fn, a, 0, "kind");
        const auto& kinds = toolpathKinds();
        if (std::find(kinds.begin(), kinds.end(), req.kind) == kinds.end()) {
            fail(fn, "unknown toolpath kind \"" + req.kind + "\" (profile, pocket, drill, engrave)");
        }
        if (arg(a, 1).isNil()) {
            req.objects = s.doc.selection().ids();
            if (req.objects.empty()) {
                fail(fn, "no objects given and nothing is selected");
            }
        } else {
            req.objects = objectList(s, fn, arg(a, 1));
        }
        if (!arg(a, 2).isNil()) {
            if (arg(a, 2).isMap()) {
                req.params = arg(a, 2).asMap();
            } else if (!(arg(a, 2).isArray() && arg(a, 2).asArray().empty())) {
                fail(fn, "params must be a table with string keys");
            }
        }
        if (!s.sv.createToolpath) {
            fail(fn, "toolpaths cannot be created here");
        }
        if (++s.toolpathCalls > s.gl.maxToolpaths) {
            fail(fn, "this gadget created too many toolpaths (limit " + std::to_string(s.gl.maxToolpaths) + ")");
        }
        ToolpathCreated r;
        {
            Session::Pause pause(s);
            r = s.sv.createToolpath(req);
        }
        if (!r.ok) {
            fail(fn, r.message.empty() ? std::string("the toolpath could not be calculated") : r.message);
        }
        s.edits += std::max<std::size_t>(1, r.ids.size());
        ScriptValue::Array ids;
        for (const ToolpathId id : r.ids) {
            s.createdToolpaths.push_back(id);
            ids.emplace_back(static_cast<std::int64_t>(id.value));
        }
        return one(ScriptValue(std::move(ids)));
    });
    reg("carvetto.toolpath", "create_custom", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.toolpath.create_custom";
        const std::string name = stringArg(fn, a, 0, "name");
        checkName(fn, name, "a toolpath name");
        const std::string toolId = stringArg(fn, a, 1, "tool");
        if (!s.sv.toolSnapshot) {
            fail(fn, "toolpaths cannot be created here");
        }
        const auto snap = s.sv.toolSnapshot(toolId);
        if (!snap) {
            fail(fn, "unknown tool \"" + toolId + "\"");
        }
        std::vector<toolpath::Move> moves = parseMoves(fn, arg(a, 2), s.gl.maxCustomMoves);
        if (++s.toolpathCalls > s.gl.maxToolpaths) {
            fail(fn, "this gadget created too many toolpaths (limit " + std::to_string(s.gl.maxToolpaths) + ")");
        }
        // A custom path has no operation of its own in the record format (ADR-0108): it is stored
        // as an engrave-operation record with no vector selection and the computed moves.
        document::ToolpathRecord rec;
        rec.name = name;
        toolpath::EngraveParams params;
        params.name = name;
        rec.params = params;
        rec.selection = document::VectorSelection{};
        rec.tool = *snap;
        rec.toolSource.toolId = toolId;
        rec.moves = document::shareMoves(std::move(moves));
        auto cmd = std::make_unique<document::AddToolpathCommand>(std::move(rec));
        const document::AddToolpathCommand* raw = cmd.get();
        execute(s, fn, std::move(cmd));
        s.createdToolpaths.push_back(raw->id());
        return one(ScriptValue(static_cast<std::int64_t>(raw->id().value)));
    });

    // ---- user interface -----------------------------------------------------------------------
    reg("carvetto.ui", "form", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.ui.form";
        const FormSpec spec = parseForm(s, fn, arg(a, 0));
        if (!s.sv.ui.form) {
            fail(fn, "no user interface available");
        }
        std::optional<ScriptValue::Map> got;
        {
            Session::Pause pause(s);
            got = s.sv.ui.form(spec);
        }
        if (!got) {
            return one(ScriptValue());
        }
        return one(ScriptValue(checkedFormValues(s, fn, spec, std::move(*got))));
    });
    reg("carvetto.ui", "message", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.ui.message";
        const std::string text = stringArg(fn, a, 0, "text");
        if (text.size() > kMaxTextBytes) {
            fail(fn, "text is too long");
        }
        if (!s.sv.ui.message) {
            fail(fn, "no user interface available");
        }
        Session::Pause pause(s);
        s.sv.ui.message(text);
        return ScriptValues{};
    });
    reg("carvetto.ui", "confirm", [&s](const ScriptValues& a) {
        const std::string fn = "carvetto.ui.confirm";
        const std::string text = stringArg(fn, a, 0, "text");
        if (text.size() > kMaxTextBytes) {
            fail(fn, "text is too long");
        }
        if (!s.sv.ui.confirm) {
            fail(fn, "no user interface available");
        }
        Session::Pause pause(s);
        return one(ScriptValue(s.sv.ui.confirm(text)));
    });

    // ---- run -----------------------------------------------------------------------------------
    const std::vector<ObjectId> selectionBefore = doc.selection().ids();
    const std::size_t stepsBefore = undo.count();
    undo.breakCoalescing();
    undo.beginMacro(options.undoText);
    result.script = host.run(source, chunkName, options.cancel);
    if (result.script.ok()) {
        undo.endMacro();
        result.committed = undo.count() > stepsBefore;
    } else {
        undo.abortMacro(doc);
        std::vector<ObjectId> keep;
        for (const ObjectId id : selectionBefore) {
            if (doc.hasObject(id)) {
                keep.push_back(id);
            }
        }
        doc.selection().assign(keep);
        s.createdObjects.clear();
        s.createdToolpaths.clear();
    }
    result.edits = s.edits;
    result.log = std::move(s.log);
    result.createdObjects = std::move(s.createdObjects);
    result.createdToolpaths = std::move(s.createdToolpaths);
    return result;
}

}  // namespace carvetto::scripting
