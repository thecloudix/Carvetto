#include "carvetto/viewmodel/tools.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <memory>
#include <system_error>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/shape_params.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/viewmodel/transform_ops.hpp"
#include "carvetto/viewmodel/units_format.hpp"

namespace carvetto::viewmodel {

using document::CreationParams;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMaxMagnitude = 1e9;
constexpr int kMaxCount = 100000;

double toRadians(double degrees) {
    return degrees * kPi / 180.0;
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) {
        s.remove_prefix(1);
    }
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) {
        s.remove_suffix(1);
    }
    return s;
}

bool endsWith(std::string_view s, std::string_view suffix) {
    return s.size() >= suffix.size() && s.substr(s.size() - suffix.size()) == suffix;
}

NumericField field(const char* id, const char* label, FieldKind kind, bool allowNegative = false,
                   bool allowZero = false) {
    NumericField f;
    f.id = id;
    f.label = label;
    f.kind = kind;
    f.allowNegative = allowNegative;
    f.allowZero = allowZero;
    return f;
}

Vec2 constrainTo45(Vec2 from, Vec2 to) {
    const Vec2 v = to - from;
    const double len = v.length();
    if (!(len > 0.0)) {
        return to;
    }
    const double step = kPi / 4.0;
    const double k = std::round(std::atan2(v.y, v.x) / step);
    const Vec2 dir = Vec2::fromAngle(k * step);
    const double along = v.dot(dir);
    return from + dir * along;
}

}  // namespace

const char* toolName(ToolKind tool) {
    switch (tool) {
        case ToolKind::Select: return "Select";
        case ToolKind::Rectangle: return "Rectangle";
        case ToolKind::Circle: return "Circle";
        case ToolKind::Ellipse: return "Ellipse";
        case ToolKind::Polygon: return "Polygon";
        case ToolKind::Star: return "Star";
        case ToolKind::Arc: return "Arc";
        case ToolKind::Polyline: return "Polyline";
        case ToolKind::NodeEdit: return "Node Edit";
    }
    return "";
}

ParsedNumber parseNumber(std::string_view text, FieldKind kind, document::Units units) {
    ParsedNumber out;
    std::string_view t = trim(text);
    if (t.empty()) {
        out.error = "enter a number";
        return out;
    }
    document::Units entered = units;
    if (kind == FieldKind::Length) {
        if (endsWith(t, "mm")) {
            entered = document::Units::Millimetres;
            t = trim(t.substr(0, t.size() - 2));
        } else if (endsWith(t, "in")) {
            entered = document::Units::Inches;
            t = trim(t.substr(0, t.size() - 2));
        }
    } else if (kind == FieldKind::Angle) {
        if (endsWith(t, "deg")) {
            t = trim(t.substr(0, t.size() - 3));
        } else if (endsWith(t, "\xC2\xB0")) {
            t = trim(t.substr(0, t.size() - 2));
        }
    } else if (kind == FieldKind::Percent) {
        if (endsWith(t, "%")) {
            t = trim(t.substr(0, t.size() - 1));
        }
    }
    if (t.empty()) {
        out.error = "enter a number";
        return out;
    }
    if (t.front() == '+') {
        t.remove_prefix(1);
    }
    // Only plain decimal / exponent notation: reject "inf", "nan", hex.
    const char first = t.front();
    const bool numericStart = (first >= '0' && first <= '9') || first == '.' || first == '-';
    if (!numericStart || t.size() > 64) {
        out.error = "not a number";
        return out;
    }
    double v = 0.0;
    const char* begin = t.data();
    const char* end = t.data() + t.size();
    const auto res = std::from_chars(begin, end, v);
    if (res.ec != std::errc() || res.ptr != end || !std::isfinite(v)) {
        out.error = "not a number";
        return out;
    }
    if (kind == FieldKind::Count) {
        if (v != std::floor(v) || std::fabs(v) > kMaxCount * 10.0) {
            out.error = "enter a whole number";
            return out;
        }
    }
    if (kind == FieldKind::Length) {
        v = document::fromDisplay(v, entered);
    }
    if (std::fabs(v) > kMaxMagnitude) {
        out.error = "value is too large";
        return out;
    }
    out.ok = true;
    out.value = v;
    return out;
}

// ---------------------------------------------------------------------------------------------

ToolController::ToolController(document::Document* doc) : doc_(doc), nodeEditor_(doc) {
    snapSettings_.setEnabled(SnapKind::Grid, false);
}

void ToolController::setDocument(document::Document* doc) {
    // The old document may already be gone (the file session frees it before the window rebinds the
    // tools): drop every drag state first, without touching it, then cancel what is left.
    gesture_ = TransformGesture{};
    nodeEditor_.setDocument(nullptr);
    xfReset();
    cancel();
    doc_ = doc;
    nodeEditor_.setDocument(doc);
    snapIndex_.reset();
    snap_.reset();
}

void ToolController::setView(const ViewTransform& view) {
    view_ = view;
    nodeEditor_.setView(view);
}

void ToolController::setTool(ToolKind tool) {
    cancel();
    tool_ = tool;
}

bool ToolController::isTwoPointTool() const {
    return tool_ == ToolKind::Rectangle || tool_ == ToolKind::Circle || tool_ == ToolKind::Ellipse ||
           tool_ == ToolKind::Polygon || tool_ == ToolKind::Star;
}

bool ToolController::active() const {
    return !pts_.empty() || pressed_ || selPressed_ || nodeEditor_.pressed();
}

void ToolController::cancel() {
    pts_.clear();
    pressed_ = false;
    typed_.clear();
    selPressed_ = false;
    selDragging_ = false;
    nodeEditor_.escape();
    if (gesture_.active() && doc_ != nullptr) {
        gesture_.cancel(*doc_);
    }
    xfReset();
}

ToolResult ToolController::refuse(std::string message) const {
    ToolResult r;
    r.message = std::move(message);
    return r;
}

bool ToolController::wantsKeys() const {
    if (tool_ == ToolKind::NodeEdit) {
        return nodeEditor_.pressed() || nodeEditor_.gestureActive();
    }
    return gesture_.active() || (tool_ != ToolKind::Select && (active() || !typed_.empty()));
}

bool ToolController::layerWritable(std::string* why) const {
    const document::Layer* layer = doc_ != nullptr ? doc_->findLayer(doc_->activeLayer()) : nullptr;
    if (layer == nullptr) {
        *why = "there is no active layer";
        return false;
    }
    if (layer->locked) {
        *why = "the active layer is locked";
        return false;
    }
    if (!layer->visible) {
        *why = "the active layer is hidden";
        return false;
    }
    return true;
}

const SnapIndex& ToolController::snapIndex() {
    // The undo stack's generation is process-wide unique per edit state, so it cannot repeat for
    // different content (undo/redo/new command/other document).
    const std::array<std::uint64_t, 2> sig{doc_->undoStack().generation(), doc_->activeSheet().value};
    if (!snapIndex_.has_value() || sig != snapSignature_) {
        snapIndex_.emplace(*doc_);
        ++snapIndexBuilds_;
        snapSignature_ = sig;
    }
    return *snapIndex_;
}

void ToolController::updatePointer(Vec2 px, Modifiers mods) {
    cursorPx_ = px;
    mods_ = mods;
    hasCursor_ = true;
    if (tool_ == ToolKind::NodeEdit) {
        nodeEditor_.validateGesture();
        if (!nodeEditor_.gestureActive()) {
            gestureSnapIndex_.reset();  // a drag ended behind our back: its index is stale
        }
        nodeIndexWasValid_ = gestureSnapIndex_.has_value() && gestureIndexValid();
    }
    snap_.reset();
    Vec2 raw = view_.screenToWorld(px);
    if (doc_ == nullptr || (tool_ == ToolKind::Select && xfPress_ == XfPress::None)) {
        cursorMm_ = raw;
        return;
    }
    if (mods.shift) {  // Shift: no snapping; the polyline additionally constrains to 45 degrees.
        if (tool_ == ToolKind::Polyline && !pts_.empty()) {
            raw = constrainTo45(pts_.back(), raw);
        }
        cursorMm_ = raw;
        return;
    }
    SnapQuery query;
    if (!pts_.empty()) {
        query.pendingStartMm = pts_.back();
    }
    if (tool_ == ToolKind::Select) {
        query.exclude = gesture_.targets();  // the objects being dragged never snap to themselves
    }
    // During a transform drag the index of the other objects is built once (the dragged objects are
    // left out, and nothing else changes), not once per pointer step.
    // One rule for both gestures (ADR-0029 "Gesture generation rule", ADR-0030): the frozen index is
    // valid only for the gesture's own pushes; its key is {undo generation, active sheet}.
    const bool frozen = gestureSnapIndex_.has_value() &&
                        (gesture_.active() || (tool_ == ToolKind::NodeEdit && nodeEditor_.gestureActive())) &&
                        gestureIndexValid();
    snap_ = (frozen ? *gestureSnapIndex_ : snapIndex()).best(view_, px, snapSettings_, query);
    cursorMm_ = snap_.has_value() ? snap_->pointMm : raw;
}

std::optional<double> ToolController::typed(const std::string& id) const {
    const auto it = typed_.find(id);
    if (it == typed_.end()) {
        return std::nullopt;
    }
    return it->second;
}

Vec2 ToolController::anchorFor() const {
    Vec2 a = pts_.empty() ? cursorMm_ : pts_.front();
    if (const auto x = typed("x")) {
        a.x = *x;
    }
    if (const auto y = typed("y")) {
        a.y = *y;
    }
    return a;
}

// --- Drafts ----------------------------------------------------------------------------------

void ToolController::finishParams(Draft& d, const CreationParams& params) {
    d.params = params;
    auto built = document::buildContours(params);
    if (!built.ok()) {
        d.valid = false;
        d.message = built.error();
        return;
    }
    d.contours = std::move(built.value());
    d.valid = true;
}

ToolController::Draft ToolController::rectangleDraft(Vec2 a) const {
    Draft d;
    const Vec2 c = cursorMm_;
    const double dx = c.x - a.x;
    const double dy = c.y - a.y;
    const double sx = dx < 0.0 ? -1.0 : 1.0;
    const double sy = dy < 0.0 ? -1.0 : 1.0;
    const double adx = std::fabs(dx);
    const double ady = std::fabs(dy);
    const auto tw = typed("width");
    const auto th = typed("height");
    double w = 0.0;
    double h = 0.0;
    if (mods_.ctrl) {  // square
        const double m = std::max(adx, ady);
        const double base = mods_.alt ? 2.0 * m : m;
        w = tw ? *tw : (th ? *th : base);
        h = th ? *th : (tw ? *tw : base);
    } else {
        w = tw ? *tw : (mods_.alt ? 2.0 * adx : adx);
        h = th ? *th : (mods_.alt ? 2.0 * ady : ady);
    }
    d.values["x"] = a.x;
    d.values["y"] = a.y;
    d.values["width"] = w;
    d.values["height"] = h;
    const bool square = options_.cornerStyle == document::RectangleParams::CornerStyle::Square;
    double corner = 0.0;
    if (!square) {
        corner = typed("corner") ? *typed("corner")
                                 : std::clamp(options_.cornerSizeMm, 0.0, 0.5 * std::min(w, h));
    }
    d.values["corner"] = corner;
    if (!(w > 0.0) || !(h > 0.0)) {
        return d;
    }
    document::RectangleParams p;
    p.centreMm = mods_.alt ? a : Vec2{a.x + sx * w * 0.5, a.y + sy * h * 0.5};
    p.widthMm = w;
    p.heightMm = h;
    p.corners = options_.cornerStyle;
    p.cornerRadiusMm = corner;
    finishParams(d, p);
    return d;
}

ToolController::Draft ToolController::circleDraft(Vec2 a) const {
    Draft d;
    const Vec2 c = cursorMm_;
    const auto tr = typed("radius");
    Vec2 centre = a;
    double r = geometry::distance(a, c);
    if (mods_.alt) {  // the pointer marks the far end of the diameter
        centre = {(a.x + c.x) * 0.5, (a.y + c.y) * 0.5};
        r *= 0.5;
    }
    if (tr) {
        centre = a;
        r = *tr;
    }
    d.values["x"] = centre.x;
    d.values["y"] = centre.y;
    d.values["radius"] = r;
    if (!(r > 0.0)) {
        return d;
    }
    finishParams(d, document::CircleParams{centre, r});
    return d;
}

ToolController::Draft ToolController::ellipseDraft(Vec2 a) const {
    Draft d;
    const Vec2 c = cursorMm_;
    const double dx = c.x - a.x;
    const double dy = c.y - a.y;
    const double sx = dx < 0.0 ? -1.0 : 1.0;
    const double sy = dy < 0.0 ? -1.0 : 1.0;
    const double adx = std::fabs(dx);
    const double ady = std::fabs(dy);
    const auto tw = typed("width");
    const auto th = typed("height");
    double w = 0.0;
    double h = 0.0;
    if (mods_.ctrl) {  // circle
        const double m = std::max(adx, ady);
        const double base = mods_.alt ? 2.0 * m : m;
        w = tw ? *tw : (th ? *th : base);
        h = th ? *th : (tw ? *tw : base);
    } else {
        w = tw ? *tw : (mods_.alt ? 2.0 * adx : adx);
        h = th ? *th : (mods_.alt ? 2.0 * ady : ady);
    }
    d.values["x"] = a.x;
    d.values["y"] = a.y;
    d.values["width"] = w;
    d.values["height"] = h;
    if (!(w > 0.0) || !(h > 0.0)) {
        return d;
    }
    document::EllipseParams p;
    p.centreMm = mods_.alt ? a : Vec2{a.x + sx * w * 0.5, a.y + sy * h * 0.5};
    p.widthMm = w;
    p.heightMm = h;
    p.angle = 0.0;
    finishParams(d, p);
    return d;
}

ToolController::Draft ToolController::polygonDraft(Vec2 a) const {
    Draft d;
    const Vec2 v = cursorMm_ - a;
    const double r = typed("radius") ? *typed("radius") : v.length();
    const double angleDeg =
        typed("angle") ? *typed("angle") : (v.length() > 0.0 ? std::atan2(v.y, v.x) * 180.0 / kPi : 0.0);
    const double sides = typed("sides") ? *typed("sides") : static_cast<double>(options_.polygonSides);
    d.values["x"] = a.x;
    d.values["y"] = a.y;
    d.values["sides"] = sides;
    d.values["radius"] = r;
    d.values["angle"] = angleDeg;
    if (!(r > 0.0)) {
        return d;
    }
    document::PolygonParams p;
    p.centreMm = a;
    p.sides = static_cast<int>(sides);
    p.radiusMm = r;
    p.angle = toRadians(angleDeg);
    finishParams(d, p);
    return d;
}

ToolController::Draft ToolController::starDraft(Vec2 a) const {
    Draft d;
    const Vec2 v = cursorMm_ - a;
    const double r = typed("radius") ? *typed("radius") : v.length();
    const double angleDeg =
        typed("angle") ? *typed("angle") : (v.length() > 0.0 ? std::atan2(v.y, v.x) * 180.0 / kPi : 0.0);
    const double points = typed("points") ? *typed("points") : static_cast<double>(options_.starPoints);
    const double inner = typed("inner") ? *typed("inner") : options_.starInnerPercent;
    d.values["x"] = a.x;
    d.values["y"] = a.y;
    d.values["points"] = points;
    d.values["radius"] = r;
    d.values["inner"] = inner;
    d.values["angle"] = angleDeg;
    if (!(r > 0.0)) {
        return d;
    }
    document::StarParams p;
    p.centreMm = a;
    p.points = static_cast<int>(points);
    p.outerRadiusMm = r;
    p.innerRadiusMm = r * inner / 100.0;
    p.angle = toRadians(angleDeg);
    finishParams(d, p);
    return d;
}

ToolController::Draft ToolController::arcDraft() const {
    Draft d;
    const Vec2 c = cursorMm_;
    if (pts_.empty()) {
        return d;
    }
    if (pts_.size() == 1) {  // rubber band to the cursor
        if (geometry::distance(pts_[0], c) > 0.0) {
            Contour band;
            band.append(LineSpan{pts_[0], c});
            d.contours.push_back(std::move(band));
        }
        return d;
    }
    const Vec2 p0 = pts_[0];
    const Vec2 p1 = pts_[1];
    geometry::ShapeResult<Contour> arc{std::string("arc: incomplete")};
    switch (options_.arcMode) {
        case ArcMode::ThreePoint:
            arc = geometry::buildArcThreePoint(p0, p1, c);
            break;
        case ArcMode::CentreStartEnd:
            arc = geometry::buildArcCentreStartEnd(p0, p1, c, options_.arcDirection);
            break;
        case ArcMode::StartEndRadius: {
            const Vec2 chord = p1 - p0;
            const double len = chord.length();
            const double side = chord.cross(c - p0);  // > 0: the pointer is left of start->end
            const geometry::ArcDirection dir =
                side > 0.0 ? geometry::ArcDirection::CW : geometry::ArcDirection::CCW;
            double radius = 0.0;
            if (const auto tr = typed("radius")) {
                radius = *tr;
            } else {
                // Radius of the circle through start, end and the pointer, never below half the chord.
                const double area2 = std::fabs(side);
                const double a2 = (p1 - c).length();
                const double b2 = (c - p0).length();
                radius = area2 > 0.0 ? (len * a2 * b2) / (2.0 * area2) : 0.0;
                radius = std::max(radius, 0.5 * len);
            }
            d.values["radius"] = radius;
            arc = geometry::buildArcStartEndRadius(p0, p1, radius, dir, options_.arcLarge);
            break;
        }
        case ArcMode::StartEndHeight: {
            const Vec2 chord = p1 - p0;
            const double len = chord.length();
            double height = 0.0;
            if (const auto th = typed("height")) {
                height = *th;
            } else if (len > 0.0) {
                height = chord.cross(c - p0) / len;  // signed distance, positive to the left
            }
            d.values["height"] = height;
            arc = geometry::buildArcStartEndHeight(p0, p1, height);
            break;
        }
    }
    if (arc.ok()) {
        d.valid = true;
        d.params = document::NoParams{};
        d.contours.push_back(arc.value());
    } else {
        d.message = arc.error();
        if (geometry::distance(p0, p1) > 0.0) {  // keep a chord on screen while the arc is invalid
            Contour band;
            band.append(LineSpan{p0, p1});
            d.contours.push_back(std::move(band));
        }
    }
    return d;
}

ToolController::Draft ToolController::polylineDraft(bool includeCursor, bool closeIt) const {
    Draft d;
    std::vector<Vec2> pts = pts_;
    const double eps = snapSettings_.tolerance.distance;
    if (includeCursor && !pts.empty() && geometry::distance(pts.back(), cursorMm_) > eps) {
        pts.push_back(cursorMm_);
    }
    if (pts.size() < 2) {
        return d;
    }
    Contour c;
    for (std::size_t i = 1; i < pts.size(); ++i) {
        c.append(LineSpan{pts[i - 1], pts[i]});
    }
    if (closeIt) {
        if (pts.size() < 3) {
            d.message = "a closed polyline needs at least three points";
            return d;
        }
        c.append(LineSpan{pts.back(), pts.front()});
        c.closed = true;
    }
    d.contours.push_back(std::move(c));
    d.params = document::NoParams{};
    d.valid = true;
    return d;
}

ToolController::Draft ToolController::makeDraft() const {
    switch (tool_) {
        case ToolKind::Rectangle:
        case ToolKind::Circle:
        case ToolKind::Ellipse:
        case ToolKind::Polygon:
        case ToolKind::Star: {
            if (pts_.empty() && typed_.empty()) {
                return {};
            }
            const Vec2 a = anchorFor();
            if (tool_ == ToolKind::Rectangle) {
                return rectangleDraft(a);
            }
            if (tool_ == ToolKind::Circle) {
                return circleDraft(a);
            }
            if (tool_ == ToolKind::Ellipse) {
                return ellipseDraft(a);
            }
            return tool_ == ToolKind::Polygon ? polygonDraft(a) : starDraft(a);
        }
        case ToolKind::Arc:
            return arcDraft();
        case ToolKind::Polyline:
            return polylineDraft(hasCursor_, false);
        case ToolKind::Select:
        case ToolKind::NodeEdit:
            break;
    }
    return {};
}

ToolPreview ToolController::preview() const {
    ToolPreview out;
    if (tool_ == ToolKind::NodeEdit) {
        out.marqueePx = nodeEditor_.marqueePx();
        return out;
    }
    if (tool_ == ToolKind::Select) {
        if (selDragging_) {
            out.marqueePx = geometry::Box2(selStartPx_, selCurPx_);
        }
        return out;
    }
    Draft d = makeDraft();
    out.contours = std::move(d.contours);
    out.valid = d.valid;
    out.message = d.message;
    out.anchorsMm = pts_;
    return out;
}

// --- Commit ----------------------------------------------------------------------------------

ToolResult ToolController::commitDraft(const Draft& draft) {
    if (doc_ == nullptr) {
        return refuse("no document");
    }
    if (!draft.valid || !draft.params.has_value() || draft.contours.empty()) {
        return refuse(draft.message.empty() ? "the shape is not complete" : draft.message);
    }
    std::string why;
    if (!layerWritable(&why)) {
        return refuse(why);
    }
    for (const Contour& c : draft.contours) {
        const geometry::Box2 b = c.bounds();
        if (!(std::isfinite(b.min.x) && std::isfinite(b.min.y) && std::isfinite(b.max.x) &&
              std::isfinite(b.max.y) && std::fabs(b.min.x) <= kMaxMagnitude && std::fabs(b.min.y) <= kMaxMagnitude &&
              std::fabs(b.max.x) <= kMaxMagnitude && std::fabs(b.max.y) <= kMaxMagnitude)) {
            return refuse("coordinates must be finite and within 1e9 mm");
        }
    }
    document::Object obj = document::Object::vector(draft.contours, doc_->activeLayer(), doc_->activeSheet(),
                                                    *draft.params);
    if (!doc_->execute(std::make_unique<document::AddObjectCommand>(std::move(obj)))) {
        return refuse("the object could not be added");
    }
    pts_.clear();
    pressed_ = false;
    typed_.clear();
    ToolResult r;
    r.committed = true;
    return r;
}

ToolResult ToolController::finishPolyline(bool closeIt) {
    if (pts_.size() < 2) {
        return refuse("a polyline needs at least two points");
    }
    return commitDraft(polylineDraft(false, closeIt));
}

// --- Events ----------------------------------------------------------------------------------

namespace {
ToolResult fromNodeEdit(const NodeEditResult& r) {
    ToolResult t;
    t.committed = r.changed;
    t.selectionChanged = r.selectionChanged;
    t.message = r.message;
    return t;
}
}  // namespace

ToolResult ToolController::afterNodeEdit(const NodeEditResult& r) {
    if (!nodeEditor_.gestureActive()) {
        gestureSnapIndex_.reset();
    } else if (nodeIndexWasValid_ && gestureSnapIndex_.has_value()) {
        gestureIndexKey_[0] = nodeEditor_.expectedGeneration();  // the gesture's own push
    } else if (doc_ != nullptr) {
        // The drag began in this event (or the index was not valid): index everything but the edited
        // objects, from the document as it is now.
        gestureSnapIndex_.emplace(*doc_, false, nodeEditor_.gestureObjects());
        ++snapIndexBuilds_;
        gestureIndexKey_ = {nodeEditor_.expectedGeneration(), doc_->activeSheet().value};
    }
    return fromNodeEdit(r);
}

ToolResult ToolController::pointerMove(Vec2 px, Modifiers mods) {
    updatePointer(px, mods);
    if (tool_ == ToolKind::NodeEdit) {
        const bool wasActive = nodeEditor_.gestureActive();
        ToolResult r = afterNodeEdit(nodeEditor_.move(px, cursorMm_, mods.shift, mods.ctrl, mods.alt));
        if (!wasActive && nodeEditor_.gestureActive() && snap_.has_value() && gestureSnapIndex_.has_value()) {
            // The drag began in this event, so this first position was snapped against the full
            // index, the edited objects included. Snap it again without them and replace the step.
            snap_ = gestureSnapIndex_->best(view_, px, snapSettings_, SnapQuery{});
            cursorMm_ = snap_.has_value() ? snap_->pointMm : view_.screenToWorld(px);
            nodeIndexWasValid_ = true;
            r = afterNodeEdit(nodeEditor_.move(px, cursorMm_, mods.shift, mods.ctrl, mods.alt));
        }
        return r;
    }
    if (tool_ == ToolKind::Select && selPressed_) {
        selCurPx_ = px;
        if (!selDragging_ && geometry::distance(px, selStartPx_) >= kDragThresholdPx) {
            selDragging_ = true;
        }
    }
    if (tool_ == ToolKind::Select && xfPress_ != XfPress::None) {
        return xfMove(px);
    }
    return {};
}

ToolResult ToolController::selectPress(Vec2 px, Modifiers mods) {
    if (doc_ != nullptr) {
        // Shift/Ctrl on a handle are transform modifiers; on the object body they extend the selection.
        xfPressBegin(px, mods.shift || mods.ctrl);
        if (xfPress_ != XfPress::None) {
            return {};
        }
    }
    selPressed_ = true;
    selDragging_ = false;
    selStartPx_ = px;
    selCurPx_ = px;
    selMode_ = mods.ctrl ? SelectMode::Toggle : (mods.shift ? SelectMode::Add : SelectMode::Replace);
    return {};
}

ToolResult ToolController::selectRelease(Vec2 px, Modifiers mods) {
    if (xfPress_ != XfPress::None) {
        return xfRelease();
    }
    if (!selPressed_) {
        return {};
    }
    (void)mods;
    const bool dragged = selDragging_ || geometry::distance(px, selStartPx_) >= kDragThresholdPx;
    selPressed_ = false;
    selDragging_ = false;
    ToolResult r;
    if (doc_ == nullptr) {
        return r;
    }
    if (dragged) {
        const MarqueeMode mm = px.x >= selStartPx_.x ? MarqueeMode::Window : MarqueeMode::Crossing;
        marqueeApply(*doc_, view_, geometry::Box2(selStartPx_, px), mm, selMode_);
    } else {
        clickSelect(*doc_, view_, px, selMode_);
    }
    r.selectionChanged = true;
    pivot_.reset();
    return r;
}

ToolResult ToolController::shapePress(Modifiers mods) {
    (void)mods;
    if (doc_ == nullptr) {
        return refuse("no document");
    }
    if (isTwoPointTool()) {
        if (pts_.empty()) {
            std::string why;
            if (!layerWritable(&why)) {
                return refuse(why);
            }
            pts_.push_back(cursorMm_);
            pressed_ = true;
            pressPx_ = cursorPx_;
            return {};
        }
        return commitDraft(makeDraft());
    }
    if (tool_ == ToolKind::Arc) {
        if (pts_.size() < 2) {
            if (pts_.empty()) {
                std::string why;
                if (!layerWritable(&why)) {
                    return refuse(why);
                }
            }
            pts_.push_back(cursorMm_);
            return {};
        }
        return commitDraft(arcDraft());
    }
    if (tool_ == ToolKind::Polyline) {
        if (pts_.empty()) {
            std::string why;
            if (!layerWritable(&why)) {
                return refuse(why);
            }
        }
        if (!std::isfinite(cursorMm_.x) || !std::isfinite(cursorMm_.y) || std::fabs(cursorMm_.x) > kMaxMagnitude ||
            std::fabs(cursorMm_.y) > kMaxMagnitude) {
            return refuse("coordinates must be finite and within 1e9 mm");
        }
        if (pts_.empty() || geometry::distance(pts_.back(), cursorMm_) > snapSettings_.tolerance.distance) {
            pts_.push_back(cursorMm_);
        }
        return {};
    }
    return {};
}

ToolResult ToolController::shapeRelease(Vec2 px) {
    if (!pressed_) {
        return {};
    }
    pressed_ = false;
    if (geometry::distance(px, pressPx_) >= kDragThresholdPx) {
        return commitDraft(makeDraft());
    }
    return {};
}

ToolResult ToolController::pointerPress(Vec2 px, Modifiers mods) {
    updatePointer(px, mods);
    if (tool_ == ToolKind::NodeEdit) {
        return afterNodeEdit(nodeEditor_.press(px, cursorMm_, mods.shift, mods.ctrl, mods.alt));
    }
    if (tool_ == ToolKind::Select) {
        return selectPress(px, mods);
    }
    return shapePress(mods);
}

ToolResult ToolController::pointerRelease(Vec2 px, Modifiers mods) {
    updatePointer(px, mods);
    if (tool_ == ToolKind::NodeEdit) {
        return afterNodeEdit(nodeEditor_.release(px, cursorMm_, mods.shift, mods.ctrl, mods.alt));
    }
    if (tool_ == ToolKind::Select) {
        return selectRelease(px, mods);
    }
    return shapeRelease(px);
}

ToolResult ToolController::doubleClick(Vec2 px, Modifiers mods) {
    if (tool_ == ToolKind::Polyline) {
        ToolResult r = pointerPress(px, mods);
        if (!r.message.empty()) {
            return r;
        }
        return finishPolyline(false);
    }
    return pointerPress(px, mods);
}

ToolResult ToolController::key(ToolKey k) {
    switch (k) {
        case ToolKey::Escape: {
            const bool wasTransforming = gesture_.active();
            if (tool_ == ToolKind::NodeEdit) {
                NodeEditResult r;
                nodeEditor_.escape(&r);
                cancel();
                return fromNodeEdit(r);
            }
            cancel();
            ToolResult r;
            r.transformed = wasTransforming;
            return r;
        }
        case ToolKey::Enter:
            return commitNumeric();
        case ToolKey::Backspace:
            if (!pts_.empty() && (tool_ == ToolKind::Polyline || tool_ == ToolKind::Arc)) {
                pts_.pop_back();
            }
            return {};
        case ToolKey::Close:
            if (tool_ == ToolKind::Polyline) {
                if (pts_.size() < 3) {
                    return refuse("a closed polyline needs at least three points");
                }
                return finishPolyline(true);
            }
            return {};
    }
    return {};
}

// --- Numeric entry ---------------------------------------------------------------------------

std::vector<NumericField> ToolController::fields() const {
    std::vector<NumericField> list;
    switch (tool_) {
        case ToolKind::Select:
        case ToolKind::NodeEdit:
            return list;
        case ToolKind::Rectangle:
            list = {field("x", "X", FieldKind::Length, true, true), field("y", "Y", FieldKind::Length, true, true),
                    field("width", "Width", FieldKind::Length), field("height", "Height", FieldKind::Length),
                    field("corner", "Corner", FieldKind::Length, false, true)};
            break;
        case ToolKind::Circle:
            list = {field("x", "X", FieldKind::Length, true, true), field("y", "Y", FieldKind::Length, true, true),
                    field("radius", "Radius", FieldKind::Length)};
            break;
        case ToolKind::Ellipse:
            list = {field("x", "X", FieldKind::Length, true, true), field("y", "Y", FieldKind::Length, true, true),
                    field("width", "Width", FieldKind::Length), field("height", "Height", FieldKind::Length)};
            break;
        case ToolKind::Polygon:
            list = {field("x", "X", FieldKind::Length, true, true), field("y", "Y", FieldKind::Length, true, true),
                    field("sides", "Sides", FieldKind::Count), field("radius", "Radius", FieldKind::Length),
                    field("angle", "Angle", FieldKind::Angle, true, true)};
            break;
        case ToolKind::Star:
            list = {field("x", "X", FieldKind::Length, true, true), field("y", "Y", FieldKind::Length, true, true),
                    field("points", "Points", FieldKind::Count), field("radius", "Outer radius", FieldKind::Length),
                    field("inner", "Inner radius %", FieldKind::Percent),
                    field("angle", "Angle", FieldKind::Angle, true, true)};
            break;
        case ToolKind::Arc:
            if (options_.arcMode == ArcMode::StartEndRadius) {
                list = {field("radius", "Radius", FieldKind::Length)};
            } else if (options_.arcMode == ArcMode::StartEndHeight) {
                list = {field("height", "Height", FieldKind::Length, true, false)};
            }
            break;
        case ToolKind::Polyline:
            list = {field("x", "X", FieldKind::Length, true, true), field("y", "Y", FieldKind::Length, true, true),
                    field("length", "Length", FieldKind::Length), field("angle", "Angle", FieldKind::Angle, true, true)};
            break;
    }
    const Draft d = makeDraft();
    const document::Units units = doc_ != nullptr ? doc_->job().displayUnits : document::Units::Millimetres;
    for (NumericField& f : list) {
        f.typed = typed_.contains(f.id);
        const auto it = d.values.find(f.id);
        if (it == d.values.end()) {
            continue;
        }
        switch (f.kind) {
            case FieldKind::Length: f.text = formatLength(it->second, units); break;
            case FieldKind::Angle:
            case FieldKind::Percent: f.text = formatFixed(it->second, 2); break;
            case FieldKind::Count: f.text = formatFixed(it->second, 0); break;
        }
    }
    return list;
}

FieldResult ToolController::setField(const std::string& id, const std::string& text) {
    FieldResult res;
    const std::vector<NumericField> list = fields();
    const auto it = std::find_if(list.begin(), list.end(), [&](const NumericField& f) { return f.id == id; });
    if (it == list.end()) {
        res.ok = false;
        res.message = "unknown field";
        return res;
    }
    if (trim(text).empty()) {
        typed_.erase(id);
        return res;
    }
    const document::Units units = doc_ != nullptr ? doc_->job().displayUnits : document::Units::Millimetres;
    const ParsedNumber n = parseNumber(text, it->kind, units);
    if (!n.ok) {
        res.ok = false;
        res.message = n.error;
        return res;
    }
    if (!it->allowNegative && n.value < 0.0) {
        res.ok = false;
        res.message = "must not be negative";
        return res;
    }
    if (!it->allowZero && n.value == 0.0) {
        res.ok = false;
        res.message = "must not be zero";
        return res;
    }
    if (it->kind == FieldKind::Count && (n.value < 3.0 || n.value > kMaxCount)) {
        res.ok = false;
        res.message = "must be between 3 and 100000";
        return res;
    }
    if (it->kind == FieldKind::Percent && !(n.value > 0.0 && n.value < 100.0)) {
        res.ok = false;
        res.message = "must be between 0 and 100 percent";
        return res;
    }
    typed_[id] = n.value;
    return res;
}

ToolResult ToolController::commitNumeric() {
    switch (tool_) {
        case ToolKind::Select:
        case ToolKind::NodeEdit:
            return {};
        case ToolKind::Rectangle:
        case ToolKind::Circle:
        case ToolKind::Ellipse:
        case ToolKind::Polygon:
        case ToolKind::Star:
            if (pts_.empty() && typed_.empty()) {
                return refuse("nothing to create yet");
            }
            return commitDraft(makeDraft());
        case ToolKind::Arc:
            if (pts_.size() < 2) {
                return refuse("place the first two points first");
            }
            return commitDraft(arcDraft());
        case ToolKind::Polyline: {
            const auto tx = typed("x");
            const auto ty = typed("y");
            const auto tl = typed("length");
            if (!tx && !ty && !tl) {
                return finishPolyline(false);
            }
            Vec2 p{};
            if (tx || ty) {
                if (!tx || !ty) {
                    return refuse("enter both X and Y");
                }
                p = {*tx, *ty};
            } else {
                if (pts_.empty()) {
                    return refuse("place the first point before entering a length");
                }
                const Vec2 v = cursorMm_ - pts_.back();
                const double angle = typed("angle") ? toRadians(*typed("angle")) : std::atan2(v.y, v.x);
                p = pts_.back() + Vec2::fromAngle(angle, *tl);
            }
            if (pts_.empty()) {
                std::string why;
                if (!layerWritable(&why)) {
                    return refuse(why);
                }
            }
            typed_.clear();
            if (pts_.empty() || geometry::distance(pts_.back(), p) > snapSettings_.tolerance.distance) {
                pts_.push_back(p);
            }
            return {};
        }
    }
    return {};
}

// --- Selection box and transform gestures --------------------------------------------------------

namespace {

bool isCornerHandle(HandleKind h) {
    return h == HandleKind::TopLeft || h == HandleKind::TopRight || h == HandleKind::BottomLeft ||
           h == HandleKind::BottomRight;
}

constexpr double kHandleHitPx = 7.0;

CursorHint scaleHint(HandleKind h) {
    switch (h) {
        case HandleKind::Left:
        case HandleKind::Right:
            return CursorHint::ScaleHorizontal;
        case HandleKind::Top:
        case HandleKind::Bottom:
            return CursorHint::ScaleVertical;
        case HandleKind::TopLeft:
        case HandleKind::BottomRight:
            return CursorHint::ScaleDiagonalDown;
        case HandleKind::TopRight:
        case HandleKind::BottomLeft:
            return CursorHint::ScaleDiagonalUp;
        case HandleKind::Centre:
            break;
    }
    return CursorHint::Move;
}

}  // namespace

std::optional<SelectionBox> ToolController::selectionHandles() const {
    if (tool_ != ToolKind::Select || doc_ == nullptr) {
        return std::nullopt;
    }
    const std::optional<geometry::Box2> bounds = transformBounds(*doc_);
    if (!bounds.has_value()) {
        return std::nullopt;
    }
    return selectionBox(*bounds, view_);
}

std::optional<Vec2> ToolController::pivotMm() const {
    if (doc_ == nullptr) {
        return std::nullopt;
    }
    const std::optional<geometry::Box2> bounds = transformBounds(*doc_);
    if (!bounds.has_value()) {
        return std::nullopt;
    }
    return pivot_.value_or(bounds->centre());
}

void ToolController::setPivotAnchor(HandleKind anchor) {
    if (doc_ == nullptr) {
        return;
    }
    if (const std::optional<geometry::Box2> bounds = transformBounds(*doc_)) {
        pivot_ = anchorPoint(*bounds, anchor);
    }
}

CursorHint ToolController::cursorHint() const {
    if (gesture_.active()) {
        switch (gesture_.kind()) {
            case GestureKind::Move: return CursorHint::Move;
            case GestureKind::Rotate: return CursorHint::Rotate;
            case GestureKind::Scale: return scaleHint(xfHandle_);
            case GestureKind::None: break;
        }
    }
    if (xfPress_ == XfPress::Pivot) {
        return CursorHint::Pivot;
    }
    if (tool_ != ToolKind::Select || !hasCursor_) {
        return CursorHint::Default;
    }
    const std::optional<SelectionBox> box = selectionHandles();
    if (!box.has_value()) {
        return CursorHint::Default;
    }
    const std::optional<HandleKind> h = hitHandle(*box, cursorPx_, kHandleHitPx);
    if (!h.has_value()) {
        return CursorHint::Default;
    }
    if (xfMode_ == TransformMode::Scale) {
        return scaleHint(*h);
    }
    if (*h == HandleKind::Centre) {
        return CursorHint::Pivot;
    }
    return isCornerHandle(*h) ? CursorHint::Rotate : CursorHint::Default;
}

bool ToolController::gestureIndexValid() const {
    return gestureSnapIndex_.has_value() && doc_ != nullptr &&
           gestureIndexKey_[0] == doc_->undoStack().generation() && gestureIndexKey_[1] == doc_->activeSheet().value;
}

void ToolController::xfReset() {
    gestureSnapIndex_.reset();
    xfPress_ = XfPress::None;
    xfDragging_ = false;
    xfToggleOnClick_ = false;
}

ToolResult ToolController::xfPressBegin(Vec2 px, bool handlesOnly) {
    xfReset();
    const std::optional<geometry::Box2> bounds = transformBounds(*doc_);
    if (!bounds.has_value()) {
        return {};
    }
    const SelectionBox box = selectionBox(*bounds, view_);
    const std::optional<HandleKind> h = hitHandle(box, px, kHandleHitPx);
    XfPress kind = XfPress::None;
    bool toggle = false;
    if (h.has_value()) {
        if (*h == HandleKind::Centre) {
            kind = xfMode_ == TransformMode::Scale ? XfPress::Move : XfPress::Pivot;
            toggle = true;
        } else if (xfMode_ == TransformMode::Scale) {
            kind = XfPress::Scale;
        } else if (isCornerHandle(*h)) {
            kind = XfPress::Rotate;
        }
    }
    if (kind == XfPress::None && !handlesOnly) {
        HitOptions options;
        options.fill = FillHit::StrokeAndFill;
        const std::optional<Hit> hit = hitTest(*doc_, view_, px, options);
        if (hit.has_value() && doc_->selection().contains(hit->object)) {
            kind = XfPress::Move;
            toggle = true;
        }
    }
    if (kind == XfPress::None) {
        return {};
    }
    xfPress_ = kind;
    xfHandle_ = h.value_or(HandleKind::Centre);
    xfToggleOnClick_ = toggle;
    xfStartPx_ = px;
    updatePointer(px, mods_);  // from now on the pointer snaps
    xfStartMm_ = cursorMm_;
    return {};
}

ToolResult ToolController::xfMove(Vec2 px) {
    ToolResult r;
    if (!xfDragging_) {
        if (geometry::distance(px, xfStartPx_) < kDragThresholdPx) {
            return r;
        }
        xfDragging_ = true;
        if (xfPress_ == XfPress::Move) {
            gesture_.begin(*doc_, GestureKind::Move, xfHandle_, xfStartMm_, xfStartMm_);
        } else if (xfPress_ == XfPress::Scale) {
            gesture_.begin(*doc_, GestureKind::Scale, xfHandle_, xfStartMm_, xfStartMm_);
        } else if (xfPress_ == XfPress::Rotate) {
            const std::optional<Vec2> pivot = pivotMm();
            if (pivot.has_value()) {
                gesture_.begin(*doc_, GestureKind::Rotate, xfHandle_, xfStartMm_, *pivot);
            }
        }
    }
    if (xfPress_ == XfPress::Pivot) {
        pivot_ = cursorMm_;
        return r;
    }
    if (gesture_.active() && !gestureSnapIndex_.has_value()) {
        gestureSnapIndex_.emplace(*doc_, false, gesture_.targets());
        ++snapIndexBuilds_;
        gestureIndexKey_ = {doc_->undoStack().generation(), doc_->activeSheet().value};
    }
    if (gesture_.active()) {
        const bool valid = gestureIndexValid();
        r.transformed = gesture_.update(*doc_, cursorMm_, mods_.shift, mods_.ctrl, mods_.alt);
        if (!valid || !gesture_.active()) {
            gestureSnapIndex_.reset();  // an outside edit (or the end of the gesture) invalidates it
        } else if (r.transformed) {
            gestureIndexKey_[0] = doc_->undoStack().generation();  // the gesture's own push
        }
    }
    return r;
}

ToolResult ToolController::xfRelease() {
    ToolResult r;
    if (xfDragging_) {
        r = xfMove(cursorPx_);
    }
    if (gesture_.active()) {
        gesture_.end(*doc_);
    } else if (!xfDragging_ && xfToggleOnClick_) {
        xfMode_ = xfMode_ == TransformMode::Scale ? TransformMode::Rotate : TransformMode::Scale;
    }
    xfReset();
    return r;
}

}  // namespace carvetto::viewmodel
