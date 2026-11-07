#include "carvetto/viewmodel/transform_ops.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <numeric>
#include <unordered_set>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/contour.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Transform2;
using geometry::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;

bool layerUsable(const Document& doc, document::LayerId id) {
    const document::Layer* layer = doc.findLayer(id);
    return layer != nullptr && layer->visible && !layer->locked;
}

/// True when the object and every ancestor group sit on visible, unlocked layers.
bool chainUsable(const Document& doc, const Object& obj) {
    const Object* cur = &obj;
    for (std::size_t guard = 0; guard <= doc.objectCount(); ++guard) {
        if (!layerUsable(doc, cur->layer)) {
            return false;
        }
        if (!cur->parent.has_value()) {
            return true;
        }
        cur = doc.findObject(*cur->parent);
        if (cur == nullptr) {
            return false;
        }
    }
    return false;
}

/// One selected object (a leaf, or a group with all its leaves).
struct Operand {
    ObjectId root{};
    std::vector<ObjectId> leaves;
    Box2 bounds;
};

/// Collects the leaves under `obj`; false when a leaf is not usable (the whole operand is skipped).
bool gatherLeaves(const Document& doc, const Object& obj, std::vector<ObjectId>& out, std::size_t depth) {
    if (depth > doc.objectCount()) {
        return false;
    }
    if (const document::GroupObject* group = obj.asGroup(); group != nullptr) {
        bool any = false;
        for (ObjectId child : group->children) {
            const Object* c = doc.findObject(child);
            if (c == nullptr || !gatherLeaves(doc, *c, out, depth + 1)) {
                return false;
            }
            any = true;
        }
        return any;
    }
    if (obj.sheet != doc.activeSheet() || !chainUsable(doc, obj)) {
        return false;
    }
    out.push_back(obj.id);
    return true;
}

std::optional<Box2> leafBounds(const Document& doc, ObjectId id) {
    const Object* obj = doc.findObject(id);
    if (obj == nullptr) {
        return std::nullopt;
    }
    Box2 b;
    if (const auto* v = obj->asVector(); v != nullptr) {
        b = v->bounds();
    } else if (const auto* bm = obj->asBitmap(); bm != nullptr) {
        b = bm->bounds();
    }
    if (b.isEmpty()) {
        return std::nullopt;
    }
    return b;
}

/// Operands in selection order. A leaf already taken by an earlier operand is not taken again.
std::vector<Operand> collectOperands(const Document& doc) {
    std::vector<Operand> operands;
    std::unordered_set<ObjectId> seen;
    for (ObjectId id : doc.selection().ids()) {
        const Object* obj = doc.findObject(id);
        if (obj == nullptr) {
            continue;
        }
        Operand op;
        op.root = id;
        if (!gatherLeaves(doc, *obj, op.leaves, 0)) {
            continue;
        }
        std::vector<ObjectId> fresh;
        for (ObjectId leaf : op.leaves) {
            if (!seen.contains(leaf)) {
                fresh.push_back(leaf);
            }
        }
        for (ObjectId leaf : fresh) {
            seen.insert(leaf);
            if (const auto b = leafBounds(doc, leaf)) {
                op.bounds.expand(*b);
            }
        }
        op.leaves = std::move(fresh);
        if (!op.leaves.empty() && !op.bounds.isEmpty()) {
            operands.push_back(std::move(op));
        }
    }
    return operands;
}

bool finite(Vec2 p) { return std::isfinite(p.x) && std::isfinite(p.y); }
bool inRange(Vec2 p) {
    return finite(p) && std::fabs(p.x) <= kMaxTransformCoordinateMm && std::fabs(p.y) <= kMaxTransformCoordinateMm;
}
bool inRange(const Box2& b) { return !b.isEmpty() && inRange(b.min) && inRange(b.max); }

bool transformFinite(const Transform2& m) {
    return std::isfinite(m.a) && std::isfinite(m.b) && std::isfinite(m.c) && std::isfinite(m.d) &&
           std::isfinite(m.tx) && std::isfinite(m.ty);
}

/// The box's corners under `m` stay in range (a conservative check for rotations).
bool resultInRange(const Box2& b, const Transform2& m) {
    const Vec2 corners[4] = {b.min, {b.max.x, b.min.y}, b.max, {b.min.x, b.max.y}};
    for (const Vec2& c : corners) {
        if (!inRange(m.apply(c))) {
            return false;
        }
    }
    return true;
}

struct Step {
    const Operand* operand{nullptr};
    Transform2 m;
};

TransformResult refuse(std::string message) {
    TransformResult r;
    r.message = std::move(message);
    return r;
}

/// Validates and runs the steps as one macro. Identity steps are skipped; nothing to do is a
/// successful no-op.
TransformResult runSteps(Document& doc, const std::vector<Step>& steps, const std::string& text) {
    for (const Step& s : steps) {
        if (!transformFinite(s.m) || !s.m.isInvertible(1e-13)) {
            return refuse("the transform is not valid");
        }
        if (!inRange(s.operand->bounds)) {
            return refuse("coordinates must be finite and within 1e9 mm");
        }
        if (!resultInRange(s.operand->bounds, s.m)) {
            return refuse("the result would leave the supported range of 1e9 mm");
        }
    }
    TransformResult result;
    result.ok = true;
    std::vector<const Step*> active;
    for (const Step& s : steps) {
        if (!(s.m == Transform2::identity())) {
            active.push_back(&s);
        }
    }
    if (active.empty()) {
        return result;
    }
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro(text);
    for (const Step* s : active) {
        if (doc.execute(std::make_unique<document::TransformObjectsCommand>(s->operand->leaves, s->m))) {
            result.objects += s->operand->leaves.size();
        }
    }
    doc.undoStack().endMacro();
    doc.undoStack().breakCoalescing();
    return result;
}

Box2 unionBounds(const std::vector<Operand>& operands) {
    Box2 all;
    for (const Operand& op : operands) {
        all.expand(op.bounds);
    }
    return all;
}

TransformResult noTargets() { return refuse("nothing to transform: select objects on unlocked, visible layers"); }

bool validFactor(double f) { return std::isfinite(f) && f >= kMinScaleFactor && f <= kMaxScaleFactor; }

}  // namespace

std::vector<ObjectId> transformTargets(const Document& doc) {
    std::vector<ObjectId> out;
    for (const Operand& op : collectOperands(doc)) {
        out.insert(out.end(), op.leaves.begin(), op.leaves.end());
    }
    return out;
}

std::optional<Box2> transformBounds(const Document& doc) {
    const Box2 all = unionBounds(collectOperands(doc));
    if (all.isEmpty()) {
        return std::nullopt;
    }
    return all;
}

Vec2 anchorPoint(const Box2& box, HandleKind anchor) {
    const double xs[3] = {box.min.x, 0.5 * (box.min.x + box.max.x), box.max.x};
    const double ys[3] = {box.max.y, 0.5 * (box.min.y + box.max.y), box.min.y};  // top row first
    const int k = static_cast<int>(anchor);
    return {xs[k % 3], ys[k / 3]};
}

bool canTransform(const Document& doc) { return !collectOperands(doc).empty(); }

TransformResult applyTransform(Document& doc, const Transform2& m, const std::string& text) {
    const std::vector<Operand> operands = collectOperands(doc);
    if (operands.empty()) {
        return noTargets();
    }
    std::vector<Step> steps;
    for (const Operand& op : operands) {
        steps.push_back({&op, m});
    }
    return runSteps(doc, steps, text);
}

// --- Move -----------------------------------------------------------------------------------------

TransformResult moveSelection(Document& doc, const MoveOptions& options) {
    if (!std::isfinite(options.xMm) || !std::isfinite(options.yMm) || std::fabs(options.xMm) > kMaxTransformCoordinateMm ||
        std::fabs(options.yMm) > kMaxTransformCoordinateMm) {
        return refuse("the position must be a finite number within 1e9 mm");
    }
    const std::vector<Operand> operands = collectOperands(doc);
    if (operands.empty()) {
        return noTargets();
    }
    Vec2 delta{options.xMm, options.yMm};
    if (options.absolute) {
        delta = delta - anchorPoint(unionBounds(operands), options.anchor);
    }
    std::vector<Step> steps;
    for (const Operand& op : operands) {
        steps.push_back({&op, Transform2::translation(delta)});
    }
    return runSteps(doc, steps, "Move");
}

// --- Scale ----------------------------------------------------------------------------------------

TransformResult scaleSelection(Document& doc, const ScaleOptions& options) {
    if (!validFactor(options.scaleX) || !validFactor(options.scaleY)) {
        return refuse("the scale factor must be a positive number");
    }
    const std::vector<Operand> operands = collectOperands(doc);
    if (operands.empty()) {
        return noTargets();
    }
    std::vector<Step> steps;
    if (options.mode == ScaleMode::Selection) {
        const Vec2 a = anchorPoint(unionBounds(operands), options.anchor);
        for (const Operand& op : operands) {
            steps.push_back({&op, Transform2::scaling(options.scaleX, options.scaleY, a)});
        }
    } else {
        for (const Operand& op : operands) {
            const Vec2 a = anchorPoint(op.bounds, options.anchor);
            steps.push_back({&op, Transform2::scaling(options.scaleX, options.scaleY, a)});
        }
    }
    return runSteps(doc, steps, "Scale");
}

std::optional<ScaleFactors> scaleFactorsForSize(const Box2& bounds, std::optional<double> widthMm,
                                                std::optional<double> heightMm, bool linkXY) {
    if (bounds.isEmpty()) {
        return std::nullopt;
    }
    for (const std::optional<double>& v : {widthMm, heightMm}) {
        if (v.has_value() && (!std::isfinite(*v) || *v <= 0.0)) {
            return std::nullopt;
        }
    }
    constexpr double kThin = 1e-9;
    ScaleFactors f;
    bool haveX = false;
    bool haveY = false;
    if (widthMm.has_value()) {
        if (bounds.width() <= kThin) {
            return std::nullopt;
        }
        f.x = *widthMm / bounds.width();
        haveX = true;
    }
    if (heightMm.has_value()) {
        if (bounds.height() <= kThin) {
            return std::nullopt;
        }
        f.y = *heightMm / bounds.height();
        haveY = true;
    }
    if (linkXY) {
        if (haveX && !haveY) {
            f.y = f.x;
        } else if (haveY && !haveX) {
            f.x = f.y;
        } else if (haveX && haveY) {
            f.y = f.x;  // linked: the width decides
        }
    }
    return f;
}

// --- Rotate ---------------------------------------------------------------------------------------

double selectionOrientation(const Document& doc) {
    const std::vector<Operand> operands = collectOperands(doc);
    if (operands.size() != 1 || operands.front().leaves.size() != 1) {
        return 0.0;
    }
    const Object* obj = doc.findObject(operands.front().leaves.front());
    const auto* vec = obj != nullptr ? obj->asVector() : nullptr;
    if (vec == nullptr) {
        return 0.0;
    }
    if (const auto* e = std::get_if<document::EllipseParams>(&vec->params)) {
        return e->angle;
    }
    if (const auto* p = std::get_if<document::PolygonParams>(&vec->params)) {
        return p->angle;
    }
    if (const auto* s = std::get_if<document::StarParams>(&vec->params)) {
        return s->angle;
    }
    return 0.0;
}

TransformResult rotateSelection(Document& doc, const RotateOptions& options) {
    if (!std::isfinite(options.angleRad) || std::fabs(options.angleRad) > 1.0e6) {
        return refuse("the angle must be a finite number");
    }
    if (options.pivotMm.has_value() && !inRange(*options.pivotMm)) {
        return refuse("the pivot must be finite and within 1e9 mm");
    }
    const std::vector<Operand> operands = collectOperands(doc);
    if (operands.empty()) {
        return noTargets();
    }
    const Vec2 pivot = options.pivotMm.value_or(anchorPoint(unionBounds(operands), options.pivotAnchor));
    const double delta = options.absolute ? options.angleRad - selectionOrientation(doc) : options.angleRad;
    const Transform2 m = Transform2::rotation(delta, pivot);
    std::vector<Step> steps;
    for (const Operand& op : operands) {
        steps.push_back({&op, m});
    }
    return runSteps(doc, steps, "Rotate");
}

// --- Mirror ---------------------------------------------------------------------------------------

namespace {

struct OrientedBox {
    Vec2 centre;
    Vec2 u{1.0, 0.0};  ///< unit direction of the first side, within (-45, 45] degrees of +X
};

std::vector<Vec2> hull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
    pts.erase(std::unique(pts.begin(), pts.end(), [](Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }), pts.end());
    if (pts.size() < 3) {
        return pts;
    }
    std::vector<Vec2> h(2 * pts.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && (h[k - 1] - h[k - 2]).cross(pts[i] - h[k - 2]) <= 0.0) {
            --k;
        }
        h[k++] = pts[i];
    }
    for (std::size_t i = pts.size() - 1, t = k + 1; i > 0; --i) {
        while (k >= t && (h[k - 1] - h[k - 2]).cross(pts[i - 1] - h[k - 2]) <= 0.0) {
            --k;
        }
        h[k++] = pts[i - 1];
    }
    h.resize(k - 1);
    return h;
}

/// Minimum-area rotated bounding box of the target geometry (curves sampled at 64 points per span).
OrientedBox orientedBounds(const Document& doc, const std::vector<Operand>& operands) {
    std::vector<Vec2> pts;
    for (const Operand& op : operands) {
        for (ObjectId leaf : op.leaves) {
            const Object* obj = doc.findObject(leaf);
            if (const auto* v = obj->asVector(); v != nullptr) {
                for (const geometry::Contour& c : v->contours) {
                    for (const geometry::Span& s : c.spans) {
                        const int n = std::holds_alternative<geometry::LineSpan>(s) ? 1 : 64;
                        for (int i = 0; i <= n; ++i) {
                            pts.push_back(geometry::pointAt(s, static_cast<double>(i) / n));
                        }
                    }
                }
            } else if (const auto* bm = obj->asBitmap(); bm != nullptr) {
                pts.push_back(bm->placementMm.min);
                pts.push_back(bm->placementMm.max);
                pts.push_back({bm->placementMm.min.x, bm->placementMm.max.y});
                pts.push_back({bm->placementMm.max.x, bm->placementMm.min.y});
            }
        }
    }
    OrientedBox best;
    best.centre = unionBounds(operands).centre();
    const std::vector<Vec2> h = hull(std::move(pts));
    if (h.size() < 3) {
        return best;
    }
    const std::size_t stride = std::max<std::size_t>(1, h.size() / 1024);
    double bestArea = -1.0;
    for (std::size_t i = 0; i < h.size(); i += stride) {
        const Vec2 edge = h[(i + 1) % h.size()] - h[i];
        if (edge.length() < 1e-12) {
            continue;
        }
        const Vec2 u = edge.normalized();
        const Vec2 v = u.perp();
        double lo0 = 1e300;
        double hi0 = -1e300;
        double lo1 = 1e300;
        double hi1 = -1e300;
        for (const Vec2& p : h) {
            const double a = p.dot(u);
            const double b = p.dot(v);
            lo0 = std::min(lo0, a);
            hi0 = std::max(hi0, a);
            lo1 = std::min(lo1, b);
            hi1 = std::max(hi1, b);
        }
        const double area = (hi0 - lo0) * (hi1 - lo1);
        if (bestArea < 0.0 || area < bestArea - 1e-12) {
            bestArea = area;
            // Normalise the direction into (-45, 45] degrees of +X (u and perp are the two sides).
            Vec2 d = u;
            double ang = std::atan2(d.y, d.x);
            while (ang > kPi / 4.0) {
                ang -= kPi / 2.0;
            }
            while (ang <= -kPi / 4.0) {
                ang += kPi / 2.0;
            }
            best.u = Vec2::fromAngle(ang);
            best.centre = u * (0.5 * (lo0 + hi0)) + v * (0.5 * (lo1 + hi1));
        }
    }
    return best;
}

/// The last operand when it is a single straight line: its (start, direction).
std::optional<std::pair<Vec2, Vec2>> lineAxis(const Document& doc, const Operand& op) {
    if (op.leaves.size() != 1) {
        return std::nullopt;
    }
    const auto* vec = doc.findObject(op.leaves.front())->asVector();
    if (vec == nullptr || vec->contours.size() != 1 || vec->contours.front().spans.size() != 1) {
        return std::nullopt;
    }
    const auto* line = std::get_if<geometry::LineSpan>(&vec->contours.front().spans.front());
    if (line == nullptr || line->length() < 1e-9) {
        return std::nullopt;
    }
    return std::make_pair(line->start, (line->end - line->start).normalized());
}

}  // namespace

TransformResult mirrorSelection(Document& doc, const MirrorOptions& options) {
    std::vector<Operand> operands = collectOperands(doc);
    if (operands.empty()) {
        return noTargets();
    }
    Transform2 m;
    if (options.aboutSelectedLine) {
        const auto axis = lineAxis(doc, operands.back());
        if (!axis.has_value()) {
            return refuse("select a single straight line (last) to mirror about");
        }
        operands.pop_back();
        if (operands.empty()) {
            return refuse("select objects to mirror before the line");
        }
        m = Transform2::mirror(axis->first, axis->second);
    } else if (options.useRotatedBounds) {
        const OrientedBox box = orientedBounds(doc, operands);
        const Vec2 dir = options.axis == MirrorAxis::Horizontal ? box.u.perp() : box.u;
        m = Transform2::mirror(box.centre, dir);
    } else {
        Vec2 c = unionBounds(operands).centre();
        if (options.aboutMaterialCentre) {
            const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
            if (sheet == nullptr) {
                return refuse("there is no active sheet");
            }
            c = sheet->materialBoundsMm().centre();
        }
        m = options.axis == MirrorAxis::Horizontal ? Transform2::mirrorX(c.x) : Transform2::mirrorY(c.y);
    }
    if (!transformFinite(m)) {
        return refuse("the mirror axis is not valid");
    }
    std::vector<Step> steps;
    for (const Operand& op : operands) {
        steps.push_back({&op, m});
    }
    if (!options.createCopy) {
        return runSteps(doc, steps, "Mirror");
    }

    // Copies: validate first, then paste and transform the pasted roots inside one macro.
    for (const Step& s : steps) {
        if (!inRange(s.operand->bounds) || !resultInRange(s.operand->bounds, s.m)) {
            return refuse("the result would leave the supported range of 1e9 mm");
        }
    }
    std::vector<ObjectId> roots;
    std::size_t needed = 0;
    for (const Operand& op : operands) {
        roots.push_back(op.root);
    }
    std::vector<Object> clipboard = doc.copyObjects(roots);
    needed = clipboard.size();
    if (clipboard.empty() || !doc.idGenerator().canAllocate(needed)) {
        return refuse("no free object ids for the copies");
    }
    const std::unordered_set<ObjectId> rootSet(roots.begin(), roots.end());
    std::vector<ObjectId> clipboardIds;
    for (const Object& o : clipboard) {
        clipboardIds.push_back(o.id);
    }
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Mirror copy");
    auto paste = std::make_unique<document::PasteObjectsCommand>(std::move(clipboard));
    const document::PasteObjectsCommand* pasteRaw = paste.get();
    std::vector<ObjectId> created;
    TransformResult result;
    if (doc.execute(std::move(paste))) {
        const std::vector<ObjectId>& pasted = pasteRaw->pastedIds();
        for (std::size_t i = 0; i < pasted.size() && i < clipboardIds.size(); ++i) {
            if (rootSet.contains(clipboardIds[i])) {
                created.push_back(pasted[i]);
            }
        }
        if (doc.execute(std::make_unique<document::TransformObjectsCommand>(created, m))) {
            result.ok = true;
        }
    }
    doc.undoStack().endMacro();
    doc.undoStack().breakCoalescing();
    if (!result.ok) {
        return refuse("the copies could not be created");
    }
    result.objects = created.size();
    result.created = created;
    doc.selection().assign(created);
    return result;
}

// --- Align and space --------------------------------------------------------------------------------

TransformResult alignToLast(Document& doc, AlignKind kind) {
    const std::vector<Operand> operands = collectOperands(doc);
    if (operands.size() < 2) {
        return refuse("select at least two objects; the last selected is the reference");
    }
    const Box2& ref = operands.back().bounds;
    std::vector<Step> steps;
    for (std::size_t i = 0; i + 1 < operands.size(); ++i) {
        const Box2& b = operands[i].bounds;
        Vec2 d{0.0, 0.0};
        switch (kind) {
            case AlignKind::Centre: d = ref.centre() - b.centre(); break;
            case AlignKind::VerticalCentre: d.y = ref.centre().y - b.centre().y; break;
            case AlignKind::HorizontalCentre: d.x = ref.centre().x - b.centre().x; break;
            case AlignKind::Left: d.x = ref.min.x - b.min.x; break;
            case AlignKind::Right: d.x = ref.max.x - b.max.x; break;
            case AlignKind::OutsideTop: d.y = ref.max.y - b.min.y; break;
            case AlignKind::OutsideBottom: d.y = ref.min.y - b.max.y; break;
        }
        steps.push_back({&operands[i], Transform2::translation(d)});
    }
    return runSteps(doc, steps, "Align");
}

TransformResult centreInMaterial(Document& doc, CentreAxes axes) {
    const std::vector<Operand> operands = collectOperands(doc);
    if (operands.empty()) {
        return noTargets();
    }
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    if (sheet == nullptr) {
        return refuse("there is no active sheet");
    }
    Vec2 d = sheet->materialBoundsMm().centre() - unionBounds(operands).centre();
    if (axes == CentreAxes::XOnly) {
        d.y = 0.0;
    } else if (axes == CentreAxes::YOnly) {
        d.x = 0.0;
    }
    std::vector<Step> steps;
    for (const Operand& op : operands) {
        steps.push_back({&op, Transform2::translation(d)});
    }
    return runSteps(doc, steps, "Centre in material");
}

TransformResult spaceSelection(Document& doc, const SpaceOptions& options) {
    std::vector<Operand> operands = collectOperands(doc);
    const bool horizontal = options.direction == SpaceDirection::Horizontal;
    const std::size_t minimum = options.insideLast ? 2 : 3;
    if (operands.size() < minimum) {
        return refuse(options.insideLast ? "select the objects to space and then the container"
                                         : "select at least three objects to space evenly");
    }
    const auto lo = [&](const Box2& b) { return horizontal ? b.min.x : b.min.y; };
    const auto hi = [&](const Box2& b) { return horizontal ? b.max.x : b.max.y; };
    const auto mid = [&](const Box2& b) { return 0.5 * (lo(b) + hi(b)); };

    std::vector<const Operand*> items;
    const Operand* container = nullptr;
    for (std::size_t i = 0; i < operands.size(); ++i) {
        if (options.insideLast && i + 1 == operands.size()) {
            container = &operands[i];
        } else {
            items.push_back(&operands[i]);
        }
    }
    if (!options.selectionOrder) {
        std::stable_sort(items.begin(), items.end(),
                         [&](const Operand* a, const Operand* b) { return mid(a->bounds) < mid(b->bounds); });
    }
    double sum = 0.0;
    for (const Operand* it : items) {
        sum += hi(it->bounds) - lo(it->bounds);
    }
    const double n = static_cast<double>(items.size());

    // Work along a forward axis t = sign * coordinate so a selection order that runs against the
    // axis still places the first object first.
    double sign = 1.0;
    double start = 0.0;
    double gap = 0.0;
    if (container != nullptr) {
        const double length = hi(container->bounds) - lo(container->bounds);
        if (sum > length) {
            return refuse("the objects do not fit inside the last selected object");
        }
        start = lo(container->bounds);
        gap = (length - sum) / (n + 1.0);
        start += gap;
    } else {
        sign = mid(items.front()->bounds) <= mid(items.back()->bounds) ? 1.0 : -1.0;
        const auto tlo = [&](const Box2& b) { return sign > 0.0 ? lo(b) : -hi(b); };
        const auto thi = [&](const Box2& b) { return sign > 0.0 ? hi(b) : -lo(b); };
        start = tlo(items.front()->bounds);
        const double end = thi(items.back()->bounds);
        gap = (end - start - sum) / (n - 1.0);
    }
    std::vector<Step> steps;
    double cursor = start;
    for (const Operand* it : items) {
        const double t0 = sign > 0.0 ? lo(it->bounds) : -hi(it->bounds);
        const double shift = sign * (cursor - t0);
        cursor += (hi(it->bounds) - lo(it->bounds)) + gap;
        const Vec2 d = horizontal ? Vec2{shift, 0.0} : Vec2{0.0, shift};
        steps.push_back({it, Transform2::translation(d)});
    }
    return runSteps(doc, steps, "Space selection");
}

}  // namespace carvetto::viewmodel
