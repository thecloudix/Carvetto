#include "carvetto/viewmodel/selection_tools.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <unordered_set>

#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/tolerance.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;

namespace {

/// Distances within this many pixels tie.
constexpr double kTiePx = 1e-9;
/// Chordal tolerance, in pixels, for polygonising curves in marquee tests.
constexpr double kMarqueeChordPx = 0.05;
constexpr double kMinChordalMm = 1e-6;

ObjectId topOf(const Document& doc, const Object& obj) {
    ObjectId top = obj.id;
    const Object* cur = &obj;
    // Parent links are acyclic in a consistent document; bound the walk anyway.
    for (std::size_t guard = 0; cur->parent.has_value() && guard <= doc.objectCount(); ++guard) {
        const Object* parent = doc.findObject(*cur->parent);
        if (parent == nullptr) {
            break;
        }
        top = parent->id;
        cur = parent;
    }
    return top;
}

bool layerPickable(const Document& doc, document::LayerId id) {
    const document::Layer* layer = doc.findLayer(id);
    return layer != nullptr && layer->visible && !layer->locked;
}

/// True when the object and every ancestor group sit on visible, unlocked layers.
bool chainPickable(const Document& doc, const Object& obj) {
    const Object* cur = &obj;
    for (std::size_t guard = 0; guard <= doc.objectCount(); ++guard) {
        if (!layerPickable(doc, cur->layer)) {
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

geometry::Tolerance toleranceFor(double scale, double chordalPx) {
    geometry::Tolerance tol;
    tol.chordal = std::max(chordalPx / scale, kMinChordalMm);
    tol.distance = std::min(tol.distance, tol.chordal);
    return tol;
}

/// Inclusive segment-vs-box intersection (Liang-Barsky).
bool segmentTouchesBox(Vec2 a, Vec2 b, const Box2& box) {
    double t0 = 0.0;
    double t1 = 1.0;
    const double dx = b.x - a.x;
    const double dy = b.y - a.y;
    const double p[4] = {-dx, dx, -dy, dy};
    const double q[4] = {a.x - box.min.x, box.max.x - a.x, a.y - box.min.y, box.max.y - a.y};
    for (int i = 0; i < 4; ++i) {
        if (p[i] == 0.0) {
            if (q[i] < 0.0) {
                return false;
            }
        } else {
            const double r = q[i] / p[i];
            if (p[i] < 0.0) {
                t0 = std::max(t0, r);
            } else {
                t1 = std::min(t1, r);
            }
            if (t0 > t1) {
                return false;
            }
        }
    }
    return true;
}

bool contourTouchesBox(const Contour& c, const Box2& box, const geometry::Tolerance& tol) {
    if (c.isEmpty() || !c.bounds().intersects(box)) {
        return false;
    }
    const geometry::Polyline pl = c.polygonise(tol);
    if (pl.points.size() == 1) {
        return box.contains(pl.points.front());
    }
    for (std::size_t i = 0; i < pl.segmentCount(); ++i) {
        if (segmentTouchesBox(pl.segmentStart(i), pl.segmentEnd(i), box)) {
            return true;
        }
    }
    return false;
}

std::vector<Contour> closedContours(const document::VectorObject& vec) {
    std::vector<Contour> out;
    for (const Contour& c : vec.contours) {
        if (c.closed && !c.isEmpty()) {
            out.push_back(c);
        }
    }
    return out;
}

Box2 worldRect(const ViewTransform& view, const Box2& screenRect) {
    if (screenRect.isEmpty()) {
        return Box2{};
    }
    return Box2{view.screenToWorld(screenRect.min), view.screenToWorld(screenRect.max)};
}

struct Candidate {
    Hit hit;
    std::size_t layerIndex{0};
    double areaMm2{0.0};
};

/// True when `a` should win over `b` (see the ordering in the header).
bool better(const Candidate& a, const Candidate& b) {
    if (a.hit.viaFill != b.hit.viaFill) {
        return !a.hit.viaFill;
    }
    if (!a.hit.viaFill && std::fabs(a.hit.distancePx - b.hit.distancePx) > kTiePx) {
        return a.hit.distancePx < b.hit.distancePx;
    }
    if (a.layerIndex != b.layerIndex) {
        return a.layerIndex > b.layerIndex;
    }
    if (a.areaMm2 != b.areaMm2) {
        return a.areaMm2 < b.areaMm2;
    }
    return a.hit.leaf.value > b.hit.leaf.value;
}

}  // namespace

PickIndex::PickIndex(const Document& doc) : doc_(&doc) {
    std::vector<Box2> boxes;
    for (const auto& [id, obj] : doc.objects()) {
        if (obj.sheet != doc.activeSheet() || obj.isGroup() || !chainPickable(doc, obj)) {
            continue;
        }
        Box2 b;
        if (const auto* vec = obj.asVector()) {
            b = vec->bounds();
        } else if (const auto* bmp = obj.asBitmap()) {
            b = bmp->bounds();
        }
        if (b.isEmpty()) {
            continue;
        }
        Entry e;
        e.leaf = id;
        e.top = topOf(doc, obj);
        e.layerIndex = doc.layerIndex(obj.layer).value_or(0);
        e.bounds = b;
        entries_.push_back(e);
        boxes.push_back(b);
    }
    index_ = toolpath::BoxIndex(std::move(boxes));
}

std::optional<Hit> PickIndex::hitTest(const ViewTransform& view, Vec2 screenPx, const HitOptions& options) const {
    const double scale = view.scale();
    const double tolPx = std::max(options.tolerancePx, 0.0);
    const double tolMm = tolPx / scale;
    const Vec2 p = view.screenToWorld(screenPx);
    const geometry::Tolerance geomTol = toleranceFor(scale, 0.25);

    std::optional<Candidate> best;
    for (const std::size_t i : index_.query(Box2{p, p}.inflated(tolMm))) {
        const Entry& e = entries_[i];
        const Object* obj = doc_->findObject(e.leaf);
        if (obj == nullptr) {
            continue;
        }
        Candidate cand;
        cand.hit.object = e.top;
        cand.hit.leaf = e.leaf;
        cand.layerIndex = e.layerIndex;
        cand.areaMm2 = e.bounds.area();
        bool hit = false;
        if (const auto* vec = obj->asVector()) {
            double dMm = std::numeric_limits<double>::infinity();
            for (const Contour& c : vec->contours) {
                if (c.isEmpty() || !c.bounds().inflated(tolMm).contains(p)) {
                    continue;
                }
                dMm = std::min(dMm, geometry::distanceTo(c, p));
            }
            if (dMm * scale <= tolPx) {
                cand.hit.distancePx = dMm * scale;
                hit = true;
            } else if (options.fill == FillHit::StrokeAndFill) {
                const std::vector<Contour> closed = closedContours(*vec);
                if (!closed.empty() &&
                    geometry::isInside(closed, p, geometry::FillRule::EvenOdd, geomTol)) {
                    cand.hit.viaFill = true;
                    hit = true;
                }
            }
        } else if (obj->asBitmap() != nullptr) {
            if (e.bounds.contains(p, tolMm)) {
                cand.hit.viaFill = true;
                hit = true;
            }
        }
        if (hit && (!best.has_value() || better(cand, *best))) {
            best = cand;
        }
    }
    if (!best.has_value()) {
        return std::nullopt;
    }
    return best->hit;
}

std::vector<ObjectId> PickIndex::marquee(const ViewTransform& view, const Box2& screenRect, MarqueeMode mode,
                                         FillHit fill) const {
    const Box2 rect = worldRect(view, screenRect);
    if (rect.isEmpty()) {
        return {};
    }
    const geometry::Tolerance tol = toleranceFor(view.scale(), kMarqueeChordPx);
    const std::vector<std::size_t> nearby = index_.query(rect);

    std::unordered_set<ObjectId> candidates;
    for (const std::size_t i : nearby) {
        candidates.insert(entries_[i].top);
    }
    std::unordered_set<std::size_t> touching;  // entries whose geometry touches the rectangle
    if (mode == MarqueeMode::Crossing) {
        for (const std::size_t i : nearby) {
            const Entry& e = entries_[i];
            const Object* obj = doc_->findObject(e.leaf);
            if (obj == nullptr) {
                continue;
            }
            bool t = false;
            if (const auto* vec = obj->asVector()) {
                for (const Contour& c : vec->contours) {
                    if (contourTouchesBox(c, rect, tol)) {
                        t = true;
                        break;
                    }
                }
                if (!t && fill == FillHit::StrokeAndFill) {
                    const std::vector<Contour> closed = closedContours(*vec);
                    t = !closed.empty() && geometry::isInside(closed, rect.min, geometry::FillRule::EvenOdd, tol);
                }
            } else {
                t = true;  // bitmap whose placement box meets the rectangle
            }
            if (t) {
                touching.insert(i);
            }
        }
    }

    // Per-top verdicts.
    std::unordered_map<ObjectId, bool> verdict;
    for (const ObjectId top : candidates) {
        verdict[top] = (mode == MarqueeMode::Window);
    }
    for (std::size_t i = 0; i < entries_.size(); ++i) {
        const auto it = verdict.find(entries_[i].top);
        if (it == verdict.end()) {
            continue;
        }
        if (mode == MarqueeMode::Window) {
            if (!rect.contains(entries_[i].bounds)) {
                it->second = false;
            }
        } else if (touching.contains(i)) {
            it->second = true;
        }
    }
    std::vector<ObjectId> out;
    for (const auto& [top, ok] : verdict) {
        if (ok) {
            out.push_back(top);
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

std::vector<ObjectId> PickIndex::topLevelObjects() const {
    std::vector<ObjectId> out;
    for (const Entry& e : entries_) {
        out.push_back(e.top);
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

std::optional<Hit> hitTest(const Document& doc, const ViewTransform& view, Vec2 screenPx, const HitOptions& options) {
    return PickIndex(doc).hitTest(view, screenPx, options);
}

std::vector<ObjectId> marqueeSelect(const Document& doc, const ViewTransform& view, const Box2& screenRect,
                                    MarqueeMode mode, FillHit fill) {
    return PickIndex(doc).marquee(view, screenRect, mode, fill);
}

void applySelection(document::Selection& selection, const std::vector<ObjectId>& ids, SelectMode mode) {
    switch (mode) {
        case SelectMode::Replace:
            selection.assign(ids);
            break;
        case SelectMode::Add:
            for (const ObjectId id : ids) {
                selection.add(id);
            }
            break;
        case SelectMode::Toggle:
            for (const ObjectId id : ids) {
                selection.toggle(id);
            }
            break;
        case SelectMode::Subtract:
            for (const ObjectId id : ids) {
                selection.remove(id);
            }
            break;
    }
}

std::optional<Hit> clickSelect(Document& doc, const ViewTransform& view, Vec2 screenPx, SelectMode mode,
                               const HitOptions& options) {
    const std::optional<Hit> hit = PickIndex(doc).hitTest(view, screenPx, options);
    if (hit.has_value()) {
        applySelection(doc.selection(), {hit->object}, mode);
    } else if (mode == SelectMode::Replace) {
        doc.selection().clear();
    }
    return hit;
}

std::vector<ObjectId> marqueeApply(Document& doc, const ViewTransform& view, const Box2& screenRect,
                                   MarqueeMode marqueeMode, SelectMode mode, FillHit fill) {
    const std::vector<ObjectId> ids = PickIndex(doc).marquee(view, screenRect, marqueeMode, fill);
    applySelection(doc.selection(), ids, mode);
    return ids;
}

void selectAll(Document& doc) {
    doc.selection().assign(PickIndex(doc).topLevelObjects());
}

void invertSelection(Document& doc) {
    std::vector<ObjectId> next;
    for (const ObjectId id : PickIndex(doc).topLevelObjects()) {
        if (!doc.selection().contains(id)) {
            next.push_back(id);
        }
    }
    doc.selection().assign(next);
}

void selectByLayer(Document& doc, document::LayerId layer) {
    const PickIndex index(doc);
    std::vector<ObjectId> ids;
    for (const ObjectId top : index.topLevelObjects()) {
        for (const ObjectId id : doc.withDescendants({top})) {
            const Object* obj = doc.findObject(id);
            if (obj != nullptr && !obj->isGroup() && obj->layer == layer && chainPickable(doc, *obj)) {
                ids.push_back(top);
                break;
            }
        }
    }
    doc.selection().assign(ids);
}

std::optional<Box2> selectionBounds(const Document& doc) {
    Box2 all;
    for (const ObjectId id : doc.selection().ids()) {
        if (const auto b = doc.objectBounds(id)) {
            all.expand(*b);
        }
    }
    if (all.isEmpty()) {
        return std::nullopt;
    }
    return all;
}

SelectionBox selectionBox(const Box2& boundsMm, const ViewTransform& view) {
    SelectionBox box;
    box.boundsMm = boundsMm;
    if (boundsMm.isEmpty()) {
        return box;
    }
    const double xs[3] = {boundsMm.min.x, 0.5 * (boundsMm.min.x + boundsMm.max.x), boundsMm.max.x};
    const double ys[3] = {boundsMm.max.y, 0.5 * (boundsMm.min.y + boundsMm.max.y), boundsMm.min.y};  // top first
    std::size_t k = 0;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            Handle& h = box.handles[k];
            h.kind = static_cast<HandleKind>(k);
            h.positionMm = {xs[col], ys[row]};
            h.positionPx = view.worldToScreen(h.positionMm);
            ++k;
        }
    }
    return box;
}

std::optional<HandleKind> hitHandle(const SelectionBox& box, Vec2 screenPx, double tolerancePx) {
    if (box.boundsMm.isEmpty()) {
        return std::nullopt;
    }
    std::optional<HandleKind> best;
    double bestD = std::numeric_limits<double>::infinity();
    for (const Handle& h : box.handles) {
        const double d = geometry::distance(h.positionPx, screenPx);
        if (d <= tolerancePx && d < bestD) {
            bestD = d;
            best = h.kind;
        }
    }
    return best;
}

}  // namespace carvetto::viewmodel
