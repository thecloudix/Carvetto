#include "carvetto/viewmodel/node_edit.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <map>
#include <memory>
#include <set>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/edit_ops.hpp"
#include "carvetto/viewmodel/selection_tools.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::Object;
using document::ObjectId;
using geometry::Contour;
using geometry::Span;
using geometry::Vec2;

namespace {

using ContourKey = std::pair<ObjectId, std::size_t>;

bool layerOk(const Document& doc, document::LayerId id) {
    const document::Layer* layer = doc.findLayer(id);
    return layer != nullptr && layer->visible && !layer->locked;
}

bool chainOk(const Document& doc, const Object& obj) {
    const Object* cur = &obj;
    for (std::size_t guard = 0; guard <= doc.objectCount(); ++guard) {
        if (!layerOk(doc, cur->layer)) {
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

NodeEditResult fail(std::string message) {
    NodeEditResult r;
    r.ok = false;
    r.message = std::move(message);
    return r;
}

NodeEditResult success(bool changed = false) {
    NodeEditResult r;
    r.ok = true;
    r.changed = changed;
    return r;
}

bool finite(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

int sideOf(double v, double centre) {
    const double d = v - centre;
    return std::fabs(d) < 1e-9 ? 0 : (d < 0.0 ? -1 : 1);
}

/// Displacement of a selected node when the grabbed node moves by `d` under a mirror mode: nodes on
/// the opposite side of the axis through `centre` move mirrored, nodes on the axis stay on it.
Vec2 mirroredDisplacement(MirrorMode mode, Vec2 d, Vec2 node, Vec2 grabbed, Vec2 centre) {
    if (mode == MirrorMode::Off) {
        return d;
    }
    const bool horizontal = mode == MirrorMode::Horizontal;  // left / right symmetry about x = centre.x
    const int sn = horizontal ? sideOf(node.x, centre.x) : sideOf(node.y, centre.y);
    const int sg = horizontal ? sideOf(grabbed.x, centre.x) : sideOf(grabbed.y, centre.y);
    if (sg == 0) {
        return d;
    }
    if (sn == 0) {
        return horizontal ? Vec2{0.0, d.y} : Vec2{d.x, 0.0};
    }
    if (sn == sg) {
        return d;
    }
    return horizontal ? Vec2{-d.x, d.y} : Vec2{d.x, -d.y};
}

Vec2 boxCentre(const std::vector<Vec2>& pts) {
    geometry::Box2 box;
    for (const Vec2& p : pts) {
        box.expand(p);
    }
    return box.isEmpty() ? Vec2{} : box.centre();
}

}  // namespace

std::optional<NodeOp> opForKey(char key) {
    switch (std::tolower(static_cast<unsigned char>(key))) {
        case 'i': return NodeOp::InsertPoint;
        case 'd': return NodeOp::DeletePoint;
        case 's': return NodeOp::Smooth;
        case 'c': return NodeOp::CutVector;
        case 'b': return NodeOp::ConvertBezier;
        case 'a': return NodeOp::ConvertArc;
        case 'l': return NodeOp::ConvertLine;
        case 'x': return NodeOp::MatchX;
        case 'y': return NodeOp::MatchY;
        case 'j': return NodeOp::Join;
        default: return std::nullopt;
    }
}

const char* opName(NodeOp op) {
    switch (op) {
        case NodeOp::ConvertLine: return "Convert to Line";
        case NodeOp::ConvertBezier: return "Convert to Bezier";
        case NodeOp::ConvertArc: return "Convert to Arc";
        case NodeOp::InsertPoint: return "Insert Point";
        case NodeOp::InsertMidpoint: return "Insert Midpoint";
        case NodeOp::DeletePoint: return "Delete Point";
        case NodeOp::DeleteSpan: return "Delete Span";
        case NodeOp::CutVector: return "Cut Vector";
        case NodeOp::Smooth: return "Smooth";
        case NodeOp::Reverse: return "Reverse";
        case NodeOp::Close: return "Close";
        case NodeOp::Join: return "Join";
        case NodeOp::StartPoint: return "Start Point";
        case NodeOp::Extend: return "Extend";
        case NodeOp::MatchX: return "Match X";
        case NodeOp::MatchY: return "Match Y";
    }
    return "";
}

NodeEditor::NodeEditor(Document* doc) : doc_(doc) {}

void NodeEditor::setDocument(Document* doc) {
    doc_ = doc;
    nodes_.clear();
    spans_.clear();
    pressed_ = false;
    dragged_ = false;
    marquee_ = false;
    pressHit_.reset();
    gesture_ = Gesture::None;
    gestureObjects_.clear();
    snapshots_.clear();
    lastApplied_.clear();
    pushed_ = false;
}

bool NodeEditor::eligible(const Object& obj) const {
    return doc_ != nullptr && obj.isVector() && obj.sheet == doc_->activeSheet() && chainOk(*doc_, obj);
}

std::vector<ObjectId> NodeEditor::editedObjects() const {
    std::vector<ObjectId> out;
    if (doc_ == nullptr) {
        return out;
    }
    std::set<ObjectId> seen;
    // Iterative depth-first walk keeping selection order; the depth bound guards corrupt cycles.
    struct Item {
        ObjectId id;
        std::size_t depth;
    };
    std::vector<Item> stack;
    const auto& sel = doc_->selection().ids();
    for (auto it = sel.rbegin(); it != sel.rend(); ++it) {
        stack.push_back({*it, 0});
    }
    while (!stack.empty()) {
        const Item item = stack.back();
        stack.pop_back();
        const Object* obj = doc_->findObject(item.id);
        if (obj == nullptr) {
            continue;
        }
        if (const auto* group = obj->asGroup()) {
            if (item.depth < 64) {
                for (auto it = group->children.rbegin(); it != group->children.rend(); ++it) {
                    stack.push_back({*it, item.depth + 1});
                }
            }
        } else if (eligible(*obj) && seen.insert(obj->id).second) {
            out.push_back(obj->id);
        }
    }
    return out;
}

namespace {

/// The contour as the editor sees it: full circles split into two half arcs.
const Contour& modelOf(const Contour& c, Contour& store) {
    if (nodeops::isNormalised(c)) {
        return c;
    }
    store = nodeops::normalised(c);
    return store;
}

std::vector<Contour> modelContours(const Object& obj) {
    std::vector<Contour> out;
    if (const auto* vec = obj.asVector()) {
        out.reserve(vec->contours.size());
        for (const Contour& c : vec->contours) {
            out.push_back(nodeops::normalised(c));
        }
    }
    return out;
}

}  // namespace

bool NodeEditor::pruneSelection() const {
    if (doc_ == nullptr) {
        nodes_.clear();
        spans_.clear();
        return false;
    }
    const std::vector<ObjectId> edited = editedObjects();
    const std::set<ObjectId> editedSet(edited.begin(), edited.end());
    auto valid = [&](ObjectId id, std::size_t contour, std::size_t position, bool isNode) {
        if (!editedSet.contains(id)) {
            return false;
        }
        const Object* obj = doc_->findObject(id);
        const auto* vec = obj != nullptr ? obj->asVector() : nullptr;
        if (vec == nullptr || contour >= vec->contours.size()) {
            return false;
        }
        Contour store;
        const Contour& c = modelOf(vec->contours[contour], store);
        return isNode ? position < nodeops::nodeCount(c) : position < c.spans.size();
    };
    std::erase_if(nodes_, [&](const NodeRef& r) { return !valid(r.object, r.contour, r.node, true); });
    std::erase_if(spans_, [&](const SpanRef& r) { return !valid(r.object, r.contour, r.span, false); });
    return true;
}

std::vector<NodeRef> NodeEditor::selectedNodes() const {
    pruneSelection();
    return nodes_;
}

std::vector<SpanRef> NodeEditor::selectedSpans() const {
    pruneSelection();
    return spans_;
}

void NodeEditor::clearSelection() {
    nodes_.clear();
    spans_.clear();
}

void NodeEditor::selectNode(const NodeRef& ref, bool add) {
    spans_.clear();
    if (!add) {
        nodes_.clear();
    }
    if (std::find(nodes_.begin(), nodes_.end(), ref) == nodes_.end()) {
        nodes_.push_back(ref);
    }
}

void NodeEditor::selectSpan(const SpanRef& ref, bool add) {
    nodes_.clear();
    if (!add) {
        spans_.clear();
    }
    if (std::find(spans_.begin(), spans_.end(), ref) == spans_.end()) {
        spans_.push_back(ref);
    }
}

void NodeEditor::selectAllNodes() {
    spans_.clear();
    nodes_.clear();
    if (doc_ == nullptr) {
        return;
    }
    for (const ObjectId id : editedObjects()) {
        const auto* vec = doc_->findObject(id)->asVector();
        for (std::size_t ci = 0; ci < vec->contours.size(); ++ci) {
            Contour store;
            const Contour& c = modelOf(vec->contours[ci], store);
            for (std::size_t n = 0; n < nodeops::nodeCount(c); ++n) {
                nodes_.push_back({id, ci, n});
            }
        }
    }
}

std::optional<Vec2> NodeEditor::nodePosition(const NodeRef& ref) const {
    if (doc_ == nullptr) {
        return std::nullopt;
    }
    const Object* obj = doc_->findObject(ref.object);
    const auto* vec = obj != nullptr ? obj->asVector() : nullptr;
    if (vec == nullptr || ref.contour >= vec->contours.size()) {
        return std::nullopt;
    }
    Contour store;
    const Contour& c = modelOf(vec->contours[ref.contour], store);
    if (ref.node >= nodeops::nodeCount(c)) {
        return std::nullopt;
    }
    return nodeops::nodePoint(c, ref.node);
}

std::vector<NodeHandle> NodeEditor::handles() const {
    std::vector<NodeHandle> out;
    if (doc_ == nullptr) {
        return out;
    }
    pruneSelection();
    const std::set<NodeRef> selected(nodes_.begin(), nodes_.end());
    const std::set<SpanRef> selectedSp(spans_.begin(), spans_.end());
    for (const ObjectId id : editedObjects()) {
        const auto* vec = doc_->findObject(id)->asVector();
        for (std::size_t ci = 0; ci < vec->contours.size(); ++ci) {
            Contour store;
            const Contour& c = modelOf(vec->contours[ci], store);
            for (std::size_t n = 0; n < nodeops::nodeCount(c); ++n) {
                NodeHandle h;
                h.type = HandleType::Node;
                h.node = {id, ci, n};
                h.positionMm = nodeops::nodePoint(c, n);
                h.selected = selected.contains(h.node);
                out.push_back(h);
            }
            for (std::size_t s = 0; s < c.spans.size(); ++s) {
                const SpanRef sref{id, ci, s};
                NodeHandle mid;
                mid.type = HandleType::Midpoint;
                mid.span = sref;
                mid.positionMm = nodeops::midHandle(c.spans[s]);
                mid.selected = selectedSp.contains(sref);
                out.push_back(mid);
                if (const auto* b = std::get_if<geometry::BezierSpan>(&c.spans[s])) {
                    for (int which = 0; which < 2; ++which) {
                        NodeHandle h;
                        h.type = HandleType::Control;
                        h.span = sref;
                        h.control = which;
                        h.positionMm = which == 0 ? b->p1 : b->p2;
                        h.anchorMm = which == 0 ? b->p0 : b->p3;
                        out.push_back(h);
                    }
                }
            }
            if (out.size() > kMaxHandles) {
                return out;
            }
        }
    }
    return out;
}

std::optional<NodeHit> NodeEditor::hitTest(Vec2 px) const {
    if (doc_ == nullptr) {
        return std::nullopt;
    }
    const double tol = options_.hitTolerancePx;
    const double scale = view_.scale();
    const Vec2 cursor = view_.screenToWorld(px);
    const double tolMm = tol / scale;
    std::optional<NodeHit> bestNode;
    std::optional<NodeHit> bestControl;
    std::optional<NodeHit> bestMid;
    std::optional<NodeHit> bestSpan;
    auto consider = [&](std::optional<NodeHit>& best, NodeHit hit) {
        if (hit.distancePx <= tol && (!best.has_value() || hit.distancePx < best->distancePx)) {
            best = hit;
        }
    };
    for (const ObjectId id : editedObjects()) {
        const auto* vec = doc_->findObject(id)->asVector();
        for (std::size_t ci = 0; ci < vec->contours.size(); ++ci) {
            Contour store;
            const Contour& c = modelOf(vec->contours[ci], store);
            if (!c.bounds().inflated(tolMm).contains(cursor)) {
                // A Bezier's control points can lie outside its curve bounds: widen by them below.
                bool nearHull = false;
                for (const Span& s : c.spans) {
                    if (const auto* b = std::get_if<geometry::BezierSpan>(&s)) {
                        geometry::Box2 hull{b->p0, b->p3};
                        hull.expand(b->p1);
                        hull.expand(b->p2);
                        nearHull = nearHull || hull.inflated(tolMm).contains(cursor);
                    }
                }
                if (!nearHull) {
                    continue;
                }
            }
            for (std::size_t n = 0; n < nodeops::nodeCount(c); ++n) {
                NodeHit hit;
                hit.type = HandleType::Node;
                hit.node = {id, ci, n};
                hit.positionMm = nodeops::nodePoint(c, n);
                hit.distancePx = geometry::distance(hit.positionMm, cursor) * scale;
                consider(bestNode, hit);
            }
            for (std::size_t s = 0; s < c.spans.size(); ++s) {
                const SpanRef sref{id, ci, s};
                if (const auto* b = std::get_if<geometry::BezierSpan>(&c.spans[s])) {
                    for (int which = 0; which < 2; ++which) {
                        NodeHit hit;
                        hit.type = HandleType::Control;
                        hit.span = sref;
                        hit.control = which;
                        hit.positionMm = which == 0 ? b->p1 : b->p2;
                        hit.distancePx = geometry::distance(hit.positionMm, cursor) * scale;
                        consider(bestControl, hit);
                    }
                }
                NodeHit mid;
                mid.type = HandleType::Midpoint;
                mid.span = sref;
                mid.positionMm = nodeops::midHandle(c.spans[s]);
                mid.distancePx = geometry::distance(mid.positionMm, cursor) * scale;
                consider(bestMid, mid);
                if (geometry::bounds(c.spans[s]).inflated(tolMm).contains(cursor)) {
                    const geometry::ClosestPoint cp = geometry::closestPoint(c.spans[s], cursor);
                    NodeHit hit;
                    hit.type = HandleType::Span;
                    hit.span = sref;
                    hit.positionMm = cp.point;
                    hit.t = cp.t;
                    hit.distancePx = cp.distance * scale;
                    consider(bestSpan, hit);
                }
            }
        }
    }
    if (bestNode) {
        return bestNode;
    }
    if (bestControl) {
        return bestControl;
    }
    if (bestMid) {
        return bestMid;
    }
    return bestSpan;
}

std::optional<geometry::Box2> NodeEditor::marqueePx() const {
    if (!pressed_ || !marquee_ || !dragged_) {
        return std::nullopt;
    }
    geometry::Box2 box{pressPx_, curPx_};
    return box;
}

// ------------------------------------------------------------------------------- pointer events

void NodeEditor::validateGesture() {
    if (gesture_ == Gesture::None) {
        return;
    }
    if (doc_ == nullptr || doc_->undoStack().generation() != generation_ || doc_->activeSheet() != sheet_) {
        // Somebody else edited (or replaced) the document during the drag: it is over, and our
        // snapshots are stale. Nothing is reverted.
        gesture_ = Gesture::None;
        pressed_ = false;
        dragged_ = false;
        pushed_ = false;
        gestureObjects_.clear();
        snapshots_.clear();
        lastApplied_.clear();
    }
}

NodeEditResult NodeEditor::press(Vec2 px, Vec2 snappedMm, bool shift, bool ctrl, bool alt) {
    (void)snappedMm;
    (void)ctrl;
    (void)alt;
    validateGesture();
    if (doc_ == nullptr) {
        return fail("no document");
    }
    NodeEditResult result = success();
    pressed_ = true;
    dragged_ = false;
    marquee_ = false;
    pressPx_ = px;
    curPx_ = px;
    pressShift_ = shift;
    pressHit_.reset();
    pressedNodeWasSelected_ = false;
    pruneSelection();
    const std::optional<NodeHit> hit = hitTest(px);
    if (hit.has_value()) {
        pressHit_ = hit;
        switch (hit->type) {
            case HandleType::Node: {
                const auto found = std::find(nodes_.begin(), nodes_.end(), hit->node);
                pressedNodeWasSelected_ = found != nodes_.end();
                if (shift) {
                    spans_.clear();
                    if (found != nodes_.end()) {
                        nodes_.erase(found);
                    } else {
                        nodes_.push_back(hit->node);
                    }
                } else if (!pressedNodeWasSelected_) {
                    selectNode(hit->node, false);
                }
                break;
            }
            case HandleType::Span: {
                const auto found = std::find(spans_.begin(), spans_.end(), hit->span);
                if (shift) {
                    nodes_.clear();
                    if (found != spans_.end()) {
                        spans_.erase(found);
                    } else {
                        spans_.push_back(hit->span);
                    }
                } else if (found == spans_.end()) {
                    selectSpan(hit->span, false);
                }
                break;
            }
            case HandleType::Control:
            case HandleType::Midpoint:
                break;
        }
        return result;
    }
    // Not on a handle: another object, or empty canvas (rectangle selection).
    const std::optional<Hit> object = viewmodel::hitTest(*doc_, view_, px);
    if (object.has_value()) {
        applySelection(doc_->selection(), {object->object}, shift ? SelectMode::Add : SelectMode::Replace);
        clearSelection();
        result.selectionChanged = true;
        return result;
    }
    marquee_ = true;
    return result;
}

NodeEditResult NodeEditor::move(Vec2 px, Vec2 snappedMm, bool shift, bool ctrl, bool alt) {
    (void)shift;
    (void)alt;
    validateGesture();
    curPx_ = px;
    if (!pressed_ || doc_ == nullptr) {
        return success();
    }
    if (!dragged_) {
        if (geometry::distance(px, pressPx_) < kDragThresholdPx) {
            return success();
        }
        dragged_ = true;
        if (pressHit_.has_value() && !marquee_) {
            beginGesture(snappedMm, ctrl);
        }
    }
    if (gesture_ != Gesture::None) {
        applyGesture(snappedMm);
    }
    return success();
}

NodeEditResult NodeEditor::release(Vec2 px, Vec2 snappedMm, bool shift, bool ctrl, bool alt) {
    (void)ctrl;
    (void)alt;
    validateGesture();
    curPx_ = px;
    NodeEditResult result = success();
    if (!pressed_ || doc_ == nullptr) {
        pressed_ = false;
        return result;
    }
    if (gesture_ != Gesture::None) {
        applyGesture(snappedMm);
        result = finishGesture();
    } else if (marquee_) {
        if (dragged_) {
            const geometry::Box2 rect{pressPx_, px};
            std::vector<NodeRef> inside;
            for (const NodeHandle& h : handles()) {
                if (h.type == HandleType::Node && rect.contains(view_.worldToScreen(h.positionMm))) {
                    inside.push_back(h.node);
                }
            }
            spans_.clear();
            if (!shift) {
                nodes_.clear();
            }
            for (const NodeRef& r : inside) {
                if (std::find(nodes_.begin(), nodes_.end(), r) == nodes_.end()) {
                    nodes_.push_back(r);
                }
            }
        } else if (!shift) {
            clearSelection();
        }
    } else if (pressHit_.has_value() && !dragged_ && pressHit_->type == HandleType::Node && !shift &&
               pressedNodeWasSelected_ && nodes_.size() > 1) {
        selectNode(pressHit_->node, false);  // a click on one of several selected nodes keeps just it
    }
    pressed_ = false;
    dragged_ = false;
    marquee_ = false;
    pressHit_.reset();
    return result;
}

bool NodeEditor::escape(NodeEditResult* result) {
    const bool inProgress = pressed_ || gesture_ != Gesture::None;
    if (!inProgress) {
        return false;
    }
    NodeEditResult r = success();
    if (gesture_ != Gesture::None && pushed_ && doc_ != nullptr && doc_->undoStack().generation() == generation_) {
        r.changed = doc_->undoStack().undoAndDiscard(*doc_);
    }
    gesture_ = Gesture::None;
    pressed_ = false;
    dragged_ = false;
    marquee_ = false;
    pushed_ = false;
    pressHit_.reset();
    gestureObjects_.clear();
    snapshots_.clear();
    lastApplied_.clear();
    if (result != nullptr) {
        *result = r;
    }
    return true;
}

// ------------------------------------------------------------------------------------- gestures

bool NodeEditor::beginGesture(const Vec2& snappedMm, bool ctrl) {
    (void)snappedMm;
    if (!pressHit_.has_value() || doc_ == nullptr) {
        return false;
    }
    const NodeHit& hit = *pressHit_;
    snapshots_.clear();
    lastApplied_.clear();
    gestureObjects_.clear();
    dragNodes_.clear();
    dragStart_.clear();
    insertedOnBegin_ = false;
    pushed_ = false;
    auto snapshotOf = [&](ObjectId id) {
        for (const Snapshot& s : snapshots_) {
            if (s.id == id) {
                return;
            }
        }
        const Object* obj = doc_->findObject(id);
        if (obj == nullptr || !obj->isVector()) {
            return;
        }
        snapshots_.push_back({id, modelContours(*obj)});
    };
    auto snapshotContours = [&](ObjectId id) -> std::vector<Contour>* {
        for (Snapshot& s : snapshots_) {
            if (s.id == id) {
                return &s.contours;
            }
        }
        return nullptr;
    };
    Gesture kind = Gesture::None;
    switch (hit.type) {
        case HandleType::Node: {
            pruneSelection();
            if (std::find(nodes_.begin(), nodes_.end(), hit.node) == nodes_.end()) {
                return false;
            }
            kind = Gesture::Nodes;
            dragNodes_ = nodes_;
            grabbedNode_ = hit.node;
            for (const NodeRef& n : dragNodes_) {
                snapshotOf(n.object);
            }
            for (const NodeRef& n : dragNodes_) {
                const std::vector<Contour>* cs = snapshotContours(n.object);
                if (cs == nullptr || n.contour >= cs->size() || n.node >= nodeops::nodeCount((*cs)[n.contour])) {
                    return false;
                }
                dragStart_.push_back(nodeops::nodePoint((*cs)[n.contour], n.node));
                if (n == grabbedNode_) {
                    grabMm_ = dragStart_.back();
                }
            }
            centreMm_ = boxCentre(dragStart_);
            break;
        }
        case HandleType::Control: {
            kind = Gesture::Control;
            dragSpan_ = hit.span;
            dragControl_ = hit.control;
            grabMm_ = hit.positionMm;
            snapshotOf(hit.span.object);
            break;
        }
        case HandleType::Midpoint: {
            snapshotOf(hit.span.object);
            std::vector<Contour>* cs = snapshotContours(hit.span.object);
            if (cs == nullptr || hit.span.contour >= cs->size()) {
                return false;
            }
            const auto inserted = nodeops::insertPoint((*cs)[hit.span.contour], hit.span.span, 0.5);
            if (!inserted.has_value()) {
                return false;
            }
            (*cs)[hit.span.contour] = *inserted;
            kind = Gesture::Nodes;
            insertedOnBegin_ = true;
            grabbedNode_ = {hit.span.object, hit.span.contour, hit.span.span + 1};
            dragNodes_ = {grabbedNode_};
            grabMm_ = nodeops::nodePoint(*inserted, grabbedNode_.node);
            dragStart_ = {grabMm_};
            centreMm_ = grabMm_;
            nodes_ = {grabbedNode_};
            spans_.clear();
            break;
        }
        case HandleType::Span: {
            kind = ctrl ? Gesture::SpanMove : Gesture::SpanReshape;
            dragSpan_ = hit.span;
            dragT_ = hit.t;
            grabMm_ = hit.positionMm;
            snapshotOf(hit.span.object);
            break;
        }
    }
    if (kind == Gesture::None || snapshots_.empty()) {
        snapshots_.clear();
        return false;
    }
    for (const Snapshot& s : snapshots_) {
        gestureObjects_.push_back(s.id);
    }
    generation_ = doc_->undoStack().generation();
    sheet_ = doc_->activeSheet();
    gesture_ = kind;
    return true;
}

bool NodeEditor::applyGesture(Vec2 snappedMm) {
    if (gesture_ == Gesture::None || doc_ == nullptr || !finite(snappedMm)) {
        return false;
    }
    std::vector<std::vector<Contour>> next;
    next.reserve(snapshots_.size());
    for (const Snapshot& s : snapshots_) {
        next.push_back(s.contours);
    }
    auto indexOf = [&](ObjectId id) -> std::size_t {
        for (std::size_t i = 0; i < snapshots_.size(); ++i) {
            if (snapshots_[i].id == id) {
                return i;
            }
        }
        return snapshots_.size();
    };
    bool ok = true;
    switch (gesture_) {
        case Gesture::Nodes: {
            const Vec2 d = snappedMm - grabMm_;
            std::map<ContourKey, std::map<std::size_t, Vec2>> targets;
            for (std::size_t i = 0; i < dragNodes_.size(); ++i) {
                const Vec2 disp = mirroredDisplacement(options_.mirror, d, dragStart_[i], grabMm_, centreMm_);
                targets[{dragNodes_[i].object, dragNodes_[i].contour}][dragNodes_[i].node] = dragStart_[i] + disp;
            }
            for (auto& [key, map] : targets) {
                const std::size_t si = indexOf(key.first);
                ok = ok && si < next.size() && key.second < next[si].size() &&
                     nodeops::moveNodes(next[si][key.second], map);
            }
            break;
        }
        case Gesture::Control: {
            const std::size_t si = indexOf(dragSpan_.object);
            ok = si < next.size() && dragSpan_.contour < next[si].size() &&
                 nodeops::setControlPoint(next[si][dragSpan_.contour], dragSpan_.span, dragControl_, snappedMm,
                                          options_.keepBezierTangency);
            break;
        }
        case Gesture::SpanReshape:
        case Gesture::SpanMove: {
            const std::size_t si = indexOf(dragSpan_.object);
            const Vec2 delta = snappedMm - grabMm_;
            ok = si < next.size() && dragSpan_.contour < next[si].size();
            if (ok) {
                ok = gesture_ == Gesture::SpanMove
                         ? nodeops::moveSpan(next[si][dragSpan_.contour], dragSpan_.span, delta)
                         : nodeops::reshapeSpan(next[si][dragSpan_.contour], dragSpan_.span, dragT_, delta);
            }
            break;
        }
        case Gesture::None:
            return false;
    }
    if (!ok) {
        return false;  // keep the last valid step
    }
    if (pushed_ ? next == lastApplied_ : (!insertedOnBegin_ && [&] {
            for (std::size_t i = 0; i < snapshots_.size(); ++i) {
                if (!(next[i] == snapshots_[i].contours)) {
                    return false;
                }
            }
            return true;
        }())) {
        return false;
    }
    std::vector<document::ContourReplacement> items;
    for (std::size_t i = 0; i < snapshots_.size(); ++i) {
        items.push_back({snapshots_[i].id, next[i]});
    }
    if (!pushed_) {
        doc_->undoStack().breakCoalescing();
    }
    if (!doc_->execute(std::make_unique<document::SetObjectContoursCommand>(std::move(items), "Edit nodes"))) {
        return false;
    }
    pushed_ = true;
    lastApplied_ = std::move(next);
    generation_ = doc_->undoStack().generation();
    return true;
}

NodeEditResult NodeEditor::finishGesture() {
    NodeEditResult r = success();
    if (pushed_ && doc_ != nullptr) {
        doc_->undoStack().breakCoalescing();
        r.changed = true;
    }
    gesture_ = Gesture::None;
    pushed_ = false;
    gestureObjects_.clear();
    snapshots_.clear();
    lastApplied_.clear();
    return r;
}

// ------------------------------------------------------------------------------------ operations

namespace {

/// Working copy of the edited objects' contours (normalised), by object.
struct Work {
    std::map<ObjectId, std::vector<Contour>> contours;
    std::set<ObjectId> touched;

    std::vector<Contour>& of(ObjectId id) { return contours.at(id); }
};

void spansOf(const std::vector<SpanRef>& explicitSpans, const std::vector<NodeRef>& nodes,
             const std::map<ContourKey, std::pair<bool, std::size_t>>& shape, std::set<SpanRef>* out) {
    for (const SpanRef& s : explicitSpans) {
        out->insert(s);
    }
    // A span whose both end nodes are selected counts as selected.
    std::set<NodeRef> set(nodes.begin(), nodes.end());
    for (const NodeRef& n : nodes) {
        const auto it = shape.find({n.object, n.contour});
        if (it == shape.end()) {
            continue;
        }
        const bool closed = it->second.first;
        const std::size_t spanCount = it->second.second;
        if (n.node < spanCount) {
            const std::size_t next = closed ? (n.node + 1) % spanCount : n.node + 1;
            if (next != n.node && set.contains({n.object, n.contour, next})) {
                out->insert({n.object, n.contour, n.node});
            }
        }
    }
}

}  // namespace

NodeEditResult NodeEditor::commitContours(std::vector<document::ContourReplacement> items,
                                          const std::vector<ObjectId>& removals, const char* text) {
    if (doc_ == nullptr) {
        return fail("no document");
    }
    document::UndoStack& stack = doc_->undoStack();
    stack.breakCoalescing();
    bool ok = true;
    if (removals.empty()) {
        ok = doc_->execute(std::make_unique<document::SetObjectContoursCommand>(std::move(items), text));
    } else {
        stack.beginMacro(text);
        if (!items.empty()) {
            ok = doc_->execute(std::make_unique<document::SetObjectContoursCommand>(std::move(items), text));
        }
        if (ok) {
            ok = doc_->execute(std::make_unique<document::RemoveObjectsCommand>(removals));
        }
        stack.endMacro();
    }
    stack.breakCoalescing();
    if (!ok) {
        return fail("the edit could not be applied");
    }
    for (const ObjectId id : removals) {
        doc_->selection().remove(id);
    }
    return success(true);
}

bool NodeEditor::canRun(NodeOp op, std::string* why) const {
    auto no = [&](const char* text) {
        if (why != nullptr) {
            *why = text;
        }
        return false;
    };
    if (doc_ == nullptr) {
        return no("there is no document");
    }
    if (gesture_ != Gesture::None) {
        return no("finish the drag first");
    }
    pruneSelection();
    const std::vector<ObjectId> edited = editedObjects();
    if (edited.empty()) {
        return no("select an unlocked, visible vector on the active sheet");
    }
    std::map<ContourKey, std::pair<bool, std::size_t>> shape;
    auto shapeOf = [&](ObjectId id, std::size_t ci) -> const Contour* {
        const Object* obj = doc_->findObject(id);
        const auto* vec = obj != nullptr ? obj->asVector() : nullptr;
        return vec != nullptr && ci < vec->contours.size() ? &vec->contours[ci] : nullptr;
    };
    for (const NodeRef& n : nodes_) {
        if (const Contour* c = shapeOf(n.object, n.contour)) {
            Contour store;
            const Contour& m = modelOf(*c, store);
            shape[{n.object, n.contour}] = {m.closed, m.spans.size()};
        }
    }
    std::set<SpanRef> spans;
    spansOf(spans_, nodes_, shape, &spans);
    switch (op) {
        case NodeOp::ConvertLine:
        case NodeOp::ConvertBezier:
        case NodeOp::ConvertArc:
        case NodeOp::InsertPoint:
        case NodeOp::InsertMidpoint:
        case NodeOp::DeleteSpan:
            return spans.empty() ? no("select a span (or two neighbouring nodes) first") : true;
        case NodeOp::DeletePoint:
        case NodeOp::Smooth:
            return nodes_.empty() ? no("select a node first") : true;
        case NodeOp::CutVector:
        case NodeOp::Reverse:
            return nodes_.empty() && spans.empty() ? no("select a node or span first") : true;
        case NodeOp::Close: {
            std::set<ContourKey> keys;
            for (const NodeRef& n : nodes_) {
                keys.insert({n.object, n.contour});
            }
            for (const SpanRef& s : spans_) {
                keys.insert({s.object, s.contour});
            }
            for (const ContourKey& k : keys) {
                const Contour* c = shapeOf(k.first, k.second);
                if (c != nullptr && !c->closed) {
                    return true;
                }
            }
            return no("select a node or span of an open vector");
        }
        case NodeOp::Join: {
            std::size_t open = 0;
            for (const ObjectId id : edited) {
                for (const Contour& c : doc_->findObject(id)->asVector()->contours) {
                    open += (c.closed || c.spans.empty()) ? 0U : 1U;
                }
            }
            return open >= 2 ? true : no("Join needs two open vectors among the selected objects");
        }
        case NodeOp::StartPoint: {
            for (const NodeRef& n : nodes_) {
                if (shape[{n.object, n.contour}].first) {
                    return true;
                }
            }
            return no("select a node of a closed vector");
        }
        case NodeOp::Extend: {
            for (const NodeRef& n : nodes_) {
                const auto& sh = shape[{n.object, n.contour}];
                if (!sh.first && (n.node == 0 || n.node == sh.second)) {
                    return true;
                }
            }
            return no("select the end node of an open vector");
        }
        case NodeOp::MatchX:
        case NodeOp::MatchY:
            return nodes_.size() >= 2 ? true : no("select two or more nodes (the first selected is the reference)");
    }
    return no("unknown operation");
}

NodeEditResult NodeEditor::run(NodeOp op, const NodeOpArgs& args) {
    std::string why;
    if (!canRun(op, &why)) {
        return fail(why);
    }
    const std::vector<ObjectId> edited = editedObjects();
    Work work;
    for (const ObjectId id : edited) {
        work.contours[id] = modelContours(*doc_->findObject(id));
    }
    const std::vector<NodeRef> nodes = nodes_;
    std::map<ContourKey, std::pair<bool, std::size_t>> shape;
    for (const NodeRef& n : nodes) {
        const Contour& c = work.of(n.object)[n.contour];
        shape[{n.object, n.contour}] = {c.closed, c.spans.size()};
    }
    std::set<SpanRef> spanSet;
    spansOf(spans_, nodes, shape, &spanSet);
    std::map<ContourKey, std::set<std::size_t>> spansBy;
    for (const SpanRef& s : spanSet) {
        spansBy[{s.object, s.contour}].insert(s.span);
    }
    std::map<ContourKey, std::set<std::size_t>> nodesBy;
    for (const NodeRef& n : nodes) {
        nodesBy[{n.object, n.contour}].insert(n.node);
    }
    std::vector<ObjectId> removals;
    enum class Sel { Keep, Clear, Set };
    Sel selPolicy = Sel::Clear;
    std::vector<NodeRef> newSelection;
    std::string why2;

    // Inserts points into the spans of one contour (descending); returns the new node indices.
    auto insertInto = [&](Contour& c, const std::set<std::size_t>& at, const std::optional<Vec2>& pos,
                          std::vector<std::size_t>* newNodes, std::string* reason) {
        std::vector<std::size_t> asc(at.begin(), at.end());
        for (auto it = asc.rbegin(); it != asc.rend(); ++it) {
            double t = 0.5;
            if (pos.has_value()) {
                t = geometry::closestPoint(c.spans[*it], *pos).t;
            }
            const auto r = nodeops::insertPoint(c, *it, t, reason);
            if (!r.has_value()) {
                return false;
            }
            c = *r;
        }
        for (std::size_t i = 0; i < asc.size(); ++i) {
            newNodes->push_back(asc[i] + 1 + i);
        }
        return true;
    };

    switch (op) {
        case NodeOp::ConvertLine:
        case NodeOp::ConvertBezier:
        case NodeOp::ConvertArc: {
            nodeops::ConvertSpec spec;
            spec.target = op == NodeOp::ConvertLine     ? nodeops::SpanTarget::Line
                          : op == NodeOp::ConvertBezier ? nodeops::SpanTarget::Bezier
                                                        : nodeops::SpanTarget::Arc;
            if (spanSet.size() == 1) {
                spec.through = args.throughMm;
                spec.bulge = args.bulge;
            }
            if ((spec.through.has_value() && !finite(*spec.through)) ||
                (spec.bulge.has_value() && !std::isfinite(*spec.bulge))) {
                return fail("the point or bulge is not a finite number");
            }
            bool any = false;
            bool structural = false;
            for (const auto& [key, set] : spansBy) {
                Contour& c = work.of(key.first)[key.second];
                for (auto it = set.rbegin(); it != set.rend(); ++it) {
                    std::size_t produced = 1;
                    const auto r = nodeops::convertSpan(c, *it, spec, &produced, &why2);
                    if (!r.has_value()) {
                        return fail(why2);
                    }
                    if (!(*r == c)) {
                        any = true;
                        work.touched.insert(key.first);
                    }
                    structural = structural || produced != 1;
                    c = *r;
                }
            }
            if (!any) {
                return fail("the span already has that shape");
            }
            selPolicy = structural ? Sel::Clear : Sel::Keep;
            break;
        }
        case NodeOp::InsertPoint:
        case NodeOp::InsertMidpoint: {
            std::optional<Vec2> pos;
            if (op == NodeOp::InsertPoint && args.positionMm.has_value() && spanSet.size() == 1) {
                if (!finite(*args.positionMm)) {
                    return fail("the position is not a finite number");
                }
                pos = args.positionMm;
            }
            for (const auto& [key, set] : spansBy) {
                std::vector<std::size_t> fresh;
                if (!insertInto(work.of(key.first)[key.second], set, pos, &fresh, &why2)) {
                    return fail(why2);
                }
                work.touched.insert(key.first);
                for (const std::size_t n : fresh) {
                    newSelection.push_back({key.first, key.second, n});
                }
            }
            selPolicy = Sel::Set;
            break;
        }
        case NodeOp::DeletePoint: {
            for (const auto& [key, set] : nodesBy) {
                Contour& c = work.of(key.first)[key.second];
                const auto r = nodeops::deleteNodes(c, set, &why2);
                if (!r.has_value()) {
                    return fail(why2);
                }
                c = *r;
                work.touched.insert(key.first);
            }
            break;
        }
        case NodeOp::DeleteSpan: {
            for (auto it = spansBy.rbegin(); it != spansBy.rend(); ++it) {
                std::vector<Contour>& list = work.of(it->first.first);
                const std::size_t ci = it->first.second;
                std::vector<Contour> pieces;
                if (it->second.size() < list[ci].spans.size()) {
                    pieces = nodeops::deleteSpans(list[ci], it->second);
                    if (pieces.empty()) {
                        return fail("those spans cannot be deleted");
                    }
                }
                list.erase(list.begin() + static_cast<std::ptrdiff_t>(ci));
                list.insert(list.begin() + static_cast<std::ptrdiff_t>(ci), pieces.begin(), pieces.end());
                work.touched.insert(it->first.first);
            }
            for (const ObjectId id : work.touched) {
                if (work.of(id).empty()) {
                    return fail("that would delete every span of the vector; delete the object instead");
                }
            }
            break;
        }
        case NodeOp::CutVector: {
            std::set<ContourKey> keys;
            for (const auto& [key, set] : nodesBy) {
                keys.insert(key);
            }
            for (const auto& [key, set] : spansBy) {
                keys.insert(key);
            }
            // Contours are replaced by their pieces, so go from the last contour backwards.
            for (auto it = keys.rbegin(); it != keys.rend(); ++it) {
                std::vector<Contour>& list = work.of(it->first);
                Contour c = list[it->second];
                std::set<std::size_t> cutNodes;
                std::vector<std::size_t> inserted;
                const auto sp = spansBy.find(*it);
                if (sp != spansBy.end()) {
                    std::optional<Vec2> pos;
                    if (args.positionMm.has_value() && spanSet.size() == 1 && finite(*args.positionMm)) {
                        pos = args.positionMm;
                    }
                    if (!insertInto(c, sp->second, pos, &inserted, &why2)) {
                        return fail(why2);
                    }
                    cutNodes.insert(inserted.begin(), inserted.end());
                }
                const auto np = nodesBy.find(*it);
                if (np != nodesBy.end()) {
                    // Spans of the original below a node's index were each split once.
                    std::vector<std::size_t> splitAt;
                    if (sp != spansBy.end()) {
                        splitAt.assign(sp->second.begin(), sp->second.end());
                    }
                    for (const std::size_t n : np->second) {
                        std::size_t shifted = n;
                        for (const std::size_t s : splitAt) {
                            if (s < n) {
                                ++shifted;
                            }
                        }
                        cutNodes.insert(shifted);
                    }
                }
                const std::vector<Contour> pieces = nodeops::cutAtNodes(c, cutNodes, &why2);
                if (pieces.empty()) {
                    return fail(why2);
                }
                list.erase(list.begin() + static_cast<std::ptrdiff_t>(it->second));
                list.insert(list.begin() + static_cast<std::ptrdiff_t>(it->second), pieces.begin(), pieces.end());
                work.touched.insert(it->first);
            }
            break;
        }
        case NodeOp::Smooth: {
            for (const auto& [key, set] : nodesBy) {
                Contour& c = work.of(key.first)[key.second];
                const auto r = nodeops::smoothNodes(c, set, &why2);
                if (!r.has_value()) {
                    return fail(why2);
                }
                c = *r;
                work.touched.insert(key.first);
            }
            break;
        }
        case NodeOp::Reverse: {
            std::set<ContourKey> keys;
            for (const auto& [key, set] : nodesBy) {
                keys.insert(key);
            }
            for (const auto& [key, set] : spansBy) {
                keys.insert(key);
            }
            for (const ContourKey& key : keys) {
                Contour& c = work.of(key.first)[key.second];
                c = c.reversed();
                work.touched.insert(key.first);
            }
            break;
        }
        case NodeOp::Close: {
            std::set<ContourKey> keys;
            for (const auto& [key, set] : nodesBy) {
                keys.insert(key);
            }
            for (const SpanRef& s : spans_) {
                keys.insert({s.object, s.contour});
            }
            bool any = false;
            for (const ContourKey& key : keys) {
                Contour& c = work.of(key.first)[key.second];
                if (c.closed) {
                    continue;
                }
                const geometry::CloseResult r = geometry::closeContour(c, options_.tolerance);
                if (r.status != geometry::EditStatus::Ok) {
                    return fail(geometry::toString(r.status));
                }
                if (!nodeops::validContour(r.contour, &why2)) {
                    return fail(why2);
                }
                c = r.contour;
                work.touched.insert(key.first);
                any = true;
            }
            if (!any) {
                return fail("the vector is already closed");
            }
            break;
        }
        case NodeOp::Join: {
            std::vector<Contour> inputs;
            std::vector<ContourKey> origin;
            for (const ObjectId id : edited) {
                const std::vector<Contour>& list = work.of(id);
                for (std::size_t ci = 0; ci < list.size(); ++ci) {
                    if (!list[ci].closed && !list[ci].spans.empty()) {
                        inputs.push_back(list[ci]);
                        origin.push_back({id, ci});
                    }
                }
            }
            const geometry::JoinResult joined = geometry::joinContours(inputs, options_.tolerance);
            if (joined.status != geometry::EditStatus::Ok) {
                return fail(geometry::toString(joined.status));
            }
            bool anyJoin = false;
            for (std::size_t k = 0; k < joined.contours.size(); ++k) {
                if (joined.sources[k].size() > 1 || joined.contours[k].closed) {
                    anyJoin = true;
                }
                if (!nodeops::validContour(joined.contours[k], &why2)) {
                    return fail(why2);
                }
            }
            if (!anyJoin) {
                return fail("no open ends lie within the tolerance of each other");
            }
            // Where each input goes: the chain's first source keeps the chain, the others vanish.
            std::map<ContourKey, const Contour*> placed;
            for (std::size_t k = 0; k < joined.contours.size(); ++k) {
                placed[origin[joined.sources[k].front()]] = &joined.contours[k];
            }
            std::set<ContourKey> consumed;
            for (const ContourKey& o : origin) {
                consumed.insert(o);
            }
            for (const ObjectId id : edited) {
                std::vector<Contour> rebuilt;
                const std::vector<Contour>& list = work.of(id);
                for (std::size_t ci = 0; ci < list.size(); ++ci) {
                    const ContourKey key{id, ci};
                    if (!consumed.contains(key)) {
                        rebuilt.push_back(list[ci]);
                    } else if (const auto p = placed.find(key); p != placed.end()) {
                        rebuilt.push_back(*p->second);
                    }
                }
                if (!(rebuilt == list)) {
                    if (rebuilt.empty()) {
                        removals.push_back(id);
                    } else {
                        work.contours[id] = std::move(rebuilt);
                        work.touched.insert(id);
                    }
                }
            }
            break;
        }
        case NodeOp::StartPoint: {
            for (const auto& [key, set] : nodesBy) {
                Contour& c = work.of(key.first)[key.second];
                if (!c.closed) {
                    continue;
                }
                if (set.size() != 1) {
                    return fail("select a single node per closed vector");
                }
                const auto r = nodeops::rotateStart(c, *set.begin(), &why2);
                if (!r.has_value()) {
                    return fail(why2);
                }
                c = *r;
                work.touched.insert(key.first);
                newSelection.push_back({key.first, key.second, 0});
            }
            selPolicy = Sel::Set;
            break;
        }
        case NodeOp::Extend: {
            std::vector<Contour> targets;
            std::vector<ContourKey> targetKeys;
            for (const auto& [id, obj] : doc_->objects()) {
                if (!eligible(obj)) {
                    continue;
                }
                const auto* vec = obj.asVector();
                for (std::size_t ci = 0; ci < vec->contours.size(); ++ci) {
                    targets.push_back(vec->contours[ci]);
                    targetKeys.push_back({id, ci});
                }
            }
            bool any = false;
            for (const auto& [key, set] : nodesBy) {
                Contour& c = work.of(key.first)[key.second];
                if (c.closed) {
                    continue;
                }
                for (const std::size_t n : set) {
                    if (n != 0 && n != c.spans.size()) {
                        continue;
                    }
                    std::vector<Contour> others;
                    for (std::size_t i = 0; i < targets.size(); ++i) {
                        if (targetKeys[i] != key) {
                            others.push_back(targets[i]);
                        }
                    }
                    const geometry::ExtendResult r = geometry::extendSpan(
                        c, n == 0 ? geometry::ContourEnd::Start : geometry::ContourEnd::End, others, options_.tolerance);
                    if (r.status != geometry::EditStatus::Ok) {
                        return fail(geometry::toString(r.status));
                    }
                    if (!nodeops::validContour(r.contour, &why2)) {
                        return fail(why2);
                    }
                    c = r.contour;
                    work.touched.insert(key.first);
                    any = true;
                }
            }
            if (!any) {
                return fail("select the end node of an open vector");
            }
            break;
        }
        case NodeOp::MatchX:
        case NodeOp::MatchY: {
            const auto first = nodePosition(nodes.front());
            if (!first.has_value()) {
                return fail("the reference node no longer exists");
            }
            std::map<ContourKey, std::map<std::size_t, Vec2>> targets;
            for (const NodeRef& n : nodes) {
                Vec2 p = nodeops::nodePoint(work.of(n.object)[n.contour], n.node);
                (op == NodeOp::MatchX ? p.x : p.y) = op == NodeOp::MatchX ? first->x : first->y;
                targets[{n.object, n.contour}][n.node] = p;
            }
            for (auto& [key, map] : targets) {
                Contour& c = work.of(key.first)[key.second];
                if (!nodeops::moveNodes(c, map, &why2)) {
                    return fail(why2);
                }
                work.touched.insert(key.first);
            }
            selPolicy = Sel::Keep;
            break;
        }
    }

    std::vector<document::ContourReplacement> items;
    for (const ObjectId id : work.touched) {
        // An object whose edit changed nothing (a zero-length move) is left alone, circles included.
        if (work.contours.at(id) == modelContours(*doc_->findObject(id)) && !std::count(removals.begin(), removals.end(), id)) {
            continue;
        }
        items.push_back({id, work.contours.at(id)});
    }
    if (items.empty() && removals.empty()) {
        return fail("nothing changed");
    }
    const NodeEditResult result = commitContours(std::move(items), removals, opName(op));
    if (!result.ok) {
        return result;
    }
    switch (selPolicy) {
        case Sel::Keep:
            break;
        case Sel::Clear:
            clearSelection();
            break;
        case Sel::Set:
            spans_.clear();
            nodes_ = newSelection;
            break;
    }
    return result;
}

NodeEditResult NodeEditor::moveSelectedNodes(Vec2 deltaMm) {
    if (!finite(deltaMm)) {
        return fail("the offset is not a finite number");
    }
    return setSelectedNodeCoordinates(deltaMm.x, deltaMm.y, true);
}

NodeEditResult NodeEditor::setSelectedNodeCoordinates(std::optional<double> xMm, std::optional<double> yMm,
                                                      bool relative) {
    if (doc_ == nullptr) {
        return fail("there is no document");
    }
    if (gesture_ != Gesture::None) {
        return fail("finish the drag first");
    }
    if ((xMm.has_value() && !std::isfinite(*xMm)) || (yMm.has_value() && !std::isfinite(*yMm))) {
        return fail("the value is not a finite number");
    }
    if (!xMm.has_value() && !yMm.has_value()) {
        return fail("enter an X or Y value");
    }
    pruneSelection();
    const std::vector<NodeRef> nodes = nodes_;
    if (nodes.empty()) {
        return fail("select a node first");
    }
    Work work;
    for (const NodeRef& n : nodes) {
        if (!work.contours.contains(n.object)) {
            work.contours[n.object] = modelContours(*doc_->findObject(n.object));
        }
    }
    std::vector<Vec2> start;
    for (const NodeRef& n : nodes) {
        start.push_back(nodeops::nodePoint(work.of(n.object)[n.contour], n.node));
    }
    const Vec2 centre = boxCentre(start);
    std::map<ContourKey, std::map<std::size_t, Vec2>> targets;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        Vec2 target = start[i];
        if (relative) {
            const Vec2 d{xMm.value_or(0.0), yMm.value_or(0.0)};
            target = start[i] + mirroredDisplacement(options_.mirror, d, start[i], start.front(), centre);
        } else {
            target.x = xMm.value_or(target.x);
            target.y = yMm.value_or(target.y);
        }
        targets[{nodes[i].object, nodes[i].contour}][nodes[i].node] = target;
    }
    std::string why;
    for (auto& [key, map] : targets) {
        if (!nodeops::moveNodes(work.of(key.first)[key.second], map, &why)) {
            return fail(why);
        }
        work.touched.insert(key.first);
    }
    std::vector<document::ContourReplacement> items;
    for (const ObjectId id : work.touched) {
        if (work.contours.at(id) == modelContours(*doc_->findObject(id))) {
            continue;  // zero-length move: nothing to record (a circle is not normalised for it)
        }
        items.push_back({id, work.contours.at(id)});
    }
    if (items.empty()) {
        return fail("the nodes are already there");
    }
    const NodeEditResult r = commitContours(std::move(items), {}, "Move nodes");
    return r.ok ? r : fail("the nodes are already there");
}

NodeEditResult NodeEditor::handleKey(char key) {
    const int k = std::tolower(static_cast<unsigned char>(key));
    NodeEditResult r;
    if (k == 'h' || k == 'v') {
        const MirrorMode want = k == 'h' ? MirrorMode::Horizontal : MirrorMode::Vertical;
        options_.mirror = options_.mirror == want ? MirrorMode::Off : want;
        r = success();
        r.handled = true;
        return r;
    }
    if (k == 'k') {
        options_.keepBezierTangency = !options_.keepBezierTangency;
        r = success();
        r.handled = true;
        return r;
    }
    if (k == 'p') {
        r = success();
        r.handled = true;
        r.openProperties = true;
        return r;
    }
    if (const auto op = opForKey(key)) {
        r = run(*op);
        r.handled = true;
        return r;
    }
    return r;
}

}  // namespace carvetto::viewmodel
