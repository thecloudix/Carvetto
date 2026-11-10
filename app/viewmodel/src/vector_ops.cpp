#include "carvetto/viewmodel/vector_ops.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <memory>
#include <numeric>
#include <unordered_set>
#include <utility>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/toolpath/box_index.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::Object;
using document::ObjectId;
using geometry::BooleanOp;
using geometry::Box2;
using geometry::Contour;
using geometry::FillRule;

namespace {

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

struct Operand {
    std::vector<ObjectId> leaves;
    document::LayerId layer{};
};

void gatherLeaves(const Document& doc, const Object& obj, std::unordered_set<ObjectId>& seen,
                  std::vector<ObjectId>& out, std::size_t depth) {
    if (depth > doc.objectCount()) {
        return;  // cyclic group links cannot occur in a consistent document; stay bounded anyway
    }
    if (const document::GroupObject* group = obj.asGroup(); group != nullptr) {
        for (ObjectId child : group->children) {
            if (const Object* c = doc.findObject(child); c != nullptr) {
                gatherLeaves(doc, *c, seen, out, depth + 1);
            }
        }
        return;
    }
    if (!obj.isVector() || obj.sheet != doc.activeSheet() || !chainUsable(doc, obj)) {
        return;
    }
    if (seen.insert(obj.id).second) {
        out.push_back(obj.id);
    }
}

/// Operands in selection order. With `closedOnly`, leaves without a closed contour are dropped.
std::vector<Operand> collectOperands(const Document& doc, bool closedOnly) {
    std::vector<Operand> operands;
    std::unordered_set<ObjectId> seen;
    for (ObjectId id : doc.selection().ids()) {
        const Object* obj = doc.findObject(id);
        if (obj == nullptr) {
            continue;
        }
        std::vector<ObjectId> leaves;
        gatherLeaves(doc, *obj, seen, leaves, 0);
        Operand op;
        for (ObjectId leaf : leaves) {
            const document::VectorObject* vec = doc.findObject(leaf)->asVector();
            bool keep = !closedOnly;
            if (closedOnly) {
                for (const Contour& c : vec->contours) {
                    if (c.closed && !c.spans.empty()) {
                        keep = true;
                        break;
                    }
                }
            }
            if (keep) {
                if (op.leaves.empty()) {
                    op.layer = doc.findObject(leaf)->layer;
                }
                op.leaves.push_back(leaf);
            }
        }
        if (!op.leaves.empty()) {
            operands.push_back(std::move(op));
        }
    }
    return operands;
}

bool coordInRange(double v) { return std::isfinite(v) && std::fabs(v) <= kMaxVectorOpCoordinateMm; }
bool pointInRange(geometry::Vec2 p) { return coordInRange(p.x) && coordInRange(p.y); }

/// Every defining number of the span is finite and within the supported coordinate range.
bool spanInRange(const geometry::Span& span) {
    if (const auto* l = std::get_if<geometry::LineSpan>(&span)) {
        return pointInRange(l->start) && pointInRange(l->end);
    }
    if (const auto* a = std::get_if<geometry::ArcSpan>(&span)) {
        return pointInRange(a->centre) && coordInRange(a->radius) && std::isfinite(a->startAngle) &&
               std::isfinite(a->endAngle);
    }
    const auto& b = std::get<geometry::BezierSpan>(span);
    return pointInRange(b.p0) && pointInRange(b.p1) && pointInRange(b.p2) && pointInRange(b.p3);
}

/// Empty when the operand leaves are acceptable, else the reason.
std::string checkSize(const Document& doc, const std::vector<ObjectId>& leaves) {
    std::size_t spans = 0;
    for (ObjectId id : leaves) {
        for (const Contour& c : doc.findObject(id)->asVector()->contours) {
            spans += c.spans.size();
            if (spans > kMaxVectorOpSpans) {
                return "The selection is too large for this operation (more than 2 million spans).";
            }
            if (c.spans.empty()) {
                continue;
            }
            for (const geometry::Span& span : c.spans) {
                if (!spanInRange(span)) {
                    return "The selection has coordinates outside the supported range (1e9 mm) or not finite.";
                }
            }
        }
    }
    return {};
}

std::vector<Contour> closedOf(const document::VectorObject& vec) {
    std::vector<Contour> out;
    for (const Contour& c : vec.contours) {
        if (c.closed && !c.spans.empty()) {
            out.push_back(c);
        }
    }
    return out;
}

/// The region of one leaf as counter-clockwise outers and clockwise holes.
std::vector<Contour> leafRegion(const document::VectorObject& vec, const geometry::Tolerance& tol) {
    std::vector<Contour> closed = closedOf(vec);
    if (closed.size() == 1) {
        if (geometry::signedArea(closed.front()) < 0.0) {
            closed.front().reverse();
        }
        return closed;
    }
    return geometry::unionContours(closed, tol, FillRule::EvenOdd);
}

/// The region of an operand (its leaves united).
std::vector<Contour> operandRegion(const Document& doc, const Operand& op, const geometry::Tolerance& tol) {
    std::vector<Contour> all;
    for (ObjectId leaf : op.leaves) {
        std::vector<Contour> r = leafRegion(*doc.findObject(leaf)->asVector(), tol);
        std::move(r.begin(), r.end(), std::back_inserter(all));
    }
    if (op.leaves.size() == 1) {
        return all;
    }
    return geometry::unionContours(all, tol, FillRule::NonZero);
}

/// Groups boolean output (CCW outers, CW holes) into one contour set per outer with its holes.
std::vector<std::vector<Contour>> splitRegions(std::vector<Contour> closed) {
    std::vector<std::size_t> outers;
    std::vector<std::size_t> holes;
    std::vector<double> areas(closed.size(), 0.0);
    for (std::size_t i = 0; i < closed.size(); ++i) {
        areas[i] = geometry::signedArea(closed[i]);
        (areas[i] >= 0.0 ? outers : holes).push_back(i);
    }
    std::vector<std::vector<Contour>> regions;
    regions.reserve(outers.size());
    std::vector<Box2> boxes;
    boxes.reserve(outers.size());
    for (std::size_t o : outers) {
        regions.push_back({closed[o]});
        boxes.push_back(closed[o].bounds());
    }
    const toolpath::BoxIndex index(boxes);
    const geometry::Tolerance tol = geometry::Tolerance::defaults();
    std::vector<Contour> orphans;
    for (std::size_t h : holes) {
        const Box2 hb = closed[h].bounds();
        const double holeArea = -areas[h];
        std::size_t best = outers.size();
        double bestArea = 0.0;
        for (std::size_t k : index.query(hb)) {
            const std::size_t o = outers[k];
            if (!boxes[k].contains(hb) || areas[o] < holeArea) {
                continue;
            }
            if (best != outers.size() && areas[o] >= bestArea) {
                continue;
            }
            if (geometry::locatePoint(closed[o], closed[h].startPoint(), FillRule::NonZero, tol) ==
                geometry::PointLocation::Outside) {
                continue;
            }
            best = k;
            bestArea = areas[o];
        }
        if (best == outers.size()) {
            orphans.push_back(std::move(closed[h]));
        } else {
            regions[best].push_back(std::move(closed[h]));
        }
    }
    if (!orphans.empty()) {
        regions.push_back(std::move(orphans));  // never lose geometry
    }
    return regions;
}

struct PendingObject {
    std::vector<Contour> contours;
    document::LayerId layer{};
};

/// Applies the computed objects and removals as one undo step. Selection becomes the new objects.
VectorOpResult commit(Document& doc, const char* text, std::vector<PendingObject> pending,
                      const std::vector<ObjectId>& remove) {
    VectorOpResult result;
    if (!doc.idGenerator().canAllocate(pending.size())) {
        result.message = "The document has no free object ids left.";
        return result;
    }
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro(text);
    for (PendingObject& p : pending) {
        auto cmd = std::make_unique<document::AddObjectCommand>(
            Object::vector(std::move(p.contours), p.layer, doc.activeSheet()));
        const document::AddObjectCommand* raw = cmd.get();
        if (doc.execute(std::move(cmd))) {
            result.created.push_back(raw->id());
        }
    }
    if (!remove.empty()) {
        doc.execute(std::make_unique<document::RemoveObjectsCommand>(remove));
    }
    doc.undoStack().endMacro();
    result.removed = remove.size();
    doc.selection().assign(result.created);
    result.ok = true;
    return result;
}

}  // namespace

bool canBoolean(const Document& doc, BooleanKind /*kind*/) {
    // Every boolean needs two operands holding a closed vector.
    return collectOperands(doc, true).size() >= 2;
}
bool canWeld(const Document& doc) { return canBoolean(doc, BooleanKind::Weld); }
bool canSubtract(const Document& doc) { return canBoolean(doc, BooleanKind::Subtract); }
bool canIntersect(const Document& doc) { return canBoolean(doc, BooleanKind::Intersect); }
bool canExclude(const Document& doc) { return canBoolean(doc, BooleanKind::Exclude); }
bool canOffset(const Document& doc) { return !collectOperands(doc, false).empty(); }

VectorOpResult applyBoolean(Document& doc, BooleanKind kind, const geometry::Tolerance& tol) {
    VectorOpResult fail;
    const std::vector<Operand> operands = collectOperands(doc, true);
    if (operands.empty()) {
        fail.message = "Select closed vectors first.";
        return fail;
    }
    if (operands.size() < 2) {
        fail.message = "Select at least two closed vectors.";
        return fail;
    }
    std::vector<ObjectId> allLeaves;
    for (const Operand& op : operands) {
        allLeaves.insert(allLeaves.end(), op.leaves.begin(), op.leaves.end());
    }
    if (std::string size = checkSize(doc, allLeaves); !size.empty()) {
        fail.message = std::move(size);
        return fail;
    }

    std::vector<PendingObject> pending;
    const auto addRegions = [&pending](std::vector<Contour> closed, document::LayerId layer) {
        for (std::vector<Contour>& region : splitRegions(std::move(closed))) {
            pending.push_back({std::move(region), layer});
        }
    };

    switch (kind) {
        case BooleanKind::Weld: {
            std::vector<Contour> all;
            for (const Operand& op : operands) {
                for (ObjectId leaf : op.leaves) {
                    std::vector<Contour> r = leafRegion(*doc.findObject(leaf)->asVector(), tol);
                    std::move(r.begin(), r.end(), std::back_inserter(all));
                }
            }
            addRegions(geometry::unionContours(all, tol, FillRule::NonZero), operands.front().layer);
            break;
        }
        case BooleanKind::Subtract:
        case BooleanKind::Intersect: {
            const std::vector<Contour> last = operandRegion(doc, operands.back(), tol);
            const BooleanOp op = kind == BooleanKind::Subtract ? BooleanOp::Difference : BooleanOp::Intersection;
            for (std::size_t i = 0; i + 1 < operands.size(); ++i) {
                geometry::BooleanResult r =
                    geometry::booleanOp(op, operandRegion(doc, operands[i], tol), last, tol, FillRule::NonZero);
                addRegions(std::move(r.closed), operands[i].layer);
            }
            break;
        }
        case BooleanKind::Exclude: {
            std::vector<Contour> acc = operandRegion(doc, operands.front(), tol);
            for (std::size_t i = 1; i < operands.size(); ++i) {
                acc = geometry::booleanOp(BooleanOp::Xor, acc, operandRegion(doc, operands[i], tol), tol,
                                          FillRule::NonZero)
                          .closed;
            }
            addRegions(std::move(acc), operands.front().layer);
            break;
        }
    }
    if (pending.empty()) {
        fail.message = "Nothing is left after the operation.";
        return fail;
    }
    const char* text = kind == BooleanKind::Weld        ? "Weld vectors"
                       : kind == BooleanKind::Subtract  ? "Subtract vectors"
                       : kind == BooleanKind::Intersect ? "Intersect vectors"
                                                        : "Exclude vectors";
    return commit(doc, text, std::move(pending), allLeaves);
}

std::string validateOffsetOptions(const OffsetOptions& options) {
    if (!std::isfinite(options.distanceMm) || options.distanceMm <= 0.0) {
        return "The offset distance must be a number greater than zero.";
    }
    if (options.distanceMm > kMaxOffsetDistanceMm) {
        return "The offset distance is too large.";
    }
    if (!std::isfinite(options.mitreLimit) || options.mitreLimit < 1.0) {
        return "The mitre limit must be at least 1.";
    }
    return {};
}

VectorOpResult applyOffset(Document& doc, const OffsetOptions& options, const geometry::Tolerance& tol) {
    VectorOpResult fail;
    if (std::string bad = validateOffsetOptions(options); !bad.empty()) {
        fail.message = std::move(bad);
        return fail;
    }
    std::vector<ObjectId> leaves;
    for (const Operand& op : collectOperands(doc, false)) {
        leaves.insert(leaves.end(), op.leaves.begin(), op.leaves.end());
    }
    if (leaves.empty()) {
        fail.message = "Select vectors first.";
        return fail;
    }
    if (std::string size = checkSize(doc, leaves); !size.empty()) {
        fail.message = std::move(size);
        return fail;
    }
    const bool outwards = options.direction == OffsetDirection::Outwards;
    const double closedDistance = outwards ? options.distanceMm : -options.distanceMm;
    const geometry::OffsetSide side = outwards ? geometry::OffsetSide::Right : geometry::OffsetSide::Left;

    std::vector<PendingObject> pending;
    std::vector<ObjectId> consumed;
    for (ObjectId leaf : leaves) {
        const Object& obj = *doc.findObject(leaf);
        const document::VectorObject& vec = *obj.asVector();
        std::vector<Contour> result;
        const std::vector<Contour> closed = closedOf(vec);
        if (!closed.empty()) {
            result = geometry::offsetContours(closed, closedDistance, options.join, tol, options.mitreLimit);
        }
        for (const Contour& c : vec.contours) {
            if (!c.closed && !c.spans.empty()) {
                std::vector<Contour> chains =
                    geometry::offsetOpenContour(c, options.distanceMm, side, options.join, tol, options.mitreLimit);
                std::move(chains.begin(), chains.end(), std::back_inserter(result));
            }
        }
        if (result.empty()) {
            continue;
        }
        pending.push_back({std::move(result), obj.layer});
        consumed.push_back(leaf);
    }
    if (pending.empty()) {
        fail.message = "Nothing is left after the offset.";
        return fail;
    }
    if (!options.deleteOriginals) {
        consumed.clear();
    }
    return commit(doc, "Offset vectors", std::move(pending), consumed);
}

SelectionRegion selectedClosedRegion(const Document& doc, const geometry::Tolerance& tol) {
    SelectionRegion out;
    const std::vector<Operand> operands = collectOperands(doc, true);
    if (operands.empty()) {
        out.message = "Select closed vectors first.";
        return out;
    }
    std::vector<ObjectId> leaves;
    for (const Operand& op : operands) {
        leaves.insert(leaves.end(), op.leaves.begin(), op.leaves.end());
    }
    if (std::string size = checkSize(doc, leaves); !size.empty()) {
        out.message = std::move(size);
        return out;
    }
    std::vector<Contour> all;
    for (ObjectId leaf : leaves) {
        std::vector<Contour> r = leafRegion(*doc.findObject(leaf)->asVector(), tol);
        std::move(r.begin(), r.end(), std::back_inserter(all));
    }
    out.contours = leaves.size() == 1 ? std::move(all) : geometry::unionContours(all, tol, FillRule::NonZero);
    if (out.contours.empty()) {
        out.message = "The selected vectors enclose no area.";
        return out;
    }
    out.ok = true;
    return out;
}

}  // namespace carvetto::viewmodel
