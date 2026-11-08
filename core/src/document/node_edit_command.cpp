#include "carvetto/document/node_edit_command.hpp"

#include <cmath>
#include <set>
#include <variant>

#include "carvetto/document/document.hpp"

namespace carvetto::document {

namespace {

bool coordOk(double v) {
    return std::isfinite(v) && std::fabs(v) <= SetObjectContoursCommand::kMaxCoordinateMm;
}

bool pointOk(geometry::Vec2 p) {
    return coordOk(p.x) && coordOk(p.y);
}

bool spanOk(const geometry::Span& s) {
    struct Visitor {
        bool operator()(const geometry::LineSpan& l) const { return pointOk(l.start) && pointOk(l.end); }
        bool operator()(const geometry::ArcSpan& a) const {
            return pointOk(a.centre) && coordOk(a.radius) && std::isfinite(a.startAngle) &&
                   std::isfinite(a.endAngle) && a.radius >= 0.0;
        }
        bool operator()(const geometry::BezierSpan& b) const {
            return pointOk(b.p0) && pointOk(b.p1) && pointOk(b.p2) && pointOk(b.p3);
        }
    };
    return std::visit(Visitor{}, s);
}

}  // namespace

bool SetObjectContoursCommand::apply(Document& doc) {
    if (!after_.empty()) {
        // Redo: restore the recorded result.
        for (const auto& [id, data] : after_) {
            if (Object* obj = doc.findObject(id)) {
                obj->data = data;
            }
        }
        return true;
    }
    if (items_.empty()) {
        return false;
    }
    std::set<ObjectId> seen;
    for (const ContourReplacement& item : items_) {
        const Object* obj = doc.findObject(item.id);
        if (obj == nullptr || !obj->isVector() || !seen.insert(item.id).second) {
            return false;
        }
        for (const geometry::Contour& c : item.contours) {
            for (const geometry::Span& s : c.spans) {
                if (!spanOk(s)) {
                    return false;
                }
            }
        }
    }
    bool changes = false;
    for (const ContourReplacement& item : items_) {
        const VectorObject* vec = doc.findObject(item.id)->asVector();
        if (!(vec->contours == item.contours) || !std::holds_alternative<NoParams>(vec->params)) {
            changes = true;
        }
    }
    if (!changes) {
        return false;
    }
    std::vector<std::pair<ObjectId, ObjectData>> before;
    std::vector<std::pair<ObjectId, ObjectData>> after;
    for (ContourReplacement& item : items_) {
        Object* obj = doc.findObject(item.id);
        before.emplace_back(item.id, obj->data);
        VectorObject* vec = obj->asVector();
        vec->contours = item.contours;
        vec->params = NoParams{};
        after.emplace_back(item.id, obj->data);
    }
    before_ = std::move(before);
    after_ = std::move(after);
    items_.clear();  // the snapshots carry everything redo needs
    return true;
}

void SetObjectContoursCommand::revert(Document& doc) {
    for (const auto& [id, data] : before_) {
        if (Object* obj = doc.findObject(id)) {
            obj->data = data;
        }
    }
}

bool SetObjectContoursCommand::mergeWith(const Command& next) {
    const auto* other = dynamic_cast<const SetObjectContoursCommand*>(&next);
    if (other == nullptr || other->after_.size() != after_.size()) {
        return false;
    }
    for (std::size_t i = 0; i < after_.size(); ++i) {
        if (after_[i].first != other->after_[i].first) {
            return false;
        }
    }
    after_ = other->after_;
    return true;
}

}  // namespace carvetto::document
