#include "carvetto/document/keepout_commands.hpp"

#include <algorithm>
#include <cmath>

#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour_ops.hpp"

namespace carvetto::document {

namespace {

bool usableOutline(const geometry::Contour& c) {
    return c.closed && !c.spans.empty() && std::fabs(geometry::signedArea(c)) > 1e-9;
}

}  // namespace

std::vector<geometry::Contour> closedContoursOf(const Document& doc, const std::vector<ObjectId>& ids) {
    std::vector<ObjectId> all = doc.withDescendants(ids);
    std::sort(all.begin(), all.end());
    all.erase(std::unique(all.begin(), all.end()), all.end());
    std::vector<geometry::Contour> out;
    for (const ObjectId id : all) {
        const Object* o = doc.findObject(id);
        const VectorObject* v = o ? o->asVector() : nullptr;
        if (!v) {
            continue;
        }
        for (const geometry::Contour& c : v->contours) {
            if (usableOutline(c)) {
                out.push_back(c);
            }
        }
    }
    return out;
}

CreateKeepOutZonesCommand::CreateKeepOutZonesCommand(const Document& doc, const std::vector<ObjectId>& ids)
    : outlines_(closedContoursOf(doc, ids)) {}

bool CreateKeepOutZonesCommand::apply(Document& doc) {
    if (!applied_) {
        std::vector<geometry::Contour> usable;
        std::size_t spans = 0;
        for (const KeepOutZone& z : doc.keepOutZones()) {
            spans += z.outline.spans.size();
        }
        for (const geometry::Contour& c : outlines_) {
            if (usableOutline(c)) {
                spans += c.spans.size();
                usable.push_back(c);
            }
        }
        if (usable.empty() || doc.keepOutZones().size() + usable.size() > kMaxKeepOutZones ||
            spans > kMaxKeepOutSpans || !doc.idGenerator().canAllocate(usable.size())) {
            return false;
        }
        outlines_ = std::move(usable);
        ids_.clear();
        for (std::size_t i = 0; i < outlines_.size(); ++i) {
            ids_.push_back(doc.allocateKeepOutZoneId());
        }
        applied_ = true;
    }
    for (std::size_t i = 0; i < outlines_.size(); ++i) {
        doc.insertKeepOutZone(KeepOutZone{ids_[i], outlines_[i]});
    }
    return true;
}

void CreateKeepOutZonesCommand::revert(Document& doc) {
    for (const KeepOutZoneId id : ids_) {
        doc.takeKeepOutZone(id);
    }
}

bool RemoveKeepOutZonesCommand::apply(Document& doc) {
    removed_.clear();
    std::vector<KeepOutZoneId> ids = ids_;
    if (all_) {
        for (const KeepOutZone& z : doc.keepOutZones()) {
            ids.push_back(z.id);
        }
    }
    for (const KeepOutZoneId id : ids) {
        const auto index = doc.keepOutZoneIndex(id);
        if (!index) {
            continue;  // unknown, or listed twice
        }
        auto zone = doc.takeKeepOutZone(id);
        removed_.emplace_back(*index, std::move(*zone));
    }
    return !removed_.empty();
}

void RemoveKeepOutZonesCommand::revert(Document& doc) {
    for (auto it = removed_.rbegin(); it != removed_.rend(); ++it) {
        doc.insertKeepOutZone(it->second, it->first);
    }
}

bool SetKeepOutClearanceCommand::apply(Document& doc) {
    if (!std::isfinite(after_) || after_ < 0.0 || after_ > kMaxKeepOutClearanceMm ||
        after_ == doc.keepOutClearance()) {
        return false;
    }
    before_ = doc.keepOutClearance();
    doc.setKeepOutClearance(after_);
    return true;
}

void SetKeepOutClearanceCommand::revert(Document& doc) {
    doc.setKeepOutClearance(before_);
}

bool SetKeepOutClearanceCommand::mergeWith(const Command& next) {
    const auto* n = dynamic_cast<const SetKeepOutClearanceCommand*>(&next);
    if (!n) {
        return false;
    }
    after_ = n->after_;
    return true;
}

}  // namespace carvetto::document
