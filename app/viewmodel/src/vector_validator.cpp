#include "carvetto/viewmodel/vector_validator.hpp"

#include <algorithm>
#include <memory>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/node_edit_command.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::Object;
using document::ObjectId;
namespace gv = geometry::validator;

namespace {

bool layerUsable(const Document& doc, document::LayerId id) {
    const document::Layer* layer = doc.findLayer(id);
    return layer != nullptr && layer->visible && !layer->locked;
}

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

void gather(const Document& doc, const Object& obj, std::unordered_set<ObjectId>& seen, std::vector<ObjectId>& out,
            std::size_t depth) {
    if (depth > doc.objectCount()) {
        return;
    }
    if (const document::GroupObject* group = obj.asGroup(); group != nullptr) {
        for (ObjectId child : group->children) {
            if (const Object* c = doc.findObject(child); c != nullptr) {
                gather(doc, *c, seen, out, depth + 1);
            }
        }
        return;
    }
    if (obj.isVector() && obj.sheet == doc.activeSheet() && chainUsable(doc, obj) && seen.insert(obj.id).second) {
        out.push_back(obj.id);
    }
}

/// Vector objects of the scope, in document (id) order.
std::vector<ObjectId> scopeObjects(const Document& doc, ValidatorScope scope) {
    std::unordered_set<ObjectId> seen;
    std::vector<ObjectId> out;
    if (scope == ValidatorScope::Selection) {
        for (ObjectId id : doc.selection().ids()) {
            if (const Object* o = doc.findObject(id); o != nullptr) {
                gather(doc, *o, seen, out, 0);
            }
        }
    } else {
        for (const auto& [id, obj] : doc.objects()) {
            gather(doc, obj, seen, out, 0);
        }
    }
    std::sort(out.begin(), out.end());
    return out;
}

bool isText(const Object& obj) {
    const document::VectorObject* v = obj.asVector();
    return v != nullptr && std::holds_alternative<document::TextParams>(v->params);
}

const char* kindName(gv::IssueKind k) {
    switch (k) {
        case gv::IssueKind::ZeroLengthSpan:
            return "Zero-length spans";
        case gv::IssueKind::TinyContour:
            return "Tiny contours";
        case gv::IssueKind::OpenContour:
            return "Open contours";
        case gv::IssueKind::DuplicateContour:
            return "Duplicate contours";
        case gv::IssueKind::SelfIntersecting:
            return "Self-intersecting contours";
        case gv::IssueKind::WrongDirection:
            return "Wrong-direction contours";
        case gv::IssueKind::InvalidGeometry:
            return "Invalid geometry";
    }
    return "";
}

}  // namespace

ValidatorScope defaultValidatorScope(const Document& doc) {
    return doc.selection().empty() ? ValidatorScope::AllVisible : ValidatorScope::Selection;
}

std::vector<ObjectId> ValidatorReport::offenders() const {
    std::set<std::size_t> idx;
    for (const gv::Issue& i : report.issues) {
        idx.insert(i.object);
    }
    std::vector<ObjectId> out;
    for (std::size_t i : idx) {
        out.push_back(objects[i]);
    }
    return out;
}

std::vector<ObjectId> ValidatorReport::offenders(gv::IssueKind kind) const {
    std::set<std::size_t> idx;
    for (const gv::Issue& i : report.issues) {
        if (i.kind == kind) {
            idx.insert(i.object);
        }
    }
    std::vector<ObjectId> out;
    for (std::size_t i : idx) {
        out.push_back(objects[i]);
    }
    return out;
}

std::string ValidatorReport::summary() const {
    std::ostringstream out;
    if (report.total() == 0) {
        out << "No problems found.\n";
    } else {
        out << report.total() << (report.total() == 1 ? " problem found" : " problems found") << " ("
            << report.fixableCount() << " fixable):\n";
        for (std::size_t k = 0; k < gv::kIssueKindCount; ++k) {
            const auto kind = static_cast<gv::IssueKind>(k);
            if (report.count(kind) > 0) {
                out << "  " << kindName(kind) << ": " << report.count(kind) << "\n";
            }
        }
    }
    if (report.skipped > 0) {
        out << report.skipped << " contour(s) or object(s) were too large to check completely.\n";
    }
    if (report.truncated) {
        out << "The drawing exceeded the validator's work limit; results are incomplete.\n";
    }
    return out.str();
}

ValidatorReport validateVectors(const Document& doc, const ValidatorOptions& options) {
    ValidatorReport result;
    result.objects = scopeObjects(doc, options.scope);
    std::vector<const gv::ObjectContours*> view;
    std::vector<bool> skip;
    view.reserve(result.objects.size());
    for (ObjectId id : result.objects) {
        const Object* obj = doc.findObject(id);
        view.push_back(&obj->asVector()->contours);
        skip.push_back(options.vCarvingMode && isText(*obj));
    }
    result.report = gv::validate(view, options.geometry, skip);
    return result;
}

std::size_t selectOffenders(Document& doc, const ValidatorReport& report) {
    const std::vector<ObjectId> ids = report.offenders();
    if (!ids.empty()) {
        doc.selection().assign(ids);
    }
    return ids.size();
}

std::size_t selectOffenders(Document& doc, const ValidatorReport& report, gv::IssueKind kind) {
    const std::vector<ObjectId> ids = report.offenders(kind);
    if (!ids.empty()) {
        doc.selection().assign(ids);
    }
    return ids.size();
}

std::size_t selectDuplicateVectors(Document& doc, const ValidatorOptions& options) {
    return selectOffenders(doc, validateVectors(doc, options), gv::IssueKind::DuplicateContour);
}

ValidatorFixResult fixVectors(Document& doc, const ValidatorOptions& options) {
    ValidatorFixResult result;
    const std::vector<ObjectId> ids = scopeObjects(doc, options.scope);
    std::vector<gv::ObjectContours> work;
    work.reserve(ids.size());
    for (ObjectId id : ids) {
        work.push_back(doc.findObject(id)->asVector()->contours);
    }
    const std::vector<gv::ObjectContours> original = work;
    result.fixed = gv::fix(work, options.geometry, options.fixes);

    std::vector<document::ContourReplacement> changes;
    std::vector<ObjectId> removals;
    for (std::size_t i = 0; i < ids.size(); ++i) {
        if (work[i] == original[i]) {
            continue;
        }
        if (work[i].empty()) {
            removals.push_back(ids[i]);
        } else {
            changes.push_back({ids[i], std::move(work[i])});
        }
    }
    if (changes.empty() && removals.empty()) {
        result.ok = true;
        result.message = "Nothing to fix.";
        return result;
    }
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Validate vectors");
    for (document::ContourReplacement& c : changes) {
        std::vector<document::ContourReplacement> one;
        one.push_back(std::move(c));
        if (doc.execute(std::make_unique<document::SetObjectContoursCommand>(std::move(one), "Validate vectors"))) {
            ++result.objectsChanged;
        }
    }
    if (!removals.empty()) {
        if (doc.execute(std::make_unique<document::RemoveObjectsCommand>(removals))) {
            result.objectsRemoved = removals.size();
        }
    }
    doc.undoStack().endMacro();
    result.ok = true;
    result.message = "Fixed " + std::to_string(result.fixed.total()) + " problem(s).";
    return result;
}

}  // namespace carvetto::viewmodel
