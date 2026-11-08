#include "carvetto/document/text_commands.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <variant>

#include "carvetto/document/text_object.hpp"

namespace carvetto::document {

namespace {

constexpr double kMaxCoordinateMm = 1.0e9;

bool coordOk(double v) {
    return std::isfinite(v) && std::fabs(v) <= kMaxCoordinateMm;
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

bool AddFontCommand::apply(Document& doc) {
    added_ = doc.insertFont(font_);
    return added_;
}

void AddFontCommand::revert(Document& doc) {
    if (added_) {
        doc.takeFont(font_.key);
    }
}

bool SetTextCommand::apply(Document& doc) {
    Object* obj = doc.findObject(id_);
    VectorObject* vec = obj ? obj->asVector() : nullptr;
    if (vec == nullptr || !std::holds_alternative<TextParams>(vec->params) || doc.findFont(params_.fontKey) == nullptr ||
        !textParamsProblem(params_).empty()) {
        return false;
    }
    for (const geometry::Contour& c : contours_) {
        if (!std::all_of(c.spans.begin(), c.spans.end(), spanOk)) {
            return false;
        }
    }
    if (!before_) {
        if (vec->contours == contours_ && std::get<TextParams>(vec->params) == params_) {
            return false;
        }
        before_ = *vec;
    }
    vec->params = params_;
    vec->contours = contours_;
    return true;
}

void SetTextCommand::revert(Document& doc) {
    Object* obj = doc.findObject(id_);
    if (VectorObject* vec = obj ? obj->asVector() : nullptr; vec && before_) {
        *vec = *before_;
    }
}

bool ConvertTextToCurvesCommand::apply(Document& doc) {
    if (!before_.empty()) {
        for (const auto& [id, params] : before_) {
            if (Object* obj = doc.findObject(id)) {
                obj->asVector()->params = NoParams{};
            }
        }
        return true;
    }
    if (ids_.empty()) {
        return false;
    }
    std::set<ObjectId> seen;
    for (const ObjectId id : ids_) {
        const Object* obj = doc.findObject(id);
        if (obj == nullptr || !obj->isVector() || !std::holds_alternative<TextParams>(obj->asVector()->params) ||
            !seen.insert(id).second) {
            return false;
        }
    }
    for (const ObjectId id : ids_) {
        VectorObject* vec = doc.findObject(id)->asVector();
        before_.emplace_back(id, vec->params);
        vec->params = NoParams{};
    }
    return true;
}

void ConvertTextToCurvesCommand::revert(Document& doc) {
    for (const auto& [id, params] : before_) {
        if (Object* obj = doc.findObject(id); obj && obj->isVector()) {
            obj->asVector()->params = params;
        }
    }
}

}  // namespace carvetto::document
