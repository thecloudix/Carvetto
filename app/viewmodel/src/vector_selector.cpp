#include "carvetto/viewmodel/vector_selector.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <set>
#include <type_traits>
#include <unordered_set>
#include <utility>
#include <variant>

#include "carvetto/geometry/span.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::LayerId;
using document::Object;
using document::ObjectId;
using geometry::Contour;

namespace {

constexpr double kTwoPi = 6.283185307179586;

bool layerUsable(const Document& doc, LayerId id, bool& hidden) {
    const document::Layer* layer = doc.findLayer(id);
    if (layer == nullptr) {
        hidden = true;
        return false;
    }
    if (!layer->visible) {
        hidden = true;
        return false;
    }
    hidden = false;
    return !layer->locked;
}

enum class Usable { Yes, Hidden, Locked };

/// An object and every ancestor group must sit on visible, unlocked layers.
Usable chainUsable(const Document& doc, const Object& obj) {
    const Object* cur = &obj;
    for (std::size_t guard = 0; guard <= doc.objectCount(); ++guard) {
        bool hidden = false;
        if (!layerUsable(doc, cur->layer, hidden)) {
            return hidden ? Usable::Hidden : Usable::Locked;
        }
        if (!cur->parent.has_value()) {
            return Usable::Yes;
        }
        cur = doc.findObject(*cur->parent);
        if (cur == nullptr) {
            return Usable::Hidden;
        }
    }
    return Usable::Hidden;
}

bool allocated(const Document& doc, std::uint64_t v) {
    return v != 0 && v < document::IdGenerator::kMaxId && v < doc.idGenerator().peek();
}

template <class IdT>
bool hasRepeats(const std::vector<IdT>& ids) {
    std::set<IdT> seen;
    for (const IdT id : ids) {
        if (!seen.insert(id).second) {
            return true;
        }
    }
    return false;
}

}  // namespace

VectorSelector VectorSelector::fromLayers(std::vector<LayerId> layers) {
    VectorSelector s;
    s.base = document::VectorSelection{false, std::move(layers), {}};
    return s;
}

VectorSelector VectorSelector::fromObjects(std::vector<ObjectId> objects) {
    VectorSelector s;
    s.base = document::VectorSelection{false, {}, std::move(objects)};
    return s;
}

VectorSelector VectorSelector::fromSelection(const Document& doc) {
    std::vector<ObjectId> ids;
    for (const ObjectId id : doc.selection().ids()) {
        if (doc.hasObject(id)) {
            ids.push_back(id);
        }
    }
    return fromObjects(std::move(ids));
}

std::vector<Contour> ResolvedVectors::copyContours() const {
    std::vector<Contour> out;
    out.reserve(contours.size());
    for (const ContourRef& c : contours) {
        out.push_back(*c.contour);
    }
    return out;
}

ObjectId ResolvedVectors::ownerOf(std::size_t contour) const {
    return contour < contours.size() ? contours[contour].object : ObjectId{};
}

bool isCircleContour(const Contour& contour, double diameterMm, double toleranceMm) {
    if (!contour.closed || contour.spans.empty()) {
        return false;
    }
    const auto* first = std::get_if<geometry::ArcSpan>(&contour.spans.front());
    if (first == nullptr || !(first->radius > 0.0) || !std::isfinite(first->radius)) {
        return false;
    }
    double sweep = 0.0;  // signed: every arc must turn the same way, so an out-and-back pair is no circle
    for (const geometry::Span& s : contour.spans) {
        const auto* arc = std::get_if<geometry::ArcSpan>(&s);
        if (arc == nullptr) {
            return false;
        }
        const double tol = 1e-6 * std::max(1.0, first->radius);
        if (std::fabs(arc->radius - first->radius) > tol || std::fabs(arc->centre.x - first->centre.x) > tol ||
            std::fabs(arc->centre.y - first->centre.y) > tol) {
            return false;
        }
        if (arc->direction != first->direction) {
            return false;
        }
        sweep += std::fabs(arc->sweepAngle());
    }
    if (std::fabs(sweep - kTwoPi) > 1e-6) {
        return false;
    }
    if (diameterMm > 0.0) {
        return std::fabs(2.0 * first->radius - diameterMm) <= toleranceMm;
    }
    return true;
}

ResolvedVectors resolveVectors(const Document& doc, const VectorSelector& selector) {
    ResolvedVectors out;
    for (const ObjectId id : doc.selectedVectors(selector.base)) {
        const Object* obj = doc.findObject(id);
        const document::VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
        if (vec == nullptr) {
            continue;
        }
        if (obj->sheet != doc.activeSheet()) {
            ++out.skippedOtherSheet;
            continue;
        }
        const Usable usable = chainUsable(doc, *obj);
        if (usable == Usable::Hidden) {
            ++out.skippedHidden;
            continue;
        }
        if (usable == Usable::Locked) {
            ++out.skippedLocked;
            continue;
        }
        bool kept = false;
        for (const Contour& c : vec->contours) {
            bool pass = c.closed ? selector.includeClosed : selector.includeOpen;
            if (pass && selector.onlyCircles) {
                pass = isCircleContour(c, selector.circleDiameterMm, selector.circleToleranceMm);
            }
            if (!pass) {
                ++out.filteredOut;
                continue;
            }
            out.contours.push_back(ContourRef{id, &c});
            out.spanCount += c.spans.size();
            kept = true;
        }
        if (kept) {
            out.objects.push_back(id);
        }
    }
    return out;
}

std::string validateSelector(const Document& doc, const VectorSelector& selector) {
    const document::VectorSelection& sel = selector.base;
    if (sel.all && (!sel.layers.empty() || !sel.objects.empty())) {
        return "a selection of every vector cannot also list layers or objects";
    }
    if (hasRepeats(sel.layers) || hasRepeats(sel.objects)) {
        return "the vector selection lists an item twice";
    }
    for (const LayerId id : sel.layers) {
        if (!allocated(doc, id.value) || doc.findSheet(document::SheetId{id.value}) != nullptr ||
            doc.hasObject(ObjectId{id.value}) || doc.findToolpath(document::ToolpathId{id.value}) != nullptr) {
            return "the vector selection names something that is not a layer";
        }
    }
    for (const ObjectId id : sel.objects) {
        if (!allocated(doc, id.value) || doc.findSheet(document::SheetId{id.value}) != nullptr ||
            doc.findLayer(LayerId{id.value}) != nullptr || doc.findToolpath(document::ToolpathId{id.value}) != nullptr) {
            return "the vector selection names something that is not a drawing object";
        }
    }
    if (!std::isfinite(selector.circleDiameterMm) || selector.circleDiameterMm < 0.0 ||
        !std::isfinite(selector.circleToleranceMm) || selector.circleToleranceMm < 0.0) {
        return "the circle filter needs a finite, non-negative diameter and tolerance";
    }
    return {};
}

VectorSelector normalisedSelector(VectorSelector selector) {
    const auto dedupe = [](auto& ids) {
        std::set<typename std::decay_t<decltype(ids)>::value_type> seen;
        std::erase_if(ids, [&seen](auto id) { return !seen.insert(id).second; });
    };
    dedupe(selector.base.layers);
    dedupe(selector.base.objects);
    if (selector.base.all) {
        selector.base.layers.clear();
        selector.base.objects.clear();
    }
    return selector;
}

document::VectorSelection storedSelection(const VectorSelector& selector, const ResolvedVectors& resolved) {
    if (!selector.hasFilters()) {
        return normalisedSelector(selector).base;
    }
    document::VectorSelection sel;
    sel.objects = resolved.objects;
    return sel;
}

// --- Fingerprint ---------------------------------------------------------------------------------

void Fingerprint::add(std::uint64_t value) {
    for (int i = 0; i < 8; ++i) {
        state_ ^= (value >> (8 * i)) & 0xFFU;
        state_ *= 1099511628211ULL;
    }
}

void Fingerprint::add(double value) {
    if (value == 0.0) {
        value = 0.0;  // -0 and 0 alike
    }
    add(std::bit_cast<std::uint64_t>(value));
}

void Fingerprint::add(std::string_view text) {
    add(static_cast<std::uint64_t>(text.size()));
    for (const char c : text) {
        state_ ^= static_cast<unsigned char>(c);
        state_ *= 1099511628211ULL;
    }
}

void Fingerprint::add(const Contour& contour) {
    add(static_cast<std::uint64_t>(contour.closed ? 1 : 0));
    add(static_cast<std::uint64_t>(contour.spans.size()));
    for (const geometry::Span& s : contour.spans) {
        add(static_cast<std::uint64_t>(s.index()));
        std::visit(
            [this](const auto& span) {
                using T = std::decay_t<decltype(span)>;
                if constexpr (std::is_same_v<T, geometry::LineSpan>) {
                    add(span.start.x);
                    add(span.start.y);
                    add(span.end.x);
                    add(span.end.y);
                } else if constexpr (std::is_same_v<T, geometry::ArcSpan>) {
                    add(span.centre.x);
                    add(span.centre.y);
                    add(span.radius);
                    add(span.startAngle);
                    add(span.endAngle);
                    add(static_cast<std::uint64_t>(span.direction == geometry::ArcDirection::CW ? 1 : 0));
                } else {
                    add(span.p0.x);
                    add(span.p0.y);
                    add(span.p1.x);
                    add(span.p1.y);
                    add(span.p2.x);
                    add(span.p2.y);
                    add(span.p3.x);
                    add(span.p3.y);
                }
            },
            s);
    }
}

std::uint64_t fingerprint(const ResolvedVectors& resolved) {
    Fingerprint fp;
    fp.add(static_cast<std::uint64_t>(resolved.contours.size()));
    for (const ContourRef& c : resolved.contours) {
        fp.add(c.object.value);
        fp.add(*c.contour);
    }
    return fp.value();
}

}  // namespace carvetto::viewmodel
