#include "carvetto/document/dimension.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include "carvetto/document/commands.hpp"

namespace carvetto::document {

namespace {

using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

constexpr double kTiny = 1e-12;

bool finite(double v) {
    return std::isfinite(v);
}

bool finite(Vec2 p) {
    return finite(p.x) && finite(p.y);
}

/// Measuring direction of a linear dimension; for aligned the direction from point 0 to 1.
Vec2 axisOf(const DimensionObject& dim, const std::array<Vec2, 3>& p) {
    if (dim.kind == DimensionKind::Linear) {
        return dim.vertical ? Vec2{0.0, 1.0} : Vec2{1.0, 0.0};
    }
    return (p[1] - p[0]).normalized();
}

/// Rotates a direction by multiples of pi so text never reads upside down: (-pi/2, pi/2].
double readableAngle(double angle) {
    while (angle > geometry::kPi / 2.0) {
        angle -= geometry::kPi;
    }
    while (angle <= -geometry::kPi / 2.0) {
        angle += geometry::kPi;
    }
    return angle;
}

/// Closed triangle with its tip at `tip`, pointing against `into` (the direction from the tip
/// towards the arrow's tail).
Contour arrowhead(Vec2 tip, Vec2 into, double length) {
    const Vec2 base = tip + into * length;
    const Vec2 half = into.perp() * (length / 6.0);
    Contour c;
    c.closed = true;
    c.spans.push_back(LineSpan{tip, base + half});
    c.spans.push_back(LineSpan{base + half, base - half});
    c.spans.push_back(LineSpan{base - half, tip});
    return c;
}

void addLine(std::vector<Contour>& out, Vec2 a, Vec2 b) {
    if (geometry::distance(a, b) <= kTiny) {
        return;
    }
    Contour c;
    c.spans.push_back(LineSpan{a, b});
    out.push_back(std::move(c));
}

std::string fixed(double v, int decimals) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%.*f", decimals, v);
    return buf;
}

}  // namespace

std::size_t dimensionPointCount(DimensionKind kind) {
    return kind == DimensionKind::Angular ? 3 : 2;
}

std::string dimensionProblem(const DimensionObject& dim) {
    for (const DimensionPoint& p : dim.points) {
        if (!finite(p.pos) || std::fabs(p.pos.x) > kMaxDimensionMagnitudeMm ||
            std::fabs(p.pos.y) > kMaxDimensionMagnitudeMm) {
            return "the dimension points must be finite and within 1e9 mm";
        }
    }
    if (!finite(dim.offsetMm) || std::fabs(dim.offsetMm) > kMaxDimensionMagnitudeMm) {
        return "the dimension offset must be finite and within 1e9 mm";
    }
    if (!finite(dim.textHeightMm) || !(dim.textHeightMm > 0.0) || dim.textHeightMm > kMaxDimensionTextHeightMm) {
        return "the text height must be greater than zero and at most " +
               std::to_string(static_cast<int>(kMaxDimensionTextHeightMm)) + " mm";
    }
    if (dim.decimals < 0 || dim.decimals > kMaxDimensionDecimals) {
        return "the decimal places must be between 0 and " + std::to_string(kMaxDimensionDecimals);
    }
    if (dim.angleDecimals < 0 || dim.angleDecimals > kMaxDimensionAngleDecimals) {
        return "the angle decimal places must be between 0 and " + std::to_string(kMaxDimensionAngleDecimals);
    }
    if (dim.customText.size() > kMaxDimensionTextBytes) {
        return "the custom text is longer than " + std::to_string(kMaxDimensionTextBytes) + " bytes";
    }
    return {};
}

std::optional<Vec2> resolveAnchor(const Document& doc, const DimensionAnchor& anchor) {
    const Object* obj = doc.findObject(anchor.object);
    const VectorObject* vec = obj != nullptr ? obj->asVector() : nullptr;
    if (vec == nullptr || anchor.contour >= vec->contours.size()) {
        return std::nullopt;
    }
    const Contour& c = vec->contours[anchor.contour];
    if (anchor.span >= c.spans.size()) {
        return std::nullopt;
    }
    const geometry::Span& s = c.spans[anchor.span];
    switch (anchor.which) {
        case DimensionAnchor::Which::Start:
            return geometry::startPoint(s);
        case DimensionAnchor::Which::End:
            return geometry::endPoint(s);
        case DimensionAnchor::Which::Centre:
            if (const auto* arc = std::get_if<ArcSpan>(&s)) {
                return arc->centre;
            }
            break;
    }
    return std::nullopt;
}

std::array<Vec2, 3> resolveDimensionPoints(const Document& doc, const DimensionObject& dim) {
    std::array<Vec2, 3> out;
    for (std::size_t i = 0; i < out.size(); ++i) {
        out[i] = dim.points[i].pos;
        if (dim.points[i].anchor) {
            if (const auto p = resolveAnchor(doc, *dim.points[i].anchor)) {
                out[i] = *p;
            }
        }
    }
    return out;
}

std::optional<double> measureDimensionPoints(const DimensionObject& dim, const std::array<Vec2, 3>& p) {
    const std::size_t n = dimensionPointCount(dim.kind);
    for (std::size_t i = 0; i < n; ++i) {
        if (!finite(p[i])) {
            return std::nullopt;
        }
    }
    double value = 0.0;
    switch (dim.kind) {
        case DimensionKind::Linear:
            value = std::fabs((p[1] - p[0]).dot(axisOf(dim, p)));
            break;
        case DimensionKind::Aligned:
            value = geometry::distance(p[0], p[1]);
            break;
        case DimensionKind::Angular: {
            const Vec2 a = p[1] - p[0];
            const Vec2 b = p[2] - p[0];
            if (a.length() <= kTiny || b.length() <= kTiny) {
                return std::nullopt;
            }
            value = std::atan2(std::fabs(a.cross(b)), a.dot(b));
            if (value > kTiny && dim.offsetMm < 0.0) {
                value = 2.0 * geometry::kPi - value;  // reflex: the other way round
            }
            break;
        }
        case DimensionKind::Radial:
            value = geometry::distance(p[0], p[1]) * (dim.diameter ? 2.0 : 1.0);
            break;
    }
    if (!finite(value) || value <= kTiny) {
        return std::nullopt;
    }
    return value;
}

std::optional<double> measureDimension(const Document& doc, const DimensionObject& dim) {
    return measureDimensionPoints(dim, resolveDimensionPoints(doc, dim));
}

std::string formatDimensionValue(const DimensionObject& dim, double value, Units units) {
    if (dim.kind == DimensionKind::Angular) {
        const int d = std::clamp(dim.angleDecimals, 0, kMaxDimensionAngleDecimals);
        return fixed(value * 180.0 / geometry::kPi, d) + "\xC2\xB0";
    }
    const int d = std::clamp(dim.decimals, 0, kMaxDimensionDecimals);
    std::string text = fixed(toDisplay(value, units), d);
    if (dim.kind == DimensionKind::Radial) {
        text = (dim.diameter ? std::string("\xC3\x98") : std::string("R")) + text;
    }
    return text;
}

std::string dimensionText(const Document& doc, const DimensionObject& dim, Units units) {
    if (dim.useCustomText) {
        return dim.customText;
    }
    const auto value = measureDimension(doc, dim);
    return value ? formatDimensionValue(dim, *value, units) : std::string{};
}

std::optional<double> offsetForPlacement(const DimensionObject& dim, const std::array<Vec2, 3>& p,
                                         Vec2 placement) {
    if (!measureDimensionPoints(dim, p) || !finite(placement)) {
        return std::nullopt;
    }
    switch (dim.kind) {
        case DimensionKind::Linear:
        case DimensionKind::Aligned:
            return (placement - (p[0] + p[1]) * 0.5).dot(axisOf(dim, p).perp());
        case DimensionKind::Angular: {
            // Placing the arc outside the smaller angle measures the reflex angle (negative offset).
            const double dist = geometry::distance(placement, p[0]);
            const Vec2 a = p[1] - p[0];
            const Vec2 d = placement - p[0];
            const double sweep = std::atan2(a.cross(p[2] - p[0]), a.dot(p[2] - p[0]));
            const double t = std::atan2(a.cross(d), a.dot(d));
            const bool inside = sweep >= 0.0 ? (t >= 0.0 && t <= sweep) : (t <= 0.0 && t >= sweep);
            return inside || dist <= kTiny ? dist : -dist;
        }
        case DimensionKind::Radial: {
            const Vec2 dir = (p[1] - p[0]).normalized();
            return std::max(0.0, (placement - p[0]).dot(dir) - geometry::distance(p[0], p[1]));
        }
    }
    return std::nullopt;
}

DimensionGeometry dimensionGeometry(const Document& doc, const DimensionObject& dim) {
    DimensionGeometry g;
    const std::array<Vec2, 3> p = resolveDimensionPoints(doc, dim);
    if (!measureDimensionPoints(dim, p) || !finite(dim.offsetMm) || !(dim.textHeightMm > 0.0)) {
        return g;
    }
    const double h = dim.textHeightMm;
    const double arrow = h * kDimensionArrowFactor;
    switch (dim.kind) {
        case DimensionKind::Linear:
        case DimensionKind::Aligned: {
            const Vec2 u = axisOf(dim, p);
            const Vec2 n = u.perp();
            const Vec2 mid = (p[0] + p[1]) * 0.5;
            const double line = mid.dot(n) + dim.offsetMm;
            const Vec2 a = u * p[0].dot(u) + n * line;
            const Vec2 b = u * p[1].dot(u) + n * line;
            addLine(g.contours, p[0], a);
            addLine(g.contours, p[1], b);
            addLine(g.contours, a, b);
            const Vec2 dir = (b - a).normalized();
            g.contours.push_back(arrowhead(a, dir, arrow));
            g.contours.push_back(arrowhead(b, -dir, arrow));
            g.textCentre = (a + b) * 0.5 + n * (0.75 * h);
            g.textAngle = readableAngle(u.angle());
            break;
        }
        case DimensionKind::Angular: {
            const double r = std::fabs(dim.offsetMm);
            if (!(r > kTiny)) {
                return g;
            }
            const Vec2 v = p[0];
            const Vec2 a = p[1] - v;
            const Vec2 b = p[2] - v;
            const double a1 = a.angle();
            double sweep = std::atan2(a.cross(b), a.dot(b));  // signed, (-pi, pi]
            if (dim.offsetMm < 0.0) {
                sweep -= (sweep >= 0.0 ? 1.0 : -1.0) * 2.0 * geometry::kPi;  // reflex: the long way round
            }
            const ArcDirection dir = sweep >= 0.0 ? ArcDirection::CCW : ArcDirection::CW;
            ArcSpan arc;
            arc.centre = v;
            arc.radius = r;
            arc.startAngle = a1;
            arc.endAngle = a1 + sweep;
            arc.direction = dir;
            Contour c;
            c.spans.push_back(arc);
            g.contours.push_back(c);
            const Vec2 arms[2] = {a, b};
            for (const Vec2& arm : arms) {
                const double len = arm.length();
                if (len < r) {
                    addLine(g.contours, v + arm, v + arm.normalized() * r);
                }
            }
            const double sign = sweep >= 0.0 ? 1.0 : -1.0;
            const Vec2 startTip = arc.startPoint();
            const Vec2 endTip = arc.endPoint();
            const Vec2 startTangent = Vec2::fromAngle(a1 + sign * geometry::kPi / 2.0);
            const Vec2 endTangent = Vec2::fromAngle(arc.endAngle + sign * geometry::kPi / 2.0);
            g.contours.push_back(arrowhead(startTip, startTangent, arrow));
            g.contours.push_back(arrowhead(endTip, -endTangent, arrow));
            const double mid = a1 + sweep / 2.0;
            g.textCentre = v + Vec2::fromAngle(mid, r + 0.75 * h);
            g.textAngle = readableAngle(mid + geometry::kPi / 2.0);
            break;
        }
        case DimensionKind::Radial: {
            const Vec2 dir = (p[1] - p[0]).normalized();
            const double radius = geometry::distance(p[0], p[1]);
            const double lead = std::max(dim.offsetMm, 0.0);
            const Vec2 end = p[1] + dir * lead;
            const Vec2 start = dim.diameter ? p[0] - dir * radius : p[0];
            addLine(g.contours, start, end);
            g.contours.push_back(arrowhead(p[1], -dir, arrow));
            if (dim.diameter) {
                g.contours.push_back(arrowhead(start, dir, arrow));
            }
            g.textCentre = end + dir * (0.75 * h);
            g.textAngle = readableAngle(dir.angle());
            break;
        }
    }
    g.valid = true;
    return g;
}

namespace {

/// End of the dimension line (towards point 1) of a linear or aligned dimension.
Vec2 dimensionLineEnd(const DimensionObject& dim, const std::array<Vec2, 3>& p) {
    const Vec2 u = axisOf(dim, p);
    const Vec2 n = u.perp();
    const double line = ((p[0] + p[1]) * 0.5).dot(n) + dim.offsetMm;
    return u * p[1].dot(u) + n * line;
}

/// Text lies this many text heights beyond the dimension line / arc / leader end.
constexpr double kTextGap = 0.75;

}  // namespace

std::vector<DimensionHandle> dimensionHandles(const Document& doc, const DimensionObject& dim) {
    std::vector<DimensionHandle> out;
    const DimensionGeometry g = dimensionGeometry(doc, dim);
    if (!g.valid) {
        return out;
    }
    const std::array<Vec2, 3> p = resolveDimensionPoints(doc, dim);
    for (std::size_t i = 0; i < dimensionPointCount(dim.kind); ++i) {
        out.push_back({DimensionHandleKind::Point, i, p[i], true});
    }
    out.push_back({DimensionHandleKind::Text, 0, g.textCentre, true});
    if (dim.kind == DimensionKind::Angular) {
        const Vec2 a = p[1] - p[0];
        const Vec2 b = p[2] - p[0];
        double sweep = std::atan2(a.cross(b), a.dot(b));
        if (dim.offsetMm < 0.0) {
            sweep -= (sweep >= 0.0 ? 1.0 : -1.0) * 2.0 * geometry::kPi;
        }
        out.push_back({DimensionHandleKind::ArcRadius, 0,
                       p[0] + Vec2::fromAngle(a.angle() + sweep / 2.0, std::fabs(dim.offsetMm)), true});
    }
    if (dim.kind == DimensionKind::Linear || dim.kind == DimensionKind::Aligned) {
        out.push_back({DimensionHandleKind::Leg, 0, dimensionLineEnd(dim, p), false});
    }
    const Vec2 up = Vec2::fromAngle(g.textAngle + geometry::kPi / 2.0);
    out.push_back({DimensionHandleKind::TextHeight, 0, g.textCentre + up * (dim.textHeightMm * 0.5), false});
    return out;
}

std::optional<DimensionObject> moveDimensionHandle(const Document& doc, const DimensionObject& dim,
                                                   const DimensionHandle& handle, Vec2 to) {
    if (!finite(to) || std::fabs(to.x) > kMaxDimensionMagnitudeMm || std::fabs(to.y) > kMaxDimensionMagnitudeMm) {
        return std::nullopt;
    }
    DimensionObject next = dim;
    const std::array<Vec2, 3> p = resolveDimensionPoints(doc, dim);
    for (std::size_t i = 0; i < next.points.size(); ++i) {
        next.points[i].pos = p[i];  // anchored points keep their anchor; the position is current
    }
    switch (handle.kind) {
        case DimensionHandleKind::Point:
            if (handle.index >= dimensionPointCount(dim.kind)) {
                return std::nullopt;
            }
            next.points[handle.index].pos = to;
            next.points[handle.index].anchor.reset();
            break;
        case DimensionHandleKind::Leg: {
            if (dim.kind != DimensionKind::Linear && dim.kind != DimensionKind::Aligned) {
                return std::nullopt;
            }
            const auto o = offsetForPlacement(dim, p, to);
            if (!o) {
                return std::nullopt;
            }
            next.offsetMm = *o;
            break;
        }
        case DimensionHandleKind::ArcRadius: {
            if (dim.kind != DimensionKind::Angular) {
                return std::nullopt;
            }
            const auto o = offsetForPlacement(dim, p, to);
            if (!o || std::fabs(*o) <= kTiny) {
                return std::nullopt;
            }
            next.offsetMm = *o;
            break;
        }
        case DimensionHandleKind::Text: {
            const auto o = offsetForPlacement(dim, p, to);
            if (!o) {
                return std::nullopt;
            }
            const double gap = kTextGap * dim.textHeightMm;
            switch (dim.kind) {
                case DimensionKind::Linear:
                case DimensionKind::Aligned:
                    next.offsetMm = *o - gap;
                    break;
                case DimensionKind::Angular: {
                    const double r = std::fabs(*o) - gap;
                    if (!(r > kTiny)) {
                        return std::nullopt;
                    }
                    next.offsetMm = *o < 0.0 ? -r : r;
                    break;
                }
                case DimensionKind::Radial:
                    next.offsetMm = std::max(0.0, *o - gap);
                    break;
            }
            break;
        }
        case DimensionHandleKind::TextHeight: {
            const DimensionGeometry g = dimensionGeometry(doc, dim);
            if (!g.valid) {
                return std::nullopt;
            }
            const Vec2 up = Vec2::fromAngle(g.textAngle + geometry::kPi / 2.0);
            const double k = std::fabs((to - g.textCentre).dot(up)) / (dim.textHeightMm * 0.5);
            const double h = dim.textHeightMm * k;
            if (!finite(h) || !(h >= 1e-3) || h > kMaxDimensionTextHeightMm) {
                return std::nullopt;
            }
            next.textHeightMm = h;
            break;
        }
    }
    if (!dimensionProblem(next).empty() || !measureDimension(doc, next)) {
        return std::nullopt;
    }
    return next;
}

std::optional<AnchorHit> findAnchorNear(const Document& doc, SheetId sheet, Vec2 p, double toleranceMm) {
    std::optional<AnchorHit> best;
    double bestDist = toleranceMm;
    if (!finite(p) || !finite(toleranceMm) || toleranceMm < 0.0) {
        return best;
    }
    const auto consider = [&](ObjectId id, std::size_t ci, std::size_t si, DimensionAnchor::Which which, Vec2 at) {
        const double d = geometry::distance(p, at);
        if (d <= bestDist && (!best || d < bestDist)) {
            bestDist = d;
            DimensionAnchor a;
            a.object = id;
            a.contour = static_cast<std::uint32_t>(ci);
            a.span = static_cast<std::uint32_t>(si);
            a.which = which;
            best = AnchorHit{a, at};
        }
    };
    for (const auto& [id, obj] : doc.objects()) {  // id order: ties go to the lower id
        const VectorObject* vec = obj.asVector();
        if (vec == nullptr || obj.sheet != sheet) {
            continue;
        }
        for (std::size_t ci = 0; ci < vec->contours.size(); ++ci) {
            const Contour& c = vec->contours[ci];
            for (std::size_t si = 0; si < c.spans.size(); ++si) {
                consider(id, ci, si, DimensionAnchor::Which::Start, geometry::startPoint(c.spans[si]));
                if (!c.closed && si + 1 == c.spans.size()) {
                    consider(id, ci, si, DimensionAnchor::Which::End, geometry::endPoint(c.spans[si]));
                }
                if (const auto* arc = std::get_if<ArcSpan>(&c.spans[si])) {
                    consider(id, ci, si, DimensionAnchor::Which::Centre, arc->centre);
                }
            }
        }
    }
    return best;
}

std::optional<ObjectId> addDimension(Document& doc, const DimensionObject& dim, SheetId sheet, LayerId layer) {
    if (!dimensionProblem(dim).empty() || doc.findSheet(sheet) == nullptr ||
        !doc.idGenerator().canAllocate(2)) {
        return std::nullopt;
    }
    if (layer.isNull()) {
        for (const Layer& l : doc.layers()) {
            if (l.name == kDimensionLayerName) {
                layer = l.id;
                break;
            }
        }
    } else if (doc.findLayer(layer) == nullptr) {
        return std::nullopt;
    }
    UndoStack& stack = doc.undoStack();
    stack.beginMacro("Add dimension");
    if (layer.isNull()) {
        Layer fresh;
        fresh.name = kDimensionLayerName;
        auto cmd = std::make_unique<AddLayerCommand>(fresh);
        AddLayerCommand* raw = cmd.get();
        if (!doc.execute(std::move(cmd))) {
            stack.endMacro();
            return std::nullopt;
        }
        layer = raw->id();
    }
    Object obj;
    obj.layer = layer;
    obj.sheet = sheet;
    obj.name = "Dimension";
    obj.data = dim;
    auto add = std::make_unique<AddObjectCommand>(std::move(obj));
    AddObjectCommand* rawAdd = add.get();
    const bool ok = doc.execute(std::move(add));
    const ObjectId id = ok ? rawAdd->id() : ObjectId{};
    stack.endMacro();
    if (!ok) {
        return std::nullopt;
    }
    return id;
}

bool SetDimensionCommand::apply(Document& doc) {
    Object* obj = doc.findObject(id_);
    DimensionObject* cur = obj != nullptr ? obj->asDimension() : nullptr;
    if (cur == nullptr || !dimensionProblem(dim_).empty()) {
        return false;
    }
    if (!before_) {
        if (*cur == dim_) {
            return false;
        }
        before_ = *cur;
    }
    *cur = dim_;
    return true;
}

void SetDimensionCommand::revert(Document& doc) {
    Object* obj = doc.findObject(id_);
    if (DimensionObject* cur = obj != nullptr ? obj->asDimension() : nullptr; cur && before_) {
        *cur = *before_;
    }
}

}  // namespace carvetto::document
