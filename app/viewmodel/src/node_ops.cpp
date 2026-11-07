#include "carvetto/viewmodel/node_ops.hpp"

#include <algorithm>
#include <cmath>
#include <utility>
#include <variant>

#include "carvetto/geometry/vec2.hpp"

namespace carvetto::viewmodel::nodeops {

using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::kPi;
using geometry::kTwoPi;
using geometry::LineSpan;
using geometry::Span;
using geometry::Vec2;

namespace {

void setWhy(std::string* why, const char* text) {
    if (why != nullptr) {
        *why = text;
    }
}

bool coordOk(double v) {
    return std::isfinite(v) && std::fabs(v) <= kMaxCoordinateMm;
}

bool pointOk(Vec2 p) {
    return coordOk(p.x) && coordOk(p.y);
}

bool spanOk(const Span& s) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return pointOk(l->start) && pointOk(l->end) && geometry::distance(l->start, l->end) >= kMinSpanMm;
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        return pointOk(a->centre) && coordOk(a->radius) && std::isfinite(a->startAngle) &&
               std::isfinite(a->endAngle) && a->radius >= kMinSpanMm && a->length() >= kMinSpanMm;
    }
    const auto& b = std::get<BezierSpan>(s);
    if (!pointOk(b.p0) || !pointOk(b.p1) || !pointOk(b.p2) || !pointOk(b.p3)) {
        return false;
    }
    const double reach = std::max({geometry::distance(b.p0, b.p1), geometry::distance(b.p0, b.p2),
                                   geometry::distance(b.p0, b.p3)});
    return reach >= kMinSpanMm;
}

bool isBezier(const Span& s) {
    return std::holds_alternative<BezierSpan>(s);
}

BezierSpan lineToBezier(const LineSpan& l) {
    return {l.start, geometry::lerp(l.start, l.end, 1.0 / 3.0), geometry::lerp(l.start, l.end, 2.0 / 3.0), l.end};
}

Vec2 unitOr(Vec2 v, Vec2 fallback) {
    const double len = v.length();
    return len > 0.0 && std::isfinite(len) ? v / len : fallback;
}

/// Start / end re-placement of one span. `newStart` / `newEnd` are absolute positions.
bool reposition(Span& s, Vec2 newStart, Vec2 newEnd) {
    if (auto* l = std::get_if<LineSpan>(&s)) {
        l->start = newStart;
        l->end = newEnd;
        return true;
    }
    if (auto* b = std::get_if<BezierSpan>(&s)) {
        const Vec2 ds = newStart - b->p0;
        const Vec2 de = newEnd - b->p3;
        b->p1 = b->p1 + ds;
        b->p2 = b->p2 + de;
        b->p0 = newStart;
        b->p3 = newEnd;
        return true;
    }
    auto& a = std::get<ArcSpan>(s);
    const Vec2 oldStart = a.startPoint();
    const Vec2 oldEnd = a.endPoint();
    const Vec2 ds = newStart - oldStart;
    const Vec2 de = newEnd - oldEnd;
    if (ds == de) {
        a.centre = a.centre + ds;
        return true;
    }
    const auto arc = arcFromSweep(newStart, newEnd, a.sweepAngle());
    if (!arc.has_value()) {
        return false;
    }
    a = *arc;
    return true;
}

Vec2 tangentIn(const Span& s) {
    return geometry::tangentAt(s, 1.0);
}

Vec2 tangentOut(const Span& s) {
    return geometry::tangentAt(s, 0.0);
}

std::size_t nextNode(const Contour& c, std::size_t i) {
    return c.closed ? (i + 1) % c.spans.size() : i + 1;
}

/// Joins a run of consecutive spans into one (see `deleteNodes`).
Span mergeRun(const std::vector<Span>& run) {
    if (run.size() == 1) {
        return run.front();
    }
    const Vec2 a = geometry::startPoint(run.front());
    const Vec2 b = geometry::endPoint(run.back());
    bool anyBezier = false;
    for (const Span& s : run) {
        anyBezier = anyBezier || isBezier(s);
    }
    if (!anyBezier) {
        return LineSpan{a, b};
    }
    const auto* first = std::get_if<BezierSpan>(&run.front());
    const auto* last = std::get_if<BezierSpan>(&run.back());
    return BezierSpan{a, first != nullptr ? first->p1 : geometry::lerp(a, b, 1.0 / 3.0),
                      last != nullptr ? last->p2 : geometry::lerp(a, b, 2.0 / 3.0), b};
}

Contour withSpans(const Contour& like, std::vector<Span> spans) {
    Contour out;
    out.closed = like.closed;
    out.spans = std::move(spans);
    return out;
}

}  // namespace

std::size_t nodeCount(const Contour& c) {
    if (c.spans.empty()) {
        return 0;
    }
    return c.closed ? c.spans.size() : c.spans.size() + 1;
}

Vec2 nodePoint(const Contour& c, std::size_t node) {
    if (node < c.spans.size()) {
        return geometry::startPoint(c.spans[node]);
    }
    return geometry::endPoint(c.spans.back());
}

Contour normalised(const Contour& c) {
    if (isNormalised(c)) {
        return c;
    }
    Contour out;
    out.closed = c.closed;
    for (const Span& s : c.spans) {
        if (const auto* a = std::get_if<ArcSpan>(&s); a != nullptr && a->isFullCircle()) {
            const double sweep = a->sweepAngle();
            const double mid = a->startAngle + sweep * 0.5;
            out.spans.emplace_back(ArcSpan{a->centre, a->radius, a->startAngle, mid, a->direction});
            out.spans.emplace_back(ArcSpan{a->centre, a->radius, mid, a->endAngle, a->direction});
        } else {
            out.spans.push_back(s);
        }
    }
    return out;
}

bool isNormalised(const Contour& c) {
    for (const Span& s : c.spans) {
        if (const auto* a = std::get_if<ArcSpan>(&s); a != nullptr && a->isFullCircle()) {
            return false;
        }
    }
    return true;
}

bool validContour(const Contour& c, std::string* why) {
    if (c.spans.empty()) {
        setWhy(why, "the vector would have no spans");
        return false;
    }
    for (const Span& s : c.spans) {
        if (!spanOk(s)) {
            setWhy(why, "the edit would make a span degenerate or move it out of range");
            return false;
        }
    }
    return true;
}

std::optional<ArcSpan> arcFromSweep(Vec2 a, Vec2 b, double sweep) {
    const double mag = std::fabs(sweep);
    if (!std::isfinite(sweep) || !(mag > 1e-9) || mag > kTwoPi - 1e-9) {
        return std::nullopt;
    }
    const Vec2 d = b - a;
    const double chord = d.length();
    if (!std::isfinite(chord) || chord < kMinSpanMm) {
        return std::nullopt;
    }
    const double radius = chord / (2.0 * std::sin(mag * 0.5));
    const Vec2 left = d.perp() / chord;
    const Vec2 mid = (a + b) * 0.5;
    const double off = radius * std::cos(mag * 0.5);
    const Vec2 centre = sweep > 0.0 ? mid + left * off : mid - left * off;
    ArcSpan arc{centre, radius, (a - centre).angle(), (b - centre).angle(),
                sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW};
    if (!std::isfinite(radius) || !pointOk(centre) || std::fabs(arc.sweepAngle() - sweep) > 1e-7) {
        return std::nullopt;
    }
    return arc;
}

std::optional<ArcSpan> arcThrough(Vec2 a, Vec2 p, Vec2 b) {
    const Vec2 ab = b - a;
    const Vec2 ap = p - a;
    const double lab = ab.length();
    const double lap = ap.length();
    if (!(lab >= kMinSpanMm) || !(lap >= kMinSpanMm)) {
        return std::nullopt;
    }
    const double cross = ab.cross(ap);
    if (std::fabs(cross) <= 1e-9 * lab * lap) {
        return std::nullopt;
    }
    // Circumcentre relative to a.
    const double d = 2.0 * cross;
    const double ux = (ap.y * ab.lengthSquared() - ab.y * ap.lengthSquared()) / d;
    const double uy = (ab.x * ap.lengthSquared() - ap.x * ab.lengthSquared()) / d;
    const Vec2 centre = a + Vec2{ux, uy};
    const double radius = geometry::distance(centre, a);
    if (!pointOk(centre) || !coordOk(radius)) {
        return std::nullopt;
    }
    // p on the right of a -> b means the arc runs counter-clockwise around the centre.
    const ArcDirection dir = cross < 0.0 ? ArcDirection::CCW : ArcDirection::CW;
    return ArcSpan{centre, radius, (a - centre).angle(), (b - centre).angle(), dir};
}

std::optional<ArcSpan> arcFromBulge(Vec2 a, Vec2 b, double bulge) {
    if (!std::isfinite(bulge)) {
        return std::nullopt;
    }
    return arcFromSweep(a, b, 4.0 * std::atan(bulge));
}

Vec2 midHandle(const Span& s) {
    return geometry::pointAt(s, 0.5);
}

// ------------------------------------------------------------------------------------- moving

bool moveNodes(Contour& c, const std::map<std::size_t, Vec2>& targets, std::string* why) {
    if (c.spans.empty()) {
        setWhy(why, "the vector has no spans");
        return false;
    }
    const std::size_t count = nodeCount(c);
    for (const auto& [node, pos] : targets) {
        if (node >= count || !pointOk(pos)) {
            setWhy(why, "the node or its target position is out of range");
            return false;
        }
    }
    Contour work = c;
    for (std::size_t i = 0; i < work.spans.size(); ++i) {
        const auto s = targets.find(i);
        const auto e = targets.find(nextNode(work, i));
        if (s == targets.end() && e == targets.end()) {
            continue;
        }
        const Vec2 start = s != targets.end() ? s->second : geometry::startPoint(work.spans[i]);
        const Vec2 end = e != targets.end() ? e->second : geometry::endPoint(work.spans[i]);
        if (!reposition(work.spans[i], start, end)) {
            setWhy(why, "an arc would become degenerate");
            return false;
        }
    }
    if (!validContour(work, why)) {
        return false;
    }
    c = std::move(work);
    return true;
}

bool setControlPoint(Contour& c, std::size_t span, int which, Vec2 pos, bool keepTangency, std::string* why) {
    if (span >= c.spans.size() || (which != 0 && which != 1) || !pointOk(pos)) {
        setWhy(why, "the control point or its position is out of range");
        return false;
    }
    Contour work = c;
    auto* b = std::get_if<BezierSpan>(&work.spans[span]);
    if (b == nullptr) {
        setWhy(why, "only Bezier spans have control points");
        return false;
    }
    (which == 0 ? b->p1 : b->p2) = pos;
    if (keepTangency) {
        const Vec2 node = which == 0 ? b->p0 : b->p3;
        // The neighbouring span on the other side of the node.
        std::optional<std::size_t> other;
        const std::size_t n = work.spans.size();
        if (which == 0) {
            if (span > 0) {
                other = span - 1;
            } else if (work.closed) {
                other = n - 1;
            }
        } else if (span + 1 < n) {
            other = span + 1;
        } else if (work.closed) {
            other = 0;
        }
        if (other.has_value() && *other != span) {
            if (auto* ob = std::get_if<BezierSpan>(&work.spans[*other])) {
                Vec2& oppositeHandle = which == 0 ? ob->p2 : ob->p1;
                const double len = geometry::distance(oppositeHandle, node);
                const Vec2 dir = node - pos;  // away from the dragged handle
                if (dir.length() > 0.0 && len > 0.0) {
                    oppositeHandle = node + dir.normalized() * len;
                }
            }
        }
    }
    if (!validContour(work, why)) {
        return false;
    }
    c = std::move(work);
    return true;
}

bool moveSpan(Contour& c, std::size_t span, Vec2 delta, std::string* why) {
    if (span >= c.spans.size() || !pointOk(delta)) {
        setWhy(why, "the span or the offset is out of range");
        return false;
    }
    std::map<std::size_t, Vec2> targets;
    targets[span] = nodePoint(c, span) + delta;
    const std::size_t e = nextNode(c, span);
    targets[e] = nodePoint(c, e) + delta;
    return moveNodes(c, targets, why);
}

bool reshapeSpan(Contour& c, std::size_t span, double t, Vec2 delta, std::string* why) {
    if (span >= c.spans.size() || !std::isfinite(t) || !pointOk(delta)) {
        setWhy(why, "the span or the offset is out of range");
        return false;
    }
    Contour work = c;
    Span& s = work.spans[span];
    if (std::holds_alternative<LineSpan>(s)) {
        return moveSpan(c, span, delta, why);
    }
    if (auto* b = std::get_if<BezierSpan>(&s)) {
        const double u = std::clamp(t, 0.05, 0.95);
        const double w = 1.0 - u;
        const double b1 = 3.0 * w * w * u;
        const double b2 = 3.0 * w * u * u;
        const double den = b1 * b1 + b2 * b2;
        b->p1 = b->p1 + delta * (b1 / den);
        b->p2 = b->p2 + delta * (b2 / den);
    } else {
        const auto& a = std::get<ArcSpan>(s);
        const Vec2 p = a.pointAt(std::clamp(t, 0.0, 1.0)) + delta;
        const auto arc = arcThrough(a.startPoint(), p, a.endPoint());
        if (!arc.has_value()) {
            setWhy(why, "the arc cannot pass through that point (collinear with its end points)");
            return false;
        }
        s = *arc;
    }
    if (!validContour(work, why)) {
        return false;
    }
    c = std::move(work);
    return true;
}

// ------------------------------------------------------------------------------------ structure

std::pair<Span, Span> splitSpan(const Span& s, double t) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        const Vec2 m = l->pointAt(t);
        return {LineSpan{l->start, m}, LineSpan{m, l->end}};
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        const double mid = a->angleAt(t);
        return {ArcSpan{a->centre, a->radius, a->startAngle, mid, a->direction},
                ArcSpan{a->centre, a->radius, mid, a->endAngle, a->direction}};
    }
    const auto parts = std::get<BezierSpan>(s).split(t);
    return {parts.first, parts.second};
}

std::optional<Contour> insertPoint(const Contour& c, std::size_t span, double t, std::string* why) {
    if (span >= c.spans.size() || !(t > 0.0) || !(t < 1.0)) {
        setWhy(why, "there is no span at that position");
        return std::nullopt;
    }
    const auto [first, second] = splitSpan(c.spans[span], t);
    if (!spanOk(first) || !spanOk(second)) {
        setWhy(why, "the point is too close to an end of the span");
        return std::nullopt;
    }
    Contour out = c;
    out.spans[span] = first;
    out.spans.insert(out.spans.begin() + static_cast<std::ptrdiff_t>(span) + 1, second);
    return out;
}

std::optional<Contour> deleteNodes(const Contour& c, const std::set<std::size_t>& nodes, std::string* why) {
    const std::size_t count = nodeCount(c);
    if (nodes.empty() || *nodes.rbegin() >= count) {
        setWhy(why, "there is no such node");
        return std::nullopt;
    }
    std::vector<std::size_t> kept;
    for (std::size_t i = 0; i < count; ++i) {
        if (!nodes.contains(i)) {
            kept.push_back(i);
        }
    }
    if (kept.size() < 2) {
        setWhy(why, "a vector needs at least two nodes");
        return std::nullopt;
    }
    const std::size_t n = c.spans.size();
    std::vector<Span> spans;
    if (c.closed) {
        for (std::size_t j = 0; j < kept.size(); ++j) {
            const std::size_t from = kept[j];
            const std::size_t to = kept[(j + 1) % kept.size()];
            std::vector<Span> run;
            std::size_t i = from;
            do {
                run.push_back(c.spans[i]);
                i = (i + 1) % n;
            } while (i != to);
            spans.push_back(mergeRun(run));
        }
        bool allLines = true;
        for (const Span& s : spans) {
            allLines = allLines && std::holds_alternative<LineSpan>(s);
        }
        if (allLines && spans.size() < 3) {
            setWhy(why, "a closed vector of lines needs at least three nodes");
            return std::nullopt;
        }
    } else {
        for (std::size_t j = 0; j + 1 < kept.size(); ++j) {
            std::vector<Span> run(c.spans.begin() + static_cast<std::ptrdiff_t>(kept[j]),
                                  c.spans.begin() + static_cast<std::ptrdiff_t>(kept[j + 1]));
            spans.push_back(mergeRun(run));
        }
    }
    Contour out = withSpans(c, std::move(spans));
    if (!validContour(out, why)) {
        return std::nullopt;
    }
    return out;
}

std::vector<Contour> deleteSpans(const Contour& c, const std::set<std::size_t>& spans) {
    std::vector<Contour> out;
    const std::size_t n = c.spans.size();
    if (spans.empty() || *spans.rbegin() >= n || spans.size() >= n) {
        return out;
    }
    // Start scanning right after a removed span so that no kept run wraps around the scan origin.
    const std::size_t origin = c.closed ? (*spans.begin() + 1) % n : 0;
    Contour current;
    for (std::size_t k = 0; k < n; ++k) {
        const std::size_t i = (origin + k) % n;
        if (spans.contains(i)) {
            if (!current.spans.empty()) {
                out.push_back(std::move(current));
                current = Contour{};
            }
        } else {
            current.spans.push_back(c.spans[i]);
        }
    }
    if (!current.spans.empty()) {
        out.push_back(std::move(current));
    }
    return out;
}

std::vector<Contour> cutAtNodes(const Contour& c, const std::set<std::size_t>& nodes, std::string* why) {
    std::vector<Contour> out;
    const std::size_t n = c.spans.size();
    if (nodes.empty() || *nodes.rbegin() >= nodeCount(c)) {
        setWhy(why, "there is no such node");
        return out;
    }
    auto slice = [&](std::size_t from, std::size_t to) {  // spans [from, to), cyclic when to <= from
        Contour piece;
        std::size_t i = from;
        do {
            piece.spans.push_back(c.spans[i]);
            i = (i + 1) % n;
        } while (i != to);
        return piece;
    };
    if (c.closed) {
        const std::vector<std::size_t> cuts(nodes.begin(), nodes.end());
        for (std::size_t j = 0; j < cuts.size(); ++j) {
            const std::size_t to = cuts[(j + 1) % cuts.size()];
            out.push_back(slice(cuts[j], to));
        }
        return out;
    }
    if (*nodes.begin() == 0 || *nodes.rbegin() == n) {
        setWhy(why, "the end points of an open vector cannot be cut");
        return out;
    }
    std::size_t from = 0;
    for (const std::size_t cut : nodes) {
        Contour piece;
        piece.spans.assign(c.spans.begin() + static_cast<std::ptrdiff_t>(from),
                           c.spans.begin() + static_cast<std::ptrdiff_t>(cut));
        out.push_back(std::move(piece));
        from = cut;
    }
    Contour tail;
    tail.spans.assign(c.spans.begin() + static_cast<std::ptrdiff_t>(from), c.spans.end());
    out.push_back(std::move(tail));
    return out;
}

std::optional<Contour> convertSpan(const Contour& c, std::size_t span, const ConvertSpec& spec,
                                   std::size_t* produced, std::string* why) {
    if (span >= c.spans.size()) {
        setWhy(why, "there is no such span");
        return std::nullopt;
    }
    const Span& s = c.spans[span];
    const Vec2 a = geometry::startPoint(s);
    const Vec2 b = geometry::endPoint(s);
    std::vector<Span> repl;
    switch (spec.target) {
        case SpanTarget::Line:
            repl.emplace_back(LineSpan{a, b});
            break;
        case SpanTarget::Bezier: {
            if (const auto* arc = std::get_if<ArcSpan>(&s)) {
                for (const BezierSpan& piece : arc->toBeziers()) {
                    repl.emplace_back(piece);
                }
            } else if (isBezier(s)) {
                repl.push_back(s);
            } else if (spec.through.has_value()) {
                // Quadratic control Q with B(0.5) = through, raised to a cubic.
                const Vec2 q = *spec.through * 2.0 - (a + b) * 0.5;
                repl.emplace_back(BezierSpan{a, a + (q - a) * (2.0 / 3.0), b + (q - b) * (2.0 / 3.0), b});
            } else {
                repl.emplace_back(lineToBezier(std::get<LineSpan>(s)));
            }
            break;
        }
        case SpanTarget::Arc: {
            std::optional<ArcSpan> arc;
            if (const auto* existing = std::get_if<ArcSpan>(&s)) {
                arc = *existing;
            } else if (spec.through.has_value()) {
                arc = arcThrough(a, *spec.through, b);
            } else if (spec.bulge.has_value()) {
                arc = arcFromBulge(a, b, *spec.bulge);
            } else if (isBezier(s)) {
                arc = arcThrough(a, geometry::pointAt(s, 0.5), b);
            } else {
                arc = arcFromBulge(a, b, kDefaultBulge);
            }
            if (!arc.has_value()) {
                setWhy(why, "no arc passes through those points");
                return std::nullopt;
            }
            repl.emplace_back(*arc);
            break;
        }
    }
    Contour out = c;
    out.spans.erase(out.spans.begin() + static_cast<std::ptrdiff_t>(span));
    out.spans.insert(out.spans.begin() + static_cast<std::ptrdiff_t>(span), repl.begin(), repl.end());
    if (produced != nullptr) {
        *produced = repl.size();
    }
    if (!validContour(out, why)) {
        return std::nullopt;
    }
    return out;
}

std::optional<Contour> smoothNodes(const Contour& c, const std::set<std::size_t>& nodes, std::string* why) {
    const std::size_t count = nodeCount(c);
    if (nodes.empty() || *nodes.rbegin() >= count) {
        setWhy(why, "there is no such node");
        return std::nullopt;
    }
    Contour work = c;
    bool changed = false;
    bool cusp = false;
    bool anyJoint = false;
    for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
        const std::size_t node = *it;
        const std::size_t n = work.spans.size();
        if (!work.closed && (node == 0 || node >= count - 1)) {
            continue;  // the ends of an open vector have no continuity
        }
        // After arc conversions earlier in this loop the span count may have grown; nodes are
        // processed from the highest, so the spans below `node` are still where they were.
        const std::size_t out = node % n;
        std::size_t in = node == 0 ? n - 1 : node - 1;
        if (in == out) {
            continue;
        }
        anyJoint = true;
        Span& si = work.spans[in];
        Span& so = work.spans[out];
        const Vec2 pin = tangentIn(si);
        const Vec2 pout = tangentOut(so);
        const double cross = pin.cross(pout);
        const double dot = pin.dot(pout);
        const double angle = std::atan2(std::fabs(cross), dot);
        if (angle < 1e-9) {
            continue;
        }
        if (kPi - angle < 1e-9) {
            cusp = true;
            continue;
        }
        const Vec2 at = geometry::endPoint(si);
        auto handleLength = [](const BezierSpan& b, bool atEnd) {
            const double len = atEnd ? geometry::distance(b.p3, b.p2) : geometry::distance(b.p0, b.p1);
            return len > 0.0 ? len : geometry::distance(b.p0, b.p3) / 3.0;
        };
        const bool bezIn = isBezier(si);
        const bool bezOut = isBezier(so);
        const bool arcIn = std::holds_alternative<ArcSpan>(si);
        const bool arcOut = std::holds_alternative<ArcSpan>(so);
        if (!bezIn && !bezOut) {
            if (arcIn && arcOut) {
                // Keep the incoming arc; the outgoing one becomes Beziers.
                const std::vector<BezierSpan> pieces = std::get<ArcSpan>(so).toBeziers();
                work.spans.erase(work.spans.begin() + static_cast<std::ptrdiff_t>(out));
                work.spans.insert(work.spans.begin() + static_cast<std::ptrdiff_t>(out), pieces.begin(), pieces.end());
                if (in > out) {
                    in += pieces.size() - 1;
                }
            } else {
                // A line next to an arc becomes a Bezier; two lines both do.
                if (!arcIn) {
                    work.spans[in] = lineToBezier(std::get<LineSpan>(work.spans[in]));
                }
                if (!arcOut) {
                    work.spans[out] = lineToBezier(std::get<LineSpan>(work.spans[out]));
                }
            }
        }
        Span& ci = work.spans[in];
        Span& co = work.spans[out];
        auto* bi = std::get_if<BezierSpan>(&ci);
        auto* bo = std::get_if<BezierSpan>(&co);
        const Vec2 ti = tangentIn(ci);
        const Vec2 to = tangentOut(co);
        if (bi != nullptr && bo != nullptr) {
            const Vec2 dir = unitOr(ti + to, to);
            bi->p2 = at - dir * handleLength(*bi, true);
            bo->p1 = at + dir * handleLength(*bo, false);
        } else if (bi != nullptr) {
            bi->p2 = at - to * handleLength(*bi, true);
        } else if (bo != nullptr) {
            bo->p1 = at + ti * handleLength(*bo, false);
        }
        changed = true;
    }
    if (!changed) {
        setWhy(why, cusp ? "a cusp cannot be smoothed"
                         : anyJoint ? "the selected nodes are already smooth" : "end points have no tangent to smooth");
        return std::nullopt;
    }
    if (!validContour(work, why)) {
        return std::nullopt;
    }
    return work;
}

std::optional<Contour> rotateStart(const Contour& c, std::size_t node, std::string* why) {
    if (!c.closed) {
        setWhy(why, "the start point can only be moved on a closed vector");
        return std::nullopt;
    }
    if (node >= c.spans.size()) {
        setWhy(why, "there is no such node");
        return std::nullopt;
    }
    if (node == 0) {
        setWhy(why, "that node already is the start point");
        return std::nullopt;
    }
    Contour out = c;
    std::rotate(out.spans.begin(), out.spans.begin() + static_cast<std::ptrdiff_t>(node), out.spans.end());
    return out;
}

}  // namespace carvetto::viewmodel::nodeops
