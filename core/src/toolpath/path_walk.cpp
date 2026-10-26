#include "carvetto/toolpath/path_walk.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iterator>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour_ops.hpp"

namespace carvetto::toolpath {

using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Span;

namespace {

// Pieces shorter than this (mm) carry no geometry worth a move.
constexpr double kTinyLength = 1e-9;

Span pieceOf(const Span& span, double t0, double t1) {
    if (t0 <= 0.0 && t1 >= 1.0) {
        return span;
    }
    return std::visit(
        [&](const auto& s) -> Span {
            using S = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<S, LineSpan>) {
                return LineSpan{s.pointAt(t0), s.pointAt(t1)};
            } else if constexpr (std::is_same_v<S, ArcSpan>) {
                return ArcSpan{s.centre, s.radius, s.angleAt(t0), s.angleAt(t1), s.direction};
            } else {
                // Not reached: Beziers are flattened on construction.
                return LineSpan{s.pointAt(t0), s.pointAt(t1)};
            }
        },
        span);
}

}  // namespace

ArcLengthPath::ArcLengthPath(const Contour& contour, const geometry::Tolerance& tol) {
    contour_.closed = contour.closed;
    for (const Span& s : contour.spans) {
        if (std::holds_alternative<BezierSpan>(s)) {
            std::vector<geometry::Vec2> pts;
            geometry::polygoniseInto(s, tol, pts);
            geometry::Vec2 prev = geometry::startPoint(s);
            for (const geometry::Vec2& p : pts) {
                contour_.spans.emplace_back(LineSpan{prev, p});
                prev = p;
            }
        } else {
            contour_.spans.push_back(s);
        }
    }
    cum_.reserve(contour_.spans.size() + 1);
    cum_.push_back(0.0);
    for (const Span& s : contour_.spans) {
        cum_.push_back(cum_.back() + geometry::length(s));
    }
}

double ArcLengthPath::wrap(double u) const {
    const double len = length();
    if (len <= 0.0) {
        return 0.0;
    }
    if (!closed()) {
        return std::clamp(u, 0.0, len);
    }
    double w = std::fmod(u, len);
    if (w < 0.0) {
        w += len;
    }
    if (w >= len) {
        w = 0.0;
    }
    return w;
}

std::size_t ArcLengthPath::spanAt(double u) const {
    // Last span whose start is <= u.
    const auto it = std::upper_bound(cum_.begin(), cum_.end() - 1, u);
    const auto idx = static_cast<std::size_t>(std::distance(cum_.begin(), it));
    return idx == 0 ? 0 : std::min(idx - 1, contour_.spans.size() - 1);
}

geometry::Vec2 ArcLengthPath::pointAt(double u) const {
    if (isEmpty()) {
        return {};
    }
    const double w = wrap(u);
    const std::size_t i = spanAt(w);
    const double len = spanLength(i);
    const double t = len > 0.0 ? std::clamp((w - cum_[i]) / len, 0.0, 1.0) : 0.0;
    return geometry::pointAt(contour_.spans[i], t);
}

geometry::Vec2 ArcLengthPath::tangentAt(double u) const {
    if (isEmpty()) {
        return {};
    }
    const double w = wrap(u);
    std::size_t i = spanAt(w);
    // Skip zero-length spans so the tangent is meaningful.
    for (std::size_t k = 0; k < contour_.spans.size() && spanLength(i) <= kTinyLength; ++k) {
        i = (i + 1) % contour_.spans.size();
    }
    const double len = spanLength(i);
    const double t = len > 0.0 ? std::clamp((w - cum_[i]) / len, 0.0, 1.0) : 0.0;
    return geometry::tangentAt(contour_.spans[i], t);
}

std::vector<Span> ArcLengthPath::extract(double u0, double u1) const {
    std::vector<Span> out;
    if (isEmpty()) {
        return out;
    }
    if (u1 < u0) {
        out = extract(u1, u0);
        std::reverse(out.begin(), out.end());
        for (Span& s : out) {
            s = geometry::reversed(s);
        }
        return out;
    }
    const double len = length();
    if (len <= kTinyLength) {
        return out;
    }
    double a = u0;
    double b = u1;
    if (!closed()) {
        a = std::clamp(a, 0.0, len);
        b = std::clamp(b, 0.0, len);
    } else {
        const double k = std::floor(a / len);
        a -= k * len;
        b -= k * len;
        if (a >= len) {  // rounding
            a -= len;
            b -= len;
        }
    }
    if (b - a <= kTinyLength) {
        return out;
    }
    // Walk spans from the one containing `a`, wrapping (closed) with an offset of `base`.
    double base = 0.0;
    std::size_t i = spanAt(a);
    const std::size_t n = contour_.spans.size();
    // Bound the walk: whole revolutions plus one.
    const std::size_t maxSteps = n * (static_cast<std::size_t>((b - a) / len) + 2);
    for (std::size_t step = 0; step < maxSteps; ++step) {
        const double s0 = base + cum_[i];
        const double s1 = base + cum_[i + 1];
        const double l = s1 - s0;
        const double from = std::max(a, s0);
        const double to = std::min(b, s1);
        if (to - from > kTinyLength && l > 0.0) {
            const double t0 = (from - s0) / l;
            const double t1 = (to - s0) / l;
            out.push_back(pieceOf(contour_.spans[i], t0 <= 1e-15 ? 0.0 : t0, t1 >= 1.0 - 1e-15 ? 1.0 : t1));
        }
        if (s1 >= b) {
            break;
        }
        ++i;
        if (i == n) {
            if (!closed()) {
                break;
            }
            i = 0;
            base += len;
        }
    }
    // Re-join consecutive pieces of one circle (a revolution of a one-arc path started mid-way
    // comes out as two pieces), as long as the joined arc does not exceed a full turn.
    std::vector<Span> merged;
    for (Span& s : out) {
        if (!merged.empty()) {
            auto* pa = std::get_if<ArcSpan>(&merged.back());
            const auto* pb = std::get_if<ArcSpan>(&s);
            if (pa != nullptr && pb != nullptr && pa->centre == pb->centre && pa->radius == pb->radius &&
                pa->direction == pb->direction) {
                const double sa = pa->sweepAngle();
                const double sb = pb->sweepAngle();
                const double gap = std::remainder(pb->startAngle - (pa->startAngle + sa), geometry::kTwoPi);
                if (std::fabs(gap) <= 1e-12 && std::fabs(sa + sb) <= geometry::kTwoPi + 1e-12) {
                    // A whole turn is stored with equal angles (the ArcSpan full-circle form).
                    pa->endAngle = std::fabs(sa + sb) >= geometry::kTwoPi - 1e-12 ? pa->startAngle
                                                                                   : pa->startAngle + sa + sb;
                    continue;
                }
            }
        }
        merged.push_back(std::move(s));
    }
    return merged;
}

double ArcLengthPath::project(geometry::Vec2 p) const {
    if (isEmpty()) {
        return 0.0;
    }
    const geometry::ContourClosestPoint cp = geometry::closestPoint(contour_, p);
    const double u = cum_[cp.spanIndex] + cp.t * spanLength(cp.spanIndex);
    return wrap(u);
}

std::vector<ArcLengthPath::Kink> ArcLengthPath::kinks(double minAngle) const {
    std::vector<Kink> out;
    const std::size_t n = contour_.spans.size();
    if (n == 0) {
        return out;
    }
    const std::size_t last = closed() ? n : n - 1;
    for (std::size_t j = 0; j < last; ++j) {
        const std::size_t next = (j + 1) % n;
        if (spanLength(j) <= kTinyLength || spanLength(next) <= kTinyLength) {
            continue;
        }
        const geometry::Vec2 a = geometry::tangentAt(contour_.spans[j], 1.0);
        const geometry::Vec2 b = geometry::tangentAt(contour_.spans[next], 0.0);
        const double turn = std::atan2(a.cross(b), a.dot(b));
        if (std::fabs(turn) > minAngle) {
            out.push_back({next == 0 ? 0.0 : cum_[next], turn});
        }
    }
    std::sort(out.begin(), out.end(), [](const Kink& x, const Kink& y) { return x.u < y.u; });
    return out;
}

// ------------------------------------------------------------------------------- ZProfile

ZProfile ZProfile::linear(double z0, double z1, double length) {
    if (length <= 0.0) {
        return ZProfile{{ZKnot{0.0, z0, z1}}};
    }
    return ZProfile{{ZKnot{0.0, z0, z0}, ZKnot{length, z1, z1}}};
}

double ZProfile::before(double s) const {
    if (knots.empty()) {
        return 0.0;
    }
    if (s <= knots.front().s) {
        return knots.front().zBefore;
    }
    // First knot with knot.s >= s.
    const auto it = std::lower_bound(knots.begin(), knots.end(), s,
                                     [](const ZKnot& k, double v) { return k.s < v; });
    if (it == knots.end()) {
        return knots.back().zAfter;
    }
    if (it->s == s) {
        return it->zBefore;
    }
    const ZKnot& hi = *it;
    const ZKnot& lo = *(it - 1);
    const double t = (s - lo.s) / (hi.s - lo.s);
    return lo.zAfter + t * (hi.zBefore - lo.zAfter);
}

double ZProfile::after(double s) const {
    if (knots.empty()) {
        return 0.0;
    }
    if (s < knots.front().s) {
        return knots.front().zBefore;
    }
    // Last knot with knot.s <= s.
    const auto it = std::upper_bound(knots.begin(), knots.end(), s,
                                     [](double v, const ZKnot& k) { return v < k.s; });
    const ZKnot& lo = *(it - 1);
    if (lo.s == s || it == knots.end()) {
        return lo.zAfter;
    }
    const ZKnot& hi = *it;
    const double t = (s - lo.s) / (hi.s - lo.s);
    return lo.zAfter + t * (hi.zBefore - lo.zAfter);
}

ZProfile ZProfile::max(const ZProfile& a, const ZProfile& b) {
    if (a.knots.empty()) {
        return b;
    }
    if (b.knots.empty()) {
        return a;
    }
    std::vector<double> ss;
    for (const ZKnot& k : a.knots) {
        ss.push_back(k.s);
    }
    for (const ZKnot& k : b.knots) {
        ss.push_back(k.s);
    }
    std::sort(ss.begin(), ss.end());
    ss.erase(std::unique(ss.begin(), ss.end()), ss.end());

    std::vector<ZKnot> raw;
    for (std::size_t i = 0; i < ss.size(); ++i) {
        const double s = ss[i];
        raw.push_back({s, std::max(a.before(s), b.before(s)), std::max(a.after(s), b.after(s))});
        if (i + 1 < ss.size()) {
            const double s1 = ss[i + 1];
            const double d0 = a.after(s) - b.after(s);
            const double d1 = a.before(s1) - b.before(s1);
            if ((d0 < 0.0 && d1 > 0.0) || (d0 > 0.0 && d1 < 0.0)) {
                const double t = d0 / (d0 - d1);
                const double sx = s + t * (s1 - s);
                if (sx > s && sx < s1) {
                    const double za = a.after(s) + t * (a.before(s1) - a.after(s));
                    raw.push_back({sx, za, za});
                }
            }
        }
    }
    // Drop knots that neither jump nor change slope (the profile is constant beyond its ends).
    ZProfile out;
    for (std::size_t i = 0; i < raw.size(); ++i) {
        const ZKnot& k = raw[i];
        if (k.zBefore == k.zAfter) {
            const bool hasPrev = !out.knots.empty();
            const double slopeIn = hasPrev ? (k.zBefore - out.knots.back().zAfter) / (k.s - out.knots.back().s) : 0.0;
            const double slopeOut =
                i + 1 < raw.size() ? (raw[i + 1].zBefore - k.zAfter) / (raw[i + 1].s - k.s) : 0.0;
            if (std::fabs(slopeIn - slopeOut) <= 1e-12) {
                continue;
            }
        }
        out.knots.push_back(k);
    }
    if (out.knots.empty()) {
        out.knots.push_back(raw.front());
    }
    return out;
}

// ------------------------------------------------------------------------------ appendLeg

namespace {

void appendVertical(Toolpath& out, double z) {
    const Vec3 at = out.moves.back().to;
    if (at.z == z) {
        return;
    }
    const Vec3 to{at.xy(), z};
    out.append(z < at.z ? Move::plunge(to) : Move::feed(to));
}

void appendSpan(const Span& span, double z, double minArcChord, Toolpath& out) {
    std::visit(
        [&](const auto& s) {
            using S = std::decay_t<decltype(s)>;
            if constexpr (std::is_same_v<S, ArcSpan>) {
                const double sweep = s.sweepAngle();
                const bool full = std::fabs(sweep) >= geometry::kTwoPi - 1e-9;
                const geometry::Vec2 from = out.isEmpty() ? s.startPoint() : out.moves.back().to.xy();
                const geometry::Vec2 end = full ? from : s.endPoint();
                if (!full && geometry::distance(s.startPoint(), end) < minArcChord) {
                    out.append(Move::feed(Vec3{end, z}));
                    return;
                }
                const Vec3 to{end, z};
                out.append(s.direction == ArcDirection::CW ? Move::arcCW(to, s.centre) : Move::arcCCW(to, s.centre));
            } else if constexpr (std::is_same_v<S, LineSpan>) {
                out.append(Move::feed(Vec3{s.end, z}));
            } else {
                out.append(Move::feed(Vec3{s.p3, z}));
            }
        },
        span);
}

}  // namespace

void appendLeg(const ArcLengthPath& path, double u0, double u1, const ZProfile& z, Toolpath& out,
               const std::vector<LegInsert>& inserts, double minArcChord, double joinTolerance) {
    if (path.isEmpty()) {
        return;
    }
    const double dir = u1 >= u0 ? 1.0 : -1.0;
    double len = std::fabs(u1 - u0);
    if (!path.closed()) {
        // Clamp the travelled distance to the path.
        const double a = path.wrap(u0);
        const double b = path.wrap(u1);
        len = std::fabs(b - a);
        u0 = a;
    }
    std::vector<double> breaks{0.0, len};
    for (const ZKnot& k : z.knots) {
        if (k.s > 0.0 && k.s < len) {
            breaks.push_back(k.s);
        }
    }
    for (const LegInsert& ins : inserts) {
        if (ins.s >= 0.0 && ins.s <= len) {
            breaks.push_back(ins.s);
        }
    }
    std::sort(breaks.begin(), breaks.end());
    breaks.erase(std::unique(breaks.begin(), breaks.end(), [](double x, double y) { return y - x <= kTinyLength; }),
                 breaks.end());
    if (breaks.back() < len) {
        breaks.back() = len;
    }

    // Each insert runs at its nearest break (breaks closer than kTinyLength were merged).
    std::vector<std::vector<const LegInsert*>> at(breaks.size());
    for (const LegInsert& ins : inserts) {
        if (ins.s < -kTinyLength || ins.s > len + kTinyLength) {
            continue;
        }
        std::size_t best = 0;
        for (std::size_t j = 1; j < breaks.size(); ++j) {
            if (std::fabs(breaks[j] - ins.s) < std::fabs(breaks[best] - ins.s)) {
                best = j;
            }
        }
        at[best].push_back(&ins);
    }
    auto runInserts = [&](std::size_t j) {
        for (const LegInsert* ins : at[j]) {
            out.moves.insert(out.moves.end(), ins->moves.begin(), ins->moves.end());
        }
    };

    // Change height where the tool stands before moving on (a closed offset may close within the
    // tolerance, so "at the start" allows `joinTolerance`): a level change is a vertical move,
    // never a slope along the first span.
    const geometry::Vec2 start = path.pointAt(u0);
    if (!out.isEmpty() && geometry::distance(out.moves.back().to.xy(), start) <= joinTolerance) {
        appendVertical(out, z.after(0.0));
    }
    runInserts(0);
    for (std::size_t i = 0; i + 1 < breaks.size(); ++i) {
        const double a = breaks[i];
        const double b = breaks[i + 1];
        const double za = z.after(a);
        const double zb = z.before(b);
        const std::vector<Span> spans = path.extract(u0 + dir * a, u0 + dir * b);
        double total = 0.0;
        for (const Span& s : spans) {
            total += geometry::length(s);
        }
        double acc = 0.0;
        for (std::size_t k = 0; k < spans.size(); ++k) {
            acc += geometry::length(spans[k]);
            const double zz = (k + 1 == spans.size() || total <= 0.0) ? zb : za + (zb - za) * (acc / total);
            appendSpan(spans[k], zz, minArcChord, out);
        }
        if (spans.empty() && !out.isEmpty()) {
            appendVertical(out, zb);
        }
        if (i + 2 < breaks.size()) {
            appendVertical(out, z.after(b));
        }
        runInserts(i + 1);
    }
}

}  // namespace carvetto::toolpath
