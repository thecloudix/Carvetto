// Independent oracle for the T-0120 verification: own distance and point-in-region code.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <limits>
#include <random>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"

namespace v {
using namespace carvetto;
using geometry::Contour;
using geometry::Span;
using geometry::Vec2;
using toolpath::Move;
using toolpath::MoveKind;
constexpr double PI = 3.14159265358979323846;

inline double segDist(Vec2 p, Vec2 a, Vec2 b) {
    double dx = b.x - a.x, dy = b.y - a.y, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / l2 : 0;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.x - a.x - t * dx, p.y - a.y - t * dy);
}
// own sweep: CCW positive (0,2pi], CW negative
inline double mySweep(const geometry::ArcSpan& a) {
    double raw = a.direction == geometry::ArcDirection::CCW ? a.endAngle - a.startAngle : a.startAngle - a.endAngle;
    raw = std::fmod(raw, 2 * PI);
    if (raw < 0) raw += 2 * PI;
    if (raw <= 1e-15) raw = 2 * PI;
    return a.direction == geometry::ArcDirection::CCW ? raw : -raw;
}
inline double arcDist(Vec2 p, const geometry::ArcSpan& a) {
    double sw = mySweep(a);
    double ang = std::atan2(p.y - a.centre.y, p.x - a.centre.x);
    double rel = (ang - a.startAngle) * (sw >= 0 ? 1 : -1);
    rel = std::fmod(rel, 2 * PI);
    if (rel < 0) rel += 2 * PI;
    double rp = std::hypot(p.x - a.centre.x, p.y - a.centre.y);
    double best = 1e300;
    if (rel <= std::fabs(sw)) best = std::fabs(rp - a.radius);
    Vec2 s{a.centre.x + a.radius * std::cos(a.startAngle), a.centre.y + a.radius * std::sin(a.startAngle)};
    Vec2 e{a.centre.x + a.radius * std::cos(a.startAngle + sw), a.centre.y + a.radius * std::sin(a.startAngle + sw)};
    best = std::min({best, std::hypot(p.x - s.x, p.y - s.y), std::hypot(p.x - e.x, p.y - e.y)});
    return best;
}
inline Vec2 bez(const geometry::BezierSpan& b, double t) {
    double u = 1 - t;
    return b.p0 * (u * u * u) + b.p1 * (3 * u * u * t) + b.p2 * (3 * u * t * t) + b.p3 * (t * t * t);
}

// Pre-processed obstacle span with bbox (and polyline for Bezier / side tests).
struct OSpan {
    int kind;  // 0 line 1 arc 2 bezier
    Vec2 a, b;
    geometry::ArcSpan arc;
    std::vector<Vec2> poly;  // bezier polyline
    double minx, miny, maxx, maxy;
};
struct Obstacles {
    std::vector<OSpan> spans;
    std::vector<std::vector<Vec2>> closedPolys;  // for even-odd
    std::vector<std::array<double, 4>> polyBox;
    explicit Obstacles(const std::vector<Contour>& cs) {
        for (const Contour& c : cs) {
            std::vector<Vec2> poly;
            for (const Span& s : c.spans) {
                OSpan o{};
                if (auto* l = std::get_if<geometry::LineSpan>(&s)) {
                    o.kind = 0;
                    o.a = l->start;
                    o.b = l->end;
                    o.minx = std::min(o.a.x, o.b.x);
                    o.maxx = std::max(o.a.x, o.b.x);
                    o.miny = std::min(o.a.y, o.b.y);
                    o.maxy = std::max(o.a.y, o.b.y);
                    poly.push_back(l->start);
                } else if (auto* a = std::get_if<geometry::ArcSpan>(&s)) {
                    o.kind = 1;
                    o.arc = *a;
                    o.minx = a->centre.x - a->radius;
                    o.maxx = a->centre.x + a->radius;
                    o.miny = a->centre.y - a->radius;
                    o.maxy = a->centre.y + a->radius;
                    double sw = mySweep(*a);
                    int n = std::max(8, (int)std::ceil(std::fabs(sw) * a->radius / 0.01));
                    for (int k = 0; k < n; ++k) {
                        double t = a->startAngle + sw * k / n;
                        poly.push_back({a->centre.x + a->radius * std::cos(t), a->centre.y + a->radius * std::sin(t)});
                    }
                } else if (auto* b = std::get_if<geometry::BezierSpan>(&s)) {
                    o.kind = 2;
                    o.minx = std::min({b->p0.x, b->p1.x, b->p2.x, b->p3.x});
                    o.maxx = std::max({b->p0.x, b->p1.x, b->p2.x, b->p3.x});
                    o.miny = std::min({b->p0.y, b->p1.y, b->p2.y, b->p3.y});
                    o.maxy = std::max({b->p0.y, b->p1.y, b->p2.y, b->p3.y});
                    for (int k = 0; k <= 3000; ++k) o.poly.push_back(bez(*b, k / 3000.0));
                    for (int k = 0; k < 3000; ++k) poly.push_back(o.poly[(size_t)k]);
                }
                spans.push_back(std::move(o));
            }
            if (c.closed) {
                std::array<double, 4> bx{1e300, 1e300, -1e300, -1e300};
                for (auto& p : poly) {
                    bx[0] = std::min(bx[0], p.x);
                    bx[1] = std::min(bx[1], p.y);
                    bx[2] = std::max(bx[2], p.x);
                    bx[3] = std::max(bx[3], p.y);
                }
                closedPolys.push_back(std::move(poly));
                polyBox.push_back(bx);
            }
        }
    }
    // distance capped at `cap`
    double dist(Vec2 p, double cap) const {
        double best = cap;
        for (const OSpan& o : spans) {
            double dx = std::max({o.minx - p.x, 0.0, p.x - o.maxx});
            double dy = std::max({o.miny - p.y, 0.0, p.y - o.maxy});
            if (std::hypot(dx, dy) >= best) continue;
            double d;
            if (o.kind == 0)
                d = segDist(p, o.a, o.b);
            else if (o.kind == 1)
                d = arcDist(p, o.arc);
            else {
                d = 1e300;
                for (size_t k = 0; k + 1 < o.poly.size(); ++k) d = std::min(d, segDist(p, o.poly[k], o.poly[k + 1]));
            }
            best = std::min(best, d);
        }
        return best;
    }
    // number of closed contours containing p (even-odd crossing per contour)
    int depth(Vec2 p) const {
        int n = 0;
        for (size_t i = 0; i < closedPolys.size(); ++i) {
            auto& b = polyBox[i];
            if (p.x < b[0] || p.x > b[2] || p.y < b[1] || p.y > b[3]) continue;
            const auto& pl = closedPolys[i];
            bool in = false;
            for (size_t k = 0, j = pl.size() - 1; k < pl.size(); j = k++) {
                if (((pl[k].y > p.y) != (pl[j].y > p.y)) &&
                    (p.x < (pl[j].x - pl[k].x) * (p.y - pl[k].y) / (pl[j].y - pl[k].y) + pl[k].x))
                    in = !in;
            }
            n += in;
        }
        return n;
    }
};

// Sample every non-rapid move in XY; f(point, z, moveIndex)
template <class F>
void sampleMoves(const toolpath::Toolpath& tp, double step, F&& f) {
    for (size_t i = 1; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        if (m.kind == MoveKind::Rapid || m.kind == MoveKind::Dwell) continue;
        auto A = tp.moves[i - 1].to, B = m.to;
        if (toolpath::isArc(m.kind)) {
            Vec2 a = A.xy(), b = B.xy(), c = m.centre;
            double r = std::hypot(a.x - c.x, a.y - c.y);
            double a0 = std::atan2(a.y - c.y, a.x - c.x), a1 = std::atan2(b.y - c.y, b.x - c.x);
            double sw = a1 - a0;
            if (m.kind == MoveKind::ArcCCW) {
                while (sw <= 0) sw += 2 * PI;
            } else {
                while (sw >= 0) sw -= 2 * PI;
            }
            if (a.x == b.x && a.y == b.y) sw = m.kind == MoveKind::ArcCCW ? 2 * PI : -2 * PI;
            int n = std::max(1, (int)std::ceil(std::fabs(sw) * r / step));
            for (int k = 0; k <= n; ++k) {
                double t = a0 + sw * k / n;
                f(Vec2{c.x + r * std::cos(t), c.y + r * std::sin(t)}, A.z + (B.z - A.z) * k / n, i);
            }
        } else {
            double len = std::hypot(B.x - A.x, B.y - A.y);
            int n = std::max(1, (int)std::ceil(len / step));
            for (int k = 0; k <= n; ++k) {
                double t = (double)k / n;
                f(Vec2{A.x + (B.x - A.x) * t, A.y + (B.y - A.y) * t}, A.z + (B.z - A.z) * t, i);
            }
        }
    }
}

inline tooling::Tool endMill(double d) {
    tooling::Tool t;
    t.id = "em";
    t.name = "EM";
    t.type = tooling::ToolType::EndMill;
    t.diameter = d;
    t.defaultCuttingData.passDepth = 4.0;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 18000.0;
    t.defaultCuttingData.toolNumber = 1;
    return t;
}
inline tooling::Tool vbit(double d, double angleDeg) {
    tooling::Tool t = endMill(d);
    t.type = tooling::ToolType::VBit;
    t.includedAngle = angleDeg * PI / 180;
    return t;
}

inline std::string fmt(const toolpath::Toolpath& tp) {
    std::string s;
    char buf[200];
    for (auto& m : tp.moves) {
        std::snprintf(buf, sizeof buf, "%d %a %a %a %a %a\n", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x,
                      m.centre.y);
        s += buf;
    }
    return s;
}
inline const char* kn(MoveKind k) {
    switch (k) {
        case MoveKind::Rapid: return "rapid";
        case MoveKind::Feed: return "feed";
        case MoveKind::Plunge: return "plunge";
        case MoveKind::Retract: return "retract";
        case MoveKind::ArcCW: return "arc_cw";
        case MoveKind::ArcCCW: return "arc_ccw";
        case MoveKind::Dwell: return "dwell";
    }
    return "?";
}
inline void dump(const toolpath::Toolpath& tp, FILE* f = stdout) {
    for (auto& m : tp.moves) {
        std::fprintf(f, "%s %.6f %.6f %.6f", kn(m.kind), m.to.x, m.to.y, m.to.z);
        if (toolpath::isArc(m.kind)) std::fprintf(f, " c %.6f %.6f", m.centre.x, m.centre.y);
        std::fprintf(f, "\n");
    }
}
// XY length of move i
inline double xyLen(const toolpath::Toolpath& tp, size_t i) {
    const Move& m = tp.moves[i];
    auto A = tp.moves[i - 1].to.xy(), B = m.to.xy();
    if (toolpath::isArc(m.kind)) {
        Vec2 c = m.centre;
        double r = std::hypot(A.x - c.x, A.y - c.y);
        double a0 = std::atan2(A.y - c.y, A.x - c.x), a1 = std::atan2(B.y - c.y, B.x - c.x);
        double sw = a1 - a0;
        if (m.kind == MoveKind::ArcCCW) {
            while (sw <= 0) sw += 2 * PI;
        } else {
            while (sw >= 0) sw -= 2 * PI;
        }
        if (A.x == B.x && A.y == B.y) sw = 2 * PI;
        return std::fabs(sw) * r;
    }
    return std::hypot(B.x - A.x, B.y - A.y);
}
}  // namespace v
