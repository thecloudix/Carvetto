#include "carvetto/toolpath/keepout.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"

namespace carvetto::toolpath {

using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;
using geometry::Vec3;

namespace {

// A point this close to a region edge counts as on the boundary, i.e. outside (mm).
constexpr double kBoundaryEps = 1e-9;

double distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 d = b - a;
    const double len2 = d.dot(d);
    double t = len2 > 0.0 ? (p - a).dot(d) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    return geometry::distance(p, a + d * t);
}

}  // namespace

KeepOutField::KeepOutField(const std::vector<Contour>& zones, double clearance, const geometry::Tolerance& tol) {
    if (!std::isfinite(clearance) || clearance < 0.0) {
        clearance = 0.0;
    }
    std::vector<Contour> closed;
    for (const Contour& c : zones) {
        if (c.closed && c.spans.size() >= 1) {
            closed.push_back(c);
        }
    }
    if (closed.empty()) {
        return;
    }
    std::vector<Contour> region = geometry::unionContours(closed, tol);
    if (clearance > 0.0) {
        region = geometry::offsetContours(region, clearance, geometry::JoinType::Round, tol);
    }
    for (const Contour& c : region) {
        const geometry::Polyline poly = c.polygonise(tol);
        const std::vector<Vec2>& pts = poly.points;
        if (pts.size() < 3) {
            continue;
        }
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2 a = pts[i];
            const Vec2 b = pts[(i + 1) % pts.size()];
            Edge e{a, b, Box2{}};
            e.box.expand(a);
            e.box.expand(b);
            edges_.push_back(e);
            bounds_.expand(a);
            vertices_.push_back(a);
        }
    }
}

bool KeepOutField::contains(Vec2 p) const {
    if (edges_.empty() || !bounds_.contains(p)) {
        return false;
    }
    bool inside = false;
    for (const Edge& e : edges_) {
        if (distToSegment(p, e.a, e.b) <= kBoundaryEps) {
            return false;
        }
        // Even-odd crossing of the ray towards +x.
        if ((e.a.y > p.y) != (e.b.y > p.y)) {
            const double x = e.a.x + (p.y - e.a.y) / (e.b.y - e.a.y) * (e.b.x - e.a.x);
            if (x > p.x) {
                inside = !inside;
            }
        }
    }
    return inside;
}

bool KeepOutField::segmentClear(Vec2 a, Vec2 b) const {
    Box2 sb;
    sb.expand(a);
    sb.expand(b);
    if (edges_.empty() || !bounds_.intersects(sb)) {
        return true;
    }
    const Vec2 d = b - a;
    std::vector<double> ts{0.0, 1.0};
    for (const Edge& e : edges_) {
        if (!e.box.intersects(sb)) {
            continue;
        }
        const Vec2 f = e.b - e.a;
        const double denom = d.cross(f);
        if (std::fabs(denom) <= 1e-18 * std::max(1.0, std::sqrt(d.dot(d) * f.dot(f)))) {
            continue;  // parallel: any overlap is on the boundary line, handled by the midpoint test
        }
        const Vec2 w = e.a - a;
        const double t = w.cross(f) / denom;
        const double u = w.cross(d) / denom;
        if (t > 0.0 && t < 1.0 && u >= 0.0 && u <= 1.0) {
            ts.push_back(t);
        }
    }
    std::sort(ts.begin(), ts.end());
    for (std::size_t i = 0; i + 1 < ts.size(); ++i) {
        if (ts[i + 1] - ts[i] <= 1e-12) {
            continue;
        }
        const double m = 0.5 * (ts[i] + ts[i + 1]);
        if (contains(a + d * m)) {
            return false;
        }
    }
    return true;
}

bool KeepOutField::blocks(Vec2 a, Vec2 b) const {
    return !segmentClear(a, b);
}

bool KeepOutField::vertexVisible(std::size_t i, std::size_t j) {
    if (i > j) {
        std::swap(i, j);
    }
    const std::uint64_t key = (static_cast<std::uint64_t>(i) << 32) | static_cast<std::uint64_t>(j);
    const auto it = visibleMemo_.find(key);
    if (it != visibleMemo_.end()) {
        return it->second;
    }
    const bool v = segmentClear(vertices_[i], vertices_[j]);
    visibleMemo_.emplace(key, v);
    return v;
}

RouteResult KeepOutField::route(Vec2 a, Vec2 b) {
    RouteResult out;
    if (segmentClear(a, b)) {
        return out;
    }
    if (contains(a) || contains(b)) {
        out.status = RouteStatus::EndpointInside;
        return out;
    }
    // Nodes: 0..n-1 region vertices, n = a, n+1 = b. Dense Dijkstra, edges tested lazily.
    const std::size_t n = vertices_.size();
    const std::size_t src = n;
    const std::size_t dst = n + 1;
    const auto pos = [&](std::size_t i) { return i == src ? a : i == dst ? b : vertices_[i]; };
    const auto visible = [&](std::size_t i, std::size_t j) {
        if (i < n && j < n) {
            return vertexVisible(i, j);
        }
        return segmentClear(pos(i), pos(j));
    };
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(n + 2, kInf);
    std::vector<std::size_t> prev(n + 2, n + 2);
    std::vector<bool> done(n + 2, false);
    dist[src] = 0.0;
    for (;;) {
        std::size_t u = n + 2;
        double best = kInf;
        for (std::size_t i = 0; i < n + 2; ++i) {
            if (!done[i] && dist[i] < best) {
                best = dist[i];
                u = i;
            }
        }
        if (u == n + 2) {
            break;
        }
        done[u] = true;
        if (u == dst) {
            break;
        }
        for (std::size_t v = 0; v < n + 2; ++v) {
            if (done[v] || v == u || (u == src && v == dst)) {
                continue;
            }
            const double nd = dist[u] + geometry::distance(pos(u), pos(v));
            if (nd < dist[v] && visible(u, v)) {
                dist[v] = nd;
                prev[v] = u;
            }
        }
    }
    if (!done[dst]) {
        out.status = RouteStatus::NoRoute;
        return out;
    }
    std::vector<Vec2> rev;
    for (std::size_t v = dst; v != src; v = prev[v]) {
        rev.push_back(pos(v));
    }
    out.points.assign(rev.rbegin(), rev.rend());
    out.status = RouteStatus::Routed;
    return out;
}

RoutingReport routeRapids(Toolpath& tp, KeepOutField& field, std::optional<Vec3> from) {
    RoutingReport report;
    if (field.empty() || tp.moves.empty()) {
        return report;
    }
    std::vector<Move> out;
    out.reserve(tp.moves.size());
    std::optional<Vec3> pos = from;
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Rapid && pos) {
            const RouteResult r = field.route(pos->xy(), m.to.xy());
            if (r.status == RouteStatus::Routed) {
                const double zHigh = std::max(pos->z, m.to.z);
                for (std::size_t i = 0; i < r.points.size(); ++i) {
                    const bool last = i + 1 == r.points.size();
                    out.push_back(Move::rapid(last ? m.to : Vec3{r.points[i], zHigh},
                                              i == 0 ? RapidRoute::DetourFirst : RapidRoute::DetourNext));
                }
                ++report.routed;
                pos = m.to;
                continue;
            }
            if (r.status != RouteStatus::Clear) {
                ++report.unroutable;
            }
        }
        out.push_back(m);
        pos = m.to;
    }
    tp.moves = std::move(out);
    return report;
}

std::size_t countBlockedRapids(const Toolpath& tp, const KeepOutField& field, std::optional<Vec3> from) {
    std::size_t n = 0;
    std::optional<Vec3> pos = from;
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Rapid && pos && field.blocks(pos->xy(), m.to.xy())) {
            ++n;
        }
        pos = m.to;
    }
    return n;
}

}  // namespace carvetto::toolpath
