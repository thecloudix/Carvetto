#include "carvetto/toolpath/keepout.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <utility>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/cancel.hpp"
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
    std::vector<std::pair<std::size_t, std::size_t>> rings;
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
        const std::size_t ringStart = edges_.size();
        for (std::size_t i = 0; i < pts.size(); ++i) {
            const Vec2 a = pts[i];
            const Vec2 b = pts[(i + 1) % pts.size()];
            Edge e{a, b, Box2{}};
            e.box.expand(a);
            e.box.expand(b);
            edges_.push_back(e);
            bounds_.expand(a);
            vertices_.push_back(a);
            Link l;
            l.prev = static_cast<std::uint32_t>(ringStart + (i + pts.size() - 1) % pts.size());
            l.next = static_cast<std::uint32_t>(ringStart + (i + 1) % pts.size());
            const auto unit = [](Vec2 v) {
                const double len = std::sqrt(v.dot(v));
                return len > 0.0 ? Vec2{v.x / len, v.y / len} : Vec2{0.0, 0.0};
            };
            l.unitNext = unit(pts[(i + 1) % pts.size()] - a);
            l.unitPrev = unit(pts[(i + pts.size() - 1) % pts.size()] - a);
            // A degenerate (zero-length) neighbour edge: never prune at this vertex.
            l.prune = l.unitNext.dot(l.unitNext) > 0.5 && l.unitPrev.dot(l.unitPrev) > 0.5;
            links_.push_back(l);
        }
        rings.emplace_back(ringStart, pts.size());
    }
    buildGrid();
    // Which side of each ring is the region's interior? Probe just left and just right of the
    // first edge; when the probes do not disagree the ring is never pruned.
    for (const auto& [start, count] : rings) {
        bool decided = false;
        for (std::size_t k = 0; k < count && !decided; ++k) {
            const Edge& e = edges_[start + k];
            const Vec2 d = e.b - e.a;
            const double len = std::sqrt(d.dot(d));
            if (len <= 0.0) {
                continue;
            }
            const double delta = std::min(len * 1e-4, 1e-6);
            const Vec2 mid = (e.a + e.b) * 0.5;
            const Vec2 left{-d.y / len * delta, d.x / len * delta};
            const bool inL = contains(mid + left);
            const bool inR = contains(mid - left);
            decided = true;
            for (std::size_t i = start; i < start + count; ++i) {
                links_[i].interiorLeft = inL;
                links_[i].prune = links_[i].prune && inL != inR;
            }
        }
        if (!decided) {
            for (std::size_t i = start; i < start + count; ++i) {
                links_[i].prune = false;
            }
        }
    }
}

namespace {
constexpr double kIndexMargin = 1e-7;
}

void KeepOutField::buildGrid() {
    const std::size_t e = edges_.size();
    if (e == 0) {
        return;
    }
    const std::size_t side = std::clamp<std::size_t>(static_cast<std::size_t>(std::sqrt(static_cast<double>(e))), 1, 512);
    gridCols_ = gridRows_ = side;
    cellW_ = std::max(bounds_.width() / static_cast<double>(side), 1e-9);
    cellH_ = std::max(bounds_.height() / static_cast<double>(side), 1e-9);
    cells_.assign(side * side, {});
    stamp_.assign(e, 0);
    std::vector<std::uint32_t> ids;
    for (std::size_t i = 0; i < e; ++i) {
        // Registration visits each cell once, so no de-duplication is needed here.
        const Edge& ed = edges_[i];
        const double x0 = std::min(ed.a.x, ed.b.x) - kIndexMargin;
        const double x1 = std::max(ed.a.x, ed.b.x) + kIndexMargin;
        const auto col = [&](double x) {
            const double c = std::floor((x - bounds_.min.x) / cellW_);
            return static_cast<std::size_t>(std::clamp(c, 0.0, static_cast<double>(side - 1)));
        };
        const auto row = [&](double y) {
            const double r = std::floor((y - bounds_.min.y) / cellH_);
            return static_cast<std::size_t>(std::clamp(r, 0.0, static_cast<double>(side - 1)));
        };
        const Vec2 d = ed.b - ed.a;
        for (std::size_t c = col(x0); c <= col(x1); ++c) {
            const double cl = std::max(x0, bounds_.min.x + static_cast<double>(c) * cellW_);
            const double cr = std::min(x1, bounds_.min.x + static_cast<double>(c + 1) * cellW_);
            double ylo = std::min(ed.a.y, ed.b.y);
            double yhi = std::max(ed.a.y, ed.b.y);
            if (std::fabs(d.x) > 0.0) {
                const double ya = ed.a.y + (std::clamp(cl, x0 + kIndexMargin, x1 - kIndexMargin) - ed.a.x) / d.x * d.y;
                const double yb = ed.a.y + (std::clamp(cr, x0 + kIndexMargin, x1 - kIndexMargin) - ed.a.x) / d.x * d.y;
                ylo = std::min(ya, yb);
                yhi = std::max(ya, yb);
            }
            for (std::size_t r = row(ylo - kIndexMargin); r <= row(yhi + kIndexMargin); ++r) {
                cells_[r * side + c].push_back(static_cast<std::uint32_t>(i));
            }
        }
    }
}

void KeepOutField::gather(Vec2 p, Vec2 q, double margin, std::vector<std::uint32_t>& out) const {
    out.clear();
    const std::size_t side = gridCols_;
    if (++stampGen_ == 0) {
        std::fill(stamp_.begin(), stamp_.end(), 0U);
        stampGen_ = 1;
    }
    const auto col = [&](double x) {
        const double c = std::floor((x - bounds_.min.x) / cellW_);
        return static_cast<std::size_t>(std::clamp(c, 0.0, static_cast<double>(side - 1)));
    };
    const auto row = [&](double y) {
        const double r = std::floor((y - bounds_.min.y) / cellH_);
        return static_cast<std::size_t>(std::clamp(r, 0.0, static_cast<double>(side - 1)));
    };
    const double x0 = std::min(p.x, q.x) - margin;
    const double x1 = std::max(p.x, q.x) + margin;
    const Vec2 d = q - p;
    for (std::size_t c = col(x0); c <= col(x1); ++c) {
        const double cl = std::max(x0, bounds_.min.x + static_cast<double>(c) * cellW_);
        const double cr = std::min(x1, bounds_.min.x + static_cast<double>(c + 1) * cellW_);
        double ylo = std::min(p.y, q.y);
        double yhi = std::max(p.y, q.y);
        if (std::fabs(d.x) > 0.0) {
            const double ya = p.y + (std::clamp(cl, std::min(p.x, q.x), std::max(p.x, q.x)) - p.x) / d.x * d.y;
            const double yb = p.y + (std::clamp(cr, std::min(p.x, q.x), std::max(p.x, q.x)) - p.x) / d.x * d.y;
            ylo = std::min(ya, yb);
            yhi = std::max(ya, yb);
        }
        for (std::size_t r = row(ylo - margin); r <= row(yhi + margin); ++r) {
            for (const std::uint32_t id : cells_[r * side + c]) {
                if (stamp_[id] != stampGen_) {
                    stamp_[id] = stampGen_;
                    out.push_back(id);
                }
            }
        }
    }
}

bool KeepOutField::leavesVertexFree(std::size_t vertex, Vec2 towards) const {
    const Link& l = links_[vertex];
    if (!l.prune) {
        return true;
    }
    const Vec2 u = vertices_[vertex];
    const Vec2 d = towards - u;
    const double dl = std::sqrt(d.dot(d));
    if (!(dl > 0.0)) {
        return true;
    }
    // Interior wedge: from the next edge direction `n` to the previous edge direction `p`,
    // counter-clockwise when the region is on the left of the ring, clockwise otherwise.
    // Cross products are of unit vectors, so kTol is an angle tolerance in radians.
    const double sgn = l.interiorLeft ? 1.0 : -1.0;
    const Vec2 dir{d.x / dl, d.y / dl};
    const double cn = sgn * l.unitNext.cross(dir);  // > 0: dir is counter-clockwise of next
    const double cp = sgn * dir.cross(l.unitPrev);  // > 0: dir is clockwise of prev
    constexpr double kTol = 1e-9;
    const double wedgeCross = sgn * l.unitNext.cross(l.unitPrev);
    if (wedgeCross > kTol) {  // convex corner, wedge < pi
        return !(cn > kTol && cp > kTol);
    }
    if (wedgeCross < -kTol) {  // reflex corner, wedge > pi: free only in the closed complement
        return cn <= kTol && cp <= kTol;
    }
    // Straight (wedge = pi) or a spike (wedge = 0): straight is free on the clockwise side only.
    if (l.unitNext.dot(l.unitPrev) < 0.0) {
        return !(cn > kTol);
    }
    return true;
}

bool KeepOutField::contains(Vec2 p) const {
    if (edges_.empty() || !bounds_.contains(p)) {
        return false;
    }
    bool inside = false;
    std::vector<std::uint32_t> ids;
    // Edges that can be within the boundary tolerance of p or cross the +x ray from p.
    gather(Vec2{p.x - kBoundaryEps, p.y}, Vec2{bounds_.max.x + kBoundaryEps, p.y}, kBoundaryEps, ids);
    if (scalable_) {
        work_ += ids.size();  // the tangent search charges its point tests too
    }
    for (const std::uint32_t id : ids) {
        const Edge& e = edges_[id];
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
    std::vector<std::uint32_t> ids;
    gather(a, b, 0.0, ids);
    work_ += scalable_ ? ids.size() + 1 : edges_.size();
    const Vec2 d = b - a;
    std::vector<double> ts{0.0, 1.0};
    for (const std::uint32_t id : ids) {
        const Edge& e = edges_[id];
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
    if (!scalable_) {
        out = routeFull(a, b);
        if (out.status != RouteStatus::TooComplex || geometry::cancelRequested()) {
            return out;
        }
        scalable_ = true;
    }
    return routeScalable(a, b);
}

RouteResult KeepOutField::routeScalable(Vec2 a, Vec2 b) {
    RouteResult out;
    const std::size_t n = vertices_.size();
    const std::size_t src = n;
    const std::size_t dst = n + 1;
    const auto pos = [&](std::size_t i) { return i == src ? a : i == dst ? b : vertices_[i]; };
    constexpr double kInf = std::numeric_limits<double>::infinity();
    std::vector<double> dist(n + 2, kInf);
    std::vector<std::size_t> prev(n + 2, n + 2);
    std::vector<char> done(n + 2, 0);
    using Item = std::pair<double, std::size_t>;
    std::priority_queue<Item, std::vector<Item>, std::greater<>> open;
    dist[src] = 0.0;
    open.emplace(geometry::distance(a, b), src);
    const std::uint64_t workStart = work_;
    std::uint32_t steps = 0;
    while (!open.empty()) {
        const std::size_t u = open.top().second;
        open.pop();
        if (done[u]) {
            continue;
        }
        done[u] = 1;
        if (u == dst) {
            break;
        }
        const Vec2 pu = pos(u);
        // Scanning the candidate neighbours costs about a quarter of an edge scan per vertex.
        work_ += n / 4 + 1;
        if (work_ - workStart > workLimit_) {
            out.status = RouteStatus::TooComplex;
            return out;
        }
        for (std::size_t v = 0; v < n + 2; ++v) {
            if (done[v] || v == u || (u == src && v == dst)) {
                continue;
            }
            const Vec2 pv = pos(v);
            const double nd = dist[u] + geometry::distance(pu, pv);
            if (nd >= dist[v]) {
                continue;
            }
            // Tangent pruning: the shortest path only joins vertices through free space at both ends.
            if (u < n && !leavesVertexFree(u, pv)) {
                continue;
            }
            if (v < n && !leavesVertexFree(v, pu)) {
                continue;
            }
            if (work_ - workStart > workLimit_ || ((++steps & 63U) == 0 && geometry::cancelRequested())) {
                out.status = RouteStatus::TooComplex;
                return out;
            }
            const bool ok = (u < n && v < n) ? vertexVisible(u, v) : segmentClear(pu, pv);
            if (ok) {
                dist[v] = nd;
                prev[v] = u;
                open.emplace(nd + geometry::distance(pv, b), v);
            }
        }
        if ((++steps & 63U) == 0 && geometry::cancelRequested()) {
            out.status = RouteStatus::TooComplex;
            return out;
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

RouteResult KeepOutField::routeFull(Vec2 a, Vec2 b) {
    RouteResult out;
    // Nodes: 0..n-1 region vertices, n = a, n+1 = b. A* (straight-line heuristic) with edges
    // tested lazily. The cost is bounded: every visibility test adds its edge scans to `work_`
    // and the search gives up (TooComplex) past `workLimit_` or when a cancel is requested.
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
    const std::uint64_t workStart = work_;
    std::uint32_t tests = 0;
    for (;;) {
        std::size_t u = n + 2;
        double best = kInf;
        for (std::size_t i = 0; i < n + 2; ++i) {
            if (!done[i] && dist[i] < kInf) {
                const double f = dist[i] + geometry::distance(pos(i), b);
                if (f < best) {
                    best = f;
                    u = i;
                }
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
            if (nd < dist[v]) {
                if (work_ - workStart > workLimit_ || ((++tests & 63U) == 0 && geometry::cancelRequested())) {
                    out.status = RouteStatus::TooComplex;
                    return out;
                }
                if (visible(u, v)) {
                    dist[v] = nd;
                    prev[v] = u;
                }
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
            if (r.status == RouteStatus::TooComplex) {
                ++report.tooComplex;
            } else if (r.status != RouteStatus::Clear) {
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
