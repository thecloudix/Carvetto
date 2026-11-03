#include "delaunay.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

#include "robust_predicates.hpp"

namespace carvetto::geometry::detail {

namespace {

constexpr double kSqrt3 = 1.7320508075688772;

std::size_t idx(int i) {
    return static_cast<std::size_t>(i);
}

}  // namespace

Delaunay::Delaunay(const Box2& bounds) {
    const Vec2 c = bounds.centre();
    double m = std::max(bounds.width(), bounds.height());
    if (!(m > 0.0)) {
        m = 1.0;
    }
    // The super triangle's incircle has radius k, far larger than the bounds' circumradius, so
    // every input point lies well inside it and never on its edges.
    const double k = 64.0 * m;
    points_ = {c + Vec2{0.0, 2.0 * k}, c + Vec2{-kSqrt3 * k, -k}, c + Vec2{kSqrt3 * k, -k}};
    incident_ = {0, 0, 0};
    tris_.push_back(Triangle{{0, 1, 2}, {-1, -1, -1}});
    alive_.push_back(1);
    mark_.push_back(0);
    created_.push_back(0);
}

int Delaunay::newTriangle() {
    if (!free_.empty()) {
        const int t = free_.back();
        free_.pop_back();
        alive_[idx(t)] = 1;
        return t;
    }
    tris_.emplace_back();
    alive_.push_back(1);
    mark_.push_back(0);
    return static_cast<int>(tris_.size() - 1);
}

int Delaunay::indexOf(int t, int v) const {
    const Triangle& tr = tris_[idx(t)];
    for (int i = 0; i < 3; ++i) {
        if (tr.v[idx(i)] == v) {
            return i;
        }
    }
    return -1;
}

int Delaunay::locate(Vec2 p, int startTri) const {
    int t = startTri;
    int prev = -1;
    const std::size_t limit = tris_.size() + 16;
    for (std::size_t step = 0; step < limit; ++step) {
        const Triangle& tr = tris_[idx(t)];
        bool moved = false;
        for (int kk = 0; kk < 3; ++kk) {
            // Rotating the first edge tried keeps the walk from cycling in degenerate layouts.
            const int k = static_cast<int>((static_cast<std::size_t>(kk) + step) % 3);
            const int nb = tr.n[idx(k)];
            if (nb == prev && nb != -1) {
                continue;
            }
            const Vec2 a = points_[idx(tr.v[idx((k + 1) % 3)])];
            const Vec2 b = points_[idx(tr.v[idx((k + 2) % 3)])];
            if (orient2dSign(a, b, p) < 0) {
                if (nb == -1) {
                    return t;  // outside the super triangle (cannot happen for valid input)
                }
                prev = t;
                t = nb;
                moved = true;
                break;
            }
        }
        if (!moved) {
            return t;
        }
    }
    // Fallback (never expected): scan every live triangle.
    for (std::size_t s = 0; s < tris_.size(); ++s) {
        if (alive_[s] == 0) {
            continue;
        }
        const Triangle& tr = tris_[s];
        if (orient2dSign(points_[idx(tr.v[0])], points_[idx(tr.v[1])], p) >= 0 &&
            orient2dSign(points_[idx(tr.v[1])], points_[idx(tr.v[2])], p) >= 0 &&
            orient2dSign(points_[idx(tr.v[2])], points_[idx(tr.v[0])], p) >= 0) {
            return static_cast<int>(s);
        }
    }
    return t;
}

int Delaunay::insert(Vec2 p, int hintVertex) {
    int start = lastTri_;
    if (hintVertex >= 0 && idx(hintVertex) < incident_.size()) {
        start = incident_[idx(hintVertex)];
    }
    if (start < 0 || idx(start) >= tris_.size() || alive_[idx(start)] == 0) {
        start = lastTri_;
    }
    const int t0 = locate(p, start);
    for (int v : tris_[idx(t0)].v) {
        if (points_[idx(v)] == p) {
            ring_.clear();
            newTris_.clear();
            removed_.clear();
            return v;
        }
    }

    // Grow the cavity: triangles whose open circumdisk contains p.
    ++epoch_;
    if (epoch_ == 0) {  // wrapped: reset marks
        std::fill(mark_.begin(), mark_.end(), 0u);
        epoch_ = 2;
    }
    const unsigned inCavity = epoch_;
    ++epoch_;
    const unsigned rejected = epoch_;
    std::vector<int>& cavity = cavity_;
    std::vector<BoundaryEdge>& boundary = boundary_;
    cavity.assign(1, t0);
    boundary.clear();
    removed_.clear();
    mark_[idx(t0)] = inCavity;
    for (std::size_t s = 0; s < cavity.size(); ++s) {
        const int c = cavity[s];
        for (int k = 0; k < 3; ++k) {
            const Triangle& tr = tris_[idx(c)];
            const int nb = tr.n[idx(k)];
            const int a = tr.v[idx((k + 1) % 3)];
            const int b = tr.v[idx((k + 2) % 3)];
            if (nb == -1) {
                boundary.push_back({a, b, -1, c});
                continue;
            }
            if (mark_[idx(nb)] == inCavity) {
                if (nb > c) {
                    removed_.emplace_back(a, b);  // shared by two cavity triangles
                }
                continue;
            }
            if (mark_[idx(nb)] != rejected) {
                const Triangle& nt = tris_[idx(nb)];
                if (incircleSign(points_[idx(nt.v[0])], points_[idx(nt.v[1])], points_[idx(nt.v[2])], p) > 0) {
                    mark_[idx(nb)] = inCavity;
                    cavity.push_back(nb);
                    removed_.emplace_back(a, b);
                    continue;
                }
                mark_[idx(nb)] = rejected;
            }
            boundary.push_back({a, b, nb, c});
        }
    }

    const int pv = static_cast<int>(points_.size());
    points_.push_back(p);
    incident_.push_back(-1);

    // Fan around p: one triangle (a, b, p) per boundary edge, reusing the cavity's slots.
    std::vector<std::pair<int, int>>& byStart = byStart_;  // (a, new triangle)
    byStart.clear();
    std::size_t reuse = 0;
    ring_.clear();
    newTris_.clear();
    for (const BoundaryEdge& e : boundary) {
        int t = -1;
        if (reuse < cavity.size()) {
            t = cavity[reuse++];
        } else {
            t = newTriangle();
        }
        tris_[idx(t)] = Triangle{{e.a, e.b, pv}, {-1, -1, e.outside}};
        if (e.outside != -1) {
            Triangle& out = tris_[idx(e.outside)];
            for (int j = 0; j < 3; ++j) {
                if (out.n[idx(j)] == e.old && out.v[idx((j + 1) % 3)] == e.b) {
                    out.n[idx(j)] = t;
                    break;
                }
            }
        }
        byStart.emplace_back(e.a, t);
        ring_.push_back(e.a);
        newTris_.push_back(t);
        created_.push_back(t);
    }
    for (std::size_t s = reuse; s < cavity.size(); ++s) {
        alive_[idx(cavity[s])] = 0;
        free_.push_back(cavity[s]);
    }
    // The cavity boundary is a simple cycle: every vertex on it starts one boundary edge and ends
    // one. Index the new triangles by both (scratch arrays over vertex ids, O(1) lookups even for
    // cavities of thousands of triangles).
    if (startOf_.size() < points_.size()) {
        startOf_.resize(points_.size(), -1);
        endOf_.resize(points_.size(), -1);
    }
    for (const auto& [a, t] : byStart) {
        startOf_[idx(a)] = t;
        endOf_[idx(tris_[idx(t)].v[1])] = t;
    }
    for (const auto& [a, t] : byStart) {
        Triangle& tr = tris_[idx(t)];
        const int b = tr.v[1];
        tr.n[0] = startOf_[idx(b)];  // edge b -> p is shared with the triangle starting at b
        tr.n[1] = endOf_[idx(a)];    // edge p -> a with the triangle whose boundary edge ends at a
        incident_[idx(a)] = t;
        incident_[idx(b)] = t;
    }
    incident_[idx(pv)] = byStart.empty() ? -1 : byStart.front().second;
    lastTri_ = incident_[idx(pv)];
    return pv;
}

int Delaunay::triangleWithEdge(int a, int b) const {
    // Rotate around a and around b in lockstep, so the cost is that of the endpoint with the
    // smaller degree (a fan centre can have thousands of triangles).
    const int a0 = incident_[idx(a)];
    const int b0 = incident_[idx(b)];
    int ta = a0;
    int tb = b0;
    for (int guard = 0; guard < 1000000 && (ta >= 0 || tb >= 0); ++guard) {
        if (ta >= 0) {
            const int i = indexOf(ta, a);
            if (i < 0) {
                ta = -1;
            } else {
                const Triangle& tr = tris_[idx(ta)];
                if (tr.v[idx((i + 1) % 3)] == b) {
                    return ta;  // a -> b
                }
                ta = tr.n[idx((i + 1) % 3)];
                if (ta == a0) {
                    return -1;  // full turn around a: no edge a-b
                }
            }
        }
        if (tb >= 0) {
            const int j = indexOf(tb, b);
            if (j < 0) {
                tb = -1;
            } else {
                const Triangle& tr = tris_[idx(tb)];
                if (tr.v[idx((j + 2) % 3)] == a) {
                    return tb;  // a precedes b: a -> b
                }
                tb = tr.n[idx((j + 1) % 3)];
                if (tb == b0) {
                    return -1;  // full turn around b
                }
            }
        }
    }
    return -1;
}

std::vector<int> Delaunay::takeCreated() {
    std::vector<int> out;
    out.swap(created_);
    return out;
}

}  // namespace carvetto::geometry::detail
