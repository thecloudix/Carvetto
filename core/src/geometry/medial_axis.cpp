#include "carvetto/geometry/medial_axis.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "delaunay.hpp"

// Medial axis by a refined Delaunay triangulation of boundary samples (ADR-0023).
//
// 1. Contours are cut into pieces (lines; arcs of at most 45 degrees; Bezier sub-curves), each
//    knowing which side is inside. A search tree over the pieces (nearly straight chunks with
//    capsule bounds under a spatial tree with 8-direction bounds) answers exact nearest-boundary
//    queries (closest point on the true line/arc/Bezier).
// 2. Samples: every piece end, plus curve points at the chordal tolerance. They are inserted
//    into a Delaunay triangulation (exact predicates) in a biased randomised Hilbert order.
// 3. Gabriel refinement: a boundary sub-edge (two consecutive samples) whose diametral circle
//    holds another sample is split at its middle, until none is encroached (not below a length
//    of tol.chordal, and ignoring samples within 2 tol.chordal of the sub-edge: such gaps hold
//    no inscribed circle above the tolerance). Then no Voronoi edge of an interior part crosses
//    the boundary.
// 4. Accuracy refinement: for each triangle whose circumcentre is inside the region, the true
//    distance r of the circumcentre is compared with the circumradius R (the empty sample
//    circle); where R - r > tol.chordal / 2 (also at points along the dual Voronoi edges), the
//    boundary sub-edge holding the nearest point is split there. Repeat 3-4 until stable or a
//    sample/work budget is reached.
// 5. Output: the Voronoi edges dual to Delaunay edges between two interior triangles, except
//    "spokes" (generators that are consecutive samples or whose chord runs within tol.chordal/2
//    of the boundary), form the skeleton. Near-coincident circumcentres (co-circular samples)
//    are merged, degree-2 chains become branches, and branches are subdivided until the radius
//    interpolates within tolerance.

namespace carvetto::geometry {

namespace {

using detail::Delaunay;

std::size_t idx(int i) {
    return static_cast<std::size_t>(i);
}

// ------------------------------------------------------------------------- bounding-volume tree

/// Static bounding-volume tree over boxes (median split on the longer axis) for point stabbing.
class BoxTree {
public:
    explicit BoxTree(const std::vector<Box2>& boxes) : boxes_(boxes) {
        order_.resize(boxes.size());
        std::iota(order_.begin(), order_.end(), 0);
        if (!boxes.empty()) {
            build(0, boxes.size());
        }
    }

    /// Calls `f(item)` for every item whose box contains `p`.
    template <class F>
    void stab(Vec2 p, F&& f) const {
        if (nodes_.empty()) {
            return;
        }
        std::vector<int> stack{0};
        while (!stack.empty()) {
            const Node& n = nodes_[idx(stack.back())];
            stack.pop_back();
            if (!n.box.contains(p)) {
                continue;
            }
            if (n.left < 0) {
                for (std::size_t k = n.begin; k < n.end; ++k) {
                    f(order_[k]);
                }
            } else {
                stack.push_back(n.left);
                stack.push_back(n.right);
            }
        }
    }

private:
    struct Node {
        Box2 box;
        int left{-1};
        int right{-1};
        std::size_t begin{0};
        std::size_t end{0};
    };

    int build(std::size_t begin, std::size_t end) {
        const int id = static_cast<int>(nodes_.size());
        nodes_.emplace_back();
        Box2 box;
        for (std::size_t k = begin; k < end; ++k) {
            box.expand(boxes_[order_[k]]);
        }
        nodes_[idx(id)].box = box;
        nodes_[idx(id)].begin = begin;
        nodes_[idx(id)].end = end;
        if (end - begin <= 4) {
            return id;
        }
        const bool alongX = box.width() >= box.height();
        auto key = [&](std::size_t item) {
            const Vec2 c = boxes_[item].centre();
            return alongX ? c.x : c.y;
        };
        const std::size_t mid = begin + (end - begin) / 2;
        std::nth_element(order_.begin() + static_cast<std::ptrdiff_t>(begin),
                         order_.begin() + static_cast<std::ptrdiff_t>(mid),
                         order_.begin() + static_cast<std::ptrdiff_t>(end), [&](std::size_t a, std::size_t b) {
                             const double ka = key(a);
                             const double kb = key(b);
                             return ka < kb || (ka == kb && a < b);
                         });
        const int l = build(begin, mid);
        const int r = build(mid, end);
        nodes_[idx(id)].left = l;
        nodes_[idx(id)].right = r;
        return id;
    }

    const std::vector<Box2>& boxes_;
    std::vector<std::size_t> order_;
    std::vector<Node> nodes_;
};

// ------------------------------------------------------------------------------------- pieces

struct Piece {
    Span span;
    std::size_t contour{0};
    std::size_t spanIndex{0};
    double t0{0.0};
    double t1{1.0};
    bool leftInside{true};
    int prev{-1};
    int next{-1};
    /// Beziers: largest distance of a control point from the chord (0 for lines and arcs).
    double hullSlack{0.0};
    /// Piece parameter -> sample id; the last entry (u = 1) is the next piece's first sample.
    /// (A map: a long piece can receive hundreds of thousands of samples one at a time.)
    std::map<double, int> samples;
};

struct Sample {
    int vertex{-1};
    int piece{-1};
    double u{0.0};
    int next{-1};
    int prev{-1};
    int nextAtVertex{-1};
    bool frozen{false};
    /// Refinement round in which the sub-edge starting here was last split or created.
    unsigned splitRound{0};
};

struct NearPiece {
    int piece{-1};
    double u{0.0};
    Vec2 point;
};

struct Nearest {
    double dist{std::numeric_limits<double>::infinity()};
    int piece{-1};
    double u{0.0};
    Vec2 point;
    /// Other pieces at the same distance (coincident walls), up to three.
    std::array<NearPiece, 3> ties{};
    int tieCount{0};
};

double segmentDistance(Vec2 a, Vec2 b, Vec2 p) {
    const Vec2 d = b - a;
    const double len2 = d.lengthSquared();
    double t = len2 > 0.0 ? (p - a).dot(d) / len2 : 0.0;
    t = std::clamp(t, 0.0, 1.0);
    const Vec2 q = a + d * t;
    const Vec2 w = p - q;
    return std::sqrt(w.lengthSquared());
}

/// Nearest-piece search tree. Each contour is cut into nearly straight chunks of consecutive
/// pieces; the pieces of a chunk form a balanced binary tree in boundary order, so every node
/// covers a contiguous chain, bounded by a box and by a "capsule": the chord from the chain's
/// start to its end and the largest distance of the chain from that chord. A spatial median tree
/// over the chunks sits on top. A node's lower bound is the larger of
/// its box distance and its chord distance less the slack, which stays tight along densely
/// subdivided smooth curves, where boxes alone leave every nearly equidistant piece unpruned.
class PieceTree {
public:
    PieceTree(const std::vector<Piece>& pieces, const std::vector<std::pair<int, int>>& contourRanges)
        : pieces_(pieces) {
        // Chunks: runs of up to 32 consecutive pieces of a contour that stay nearly straight
        // (capsule slack at most a quarter of the chord). Smooth curves give long chunks with
        // tight capsules; zig-zags and spikes give single pieces, which the spatial tree on top
        // then separates.
        std::vector<int> roots;
        for (const auto& [first, last] : contourRanges) {
            int begin = first;
            while (begin <= last) {
                int end = begin;
                while (end < last && end - begin < 31 && nearlyStraight(begin, end + 1)) {
                    ++end;
                }
                roots.push_back(buildChain(begin, end));
                begin = end + 1;
            }
        }
        if (!roots.empty()) {
            root_ = buildTop(roots, 0, roots.size());
        }
    }

    /// Branch and bound: `f(piece, bestSoFar)` returns the piece's exact distance (or any value
    /// above `bestSoFar` when it can tell the piece is farther); nodes are visited near-first and
    /// skipped when their lower bound exceeds the best found. Returns the piece with the smallest
    /// (distance, index), or -1.
    template <class F>
    int nearest(Vec2 p, F&& f, double& bestDist, double slack, std::uint64_t& work) const {
        bestDist = std::numeric_limits<double>::infinity();
        int best = -1;
        if (root_ < 0) {
            return best;
        }
        struct Entry {
            int node;
            double d;
        };
        // Depth-first with a fixed stack: both tree levels are balanced (depth < 64 for any
        // realistic input) and each level adds at most one pending entry.
        std::array<Entry, 160> stack{};
        std::size_t top = 0;
        stack[top++] = {root_, lowerBound(nodes_[idx(root_)], p)};
        while (top > 0) {
            ++work;
            const Entry e = stack[--top];
            if (e.d > bestDist + slack) {
                continue;
            }
            const Node& n = nodes_[idx(e.node)];
            if (n.piece >= 0) {
                const double d = f(n.piece, bestDist);
                if (d < bestDist || (d == bestDist && n.piece < best)) {
                    bestDist = d;
                    best = n.piece;
                }
                continue;
            }
            const double dl = lowerBound(nodes_[idx(n.left)], p);
            const double dr = lowerBound(nodes_[idx(n.right)], p);
            if (top + 2 > stack.size()) {
                continue;  // unreachable for balanced trees; never overflow
            }
            const double limit = bestDist + slack;
            if (dl <= dr) {
                if (dr <= limit) {
                    stack[top++] = {n.right, dr};
                }
                if (dl <= limit) {
                    stack[top++] = {n.left, dl};
                }
            } else {
                if (dl <= limit) {
                    stack[top++] = {n.left, dl};
                }
                if (dr <= limit) {
                    stack[top++] = {n.right, dr};
                }
            }
        }
        return best;
    }

private:
    /// Bounds along x, y and the two diagonals (an 8-sided "k-DOP"): far tighter than a box for
    /// long diagonal pieces, which star-shaped and spiky outlines are full of.
    struct Dop {
        std::array<double, 4> lo{kInf, kInf, kInf, kInf};
        std::array<double, 4> hi{-kInf, -kInf, -kInf, -kInf};
        static constexpr double kInf = std::numeric_limits<double>::infinity();
        static constexpr double kDiag = 0.70710678118654752;
        static std::array<double, 4> project(Vec2 p) {
            return {p.x, p.y, kDiag * (p.x + p.y), kDiag * (p.x - p.y)};
        }
        void expand(Vec2 p) {
            const auto v = project(p);
            for (std::size_t k = 0; k < 4; ++k) {
                lo[k] = std::min(lo[k], v[k]);
                hi[k] = std::max(hi[k], v[k]);
            }
        }
        void expand(const Dop& o) {
            for (std::size_t k = 0; k < 4; ++k) {
                lo[k] = std::min(lo[k], o.lo[k]);
                hi[k] = std::max(hi[k], o.hi[k]);
            }
        }
        void inflate(double r) {
            for (std::size_t k = 0; k < 4; ++k) {
                lo[k] -= r;
                hi[k] += r;
            }
        }
        double distance(Vec2 p) const {
            const auto v = project(p);
            const double dx = std::max({lo[0] - v[0], 0.0, v[0] - hi[0]});
            const double dy = std::max({lo[1] - v[1], 0.0, v[1] - hi[1]});
            const double d2 = std::max({lo[2] - v[2], lo[3] - v[3], v[2] - hi[2], v[3] - hi[3], 0.0});
            return std::max(std::sqrt(dx * dx + dy * dy), d2);
        }
        Vec2 centre() const { return {0.5 * (lo[0] + hi[0]), 0.5 * (lo[1] + hi[1])}; }
        double width() const { return hi[0] - lo[0]; }
        double height() const { return hi[1] - lo[1]; }
    };

    struct Node {
        Dop box;
        Vec2 a;
        Vec2 b;
        double slack{0.0};
        bool chain{false};
        int left{-1};
        int right{-1};
        int piece{-1};
    };

    /// True when pieces first..last lie within a quarter of their chord's length from it.
    bool nearlyStraight(int first, int last) const {
        const Vec2 a = startPoint(pieces_[idx(first)].span);
        const Vec2 b = endPoint(pieces_[idx(last)].span);
        const double limit = 0.25 * distance(a, b);
        for (int k = first; k <= last; ++k) {
            const Piece& pc = pieces_[idx(k)];
            double slack = pc.hullSlack;
            if (const auto* arc = std::get_if<ArcSpan>(&pc.span)) {
                slack = arc->radius * (1.0 - std::cos(0.5 * std::fabs(arc->sweepAngle())));
            }
            if (segmentDistance(a, b, endPoint(pc.span)) + slack > limit) {
                return false;
            }
        }
        return true;
    }

    static double lowerBound(const Node& n, Vec2 p) {
        const double bd = n.box.distance(p);
        if (!n.chain) {
            return bd;
        }
        return std::max(bd, segmentDistance(n.a, n.b, p) - n.slack);
    }

    int buildChain(int first, int last) {
        Node n;
        n.chain = true;
        if (first == last) {
            const Piece& pc = pieces_[idx(first)];
            n.piece = first;
            n.a = startPoint(pc.span);
            n.b = endPoint(pc.span);
            n.box.expand(n.a);
            n.box.expand(n.b);
            if (const auto* arc = std::get_if<ArcSpan>(&pc.span)) {
                // Sagitta: a piece sweeps at most 45 degrees, so it projects inside its chord
                // and lies within the sagitta of it.
                n.slack = arc->radius * (1.0 - std::cos(0.5 * std::fabs(arc->sweepAngle())));
                n.box.inflate(n.slack * (1.0 + 1e-9));
            } else {
                n.slack = pc.hullSlack;
                if (const auto* bz = std::get_if<BezierSpan>(&pc.span)) {
                    n.box.expand(bz->p1);  // the control hull holds the curve
                    n.box.expand(bz->p2);
                }
            }
            n.slack *= 1.0 + 1e-9;
            nodes_.push_back(n);
            return static_cast<int>(nodes_.size() - 1);
        }
        const int mid = first + (last - first) / 2;
        const int l = buildChain(first, mid);
        const int r = buildChain(mid + 1, last);
        const Node& ln = nodes_[idx(l)];
        const Node& rn = nodes_[idx(r)];
        n.left = l;
        n.right = r;
        n.box = ln.box;
        n.box.expand(rn.box);
        n.a = ln.a;
        n.b = rn.b;
        double slack = 0.0;
        for (const Node* c : {&ln, &rn}) {
            slack = std::max(slack, std::max(segmentDistance(n.a, n.b, c->a), segmentDistance(n.a, n.b, c->b)) + c->slack);
        }
        n.slack = slack * (1.0 + 1e-9);
        nodes_.push_back(n);
        return static_cast<int>(nodes_.size() - 1);
    }

    int buildTop(std::vector<int>& roots, std::size_t begin, std::size_t end) {
        if (end - begin == 1) {
            return roots[begin];
        }
        Dop box;
        for (std::size_t k = begin; k < end; ++k) {
            box.expand(nodes_[idx(roots[k])].box);
        }
        const bool alongX = box.width() >= box.height();
        auto key = [&](int r) {
            const Vec2 c = nodes_[idx(r)].box.centre();
            return alongX ? c.x : c.y;
        };
        const std::size_t mid = begin + (end - begin) / 2;
        std::nth_element(roots.begin() + static_cast<std::ptrdiff_t>(begin), roots.begin() + static_cast<std::ptrdiff_t>(mid),
                         roots.begin() + static_cast<std::ptrdiff_t>(end), [&](int x, int y) {
                             const double kx = key(x);
                             const double ky = key(y);
                             return kx < ky || (kx == ky && x < y);
                         });
        const int l = buildTop(roots, begin, mid);
        const int r = buildTop(roots, mid, end);
        Node n;
        n.left = l;
        n.right = r;
        n.box = box;
        nodes_.push_back(n);
        return static_cast<int>(nodes_.size() - 1);
    }

    const std::vector<Piece>& pieces_;
    std::vector<Node> nodes_;
    int root_{-1};
};

// ------------------------------------------------------------- closest point on a Bezier piece

using Quintic = std::array<double, 6>;

int signChanges(const Quintic& c) {
    int changes = 0;
    int prev = 0;
    for (double v : c) {
        const int sg = v > 0.0 ? 1 : (v < 0.0 ? -1 : 0);
        if (sg != 0) {
            if (prev != 0 && sg != prev) {
                ++changes;
            }
            prev = sg;
        }
    }
    return changes;
}

std::pair<Quintic, Quintic> splitQuintic(const Quintic& c) {
    Quintic left{};
    Quintic right{};
    Quintic w = c;
    for (std::size_t k = 0; k < 6; ++k) {
        left[k] = w[0];
        right[5 - k] = w[5 - k];
        for (std::size_t i = 0; i + 1 < 6 - k; ++i) {
            w[i] = 0.5 * (w[i] + w[i + 1]);
        }
    }
    return {left, right};
}

/// Closest point on a cubic Bezier piece: as `closestPoint(BezierSpan)` (contour_ops), every root
/// of g(t) = (B(t) - p) . B'(t) is isolated by subdividing its Bernstein coefficients, so the
/// minimum is global; each isolated root is then refined by Newton steps kept inside its bracket
/// (bisection when a step leaves it), a few evaluations instead of a full bisection.
ClosestPoint bezierClosest(const BezierSpan& s, Vec2 p) {
    const std::array<Vec2, 4> rel{s.p0 - p, s.p1 - p, s.p2 - p, s.p3 - p};
    const std::array<Vec2, 3> der{(s.p1 - s.p0) * 3.0, (s.p2 - s.p1) * 3.0, (s.p3 - s.p2) * 3.0};
    const std::array<Vec2, 2> der2{(der[1] - der[0]) * 2.0, (der[2] - der[1]) * 2.0};
    constexpr std::array<double, 4> binom3{1.0, 3.0, 3.0, 1.0};
    constexpr std::array<double, 3> binom2{1.0, 2.0, 1.0};
    constexpr std::array<double, 6> binom5{1.0, 5.0, 10.0, 10.0, 5.0, 1.0};
    Quintic coeffs{};
    for (std::size_t i = 0; i < 4; ++i) {
        for (std::size_t j = 0; j < 3; ++j) {
            coeffs[i + j] += binom3[i] * binom2[j] * rel[i].dot(der[j]);
        }
    }
    for (std::size_t k = 0; k < 6; ++k) {
        coeffs[k] /= binom5[k];
    }
    auto point = [&](double t) {
        const double u = 1.0 - t;
        return rel[0] * (u * u * u) + rel[1] * (3.0 * u * u * t) + rel[2] * (3.0 * u * t * t) + rel[3] * (t * t * t);
    };
    auto deriv = [&](double t) {
        const double u = 1.0 - t;
        return der[0] * (u * u) + der[1] * (2.0 * u * t) + der[2] * (t * t);
    };
    auto deriv2 = [&](double t) { return der2[0] * (1.0 - t) + der2[1] * t; };
    auto g = [&](double t) { return point(t).dot(deriv(t)); };

    std::array<double, 128> cand{};
    std::size_t nc = 0;
    auto add = [&](double t) {
        if (nc < cand.size()) {
            cand[nc++] = t;
        }
    };
    add(0.0);
    add(1.0);
    struct Item {
        Quintic c;
        double t0;
        double t1;
        int depth;
    };
    std::array<Item, 64> stack{};
    std::size_t top = 0;
    stack[top++] = {coeffs, 0.0, 1.0, 0};
    while (top > 0) {
        const Item it = stack[--top];
        const int changes = signChanges(it.c);
        if (changes == 0) {
            continue;
        }
        if (changes == 1) {
            double lo = it.t0;
            double hi = it.t1;
            double glo = g(lo);
            const double ghi = g(hi);
            if ((glo < 0.0 && ghi > 0.0) || (glo > 0.0 && ghi < 0.0)) {
                double t = 0.5 * (lo + hi);
                for (int iter = 0; iter < 100 && hi - lo > 1e-15; ++iter) {
                    const double gt = g(t);
                    if (gt == 0.0) {
                        lo = hi = t;
                        break;
                    }
                    if ((gt < 0.0) == (glo < 0.0)) {
                        lo = t;
                        glo = gt;
                    } else {
                        hi = t;
                    }
                    const double dg = deriv(t).lengthSquared() + point(t).dot(deriv2(t));
                    double next = dg != 0.0 ? t - gt / dg : 0.5 * (lo + hi);
                    if (!(next > lo && next < hi)) {
                        next = 0.5 * (lo + hi);
                    }
                    if (std::fabs(next - t) < 1e-15) {
                        t = next;
                        break;
                    }
                    t = next;
                }
                add(std::clamp(t, it.t0, it.t1));
                continue;
            }
        }
        if (it.depth >= 50 || it.t1 - it.t0 < 1e-15 || top + 2 > stack.size()) {
            add(0.5 * (it.t0 + it.t1));
            continue;
        }
        const auto [left, right] = splitQuintic(it.c);
        const double tm = 0.5 * (it.t0 + it.t1);
        add(tm);  // a root exactly on the split point may be claimed by neither half
        stack[top++] = {right, tm, it.t1, it.depth + 1};
        stack[top++] = {left, it.t0, tm, it.depth + 1};
    }
    ClosestPoint best{s.p0, 0.0, std::numeric_limits<double>::infinity()};
    for (std::size_t k = 0; k < nc; ++k) {
        const double t = cand[k];
        const Vec2 q = s.pointAt(t);
        const double d = distance(q, p);
        if (d < best.distance || (d == best.distance && t < best.t)) {
            best = {q, t, d};
        }
    }
    return best;
}

/// Sub-span of `s` over its parameter range [a, b].
Span subSpan(const Span& s, double a, double b) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return LineSpan{l->pointAt(a), l->pointAt(b)};
    }
    if (const auto* arc = std::get_if<ArcSpan>(&s)) {
        ArcSpan r = *arc;
        r.startAngle = arc->angleAt(a);
        r.endAngle = arc->angleAt(b);
        return r;
    }
    const auto& bz = std::get<BezierSpan>(s);
    BezierSpan right = bz;
    if (a > 0.0) {
        right = bz.split(a).second;
    }
    if (b < 1.0) {
        const double local = (b - a) / (1.0 - a);
        return right.split(local).first;
    }
    return right;
}

bool finitePoint(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

bool finiteSpan(const Span& s) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return finitePoint(l->start) && finitePoint(l->end);
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        return finitePoint(a->centre) && std::isfinite(a->radius) && std::isfinite(a->startAngle) &&
               std::isfinite(a->endAngle);
    }
    const auto& b = std::get<BezierSpan>(s);
    return finitePoint(b.p0) && finitePoint(b.p1) && finitePoint(b.p2) && finitePoint(b.p3);
}

/// True for spans of exactly zero extent (dropped; tiny but non-zero spans are kept).
bool zeroSpan(const Span& s) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return l->start == l->end;
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        return !(a->radius > 0.0);
    }
    const auto& b = std::get<BezierSpan>(s);
    return b.p0 == b.p1 && b.p1 == b.p2 && b.p2 == b.p3;
}

/// Number of pieces for a span: arcs at most 45 degrees each, Beziers by total turning.
int pieceCount(const Span& s) {
    if (std::holds_alternative<LineSpan>(s)) {
        return 1;
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        return std::max(1, static_cast<int>(std::ceil(std::fabs(a->sweepAngle()) / (kPi / 4.0) - 1e-9)));
    }
    const auto& b = std::get<BezierSpan>(s);
    double turn = 0.0;
    Vec2 prevT = b.tangentAt(0.0);
    constexpr int kSteps = 16;
    for (int k = 1; k <= kSteps; ++k) {
        const Vec2 t = b.tangentAt(static_cast<double>(k) / kSteps);
        turn += std::fabs(std::atan2(prevT.cross(t), prevT.dot(t)));
        prevT = t;
    }
    return std::clamp(static_cast<int>(std::ceil(turn / (kPi / 4.0))), 2, 64);
}

/// Interior sample count for a piece so that the chordal deviation is about `chordal`.
int interiorSamples(const Span& s, double chordal) {
    if (std::holds_alternative<LineSpan>(s)) {
        return 0;
    }
    if (const auto* a = std::get_if<ArcSpan>(&s)) {
        // At most 64 per piece: refinement adds samples where they are needed.
        const std::size_t n = a->segmentCountFor(chordal);
        return static_cast<int>(std::min<std::size_t>(n, 64)) - 1;
    }
    const auto& b = std::get<BezierSpan>(s);
    const double flat = b.flatness();
    if (!(flat > chordal)) {
        return 1;
    }
    return std::clamp(static_cast<int>(std::ceil(std::sqrt(flat / chordal))), 1, 63);
}

/// Winding number of the closed polygon `r` around `p` (crossings of the ray towards +x, with
/// the half-open rule on y so vertices count once).
int ringWinding(const std::vector<Vec2>& r, Vec2 p) {
    int w = 0;
    const std::size_t n = r.size();
    for (std::size_t k = 0; k < n; ++k) {
        const Vec2 a = r[k];
        const Vec2 b = r[(k + 1) % n];
        if (a.y <= p.y) {
            if (b.y > p.y && (b - a).cross(p - a) > 0.0) {
                ++w;
            }
        } else if (b.y <= p.y && (b - a).cross(p - a) < 0.0) {
            --w;
        }
    }
    return w;
}

/// Distance from `p` to the closed polygon `r`.
double ringDistance(const std::vector<Vec2>& r, Vec2 p) {
    double best = std::numeric_limits<double>::infinity();
    const std::size_t n = r.size();
    for (std::size_t k = 0; k < n; ++k) {
        best = std::min(best, segmentDistance(r[k], r[(k + 1) % n], p));
    }
    return best;
}

/// Winding numbers of points against many polygons: the edges are bucketed into horizontal rows
/// and sorted by their right end within each row, so a query casts its ray towards +x through
/// one row and stops at the first edge that ends left of it.
class WindingIndex {
public:
    void build(const std::vector<std::vector<Vec2>>& rings, const std::vector<char>& use) {
        edges_.clear();
        Box2 box;
        for (std::size_t i = 0; i < rings.size(); ++i) {
            if (use[i] == 0) {
                continue;
            }
            const auto& r = rings[i];
            for (std::size_t k = 0; k < r.size(); ++k) {
                const Vec2 a = r[k];
                const Vec2 b = r[(k + 1) % r.size()];
                if (a.y != b.y) {  // horizontal edges never count
                    edges_.push_back({a, b});
                    box.expand(a);
                    box.expand(b);
                }
            }
        }
        rowStart_.clear();
        items_.clear();
        if (edges_.empty()) {
            return;
        }
        y0_ = box.min.y;
        const auto rows = static_cast<std::size_t>(std::clamp(std::sqrt(static_cast<double>(edges_.size())), 1.0, 65536.0));
        rows_ = rows;
        rowH_ = std::max(box.height() / static_cast<double>(rows), 1e-300);
        std::vector<std::size_t> count(rows + 1, 0);
        for (const Edge& e : edges_) {
            const auto [lo, hi] = rowRange(e);
            for (std::size_t r = lo; r <= hi; ++r) {
                ++count[r + 1];
            }
        }
        for (std::size_t r = 0; r < rows; ++r) {
            count[r + 1] += count[r];
        }
        rowStart_ = count;
        items_.assign(count[rows], 0);
        std::vector<std::size_t> fill(count.begin(), count.end() - 1);
        for (std::size_t k = 0; k < edges_.size(); ++k) {
            const auto [lo, hi] = rowRange(edges_[k]);
            for (std::size_t r = lo; r <= hi; ++r) {
                items_[fill[r]++] = static_cast<int>(k);
            }
        }
        for (std::size_t r = 0; r < rows; ++r) {
            std::sort(items_.begin() + static_cast<std::ptrdiff_t>(rowStart_[r]),
                      items_.begin() + static_cast<std::ptrdiff_t>(rowStart_[r + 1]), [&](int x, int y) {
                          const double mx = maxX(edges_[idx(x)]);
                          const double my = maxX(edges_[idx(y)]);
                          return mx > my || (mx == my && x < y);
                      });
        }
    }

    int winding(Vec2 p) const {
        if (rowStart_.empty() || !(p.y >= y0_)) {
            return 0;
        }
        const double f = (p.y - y0_) / rowH_;
        if (!(f < static_cast<double>(rows_))) {
            return 0;
        }
        const auto r = static_cast<std::size_t>(f);
        int w = 0;
        for (std::size_t k = rowStart_[r]; k < rowStart_[r + 1]; ++k) {
            const Edge& e = edges_[idx(items_[k])];
            if (maxX(e) < p.x) {
                break;
            }
            if (e.a.y <= p.y) {
                if (e.b.y > p.y && (e.b - e.a).cross(p - e.a) > 0.0) {
                    ++w;
                }
            } else if (e.b.y <= p.y && (e.b - e.a).cross(p - e.a) < 0.0) {
                --w;
            }
        }
        return w;
    }

private:
    struct Edge {
        Vec2 a;
        Vec2 b;
    };
    static double maxX(const Edge& e) { return std::max(e.a.x, e.b.x); }
    std::pair<std::size_t, std::size_t> rowRange(const Edge& e) const {
        auto row = [&](double y) {
            const double f = std::floor((y - y0_) / rowH_);
            return static_cast<std::size_t>(std::clamp(f, 0.0, static_cast<double>(rows_ - 1)));
        };
        return {row(std::min(e.a.y, e.b.y)), row(std::max(e.a.y, e.b.y))};
    }
    std::vector<Edge> edges_;
    std::vector<std::size_t> rowStart_;
    std::vector<int> items_;
    double y0_{0.0};
    double rowH_{1.0};
    std::size_t rows_{1};
};

// --------------------------------------------------------------------------- hilbert ordering

std::uint64_t hilbertKey(std::uint32_t x, std::uint32_t y) {
    std::uint64_t d = 0;
    constexpr std::uint32_t kN = 1u << 16;
    for (std::uint32_t s = kN / 2; s > 0; s /= 2) {
        const std::uint32_t rx = (x & s) != 0 ? 1u : 0u;
        const std::uint32_t ry = (y & s) != 0 ? 1u : 0u;
        d += static_cast<std::uint64_t>(s) * s * ((3u * rx) ^ ry);
        if (ry == 0) {
            if (rx == 1) {
                x = kN - 1 - x;
                y = kN - 1 - y;
            }
            std::swap(x, y);
        }
    }
    return d;
}

std::uint64_t splitMix(std::uint64_t& state) {
    std::uint64_t z = (state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

// ------------------------------------------------------------------------------------- solver

struct TriData {
    Vec2 cc;
    double circumradius{0.0};
    double radius{0.0};
    bool interior{false};
    bool requested{false};  // this triangle asked for a split in the current round
    unsigned stamp{0};
};

struct SplitRequest {
    int piece;
    double u;
    int tri;  // triangle (or one triangle of the Voronoi edge) that asked; re-checked next round
    double spacing;  // sample spacing the local radius needs (see Solver::spacingFor)
};

class Solver {
public:
    Solver(const std::vector<Contour>& contours, const Tolerance& tol, const MedialAxisOptions& options)
        : contours_(contours), tol_(tol), options_(options) {}

    MedialAxis run();

private:
    void classifyContours(std::vector<int>& leftInside);
    void buildPieces(const std::vector<int>& leftInside);
    void insertInitialSamples();
    Vec2 samplePos(int s) const { return dt_->point(samples_[idx(s)].vertex); }
    int addSample(int vertex, int piece, double u);
    Nearest nearest(Vec2 p) const;
    Nearest cachedNearest(Vec2 p);
    bool insideOf(int piece, double u, Vec2 q, Vec2 v) const;
    bool insideAt(const Nearest& nr, Vec2 v) const;
    bool windingInside(Vec2 v) const;
    int windingAt(Vec2 v) const;
    bool inRegion(int w) const { return options_.fillRule == FillRule::NonZero ? w != 0 : (w % 2) != 0; }
    bool isBoundaryEdge(int a, int b) const;
    /// True when the Voronoi edge dual to Delaunay edge a-b (between triangles with data `d1` and
    /// `d2`) is not part of the skeleton because a and b are not two distinct boundary features:
    /// they are consecutive samples, or the chord a-b runs within hmin_ of the boundary (its
    /// middle is that close), as across a gap narrower than the tolerance, between samples of a
    /// gently curved boundary, or at the very tip of a sharp corner.
    bool isSpoke(int a, int b, const TriData& d1, const TriData& d2) const;
    bool encroached(int a, int b, int hint) const;
    ClosestPoint pieceClosest(int piece, Vec2 p) const;
    enum class SplitMode { Midpoint, Near, Exact };
    bool splitSubEdge(int s, double u, SplitMode mode);
    bool splitAt(int piece, double u, double spacing);
    void processGabriel();
    void evaluateCreated(std::vector<SplitRequest>* requests);
    bool checkSegment(Vec2 p1, double r1, Vec2 p2, double r2, Vec2 gen, int depth,
                      std::vector<SplitRequest>* requests, std::vector<MedialPoint>* out) const;
    MedialBoundaryRef refOf(int vertex) const;
    /// Sample spacing h at which a straight wall dips at most emax_ into an empty circle of
    /// radius r (h^2 / 8r <= emax_), with some margin.
    double spacingFor(double r) const { return 0.9 * std::sqrt(8.0 * emax_ * std::max(r, hmin_)); }
    MedialAxis assemble();

    const std::vector<Contour>& contours_;
    Tolerance tol_;
    MedialAxisOptions options_;
    double chordal_{0.01};
    double emax_{0.005};
    double hmin_{0.005};
    double gapTol_{0.02};  // gaps narrower than this (2 x chordal) are not refined
    std::size_t maxSamples_{0};
    /// Work budget: nodes visited by nearest-boundary queries (deterministic, unlike a clock).
    /// Text needs ~200 per sample, spiky adversarial outlines ~2,000.
    std::uint64_t maxWork_{0};
    mutable std::uint64_t work_{0};
    double mergeTol_{1e-9};
    double scale_{1.0};
    Box2 bounds_;
    std::size_t ignored_{0};
    bool capped_{false};
    bool beyondScale_{false};  // coordinates so large that 1e-12 x scale exceeds chordal / 2

    std::vector<Piece> pieces_;
    std::vector<std::pair<int, int>> contourRanges_;  // first and last piece of each contour
    std::vector<Box2> contourBoxes_;
    std::vector<char> usable_;  // closed, finite, non-empty input contours
    std::unique_ptr<BoxTree> contourTree_;
    std::vector<std::vector<Vec2>> rings_;  // usable contours polygonised at the chordal tolerance
    WindingIndex windingIndex_;
    double tieTol_{0.0};
    std::unique_ptr<PieceTree> tree_;
    std::unique_ptr<Delaunay> dt_;
    std::vector<Sample> samples_;
    std::vector<int> vertexSample_;  // first sample at each vertex, -1 if none
    /// Sub-edges to check: (first sample, hint) with hint a live triangle having the sub-edge
    /// (kUnknownEdge: look it up; kRemovedEdge: known not to be a Delaunay edge any more).
    std::vector<std::pair<int, int>> gabrielQueue_;
    static constexpr int kUnknownEdge = -2;
    static constexpr int kRemovedEdge = -1;
    void queueSubEdge(int a, int b, int hint);
    std::vector<TriData> tri_;
    std::vector<int> recheck_;  // triangles to evaluate again in the next round

    struct CellHash {
        std::size_t operator()(const std::pair<long long, long long>& k) const {
            const auto x = static_cast<unsigned long long>(k.first);
            const auto y = static_cast<unsigned long long>(k.second);
            return static_cast<std::size_t>(x * 0x9E3779B97F4A7C15ull ^ (y + 0x632BE59BD9B4E019ull + (x << 6) + (x >> 2)));
        }
    };
    /// Nearest-boundary results of circumcentres by cell of size mergeTol_ (co-circular samples
    /// share a circumcentre; a dense polygonal circle would otherwise query its centre n times).
    std::unordered_map<std::pair<long long, long long>, Nearest, CellHash> nearCache_;
    unsigned round_{0};
};

void Solver::classifyContours(std::vector<int>& leftInside) {
    const std::size_t n = contours_.size();
    leftInside.assign(n, -1);
    std::vector<Box2>& boxes = contourBoxes_;
    std::vector<char>& usable = usable_;
    boxes.assign(n, Box2{});
    usable.assign(n, 0);
    for (std::size_t i = 0; i < n; ++i) {
        const Contour& c = contours_[i];
        if (!c.closed || c.spans.empty()) {
            continue;
        }
        bool ok = true;
        bool nonZero = false;
        for (const Span& s : c.spans) {
            ok = ok && finiteSpan(s);
            nonZero = nonZero || !zeroSpan(s);
        }
        if (!ok || !nonZero) {
            continue;
        }
        boxes[i] = c.bounds().inflated(tol_.distance);
        usable[i] = 1;
    }
    contourTree_ = std::make_unique<BoxTree>(boxes);
    const BoxTree& tree = *contourTree_;
    // Each usable contour is polygonised once at the chordal tolerance for winding numbers
    // (windingNumber() would polygonise Beziers again on every call).
    rings_.assign(n, {});
    for (std::size_t i = 0; i < n; ++i) {
        if (usable[i] != 0) {
            rings_[i] = contours_[i].polygonise(tol_).points;
        }
    }
    windingIndex_.build(rings_, usable);
    for (std::size_t i = 0; i < n; ++i) {
        if (usable[i] == 0) {
            continue;
        }
        const Contour& c = contours_[i];
        // Area relative to the contour's own box centre: the shoelace on absolute coordinates
        // cancels to zero for small contours far from the origin (a 1 x 0.5 mm rectangle at 1e8).
        const Contour local = c.transformed(Transform2::translation(-boxes[i].centre()));
        const double area = signedArea(local);
        if (!(area != 0.0) || !std::isfinite(area)) {
            continue;
        }
        const bool ccw = area > 0.0;
        // Probe a point on the contour that is clear of every other contour: the winding number
        // of the others there, plus this contour's own side, gives the region on each side.
        // T-0153: up to 16 probes spread over the contour (several per span when it has few
        // spans): one probe per span left a one-arc circle near another contour (a hole 0.005 mm
        // from a wall) without a clear probe, and the hole was ignored.
        const std::size_t spans = c.spans.size();
        constexpr std::size_t kProbes = 16;
        const std::size_t perSpan = spans >= kProbes ? 1 : (kProbes + spans - 1) / spans;
        const std::size_t probes = spans >= kProbes ? kProbes : spans * perSpan;
        bool found = false;
        int wl = 0;
        int wr = 0;
        const Span* firstProbe = nullptr;
        for (std::size_t k = 0; k < probes && !found; ++k) {
            const Span& sp = spans >= kProbes ? c.spans[(k * spans) / kProbes] : c.spans[k / perSpan];
            if (zeroSpan(sp)) {
                continue;
            }
            if (firstProbe == nullptr) {
                firstProbe = &sp;
            }
            const double tProbe =
                spans >= kProbes ? 0.5 : (static_cast<double>(k % perSpan) + 0.5) / static_cast<double>(perSpan);
            const Vec2 p = pointAt(sp, tProbe);
            bool clear = true;
            int w = 0;
            tree.stab(p, [&](std::size_t j) {
                if (j == i || usable[j] == 0) {
                    return;
                }
                if (ringDistance(rings_[j], p) <= tol_.distance + chordal_) {
                    clear = false;
                }
                w += ringWinding(rings_[j], p);
            });
            if (clear) {
                found = true;
                wl = w + (ccw ? 1 : 0);
                wr = w + (ccw ? 0 : -1);
            }
        }
        if (!found) {
            if (firstProbe == nullptr) {
                continue;
            }
            // Every probe lies on another contour (duplicated or coincident contours): count all
            // contours, this one included, just off both sides of the first probe.
            const Vec2 p = pointAt(*firstProbe, 0.5);
            const Vec2 nrm = tangentAt(*firstProbe, 0.5).perp();
            const double delta = 2.0 * chordal_ + 1e-9 * (std::fabs(p.x) + std::fabs(p.y));
            wl = windingAt(p + nrm * delta);
            wr = windingAt(p - nrm * delta);
        }
        if (inRegion(wl) == inRegion(wr)) {
            continue;
        }
        leftInside[i] = inRegion(wl) ? 1 : 0;
    }
}

void Solver::buildPieces(const std::vector<int>& leftInside) {
    for (std::size_t i = 0; i < contours_.size(); ++i) {
        if (leftInside[i] < 0) {
            ++ignored_;
            continue;
        }
        const Contour& c = contours_[i];
        const int first = static_cast<int>(pieces_.size());
        for (std::size_t j = 0; j < c.spans.size(); ++j) {
            const Span& s = c.spans[j];
            if (zeroSpan(s)) {
                continue;
            }
            const int m = pieceCount(s);
            for (int k = 0; k < m; ++k) {
                Piece p;
                p.t0 = static_cast<double>(k) / m;
                p.t1 = static_cast<double>(k + 1) / m;
                p.span = m == 1 ? s : subSpan(s, p.t0, p.t1);
                p.contour = i;
                p.spanIndex = j;
                p.leftInside = leftInside[i] == 1;
                pieces_.push_back(std::move(p));
            }
        }
        const int last = static_cast<int>(pieces_.size()) - 1;
        contourRanges_.emplace_back(first, last);
        for (int k = first; k <= last; ++k) {
            pieces_[idx(k)].prev = k == first ? last : k - 1;
            pieces_[idx(k)].next = k == last ? first : k + 1;
        }
    }
    for (Piece& p : pieces_) {
        if (const auto* bz = std::get_if<BezierSpan>(&p.span)) {
            const LineSpan chord{bz->p0, bz->p3};
            p.hullSlack = std::max(closestPoint(chord, bz->p1).distance, closestPoint(chord, bz->p2).distance);
            p.hullSlack *= 1.0 + 1e-12;
        }
        bounds_.expand(bounds(p.span));
    }
    tree_ = std::make_unique<PieceTree>(pieces_, contourRanges_);
}

int Solver::addSample(int vertex, int piece, double u) {
    const int s = static_cast<int>(samples_.size());
    Sample smp;
    smp.vertex = vertex;
    smp.piece = piece;
    smp.u = u;
    samples_.push_back(smp);
    if (vertex >= 0) {
        if (vertexSample_.size() <= idx(vertex)) {
            vertexSample_.resize(idx(vertex) + 1, -1);
        }
        samples_[idx(s)].nextAtVertex = vertexSample_[idx(vertex)];
        vertexSample_[idx(vertex)] = s;
    }
    return s;
}

void Solver::insertInitialSamples() {
    // Positions first (vertex ids assigned on insertion).
    std::vector<Vec2> pos;
    for (std::size_t pi = 0; pi < pieces_.size(); ++pi) {
        Piece& p = pieces_[pi];
        const int n = interiorSamples(p.span, chordal_);
        for (int k = 0; k <= n; ++k) {
            const double u = static_cast<double>(k) / (n + 1);
            const int s = addSample(-1, static_cast<int>(pi), u);
            p.samples.emplace(u, s);
            pos.push_back(k == 0 ? startPoint(p.span) : pointAt(p.span, u));
        }
    }
    for (Piece& p : pieces_) {
        p.samples.emplace(1.0, pieces_[idx(p.next)].samples.begin()->second);
    }
    for (Piece& p : pieces_) {
        for (auto it = p.samples.begin(); std::next(it) != p.samples.end(); ++it) {
            const int a = it->second;
            const int b = std::next(it)->second;
            samples_[idx(a)].next = b;
            samples_[idx(b)].prev = a;
        }
    }

    // Biased randomised insertion order: a deterministic shuffle cut into rounds of doubling
    // size, each round sorted along a Hilbert curve.
    const std::size_t n = pos.size();
    std::vector<std::size_t> order(n);
    std::iota(order.begin(), order.end(), 0);
    std::uint64_t state = 0x5DEECE66Dull;
    for (std::size_t k = n; k > 1; --k) {
        const std::size_t j = static_cast<std::size_t>(splitMix(state) % k);
        std::swap(order[k - 1], order[j]);
    }
    const double w = std::max(bounds_.width(), 1e-300);
    const double h = std::max(bounds_.height(), 1e-300);
    std::vector<std::uint64_t> keys(n);
    for (std::size_t k = 0; k < n; ++k) {
        const Vec2 q = pos[k];
        const double fx = std::clamp((q.x - bounds_.min.x) / w, 0.0, 1.0) * 65535.0;
        const double fy = std::clamp((q.y - bounds_.min.y) / h, 0.0, 1.0) * 65535.0;
        keys[k] = hilbertKey(static_cast<std::uint32_t>(fx), static_cast<std::uint32_t>(fy));
    }
    std::size_t end = n;
    while (end > 0) {
        const std::size_t begin = end <= 64 ? 0 : end / 2;
        std::sort(order.begin() + static_cast<std::ptrdiff_t>(begin), order.begin() + static_cast<std::ptrdiff_t>(end),
                  [&](std::size_t a, std::size_t b) { return keys[a] < keys[b] || (keys[a] == keys[b] && a < b); });
        end = begin;
    }
    int hint = -1;
    for (std::size_t k : order) {
        const int v = dt_->insert(pos[k], hint);
        hint = v;
        Sample& smp = samples_[k];
        smp.vertex = v;
        if (vertexSample_.size() <= idx(v)) {
            vertexSample_.resize(idx(v) + 1, -1);
        }
        smp.nextAtVertex = vertexSample_[idx(v)];
        vertexSample_[idx(v)] = static_cast<int>(k);
    }
    gabrielQueue_.reserve(n);
    for (std::size_t k = n; k-- > 0;) {
        gabrielQueue_.emplace_back(static_cast<int>(k), kUnknownEdge);
    }
}

ClosestPoint Solver::pieceClosest(int piece, Vec2 p) const {
    const Piece& pc = pieces_[idx(piece)];
    return pc.hullSlack > 0.0 ? bezierClosest(std::get<BezierSpan>(pc.span), p) : closestPoint(pc.span, p);
}

/// The best piece seen so far and the runners-up within tieTol of it (coincident walls).
struct NearCands {
    std::array<int, 4> piece{};
    std::array<ClosestPoint, 4> cp{};
    int count{0};

    double best() const { return count > 0 ? cp[0].distance : std::numeric_limits<double>::infinity(); }

    void consider(int item, const ClosestPoint& c, double tieTol) {
        const double bestNow = best();
        if (c.distance < bestNow || (c.distance == bestNow && item < piece[0])) {
            std::array<int, 4> np{};
            std::array<ClosestPoint, 4> nc{};
            int n = 0;
            np[0] = item;
            nc[0] = c;
            n = 1;
            for (int k = 0; k < count && n < 4; ++k) {
                if (cp[idx(k)].distance <= c.distance + tieTol) {
                    np[idx(n)] = piece[idx(k)];
                    nc[idx(n)] = cp[idx(k)];
                    ++n;
                }
            }
            piece = np;
            cp = nc;
            count = n;
        } else if (c.distance <= bestNow + tieTol && count < 4) {
            for (int k = 0; k < count; ++k) {
                if (piece[idx(k)] == item) {
                    return;
                }
            }
            piece[idx(count)] = item;
            cp[idx(count)] = c;
            ++count;
        }
    }

    void fill(Nearest& out) const {
        if (count == 0) {
            return;
        }
        out.dist = cp[0].distance;
        out.piece = piece[0];
        out.u = cp[0].t;
        out.point = cp[0].point;
        out.tieCount = 0;
        for (int k = 1; k < count; ++k) {
            out.ties[idx(out.tieCount++)] = {piece[idx(k)], cp[idx(k)].t, cp[idx(k)].point};
        }
    }
};

Nearest Solver::nearest(Vec2 p) const {
    Nearest best;
    double bestDist = 0.0;
    NearCands cands;
    tree_->nearest(
        p,
        [&](int item, double bestSoFar) {
            const Piece& pc = pieces_[idx(item)];
            if (pc.hullSlack > 0.0) {
                // The Bezier lies in its control hull, within hullSlack of its chord.
                const auto& bz = std::get<BezierSpan>(pc.span);
                const double lb = closestPoint(LineSpan{bz.p0, bz.p3}, p).distance - pc.hullSlack;
                if (lb > bestSoFar + tieTol_) {
                    return std::numeric_limits<double>::infinity();
                }
            }
            const ClosestPoint cp = pieceClosest(item, p);
            cands.consider(item, cp, tieTol_);
            return cp.distance;
        },
        bestDist, tieTol_, work_);
    cands.fill(best);
    return best;
}

Nearest Solver::cachedNearest(Vec2 p) {
    const std::pair<long long, long long> key{std::llround(p.x / mergeTol_), std::llround(p.y / mergeTol_)};
    const auto it = nearCache_.find(key);
    if (it != nearCache_.end()) {
        Nearest r = it->second;
        r.dist = distance(p, r.point);  // within a cell diagonal of the exact distance
        return r;
    }
    const Nearest r = nearest(p);
    nearCache_.emplace(key, r);
    return r;
}

bool Solver::insideOf(int piece, double u, Vec2 q, Vec2 v) const {
    const Piece& p = pieces_[idx(piece)];
    const Vec2 w = v - q;
    bool left = false;
    if (u > 0.0 && u < 1.0) {
        left = tangentAt(p.span, u).cross(w) > 0.0;
    } else {
        // At a piece end the local interior is the intersection (convex corner) or the union
        // (reflex corner) of the two pieces' left half-planes.
        Vec2 tin;
        Vec2 tout;
        if (u <= 0.0) {
            tin = tangentAt(pieces_[idx(p.prev)].span, 1.0);
            tout = tangentAt(p.span, 0.0);
        } else {
            tin = tangentAt(p.span, 1.0);
            tout = tangentAt(pieces_[idx(p.next)].span, 0.0);
        }
        const bool l1 = tin.cross(w) > 0.0;
        const bool l2 = tout.cross(w) > 0.0;
        left = tin.cross(tout) >= 0.0 ? (l1 && l2) : (l1 || l2);
    }
    return left == p.leftInside;
}

bool Solver::insideAt(const Nearest& nr, Vec2 v) const {
    if (nr.piece < 0 || !(nr.dist > 0.0)) {
        return false;
    }
    const bool in = insideOf(nr.piece, nr.u, nr.point, v);
    for (int k = 0; k < nr.tieCount; ++k) {
        const NearPiece& t = nr.ties[idx(k)];
        if (insideOf(t.piece, t.u, t.point, v) != in) {
            // Coincident walls that disagree (two regions sharing an edge, a hole lying on the
            // outer boundary): only the fill rule over all contours can decide.
            return windingInside(v);
        }
    }
    return in;
}

int Solver::windingAt(Vec2 v) const {
    return windingIndex_.winding(v);
}

bool Solver::windingInside(Vec2 v) const {
    return inRegion(windingAt(v));
}

bool Solver::isBoundaryEdge(int a, int b) const {
    if (idx(a) >= vertexSample_.size()) {
        return false;
    }
    for (int s = vertexSample_[idx(a)]; s >= 0; s = samples_[idx(s)].nextAtVertex) {
        const Sample& smp = samples_[idx(s)];
        if (samples_[idx(smp.next)].vertex == b || samples_[idx(smp.prev)].vertex == b) {
            return true;
        }
    }
    return false;
}

bool Solver::isSpoke(int a, int b, const TriData& d1, const TriData& d2) const {
    if (isBoundaryEdge(a, b)) {
        return true;
    }
    const Vec2 pa = dt_->point(a);
    const Vec2 pb = dt_->point(b);
    if (distance(pa, pb) < 2.0 * hmin_) {
        return true;  // the middle is within hmin_ of a
    }
    const Vec2 m = (pa + pb) * 0.5;
    // Distance is 1-Lipschitz: a middle near a deep circumcentre is deep (no query needed).
    const double lb = std::max(d1.radius - distance(m, d1.cc), d2.radius - distance(m, d2.cc));
    if (lb >= hmin_) {
        return false;
    }
    return nearest(m).dist < hmin_;
}

bool Solver::encroached(int a, int b, int hint) const {
    // A sample closer than gapTol_ to the sub-edge marks a gap where every inscribed circle is
    // below the tolerance (near-touching contours, near-duplicate walls, a sliver, the tip of a
    // sharp corner): refining there would cost samples in proportion to the gap's length over
    // its width for nothing visible, so it does not count.
    const Vec2 pa = dt_->point(a);
    const Vec2 pb = dt_->point(b);
    auto counts = [&](int v) {
        if (v == a || v == b || Delaunay::isSuper(v)) {
            return false;
        }
        return segmentDistance(pa, pb, dt_->point(v)) >= gapTol_;
    };
    int t1 = -1;
    if (hint >= 0 && dt_->alive(hint) && dt_->indexOf(hint, a) >= 0 && dt_->indexOf(hint, b) >= 0) {
        // The hint has the edge in one direction; the neighbour across it has the other.
        const auto& tr = dt_->triangle(hint);
        const int i = dt_->indexOf(hint, a);
        t1 = tr.v[idx((i + 1) % 3)] == b ? hint : tr.n[idx((i + 1) % 3)];
    } else if (hint != kRemovedEdge) {
        t1 = dt_->triangleWithEdge(a, b);
    }
    if (t1 < 0) {
        // Not a Delaunay edge: some sample lies in every circle through a and b. The edge the
        // segment leaves an endpoint's star through has the nearest such samples at its ends
        // (looked up around a, then around b, a bounded number of steps each: a fan centre can
        // have thousands of triangles; if not found, the sub-edge counts as encroached).
        for (const auto& [from, to] : {std::pair{a, b}, std::pair{b, a}}) {
            const Vec2 pf = dt_->point(from);
            const Vec2 pt = dt_->point(to);
            const int t0 = dt_->incidentTriangle(from);
            int t = t0;
            for (int step = 0; step < 64 && t >= 0; ++step) {
                const int i = dt_->indexOf(t, from);
                const auto& tr = dt_->triangle(t);
                const int v1 = tr.v[idx((i + 1) % 3)];
                const int v2 = tr.v[idx((i + 2) % 3)];
                if ((dt_->point(v1) - pf).cross(pt - pf) >= 0.0 && (pt - pf).cross(dt_->point(v2) - pf) >= 0.0) {
                    return counts(v1) || counts(v2);
                }
                t = tr.n[idx((i + 1) % 3)];
                if (t == t0) {
                    break;
                }
            }
        }
        return true;
    }
    const int ia = dt_->indexOf(t1, a);
    // The neighbour across a -> b is opposite the vertex following b, i.e. preceding a.
    const int t2 = dt_->triangle(t1).n[idx((ia + 2) % 3)];
    if (t2 < 0) {
        return true;
    }
    auto inDisc = [&](int v) {
        const Vec2 pf = dt_->point(v);
        return (pa - pf).dot(pb - pf) < 0.0;
    };
    bool hidden = false;  // an exempt sample inside the diametral circle may hide others
    for (int t : {t1, t2}) {
        for (int v : dt_->triangle(t).v) {
            if (v == a || v == b || Delaunay::isSuper(v) || !inDisc(v)) {
                continue;
            }
            if (counts(v)) {
                return true;
            }
            hidden = true;
        }
    }
    if (!hidden) {
        return false;
    }
    // T-0153: the opposite vertex lies in the circle but is exempt (within gapTol_ of the
    // sub-edge, e.g. the neighbour sample of a flat corner), so it says nothing about the other
    // samples in the circle: a thin band with such corners at both ends of its long edge had no
    // refinement and an empty skeleton. Look at every triangle reaching into the diametral disc
    // (bounded; when the bound is hit, count the sub-edge as encroached).
    const Vec2 centre = (pa + pb) * 0.5;
    const double radius = 0.5 * distance(pa, pb);
    std::vector<int> stack{t1, t2};
    std::vector<int> seen{t1, t2};
    constexpr std::size_t kMaxVisit = 512;
    while (!stack.empty()) {
        const int t = stack.back();
        stack.pop_back();
        const auto& tr = dt_->triangle(t);
        for (int k = 0; k < 3; ++k) {
            const int v = tr.v[idx(k)];
            if (v != a && v != b && !Delaunay::isSuper(v) && inDisc(v) && counts(v)) {
                return true;
            }
            const int nb = tr.n[idx(k)];
            if (nb < 0 || std::find(seen.begin(), seen.end(), nb) != seen.end()) {
                continue;
            }
            // Cross the edge opposite v[k] only when it reaches into the disc.
            const Vec2 e0 = dt_->point(tr.v[idx((k + 1) % 3)]);
            const Vec2 e1 = dt_->point(tr.v[idx((k + 2) % 3)]);
            if (segmentDistance(e0, e1, centre) >= radius) {
                continue;
            }
            if (seen.size() >= kMaxVisit) {
                return true;
            }
            seen.push_back(nb);
            stack.push_back(nb);
        }
    }
    return false;
}

bool Solver::splitSubEdge(int s, double u, SplitMode mode) {
    Sample& smp = samples_[idx(s)];
    if (smp.frozen) {
        return false;
    }
    const int s2 = smp.next;
    const int piece = smp.piece;
    Piece& p = pieces_[idx(piece)];
    const double u0 = smp.u;
    // The sub-edge's end parameter on this piece (the next sample may start the next piece).
    const auto it = p.samples.find(u0);
    if (it == p.samples.end() || std::next(it) == p.samples.end() || it->second != s) {
        return false;
    }
    const double u1 = std::next(it)->first;
    const Vec2 a = samplePos(s);
    const Vec2 b = samplePos(s2);
    if (distance(a, b) < 2.0 * hmin_) {
        return false;
    }
    const double du = u1 - u0;
    double um = 0.5 * (u0 + u1);
    if ((mode == SplitMode::Near && u > u0 + 0.25 * du && u < u1 - 0.25 * du) || mode == SplitMode::Exact) {
        um = u;
    }
    const Vec2 q = pointAt(p.span, um);
    if (!finitePoint(q) || q == a || q == b || !(um > u0) || !(um < u1)) {
        smp.frozen = true;
        return false;
    }
    if (samples_.size() >= maxSamples_ || work_ > maxWork_) {
        capped_ = true;
        return false;
    }
    const int v = dt_->insert(q, smp.vertex);
    const int ns = addSample(v, piece, um);
    samples_[idx(ns)].prev = s;
    samples_[idx(ns)].next = s2;
    samples_[idx(s)].next = ns;
    samples_[idx(s2)].prev = ns;
    samples_[idx(s)].splitRound = round_;
    samples_[idx(ns)].splitRound = round_;
    p.samples.emplace_hint(std::next(it), um, ns);
    // Only sub-edges whose neighbourhood changed need a new check: those that vanished inside
    // the cavity, and those on the new triangles (with the triangle as a lookup hint).
    gabrielQueue_.emplace_back(s, kUnknownEdge);
    gabrielQueue_.emplace_back(ns, kUnknownEdge);
    for (const auto& [ea, eb] : dt_->lastRemovedEdges()) {
        queueSubEdge(ea, eb, kRemovedEdge);
    }
    for (int t : dt_->lastNewTriangles()) {
        const auto& tr = dt_->triangle(t);
        for (int k = 0; k < 3; ++k) {
            queueSubEdge(tr.v[idx(k)], tr.v[idx((k + 1) % 3)], t);
        }
    }
    return true;
}

bool Solver::splitAt(int piece, double u, double spacing) {
    const Piece& p = pieces_[idx(piece)];
    auto it = p.samples.upper_bound(u);
    if (it == p.samples.begin()) {
        ++it;
    }
    if (it == p.samples.end()) {
        --it;
    }
    const auto prevIt = std::prev(it);
    const int s = prevIt->second;
    if (samples_[idx(s)].splitRound == round_) {
        return false;  // already split this round: the request is stale, re-checked next round
    }
    const double u0 = prevIt->first;
    const double u1 = it->first;
    const double len = distance(samplePos(s), samplePos(samples_[idx(s)].next));
    // A sub-edge much longer than the local radius needs is cut into equal parts at once,
    // instead of one split per round.
    int k = 1;
    if (spacing > 0.0 && len > 2.0 * spacing) {
        k = static_cast<int>(std::min({std::ceil(len / spacing), std::floor(len / hmin_), 64.0}));
    }
    if (k <= 2) {
        return splitSubEdge(s, u, SplitMode::Near);
    }
    bool any = false;
    int cur = s;
    for (int i = 1; i < k; ++i) {
        const double ui = u0 + (u1 - u0) * static_cast<double>(i) / static_cast<double>(k);
        if (!splitSubEdge(cur, ui, SplitMode::Exact)) {
            break;
        }
        any = true;
        cur = samples_[idx(cur)].next;
    }
    return any;
}

void Solver::queueSubEdge(int a, int b, int hint) {
    if (Delaunay::isSuper(a) || Delaunay::isSuper(b) || idx(a) >= vertexSample_.size()) {
        return;
    }
    for (int r = vertexSample_[idx(a)]; r >= 0; r = samples_[idx(r)].nextAtVertex) {
        const Sample& smp = samples_[idx(r)];
        if (samples_[idx(smp.next)].vertex == b) {
            gabrielQueue_.emplace_back(r, hint);
        }
        if (samples_[idx(smp.prev)].vertex == b) {
            gabrielQueue_.emplace_back(smp.prev, hint);
        }
    }
}

void Solver::processGabriel() {
    // First in, first out: sub-edges are refined level by level, not one sub-edge depth-first
    // along a line (which makes every insertion's cavity long in thin channels).
    std::size_t head = 0;
    while (head < gabrielQueue_.size() && !capped_) {
        const auto [s, hint] = gabrielQueue_[head++];
        if (head > 4096 && head * 2 > gabrielQueue_.size()) {
            gabrielQueue_.erase(gabrielQueue_.begin(), gabrielQueue_.begin() + static_cast<std::ptrdiff_t>(head));
            head = 0;
        }
        const Sample& smp = samples_[idx(s)];
        if (smp.frozen) {
            continue;
        }
        const int a = smp.vertex;
        const int b = samples_[idx(smp.next)].vertex;
        if (a == b) {
            continue;
        }
        if (distance(dt_->point(a), dt_->point(b)) < 2.0 * hmin_) {
            continue;
        }
        if (encroached(a, b, hint)) {
            splitSubEdge(s, 0.0, SplitMode::Midpoint);
        }
    }
    gabrielQueue_.erase(gabrielQueue_.begin(), gabrielQueue_.begin() + static_cast<std::ptrdiff_t>(head));
}

bool Solver::checkSegment(Vec2 p1, double r1, Vec2 p2, double r2, Vec2 gen, int depth,
                          std::vector<SplitRequest>* requests, std::vector<MedialPoint>* out) const {
    if (depth > 20 || distance(p1, p2) <= mergeTol_) {
        return true;
    }
    const Vec2 m = (p1 + p2) * 0.5;
    const Nearest nr = nearest(m);
    if (!insideAt(nr, m)) {
        return false;
    }
    const double rho = distance(m, gen);
    if (requests != nullptr && rho - nr.dist > emax_) {
        // One request per edge and round: the triangulation changes around the split, and the
        // new triangles are checked in the next round.
        requests->push_back({nr.piece, nr.u, -1, spacingFor(nr.dist)});
        return true;
    }
    if (std::fabs(nr.dist - 0.5 * (r1 + r2)) > 0.25 * chordal_) {
        const std::size_t before = requests != nullptr ? requests->size() : 0;
        bool ok = checkSegment(p1, r1, m, nr.dist, gen, depth + 1, requests, out);
        if (requests != nullptr && requests->size() > before) {
            return ok;
        }
        if (out != nullptr) {
            out->push_back({m, nr.dist});
        }
        ok = checkSegment(m, nr.dist, p2, r2, gen, depth + 1, requests, out) && ok;
        return ok;
    }
    return true;
}

void Solver::evaluateCreated(std::vector<SplitRequest>* requests) {
    ++round_;
    std::vector<int> created = dt_->takeCreated();
    created.insert(created.end(), recheck_.begin(), recheck_.end());
    recheck_.clear();
    if (tri_.size() < dt_->triangleSlots()) {
        tri_.resize(dt_->triangleSlots());
    }
    const double farLimit = 4.0 * std::max(bounds_.width(), bounds_.height()) + 1.0;
    std::vector<int> fresh;
    fresh.reserve(created.size());
    for (int t : created) {
        if (!dt_->alive(t) || tri_[idx(t)].stamp == round_) {
            continue;
        }
        TriData& d = tri_[idx(t)];
        d.stamp = round_;
        d.interior = false;
        d.requested = false;
        fresh.push_back(t);
        const auto& tr = dt_->triangle(t);
        if (Delaunay::isSuper(tr.v[0]) || Delaunay::isSuper(tr.v[1]) || Delaunay::isSuper(tr.v[2])) {
            continue;
        }
        const Vec2 a = dt_->point(tr.v[0]);
        const Vec2 d1 = dt_->point(tr.v[1]) - a;
        const Vec2 d2 = dt_->point(tr.v[2]) - a;
        const double den = 2.0 * d1.cross(d2);
        if (den == 0.0) {
            continue;
        }
        const double l1 = d1.lengthSquared();
        const double l2 = d2.lengthSquared();
        const Vec2 off{(d2.y * l1 - d1.y * l2) / den, (d1.x * l2 - d2.x * l1) / den};
        const double rr = off.length();
        if (!std::isfinite(rr) || rr > farLimit) {
            continue;
        }
        d.cc = a + off;
        d.circumradius = rr;
        if (!bounds_.contains(d.cc)) {
            continue;
        }
        const Nearest nr = cachedNearest(d.cc);
        d.radius = nr.dist;
        d.interior = insideAt(nr, d.cc);
        if (d.interior && requests != nullptr && rr - nr.dist > emax_) {
            requests->push_back({nr.piece, nr.u, t, spacingFor(nr.dist)});
            d.requested = true;
        } else if (!d.interior && requests != nullptr && nr.piece >= 0 && nr.dist <= mergeTol_ && rr > emax_) {
            // T-0153: a circumcentre exactly on the boundary (symmetric input: the base of an
            // isosceles triangle split at its middle under an obtuse apex puts both circumcentres
            // on the legs) is neither inside nor outside, so no interior triangle would ever ask
            // for refinement and the skeleton stayed empty. Sample the boundary there.
            requests->push_back({nr.piece, nr.u, t, spacingFor(rr)});
            d.requested = true;
        }
    }
    if (requests == nullptr) {
        return;
    }
    for (int t : fresh) {
        const TriData& d = tri_[idx(t)];
        if (!d.interior || d.requested) {
            continue;
        }
        const auto& tr = dt_->triangle(t);
        for (int k = 0; k < 3; ++k) {
            const int nb = tr.n[idx(k)];
            if (nb < 0 || !tri_[idx(nb)].interior || (tri_[idx(nb)].stamp == round_ && tri_[idx(nb)].requested)) {
                continue;
            }
            if (tri_[idx(nb)].stamp == round_ && nb < t) {
                continue;  // both new: checked from the other side
            }
            const int va = tr.v[idx((k + 1) % 3)];
            const int vb = tr.v[idx((k + 2) % 3)];
            const TriData& e = tri_[idx(nb)];
            if (isSpoke(va, vb, d, e)) {
                continue;
            }
            const std::size_t before = requests->size();
            checkSegment(d.cc, d.radius, e.cc, e.radius, dt_->point(va), 0, requests, nullptr);
            if (requests->size() > before) {
                requests->back().tri = t;
                recheck_.push_back(nb);
            }
        }
    }
}

MedialBoundaryRef Solver::refOf(int vertex) const {
    MedialBoundaryRef ref;
    if (idx(vertex) >= vertexSample_.size() || vertexSample_[idx(vertex)] < 0) {
        return ref;
    }
    const Sample& smp = samples_[idx(vertexSample_[idx(vertex)])];
    const Piece& p = pieces_[idx(smp.piece)];
    ref.contour = p.contour;
    ref.span = p.spanIndex;
    ref.t = p.t0 + smp.u * (p.t1 - p.t0);
    return ref;
}

MedialAxis Solver::assemble() {
    MedialAxis out;
    const int slots = static_cast<int>(dt_->triangleSlots());
    std::vector<int> parent(idx(slots), -1);
    auto find = [&](int x) {
        while (parent[idx(x)] != x) {
            parent[idx(x)] = parent[idx(parent[idx(x)])];
            x = parent[idx(x)];
        }
        return x;
    };
    std::vector<int> interior;
    for (int t = 0; t < slots; ++t) {
        if (dt_->alive(t) && idx(t) < tri_.size() && tri_[idx(t)].interior) {
            parent[idx(t)] = t;
            interior.push_back(t);
        }
    }
    struct DualEdge {
        int t;
        int nb;
        int va;
        int vb;
    };
    std::vector<DualEdge> duals;
    for (int t : interior) {
        const auto& tr = dt_->triangle(t);
        for (int k = 0; k < 3; ++k) {
            const int nb = tr.n[idx(k)];
            if (nb < t || parent[idx(nb)] < 0) {
                continue;
            }
            const int va = tr.v[idx((k + 1) % 3)];
            const int vb = tr.v[idx((k + 2) % 3)];
            // Coincident circumcentres (co-circular samples) are one point whatever the edge.
            if (distance(tri_[idx(t)].cc, tri_[idx(nb)].cc) <= mergeTol_) {
                const int ra = find(t);
                const int rb = find(nb);
                if (ra != rb) {
                    parent[idx(std::max(ra, rb))] = std::min(ra, rb);
                }
                continue;
            }
            if (isSpoke(va, vb, tri_[idx(t)], tri_[idx(nb)])) {
                continue;
            }
            duals.push_back({t, nb, va, vb});
        }
    }
    // Graph nodes: cluster roots, numbered in triangle order.
    std::vector<int> nodeOf(idx(slots), -1);
    std::vector<int> nodeTri;
    for (int t : interior) {
        const int r = find(t);
        if (nodeOf[idx(r)] < 0) {
            nodeOf[idx(r)] = static_cast<int>(nodeTri.size());
            nodeTri.push_back(r);
        }
    }
    struct GEdge {
        int a;
        int b;
        int va;
        int vb;
        std::vector<MedialPoint> inner;  // subdivision points from node a towards node b
    };
    std::vector<GEdge> edges;
    for (const DualEdge& e : duals) {
        const int na = nodeOf[idx(find(e.t))];
        const int nb = nodeOf[idx(find(e.nb))];
        if (na == nb) {
            continue;
        }
        edges.push_back({std::min(na, nb), std::max(na, nb), e.va, e.vb, {}});
    }
    std::sort(edges.begin(), edges.end(), [](const GEdge& x, const GEdge& y) {
        return x.a < y.a || (x.a == y.a && x.b < y.b);
    });
    edges.erase(std::unique(edges.begin(), edges.end(),
                            [](const GEdge& x, const GEdge& y) { return x.a == y.a && x.b == y.b; }),
                edges.end());
    // Subdivide every edge until the radius interpolates within tolerance, and drop edges that
    // leave the region (possible only below the refinement floor).
    {
        std::vector<GEdge> kept;
        kept.reserve(edges.size());
        for (GEdge& e : edges) {
            const TriData& da = tri_[idx(nodeTri[idx(e.a)])];
            const TriData& db = tri_[idx(nodeTri[idx(e.b)])];
            if (checkSegment(da.cc, da.radius, db.cc, db.radius, dt_->point(e.va), 0, nullptr, &e.inner)) {
                kept.push_back(std::move(e));
            }
        }
        edges.swap(kept);
    }
    const std::size_t nodeCount = nodeTri.size();
    std::vector<std::vector<int>> adj(nodeCount);
    for (std::size_t e = 0; e < edges.size(); ++e) {
        adj[idx(edges[e].a)].push_back(static_cast<int>(e));
        adj[idx(edges[e].b)].push_back(static_cast<int>(e));
    }
    std::vector<int> outNode(nodeCount, -1);
    auto nodeFor = [&](int g) {
        if (outNode[idx(g)] < 0) {
            outNode[idx(g)] = static_cast<int>(out.nodes.size());
            const TriData& d = tri_[idx(nodeTri[idx(g)])];
            out.nodes.push_back(MedialNode{d.cc, d.radius, {}});
        }
        return outNode[idx(g)];
    };
    std::vector<char> used(edges.size(), 0);
    auto walk = [&](int startNode, int firstEdge) {
        MedialBranch br;
        const int startOut = nodeFor(startNode);
        br.startNode = idx(startOut);
        const TriData& sd = tri_[idx(nodeTri[idx(startNode)])];
        br.points.push_back({sd.cc, sd.radius});
        int cur = startNode;
        int e = firstEdge;
        while (true) {
            used[idx(e)] = 1;
            const GEdge& ge = edges[idx(e)];
            const int nxt = ge.a == cur ? ge.b : ge.a;
            const TriData& da = tri_[idx(nodeTri[idx(cur)])];
            const TriData& db = tri_[idx(nodeTri[idx(nxt)])];
            const Vec2 pa = dt_->point(ge.va);
            const bool aLeft = (db.cc - da.cc).cross(pa - da.cc) > 0.0;
            const MedialBoundaryRef ra = refOf(ge.va);
            const MedialBoundaryRef rb = refOf(ge.vb);
            const std::array<MedialBoundaryRef, 2> side = aLeft ? std::array{ra, rb} : std::array{rb, ra};
            const std::size_t before = br.points.size();
            if (ge.a == cur) {
                br.points.insert(br.points.end(), ge.inner.begin(), ge.inner.end());
            } else {
                br.points.insert(br.points.end(), ge.inner.rbegin(), ge.inner.rend());
            }
            br.points.push_back({db.cc, db.radius});
            for (std::size_t k = before; k < br.points.size(); ++k) {
                br.sides.push_back(side);
            }
            cur = nxt;
            if (adj[idx(cur)].size() != 2 || outNode[idx(cur)] >= 0) {
                break;
            }
            const int e0 = adj[idx(cur)][0];
            const int e1 = adj[idx(cur)][1];
            const int ne = e0 == e ? e1 : e0;
            if (used[idx(ne)] != 0) {
                break;
            }
            e = ne;
        }
        const int endOut = nodeFor(cur);
        br.endNode = idx(endOut);
        const std::size_t bi = out.branches.size();
        out.nodes[br.startNode].branches.push_back(bi);
        out.nodes[br.endNode].branches.push_back(bi);
        out.branches.push_back(std::move(br));
    };
    // An isolated point is kept only when its own samples are distinct features (e.g. the
    // centre of a circle); three consecutive samples of a curve give the centre of its
    // osculating circle, which is no medial point.
    std::vector<char> significant(nodeCount, 0);
    for (int t : interior) {
        const auto g = idx(nodeOf[idx(find(t))]);
        if (significant[g] != 0 || !adj[g].empty()) {
            continue;
        }
        const auto& tr = dt_->triangle(t);
        const TriData& d = tri_[idx(t)];
        for (int k = 0; k < 3; ++k) {
            if (!isSpoke(tr.v[idx(k)], tr.v[idx((k + 1) % 3)], d, d)) {
                significant[g] = 1;
                break;
            }
        }
    }
    for (std::size_t g = 0; g < nodeCount; ++g) {
        if (adj[g].empty() && significant[g] == 0) {
            continue;
        }
        if (adj[g].size() != 2) {
            nodeFor(static_cast<int>(g));
            for (int e : adj[g]) {
                if (used[idx(e)] == 0) {
                    walk(static_cast<int>(g), e);
                }
            }
        }
    }
    for (std::size_t g = 0; g < nodeCount; ++g) {  // closed loops without junctions
        for (int e : adj[g]) {
            if (used[idx(e)] == 0) {
                nodeFor(static_cast<int>(g));
                walk(static_cast<int>(g), e);
            }
        }
    }
    return out;
}

MedialAxis Solver::run() {
    chordal_ = tol_.chordal > 0.0 && std::isfinite(tol_.chordal) ? tol_.chordal : 0.01;
    std::vector<int> leftInside;
    classifyContours(leftInside);
    buildPieces(leftInside);
    MedialAxis out;
    // Beyond ~1e10 mm the rounding floors exceed the tolerance budget: the result is still
    // computed, but flagged (the guarantees cannot hold there). Judged on every usable input
    // contour, before any early return, so an empty result is flagged too.
    Box2 inputBox;
    for (std::size_t i = 0; i < contours_.size(); ++i) {
        if (usable_[i] != 0) {
            inputBox.expand(contourBoxes_[i]);
        }
    }
    if (!inputBox.isEmpty()) {
        const double inputScale = std::max({std::fabs(inputBox.min.x), std::fabs(inputBox.min.y), std::fabs(inputBox.max.x),
                                            std::fabs(inputBox.max.y), inputBox.width(), inputBox.height()});
        beyondScale_ = 1e-12 * inputScale > 0.5 * chordal_;
    }
    if (pieces_.empty() || bounds_.isEmpty()) {
        out.ignoredContours = ignored_;
        out.accuracyCapped = beyondScale_;
        return out;
    }
    scale_ = std::max({std::fabs(bounds_.min.x), std::fabs(bounds_.min.y), std::fabs(bounds_.max.x),
                       std::fabs(bounds_.max.y), bounds_.width(), bounds_.height()});
    const double fpFloor = 1e-12 * scale_;
    beyondScale_ = beyondScale_ || fpFloor > 0.5 * chordal_;
    emax_ = std::max(0.5 * chordal_, fpFloor);
    hmin_ = std::max(0.5 * chordal_, fpFloor);
    tieTol_ = fpFloor;
    gapTol_ = std::max(2.0 * chordal_, fpFloor);
    maxSamples_ = options_.maxSamples > 0 ? options_.maxSamples
                                          : std::max<std::size_t>(250000, 64 * pieces_.size());
    maxWork_ = 300 * static_cast<std::uint64_t>(maxSamples_);
    mergeTol_ = std::max(1e-3 * std::min(tol_.distance > 0.0 ? tol_.distance : chordal_, chordal_), fpFloor);
    dt_ = std::make_unique<Delaunay>(bounds_.inflated(std::max(bounds_.width(), bounds_.height()) * 0.01 + 1e-9));
    insertInitialSamples();
    std::vector<SplitRequest> requests;
    for (int rounds = 0; rounds < 10000; ++rounds) {
        processGabriel();
        requests.clear();
        evaluateCreated(&requests);
        if (requests.empty() || capped_) {
            break;
        }
        std::size_t applied = 0;
        for (const SplitRequest& r : requests) {
            if (splitAt(r.piece, r.u, r.spacing)) {
                ++applied;
            }
            recheck_.push_back(r.tri);
            if (capped_) {
                break;
            }
        }
        if (applied == 0) {
            break;
        }
    }
    evaluateCreated(nullptr);
    out = assemble();
    out.ignoredContours = ignored_;
    out.accuracyCapped = capped_ || beyondScale_;
    out.boundarySamples = samples_.size();
    return out;
}

}  // namespace

MedialAxis medialAxis(const std::vector<Contour>& contours, const Tolerance& tol, const MedialAxisOptions& options) {
    Solver solver(contours, tol, options);
    return solver.run();
}

}  // namespace carvetto::geometry
