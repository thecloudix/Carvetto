#include "enclosed_pieces.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <iterator>
#include <limits>
#include <numbers>
#include <numeric>
#include <optional>
#include <span>
#include <tuple>
#include <utility>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fixed_point.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/span.hpp"

namespace carvetto::toolpath::detail {

namespace {

using geometry::Box2;
using geometry::Contour;
using geometry::Vec2;

struct Ring {
    std::vector<Vec2> pts;
    Box2 box;
};

// A coordinate on the nanometre grid of the booleans, as an integer and back.
// (Rounded half away from zero like llround, without its library call.)
std::int64_t gridUnits(double v) {
    const double u = v * geometry::kFixedPointScale;
    return u >= 0.0 ? static_cast<std::int64_t>(u + 0.5) : -static_cast<std::int64_t>(0.5 - u);
}
double onGrid(double v) { return static_cast<double>(gridUnits(v)) / geometry::kFixedPointScale; }

// Rings are clipped this far beyond a cut: exact within it, and their clipped edges far from the
// leaf's sides compared with the grid.
constexpr double kCropMargin = 1e-3;

// Parts on either side of a leaf side are joined where they meet over at least this many grid
// units (10 nm): the two leaves compute where the union crosses their side separately, a few
// units apart, and pieces must not join through a gap that narrow.
constexpr std::int64_t kMinOverlap = 10;

// Leaf cuts lie at least this many grid units from every vertex. Clipper2 truncates (does not
// round) where an edge crosses a leaf side, so such a vertex of the output can lie a unit off the
// side; processLeaf moves output vertices within a unit of a side onto it, which then never moves
// a vertex of the rings.
constexpr std::int64_t kVertexClear = 2;

double along(Vec2 p, int axis) { return axis == 0 ? p.x : p.y; }

// The part of `ring` on one side of the line along `axis` = cut (`low`: coordinate <= cut),
// Sutherland-Hodgman: the winding number of every point strictly on that side is the ring's (each
// excursion beyond the line is replaced by the segment along it between its ends).
std::vector<Vec2> clipHalf(const std::vector<Vec2>& in, int axis, double cut, bool low) {
    auto inside = [&](Vec2 p) { return low ? along(p, axis) <= cut : along(p, axis) >= cut; };
    auto cross = [&](Vec2 p, Vec2 q) {
        if (along(q, axis) < along(p, axis)) std::swap(p, q);
        const double t = (cut - along(p, axis)) / (along(q, axis) - along(p, axis));
        Vec2 r = p + (q - p) * t;
        (axis == 0 ? r.x : r.y) = cut;
        return r;
    };
    std::vector<Vec2> out;
    const std::size_t n = in.size();
    auto push = [&](Vec2 v) {
        if (out.empty() || out.back() != v) out.push_back(v);
    };
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 p = in[i];
        const Vec2 q = in[(i + 1) % n];
        const bool pin = inside(p);
        const bool qin = inside(q);
        if (pin && qin) {
            push(q);
        } else if (pin) {
            push(cross(p, q));
        } else if (qin) {
            push(cross(p, q));
            push(q);
        }
    }
    while (out.size() > 1 && out.front() == out.back()) out.pop_back();
    if (out.size() < 3) out.clear();
    return out;
}

Box2 boxOf(const std::vector<Vec2>& pts) {
    Box2 b;
    for (const Vec2 p : pts) b.expand(p);
    return b;
}

// One way to split a set of rings at a cut: where each ring goes and the clipped halves.
struct Split {
    int axis{0};
    double cut{0.0};
    std::vector<int> where;   // -1 below, +1 above, 0 both (its two halves next in `halves`)
    std::vector<Ring> halves;  // below and above versions of the rings that go to both (or empty)
};

// The split of `rings` (holding `count` vertices) across `axis` inside `rect`, or nothing when
// a side would be narrower than `minSide`, no cut lies kVertexClear grid units clear of every
// vertex, or a side would keep more than 4/5 of the vertices (rings crossing the cut are copied to both sides, e.g. long edges running across the
// whole rectangle), so the depth stays logarithmic.
std::optional<Split> planSplit(const Box2& rect, const std::vector<Ring>& rings, std::size_t count, int axis,
                               double minSide) {
    const double lo = axis == 0 ? rect.min.x : rect.min.y;
    const double hi = axis == 0 ? rect.max.x : rect.max.y;
    if (!(hi - lo > 4.0 * minSide)) return std::nullopt;
    // The median of every k-th vertex coordinate (at most about 2048 of them), moved half way to
    // a neighbouring coordinate on the grid so that every vertex lies at least kVertexClear grid
    // units off the cut (a ring edge along a leaf side would meet the side of the leaf's rectangle
    // in the boolean, and processLeaf moves output vertices within a unit of a side onto it).
    const std::size_t stride = count / 2048 + 1;
    std::vector<double> sample;
    sample.reserve(count / stride + 1);
    std::size_t skip = 0;  // vertices of the next ring to pass over first
    for (const Ring& r : rings) {
        std::size_t k = skip;
        for (; k < r.pts.size(); k += stride) sample.push_back(along(r.pts[k], axis));
        skip = k - r.pts.size();
    }
    if (sample.empty()) return std::nullopt;
    const auto mid = sample.begin() + static_cast<std::ptrdiff_t>(sample.size() / 2);
    std::nth_element(sample.begin(), mid, sample.end());
    double m = *mid;
    if (!(m > lo + minSide && m < hi - minSide)) m = 0.5 * (lo + hi);
    double next = std::numeric_limits<double>::infinity();
    double prev = -std::numeric_limits<double>::infinity();
    for (const Ring& r : rings) {
        for (const Vec2 p : r.pts) {
            const double v = along(p, axis);
            next = v > m ? std::min(next, v) : next;
            prev = v < m ? std::max(prev, v) : prev;
        }
    }
    // No coordinate lies strictly between m and next (or prev and m), so no vertex rounds to a
    // grid value strictly between theirs. Otherwise the middle of the rectangle, when it is clear
    // of every vertex.
    const std::int64_t gm = gridUnits(m);
    std::int64_t c = 0;
    if (std::isfinite(next) && gridUnits(next) - gm >= 2 * kVertexClear) {
        c = gm + (gridUnits(next) - gm) / 2;
    } else if (std::isfinite(prev) && gm - gridUnits(prev) >= 2 * kVertexClear) {
        c = gridUnits(prev) + (gm - gridUnits(prev)) / 2;
    } else {
        c = gridUnits(0.5 * (lo + hi));
        for (const Ring& r : rings) {
            for (const Vec2 p : r.pts) {
                if (std::abs(gridUnits(along(p, axis)) - c) < kVertexClear) return std::nullopt;
            }
        }
    }
    Split sp;
    sp.axis = axis;
    sp.cut = static_cast<double>(c) / geometry::kFixedPointScale;
    if (!(sp.cut >= lo + 0.5 * minSide && sp.cut <= hi - 0.5 * minSide)) return std::nullopt;
    // Each ring goes to the side it lies on, or to both when it reaches within kCropMargin of the
    // cut, clipped kCropMargin beyond it (so the clipped edges along the cut stay off the leaf's
    // sides).
    const double cut = sp.cut;
    sp.where.assign(rings.size(), 0);
    std::size_t nBelow = 0;
    std::size_t nAbove = 0;
    for (std::size_t i = 0; i < rings.size(); ++i) {
        const Ring& r = rings[i];
        const double rlo = axis == 0 ? r.box.min.x : r.box.min.y;
        const double rhi = axis == 0 ? r.box.max.x : r.box.max.y;
        // Each side needs the ring when it reaches that side's crop line, whole when it lies
        // within it, clipped otherwise.
        const bool toBelow = rlo < cut + kCropMargin;
        const bool toAbove = rhi > cut - kCropMargin;
        const bool wholeBelow = rhi <= cut + kCropMargin;
        const bool wholeAbove = rlo >= cut - kCropMargin;
        if (toBelow && wholeBelow && !toAbove) {
            sp.where[i] = -1;
            nBelow += r.pts.size();
            continue;
        }
        if (toAbove && wholeAbove && !toBelow) {
            sp.where[i] = 1;
            nAbove += r.pts.size();
            continue;
        }
        for (const bool low : {true, false}) {
            Ring half;
            if (low ? toBelow : toAbove) {
                if (low ? wholeBelow : wholeAbove) {
                    half = r;
                } else {
                    half.pts = clipHalf(r.pts, axis, low ? cut + kCropMargin : cut - kCropMargin, low);
                    half.box = boxOf(half.pts);
                }
            }
            (low ? nBelow : nAbove) += half.pts.size();
            sp.halves.push_back(std::move(half));
        }
    }
    if (5 * std::max(nBelow, nAbove) > 4 * count) return std::nullopt;
    return sp;
}

// True when an edge of the ring meets the closed rectangle.
bool entersRect(const Ring& r, const Box2& rect) {
    const std::size_t n = r.pts.size();
    for (std::size_t k = 0; k < n; ++k) {
        Vec2 a = r.pts[k];
        Vec2 b = r.pts[(k + 1) % n];
        // Liang-Barsky: clip the segment to the rectangle.
        double t0 = 0.0;
        double t1 = 1.0;
        const Vec2 d = b - a;
        bool hit = true;
        const double p[4] = {-d.x, d.x, -d.y, d.y};
        const double q[4] = {a.x - rect.min.x, rect.max.x - a.x, a.y - rect.min.y, rect.max.y - a.y};
        for (int i = 0; i < 4 && hit; ++i) {
            if (p[i] == 0.0) {
                if (q[i] < 0.0) hit = false;
            } else {
                const double t = q[i] / p[i];
                if (p[i] < 0.0) {
                    t0 = std::max(t0, t);
                } else {
                    t1 = std::min(t1, t);
                }
                if (t0 > t1) hit = false;
            }
        }
        if (hit) return true;
    }
    return false;
}

// The winding number of the ring about q.
int windingAt(const Ring& r, Vec2 q) {
    int w = 0;
    const std::size_t n = r.pts.size();
    for (std::size_t k = 0; k < n; ++k) {
        const Vec2 u = r.pts[k];
        const Vec2 v = r.pts[(k + 1) % n];
        if (u.y <= q.y) {
            if (v.y > q.y && (v - u).cross(q - u) > 0.0) ++w;
        } else if (v.y <= q.y && (v - u).cross(q - u) < 0.0) {
            --w;
        }
    }
    return w;
}

// The k-d subdivision (see the header): across the longer side of the rectangle, or the other
// when that keeps too much on one side. A ring (or what is left of it) that surrounds the whole
// rectangle without an edge in it only adds a constant to the winding number there: it is taken
// out and its winding number added to `winding` (rings that surround a region in pairs, a band
// of a closed path round it, cancel), so it neither counts as vertices here nor goes down to
// every leaf inside. `leaf(rect, rings, winding)` is called for each leaf, depth first, low side
// first.
template <class LeafFn>
void subdivide(const Box2& rect, std::vector<Ring> rings, int winding, std::size_t leafVertices, double minSide,
               int depth, LeafFn& leaf) {
    std::size_t kept = 0;
    for (std::size_t i = 0; i < rings.size(); ++i) {
        if (rings[i].box.contains(rect) && !entersRect(rings[i], rect)) {
            winding += windingAt(rings[i], rect.centre());
            continue;
        }
        if (kept != i) rings[kept] = std::move(rings[i]);
        ++kept;
    }
    rings.resize(kept);
    std::size_t count = 0;
    for (const Ring& r : rings) count += r.pts.size();
    std::optional<Split> sp;
    if (count > leafVertices && depth < 64) {
        const int first = rect.width() >= rect.height() ? 0 : 1;
        sp = planSplit(rect, rings, count, first, minSide);
        if (!sp) sp = planSplit(rect, rings, count, 1 - first, minSide);
    }
    if (!sp) {
        leaf(rect, std::move(rings), winding);
        return;
    }
    std::vector<Ring> below;
    std::vector<Ring> above;
    std::size_t h = 0;
    for (std::size_t i = 0; i < rings.size(); ++i) {
        if (sp->where[i] < 0) {
            below.push_back(std::move(rings[i]));
        } else if (sp->where[i] > 0) {
            above.push_back(std::move(rings[i]));
        } else {
            for (std::vector<Ring>* side : {&below, &above}) {
                if (!sp->halves[h].pts.empty()) side->push_back(std::move(sp->halves[h]));
                ++h;
            }
        }
    }
    rings.clear();
    rings.shrink_to_fit();
    Box2 rlo = rect;
    Box2 rhi = rect;
    (sp->axis == 0 ? rlo.max.x : rlo.max.y) = sp->cut;
    (sp->axis == 0 ? rhi.min.x : rhi.min.y) = sp->cut;
    sp.reset();
    subdivide(rlo, std::move(below), winding, leafVertices, minSide, depth + 1, leaf);
    subdivide(rhi, std::move(above), winding, leafVertices, minSide, depth + 1, leaf);
}

Contour polygon(const std::vector<Vec2>& pts) {
    Contour c;
    c.closed = true;
    c.spans.reserve(pts.size());
    for (std::size_t k = 0; k < pts.size(); ++k) c.spans.push_back(geometry::LineSpan{pts[k], pts[(k + 1) % pts.size()]});
    return c;
}

// A part of material in one leaf: an outer boundary and the islands inside it.
struct Part {
    std::size_t outer{0};  // index into the leaf output
    std::vector<std::size_t> islands;
    double area{0.0};
    Box2 box;
    bool frame{false};  // touches the outer frame: the outside
};

// Where a part meets a leaf edge: on the line along `axis` (0: x = coord) over [lo, hi], with
// the part on the side `side` (+1 above, -1 below the line).
struct Contact {
    int axis{0};
    std::int64_t coord{0};
    std::int64_t lo{0};
    std::int64_t hi{0};
    int side{0};
    std::size_t part{0};
};

// The area centroid of a polygon of line spans (its first vertex when it has no area).
Vec2 centroid(const Contour& c) {
    double a = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    const Vec2 o = geometry::startPoint(c.spans.front());  // origin, for accuracy far from (0, 0)
    for (const geometry::Span& sp : c.spans) {
        const Vec2 u = geometry::startPoint(sp) - o;
        const Vec2 v = geometry::endPoint(sp) - o;
        const double w = u.cross(v);
        a += w;
        cx += (u.x + v.x) * w;
        cy += (u.y + v.y) * w;
    }
    if (a == 0.0) return o;
    return o + Vec2{cx / (3.0 * a), cy / (3.0 * a)};
}

// True when q lies inside the region (winding number of the boundaries, outer counter-clockwise
// and islands clockwise, not zero) at least `clearance` from every edge.
bool clearOf(const std::vector<Contour>& region, Vec2 q, double clearance) {
    int winding = 0;
    for (const Contour& c : region) {
        for (const geometry::Span& sp : c.spans) {
            const Vec2 u = geometry::startPoint(sp);
            const Vec2 v = geometry::endPoint(sp);
            const Vec2 d = v - u;
            const double len2 = d.dot(d);
            const double t = len2 > 0.0 ? std::clamp((q - u).dot(d) / len2, 0.0, 1.0) : 0.0;
            const Vec2 e = u + d * t - q;
            if (e.dot(e) < clearance * clearance) return false;
            if (u.y <= q.y) {
                if (v.y > q.y && d.cross(q - u) > 0.0) ++winding;
            } else if (v.y <= q.y && d.cross(q - u) < 0.0) {
                --winding;
            }
        }
    }
    return winding != 0;
}

struct UnionFind {
    std::vector<std::size_t> parent;
    std::size_t find(std::size_t x) {
        while (parent[x] != x) {
            parent[x] = parent[parent[x]];
            x = parent[x];
        }
        return x;
    }
    void join(std::size_t a, std::size_t b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (b < a) std::swap(a, b);
        parent[b] = a;
    }
};

}  // namespace

std::vector<EnclosedPiece> enclosedPieces(std::vector<PointRing> rings, const EnclosedOptions& opt) {
    std::vector<Ring> input;
    input.reserve(rings.size());
    Box2 all;
    for (PointRing& pts : rings) {
        while (pts.size() > 1 && pts.front() == pts.back()) pts.pop_back();
        if (pts.size() < 3) continue;
        const Box2 b = boxOf(pts);
        all.expand(b);
        input.push_back({std::move(pts), b});
    }
    rings.clear();
    if (input.empty()) return {};
    // The frame, on the grid, 1 mm clear of every ring: material touching it is the outside.
    Box2 frame;
    frame.min = {onGrid(all.min.x - 1.0), onGrid(all.min.y - 1.0)};
    frame.max = {onGrid(all.max.x + 1.0), onGrid(all.max.y + 1.0)};
    const std::int64_t fx0 = gridUnits(frame.min.x);
    const std::int64_t fx1 = gridUnits(frame.max.x);
    const std::int64_t fy0 = gridUnits(frame.min.y);
    const std::int64_t fy1 = gridUnits(frame.max.y);
    // The booleans drop input vertices closer than the distance tolerance, which shifts edges by
    // up to that much next to the cuts (where clipping adds vertices) and changes areas: they get
    // the grid unit instead (only exact duplicates go).
    geometry::Tolerance booleanTol = opt.tol;
    booleanTol.distance = 1.0 / geometry::kFixedPointScale;
    std::vector<Part> parts;
    std::vector<std::size_t> partLeaf;
    std::vector<std::vector<Contour>> leafOut;
    std::vector<Contact> contacts;
    // Each leaf as the subdivision reaches it: its parts, their islands and where they meet its sides.
    auto processLeaf = [&](const Box2& rc, std::vector<Ring> leafRings, int winding) {
        const std::size_t li = leafOut.size();
        leafOut.emplace_back();
        std::vector<Contour> clip;
        clip.reserve(leafRings.size() + static_cast<std::size_t>(std::abs(winding)));
        for (const Ring& r : leafRings) clip.push_back(polygon(r.pts));
        // The rings taken out on the way down, as copies of the rectangle (grown, so that their
        // edges stay off its sides) wound the same way.
        const Box2 g = rc.inflated(kCropMargin);
        for (int k = 0; k < std::abs(winding); ++k) {
            std::vector<Vec2> sq{g.min, {g.max.x, g.min.y}, g.max, {g.min.x, g.max.y}};
            if (winding < 0) std::reverse(sq.begin(), sq.end());
            clip.push_back(polygon(sq));
        }
        leafRings.clear();
        leafRings.shrink_to_fit();
        const Contour rectC = polygon({rc.min, {rc.max.x, rc.min.y}, rc.max, {rc.min.x, rc.max.y}});
        std::vector<Contour>& out = leafOut[li];
        out = clip.empty() ? std::vector<Contour>{rectC}
                           : geometry::booleanOp(geometry::BooleanOp::Difference, {rectC}, clip, booleanTol,
                                                 geometry::FillRule::NonZero)
                                 .closed;
        const std::int64_t x0 = gridUnits(rc.min.x);
        const std::int64_t x1 = gridUnits(rc.max.x);
        const std::int64_t y0 = gridUnits(rc.min.y);
        const std::int64_t y1 = gridUnits(rc.max.y);
        // Where a ring crosses a side, Clipper2's vertex can lie a unit off it (it truncates):
        // every vertex within a unit of a side goes onto it (no vertex of the rings lies that near,
        // kVertexClear), so the parts' edges along the sides are exact and both leaves see the
        // same contacts.
        auto snap = [](double& v, std::int64_t a, double av, std::int64_t b, double bv) {
            const std::int64_t u = gridUnits(v);
            if (std::abs(u - a) <= 1) {
                v = av;
            } else if (std::abs(u - b) <= 1) {
                v = bv;
            }
        };
        for (Contour& c : out) {
            std::vector<Vec2> pts;
            pts.reserve(c.spans.size());
            for (const geometry::Span& sp : c.spans) {
                Vec2 p = geometry::startPoint(sp);
                snap(p.x, x0, rc.min.x, x1, rc.max.x);
                snap(p.y, y0, rc.min.y, y1, rc.max.y);
                if (pts.empty() || pts.back() != p) pts.push_back(p);
            }
            while (pts.size() > 1 && pts.front() == pts.back()) pts.pop_back();
            c = pts.size() >= 3 ? polygon(pts) : Contour{};
        }
        out.erase(std::remove_if(out.begin(), out.end(), [](const Contour& c) { return c.spans.empty(); }), out.end());
        // Outer boundaries (counter-clockwise) are parts; clockwise ones are islands of the
        // smallest part that holds them.
        const std::size_t first = parts.size();
        std::vector<double> sa(out.size());
        std::vector<Box2> bx(out.size());
        for (std::size_t k = 0; k < out.size(); ++k) {
            sa[k] = geometry::signedArea(out[k]);
            bx[k] = out[k].bounds();
            if (sa[k] > 0.0) {
                Part p;
                p.outer = k;
                p.area = sa[k];
                p.box = bx[k];
                parts.push_back(p);
                partLeaf.push_back(li);
            }
        }
        for (std::size_t k = 0; k < out.size(); ++k) {
            if (sa[k] >= 0.0) continue;
            std::size_t best = parts.size();
            const Vec2 q = geometry::startPoint(out[k].spans.front());
            for (std::size_t j = first; j < parts.size(); ++j) {
                if (!parts[j].box.contains(bx[k])) continue;
                if (best < parts.size() && sa[parts[j].outer] >= sa[parts[best].outer]) continue;
                if (geometry::locatePoint(out[parts[j].outer], q, geometry::FillRule::NonZero, geometry::Tolerance{}) !=
                    geometry::PointLocation::Outside) {
                    best = j;
                }
            }
            if (best == parts.size()) continue;  // not inside any part (cannot happen for a union's output)
            parts[best].islands.push_back(k);
            parts[best].area += sa[k];
        }
        // Edges along the leaf's sides.
        for (std::size_t j = first; j < parts.size(); ++j) {
            const Contour& c = out[parts[j].outer];
            for (const geometry::Span& sp : c.spans) {
                const Vec2 u = geometry::startPoint(sp);
                const Vec2 v = geometry::endPoint(sp);
                const std::int64_t ux = gridUnits(u.x);
                const std::int64_t uy = gridUnits(u.y);
                const std::int64_t vx = gridUnits(v.x);
                const std::int64_t vy = gridUnits(v.y);
                auto add = [&](int axis, std::int64_t coord, std::int64_t a, std::int64_t b, int side, bool outer) {
                    if (a == b) return;
                    if (outer) {
                        parts[j].frame = true;
                        return;
                    }
                    contacts.push_back({axis, coord, std::min(a, b), std::max(a, b), side, j});
                };
                if (ux == vx && ux == x0) add(0, x0, uy, vy, +1, x0 == fx0);
                if (ux == vx && ux == x1) add(0, x1, uy, vy, -1, x1 == fx1);
                if (uy == vy && uy == y0) add(1, y0, ux, vx, +1, y0 == fy0);
                if (uy == vy && uy == y1) add(1, y1, ux, vx, -1, y1 == fy1);
            }
        }
    };
    // Leaves stay at least 10 um (10 distance tolerances, when larger) wide: a cluster of vertices
    // closer than that is not split ever finer (a guard on depth and cost; the booleans keep all
    // but exact duplicates, so narrower leaves would be exact too).
    const double minSide = std::max(0.01, 10.0 * opt.tol.distance);
    subdivide(frame, std::move(input), 0, std::max<std::size_t>(opt.leafVertices, 1), minSide, 0, processLeaf);

    // Parts that meet across a leaf side over at least kMinOverlap are one piece. On one line the
    // parts on the same side never overlap (they lie in disjoint leaves, or are disjoint parts of
    // one leaf), so a merge of the two sides, each sorted, finds every overlap.
    UnionFind uf;
    uf.parent.resize(parts.size());
    std::iota(uf.parent.begin(), uf.parent.end(), std::size_t{0});
    std::sort(contacts.begin(), contacts.end(), [](const Contact& a, const Contact& b) {
        return std::tie(a.axis, a.coord, a.side, a.lo, a.hi, a.part) < std::tie(b.axis, b.coord, b.side, b.lo, b.hi, b.part);
    });
    for (std::size_t s = 0; s < contacts.size();) {
        std::size_t e = s;
        while (e < contacts.size() && contacts[e].axis == contacts[s].axis && contacts[e].coord == contacts[s].coord) ++e;
        std::size_t m = s;
        while (m < e && contacts[m].side < 0) ++m;
        // [s, m): side -1; [m, e): side +1, each sorted by lo.
        std::size_t i = s;
        std::size_t j = m;
        while (i < m && j < e) {
            const Contact& a = contacts[i];
            const Contact& b = contacts[j];
            if (std::min(a.hi, b.hi) - std::max(a.lo, b.lo) >= kMinOverlap) uf.join(a.part, b.part);
            if (a.hi < b.hi) {
                ++i;
            } else {
                ++j;
            }
        }
        s = e;
    }

    // The pieces: classes of parts none of which touches the frame.
    std::vector<std::vector<std::size_t>> classOf(parts.size());
    std::vector<bool> outside(parts.size(), false);
    for (std::size_t j = 0; j < parts.size(); ++j) {
        const std::size_t r = uf.find(j);
        classOf[r].push_back(j);
        if (parts[j].frame) outside[r] = true;
    }
    auto region = [&](std::size_t j) {
        const Part& p = parts[j];
        const std::vector<Contour>& out = leafOut[partLeaf[j]];
        std::vector<Contour> reg{out[p.outer]};
        for (std::size_t k : p.islands) reg.push_back(out[k]);
        return reg;
    };
    // The erosion's round joins are chords at most a fifth of the clearance inside their arcs, so
    // it overstates the thickness by no more than that.
    geometry::Tolerance erodeTol = opt.tol;
    if (opt.clearance > 0.0) erodeTol.chordal = std::max(std::min(opt.tol.chordal, 0.2 * opt.clearance), 1e-4);
    erodeTol.distance = std::min(erodeTol.distance, erodeTol.chordal);
    auto eroded = [&](const std::vector<Contour>& reg, double clearance) -> std::optional<Vec2> {
        if (reg.empty()) return std::nullopt;
        // First a cheap sufficient test: the centre of the outer boundary's box or its centroid,
        // when it lies inside with every edge at least `clearance` away (most pieces are round
        // enough). Otherwise the erosion itself.
        for (const Vec2 q : {reg.front().bounds().centre(), centroid(reg.front())}) {
            if (clearOf(reg, q, clearance)) return q;
        }
        const std::vector<Contour> left =
            geometry::offsetContours(reg, -clearance, geometry::JoinType::Round, erodeTol, opt.mitreLimit);
        for (const Contour& c : left) {
            if (!c.isEmpty()) return geometry::startPoint(c.spans.front());
        }
        return std::nullopt;
    };
    std::vector<EnclosedPiece> pieces;
    for (std::size_t r = 0; r < parts.size(); ++r) {
        if (classOf[r].empty() || outside[r]) continue;
        EnclosedPiece piece;
        bool firstVertex = true;
        for (std::size_t j : classOf[r]) {
            piece.area += parts[j].area;
            piece.bounds.expand(parts[j].box);
            for (const geometry::Span& sp : leafOut[partLeaf[j]][parts[j].outer].spans) {
                const Vec2 p = geometry::startPoint(sp);
                if (firstVertex || p.x < piece.anchor.x || (p.x == piece.anchor.x && p.y < piece.anchor.y)) {
                    piece.anchor = p;
                    firstVertex = false;
                }
            }
        }
        if (piece.area < opt.minArea) continue;
        for (std::size_t j : classOf[r]) {
            piece.inner = eroded(region(j), opt.clearance);
            if (piece.inner) break;
        }
        if (!piece.inner && classOf[r].size() > 1) {
            // The parts together. Converting them for Clipper2 drops vertices closer than the
            // distance tolerance, which can open a crack along a leaf side between two parts, so
            // they are grown by twice that first (closing such cracks) and eroded that much more.
            std::vector<Contour> whole;
            for (std::size_t j : classOf[r]) {
                for (Contour& c : region(j)) whole.push_back(std::move(c));
            }
            const double grow = 2.0 * opt.tol.distance;
            piece.inner = eroded(
                geometry::offsetContours(whole, grow, geometry::JoinType::Round, erodeTol, opt.mitreLimit),
                opt.clearance + grow);
        }
        pieces.push_back(std::move(piece));
    }
    return pieces;
}

std::vector<PointRing> bandOutline(const Contour& path, double w, double chordal, bool closedPath) {
    struct Piece {
        Vec2 a, b;        // start and end points
        Vec2 ta, tb;      // unit tangents there
        bool arc{false};
        double len{0.0};  // a line's length
        Vec2 c;           // arc centre
        double rho{0.0};  // arc radius
        double a0{0.0};   // start angle
        double sweep{0.0};
    };
    std::vector<Piece> ps;
    for (const geometry::Span& sp : path.spans) {
        if (const auto* l = std::get_if<geometry::LineSpan>(&sp)) {
            const Vec2 d = l->end - l->start;
            const double len = d.length();
            if (!(len > 1e-12)) continue;
            Piece q;
            q.a = l->start;
            q.b = l->end;
            q.ta = d / len;
            q.tb = q.ta;
            q.len = len;
            ps.push_back(q);
        } else if (const auto* a = std::get_if<geometry::ArcSpan>(&sp)) {
            const double sweep = a->sweepAngle();
            if (!(a->radius * std::fabs(sweep) > 1e-12)) continue;
            Piece q;
            q.a = a->startPoint();
            q.b = a->endPoint();
            q.ta = a->tangentAt(0.0);
            q.tb = a->tangentAt(1.0);
            q.arc = true;
            q.c = a->centre;
            q.rho = a->radius;
            q.a0 = a->startAngle;
            q.sweep = sweep;
            ps.push_back(q);
        } else {
            return {};
        }
    }
    if (ps.empty()) return {};
    std::vector<Vec2> pts;
    // Points of the arc about q of radius r from angle t0 through the signed sweep (both ends
    // exactly). On the band's convex side (`outer`) the polygon runs along tangents outside the
    // arc, at most `chordal` beyond it, so the band polygon covers the true band (no channel
    // opens between two bands where their chords would sit inside the arcs, round 3); on its
    // concave side chords at most `chordal` inside it do the same.
    auto arcPoints = [&](Vec2 q, double r, double t0, double sweep, bool outer) {
        std::size_t n = 1;
        if (r > chordal) {
            const double step = outer ? 2.0 * std::acos(r / (r + chordal)) : 2.0 * std::acos(1.0 - chordal / r);
            n = static_cast<std::size_t>(std::ceil(std::fabs(sweep) / step));
            n = std::clamp(n, std::size_t{1}, std::size_t{100000});
        } else if (outer) {
            n = static_cast<std::size_t>(std::ceil(std::fabs(sweep) / (0.25 * std::numbers::pi)));
        }
        // The ends exactly, the points between by repeated rotation (one sine and cosine per arc):
        // tangent points at the middles of n equal steps (radius r / cos(half step)), or chord ends.
        pts.push_back(q + Vec2::fromAngle(t0, r));
        const double da = sweep / static_cast<double>(n);
        const double c = std::cos(da);
        const double sn = std::sin(da);
        if (outer) {
            Vec2 v = Vec2::fromAngle(t0 + 0.5 * da, r / std::cos(0.5 * da));
            for (std::size_t k = 0; k < n; ++k) {
                pts.push_back(q + v);
                v = Vec2{v.x * c - v.y * sn, v.x * sn + v.y * c};
            }
        } else {
            Vec2 v = Vec2::fromAngle(t0, r);
            for (std::size_t k = 1; k < n; ++k) {
                v = Vec2{v.x * c - v.y * sn, v.x * sn + v.y * c};
                pts.push_back(q + v);
            }
        }
        pts.push_back(q + Vec2::fromAngle(t0 + sweep, r));
    };
    auto angleOf = [](Vec2 v) { return std::atan2(v.y, v.x); };
    // One side of the path, forward: side = -1 the right, +1 the left.
    auto chain = [&](double side) {
        for (std::size_t i = 0; i < ps.size(); ++i) {
            const Piece& q = ps[i];
            if (!q.arc) {
                pts.push_back(q.a + q.ta.perp() * (side * w));
                pts.push_back(q.b + q.tb.perp() * (side * w));
            } else {
                // The centre lies on the left of a CCW arc: there this side's offset is rho - w.
                const bool inner = (q.sweep > 0.0) == (side > 0.0);
                const double r = inner ? q.rho - w : q.rho + w;
                if (r > 0.0) {
                    arcPoints(q.c, r, q.a0, q.sweep, !inner);
                } else {
                    pts.push_back(q.c + Vec2::fromAngle(q.a0, r));
                    pts.push_back(q.c);
                    pts.push_back(q.c + Vec2::fromAngle(q.a0 + q.sweep, r));
                }
            }
            const bool closing = i + 1 == ps.size();
            if (closing && !closedPath) break;
            // The corner to the next piece: a wedge on its outer side, the vertex on the inner.
            const Piece& next = ps[closing ? 0 : i + 1];
            const Vec2 tin = q.tb;
            const Vec2 tout = next.ta;
            const double turn = std::atan2(tin.cross(tout), tin.dot(tout));
            if (std::fabs(turn) <= 1e-12) continue;
            if ((turn > 0.0) == (side > 0.0)) {
                // Leaving the vertex out takes the triangle it spans with the two offset ends off
                // the winding; where both neighbours are lines whose pieces cover that triangle
                // (they reach w sin(turn) back from the vertex) it stays covered by the other, so
                // a polygonised curve needs no spike at each vertex. (Not at the start of a closed
                // path, where the chains are joined through it.)
                const bool covered = !closing && !q.arc && !next.arc && std::fabs(turn) < 0.5 * std::numbers::pi &&
                                     std::min(q.len, next.len) >= w * std::sin(std::fabs(turn));
                if (!covered) pts.push_back(q.b);
            } else {
                arcPoints(q.b, w, angleOf(tin.perp() * side), turn, true);
            }
        }
    };
    // The ring of `pts` from `from` on, without repeated points; nothing when degenerate.
    std::vector<std::vector<Vec2>> rings;
    auto ring = [&](std::size_t from) {
        std::vector<Vec2> out;
        out.reserve(pts.size() - from);
        for (std::size_t k = from; k < pts.size(); ++k) {
            if (out.empty() || out.back() != pts[k]) out.push_back(pts[k]);
        }
        while (out.size() > 1 && out.front() == out.back()) out.pop_back();
        if (out.size() >= 3) rings.push_back(std::move(out));
    };
    chain(-1.0);
    if (closedPath) {
        ring(0);
        pts.clear();
        chain(1.0);
        std::reverse(pts.begin(), pts.end());
        ring(0);
        return rings;
    }
    const Piece& last = ps.back();
    arcPoints(last.b, w, angleOf(-last.tb.perp()), std::numbers::pi, true);
    const std::size_t leftFrom = pts.size();
    chain(1.0);
    std::reverse(pts.begin() + static_cast<std::ptrdiff_t>(leftFrom), pts.end());
    const Piece& first = ps.front();
    arcPoints(first.a, w, angleOf(first.ta.perp()), std::numbers::pi, true);
    ring(0);
    return rings;
}

namespace {

// One convex piece of a path's band (pathsToSearch): the convex hull of a line or of a stretch of
// an arc (its chord a-b, the arc at most `sag` beyond it), grown by the reach.
struct Element {
    Vec2 a, b;
    Vec2 apex;  // where the arc's tangents at a and b meet (a line: its middle)
    double s0{0.0}, s1{0.0};  // arc length along the path at its ends
    double sag{0.0};
    Vec2 centre;     // an arc's
    double rho{0.0};  // an arc's radius (0: a line)
    double da{0.0};   // an arc's signed sweep
    std::uint32_t path{0};
};

// The closest points of segments [p1, q1] and [p2, q2] (c1 on the first, c2 on the second).
void closestPoints(Vec2 p1, Vec2 q1, Vec2 p2, Vec2 q2, Vec2& c1, Vec2& c2) {
    const Vec2 d1 = q1 - p1;
    const Vec2 d2 = q2 - p2;
    const Vec2 r = p1 - p2;
    const double a = d1.dot(d1);
    const double e = d2.dot(d2);
    const double f = d2.dot(r);
    double s = 0.0;
    double t = 0.0;
    if (a <= 0.0 && e <= 0.0) {
        c1 = p1;
        c2 = p2;
        return;
    }
    if (a <= 0.0) {
        t = std::clamp(f / e, 0.0, 1.0);
    } else {
        const double c = d1.dot(r);
        if (e <= 0.0) {
            s = std::clamp(-c / a, 0.0, 1.0);
        } else {
            const double b = d1.dot(d2);
            const double den = a * e - b * b;
            s = den > 0.0 ? std::clamp((b * f - c * e) / den, 0.0, 1.0) : 0.0;
            t = (b * s + f) / e;
            if (t < 0.0) {
                t = 0.0;
                s = std::clamp(-c / a, 0.0, 1.0);
            } else if (t > 1.0) {
                t = 1.0;
                s = std::clamp((b - c) / a, 0.0, 1.0);
            }
        }
    }
    c1 = p1 + d1 * s;
    c2 = p2 + d2 * t;
}

double pointSegment(Vec2 q, Vec2 a, Vec2 b) {
    const Vec2 d = b - a;
    const double len2 = d.dot(d);
    const double t = len2 > 0.0 ? std::clamp((q - a).dot(d) / len2, 0.0, 1.0) : 0.0;
    return (a + d * t - q).length();
}

// True when q lies in the triangle (a, b, c) of either orientation (closed).
bool inTriangle(Vec2 q, Vec2 a, Vec2 b, Vec2 c) {
    const double d1 = (b - a).cross(q - a);
    const double d2 = (c - b).cross(q - b);
    const double d3 = (a - c).cross(q - c);
    const bool neg = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
    const bool pos = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
    return !(neg && pos);
}

// An arc element in k equal stretches: their ends (on the arc, so inside its hull) and the
// tangent apexes between them pushed out by `pad`, as elements; a line as it is.
void refine(const Element& e, int k, double pad, std::vector<Element>& out) {
    out.clear();
    if (e.rho <= 0.0) {
        out.push_back(e);
        return;
    }
    const double step = e.da / static_cast<double>(k);
    const double c = std::cos(step);
    const double sn = std::sin(step);
    const double ch = std::cos(0.5 * step);
    Vec2 v = e.a - e.centre;
    Vec2 p = e.a;
    for (int i = 1; i <= k; ++i) {
        v = Vec2{v.x * c - v.y * sn, v.x * sn + v.y * c};
        const Vec2 q = i == k ? e.b : e.centre + v;
        Element sub = e;
        sub.a = p;
        sub.b = q;
        const Vec2 mid = (p + q) * 0.5 - e.centre;
        sub.apex = e.centre + mid / (ch * ch) + mid * (pad / std::max(mid.length(), 1e-300));
        out.push_back(sub);
        p = q;
    }
}

// The distance between the tangent triangles (a, apex, b) of two elements, which hold the convex
// hulls of their stretches (a line's is the line itself).
double hullDistance(const Element& e, const Element& f) {
    const Vec2 pe[3] = {e.a, e.apex, e.b};
    const Vec2 pf[3] = {f.a, f.apex, f.b};
    const int ne = e.sag > 0.0 ? 3 : 2;
    const int nf = f.sag > 0.0 ? 3 : 2;
    if (ne == 3) {
        for (const Vec2 q : pf) {
            if (inTriangle(q, e.a, e.apex, e.b)) return 0.0;
        }
    }
    if (nf == 3) {
        for (const Vec2 q : pe) {
            if (inTriangle(q, f.a, f.apex, f.b)) return 0.0;
        }
    }
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < ne; ++i) {
        const Vec2 p1 = ne == 2 ? e.a : pe[i];
        const Vec2 q1 = ne == 2 ? e.b : pe[(i + 1) % 3];
        for (int j = 0; j < nf; ++j) {
            const Vec2 p2 = nf == 2 ? f.a : pf[j];
            const Vec2 q2 = nf == 2 ? f.b : pf[(j + 1) % 3];
            Vec2 c1;
            Vec2 c2;
            closestPoints(p1, q1, p2, q2, c1, c2);
            best = std::min(best, (c2 - c1).length());
            if (nf == 2) break;
        }
        if (ne == 2) break;
    }
    return best;
}

}  // namespace

std::vector<char> pathsToSearch(const std::vector<Contour>& paths, const std::vector<char>& closedPath, double reach,
                                double sagitta) {
    const std::size_t n = paths.size();
    std::vector<char> search(n, 0);
    std::vector<Box2> pathBox(n);
    std::vector<Element> el;
    std::vector<std::size_t> first(n + 1, 0);
    // Rounding: distances are compared with this much to spare (well above the rounding of
    // coordinates up to 1e9 mm), always towards searching.
    double scale = 0.0;
    std::size_t spans = 0;
    for (const Contour& c : paths) {
        for (const geometry::Span& sp : c.spans) {
            const auto* a = std::get_if<geometry::ArcSpan>(&sp);
            const Vec2 p = a ? a->centre : geometry::startPoint(sp);
            const double r = a ? a->radius : 0.0;
            scale = std::max({scale, std::fabs(p.x) + r, std::fabs(p.y) + r});
        }
        spans += c.spans.size();
    }
    el.reserve(2 * spans);
    const double eps = 1e-12 * (1.0 + scale);
    const double sagCap = std::max(sagitta, 1e-9);
    const double chunk = std::max(4.0 * reach, 0.5);
    for (std::size_t i = 0; i < n; ++i) {
        first[i] = el.size();
        const Contour& path = paths[i];
        bool ok = !closedPath[i] && !path.spans.empty();
        double s = 0.0;
        bool started = false;
        Vec2 last{};
        auto push = [&](Vec2 a, Vec2 b, double len, double sag) {
            Element& e = el.emplace_back();
            e.a = a;
            e.b = b;
            e.apex = (a + b) * 0.5;
            e.s0 = s;
            s += len;
            e.s1 = s;
            e.sag = sag;
            e.path = static_cast<std::uint32_t>(i);
        };
        auto room = [&](std::size_t k) {
            if (el.capacity() < el.size() + k) el.reserve(2 * (el.size() + k));
        };
        for (const geometry::Span& sp : path.spans) {
            if (!ok) break;
            const auto* l = std::get_if<geometry::LineSpan>(&sp);
            const auto* arc = std::get_if<geometry::ArcSpan>(&sp);
            if (!l && !arc) {
                ok = false;  // Bezier
                break;
            }
            const Vec2 a = l ? l->start : arc->startPoint();
            const Vec2 z = l ? l->end : arc->endPoint();
            // A gap between spans (within the tolerance) is a line of its own.
            if (started && a != last) push(last, a, (a - last).length(), 0.0);
            last = z;
            started = true;
            if (l) {
                // Long lines in pieces about two grid cells long, so that each meets a few cells.
                const double len = (l->end - l->start).length();
                const double m = std::ceil(len / chunk);
                const std::size_t k = m > 1.0 && m <= 1e6 ? static_cast<std::size_t>(m) : 1;
                room(k + 1);
                Vec2 p = l->start;
                for (std::size_t j = 1; j <= k; ++j) {
                    const Vec2 q = j == k ? l->end : l->start + (l->end - l->start) * (static_cast<double>(j) / static_cast<double>(k));
                    push(p, q, len / static_cast<double>(k), 0.0);
                    p = q;
                }
            } else {
                const double sweep = arc->sweepAngle();
                const double rho = arc->radius;
                // Stretches of at most a quarter turn whose sagitta is at most sagCap.
                double step = 0.5 * std::numbers::pi;
                // (rho (1 - cos(da / 2)) <= rho da^2 / 8 <= sagCap.)
                if (rho > sagCap) step = std::min(step, std::sqrt(8.0 * sagCap / rho));
                const double m = std::ceil(std::fabs(sweep) / step);
                if (!(m >= 1.0 && m <= 1e6)) {
                    ok = false;
                    break;
                }
                const auto k = static_cast<std::size_t>(m);
                room(k + 1);
                const double da = sweep / static_cast<double>(k);
                const double cosHalf = std::cos(0.5 * da);
                const double sinHalf = std::sin(0.5 * da);
                const double sag = rho * (1.0 - cosHalf);
                const double len = rho * std::fabs(da);
                const double c = cosHalf * cosHalf - sinHalf * sinHalf;
                const double sn = 2.0 * sinHalf * cosHalf;
                // mid / cos^2(da / 2) has length rho / cos(da / 2); eps more.
                const double midLength = rho * cosHalf;
                const double apexScale = 1.0 / (cosHalf * cosHalf) + (midLength > 0.0 ? eps / midLength : 0.0);
                Vec2 v = a - arc->centre;
                Vec2 p = a;
                for (std::size_t j = 1; j <= k; ++j) {
                    Vec2 q;
                    if (j == k) {
                        q = z;
                    } else {
                        v = Vec2{v.x * c - v.y * sn, v.x * sn + v.y * c};
                        q = arc->centre + v;
                    }
                    // (The rotated point drifts off the circle by rounding only, far below eps.)
                    push(p, q, len, sag + eps);
                    // The tangents meet on the bisector at rho / cos(da / 2) from the centre (a
                    // quarter turn at most, so not far), pushed out by the rounding allowance.
                    const Vec2 mid = (p + q) * 0.5 - arc->centre;
                    el.back().apex = arc->centre + mid * apexScale;
                    el.back().centre = arc->centre;
                    el.back().rho = rho;
                    el.back().da = da;
                    p = q;
                }
            }
        }
        if (!ok || el.size() == first[i]) {
            el.resize(first[i]);
            search[i] = 1;
            Box2 b = path.bounds();
            pathBox[i] = b.isEmpty() ? b : b.inflated(reach + sagCap + 1e-3);
            continue;
        }
        Box2 b;
        for (std::size_t k = first[i]; k < el.size(); ++k) {
            b.expand(el[k].a);
            b.expand(el[k].b);
        }
        pathBox[i] = b.inflated(reach + sagCap + eps);
    }
    first[n] = el.size();

    // Each piece's box, and a disc round its chord's middle outside which no point of the piece
    // lies (each pair's discs are tested before their distance: half a chord, the sagitta and the
    // reach, so two pieces whose discs do not meet are apart).
    struct Disc {
        Vec2 c;
        double r;
    };
    std::vector<Box2> elBox(el.size());
    std::vector<Disc> elDisc(el.size());
    for (std::size_t k = 0; k < el.size(); ++k) {
        elBox[k].expand(el[k].a);
        elBox[k].expand(el[k].b);
        elBox[k] = elBox[k].inflated(reach + el[k].sag + eps);
        elDisc[k] = {(el[k].a + el[k].b) * 0.5, 0.5 * (el[k].b - el[k].a).length() + el[k].sag + reach + 2.0 * eps};
    }

    // Paths whose boxes overlap form groups (union-find over a sweep in x); pieces are compared
    // within a group only.
    UnionFind groups;
    groups.parent.resize(n);
    std::iota(groups.parent.begin(), groups.parent.end(), std::size_t{0});
    {
        std::vector<std::size_t> order;
        for (std::size_t i = 0; i < n; ++i) {
            if (!pathBox[i].isEmpty()) order.push_back(i);
        }
        std::sort(order.begin(), order.end(), [&](std::size_t x, std::size_t y) {
            return std::tie(pathBox[x].min.x, x) < std::tie(pathBox[y].min.x, y);
        });
        std::vector<std::size_t> active;
        for (std::size_t i : order) {
            std::size_t kept = 0;
            for (std::size_t a : active) {
                if (pathBox[a].max.x < pathBox[i].min.x) continue;
                active[kept++] = a;
                if (pathBox[a].min.y <= pathBox[i].max.y && pathBox[i].min.y <= pathBox[a].max.y) groups.join(a, i);
            }
            active.resize(kept);
            active.push_back(i);
        }
    }
    // The groups' paths, group by group (by root, then index).
    std::vector<std::size_t> groupStart(n + 1, 0);
    for (std::size_t i = 0; i < n; ++i) {
        if (!pathBox[i].isEmpty()) ++groupStart[groups.find(i) + 1];
    }
    for (std::size_t g = 0; g < n; ++g) groupStart[g + 1] += groupStart[g];
    std::vector<std::size_t> grouped(groupStart[n]);
    {
        std::vector<std::size_t> at(groupStart.begin(), groupStart.end() - 1);
        for (std::size_t i = 0; i < n; ++i) {
            if (!pathBox[i].isEmpty()) grouped[at[groups.find(i)]++] = i;
        }
    }
    // Paths whose pieces meet pieces of another path (contacts): one cluster, searched.
    UnionFind contact;
    contact.parent.resize(n);
    std::iota(contact.parent.begin(), contact.parent.end(), std::size_t{0});
    const double local = 2.0 * reach;
    constexpr int kRefine = 8;
    constexpr std::size_t kOneCell = 64;
    std::vector<Element> subE;
    std::vector<Element> subF;
    std::vector<std::uint32_t> cellStart;
    std::vector<std::uint32_t> cellItems;
    std::vector<Box2> cellBox;  // the boxes of cellItems, in the same order
    std::vector<std::uint64_t> tableKey;
    std::vector<std::uint32_t> tableId;
    std::vector<std::uint64_t> cellKey;
    std::vector<std::uint32_t> insertCell;
    std::vector<std::uint32_t> insertItem;
    for (std::size_t g = 0; g < n; ++g) {
        if (groupStart[g + 1] == groupStart[g]) continue;
        const std::span<const std::size_t> mem(grouped.data() + groupStart[g], groupStart[g + 1] - groupStart[g]);
        std::size_t count = 0;
        for (std::size_t i : mem) count += first[i + 1] - first[i];
        if (count < 2) continue;
        if (mem.size() == 1 && search[mem.front()]) continue;
        // A grid of cells about two reaches wide, kept in a hash table (only the cells some piece
        // meets: a long outline's box is mostly empty), each piece in every cell its box meets; a
        // pair is tested in the cell holding the lower left corner of the overlap of their boxes
        // only. Each cell lists its pieces in path order.
        Box2 gb;
        for (std::size_t i : mem) gb.expand(pathBox[i]);
        const double grow = reach + sagCap + eps;
        double cell = std::max(2.0 * grow, 0.5 * chunk);
        cell = std::max(cell, std::max(gb.width(), gb.height()) * 1e-9);  // cell coordinates below 2^31
        const double perCell = 1.0 / cell;
        auto cx = [&](double x) { return static_cast<std::uint64_t>(std::max(0.0, (x - gb.min.x) * perCell)); };
        auto cy = [&](double y) { return static_cast<std::uint64_t>(std::max(0.0, (y - gb.min.y) * perCell)); };
        auto keyOf = [](std::uint64_t x, std::uint64_t y) { return (x << 32) | y; };
        // A small group is one cell (every pair compared directly).
        const bool oneCell = count <= kOneCell;
        if (oneCell) {
            cellKey.assign(1, 0);
            cellStart.assign({0, 0});
            cellItems.clear();
            cellBox.clear();
            for (std::size_t i : mem) {
                for (std::size_t k = first[i]; k < first[i + 1]; ++k) {
                    cellItems.push_back(static_cast<std::uint32_t>(k));
                    cellBox.push_back(elBox[k]);
                }
            }
            cellStart[1] = static_cast<std::uint32_t>(cellItems.size());
        } else {
            std::size_t inserts = 0;
            for (std::size_t i : mem) {
                for (std::size_t k = first[i]; k < first[i + 1]; ++k) {
                    const Box2& b = elBox[k];
                    inserts += static_cast<std::size_t>((cx(b.max.x) - cx(b.min.x) + 1) * (cy(b.max.y) - cy(b.min.y) + 1));
                }
            }
            std::size_t tableSize = 16;
            while (tableSize < 2 * inserts) tableSize *= 2;
            constexpr std::uint64_t kEmpty = ~std::uint64_t{0};
            tableKey.assign(tableSize, kEmpty);
            tableId.resize(tableSize);
            cellKey.clear();
            insertCell.clear();
            insertItem.clear();
            // Consecutive pieces along a path meet mostly the same cells: a small cache first.
            std::uint64_t recentKey[64];
            std::uint32_t recentId[64] = {};
            std::fill(std::begin(recentKey), std::end(recentKey), kEmpty);
            for (std::size_t i : mem) {
                for (std::size_t k = first[i]; k < first[i + 1]; ++k) {
                    const Box2& b = elBox[k];
                    const std::uint64_t x0 = cx(b.min.x);
                    const std::uint64_t x1 = cx(b.max.x);
                    const std::uint64_t y1 = cy(b.max.y);
                    for (std::uint64_t y = cy(b.min.y); y <= y1; ++y) {
                        for (std::uint64_t x = x0; x <= x1; ++x) {
                            const std::uint64_t key = keyOf(x, y);
                            const std::size_t r = static_cast<std::size_t>(x * 7 + y * 13) & 63;
                            if (recentKey[r] != key) {
                                std::size_t slot = static_cast<std::size_t>((key * 0x9E3779B97F4A7C15ULL) >> 20) & (tableSize - 1);
                                while (tableKey[slot] != key && tableKey[slot] != kEmpty) slot = (slot + 1) & (tableSize - 1);
                                if (tableKey[slot] == kEmpty) {
                                    tableKey[slot] = key;
                                    tableId[slot] = static_cast<std::uint32_t>(cellKey.size());
                                    cellKey.push_back(key);
                                }
                                recentKey[r] = key;
                                recentId[r] = tableId[slot];
                            }
                            insertCell.push_back(recentId[r]);
                            insertItem.push_back(static_cast<std::uint32_t>(k));
                        }
                    }
                }
            }
            cellStart.assign(cellKey.size() + 1, 0);
            for (const std::uint32_t c : insertCell) ++cellStart[c + 1];
            for (std::size_t c = 1; c < cellStart.size(); ++c) cellStart[c] += cellStart[c - 1];
            cellItems.resize(insertItem.size());
            cellBox.resize(insertItem.size());
            {
                std::vector<std::uint32_t> fill(cellStart.begin(), cellStart.end() - 1);
                for (std::size_t t = 0; t < insertItem.size(); ++t) {
                    const std::uint32_t at = fill[insertCell[t]]++;
                    cellItems[at] = insertItem[t];
                    cellBox[at] = elBox[insertItem[t]];
                }
            }
        }
        for (std::size_t c = 0; c + 1 < cellStart.size(); ++c) {
            // A cell's pieces are in path order (by path, then along it), so the pieces of the same
            // path at most 2 reaches along it after a piece follow it in one run, whose end only
            // moves on with the piece: pieces of stretches that close along the path both hold the
            // path point a reach back from the later one's start, and are not compared.
            std::uint32_t runEnd = cellStart[c];
            for (std::uint32_t u = cellStart[c]; u < cellStart[c + 1]; ++u) {
                const Element& e = el[cellItems[u]];
                const Box2& be = elBox[cellItems[u]];
                if (runEnd <= u || el[cellItems[runEnd - 1]].path != e.path) runEnd = u + 1;
                while (runEnd < cellStart[c + 1] && el[cellItems[runEnd]].path == e.path &&
                       el[cellItems[runEnd]].s0 - e.s1 <= local - eps * (1.0 + el[cellItems[runEnd]].s0)) {
                    ++runEnd;
                }
                const double bx0 = be.min.x;
                const double by0 = be.min.y;
                const double bx1 = be.max.x;
                const double by1 = be.max.y;
                const Disc& de = elDisc[cellItems[u]];
                for (std::uint32_t v = runEnd; v < cellStart[c + 1]; ++v) {
                    const Box2& bf = cellBox[v];
                    if (bf.min.x > bx1 || bf.max.x < bx0 || bf.min.y > by1 || bf.max.y < by0) continue;
                    if (!oneCell && keyOf(cx(std::max(bx0, bf.min.x)), cy(std::max(by0, bf.min.y))) != cellKey[c]) continue;
                    // Chords farther apart than their half lengths and the pieces' reach: apart.
                    const Disc& df = elDisc[cellItems[v]];
                    const double rr = de.r + df.r;
                    if ((df.c - de.c).dot(df.c - de.c) > rr * rr) continue;
                    const Element& f = el[cellItems[v]];
                    if (e.path == f.path && search[e.path]) continue;
                    Vec2 c1;
                    Vec2 c2;
                    closestPoints(e.a, e.b, f.a, f.b, c1, c2);
                    double d = (c2 - c1).length();
                    if (d > 2.0 * reach + e.sag + f.sag + eps) continue;  // the pieces do not meet
                    const bool arcs = e.rho > 0.0 || f.rho > 0.0;
                    if (arcs && d > 2.0 * reach - eps) {
                        // Near two reaches apart: the tangent triangles (outside the hulls), then
                        // finer stretches, whose triangles bound the distance from below and whose
                        // chords (inside the hulls) give a common point when there is one.
                        if (hullDistance(e, f) > 2.0 * reach + eps) continue;
                        double lower = 0.0;
                        for (const int k : {kRefine, kRefine * kRefine}) {
                            refine(e, k, eps, subE);
                            refine(f, k, eps, subF);
                            lower = std::numeric_limits<double>::infinity();
                            for (const Element& x : subE) {
                                for (const Element& y : subF) {
                                    lower = std::min(lower, hullDistance(x, y));
                                    Vec2 p1;
                                    Vec2 p2;
                                    closestPoints(x.a, x.b, y.a, y.b, p1, p2);
                                    if ((p2 - p1).length() < d) {
                                        d = (p2 - p1).length();
                                        c1 = p1;
                                        c2 = p2;
                                    }
                                }
                            }
                            if (lower > 2.0 * reach + eps || d <= 2.0 * reach - eps) break;
                        }
                        if (lower > 2.0 * reach + eps) continue;
                    }
                    if (e.path != f.path) {
                        contact.join(e.path, f.path);
                        search[e.path] = 1;
                        search[f.path] = 1;
                        continue;
                    }
                    // A later piece meeting an earlier one far back along the path: they must meet
                    // at a point that one of the later piece's near predecessors holds too.
                    const std::size_t hiIdx = e.s0 < f.s0 ? cellItems[v] : cellItems[u];
                    bool joined = false;
                    if (d <= 2.0 * reach - eps) {
                        const Vec2 q = (c1 + c2) * 0.5;  // within a reach of both chords
                        const double from = el[hiIdx].s0 - local + eps * (1.0 + el[hiIdx].s0);
                        for (std::size_t k = hiIdx; k-- > first[e.path] && el[k].s1 >= from;) {
                            const double dk = pointSegment(q, el[k].a, el[k].b);
                            if (dk <= reach - eps) {
                                joined = true;
                                break;
                            }
                            if (el[k].rho > 0.0 && dk <= reach + el[k].sag) {
                                refine(el[k], kRefine, eps, subE);
                                for (const Element& x : subE) joined = joined || pointSegment(q, x.a, x.b) <= reach - eps;
                                if (joined) break;
                            }
                        }
                    }
                    if (!joined) search[e.path] = 1;
                }
            }
        }
    }
    // Every path whose box reaches into the box of a searched cluster is searched with it: the
    // pieces such a cluster encloses lie within its box, and so does every band around them.
    std::vector<Box2> clusterBox(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (search[i] && !pathBox[i].isEmpty()) clusterBox[contact.find(i)].expand(pathBox[i]);
    }
    std::vector<Box2> boxes;
    for (const Box2& b : clusterBox) {
        if (!b.isEmpty()) boxes.push_back(b);
    }
    if (boxes.empty()) return search;
    std::sort(boxes.begin(), boxes.end(), [](const Box2& x, const Box2& y) { return x.min.x < y.min.x; });
    // Boxes sorted by left edge; a path's box meets one of them when some box starting at or left
    // of its right edge reaches past its left edge and overlaps in y (prefix of right edges).
    std::vector<double> maxRight(boxes.size());
    for (std::size_t k = 0; k < boxes.size(); ++k) maxRight[k] = std::max(k ? maxRight[k - 1] : boxes[k].max.x, boxes[k].max.x);
    for (std::size_t i = 0; i < n; ++i) {
        if (search[i] || pathBox[i].isEmpty()) continue;
        const Box2& b = pathBox[i];
        auto end = std::upper_bound(boxes.begin(), boxes.end(), b.max.x, [](double x, const Box2& y) { return x < y.min.x; });
        for (auto it = end; it != boxes.begin();) {
            --it;
            const std::size_t k = static_cast<std::size_t>(it - boxes.begin());
            if (maxRight[k] < b.min.x) break;
            if (it->max.x >= b.min.x && it->intersects(b)) {
                search[i] = 2;
                break;
            }
        }
    }
    for (char& c : search) c = c ? 1 : 0;
    return search;
}

}  // namespace carvetto::toolpath::detail
