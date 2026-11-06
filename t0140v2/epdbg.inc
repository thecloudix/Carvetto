

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <numeric>
#include <optional>
#include <tuple>
#include <utility>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/fixed_point.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/span.hpp"

namespace epdbg { inline long g_tot = 0, g_off = 0; using namespace carvetto; using namespace carvetto::toolpath::detail;

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
// a side would be narrower than `minSide` (the booleans drop vertices closer than the distance
// tolerance, which would collapse a thinner leaf's rectangle) or would keep more than 4/5 of the
// vertices (rings crossing the cut are copied to both sides, e.g. long edges running across the
// whole rectangle), so the depth stays logarithmic.
std::optional<Split> planSplit(const Box2& rect, const std::vector<Ring>& rings, std::size_t count, int axis,
                               double minSide) {
    const double lo = axis == 0 ? rect.min.x : rect.min.y;
    const double hi = axis == 0 ? rect.max.x : rect.max.y;
    if (!(hi - lo > 4.0 * minSide)) return std::nullopt;
    // The median of every k-th vertex coordinate (at most about 2048 of them), moved half way to
    // a neighbouring coordinate on the grid so that no vertex lies on the cut (a ring edge along a
    // leaf side would meet the side of the leaf's rectangle in the boolean).
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
    // grid value strictly between theirs.
    const std::int64_t gm = gridUnits(m);
    std::int64_t c = gridUnits(0.5 * (lo + hi));
    if (std::isfinite(next) && gridUnits(next) - gm >= 2) {
        c = gm + (gridUnits(next) - gm) / 2;
    } else if (std::isfinite(prev) && gm - gridUnits(prev) >= 2) {
        c = gridUnits(prev) + (gm - gridUnits(prev)) / 2;
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
                           : geometry::booleanOp(geometry::BooleanOp::Difference, {rectC}, clip, opt.tol,
                                                 geometry::FillRule::NonZero)
                                 .closed;
        if (std::getenv("EPLEAF")) { int want1 = -1, want2 = -1; std::sscanf(std::getenv("EPLEAF"), "%d,%d", &want1, &want2);
            if ((int)li == want1 || (int)li == want2) { std::printf("LEAF %zu rect %.7f,%.7f-%.7f,%.7f clip %zu winding %d out %zu\n", li, rc.min.x, rc.min.y, rc.max.x, rc.max.y, clip.size(), winding, out.size());
                for (auto& c : out) { std::printf("   out a=%.6f:", geometry::signedArea(c)); for (auto& sp : c.spans) { auto q = geometry::startPoint(sp); std::printf(" (%.7f,%.7f)", q.x, q.y); } std::printf("\n"); }
                if (std::getenv("EPCLIP")) for (auto& c : clip) { auto b = c.bounds(); std::printf("   clip n=%zu box %.7f,%.7f-%.7f,%.7f\n", c.spans.size(), b.min.x, b.min.y, b.max.x, b.max.y); } } }
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
        const std::int64_t x0 = gridUnits(rc.min.x);
        const std::int64_t x1 = gridUnits(rc.max.x);
        const std::int64_t y0 = gridUnits(rc.min.y);
        const std::int64_t y1 = gridUnits(rc.max.y);
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
                { auto& tot = g_tot; auto& off = g_off; auto nearS = [](std::int64_t a, std::int64_t b) { return a == b; }; auto near1 = [](std::int64_t a, std::int64_t b) { return std::llabs(a - b) <= 2 && a != b; };
                  bool exact = (ux == vx && (ux == x0 || ux == x1)) || (uy == vy && (uy == y0 || uy == y1));
                  bool o = (near1(ux, x0) && nearS(vx, x0)) || (nearS(ux, x0) && near1(vx, x0)) || (near1(ux, x1) && nearS(vx, x1)) || (nearS(ux, x1) && near1(vx, x1)) ||
                           (near1(uy, y0) && nearS(vy, y0)) || (nearS(uy, y0) && near1(vy, y0)) || (near1(uy, y1) && nearS(vy, y1)) || (nearS(uy, y1) && near1(vy, y1));
                  if (exact) ++tot; if (o) { ++off; if (std::getenv("EPOFF")) std::printf("OFFEDGE (%lld,%lld)-(%lld,%lld) leaf x %lld..%lld y %lld..%lld\n", (long long)ux, (long long)uy, (long long)vx, (long long)vy, (long long)x0, (long long)x1, (long long)y0, (long long)y1); }
                }
                if (ux == vx && ux == x0) add(0, x0, uy, vy, +1, x0 == fx0);
                if (ux == vx && ux == x1) add(0, x1, uy, vy, -1, x1 == fx1);
                if (uy == vy && uy == y0) add(1, y0, ux, vx, +1, y0 == fy0);
                if (uy == vy && uy == y1) add(1, y1, ux, vx, -1, y1 == fy1);
            }
        }
    };
    // Leaves stay at least 10 distance tolerances (and 10 um) wide, so that the booleans, which
    // drop vertices closer than that tolerance, keep their rectangles.
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
    auto eroded = [&](const std::vector<Contour>& reg) -> std::optional<Vec2> {
        // First a cheap sufficient test: the centre of the outer boundary's box or its centroid,
        // when it lies inside with every edge at least `clearance` away (most pieces are round
        // enough). Otherwise the erosion itself.
        for (const Vec2 q : {reg.front().bounds().centre(), centroid(reg.front())}) {
            if (clearOf(reg, q, opt.clearance)) return q;
        }
        const std::vector<Contour> left =
            geometry::offsetContours(reg, -opt.clearance, geometry::JoinType::Round, opt.tol, opt.mitreLimit);
        for (const Contour& c : left) {
            if (!c.isEmpty()) return geometry::startPoint(c.spans.front());
        }
        return std::nullopt;
    };
    std::vector<EnclosedPiece> pieces;
    for (std::size_t r = 0; r < parts.size(); ++r) {
        if (classOf[r].empty() || outside[r]) continue;
        EnclosedPiece piece;
        for (std::size_t j : classOf[r]) {
            piece.area += parts[j].area;
            piece.bounds.expand(parts[j].box);
        }
        if (piece.area < opt.minArea) continue;
        for (std::size_t j : classOf[r]) {
            piece.inner = eroded(region(j));
            if (piece.inner) break;
        }
        if (std::getenv("EPDBG") && std::fabs(piece.area - std::atof(std::getenv("EPDBG"))) < 1e-4) {
            std::printf("DBG piece area %.6f parts %zu inner %d\n", piece.area, classOf[r].size(), (int)piece.inner.has_value());
            for (std::size_t j : classOf[r]) { auto reg = region(j); std::printf("  part %zu leaf %zu area %.6f box %.5f,%.5f-%.5f,%.5f contours %zu\n", j, partLeaf[j], parts[j].area, parts[j].box.min.x, parts[j].box.min.y, parts[j].box.max.x, parts[j].box.max.y, reg.size()); }
        }
        if (!piece.inner && classOf[r].size() > 1) {
            std::vector<Contour> whole;
            for (std::size_t j : classOf[r]) {
                for (Contour& c : region(j)) whole.push_back(std::move(c));
            }
            auto U = geometry::unionContours(whole, opt.tol, geometry::FillRule::NonZero);
            piece.inner = eroded(U);
            if (std::getenv("EPDBG") && std::fabs(piece.area - std::atof(std::getenv("EPDBG"))) < 1e-4) { std::printf("  union -> %zu contours, inner %d\n", U.size(), (int)piece.inner.has_value()); for (auto& c : U) { auto b = c.bounds(); std::printf("    u area %.6f box %.5f,%.5f-%.5f,%.5f spans %zu\n", geometry::signedArea(c), b.min.x, b.min.y, b.max.x, b.max.y, c.spans.size()); }
              if (std::getenv("EPV")) for (auto& c : U) { std::printf("    contour:"); for (auto& sp : c.spans) { auto q = geometry::startPoint(sp); std::printf(" (%.6f,%.6f)", q.x, q.y); } std::printf("\n"); }
              for (std::size_t j : classOf[r]) { if (!std::getenv("EPV")) break; auto reg = region(j); for (auto& c : reg) { std::printf("    part %zu:", j); for (auto& sp : c.spans) { auto q = geometry::startPoint(sp); std::printf(" (%.6f,%.6f)", q.x, q.y); } std::printf("\n"); } }
              auto E = geometry::offsetContours(U, -opt.clearance, geometry::JoinType::Round, opt.tol, opt.mitreLimit); std::printf("    erode -> %zu\n", E.size()); }
        }
        pieces.push_back(std::move(piece));
    }
    return pieces;
}

}  // namespace carvetto::toolpath::detail
