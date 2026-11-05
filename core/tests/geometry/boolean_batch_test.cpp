// T-0142: booleans over many far-apart shapes are clipped in separate Clipper2 calls (ADR-0006
// §10), and `OpenPathClipper` crops a big clip to each batch's strip, so a row of thousands of
// shapes and a raster over a comb of thousands of teeth cost about linear time.
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include "boolean_batch.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "offset_batch.hpp"
#include "test_rng.hpp"
// Last: on Windows it includes <windows.h>.
#include "cpu_scaling.hpp"

using namespace carvetto::geometry;
using detail::GridPoint;
using detail::GridRing;

namespace {

#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif

constexpr std::size_t kNever = std::numeric_limits<std::size_t>::max();

Contour polygon(const std::vector<Vec2>& pts) {
    Contour c;
    c.closed = true;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        c.spans.emplace_back(LineSpan{pts[i], pts[(i + 1) % pts.size()]});
    }
    return c;
}

Contour rect(Vec2 lo, Vec2 hi) {
    return polygon({lo, {hi.x, lo.y}, hi, {lo.x, hi.y}});
}

Contour segment(Vec2 a, Vec2 b) {
    Contour c;
    c.append(LineSpan{a, b});
    return c;
}

// A row of n rectangles 0.5 wide, alternately 0.3 and 0.2 high, 0.7 apart (T-0141's row).
std::vector<Contour> rectangleRow(int n, double y = 0.0) {
    std::vector<Contour> row;
    for (int i = 0; i < n; ++i) {
        const double x = 0.7 * i;
        row.push_back(rect({x, y}, {x + 0.5, y + (i % 2 != 0 ? 0.2 : 0.3)}));
    }
    return row;
}

// The T-0121 verifier's comb: `teeth` teeth at 1 mm pitch, 0.5 mm gaps 20 mm deep, on a 5 mm base.
Contour comb(int teeth) {
    std::vector<Vec2> pts;
    for (int i = 0; i < teeth; ++i) {
        pts.push_back({i * 1.0, 0});
        pts.push_back({i * 1.0, 20});
        pts.push_back({i + 0.5, 20});
        pts.push_back({i + 0.5, 0.0});
    }
    pts.push_back({double(teeth), -5});
    pts.push_back({0, -5});
    return Contour::fromPolyline(carvetto::geometry::Polyline{pts, true});
}

using Ring = std::vector<Vec2>;

std::vector<Ring> ringsOf(const std::vector<Contour>& cs) {
    std::vector<Ring> out;
    for (const Contour& c : cs) {
        Ring r;
        for (const Span& s : c.spans) {
            r.push_back(startPoint(s));
        }
        out.push_back(r);
    }
    return out;
}

double distanceToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 d = b - a;
    const double len2 = d.dot(d);
    const double t = len2 > 0.0 ? std::clamp((p - a).dot(d) / len2, 0.0, 1.0) : 0.0;
    return distance(p, a + d * t);
}

// Largest distance from a vertex of `a` to the edges of `b` (brute force).
double oneSidedDistance(const std::vector<Ring>& a, const std::vector<Ring>& b) {
    double worst = 0.0;
    for (const Ring& ra : a) {
        for (const Vec2 p : ra) {
            double best = std::numeric_limits<double>::infinity();
            for (const Ring& rb : b) {
                for (std::size_t i = 0, n = rb.size(); i < n; ++i) {
                    best = std::min(best, distanceToSegment(p, rb[i], rb[(i + 1) % n]));
                }
            }
            worst = std::max(worst, best);
        }
    }
    return worst;
}

double totalArea(const std::vector<Contour>& cs) {
    double a = 0.0;
    for (const Contour& c : cs) {
        a += area(c);
    }
    return a;
}

// Winding number of p for grid rings (crossing count with signs), in exact integer arithmetic.
int windingOf(const std::vector<GridRing>& rings, GridPoint p) {
    int w = 0;
    for (const GridRing& r : rings) {
        for (std::size_t i = 0, n = r.size(); i < n; ++i) {
            const GridPoint a = r[i];
            const GridPoint b = r[(i + 1) % n];
            const auto cross = [&] {
                return static_cast<long double>(b.x - a.x) * static_cast<long double>(p.y - a.y) -
                       static_cast<long double>(p.x - a.x) * static_cast<long double>(b.y - a.y);
            };
            if (a.y <= p.y) {
                if (b.y > p.y && cross() > 0) {
                    ++w;
                }
            } else if (b.y <= p.y && cross() < 0) {
                --w;
            }
        }
    }
    return w;
}

}  // namespace

TEST_CASE("T-0142 a: fewer than 32 groups side by side take one Clipper2 call, more are batched",
          "[geometry][boolean][batch]") {
    const Tolerance tol{0.001, 1e-9, 0.001};
    const int below = static_cast<int>(detail::kOffsetBatchMinDepth) - 1;
    const std::vector<Contour> narrow = rectangleRow(below);
    // Below the threshold: byte-identical to one call, for every operation.
    std::vector<Contour> clips;
    for (int i = 0; i < below; ++i) {
        clips.push_back(rect({0.7 * i + 0.25, 0.1}, {0.7 * i + 0.6, 0.25}));
    }
    std::vector<Contour> lines;
    for (int i = 0; i < below; ++i) {
        lines.push_back(segment({0.7 * i - 0.05, 0.15}, {0.7 * i + 0.62, 0.15}));
    }
    std::vector<Contour> subjects = narrow;
    subjects.insert(subjects.end(), lines.begin(), lines.end());
    for (const BooleanOp op : {BooleanOp::Union, BooleanOp::Difference, BooleanOp::Intersection, BooleanOp::Xor}) {
        for (const FillRule fill : {FillRule::NonZero, FillRule::EvenOdd}) {
            const BooleanResult one = detail::booleanOpBatched(op, subjects, clips, tol, fill, kNever);
            const BooleanResult r = booleanOp(op, subjects, clips, tol, fill);
            CHECK(r.closed == one.closed);
            CHECK(r.open == one.open);
        }
    }
    // At the threshold: batched, and the same as batching whenever the input splits.
    const std::vector<Contour> wide = rectangleRow(static_cast<int>(detail::kOffsetBatchMinDepth));
    const std::vector<Contour> batched = unionContours(wide, tol);
    CHECK(batched == detail::booleanOpBatched(BooleanOp::Union, wide, {}, tol, FillRule::NonZero, 0).closed);
    CHECK(batched.size() == wide.size());
    // The order differs from one call (batch by batch), the rings do not.
    const std::vector<Contour> one = detail::booleanOpBatched(BooleanOp::Union, wide, {}, tol, FillRule::NonZero, kNever).closed;
    CHECK(oneSidedDistance(ringsOf(batched), ringsOf(one)) == 0.0);
    CHECK(oneSidedDistance(ringsOf(one), ringsOf(batched)) == 0.0);
    // A column never batches: one batch per line.
    std::vector<Contour> column;
    for (int i = 0; i < 64; ++i) {
        column.push_back(rect({0.0, 0.7 * i}, {0.5, 0.7 * i + 0.3}));
    }
    CHECK(unionContours(column, tol) ==
          detail::booleanOpBatched(BooleanOp::Union, column, {}, tol, FillRule::NonZero, kNever).closed);
}

TEST_CASE("T-0142 a: batched booleans match one Clipper2 call up to grid rounding", "[geometry][boolean][batch]") {
    // Groups never touch, so every winding number is the same; Clipper2 rounds intersections per
    // scanbeam, which shapes at the same height subdivide, so vertices can move by a few grid
    // units. Ring counts, areas and open pieces stay the same.
    carvetto::test::SplitMix rng{1420};
    const double kSlack = 2e-5;  // mm, 20 grid units
    const Tolerance tol{0.001, 1e-9, 0.002};
    int batchedScenes = 0;
    for (int scene = 0; scene < 40; ++scene) {
        const double size = rng.uniform(0.3, 2.0);
        const double pitch = size * rng.uniform(3.3, 4.5);  // boxes reach 1.6 x size from a centre
        const int nx = 32 + static_cast<int>(rng.uniform(0.0, 30.0));
        const int ny = 1 + static_cast<int>(rng.uniform(0.0, 3.0));
        std::vector<Contour> subject;
        std::vector<Contour> clip;
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < ny; ++j) {
                const Vec2 c{pitch * i + rng.uniform(-0.1, 0.1) * size, pitch * j * 1.3 + rng.uniform(-0.1, 0.1) * size};
                const int kind = static_cast<int>(rng.uniform(0.0, 4.0));
                Contour s = kind == 0   ? rect(c - Vec2{size, 0.6 * size}, c + Vec2{size, 0.6 * size})
                            : kind == 1 ? Contour::circle(c, size)
                            : kind == 2 ? polygon({c + Vec2{-size, -size}, c + Vec2{size, -0.9 * size}, c + Vec2{-0.7 * size, size}})
                                        : rect(c - Vec2{size, 0.05 * size}, c + Vec2{size, 0.05 * size});
                subject.push_back(rng.uniform(0.0, 1.0) < 0.3 ? s.reversed() : s);
                if (rng.uniform(0.0, 1.0) < 0.6) {
                    const Vec2 o{rng.uniform(-0.6, 0.6) * size, rng.uniform(-0.6, 0.6) * size};
                    clip.push_back(Contour::circle(c + o, size * rng.uniform(0.3, 0.9)));
                }
                if (rng.uniform(0.0, 1.0) < 0.5) {
                    subject.push_back(segment(c - Vec2{1.2 * size, rng.uniform(-0.5, 0.5) * size},
                                              c + Vec2{1.2 * size, rng.uniform(-0.5, 0.5) * size}));
                }
            }
        }
        const auto op = static_cast<BooleanOp>(scene % 4);
        const FillRule fill = scene % 3 == 0 ? FillRule::EvenOdd : FillRule::NonZero;
        const BooleanResult one = detail::booleanOpBatched(op, subject, clip, tol, fill, kNever);
        const BooleanResult bat = booleanOp(op, subject, clip, tol, fill);
        INFO("scene " << scene << " op " << static_cast<int>(op));
        batchedScenes += bat.closed == one.closed && bat.open == one.open ? 0 : 1;
        CHECK(bat.closed.size() == one.closed.size());
        CHECK(bat.open.size() == one.open.size());
        CHECK(std::abs(totalArea(bat.closed) - totalArea(one.closed)) <= 1e-6 * (1.0 + std::abs(totalArea(one.closed))));
        CHECK(oneSidedDistance(ringsOf(bat.closed), ringsOf(one.closed)) <= kSlack);
        CHECK(oneSidedDistance(ringsOf(one.closed), ringsOf(bat.closed)) <= kSlack);
        // Open pieces: every end of one result lies on (within slack of) a piece end of the other.
        auto ends = [](const std::vector<Contour>& cs) {
            std::vector<Ring> out;
            for (const Contour& c : cs) {
                out.push_back({c.startPoint(), c.endPoint()});
            }
            return out;
        };
        CHECK(oneSidedDistance(ends(bat.open), ends(one.open)) <= kSlack);  // pieces are straight
        CHECK(oneSidedDistance(ends(one.open), ends(bat.open)) <= kSlack);
    }
    // The scenes were batched (results ordered batch by batch), not one call.
    CHECK(batchedScenes > 30);
}

TEST_CASE("T-0142 a: union and difference of a long row stay near-linear", "[geometry][boolean][batch][perf]") {
    // Before T-0142 one Clipper2 sweep held every shape of the row in its active edge list:
    // unionContours on a 10,000-rectangle row ~1.3 s, 4.4x per doubling. Release, CPU time:
    // 10,000 -> 20,000 rectangles (union, and a difference with a second row): ~2.0x.
    const int nSmall = kRelease ? 10000 : 2000;
    const int nLarge = 2 * nSmall;
    const Tolerance tol{0.001, 1e-9, 0.001};
    const std::vector<Contour> rs = rectangleRow(nSmall);
    const std::vector<Contour> rl = rectangleRow(nLarge);
    std::vector<Contour> cs;
    std::vector<Contour> cl;
    for (int i = 0; i < nLarge; ++i) {
        (i < nSmall ? cs : cl).push_back(rect({0.7 * i + 0.2, 0.1}, {0.7 * i + 0.4, 0.25}));
    }
    cl.insert(cl.begin(), cs.begin(), cs.end());
    std::size_t ringsSmall = 0;
    std::size_t ringsLarge = 0;
    const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
        3,
        [&] { ringsSmall = unionContours(rs, tol).size() + booleanOp(BooleanOp::Difference, rs, cs, tol).closed.size(); },
        [&] { ringsLarge = unionContours(rl, tol).size() + booleanOp(BooleanOp::Difference, rl, cl, tol).closed.size(); });
    INFO("CPU seconds: " << nSmall << " rectangles " << t.smallSeconds << ", " << nLarge << " rectangles "
                         << t.largeSeconds << ", ratio " << t.ratio());
    // n union rings; the difference leaves a frame (two rings) of every 0.3 high rectangle and a
    // U of every 0.2 high one.
    CHECK(ringsSmall == 5 * static_cast<std::size_t>(nSmall) / 2);
    CHECK(ringsLarge == 5 * static_cast<std::size_t>(nLarge) / 2);
    CHECK(t.ratio() <= 2.5);
}

TEST_CASE("T-0142 e: StripIndex crops rings to a strip and keeps every winding number in it",
          "[geometry][boolean][strip]") {
    carvetto::test::SplitMix rng{1421};
    for (int trial = 0; trial < 150; ++trial) {
        // Random star-shaped rings (CCW or CW), some nested, some apart.
        std::vector<GridRing> rings;
        const int nr = 1 + static_cast<int>(rng.uniform(0.0, 6.0));
        for (int r = 0; r < nr; ++r) {
            const double cx = rng.uniform(-1000.0, 1000.0);
            const double cy = rng.uniform(-1000.0, 1000.0);
            const double rad = rng.uniform(50.0, 800.0);
            const int nv = 3 + static_cast<int>(rng.uniform(0.0, 40.0));
            GridRing ring;
            for (int k = 0; k < nv; ++k) {
                const double a = 6.283185307179586 * k / nv;
                const double rr = rad * rng.uniform(0.3, 1.0);
                ring.push_back({static_cast<std::int64_t>(std::llround(cx + rr * std::cos(a))),
                                static_cast<std::int64_t>(std::llround(cy + rr * std::sin(a)))});
            }
            if (rng.uniform(0.0, 1.0) < 0.4) {
                std::reverse(ring.begin(), ring.end());
            }
            rings.push_back(ring);
        }
        const double ang = rng.uniform(0.0, 6.283185307179586);
        const double ax = std::cos(ang);
        const double ay = std::sin(ang);
        const detail::StripIndex index(rings, ax, ay);
        const double lo = rng.uniform(-1500.0, 1500.0);
        const double hi = lo + (trial % 5 == 0 ? 0.0 : rng.uniform(0.0, 400.0));
        const double margin = 2.0;
        const std::vector<GridRing> cropped = index.crop(lo, hi, margin);
        // Brute force: the same construction from a direct scan of every edge.
        std::vector<GridRing> expect;
        for (const GridRing& ring : rings) {
            const std::size_t n = ring.size();
            std::vector<bool> keep(n);
            std::size_t kept = 0;
            for (std::size_t i = 0; i < n; ++i) {
                const double va = index.along(ring[i]);
                const double vb = index.along(ring[(i + 1) % n]);
                keep[i] = std::max(va, vb) >= lo - margin && std::min(va, vb) <= hi + margin;
                kept += keep[i] ? 1U : 0U;
            }
            if (kept == 0) {
                continue;
            }
            if (kept == n) {
                expect.push_back(ring);
                continue;
            }
            GridRing c;
            for (std::size_t i = 0; i < n; ++i) {
                if (!keep[i]) {
                    continue;
                }
                if (c.empty() || !(c.back() == ring[i])) {
                    c.push_back(ring[i]);
                }
                c.push_back(ring[(i + 1) % n]);
            }
            if (c.size() > 1 && c.front() == c.back()) {
                c.pop_back();
            }
            expect.push_back(c);
        }
        INFO("trial " << trial);
        REQUIRE(cropped == expect);
        // Winding numbers inside the strip are unchanged.
        for (int k = 0; k < 200; ++k) {
            const double v = rng.uniform(lo, hi);
            const double u = rng.uniform(-2500.0, 2500.0);
            const GridPoint p{static_cast<std::int64_t>(std::llround(v * ax - u * ay)),
                              static_cast<std::int64_t>(std::llround(v * ay + u * ax))};
            if (index.along(p) < lo || index.along(p) > hi) {
                continue;
            }
            CHECK(windingOf(cropped, p) == windingOf(rings, p));
        }
    }
}

TEST_CASE("T-0142 e: OpenPathClipper clips like booleanOp, cropped or not", "[geometry][boolean][strip]") {
    const Tolerance tol = Tolerance::defaults();
    for (const int teeth : {20, 400}) {  // below and above kCropMinEdges after the inset
        const std::vector<Contour> clip = offsetContours({comb(teeth)}, -0.15, JoinType::Round, tol);
        for (const double deg : {90.0, 45.0, 30.0, 0.0, 7.0}) {
            const double a = deg * 3.14159265358979323846 / 180.0;
            const Vec2 u{std::cos(a), std::sin(a)};
            const Vec2 v = u.perp();
            double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
            for (const Contour& c : clip) {
                const Box2 b = c.bounds();
                for (const Vec2 q : {b.min, b.max, Vec2{b.min.x, b.max.y}, Vec2{b.max.x, b.min.y}}) {
                    umin = std::min(umin, q.dot(u));
                    umax = std::max(umax, q.dot(u));
                    vmin = std::min(vmin, q.dot(v));
                    vmax = std::max(vmax, q.dot(v));
                }
            }
            const OpenPathClipper clipper(clip, v, tol, FillRule::EvenOdd);
            const auto n = static_cast<std::size_t>(std::ceil((vmax - vmin) / 0.37));
            const double pitch = (vmax - vmin) / static_cast<double>(n);
            std::size_t pieces = 0;
            for (std::size_t k0 = 0; k0 < n; k0 += 64) {
                std::vector<Contour> lines;
                for (std::size_t k = k0; k < std::min(n, k0 + 64); ++k) {
                    const double vk = vmin + (static_cast<double>(k) + 0.5) * pitch;
                    lines.push_back(segment(u * (umin - 1.0) + v * vk, u * (umax + 1.0) + v * vk));
                }
                const std::vector<Contour> got = clipper.clip(BooleanOp::Intersection, lines);
                const BooleanResult want = booleanOp(BooleanOp::Intersection, lines, clip, tol, FillRule::EvenOdd);
                INFO("teeth " << teeth << " angle " << deg << " batch " << k0);
                pieces += got.size();
                if (teeth == 20) {
                    CHECK(got == want.open);  // below the threshold: booleanOp itself
                } else {
                    // The same pieces (possibly in another order); an end can move by a grid unit
                    // where Clipper2 clamps it.
                    REQUIRE(got.size() == want.open.size());
                    std::vector<bool> used(want.open.size(), false);
                    std::size_t matched = 0;
                    for (const Contour& g : got) {
                        for (std::size_t i = 0; i < want.open.size(); ++i) {
                            if (!used[i] && distance(g.startPoint(), want.open[i].startPoint()) <= 2e-6 &&
                                distance(g.endPoint(), want.open[i].endPoint()) <= 2e-6) {
                                used[i] = true;
                                ++matched;
                                break;
                            }
                        }
                    }
                    CHECK(matched == got.size());
                }
                const std::vector<Contour> diff = clipper.clip(BooleanOp::Difference, lines);
                const BooleanResult wantDiff = booleanOp(BooleanOp::Difference, lines, clip, tol, FillRule::EvenOdd);
                CHECK(diff.size() == wantDiff.open.size());
            }
            CHECK(pieces > 0);
        }
    }
    // No clip, closed subjects and empty subjects give nothing.
    const OpenPathClipper none({}, {0.0, 1.0}, tol);
    CHECK(none.clip(BooleanOp::Intersection, {segment({0, 0}, {1, 0})}).empty());
    const OpenPathClipper some({rect({0, 0}, {1, 1})}, {0.0, 1.0}, tol);
    CHECK(some.clip(BooleanOp::Intersection, {rect({0, 0}, {2, 2})}).empty());
    CHECK(some.clip(BooleanOp::Intersection, {}).empty());
    CHECK(some.clip(BooleanOp::Intersection, {segment({-1, 0.5}, {2, 0.5})}).size() == 1);
}

TEST_CASE("T-0142 e: a 90-degree raster over a comb clips in near-linear time", "[geometry][boolean][strip][perf]") {
    // The T-0131 verifier's clip90: raster lines in batches of 256 against the inset comb. Each
    // batch swept the whole comb: 0.044 / 0.268 / 2.10 s at 500 / 1,000 / 2,000 teeth (Release).
    // Cropped to each batch's strip: ~2x per doubling.
    const int nSmall = kRelease ? 1000 : 250;
    const int nLarge = 2 * nSmall;
    const Tolerance tol = Tolerance::defaults();
    const Vec2 u{0.0, 1.0};
    const Vec2 v = u.perp();
    auto run = [&](const std::vector<Contour>& clip, int teeth) {
        const OpenPathClipper clipper(clip, v, tol, FillRule::EvenOdd);
        const double vmin = -static_cast<double>(teeth);
        const double vmax = 0.0;
        const auto n = static_cast<std::size_t>(std::ceil((vmax - vmin) / 0.12));
        const double pitch = (vmax - vmin) / static_cast<double>(n);
        std::size_t pieces = 0;
        for (std::size_t k0 = 0; k0 < n; k0 += 256) {
            std::vector<Contour> lines;
            for (std::size_t k = k0; k < std::min(n, k0 + 256); ++k) {
                const double vk = vmin + (static_cast<double>(k) + 0.5) * pitch;
                lines.push_back(segment(u * -6.0 + v * vk, u * 21.0 + v * vk));
            }
            pieces += clipper.clip(BooleanOp::Intersection, lines).size();
        }
        return pieces;
    };
    const std::vector<Contour> cs = offsetContours({comb(nSmall)}, -0.15, JoinType::Round, tol);
    const std::vector<Contour> cl = offsetContours({comb(nLarge)}, -0.15, JoinType::Round, tol);
    std::size_t ps = 0;
    std::size_t pl = 0;
    const carvetto::test::ScalingTimes t =
        carvetto::test::interleavedCpuSeconds(3, [&] { ps = run(cs, nSmall); }, [&] { pl = run(cl, nLarge); });
    INFO("CPU seconds: " << nSmall << " teeth " << t.smallSeconds << ", " << nLarge << " teeth " << t.largeSeconds
                         << ", ratio " << t.ratio());
    CHECK(ps >= static_cast<std::size_t>(nSmall) * 3);
    CHECK(pl >= static_cast<std::size_t>(nLarge) * 3);
    CHECK(t.ratio() <= 2.5);
}
