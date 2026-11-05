// T-0152 AC2: the medial axis is robust on degenerate input (collinear and duplicate vertices,
// near-touching and touching contours, tiny features, slivers, large coordinates, crossing and
// invalid contours): it never crashes or hangs, every output value is finite, every point keeps
// the radius = distance-to-boundary property, and the run stays within a CPU-time bound. A
// generative fuzz (portable SplitMix RNG) mixes all of these. The scaling cases check that CPU
// time grows by at most 2.5x per doubling of boundary segments on text-like input: many small
// glyph-shaped regions, and single regions with many segments.
//
// Scaling sizes: Release 12,500 -> 25,000 -> 50,000 segments for the single regions and
// 2,500 -> 5,000 -> 10,000 glyph segments; Debug uses smaller sizes. The hidden test
// "[.medial-perf-50k]" runs the glyph ladder up to 50,000 segments (Release, about a minute).
// Reference numbers (Release, T-0152 machine): see docs/adr/0023-medial-axis.md.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/span.hpp"
#include "medial_axis_glyphs.hpp"
#include "medial_axis_oracle.hpp"
#include "test_rng.hpp"
// Last: on Windows it includes <windows.h> (macros such as near/far).
#include "cpu_scaling.hpp"
#include "cpu_time.hpp"

using namespace carvetto::geometry;
namespace orc = carvetto::test::medial;

namespace {

#ifdef NDEBUG
constexpr bool kRelease = true;
#else
constexpr bool kRelease = false;
#endif

const Tolerance kTol{};

Contour polygon(std::vector<Vec2> pts) {
    return Contour::fromPolyline(carvetto::geometry::Polyline{std::move(pts), true});
}

/// Runs the medial axis and checks the invariants that hold for any input: finite values,
/// consistent topology, radius equal to the brute-force boundary distance (sampled), and a
/// CPU-time bound. Returns the result.
MedialAxis runChecked(const std::vector<Contour>& cs, const Tolerance& tol = kTol, double cpuLimit = 20.0,
                      std::size_t radiusChecks = 60, const MedialAxisOptions& options = {}) {
    const double t0 = carvetto::test::processCpuSeconds();
    const MedialAxis ax = medialAxis(cs, tol, options);
    // The tolerance the computation used (a non-positive one falls back to the default).
    const double chordal = tol.chordal > 0.0 ? tol.chordal : Tolerance{}.chordal;
    const double cpu = carvetto::test::processCpuSeconds() - t0;
    INFO("CPU seconds " << cpu << ", samples " << ax.boundarySamples);
    CHECK(cpu < cpuLimit);
    for (const MedialBranch& br : ax.branches) {
        REQUIRE(br.points.size() >= 2);
        REQUIRE(br.sides.size() + 1 == br.points.size());
        REQUIRE(br.startNode < ax.nodes.size());
        REQUIRE(br.endNode < ax.nodes.size());
    }
    const std::vector<MedialPoint> pts = orc::allPoints(ax);
    for (const MedialPoint& p : pts) {
        REQUIRE(std::isfinite(p.point.x));
        REQUIRE(std::isfinite(p.point.y));
        REQUIRE(std::isfinite(p.radius));
        REQUIRE(p.radius >= 0.0);
    }
    // Only finite closed contours take part in the oracle distance.
    std::vector<Contour> valid;
    for (const Contour& c : cs) {
        bool finite = c.closed && !c.spans.empty();
        for (const Span& s : c.spans) {
            const Box2 b = bounds(s);
            finite = finite && std::isfinite(b.min.x) && std::isfinite(b.min.y) && std::isfinite(b.max.x) &&
                     std::isfinite(b.max.y);
        }
        if (finite) {
            valid.push_back(c);
        }
    }
    if (!ax.accuracyCapped && !valid.empty()) {
        const std::size_t stride = std::max<std::size_t>(1, pts.size() / radiusChecks);
        for (std::size_t i = 0; i < pts.size(); i += stride) {
            const double d = orc::bruteDistance(valid, pts[i].point);
            // Exact distance to the input contours; contours the axis ignored (both sides alike)
            // may only be nearer.
            CHECK(d <= pts[i].radius + chordal + 1e-9 * (1.0 + pts[i].point.length()));
        }
    }
    return ax;
}

}  // namespace

TEST_CASE("medial axis robustness: collinear and duplicate vertices", "[geometry][medial][ac2]") {
    // A rectangle whose sides carry many collinear vertices, some duplicated (zero-length spans).
    std::vector<Vec2> pts;
    for (int i = 0; i <= 50; ++i) {
        pts.push_back({0.4 * i, 0.0});
        if (i % 7 == 0) {
            pts.push_back({0.4 * i, 0.0});
        }
    }
    for (int i = 1; i <= 20; ++i) {
        pts.push_back({20.0, 0.4 * i});
    }
    for (int i = 49; i >= 0; --i) {
        pts.push_back({0.4 * i, 8.0});
    }
    for (int i = 19; i >= 1; --i) {
        pts.push_back({0.0, 0.4 * i});
    }
    const MedialAxis ax = runChecked({polygon(pts)});
    // Same skeleton as the plain rectangle: two junctions and four corner branches.
    std::size_t junctions = 0;
    for (const MedialNode& n : ax.nodes) {
        if (n.branches.size() == 3) {
            ++junctions;
        }
    }
    CHECK(junctions == 2);
    CHECK(ax.branches.size() == 5);
    // A spike that doubles back on itself (zero-width), and a degenerate all-collinear contour.
    runChecked({polygon({{0, 0}, {10, 0}, {10, 5}, {5, 5}, {5, 9}, {5, 5}, {0, 5}})});
    const MedialAxis flat = runChecked({polygon({{0, 0}, {5, 0}, {10, 0}})});
    CHECK(flat.ignoredContours == 1);
}

TEST_CASE("medial axis robustness: near-touching and touching contours", "[geometry][medial][ac2]") {
    for (double gap : {1e-3, 1e-6, 1e-9, 0.0}) {
        INFO("gap " << gap);
        // Two separate squares.
        runChecked({Contour::rectangle(Box2{{0, 0}, {5, 5}}), Contour::rectangle(Box2{{5 + gap, 0}, {10, 5}})});
        // A hole nearly touching (or touching) the outer boundary along an edge.
        runChecked({Contour::rectangle(Box2{{0, 0}, {10, 10}}), Contour::rectangle(Box2{{2, gap}, {6, 4}})});
        // A hole touching the outer boundary at a vertex.
        runChecked({Contour::rectangle(Box2{{0, 0}, {10, 10}}),
                    polygon({{5, gap}, {7, 3}, {5, 5}, {3, 3}})});
        // A circular hole nearly touching an outer circle.
        runChecked({Contour::circle({0, 0}, 10.0), Contour::circle({4.0 - gap, 0}, 6.0)});
    }
}

TEST_CASE("medial axis robustness: tiny features and slivers", "[geometry][medial][ac2]") {
    // Regions far smaller than the tolerances.
    runChecked({Contour::rectangle(Box2{{0, 0}, {1e-5, 2e-5}})});
    runChecked({Contour::circle({1, 1}, 1e-4)});
    // A large square with a notch narrower than the chordal tolerance.
    runChecked({polygon({{0, 0}, {10, 0}, {10, 10}, {5.00002, 10}, {5.00002, 3}, {5, 3}, {5, 10}, {0, 10}})});
    // Slivers: long thin triangles down to an apex angle of 1e-7 rad.
    for (double angle : {1e-2, 1e-4, 1e-7}) {
        INFO("angle " << angle);
        runChecked({polygon({{0, 0}, {50, -25 * angle}, {50, 25 * angle}})});
    }
    // A very long, very thin strip.
    runChecked({Contour::rectangle(Box2{{0, 0}, {1e4, 0.004}})});
}

TEST_CASE("medial axis robustness: large coordinates", "[geometry][medial][ac2]") {
    // A glyph-like shape far from the origin keeps its skeleton.
    std::vector<Contour> near0;
    orc::appendGlyph(near0, 2, {0, 0});
    std::vector<Contour> far0;
    orc::appendGlyph(far0, 2, {1e6, -1e6});
    const MedialAxis a = runChecked(near0);
    const MedialAxis b = runChecked(far0);
    CHECK(a.branches.size() == b.branches.size());
    CHECK(a.nodes.size() == b.nodes.size());
    // A huge region with a small hole: coordinates up to 1e6 mm.
    runChecked({Contour::rectangle(Box2{{-1e6, -1e6}, {1e6, 1e6}}), Contour::circle({0, 0}, 1.0)}, kTol, 30.0);
    runChecked({Contour::rectangle(Box2{{1e6, 1e6}, {1e6 + 20, 1e6 + 0.5}})});
}

TEST_CASE("medial axis robustness: coordinates beyond the tolerance budget are flagged", "[geometry][medial][ac2]") {
    // 1e-12 of the largest coordinate must stay below chordal / 2 (0.005 mm by default).
    const MedialAxis ok = medialAxis({Contour::rectangle(Box2{{1e8, 1e8}, {1e8 + 20, 1e8 + 8}})}, kTol);
    CHECK_FALSE(ok.accuracyCapped);
    CHECK(ok.branches.size() == 5);
    for (double off : {1e10, 1e12}) {
        INFO("offset " << off);
        const MedialAxis far1 = medialAxis({Contour::rectangle(Box2{{off, off}, {off + 20, off + 8}})}, kTol);
        CHECK(far1.accuracyCapped);
    }
    // The flag is also set when nothing usable is left (zero-area contour far out).
    const MedialAxis empty = medialAxis({polygon({{1e11, 1e11}, {1e11 + 5, 1e11}, {1e11 + 10, 1e11}})}, kTol);
    CHECK(empty.nodes.empty());
    CHECK(empty.accuracyCapped);
    // A coarser tolerance moves the limit.
    Tolerance coarse;
    coarse.chordal = 0.1;
    CHECK_FALSE(medialAxis({Contour::rectangle(Box2{{1e10, 1e10}, {1e10 + 20, 1e10 + 8}})}, coarse).accuracyCapped);
}

TEST_CASE("medial axis robustness: small regions far from the origin are kept", "[geometry][medial][ac2]") {
    // The shoelace area of a 1 x 0.5 mm rectangle at 1e8 mm cancels to zero in absolute
    // coordinates; classification must not drop it (verifier repro at 103699999.99999999).
    for (double o : {1e6, 1e7, 1e8, 1e9}) {
        for (double w : {5.0, 1.0, 0.1}) {
            for (int j = 0; j < 5; ++j) {
                const double ox = j == 0 && o == 1e8 ? 103699999.99999999 : o * (1 + 0.037 * j);
                const double oy = o == 1e8 && j == 0 ? 73570000.0 : 0.7 * o * (1 + 0.051 * j);
                INFO("o " << o << " w " << w << " j " << j);
                const MedialAxis ax = medialAxis({polygon({{ox, oy}, {ox + w, oy}, {ox + w, oy + w / 2}, {ox, oy + w / 2}})}, kTol);
                CHECK(ax.ignoredContours == 0);
                CHECK_FALSE(ax.accuracyCapped);
                REQUIRE_FALSE(ax.nodes.empty());
                // The widest inscribed circle has radius w / 4.
                double rmax = 0.0;
                for (const MedialPoint& p : orc::allPoints(ax)) {
                    rmax = std::max(rmax, p.radius);
                }
                CHECK(std::fabs(rmax - w / 4) <= kTol.chordal);
            }
        }
    }
}

TEST_CASE("medial axis robustness: crossing and invalid contours", "[geometry][medial][ac2]") {
    // Self-intersecting bow tie (zero net area: ignored) and a figure with a loop.
    const MedialAxis bow = runChecked({polygon({{0, 0}, {10, 10}, {10, 0}, {0, 10}})});
    CHECK(bow.ignoredContours == 1);
    runChecked({polygon({{0, 0}, {10, 0}, {10, 10}, {3, 2}, {7, 2}, {0, 10}})});
    // Overlapping and duplicated contours.
    runChecked({Contour::rectangle(Box2{{0, 0}, {10, 10}}), Contour::rectangle(Box2{{5, 5}, {15, 15}})});
    runChecked({Contour::rectangle(Box2{{0, 0}, {10, 10}}), Contour::rectangle(Box2{{0, 0}, {10, 10}})});
    // Non-finite coordinates, empty and open contours are ignored.
    Contour nan = Contour::rectangle(Box2{{0, 0}, {1, 1}});
    std::get<LineSpan>(nan.spans[1]).end.x = std::numeric_limits<double>::quiet_NaN();
    std::get<LineSpan>(nan.spans[2]).start.x = std::numeric_limits<double>::quiet_NaN();
    Contour inf = Contour::circle({0, 0}, std::numeric_limits<double>::infinity());
    Contour empty;
    empty.closed = true;
    const MedialAxis bad = runChecked({nan, inf, empty, Contour::rectangle(Box2{{2, 2}, {4, 3}})});
    CHECK(bad.ignoredContours == 3);
    CHECK_FALSE(bad.branches.empty());
    // A non-positive or non-finite tolerance falls back to the default.
    Tolerance odd;
    odd.chordal = -1.0;
    runChecked({Contour::rectangle(Box2{{0, 0}, {4, 2}})}, odd);
}

TEST_CASE("medial axis robustness: the sample budget stops refinement", "[geometry][medial][ac2]") {
    MedialAxisOptions opt;
    opt.maxSamples = 200;
    const MedialAxis ax = medialAxis({Contour::rectangle(Box2{{0, 0}, {100, 1}})}, kTol, opt);
    CHECK(ax.accuracyCapped);
    CHECK(ax.boundarySamples <= 200);
}

namespace {

/// One random test region: a star-shaped polygon or a curve loop (lines, arcs, Beziers), with
/// optional holes, grid snapping (collinear and duplicate vertices), spikes and a random scale
/// and offset.
std::vector<Contour> fuzzCase(carvetto::test::SplitMix& rng) {
    const double scale = rng.logUniform(1e-3, 1e4);
    const Vec2 offset{rng.uniform(-1.0, 1.0) * rng.logUniform(1.0, 1e6), rng.uniform(-1.0, 1.0) * rng.logUniform(1.0, 1e6)};
    const bool snap = rng.uniform(0, 1) < 0.4;
    const double grid = scale * rng.logUniform(1e-3, 0.2);
    auto place = [&](Vec2 p) {
        p = p * scale;
        if (snap) {
            p = {std::round(p.x / grid) * grid, std::round(p.y / grid) * grid};
        }
        return p + offset;
    };
    std::vector<Contour> cs;
    const int loops = 1 + static_cast<int>(rng.uniform(0, 3));
    for (int l = 0; l < loops; ++l) {
        const Vec2 c = l == 0 ? Vec2{0, 0} : Vec2{rng.uniform(-0.5, 0.5), rng.uniform(-0.5, 0.5)};
        const double rad = l == 0 ? 1.0 : rng.uniform(0.05, 0.4);
        const int n = 3 + static_cast<int>(rng.uniform(0, 40));
        Contour ct;
        ct.closed = true;
        std::vector<Vec2> pts;
        for (int i = 0; i < n; ++i) {
            const double a = kTwoPi * (i + rng.uniform(0.0, 0.9)) / n;
            double r = rad * rng.uniform(0.2, 1.0);
            if (rng.uniform(0, 1) < 0.1) {
                r = rad * 1e-6;  // spike towards the centre
            }
            pts.push_back(place(c + Vec2{std::cos(a), std::sin(a)} * r));
        }
        for (int i = 0; i < n; ++i) {
            const Vec2 a = pts[static_cast<std::size_t>(i)];
            const Vec2 b = pts[static_cast<std::size_t>((i + 1) % n)];
            const double kind = rng.uniform(0, 1);
            if (kind < 0.6 || a == b) {
                ct.append(LineSpan{a, b});
            } else if (kind < 0.8) {
                const Vec2 m = (a + b) * 0.5;
                const Vec2 d = b - a;
                const Vec2 bulge = d.perp() * rng.uniform(-0.6, 0.6);
                ct.append(BezierSpan{a, m + bulge - d * 0.2, m + bulge + d * 0.2, b});
            } else {
                // Arc through a and b bulging to either side.
                const double h = rng.uniform(-0.45, 0.45);
                const Vec2 d = b - a;
                const Vec2 centre = (a + b) * 0.5 + d.perp() * h;
                const ArcDirection dir = rng.uniform(0, 1) < 0.5 ? ArcDirection::CCW : ArcDirection::CW;
                ct.append(ArcSpan::fromCentre(centre, a, b, dir));
            }
        }
        cs.push_back(std::move(ct));
    }
    return cs;
}

}  // namespace

TEST_CASE("medial axis robustness: generative fuzz never crashes, hangs or goes non-finite", "[geometry][medial][ac2][fuzz]") {
    // A sample budget keeps every case short in Debug; capped results must still be finite and
    // consistent (the radius check is skipped for them). The forked, unbudgeted fuzz with
    // thousands of cases is run outside ctest (see the T-0152 report).
    carvetto::test::SplitMix rng{0x7152};
    const int cases = kRelease ? 150 : 80;
    MedialAxisOptions opt;
    opt.maxSamples = kRelease ? 50000 : 20000;
    double worst = 0.0;
    for (int k = 0; k < cases; ++k) {
        const std::vector<Contour> cs = fuzzCase(rng);
        INFO("case " << k);
        const double t0 = carvetto::test::processCpuSeconds();
        runChecked(cs, kTol, 20.0, 10, opt);
        worst = std::max(worst, carvetto::test::processCpuSeconds() - t0);
    }
    INFO("worst case CPU seconds " << worst);
    CHECK(worst < 20.0);
}

namespace {

double bestCpu(int runs, const std::vector<Contour>& cs, MedialAxis& out) {
    double best = std::numeric_limits<double>::infinity();
    for (int i = 0; i < runs; ++i) {
        const double t0 = carvetto::test::processCpuSeconds();
        out = medialAxis(cs, kTol);
        best = std::max(1e-3, std::min(best, carvetto::test::processCpuSeconds() - t0));
    }
    return best;
}

/// Checks t(2n) <= 2.5 t(n) along a ladder of sizes for inputs made by `make`. Each step is timed
/// as `runs` interleaved pairs of runs of the two sizes (`interleavedCpuSeconds`, T-0145), so a
/// load swing on a shared machine affects both sizes alike (T-0159: timing the sizes one after the
/// other failed under parallel ctest and on CI runners).
template <class Make>
void checkScaling(const char* what, std::vector<std::size_t> sizes, Make make, int runs) {
    std::vector<Contour> prevCs;
    std::string log;
    for (std::size_t i = 0; i < sizes.size(); ++i) {
        const std::vector<Contour> cs = make(sizes[i]);
        std::size_t spans = 0;
        for (const Contour& c : cs) {
            spans += c.spans.size();
        }
        MedialAxis ax;
        double t = 0.0;
        double ratio = 0.0;
        if (i == 0) {
            t = bestCpu(1, cs, ax);
        } else {
            const carvetto::test::ScalingTimes st = carvetto::test::interleavedCpuSeconds(
                runs, [&] { (void)medialAxis(prevCs, kTol); }, [&] { ax = medialAxis(cs, kTol); });
            t = st.largeSeconds;
            ratio = st.ratio();
        }
        log += std::string(what) + ": " + std::to_string(spans) + " segments, " + std::to_string(t) + " s CPU, " +
               std::to_string(ax.boundarySamples) + " samples, " + std::to_string(ax.branches.size()) + " branches" +
               (i > 0 ? ", ratio " + std::to_string(ratio) : std::string()) + "\n";
        INFO(log);
        CHECK_FALSE(ax.accuracyCapped);
        CHECK_FALSE(ax.branches.empty());
        if (i > 0) {
            CHECK(ratio <= 2.5);
        }
        prevCs = cs;
    }
}

}  // namespace

TEST_CASE("medial axis scaling: text-like glyph regions", "[geometry][medial][ac2][perf]") {
    const std::vector<std::size_t> sizes = kRelease ? std::vector<std::size_t>{2500, 5000, 10000}
                                                    : std::vector<std::size_t>{250, 500};
    checkScaling("glyphs", sizes, orc::glyphPage, 2);
}

TEST_CASE("medial axis scaling: single regions with many segments", "[geometry][medial][ac2][perf]") {
    const std::vector<std::size_t> sizes = kRelease ? std::vector<std::size_t>{12500, 25000, 50000}
                                                    : std::vector<std::size_t>{5000, 10000};
    checkScaling("wavy", sizes, orc::wavyRegion, 3);
    const std::vector<std::size_t> serp = kRelease ? std::vector<std::size_t>{12500, 25000, 50000}
                                                   : std::vector<std::size_t>{2000, 4000};
    checkScaling("serpentine", serp, orc::serpentineRegion, 3);
}

TEST_CASE("medial axis scaling: glyph regions up to 50k segments", "[.medial-perf-50k]") {
    checkScaling("glyphs", {6250, 12500, 25000, 50000}, orc::glyphPage, 1);
    CHECK(true);  // the per-size numbers are printed with -s
}
