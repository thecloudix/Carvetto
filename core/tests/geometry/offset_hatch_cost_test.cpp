// T-0016: Sharp/Square open-offset cost on dense hatch-like vectors (zig-zags offset by many times
// their pitch). Every corner's chamfer (or mitre beyond the limit) lies on one line with those of
// hundreds of neighbouring peaks; the offset used to test each of the ~k^2 overlapping pieces of
// that line, 10-30x the cost of Round.

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/offset.hpp"
#include "cpu_scaling.hpp"
#include "offset_chain_checks.hpp"

using namespace carvetto::geometry;

namespace {

const Tolerance kTol{};  // distance 0.001

// Open zig-zag through (i * pitch, (i % 2) * amplitude), i = 0..lines.
Contour zigZag(int lines, double pitch, double amplitude) {
    Contour c;
    Vec2 prev{0.0, 0.0};
    for (int i = 1; i <= lines; ++i) {
        const Vec2 p{i * pitch, (i % 2) * amplitude};
        c.spans.emplace_back(LineSpan{prev, p});
        prev = p;
    }
    return c;
}

double distanceToInput(Vec2 p, const Contour& input) {
    double best = 1e300;
    for (const Span& s : input.spans) {
        best = std::min(best, carvetto::geometry::testing::distanceToSpanExact(p, s));
    }
    return best;
}

double distanceToChains(Vec2 p, const std::vector<Contour>& chains) {
    double best = 1e300;
    for (const Contour& c : chains) {
        for (const Span& s : c.spans) {
            best = std::min(best, carvetto::geometry::testing::distanceToSpanExact(p, s));
        }
    }
    return best;
}

}  // namespace

TEST_CASE("T-0016: dense zig-zag, Sharp and Square: one chain on the chamfer line, exact and >= d",
          "[geometry][offset][open][t0016]") {
    // Pitch 0.02, amplitude 1, d = 7: on the side of the peaks every corner falls back to a
    // chamfer at amplitude + d, and all 150 of them overlap. Only one copy of each stretch of
    // that line is tested now; none may go missing, and the handed-over junctions between copies
    // must still be exact.
    const int lines = 300;
    const double pitch = 0.02;
    const double amplitude = 1.0;
    const double d = 7.0;
    const Contour in = zigZag(lines, pitch, amplitude);
    for (const JoinType join : {JoinType::Sharp, JoinType::Square}) {
        for (const OffsetSide side : {OffsetSide::Left, OffsetSide::Right}) {
            CAPTURE(static_cast<int>(join), static_cast<int>(side));
            const std::vector<Contour> chains = offsetOpenContour(in, d, side, join, kTol);
            REQUIRE(chains.size() == 1);
            const Contour& c = chains.front();
            std::size_t gaps = 0;
            std::size_t notIdentical = 0;
            for (std::size_t k = 1; k < c.spans.size(); ++k) {
                const Vec2 e = endPoint(c.spans[k - 1]);
                const Vec2 s = startPoint(c.spans[k]);
                if (distance(e, s) > 1e-12) {
                    ++gaps;
                }
                if (!(e == s)) {  // the input has lines only, so every span is a line
                    ++notIdentical;
                }
            }
            CHECK(gaps == 0);
            CHECK(notIdentical == 0);
            CHECK(carvetto::geometry::testing::countBackwardSteps(c, kTol) == 0);
            std::size_t close = 0;
            for (const Span& s : c.spans) {
                CHECK(std::holds_alternative<LineSpan>(s));
                for (int k = 0; k <= 4; ++k) {
                    if (distanceToInput(pointAt(s, k / 4.0), in) < d - 1e-6 * d) {
                        ++close;
                    }
                }
            }
            CHECK(close == 0);
            // The chamfer line runs over every corner on the offset side (Left: the peaks at
            // amplitude, Right: the valleys at 0; the path's two ends are not corners), at d
            // from it.
            const double y = side == OffsetSide::Left ? amplitude + d : -d;
            std::size_t missing = 0;
            for (int i = side == OffsetSide::Left ? 1 : 2; i < lines; i += 2) {
                if (distanceToChains(Vec2{i * pitch, y}, chains) > 1e-9) {
                    ++missing;
                }
            }
            CHECK(missing == 0);
        }
    }
}

TEST_CASE("T-0016 AC1: dense zig-zag, Sharp and Square cost bounded against Round (CPU-time ratio)",
          "[geometry][offset][open][t0016][perf]") {
    // The card's case is 1000 spans, pitch 0.02, amplitude 1, d = 7: before T-0016 Sharp and
    // Square took 10-30x as long as Round (~14-20 s vs 0.7 s in Release; measured in the T-0016
    // report). Here a shorter zig-zag of the same density, whose chamfers all overlap as they do
    // there (400 spans: ~15x before; 200 in Debug).
#ifdef NDEBUG
    const int lines = 400;
#else
    const int lines = 200;
#endif
    const Contour in = zigZag(lines, 0.02, 1.0);
    const double d = 7.0;
    for (const auto& [join, side] : {std::pair{JoinType::Sharp, OffsetSide::Left},
                                     std::pair{JoinType::Square, OffsetSide::Right}}) {
        CAPTURE(static_cast<int>(join), static_cast<int>(side));
        std::size_t spans = 0;
        const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
            3,
            [&, s = side] {
                for (const Contour& c : offsetOpenContour(in, d, s, JoinType::Round, kTol)) {
                    spans += c.spans.size();
                }
            },
            [&, j = join, s = side] {
                for (const Contour& c : offsetOpenContour(in, d, s, j, kTol)) {
                    spans += c.spans.size();
                }
            });
        CHECK(spans > 0);
        CAPTURE(t.smallSeconds, t.largeSeconds, t.ratio());
        // T-0168 made Round on this zig-zag ~8x faster (its long arcs are mostly inside the
        // neighbourhood of single input points), while Sharp and Square, whose collinear chamfer
        // copies still split each other, did not change. The 3x bound of T-0016 against the old
        // Round is therefore 30x against the new one; before T-0016 the ratio was ~15 x 8 = 120.
        CHECK(t.ratio() < 30.0);
    }
}

TEST_CASE("T-0016 AC1: dense zig-zag, Sharp and Square cost about linear in the span count",
          "[geometry][offset][open][t0016][perf]") {
    // Pitch 0.05, amplitude 0.2: ~2d / 0.1 overlapping chamfers per stretch. Each small run
    // offsets the short zig-zag twice, each large run the zig-zag twice as long once, so the
    // per-offset ratio is the measured ratio times two; the card's bound is 2.5 per doubling
    // (measured up to 4000 spans at d = 5 and 7 in the T-0016 report). d is small enough here that
    // the zig-zag reaches well past 2d, so the cheaper ends do not skew the ratio.
#ifdef NDEBUG
    const int lines = 600;
    const double d = 2.0;  // ~40 overlapping chamfers
#else
    const int lines = 300;
    const double d = 1.0;
#endif
    const Contour shortZig = zigZag(lines, 0.05, 0.2);
    const Contour longZig = zigZag(2 * lines, 0.05, 0.2);
    for (const JoinType join : {JoinType::Sharp, JoinType::Square}) {
        CAPTURE(static_cast<int>(join));
        std::size_t chains = 0;
        const carvetto::test::ScalingTimes t = carvetto::test::interleavedCpuSeconds(
            3,
            [&] {
                for (int r = 0; r < 2; ++r) {
                    chains += offsetOpenContour(shortZig, d, OffsetSide::Left, join, kTol).size();
                }
            },
            [&] { chains += offsetOpenContour(longZig, d, OffsetSide::Left, join, kTol).size(); });
        CHECK(chains > 0);
        const double perOffsetRatio = t.ratio() * 2.0;
        CAPTURE(t.smallSeconds, t.largeSeconds, perOffsetRatio);
        CHECK(perOffsetRatio < 2.5);
    }
}
