#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/io/dxf.hpp"
#include "dxf_test_util.hpp"

using namespace carvetto::geometry;
using namespace carvetto::io;
using namespace dxftest;

namespace {

// Process CPU time, so the bounds below are immune to other processes competing for the cores.
// (On MSVC std::clock() is wall-clock; the Windows CI runs ctest serially.)
double cpuSeconds() {
    return static_cast<double>(std::clock()) / static_cast<double>(CLOCKS_PER_SEC);
}

/// Imports and checks the result is internally consistent when it succeeds.
void importAndCheck(const Bytes& bytes, const DxfImportOptions& options) {
    auto r = importDxf(bytes, options);
    if (!r.ok()) {
        CHECK_FALSE(r.error().message.empty());
        return;
    }
    const DxfDrawing& d = r.value();
    std::size_t spans = 0;
    for (const DxfContour& c : d.contours) {
        REQUIRE(c.layer < d.layers.size());
        spans += c.contour.spans.size();
    }
    for (const DxfText& t : d.texts) {
        REQUIRE(t.layer < d.layers.size());
    }
    CHECK(spans <= options.maxSpans);
}

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> lines;
    std::size_t start = 0;
    while (start < text.size()) {
        const std::size_t nl = text.find('\n', start);
        const std::size_t end = nl == std::string::npos ? text.size() : nl;
        lines.push_back(text.substr(start, end - start));
        start = end + 1;
    }
    return lines;
}

std::string joinLines(const std::vector<std::string>& lines) {
    std::string out;
    for (const std::string& l : lines) {
        out += l;
        out += '\n';
    }
    return out;
}

Bytes mutate(const Bytes& base, std::mt19937& rng) {
    static const std::vector<std::string> kTokens{
        "0",      "SECTION",  "ENDSEC", "ENTITIES",   "BLOCKS",  "BLOCK", "ENDBLK", "INSERT", "SEQEND",
        "VERTEX", "POLYLINE", "SPLINE", "LWPOLYLINE", "ELLIPSE", "EOF",   "-1",     "1e308",  "-1e308",
        "nan",    "999999",   "71",     "40",         "42",      "10",    "2",      "SQ",     "LOOP",
        "MID",    "32767",    "1e-300", "",           "  ",      "inf"};
    Bytes b = base;
    switch (rng() % 7) {
        case 0: {  // bit flips
            const unsigned flips = 1 + static_cast<unsigned>(rng() % 8);
            for (unsigned f = 0; f < flips && !b.empty(); ++f) {
                b[rng() % b.size()] ^= static_cast<std::uint8_t>(1u << (rng() % 8));
            }
            break;
        }
        case 1:  // truncation
            b.resize(rng() % (b.size() + 1));
            break;
        case 2: {  // random byte replacement
            const unsigned edits = 1 + static_cast<unsigned>(rng() % 16);
            for (unsigned f = 0; f < edits && !b.empty(); ++f) {
                b[rng() % b.size()] = static_cast<std::uint8_t>(rng());
            }
            break;
        }
        default: {  // line-level edits: group codes, values, duplicated/deleted/swapped lines
            std::vector<std::string> lines = splitLines(toString(base));
            if (lines.empty()) {
                break;
            }
            const unsigned edits = 1 + static_cast<unsigned>(rng() % 6);
            for (unsigned e = 0; e < edits; ++e) {
                const std::size_t i = rng() % lines.size();
                switch (rng() % 6) {
                    case 0:
                        lines[i] = std::to_string(static_cast<int>(rng() % 1100) - 10);
                        break;
                    case 1:
                        lines[i] = kTokens[rng() % kTokens.size()];
                        break;
                    case 2:
                        lines.erase(lines.begin() + static_cast<std::ptrdiff_t>(i));
                        break;
                    case 3: {
                        const std::size_t j = rng() % lines.size();
                        const std::size_t n = std::min<std::size_t>(1 + rng() % 40, lines.size() - j);
                        std::vector<std::string> chunk(lines.begin() + static_cast<std::ptrdiff_t>(j),
                                                       lines.begin() + static_cast<std::ptrdiff_t>(j + n));
                        lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(i), chunk.begin(),
                                     chunk.end());
                        break;
                    }
                    case 4:
                        std::swap(lines[i], lines[rng() % lines.size()]);
                        break;
                    default:
                        lines[i] = std::to_string(static_cast<double>(rng()) * 1e-3 - 2e6);
                        break;
                }
                if (lines.empty()) {
                    break;
                }
            }
            b = toBytes(joinLines(lines));
            break;
        }
    }
    return b;
}

}  // namespace

TEST_CASE("AC5: fuzzed DXF never crashes or hangs", "[io][dxf][ac5][fuzz]") {
    // Bases: the hand-written fixtures, a Carvetto export, and binary encodings (both layouts).
    std::vector<Bytes> bases{readFixture("all_entities.dxf"), readFixture("nested_inserts.dxf"),
                             readFixture("inches.dxf")};
    {
        std::vector<DxfLayer> layers(2);
        layers[1].name = "B";
        std::vector<DxfContour> contours;
        contours.push_back({0, Contour::rectangle(Box2{{0, 0}, {10, 5}})});
        contours.push_back({1, Contour::circle({3, 3}, 2)});
        Contour mixed;
        mixed.spans.emplace_back(LineSpan{{0, 0}, {1, 0}});
        mixed.spans.emplace_back(BezierSpan{{1, 0}, {2, 1}, {3, 1}, {4, 0}});
        mixed.spans.emplace_back(ArcSpan::fromCentre({7, 0}, {4, 0}, {10, 0}, ArcDirection::CW));
        contours.push_back({1, mixed});
        bases.push_back(exportDxf(layers, contours).value());
    }
    bases.push_back(toBinary(toString(bases[0]), true));
    bases.push_back(toBinary(toString(bases[1]), false));

    // Smaller budgets keep each iteration fast while still exercising the limits.
    DxfImportOptions options;
    options.maxSpans = 200'000;
    options.maxInstances = 20'000;
    options.maxTexts = 20'000;

    std::mt19937 rng(20261025);
    constexpr int kIterations = 3000;
    double worst = 0.0;
    const double start = cpuSeconds();
    for (int i = 0; i < kIterations; ++i) {
        const Bytes& base = bases[static_cast<std::size_t>(i) % bases.size()];
        const Bytes input = mutate(base, rng);
        const double t0 = cpuSeconds();
        importAndCheck(input, options);
        worst = std::max(worst, cpuSeconds() - t0);
    }
    const double total = cpuSeconds() - start;
    INFO("total CPU " << total << " s, worst single import " << worst << " s");
    CHECK(worst < 2.0);
    CHECK(total < 120.0);
}

TEST_CASE("AC5: 100 MB of garbage fails fast or imports with warnings", "[io][dxf][ac5][garbage]") {
    constexpr std::size_t kSize = 100u * 1024u * 1024u;
    SECTION("random bytes") {
        std::mt19937 rng(42);
        Bytes junk(kSize);
        for (auto& b : junk) {
            b = static_cast<std::uint8_t>(rng());
        }
        const double t0 = cpuSeconds();
        auto r = importDxf(junk);
        CHECK_FALSE(r.ok());
        CHECK(cpuSeconds() - t0 < 5.0);
        // Random bytes behind a binary sentinel.
        std::copy(carvetto::io::detail::kBinarySentinel.begin(), carvetto::io::detail::kBinarySentinel.end(),
                  junk.begin());
        CHECK_FALSE(importDxf(junk).ok());
    }
    SECTION("well-formed pairs of unknown entities") {
        std::string text = "0\nSECTION\n2\nENTITIES\n";
        const std::string unit = "0\nJUNK\n8\nL\n10\n1.5\n999\ncomment\n";
        text.reserve(kSize + 64);
        while (text.size() < kSize) {
            text += unit;
        }
        text += "0\nENDSEC\n0\nEOF\n";
        const Bytes bytes = toBytes(text);
        text.clear();
        text.shrink_to_fit();
        const double t0 = cpuSeconds();
        auto r = importDxf(bytes);
        const double seconds = cpuSeconds() - t0;
        REQUIRE(r.ok());
        INFO("CPU " << seconds << " s");
        // Linear-time bound: measured ~9 s in Debug and ~40 s under ASan+UBSan (2026-10-25).
        CHECK(seconds < 120.0);
        REQUIRE(r.value().warnings.size() == 1);
        CHECK(r.value().warnings[0].count > 2'000'000);
        CHECK(r.value().contours.empty());
    }
}

TEST_CASE("pathological but valid inputs stay bounded", "[io][dxf][fuzz]") {
    SECTION("a spline with a tiny chordal tolerance hits the span budget, not memory") {
        DxfImportOptions o;
        o.tolerance.chordal = 1e-300;
        o.maxSpans = 10'000;
        auto r = importDxf(readFixture("all_entities.dxf"), o);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == DxfError::Code::ResourceLimit);
    }
    SECTION("huge coordinates are skipped with a warning") {
        auto r = importText(
            "0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n1e300\n20\n0\n11\n0\n21\n0\n"
            "0\nARC\n10\n0\n20\n0\n40\n1e300\n50\n0\n51\n90\n"
            "0\nLWPOLYLINE\n90\n2\n10\n0\n20\n0\n42\n1e300\n10\n1\n20\n0\n"
            "0\nENDSEC\n0\nEOF\n");
        REQUIRE(r.ok());
        CHECK(r.value().contours.empty());
        CHECK_FALSE(r.value().warnings.empty());
    }
    SECTION("angles and curve parameters too large to place a point are skipped with a warning") {
        // ELLIPSE 41/42 of +-1e300 or 1e17 (where the double spacing exceeds 2*pi), ARC angles
        // of +-1e300 degrees: never silent garbage, the rest of the file stays.
        const std::string ellipse = "0\nELLIPSE\n10\n0\n20\n0\n11\n10\n21\n0\n40\n0.5\n";
        for (const std::string params :
             {"41\n-1e300\n42\n1e300\n", "41\n1e17\n42\n1e17\n", "41\n1e17\n42\n100000000000000003\n"}) {
            INFO(params);
            auto r = importText("0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n1\n20\n2\n11\n4\n21\n6\n" + ellipse +
                                params + "0\nENDSEC\n0\nEOF\n");
            REQUIRE(r.ok());
            CHECK(r.value().contours.size() == 1);
            CHECK(hasWarning(r.value(), "ELLIPSE with start/end parameter out of range skipped"));
        }
        auto arc = importText(
            "0\nSECTION\n2\nENTITIES\n0\nARC\n10\n0\n20\n0\n40\n5\n50\n-1e300\n51\n1e300\n"
            "0\nARC\n10\n0\n20\n0\n40\n5\n50\n0\n51\n90\n0\nENDSEC\n0\nEOF\n");
        REQUIRE(arc.ok());
        CHECK(arc.value().contours.size() == 1);
        CHECK(hasWarning(arc.value(), "ARC with start/end angle out of range skipped"));
    }
    SECTION("a large but representable ELLIPSE start parameter gives the exact arc") {
        // 41 = 1000.25 is 1000.25 - 159 * 2*pi on the ellipse; the pieces must stay on it.
        const double start = 1000.25;
        const double end = start + 2.0;
        DxfImportOptions tight;
        tight.tolerance.chordal = 1e-7;
        auto r =
            importText("0\nSECTION\n2\nENTITIES\n0\nELLIPSE\n10\n0\n20\n0\n11\n10\n21\n0\n40\n0.5\n41\n" +
                           std::to_string(start) + "\n42\n" + std::to_string(end) + "\n0\nENDSEC\n0\nEOF\n",
                       tight);
        REQUIRE(r.ok());
        CHECK(r.value().warnings.empty());
        REQUIRE(r.value().contours.size() == 1);
        const Contour& c = r.value().contours[0].contour;
        auto onEllipse = [](double t) { return Vec2{10.0 * std::cos(t), 5.0 * std::sin(t)}; };
        CHECK(distance(startPoint(c.spans.front()), onEllipse(start)) <= 1e-9);
        CHECK(distance(endPoint(c.spans.back()), onEllipse(end)) <= 1e-9);
        for (const Span& sp : c.spans) {
            const Vec2 q = pointAt(sp, 0.5);
            CHECK(std::fabs(q.x * q.x / 100.0 + q.y * q.y / 25.0 - 1.0) <=
                  1e-6);  // 1e-7 mm chordal tolerance
        }
    }
    SECTION("inch coordinates overflowing to infinity are reported as out of range") {
        // Both end points become (inf, inf) in mm: equal, but not a zero-length LINE.
        DxfImportOptions o;
        auto r = importText(
            "0\nSECTION\n2\nHEADER\n9\n$INSUNITS\n70\n1\n0\nENDSEC\n"
            "0\nSECTION\n2\nENTITIES\n0\nLINE\n10\n1e308\n20\n1e308\n11\n1e307\n21\n1e308\n0\nENDSEC\n0\nEOF"
            "\n",
            o);
        REQUIRE(r.ok());
        CHECK(r.value().contours.empty());
        CHECK(hasWarning(r.value(), "out of range"));
        CHECK_FALSE(hasWarning(r.value(), "zero-length"));
    }
    SECTION("cycles through two blocks") {
        auto r = importText(
            "0\nSECTION\n2\nBLOCKS\n"
            "0\nBLOCK\n2\nA\n10\n0\n20\n0\n0\nLINE\n10\n0\n20\n0\n11\n1\n21\n0\n0\nINSERT\n2\nB\n10\n0\n20\n0"
            "\n0\nENDBLK\n"
            "0\nBLOCK\n2\nB\n10\n0\n20\n0\n0\nINSERT\n2\nA\n10\n0\n20\n0\n0\nENDBLK\n"
            "0\nENDSEC\n0\nSECTION\n2\nENTITIES\n0\nINSERT\n2\nA\n10\n0\n20\n0\n0\nENDSEC\n0\nEOF\n");
        REQUIRE(r.ok());
        CHECK(r.value().contours.size() == 1);
        CHECK(hasWarning(r.value(), "inserts itself"));
    }
}
