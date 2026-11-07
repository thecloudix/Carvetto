#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <random>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "carvetto/io/svg.hpp"
#include "svg_test_util.hpp"

using namespace carvetto::geometry;
using namespace carvetto::io;
using namespace svgtest;

namespace {

// Process CPU time (immune to other processes competing for the cores).
double cpuSeconds() {
    return static_cast<double>(std::clock()) / static_cast<double>(CLOCKS_PER_SEC);
}

std::size_t rnd(std::mt19937& r, std::size_t n) {
    return static_cast<std::size_t>(r()) % n;
}

bool finiteSpan(const Span& s) {
    return std::visit(
        [](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            auto fin = [](Vec2 p) { return std::isfinite(p.x) && std::isfinite(p.y) && std::fabs(p.x) <= 1e12 && std::fabs(p.y) <= 1e12; };
            if constexpr (std::is_same_v<T, LineSpan>) {
                return fin(v.start) && fin(v.end);
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                return fin(v.centre) && std::isfinite(v.radius) && v.radius > 0.0 && std::isfinite(v.startAngle) &&
                       std::isfinite(v.endAngle);
            } else {
                return fin(v.p0) && fin(v.p1) && fin(v.p2) && fin(v.p3);
            }
        },
        s);
}

void importAndCheck(const Bytes& bytes, const SvgImportOptions& o) {
    const auto r = importSvg(bytes, o);
    if (!r.ok()) {
        CHECK_FALSE(r.error().message.empty());
        return;
    }
    const SvgDrawing& d = r.value();
    std::size_t spans = 0;
    for (const SvgContour& c : d.contours) {
        REQUIRE(c.layer < d.layers.size());
        REQUIRE_FALSE(c.contour.spans.empty());
        for (const Span& s : c.contour.spans) {
            REQUIRE(finiteSpan(s));
        }
        spans += c.contour.spans.size();
    }
    CHECK(spans <= o.maxSpans);
}

SvgImportOptions smallLimits() {
    SvgImportOptions o;
    o.maxSpans = 20000;
    o.maxVisits = 20000;
    o.maxElements = 20000;
    o.maxInputBytes = 1U << 20;
    return o;
}

std::string randomPathData(std::mt19937& rng, std::size_t tokens) {
    static const char kCmds[] = "MmLlHhVvCcSsQqTtAaZz";
    static const char* const kNums[] = {"0", "1", "-1", "5.5", "-.5", "1e2", "1e999", "1e-999", "NaN", "10", "100",
                                        "1e300", "-1e300", "0.0000001", "3.14159", "+7", "1e", "--", ".", "9999999999"};
    std::string s;
    for (std::size_t i = 0; i < tokens; ++i) {
        const std::size_t k = rnd(rng, 10);
        if (k < 3) {
            s += kCmds[rnd(rng, (sizeof(kCmds)) - 1)];
        } else if (k < 9) {
            s += kNums[rnd(rng, (sizeof(kNums)) / sizeof(kNums[0]))];
        } else {
            s += (rnd(rng, 2) != 0) ? "," : "";
        }
        s += (rnd(rng, 3) == 0) ? "" : " ";
    }
    return s;
}

std::string mutate(std::string s, std::mt19937& rng) {
    const std::size_t edits = 1 + rnd(rng, 8);
    static const char kBytes[] = "<>&\"'/=;#()- 019eE.,MLAZ\n\0";
    for (std::size_t i = 0; i < edits && !s.empty(); ++i) {
        const std::size_t at = rnd(rng, s.size());
        switch (rnd(rng, 5)) {
            case 0: s[at] = kBytes[rnd(rng, (sizeof(kBytes)) - 1)]; break;
            case 1: s.erase(at, 1 + rnd(rng, 8)); break;
            case 2: s.insert(at, 1, kBytes[rnd(rng, (sizeof(kBytes)) - 1)]); break;
            case 3: s.insert(at, s.substr(rnd(rng, s.size()), 1 + rnd(rng, 40))); break;
            default: s.resize(at); break;
        }
    }
    return s;
}

}  // namespace

TEST_CASE("svg fuzz: random path data", "[svg][io][fuzz]") {
    std::mt19937 rng(12345);
    const SvgImportOptions o = smallLimits();
    const double start = cpuSeconds();
    for (int i = 0; i < 1500; ++i) {
        const std::string d = randomPathData(rng, 1 + rnd(rng, 60));
        importAndCheck(toBytes(page("<path d=\"" + d + "\"/><polyline points=\"" + d + "\"/>")), o);
    }
    CHECK(cpuSeconds() - start < 60.0);
}

TEST_CASE("svg fuzz: mutated fixtures", "[svg][io][fuzz]") {
    std::mt19937 rng(777);
    const SvgImportOptions o = smallLimits();
    const double start = cpuSeconds();
    for (const char* name : {"shapes.svg", "paths.svg", "layers.svg", "transforms.svg", "px_arcs.svg"}) {
        const Bytes raw = readFixture(name);
        REQUIRE_FALSE(raw.empty());
        const std::string base(raw.begin(), raw.end());
        for (int i = 0; i < 400; ++i) {
            importAndCheck(toBytes(mutate(base, rng)), o);
        }
        // Every prefix of the fixture (truncation) is handled.
        for (std::size_t n = 0; n < base.size(); n += 1 + base.size() / 150) {
            importAndCheck(toBytes(base.substr(0, n)), o);
        }
    }
    CHECK(cpuSeconds() - start < 60.0);
}

TEST_CASE("svg fuzz: random bytes", "[svg][io][fuzz]") {
    std::mt19937 rng(99);
    const SvgImportOptions o = smallLimits();
    for (int i = 0; i < 500; ++i) {
        Bytes b(rnd(rng, 300));
        for (auto& x : b) {
            x = static_cast<std::uint8_t>(rng());
        }
        importAndCheck(b, o);
    }
    // Random bytes behind a valid svg start.
    for (int i = 0; i < 300; ++i) {
        std::string s = "<svg xmlns=\"http://www.w3.org/2000/svg\" ";
        const std::size_t n = rnd(rng, 200);
        for (std::size_t k = 0; k < n; ++k) {
            s += static_cast<char>(rnd(rng, 256));
        }
        importAndCheck(toBytes(s), o);
    }
}

TEST_CASE("svg fuzz: hostile structures stay bounded", "[svg][io][fuzz]") {
    const SvgImportOptions o = smallLimits();
    const double start = cpuSeconds();
    SECTION("path with millions of tiny segments is refused at the span budget") {
        std::string d = "M0 0";
        for (int i = 0; i < 50000; ++i) {
            d += "l1 1";
        }
        const auto r = importText(page("<path d=\"" + d + "\"/>"), o);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::ResourceLimit);
    }
    SECTION("tiny-radius arcs with a huge arc count") {
        std::string d = "M0 0";
        for (int i = 0; i < 30000; ++i) {
            d += "a1 2 0 1 1 1 0";
        }
        const auto r = importText(page("<path d=\"" + d + "\"/>"), o);
        CHECK((!r.ok() || r.value().contours.size() <= 1));
    }
    SECTION("an extreme anisotropic ellipse needs bounded spans") {
        const auto r = importText(page("<ellipse rx=\"1e11\" ry=\"1e-11\"/><g transform=\"scale(1e-9 1e9)\"><circle r=\"1e3\"/></g>"), o);
        CHECK((!r.ok() || r.value().contours.size() <= 2));
    }
    SECTION("exponential use expansion") {
        std::string s = "<defs><path id=\"u0\" d=\"M0 0 L1 1\"/>";
        for (int i = 1; i <= 14; ++i) {
            s += "<g id=\"u" + std::to_string(i) + "\">";
            for (int k = 0; k < 10; ++k) {
                s += "<use href=\"#u" + std::to_string(i - 1) + "\"/>";
            }
            s += "</g>";
        }
        s += "</defs><use href=\"#u14\"/>";
        SvgImportOptions big = o;
        big.maxUseDepth = 100;
        const auto r = importText(page(s), big);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::ResourceLimit);
    }
    SECTION("many attributes and long names") {
        std::string s = "<svg xmlns=\"http://www.w3.org/2000/svg\"";
        for (int i = 0; i < 1000; ++i) {
            s += " a" + std::to_string(i) + "=\"1\"";
        }
        s += "/>";
        CHECK_FALSE(importText(s, o).ok());
        CHECK_FALSE(importText("<svg " + std::string(100000, 'a') + "=\"1\"/>", o).ok());
    }
    SECTION("long number runs") {
        const std::string digits(100000, '9');
        importAndCheck(toBytes(page("<path d=\"M" + digits + " 0 L1 1\"/>")), o);
        importAndCheck(toBytes(page("<path d=\"M0." + digits + "e-" + digits + " 0 L1 1\"/>")), o);
        importAndCheck(toBytes(page("<path d=\"M1e" + digits + " 0 L1 1\"/>")), o);
    }
    CHECK(cpuSeconds() - start < 60.0);
}
