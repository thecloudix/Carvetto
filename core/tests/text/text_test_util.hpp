#pragma once

// Shared helpers of the text tests (T-0130): the OFL test font, an independent signed-area oracle
// and a stable hash of layout output.

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <variant>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/text/layout.hpp"

namespace carvetto::text::testutil {

inline std::string fixturePath() {
    return std::string(CARVETTO_TEXT_FIXTURE_DIR) + "/CarvettoTest-Regular.ttf";
}

inline std::string cffFixturePath() {
    return std::string(CARVETTO_TEXT_FIXTURE_DIR) + "/CarvettoTestCFF-Regular.otf";
}

inline std::vector<std::uint8_t> fontBytes(const std::string& path = std::string()) {
    std::ifstream in(path.empty() ? fixturePath() : path, std::ios::binary);
    REQUIRE(in.good());
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

inline Font loadTestFont() {
    const std::vector<std::uint8_t> bytes = fontBytes();
    FontLoad r = Font::load(bytes);
    REQUIRE(r.ok());
    return std::move(r.font);
}

inline Font loadCffFont() {
    FontLoad r = Font::load(fontBytes(cffFixturePath()));
    REQUIRE(r.ok());
    return std::move(r.font);
}

/// Signed area of a closed contour by Green's theorem, 1/2 * integral of (x dy - y dx). Lines are
/// exact; cubic Beziers integrate the degree-5 integrand with 3-point Gauss-Legendre, which is
/// exact for polynomials up to that degree (so this does not share any code with the library).
inline double signedArea(const geometry::Contour& c) {
    static const double nodes[3] = {-0.7745966692414834, 0.0, 0.7745966692414834};
    static const double weights[3] = {5.0 / 9.0, 8.0 / 9.0, 5.0 / 9.0};
    double sum = 0.0;
    for (const geometry::Span& s : c.spans) {
        if (const auto* l = std::get_if<geometry::LineSpan>(&s)) {
            sum += 0.5 * (l->start.x * l->end.y - l->end.x * l->start.y);
        } else if (const auto* b = std::get_if<geometry::BezierSpan>(&s)) {
            double acc = 0.0;
            for (int k = 0; k < 3; ++k) {
                const double t = 0.5 * (nodes[k] + 1.0);
                const double u = 1.0 - t;
                const double x = u * u * u * b->p0.x + 3 * u * u * t * b->p1.x + 3 * u * t * t * b->p2.x + t * t * t * b->p3.x;
                const double y = u * u * u * b->p0.y + 3 * u * u * t * b->p1.y + 3 * u * t * t * b->p2.y + t * t * t * b->p3.y;
                const double dx = 3 * (u * u * (b->p1.x - b->p0.x) + 2 * u * t * (b->p2.x - b->p1.x) + t * t * (b->p3.x - b->p2.x));
                const double dy = 3 * (u * u * (b->p1.y - b->p0.y) + 2 * u * t * (b->p2.y - b->p1.y) + t * t * (b->p3.y - b->p2.y));
                acc += weights[k] * (x * dy - y * dx);
            }
            sum += 0.5 * 0.5 * acc;  // 1/2 (Green) * 1/2 (interval half-length)
        }
    }
    return sum;
}

inline double netArea(const std::vector<geometry::Contour>& contours, std::size_t first, std::size_t count) {
    double a = 0.0;
    for (std::size_t i = first; i < first + count; ++i) {
        a += signedArea(contours[i]);
    }
    return a;
}

/// FNV-1a over the exact bits of every coordinate (-0.0 folded to +0.0) and the structure.
inline std::uint64_t hashLayout(const TextLayout& l) {
    std::uint64_t h = 1469598103934665603ull;
    const auto mix = [&](std::uint64_t v) {
        for (int i = 0; i < 8; ++i) {
            h ^= (v >> (8 * i)) & 0xFFu;
            h *= 1099511628211ull;
        }
    };
    const auto mixD = [&](double d) {
        if (d == 0.0) {
            d = 0.0;
        }
        std::uint64_t bits = 0;
        std::memcpy(&bits, &d, sizeof bits);
        mix(bits);
    };
    mix(static_cast<std::uint64_t>(l.status));
    mix(l.glyphs.size());
    for (const PlacedGlyph& g : l.glyphs) {
        mix(g.glyphId);
        mix(g.cluster);
        mixD(g.origin.x);
        mixD(g.origin.y);
    }
    mix(l.contours.size());
    for (const geometry::Contour& c : l.contours) {
        mix(c.closed ? 1 : 0);
        mix(c.spans.size());
        for (const geometry::Span& s : c.spans) {
            if (const auto* ln = std::get_if<geometry::LineSpan>(&s)) {
                mix(1);
                for (const geometry::Vec2 p : {ln->start, ln->end}) {
                    mixD(p.x);
                    mixD(p.y);
                }
            } else if (const auto* b = std::get_if<geometry::BezierSpan>(&s)) {
                mix(2);
                for (const geometry::Vec2 p : {b->p0, b->p1, b->p2, b->p3}) {
                    mixD(p.x);
                    mixD(p.y);
                }
            } else {
                mix(3);
            }
        }
    }
    return h;
}

}  // namespace carvetto::text::testutil
