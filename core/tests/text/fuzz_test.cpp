#include <cmath>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "text_test_util.hpp"

using namespace carvetto;
using namespace carvetto::text;
using testutil::fontBytes;

// Corrupt-font fuzzing (T-0130 criterion 3, ADR-0033 section 6). Deterministic: a fixed PRNG mutates
// the test font; every result must be a clean refusal or a finite, bounded layout. Run under
// ASan/UBSan in CI's sanitizer job and by hand: -DCMAKE_CXX_FLAGS="-fsanitize=address,undefined".

namespace {

struct SplitMix {
    std::uint64_t s;
    std::uint64_t next() {
        std::uint64_t z = (s += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    std::size_t below(std::size_t n) { return static_cast<std::size_t>(next() % n); }
};

bool finiteLayout(const TextLayout& l) {
    for (const geometry::Contour& c : l.contours) {
        for (const geometry::Span& s : c.spans) {
            if (const auto* ln = std::get_if<geometry::LineSpan>(&s)) {
                if (!std::isfinite(ln->start.x) || !std::isfinite(ln->start.y) || !std::isfinite(ln->end.x) ||
                    !std::isfinite(ln->end.y)) {
                    return false;
                }
            } else if (const auto* b = std::get_if<geometry::BezierSpan>(&s)) {
                for (const geometry::Vec2 p : {b->p0, b->p1, b->p2, b->p3}) {
                    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
                        return false;
                    }
                }
            }
        }
    }
    return true;
}

std::vector<std::uint8_t> mutate(const std::vector<std::uint8_t>& original, SplitMix& rng) {
    std::vector<std::uint8_t> b = original;
    const std::size_t directoryEnd = 12 + 16 * 11;  // the table directory: most interesting bytes
    switch (rng.below(6)) {
        case 0: {  // random byte changes anywhere
            for (std::size_t k = 1 + rng.below(8); k > 0; --k) {
                b[rng.below(b.size())] = static_cast<std::uint8_t>(rng.next());
            }
            break;
        }
        case 1: {  // changes in the table directory
            for (std::size_t k = 1 + rng.below(4); k > 0; --k) {
                b[rng.below(directoryEnd)] = static_cast<std::uint8_t>(rng.next());
            }
            break;
        }
        case 2: {  // 32-bit extremes
            const std::size_t at = rng.below(b.size() - 4);
            const std::uint8_t v = (rng.next() & 1) ? 0xFF : 0x00;
            for (std::size_t i = 0; i < 4; ++i) {
                b[at + i] = v;
            }
            break;
        }
        case 3:  // truncation
            b.resize(1 + rng.below(b.size() - 1));
            break;
        case 4: {  // copy a block over another
            const std::size_t len = 1 + rng.below(64);
            const std::size_t from = rng.below(b.size() - len);
            const std::size_t to = rng.below(b.size() - len);
            for (std::size_t i = 0; i < len; ++i) {
                b[to + i] = original[from + i];
            }
            break;
        }
        default: {  // bit flips
            for (std::size_t k = 1 + rng.below(16); k > 0; --k) {
                b[rng.below(b.size())] ^= static_cast<std::uint8_t>(1u << rng.below(8));
            }
            break;
        }
    }
    return b;
}

constexpr std::size_t kIterations = 4000;

}  // namespace

TEST_CASE("Corrupt fonts never crash or hang", "[text][fuzz][hostile]") {
    const std::vector<std::uint8_t> original = fontBytes();
    SplitMix rng{0xC0FFEEull};
    TextLimits lim;
    lim.maxWork = 200'000;
    lim.maxSpans = 20'000;
    lim.maxGlyphPoints = 2'000;
    const std::string text = "TOI IT\xC3\x89 \xD7\x90\xCC\x81 Z\n\xF0\x9F\x98\x80I";
    geometry::Contour path = geometry::Contour::rectangle(geometry::Box2{{0, 0}, {50, 30}});

    std::size_t loaded = 0;
    std::size_t refused = 0;
    const std::size_t n = kIterations;
    for (std::size_t i = 0; i < n; ++i) {
        const std::vector<std::uint8_t> bytes = mutate(original, rng);
        FontLoad r = Font::load(bytes);
        if (!r.ok()) {
            ++refused;
            CHECK_FALSE(r.font.valid());
            CHECK_FALSE(r.message.empty());
            continue;
        }
        ++loaded;
        for (std::uint32_t id = 0; id < r.font.metrics().glyphCount && id < 64; ++id) {
            const GlyphOutline g = r.font.outline(id, 2000);
            for (const geometry::Contour& c : g.contours) {
                for (const geometry::Span& s : c.spans) {
                    const geometry::Vec2 p = geometry::startPoint(s);
                    REQUIRE(std::isfinite(p.x));
                    REQUIRE(std::isfinite(p.y));
                }
            }
        }
        TextStyle style;
        style.sizeMm = 10.0;
        for (const TextDirection d : {TextDirection::Auto, TextDirection::RightToLeft}) {
            style.direction = d;
            const TextLayout l = layoutText(r.font, text, style, lim);
            CHECK(l.work <= lim.maxWork + 100'000);  // bounded (the meter is checked at each charge)
            if (l.ok()) {
                REQUIRE(finiteLayout(l));
            } else {
                CHECK(l.glyphs.empty());
            }
            const TextLayout p = layoutTextOnPath(r.font, text, style, path, {}, lim);
            if (p.ok()) {
                REQUIRE(finiteLayout(p));
            }
        }
    }
    // The mutator must exercise both outcomes, otherwise the test proves nothing.
    CHECK(refused > n / 20);
    CHECK(loaded > n / 20);
}
