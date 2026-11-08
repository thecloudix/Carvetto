#include <catch2/catch_test_macros.hpp>

#include <ft2build.h>
#include FT_FREETYPE_H

#include "text_test_util.hpp"

// Environment independence (T-0130 verifier B2, ADR-0033). HarfBuzz reads HB_SHAPER_LIST and friends
// once per process, so the variables must be set before the process starts: CMake registers this
// executable several times (core/tests/CMakeLists.txt), each with different HB_*/FREETYPE_PROPERTIES/
// FT2_* settings, and every run must reproduce the same golden hash as the unmodified environment.

#ifdef FT_CONFIG_OPTION_ENVIRONMENT_PROPERTIES
#error "FreeType would read FREETYPE_PROPERTIES: core/src/text/freetype_options.h is not in effect"
#endif
#ifdef FT_CONFIG_OPTION_USE_ZLIB
#error "FreeType's bundled zlib (gzip/WOFF) must be compiled out: core/src/text/freetype_options.h is not in effect"
#endif

using namespace carvetto;
using namespace carvetto::text;

TEST_CASE("Layout hashes are independent of the environment", "[text][determinism][env]") {
    const Font f = testutil::loadTestFont();
    TextStyle s;
    s.sizeMm = 12.5;
    s.letterSpacingMm = 0.3;
    s.alignment = Alignment::Centre;
    const TextLayout a = layoutText(f, "TOI IT\xC3\x89\n\xD7\x90 Z\xCC\x81", s);  // same input as layout_test.cpp
    REQUIRE(a.ok());
    CHECK(testutil::hashLayout(a) == 0x33c4912177b82920ull);
    TextStyle p;
    p.sizeMm = 9.0;
    p.letterSpacingMm = 0.25;
    geometry::Contour path;
    path.spans.emplace_back(geometry::LineSpan{{0, 0}, {30, 0}});
    path.spans.emplace_back(geometry::BezierSpan{{30, 0}, {60, 0}, {90, 40}, {90, 80}});
    path.spans.emplace_back(geometry::LineSpan{{90, 80}, {10, 100}});
    const TextLayout b = layoutTextOnPath(f, "TOI IT TOO", p, path);  // same input as path_test.cpp
    REQUIRE(b.ok());
    CHECK(testutil::hashLayout(b) == 0xaeaf74a71756a6acull);
}
