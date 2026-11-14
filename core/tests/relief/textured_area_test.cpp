#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include <catch2/catch_test_macros.hpp>

#include "carvetto/relief/textured_area.hpp"

using namespace carvetto::relief;

namespace {
// 2x2 seed, cell 1: row 0 = {1, 2}, row 1 = {3, 4}.
HeightField seed2() {
    HeightField f;
    REQUIRE(HeightField::fromData(Grid{2, 2, 1.0, {0.5, 0.5}}, {1, 2, 3, 4}, f) == Status::Ok);
    return f;
}

std::vector<Point> rect(double x0, double y0, double x1, double y1) {
    return {{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}};
}

TexturedAreaParams base(double w = 6, double h = 4) {
    TexturedAreaParams p;
    p.seed = seed2();
    p.rings = {rect(0, 0, w, h)};
    return p;
}

HeightField run(const TexturedAreaParams& p) {
    HeightField out;
    REQUIRE(texturedArea(p, out) == TexturedStatus::Ok);
    return out;
}

// Rows bottom to top as strings of values; '.' for transparent.
std::vector<std::vector<float>> rows(const HeightField& f) {
    std::vector<std::vector<float>> r;
    for (std::size_t j = 0; j < f.height(); ++j) {
        std::vector<float> row;
        for (std::size_t i = 0; i < f.width(); ++i) row.push_back(f.at(i, j));
        r.push_back(row);
    }
    return r;
}
const float N = kTransparent;

bool eq(const std::vector<std::vector<float>>& a, const std::vector<std::vector<float>>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t j = 0; j < a.size(); ++j) {
        if (a[j].size() != b[j].size()) return false;
        for (std::size_t i = 0; i < a[j].size(); ++i) {
            const bool na = std::isnan(a[j][i]), nb = std::isnan(b[j][i]);
            if (na != nb || (!na && a[j][i] != b[j][i])) return false;
        }
    }
    return true;
}
}  // namespace

TEST_CASE("textured: plain tiling", "[relief][textured]") {
    const auto f = run(base());
    REQUIRE(f.width() == 6);
    REQUIRE(f.height() == 4);
    CHECK(f.origin().x == 0.5);
    CHECK(eq(rows(f), {{1, 2, 1, 2, 1, 2}, {3, 4, 3, 4, 3, 4}, {1, 2, 1, 2, 1, 2}, {3, 4, 3, 4, 3, 4}}));
}

TEST_CASE("textured: spacing leaves gaps, percent of tile size", "[relief][textured]") {
    auto p = base(6, 2);
    p.spacingXPercent = 50;  // pitch 3: tile 2 wide, gap 1
    const auto f = run(p);
    CHECK(eq(rows(f), {{1, 2, N, 1, 2, N}, {3, 4, N, 3, 4, N}}));
    auto q = base(4, 4);
    q.spacingYPercent = 50;  // pitch 3 in Y
    CHECK(eq(rows(run(q)), {{1, 2, 1, 2}, {3, 4, 3, 4}, {N, N, N, N}, {1, 2, 1, 2}}));
}

TEST_CASE("textured: negative spacing overlaps, later tile combined by mode", "[relief][textured]") {
    auto p = base(3, 2);
    p.spacingXPercent = -50;  // pitch 1
    // Every cell is covered by a tile's right column (2, 4) and the next tile's left column (1, 3).
    p.overlapMode = CombineMode::MergeHigh;
    CHECK(eq(rows(run(p)), {{2, 2, 2}, {4, 4, 4}}));
    p.overlapMode = CombineMode::MergeLow;
    CHECK(eq(rows(run(p)), {{1, 1, 1}, {3, 3, 3}}));
    p.overlapMode = CombineMode::Add;
    CHECK(eq(rows(run(p)), {{3, 3, 3}, {7, 7, 7}}));
    p.overlapMode = CombineMode::Subtract;  // later tile (left column) is subtracted from the earlier
    CHECK(eq(rows(run(p)), {{1, 1, 1}, {1, 1, 1}}));
}

TEST_CASE("textured: shift moves alternate rows in X and columns in Y", "[relief][textured]") {
    auto p = base(6, 4);
    p.shiftXPercent = 50;  // odd rows move +1 in X
    const auto f = run(p);
    // tile row 0 (y 0..2) unshifted, tile row 1 (y 2..4) shifted by +1: columns -1.. so x=0 is seed col 1.
    CHECK(eq(rows(f), {{1, 2, 1, 2, 1, 2}, {3, 4, 3, 4, 3, 4}, {2, 1, 2, 1, 2, 1}, {4, 3, 4, 3, 4, 3}}));
    auto q = base(4, 2);
    q.shiftYPercent = 50;  // odd columns move +1 in Y: tiles there are y 1..3, and y -1..1
    // col tile 0 (x 0..2): y 0..2 normal. col tile 1 (x 2..4): rows r=-1 gives y -1..1, r=0 gives 1..3.
    // y cell 0 (0..1): upper half of tile(-1) -> seed row 1 = {3,4}; y cell 1 (1..2): seed row 0 of r=0 = {1,2}.
    CHECK(eq(rows(run(q)), {{1, 2, 3, 4}, {3, 4, 1, 2}}));
}

TEST_CASE("textured: reflection", "[relief][textured]") {
    auto p = base(4, 4);
    p.reflection = Reflection::Horizontal;
    CHECK(eq(rows(run(p)), {{1, 2, 2, 1}, {3, 4, 4, 3}, {1, 2, 2, 1}, {3, 4, 4, 3}}));
    p.reflection = Reflection::Vertical;
    CHECK(eq(rows(run(p)), {{1, 2, 1, 2}, {3, 4, 3, 4}, {3, 4, 3, 4}, {1, 2, 1, 2}}));
    p.reflection = Reflection::Both;
    CHECK(eq(rows(run(p)), {{1, 2, 2, 1}, {3, 4, 4, 3}, {3, 4, 4, 3}, {1, 2, 2, 1}}));
    p.reflection = Reflection::None;
    CHECK(eq(rows(run(p)), {{1, 2, 1, 2}, {3, 4, 3, 4}, {1, 2, 1, 2}, {3, 4, 3, 4}}));
}

TEST_CASE("textured: clipping to the boundary is exact per cell", "[relief][textured]") {
    auto p = base();
    // Triangle (0,0) (6,0) (0,4): centre (i+.5, j+.5) inside iff x/6 + y/4 < 1 (strictly here).
    p.rings = {{{0, 0}, {6, 0}, {0, 4}}};
    const auto f = run(p);
    for (std::size_t j = 0; j < f.height(); ++j)
        for (std::size_t i = 0; i < f.width(); ++i) {
            const double x = static_cast<double>(i) + 0.5, y = static_cast<double>(j) + 0.5;
            const bool in = x / 6 + y / 4 < 1.0;
            CHECK(std::isnan(f.at(i, j)) == !in);
        }
}

TEST_CASE("textured: holes by even-odd; hole cells are transparent", "[relief][textured]") {
    auto p = base(6, 4);
    p.rings = {rect(0, 0, 6, 4), rect(2, 1, 4, 3)};
    CHECK(eq(rows(run(p)), {{1, 2, 1, 2, 1, 2}, {3, 4, N, N, 3, 4}, {1, 2, N, N, 1, 2}, {3, 4, 3, 4, 3, 4}}));
}

TEST_CASE("textured: anchor moves the tile lattice; transparent seed cells stay transparent", "[relief][textured]") {
    auto p = base(4, 2);
    p.anchor = Point{1, 0};  // tile columns start at x = 1, so x 0..1 is the right half of a tile
    CHECK(eq(rows(run(p)), {{2, 1, 2, 1}, {4, 3, 4, 3}}));
    HeightField s;
    REQUIRE(HeightField::fromData(Grid{2, 2, 1.0, {0.5, 0.5}}, {1, N, 3, 4}, s) == Status::Ok);
    auto q = base(4, 2);
    q.seed = s;
    CHECK(eq(rows(run(q)), {{1, N, 1, N}, {3, 4, 3, 4}}));
}

TEST_CASE("textured: boundary not a multiple of the cell rounds the grid up", "[relief][textured]") {
    auto p = base(5.5, 3.2);
    const auto f = run(p);
    CHECK(f.width() == 6);
    CHECK(f.height() == 4);
    CHECK(std::isnan(f.at(5, 0)));  // centre x = 5.5 is on the boundary: half-open, right edge outside
    CHECK(f.at(4, 0) == 1.0f);
}

TEST_CASE("textured: tile cap gives a clear error", "[relief][textured]") {
    auto p = base(1000, 1000);
    p.spacingXPercent = 0;
    // 500 x 500 tiles = 250000 > kMaxTexturedTiles
    HeightField out;
    CHECK(texturedArea(p, out) == TexturedStatus::TooManyTiles);
    CHECK(out.empty());
    CHECK(texturedTileCount(p) == 0);
    CHECK(std::string(texturedStatusMessage(TexturedStatus::TooManyTiles)).find("too many tiles") == 0);
    // Just under the cap works: 300 x 300 = 90000 conservative-range tiles ~ <= 100000.
    auto q = base(580, 580);
    CHECK(texturedArea(q, out) == TexturedStatus::Ok);
    CHECK(texturedTileCount(q) <= kMaxTexturedTiles);
    CHECK(texturedTileCount(q) > 80000);
}

TEST_CASE("textured: validation", "[relief][textured]") {
    HeightField out;
    auto p = base();
    p.seed = HeightField();
    CHECK(texturedArea(p, out) == TexturedStatus::BadSeed);
    p = base();
    p.rings.clear();
    CHECK(texturedArea(p, out) == TexturedStatus::BadBoundary);
    p.rings = {{{0, 0}, {1, 1}}};
    CHECK(texturedArea(p, out) == TexturedStatus::BadBoundary);
    p.rings = {{{0, 0}, {1, 1}, {2, 2}}};  // zero area extent along a diagonal is fine; flat is not
    CHECK(texturedArea(p, out) == TexturedStatus::Ok);
    p.rings = {{{0, 0}, {1, 0}, {2, 0}}};
    CHECK(texturedArea(p, out) == TexturedStatus::BadBoundary);
    p.rings = {{{0, 0}, {1, 0}, {NAN, 1}}};
    CHECK(texturedArea(p, out) == TexturedStatus::BadBoundary);
    p = base();
    p.spacingXPercent = -150;
    CHECK(texturedArea(p, out) == TexturedStatus::BadParameter);
    p.spacingXPercent = -100;
    CHECK(texturedArea(p, out) == TexturedStatus::BadParameter);
    p.spacingXPercent = 0;
    p.spacingYPercent = -150;
    CHECK(texturedArea(p, out) == TexturedStatus::BadParameter);
    p = base();
    p.rings = {{{1, 0}, {1, 1}, {1, 2}}};  // zero width
    CHECK(texturedArea(p, out) == TexturedStatus::BadBoundary);
    p = base();
    p.spacingXPercent = -100;
    CHECK(texturedArea(p, out) == TexturedStatus::BadParameter);
    p = base();
    p.spacingYPercent = -100;
    CHECK(texturedArea(p, out) == TexturedStatus::BadParameter);
    p = base();
    p.spacingYPercent = -99.9;
    CHECK(validateTexturedArea(p) != TexturedStatus::BadParameter);
    p = base();
    p.shiftXPercent = INFINITY;
    CHECK(texturedArea(p, out) == TexturedStatus::BadParameter);
    p = base();
    p.shiftYPercent = NAN;
    CHECK(texturedArea(p, out) == TexturedStatus::BadParameter);
    p = base();
    p.anchor = Point{NAN, 0};
    CHECK(texturedArea(p, out) == TexturedStatus::BadParameter);
    p = base(1e6, 1e6);
    CHECK(texturedArea(p, out) != TexturedStatus::Ok);
}

TEST_CASE("textured: component wrapper and determinism", "[relief][textured]") {
    auto p = base();
    Component c;
    REQUIRE(makeTexturedComponent(p, CombineMode::MergeLow, c) == TexturedStatus::Ok);
    CHECK(c.mode == CombineMode::MergeLow);
    CHECK(c.shapeHeightPercent == 100.0);
    CHECK(eq(rows(c.field), rows(run(p))));
    p.rings.clear();
    CHECK(makeTexturedComponent(p, CombineMode::Add, c) == TexturedStatus::BadBoundary);
}

TEST_CASE("textured: ties on tile edges and boundary vertices are half-open", "[relief][textured]") {
    // Tile edges fall exactly on cell centres: each centre belongs to the tile starting there.
    auto p = base(4, 4);
    p.anchor = Point{0.5, 0.5};
    CHECK(eq(rows(run(p)), {{1, 2, 1, 2}, {3, 4, 3, 4}, {1, 2, 1, 2}, {3, 4, 3, 4}}));
    // L-shaped ring with vertices at y = 1.5 (a cell-centre row) and x = 2: the row at y = 1.5
    // is inside for x in [0, 2) only.
    auto q = base(4, 4);
    q.rings = {{{0, 0}, {4, 0}, {4, 1.5}, {2, 1.5}, {2, 4}, {0, 4}}};
    CHECK(eq(rows(run(q)), {{1, 2, 1, 2}, {3, 4, N, N}, {1, 2, N, N}, {3, 4, N, N}}));
}

TEST_CASE("textured: tile cap boundary is inclusive", "[relief][textured]") {
    // 2x2 seed: (W/2 + 2) columns x (H/2 + 2) rows of tile positions = 250 x 400 = 100000.
    auto p = base(496, 796);
    CHECK(texturedTileCount(p) == kMaxTexturedTiles);
    HeightField out;
    CHECK(texturedArea(p, out) == TexturedStatus::Ok);
    p = base(498, 796);
    CHECK(texturedTileCount(p) == 0);
    CHECK(texturedArea(p, out) == TexturedStatus::TooManyTiles);
}

namespace {
bool insideRef(const std::vector<std::vector<Point>>& rings, double x, double y) {
    bool in = false;
    for (const auto& r : rings)
        for (std::size_t k = 0; k < r.size(); ++k) {
            const Point& a = r[k];
            const Point& b = r[(k + 1) % r.size()];
            if ((a.y > y) != (b.y > y) && x >= a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y)) in = !in;
        }
    return in;
}
}  // namespace

TEST_CASE("textured: matches a brute-force per-cell reference", "[relief][textured]") {
    // Non-square 3x2 seed at cell 0.5 with distinct values, some transparent.
    HeightField s;
    REQUIRE(HeightField::fromData(Grid{3, 2, 0.5, {0.25, 0.25}}, {1, 2, 3, 4, N, 6}, s) == Status::Ok);
    struct Case {
        double sx, sy, shx, shy;
        Reflection refl;
        std::optional<Point> anchor;
        CombineMode mode;
    };
    const Case cases[] = {
        {0, 0, 0, 0, Reflection::None, std::nullopt, CombineMode::MergeHigh},
        {20, 70, 0, 0, Reflection::Both, std::nullopt, CombineMode::MergeHigh},
        {-30, -20, 0, 0, Reflection::Horizontal, std::nullopt, CombineMode::Add},
        {10, 0, 50, 0, Reflection::None, std::nullopt, CombineMode::MergeHigh},
        {10, 40, 0, 50, Reflection::Vertical, std::nullopt, CombineMode::MergeLow},
        {0, 0, -60, 35, Reflection::Both, Point{0.3, -0.7}, CombineMode::MergeHigh},
        {25, -40, 80, -80, Reflection::Horizontal, Point{-2.2, 1.9}, CombineMode::Add},
        {-50, -50, 100, 100, Reflection::None, std::nullopt, CombineMode::Multiply},
        {0, 0, 300, -300, Reflection::Both, Point{2.0, 2.0}, CombineMode::Add},
        {0, 0, -300, 300, Reflection::None, Point{2.0, 2.0}, CombineMode::Add},
        {0, 0, 350, 350, Reflection::None, Point{5.0, 4.0}, CombineMode::Add},
        {0, 0, -350, -350, Reflection::None, Point{5.0, 4.0}, CombineMode::Add},
        {5, 15, -150, 120, Reflection::Vertical, Point{4.1, 3.3}, CombineMode::Subtract},
    };
    const std::vector<std::vector<Point>> rings = {{{0, 0}, {9, 0.5}, {8, 7}, {1, 6}}, {{3, 2}, {5, 2}, {4, 4}}};
    for (const Case& c : cases) {
        TexturedAreaParams p;
        p.seed = s;
        p.rings = rings;
        p.spacingXPercent = c.sx;
        p.spacingYPercent = c.sy;
        p.shiftXPercent = c.shx;
        p.shiftYPercent = c.shy;
        p.reflection = c.refl;
        p.anchor = c.anchor;
        p.overlapMode = c.mode;
        const HeightField f = run(p);
        const double cs = 0.5, w = 1.5, h = 1.0, px = w * (1 + c.sx / 100), py = h * (1 + c.sy / 100);
        const Point an = c.anchor ? *c.anchor : Point{0, 0};
        for (std::size_t j = 0; j < f.height(); ++j)
            for (std::size_t i = 0; i < f.width(); ++i) {
                const double x = f.origin().x + static_cast<double>(i) * cs;
                const double y = f.origin().y + static_cast<double>(j) * cs;
                float want = N;
                if (insideRef(rings, x, y)) {
                    for (int r = -80; r <= 80; ++r)
                        for (int cc = -80; cc <= 80; ++cc) {
                            const double left = an.x + cc * px + ((r & 1) ? c.shx / 100 * px : 0.0);
                            const double bottom = an.y + r * py + ((cc & 1) ? c.shy / 100 * py : 0.0);
                            const double u = x - left, v = y - bottom;
                            if (u < 0 || u >= w || v < 0 || v >= h) continue;
                            long si = static_cast<long>(u / cs), sj = static_cast<long>(v / cs);
                            const bool fh = c.refl == Reflection::Horizontal || c.refl == Reflection::Both;
                            const bool fv = c.refl == Reflection::Vertical || c.refl == Reflection::Both;
                            if (fh && (cc & 1)) si = 2 - si;
                            if (fv && (r & 1)) sj = 1 - sj;
                            const float val = s.at(static_cast<std::size_t>(si), static_cast<std::size_t>(sj));
                            if (std::isnan(val)) continue;
                            want = std::isnan(want) ? val : combine(c.mode, want, val);
                        }
                }
                const float got = f.at(i, j);
                INFO("case sx=" << c.sx << " cell " << i << "," << j);
                REQUIRE(std::isnan(got) == std::isnan(want));
                if (!std::isnan(got)) REQUIRE(got == want);
            }
    }
}
