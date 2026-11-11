// T-0250: Photo V-Carve (photo_vcarve.hpp, ADR-0047): depth mapping, tool depth limit, strategies,
// stepover retract, determinism, cancel and hostile input.

#include <atomic>
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <optional>
#include <set>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/photo_vcarve.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using Catch::Approx;
using geometry::Box2;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

constexpr double kPi = 3.14159265358979323846;

/// 90 degree V-bit, 6 mm wide: full width at depth 3.
tooling::Tool vbit() {
    tooling::Tool t;
    t.id = "v";
    t.name = "V90";
    t.type = tooling::ToolType::VBit;
    t.diameter = 6.0;
    t.includedAngle = kPi / 2.0;
    t.defaultCuttingData.toolNumber = 1;
    t.defaultCuttingData.feedRate = 1000.0;
    t.defaultCuttingData.plungeRate = 300.0;
    t.defaultCuttingData.spindleSpeed = 16000.0;
    t.defaultCuttingData.passDepth = 5.0;
    t.defaultCuttingData.stepover = 1.0;
    return t;
}

GrayImage uniform(int w, int h, std::uint8_t g) {
    GrayImage im;
    im.width = w;
    im.height = h;
    im.pixels.assign(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), g);
    return im;
}

/// Horizontal gradient: column x has gray 255*x/(w-1) (black on the left).
GrayImage gradient(int w, int h) {
    GrayImage im = uniform(w, h, 0);
    for (int y = 0; y < h; ++y) {
        for (int x = 0; x < w; ++x) {
            im.pixels[static_cast<std::size_t>(y * w + x)] = static_cast<std::uint8_t>(255 * x / (w - 1));
        }
    }
    return im;
}

const Box2 kRect{{0, 0}, {20, 10}};

PhotoVCarveParams params() {
    PhotoVCarveParams p;
    p.maxDepth = 2.0;
    p.lineSpacingPercent = 50.0;  // 3 mm
    return p;
}

PhotoVCarveResult run(const GrayImage& im, const PhotoVCarveParams& p, const std::vector<Contour>& c = {},
                      const ToolpathRunControl& ctl = {}) {
    return photoVCarve(im, kRect, vbit(), std::nullopt, p, c, ctl);
}

std::vector<Vec3> cutPoints(const Toolpath& tp) {
    std::vector<Vec3> out;
    for (const Move& m : tp.moves) {
        if (m.kind == MoveKind::Feed || m.kind == MoveKind::Plunge) {
            out.push_back(m.to);
        }
    }
    return out;
}

}  // namespace

TEST_CASE("depth mapping: darkness times max depth, invert, clamp", "[photo_vcarve]") {
    PhotoVCarveParams p;
    p.maxDepth = 2.0;
    CHECK(photoVCarveDepth(p, 3.0, 0.0) == Approx(2.0));
    CHECK(photoVCarveDepth(p, 3.0, 255.0) == Approx(0.0));
    CHECK(photoVCarveDepth(p, 3.0, 127.5) == Approx(1.0));
    CHECK(photoVCarveDepth(p, 1.5, 0.0) == Approx(1.5));
    p.invert = true;
    CHECK(photoVCarveDepth(p, 3.0, 0.0) == Approx(0.0));
    CHECK(photoVCarveDepth(p, 3.0, 255.0) == Approx(2.0));
    CHECK(photoVCarveDepth(p, 3.0, 300.0) == Approx(2.0));
    CHECK(photoVCarveDepth(p, 3.0, std::nan("")) == 0.0);
}

TEST_CASE("tool depth limit is where the cone is as wide as the tool", "[photo_vcarve]") {
    CHECK(photoVCarveDepthLimit(vbit()) == Approx(3.0));
    tooling::Tool v60 = vbit();
    v60.includedAngle = kPi / 3.0;
    CHECK(photoVCarveDepthLimit(v60) == Approx(3.0 / std::tan(kPi / 6.0)));
    tooling::Tool eng = vbit();
    eng.type = tooling::ToolType::Engraving;
    eng.tipDiameter = 2.0;
    CHECK(photoVCarveDepthLimit(eng) == Approx(2.0));
    tooling::Tool em = vbit();
    em.type = tooling::ToolType::EndMill;
    CHECK(photoVCarveDepthLimit(em) == 0.0);
    CHECK_FALSE(photoVCarveToolAllowed(tooling::ToolType::BallNose));
    CHECK(photoVCarveToolAllowed(tooling::ToolType::VBit));
    CHECK(photoVCarveToolAllowed(tooling::ToolType::Engraving));
}

TEST_CASE("a uniform image cuts at a constant depth", "[photo_vcarve]") {
    const PhotoVCarveResult r = run(uniform(16, 8, 127), params());  // 127/255 -> darkness ~0.502
    REQUIRE(r.ok());
    const double expect = 2.0 * (1.0 - 127.0 / 255.0);
    for (const Vec3& p : cutPoints(r.toolpath)) {
        CHECK(p.z == Approx(-expect).margin(1e-9));
    }
    CHECK(r.deepest == Approx(expect));
    // A white image cuts nothing.
    const PhotoVCarveResult w = run(uniform(16, 8, 255), params());
    CHECK_FALSE(w.ok());
    CHECK(w.has(ToolpathIssue::NoRegion));
    // Inverted, white cuts full depth.
    PhotoVCarveParams p = params();
    p.invert = true;
    const PhotoVCarveResult wi = run(uniform(16, 8, 255), p);
    REQUIRE(wi.ok());
    CHECK(wi.deepest == Approx(2.0));
}

TEST_CASE("depth never exceeds the tool's full-width depth", "[photo_vcarve]") {
    PhotoVCarveParams p = params();
    p.maxDepth = 10.0;
    const PhotoVCarveResult r = run(uniform(8, 8, 0), p);
    REQUIRE(r.ok());
    CHECK(r.has(ToolpathIssue::CarveDepthLimited));
    CHECK(r.deepest == Approx(3.0));
    for (const Vec3& q : cutPoints(r.toolpath)) {
        CHECK(q.z >= -3.0 - 1e-9);
    }
    CHECK(r.toolDepthLimit == Approx(3.0));
}

TEST_CASE("raster lines: spacing, angle, zig-zag and depth follow the image", "[photo_vcarve]") {
    const PhotoVCarveResult r = run(gradient(41, 5), params());
    REQUIRE(r.ok());
    // 10 mm tall at 3 mm spacing: ceil(10/3) = 4 lines, pitch 2.5 mm at y = 1.25, 3.75, 6.25, 8.75.
    CHECK(r.lineCount == 4);
    std::set<long> ys;
    for (const Vec3& q : cutPoints(r.toolpath)) {
        ys.insert(std::lround(q.y * 100.0));
    }
    CHECK(ys == std::set<long>{125, 375, 625, 875});
    // Depth along the first line: deep (2) at the left (black), shallow at the right.
    std::vector<Vec3> pts = cutPoints(r.toolpath);
    REQUIRE(pts.size() > 3);
    CHECK(pts.front().x == Approx(0.0).margin(1e-9));
    CHECK(pts.front().z == Approx(-2.0).margin(0.05));
    // Linear gradient: depth at x is 2 * (1 - x/20) between the pixel centres.
    for (const Vec3& q : pts) {
        if (q.y == Approx(1.25) && q.x > 0.25 && q.x < 19.75) {
            CHECK(-q.z == Approx(2.0 * (1.0 - q.x / 20.0)).margin(0.03));
        }
    }
    // Second line runs the other way (zig-zag): its first cut point is at the right.
    bool seenSecond = false;
    for (const Vec3& q : pts) {
        if (q.y == Approx(3.75)) {
            CHECK(q.x == Approx(20.0).margin(1e-9));
            seenSecond = true;
            break;
        }
    }
    CHECK(seenSecond);
}

TEST_CASE("line angle rotates the lines and they stay inside the placement", "[photo_vcarve]") {
    PhotoVCarveParams p = params();
    p.lineAngle = kPi / 4.0;
    const PhotoVCarveResult r = run(uniform(20, 10, 0), p);
    REQUIRE(r.ok());
    for (const Move& m : r.toolpath.moves) {
        if (m.kind == MoveKind::Feed) {
            CHECK(m.to.x >= -1e-9);
            CHECK(m.to.x <= 20.0 + 1e-9);
            CHECK(m.to.y >= -1e-9);
            CHECK(m.to.y <= 10.0 + 1e-9);
        }
    }
    // Consecutive cut points of one line lie on a 45 degree line.
    const std::vector<Vec3> pts = cutPoints(r.toolpath);
    REQUIRE(pts.size() > 4);
    const double dx = pts[1].x - pts[0].x;
    const double dy = pts[1].y - pts[0].y;
    CHECK(std::fabs(std::fabs(dx) - std::fabs(dy)) < 1e-6);
    // 90 degrees: lines run along Y.
    p.lineAngle = kPi / 2.0;
    const PhotoVCarveResult v = run(uniform(20, 10, 0), p);
    REQUIRE(v.ok());
    const std::vector<Vec3> vp = cutPoints(v.toolpath);
    CHECK(vp[0].x == Approx(vp[1].x).margin(1e-9));
    CHECK(v.lineCount == 7);  // ceil(20 / 3)
}

TEST_CASE("hatch adds a perpendicular pass", "[photo_vcarve]") {
    PhotoVCarveParams p = params();
    const PhotoVCarveResult raster = run(uniform(20, 10, 0), p);
    p.strategy = PhotoStrategy::Hatch;
    const PhotoVCarveResult hatch = run(uniform(20, 10, 0), p);
    REQUIRE(hatch.ok());
    CHECK(hatch.lineCount == raster.lineCount + 7);
    CHECK(hatch.toolpath.moves.size() > raster.toolpath.moves.size());
    // The toolpath ends with a retract to safe Z.
    CHECK(hatch.toolpath.moves.back().kind == MoveKind::Retract);
    CHECK(hatch.toolpath.moves.back().to.z == Approx(5.0));
}

TEST_CASE("stepover retract: link at depth when the change is small, retract otherwise", "[photo_vcarve]") {
    PhotoVCarveParams p = params();
    const GrayImage im = gradient(41, 5);
    // Zig-zag lines of a horizontal gradient meet end to end at the same depth: all links stay down.
    const PhotoVCarveResult stay = run(im, p);
    CHECK(stay.toolpath.count(MoveKind::Rapid) == 1);
    // With stepover retract 0 every line retracts.
    p.stepoverRetract = 0.0;
    const PhotoVCarveResult always = run(im, p);
    CHECK(always.toolpath.count(MoveKind::Rapid) == always.lineCount);
    CHECK(always.toolpath.count(MoveKind::Retract) == always.lineCount);
    // Vertical gradient, lines along X: each line has its own constant depth, 0.5 mm apart.
    GrayImage vert = uniform(8, 41, 0);
    for (int y = 0; y < 41; ++y) {
        for (int x = 0; x < 8; ++x) {
            vert.pixels[static_cast<std::size_t>(y * 8 + x)] = static_cast<std::uint8_t>(255 * y / 40);
        }
    }
    p.stepoverRetract = 2.0;
    p.lineAngle = 0.0;
    const PhotoVCarveResult big = run(vert, p);
    p.stepoverRetract = 0.0001;
    const PhotoVCarveResult tiny = run(vert, p);
    CHECK(big.toolpath.count(MoveKind::Rapid) == 1);
    CHECK(tiny.toolpath.count(MoveKind::Rapid) == tiny.lineCount);
}

TEST_CASE("selected vectors: depth sampled along the vector, outside parts not cut", "[photo_vcarve]") {
    PhotoVCarveParams p = params();
    p.strategy = PhotoStrategy::SelectedVectors;
    Contour c;
    c.append(LineSpan{{2, 5}, {18, 5}});
    const PhotoVCarveResult r = run(gradient(41, 5), p, {c});
    REQUIRE(r.ok());
    CHECK(r.lineCount == 1);
    for (const Vec3& q : cutPoints(r.toolpath)) {
        CHECK(q.y == Approx(5.0));
        CHECK(-q.z == Approx(2.0 * (1.0 - q.x / 20.0)).margin(0.03));
    }
    // A vector wholly outside the image is not cut.
    Contour away;
    away.append(LineSpan{{30, 5}, {40, 5}});
    const PhotoVCarveResult none = run(gradient(41, 5), p, {away});
    CHECK_FALSE(none.ok());
    // Empty and discontinuous vectors are named; no vectors at all is NoRegion.
    Contour gap;
    gap.append(LineSpan{{0, 0}, {1, 0}});
    gap.append(LineSpan{{5, 5}, {6, 5}});
    const PhotoVCarveResult bad = run(gradient(41, 5), p, {Contour{}, gap, c});
    CHECK(bad.ok());
    CHECK(bad.has(ToolpathIssue::EmptyContour));
    CHECK(bad.has(ToolpathIssue::DiscontinuousContour));
    CHECK(run(gradient(41, 5), p, {}).has(ToolpathIssue::NoRegion));
}

TEST_CASE("degenerate and hostile input gives a diagnostic, never a crash", "[photo_vcarve]") {
    const GrayImage ok = uniform(4, 4, 0);
    CHECK(run(GrayImage{}, params()).has(ToolpathIssue::InvalidParameters));
    GrayImage bad = ok;
    bad.pixels.pop_back();
    CHECK_FALSE(run(bad, params()).ok());
    CHECK_FALSE(photoVCarve(ok, Box2{}, vbit(), std::nullopt, params()).ok());
    CHECK_FALSE(photoVCarve(ok, Box2{{0, 0}, {0, 5}}, vbit(), std::nullopt, params()).ok());
    PhotoVCarveParams p = params();
    p.lineSpacingPercent = 0.0;
    CHECK(run(ok, p).has(ToolpathIssue::InvalidParameters));
    p = params();
    p.maxDepth = 0.0;
    CHECK(run(ok, p).has(ToolpathIssue::InvalidParameters));
    p = params();
    p.maxDepth = std::numeric_limits<double>::quiet_NaN();
    CHECK(run(ok, p).has(ToolpathIssue::InvalidParameters));
    p = params();
    p.lineAngle = std::numeric_limits<double>::infinity();
    CHECK(run(ok, p).has(ToolpathIssue::InvalidParameters));
    p = params();
    p.safeZ = 0.0;
    CHECK(run(ok, p).has(ToolpathIssue::InvalidParameters));
    p = params();
    p.stepoverRetract = -1.0;
    CHECK(run(ok, p).has(ToolpathIssue::InvalidParameters));
    tooling::Tool em = vbit();
    em.type = tooling::ToolType::EndMill;
    CHECK(photoVCarve(ok, kRect, em, std::nullopt, params()).has(ToolpathIssue::InvalidTool));
    tooling::Tool noFeed = vbit();
    noFeed.defaultCuttingData.feedRate = 0.0;
    CHECK_FALSE(photoVCarve(ok, kRect, noFeed, std::nullopt, params()).ok());
}

TEST_CASE("the work budget refuses absurdly fine spacing quickly", "[photo_vcarve]") {
    PhotoVCarveParams p = params();
    p.lineSpacingPercent = 0.0001;
    const PhotoVCarveResult r = photoVCarve(uniform(64, 64, 0), Box2{{0, 0}, {100000, 100000}}, vbit(), std::nullopt, p);
    CHECK_FALSE(r.ok());
    CHECK(r.has(ToolpathIssue::WorkLimitExceeded));
    ToolpathRunControl ctl;
    ctl.workLimit = 100;
    const PhotoVCarveResult s = run(uniform(64, 64, 0), params(), {}, ctl);
    CHECK(s.has(ToolpathIssue::WorkLimitExceeded));
    CHECK(s.toolpath.isEmpty());
    ToolpathRunControl moves;
    moves.moveLimit = 5;
    const PhotoVCarveResult m = run(uniform(64, 64, 0), params(), {}, moves);
    CHECK(m.has(ToolpathIssue::WorkLimitExceeded));
    CHECK(m.toolpath.isEmpty());
}

TEST_CASE("cancel returns nothing partial", "[photo_vcarve]") {
    std::atomic<bool> flag{true};
    ToolpathRunControl ctl;
    ctl.cancel = &flag;
    const PhotoVCarveResult r = run(uniform(64, 64, 0), params(), {}, ctl);
    CHECK(r.has(ToolpathIssue::Cancelled));
    CHECK(r.toolpath.isEmpty());
    PhotoVCarveParams p = params();
    p.strategy = PhotoStrategy::SelectedVectors;
    Contour c;
    c.append(LineSpan{{2, 5}, {18, 5}});
    const PhotoVCarveResult v = run(uniform(64, 64, 0), p, {c}, ctl);
    CHECK(v.has(ToolpathIssue::Cancelled));
    CHECK(v.toolpath.isEmpty());
    flag = false;
    CHECK(run(uniform(64, 64, 0), params(), {}, ctl).ok());
}

TEST_CASE("deterministic and bounded for a large image", "[photo_vcarve]") {
    GrayImage big = uniform(1024, 1024, 0);
    for (std::size_t i = 0; i < big.pixels.size(); ++i) {
        big.pixels[i] = static_cast<std::uint8_t>((i * 2654435761u) >> 24);
    }
    const PhotoVCarveResult a = photoVCarve(big, Box2{{0, 0}, {100, 100}}, vbit(), std::nullopt, params());
    const PhotoVCarveResult b = photoVCarve(big, Box2{{0, 0}, {100, 100}}, vbit(), std::nullopt, params());
    REQUIRE(a.ok());
    CHECK(a.toolpath == b.toolpath);
    CHECK(a.toolpath.moves.size() < 20000000);
}

TEST_CASE("depth tolerance merges colinear samples", "[photo_vcarve]") {
    PhotoVCarveParams p = params();
    const PhotoVCarveResult flat = run(uniform(200, 100, 100), p);
    // A flat line is one plunge and one feed per line.
    CHECK(flat.toolpath.count(MoveKind::Feed) <= flat.lineCount * 2 + 4);
    p.depthTolerance = 0.0001;
    const PhotoVCarveResult fine = run(gradient(41, 5), p);
    p.depthTolerance = 0.5;
    const PhotoVCarveResult coarse = run(gradient(41, 5), p);
    CHECK(coarse.toolpath.moves.size() <= fine.toolpath.moves.size());
}

TEST_CASE("grayFromRaster uses luma weights", "[photo_vcarve]") {
    io::RasterImage im;
    im.width = 2;
    im.height = 1;
    im.rgb = {255, 0, 0, 255, 255, 255};
    const GrayImage g = grayFromRaster(im);
    REQUIRE(g.valid());
    CHECK(g.pixels[0] == 76);
    CHECK(g.pixels[1] == 255);
    CHECK_FALSE(grayFromRaster(io::RasterImage{}).valid());
}
