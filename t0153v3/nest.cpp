// T-0153 round 2: region partition by containment and machining order.
// Layouts: concentric squares 5 deep (shuffled input order and orientations), an L whose box holds a
// square that is not inside it, a frame with letters inside, letters cut out of a plate (letters = holes,
// counters = islands), two interleaved C shapes. Checks r.regions, that every region is carved, and that the
// carve visits regions contiguously in the requested order (selection order, left to right, nearest).
// usage: nest [dumpdir]   (writes the layouts in the vc2 VC_FILE format when dumpdir is given)
#include <algorithm>
#include <cstdio>
#include <fstream>
#include <functional>
#include <map>
#include "gen2.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
static int fails = 0;
#define CHECKV(c, ...) do { if (!(c)) { ++fails; std::printf("FAIL %s: ", #c); std::printf(__VA_ARGS__); std::printf("\n"); } } while (0)
static tooling::Tool vbit() { tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.diameter = 12.7; t.includedAngle = 60 * vo::PI / 180; t.defaultCuttingData = {1, 0.3, 0, 0, 18000, 1500, 500, 0, 1}; return t; }
static Contour sq(double x0, double y0, double x1, double y1) { return vs::poly({{x0, y0}, {x1, y0}, {x1, y1}, {x0, y1}}); }
static void dumpTo(const std::string& path, const std::vector<Contour>& cs) {
    std::ofstream o(path); char b[512];
    for (auto& c : cs) { o << "contour\n"; for (auto& sp : c.spans) {
        if (auto* l = std::get_if<geometry::LineSpan>(&sp)) { std::snprintf(b, sizeof b, "    L %.17g %.17g %.17g %.17g\n", l->start.x, l->start.y, l->end.x, l->end.y); o << b; }
        else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) { std::snprintf(b, sizeof b, "    A %.17g %.17g r %.17g %.17g %.17g dir %d\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, (int)a->direction); o << b; }
        else { auto z = std::get<geometry::BezierSpan>(sp); std::snprintf(b, sizeof b, "    B %.17g %.17g %.17g %.17g %.17g %.17g %.17g %.17g\n", z.p0.x, z.p0.y, z.p1.x, z.p1.y, z.p2.x, z.p2.y, z.p3.x, z.p3.y); o << b; } } }
}
// region id of a point: index of the smallest-area "outer" in `outers` (by the caller's ground truth)
static std::vector<int> regionSequence(const VCarvingResult& r, const std::function<int(Vec2)>& regionOf) {
    std::vector<int> seq; for (auto& m : r.toolpath.moves) if (m.kind == MoveKind::Plunge) { int g = regionOf({m.to.x, m.to.y}); if (seq.empty() || seq.back() != g) seq.push_back(g); } return seq;
}
static bool contiguous(const std::vector<int>& seq) { std::map<int, int> seen; for (int g : seq) if (seen[g]++) return false; return true; }
static bool insideC(const std::vector<Contour>& cs, Vec2 p) { vo::Boundary B = vo::makeBoundary(cs, 0.01); return vo::insideEO(B, p); }
int main(int argc, char** argv) {
    std::string dd = argc > 1 ? argv[1] : "";
    auto tool = vbit();
    // 1. concentric squares, 5 deep, shuffled and random orientation -> EO regions: ring 25/20, ring 15/10, square 5 (half sizes)
    for (int trial = 0; trial < 20; ++trial) {
        g2::rng.seed(trial);
        std::vector<Contour> cs; for (int i = 0; i < 5; ++i) { double h = 25 - 5 * i; Contour c = sq(-h, -h, h, h); if (g2::I(0, 1)) c.reverse(); cs.push_back(c); }
        std::shuffle(cs.begin(), cs.end(), g2::rng);
        if (trial == 0 && !dd.empty()) dumpTo(dd + "/nest_concentric.txt", cs);
        VCarvingParams p; p.tolerance.chordal = 0.05; auto r = vCarvingToolpath(cs, tool, std::nullopt, {}, p);
        CHECKV(r.ok() && r.regions == 3, "concentric trial %d regions %zu", trial, r.regions);
        auto reg = [](Vec2 q) { double m = std::max(std::fabs(q.x), std::fabs(q.y)); return m > 20 ? 0 : m > 15 ? 1 : m > 10 ? 2 : m > 5 ? 3 : 4; };
        auto seq = regionSequence(r, reg);
        CHECKV(contiguous(seq), "concentric trial %d regions visited non-contiguously", trial);
        CHECKV(std::find(seq.begin(), seq.end(), 1) == seq.end() && std::find(seq.begin(), seq.end(), 3) == seq.end(), "concentric trial %d plunge in a hole", trial);
        CHECKV(std::count(seq.begin(), seq.end(), 2) == 1 && std::count(seq.begin(), seq.end(), 0) == 1 && std::count(seq.begin(), seq.end(), 4) == 1, "concentric trial %d: not every region carved", trial);
    }
    // 2. L shape whose box contains a square that is outside the L
    { std::vector<Contour> cs{vs::poly({{0, 0}, {40, 0}, {40, 8}, {8, 8}, {8, 40}, {0, 40}}), sq(25, 25, 35, 35)};
      if (!dd.empty()) dumpTo(dd + "/nest_Lbox.txt", cs);
      auto r = vCarvingToolpath(cs, tool, std::nullopt, {}, VCarvingParams{});
      bool sqCarved = false; for (auto& m : r.toolpath.moves) if (m.kind != MoveKind::Rapid && m.to.z < -0.1 && m.to.x > 25 && m.to.y > 25) sqCarved = true;
      CHECKV(r.regions == 2 && sqCarved, "L+box regions %zu square carved %d", r.regions, (int)sqCarved); }
    // 3. frame with letters inside and letters cut out of a plate
    for (int plate = 0; plate < 2; ++plate) {
        std::vector<Contour> cs; int n = 6; for (int k = 0; k < n; ++k) for (auto& c : g2::glyph(k, 6, {k * 8.0, 0})) cs.push_back(c);
        size_t letters = cs.size();
        if (plate == 0) { cs.push_back(sq(-3, -3, n * 8 + 1, 9)); cs.push_back(vs::rev(sq(-1.5, -1.5, n * 8 - 0.5, 7.5))); }
        else cs.push_back(sq(-3, -3, n * 8 + 1, 9));
        if (!dd.empty()) dumpTo(dd + (plate ? "/nest_plate.txt" : "/nest_frame.txt"), cs);
        auto r = vCarvingToolpath(cs, tool, std::nullopt, {}, VCarvingParams{});
        // expected: frame (1) + letters (6) ; plate: plate-with-letter-holes (1) + counters as islands (A, O have counters: k%4==0 A, k%4==2 O)
        size_t counters = 0; for (int k = 0; k < n; ++k) if ((k & 3) == 0 || (k & 3) == 2) ++counters;
        size_t want = plate ? 1 + counters : 1 + n;
        CHECKV(r.ok() && r.regions == want, "%s regions %zu want %zu (letters contours %zu)", plate ? "plate" : "frame", r.regions, want, letters);
    }
    // 4. machining order: 3x3 grid of squares given in a scrambled selection order
    { std::vector<Contour> cs; std::vector<int> idx{4, 0, 8, 2, 6, 1, 7, 3, 5}; for (int k : idx) { double x = (k % 3) * 20, y = (k / 3) * 20; cs.push_back(sq(x, y, x + 10, y + 10)); }
      auto reg = [](Vec2 q) { return int(std::floor(q.y / 20)) * 3 + int(std::floor(q.x / 20)); };
      VCarvingParams p; p.order.selectionOrder = true; auto r = vCarvingToolpath(cs, tool, std::nullopt, {}, p); auto seq = regionSequence(r, reg);
      CHECKV(seq == idx, "selection order");
      p.order = OrderOptions{}; p.order.leftToRight = true; r = vCarvingToolpath(cs, tool, std::nullopt, {}, p); seq = regionSequence(r, reg);
      bool ok = seq.size() == 9; for (size_t i = 1; i < seq.size(); ++i) ok = ok && (seq[i] % 3) >= (seq[i - 1] % 3); CHECKV(ok, "left to right");
      p.order = OrderOptions{}; p.order.bottomToTop = true; r = vCarvingToolpath(cs, tool, std::nullopt, {}, p); seq = regionSequence(r, reg);
      ok = seq.size() == 9; for (size_t i = 1; i < seq.size(); ++i) ok = ok && (seq[i] / 3) >= (seq[i - 1] / 3); CHECKV(ok, "bottom to top");
      p.order = OrderOptions{}; p.startPosition = {100, 100}; r = vCarvingToolpath(cs, tool, std::nullopt, {}, p); seq = regionSequence(r, reg);
      CHECKV(!seq.empty() && seq[0] == 8 && contiguous(seq), "nearest first from (100,100): first region %d", seq.empty() ? -1 : seq[0]); }
    // 5. two interleaved C shapes (boxes overlap, neither inside the other) + crossing pair
    { std::vector<Contour> cs{vs::cGlyph({0, 0}, 10, 3), g2::xform(vs::cGlyph({0, 0}, 10, 3), 1, vo::PI, {7, 0})};
      if (!dd.empty()) dumpTo(dd + "/nest_interC.txt", cs);
      auto r = vCarvingToolpath(cs, tool, std::nullopt, {}, VCarvingParams{});
      CHECKV(r.ok() && (r.regions == 2 || r.has(ToolpathIssue::CrossingVectorsResolved)), "interleaved C regions %zu", r.regions); std::printf("interleaved C: regions %zu resolved %d\n", r.regions, (int)r.has(ToolpathIssue::CrossingVectorsResolved)); }
    std::printf("nest: %d failure(s)\n", fails);
    return fails ? 1 : 0;
}
