// T-0140 verifier perf pages: perf FAMILY N [seed] -> CPU seconds of profileToolpath (and #moves, #loose msgs, moves hash).
// families: rect (grid of N close tabbed rectangles), arc (circles/slots/flowers), bez (bezBlobs/ellipses), mix (anyClosed),
//   row (N rectangles in one row), rings (N concentric circles), gear1 (one gear of N teeth), flower1 (one flower of N arcs),
//   bezwave (N open Bezier waves), plate (plate with N holes), letters (N letters close), diag (N parallel 45-degree bars 100 x 2 mm: overlapping boxes).
// env: SIDE (0 Outside, 1 Inside), A (allowance, 0), SEP (0/1), TL (tab length, 3), TC (tab count, 3), D (3.175), V (V-bit deg, 0), GAP (factor of D, 1.2)
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <random>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static std::mt19937_64 rng;
static double U(double a, double b) { return std::uniform_real_distribution<double>(a, b)(rng); }
static int I(int a, int b) { return std::uniform_int_distribution<int>(a, b)(rng); }
#include "gens.hpp"
#include "gen_helpers.inc"
static double env(const char* k, double d) { const char* v = std::getenv(k); return v ? std::atof(v) : d; }
static double cpu() { timespec t; clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t); return t.tv_sec + 1e-9 * t.tv_nsec; }
static unsigned long long fnv(const std::string& s) { unsigned long long h = 1469598103934665603ULL; for (unsigned char ch : s) { h ^= ch; h *= 1099511628211ULL; } return h; }
int main(int argc, char** argv) {
    std::string fam = argv[1]; int n = std::atoi(argv[2]); rng.seed(argc > 3 ? std::strtoul(argv[3], nullptr, 0) : 7);
    double D = env("D", 3.175); int vdeg = (int)env("V", 0); double gap = env("GAP", 1.2) * D;
    std::vector<Contour> cs;
    int cols = std::max(1, (int)std::ceil(std::sqrt((double)n)));
    auto cell = [&](int i, double s) { return Vec2{(i % cols) * (s + gap), (i / cols) * (s + gap)}; };
    if (fam == "rect") for (int i = 0; i < n; ++i) { double s = 8; Vec2 c = cell(i, s); cs.push_back(Contour::rectangle(geometry::Box2{c, c + Vec2{s * U(0.6, 1), s * U(0.6, 1)}})); }
    else if (fam == "arc") for (int i = 0; i < n; ++i) { double s = 8; Vec2 c = cell(i, s) + Vec2{4, 4}; int k = I(0, 2); cs.push_back(k == 0 ? Contour::circle(c, U(1, 4)) : k == 1 ? slot(c, U(2, 5), U(1, 1.5), U(0, PI)) : flower(c, U(2.5, 4), 2 * I(2, 4))); }
    else if (fam == "bez") for (int i = 0; i < n; ++i) { double s = 8; Vec2 c = cell(i, s) + Vec2{4, 4}; cs.push_back(I(0, 1) ? bezBlob(c, 4, I(4, 9)) : ellipse(c, 4, U(1.5, 4), U(0, PI))); }
    else if (fam == "mix") for (int i = 0; i < n; ++i) { double s = 8; Vec2 c = cell(i, s) + Vec2{4, 4}; cs.push_back(anyClosed(c, 4)); }
    else if (fam == "row") for (int i = 0; i < n; ++i) { double s = 5; cs.push_back(Contour::rectangle(geometry::Box2{{i * (s + gap), 0}, {i * (s + gap) + s, s}})); }
    else if (fam == "rings") for (int i = 0; i < n; ++i) cs.push_back(Contour::circle({0, 0}, 5 + i * (D + gap)));
    else if (fam == "gear1") cs.push_back(gear({0, 0}, n * 0.5, n, 2.0));
    else if (fam == "flower1") cs.push_back(flower({0, 0}, n * 0.4, 2 * (n / 2)));
    else if (fam == "bezwave") for (int i = 0; i < n; ++i) cs.push_back(bezWave({0, i * (6 + gap)}, 60, 12));
    else if (fam == "plate") { double W = cols * (8 + gap) + 10; cs.push_back(Contour::rectangle(geometry::Box2{{-5, -5}, {W, W}})); for (int i = 0; i < n; ++i) { Vec2 c = cell(i, 8) + Vec2{4, 4}; cs.push_back(I(0, 1) ? Contour::circle(c, U(1.5, 4)) : Contour::rectangle(geometry::Box2{c - Vec2{3, 2}, c + Vec2{3, 2}})); } }
    else if (fam == "diag") for (int i = 0; i < n; ++i) { double o = i * (2.0 + gap) * std::sqrt(2.0); Vec2 a{o, 0}, d{std::sqrt(0.5), std::sqrt(0.5)}, nn{-d.y, d.x}; double L = 100;
        cs.push_back(poly({a, a + d * L, a + d * L + nn * 2.0, a + nn * 2.0})); }
    else if (fam == "letters") for (int i = 0; i < n; ++i) { Vec2 c = cell(i, 12); cs.push_back(letter(c, 12)); }
    ProfileParams p;
    p.side = env("SIDE", 0) ? ProfileSide::Inside : ProfileSide::Outside;
    if (env("SIDE", 0) == 2) p.side = ProfileSide::On;
    p.allowance = env("A", 0); p.separateLastPass = env("SEP", 0) != 0; p.lastPassAllowance = 0.3;
    p.passes.cutDepth = vdeg ? 3 : 6; p.passes.passDepth = 3;
    p.tabs.enabled = env("TABS", 1) != 0; p.tabs.length = env("TL", 3); p.tabs.count = (int)env("TC", 3); p.tabs.thickness = 1.0;
    p.leads.type = LeadType::None;
    tooling::Tool t = vdeg ? vbit(D, vdeg) : endMill(D);
    double c0 = cpu();
    auto r = profileToolpath(cs, t, p);
    double c1 = cpu();
    int nl = 0; std::string o; char b[200];
    for (auto& d : r.diagnostics) if (d.message.find("loose piece") != std::string::npos) ++nl;
    for (auto& m : r.toolpath.moves) { snprintf(b, sizeof b, "%d %a %a %a %a %a|", (int)m.kind, m.to.x, m.to.y, m.to.z, m.centre.x, m.centre.y); o += b; }
    std::printf("P %s %d cpu %.4f moves %zu loose %d hash %016llx\n", fam.c_str(), n, c1 - c0, r.toolpath.moves.size(), nl, fnv(o));
}
