// T-0140 verifier: writes crafted probe scenes (scn) for t40 mode 99. usage: mkprobe OUTDIR
#include <cmath>
#include <cstdio>
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
#include "scene_io.hpp"
static ProfileParams base(ProfileSide side, double a) {
    ProfileParams p; p.side = side; p.allowance = a; p.passes.cutDepth = 3; p.passes.passDepth = 1.5; p.tabs.enabled = true; p.tabs.count = 2; p.tabs.length = 200; p.tabs.thickness = 1;
    p.leads.type = LeadType::None; return p;
}
int main(int argc, char** argv) {
    std::string O = argv[1];
    // 1. thin ring of waste between a hole (Inside) and a round part in it (Outside): ring thickness t, D 2 (r 1). Sides
    //    are per scene, so use On-free trick: hole Inside and part... both vectors get the same side; use Outside for both:
    //    the hole vector cut Outside cuts the plate side. Instead: plate hole radius Rh cut Inside, part radius Rp cut Inside
    //    too would cut inside the part. So build the ring from two parts cut Outside: part A = annulus outer boundary? Use
    //    two concentric circles cut Outside with nesting parity: outer circle = part outline (region left), inner circle = hole
    //    of that part: Outside of the part means the cut runs outside the outer circle and inside the inner circle (hole).
    for (double t : {0.03, 0.045, 0.052, 0.054, 0.056, 0.058, 0.06, 0.08, 0.2}) {
        // part 1: disc radius 10 with a hole radius 10 - ... ; ring of waste between part 2 (inner disc) and part 1's hole.
        std::vector<Contour> cs{Contour::circle({0, 0}, 20), Contour::circle({0, 0}, 12), Contour::circle({0, 0}, 12 - 4 - t)};
        ProfileParams p = base(ProfileSide::Outside, 0.0);
        char fn[256]; snprintf(fn, sizeof fn, "%s/ring_t%.3f.scn", O.c_str(), t); writeScene(fn, cs, p, 2.0, 0);
        p.tabs.length = 2; p.tabs.count = 3; snprintf(fn, sizeof fn, "%s/ring_t%.3f_tabs.scn", O.c_str(), t); writeScene(fn, cs, p, 2.0, 0);
    }
    // 2. long straight sliver between two bars side by side, closed off at both ends by a third bar each (waste enclosed):
    for (double t : {0.03, 0.045, 0.06}) {
        double g = 2.0 + t;  // D 2, Outside, allowance 0: kerfs 2 wide each side; the gap between the kerfs of neighbours is g + ... 
        // bars: top and bottom (long), left and right caps; the waste in the middle is a strip of width t between the kerfs.
        double W = 40, h = 5;
        std::vector<Contour> cs{Contour::rectangle(geometry::Box2{{0, 0}, {W, h}}), Contour::rectangle(geometry::Box2{{0, h + 2 * 2 + t}, {W, 2 * h + 4 + t}}),
                                Contour::rectangle(geometry::Box2{{-5 - 2.0, -3}, {-2.0 - 0.0, 2 * h + 4 + t + 3}}), Contour::rectangle(geometry::Box2{{W + 2.0, -3}, {W + 7.0, 2 * h + 4 + t + 3}})};
        ProfileParams p = base(ProfileSide::Outside, 0.0); p.tabs.length = 2; p.tabs.count = 2;
        char fn[256]; snprintf(fn, sizeof fn, "%s/strip_t%.3f.scn", O.c_str(), t); writeScene(fn, cs, p, 2.0, 0); (void)g;
    }
    // 3. nested: plate (60x40) with a hole (circle 12), a round part (circle 8) in the hole with its own hole (circle 3):
    //    Outside/Inside, tabs fitting only the plate (TL 30 > the small circles' share) -> part and slug free; attribution.
    for (int side = 0; side < 2; ++side) for (int sep = 0; sep < 2; ++sep) {
        std::vector<Contour> cs{Contour::rectangle(geometry::Box2{{0, 0}, {60, 40}}), Contour::circle({30, 20}, 12), Contour::circle({30, 20}, 7), Contour::circle({30, 20}, 3)};
        ProfileParams p = base(side ? ProfileSide::Inside : ProfileSide::Outside, 0.0); p.tabs.length = 25; p.tabs.count = 2; p.separateLastPass = sep;
        char fn[256]; snprintf(fn, sizeof fn, "%s/nest_s%d_sep%d.scn", O.c_str(), side, sep); writeScene(fn, cs, p, 2.0, 0);
    }
    // 4. waste between vectors, far from every closed cut's inside: 3x3 grid of 6 mm squares 3 mm apart, D 2, Outside, tabs too long.
    {
        std::vector<Contour> cs; for (int i = 0; i < 3; ++i) for (int j = 0; j < 3; ++j) cs.push_back(Contour::rectangle(geometry::Box2{{i * 9.0, j * 9.0}, {i * 9.0 + 6, j * 9.0 + 6}}));
        ProfileParams p = base(ProfileSide::Outside, 0.3); p.tabs.length = 30;
        char fn[256]; snprintf(fn, sizeof fn, "%s/grid3.scn", O.c_str()); writeScene(fn, cs, p, 2.0, 0);
        p.tabs.length = 2; p.tabs.count = 2; snprintf(fn, sizeof fn, "%s/grid3_tabs.scn", O.c_str()); writeScene(fn, cs, p, 2.0, 0);
    }
    // 5. V-bit spikes / gear with sharp corners, all tabs left out
    for (int k = 0; k < 4; ++k) {
        rng.seed(77 + k);
        std::vector<Contour> cs{k % 2 ? spikes({0, 0}, 15, 5) : gear({0, 0}, 15, 9, 4)};
        ProfileParams p = base(k < 2 ? ProfileSide::Outside : ProfileSide::Inside, 0.0); p.passes.cutDepth = 3; p.tabs.length = 200; p.corners.sharpExternal = true; p.corners.sharpInternal = true;
        p.ramp.type = RampType(1 + k % 2); p.ramp.distance = 10;
        char fn[256]; snprintf(fn, sizeof fn, "%s/vbit_%d.scn", O.c_str(), k); writeScene(fn, cs, p, 12.0, 90);
    }
    // 6. arcs tighter than the band: tiny circles (radius 0.2..1.2) Outside with D 6 -> path arcs radius r + R > r... plus a
    //    flower of small arcs Inside.
    {
        std::vector<Contour> cs; for (int i = 0; i < 5; ++i) cs.push_back(Contour::circle({i * 7.0, 0}, 0.2 + 0.25 * i));
        ProfileParams p = base(ProfileSide::Outside, 0.0); p.tabs.length = 200;
        char fn[256]; snprintf(fn, sizeof fn, "%s/tiny_circ.scn", O.c_str()); writeScene(fn, cs, p, 6.0, 0);
    }
}
