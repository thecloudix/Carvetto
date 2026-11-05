// T-0153 round 6: property fuzz of detail::SkeletonCheck::consistent(ax, own) with SYNTHETIC skeletons.
// usage: fuzz6 <region file> <chordal> <iterations> <seed> [verbose]
// Each iteration: random own subset of the file's contours (the "part" the wrong skeleton was computed for),
// random skeleton graph (chains of branches through shared nodes, loops, isolated nodes) built from random
// walks whose step lengths straddle the net's propagation bound (r - (2c+d)): tiny, 0.5x, 0.99x, 1.01x, 3x, jumps.
// Radii = exact distance to the OWN contours (the medial-axis guarantee the net relies on for own rings).
// Oracle over ALL contours: a point with r > 2c+d that is outside the region by > c, or whose disc reaches
// past a wall by > c (exact dist < r - clear - c), while consistent()==1, is a BYPASS (printed with the case).
// env: DROP=1 claims every contour as own although radii ignore the non-own ones (sanity: must find bypasses).
// NOOWN=1 passes own={} (144e957-like: all rings measured).  BADEND=1 lets branch ends differ from their node point.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <sstream>
#include "acc.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "v_carve_check.hpp"
using namespace carvetto::geometry;
static std::vector<Contour> load(const char* f) { std::vector<Contour> cs; std::ifstream in(f); std::string line;
    while (std::getline(in, line)) { std::istringstream is(line); std::string t; is >> t;
        if (t == "contour") { cs.emplace_back(); cs.back().closed = true; }
        else if (t == "L") { double a, b, c, d; is >> a >> b >> c >> d; cs.back().append(LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double x, y, r, s0, e; int dir; std::string rr, dd; is >> x >> y >> rr >> r >> s0 >> e >> dd >> dir; cs.back().append(ArcSpan{{x, y}, r, s0, e, dir ? ArcDirection::CW : ArcDirection::CCW}); } }
    return cs; }
int main(int argc, char** argv) {
    auto cs = load(argv[1]); double ch = atof(argv[2]); int iters = atoi(argv[3]); std::mt19937_64 rng(strtoull(argv[4], nullptr, 10)); bool verbose = argc > 5;
    Tolerance inner; inner.chordal = std::max(ch / 3.0, 1e-4); inner.distance = std::min(std::min(0.001, ch), inner.chordal);
    double clear = 2 * inner.chordal + inner.distance;
    carvetto::toolpath::detail::SkeletonCheck chk(cs, inner); va::Acc ALL; ALL.build(cs);
    Box2 bb; for (auto& c : cs) for (auto& v : c.polygonise(inner).points) bb.expand(v);
    std::uniform_real_distribution<double> U(0, 1);
    auto rnd = [&](double a, double b) { return a + (b - a) * U(rng); };
    long accepted = 0, bypass = 0, total = 0; double worstOut = 0, worstDisc = 0;
    for (int it = 0; it < iters; ++it) {
        std::vector<size_t> own; std::vector<Contour> oc;
        int mode = (int)(U(rng) * 4);
        for (size_t i = 0; i < cs.size(); ++i) { bool take = mode == 0 ? true : mode == 1 ? (i == (size_t)(it % cs.size())) : U(rng) < 0.5; if (take) { own.push_back(i); oc.push_back(cs[i]); } }
        if (oc.empty()) { own.push_back(0); oc.push_back(cs[0]); }
        if (it % 500 == 0) { std::fprintf(stderr, "it %d accepted %ld bypass %ld\n", it, accepted, bypass); }
        unsigned long long key = 0; for (auto o : own) key |= 1ull << (o % 64);
        static std::map<unsigned long long, std::pair<std::unique_ptr<std::vector<Contour>>, std::unique_ptr<va::Acc>>> cache;
        auto& slot = cache[key]; if (!slot.second) { slot.first = std::make_unique<std::vector<Contour>>(oc); slot.second = std::make_unique<va::Acc>(); slot.second->build(*slot.first); }
        va::Acc& OWN = *slot.second;
        Box2 lim = bb.inflated(0.5); auto clampP = [&](Vec2 q) { return Vec2{std::clamp(q.x, lim.min.x, lim.max.x), std::clamp(q.y, lim.min.y, lim.max.y)}; };
        MedialAxis ax; int nb = 1 + (int)(U(rng) * 6);
        auto mkpt = [&](Vec2 p) { return MedialPoint{p, OWN.dist(p)}; };
        // nodes
        auto newNode = [&](Vec2 p) { ax.nodes.push_back(MedialNode{p, OWN.dist(p), {}}); return ax.nodes.size() - 1; };
        Vec2 start{rnd(bb.min.x, bb.max.x), rnd(bb.min.y, bb.max.y)};
        for (int t = 0; t < 50 && (it % 4 != 0) && !ALL.inside(start); ++t) start = Vec2{rnd(bb.min.x, bb.max.x), rnd(bb.min.y, bb.max.y)};  // 3/4 of the skeletons start inside
        size_t cur = newNode(start);
        for (int b = 0; b < nb; ++b) {
            MedialBranch br; br.startNode = cur; Vec2 p = ax.nodes[cur].point; br.points.push_back(mkpt(p));
            int len = 2 + (int)(U(rng) * 60);
            double ang = rnd(0, 2 * kPi);
            for (int k = 0; k < len; ++k) {
                double r = OWN.dist(p); double reach = std::max(r - clear, 1e-4); int s = (int)(U(rng) * 7);
                double step = s == 0 ? 1e-3 : s == 1 ? 0.5 * reach : s == 2 ? 0.99 * reach : s == 3 ? 1.01 * reach : s == 4 ? 3 * reach : s == 5 ? rnd(0, 0.2) * (bb.max.x - bb.min.x) : 0.999999 * reach;
                ang += rnd(-0.6, 0.6); p = clampP(Vec2{p.x + step * std::cos(ang), p.y + step * std::sin(ang)});
                br.points.push_back(mkpt(p));
            }
            int closeTo = (int)(U(rng) * 3);  // 0 new node, 1 back to an existing node (loop/cycle), 2 new node then jump
            size_t to;
            if (closeTo == 1 && !ax.nodes.empty()) { to = (size_t)(U(rng) * ax.nodes.size()); br.points.push_back(mkpt(ax.nodes[to].point)); }
            else to = newNode(br.points.back().point);
            if (getenv("BADEND") && U(rng) < 0.3) { br.points.back().point = Vec2{br.points.back().point.x + rnd(-1, 1), br.points.back().point.y + rnd(-1, 1)}; if (getenv("BADEND_R")) br.points.back().radius = OWN.dist(br.points.back().point); }  // t0164v1 BADEND_R=1: keep the radius contract, break only end==node
            if (getenv("BADSTART") && U(rng) < 0.3) { br.points.front().point = Vec2{br.points.front().point.x + rnd(-1, 1), br.points.front().point.y + rnd(-1, 1)}; br.points.front().radius = OWN.dist(br.points.front().point); }  // t0164v1: branch start away from its node
            br.endNode = to; br.sides.resize(br.points.size() - 1);
            ax.nodes[cur].branches.push_back(ax.branches.size()); ax.nodes[to].branches.push_back(ax.branches.size()); ax.branches.push_back(br);
            cur = (U(rng) < 0.3) ? (size_t)(U(rng) * ax.nodes.size()) : to;
        }
        int iso = (int)(U(rng) * 3); for (int k = 0; k < iso; ++k) newNode(Vec2{rnd(bb.min.x, bb.max.x), rnd(bb.min.y, bb.max.y)});
        std::vector<size_t> passOwn = own; if (getenv("DROP")) { passOwn.clear(); for (size_t i = 0; i < cs.size(); ++i) passOwn.push_back(i); }  // DROP=1: oracle sanity (silently dropped contours)
        bool ok = chk.consistent(ax, getenv("NOOWN") ? std::vector<size_t>{} : passOwn); ++total;
        if (!ok) continue; ++accepted;
        double wo = 0, wd = 0; Vec2 at{};
        auto judge = [&](Vec2 p, double r) { if (!(r > clear)) return; double d = ALL.dist(p); bool in = ALL.inside(p);
            if (!in && d > inner.chordal && d > wo) { wo = d; at = p; } if (d < r - clear - inner.chordal && (r - clear - d) > wd) { wd = r - clear - d; at = p; } };
        for (auto& br : ax.branches) for (auto& m : br.points) judge(m.point, m.radius);
        for (auto& nd : ax.nodes) if (nd.branches.empty()) judge(nd.point, nd.radius);
        if (wo > 0 || wd > 0) { ++bypass; worstOut = std::max(worstOut, wo); worstDisc = std::max(worstDisc, wd);
            if (verbose || bypass <= 3) { std::printf("BYPASS it=%d own=", it); for (auto o : own) std::printf("%zu,", o); std::printf(" outside_by=%.4f disc_past=%.4f at (%.5f,%.5f) branches=%zu nodes=%zu\n", wo, wd, at.x, at.y, ax.branches.size(), ax.nodes.size()); } }
    }
    std::printf("%s c=%g iters=%ld accepted=%ld BYPASS=%ld worstOutside=%.4f worstDisc=%.4f\n", argv[1], ch, total, accepted, bypass, worstOut, worstDisc);
}
