// Targeted acceptance checks AC1..AC5 for T-0120 (verifier throwaway).
#include <cstdio>
#include <fstream>
#include <map>
#include <sstream>

#include "carvetto/geometry/contour_ops.hpp"
#include "oracle.hpp"

using namespace v;
using namespace carvetto::toolpath;
static int fails = 0;
#define CHECK(c, ...)                                  \
    do {                                               \
        if (!(c)) {                                    \
            ++fails;                                   \
            std::printf("  FAIL %s:%d: %s  ", __FILE__, __LINE__, #c); \
            std::printf(__VA_ARGS__);                  \
            std::printf("\n");                         \
        }                                              \
    } while (0)

static Contour rect(double x0, double y0, double x1, double y1) {
    return Contour::rectangle(geometry::Box2{{x0, y0}, {x1, y1}});
}
static double signedAreaOfLevel(const Toolpath& tp, double z) {
    // shoelace over the end points of flat moves at z (arcs approximated by their endpoints + midpoints)
    std::vector<Vec2> pts;
    for (size_t i = 1; i < tp.moves.size(); ++i) {
        auto& m = tp.moves[i];
        if ((m.kind == MoveKind::Feed || isArc(m.kind)) && std::fabs(m.to.z - z) < 1e-9 &&
            std::fabs(tp.moves[i - 1].to.z - z) < 1e-9) {
            if (pts.empty()) pts.push_back(tp.moves[i - 1].to.xy());
            if (isArc(m.kind)) {
                // add samples
                sampleMoves(Toolpath{"", {}, {tp.moves[i - 1], m}}, 0.5, [&](Vec2 p, double, size_t) { pts.push_back(p); });
            } else
                pts.push_back(m.to.xy());
        }
    }
    double a = 0;
    for (size_t k = 0; k + 1 < pts.size(); ++k) a += pts[k].x * pts[k + 1].y - pts[k + 1].x * pts[k].y;
    return a / 2;
}

static void ac1() {
    std::printf("AC1\n");
    ProfileParams p;
    p.name = "Profile 100x50";
    p.passes.cutDepth = 12;
    p.passes.passDepth = 4;
    p.safeZ = 5;
    p.tabs.enabled = true;
    p.tabs.count = 2;
    p.tabs.length = 10;
    p.tabs.thickness = 2;
    auto r = profileToolpath({rect(0, 0, 100, 50)}, endMill(6), p);
    CHECK(r.ok() && r.diagnostics.empty(), "diag %zu", r.diagnostics.size());
    FILE* f = std::fopen("ac1_actual.moves", "w");
    dump(r.toolpath, f);
    std::fclose(f);
    // tab-rise checks: all flats at -10 only on -12 pass
    int rises = 0;
    for (size_t i = 1; i < r.toolpath.moves.size(); ++i) {
        auto& m = r.toolpath.moves[i];
        if (m.kind == MoveKind::Feed && m.to.z == -10.0 && r.toolpath.moves[i - 1].to.z == -12.0) ++rises;
    }
    CHECK(rises == 2, "rises %d", rises);
}

static void ac2() {
    std::printf("AC2\n");
    for (double depth : {4.0, 12.0}) {
        ProfileParams p;
        p.side = ProfileSide::Inside;
        p.passes.cutDepth = depth;
        p.passes.passDepth = 4;
        auto r = profileToolpath({Contour::circle({5, -3}, 10)}, endMill(6), p);
        CHECK(r.ok(), "empty");
        int arcs = 0, lines = 0;
        for (size_t i = 1; i < r.toolpath.moves.size(); ++i) {
            auto& m = r.toolpath.moves[i];
            if (isArc(m.kind)) {
                ++arcs;
                double r0 = geometry::distance(r.toolpath.moves[i - 1].to.xy(), m.centre);
                double r1 = geometry::distance(m.to.xy(), m.centre);
                CHECK(std::fabs(r0 - 7) < 1e-12 && std::fabs(r1 - 7) < 1e-12, "radius %.15g %.15g", r0, r1);
                CHECK(std::fabs(m.centre.x - 5) < 1e-12 && std::fabs(m.centre.y + 3) < 1e-12, "centre");
                CHECK(m.kind == MoveKind::ArcCCW, "inside climb should be CCW");
            }
            if (m.kind == MoveKind::Feed) ++lines;
        }
        std::printf("  depth %.0f: %zu moves, %d arcs, %d feeds\n", depth, r.toolpath.moves.size(), arcs, lines);
        CHECK(arcs == (int)p.passes.levels().size() && lines == 0, "arcs %d feeds %d", arcs, lines);
    }
    for (double R : {2.9, 3.0 - 1e-9, 3.0, 3.0 + 1e-9, 3.0 + 1e-6, 3.0005, 3.002, 3.01}) {
        ProfileParams p;
        p.side = ProfileSide::Inside;
        p.passes.cutDepth = 4;
        auto r = profileToolpath({Contour::circle({0, 0}, R)}, endMill(6), p);
        std::printf("  r=%.10g: moves=%zu diags=%zu", R, r.toolpath.moves.size(), r.diagnostics.size());
        for (auto& d : r.diagnostics)
            std::printf(" [%d contour=%s '%s']", (int)d.issue, d.contour ? std::to_string(*d.contour).c_str() : "-",
                        d.message.c_str());
        double minr = 1e9, maxr = 0;
        for (size_t i = 1; i < r.toolpath.moves.size(); ++i) {
            auto& m = r.toolpath.moves[i];
            if (m.to.z < 0) {
                minr = std::min(minr, std::hypot(m.to.x, m.to.y));
                maxr = std::max(maxr, std::hypot(m.to.x, m.to.y));
            }
        }
        if (r.ok()) std::printf(" cutRadius[%.3g,%.3g]", minr, maxr);
        std::printf("\n");
        CHECK(r.ok() || !r.diagnostics.empty(), "empty without reason");
        if (R <= 3.0) CHECK(!r.ok(), "should be empty");
        if (R <= 3.0 && !r.ok()) CHECK(r.has(ProfileIssue::ToolTooLarge), "reason should be ToolTooLarge");
        if (r.ok()) CHECK(maxr <= R - 3.0 + 1e-6, "cut radius %.9g exceeds R-3", maxr);
    }
}

static void ac3() {
    std::printf("AC3\n");
    for (auto side : {ProfileSide::Outside, ProfileSide::Inside}) {
        double aC = 0, aV = 0;
        for (auto dir : {CutDirection::Climb, CutDirection::Conventional}) {
            ProfileParams p;
            p.side = side;
            p.direction = dir;
            p.passes.cutDepth = 4;
            auto r = profileToolpath({rect(0, 0, 100, 50)}, endMill(6), p);
            double a = signedAreaOfLevel(r.toolpath, -4);
            (dir == CutDirection::Climb ? aC : aV) = a;
        }
        std::printf("  side %d: climb area %.3f, conventional area %.3f\n", (int)side, aC, aV);
        if (side == ProfileSide::Outside) CHECK(aC < 0 && aV > 0, "outside climb should be CW");
        else CHECK(aC > 0 && aV < 0, "inside climb should be CCW");
        CHECK(std::fabs(aC + aV) < 1e-6, "areas not opposite");
    }
    // separate last pass
    for (bool rev : {false, true}) {
        for (auto side : {ProfileSide::Outside, ProfileSide::Inside}) {
            ProfileParams p;
            p.side = side;
            p.passes.cutDepth = 12;
            p.passes.passDepth = 4;
            p.separateLastPass = true;
            p.lastPassAllowance = 0.2;
            p.lastPassReverse = rev;
            Contour c = rect(0, 0, 100, 50);
            auto r = profileToolpath({c}, endMill(6), p);
            Obstacles ob({c});
            // classify each loop by retract: segments between rapids
            std::vector<std::pair<double, double>> loops;  // min,max distance at depth
            std::vector<double> loopZ;
            std::vector<double> areas;
            double mn = 1e9, mx = 0, zmin = 1e9;
            std::vector<Move> cur;
            for (size_t i = 1; i < r.toolpath.moves.size(); ++i) {
                auto& m = r.toolpath.moves[i];
                if (m.kind == MoveKind::Rapid) continue;
                if (m.kind == MoveKind::Retract) {
                    loops.push_back({mn, mx});
                    loopZ.push_back(zmin);
                    mn = 1e9, mx = 0, zmin = 1e9;
                    continue;
                }
                if (m.kind == MoveKind::Plunge) continue;
                sampleMoves(Toolpath{"", {}, {r.toolpath.moves[i - 1], m}}, 0.1, [&](Vec2 q, double, size_t) {
                    double d = ob.dist(q, 1e9);
                    mn = std::min(mn, d);
                    mx = std::max(mx, d);
                });
                zmin = std::min(zmin, m.to.z);
            }
            int finLevels = 0;
            for (auto& m : r.toolpath.moves)
                if (m.kind == MoveKind::Plunge) ++finLevels;
            std::printf("  side %d rev %d: %zu loops;", (int)side, rev, loops.size());
            for (size_t k = 0; k < loops.size(); ++k)
                std::printf(" [%.6f..%.6f z>=%.3f]", loops[k].first, loops[k].second, loopZ[k]);
            std::printf(" plunges=%d\n", finLevels);
            CHECK(loops.size() == 2, "loops");
            if (loops.size() == 2) {
                CHECK(std::fabs(loops[0].first - 3.2) < 1e-9 && std::fabs(loops[0].second - 3.2) < 1e-9, "rough at 3.2");
                CHECK(std::fabs(loops[1].first - 3.0) < 1e-9 && std::fabs(loops[1].second - 3.0) < 1e-9, "finish at 3.0");
                CHECK(loopZ[1] == -12.0, "finish at full depth");
            }
            // direction of finishing vs roughing
            // roughing level -12 and finishing both at -12: split
            std::vector<Move> rough, fin;
            bool second = false;
            for (size_t i = 0; i < r.toolpath.moves.size(); ++i) {
                if (i > 0 && r.toolpath.moves[i].kind == MoveKind::Rapid) second = true;
                (second ? fin : rough).push_back(r.toolpath.moves[i]);
            }
            double ar = signedAreaOfLevel(Toolpath{"", {}, rough}, -12), af = signedAreaOfLevel(Toolpath{"", {}, fin}, -12);
            std::printf("    rough area %.2f fin area %.2f\n", ar, af);
            CHECK((ar * af < 0) == rev, "finishing direction reverse=%d", rev);
        }
    }
}

// XY distance travelled from move index a (start) up to the end of move b
static void ac4() {
    std::printf("AC4\n");
    // Smooth ramp 20 mm
    for (int startCase = 0; startCase < 3; ++startCase) {
        ProfileParams p;
        p.passes.cutDepth = 12;
        p.passes.passDepth = 4;
        p.ramp.type = RampType::Smooth;
        p.ramp.distance = 20;
        Contour c = rect(0, 0, 100, 50);
        if (startCase == 1) { p.startAt = StartAt::ClosestOnBoundingBox; p.startAnchor = BoxAnchor::CentreRight; }
        if (startCase == 2) { c = Contour::circle({0, 0}, 15); }
        auto r = profileToolpath({c}, endMill(6), p);
        const auto& mv = r.toolpath.moves;
        // walk: track XY distance since each level's descent starts
        std::vector<double> levels = {-4, -8, -12};
        double ztop = 0;
        size_t lvl = 0;
        double s = 0;
        bool desc = false;
        double zprev = 0;
        for (size_t i = 1; i < mv.size(); ++i) {
            auto& m = mv[i];
            double z0 = mv[i - 1].to.z;
            if (m.kind == MoveKind::Rapid || m.kind == MoveKind::Retract) continue;
            if (m.kind == MoveKind::Plunge) {
                CHECK(mv[i - 1].to.xy() == m.to.xy(), "plunge not vertical");
                if (lvl == 0) CHECK(m.to.z == ztop, "first plunge should stop at top, got %.6f", m.to.z);
                continue;
            }
            if (!desc && m.to.z < z0) { desc = true; s = 0; zprev = z0; }
            if (desc) {
                s += xyLen(r.toolpath, i);
                double expect = zprev - (zprev - levels[lvl]) * std::min(1.0, s / 20.0);
                CHECK(std::fabs(m.to.z - expect) < 1e-9, "case %d lvl %zu: z %.9f expect %.9f at s=%.9f", startCase, lvl, m.to.z,
                      expect, s);
                if (m.to.z <= levels[lvl] + 1e-12) {
                    CHECK(std::fabs(s - 20.0) < 1e-9, "case %d: reached level %.0f at s=%.12f", startCase, levels[lvl], s);
                    std::printf("  case %d: level %.0f reached at s=%.12f\n", startCase, levels[lvl], s);
                    desc = false;
                    ++lvl;
                    if (lvl == levels.size()) break;
                }
            }
        }
        CHECK(lvl == 3, "levels ramped %zu", lvl);
        // total flat travel at each level must cover the full loop
        double P = 0;
        {
            ArcLengthPath w(c, geometry::Tolerance::defaults());
            P = w.length() + 6 * PI * (startCase == 2 ? 1 : 1);  // offset perimeter
            if (startCase == 2) P = 2 * PI * 18;
        }
        double flat12 = 0;
        for (size_t i = 1; i < mv.size(); ++i)
            if (mv[i].to.z == -12 && mv[i - 1].to.z == -12 && mv[i].kind != MoveKind::Plunge) flat12 += xyLen(r.toolpath, i);
        std::printf("    flat at -12: %.6f, loop %.6f\n", flat12, P);
        CHECK(flat12 >= P - 1e-6, "bottom level must cover the loop");
    }
    // Smooth by angle
    {
        ProfileParams p;
        p.passes.cutDepth = 4;
        p.passes.passDepth = 4;
        p.ramp.type = RampType::Smooth;
        p.ramp.useAngle = true;
        p.ramp.angle = 10 * PI / 180;
        auto r = profileToolpath({rect(0, 0, 100, 50)}, endMill(6), p);
        double s = 0;
        for (size_t i = 1; i < r.toolpath.moves.size(); ++i) {
            auto& m = r.toolpath.moves[i];
            if (m.kind == MoveKind::Plunge || m.kind == MoveKind::Rapid) continue;
            s += xyLen(r.toolpath, i);
            if (m.to.z <= -4) break;
        }
        std::printf("  angle ramp: reached -4 after %.9f (expect %.9f)\n", s, 4 / std::tan(10 * PI / 180));
        CHECK(std::fabs(s - 4 / std::tan(10 * PI / 180)) < 1e-9, "angle ramp length");
    }
    // Spiral
    for (int cs = 0; cs < 2; ++cs) {
        ProfileParams p;
        p.passes.cutDepth = 10;
        p.passes.passDepth = 2.5;
        p.ramp.type = RampType::Spiral;
        Contour c = cs == 0 ? Contour::circle({0, 0}, 20) : rect(0, 0, 40, 30);
        double P = cs == 0 ? 2 * PI * 23 : 140 + 6 * PI;
        auto r = profileToolpath({c}, endMill(6), p);
        const auto& mv = r.toolpath.moves;
        double s = 0;
        std::vector<std::pair<double, double>> sz;  // cumulative s, z
        for (size_t i = 1; i < mv.size(); ++i) {
            auto& m = mv[i];
            if (m.kind == MoveKind::Rapid || m.kind == MoveKind::Retract) continue;
            if (m.kind == MoveKind::Plunge) {
                CHECK(m.to.z == 0.0, "spiral: only the first plunge to top expected, got %.6f", m.to.z);
                continue;
            }
            s += xyLen(r.toolpath, i);
            sz.push_back({s, m.to.z});
        }
        // z(s) should be max(-10, -2.5*s/P) up to s=4P, then flat for one revolution
        double maxErr = 0;
        for (auto& [ss, z] : sz) {
            double e = std::max(-10.0, -2.5 * ss / P);
            maxErr = std::max(maxErr, std::fabs(z - e));
        }
        std::printf("  spiral case %d: total XY %.6f (expect %.6f = 5 revs), max z err %.3g, moves %zu\n", cs, s, 5 * P, maxErr,
                    mv.size());
        CHECK(std::fabs(s - 5 * P) < 1e-6, "spiral total length");
        CHECK(maxErr < 1e-9, "spiral descent per revolution");
    }
    // Leads: off-material and min distance >= r to every contour
    for (int lt = 0; lt < 2; ++lt)
        for (auto side : {ProfileSide::Outside, ProfileSide::Inside})
            for (int nb = 0; nb < 2; ++nb) {
                ProfileParams p;
                p.side = side;
                p.passes.cutDepth = 6;
                p.passes.passDepth = 3;
                p.leads.type = lt == 0 ? LeadType::Straight : LeadType::Circular;
                p.leads.length = 8;
                p.leads.radius = 8;
                p.leads.leadOut = true;
                p.leads.overcut = 2;
                std::vector<Contour> cs = {rect(0, 0, 60, 40)};
                p.startAt = StartAt::ClosestOnBoundingBox;
                p.startAnchor = BoxAnchor::BottomCentre;
                if (nb) {
                    // neighbour near the start (outside: below-left; inside: a small island near the start)
                    if (side == ProfileSide::Outside) cs.push_back(rect(-14, -16, -9, -9));
                    else cs.push_back(rect(25, 5, 35, 9));
                }
                auto r = profileToolpath(cs, endMill(6), p);
                Obstacles ob(cs);
                double md = 1e9;
                bool wrong = false;
                sampleMoves(r.toolpath, 0.01, [&](Vec2 q, double z, size_t) {
                    if (z >= 0) return;
                    md = std::min(md, ob.dist(q, 1e9));
                    int d = ob.depth(q);
                    // outer rect is depth0 region; with neighbour as island (inside case) depth 2 is not material
                    bool inMaterial = (d % 2) == 1;
                    if (side == ProfileSide::Outside && inMaterial) wrong = true;
                    if (side == ProfileSide::Inside && !inMaterial) wrong = true;
                });
                Vec2 entry = r.toolpath.moves[0].to.xy();
                int ed = ob.depth(entry);
                std::printf("  lead %d side %d nb %d: entry (%.3f,%.3f) depth %d, min dist %.6f, wrongSide %d, diags:", lt,
                            (int)side, nb, entry.x, entry.y, ed, md, wrong);
                for (auto& d : r.diagnostics) std::printf(" %d", (int)d.issue);
                std::printf("\n");
                CHECK(md >= 3 - 1e-3, "lead gouge %.6f", md);
                CHECK(!wrong, "lead on wrong side");
                CHECK(side == ProfileSide::Outside ? ed % 2 == 0 : ed % 2 == 1, "entry on material");
                // entry must differ from the cut start (lead exists) when nb==0
                if (!nb) CHECK(r.diagnostics.empty(), "unexpected diagnostics");
            }
}

// map each cut (rapid) to the nearest square index
static std::vector<int> cutOrder(const Toolpath& tp, const std::vector<Contour>& cs) {
    std::vector<int> out;
    for (size_t i = 0; i < tp.moves.size(); ++i) {
        if (tp.moves[i].kind != MoveKind::Rapid) continue;
        Vec2 q = tp.moves[i].to.xy();
        int best = -1;
        double bd = 1e9;
        for (size_t k = 0; k < cs.size(); ++k) {
            double d = Obstacles({cs[k]}).dist(q, 1e9);
            if (d < bd) { bd = d; best = (int)k; }
        }
        out.push_back(best);
    }
    return out;
}

static void ac5() {
    std::printf("AC5\n");
    // 3x3 grid of 10x10 squares spaced 25, given in a scrambled selection order
    std::vector<int> perm = {4, 8, 0, 6, 2, 7, 1, 5, 3};  // selection index -> grid cell
    std::vector<Contour> cs;
    for (int cell : perm) {
        int col = cell % 3, row = cell / 3;
        cs.push_back(rect(col * 25.0, row * 25.0, col * 25.0 + 10, row * 25.0 + 10));
    }
    auto cellOf = [&](int sel) { return perm[(size_t)sel]; };
    struct Mode { const char* name; OrderOptions o; std::vector<int> expectCells; };
    std::vector<Mode> modes;
    OrderOptions o;
    o = {}; o.selectionOrder = true; modes.push_back({"selection", o, perm});
    o = {}; o.leftToRight = true; modes.push_back({"leftToRight", o, {0, 3, 6, 1, 4, 7, 2, 5, 8}});
    o = {}; o.bottomToTop = true; modes.push_back({"bottomToTop", o, {0, 1, 2, 3, 4, 5, 6, 7, 8}});
    o = {}; o.grid = true; modes.push_back({"grid", o, {0, 1, 2, 5, 4, 3, 6, 7, 8}});
    o = {}; modes.push_back({"none(nearest from 0,0)", o, {}});
    o = {}; o.selectionOrder = true; o.grid = true; modes.push_back({"selection+grid", o, {0, 1, 2, 5, 4, 3, 6, 7, 8}});
    for (auto& m : modes) {
        ProfileParams p;
        p.passes.cutDepth = 3;
        p.order = m.o;
        auto r = profileToolpath(cs, endMill(3), p);
        auto ord = cutOrder(r.toolpath, cs);
        std::printf("  %-24s cells:", m.name);
        std::vector<int> cells;
        for (int s : ord) { cells.push_back(cellOf(s)); std::printf(" %d", cellOf(s)); }
        std::printf("\n");
        if (!m.expectCells.empty()) CHECK(cells == m.expectCells, "order mismatch for %s", m.name);
        else {
            // greedy nearest neighbour from (0,0) using cut start points
            std::vector<Vec2> starts;
            for (auto& mv : r.toolpath.moves) if (mv.kind == MoveKind::Rapid) starts.push_back(mv.to.xy());
            Vec2 pos{0, 0};
            bool ok = true;
            for (size_t k = 0; k < starts.size(); ++k) {
                double dk = geometry::distance(pos, starts[k]);
                for (size_t j = k + 1; j < starts.size(); ++j) if (geometry::distance(pos, starts[j]) < dk - 1e-9) ok = false;
                pos = starts[k];
            }
            CHECK(ok, "nearest-neighbour order not greedy");
        }
    }
    // nested: outer 200x150, hole 20..180 x 20..130, island 40..160 x 40..110 , small hole 60..80 x 60..80, sibling island 100..140 x 60..90 , separate square 220..240
    std::vector<Contour> n = {rect(0, 0, 200, 150), rect(20, 20, 180, 130), rect(40, 40, 160, 110), rect(60, 60, 80, 80),
                              rect(100, 60, 140, 90), rect(220, 0, 240, 20), rect(115, 70, 125, 80)};
    // parent tree: 1<-0, 2<-1, 3<-2, 4<-2, 6<-4, 5 root
    std::vector<int> parent = {-1, 0, 1, 2, 2, -1, 4};
    for (auto side : {ProfileSide::Outside, ProfileSide::Inside})
        for (int mode = 0; mode < 5; ++mode) {
            ProfileParams p;
            p.side = side;
            p.passes.cutDepth = 3;
            if (mode == 1) p.order.selectionOrder = true;
            if (mode == 2) p.order.leftToRight = true;
            if (mode == 3) p.order.bottomToTop = true;
            if (mode == 4) p.order.grid = true;
            auto r = profileToolpath(n, endMill(6), p);
            auto ord = cutOrder(r.toolpath, n);
            std::printf("  nested side %d mode %d:", (int)side, mode);
            for (int s : ord) std::printf(" %d", s);
            std::printf("  diags %zu\n", r.diagnostics.size());
            CHECK(ord.size() == n.size(), "cut count %zu", ord.size());
            // every vector before its ancestors
            std::vector<int> pos(n.size(), -1);
            for (size_t k = 0; k < ord.size(); ++k) pos[(size_t)ord[k]] = (int)k;
            for (size_t k = 0; k < n.size(); ++k)
                for (int a = parent[k]; a >= 0; a = parent[(size_t)a])
                    CHECK(pos[k] < pos[(size_t)a], "vector %zu after ancestor %d", k, a);
        }
}

int main() {
    ac1();
    ac2();
    ac3();
    ac4();
    ac5();
    std::printf("TOTAL FAILS %d\n", fails);
    return fails != 0;
}
