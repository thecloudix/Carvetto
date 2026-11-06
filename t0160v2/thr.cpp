// t0160v2 thr: threshold flip points. hex: regular hexagon (r 10), V90 F 1, end mill radius R = f * R*, R* = chordal / (1 - sin 60)
// (predicted leftover = R (1 - sin 60)); fil: 20 mm square with fillet rf, 6 mm end mill: predicted = (sqrt2 - 1)(R + d - rf) for
// rf >= d + 0.293 R (d = rho + c), threshold rf* = R + d - chordal / (sqrt2 - 1). Prints corner cuts (on - off moves) per f.
#include <cmath>
#include <cstdio>
#include <string>
#include "lo.hpp"
#include "shapes.hpp"
#include "carvetto/toolpath/v_carve.hpp"
using namespace carvetto; using namespace carvetto::geometry; using namespace carvetto::toolpath;
static Contour roundSq(double L, double rf, double rot) { Contour c; c.closed = true; double a = rf; std::vector<Vec2> P{{0, 0}, {L, 0}, {L, L}, {0, L}};
    Vec2 o[4] = {{L - a, a}, {L - a, L - a}, {a, L - a}, {a, a}}; double st[4] = {-vo::PI / 2, 0, vo::PI / 2, vo::PI};
    Vec2 pts[8] = {{a, 0}, {L - a, 0}, {L, a}, {L, L - a}, {L - a, L}, {a, L}, {0, L - a}, {0, a}};
    for (int i = 0; i < 4; ++i) { c.append(LineSpan{pts[2 * i], pts[2 * i + 1]}); c.append(ArcSpan{o[i], a, st[i], st[i] + vo::PI / 2, ArcDirection::CCW}); }
    return c.transformed(Transform2::rotation(rot)); }
int main(int argc, char** argv) {
    std::string mode = argv[1]; double chd = atof(argv[2]); double rot = argc > 3 ? atof(argv[3]) : 0.0;
    tooling::Tool t; t.id = t.name = "V"; t.type = tooling::ToolType::VBit; t.diameter = 12.7; t.includedAngle = vo::PI / 2; t.defaultCuttingData = {1, 0.3, 0, 0, 18000, 1500, 500, 0, 1}; t.defaultCuttingData.stepover = 0.3; t.defaultCuttingData.passDepth = 3;
    VCarvingParams p; p.flatDepth = 1.0; p.tolerance.chordal = chd; p.tolerance.distance = std::min(0.001, chd);
    double c = std::max(chd / 3, 1e-4), d = 1.0 + c;
    std::vector<double> fs; double f0 = getenv("THR_F0") ? atof(getenv("THR_F0")) : 0.96, f1 = getenv("THR_F1") ? atof(getenv("THR_F1")) : 1.04, fst = getenv("THR_FS") ? atof(getenv("THR_FS")) : 0.002; for (double f = f0; f <= f1 + 1e-9; f += fst) fs.push_back(f);
    for (double f : fs) {
        std::vector<Contour> cs; double R, pred;
        if (mode == "hex") { R = f * chd / (1 - std::sin(vo::PI / 3)); pred = R * (1 - std::sin(vo::PI / 3)); std::vector<Vec2> v; for (int i = 0; i < 6; ++i) v.push_back({10 * std::cos(rot + i * vo::PI / 3), 10 * std::sin(rot + i * vo::PI / 3)}); cs.push_back(vs::poly(v)); }
        else { R = 3.0; double rfs = R + d - chd / (std::sqrt(2.0) - 1); double rf = rfs + (1 - f) * chd / (std::sqrt(2.0) - 1); pred = (std::sqrt(2.0) - 1) * (R + d - rf); cs.push_back(roundSq(20, rf, rot)); }
        tooling::Tool e; e.id = e.name = "EM"; e.type = tooling::ToolType::EndMill; e.diameter = 2 * R; e.defaultCuttingData = {2, 0.8 * R, 0, 0, 18000, 1500, 500, 0, 2}; e.defaultCuttingData.passDepth = 3; e.defaultCuttingData.stepover = 0.8 * R;
        p.cornerSharpen = true; auto a = vCarvingToolpath(cs, t, std::nullopt, {PocketTool{e, std::nullopt}}, p); p.cornerSharpen = false; auto b = vCarvingToolpath(cs, t, std::nullopt, {PocketTool{e, std::nullopt}}, p);
        long extra = (long)a.toolpath.moves.size() - (long)b.toolpath.moves.size();
        double actOff = -1, actOn = -1;
        if (getenv("THR_ORACLE")) { va::Acc AC; AC.build(cs); Vec2 corner = mode == "hex" ? Vec2{10 * std::cos(rot), 10 * std::sin(rot)} : Transform2::rotation(rot).apply(Vec2{0, 0});
            for (int on = 0; on < 2; ++on) { const auto& r = on ? a : b; lo::Surface S; Vec3 st{0, 0, p.safeZ}; S.add(r.toolpath, lo::Shape{0, 6.35, 1.0}, 1.0 + 1e-9, st); S.add(r.clearance[0], lo::Shape{2, R, 0}, R, st); S.build(0.25);
                double best = 0; Vec2 bq = corner; double g = 0.01; for (int i = -600; i <= 600; ++i) for (int j = -600; j <= 600; ++j) { Vec2 q{corner.x + i * g, corner.y + j * g}; if (distance(q, corner) > 6) continue; if (!AC.inside(q) || AC.dist(q) < d + 1e-9) continue; double l = std::clamp(S.z(q) + 1.0, 0.0, 1.0); if (l > best) { best = l; bq = q; } }
                { double stp = g; Vec2 q = bq; while (stp > 1e-7) { bool imp = false; for (int k = 0; k < 8; ++k) { Vec2 qq{q.x + std::cos(k * vo::PI / 4) * stp, q.y + std::sin(k * vo::PI / 4) * stp}; if (!AC.inside(qq) || AC.dist(qq) < d + 1e-9) continue; double l = std::clamp(S.z(qq) + 1.0, 0.0, 1.0); if (l > best) { best = l; q = qq; imp = true; } } if (!imp) stp *= 0.5; } }
                (on ? actOn : actOff) = best; } }
        std::printf("%s chordal %g rot %g f %.4f pred/chordal %.5f extra moves %ld actual off/chordal %.4f on/chordal %.4f %s\n", mode.c_str(), chd, rot, f, pred / chd, extra, actOff / chd, actOn / chd, (pred > chd) != (extra > 0) ? "MISMATCH" : "");
    }
}
