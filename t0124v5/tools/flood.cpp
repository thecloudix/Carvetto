// v4 verifier throwaway: whole-scene material raster at the tab-top layer. Floods uncut cells from a
// part seed point and reports whether a waste seed point is reachable. End mills and V-bits.
// usage: flood scene.scn sx sy wx wy [cell] [x0 y0 x1 y1]
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <algorithm>
#include <array>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "scene_io.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Vec2;
static const double PI = 3.14159265358979323846;
struct S { double ax, ay, az, bx, by, bz; };
int main(int argc, char** argv) {
    std::vector<geometry::Contour> cs; ProfileParams p; double D; int vd = 0;
    readScene(argv[1], cs, p, D, vd);
    Vec2 sp{atof(argv[2]), atof(argv[3])}, wp{atof(argv[4]), atof(argv[5])};
    double cell = argc > 6 ? atof(argv[6]) : 0.02;
    tooling::Tool t; t.id = "t"; t.name = "t"; t.type = vd ? tooling::ToolType::VBit : tooling::ToolType::EndMill; t.diameter = D;
    if (vd) t.includedAngle = vd * PI / 180;
    t.defaultCuttingData.passDepth = 4; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.spindleSpeed = 18000; t.defaultCuttingData.toolNumber = 1;
    auto res = profileToolpath(cs, t, p);
    double bottom = p.passes.bottomZ(), zTab = bottom + std::min(p.tabs.thickness, p.passes.cutDepth), h = zTab - 1e-4;
    double R = D / 2, tanHalf = vd ? std::tan(vd * PI / 360) : 0;
    // densify all non-rapid moves into short segments
    std::vector<S> segs; double px = 0, py = 0, pz = 0; bool have = false;
    for (auto& m : res.toolpath.moves) {
        if (!have) { px = m.to.x; py = m.to.y; pz = m.to.z; have = true; continue; }
        std::vector<std::array<double,3>> pts;
        if (isArc(m.kind)) {
            double r0 = std::hypot(px - m.centre.x, py - m.centre.y);
            double a0 = std::atan2(py - m.centre.y, px - m.centre.x), a1 = std::atan2(m.to.y - m.centre.y, m.to.x - m.centre.x);
            double sw = m.kind == MoveKind::ArcCCW ? a1 - a0 : a0 - a1; sw = std::fmod(sw, 2 * PI); if (sw < 0) sw += 2 * PI; if (sw < 1e-12) sw = 2 * PI;
            if (m.kind == MoveKind::ArcCW) sw = -sw;
            int n = std::max(2, (int)std::ceil(std::fabs(sw) * r0 / 0.01));
            for (int k = 1; k <= n; ++k) { double a = a0 + sw * k / n; pts.push_back({m.centre.x + r0 * std::cos(a), m.centre.y + r0 * std::sin(a), pz + (m.to.z - pz) * k / n}); }
            pts.back() = {m.to.x, m.to.y, m.to.z};
        } else pts.push_back({m.to.x, m.to.y, m.to.z});
        for (auto& q : pts) {
            if (m.kind != MoveKind::Rapid && m.kind != MoveKind::Dwell) segs.push_back({px, py, pz, q[0], q[1], q[2]});
            px = q[0]; py = q[1]; pz = q[2];
        }
    }
    double x0, y0, x1, y1;
    if (argc > 10) { x0 = atof(argv[7]); y0 = atof(argv[8]); x1 = atof(argv[9]); y1 = atof(argv[10]); }
    else { x0 = y0 = 1e18; x1 = y1 = -1e18; for (auto& s : segs) { x0 = std::min({x0, s.ax, s.bx}); y0 = std::min({y0, s.ay, s.by}); x1 = std::max({x1, s.ax, s.bx}); y1 = std::max({y1, s.ay, s.by}); }
           x0 -= R + 2; y0 -= R + 2; x1 += R + 2; y1 += R + 2; }
    int nx = (int)std::ceil((x1 - x0) / cell), ny = (int)std::ceil((y1 - y0) / cell);
    std::vector<char> cut((size_t)nx * ny, 0);
    long nseg = 0;
    for (auto& s0 : segs) {
        S s = s0;
        if (s.az >= h && s.bz >= h) continue;
        if (s.az >= h || s.bz >= h) { double t2 = (h - s.az) / (s.bz - s.az); double cx = s.ax + (s.bx - s.ax) * t2, cy = s.ay + (s.by - s.ay) * t2; if (s.az >= h) { s.ax = cx; s.ay = cy; s.az = h; } else { s.bx = cx; s.by = cy; s.bz = h; } }
        double L = std::hypot(s.bx - s.ax, s.by - s.ay); int n = std::max(1, (int)std::ceil(L / 0.01));
        for (int k = 0; k <= n; ++k) {  // stamp discs along the segment (dense)
            double tt = (double)k / n, cx = s.ax + (s.bx - s.ax) * tt, cy = s.ay + (s.by - s.ay) * tt, cz = s.az + (s.bz - s.az) * tt;
            double rr = vd ? std::min(R, (h - cz) * tanHalf) : R;
            if (rr <= 0) continue;
            int i0 = std::max(0, (int)std::floor((cx - rr - x0) / cell)), i1 = std::min(nx - 1, (int)std::ceil((cx + rr - x0) / cell));
            int j0 = std::max(0, (int)std::floor((cy - rr - y0) / cell)), j1 = std::min(ny - 1, (int)std::ceil((cy + rr - y0) / cell));
            for (int j = j0; j <= j1; ++j) for (int i = i0; i <= i1; ++i) { double qx = x0 + (i + 0.5) * cell - cx, qy = y0 + (j + 0.5) * cell - cy; if (qx * qx + qy * qy <= rr * rr) cut[(size_t)j * nx + i] = 1; }
        }
        ++nseg;
    }
    auto idx = [&](Vec2 q) { int i = (int)((q.x - x0) / cell), j = (int)((q.y - y0) / cell); return (size_t)j * nx + i; };
    if (sp.x < x0 || sp.x > x1 || sp.y < y0 || sp.y > y1 || wp.x < x0 || wp.x > x1 || wp.y < y0 || wp.y > y1) { std::printf("seed/waste point outside grid [%.2f,%.2f]x[%.2f,%.2f]\n", x0, x1, y0, y1); return 2; }
    size_t s0 = idx(sp), w0 = idx(wp);
    std::printf("grid %dx%d segs %ld seedCut %d wasteCut %d\n", nx, ny, nseg, cut[s0], cut[w0]);
    std::vector<char> vis((size_t)nx * ny, 0); std::vector<size_t> st{s0}; vis[s0] = 1; long cnt = 0; bool reach = false;
    while (!st.empty()) { size_t k = st.back(); st.pop_back(); ++cnt; if (k == w0) reach = true; int i = (int)(k % nx), j = (int)(k / nx);
        for (int dj = -1; dj <= 1; ++dj) for (int di = -1; di <= 1; ++di) { int ii = i + di, jj = j + dj; if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue; size_t kk = (size_t)jj * nx + ii; if (vis[kk] || cut[kk]) continue; vis[kk] = 1; st.push_back(kk); } }
    int kept = 0; for (auto& d : res.diagnostics) std::printf("diag %d %s\n", (int)d.issue, d.message.c_str());
    (void)kept;
    std::printf("part region cells %ld (area %.2f mm2) -> waste reachable: %s\n", cnt, cnt * cell * cell, reach ? "YES (held)" : "NO (part free)");
    if (std::getenv("MAP")) { int st2 = atoi(std::getenv("MAP")); for (int j = ny - 1; j >= 0; j -= st2) { for (int i = 0; i < nx; i += st2) { size_t k = (size_t)j * nx + i; putchar(vis[k] ? 'P' : cut[k] ? '#' : '.'); } putchar('\n'); } }
}
