// Spatial acceleration for oracle.hpp exactDist/insideExact (same results, indexed by span boxes).
#pragma once
#include <algorithm>
#include <unordered_map>
#include "oracle.hpp"
namespace va {
using namespace carvetto::geometry;
struct Acc {
    const std::vector<Contour>* cs; struct SB { double x0, y0, x1, y1; int c, s; };
    std::vector<SB> sb; double cell = 1; std::unordered_map<long long, std::vector<int>> grid; std::vector<int> byY; // span ids
    std::vector<std::vector<int>> ybuck; double ymin = 0, yh = 1; int nyb = 1;
    static long long key(long long i, long long j) { return i * 2000003LL + j; }
    void build(const std::vector<Contour>& c) {
        cs = &c; sb.clear(); grid.clear(); double tot = 0; double gx0 = 1e300, gy0 = 1e300, gx1 = -1e300, gy1 = -1e300;
        for (int i = 0; i < (int)c.size(); ++i) for (int s = 0; s < (int)c[i].spans.size(); ++s) { const Span& sp = c[i].spans[s]; SB b{1e300, 1e300, -1e300, -1e300, i, s};
            int N = std::holds_alternative<LineSpan>(sp) ? 1 : 512; Vec2 prev{}; double sag = 0;
            for (int k = 0; k <= N; ++k) { Vec2 q = vo::evalSpan(sp, double(k) / N); b.x0 = std::min(b.x0, q.x); b.y0 = std::min(b.y0, q.y); b.x1 = std::max(b.x1, q.x); b.y1 = std::max(b.y1, q.y); if (k) sag = std::max(sag, vo::dist(q, prev)); prev = q; }
            double pad = N > 1 ? sag + 1e-9 : 1e-12; b.x0 -= pad; b.y0 -= pad; b.x1 += pad; b.y1 += pad; tot += std::max(b.x1 - b.x0, b.y1 - b.y0); sb.push_back(b);
            gx0 = std::min(gx0, b.x0); gy0 = std::min(gy0, b.y0); gx1 = std::max(gx1, b.x1); gy1 = std::max(gy1, b.y1); }
        cell = std::max(1e-9, std::max(tot / std::max<size_t>(1, sb.size()), std::max(gx1 - gx0, gy1 - gy0) / 2000.0));
        for (int k = 0; k < (int)sb.size(); ++k) { auto& b = sb[k]; for (long long i = (long long)std::floor(b.x0 / cell); i <= (long long)std::floor(b.x1 / cell); ++i) for (long long j = (long long)std::floor(b.y0 / cell); j <= (long long)std::floor(b.y1 / cell); ++j) grid[key(i, j)].push_back(k); }
        nyb = std::max<int>(1, std::min<int>(4096, (int)sb.size())); ymin = gy0; yh = std::max(1e-12, (gy1 - gy0) / nyb); ybuck.assign(nyb, {});
        for (int k = 0; k < (int)sb.size(); ++k) { int a = std::clamp((int)((sb[k].y0 - ymin) / yh), 0, nyb - 1), z = std::clamp((int)((sb[k].y1 - ymin) / yh), 0, nyb - 1); for (int q = a; q <= z; ++q) ybuck[q].push_back(k); }
    }
    vo::Foot foot(Vec2 p) const {
        vo::Foot best{1e300, {}, 0}; long long ci = (long long)std::floor(p.x / cell), cj = (long long)std::floor(p.y / cell);
        std::vector<char> seen(sb.size(), 0);
        for (long long ring = 0; ring < 4000; ++ring) {
            for (long long i = ci - ring; i <= ci + ring; ++i) for (long long j = cj - ring; j <= cj + ring; ++j) { if (std::max(std::llabs(i - ci), std::llabs(j - cj)) != ring) continue;
                auto it = grid.find(key(i, j)); if (it == grid.end()) continue;
                for (int k : it->second) { if (seen[k]) continue; seen[k] = 1; const auto& b = sb[k]; double dx = std::max({b.x0 - p.x, 0.0, p.x - b.x1}), dy = std::max({b.y0 - p.y, 0.0, p.y - b.y1}); if (std::hypot(dx, dy) >= best.d) continue;
                    vo::Foot f = vo::spanDist((*cs)[b.c].spans[b.s], p); if (f.d < best.d) best = f; } }
            if (best.d <= ring * cell) break;
        }
        return best;
    }
    double dist(Vec2 p) const { return foot(p).d; }
    // Ray parity at y shifted by tiny offsets (majority of three): a ray exactly through span end points
    // (full circles starting at angle 0 end at y = -2e-15, not 0) miscounted in round 2's insideExact.
    bool inside(Vec2 p) const { double s = 8.0 * 2.220446049250313e-16 * std::fabs(p.y) + 1e-11; int v = (int)insideH({p.x, p.y + s}) + (int)insideH({p.x, p.y - 1.37 * s}); if (v != 1) return v == 2; return insideH({p.x, p.y + 2.71 * s}); }
    bool insideH(Vec2 p) const {
        int q = (int)((p.y - ymin) / yh); if (q < 0 || q >= nyb) return false; bool in = false;
        for (int k : ybuck[q]) { const auto& b = sb[k]; if (b.y0 > p.y || b.y1 < p.y || b.x1 <= p.x) continue; const Span& s = (*cs)[b.c].spans[b.s];
            if (auto* l = std::get_if<LineSpan>(&s)) { const Vec2 &a = l->start, &e = l->end; if ((a.y > p.y) != (e.y > p.y)) { double x = a.x + (p.y - a.y) * (e.x - a.x) / (e.y - a.y); if (x > p.x) in = !in; } continue; }
            const int N = 128; Vec2 prev = vo::evalSpan(s, 0); bool sp = prev.y > p.y;
            for (int i = 1; i <= N; ++i) { double t1 = double(i) / N; Vec2 qq = vo::evalSpan(s, t1); bool sq = qq.y > p.y;
                if (sq != sp) { double lo = double(i - 1) / N, hi = t1; for (int it = 0; it < 60; ++it) { double m = 0.5 * (lo + hi); if ((vo::evalSpan(s, m).y > p.y) == sp) lo = m; else hi = m; } if (vo::evalSpan(s, 0.5 * (lo + hi)).x > p.x) in = !in; }
                prev = qq; sp = sq; } }
        return in;
    }
};
}
