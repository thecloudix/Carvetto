#include "carvetto/io/bitmap_trace.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace carvetto::io {

using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

namespace {

constexpr double kFitTol = 0.5;  // pixels

// ------------------------------------------------------------------ masks

using Mask = std::vector<std::uint8_t>;  // 1 = foreground, row-major, width*height

/// Removes foreground components (4-connected) and background holes (8-connected, not touching
/// the border) smaller than `minPixels`.
void noiseFilter(Mask& m, int w, int h, int minPixels) {
    if (minPixels <= 1) return;
    const std::size_t n = m.size();
    std::vector<std::uint8_t> seen(n, 0);
    std::vector<std::size_t> stack, comp;
    for (int pass = 0; pass < 2; ++pass) {
        const std::uint8_t want = pass == 0 ? 1 : 0;
        std::fill(seen.begin(), seen.end(), std::uint8_t{0});
        for (std::size_t s = 0; s < n; ++s) {
            if (seen[s] || m[s] != want) continue;
            comp.clear();
            stack.assign(1, s);
            seen[s] = 1;
            bool border = false;
            while (!stack.empty()) {
                const std::size_t i = stack.back();
                stack.pop_back();
                comp.push_back(i);
                const int x = static_cast<int>(i % static_cast<std::size_t>(w));
                const int y = static_cast<int>(i / static_cast<std::size_t>(w));
                if (x == 0 || y == 0 || x == w - 1 || y == h - 1) border = true;
                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        if (pass == 0 && dx != 0 && dy != 0) continue;  // fg: 4-neighbours only
                        const int nx = x + dx, ny = y + dy;
                        if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
                        const std::size_t j = static_cast<std::size_t>(ny) * static_cast<std::size_t>(w) + static_cast<std::size_t>(nx);
                        if (seen[j] || m[j] != want) continue;
                        seen[j] = 1;
                        stack.push_back(j);
                    }
                }
            }
            if (static_cast<long long>(comp.size()) < minPixels && (pass == 0 || !border)) {
                for (std::size_t i : comp) m[i] = static_cast<std::uint8_t>(1 - want);
            }
        }
    }
}

// ------------------------------------------------------------------ quantiser (median cut)

struct Bin {
    int r, g, b;  // 5-bit
    std::size_t count;
    int box;
};

struct Palette {
    std::vector<std::array<std::uint8_t, 3>> colour;
    std::vector<int> binBox;  // 32768 entries -> palette index
};

Palette medianCut(const RasterImage& img, int colours) {
    std::vector<std::size_t> hist(32768, 0);
    const std::size_t n = static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height);
    for (std::size_t i = 0; i < n; ++i) {
        const std::size_t k = (static_cast<std::size_t>(img.rgb[i * 3] >> 3) << 10) |
                              (static_cast<std::size_t>(img.rgb[i * 3 + 1] >> 3) << 5) |
                              static_cast<std::size_t>(img.rgb[i * 3 + 2] >> 3);
        ++hist[k];
    }
    std::vector<Bin> bins;
    for (std::size_t k = 0; k < hist.size(); ++k) {
        if (hist[k]) bins.push_back({static_cast<int>(k >> 10), static_cast<int>((k >> 5) & 31), static_cast<int>(k & 31), hist[k], 0});
    }
    struct Box { std::size_t lo, hi; };  // range in `bins` (sorted in place per split)
    std::vector<Box> boxes{{0, bins.size()}};
    auto chan = [](const Bin& b, int c) { return c == 0 ? b.r : (c == 1 ? b.g : b.b); };
    while (static_cast<int>(boxes.size()) < colours) {
        // Split the box with the widest channel range (ties: first), if any can be split.
        int bi = -1, bc = 0, bestRange = 0;
        for (std::size_t i = 0; i < boxes.size(); ++i) {
            for (int c = 0; c < 3; ++c) {
                int lo = 99, hi = -1;
                for (std::size_t j = boxes[i].lo; j < boxes[i].hi; ++j) {
                    lo = std::min(lo, chan(bins[j], c));
                    hi = std::max(hi, chan(bins[j], c));
                }
                if (hi - lo > bestRange) { bestRange = hi - lo; bi = static_cast<int>(i); bc = c; }
            }
        }
        if (bi < 0) break;
        Box b = boxes[static_cast<std::size_t>(bi)];
        std::stable_sort(bins.begin() + static_cast<std::ptrdiff_t>(b.lo), bins.begin() + static_cast<std::ptrdiff_t>(b.hi),
                         [&](const Bin& x, const Bin& y) { return chan(x, bc) < chan(y, bc); });
        std::size_t total = 0;
        for (std::size_t j = b.lo; j < b.hi; ++j) total += bins[j].count;
        std::size_t acc = 0, cut = b.lo + 1;
        for (std::size_t j = b.lo; j + 1 < b.hi; ++j) {
            acc += bins[j].count;
            cut = j + 1;
            if (acc * 2 >= total) break;
        }
        // Keep equal channel values together so the cut separates distinct values.
        while (cut < b.hi && chan(bins[cut], bc) == chan(bins[cut - 1], bc)) ++cut;
        if (cut >= b.hi) {
            cut = b.lo + 1;
            while (chan(bins[cut], bc) == chan(bins[cut - 1], bc)) ++cut;
        }
        boxes[static_cast<std::size_t>(bi)] = {b.lo, cut};
        boxes.push_back({cut, b.hi});
    }
    Palette pal;
    pal.binBox.assign(32768, 0);
    for (std::size_t i = 0; i < boxes.size(); ++i) {
        double sr = 0, sg = 0, sb = 0, tot = 0;
        for (std::size_t j = boxes[i].lo; j < boxes[i].hi; ++j) {
            const double c = static_cast<double>(bins[j].count);
            sr += bins[j].r * c; sg += bins[j].g * c; sb += bins[j].b * c; tot += c;
            pal.binBox[static_cast<std::size_t>((bins[j].r << 10) | (bins[j].g << 5) | bins[j].b)] = static_cast<int>(i);
        }
        auto to8 = [&](double v) { return static_cast<std::uint8_t>(std::min(255.0, std::floor(v / tot * 8.0 + 4.0))); };
        pal.colour.push_back({to8(sr), to8(sg), to8(sb)});
    }
    return pal;
}

// ------------------------------------------------------------------ boundary loops

/// Lattice loops (x,y in pixel-corner coordinates, y down), foreground on the left of travel.
std::vector<std::vector<std::pair<int, int>>> boundaryLoops(const Mask& m, int w, int h, bool& tooMany) {
    auto fg = [&](int x, int y) { return x >= 0 && y >= 0 && x < w && y < h && m[static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)] != 0; };
    const int vw = w + 1, vh = h + 1;
    std::vector<std::uint8_t> out(static_cast<std::size_t>(vw) * static_cast<std::size_t>(vh), 0);  // bit d: unused edge leaving in dir d
    auto vi = [&](int x, int y) { return static_cast<std::size_t>(y) * static_cast<std::size_t>(vw) + static_cast<std::size_t>(x); };
    for (int y = 0; y <= h; ++y) {
        for (int x = 0; x <= w; ++x) {
            std::uint8_t b = 0;
            if (fg(x, y - 1) && !fg(x, y)) b |= 1;          // E
            if (fg(x, y) && !fg(x - 1, y)) b |= 2;          // S
            if (fg(x - 1, y) && !fg(x - 1, y - 1)) b |= 4;  // W
            if (fg(x - 1, y - 1) && !fg(x, y - 1)) b |= 8;  // N
            out[vi(x, y)] = b;
        }
    }
    static constexpr int dx[4] = {1, 0, -1, 0};
    static constexpr int dy[4] = {0, 1, 0, -1};
    std::vector<std::vector<std::pair<int, int>>> loops;
    for (int y = 0; y <= h; ++y) {
        for (int x = 0; x <= w; ++x) {
            for (int d0 = 0; d0 < 4; ++d0) {
                if (!(out[vi(x, y)] & (1 << d0))) continue;
                if (loops.size() >= kTraceMaxContours) { tooMany = true; return loops; }
                std::vector<std::pair<int, int>> loop;
                int cx = x, cy = y, d = d0;
                for (;;) {
                    out[vi(cx, cy)] = static_cast<std::uint8_t>(out[vi(cx, cy)] & ~(1 << d));
                    loop.emplace_back(cx, cy);
                    cx += dx[d];
                    cy += dy[d];
                    // Left turn first, then straight, then right.
                    int nd = -1;
                    for (int t : {3, 0, 1}) {
                        const int c = (d + t) % 4;
                        if (out[vi(cx, cy)] & (1 << c)) { nd = c; break; }
                    }
                    if (nd < 0) break;  // back at the start vertex with its edge consumed
                    d = nd;
                }
                loops.push_back(std::move(loop));
            }
        }
    }
    return loops;
}

// ------------------------------------------------------------------ Bezier fitting (Schneider-style)

using Pts = std::vector<Vec2>;

Vec2 bezAt(const Vec2* c, double t) {
    const double mt = 1.0 - t;
    return c[0] * (mt * mt * mt) + c[1] * (3.0 * mt * mt * t) + c[2] * (3.0 * mt * t * t) + c[3] * (t * t * t);
}
Vec2 bezD1(const Vec2* c, double t) {
    const double mt = 1.0 - t;
    return (c[1] - c[0]) * (3.0 * mt * mt) + (c[2] - c[1]) * (6.0 * mt * t) + (c[3] - c[2]) * (3.0 * t * t);
}
Vec2 bezD2(const Vec2* c, double t) {
    const double mt = 1.0 - t;
    return (c[2] - c[1] * 2.0 + c[0]) * (6.0 * mt) + (c[3] - c[2] * 2.0 + c[1]) * (6.0 * t);
}

Vec2 unit(Vec2 v, Vec2 fallback) {
    const double l = v.length();
    return l > 1e-12 ? v / l : fallback;
}

double distToSegment(Vec2 p, Vec2 a, Vec2 b) {
    const Vec2 ab = b - a;
    const double l2 = ab.lengthSquared();
    if (l2 <= 0.0) return (p - a).length();
    const double t = std::clamp((p - a).dot(ab) / l2, 0.0, 1.0);
    return (p - (a + ab * t)).length();
}

void fitCubic(const Pts& d, std::size_t first, std::size_t last, Vec2 t1, Vec2 t2, std::vector<geometry::Span>& out,
              int depth) {
    const std::size_t n = last - first + 1;
    const Vec2 a = d[first], b = d[last];
    // Straight enough: one line span.
    {
        double worst = 0.0;
        for (std::size_t i = first + 1; i < last; ++i) worst = std::max(worst, distToSegment(d[i], a, b));
        if (n == 2 || (a != b && worst <= kFitTol * 0.5)) {
            out.push_back(LineSpan{a, b});
            return;
        }
    }
    std::vector<double> u(n, 0.0);
    for (std::size_t i = 1; i < n; ++i) u[i] = u[i - 1] + (d[first + i] - d[first + i - 1]).length();
    const double total = u[n - 1];
    if (total <= 0.0) {
        out.push_back(LineSpan{a, b});
        return;
    }
    for (auto& v : u) v /= total;

    Vec2 c[4];
    std::size_t split = (first + last) / 2;
    for (int iter = 0; iter < 5; ++iter) {
        // Least-squares alpha for the two inner control points.
        double c00 = 0, c01 = 0, c11 = 0, x0 = 0, x1 = 0;
        for (std::size_t i = 0; i < n; ++i) {
            const double t = u[i], mt = 1.0 - t;
            const double b0 = mt * mt * mt, b1 = 3 * mt * mt * t, b2 = 3 * mt * t * t, b3 = t * t * t;
            const Vec2 a1 = t1 * b1, a2 = t2 * b2;
            c00 += a1.dot(a1);
            c01 += a1.dot(a2);
            c11 += a2.dot(a2);
            const Vec2 tmp = d[first + i] - (a * (b0 + b1) + b * (b2 + b3));
            x0 += a1.dot(tmp);
            x1 += a2.dot(tmp);
        }
        const double det = c00 * c11 - c01 * c01;
        double al = 0, ar = 0;
        if (std::fabs(det) > 1e-12) {
            al = (x0 * c11 - x1 * c01) / det;
            ar = (c00 * x1 - c01 * x0) / det;
        }
        const double seg = (b - a).length();
        if (al < 1e-6 * seg || ar < 1e-6 * seg || !(al == al) || !(ar == ar)) al = ar = std::max(seg, total * 0.5) / 3.0;
        c[0] = a;
        c[1] = a + t1 * al;
        c[2] = b + t2 * ar;
        c[3] = b;
        double worst = 0.0;
        for (std::size_t i = 0; i < n; ++i) {
            const double e = (bezAt(c, u[i]) - d[first + i]).lengthSquared();
            if (e >= worst) { worst = e; split = first + i; }
        }
        if (worst <= kFitTol * kFitTol) {
            out.push_back(BezierSpan{c[0], c[1], c[2], c[3]});
            return;
        }
        if (worst > 16.0 * kFitTol * kFitTol) break;
        // Newton-Raphson reparametrisation.
        for (std::size_t i = 0; i < n; ++i) {
            const Vec2 q = bezAt(c, u[i]) - d[first + i];
            const Vec2 q1 = bezD1(c, u[i]), q2 = bezD2(c, u[i]);
            const double den = q1.dot(q1) + q.dot(q2);
            if (std::fabs(den) > 1e-12) u[i] = std::clamp(u[i] - q.dot(q1) / den, 0.0, 1.0);
        }
        for (std::size_t i = 1; i < n; ++i) u[i] = std::max(u[i], u[i - 1]);
    }
    if (depth > 60 || split <= first || split >= last) {
        split = (first + last) / 2;
        if (split <= first || split >= last) {
            out.push_back(BezierSpan{c[0], c[1], c[2], c[3]});
            return;
        }
    }
    const Vec2 ctr = unit(d[split - 1] - d[split + 1], t1);
    fitCubic(d, first, split, t1, ctr, out, depth + 1);
    fitCubic(d, split, last, ctr * -1.0, t2, out, depth + 1);
}

/// Smooths the interior of `chain` (ends fixed) with a box window truncated to stay symmetric.
Pts smoothOpen(const std::vector<std::pair<int, int>>& L, std::size_t from, std::size_t count, int k) {
    const std::size_t n = L.size();
    Pts raw(count + 1);
    for (std::size_t i = 0; i <= count; ++i) {
        const auto& q = L[(from + i) % n];
        raw[i] = {static_cast<double>(q.first), static_cast<double>(q.second)};
    }
    Pts sm = raw;
    for (std::size_t i = 1; i < count; ++i) {
        const std::size_t kk = std::min<std::size_t>({static_cast<std::size_t>(k), i, count - i});
        Vec2 sum{0, 0};
        for (std::size_t j = i - kk; j <= i + kk; ++j) sum += raw[j];
        sm[i] = sum / static_cast<double>(2 * kk + 1);
    }
    return sm;
}

Vec2 endTangent(const Pts& p, bool atStart) {
    const std::size_t m = p.size() - 1;
    const std::size_t j = std::min<std::size_t>(2, m);
    const Vec2 v = atStart ? p[j] - p[0] : p[m - j] - p[m];
    return unit(v, atStart ? p[m] - p[0] : p[0] - p[m]);
}

/// One closed contour from a lattice loop, in image-pixel coordinates (y down).
Contour fitLoop(const std::vector<std::pair<int, int>>& L, int cornerRun, int smooth) {
    const std::size_t n = L.size();
    Contour c;
    c.closed = true;
    auto dirOf = [&](std::size_t i) {
        const auto& a = L[i];
        const auto& b = L[(i + 1) % n];
        return std::pair<int, int>{b.first - a.first, b.second - a.second};
    };
    // Turning vertices and run lengths.
    std::vector<std::size_t> turn;
    for (std::size_t i = 0; i < n; ++i) {
        if (dirOf((i + n - 1) % n) != dirOf(i)) turn.push_back(i);
    }
    auto asVec = [&](std::size_t i) { return Vec2{static_cast<double>(L[i].first), static_cast<double>(L[i].second)}; };
    if (n < 16 || turn.size() < 4) {
        for (std::size_t t = 0; t < turn.size(); ++t) c.spans.push_back(LineSpan{asVec(turn[t]), asVec(turn[(t + 1) % turn.size()])});
        return c;
    }
    std::vector<std::size_t> corners;  // indices into L
    for (std::size_t t = 0; t < turn.size(); ++t) {
        const std::size_t prev = turn[(t + turn.size() - 1) % turn.size()];
        const std::size_t next = turn[(t + 1) % turn.size()];
        const std::size_t here = turn[t];
        const std::size_t lenIn = (here + n - prev) % n;
        const std::size_t lenOut = (next + n - here) % n;
        if (lenIn >= static_cast<std::size_t>(cornerRun) && lenOut >= static_cast<std::size_t>(cornerRun)) corners.push_back(here);
    }
    if (corners.empty()) {
        // Smooth closed loop: cyclic box average, split in two at 0 and n/2.
        Pts raw(n), sm(n);
        for (std::size_t i = 0; i < n; ++i) raw[i] = asVec(i);
        const long long kk = smooth;
        for (std::size_t i = 0; i < n; ++i) {
            Vec2 sum{0, 0};
            for (long long j = -kk; j <= kk; ++j) sum += raw[static_cast<std::size_t>(static_cast<long long>(i + n) + j) % n];
            sm[i] = sum / static_cast<double>(2 * kk + 1);
        }
        const std::size_t h = n / 2;
        auto tan = [&](std::size_t i) { return unit(sm[(i + 2) % n] - sm[(i + n - 2) % n], Vec2{1, 0}); };
        Pts a(sm.begin(), sm.begin() + static_cast<std::ptrdiff_t>(h) + 1);
        Pts b(sm.begin() + static_cast<std::ptrdiff_t>(h), sm.end());
        b.push_back(sm[0]);
        fitCubic(a, 0, a.size() - 1, tan(0), tan(h) * -1.0, c.spans, 0);
        fitCubic(b, 0, b.size() - 1, tan(h), tan(0) * -1.0, c.spans, 0);
        return c;
    }
    for (std::size_t ci = 0; ci < corners.size(); ++ci) {
        const std::size_t from = corners[ci];
        const std::size_t to = corners[(ci + 1) % corners.size()];
        std::size_t count = (to + n - from) % n;
        if (count == 0) count = n;  // a single corner: the chain is the whole loop
        const Pts chain = smoothOpen(L, from, count, smooth);
        fitCubic(chain, 0, chain.size() - 1, endTangent(chain, true), endTangent(chain, false), c.spans, 0);
    }
    return c;
}

geometry::Span mapSpan(const geometry::Span& s, const TraceParams& p, int h) {
    auto m = [&](Vec2 v) { return Vec2{p.origin.x + v.x * p.mmPerPixel, p.origin.y + (static_cast<double>(h) - v.y) * p.mmPerPixel}; };
    if (const auto* l = std::get_if<LineSpan>(&s)) return LineSpan{m(l->start), m(l->end)};
    const auto& b = std::get<BezierSpan>(s);
    return BezierSpan{m(b.p0), m(b.p1), m(b.p2), m(b.p3)};
}

}  // namespace

geometry::ShapeResult<std::vector<TraceLayer>> traceBitmap(const RasterImage& img, const TraceParams& p) {
    using R = geometry::ShapeResult<std::vector<TraceLayer>>;
    if (!img.valid()) return R(std::string("invalid or oversize image"));
    if (!(p.mmPerPixel > 0.0) || !std::isfinite(p.mmPerPixel) || !std::isfinite(p.origin.x) || !std::isfinite(p.origin.y)) {
        return R(std::string("mmPerPixel must be positive and the origin finite"));
    }
    if (!(p.cornerFit >= 0.0 && p.cornerFit <= 1.0)) return R(std::string("cornerFit must be in [0,1]"));
    if (p.noiseFilter < 0) return R(std::string("noiseFilter must be >= 0"));
    if (p.mode == TraceMode::Colour && (p.colourCount < 2 || p.colourCount > 16)) return R(std::string("colourCount must be 2..16"));

    const int w = img.width, h = img.height;
    const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    struct Plane { Mask mask; std::array<std::uint8_t, 3> colour; };
    std::vector<Plane> planes;
    if (p.mode == TraceMode::BlackWhite) {
        Plane pl{Mask(n, 0), {0, 0, 0}};
        for (std::size_t i = 0; i < n; ++i) {
            const unsigned luma = (299u * img.rgb[i * 3] + 587u * img.rgb[i * 3 + 1] + 114u * img.rgb[i * 3 + 2] + 500u) / 1000u;
            const bool dark = luma < p.threshold;
            pl.mask[i] = (dark != p.invert) ? 1 : 0;
        }
        planes.push_back(std::move(pl));
    } else {
        const Palette pal = medianCut(img, p.colourCount);
        for (const auto& col : pal.colour) planes.push_back({Mask(n, 0), col});
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t k = (static_cast<std::size_t>(img.rgb[i * 3] >> 3) << 10) | (static_cast<std::size_t>(img.rgb[i * 3 + 1] >> 3) << 5) |
                                  static_cast<std::size_t>(img.rgb[i * 3 + 2] >> 3);
            planes[static_cast<std::size_t>(pal.binBox[k])].mask[i] = 1;
        }
    }

    const int cornerRun = 8 - static_cast<int>(std::lround(p.cornerFit * 5.0));  // 8 loose .. 3 tight
    const int smooth = 1 + static_cast<int>(std::lround((1.0 - p.cornerFit) * 3.0));  // 4 loose .. 1 tight
    std::vector<TraceLayer> layers;
    std::size_t spanTotal = 0;
    for (auto& pl : planes) {
        noiseFilter(pl.mask, w, h, p.noiseFilter);
        TraceLayer layer;
        layer.r = pl.colour[0];
        layer.g = pl.colour[1];
        layer.b = pl.colour[2];
        for (std::uint8_t v : pl.mask) layer.pixelCount += v;
        if (layer.pixelCount == 0) continue;
        bool tooMany = false;
        const auto loops = boundaryLoops(pl.mask, w, h, tooMany);
        if (tooMany) return R(std::string("too many contours"));
        for (const auto& loop : loops) {
            Contour c = fitLoop(loop, cornerRun, smooth);
            spanTotal += c.spans.size();
            if (spanTotal > kTraceMaxSpans) return R(std::string("result too large"));
            for (auto& s : c.spans) s = mapSpan(s, p, h);
            layer.contours.push_back(std::move(c));
        }
        layers.push_back(std::move(layer));
    }
    std::stable_sort(layers.begin(), layers.end(), [](const TraceLayer& a, const TraceLayer& b) { return a.pixelCount > b.pixelCount; });
    return R(std::move(layers));
}

}  // namespace carvetto::io
