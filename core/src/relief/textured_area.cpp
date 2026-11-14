#include "carvetto/relief/textured_area.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace carvetto::relief {

namespace {

struct Bounds {
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
};

bool finite(double v) { return std::isfinite(v); }

TexturedStatus bounds(const TexturedAreaParams& p, Bounds& b) {
    if (p.rings.empty()) return TexturedStatus::BadBoundary;
    bool first = true;
    for (const auto& r : p.rings) {
        if (r.size() < 3) return TexturedStatus::BadBoundary;
        for (const Point& q : r) {
            if (!finite(q.x) || !finite(q.y)) return TexturedStatus::BadBoundary;
            if (first) {
                b = {q.x, q.y, q.x, q.y};
                first = false;
            } else {
                b.x0 = std::min(b.x0, q.x);
                b.y0 = std::min(b.y0, q.y);
                b.x1 = std::max(b.x1, q.x);
                b.y1 = std::max(b.y1, q.y);
            }
        }
    }
    if (!(b.x1 > b.x0) || !(b.y1 > b.y0)) return TexturedStatus::BadBoundary;
    return TexturedStatus::Ok;
}

struct Layout {
    Bounds b;
    Point anchor;
    double w = 0, h = 0;      // tile size
    double px = 0, py = 0;    // pitch
    double shx = 0, shy = 0;  // shifts in mm
    long long r0 = 0, r1 = 0, c0 = 0, c1 = 0;  // inclusive index ranges
    double nTiles = 0;
};

TexturedStatus layout(const TexturedAreaParams& p, Layout& L) {
    if (validate(p.seed.grid()) != Status::Ok) return TexturedStatus::BadSeed;
    if (!finite(p.spacingXPercent) || !finite(p.spacingYPercent) || !finite(p.shiftXPercent) ||
        !finite(p.shiftYPercent))
        return TexturedStatus::BadParameter;
    if (p.anchor && (!finite(p.anchor->x) || !finite(p.anchor->y))) return TexturedStatus::BadParameter;
    if (auto s = bounds(p, L.b); s != TexturedStatus::Ok) return s;
    L.anchor = p.anchor ? *p.anchor : Point{L.b.x0, L.b.y0};
    const double cs = p.seed.cellSize();
    L.w = static_cast<double>(p.seed.width()) * cs;
    L.h = static_cast<double>(p.seed.height()) * cs;
    L.px = L.w * (1.0 + p.spacingXPercent / 100.0);
    L.py = L.h * (1.0 + p.spacingYPercent / 100.0);
    L.shx = p.shiftXPercent / 100.0 * L.px;
    L.shy = p.shiftYPercent / 100.0 * L.py;
    if (!(L.px > 0) || !(L.py > 0) || !finite(L.shx) || !finite(L.shy)) return TexturedStatus::BadParameter;
    const double rLo = std::floor((L.b.y0 - L.h - L.anchor.y - std::max(0.0, L.shy)) / L.py);
    const double rHi = std::ceil((L.b.y1 - L.anchor.y - std::min(0.0, L.shy)) / L.py);
    const double cLo = std::floor((L.b.x0 - L.w - L.anchor.x - std::max(0.0, L.shx)) / L.px);
    const double cHi = std::ceil((L.b.x1 - L.anchor.x - std::min(0.0, L.shx)) / L.px);
    if (!finite(rLo) || !finite(rHi) || !finite(cLo) || !finite(cHi)) return TexturedStatus::BadParameter;
    L.nTiles = (rHi - rLo + 1.0) * (cHi - cLo + 1.0);
    if (L.nTiles > static_cast<double>(kMaxTexturedTiles)) return TexturedStatus::TooManyTiles;
    L.r0 = static_cast<long long>(rLo);
    L.r1 = static_cast<long long>(rHi);
    L.c0 = static_cast<long long>(cLo);
    L.c1 = static_cast<long long>(cHi);
    return TexturedStatus::Ok;
}

bool oddIndex(long long v) { return (v & 1) != 0; }

}  // namespace

const char* texturedStatusMessage(TexturedStatus s) {
    switch (s) {
        case TexturedStatus::Ok: return "ok";
        case TexturedStatus::BadSeed: return "the seed component is empty or invalid";
        case TexturedStatus::BadBoundary:
            return "the boundary needs closed rings of at least 3 finite vertices and a non-zero extent";
        case TexturedStatus::BadParameter:
            return "spacing must be above -100 % and all values finite";
        case TexturedStatus::TooManyTiles:
            return "too many tiles: increase the spacing, use a larger seed or a smaller boundary";
        case TexturedStatus::TooLarge: return "the textured area is too large";
    }
    return "unknown error";
}

TexturedStatus validateTexturedArea(const TexturedAreaParams& p) {
    Layout L;
    return layout(p, L);
}

std::size_t texturedTileCount(const TexturedAreaParams& p) {
    Layout L;
    if (layout(p, L) != TexturedStatus::Ok) return 0;
    return static_cast<std::size_t>(L.nTiles);
}

TexturedStatus texturedArea(const TexturedAreaParams& p, HeightField& out) {
    out = HeightField();
    Layout L;
    if (auto s = layout(p, L); s != TexturedStatus::Ok) return s;
    const double cs = p.seed.cellSize();
    const double nx = std::ceil((L.b.x1 - L.b.x0) / cs);
    const double ny = std::ceil((L.b.y1 - L.b.y0) / cs);
    if (!(nx * ny <= static_cast<double>(kMaxCells))) return TexturedStatus::TooLarge;
    Grid g;
    g.width = static_cast<std::size_t>(nx);
    g.height = static_cast<std::size_t>(ny);
    g.cellSize = cs;
    g.origin = {L.b.x0 + 0.5 * cs, L.b.y0 + 0.5 * cs};
    if (validate(g) != Status::Ok) return TexturedStatus::TooLarge;

    // Inside mask by scanline, even-odd, half-open: a cell centre is inside when an odd number of
    // edge crossings lie at x <= centre; an edge spans the rows (min(y), max(y)].
    std::vector<unsigned char> inside(g.width * g.height, 0);
    std::vector<double> xs;
    for (std::size_t j = 0; j < g.height; ++j) {
        const double y = g.origin.y + static_cast<double>(j) * cs;
        xs.clear();
        for (const auto& ring : p.rings) {
            for (std::size_t k = 0; k < ring.size(); ++k) {
                const Point& a = ring[k];
                const Point& b = ring[(k + 1) % ring.size()];
                if ((a.y > y) == (b.y > y)) continue;
                xs.push_back(a.x + (y - a.y) * (b.x - a.x) / (b.y - a.y));
            }
        }
        std::sort(xs.begin(), xs.end());
        std::size_t k = 0;
        for (std::size_t i = 0; i < g.width; ++i) {
            const double x = g.origin.x + static_cast<double>(i) * cs;
            while (k < xs.size() && xs[k] <= x) ++k;
            if ((k & 1) != 0) inside[j * g.width + i] = 1;
        }
    }

    std::vector<float> data(g.width * g.height, kTransparent);
    const bool flipH = p.reflection == Reflection::Horizontal || p.reflection == Reflection::Both;
    const bool flipV = p.reflection == Reflection::Vertical || p.reflection == Reflection::Both;
    const long long sw = static_cast<long long>(p.seed.width());
    const long long sh = static_cast<long long>(p.seed.height());
    const long long gw = static_cast<long long>(g.width);
    const long long gh = static_cast<long long>(g.height);
    std::size_t visits = 0;
    for (long long r = L.r0; r <= L.r1; ++r) {
        for (long long c = L.c0; c <= L.c1; ++c) {
            const double left = L.anchor.x + static_cast<double>(c) * L.px + (oddIndex(r) ? L.shx : 0.0);
            const double bottom = L.anchor.y + static_cast<double>(r) * L.py + (oddIndex(c) ? L.shy : 0.0);
            if (left >= L.b.x1 || left + L.w <= L.b.x0 || bottom >= L.b.y1 || bottom + L.h <= L.b.y0) continue;
            const long long i0 = std::max(0LL, static_cast<long long>(std::floor((left - L.b.x0) / cs)) - 1);
            const long long i1 = std::min(gw - 1, static_cast<long long>(std::floor((left + L.w - L.b.x0) / cs)) + 1);
            const long long j0 = std::max(0LL, static_cast<long long>(std::floor((bottom - L.b.y0) / cs)) - 1);
            const long long j1 = std::min(gh - 1, static_cast<long long>(std::floor((bottom + L.h - L.b.y0) / cs)) + 1);
            if (i1 < i0 || j1 < j0) continue;
            visits += static_cast<std::size_t>((i1 - i0 + 1) * (j1 - j0 + 1));
            if (visits > kMaxTexturedCellVisits) {
                out = HeightField();
                return TexturedStatus::TooManyTiles;
            }
            const bool mirrorX = flipH && oddIndex(c);
            const bool mirrorY = flipV && oddIndex(r);
            for (long long j = j0; j <= j1; ++j) {
                const double v = g.origin.y + static_cast<double>(j) * cs - bottom;
                if (v < 0.0 || v >= L.h) continue;
                long long sj = std::min(sh - 1, static_cast<long long>(std::floor(v / cs)));
                if (mirrorY) sj = sh - 1 - sj;
                for (long long i = i0; i <= i1; ++i) {
                    const std::size_t idx = static_cast<std::size_t>(j) * g.width + static_cast<std::size_t>(i);
                    if (!inside[idx]) continue;
                    const double u = g.origin.x + static_cast<double>(i) * cs - left;
                    if (u < 0.0 || u >= L.w) continue;
                    long long si = std::min(sw - 1, static_cast<long long>(std::floor(u / cs)));
                    if (mirrorX) si = sw - 1 - si;
                    const float val = p.seed.at(static_cast<std::size_t>(si), static_cast<std::size_t>(sj));
                    if (std::isnan(val)) continue;
                    data[idx] = std::isnan(data[idx]) ? val : combine(p.overlapMode, data[idx], val);
                }
            }
        }
    }
    return HeightField::fromData(g, std::move(data), out) == Status::Ok ? TexturedStatus::Ok
                                                                         : TexturedStatus::TooLarge;
}

TexturedStatus makeTexturedComponent(const TexturedAreaParams& p, CombineMode mode, Component& out) {
    Component c;
    c.name = "Textured Area";
    c.mode = mode;
    if (auto s = texturedArea(p, c.field); s != TexturedStatus::Ok) return s;
    out = std::move(c);
    return TexturedStatus::Ok;
}

}  // namespace carvetto::relief
