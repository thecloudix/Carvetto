#include "carvetto/toolpath/tiling.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>

namespace carvetto::toolpath {

namespace {

constexpr double kEps = 1e-9;
constexpr double kMinPiece = 1e-6;  // shorter clipped pieces are dropped (an arc that short would read as a circle)
constexpr double kMaxCoord = 1e7;  // mm; larger (or non-finite) coordinates are refused, see tiling.hpp
constexpr std::size_t kMaxTiles = 10000;
constexpr std::size_t kMaxPairs = 5'000'000;  // (move, tile) pairs that may need clipping in one run
constexpr double kTwoPi = 2.0 * geometry::kPi;

bool inside(const Box2& r, Vec2 p) {
    return p.x >= r.min.x - kEps && p.x <= r.max.x + kEps && p.y >= r.min.y - kEps && p.y <= r.max.y + kEps;
}

double wrap(double a) {
    a = std::fmod(a, kTwoPi);
    return a < 0.0 ? a + kTwoPi : a;
}

Vec3 shifted(Vec3 p, Vec2 d) { return {p.x + d.x, p.y + d.y, p.z}; }

/// Emits clipped pieces with relinking.
class Emitter {
public:
    Emitter(Toolpath& out, double safeZ) : out_(out), safeZ_(safeZ) {}

    void piece(Vec3 start, const Move& m) {
        link(start);
        out_.append(m);
        pos_ = m.to;
    }
    void finish() {
        if (pos_ && pos_->z < safeZ_ - kEps) {
            out_.append(Move::retract({pos_->x, pos_->y, safeZ_}));
            pos_ = Vec3{pos_->x, pos_->y, safeZ_};
        }
    }

private:
    void link(Vec3 start) {
        if (pos_ && distance(*pos_, start) <= 1e-7) {
            return;
        }
        if (pos_ && pos_->z < safeZ_ - kEps) {
            out_.append(Move::retract({pos_->x, pos_->y, safeZ_}));
        }
        const double linkZ = std::max(safeZ_, start.z);
        out_.append(Move::rapid({start.x, start.y, linkZ}));
        if (start.z < linkZ - kEps) {
            out_.append(Move::plunge(start));
        }
    }
    Toolpath& out_;
    double safeZ_;
    std::optional<Vec3> pos_;
};

void clipLine(Emitter& em, const Move& m, Vec3 from, const Box2& r) {
    double t0 = 0.0;
    double t1 = 1.0;
    const Vec3 d = m.to - from;
    const auto clipAxis = [&](double p0, double dp, double lo, double hi) {
        if (std::fabs(dp) < kEps) {
            return p0 >= lo - kEps && p0 <= hi + kEps;
        }
        double a = (lo - p0) / dp;
        double b = (hi - p0) / dp;
        if (a > b) {
            std::swap(a, b);
        }
        t0 = std::max(t0, a);
        t1 = std::min(t1, b);
        return true;
    };
    if (!clipAxis(from.x, d.x, r.min.x, r.max.x) || !clipAxis(from.y, d.y, r.min.y, r.max.y)) {
        return;
    }
    if (t1 < t0) {
        return;
    }
    const Vec3 a = t0 <= 0.0 ? from : lerp(from, m.to, t0);
    const Vec3 b = t1 >= 1.0 ? m.to : lerp(from, m.to, t1);
    if (d.length() > kMinPiece && distance(a, b) < kMinPiece) {
        return;
    }
    Move piece = m;
    piece.to = b;
    em.piece(a, piece);
}

void clipArc(Emitter& em, const Move& m, Vec3 from, const Box2& r) {
    const double rad = arcRadius(m, from.xy());
    const double sweep = arcSweep(m, from.xy());
    const double absSweep = std::fabs(sweep);
    // A degenerate arc (no radius or no sweep) is shorter than a piece in any case: nothing to cut.
    if (!(rad * absSweep >= kMinPiece)) {
        return;
    }
    const double a0 = std::atan2(from.y - m.centre.y, from.x - m.centre.x);
    const bool ccw = sweep > 0.0;

    std::vector<double> ts{0.0, 1.0};
    const auto addAngle = [&](double theta) {
        const double dth = ccw ? wrap(theta - a0) : wrap(a0 - theta);
        const double t = dth / absSweep;
        if (t > 0.0 && t < 1.0) {
            ts.push_back(t);
        }
    };
    const auto addVertical = [&](double x) {  // line x = const
        const double dx = x - m.centre.x;
        if (std::fabs(dx) <= rad) {
            const double h = std::sqrt(std::max(0.0, rad * rad - dx * dx));
            addAngle(std::atan2(h, dx));
            addAngle(std::atan2(-h, dx));
        }
    };
    const auto addHorizontal = [&](double y) {
        const double dy = y - m.centre.y;
        if (std::fabs(dy) <= rad) {
            const double h = std::sqrt(std::max(0.0, rad * rad - dy * dy));
            addAngle(std::atan2(dy, h));
            addAngle(std::atan2(dy, -h));
        }
    };
    addVertical(r.min.x);
    addVertical(r.max.x);
    addHorizontal(r.min.y);
    addHorizontal(r.max.y);
    std::sort(ts.begin(), ts.end());

    const auto at = [&](double t) -> Vec3 {
        if (t <= 0.0) {
            return from;
        }
        if (t >= 1.0) {
            return m.to;
        }
        const double th = a0 + sweep * t;
        return {m.centre.x + rad * std::cos(th), m.centre.y + rad * std::sin(th), from.z + (m.to.z - from.z) * t};
    };

    // Merge consecutive inside intervals so an arc wholly inside stays one move.
    bool inRun = false;
    double runStart = 0.0;
    double runEnd = 0.0;
    const auto flush = [&]() {
        if (!inRun) {
            return;
        }
        const double ta = runStart;
        const double tb = runEnd;
        inRun = false;
        if ((tb - ta) * absSweep * rad < kMinPiece) {
            return;
        }
        if (ta <= 0.0 && tb >= 1.0) {
            em.piece(from, m);
            return;
        }
        Move piece = m;
        piece.to = at(tb);
        em.piece(at(ta), piece);
    };
    for (std::size_t i = 0; i + 1 < ts.size(); ++i) {
        const double ta = ts[i];
        const double tb = ts[i + 1];
        if (tb - ta <= 0.0) {
            continue;
        }
        const Vec3 mid = at((ta + tb) * 0.5);
        // A midpoint is tested with a tolerance, so an arc running along an edge is kept.
        if (inside(r, mid.xy())) {
            if (!inRun) {
                inRun = true;
                runStart = ta;
            }
            runEnd = tb;
        } else {
            flush();
        }
    }
    flush();
}

bool finiteCoord(double v) { return std::isfinite(v) && std::fabs(v) <= kMaxCoord; }
bool finitePoint(Vec3 p) { return finiteCoord(p.x) && finiteCoord(p.y) && finiteCoord(p.z); }

/// Whether a move can be clipped: its end, its start and (for an arc) its centre are finite.
bool clippable(const Move& m, Vec3 from) {
    if (!finitePoint(m.to) || !finitePoint(from)) {
        return false;
    }
    return !isArc(m.kind) || (finiteCoord(m.centre.x) && finiteCoord(m.centre.y));
}

/// Conservative bounding box of a cut move (a circle's box for an arc), for rejecting early.
Box2 moveBox(const Move& m, Vec3 from) {
    Box2 b;
    b.expand(from.xy());
    b.expand(m.to.xy());
    if (isArc(m.kind)) {
        const double rad = arcRadius(m, from.xy());
        b.expand(Vec2{m.centre.x - rad, m.centre.y - rad});
        b.expand(Vec2{m.centre.x + rad, m.centre.y + rad});
    }
    return b;
}

/// True when the move's box may touch the rectangle (a margin above every clip tolerance).
bool mayTouch(const Box2& b, const Box2& r) {
    constexpr double kMargin = 1e-6;
    return b.max.x >= r.min.x - kMargin && b.min.x <= r.max.x + kMargin && b.max.y >= r.min.y - kMargin &&
           b.min.y <= r.max.y + kMargin;
}

/// Clips the moves of `tp` named by `subset` (all when null; ascending indices) to `rect`.
Toolpath clipMoves(const Toolpath& tp, const std::vector<std::uint32_t>* subset, const Box2& rect, double safeZ,
                   std::optional<Vec3> from) {
    Toolpath out;
    out.name = tp.name;
    out.tool = tp.tool;
    Emitter em(out, safeZ);
    const std::size_t n = subset ? subset->size() : tp.moves.size();
    for (std::size_t s = 0; s < n; ++s) {
        const std::size_t k = subset ? (*subset)[s] : s;
        const Move& m = tp.moves[k];
        const Vec3 pos = k == 0 ? (from ? *from : m.to) : tp.moves[k - 1].to;
        if (m.kind == MoveKind::Rapid || !clippable(m, pos)) {
            continue;
        }
        if (m.kind == MoveKind::Dwell) {
            if (inside(rect, m.to.xy())) {
                em.piece(m.to, m);
            }
            continue;
        }
        if (!mayTouch(moveBox(m, pos), rect)) {
            continue;
        }
        if (isArc(m.kind)) {
            clipArc(em, m, pos, rect);
        } else {
            clipLine(em, m, pos, rect);
        }
    }
    em.finish();
    return out;
}

}  // namespace

Toolpath clipToolpath(const Toolpath& tp, const Box2& rect, double safeZ, std::optional<Vec3> from) {
    return clipMoves(tp, nullptr, rect, safeZ, from);
}

std::vector<Box2> tileRects(const Box2& extent, const TileSpec& spec, std::string* error) {
    const auto fail = [&](const char* msg) {
        if (error) {
            *error = msg;
        }
        return std::vector<Box2>{};
    };
    if (error) {
        error->clear();
    }
    if (extent.isEmpty()) {
        return {};
    }
    const bool useW = spec.strategy != TileStrategy::FeedThroughY;
    const bool useH = spec.strategy != TileStrategy::FeedThroughX;
    if ((useW && !(spec.width > 0.0)) || (useH && !(spec.height > 0.0)) || !std::isfinite(spec.width) ||
        !std::isfinite(spec.height)) {
        return fail("tile size must be greater than zero");
    }
    if (!(spec.overlap >= 0.0) || !std::isfinite(spec.overlap) || (useW && spec.overlap >= spec.width) ||
        (useH && spec.overlap >= spec.height)) {
        return fail("tile overlap must be at least zero and smaller than the tile size");
    }
    const double w = useW ? spec.width : extent.width();
    const double h = useH ? spec.height : extent.height();
    const auto count = [](double len, double size, double ov) -> std::size_t {
        if (len <= size) {
            return 1;
        }
        return static_cast<std::size_t>(std::ceil((len - ov) / (size - ov) - 1e-9));
    };
    const std::size_t nx = useW ? count(extent.width(), w, spec.overlap) : 1;
    const std::size_t ny = useH ? count(extent.height(), h, spec.overlap) : 1;
    if (nx > kMaxTiles || ny > kMaxTiles || nx * ny > kMaxTiles) {
        return fail("too many tiles");
    }
    std::vector<Box2> out;
    for (std::size_t j = 0; j < ny; ++j) {
        for (std::size_t i = 0; i < nx; ++i) {
            const double x0 = extent.min.x + static_cast<double>(i) * (w - (useW ? spec.overlap : 0.0));
            const double y0 = extent.min.y + static_cast<double>(j) * (h - (useH ? spec.overlap : 0.0));
            out.push_back(Box2{Vec2{x0, y0}, Vec2{std::min(x0 + w, extent.max.x), std::min(y0 + h, extent.max.y)}});
        }
    }
    if (spec.smallestFirst) {
        std::stable_sort(out.begin(), out.end(), [](const Box2& a, const Box2& b) { return a.area() < b.area(); });
    }
    return out;
}

TilingResult tileToolpaths(const std::vector<Toolpath>& toolpaths, const TileSpec& spec,
                           std::optional<Box2> extentIn, const std::function<bool()>& cancel) {
    TilingResult res;
    const auto cancelled = [&]() { return cancel && cancel(); };
    // Non-finite or absurd coordinates never reach the output: refuse the whole job.
    for (const Toolpath& tp : toolpaths) {
        for (const Move& m : tp.moves) {
            if (!finitePoint(m.to) || (isArc(m.kind) && !(finiteCoord(m.centre.x) && finiteCoord(m.centre.y)))) {
                res.error = "a toolpath has coordinates that are not finite or beyond 10000 m";
                return res;
            }
        }
    }
    if (extentIn && !(finiteCoord(extentIn->min.x) && finiteCoord(extentIn->min.y) &&
                      finiteCoord(extentIn->max.x) && finiteCoord(extentIn->max.y))) {
        res.error = "the tiling extent is not finite";
        return res;
    }
    Box2 extent;
    double maxZ = -std::numeric_limits<double>::infinity();
    for (const Toolpath& tp : toolpaths) {
        extent.expand(tp.bounds());
        if (const auto z = tp.maxZ()) {
            maxZ = std::max(maxZ, *z);
        }
    }
    if (extentIn) {
        extent = *extentIn;
    }
    const double safeZ = spec.safeZ ? *spec.safeZ : (std::isfinite(maxZ) ? maxZ : 0.0);
    if (spec.safeZ && !std::isfinite(*spec.safeZ)) {
        res.error = "safe Z is not finite";
        return res;
    }
    const std::vector<Box2> rects = tileRects(extent, spec, &res.error);
    if (!res.ok()) {
        return res;
    }
    const bool useW = spec.strategy != TileStrategy::FeedThroughY;
    const bool useH = spec.strategy != TileStrategy::FeedThroughX;
    const double stepX = spec.width - spec.overlap;
    const double stepY = spec.height - spec.overlap;
    const auto colOf = [&](const Box2& r) {
        return useW && stepX > 0.0 ? static_cast<std::size_t>(std::llround((r.min.x - extent.min.x) / stepX)) : 0;
    };
    const auto rowOf = [&](const Box2& r) {
        return useH && stepY > 0.0 ? static_cast<std::size_t>(std::llround((r.min.y - extent.min.y) / stepY)) : 0;
    };

    // Bucket the cut moves by tile: a move is only clipped against tiles its box may touch.
    std::size_t nx = 1;
    std::size_t ny = 1;
    for (const Box2& r : rects) {
        nx = std::max(nx, colOf(r) + 1);
        ny = std::max(ny, rowOf(r) + 1);
    }
    std::vector<std::size_t> tileAt(nx * ny, rects.size());  // grid cell -> index into rects
    for (std::size_t k = 0; k < rects.size(); ++k) {
        tileAt[rowOf(rects[k]) * nx + colOf(rects[k])] = k;
    }
    const std::size_t nTp = toolpaths.size();
    std::vector<std::vector<std::uint32_t>> buckets(rects.size() * nTp);  // [tile * nTp + toolpath]
    const double tw = rects.empty() ? 0.0 : (useW ? spec.width : extent.width());
    const double th = rects.empty() ? 0.0 : (useH ? spec.height : extent.height());
    const auto range = [](double lo, double hi, double origin, double step, double size, std::size_t count,
                          std::size_t& first, std::size_t& last) {
        if (count <= 1 || !(step > 0.0)) {
            first = 0;
            last = 0;
            return;
        }
        const double f = std::floor((lo - origin - size) / step) - 1.0;
        const double l = std::floor((hi - origin) / step) + 1.0;
        first = f <= 0.0 ? 0 : static_cast<std::size_t>(std::min(f, static_cast<double>(count)));
        last = l <= 0.0 ? 0 : static_cast<std::size_t>(std::min(l, static_cast<double>(count - 1)));
    };
    std::size_t pairs = 0;
    for (std::size_t t = 0; t < nTp; ++t) {
        const Toolpath& tp = toolpaths[t];
        for (std::size_t k = 0; k < tp.moves.size(); ++k) {
            if ((k & 0x3ff) == 0 && cancelled()) {
                res.error = "cancelled";
                return res;
            }
            const Move& m = tp.moves[k];
            if (m.kind == MoveKind::Rapid) {
                continue;
            }
            const Vec3 pos = k == 0 ? m.to : tp.moves[k - 1].to;
            const Box2 box = m.kind == MoveKind::Dwell ? Box2{m.to.xy(), m.to.xy()} : moveBox(m, pos);
            std::size_t i0 = 0;
            std::size_t i1 = 0;
            std::size_t j0 = 0;
            std::size_t j1 = 0;
            range(box.min.x, box.max.x, extent.min.x, stepX, tw, nx, i0, i1);
            range(box.min.y, box.max.y, extent.min.y, stepY, th, ny, j0, j1);
            for (std::size_t j = j0; j <= j1; ++j) {
                for (std::size_t i = i0; i <= i1; ++i) {
                    const std::size_t tile = tileAt[j * nx + i];
                    if (tile >= rects.size() || !mayTouch(box, rects[tile])) {
                        continue;
                    }
                    if (++pairs > kMaxPairs) {
                        res.error = "the job is too large to tile (too many moves for this many tiles)";
                        return res;
                    }
                    buckets[tile * nTp + t].push_back(static_cast<std::uint32_t>(k));
                }
            }
        }
    }

    for (std::size_t k = 0; k < rects.size(); ++k) {
        if (cancelled()) {
            res.error = "cancelled";
            res.tiles.clear();
            return res;
        }
        Tile tile;
        tile.rect = rects[k];
        bool any = false;
        for (std::size_t t = 0; t < nTp; ++t) {
            Toolpath c = clipMoves(toolpaths[t], &buckets[k * nTp + t], rects[k], safeZ, {});
            any = any || !c.isEmpty();
            if (!spec.originalPosition) {
                for (Move& m : c.moves) {
                    m.to = shifted(m.to, -rects[k].min);
                    m.centre = m.centre - rects[k].min;
                }
            }
            tile.toolpaths.push_back(std::move(c));
        }
        if (any) {
            // Grid position from the rectangle's origin (also right after smallest-first sorting).
            tile.column = colOf(rects[k]);
            tile.row = rowOf(rects[k]);
            res.tiles.push_back(std::move(tile));
        }
    }
    return res;
}

bool tileFitsWorkArea(const Tile& tile, double areaX, double areaY) {
    return (areaX <= 0.0 || tile.rect.width() <= areaX + 1e-9) && (areaY <= 0.0 || tile.rect.height() <= areaY + 1e-9);
}

}  // namespace carvetto::toolpath
