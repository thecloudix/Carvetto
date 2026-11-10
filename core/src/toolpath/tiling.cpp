#include "carvetto/toolpath/tiling.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace carvetto::toolpath {

namespace {

constexpr double kEps = 1e-9;
constexpr double kMinPiece = 1e-6;  // shorter clipped pieces are dropped (an arc that short would read as a circle)
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
    const double a0 = std::atan2(from.y - m.centre.y, from.x - m.centre.x);
    const double absSweep = std::fabs(sweep);
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
    std::optional<double> runStart;
    double runEnd = 0.0;
    const auto flush = [&]() {
        if (!runStart) {
            return;
        }
        const double ta = *runStart;
        const double tb = runEnd;
        runStart.reset();
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
            if (!runStart) {
                runStart = ta;
            }
            runEnd = tb;
        } else {
            flush();
        }
    }
    flush();
}

}  // namespace

Toolpath clipToolpath(const Toolpath& tp, const Box2& rect, double safeZ, std::optional<Vec3> from) {
    Toolpath out;
    out.name = tp.name;
    out.tool = tp.tool;
    Emitter em(out, safeZ);
    Vec3 pos = from ? *from : (tp.moves.empty() ? Vec3{} : tp.moves.front().to);
    for (const Move& m : tp.moves) {
        switch (m.kind) {
            case MoveKind::Rapid:
                break;
            case MoveKind::Dwell:
                if (inside(rect, m.to.xy())) {
                    em.piece(m.to, m);
                }
                break;
            case MoveKind::ArcCW:
            case MoveKind::ArcCCW:
                clipArc(em, m, pos, rect);
                break;
            default:
                clipLine(em, m, pos, rect);
                break;
        }
        pos = m.to;
    }
    em.finish();
    return out;
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
    if (nx > 10000 || ny > 10000 || nx * ny > 100000) {
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
                           std::optional<Box2> extentIn) {
    TilingResult res;
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
    const std::vector<Box2> rects = tileRects(extent, spec, &res.error);
    if (!res.ok()) {
        return res;
    }
    const bool useW = spec.strategy != TileStrategy::FeedThroughY;
    for (std::size_t k = 0; k < rects.size(); ++k) {
        Tile tile;
        tile.rect = rects[k];
        bool any = false;
        for (const Toolpath& tp : toolpaths) {
            Toolpath c = clipToolpath(tp, rects[k], safeZ);
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
            const double stepX = spec.width - spec.overlap;
            const double stepY = spec.height - spec.overlap;
            tile.column = useW && stepX > 0.0 ? static_cast<std::size_t>(std::llround((rects[k].min.x - extent.min.x) / stepX)) : 0;
            tile.row = spec.strategy != TileStrategy::FeedThroughX && stepY > 0.0
                           ? static_cast<std::size_t>(std::llround((rects[k].min.y - extent.min.y) / stepY))
                           : 0;
            res.tiles.push_back(std::move(tile));
        }
    }
    return res;
}

bool tileFitsWorkArea(const Tile& tile, double areaX, double areaY) {
    return (areaX <= 0.0 || tile.rect.width() <= areaX + 1e-9) && (areaY <= 0.0 || tile.rect.height() <= areaY + 1e-9);
}

}  // namespace carvetto::toolpath
