#include "carvetto/relief/effects.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::relief {

namespace {
bool finitePoint(const Point& p) { return std::isfinite(p.x) && std::isfinite(p.y); }
}  // namespace

Status validateEffects(const LevelEffects& e) {
    if (!finitePoint(e.mirrorCenter)) return Status::BadParameter;
    if (e.clip.polygons.size() > kMaxClipPolygons) return Status::BadParameter;
    std::size_t vertices = 0;
    for (const auto& poly : e.clip.polygons) {
        if (poly.size() < 3) return Status::BadParameter;
        vertices += poly.size();
        if (vertices > kMaxClipVertices) return Status::BadParameter;
        for (const Point& p : poly) {
            if (!finitePoint(p)) return Status::BadParameter;
        }
    }
    return Status::Ok;
}

Status applyClip(const Clip& clip, HeightField& field) {
    if (clip.polygons.empty() || field.empty()) return Status::Ok;
    LevelEffects probe;
    probe.clip = clip;
    if (validateEffects(probe) != Status::Ok) return Status::BadParameter;
    const Grid& g = field.grid();
    std::vector<double> xs;
    for (std::size_t j = 0; j < g.height; ++j) {
        const double y = g.origin.y + static_cast<double>(j) * g.cellSize;
        xs.clear();
        for (const auto& poly : clip.polygons) {
            for (std::size_t k = 0; k < poly.size(); ++k) {
                const Point& a = poly[k];
                const Point& b = poly[(k + 1) % poly.size()];
                if ((a.y <= y) == (b.y <= y)) continue;  // half-open: no double count at vertices
                xs.push_back(a.x + (y - a.y) / (b.y - a.y) * (b.x - a.x));
            }
        }
        std::sort(xs.begin(), xs.end());
        std::size_t next = 0;  // crossings at or left of the cell centre
        for (std::size_t i = 0; i < g.width; ++i) {
            const double x = g.origin.x + static_cast<double>(i) * g.cellSize;
            while (next < xs.size() && xs[next] <= x) ++next;
            if (next % 2 == 0) field.set(i, j, kTransparent);
        }
    }
    return Status::Ok;
}

Status applyMirror(MirrorMode mode, Point center, HeightField& field) {
    if (!finitePoint(center)) return Status::BadParameter;
    if (mode == MirrorMode::None || field.empty()) return Status::Ok;
    // fromRight: source is on the right (x > cx) so the left side is replaced; likewise fromTop.
    bool mirrorX = false, mirrorY = false, srcRight = false, srcTop = false;
    switch (mode) {
        case MirrorMode::LeftToRight: mirrorX = true; srcRight = false; break;
        case MirrorMode::RightToLeft: mirrorX = true; srcRight = true; break;
        case MirrorMode::TopToBottom: mirrorY = true; srcTop = true; break;
        case MirrorMode::BottomToTop: mirrorY = true; srcTop = false; break;
        case MirrorMode::QuadTopRight: mirrorX = mirrorY = true; srcRight = srcTop = true; break;
        case MirrorMode::QuadTopLeft: mirrorX = mirrorY = true; srcRight = false; srcTop = true; break;
        case MirrorMode::QuadBottomLeft: mirrorX = mirrorY = true; break;
        case MirrorMode::QuadBottomRight: mirrorX = mirrorY = true; srcRight = true; srcTop = false; break;
        case MirrorMode::None: return Status::Ok;
    }
    const HeightField src = field;
    const Grid& g = field.grid();
    for (std::size_t j = 0; j < g.height; ++j) {
        const double y = g.origin.y + static_cast<double>(j) * g.cellSize;
        const bool flipY = mirrorY && (srcTop ? y < center.y : y > center.y);
        for (std::size_t i = 0; i < g.width; ++i) {
            const double x = g.origin.x + static_cast<double>(i) * g.cellSize;
            const bool flipX = mirrorX && (srcRight ? x < center.x : x > center.x);
            if (!flipX && !flipY) continue;
            const double sx = flipX ? 2.0 * center.x - x : x;
            const double sy = flipY ? 2.0 * center.y - y : y;
            field.set(i, j, src.sample(sx, sy));
        }
    }
    return Status::Ok;
}

Status applyEffects(const LevelEffects& e, HeightField& field) {
    const Status s = validateEffects(e);
    if (s != Status::Ok) return s;
    Status r = applyMirror(e.mirror, e.mirrorCenter, field);
    if (r != Status::Ok) return r;
    return applyClip(e.clip, field);
}

Status bakeLevel(const Level& level, Component& out) {
    out = Component{};
    Level l = level;
    l.visible = true;
    l.baseHeight = 0.0;
    l.mode = CombineMode::Add;
    Grid grid;
    Status s = unionGrid({l}, grid);
    if (s != Status::Ok) return s;
    HeightField f;
    s = compose({l}, grid, f);
    if (s != Status::Ok) return s;
    out.name = level.name + " (baked)";
    out.field = std::move(f);
    return Status::Ok;
}

}  // namespace carvetto::relief
