#include "carvetto/relief/relief.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::relief {

namespace {
bool finite(double v) { return std::isfinite(v); }
bool finite(const Point& p) { return std::isfinite(p.x) && std::isfinite(p.y); }
bool opaque(float v) { return !std::isnan(v); }
}  // namespace

const char* statusMessage(Status s) {
    switch (s) {
        case Status::Ok: return "ok";
        case Status::ZeroSize: return "height field has zero, negative or non-finite size";
        case Status::TooLarge: return "height field exceeds the cell-count cap";
        case Status::BadRegion: return "region is empty or outside the field";
        case Status::BadParameter: return "non-finite or degenerate component property";
    }
    return "unknown";
}

Status validate(const Grid& g) {
    if (g.width == 0 || g.height == 0 || !finite(g.cellSize) || g.cellSize <= 0.0 || !finite(g.origin)) {
        return Status::ZeroSize;
    }
    if (g.width > kMaxCells || g.height > kMaxCells || g.width > kMaxCells / g.height) {
        return Status::TooLarge;
    }
    return Status::Ok;
}

Status HeightField::create(const Grid& grid, float fill, HeightField& out) {
    out = HeightField{};
    const Status s = validate(grid);
    if (s != Status::Ok) return s;
    out.grid_ = grid;
    out.data_.assign(grid.width * grid.height, fill);
    return Status::Ok;
}

Status HeightField::fromData(const Grid& grid, std::vector<float> data, HeightField& out) {
    out = HeightField{};
    const Status s = validate(grid);
    if (s != Status::Ok) return s;
    if (data.size() != grid.width * grid.height) return Status::BadRegion;
    out.grid_ = grid;
    out.data_ = std::move(data);
    return Status::Ok;
}

float HeightField::sample(double x, double y) const {
    if (data_.empty()) return kTransparent;
    const double fx = (x - grid_.origin.x) / grid_.cellSize;
    const double fy = (y - grid_.origin.y) / grid_.cellSize;
    if (!(fx >= 0.0 && fy >= 0.0 && fx <= static_cast<double>(grid_.width - 1) &&
          fy <= static_cast<double>(grid_.height - 1))) {
        return kTransparent;
    }
    std::size_t i = static_cast<std::size_t>(fx);
    std::size_t j = static_cast<std::size_t>(fy);
    if (i >= grid_.width - 1 && grid_.width > 1) i = grid_.width - 2;
    if (j >= grid_.height - 1 && grid_.height > 1) j = grid_.height - 2;
    const double tx = fx - static_cast<double>(i);
    const double ty = fy - static_cast<double>(j);
    const std::size_t i1 = (tx > 0.0 && i + 1 < grid_.width) ? i + 1 : i;
    const std::size_t j1 = (ty > 0.0 && j + 1 < grid_.height) ? j + 1 : j;
    const double w00 = (1.0 - tx) * (1.0 - ty), w10 = tx * (1.0 - ty);
    const double w01 = (1.0 - tx) * ty, w11 = tx * ty;
    double sum = 0.0;
    const struct { double w; float v; } taps[4] = {{w00, at(i, j)}, {w10, at(i1, j)}, {w01, at(i, j1)}, {w11, at(i1, j1)}};
    for (const auto& t : taps) {
        if (t.w == 0.0) continue;
        if (!opaque(t.v)) return kTransparent;
        sum += t.w * static_cast<double>(t.v);
    }
    return static_cast<float>(sum);
}

bool HeightField::minMax(float& lo, float& hi) const {
    bool any = false;
    for (const float v : data_) {
        if (!opaque(v)) continue;
        if (!any) { lo = hi = v; any = true; }
        else { lo = std::min(lo, v); hi = std::max(hi, v); }
    }
    return any;
}

Status HeightField::crop(std::size_t i0, std::size_t j0, std::size_t w, std::size_t h, HeightField& out) const {
    out = HeightField{};
    if (data_.empty() || w == 0 || h == 0 || i0 >= grid_.width || j0 >= grid_.height ||
        w > grid_.width - i0 || h > grid_.height - j0) {
        return Status::BadRegion;
    }
    Grid g = grid_;
    g.width = w;
    g.height = h;
    g.origin = {grid_.origin.x + static_cast<double>(i0) * grid_.cellSize,
                grid_.origin.y + static_cast<double>(j0) * grid_.cellSize};
    std::vector<float> d;
    d.reserve(w * h);
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) d.push_back(at(i0 + i, j0 + j));
    }
    return fromData(g, std::move(d), out);
}

Status HeightField::resample(const Grid& target, HeightField& out) const {
    out = HeightField{};
    if (data_.empty()) return Status::ZeroSize;
    const Status s = validate(target);
    if (s != Status::Ok) return s;
    HeightField r;
    HeightField::create(target, kTransparent, r);
    for (std::size_t j = 0; j < target.height; ++j) {
        const double y = target.origin.y + static_cast<double>(j) * target.cellSize;
        for (std::size_t i = 0; i < target.width; ++i) {
            r.set(i, j, sample(target.origin.x + static_cast<double>(i) * target.cellSize, y));
        }
    }
    out = std::move(r);
    return Status::Ok;
}

float combine(CombineMode mode, float below, float above) {
    if (!opaque(above)) return below;
    if (!opaque(below)) {
        return (mode == CombineMode::Subtract || mode == CombineMode::Multiply) ? kTransparent : above;
    }
    switch (mode) {
        case CombineMode::Add: return below + above;
        case CombineMode::Subtract: return below - above;
        case CombineMode::MergeHigh: return above > below ? above : below;
        case CombineMode::MergeLow: return above < below ? above : below;
        case CombineMode::Multiply: return below * above;
    }
    return below;
}

namespace {
Status checkProps(const Component& c) {
    if (!finite(c.shapeHeightPercent) || !finite(c.baseHeight)) return Status::BadParameter;
    if (c.fade.enabled) {
        const double dx = c.fade.second.x - c.fade.anchor.x, dy = c.fade.second.y - c.fade.anchor.y;
        if (!finite(c.fade.anchor) || !finite(c.fade.second) || !finite(c.fade.percent) ||
            !(dx * dx + dy * dy > 0.0)) {
            return Status::BadParameter;
        }
    }
    if (c.tilt.enabled) {
        const double dx = c.tilt.raise.x - c.tilt.pivot.x, dy = c.tilt.raise.y - c.tilt.pivot.y;
        if (!finite(c.tilt.pivot) || !finite(c.tilt.raise) || !finite(c.tilt.angleRad) ||
            !(dx * dx + dy * dy > 0.0)) {
            return Status::BadParameter;
        }
    }
    return Status::Ok;
}
}  // namespace

float componentValueAt(const Component& c, double x, double y) {
    const float raw = c.field.sample(x, y);
    if (!opaque(raw)) return raw;
    double v = static_cast<double>(raw) * c.shapeHeightPercent / 100.0;
    if (c.fade.enabled) {
        const double dx = c.fade.second.x - c.fade.anchor.x, dy = c.fade.second.y - c.fade.anchor.y;
        const double t = std::clamp(((x - c.fade.anchor.x) * dx + (y - c.fade.anchor.y) * dy) / (dx * dx + dy * dy), 0.0, 1.0);
        v *= 1.0 - c.fade.percent / 100.0 * t;
    }
    if (c.tilt.enabled) {
        const double dx = c.tilt.raise.x - c.tilt.pivot.x, dy = c.tilt.raise.y - c.tilt.pivot.y;
        const double len = std::sqrt(dx * dx + dy * dy);
        v += std::tan(c.tilt.angleRad) * ((x - c.tilt.pivot.x) * dx + (y - c.tilt.pivot.y) * dy) / len;
    }
    return static_cast<float>(v + c.baseHeight);
}

Status composeComponent(const Component& c, const Grid& grid, HeightField& out) {
    out = HeightField{};
    Status s = validate(grid);
    if (s != Status::Ok) return s;
    if (c.field.empty()) return Status::ZeroSize;
    s = checkProps(c);
    if (s != Status::Ok) return s;
    HeightField r;
    HeightField::create(grid, kTransparent, r);
    for (std::size_t j = 0; j < grid.height; ++j) {
        const double y = grid.origin.y + static_cast<double>(j) * grid.cellSize;
        for (std::size_t i = 0; i < grid.width; ++i) {
            r.set(i, j, componentValueAt(c, grid.origin.x + static_cast<double>(i) * grid.cellSize, y));
        }
    }
    out = std::move(r);
    return Status::Ok;
}

Status bake(const Component& c, HeightField& out) {
    if (c.field.empty()) { out = HeightField{}; return Status::ZeroSize; }
    return composeComponent(c, c.field.grid(), out);
}

Status unionGrid(const std::vector<Level>& levels, Grid& out) {
    bool any = false;
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0, cell = 0;
    for (const Level& l : levels) {
        if (!l.visible) continue;
        for (const Component& c : l.components) {
            if (!c.visible || c.field.empty()) continue;
            const Grid& g = c.field.grid();
            const double ex = g.origin.x + static_cast<double>(g.width - 1) * g.cellSize;
            const double ey = g.origin.y + static_cast<double>(g.height - 1) * g.cellSize;
            if (!any) { x0 = g.origin.x; y0 = g.origin.y; x1 = ex; y1 = ey; cell = g.cellSize; any = true; }
            else {
                x0 = std::min(x0, g.origin.x); y0 = std::min(y0, g.origin.y);
                x1 = std::max(x1, ex); y1 = std::max(y1, ey);
                cell = std::min(cell, g.cellSize);
            }
        }
    }
    if (!any) return Status::ZeroSize;
    const double w = std::floor((x1 - x0) / cell + 0.5) + 1.0, h = std::floor((y1 - y0) / cell + 0.5) + 1.0;
    if (!(w * h <= static_cast<double>(kMaxCells))) return Status::TooLarge;
    out = Grid{static_cast<std::size_t>(w), static_cast<std::size_t>(h), cell, {x0, y0}};
    return validate(out);
}

Status compose(const std::vector<Level>& levels, const Grid& grid, HeightField& out) {
    out = HeightField{};
    Status s = validate(grid);
    if (s != Status::Ok) return s;
    HeightField model;
    HeightField::create(grid, kTransparent, model);
    for (const Level& l : levels) {
        if (!l.visible || !finite(l.baseHeight)) {
            if (l.visible) return Status::BadParameter;
            continue;
        }
        HeightField acc;
        HeightField::create(grid, kTransparent, acc);
        for (const Component& c : l.components) {
            if (!c.visible) continue;
            HeightField cf;
            s = composeComponent(c, grid, cf);
            if (s != Status::Ok) return s;
            for (std::size_t k = 0; k < acc.data().size(); ++k) {
                const std::size_t i = k % grid.width, j = k / grid.width;
                acc.set(i, j, combine(c.mode, acc.at(i, j), cf.at(i, j)));
            }
        }
        for (std::size_t j = 0; j < grid.height; ++j) {
            for (std::size_t i = 0; i < grid.width; ++i) {
                float v = acc.at(i, j);
                if (opaque(v)) v = static_cast<float>(static_cast<double>(v) + l.baseHeight);
                model.set(i, j, combine(l.mode, model.at(i, j), v));
            }
        }
    }
    out = std::move(model);
    return Status::Ok;
}

}  // namespace carvetto::relief
