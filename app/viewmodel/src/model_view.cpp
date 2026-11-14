#include "carvetto/viewmodel/model_view.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace carvetto::viewmodel {

namespace {

constexpr double kHalfPi = std::numbers::pi / 2.0;

struct Vec3 {
    double x{0}, y{0}, z{0};
};

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

struct Basis {
    Vec3 right, up, toEye;
};

Basis basisOf(const OrbitCamera& cam) {
    const double ca = std::cos(cam.azimuthRad), sa = std::sin(cam.azimuthRad);
    const double ce = std::cos(cam.elevationRad), se = std::sin(cam.elevationRad);
    Basis b;
    b.right = {ca, sa, 0.0};
    b.up = {-se * sa, se * ca, ce};
    b.toEye = {ce * sa, -ce * ca, se};
    return b;
}

Vec3 lightDirection(const Basis& b) {
    Vec3 l{-0.5 * b.right.x + 0.6 * b.up.x + 0.7 * b.toEye.x, -0.5 * b.right.y + 0.6 * b.up.y + 0.7 * b.toEye.y,
           -0.5 * b.right.z + 0.6 * b.up.z + 0.7 * b.toEye.z};
    const double len = std::sqrt(dot(l, l));
    return {l.x / len, l.y / len, l.z / len};
}

std::uint32_t shadeColour(std::uint32_t base, double intensity) {
    const double k = std::clamp(intensity, 0.0, 1.0);
    auto ch = [&](int shift) {
        const double v = static_cast<double>((base >> shift) & 0xFFu) * k;
        return static_cast<std::uint32_t>(std::clamp(std::lround(v), 0L, 255L));
    };
    return 0xFF000000u | (ch(16) << 16) | (ch(8) << 8) | ch(0);
}

double intensityOf(double lightFactor) {
    return 0.3 + 0.7 * lightFactor;
}

struct Vertex {
    double sx{0}, sy{0}, depth{0}, shade{0};
    bool valid{false};
};

}  // namespace

const char* modelMaterialName(ModelMaterial m) {
    switch (m) {
        case ModelMaterial::SolidColour: return "Solid Color";
        case ModelMaterial::Wood: return "Wood";
        case ModelMaterial::Metal: return "Metal";
        case ModelMaterial::Stone: return "Stone";
        case ModelMaterial::Plastic: return "Plastic";
    }
    return "Solid Color";
}

std::uint32_t modelMaterialColour(ModelMaterial m) {
    switch (m) {
        case ModelMaterial::SolidColour: return 0xFFC8C8D2u;
        case ModelMaterial::Wood: return 0xFFDEB887u;
        case ModelMaterial::Metal: return 0xFFB4BCC8u;
        case ModelMaterial::Stone: return 0xFF9C9890u;
        case ModelMaterial::Plastic: return 0xFF5C8CDCu;
    }
    return 0xFFC8C8D2u;
}

void OrbitCamera::orbit(double dx, double dy) {
    azimuthRad -= dx * 0.01;
    elevationRad = std::clamp(elevationRad + dy * 0.01, 0.0, kHalfPi);
}

void OrbitCamera::zoomBy(double factor) {
    if (std::isfinite(factor) && factor > 0.0) {
        zoom = std::clamp(zoom * factor, 0.05, 200.0);
    }
}

void OrbitCamera::pan(double dx, double dy) {
    panX += dx;
    panY += dy;
}

void OrbitCamera::reset() {
    *this = OrbitCamera{};
}

void OrbitCamera::viewTop() {
    azimuthRad = 0.0;
    elevationRad = kHalfPi;
    panX = panY = 0.0;
}

void OrbitCamera::viewFront() {
    azimuthRad = 0.0;
    elevationRad = 0.0;
    panX = panY = 0.0;
}

void OrbitCamera::viewRight() {
    azimuthRad = kHalfPi;
    elevationRad = 0.0;
    panX = panY = 0.0;
}

double modelLightFactor(const OrbitCamera& camera, double nx, double ny, double nz) {
    const Vec3 l = lightDirection(basisOf(camera));
    const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len <= 0.0) {
        return 0.0;
    }
    return std::clamp((l.x * nx + l.y * ny + l.z * nz) / len, 0.0, 1.0);
}

ModelSurface composeModelSurface(const document::Document& doc, document::SheetId sheet, std::size_t maxDimension) {
    ModelSurface out;
    const std::vector<relief::Level> levels = doc.reliefModel(sheet);
    if (levels.empty()) {
        out.problem = "This sheet has no relief levels.";
        return out;
    }
    relief::Grid grid;
    if (const relief::Status s = relief::unionGrid(levels, grid); s != relief::Status::Ok) {
        out.problem = relief::statusMessage(s);
        return out;
    }
    const std::size_t limit = std::max<std::size_t>(maxDimension, 2);
    const std::size_t longer = std::max(grid.width, grid.height);
    std::size_t stride = 1;
    if (longer > limit) {
        stride = (longer + limit - 1) / limit;
        grid.cellSize *= static_cast<double>(stride);
        grid.width = (grid.width - 1) / stride + 1;
        grid.height = (grid.height - 1) / stride + 1;
    }
    out.stride = stride;
    if (const relief::Status s = relief::compose(levels, grid, out.field); s != relief::Status::Ok) {
        out.problem = relief::statusMessage(s);
        out.field = relief::HeightField{};
    }
    return out;
}

Raster renderModel(const ModelSurface& surface, const OrbitCamera& camera, const ModelRenderOptions& options) {
    Raster r;
    r.width = options.width;
    r.height = options.height;
    r.stride = 1;
    if (options.width == 0 || options.height == 0 || options.width > 16384 || options.height > 16384) {
        r.width = r.height = 0;
        return r;
    }
    r.pixels.assign(r.width * r.height, options.background);
    if (!surface.ok()) {
        return r;
    }
    const relief::HeightField& f = surface.field;
    const std::size_t w = f.width(), h = f.height();
    const double cs = f.cellSize();
    const double zs = std::isfinite(options.zScale) && options.zScale > 0.0 ? options.zScale : 1.0;

    float lo = 0.0f, hi = 0.0f;
    if (!f.minMax(lo, hi)) {
        return r;
    }
    const double spanX = static_cast<double>(w - 1) * cs;
    const double spanY = static_cast<double>(h - 1) * cs;
    const double spanZ = (static_cast<double>(hi) - static_cast<double>(lo)) * zs;
    const Vec3 centre{f.origin().x + 0.5 * spanX, f.origin().y + 0.5 * spanY,
                      0.5 * (static_cast<double>(hi) + static_cast<double>(lo)) * zs};
    const double radius = std::max(0.5 * std::sqrt(spanX * spanX + spanY * spanY + spanZ * spanZ), 1e-6);
    const double scale =
        camera.zoom * 0.9 * static_cast<double>(std::min(r.width, r.height)) / (2.0 * radius);

    const Basis basis = basisOf(camera);
    const Vec3 light = lightDirection(basis);
    const double cx = 0.5 * static_cast<double>(r.width) + camera.panX;
    const double cy = 0.5 * static_cast<double>(r.height) + camera.panY;

    auto heightAt = [&](std::size_t i, std::size_t j) { return static_cast<double>(f.at(i, j)); };
    auto vertex = [&](std::size_t i, std::size_t j) {
        Vertex v;
        const float z = f.at(i, j);
        if (std::isnan(z)) {
            return v;
        }
        // Surface normal from central differences (one-sided at edges and next to holes).
        auto slope = [&](bool alongX) {
            const std::size_t i0 = alongX ? (i > 0 ? i - 1 : i) : i;
            const std::size_t i1 = alongX ? (i + 1 < w ? i + 1 : i) : i;
            const std::size_t j0 = alongX ? j : (j > 0 ? j - 1 : j);
            const std::size_t j1 = alongX ? j : (j + 1 < h ? j + 1 : j);
            double a = std::isnan(f.at(i0, j0)) ? static_cast<double>(z) : heightAt(i0, j0);
            double b = std::isnan(f.at(i1, j1)) ? static_cast<double>(z) : heightAt(i1, j1);
            const std::size_t steps = alongX ? (i1 - i0) : (j1 - j0);
            if (steps == 0) {
                return 0.0;
            }
            return (b - a) / (static_cast<double>(steps) * cs) * zs;
        };
        const double nx = -slope(true), ny = -slope(false), nz = 1.0;
        const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        const double lf = std::clamp((light.x * nx + light.y * ny + light.z * nz) / len, 0.0, 1.0);
        const Vec3 p{f.origin().x + static_cast<double>(i) * cs - centre.x,
                     f.origin().y + static_cast<double>(j) * cs - centre.y,
                     static_cast<double>(z) * zs - centre.z};
        v.sx = cx + dot(p, basis.right) * scale;
        v.sy = cy - dot(p, basis.up) * scale;
        v.depth = dot(p, basis.toEye);
        v.shade = intensityOf(lf);
        v.valid = true;
        return v;
    };

    const std::uint32_t base = modelMaterialColour(options.material);
    std::vector<float> zbuf(r.pixels.size(), -std::numeric_limits<float>::infinity());

    auto triangle = [&](const Vertex& a, const Vertex& b, const Vertex& c) {
        const double area = (b.sx - a.sx) * (c.sy - a.sy) - (b.sy - a.sy) * (c.sx - a.sx);
        if (std::fabs(area) < 1e-12) {
            return;
        }
        const double minX = std::min({a.sx, b.sx, c.sx}), maxX = std::max({a.sx, b.sx, c.sx});
        const double minY = std::min({a.sy, b.sy, c.sy}), maxY = std::max({a.sy, b.sy, c.sy});
        const long x0 = std::max(0L, static_cast<long>(std::floor(minX - 0.5)));
        const long x1 = std::min(static_cast<long>(r.width) - 1, static_cast<long>(std::ceil(maxX - 0.5)));
        const long y0 = std::max(0L, static_cast<long>(std::floor(minY - 0.5)));
        const long y1 = std::min(static_cast<long>(r.height) - 1, static_cast<long>(std::ceil(maxY - 0.5)));
        const double inv = 1.0 / area;
        for (long y = y0; y <= y1; ++y) {
            for (long x = x0; x <= x1; ++x) {
                const double px = static_cast<double>(x) + 0.5, py = static_cast<double>(y) + 0.5;
                const double w0 = ((b.sx - px) * (c.sy - py) - (b.sy - py) * (c.sx - px)) * inv;
                const double w1 = ((c.sx - px) * (a.sy - py) - (c.sy - py) * (a.sx - px)) * inv;
                const double w2 = 1.0 - w0 - w1;
                constexpr double kEps = -1e-9;
                if (w0 < kEps || w1 < kEps || w2 < kEps) {
                    continue;
                }
                const float depth = static_cast<float>(w0 * a.depth + w1 * b.depth + w2 * c.depth);
                const std::size_t idx = static_cast<std::size_t>(y) * r.width + static_cast<std::size_t>(x);
                if (depth > zbuf[idx]) {
                    zbuf[idx] = depth;
                    r.pixels[idx] = shadeColour(base, w0 * a.shade + w1 * b.shade + w2 * c.shade);
                }
            }
        }
    };

    std::vector<Vertex> prev(w), cur(w);
    for (std::size_t i = 0; i < w; ++i) {
        prev[i] = vertex(i, 0);
    }
    for (std::size_t j = 1; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            cur[i] = vertex(i, j);
        }
        for (std::size_t i = 0; i + 1 < w; ++i) {
            const Vertex &v00 = prev[i], &v10 = prev[i + 1], &v01 = cur[i], &v11 = cur[i + 1];
            if (!(v00.valid && v10.valid && v01.valid && v11.valid)) {
                continue;
            }
            triangle(v00, v10, v11);
            triangle(v00, v11, v01);
        }
        std::swap(prev, cur);
    }
    return r;
}

}  // namespace carvetto::viewmodel
