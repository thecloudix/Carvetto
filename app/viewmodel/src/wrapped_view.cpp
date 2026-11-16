#include "carvetto/viewmodel/wrapped_view.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/toolpath/move.hpp"

namespace carvetto::viewmodel {

namespace {

using geometry::Vec3;

struct Basis {
    Vec3 right, up, toEye;
};

Basis basisOf(const OrbitCamera& cam) {
    const double ca = std::cos(cam.azimuthRad), sa = std::sin(cam.azimuthRad);
    const double ce = std::cos(cam.elevationRad), se = std::sin(cam.elevationRad);
    return {{ca, sa, 0.0}, {-se * sa, se * ca, ce}, {ce * sa, -ce * ca, se}};
}

double dot(Vec3 a, Vec3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

std::uint32_t shade(std::uint32_t base, double k) {
    k = std::clamp(k, 0.0, 1.0);
    auto ch = [&](int s) {
        const double v = static_cast<double>((base >> s) & 0xFFu) * k;
        return static_cast<std::uint32_t>(std::clamp(std::lround(v), 0L, 255L));
    };
    return 0xFF000000u | (ch(16) << 16) | (ch(8) << 8) | ch(0);
}

struct Vertex {
    double sx{0}, sy{0}, depth{0}, light{0};
    bool valid{false};
};

}  // namespace

geometry::CylinderMap cylinderMapFor(const document::Job& job) {
    geometry::CylinderMap m;
    m.alongX = job.rotaryAxis == document::RotaryAxis::X;
    m.radius = job.radiusMm() > 0.0 ? job.radiusMm() : 1.0;
    m.flip = job.flipDesign;
    m.axisZ = job.rotarySurfaceZ() - job.radiusMm();
    return m;
}

WrappedSurface wrappedSurfaceFromModel(const ModelSurface& model, const document::Job& job) {
    WrappedSurface s;
    if (!model.ok()) {
        return s;
    }
    const relief::HeightField& f = model.field;
    s.nx = f.width();
    s.ny = f.height();
    s.cellSize = f.cellSize();
    s.origin = {f.origin().x, f.origin().y};
    const float base = static_cast<float>(job.rotaryModelBottomZ());
    s.z.reserve(f.data().size());
    for (const float h : f.data()) {
        s.z.push_back(std::isnan(h) ? h : h + base);
    }
    if (model.colours.size() == s.z.size()) {
        s.colours = model.colours;
    }
    return s;
}

WrappedSurface wrappedSurfaceFromSimulation(const simulate::Heightfield& field) {
    WrappedSurface s;
    s.nx = field.nx();
    s.ny = field.ny();
    s.cellSize = field.cellSize();
    s.origin = field.origin();
    // Heightfield cells are cell centres; the grid draws vertices there.
    s.origin = s.origin + geometry::Vec2{0.5 * s.cellSize, 0.5 * s.cellSize};
    s.z = field.heights();
    s.colours.assign(s.z.size(), 0u);
    for (std::size_t k = 0; k < s.z.size(); ++k) {
        if (field.cutStates()[k] != 0) {
            s.colours[k] = 0xFFC8A070u;
        }
    }
    return s;
}

Raster renderWrapped(const WrappedSurface& surface, const geometry::CylinderMap& map, const OrbitCamera& camera,
                     const WrappedRenderOptions& options) {
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
    const std::size_t w = surface.nx, h = surface.ny;
    std::vector<Vec3> world(w * h);
    std::vector<char> present(w * h, 0);
    Vec3 lo{std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()};
    Vec3 hi{-lo.x, -lo.y, -lo.z};
    for (std::size_t j = 0; j < h; ++j) {
        for (std::size_t i = 0; i < w; ++i) {
            const float z = surface.z[j * w + i];
            if (std::isnan(z)) {
                continue;
            }
            const Vec3 flat{surface.origin.x + static_cast<double>(i) * surface.cellSize,
                            surface.origin.y + static_cast<double>(j) * surface.cellSize, static_cast<double>(z)};
            const Vec3 p = map.wrap(flat);
            world[j * w + i] = p;
            present[j * w + i] = 1;
            lo = {std::min(lo.x, p.x), std::min(lo.y, p.y), std::min(lo.z, p.z)};
            hi = {std::max(hi.x, p.x), std::max(hi.y, p.y), std::max(hi.z, p.z)};
        }
    }
    if (lo.x > hi.x) {
        return r;
    }
    const Vec3 centre{0.5 * (lo.x + hi.x), 0.5 * (lo.y + hi.y), 0.5 * (lo.z + hi.z)};
    const Vec3 half{0.5 * (hi.x - lo.x), 0.5 * (hi.y - lo.y), 0.5 * (hi.z - lo.z)};
    const double radius = std::max(std::sqrt(dot(half, half)), 1e-6);
    const double scale = camera.zoom * 0.9 * static_cast<double>(std::min(r.width, r.height)) / (2.0 * radius);
    const Basis basis = basisOf(camera);
    const double cx = 0.5 * static_cast<double>(r.width) + camera.panX;
    const double cy = 0.5 * static_cast<double>(r.height) + camera.panY;
    Vec3 light{-0.5 * basis.right.x + 0.6 * basis.up.x + 0.7 * basis.toEye.x,
               -0.5 * basis.right.y + 0.6 * basis.up.y + 0.7 * basis.toEye.y,
               -0.5 * basis.right.z + 0.6 * basis.up.z + 0.7 * basis.toEye.z};
    const double ll = std::sqrt(dot(light, light));
    light = {light.x / ll, light.y / ll, light.z / ll};

    const auto project = [&](Vec3 p, Vertex& v) {
        const Vec3 q{p.x - centre.x, p.y - centre.y, p.z - centre.z};
        v.sx = cx + dot(q, basis.right) * scale;
        v.sy = cy - dot(q, basis.up) * scale;
        v.depth = dot(q, basis.toEye);
        v.valid = true;
    };
    const auto vertex = [&](std::size_t i, std::size_t j) {
        Vertex v;
        if (!present[j * w + i]) {
            return v;
        }
        const auto at = [&](std::size_t ii, std::size_t jj) {
            return present[jj * w + ii] ? world[jj * w + ii] : world[j * w + i];
        };
        const Vec3 du = at(std::min(i + 1, w - 1), j) - at(i > 0 ? i - 1 : i, j);
        const Vec3 dv = at(i, std::min(j + 1, h - 1)) - at(i, j > 0 ? j - 1 : j);
        Vec3 n = cross(du, dv);
        const double len = std::sqrt(dot(n, n));
        double lf = 1.0;
        if (len > 1e-12) {
            n = {n.x / len, n.y / len, n.z / len};
            // The surface faces away from the axis; take the normal on the outward side.
            const Vec3 p = world[j * w + i];
            const double sideY = map.alongX ? p.y : p.x;
            const Vec3 radial{map.alongX ? 0.0 : sideY, map.alongX ? sideY : 0.0, p.z};
            lf = std::clamp(std::fabs(dot(n, light)) * (dot(n, radial) < 0.0 ? 1.0 : 1.0), 0.0, 1.0);
        }
        project(world[j * w + i], v);
        v.light = 0.3 + 0.7 * lf;
        return v;
    };

    const std::uint32_t base = modelMaterialColour(options.material);
    std::vector<float> zbuf(r.pixels.size(), -std::numeric_limits<float>::infinity());
    std::uint32_t quadBase = base;
    const auto triangle = [&](const Vertex& a, const Vertex& b, const Vertex& c) {
        const double area = (b.sx - a.sx) * (c.sy - a.sy) - (b.sy - a.sy) * (c.sx - a.sx);
        if (std::fabs(area) < 1e-12) {
            return;
        }
        const long x0 = std::max(0L, static_cast<long>(std::floor(std::min({a.sx, b.sx, c.sx}) - 0.5)));
        const long x1 = std::min(static_cast<long>(r.width) - 1,
                                 static_cast<long>(std::ceil(std::max({a.sx, b.sx, c.sx}) - 0.5)));
        const long y0 = std::max(0L, static_cast<long>(std::floor(std::min({a.sy, b.sy, c.sy}) - 0.5)));
        const long y1 = std::min(static_cast<long>(r.height) - 1,
                                 static_cast<long>(std::ceil(std::max({a.sy, b.sy, c.sy}) - 0.5)));
        const double inv = 1.0 / area;
        for (long y = y0; y <= y1; ++y) {
            for (long x = x0; x <= x1; ++x) {
                const double px = static_cast<double>(x) + 0.5, py = static_cast<double>(y) + 0.5;
                const double w0 = ((b.sx - px) * (c.sy - py) - (b.sy - py) * (c.sx - px)) * inv;
                const double w1 = ((c.sx - px) * (a.sy - py) - (c.sy - py) * (a.sx - px)) * inv;
                const double w2 = 1.0 - w0 - w1;
                if (w0 < -1e-9 || w1 < -1e-9 || w2 < -1e-9) {
                    continue;
                }
                const float depth = static_cast<float>(w0 * a.depth + w1 * b.depth + w2 * c.depth);
                const std::size_t idx = static_cast<std::size_t>(y) * r.width + static_cast<std::size_t>(x);
                if (depth > zbuf[idx]) {
                    zbuf[idx] = depth;
                    r.pixels[idx] = shade(quadBase, w0 * a.light + w1 * b.light + w2 * c.light);
                }
            }
        }
    };

    const bool coloured = surface.colours.size() == w * h;
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
            if (coloured) {
                const std::uint32_t own = surface.colours[(j - 1) * w + i];
                quadBase = own != 0 ? own : base;
            }
            triangle(v00, v10, v11);
            triangle(v00, v11, v01);
        }
        std::swap(prev, cur);
    }

    // Paths: depth-tested one-pixel lines with a small bias so paths lying on the surface show.
    const float bias = static_cast<float>(radius * 0.004);
    for (const ModelPath& path : options.paths) {
        for (const auto& run : path.runs) {
            std::vector<Vec3> flat;
            flat.reserve(run.size());
            for (const ModelPathPoint& q : run) {
                flat.push_back({q.x, q.y, q.z});
            }
            const std::vector<Vec3> pts = geometry::wrapPolyline(map, flat, options.maxStepRad);
            for (std::size_t k = 1; k < pts.size(); ++k) {
                Vertex a, b;
                project(pts[k - 1], a);
                project(pts[k], b);
                const double span = std::max(std::fabs(b.sx - a.sx), std::fabs(b.sy - a.sy));
                if (!std::isfinite(span) || span > 1.0e5) {
                    continue;
                }
                const long steps = std::max(1L, static_cast<long>(std::ceil(span)));
                for (long t = 0; t <= steps; ++t) {
                    const double u = static_cast<double>(t) / static_cast<double>(steps);
                    const long x = static_cast<long>(std::floor(a.sx + (b.sx - a.sx) * u));
                    const long y = static_cast<long>(std::floor(a.sy + (b.sy - a.sy) * u));
                    if (x < 0 || y < 0 || x >= static_cast<long>(r.width) || y >= static_cast<long>(r.height)) {
                        continue;
                    }
                    const std::size_t idx = static_cast<std::size_t>(y) * r.width + static_cast<std::size_t>(x);
                    if (static_cast<float>(a.depth + (b.depth - a.depth) * u) + bias >= zbuf[idx]) {
                        r.pixels[idx] = path.colour;
                    }
                }
            }
        }
    }
    return r;
}

std::vector<ModelPath> wrappedVectorPaths(const document::Document& doc, std::uint32_t colour) {
    std::vector<ModelPath> out;
    const document::Sheet* sheet = doc.findSheet(doc.activeSheet());
    if (sheet == nullptr) {
        return out;
    }
    const double z = doc.job().rotarySurfaceZ();
    ModelPath path;
    path.colour = colour;
    for (const document::Layer& layer : doc.layers()) {
        if (!layer.visible) {
            continue;
        }
        for (const document::ObjectId id : doc.objectsOnLayer(layer.id)) {
            const document::Object* obj = doc.findObject(id);
            if (obj == nullptr || obj->sheet != sheet->id || obj->asVector() == nullptr) {
                continue;
            }
            for (const geometry::Contour& c : obj->asVector()->contours) {
                if (c.isEmpty()) {
                    continue;
                }
                const geometry::Polyline poly = c.polygonise(geometry::Tolerance{});
                std::vector<ModelPathPoint> run;
                for (const geometry::Vec2& p : poly.points) {
                    run.push_back({p.x, p.y, z});
                }
                if (poly.closed && !run.empty()) {
                    run.push_back(run.front());
                }
                if (run.size() > 1) {
                    path.runs.push_back(std::move(run));
                }
            }
        }
    }
    if (!path.runs.empty()) {
        out.push_back(std::move(path));
    }
    return out;
}

std::vector<ModelPath> wrappedToolpathPaths(const std::vector<document::ToolpathRecord>& records, double arcStepRad,
                                            std::size_t maxPoints) {
    static constexpr std::uint32_t kColours[] = {0xFFD02020u, 0xFF2060D0u, 0xFF20A040u, 0xFFA020C0u, 0xFFD08010u};
    const double step = std::clamp(std::isfinite(arcStepRad) ? arcStepRad : 0.1, 0.005, 1.0);
    std::vector<ModelPath> out;
    std::size_t colour = 0;
    for (const document::ToolpathRecord& rec : records) {
        if (!rec.hasMoves() || rec.moves->empty()) {
            continue;
        }
        ModelPath path;
        path.colour = kColours[colour++ % (sizeof(kColours) / sizeof(kColours[0]))];
        std::vector<ModelPathPoint> run;
        std::size_t points = 0;
        geometry::Vec3 prev{};
        bool havePrev = false;
        for (const toolpath::Move& m : *rec.moves) {
            const bool cutting = m.kind == toolpath::MoveKind::Feed || m.kind == toolpath::MoveKind::Plunge ||
                                 m.kind == toolpath::MoveKind::Retract || toolpath::isArc(m.kind);
            if (!cutting) {
                if (run.size() > 1) {
                    path.runs.push_back(std::move(run));
                }
                run.clear();
            } else if (points < maxPoints) {
                if (run.empty() && havePrev) {
                    run.push_back({prev.x, prev.y, prev.z});
                }
                if (toolpath::isArc(m.kind) && havePrev) {
                    const double sweep = toolpath::arcSweep(m, prev.xy());
                    const double r = toolpath::arcRadius(m, prev.xy());
                    const double n = std::min(std::ceil(std::fabs(sweep) / step), 10000.0);
                    const double a0 = std::atan2(prev.y - m.centre.y, prev.x - m.centre.x);
                    for (double k = 1.0; k < n; k += 1.0) {
                        const double t = k / n;
                        run.push_back({m.centre.x + r * std::cos(a0 + sweep * t),
                                       m.centre.y + r * std::sin(a0 + sweep * t), prev.z + (m.to.z - prev.z) * t});
                        ++points;
                    }
                }
                run.push_back({m.to.x, m.to.y, m.to.z});
                ++points;
            }
            prev = m.to;
            havePrev = true;
        }
        if (run.size() > 1) {
            path.runs.push_back(std::move(run));
        }
        if (!path.runs.empty()) {
            out.push_back(std::move(path));
        }
    }
    return out;
}

Raster renderRotaryModel(const ModelSurface& model, const document::Job& job, const RotaryViewState& state,
                         const OrbitCamera& camera, const ModelRenderOptions& options,
                         const std::vector<ModelPath>& wrappedPaths) {
    if (state.effective(job) == RotaryViewMode::Flat) {
        return renderModel(model, camera, options);
    }
    WrappedRenderOptions w;
    w.width = options.width;
    w.height = options.height;
    w.material = options.material;
    w.background = options.background;
    w.paths = wrappedPaths;
    return renderWrapped(wrappedSurfaceFromModel(model, job), cylinderMapFor(job), camera, w);
}

}  // namespace carvetto::viewmodel
