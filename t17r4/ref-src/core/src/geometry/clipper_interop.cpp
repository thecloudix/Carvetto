#include "clipper_interop.hpp"

#include <cmath>
#include <cstdint>

#include "carvetto/geometry/fixed_point.hpp"

namespace carvetto::geometry::detail {

Clipper2Lib::Point64 toClipper(Vec2 p) {
    return {static_cast<std::int64_t>(std::llround(p.x * kFixedPointScale)),
            static_cast<std::int64_t>(std::llround(p.y * kFixedPointScale))};
}

Vec2 fromClipper(const Clipper2Lib::Point64& p) {
    return {static_cast<double>(p.x) / kFixedPointScale, static_cast<double>(p.y) / kFixedPointScale};
}

double toClipperDistance(double mm) {
    return mm * kFixedPointScale;
}

Clipper2Lib::Path64 toClipperPath(const Contour& c, const Tolerance& tol) {
    Polyline pl = c.polygonise(tol);
    pl.removeDuplicates(tol.distance);
    Clipper2Lib::Path64 path;
    path.reserve(pl.points.size());
    for (const Vec2 p : pl.points) {
        const Clipper2Lib::Point64 q = toClipper(p);
        if (path.empty() || !(path.back() == q)) {
            path.push_back(q);
        }
    }
    if (c.closed && path.size() > 1 && path.front() == path.back()) {
        path.pop_back();
    }
    const std::size_t minPoints = c.closed ? 3 : 2;
    if (path.size() < minPoints) {
        path.clear();
    }
    return path;
}

namespace {
Contour contourFromClipper(const Clipper2Lib::Path64& path, bool closed) {
    Contour c;
    c.closed = closed;
    const std::size_t n = path.size();
    if (n < (closed ? 3u : 2u)) {
        return c;
    }
    const std::size_t segments = closed ? n : n - 1;
    c.spans.reserve(segments);
    for (std::size_t i = 0; i < segments; ++i) {
        c.spans.emplace_back(LineSpan{fromClipper(path[i]), fromClipper(path[(i + 1) % n])});
    }
    return c;
}
}  // namespace

Contour closedContourFromClipper(const Clipper2Lib::Path64& path) {
    return contourFromClipper(path, true);
}

Contour openContourFromClipper(const Clipper2Lib::Path64& path) {
    return contourFromClipper(path, false);
}

std::vector<Contour> contoursFromClipper(const Clipper2Lib::Paths64& paths, bool closed) {
    std::vector<Contour> out;
    out.reserve(paths.size());
    for (const auto& path : paths) {
        Contour c = contourFromClipper(path, closed);
        if (!c.isEmpty()) {
            out.push_back(std::move(c));
        }
    }
    return out;
}

}  // namespace carvetto::geometry::detail
