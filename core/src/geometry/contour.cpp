#include "carvetto/geometry/contour.hpp"

#include "carvetto/geometry/cancel.hpp"

#include <algorithm>

namespace carvetto::geometry {

Contour Contour::circle(Vec2 centre, double radius, ArcDirection dir) {
    Contour c;
    c.closed = true;
    c.spans.emplace_back(ArcSpan::fullCircle(centre, radius, dir));
    return c;
}

Contour Contour::rectangle(const Box2& box) {
    Contour c;
    c.closed = true;
    const Vec2 bl = box.min;
    const Vec2 br{box.max.x, box.min.y};
    const Vec2 tr = box.max;
    const Vec2 tl{box.min.x, box.max.y};
    c.spans.emplace_back(LineSpan{bl, br});
    c.spans.emplace_back(LineSpan{br, tr});
    c.spans.emplace_back(LineSpan{tr, tl});
    c.spans.emplace_back(LineSpan{tl, bl});
    return c;
}

Contour Contour::fromPolyline(const Polyline& pl) {
    Contour c;
    c.closed = pl.closed;
    const std::size_t n = pl.segmentCount();
    c.spans.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        c.spans.emplace_back(LineSpan{pl.segmentStart(i), pl.segmentEnd(i)});
    }
    return c;
}

Vec2 Contour::startPoint() const {
    return spans.empty() ? Vec2{} : geometry::startPoint(spans.front());
}

Vec2 Contour::endPoint() const {
    return spans.empty() ? Vec2{} : geometry::endPoint(spans.back());
}

double Contour::length() const {
    double total = 0.0;
    for (const Span& s : spans) {
        total += geometry::length(s);
    }
    return total;
}

Box2 Contour::bounds() const {
    Box2 box;
    for (const Span& s : spans) {
        box.expand(geometry::bounds(s));
    }
    return box;
}

Vec2 Contour::pointAt(double t) const {
    if (spans.empty()) {
        return {};
    }
    t = std::clamp(t, 0.0, 1.0);
    const double target = t * length();
    double travelled = 0.0;
    for (const Span& s : spans) {
        const double len = geometry::length(s);
        if (travelled + len >= target && len > 0.0) {
            return geometry::pointAt(s, (target - travelled) / len);
        }
        travelled += len;
    }
    return endPoint();
}

bool Contour::isContinuous(const Tolerance& tol) const {
    for (std::size_t i = 1; i < spans.size(); ++i) {
        if (!tol.equalPoints(geometry::endPoint(spans[i - 1]), geometry::startPoint(spans[i]))) {
            return false;
        }
    }
    if (closed && !spans.empty() && !tol.equalPoints(endPoint(), startPoint())) {
        return false;
    }
    return true;
}

void Contour::reverse() {
    std::reverse(spans.begin(), spans.end());
    for (Span& s : spans) {
        s = geometry::reversed(s);
    }
}

Contour Contour::reversed() const {
    Contour c = *this;
    c.reverse();
    return c;
}

void Contour::transform(const Transform2& m) {
    std::vector<Span> out;
    out.reserve(spans.size());
    for (const Span& s : spans) {
        for (Span& t : geometry::transformed(s, m)) {
            out.push_back(std::move(t));
        }
    }
    spans = std::move(out);
}

Contour Contour::transformed(const Transform2& m) const {
    Contour c = *this;
    c.transform(m);
    return c;
}

Polyline Contour::polygonise(const Tolerance& tol, bool* coarse) const {
    Polyline pl;
    pl.closed = closed;
    if (spans.empty()) {
        return pl;
    }
    pl.points.push_back(startPoint());
    std::size_t index = 0;
    for (const Span& s : spans) {
        // A cancel (T-0228) cuts a long contour short; the caller discards the partial polygon.
        if ((index++ & 31U) == 0U && cancelRequested()) {
            break;
        }
        if (closed) {
            polygoniseRegionInto(s, tol, pl.points, coarse);
        } else {
            polygoniseInto(s, tol, pl.points, coarse);
        }
    }
    if (closed && pl.points.size() > 1 && tol.equalPoints(pl.points.front(), pl.points.back())) {
        pl.points.pop_back();
    }
    return pl;
}

}  // namespace carvetto::geometry
