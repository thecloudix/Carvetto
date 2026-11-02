#include "carvetto/geometry/polyline.hpp"

#include <algorithm>

namespace carvetto::geometry {

std::size_t Polyline::segmentCount() const {
    if (points.size() < 2) {
        return 0;
    }
    return closed ? points.size() : points.size() - 1;
}

double Polyline::length() const {
    double total = 0.0;
    const std::size_t n = segmentCount();
    for (std::size_t i = 0; i < n; ++i) {
        total += distance(segmentStart(i), segmentEnd(i));
    }
    return total;
}

Box2 Polyline::bounds() const {
    Box2 box;
    for (const Vec2 p : points) {
        box.expand(p);
    }
    return box;
}

Vec2 Polyline::pointAt(double t) const {
    if (points.empty()) {
        return {};
    }
    const std::size_t n = segmentCount();
    if (n == 0) {
        return points.front();
    }
    t = std::clamp(t, 0.0, 1.0);
    const double target = t * length();
    double travelled = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const Vec2 a = segmentStart(i);
        const Vec2 b = segmentEnd(i);
        const double seg = distance(a, b);
        if (travelled + seg >= target && seg > 0.0) {
            return lerp(a, b, (target - travelled) / seg);
        }
        travelled += seg;
    }
    return segmentEnd(n - 1);
}

void Polyline::reverse() {
    if (points.size() < 2) {
        return;
    }
    if (closed) {
        // Keep the first vertex in place so the start point is stable; reverse the rest.
        std::reverse(points.begin() + 1, points.end());
    } else {
        std::reverse(points.begin(), points.end());
    }
}

Polyline Polyline::reversed() const {
    Polyline p = *this;
    p.reverse();
    return p;
}

void Polyline::transform(const Transform2& m) {
    for (Vec2& p : points) {
        p = m.apply(p);
    }
}

Polyline Polyline::transformed(const Transform2& m) const {
    Polyline p = *this;
    p.transform(m);
    return p;
}

void Polyline::removeDuplicates(double tol) {
    if (points.size() < 2) {
        return;
    }
    std::vector<Vec2> kept;
    kept.reserve(points.size());
    kept.push_back(points.front());
    for (std::size_t i = 1; i < points.size(); ++i) {
        if (distance(points[i], kept.back()) > tol) {
            kept.push_back(points[i]);
        }
    }
    if (closed && kept.size() > 1 && distance(kept.front(), kept.back()) <= tol) {
        kept.pop_back();
    }
    points = std::move(kept);
}

double Polyline::signedArea() const {
    if (!closed || points.size() < 3) {
        return 0.0;
    }
    double sum = 0.0;
    for (std::size_t i = 0; i < points.size(); ++i) {
        sum += segmentStart(i).cross(segmentEnd(i));
    }
    return 0.5 * sum;
}

}  // namespace carvetto::geometry
