#pragma once

// Brute-force oracles for medial axis tests (T-0152).
//
// - `bruteDistance`: exact distance to the boundary, minimum of closestPoint over every span.
// - `isMedial`: whether a point with radius r is equidistant, within eps, from two boundary
//   features. The boundary is sampled densely (true curve points, arc-length step <= eps / 8);
//   the samples within r + eps of the point form runs along each contour. The point passes when
//   there are two runs (two separate features), or one run that contains a corner (a span
//   junction where the tangent turns: both legs are within r + eps), or one run along which the
//   distance has two local minima (equidistant from two points of one curved stretch, e.g.
//   near a curved tip), or one run whose ends are much farther apart than a single straight
//   wall allows (2 sqrt(2 r eps + eps^2), by a factor max(2, sqrt(r / eps) / 2)): within a few
//   eps of the centre of curvature of an arc or of a curved tip.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <variant>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/medial_axis.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::test::medial {

using geometry::Contour;
using geometry::Span;
using geometry::Vec2;

inline double bruteDistance(const std::vector<Contour>& contours, Vec2 p) {
    double best = std::numeric_limits<double>::infinity();
    for (const Contour& c : contours) {
        for (const Span& s : c.spans) {
            best = std::min(best, geometry::closestPoint(s, p).distance);
        }
    }
    return best;
}

/// Dense samples of the boundary, one vector per contour, each marked when it is a corner.
struct DenseBoundary {
    struct Pt {
        Vec2 p;
        bool corner{false};
    };
    std::vector<std::vector<Pt>> contours;
    double step{0.0};
};

inline DenseBoundary sampleBoundary(const std::vector<Contour>& contours, double eps) {
    DenseBoundary out;
    out.step = eps / 8.0;
    for (const Contour& c : contours) {
        std::vector<DenseBoundary::Pt> pts;
        const std::size_t n = c.spans.size();
        for (std::size_t j = 0; j < n; ++j) {
            const Span& s = c.spans[j];
            const Span& prev = c.spans[(j + n - 1) % n];
            const Vec2 tin = geometry::tangentAt(prev, 1.0);
            const Vec2 tout = geometry::tangentAt(s, 0.0);
            const bool corner = std::fabs(tin.cross(tout)) > 1e-6 || tin.dot(tout) < 0.0;
            const double len = geometry::length(s);
            // Beziers are not arc-length parametrised: oversample by 4.
            const double over = std::holds_alternative<geometry::BezierSpan>(s) ? 4.0 : 1.0;
            const auto k = static_cast<std::size_t>(std::ceil(over * len / out.step)) + 1;
            for (std::size_t i = 0; i < k; ++i) {
                pts.push_back({geometry::pointAt(s, static_cast<double>(i) / static_cast<double>(k)), i == 0 && corner});
            }
        }
        out.contours.push_back(std::move(pts));
    }
    return out;
}

inline bool isMedial(const DenseBoundary& b, Vec2 p, double r, double eps) {
    const double lim = r + eps;
    int runs = 0;
    bool cornerInRun = false;
    bool twoMinima = false;
    double longestChord = 0.0;
    for (const auto& pts : b.contours) {
        const std::size_t n = pts.size();
        if (n == 0) {
            continue;
        }
        // Start at a sample outside the near set so runs do not wrap; all-near is one run.
        std::size_t start = n;
        for (std::size_t i = 0; i < n; ++i) {
            if (geometry::distance(pts[i].p, p) > lim) {
                start = i;
                break;
            }
        }
        if (start == n) {
            ++runs;
            cornerInRun = true;
            longestChord = std::numeric_limits<double>::infinity();
            continue;
        }
        bool in = false;
        Vec2 first;
        int minima = 0;
        for (std::size_t k = 1; k <= n; ++k) {
            const auto& q = pts[(start + k) % n];
            const double d = geometry::distance(q.p, p);
            const bool isNear = d <= lim;
            if (isNear && !in) {
                ++runs;
                first = q.p;
                minima = 0;
            }
            if (isNear) {
                longestChord = std::max(longestChord, geometry::distance(first, q.p));
                const auto& before = pts[(start + k + n - 1) % n];
                const auto& after = pts[(start + k + 1) % n];
                const double db = geometry::distance(before.p, p);
                const double da = geometry::distance(after.p, p);
                if (q.corner && db <= lim && da <= lim) {
                    cornerInRun = true;  // a corner with the run extending on both sides
                }
                if (d < db && d <= da && ++minima >= 2) {
                    twoMinima = true;  // equidistant from two points of one stretch
                }
            }
            in = isNear;
        }
    }
    if (runs >= 2 || (runs == 1 && (cornerInRun || twoMinima))) {
        return true;
    }
    // One run with one minimum: accept only a centre of curvature (or a curved tip). For a point
    // at distance d from the centre of an arc of radius R, the run is about sqrt(R / d) times
    // longer than at a straight wall, so requiring sqrt(r / eps) / 2 accepts points within
    // ~4 eps of the centre.
    const double wall = 2.0 * std::sqrt(2.0 * r * eps + eps * eps);
    const double ratio = std::max(2.0, 0.5 * std::sqrt(r / eps));
    return runs == 1 && longestChord > ratio * wall;
}

/// Every point of the axis (vertices of every branch, and every node).
inline std::vector<geometry::MedialPoint> allPoints(const geometry::MedialAxis& ax) {
    std::vector<geometry::MedialPoint> pts;
    for (const auto& n : ax.nodes) {
        pts.push_back({n.point, n.radius});
    }
    for (const auto& br : ax.branches) {
        pts.insert(pts.end(), br.points.begin(), br.points.end());
    }
    return pts;
}

/// Distance from p to the nearest segment of the axis.
inline double distanceToAxis(const geometry::MedialAxis& ax, Vec2 p) {
    double best = std::numeric_limits<double>::infinity();
    for (const auto& n : ax.nodes) {
        best = std::min(best, geometry::distance(n.point, p));
    }
    for (const auto& br : ax.branches) {
        for (std::size_t i = 0; i + 1 < br.points.size(); ++i) {
            const geometry::LineSpan seg{br.points[i].point, br.points[i + 1].point};
            best = std::min(best, geometry::closestPoint(seg, p).distance);
        }
    }
    return best;
}

}  // namespace carvetto::test::medial
