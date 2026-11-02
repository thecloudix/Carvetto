#include "dxf_curves.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace carvetto::io::detail {

using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::kPi;
using geometry::kTwoPi;
using geometry::LineSpan;
using geometry::Span;
using geometry::Vec2;

double unitArcBezierError(double theta) {
    // Maximum radial deviation of the 4/3*tan(theta/4) cubic from the unit circle:
    // (2/27) * sin^6(theta/4) / cos^2(theta/4) (about 2.7e-4 for a quarter circle).
    const double q = std::fabs(theta) * 0.25;
    const double s = std::sin(q);
    const double c = std::cos(q);
    const double s3 = s * s * s;
    return (2.0 / 27.0) * s3 * s3 / (c * c);
}

double maxSingularValue(Vec2 u, Vec2 v) {
    const double sum = u.lengthSquared() + v.lengthSquared();
    const double det = u.x * v.y - v.x * u.y;
    const double disc = std::sqrt(std::max(0.0, sum * sum - 4.0 * det * det));
    return std::sqrt(0.5 * (sum + disc));
}

namespace {

bool finite(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

}  // namespace

CurveStatus appendEllipticalArc(Vec2 c, Vec2 u, Vec2 v, double t0, double sweep, double chordal,
                                std::size_t budget, std::vector<Span>& out) {
    if (!finite(c) || !finite(u) || !finite(v) || !std::isfinite(t0) || !std::isfinite(sweep) ||
        !(chordal > 0.0)) {
        return CurveStatus::Invalid;
    }
    const bool full = std::fabs(sweep) >= kTwoPi - 1e-12;
    if (full) {
        sweep = sweep < 0.0 ? -kTwoPi : kTwoPi;
    }
    const double lu = u.length();
    const double lv = v.length();
    if (!(lu > 0.0) || !(lv > 0.0) || !std::isfinite(lu * lv)) {
        return CurveStatus::Invalid;
    }
    auto pointAt = [&](double t) { return c + u * std::cos(t) + v * std::sin(t); };
    if (std::fabs(sweep) < 1e-12) {
        // Too small to be an arc (ADR-0005 cannot represent zero sweep): a line, if anything.
        const Vec2 a = pointAt(t0);
        const Vec2 b = pointAt(t0 + sweep);
        if (!(a == b)) {
            if (budget < 1) {
                return CurveStatus::TooManySpans;
            }
            out.emplace_back(LineSpan{a, b});
        }
        return CurveStatus::Ok;
    }
    const double scale = std::max(lu, lv);
    const bool circular = std::fabs(lu - lv) <= 1e-10 * scale && std::fabs(u.dot(v)) <= 1e-10 * lu * lv;
    if (circular) {
        if (budget < 1) {
            return CurveStatus::TooManySpans;
        }
        const double phi = u.angle();
        const double s = u.cross(v) > 0.0 ? 1.0 : -1.0;
        ArcSpan arc;
        arc.centre = c;
        arc.radius = 0.5 * (lu + lv);
        arc.startAngle = geometry::normalizeAngle(phi + s * t0);
        arc.endAngle = full ? arc.startAngle : geometry::normalizeAngle(phi + s * (t0 + sweep));
        arc.direction = s * sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW;
        out.emplace_back(arc);
        return CurveStatus::Ok;
    }
    // General ellipse: an affine image of the unit circle, so the Bezier images of unit-circle
    // pieces deviate by at most sigma_max * unitArcBezierError(piece angle).
    const double sigma = maxSingularValue(u, v);
    const double absSweep = std::fabs(sweep);
    const double ratio = chordal / (1.1 * sigma);
    double theta = 4.0 * std::pow(13.5 * ratio, 1.0 / 6.0);
    theta = std::clamp(theta, 1e-9, kPi * 0.5);
    double nReal = std::ceil(absSweep / theta - 1e-12);
    if (!(nReal >= 1.0)) {
        nReal = 1.0;
    }
    if (nReal > static_cast<double>(budget)) {
        return CurveStatus::TooManySpans;
    }
    auto n = static_cast<std::size_t>(nReal);
    for (int guard = 0;
         guard < 64 && 1.1 * sigma * unitArcBezierError(absSweep / static_cast<double>(n)) > chordal;
         ++guard) {
        ++n;
    }
    if (n > budget) {
        return CurveStatus::TooManySpans;
    }
    const double step = sweep / static_cast<double>(n);
    const double k = (4.0 / 3.0) * std::tan(step * 0.25);
    auto tangentAt = [&](double t) { return v * std::cos(t) - u * std::sin(t); };
    Vec2 p0 = pointAt(t0);
    for (std::size_t i = 0; i < n; ++i) {
        const double a = t0 + static_cast<double>(i) * step;
        const double b = i + 1 == n ? t0 + sweep : a + step;
        const Vec2 p3 = (i + 1 == n && full) ? pointAt(t0) : pointAt(b);
        out.emplace_back(BezierSpan{p0, p0 + tangentAt(a) * k, p3 - tangentAt(b) * k, p3});
        p0 = p3;
    }
    return CurveStatus::Ok;
}

// ---------------------------------------------------------------------------------- NURBS

namespace {

/// Homogeneous 2D point (w*x, w*y, w).
struct H {
    double x{0.0};
    double y{0.0};
    double w{1.0};
};

H mix(const H& a, const H& b, double t) {
    const double s = 1.0 - t;
    return {s * a.x + t * b.x, s * a.y + t * b.y, s * a.w + t * b.w};
}

Vec2 project(const H& h) {
    return {h.x / h.w, h.y / h.w};
}

/// Homogeneous Bezier of degree <= kMaxNurbsDegree (fixed storage, no allocation).
struct HBezier {
    std::array<H, kMaxNurbsDegree + 1> pts{};
    std::size_t size{0};
};

/// Point of a homogeneous Bezier at t (de Casteljau).
Vec2 evalBezier(const HBezier& b, double t) {
    std::array<H, kMaxNurbsDegree + 1> w = b.pts;
    for (std::size_t r = 1; r < b.size; ++r) {
        for (std::size_t i = 0; i + r < b.size; ++i) {
            w[i] = mix(w[i], w[i + 1], t);
        }
    }
    return project(w[0]);
}

/// Splits a homogeneous Bezier at t = 0.5.
void splitBezier(const HBezier& b, HBezier& left, HBezier& right) {
    std::array<H, kMaxNurbsDegree + 1> work = b.pts;
    const std::size_t n = b.size;
    left.size = right.size = n;
    left.pts[0] = work[0];
    right.pts[n - 1] = work[n - 1];
    for (std::size_t r = 1; r < n; ++r) {
        for (std::size_t i = 0; i + r < n; ++i) {
            work[i] = mix(work[i], work[i + 1], 0.5);
        }
        left.pts[r] = work[0];
        right.pts[n - 1 - r] = work[n - 1 - r];
    }
}

/// Samples compared per piece, and the subdivision depth limit per knot span.
constexpr int kSamples = 24;
constexpr int kMaxDepth = 24;

struct Approximator {
    double chordal;
    std::size_t budget;
    std::vector<Span>& out;
    bool overflow{false};

    void emit(const BezierSpan& b) {
        if (b.p0 == b.p1 && b.p1 == b.p2 && b.p2 == b.p3) {
            return;
        }
        if (out.size() >= budget) {
            overflow = true;
            return;
        }
        out.emplace_back(b);
    }

    void piece(const HBezier& b, int depth) {
        if (overflow) {
            return;
        }
        const auto& hb = b.pts;
        const std::size_t p = b.size - 1;
        const Vec2 a = project(hb[0]);
        const Vec2 e = project(hb[p]);
        const double dp = static_cast<double>(p);
        const Vec2 d0 = (project(hb[1]) - a) * (dp * hb[1].w / hb[0].w);
        const Vec2 d1 = (e - project(hb[p - 1])) * (dp * hb[p - 1].w / hb[p].w);
        const BezierSpan cubic{a, a + d0 / 3.0, e - d1 / 3.0, e};
        double worst = 0.0;
        for (int j = 1; j < kSamples; ++j) {
            const double t = static_cast<double>(j) / kSamples;
            worst = std::max(worst, geometry::distance(evalBezier(b, t), cubic.pointAt(t)));
            if (worst > chordal && depth < kMaxDepth) {
                break;  // will subdivide anyway
            }
        }
        if (!(worst > chordal) || depth >= kMaxDepth) {
            emit(cubic);
            return;
        }
        HBezier left;
        HBezier right;
        splitBezier(b, left, right);
        piece(left, depth + 1);
        piece(right, depth + 1);
    }
};

}  // namespace

CurveStatus nurbsToSpans(const NurbsCurve& curve, double chordal, std::size_t budget, std::vector<Span>& out,
                         std::string& why) {
    const int p = curve.degree;
    if (p < 1 || p > kMaxNurbsDegree) {
        why = "unsupported degree " + std::to_string(p);
        return CurveStatus::Invalid;
    }
    const std::size_t up = static_cast<std::size_t>(p);
    const std::size_t n = curve.controls.size();
    if (n < up + 1) {
        why = "too few control points";
        return CurveStatus::Invalid;
    }
    const std::vector<double>& U = curve.knots;
    if (U.size() != n + up + 1) {
        why = "knot count " + std::to_string(U.size()) + " does not match " + std::to_string(n) +
              " control points of degree " + std::to_string(p);
        return CurveStatus::Invalid;
    }
    for (std::size_t i = 0; i < U.size(); ++i) {
        if (!std::isfinite(U[i]) || (i > 0 && U[i] < U[i - 1])) {
            why = "knot vector is not non-decreasing";
            return CurveStatus::Invalid;
        }
    }
    if (!curve.weights.empty() && curve.weights.size() != n) {
        why = "weight count does not match control points";
        return CurveStatus::Invalid;
    }
    bool rational = false;
    for (const double w : curve.weights) {
        if (!(w > 0.0) || !std::isfinite(w)) {
            why = "non-positive weight";
            return CurveStatus::Invalid;
        }
        rational = rational || w != curve.weights.front();
    }
    for (const Vec2& c : curve.controls) {
        if (!finite(c)) {
            why = "non-finite control point";
            return CurveStatus::Invalid;
        }
    }
    if (!(U[up] < U[n])) {
        why = "empty parameter range";
        return CurveStatus::Invalid;
    }
    std::vector<H> hom(n);
    for (std::size_t i = 0; i < n; ++i) {
        const double w = rational ? curve.weights[i] : 1.0;
        hom[i] = {curve.controls[i].x * w, curve.controls[i].y * w, w};
    }
    Approximator approx{chordal, budget, out};
    std::vector<H> d(up + 1);
    HBezier bez;
    bez.size = up + 1;
    for (std::size_t k = up; k < n; ++k) {
        if (!(U[k] < U[k + 1])) {
            continue;
        }
        // Bezier control point i of the knot span [a, b] is the blossom f(a^(p-i), b^i),
        // evaluated by the de Boor recursion with one argument per level.
        const double a = U[k];
        const double b = U[k + 1];
        for (std::size_t i = 0; i <= up; ++i) {
            for (std::size_t j = 0; j <= up; ++j) {
                d[j] = hom[k - up + j];
            }
            for (std::size_t r = 1; r <= up; ++r) {
                const double t = r <= up - i ? a : b;
                for (std::size_t j = up; j >= r; --j) {
                    const std::size_t idx = k - up + j;
                    const double denom = U[idx + up + 1 - r] - U[idx];
                    const double alpha = denom > 0.0 ? (t - U[idx]) / denom : 0.0;
                    d[j] = mix(d[j - 1], d[j], alpha);
                }
            }
            bez.pts[i] = d[up];
        }
        if (!rational && p <= 3) {
            if (out.size() >= budget) {
                return CurveStatus::TooManySpans;
            }
            const Vec2 q0 = project(bez.pts[0]);
            const Vec2 qn = project(bez.pts[up]);
            if (p == 1) {
                if (!(q0 == qn)) {
                    out.emplace_back(LineSpan{q0, qn});
                }
            } else if (p == 2) {
                const Vec2 q1 = project(bez.pts[1]);
                approx.emit({q0, q0 + (q1 - q0) * (2.0 / 3.0), qn + (q1 - qn) * (2.0 / 3.0), qn});
            } else {
                approx.emit({q0, project(bez.pts[1]), project(bez.pts[2]), qn});
            }
        } else {
            approx.piece(bez, 0);
        }
        if (approx.overflow) {
            return CurveStatus::TooManySpans;
        }
    }
    return CurveStatus::Ok;
}

// ------------------------------------------------------------------------- fit points

CurveStatus interpolateFitPoints(std::span<const Vec2> points, std::optional<Vec2> startTangent,
                                 std::optional<Vec2> endTangent, std::size_t budget, std::vector<Span>& out) {
    std::vector<Vec2> q;
    for (const Vec2& p : points) {
        if (!finite(p)) {
            return CurveStatus::Invalid;
        }
        if (q.empty() || geometry::distance(q.back(), p) > 1e-12) {
            q.push_back(p);
        }
    }
    if (q.size() < 2) {
        return CurveStatus::Invalid;
    }
    const std::size_t n = q.size() - 1;  // intervals
    if (n > budget) {
        return CurveStatus::TooManySpans;
    }
    std::vector<double> h(n);
    for (std::size_t i = 0; i < n; ++i) {
        h[i] = geometry::distance(q[i], q[i + 1]);
    }
    auto unit = [](std::optional<Vec2> t) -> std::optional<Vec2> {
        if (!t || !finite(*t) || t->length() <= 0.0) {
            return std::nullopt;
        }
        return t->normalized();
    };
    startTangent = unit(startTangent);
    endTangent = unit(endTangent);
    // Tridiagonal system for the derivatives D_i (w.r.t. chord length) of a C2 cubic spline.
    std::vector<double> lo(n + 1, 0.0);
    std::vector<double> di(n + 1, 0.0);
    std::vector<double> up(n + 1, 0.0);
    std::vector<Vec2> rhs(n + 1);
    if (startTangent) {
        di[0] = 1.0;
        rhs[0] = *startTangent;
    } else {
        di[0] = 2.0;
        up[0] = 1.0;
        rhs[0] = (q[1] - q[0]) * (3.0 / h[0]);
    }
    for (std::size_t i = 1; i < n; ++i) {
        lo[i] = h[i];
        di[i] = 2.0 * (h[i - 1] + h[i]);
        up[i] = h[i - 1];
        rhs[i] = ((q[i] - q[i - 1]) * (h[i] / h[i - 1]) + (q[i + 1] - q[i]) * (h[i - 1] / h[i])) * 3.0;
    }
    if (endTangent) {
        lo[n] = 0.0;
        di[n] = 1.0;
        rhs[n] = *endTangent;
    } else {
        lo[n] = 1.0;
        di[n] = 2.0;
        rhs[n] = (q[n] - q[n - 1]) * (3.0 / h[n - 1]);
    }
    // Thomas algorithm (the system is diagonally dominant).
    for (std::size_t i = 1; i <= n; ++i) {
        const double m = lo[i] / di[i - 1];
        di[i] -= m * up[i - 1];
        rhs[i] -= rhs[i - 1] * m;
    }
    std::vector<Vec2> deriv(n + 1);
    deriv[n] = rhs[n] / di[n];
    for (std::size_t i = n; i-- > 0;) {
        deriv[i] = (rhs[i] - deriv[i + 1] * up[i]) / di[i];
    }
    for (std::size_t i = 0; i < n; ++i) {
        out.emplace_back(BezierSpan{q[i], q[i] + deriv[i] * (h[i] / 3.0),
                                    q[i + 1] - deriv[i + 1] * (h[i] / 3.0), q[i + 1]});
    }
    return CurveStatus::Ok;
}

}  // namespace carvetto::io::detail
