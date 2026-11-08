#pragma once

// Platform-independent elementary functions for the simulator (ADR-0032 §7). The heightfield must
// be byte-identical on every compiler and standard library, and libm's sin/cos/tan/atan2 differ
// in the last bit between glibc, the Microsoft CRT and others. These versions use only + - * / and
// IEEE correctly rounded sqrt with a fixed operation order and a fixed iteration count (the core
// is built without floating-point contraction), so the same inputs give the same bits everywhere.
// Accuracy is about 1e-16 relative, far below anything the simulator resolves.

#include <cmath>

namespace carvetto::simulate::detail {

inline constexpr double kPi = 3.14159265358979323846;
inline constexpr double kHalfPi = 1.57079632679489661923;
inline constexpr double kTwoPi = 6.28318530717958647692;

/// sin and cos of x in [0, pi/2] (Taylor series, 15 terms).
inline void sinCosQuarter(double x, double& s, double& c) {
    const double x2 = x * x;
    double ts = x;
    double tc = 1.0;
    double sumS = x;
    double sumC = 1.0;
    for (int n = 1; n <= 15; ++n) {
        const double a = static_cast<double>(2 * n);
        tc = -tc * x2 / (a * (a - 1.0));
        ts = -ts * x2 / (a * (a + 1.0));
        sumC += tc;
        sumS += ts;
    }
    s = sumS;
    c = sumC;
}

/// atan(t) for t in [0, 1].
inline double atanUnit(double t) {
    // Two argument halvings (atan t = 2 atan(t / (1 + sqrt(1 + t^2)))) bring t below tan(pi/16).
    double u = t / (1.0 + std::sqrt(1.0 + t * t));
    u = u / (1.0 + std::sqrt(1.0 + u * u));
    const double u2 = u * u;
    double term = u;
    double sum = u;
    for (int n = 1; n <= 14; ++n) {
        term = -term * u2;
        sum += term / static_cast<double>(2 * n + 1);
    }
    return 4.0 * sum;
}

/// atan2(y, x) in (-pi, pi]; (0, 0) gives 0.
inline double atan2Det(double y, double x) {
    const double ay = y < 0.0 ? -y : y;
    const double ax = x < 0.0 ? -x : x;
    if (ax == 0.0 && ay == 0.0) {
        return 0.0;
    }
    double a;
    if (ay <= ax) {
        a = atanUnit(ay / ax);
    } else {
        a = kHalfPi - atanUnit(ax / ay);
    }
    if (x < 0.0) {
        a = kPi - a;
    }
    return y < 0.0 ? -a : a;
}

}  // namespace carvetto::simulate::detail
