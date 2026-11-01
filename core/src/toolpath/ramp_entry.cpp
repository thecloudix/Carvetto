#include "carvetto/toolpath/ramp_entry.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace carvetto::toolpath {

double smoothRampLength(const RampOptions& ramp, double drop) {
    if (ramp.useAngle) {
        const double t = std::tan(ramp.angle);
        return t > 0.0 ? drop / t : 0.0;
    }
    return ramp.distance;
}

std::vector<PathLeg> zigZagLegs(double u, double dir, double L, double zFrom, double zTo,
                                const RampOptions& ramp) {
    std::vector<PathLeg> legs;
    const double drop = zFrom - zTo;
    int m = 2;
    if (ramp.useAngle) {
        const double perLeg = L * std::tan(ramp.angle);
        if (perLeg > 0.0) {
            m = std::max(2, 2 * static_cast<int>(std::ceil(drop / (2.0 * perLeg) - 1e-12)));
        }
    }
    for (int j = 0; j < m; ++j) {
        const double za = zFrom - drop * j / m;
        const double zb = j + 1 == m ? zTo : zFrom - drop * (j + 1) / m;
        const bool out = j % 2 == 0;
        const double a = out ? u : u + dir * L;
        const double b = out ? u + dir * L : u;
        legs.push_back({a, b, ZProfile::linear(za, zb, L)});
    }
    return legs;
}

namespace {

void zigZag(std::vector<PathLeg>& legs, double u, double dir, double L, double zFrom, double zTo,
            const RampOptions& ramp) {
    const std::vector<PathLeg> ramped = zigZagLegs(u, dir, L, zFrom, zTo, ramp);
    legs.insert(legs.end(), ramped.begin(), ramped.end());
}

}  // namespace

PassLegs planPass(const ArcLengthPath& path, double u, double zFrom, double zTo, const RampOptions& ramp) {
    PassLegs p;
    const double len = path.length();
    if (len <= 0.0) {
        return p;
    }
    RampType type = zFrom > zTo ? ramp.type : RampType::None;
    const double drop = zFrom - zTo;
    if (path.closed()) {
        switch (type) {
            case RampType::None:
                p.legs.push_back({u, u + len, ZProfile::flat(zTo)});
                p.endU = u;
                break;
            case RampType::Smooth: {
                const double L = std::clamp(smoothRampLength(ramp, drop), 0.0, len);
                p.legs.push_back({u, u + len + L, ZProfile::linear(zFrom, zTo, L)});
                p.endU = path.wrap(u + L);
                break;
            }
            case RampType::ZigZag: {
                const double L = std::clamp(ramp.distance, 0.0, len);
                if (L > 0.0) {
                    zigZag(p.legs, u, 1.0, L, zFrom, zTo, ramp);
                }
                p.legs.push_back({u, u + len, ZProfile::flat(zTo)});
                p.endU = u;
                break;
            }
            case RampType::Spiral:
                p.legs.push_back({u, u + len, ZProfile::linear(zFrom, zTo, len)});
                p.endU = u;
                break;
        }
        return p;
    }
    // Open path: start at the end `u`, finish at the other end.
    const bool forward = u <= 0.5 * len;
    const double e = forward ? 0.0 : len;
    const double f = forward ? len : 0.0;
    const double dir = forward ? 1.0 : -1.0;
    if (type == RampType::Spiral) {
        type = RampType::Smooth;
    }
    if (type == RampType::Smooth) {
        const double L = std::clamp(smoothRampLength(ramp, drop), 0.0, len);
        if (L > 0.0) {
            p.legs.push_back({e, e + dir * L, ZProfile::linear(zFrom, zTo, L)});
            p.legs.push_back({e + dir * L, e, ZProfile::flat(zTo)});
        }
    } else if (type == RampType::ZigZag) {
        const double L = std::clamp(ramp.distance, 0.0, len);
        if (L > 0.0) {
            zigZag(p.legs, e, dir, L, zFrom, zTo, ramp);
        }
    }
    p.legs.push_back({e, f, ZProfile::flat(zTo)});
    p.endU = f;
    return p;
}

bool appendPathRamp(const geometry::Polyline& path, double zFrom, double zTo, double maxLength, double minLength,
                    const geometry::Tolerance& tol, Toolpath& out) {
    const geometry::Vec2 start = path.points.empty() ? geometry::Vec2{} : path.points.front();
    const double total = path.points.size() < 2 ? 0.0 : path.length();
    const double rampLength = std::min(maxLength, total);
    if (path.points.size() < 2 || !(rampLength > minLength)) {
        out.append(Move::plunge(Vec3{start, zTo}));
        return false;
    }
    // Zero-length segments carry no move (and would emit empty feeds).
    geometry::Polyline clean{{}, path.closed};
    for (const geometry::Vec2& q : path.points) {
        if (clean.points.empty() || geometry::distance(clean.points.back(), q) > 0.0) {
            clean.points.push_back(q);
        }
    }
    while (clean.closed && clean.points.size() > 1 &&
           geometry::distance(clean.points.back(), clean.points.front()) <= 0.0) {
        clean.points.pop_back();
    }
    const ArcLengthPath walk(geometry::Contour::fromPolyline(clean), tol);
    RampOptions zig;
    zig.type = RampType::ZigZag;
    zig.distance = rampLength;
    for (const PathLeg& leg : zigZagLegs(0.0, 1.0, rampLength, zFrom, zTo, zig)) {
        appendLeg(walk, leg.u0, leg.u1, leg.z, out);
    }
    return true;
}

}  // namespace carvetto::toolpath
