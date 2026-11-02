#include "carvetto/toolpath/ramp_entry.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace carvetto::toolpath {

double smoothRampLength(const RampOptions& ramp, double drop) {
    if (ramp.useAngle) {
        const double t = std::tan(ramp.angle);
        return t > 0.0 ? drop / t : 0.0;
    }
    return ramp.distance;
}

namespace {

void zigZag(std::vector<PathLeg>& legs, double u, double dir, double L, double zFrom, double zTo,
            const RampOptions& ramp) {
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

}  // namespace carvetto::toolpath
