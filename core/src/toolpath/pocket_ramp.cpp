#include "carvetto/toolpath/pocket_ramp.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace carvetto::toolpath {

using geometry::Polyline;

bool appendPathRamp(const Polyline& path, double zFrom, double zTo, double maxLength, double minLength,
                    Toolpath& out) {
    const std::vector<Vec2>& pts = path.points;
    const Vec2 start = pts.empty() ? Vec2{} : pts.front();
    const double total = pts.size() < 2 ? 0.0 : path.length();
    const double rampLength = std::min(maxLength, total);
    if (pts.size() < 2 || !(rampLength > minLength)) {
        out.append(Move::plunge(Vec3{start, zTo}));
        return false;
    }

    // Forward leg: walk the path (with the closing segment for a closed path) until rampLength,
    // remembering the points passed so the return leg retraces them exactly.
    const double zMid = 0.5 * (zFrom + zTo);
    const double slope = (zFrom - zTo) / (2.0 * rampLength);  // drop per mm travelled
    std::vector<Vec2> visited{start};
    double travelled = 0.0;
    const std::size_t segments = path.segmentCount();
    for (std::size_t i = 0; i < segments && travelled < rampLength; ++i) {
        const Vec2 a = path.segmentStart(i);
        const Vec2 b = path.segmentEnd(i);
        const double len = geometry::distance(a, b);
        if (len <= 0.0) {
            continue;
        }
        if (travelled + len >= rampLength) {
            const Vec2 p = geometry::lerp(a, b, (rampLength - travelled) / len);
            travelled = rampLength;
            visited.push_back(p);
            out.append(Move::feed(Vec3{p, zMid}));
            break;
        }
        travelled += len;
        visited.push_back(b);
        out.append(Move::feed(Vec3{b, zFrom - slope * travelled}));
    }

    // Return leg along the same points, arriving at the start at exactly zTo.
    double back = 0.0;
    for (std::size_t i = visited.size() - 1; i > 0; --i) {
        back += geometry::distance(visited[i], visited[i - 1]);
        const double z = i - 1 == 0 ? zTo : zMid - slope * back;
        out.append(Move::feed(Vec3{visited[i - 1], z}));
    }
    return true;
}

}  // namespace carvetto::toolpath
