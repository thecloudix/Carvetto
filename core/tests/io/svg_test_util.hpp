#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/io/svg.hpp"

namespace svgtest {

using Bytes = std::vector<std::uint8_t>;

inline Bytes readFixture(const std::string& name) {
    std::ifstream in(std::string(CARVETTO_SVG_FIXTURE_DIR) + "/" + name, std::ios::binary);
    return Bytes(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

inline Bytes toBytes(std::string_view s) {
    return Bytes(s.begin(), s.end());
}

inline carvetto::io::SvgResult<carvetto::io::SvgDrawing> importText(std::string_view text,
                                                                    const carvetto::io::SvgImportOptions& o = {}) {
    const Bytes b = toBytes(text);
    return carvetto::io::importSvg(b, o);
}

/// A 100 x 100 mm page whose user unit is 1 mm; `body` is the content.
inline std::string page(std::string_view body, std::string_view extraRootAttrs = {}) {
    return "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:xlink=\"http://www.w3.org/1999/xlink\" "
           "xmlns:inkscape=\"http://www.inkscape.org/namespaces/inkscape\" width=\"100mm\" height=\"100mm\" "
           "viewBox=\"0 0 100 100\" " +
           std::string(extraRootAttrs) + ">" + std::string(body) + "</svg>";
}

inline bool hasWarning(const carvetto::io::SvgDrawing& d, std::string_view part) {
    return std::any_of(d.warnings.begin(), d.warnings.end(),
                       [&](const carvetto::io::SvgWarning& w) { return w.message.find(part) != std::string::npos; });
}

/// Distance from `p` to the polyline of `pts`.
inline double distToPolyline(carvetto::geometry::Vec2 p, const std::vector<carvetto::geometry::Vec2>& pts) {
    double best = 1e300;
    for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
        const auto a = pts[i];
        const auto b = pts[i + 1];
        const auto ab = b - a;
        const double len2 = ab.lengthSquared();
        double t = len2 > 0.0 ? (p - a).dot(ab) / len2 : 0.0;
        t = std::clamp(t, 0.0, 1.0);
        best = std::min(best, (p - (a + ab * t)).length());
    }
    return best;
}

/// Largest distance between the two contours' outlines (both ways), using fine polygonisation.
inline double outlineDistance(const carvetto::geometry::Contour& a, const carvetto::geometry::Contour& b) {
    carvetto::geometry::Tolerance tol;
    tol.chordal = 1e-4;
    auto pa = a.polygonise(tol).points;
    auto pb = b.polygonise(tol).points;
    if (a.closed && !pa.empty() && !(pa.front() == pa.back())) {
        pa.push_back(pa.front());
    }
    if (b.closed && !pb.empty() && !(pb.front() == pb.back())) {
        pb.push_back(pb.front());
    }
    double worst = 0.0;
    for (const auto& p : pa) {
        worst = std::max(worst, distToPolyline(p, pb));
    }
    for (const auto& p : pb) {
        worst = std::max(worst, distToPolyline(p, pa));
    }
    return worst;
}

}  // namespace svgtest
