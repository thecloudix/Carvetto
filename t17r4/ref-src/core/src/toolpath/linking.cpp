#include "carvetto/toolpath/linking.hpp"

#include <type_traits>
#include <variant>

namespace carvetto::toolpath {

using geometry::ArcDirection;
using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;

void appendContourMoves(const Contour& contour, double z, const LinkingOptions& options, Toolpath& out) {
    for (const auto& span : contour.spans) {
        if (geometry::isDegenerate(span, options.tolerance)) {
            continue;
        }
        std::visit(
            [&](const auto& s) {
                using S = std::decay_t<decltype(s)>;
                if constexpr (std::is_same_v<S, LineSpan>) {
                    out.append(Move::feed(Vec3{s.end, z}));
                } else if constexpr (std::is_same_v<S, ArcSpan>) {
                    const Vec3 end{s.endPoint(), z};
                    out.append(s.direction == ArcDirection::CW ? Move::arcCW(end, s.centre)
                                                               : Move::arcCCW(end, s.centre));
                } else {
                    std::vector<Vec2> pts;
                    geometry::polygoniseInto(span, options.tolerance, pts);
                    for (const Vec2& p : pts) {
                        out.append(Move::feed(Vec3{p, z}));
                    }
                }
            },
            span);
    }
}

void linkContour(const Contour& contour, double z, const LinkingOptions& options, Toolpath& out) {
    if (contour.isEmpty()) {
        return;
    }
    const Vec2 start = contour.startPoint();
    out.append(Move::rapid(Vec3{start, options.safeZ}));
    out.append(Move::plunge(Vec3{start, z}));
    appendContourMoves(contour, z, options, out);
    out.append(Move::retract(Vec3{out.moves.back().to.xy(), options.safeZ}));
}

void linkContours(const std::vector<Contour>& contours, double z, const LinkingOptions& options,
                  Toolpath& out) {
    for (const Contour& c : contours) {
        linkContour(c, z, options, out);
    }
}

void linkContours(const std::vector<Contour>& contours, const std::vector<double>& levels,
                  const LinkingOptions& options, Toolpath& out) {
    if (levels.empty()) {
        return;
    }
    for (const Contour& c : contours) {
        if (c.isEmpty()) {
            continue;
        }
        Contour current = c;
        out.append(Move::rapid(Vec3{current.startPoint(), options.safeZ}));
        for (std::size_t i = 0; i < levels.size(); ++i) {
            out.append(Move::plunge(Vec3{out.moves.back().to.xy(), levels[i]}));
            appendContourMoves(current, levels[i], options, out);
            if (!current.closed && i + 1 < levels.size()) {
                current.reverse();
            }
        }
        out.append(Move::retract(Vec3{out.moves.back().to.xy(), options.safeZ}));
    }
}

}  // namespace carvetto::toolpath
