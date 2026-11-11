#include <cmath>
#include <string>
#include <variant>

#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/vector_texture.hpp"

namespace carvetto::geometry {

namespace {

constexpr double kMaxRegionCoord = 1e9;

bool inRange(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::fabs(p.x) <= kMaxRegionCoord &&
           std::fabs(p.y) <= kMaxRegionCoord;
}

bool spanOk(const Span& span) {
    if (const auto* l = std::get_if<LineSpan>(&span)) return inRange(l->start) && inRange(l->end);
    if (const auto* a = std::get_if<ArcSpan>(&span))
        return inRange(a->centre) && std::isfinite(a->radius) && std::fabs(a->radius) <= kMaxRegionCoord &&
               std::isfinite(a->startAngle) && std::isfinite(a->endAngle);
    const auto& b = std::get<BezierSpan>(span);
    return inRange(b.p0) && inRange(b.p1) && inRange(b.p2) && inRange(b.p3);
}

}  // namespace

ShapeResult<std::vector<Contour>> generateVectorTextureInRegion(const std::vector<Contour>& region,
                                                                const VectorTextureParams& p,
                                                                const Tolerance& tol, FillRule fill) {
    using R = ShapeResult<std::vector<Contour>>;
    std::vector<Contour> clip;
    std::size_t spans = 0;
    for (const Contour& c : region) {
        if (!c.closed || c.spans.empty()) continue;
        spans += c.spans.size();
        if (spans > kVectorTextureMaxRegionSpans)
            return R{std::string("the texture region is too complex (more than 200000 spans)")};
        for (const Span& s : c.spans)
            if (!spanOk(s)) return R{std::string("the texture region has non-finite or out-of-range coordinates")};
        clip.push_back(c);
    }
    if (clip.empty()) return R{std::string("the texture region needs a closed vector")};
    Box2 box = clip.front().bounds();
    for (const Contour& c : clip) box.expand(c.bounds());

    auto lines = generateVectorTextureContours(box, p);
    if (!lines.ok()) return R{lines.error()};

    BooleanResult clipped = booleanOp(BooleanOp::Intersection, lines.value(), clip, tol, fill);
    if (clipped.refusedContours > 0)
        return R{std::string("the texture region is too fine to clip; simplify it or use a larger tolerance")};
    return std::move(clipped.open);
}

}  // namespace carvetto::geometry
