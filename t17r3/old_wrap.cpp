// Compiled with -Dcarvetto=cvold against main's headers.
#include "pw_old.hpp"
#include <variant>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;

std::vector<PContour> oldOffset(const PContour& in, double d, int side, int join, double tolDist, double mitre) {
    Contour c;
    c.closed = in.closed;
    for (const PSpan& s : in.s) {
        if (s.kind == 0) c.spans.push_back(LineSpan{{s.a[0], s.a[1]}, {s.a[2], s.a[3]}});
        else c.spans.push_back(ArcSpan{{s.a[0], s.a[1]}, s.a[2], s.a[3], s.a[4], s.a[5] == 0 ? ArcDirection::CCW : ArcDirection::CW});
    }
    Tolerance tol;
    tol.distance = tolDist;
    auto res = offsetOpenContour(c, d, side == 0 ? OffsetSide::Left : OffsetSide::Right,
                                 join == 0 ? JoinType::Round : (join == 1 ? JoinType::Sharp : JoinType::Square), tol, mitre);
    std::vector<PContour> out;
    for (const Contour& r : res) {
        PContour p;
        p.closed = r.closed;
        for (const Span& s : r.spans) {
            PSpan q{};
            if (auto* l = std::get_if<LineSpan>(&s)) {
                q.kind = 0; q.a[0] = l->start.x; q.a[1] = l->start.y; q.a[2] = l->end.x; q.a[3] = l->end.y;
            } else if (auto* a = std::get_if<ArcSpan>(&s)) {
                q.kind = 1; q.a[0] = a->centre.x; q.a[1] = a->centre.y; q.a[2] = a->radius; q.a[3] = a->startAngle;
                q.a[4] = a->endAngle; q.a[5] = a->direction == ArcDirection::CCW ? 0 : 1;
            } else {
                q.kind = 2;
            }
            p.s.push_back(q);
        }
        out.push_back(p);
    }
    return out;
}
