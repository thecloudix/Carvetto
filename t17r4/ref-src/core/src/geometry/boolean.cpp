#include "carvetto/geometry/boolean.hpp"

#include "clipper_interop.hpp"

namespace carvetto::geometry {

namespace {

Clipper2Lib::ClipType toClipType(BooleanOp op) {
    switch (op) {
        case BooleanOp::Union:
            return Clipper2Lib::ClipType::Union;
        case BooleanOp::Difference:
            return Clipper2Lib::ClipType::Difference;
        case BooleanOp::Intersection:
            return Clipper2Lib::ClipType::Intersection;
        case BooleanOp::Xor:
            return Clipper2Lib::ClipType::Xor;
    }
    return Clipper2Lib::ClipType::Union;
}

Clipper2Lib::FillRule toFillRule(FillRule fill) {
    return fill == FillRule::EvenOdd ? Clipper2Lib::FillRule::EvenOdd : Clipper2Lib::FillRule::NonZero;
}

}  // namespace

BooleanResult booleanOp(BooleanOp op, const std::vector<Contour>& subject, const std::vector<Contour>& clip,
                        const Tolerance& tol, FillRule fill) {
    Clipper2Lib::Paths64 closedSubjects;
    Clipper2Lib::Paths64 openSubjects;
    Clipper2Lib::Paths64 clips;
    for (const Contour& c : subject) {
        Clipper2Lib::Path64 p = detail::toClipperPath(c, tol);
        if (p.empty()) {
            continue;
        }
        (c.closed ? closedSubjects : openSubjects).push_back(std::move(p));
    }
    for (const Contour& c : clip) {
        if (!c.closed) {
            continue;
        }
        Clipper2Lib::Path64 p = detail::toClipperPath(c, tol);
        if (!p.empty()) {
            clips.push_back(std::move(p));
        }
    }

    BooleanResult result;
    if (closedSubjects.empty() && openSubjects.empty()) {
        return result;
    }

    Clipper2Lib::Clipper64 clipper;
    clipper.PreserveCollinear(false);
    clipper.AddSubject(closedSubjects);
    clipper.AddOpenSubject(openSubjects);
    clipper.AddClip(clips);
    Clipper2Lib::Paths64 closedOut;
    Clipper2Lib::Paths64 openOut;
    clipper.Execute(toClipType(op), toFillRule(fill), closedOut, openOut);

    result.closed = detail::contoursFromClipper(closedOut, true);
    result.open = detail::contoursFromClipper(openOut, false);
    return result;
}

std::vector<Contour> unionContours(const std::vector<Contour>& contours, const Tolerance& tol,
                                   FillRule fill) {
    return booleanOp(BooleanOp::Union, contours, {}, tol, fill).closed;
}

}  // namespace carvetto::geometry
