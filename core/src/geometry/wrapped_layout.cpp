#include "carvetto/geometry/wrapped_layout.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry {

namespace {

constexpr double kMaxLength = 100000.0;
constexpr double kMinPitch = 1e-3;
/// Fractions of a turn closer than this to the edge of the turn are on it.
constexpr double kSeamSnap = 1e-9;
/// Strand pieces shorter than this along the axis (mm) are dropped.
constexpr double kMinPiece = 1e-9;

/// Flat point of (distance along the axis, circumference coordinate).
Vec2 flat(bool alongX, double along, double circ) { return alongX ? Vec2{along, circ} : Vec2{circ, along}; }

double wrapTo(double v, double period) {
    const double r = std::fmod(v, period);
    return r < 0.0 ? r + period : r;
}

bool finite(double v) { return std::isfinite(v); }

std::string checkCommon(double diameter, double alongStart, double alongEnd, double circStart, double offset,
                        int count, const char* what) {
    if (!finite(diameter) || !finite(alongStart) || !finite(alongEnd) || !finite(circStart) || !finite(offset)) {
        return "a value is not a finite number";
    }
    if (!(diameter > 0.0) || diameter > kMaxLength) {
        return "the diameter must be between 0 and " + std::to_string(static_cast<long long>(kMaxLength)) + " mm";
    }
    if (count < 1 || count > kMaxLayoutStrands) {
        return std::string("the number of ") + what + " must be between 1 and " + std::to_string(kMaxLayoutStrands);
    }
    if (!(alongEnd > alongStart) || std::fabs(alongStart) > kMaxLength || std::fabs(alongEnd) > kMaxLength ||
        std::fabs(circStart) > kMaxLength || std::fabs(offset) > kMaxLength) {
        return "the end along the axis must be beyond the start, and every position within " +
               std::to_string(static_cast<long long>(kMaxLength)) + " mm";
    }
    return {};
}

Contour rectangleFlute(bool alongX, double a0, double a1, double c, double w) {
    Contour out;
    out.closed = true;
    const double lo = c - 0.5 * w;
    const double hi = c + 0.5 * w;
    const Vec2 p0 = flat(alongX, a0, lo);
    const Vec2 p1 = flat(alongX, a1, lo);
    const Vec2 p2 = flat(alongX, a1, hi);
    const Vec2 p3 = flat(alongX, a0, hi);
    out.spans = {LineSpan{p0, p1}, LineSpan{p1, p2}, LineSpan{p2, p3}, LineSpan{p3, p0}};
    return out;
}

Contour stadiumFlute(bool alongX, double a0, double a1, double c, double w) {
    Contour out;
    out.closed = true;
    const double r = 0.5 * w;
    const double lo = c - r;
    const double hi = c + r;
    const Vec2 p0 = flat(alongX, a0 + r, lo);
    const Vec2 p1 = flat(alongX, a1 - r, lo);
    const Vec2 p2 = flat(alongX, a1 - r, hi);
    const Vec2 p3 = flat(alongX, a0 + r, hi);
    // Going round the loop (low side, far end, high side, near end) the turn is counter-clockwise in the
    // (along, circumference) frame, clockwise once the axes are swapped.
    const ArcDirection dir = alongX ? ArcDirection::CCW : ArcDirection::CW;
    out.spans = {LineSpan{p0, p1}, ArcSpan::fromCentre(flat(alongX, a1 - r, c), p1, p2, dir), LineSpan{p2, p3},
                 ArcSpan::fromCentre(flat(alongX, a0 + r, c), p3, p0, dir)};
    return out;
}

}  // namespace

WrappedFlutingResult wrappedFlutingLayout(const WrappedFlutingParams& p) {
    WrappedFlutingResult out;
    if (std::string bad = checkCommon(p.diameter, p.alongStart, p.alongEnd, p.circStart, p.offset, p.flutes, "flutes");
        !bad.empty()) {
        out.error = std::move(bad);
        return out;
    }
    const double turn = kPi * p.diameter;
    out.spacing = turn / p.flutes;
    if (!finite(p.depth) || !(p.depth > 0.0) || !(p.depth < 0.5 * p.diameter)) {
        out.error = "the flute depth must be above zero and less than the radius";
        return out;
    }
    const bool outline = p.shape != FluteShape::CentreLine;
    if (outline) {
        if (!finite(p.width) || !(p.width > 0.0)) {
            out.error = "the flute width must be above zero";
            return out;
        }
        if (p.width > out.spacing) {
            out.error = "the flute width is more than the spacing between flutes";
            return out;
        }
        if (p.shape == FluteShape::Stadium && p.alongEnd - p.alongStart < p.width) {
            out.error = "the flutes are shorter than their width: no room for the rounded ends";
            return out;
        }
    }
    out.contours.reserve(static_cast<std::size_t>(p.flutes));
    for (int i = 0; i < p.flutes; ++i) {
        const double c = p.circStart + wrapTo(p.offset + out.spacing * i, turn);
        switch (p.shape) {
            case FluteShape::CentreLine: {
                Contour line;
                line.spans = {LineSpan{flat(p.alongX, p.alongStart, c), flat(p.alongX, p.alongEnd, c)}};
                out.contours.push_back(std::move(line));
                break;
            }
            case FluteShape::Rectangle:
                out.contours.push_back(rectangleFlute(p.alongX, p.alongStart, p.alongEnd, c, p.width));
                break;
            case FluteShape::Stadium:
                out.contours.push_back(stadiumFlute(p.alongX, p.alongStart, p.alongEnd, c, p.width));
                break;
        }
        if (outline && (c - 0.5 * p.width < p.circStart || c + 0.5 * p.width > p.circStart + turn)) {
            out.crossingSeam.push_back(i);
        }
    }
    return out;
}

WrappedSpiralResult wrappedSpiralLayout(const WrappedSpiralParams& p) {
    WrappedSpiralResult out;
    if (std::string bad = checkCommon(p.diameter, p.alongStart, p.alongEnd, p.circStart, p.offset, p.strands, "strands");
        !bad.empty()) {
        out.error = std::move(bad);
        return out;
    }
    if (!finite(p.value) || !(p.value > 0.0)) {
        out.error = "the pitch, spacing or number of turns must be above zero";
        return out;
    }
    const double turn = kPi * p.diameter;
    const double length = p.alongEnd - p.alongStart;
    switch (p.mode) {
        case SpiralPitchMode::Pitch: out.pitch = p.value; break;
        case SpiralPitchMode::Spacing: out.pitch = p.value * p.strands; break;
        case SpiralPitchMode::Turns: out.pitch = length / p.value; break;
    }
    if (!(out.pitch >= kMinPitch)) {
        out.error = "the pitch is too small (at least 0.001 mm)";
        return out;
    }
    out.turns = length / out.pitch;
    // Vectors: each strand makes about `turns + 1` pieces.
    if (!(std::ceil(out.turns + 1.0) * p.strands <= kMaxLayoutVectors)) {
        out.error = "more than " + std::to_string(kMaxLayoutVectors) + " vectors: use a larger pitch";
        return out;
    }
    const double along = (p.alongX ? -1.0 : 1.0) * (p.flip ? -1.0 : 1.0);
    const double sign = p.twist == SpiralTwist::RightHand ? along : -along;  // circumference change per axis step
    for (int s = 0; s < p.strands; ++s) {
        // Position in turns from the start of the turn, in [0, 1).
        const double f0 = wrapTo(p.offset + turn * s / p.strands, turn) / turn;
        // Distances (in turns along the strand) where it crosses the edge of the turn, ascending.
        std::vector<double> cuts{0.0};
        double t = sign > 0.0 ? 1.0 - f0 : f0;
        if (t <= kSeamSnap) {
            t += 1.0;
        }
        for (; t * out.pitch < length - kMinPiece; t += 1.0) {
            cuts.push_back(t);
        }
        cuts.push_back(out.turns);
        int pieces = 0;
        for (std::size_t k = 0; k + 1 < cuts.size(); ++k) {
            const double ta = cuts[k];
            const double tb = cuts[k + 1];
            if ((tb - ta) * out.pitch < kMinPiece) {
                continue;
            }
            const double cell = std::floor(f0 + sign * 0.5 * (ta + tb));
            const auto circAt = [&](double tau) {
                double r = f0 + sign * tau - cell;
                r = r < kSeamSnap ? 0.0 : (r > 1.0 - kSeamSnap ? 1.0 : r);
                return p.circStart + r * turn;
            };
            Contour line;
            line.spans = {LineSpan{flat(p.alongX, p.alongStart + ta * out.pitch, circAt(ta)),
                                   flat(p.alongX, k + 2 == cuts.size() ? p.alongEnd : p.alongStart + tb * out.pitch,
                                        circAt(tb))}};
            out.contours.push_back(std::move(line));
            ++pieces;
        }
        out.segments.push_back(pieces);
    }
    return out;
}

}  // namespace carvetto::geometry
