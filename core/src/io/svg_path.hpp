#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"

/// Internal: SVG path building and number scanning used by the SVG importer (ADR-0026).
namespace carvetto::io::detail {

/// Scans an SVG number (sign, digits, fraction, exponent) at `pos`. Returns false when no
/// finite number starts there (including overflow such as 1e999); `pos` is then unspecified.
/// Underflow (1e-999) yields 0.
bool scanSvgNumber(std::string_view s, std::size_t& pos, double& out);

/// Collects contours from path commands given in user space; every coordinate is mapped through
/// `ctm` into millimetres.
class PathBuilder {
   public:
    PathBuilder(const geometry::Transform2& ctm, double chordal, std::size_t spanBudget);

    void moveTo(geometry::Vec2 p);
    void lineTo(geometry::Vec2 p);
    void cubicTo(geometry::Vec2 c1, geometry::Vec2 c2, geometry::Vec2 p);
    void quadTo(geometry::Vec2 c, geometry::Vec2 p);
    /// SVG endpoint-parameterised elliptical arc (`phiDegrees` in degrees).
    void arcTo(double rx, double ry, double phiDegrees, bool largeArc, bool sweep, geometry::Vec2 p);
    void close();
    /// A whole ellipse as one closed contour (one exact full-circle arc when circular).
    void ellipse(geometry::Vec2 centre, double rx, double ry);
    /// Ends the current subpath without closing it.
    void finish();

    geometry::Vec2 current() const { return cur_; }
    geometry::Vec2 subpathStart() const { return start_; }

    /// Takes the contours built so far.
    std::vector<geometry::Contour>& contours() { return out_; }
    /// True when the span budget ran out (the result is incomplete).
    bool overflowed() const { return overflow_; }
    /// Number of contours dropped because they held out-of-range or non-finite numbers.
    std::size_t droppedOutOfRange() const { return dropped_; }
    /// Number of arcs replaced by a line (radius out of range or degenerate).
    std::size_t arcFallbacks() const { return fallbacks_; }
    std::size_t spansUsed() const { return used_; }

   private:
    void add(geometry::Span s);
    void flush(bool closed);

    geometry::Transform2 ctm_;
    double chordal_;
    std::size_t budget_;
    std::size_t used_{0};
    geometry::Vec2 cur_;
    geometry::Vec2 start_;
    geometry::Contour open_;
    bool bad_{false};
    bool overflow_{false};
    std::size_t dropped_{0};
    std::size_t fallbacks_{0};
    std::vector<geometry::Contour> out_;
};

/// Parses path data into `builder`. On a syntax error the segments before it are kept (as the
/// SVG error handling rules say) and `error` describes the problem; otherwise `error` is empty.
void parsePathData(std::string_view d, PathBuilder& builder, std::string& error);

}  // namespace carvetto::io::detail
