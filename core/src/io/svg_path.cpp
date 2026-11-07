#include "svg_path.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "dxf_curves.hpp"

namespace carvetto::io::detail {

using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::kPi;
using geometry::kTwoPi;
using geometry::LineSpan;
using geometry::Span;
using geometry::Transform2;
using geometry::Vec2;

namespace {

constexpr double kMaxCoordinate = 1.0e12;  // mm; larger geometry is refused as out of range

bool isDigit(char c) {
    return c >= '0' && c <= '9';
}

bool isWs(char c) {
    return c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f';
}

bool inRange(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::fabs(p.x) <= kMaxCoordinate &&
           std::fabs(p.y) <= kMaxCoordinate;
}

bool spanInRange(const Span& s) {
    return std::visit(
        [](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                return inRange(v.start) && inRange(v.end);
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                return inRange(v.centre) && std::isfinite(v.radius) && v.radius <= kMaxCoordinate &&
                       std::isfinite(v.startAngle) && std::isfinite(v.endAngle);
            } else {
                return inRange(v.p0) && inRange(v.p1) && inRange(v.p2) && inRange(v.p3);
            }
        },
        s);
}

}  // namespace

bool scanSvgNumber(std::string_view s, std::size_t& pos, double& out) {
    const std::size_t begin = pos;
    std::size_t i = pos;
    if (i < s.size() && (s[i] == '+' || s[i] == '-')) {
        ++i;
    }
    std::size_t intDigits = 0;
    std::size_t firstNonZeroInt = std::string_view::npos;
    while (i < s.size() && isDigit(s[i])) {
        if (s[i] != '0' && firstNonZeroInt == std::string_view::npos) {
            firstNonZeroInt = intDigits;
        }
        ++intDigits;
        ++i;
    }
    std::size_t fracDigits = 0;
    std::size_t leadingFracZeros = 0;
    bool seenNonZeroFrac = false;
    if (i < s.size() && s[i] == '.') {
        std::size_t j = i + 1;
        while (j < s.size() && isDigit(s[j])) {
            if (s[j] == '0' && !seenNonZeroFrac) {
                ++leadingFracZeros;
            } else {
                seenNonZeroFrac = true;
            }
            ++fracDigits;
            ++j;
        }
        if (intDigits + fracDigits > 0) {
            i = j;
        }
    }
    if (intDigits + fracDigits == 0) {
        return false;
    }
    long long exponent = 0;
    if (i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
        std::size_t j = i + 1;
        bool neg = false;
        if (j < s.size() && (s[j] == '+' || s[j] == '-')) {
            neg = s[j] == '-';
            ++j;
        }
        if (j < s.size() && isDigit(s[j])) {
            while (j < s.size() && isDigit(s[j])) {
                if (exponent < 100000000) {
                    exponent = exponent * 10 + (s[j] - '0');
                }
                ++j;
            }
            if (neg) {
                exponent = -exponent;
            }
            i = j;
        }
    }
    std::size_t from = begin;
    if (from < s.size() && s[from] == '+') {
        ++from;
    }
    double v = 0.0;
    const auto [ptr, ec] = std::from_chars(s.data() + from, s.data() + i, v);
    if (ec == std::errc::result_out_of_range) {
        // Overflow fails; underflow reads as zero. Decide from the decimal magnitude.
        long long mag = 0;
        if (firstNonZeroInt != std::string_view::npos) {
            mag = static_cast<long long>(intDigits - firstNonZeroInt) - 1;
        } else {
            mag = -static_cast<long long>(leadingFracZeros) - 1;
        }
        if (mag + exponent > 0) {
            return false;
        }
        v = (s[begin] == '-') ? -0.0 : 0.0;
    } else if (ec != std::errc{} || ptr != s.data() + i) {
        return false;
    }
    if (!std::isfinite(v)) {
        return false;
    }
    out = v == 0.0 ? 0.0 : v;
    pos = i;
    return true;
}

PathBuilder::PathBuilder(const Transform2& ctm, double chordal, double distance, std::size_t spanBudget)
    : ctm_(ctm), chordal_(chordal), distance_(distance), budget_(spanBudget) {}

void PathBuilder::add(Span s) {
    if (used_ >= budget_) {
        overflow_ = true;
        return;
    }
    if (!spanInRange(s)) {
        bad_ = true;
        return;
    }
    ++used_;
    open_.spans.push_back(std::move(s));
}

void PathBuilder::flush(bool closed) {
    if (bad_) {
        ++dropped_;
    } else if (!open_.spans.empty()) {
        open_.closed = closed;
        out_.push_back(std::move(open_));
    }
    open_ = Contour{};
    bad_ = false;
}

void PathBuilder::finish() {
    flush(false);
}

void PathBuilder::moveTo(Vec2 p) {
    flush(false);
    cur_ = p;
    start_ = p;
}

void PathBuilder::lineTo(Vec2 p) {
    const Vec2 a = ctm_.apply(cur_);
    const Vec2 b = ctm_.apply(p);
    cur_ = p;
    if (!(a == b)) {
        add(LineSpan{a, b});
    }
}

void PathBuilder::cubicTo(Vec2 c1, Vec2 c2, Vec2 p) {
    const BezierSpan b{ctm_.apply(cur_), ctm_.apply(c1), ctm_.apply(c2), ctm_.apply(p)};
    cur_ = p;
    if (!(b.p0 == b.p1 && b.p1 == b.p2 && b.p2 == b.p3)) {
        add(b);
    }
}

void PathBuilder::quadTo(Vec2 c, Vec2 p) {
    const Vec2 p0 = cur_;
    cubicTo(p0 + (c - p0) * (2.0 / 3.0), p + (c - p) * (2.0 / 3.0), p);
}

void PathBuilder::arcTo(double rx, double ry, double phiDegrees, bool largeArc, bool sweep, Vec2 p) {
    const Vec2 p1 = cur_;
    if (p1 == p) {
        return;  // endpoints coincide: the arc is omitted
    }
    rx = std::fabs(rx);
    ry = std::fabs(ry);
    if (!(rx > 0.0) || !(ry > 0.0)) {
        lineTo(p);
        return;
    }
    if (rx > kMaxCoordinate || ry > kMaxCoordinate) {
        ++fallbacks_;
        lineTo(p);
        return;
    }
    const double phi = std::fmod(phiDegrees, 360.0) * kPi / 180.0;
    const double cp = std::cos(phi);
    const double sp = std::sin(phi);
    const double dx = (p1.x - p.x) * 0.5;
    const double dy = (p1.y - p.y) * 0.5;
    const double x1p = cp * dx + sp * dy;
    const double y1p = -sp * dx + cp * dy;
    double a = x1p / rx;
    double b = y1p / ry;
    const double lambda = a * a + b * b;
    if (!(lambda > 0.0) || !std::isfinite(lambda)) {
        ++fallbacks_;
        lineTo(p);
        return;
    }
    double coef = 0.0;
    if (lambda > 1.0) {
        const double k = std::sqrt(lambda);
        rx *= k;
        ry *= k;
        a /= k;
        b /= k;
    } else {
        coef = std::sqrt(std::max(0.0, (1.0 - lambda) / lambda));
        if (largeArc == sweep) {
            coef = -coef;
        }
    }
    const double cxp = coef * rx * b;
    const double cyp = -coef * ry * a;
    const Vec2 centre{cp * cxp - sp * cyp + (p1.x + p.x) * 0.5, sp * cxp + cp * cyp + (p1.y + p.y) * 0.5};
    const double ux = (x1p - cxp) / rx;
    const double uy = (y1p - cyp) / ry;
    const double vx = (-x1p - cxp) / rx;
    const double vy = (-y1p - cyp) / ry;
    const double theta1 = std::atan2(uy, ux);
    double delta = std::atan2(vy, vx) - theta1;
    if (!sweep && delta > 0.0) {
        delta -= kTwoPi;
    } else if (sweep && delta < 0.0) {
        delta += kTwoPi;
    }
    const Vec2 u = ctm_.applyVector(Vec2{rx * cp, rx * sp});
    const Vec2 v = ctm_.applyVector(Vec2{-ry * sp, ry * cp});
    std::vector<Span> spans;
    const std::size_t remaining = used_ >= budget_ ? 0 : budget_ - used_;
    const CurveStatus st = appendEllipticalArc(ctm_.apply(centre), u, v, theta1, delta, chordal_, remaining, spans);
    if (st == CurveStatus::TooManySpans) {
        overflow_ = true;
        cur_ = p;
        return;
    }
    if (st == CurveStatus::Invalid) {
        ++fallbacks_;
        lineTo(p);
        return;
    }
    cur_ = p;
    for (Span& s : spans) {
        add(std::move(s));
    }
}

void PathBuilder::close() {
    if (!(cur_ == start_)) {
        // Closer than the distance tolerance (in mm): the contour is closed as it is, without a
        // sliver line; the ends keep their own coordinates.
        if (geometry::distance(ctm_.apply(cur_), ctm_.apply(start_)) > distance_) {
            lineTo(start_);
        }
    }
    flush(true);
    cur_ = start_;
}

void PathBuilder::ellipse(Vec2 centre, double rx, double ry) {
    flush(false);
    std::vector<Span> spans;
    const std::size_t remaining = used_ >= budget_ ? 0 : budget_ - used_;
    const CurveStatus st = appendEllipticalArc(ctm_.apply(centre), ctm_.applyVector(Vec2{rx, 0.0}),
                                               ctm_.applyVector(Vec2{0.0, ry}), 0.0, kTwoPi, chordal_, remaining, spans);
    if (st == CurveStatus::TooManySpans) {
        overflow_ = true;
        return;
    }
    if (st == CurveStatus::Invalid) {
        ++dropped_;
        return;
    }
    for (Span& s : spans) {
        add(std::move(s));
    }
    flush(true);
}

namespace {

class PathParser {
   public:
    PathParser(std::string_view d, PathBuilder& b) : d_(d), b_(b) {}

    void run(std::string& error) {
        for (;;) {
            skipWs();
            if (pos_ >= d_.size()) {
                break;
            }
            const char ch = d_[pos_];
            if (isCommand(ch)) {
                if (!haveFirst_ && ch != 'M' && ch != 'm') {
                    fail(error, "path data must begin with a moveto");
                    return;
                }
                cmd_ = ch;
                ++pos_;
                haveCmd_ = true;
            } else if (numberStart(ch) && haveCmd_ && cmd_ != 'Z' && cmd_ != 'z') {
                // implicit repetition of the previous command
            } else {
                fail(error, ch == ',' ? "unexpected comma" : "unexpected character");
                return;
            }
            haveFirst_ = true;
            if (cmd_ == 'Z' || cmd_ == 'z') {
                b_.close();
                lastCubic_.reset();
                lastQuad_.reset();
                continue;
            }
            if (!runGroups(error)) {
                return;
            }
        }
    }

   private:
    struct Opt {
        bool has{false};
        Vec2 p;
        void reset() { has = false; }
        void set(Vec2 v) {
            has = true;
            p = v;
        }
    };

    static bool isCommand(char c) {
        return std::string_view{"MmLlHhVvCcSsQqTtAaZz"}.find(c) != std::string_view::npos;
    }
    static bool numberStart(char c) {
        return isDigit(c) || c == '+' || c == '-' || c == '.';
    }
    void skipWs() {
        while (pos_ < d_.size() && isWs(d_[pos_])) {
            ++pos_;
        }
    }
    void fail(std::string& error, const std::string& what) const {
        error = what + " at offset " + std::to_string(pos_);
    }
    bool num(double& v, std::string& error) {
        if (!scanSvgNumber(d_, pos_, v)) {
            fail(error, "number expected");
            return false;
        }
        return true;
    }
    bool flag(bool& v, std::string& error) {
        if (pos_ < d_.size() && (d_[pos_] == '0' || d_[pos_] == '1')) {
            v = d_[pos_] == '1';
            ++pos_;
            return true;
        }
        fail(error, "arc flag (0 or 1) expected");
        return false;
    }
    /// Separator between arguments: whitespace with at most one comma.
    void sep() {
        skipWs();
        if (pos_ < d_.size() && d_[pos_] == ',') {
            ++pos_;
            skipWs();
        }
    }

    /// Reads one or more argument groups of the current command.
    bool runGroups(std::string& error) {
        for (;;) {
            skipWs();
            if (!applyGroup(error)) {
                return false;
            }
            // Another group follows only if a number starts next (a comma may precede it).
            const std::size_t save = pos_;
            sep();
            if (pos_ < d_.size() && numberStart(d_[pos_])) {
                if (cmd_ == 'M') {
                    cmd_ = 'L';
                } else if (cmd_ == 'm') {
                    cmd_ = 'l';
                }
                continue;
            }
            pos_ = save;
            return true;
        }
    }

    bool applyGroup(std::string& error) {
        const bool rel = cmd_ >= 'a' && cmd_ <= 'z';
        const char c = static_cast<char>(rel ? cmd_ - 'a' + 'A' : cmd_);
        const Vec2 cur = b_.current();
        const Vec2 base = rel ? cur : Vec2{0.0, 0.0};
        std::array<double, 7> v{};
        auto read = [&](std::size_t n) {
            for (std::size_t i = 0; i < n; ++i) {
                if (i > 0) {
                    sep();
                }
                if (!num(v[i], error)) {
                    return false;
                }
            }
            return true;
        };
        switch (c) {
            case 'M':
                if (!read(2)) return false;
                b_.moveTo(Vec2{base.x + v[0], base.y + v[1]});
                lastCubic_.reset();
                lastQuad_.reset();
                break;
            case 'L':
                if (!read(2)) return false;
                b_.lineTo(Vec2{base.x + v[0], base.y + v[1]});
                lastCubic_.reset();
                lastQuad_.reset();
                break;
            case 'H':
                if (!read(1)) return false;
                b_.lineTo(Vec2{(rel ? cur.x : 0.0) + v[0], cur.y});
                lastCubic_.reset();
                lastQuad_.reset();
                break;
            case 'V':
                if (!read(1)) return false;
                b_.lineTo(Vec2{cur.x, (rel ? cur.y : 0.0) + v[0]});
                lastCubic_.reset();
                lastQuad_.reset();
                break;
            case 'C': {
                if (!read(6)) return false;
                const Vec2 c1{base.x + v[0], base.y + v[1]};
                const Vec2 c2{base.x + v[2], base.y + v[3]};
                const Vec2 p{base.x + v[4], base.y + v[5]};
                b_.cubicTo(c1, c2, p);
                lastCubic_.set(c2);
                lastQuad_.reset();
                break;
            }
            case 'S': {
                if (!read(4)) return false;
                const Vec2 c1 = lastCubic_.has ? cur * 2.0 - lastCubic_.p : cur;
                const Vec2 c2{base.x + v[0], base.y + v[1]};
                const Vec2 p{base.x + v[2], base.y + v[3]};
                b_.cubicTo(c1, c2, p);
                lastCubic_.set(c2);
                lastQuad_.reset();
                break;
            }
            case 'Q': {
                if (!read(4)) return false;
                const Vec2 c1{base.x + v[0], base.y + v[1]};
                const Vec2 p{base.x + v[2], base.y + v[3]};
                b_.quadTo(c1, p);
                lastQuad_.set(c1);
                lastCubic_.reset();
                break;
            }
            case 'T': {
                if (!read(2)) return false;
                const Vec2 c1 = lastQuad_.has ? cur * 2.0 - lastQuad_.p : cur;
                const Vec2 p{base.x + v[0], base.y + v[1]};
                b_.quadTo(c1, p);
                lastQuad_.set(c1);
                lastCubic_.reset();
                break;
            }
            case 'A': {
                bool large = false;
                bool sweep = false;
                if (!num(v[0], error)) return false;
                sep();
                if (!num(v[1], error)) return false;
                sep();
                if (!num(v[2], error)) return false;
                sep();
                if (!flag(large, error)) return false;
                sep();
                if (!flag(sweep, error)) return false;
                sep();
                if (!num(v[3], error)) return false;
                sep();
                if (!num(v[4], error)) return false;
                b_.arcTo(v[0], v[1], v[2], large, sweep, Vec2{base.x + v[3], base.y + v[4]});
                lastCubic_.reset();
                lastQuad_.reset();
                break;
            }
            default:
                fail(error, "unknown command");
                return false;
        }
        return !b_.overflowed();
    }

    std::string_view d_;
    PathBuilder& b_;
    std::size_t pos_{0};
    char cmd_{0};
    bool haveCmd_{false};
    bool haveFirst_{false};
    Opt lastCubic_;
    Opt lastQuad_;
};

}  // namespace

void parsePathData(std::string_view d, PathBuilder& builder, std::string& error) {
    error.clear();
    PathParser(d, builder).run(error);
    builder.finish();
}

}  // namespace carvetto::io::detail
