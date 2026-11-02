#include "carvetto/viewmodel/ruler.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <string>

namespace carvetto::viewmodel {

namespace {

double pow10(int exponent) {
    return std::pow(10.0, static_cast<double>(exponent));
}

/// Largest |index| we are willing to turn into a tick (keeps long long arithmetic safe).
constexpr double kMaxTickIndex = 1e15;
/// Upper bound on generated ticks for one ruler (a sanity guard; normal views need < 1000).
constexpr long long kMaxTicks = 20000;

}  // namespace

double RulerStep::value() const {
    return static_cast<double>(mantissa) * pow10(exponent);
}

int RulerStep::minorDivisions() const {
    switch (mantissa) {
        case 2:
            return 4;
        case 5:
            return 5;
        default:
            return 10;
    }
}

RulerStep chooseRulerStep(double pxPerDisplayUnit, double minSpacingPx) {
    if (!(pxPerDisplayUnit > 0.0) || !std::isfinite(pxPerDisplayUnit) || !(minSpacingPx > 0.0)) {
        return {};
    }
    // Start one decade below the estimate so rounding in log10 can never skip a candidate.
    const int start = static_cast<int>(std::floor(std::log10(minSpacingPx / pxPerDisplayUnit))) - 1;
    constexpr std::array<int, 3> kMantissas{1, 2, 5};
    for (int e = start; e < start + 4; ++e) {
        for (const int m : kMantissas) {
            if (static_cast<double>(m) * pow10(e) * pxPerDisplayUnit >= minSpacingPx) {
                return {m, e};
            }
        }
    }
    return {1, start + 4};
}

std::string formatScaledDecimal(long long units, int exponent) {
    if (units == 0) {
        return "0";
    }
    const bool negative = units < 0;
    // Magnitude as unsigned so LLONG_MIN cannot overflow.
    const unsigned long long magnitude =
        negative ? 0ULL - static_cast<unsigned long long>(units) : static_cast<unsigned long long>(units);
    std::string digits = std::to_string(magnitude);
    if (exponent >= 0) {
        digits.append(static_cast<std::size_t>(exponent), '0');
    } else {
        const auto decimals = static_cast<std::size_t>(-exponent);
        if (digits.size() <= decimals) {
            digits.insert(0, decimals - digits.size() + 1, '0');
        }
        digits.insert(digits.size() - decimals, 1, '.');
        while (digits.back() == '0') {
            digits.pop_back();
        }
        if (digits.back() == '.') {
            digits.pop_back();
        }
    }
    return negative ? "-" + digits : digits;
}

RulerModel rulerTicks(const ViewTransform& view, RulerOrientation orientation, document::Units units,
                      double minSpacingPx) {
    RulerModel model;
    const double lengthPx =
        orientation == RulerOrientation::Horizontal ? view.viewportWidth() : view.viewportHeight();
    if (lengthPx <= 0.0) {
        return model;
    }
    const double pxPerUnit = view.scale() * document::fromDisplay(1.0, units);
    model.step = chooseRulerStep(pxPerUnit, minSpacingPx);
    const int divisions = model.step.minorDivisions();
    const double minorStep = model.step.value() / static_cast<double>(divisions);

    // Visible document range along this axis, in display units.
    double a = 0.0;
    double b = 0.0;
    if (orientation == RulerOrientation::Horizontal) {
        a = view.screenToWorld({0.0, 0.0}).x;
        b = view.screenToWorld({lengthPx, 0.0}).x;
    } else {
        a = view.screenToWorld({0.0, lengthPx}).y;
        b = view.screenToWorld({0.0, 0.0}).y;
    }
    a = document::toDisplay(a, units);
    b = document::toDisplay(b, units);
    if (a > b) {
        std::swap(a, b);
    }
    const double first = std::ceil(a / minorStep);
    const double last = std::floor(b / minorStep);
    if (!std::isfinite(first) || !std::isfinite(last) || std::fabs(first) > kMaxTickIndex ||
        std::fabs(last) > kMaxTickIndex || last - first > static_cast<double>(kMaxTicks)) {
        return model;
    }
    const auto iFirst = static_cast<long long>(first);
    const auto iLast = static_cast<long long>(last);
    model.ticks.reserve(static_cast<std::size_t>(std::max(0LL, iLast - iFirst + 1)));
    for (long long i = iFirst; i <= iLast; ++i) {
        RulerTick tick;
        // Position from the integer index so majors are exact multiples of the step.
        const double valueDisplay =
            static_cast<double>(i) * model.step.value() / static_cast<double>(divisions);
        const double valueMm = document::fromDisplay(valueDisplay, units);
        tick.positionPx = orientation == RulerOrientation::Horizontal ? view.worldToScreen({valueMm, 0.0}).x
                                                                      : view.worldToScreen({0.0, valueMm}).y;
        tick.major = i % divisions == 0;
        if (tick.major) {
            tick.label = formatScaledDecimal((i / divisions) * model.step.mantissa, model.step.exponent);
        }
        model.ticks.push_back(std::move(tick));
    }
    if (orientation == RulerOrientation::Vertical) {
        std::reverse(model.ticks.begin(), model.ticks.end());  // top-to-bottom on screen
    }
    return model;
}

}  // namespace carvetto::viewmodel
