#pragma once

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/units.hpp"
#include "carvetto/document/variables.hpp"

// Calculation edit boxes (T-0257, feature-inventory "Calculation Edit Boxes"): a numeric box that
// holds text starting with `=` is an expression (`document::evaluateExpression`): numbers, `+ - * / ^`,
// parentheses, `{DocumentVariable}`, letter variables W/X H/Y T/Z P I M and `'` for feet. The shared
// numeric parsers (`parseNumber`, the toolpath form fields) call `calculate` so every box takes them.
//
// The expression is evaluated in the box's display units: the job's width, height and thickness are
// given to it converted to display units, and the result is the number the box then takes (a length
// in display units, an angle in degrees, ...). A leading `=` is required so plain numbers keep
// their strict parsing.
//
// The document the variables come from is supplied by the app through `setCalculationProvider`
// (the owner clears it before it goes away).

namespace carvetto::viewmodel {

/// What an expression sees: the document's variables and the job size in millimetres.
struct CalculationEnv {
    const document::DocumentVariables* variables{nullptr};
    document::ExpressionContext jobMm;
};

/// The environment of `doc`: its variables and the job's width (x), height (y) and thickness (z).
CalculationEnv environmentOf(const document::Document& doc);

using CalculationProvider = std::function<std::optional<CalculationEnv>()>;

/// Installs the provider of `owner` (replacing its earlier one); the newest installed provider
/// answers. `clearCalculationProvider` removes `owner`'s, the previous one answers again.
void setCalculationProvider(const void* owner, CalculationProvider provider);
void clearCalculationProvider(const void* owner);

/// True when the trimmed text starts with `=`.
bool isExpressionText(std::string_view text);

struct Calculation {
    bool ok{false};
    /// The value in display units (angles: degrees).
    double value{0.0};
    /// The message to show when `!ok`.
    std::string error;
    /// Document variables the expression names (sorted, unique).
    std::vector<std::string> usedVariables;
};

/// Evaluates expression text (with its leading `=`) for a box in `units`. Never throws. Without a
/// provider only constants and plain arithmetic work; `{Name}` is then an error.
Calculation calculate(std::string_view text, document::Units units);

/// `value` as the plain number text a numeric parser reads back exactly enough (17 significant
/// digits are not needed: 12 are kept), '.' separator, no exponent for ordinary magnitudes.
std::string numberText(double value);

/// Replaces `{Name}` in `text` with the variable's value (shortest plain number text); an unknown
/// or malformed `{...}` is left as typed. Used for text that may name variables.
std::string substituteVariables(std::string_view text, const document::DocumentVariables& vars);

}  // namespace carvetto::viewmodel
