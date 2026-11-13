#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace carvetto::document {

/// Named numeric values the user can reference as `{Name}` in calculation edit boxes. A name
/// must start with a letter and continue with letters, digits or `_`; names are case-sensitive.
/// Variables referenced by a toolpath (tracked with `addReference`) cannot be removed.
class DocumentVariables {
   public:
    /// Outcome of `importText`.
    struct ImportResult {
        std::size_t imported{0};
        /// 1-based line numbers that could not be parsed or had an invalid name.
        std::vector<std::size_t> rejectedLines;
    };

    /// True when `name` follows the naming rule (ASCII letter, then letters/digits/underscore).
    static bool isValidName(std::string_view name);

    /// Creates or overwrites `name`. Returns false (and changes nothing) for an invalid name.
    bool set(const std::string& name, double value);
    /// Value of `name`, or nullopt when undefined.
    std::optional<double> get(std::string_view name) const;
    bool contains(std::string_view name) const;
    /// Removes `name`. Returns false when it is undefined or still referenced.
    bool remove(std::string_view name);
    /// Removes `name` even when referenced (used by undo to restore a prior state exactly).
    void forceRemove(std::string_view name);

    /// Marks one more user of `name` (e.g. a toolpath). Unknown names are ignored.
    void addReference(std::string_view name);
    /// Releases one reference; never goes below zero.
    void removeReference(std::string_view name);
    std::size_t referenceCount(std::string_view name) const;

    std::size_t size() const { return values_.size(); }
    bool empty() const { return values_.empty(); }
    /// Names in sorted order.
    std::vector<std::string> names() const;
    const std::map<std::string, double, std::less<>>& all() const { return values_; }

    /// Serialises as one `Name = value` line per variable, sorted by name, round-trip exact.
    std::string toText() const;
    /// Parses `Name = value` lines (blank lines and `#` comments skipped). Existing same-named
    /// variables are overwritten.
    ImportResult importText(std::string_view text);

    bool operator==(const DocumentVariables& o) const { return values_ == o.values_; }

   private:
    std::map<std::string, double, std::less<>> values_;
    std::map<std::string, std::size_t, std::less<>> references_;
};

/// Values for the letter variables of a calculation edit box (mm). W/X = width, H/Y = height,
/// T/Z = thickness; P = pi, I = mm->inch and M = inch->mm are constants.
struct ExpressionContext {
    double width{0.0};
    double height{0.0};
    double thickness{0.0};
    /// Length of one foot (`'`) in the units of the box: 304.8 for millimetres, 12 for inches.
    double feet{304.8};
};

/// Result of `evaluateExpression`: either a finite value or an error message.
struct ExpressionResult {
    std::optional<double> value;
    std::string error;
    /// Document variable names the expression uses (sorted, unique), even when evaluation failed
    /// later; lets a caller register references so used variables cannot be deleted.
    std::vector<std::string> usedVariables;
    bool ok() const { return value.has_value(); }
};

/// Hard bounds so hostile input costs bounded time and stack.
inline constexpr std::size_t kMaxExpressionLength = 1024;
inline constexpr int kMaxExpressionDepth = 64;

/// Evaluates a calculation-edit-box expression: numbers, `+ - * / ^` (`^` right-associative,
/// binds tighter than unary minus), parentheses, `{Name}` document variables, letter variables
/// (case-insensitive) and a postfix `'` meaning feet (x `ctx.feet`, 304.8 mm by default). A leading `=` is ignored.
/// Division by zero, unknown names, non-finite results, over-long or too-deeply-nested input
/// are errors; never throws.
ExpressionResult evaluateExpression(std::string_view text, const DocumentVariables& vars,
                                    const ExpressionContext& ctx = {});

}  // namespace carvetto::document
