#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"

// Qt-free plate production / data merge (T-0347, ADR-0122). The selected objects on the active
// sheet are the TEMPLATE; its bounding box is the plate. For every data row of a CSV text one copy
// ("plate") is made: text objects whose text holds `!!NAME!!` placeholders get that text replaced
// by the row's value (or by a serial-number counter) and are laid out again with their embedded
// font; everything else is copied unchanged. Plates are laid out in a grid inside the sheet's
// margins, overflowing onto new sheets (Grid) or one per sheet (OnePerSheet), so every plate lies
// fully inside the material and is ready for toolpaths. The whole merge is ONE undo step and
// all-or-nothing: a malformed row, a text that cannot be laid out, a plate that does not fit or a
// cap that is exceeded refuses the merge with line-numbered diagnostics and changes nothing (unless
// `skipBadRows` is set, which drops the rows with problems and merges the rest). Selection is not
// part of undo. Placeholders are only recognised in text objects; a placeholder without a
// column assignment or counter is an error. Values are inserted verbatim.
//
// Caps: `kMaxMergePlates` plates; plates x (objects + spans of the template) <= `kMaxCopyWork`;
// the CSV caps of `carvetto/io/delimited.hpp`.

namespace carvetto::viewmodel {

inline constexpr std::size_t kMaxMergePlates = 1000;
inline constexpr int kMaxCounterWidth = 18;
inline constexpr double kMaxMergeLengthMm = 1.0e6;

enum class MergeLayout {
    Grid,        ///< Plates fill each sheet row by row from the top left; overflow goes to new sheets.
    OnePerSheet  ///< Every plate on its own sheet, at the top left inside the margins.
};

/// A placeholder name bound to a CSV column (0-based, in the file's column order).
struct VariableAssignment {
    std::string name;
    int column{0};
    bool operator==(const VariableAssignment&) const = default;
};

/// A serial-number placeholder: plate k (0-based) gets `prefix + zero-padded(start + k * step) + suffix`.
struct CounterSpec {
    std::string name;
    std::int64_t start{1};
    std::int64_t step{1};
    int width{1};  ///< Minimum digits, zero padded, 1 .. `kMaxCounterWidth`.
    std::string prefix;
    std::string suffix;
    bool operator==(const CounterSpec&) const = default;
};

struct DataMergeOptions {
    char separator{','};
    bool firstRowNames{true};
    std::vector<VariableAssignment> assignments;
    std::vector<CounterSpec> counters;
    MergeLayout layout{MergeLayout::Grid};
    double marginLeftMm{5.0};
    double marginRightMm{5.0};
    double marginTopMm{5.0};
    double marginBottomMm{5.0};
    double spacingMm{5.0};
    bool newLayer{true};         ///< Put the plates on a new layer "Plates".
    bool removeTemplate{false};  ///< Delete the template objects when the merge is done.
    bool skipBadRows{false};     ///< Drop rows with problems instead of refusing the whole merge.
};

struct MergeDiagnostic {
    std::size_t line{0};  ///< 1-based CSV line, 0 for a problem that is not about a row.
    std::string message;
    bool operator==(const MergeDiagnostic&) const = default;
};

struct DataMergeResult {
    bool ok{false};
    /// Empty on success; otherwise the reason, for a message box.
    std::string message;
    /// Problems that refused (or, with `skipBadRows`, dropped) rows, in line order.
    std::vector<MergeDiagnostic> diagnostics;
    /// Non-fatal notes (text wider than its box, missing glyphs), in line order.
    std::vector<MergeDiagnostic> warnings;
    std::size_t plates{0};
    std::size_t sheetsUsed{0};      ///< Sheets that hold plates (including the active one).
    std::size_t platesPerSheet{0};  ///< Grid capacity of one sheet (1 for OnePerSheet).
    std::size_t skippedRows{0};
    std::vector<document::ObjectId> created;  ///< Top-level plate objects, now selected (apply only).
};

/// True when the selection holds an eligible template (visible, unlocked, on the active sheet).
bool canDataMerge(const document::Document& doc);
/// Placeholder names (`!!NAME!!`) used by the template's text objects, sorted and unique.
std::vector<std::string> templatePlaceholders(const document::Document& doc);
/// Column headers of `csvText` (first record), or empty when the text cannot be parsed.
std::vector<std::string> csvHeaders(std::string_view csvText, char separator);
/// Assigns every placeholder that matches a header (case-insensitively) to that column.
std::vector<VariableAssignment> autoAssign(const std::vector<std::string>& placeholders,
                                           const std::vector<std::string>& headers);
/// Formats a counter value per `spec`.
std::string formatCounter(const CounterSpec& spec, std::size_t plateIndex);

/// Empty when `options` are usable, else the reason (names, widths, lengths).
std::string validateDataMergeOptions(const DataMergeOptions& options);

/// Dry run: the same checks and layout as `applyDataMerge`, the document is left untouched.
DataMergeResult previewDataMerge(const document::Document& doc, std::string_view csvText,
                                 const DataMergeOptions& options);
/// Makes the plates as one undo step.
DataMergeResult applyDataMerge(document::Document& doc, std::string_view csvText, const DataMergeOptions& options);

}  // namespace carvetto::viewmodel
