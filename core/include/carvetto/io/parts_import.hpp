#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Parts-list import (T-0329, ADR-0110): a generic delimited-text (CSV) parts list describing
// rectangular panels with holes and dados, read through a user-editable MAPPING FILE that says
// which column holds what. Pure text in, plain structs out: no document, no files, no Qt.
// Everything is bounded and every problem is a diagnostic with its source line (never a crash or
// an unbounded allocation); cell contents are inert data and are never evaluated.
//
// Units: lengths in the parts list are in the mapping's units (millimetres or inches, set by the
// `units` key and/or a per-row units column); every number in the structs below is millimetres.

namespace carvetto::io {

// --- Limits (ADR-0110 section 5) -----------------------------------------------------------------
inline constexpr std::size_t kMaxPartsFileBytes = 8u * 1024u * 1024u;
inline constexpr std::size_t kMaxMappingBytes = 64u * 1024u;
inline constexpr std::size_t kMaxMappingLines = 512;
inline constexpr std::size_t kMaxMappingLineBytes = 1024;
inline constexpr std::size_t kMaxCsvRecords = 20000;
inline constexpr std::size_t kMaxCsvColumns = 64;
inline constexpr std::size_t kMaxCsvFieldBytes = 4096;
inline constexpr std::size_t kMaxPartRows = 10000;       ///< Data rows that become parts (after the header).
inline constexpr std::size_t kMaxPartInstances = 5000;   ///< Sum of the quantities.
inline constexpr std::size_t kMaxQuantityPerRow = 1000;
inline constexpr std::size_t kMaxHolesPerPart = 200;
inline constexpr std::size_t kMaxDadosPerPart = 32;
inline constexpr std::size_t kMaxLabelBytes = 128;
inline constexpr std::size_t kMaxDiagnostics = 200;      ///< More are counted, not stored.
inline constexpr double kMaxPartDimensionMm = 10000.0;
inline constexpr double kMinPartDimensionMm = 1.0;       ///< Panel length, width and thickness.
inline constexpr double kMaxSheetDimensionMm = 100000.0;
inline constexpr double kInchMm = 25.4;

struct PartsDiagnostic {
    enum class Severity { Warning, Error };
    Severity severity{Severity::Error};
    /// "mapping" or "parts".
    std::string source;
    /// 1-based mapping line or parts-list record number; 0 when it concerns the file as a whole.
    std::size_t line{0};
    std::string text;
    bool operator==(const PartsDiagnostic&) const = default;
};

/// Collects diagnostics up to `kMaxDiagnostics`; the rest are only counted.
struct DiagnosticSink {
    std::vector<PartsDiagnostic> items;
    std::size_t dropped{0};
    std::size_t errors{0};
    void add(PartsDiagnostic::Severity severity, const char* source, std::size_t line, std::string text);
    void error(const char* source, std::size_t line, std::string text) {
        add(PartsDiagnostic::Severity::Error, source, line, std::move(text));
    }
    void warning(const char* source, std::size_t line, std::string text) {
        add(PartsDiagnostic::Severity::Warning, source, line, std::move(text));
    }
};

// --- CSV --------------------------------------------------------------------------------------------

struct CsvTable {
    /// Non-blank records, each a list of cells. `lines[i]` is the 1-based record number of `rows[i]`.
    std::vector<std::vector<std::string>> rows;
    std::vector<std::size_t> lines;
    char delimiter{','};
};

struct CsvResult {
    bool ok{false};
    CsvTable table;
    std::vector<PartsDiagnostic> diagnostics;
};

/// Parses RFC 4180-style text: quoted cells may hold delimiters, quotes (doubled) and line breaks;
/// a UTF-8 byte-order mark is dropped; records end at LF, CRLF or CR; blank records are skipped.
/// `delimiter` 0 = detect from the first record (comma, semicolon or tab outside quotes; most
/// frequent wins, ties prefer comma, then semicolon; a first record using several kinds is a
/// warning). Fails (ok false) on NUL bytes, an unterminated quote, and any limit above.
CsvResult parseCsv(std::string_view text, char delimiter = 0);

// --- Mapping ----------------------------------------------------------------------------------------

/// A column named in the mapping: by header text (case-insensitive) or by 1-based index (`#3`).
struct ColumnRef {
    enum class Kind { None, Name, Index };
    Kind kind{Kind::None};
    std::string name;
    std::size_t index{0};  ///< 1-based, for Kind::Index.
    bool isSet() const { return kind != Kind::None; }
    bool operator==(const ColumnRef&) const = default;
};

enum class PartsUnits { Unset, Millimetres, Inches };

struct PartsMapping {
    PartsUnits units{PartsUnits::Unset};
    /// 0 = detect.
    char delimiter{0};
    bool header{true};
    ColumnRef length, width, thickness, material, label, quantity, unitsColumn, holes, dados;
    std::optional<double> defaultThicknessMm;
    std::string defaultMaterial;
    std::string panelLayer{"Panels"};
    std::string holeLayer{"Holes"};
    std::string dadoLayer{"Dados"};
    double sheetLengthMm{2440.0};
    double sheetWidthMm{1220.0};
    double gapMm{6.0};
    double marginMm{10.0};
    bool rotate{false};
    bool operator==(const PartsMapping&) const = default;
};

struct MappingResult {
    bool ok{false};
    PartsMapping mapping;
    std::vector<PartsDiagnostic> diagnostics;
};

/// Parses the mapping file: `key = value` lines, `#` comment lines, blank lines. Unknown or repeated
/// keys, bad values and missing required keys are errors naming the line (or the key for a missing
/// one). See ADR-0110 for the keys.
MappingResult parsePartsMapping(std::string_view text);

// --- Parts ------------------------------------------------------------------------------------------

struct PartHole {
    double xMm{0.0};  ///< Centre, from the part's origin corner along its length.
    double yMm{0.0};  ///< Centre, along its width.
    double diameterMm{0.0};
    double depthMm{0.0};  ///< 0 = through.
    bool operator==(const PartHole&) const = default;
};

struct PartDado {
    bool alongLength{true};  ///< The groove runs along the length (L) or along the width (W).
    double posMm{0.0};       ///< Near edge of the groove, from the part's origin edge.
    double widthMm{0.0};
    double depthMm{0.0};
    bool operator==(const PartDado&) const = default;
};

struct Part {
    std::string label;
    std::string material;
    double lengthMm{0.0};
    double widthMm{0.0};
    double thicknessMm{0.0};
    std::size_t quantity{1};
    std::size_t line{0};  ///< Record number in the parts list.
    std::vector<PartHole> holes;
    std::vector<PartDado> dados;
    bool operator==(const Part&) const = default;
};

struct PartsList {
    /// Rows that passed; rows with errors are left out (and reported).
    std::vector<Part> parts;
    std::vector<PartsDiagnostic> diagnostics;
    std::size_t errorCount{0};
    std::size_t rowsSkipped{0};
    /// Total of the quantities of `parts`.
    std::size_t instances{0};
    bool hasErrors() const { return errorCount > 0; }
};

/// Applies `mapping` to a parsed table. Every row is judged on its own; bad rows are skipped and
/// reported. Mapping/header problems (an unresolvable column, an ambiguous header) are errors that
/// leave `parts` empty.
PartsList readParts(const CsvTable& table, const PartsMapping& mapping);

/// parseCsv (with the mapping's delimiter) + readParts; CSV failures are in `diagnostics`.
PartsList readPartsText(std::string_view csv, const PartsMapping& mapping);

/// Reads at most `maxBytes` of a file. Empty optional with `error` set when it cannot be read or is
/// larger (the size is checked before reading).
std::optional<std::string> readBoundedFile(const std::string& path, std::size_t maxBytes, std::string* error);

// --- Nesting ----------------------------------------------------------------------------------------

struct NestOptions {
    double sheetLengthMm{2440.0};  ///< Along X.
    double sheetWidthMm{1220.0};   ///< Along Y.
    double gapMm{6.0};
    double marginMm{10.0};
    /// Allow a quarter turn when it is the only way an item fits, or makes a shorter shelf.
    bool allowRotate{false};
    std::size_t maxSheets{64};
};

struct NestItem {
    double lengthMm{0.0};  ///< Along X unless rotated.
    double widthMm{0.0};
};

struct NestPlacement {
    std::size_t item{0};
    std::size_t sheet{0};
    double xMm{0.0};  ///< Lower-left corner of the placed (possibly rotated) item on the sheet.
    double yMm{0.0};
    bool rotated{false};
};

struct NestResult {
    std::vector<NestPlacement> placements;  ///< In placement order.
    std::vector<std::size_t> unplaced;      ///< Items too large for an empty sheet, or beyond `maxSheets`.
    std::size_t sheetCount{0};
    /// Shelf probes made: a deterministic work measure for tests.
    std::uint64_t steps{0};
};

/// Shelf packing, first fit decreasing by shelf height: items sorted by their height (descending,
/// ties by width then index), each put on the first shelf (sheets in order) with room, else a new
/// shelf, else a new sheet. Deterministic; every placed item lies inside the margins and no two
/// closer than `gapMm`.
NestResult nestRectangles(const std::vector<NestItem>& items, const NestOptions& options);

}  // namespace carvetto::io
