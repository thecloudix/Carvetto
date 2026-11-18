#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

// Generic delimited-text (CSV) reader for the plate-production data merge (T-0347). RFC 4180
// style: fields separated by one character, optional double-quote quoting (`""` is a quote,
// quoted fields may hold separators and line breaks), CRLF / LF / CR line ends, a leading UTF-8
// byte-order mark is skipped, blank lines are skipped. Qt-free, never throws, bounded.
//
// Malformed input never aborts parsing: the offending record is dropped and reported with its
// 1-based line number (the line the record starts on), so a caller can show all problems at once.

namespace carvetto::io {

/// Most input bytes, records, columns and bytes per field accepted.
inline constexpr std::size_t kDelimitedMaxBytes = 4u * 1024u * 1024u;
inline constexpr std::size_t kDelimitedMaxRecords = 20'000;
inline constexpr std::size_t kDelimitedMaxColumns = 256;
inline constexpr std::size_t kDelimitedMaxFieldBytes = 4096;
/// Most diagnostics kept (further ones are counted in `suppressed`).
inline constexpr std::size_t kDelimitedMaxDiagnostics = 100;

struct DelimitedOptions {
    char separator{','};
    /// Records must have as many fields as the first record; a differing one is malformed.
    bool requireUniformColumns{true};
};

struct DelimitedDiagnostic {
    std::size_t line{0};  ///< 1-based line the record starts on.
    std::string message;
};

struct DelimitedRecord {
    std::size_t line{0};  ///< 1-based line the record starts on.
    std::vector<std::string> fields;
};

struct DelimitedTable {
    /// Well-formed records in file order (the first one is the header when the caller says so).
    std::vector<DelimitedRecord> records;
    std::vector<DelimitedDiagnostic> diagnostics;
    std::size_t suppressed{0};  ///< Diagnostics beyond `kDelimitedMaxDiagnostics`.
    /// Non-empty when parsing stopped early or the input was refused as a whole (size / record cap).
    std::string fatal;
    /// Number of columns of the first record (0 when there is none).
    std::size_t columns{0};
    bool ok() const { return fatal.empty() && diagnostics.empty() && suppressed == 0; }
};

/// Parses `text`. Control characters other than tab, CR and LF (and NUL in particular) make the
/// record malformed. The separator must be a printable ASCII character other than `"`, CR, LF
/// (tab is allowed); otherwise `fatal` is set.
DelimitedTable parseDelimited(std::string_view text, const DelimitedOptions& options = {});

}  // namespace carvetto::io
