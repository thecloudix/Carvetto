#pragma once

#include <string>
#include <string_view>

#include "carvetto/tooling/tool_database.hpp"

namespace carvetto::tooling {

/// Outcome of `importToolDatabaseJson`. On failure `error` names the problem and `database`
/// is empty.
struct ImportResult {
    bool ok{false};
    std::string error;
    ToolDatabase database;

    explicit operator bool() const { return ok; }
};

/// Current tool-database file format version written by `exportToolDatabaseJson`. Version 2
/// (T-0262, ADR-0056) adds the thread-mill tool type and its fields (`shank_diameter_mm`,
/// `tooth_height_mm`, `tooth_offset_mm`, `threaded_length_mm`). Version 1 files still import
/// (the new keys are not read from them, and a thread mill in one is refused); a newer build's
/// files are refused by an older one ("unsupported version").
inline constexpr int kToolDatabaseFormatVersion = 2;

/// Serialises the database to JSON text. Deterministic: object keys are sorted, arrays are in id
/// order, lengths are rounded to 1e-6 mm, angles are written in degrees rounded to 1e-6.
/// Export → import → export reproduces the text byte for byte.
std::string exportToolDatabaseJson(const ToolDatabase& db);

/// Parses text produced by `exportToolDatabaseJson` (or hand-written to the same schema).
/// Unknown keys are ignored; missing optional keys take their defaults; a wrong type, unknown
/// tool type, duplicate id or invalid tool geometry is an error naming the item.
ImportResult importToolDatabaseJson(std::string_view text);

}  // namespace carvetto::tooling
