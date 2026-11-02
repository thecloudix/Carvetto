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

/// Current tool-database file format version written by `exportToolDatabaseJson`.
inline constexpr int kToolDatabaseFormatVersion = 1;

/// Serialises the database to JSON text. Deterministic: object keys are sorted, arrays are in id
/// order, lengths are rounded to 1e-6 mm, angles are written in degrees rounded to 1e-6.
/// Export → import → export reproduces the text byte for byte.
std::string exportToolDatabaseJson(const ToolDatabase& db);

/// Parses text produced by `exportToolDatabaseJson` (or hand-written to the same schema).
/// Unknown keys are ignored; missing optional keys take their defaults; a wrong type, unknown
/// tool type, duplicate id or invalid tool geometry is an error naming the item.
ImportResult importToolDatabaseJson(std::string_view text);

}  // namespace carvetto::tooling
