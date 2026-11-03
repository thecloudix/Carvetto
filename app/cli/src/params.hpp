#pragma once

#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/vcarve.hpp"

// params.json for `carvetto-cli toolpath profile|pocket|drill|engrave|vcarve` (T-0122, ADR-0019; owner
// decision 2026-11-01; drill T-0150; engrave T-0151; vcarve T-0153): snake_case keys mirroring
// ProfileParams / PocketParams / DrillParams / EngraveParams / VCarveParams,
// lengths in mm, times in seconds, angles in
// degrees (converted to radians here, at the CLI edge), the tool as
// {"tool": {"database": <path>, "id": <tool id>, "machine": <id>, "material": <id>}} and an
// optional "layers": [names]. Unknown keys are errors naming the key.

namespace carvetto::cli {

/// A user-facing failure of a CLI command (bad input, unreadable file, nothing produced). The
/// message is complete and printed as is.
class CliError : public std::runtime_error {
   public:
    explicit CliError(const std::string& message) : std::runtime_error(message) {}
};

/// Which tool a params file asks for.
struct ToolChoice {
    /// Tool database file; relative paths are relative to the params file. Empty = the starter
    /// database compiled into the CLI.
    std::string database;
    /// Tool id in the database.
    std::string id;
    /// Machine and material ids selecting the cutting data; both or neither.
    std::string machine;
    std::string material;
};

/// Settings every toolpath params file has besides the operation's own parameters.
struct CommonParams {
    ToolChoice tool;
    /// Names of the layers whose vectors are machined; empty = every vector.
    std::vector<std::string> layers;
};

/// A parsed profile params file.
struct ProfileJob {
    toolpath::ProfileParams params;
    CommonParams common;
};

/// A parsed pocket params file.
struct PocketJob {
    toolpath::PocketParams params;
    CommonParams common;
};

/// A parsed drill params file.
struct DrillJob {
    toolpath::DrillParams params;
    CommonParams common;
};

/// A parsed quick engrave params file.
struct EngraveJob {
    toolpath::EngraveParams params;
    CommonParams common;
};

/// A parsed V-carve params file: the carving tool is `common.tool`; the clearance tools (end
/// mills, in order) come from "clearance_tools".
struct VCarveJob {
    toolpath::VCarveParams params;
    CommonParams common;
    std::vector<ToolChoice> clearanceTools;
};

/// Parses a profile params file's text. Throws `CliError` naming the offending key.
ProfileJob parseProfileParams(std::string_view text);

/// Parses a pocket params file's text. Throws `CliError` naming the offending key.
PocketJob parsePocketParams(std::string_view text);

/// Parses a drill params file's text. Throws `CliError` naming the offending key.
DrillJob parseDrillParams(std::string_view text);

/// Parses a quick engrave params file's text. Throws `CliError` naming the offending key.
EngraveJob parseEngraveParams(std::string_view text);

/// Parses a V-carve params file's text. Throws `CliError` naming the offending key.
VCarveJob parseVCarveParams(std::string_view text);

/// The tool and the cutting data to use with it.
struct ResolvedTool {
    tooling::Tool tool;
    /// Empty = the tool's default cutting data.
    std::optional<tooling::CuttingData> cuttingData;
    /// Non-fatal notes (e.g. no cutting data for the machine/material: default used).
    std::vector<std::string> warnings;
};

/// Loads the tool database named by `choice` (relative to `baseDir`, or the built-in starter
/// database) and finds the tool and its cutting data. Throws `CliError`.
ResolvedTool resolveTool(const ToolChoice& choice, const std::filesystem::path& baseDir);

/// Help text describing every params.json key of every operation.
std::string paramsHelp();

}  // namespace carvetto::cli
