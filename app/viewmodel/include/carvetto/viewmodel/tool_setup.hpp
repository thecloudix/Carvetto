#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/tooling/machine.hpp"
#include "carvetto/tooling/tool_database.hpp"

namespace carvetto::viewmodel {

// Logic behind the tool database, machine and material editors (T-0235): persistence of the user's
// setup in a versioned JSON file, validation of what the editors produce, id generation and the
// "which toolpaths use this tool" query behind the delete warning. Qt-free; the dialogs are glue.
//
// The persisted file is the core tool-database JSON (`tooling::exportToolDatabaseJson`, format tag
// and version included), so Export/Import of the tool database and the user config are the same
// format. Reading is defensive: files over `kMaxSetupFileBytes`, non-regular files, malformed JSON,
// wrong types and absurd numbers are refused with a message and the caller keeps its defaults.

/// Largest setup/tool-database file that is read (a database of hundreds of tools is < 1 MiB).
inline constexpr std::uintmax_t kMaxSetupFileBytes = 16u * 1024u * 1024u;
/// Largest absolute value accepted for any length, rate or speed in a setup file (mm, mm/min, rpm).
inline constexpr double kMaxSetupMagnitude = 1.0e7;
/// File name of the user's setup inside the application config directory.
inline constexpr const char* kSetupFileName = "tools.json";

/// Sanity checks beyond `tooling::validate`: every number finite, |value| <= `kMaxSetupMagnitude`,
/// lengths/rates >= 0, names non-empty, machine default post among its associated posts when set.
/// Returns the first problem naming the item, or nullopt.
std::optional<std::string> validateSetup(const tooling::ToolDatabase& db);

/// One tool as the editor would store it: geometry valid for its type (`tooling::validate`), a
/// non-empty name, cutting data (default and per machine/material) finite and non-negative.
std::optional<std::string> validateToolForEdit(const tooling::Tool& tool);
/// Machine fields finite and non-negative, name non-empty, default post among associated posts.
std::optional<std::string> validateMachine(const tooling::Machine& machine);

enum class SetupLoadStatus {
    /// The file was read and validated; `database` is its content.
    Loaded,
    /// No file (first run): `database` is the defaults, no message.
    NotFound,
    /// The file was refused: `database` is the defaults, `message` says why.
    Refused,
};

struct SetupLoadResult {
    SetupLoadStatus status{SetupLoadStatus::NotFound};
    tooling::ToolDatabase database;
    std::string message;
};

/// Reads and validates a tool-database file (user setup or an import). Failure carries the reason.
tooling::ImportResult readToolDatabaseFile(const std::filesystem::path& path);
/// Loads the user's setup; a missing or refused file yields `defaults`.
SetupLoadResult loadToolSetup(const std::filesystem::path& path, const tooling::ToolDatabase& defaults);
/// Validates then writes `db` atomically (temp file + rename, parent folders created); the previous
/// file survives any failure. Returns false and fills `error` on failure.
bool writeToolDatabaseFile(const std::filesystem::path& path, const tooling::ToolDatabase& db, std::string& error);

/// Smallest unused ids "tool-N", "machine-N", "material-N", "group-N" (N from 1).
std::string newToolId(const tooling::ToolDatabase& db);
std::string newMachineId(const tooling::ToolDatabase& db);
std::string newMaterialId(const tooling::ToolDatabase& db);
std::string newGroupId(const tooling::ToolDatabase& db);
/// "<base> copy", "<base> copy 2", ... not used as a name by any tool of `db`.
std::string copyNameForTool(const tooling::ToolDatabase& db, const std::string& base);

/// A valid new tool of `type` (id and name empty): 6 mm cutter, 60 degree V angle where needed,
/// a two-point profile for form tools, one flute, zeroed cutting data.
tooling::Tool defaultTool(tooling::ToolType type);
/// A new machine named `name` (id left empty), all numbers zero (= not set).
tooling::Machine defaultMachine(const std::string& name);

/// Which geometry fields of a tool type the editor shows.
struct ToolFields {
    bool includedAngle{false};
    bool tipDiameter{false};
    bool cornerRadius{false};
    bool profile{false};
    /// False for form tools (their diameter follows from the profile).
    bool diameter{true};
};
ToolFields toolFields(tooling::ToolType type);
/// Human label of a tool type ("End mill", "Ball nose", ...).
std::string toolTypeLabel(tooling::ToolType type);

/// A toolpath of a document that was made with a tool.
struct ToolpathUse {
    document::ToolpathId id;
    std::string name;
};
/// The document's toolpaths whose tool source is `toolId` (for the "delete tool?" warning).
std::vector<ToolpathUse> toolpathsUsingTool(const document::Document& doc, const std::string& toolId);

/// The machine the app works with: `preferredId` when present in `db`, else the first machine in
/// id order, else nullptr.
const tooling::Machine* activeMachine(const tooling::ToolDatabase& db, const std::string& preferredId);

}  // namespace carvetto::viewmodel
