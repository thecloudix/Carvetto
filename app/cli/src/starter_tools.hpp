#pragma once

#include <string_view>

namespace carvetto::cli {

/// The text of `data/tools/starter_tools.json` as it was when the CLI was built: the tool
/// database used when a params file names no `tool.database` (ADR-0019).
std::string_view starterToolsJson();

}  // namespace carvetto::cli
