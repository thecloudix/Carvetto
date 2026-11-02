#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Interim storage of computed toolpaths inside a .carv (ADR-0019): one archive member
// `toolpaths/NNNN.json` per toolpath, kept by the core loader as an unknown member and written
// back on every save. Replaced once a card implements ADR-0003's job.json toolpath records.

namespace carvetto::cli {

/// `format` value of a stored toolpath member.
inline constexpr std::string_view kToolpathFormat = "carvetto-toolpath";
/// `format_version` written by this build.
inline constexpr int kToolpathFormatVersion = 1;

/// A toolpath as stored in a .carv, with what it was made from.
struct StoredToolpath {
    /// Archive member name, e.g. "toolpaths/0001.json".
    std::string member;
    /// "profile" or "pocket".
    std::string operation;
    /// The params file it came from, as canonical JSON text.
    std::string paramsJson;
    /// Absolute safe Z of the toolpath (mm).
    double safeZ{5.0};
    toolpath::Toolpath toolpath;
};

/// Encodes one toolpath member's content (canonical compact JSON, deterministic).
std::vector<std::uint8_t> encodeToolpath(const StoredToolpath& stored);

/// Decodes one member. Throws `CliError` naming the member on malformed content.
StoredToolpath decodeToolpath(std::string member, const std::vector<std::uint8_t>& bytes);

/// True for names of the form "toolpaths/NNNN.json" (four digits).
bool isToolpathMember(std::string_view name);

/// The stored toolpaths of a loaded file, in member-name (creation) order.
std::vector<StoredToolpath> storedToolpaths(const document::CarvExtras& extras);

/// The name of the next toolpath member: one above the highest existing number ("toolpaths/0001.json"
/// for the first). Throws `CliError` once 9999 is used.
std::string nextToolpathMember(const document::CarvExtras& extras);

}  // namespace carvetto::cli
