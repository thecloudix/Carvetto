#pragma once

#include <cstddef>
#include <filesystem>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

// Gadget packaging (ADR-0102, ADR-0108): a gadget is a folder holding `gadget.lua` and
// `manifest.json`. Discovery scans a gadgets folder, install copies a checked folder into it.
// No Qt; filesystem access only through std::filesystem and never outside the two folders given.
namespace carvetto::scripting {

/// Fixed names inside a gadget folder.
inline constexpr const char* kGadgetScriptFile = "gadget.lua";
inline constexpr const char* kGadgetManifestFile = "manifest.json";
/// Largest manifest file accepted.
inline constexpr std::size_t kMaxManifestBytes = 64u * 1024u;
/// Most gadget folders discovery looks at (a folder of thousands of entries is not a gadgets folder).
inline constexpr std::size_t kMaxGadgetFolders = 500;

/// The parsed `manifest.json`.
struct GadgetManifest {
    std::string name;         ///< required, 1-100 characters
    std::string version{"1"};
    int apiVersion{1};        ///< `api_version`: the API the gadget was written for
    std::string menu;         ///< path under the Gadgets menu, "Group/Item"; defaults to `name`
    std::string shortcut;     ///< canonical "Ctrl+F5" form, or empty
    std::string description;
    bool operator==(const GadgetManifest&) const = default;
};

struct ManifestParse {
    std::optional<GadgetManifest> manifest;
    std::string error;  ///< set when `manifest` is empty
};

/// Parses and validates manifest JSON text (unknown keys are ignored for forward compatibility).
ManifestParse parseGadgetManifest(std::string_view json);

/// Canonical text of a gadget shortcut ("ctrl+f5" -> "Ctrl+F5"): Ctrl or Ctrl+Shift with F1..F12
/// only; nullopt for anything else.
std::optional<std::string> normaliseShortcut(std::string_view text);

/// The path components of a manifest's menu entry ("Group/Item" -> {"Group", "Item"}), 1-4 of them.
std::vector<std::string> menuPath(const GadgetManifest& manifest);

/// One gadget found in the gadgets folder.
struct GadgetInfo {
    std::filesystem::path folder;
    std::string folderName;
    GadgetManifest manifest;
    /// The shortcut the gadget actually gets: the manifest's, unless another gadget (earlier in
    /// folder-name order) already has it.
    std::string shortcut;
    /// Non-empty when the gadget is listed but cannot run (needs a newer API, no script file, ...).
    std::string problem;
    /// Notes that do not stop it from running (a shortcut conflict).
    std::vector<std::string> notes;
    bool runnable() const { return problem.empty(); }
};

struct GadgetDiscovery {
    /// Valid manifests, in folder-name order.
    std::vector<GadgetInfo> gadgets;
    /// Folders skipped because their manifest or layout is unusable, "folder: reason".
    std::vector<std::string> skipped;
};

/// Shortcuts the user chose (Gadget Shortcuts), by gadget folder name: a canonical shortcut, or ""
/// for "no shortcut". They win over the manifest's.
using ShortcutOverrides = std::map<std::string, std::string>;

/// Scans the immediate subfolders of `root` (hidden folders and symbolic links are skipped; a
/// missing root gives an empty result). Deterministic: sorted by folder name.
GadgetDiscovery discoverGadgets(const std::filesystem::path& root, const ShortcutOverrides& overrides = {});

/// Reads `gadget.lua` of `gadget` (at most `maxBytes`) into `*out`; returns "" on success, else the reason.
std::string readGadgetSource(const GadgetInfo& gadget, std::size_t maxBytes, std::string* out);

struct InstallResult {
    bool ok{false};
    std::string error;
    GadgetInfo gadget;  ///< where it was installed (valid when ok), or what was found (inspect)
};

/// Checks that `folder` is a gadget (manifest valid, script present and within `maxScriptBytes`)
/// without copying anything: what the Install dialog shows before the user confirms.
InstallResult inspectGadgetFolder(const std::filesystem::path& folder, std::size_t maxScriptBytes);

/// Copies the gadget in `source` (its manifest and script only) to a new folder in `root`, named
/// after the gadget. Refuses a gadget that is already installed unless `replace`. The copy is
/// made in a temporary folder and renamed, so a failure leaves nothing half-installed.
InstallResult installGadget(const std::filesystem::path& source, const std::filesystem::path& root,
                            std::size_t maxScriptBytes, bool replace = false);

}  // namespace carvetto::scripting
