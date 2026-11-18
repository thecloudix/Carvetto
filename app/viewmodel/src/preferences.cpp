#include "carvetto/viewmodel/preferences.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <system_error>

namespace carvetto::viewmodel {

namespace {

constexpr int kUndoMin = 10;
constexpr int kUndoMax = 10000;
constexpr int kAutosaveMin = 1;
constexpr int kAutosaveMax = 120;
constexpr double kGridMin = 0.01;
constexpr double kGridMax = 1000.0;
constexpr double kTolMin = 2.0;
constexpr double kTolMax = 50.0;
constexpr std::size_t kPathMax = 4096;

const std::vector<PrefField>& fields() {
    static const std::vector<PrefField> list = {
        {"general/units", PrefCategory::General, "Default units for new jobs"},
        {"general/language", PrefCategory::General, "Language"},
        {"editing/undoDepth", PrefCategory::Editing, "Undo steps kept"},
        {"editing/autosaveEnabled", PrefCategory::Editing, "Autosave"},
        {"editing/autosaveMinutes", PrefCategory::Editing, "Autosave interval (minutes)"},
        {"grid/show", PrefCategory::GridSnap, "Show grid"},
        {"grid/spacingMm", PrefCategory::GridSnap, "Grid spacing (mm)"},
        {"grid/snapEnabled", PrefCategory::GridSnap, "Snap to objects and grid"},
        {"grid/snapTolerancePx", PrefCategory::GridSnap, "Snap distance (pixels)"},
        {"colours/canvasBackground", PrefCategory::Colours, "Canvas background"},
        {"colours/materialFill", PrefCategory::Colours, "Material fill"},
        {"display/antialiasing", PrefCategory::Display, "Smooth (antialiased) drawing"},
        {"display/showRulers", PrefCategory::Display, "Show rulers"},
        {"folders/open", PrefCategory::Folders, "Default open folder"},
        {"folders/save", PrefCategory::Folders, "Default save folder"},
        {"folders/export", PrefCategory::Folders, "Default export folder"},
        {"tools/databasePath", PrefCategory::Tools, "Tool database file"},
        {"simulation/quality", PrefCategory::Simulation, "Simulation quality"},
    };
    return list;
}

std::string boolText(bool v) {
    return v ? "true" : "false";
}

bool parseBool(const std::string& s, bool& out) {
    if (s == "true") {
        out = true;
        return true;
    }
    if (s == "false") {
        out = false;
        return true;
    }
    return false;
}

bool parseInt(const std::string& s, int& out) {
    const char* first = s.data();
    const char* last = first + s.size();
    int v = 0;
    const auto r = std::from_chars(first, last, v);
    if (r.ec != std::errc{} || r.ptr != last) {
        return false;
    }
    out = v;
    return true;
}

bool parseDouble(const std::string& s, double& out) {
    const char* first = s.data();
    const char* last = first + s.size();
    double v = 0.0;
    const auto r = std::from_chars(first, last, v);
    if (r.ec != std::errc{} || r.ptr != last || !std::isfinite(v)) {
        return false;
    }
    out = v;
    return true;
}

std::string doubleText(double v) {
    char buf[64];
    const auto r = std::to_chars(buf, buf + sizeof buf, v);
    return std::string(buf, r.ptr);
}

const char* qualityText(simulate::Quality q) {
    switch (q) {
        case simulate::Quality::Coarse:
            return "coarse";
        case simulate::Quality::Medium:
            return "medium";
        case simulate::Quality::Fine:
            return "fine";
    }
    return "medium";
}

bool parseQuality(const std::string& s, simulate::Quality& q) {
    if (s == "coarse") {
        q = simulate::Quality::Coarse;
    } else if (s == "medium") {
        q = simulate::Quality::Medium;
    } else if (s == "fine") {
        q = simulate::Quality::Fine;
    } else {
        return false;
    }
    return true;
}

bool pathSyntaxOk(const std::string& s) {
    if (s.size() > kPathMax) {
        return false;
    }
    return std::none_of(s.begin(), s.end(), [](char c) { return c == '\0' || c == '\n' || c == '\r'; });
}

std::filesystem::path pathOf(const std::string& utf8) {
    const std::u8string u8(utf8.begin(), utf8.end());
    return std::filesystem::path(u8);
}

bool dirExists(const std::string& utf8) {
    std::error_code ec;
    return std::filesystem::is_directory(pathOf(utf8), ec);
}

void checkFolder(std::vector<PrefIssue>& out, const char* key, const std::string& value, bool checkPaths,
                 const char* what) {
    if (value.empty()) {
        return;
    }
    if (!pathSyntaxOk(value)) {
        out.push_back({key, std::string(what) + " is not a valid path."});
    } else if (checkPaths && !dirExists(value)) {
        out.push_back({key, std::string(what) + " does not exist."});
    }
}

}  // namespace

const std::vector<PrefField>& preferenceFields() {
    return fields();
}

const char* categoryTitle(PrefCategory category) {
    switch (category) {
        case PrefCategory::General:
            return "General";
        case PrefCategory::Editing:
            return "Editing";
        case PrefCategory::GridSnap:
            return "Grid and snap";
        case PrefCategory::Colours:
            return "Colours";
        case PrefCategory::Display:
            return "Display";
        case PrefCategory::Folders:
            return "Folders";
        case PrefCategory::Tools:
            return "Tools";
        case PrefCategory::Simulation:
            return "Simulation";
    }
    return "";
}

std::string colourToString(std::uint32_t rgb) {
    static const char* hex = "0123456789abcdef";
    std::string s = "#";
    for (int shift = 20; shift >= 0; shift -= 4) {
        s += hex[(rgb >> static_cast<unsigned>(shift)) & 0xFu];
    }
    return s;
}

bool colourFromString(const std::string& text, std::uint32_t& rgb) {
    if (text.size() != 7 || text[0] != '#') {
        return false;
    }
    std::uint32_t v = 0;
    for (std::size_t i = 1; i < text.size(); ++i) {
        const char c = text[i];
        std::uint32_t d = 0;
        if (c >= '0' && c <= '9') {
            d = static_cast<std::uint32_t>(c - '0');
        } else if (c >= 'a' && c <= 'f') {
            d = static_cast<std::uint32_t>(c - 'a' + 10);
        } else if (c >= 'A' && c <= 'F') {
            d = static_cast<std::uint32_t>(c - 'A' + 10);
        } else {
            return false;
        }
        v = (v << 4) | d;
    }
    rgb = v;
    return true;
}

std::vector<PrefIssue> validate(const Preferences& p, const PrefValidation& opts) {
    std::vector<PrefIssue> out;
    if (p.language != "system" && p.language != "en") {
        out.push_back({"general/language", "Choose System or English."});
    }
    if (p.undoDepth < kUndoMin || p.undoDepth > kUndoMax) {
        out.push_back({"editing/undoDepth", "Undo steps must be between 10 and 10000."});
    }
    if (p.autosaveMinutes < kAutosaveMin || p.autosaveMinutes > kAutosaveMax) {
        out.push_back({"editing/autosaveMinutes", "The autosave interval must be between 1 and 120 minutes."});
    }
    if (!std::isfinite(p.gridSpacingMm) || p.gridSpacingMm < kGridMin || p.gridSpacingMm > kGridMax) {
        out.push_back({"grid/spacingMm", "Grid spacing must be between 0.01 and 1000 mm."});
    }
    if (!std::isfinite(p.snapTolerancePx) || p.snapTolerancePx < kTolMin || p.snapTolerancePx > kTolMax) {
        out.push_back({"grid/snapTolerancePx", "Snap distance must be between 2 and 50 pixels."});
    }
    if (p.canvasBackground > 0xFFFFFFu) {
        out.push_back({"colours/canvasBackground", "The colour must be 0xRRGGBB."});
    }
    if (p.materialFill > 0xFFFFFFu) {
        out.push_back({"colours/materialFill", "The colour must be 0xRRGGBB."});
    }
    if (p.canvasBackground == p.materialFill) {
        out.push_back({"colours/materialFill", "The material fill must differ from the canvas background."});
    }
    checkFolder(out, "folders/open", p.openFolder, opts.checkPaths, "The open folder");
    checkFolder(out, "folders/save", p.saveFolder, opts.checkPaths, "The save folder");
    checkFolder(out, "folders/export", p.exportFolder, opts.checkPaths, "The export folder");
    if (!p.toolDatabasePath.empty()) {
        if (!pathSyntaxOk(p.toolDatabasePath)) {
            out.push_back({"tools/databasePath", "The tool database path is not valid."});
        } else if (opts.checkPaths) {
            const std::filesystem::path parent = pathOf(p.toolDatabasePath).parent_path();
            std::error_code ec;
            if (!parent.empty() && !std::filesystem::is_directory(parent, ec)) {
                out.push_back({"tools/databasePath", "The tool database's folder does not exist."});
            }
        }
    }
    return out;
}

PrefKeyValues serialize(const Preferences& p) {
    PrefKeyValues kv;
    kv.emplace_back("version", std::to_string(kPreferencesVersion));
    kv.emplace_back("general/units", p.defaultUnits == document::Units::Inches ? "in" : "mm");
    kv.emplace_back("general/language", p.language);
    kv.emplace_back("editing/undoDepth", std::to_string(p.undoDepth));
    kv.emplace_back("editing/autosaveEnabled", boolText(p.autosaveEnabled));
    kv.emplace_back("editing/autosaveMinutes", std::to_string(p.autosaveMinutes));
    kv.emplace_back("grid/show", boolText(p.showGrid));
    kv.emplace_back("grid/spacingMm", doubleText(p.gridSpacingMm));
    kv.emplace_back("grid/snapEnabled", boolText(p.snapEnabled));
    kv.emplace_back("grid/snapTolerancePx", doubleText(p.snapTolerancePx));
    kv.emplace_back("colours/canvasBackground", colourToString(p.canvasBackground));
    kv.emplace_back("colours/materialFill", colourToString(p.materialFill));
    kv.emplace_back("display/antialiasing", boolText(p.antialiasing));
    kv.emplace_back("display/showRulers", boolText(p.showRulers));
    kv.emplace_back("folders/open", p.openFolder);
    kv.emplace_back("folders/save", p.saveFolder);
    kv.emplace_back("folders/export", p.exportFolder);
    kv.emplace_back("tools/databasePath", p.toolDatabasePath);
    kv.emplace_back("simulation/quality", qualityText(p.simulationQuality));
    return kv;
}

void migrate(PrefKeyValues& stored, int fromVersion) {
    (void)stored;
    (void)fromVersion;  // no step yet: versions 0 and 1 share their keys
}

Preferences deserialize(const PrefKeyValues& storedIn, std::vector<PrefIssue>* issues) {
    PrefKeyValues stored = storedIn;
    std::vector<PrefIssue> local;
    std::vector<PrefIssue>& out = issues != nullptr ? *issues : local;

    int version = 0;
    for (const auto& kv : stored) {
        if (kv.first == "version") {
            if (!parseInt(kv.second, version) || version < 0) {
                out.push_back({"version", "The stored settings version is unreadable; reading as version 0."});
                version = 0;
            }
        }
    }
    if (version > kPreferencesVersion) {
        out.push_back({"version", "The settings were written by a newer version; unknown values are ignored."});
    } else if (version < kPreferencesVersion) {
        migrate(stored, version);
    }

    auto find = [&](const char* key) -> const std::string* {
        const std::string* found = nullptr;
        for (const auto& kv : stored) {
            if (kv.first == key) {
                found = &kv.second;  // the last occurrence wins
            }
        }
        return found;
    };

    Preferences p;
    auto bad = [&](const char* key) { out.push_back({key, "Stored value is invalid; the default is used."}); };
    auto readBool = [&](const char* key, bool& field) {
        if (const std::string* s = find(key)) {
            bool v = false;
            if (parseBool(*s, v)) {
                field = v;
            } else {
                bad(key);
            }
        }
    };
    auto readInt = [&](const char* key, int& field) {
        if (const std::string* s = find(key)) {
            int v = 0;
            if (parseInt(*s, v)) {
                field = v;
            } else {
                bad(key);
            }
        }
    };
    auto readDouble = [&](const char* key, double& field) {
        if (const std::string* s = find(key)) {
            double v = 0.0;
            if (parseDouble(*s, v)) {
                field = v;
            } else {
                bad(key);
            }
        }
    };
    auto readColour = [&](const char* key, std::uint32_t& field) {
        if (const std::string* s = find(key)) {
            std::uint32_t v = 0;
            if (colourFromString(*s, v)) {
                field = v;
            } else {
                bad(key);
            }
        }
    };
    auto readText = [&](const char* key, std::string& field) {
        if (const std::string* s = find(key)) {
            field = *s;
        }
    };

    if (const std::string* s = find("general/units")) {
        if (*s == "mm") {
            p.defaultUnits = document::Units::Millimetres;
        } else if (*s == "in") {
            p.defaultUnits = document::Units::Inches;
        } else {
            bad("general/units");
        }
    }
    readText("general/language", p.language);
    readInt("editing/undoDepth", p.undoDepth);
    readBool("editing/autosaveEnabled", p.autosaveEnabled);
    readInt("editing/autosaveMinutes", p.autosaveMinutes);
    readBool("grid/show", p.showGrid);
    readDouble("grid/spacingMm", p.gridSpacingMm);
    readBool("grid/snapEnabled", p.snapEnabled);
    readDouble("grid/snapTolerancePx", p.snapTolerancePx);
    readColour("colours/canvasBackground", p.canvasBackground);
    readColour("colours/materialFill", p.materialFill);
    readBool("display/antialiasing", p.antialiasing);
    readBool("display/showRulers", p.showRulers);
    readText("folders/open", p.openFolder);
    readText("folders/save", p.saveFolder);
    readText("folders/export", p.exportFolder);
    readText("tools/databasePath", p.toolDatabasePath);
    if (const std::string* s = find("simulation/quality")) {
        if (!parseQuality(*s, p.simulationQuality)) {
            bad("simulation/quality");
        }
    }

    // Field-wise range check: an out-of-range value falls back to its own default only.
    const Preferences d;
    for (const PrefIssue& issue : validate(p)) {
        out.push_back(issue);
        const std::string& k = issue.key;
        if (k == "general/language") {
            p.language = d.language;
        } else if (k == "editing/undoDepth") {
            p.undoDepth = d.undoDepth;
        } else if (k == "editing/autosaveMinutes") {
            p.autosaveMinutes = d.autosaveMinutes;
        } else if (k == "grid/spacingMm") {
            p.gridSpacingMm = d.gridSpacingMm;
        } else if (k == "grid/snapTolerancePx") {
            p.snapTolerancePx = d.snapTolerancePx;
        } else if (k == "colours/canvasBackground" || k == "colours/materialFill") {
            p.canvasBackground = d.canvasBackground;
            p.materialFill = d.materialFill;
        } else if (k == "folders/open") {
            p.openFolder = d.openFolder;
        } else if (k == "folders/save") {
            p.saveFolder = d.saveFolder;
        } else if (k == "folders/export") {
            p.exportFolder = d.exportFolder;
        } else if (k == "tools/databasePath") {
            p.toolDatabasePath = d.toolDatabasePath;
        }
    }
    return p;
}

std::vector<PrefIssue> PreferencesModel::apply(const Preferences& candidate, bool checkPaths) {
    PrefValidation opts;
    opts.checkPaths = checkPaths;
    std::vector<PrefIssue> issues = validate(candidate, opts);
    if (!issues.empty()) {
        return issues;
    }
    if (!(candidate == current_)) {
        current_ = candidate;
        if (onChanged) {
            onChanged(current_);
        }
    }
    return issues;
}

void PreferencesModel::restoreDefaults() {
    apply(defaults());
}

std::vector<PrefIssue> PreferencesModel::load(PreferencesStore& store) {
    std::vector<PrefIssue> issues;
    const Preferences loaded = deserialize(store.load(), &issues);
    if (!(loaded == current_)) {
        current_ = loaded;
        if (onChanged) {
            onChanged(current_);
        }
    }
    return issues;
}

}  // namespace carvetto::viewmodel
