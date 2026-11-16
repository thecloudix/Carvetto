#include "carvetto/scripting/gadget_manifest.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <map>
#include <nlohmann/json.hpp>
#include <set>
#include <system_error>

#include "carvetto/scripting/gadget_api.hpp"

namespace carvetto::scripting {

namespace fs = std::filesystem;

namespace {

bool hasControl(const std::string& s) {
    return std::any_of(s.begin(), s.end(), [](char c) { return static_cast<unsigned char>(c) < 0x20; });
}

std::string lower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

std::string trim(const std::string& s) {
    std::size_t a = 0;
    std::size_t b = s.size();
    while (a < b && s[a] == ' ') {
        ++a;
    }
    while (b > a && s[b - 1] == ' ') {
        --b;
    }
    return s.substr(a, b - a);
}

/// Reads a regular file of at most `maxBytes`; empty `error` on success.
std::string readFile(const fs::path& path, std::size_t maxBytes, std::string& out) {
    std::error_code ec;
    const fs::file_status st = fs::symlink_status(path, ec);
    if (ec || !fs::is_regular_file(st)) {
        return "missing or not a regular file";
    }
    const std::uintmax_t size = fs::file_size(path, ec);
    if (ec) {
        return "cannot read the file size";
    }
    if (size > maxBytes) {
        return "file is too large (" + std::to_string(size) + " bytes, limit " + std::to_string(maxBytes) + ")";
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return "cannot open the file";
    }
    out.assign(static_cast<std::size_t>(size), '\0');
    in.read(out.data(), static_cast<std::streamsize>(size));
    if (static_cast<std::uintmax_t>(in.gcount()) != size) {
        return "cannot read the file";
    }
    return {};
}

std::string folderNameFor(const std::string& name) {
    std::string out;
    for (const char ch : name) {
        const unsigned char c = static_cast<unsigned char>(ch);
        if (std::isalnum(c) != 0 && c < 0x80) {
            out += static_cast<char>(std::tolower(c));
        } else if (!out.empty() && out.back() != '-') {
            out += '-';
        }
    }
    while (!out.empty() && out.back() == '-') {
        out.pop_back();
    }
    return out.empty() ? std::string("gadget") : out.substr(0, 60);
}

GadgetInfo infoFor(const fs::path& folder, GadgetManifest manifest) {
    GadgetInfo g;
    g.folder = folder;
    g.folderName = folder.filename().string();
    g.manifest = std::move(manifest);
    g.shortcut = g.manifest.shortcut;
    if (g.manifest.apiVersion > kGadgetApiVersion) {
        g.problem = "needs scripting API version " + std::to_string(g.manifest.apiVersion) +
                    " (this Carvetto has version " + std::to_string(kGadgetApiVersion) + ")";
    }
    return g;
}

}  // namespace

ManifestParse parseGadgetManifest(std::string_view text) {
    ManifestParse r;
    if (text.size() > kMaxManifestBytes) {
        r.error = "the manifest is too large";
        return r;
    }
    nlohmann::json j = nlohmann::json::parse(text.begin(), text.end(), nullptr, false);
    if (j.is_discarded() || !j.is_object()) {
        r.error = "the manifest is not a JSON object";
        return r;
    }
    const auto str = [&](const char* key, std::string& out, bool required, std::size_t maxLen) -> bool {
        const auto it = j.find(key);
        if (it == j.end()) {
            if (required) {
                r.error = std::string("the manifest has no \"") + key + "\"";
                return false;
            }
            return true;
        }
        if (!it->is_string()) {
            r.error = std::string("\"") + key + "\" must be a string";
            return false;
        }
        out = it->get<std::string>();
        if (out.size() > maxLen || hasControl(out)) {
            r.error = std::string("\"") + key + "\" is too long or has control characters";
            return false;
        }
        return true;
    };
    GadgetManifest m;
    if (!str("name", m.name, true, 100)) {
        return r;
    }
    if (trim(m.name).empty()) {
        r.error = "\"name\" is empty";
        return r;
    }
    if (!str("version", m.version, false, 40) || !str("menu", m.menu, false, 300) ||
        !str("description", m.description, false, 2000)) {
        return r;
    }
    const auto api = j.find("api_version");
    if (api == j.end() || !api->is_number_integer() || api->get<long long>() < 1 || api->get<long long>() > 1000000) {
        r.error = "\"api_version\" must be a whole number of at least 1";
        return r;
    }
    m.apiVersion = static_cast<int>(api->get<long long>());
    std::string shortcut;
    if (!str("shortcut", shortcut, false, 40)) {
        return r;
    }
    if (!shortcut.empty()) {
        const auto n = normaliseShortcut(shortcut);
        if (!n) {
            r.error = "\"shortcut\" must be Ctrl+F1 to Ctrl+F12 (optionally with Shift)";
            return r;
        }
        m.shortcut = *n;
    }
    if (m.menu.empty()) {
        m.menu = m.name;
    }
    if (menuPath(m).empty()) {
        r.error = "\"menu\" must be 1 to 4 non-empty parts separated by \"/\"";
        return r;
    }
    r.manifest = std::move(m);
    return r;
}

std::optional<std::string> normaliseShortcut(std::string_view text) {
    std::vector<std::string> parts;
    std::string cur;
    for (const char c : text) {
        if (c == '+') {
            parts.push_back(trim(cur));
            cur.clear();
        } else {
            cur += c;
        }
    }
    parts.push_back(trim(cur));
    if (parts.size() < 2 || parts.size() > 3) {
        return std::nullopt;
    }
    bool ctrl = false;
    bool shift = false;
    for (std::size_t i = 0; i + 1 < parts.size(); ++i) {
        const std::string p = lower(parts[i]);
        if (p == "ctrl" && !ctrl) {
            ctrl = true;
        } else if (p == "shift" && !shift) {
            shift = true;
        } else {
            return std::nullopt;
        }
    }
    const std::string key = lower(parts.back());
    if (!ctrl || key.size() < 2 || key.size() > 3 || key[0] != 'f') {
        return std::nullopt;
    }
    int n = 0;
    for (std::size_t i = 1; i < key.size(); ++i) {
        if (std::isdigit(static_cast<unsigned char>(key[i])) == 0) {
            return std::nullopt;
        }
        n = n * 10 + (key[i] - '0');
    }
    if (n < 1 || n > 12 || (key.size() == 3 && n < 10) || (key.size() == 2 && n > 9)) {
        return std::nullopt;
    }
    return std::string(shift ? "Ctrl+Shift+F" : "Ctrl+F") + std::to_string(n);
}

std::vector<std::string> menuPath(const GadgetManifest& manifest) {
    std::vector<std::string> out;
    std::string cur;
    const auto flush = [&]() -> bool {
        const std::string t = trim(cur);
        cur.clear();
        if (t.empty() || t.size() > 64) {
            return false;
        }
        out.push_back(t);
        return true;
    };
    for (const char c : manifest.menu) {
        if (c == '/') {
            if (!flush()) {
                return {};
            }
        } else {
            cur += c;
        }
    }
    if (!flush() || out.size() > 4) {
        return {};
    }
    return out;
}

GadgetDiscovery discoverGadgets(const fs::path& root, const ShortcutOverrides& overrides) {
    GadgetDiscovery d;
    std::error_code ec;
    if (!fs::is_directory(root, ec)) {
        return d;
    }
    std::vector<fs::path> folders;
    for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
        const std::string name = it->path().filename().string();
        std::error_code e2;
        const fs::file_status st = it->symlink_status(e2);
        if (e2 || !fs::is_directory(st) || name.empty() || name[0] == '.') {
            continue;
        }
        folders.push_back(it->path());
    }
    std::sort(folders.begin(), folders.end(),
              [](const fs::path& a, const fs::path& b) { return a.filename().string() < b.filename().string(); });
    if (folders.size() > kMaxGadgetFolders) {
        d.skipped.push_back("(gadgets folder): more than " + std::to_string(kMaxGadgetFolders) +
                            " folders, the rest are ignored");
        folders.resize(kMaxGadgetFolders);
    }
    std::map<std::string, std::string> shortcutOwner;
    for (const fs::path& folder : folders) {
        const std::string name = folder.filename().string();
        std::string text;
        const std::string readErr = readFile(folder / kGadgetManifestFile, kMaxManifestBytes, text);
        if (!readErr.empty()) {
            d.skipped.push_back(name + ": " + std::string(kGadgetManifestFile) + ": " + readErr);
            continue;
        }
        ManifestParse mp = parseGadgetManifest(text);
        if (!mp.manifest) {
            d.skipped.push_back(name + ": " + mp.error);
            continue;
        }
        GadgetInfo g = infoFor(folder, std::move(*mp.manifest));
        if (const auto ov = overrides.find(name); ov != overrides.end()) {
            g.shortcut = ov->second;
        }
        const fs::file_status st = fs::symlink_status(folder / kGadgetScriptFile, ec);
        if (g.problem.empty() && (ec || !fs::is_regular_file(st))) {
            g.problem = std::string(kGadgetScriptFile) + " is missing";
        }
        if (!g.shortcut.empty()) {
            const auto owner = shortcutOwner.find(g.shortcut);
            if (owner != shortcutOwner.end()) {
                g.notes.push_back("shortcut " + g.shortcut + " is already used by \"" + owner->second + "\"");
                g.shortcut.clear();
            } else if (g.runnable()) {
                shortcutOwner[g.shortcut] = g.manifest.name;
            } else {
                g.shortcut.clear();
            }
        }
        d.gadgets.push_back(std::move(g));
    }
    return d;
}

std::string readGadgetSource(const GadgetInfo& gadget, std::size_t maxBytes, std::string* out) {
    std::string text;
    const std::string err = readFile(gadget.folder / kGadgetScriptFile, maxBytes, text);
    if (err.empty() && out != nullptr) {
        *out = std::move(text);
    }
    return err;
}

InstallResult inspectGadgetFolder(const fs::path& folder, std::size_t maxScriptBytes) {
    InstallResult r;
    std::string text;
    std::string err = readFile(folder / kGadgetManifestFile, kMaxManifestBytes, text);
    if (!err.empty()) {
        r.error = std::string(kGadgetManifestFile) + ": " + err;
        return r;
    }
    ManifestParse mp = parseGadgetManifest(text);
    if (!mp.manifest) {
        r.error = mp.error;
        return r;
    }
    std::string script;
    err = readFile(folder / kGadgetScriptFile, maxScriptBytes, script);
    if (!err.empty()) {
        r.error = std::string(kGadgetScriptFile) + ": " + err;
        return r;
    }
    r.gadget = infoFor(folder, std::move(*mp.manifest));
    r.ok = true;
    return r;
}

InstallResult installGadget(const fs::path& source, const fs::path& root, std::size_t maxScriptBytes, bool replace) {
    InstallResult r = inspectGadgetFolder(source, maxScriptBytes);
    if (!r.ok) {
        return r;
    }
    r.ok = false;
    if (r.gadget.problem.size() > 0) {
        r.error = "this gadget " + r.gadget.problem;
        return r;
    }
    std::error_code ec;
    fs::create_directories(root, ec);
    if (ec) {
        r.error = "cannot create the gadgets folder: " + ec.message();
        return r;
    }
    const std::string base = folderNameFor(r.gadget.manifest.name);
    const fs::path dest = root / base;
    if (fs::exists(fs::symlink_status(dest, ec)) && !replace) {
        r.error = "\"" + r.gadget.manifest.name + "\" is already installed";
        return r;
    }
    const fs::path tmp = root / (".installing-" + base);
    fs::remove_all(tmp, ec);
    fs::create_directories(tmp, ec);
    if (ec) {
        r.error = "cannot create a temporary folder: " + ec.message();
        return r;
    }
    for (const char* file : {kGadgetManifestFile, kGadgetScriptFile}) {
        fs::copy_file(source / file, tmp / file, fs::copy_options::overwrite_existing, ec);
        if (ec) {
            r.error = std::string("cannot copy ") + file + ": " + ec.message();
            fs::remove_all(tmp, ec);
            return r;
        }
    }
    if (replace) {
        fs::remove_all(dest, ec);
    }
    fs::rename(tmp, dest, ec);
    if (ec) {
        r.error = "cannot move the gadget into place: " + ec.message();
        fs::remove_all(tmp, ec);
        return r;
    }
    GadgetInfo g = infoFor(dest, r.gadget.manifest);
    r.gadget = std::move(g);
    r.ok = true;
    return r;
}

}  // namespace carvetto::scripting
