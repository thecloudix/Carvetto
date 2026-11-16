#include "carvetto/viewmodel/clipart_library.hpp"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <utility>

namespace carvetto::viewmodel {

namespace fs = std::filesystem;

namespace {

fs::path pathOf(const std::string& utf8) {
    return fs::path(std::u8string(reinterpret_cast<const char8_t*>(utf8.data()), utf8.size()));
}

std::string utf8Of(const fs::path& p) {
    const std::u8string u = p.u8string();
    return std::string(reinterpret_cast<const char*>(u.data()), u.size());
}

std::string lowerAscii(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

bool hasControl(const std::string& s) {
    return std::any_of(s.begin(), s.end(), [](char c) { return static_cast<unsigned char>(c) < 0x20 || c == 0x7f; });
}

/// Paths worth keeping from a store: non-empty, no control characters, bounded length.
bool plausiblePath(const std::string& s) { return !s.empty() && s.size() <= 4096 && !hasControl(s); }

struct Pending {
    fs::path dir;
    int depth;
    std::string rel;  ///< Directory relative to the library folder, '/'-separated.
};

}  // namespace

ClipartScan scanClipartFolder(const std::string& root, const ClipartScanLimits& scanLimits, const ClipartLimits& limits) {
    ClipartScan out;
    std::error_code ec;
    const fs::path rootPath = pathOf(root);
    if (!fs::is_directory(rootPath, ec) || ec) {
        out.messages.push_back(root + ": the folder cannot be read");
        return out;
    }
    std::vector<Pending> stack;
    stack.push_back({rootPath, 0, std::string()});
    bool stopMessage = false;
    while (!stack.empty() && !out.truncated) {
        const Pending cur = std::move(stack.back());
        stack.pop_back();
        std::vector<std::pair<fs::path, std::string>> dirs;
        std::vector<ClipartEntry> files;
        fs::directory_iterator it(cur.dir, fs::directory_options::skip_permission_denied, ec);
        if (ec) {
            out.messages.push_back(utf8Of(cur.dir) + ": the folder cannot be read");
            continue;
        }
        for (; it != fs::directory_iterator(); it.increment(ec)) {
            if (ec) {
                out.messages.push_back(utf8Of(cur.dir) + ": the folder listing stopped on an error");
                break;
            }
            if (out.visited >= scanLimits.maxVisited) {
                out.truncated = true;
                stopMessage = true;
                out.messages.push_back("stopped after examining " + std::to_string(out.visited) +
                                       " directory entries; use a more specific folder");
                break;
            }
            ++out.visited;
            const fs::path& p = it->path();
            const std::string fileName = utf8Of(p.filename());
            if (fileName.empty() || fileName[0] == '.') {
                continue;
            }
            std::error_code sec;
            const fs::file_status st = it->symlink_status(sec);
            if (sec) {
                continue;
            }
            if (fs::is_symlink(st)) {
                continue;
            }
            if (fs::is_directory(st)) {
                if (cur.depth < scanLimits.maxDepth) {
                    dirs.emplace_back(p, cur.rel.empty() ? fileName : cur.rel + "/" + fileName);
                }
                continue;
            }
            if (!fs::is_regular_file(st)) {
                continue;
            }
            const auto kind = clipartKindFromExtension(fileName);
            if (!kind) {
                continue;
            }
            ClipartEntry e;
            e.path = utf8Of(p);
            e.name = utf8Of(p.stem());
            e.kind = *kind;
            std::error_code fec;
            const std::uintmax_t size = fs::file_size(p, fec);
            e.sizeBytes = fec ? 0U : static_cast<std::uint64_t>(size);
            const std::size_t cap = *kind == ClipartKind::Png ? limits.maxPngFileBytes : limits.maxVectorFileBytes;
            if (fec) {
                e.refusal = "the file size cannot be read";
            } else if (size == 0) {
                e.refusal = "the file is empty";
            } else if (size > cap) {
                e.refusal = "the file is " + formatClipartBytes(size) + "; the limit for ." + clipartKindName(*kind) +
                            " clipart is " + formatClipartBytes(cap);
            }
            e.folder = cur.rel;
            files.push_back(std::move(e));
        }
        for (ClipartEntry& e : files) {
            if (out.entries.size() >= scanLimits.maxFiles) {
                out.truncated = true;
                if (!stopMessage) {
                    out.messages.push_back("stopped after " + std::to_string(scanLimits.maxFiles) +
                                           " clipart files; use a more specific folder");
                    stopMessage = true;
                }
                break;
            }
            out.entries.push_back(std::move(e));
        }
        std::sort(dirs.begin(), dirs.end(), [](const auto& a, const auto& b) { return a.second < b.second; });
        // Pushed in reverse so the alphabetically first folder is scanned first.
        for (auto d = dirs.rbegin(); d != dirs.rend(); ++d) {
            stack.push_back({d->first, cur.depth + 1, d->second});
        }
    }
    std::sort(out.entries.begin(), out.entries.end(), [](const ClipartEntry& a, const ClipartEntry& b) {
        if (a.folder != b.folder) {
            return lowerAscii(a.folder) < lowerAscii(b.folder) || (lowerAscii(a.folder) == lowerAscii(b.folder) && a.folder < b.folder);
        }
        const std::string la = lowerAscii(a.name);
        const std::string lb = lowerAscii(b.name);
        if (la != lb) {
            return la < lb;
        }
        return a.path < b.path;
    });
    return out;
}

ClipartLibrary::ClipartLibrary(ClipartStore& store, ClipartScanLimits scanLimits, ClipartLimits limits)
    : store_(store), scanLimits_(scanLimits), limits_(limits) {
    loadState();
}

void ClipartLibrary::reload() {
    loadState();
    rescan();
}

void ClipartLibrary::loadState() {
    folders_.clear();
    favourites_.clear();
    const ClipartState s = store_.load();
    for (const std::string& f : s.folders) {
        if (folders_.size() < kMaxClipartFolders && plausiblePath(f) &&
            std::find(folders_.begin(), folders_.end(), f) == folders_.end()) {
            folders_.push_back(f);
        }
    }
    for (const std::string& f : s.favourites) {
        if (favourites_.size() < kMaxClipartFavourites && plausiblePath(f)) {
            favourites_.insert(f);
        }
    }
}

bool ClipartLibrary::save() {
    ClipartState s;
    s.folders = folders_;
    s.favourites.assign(favourites_.begin(), favourites_.end());
    if (store_.save(s)) {
        return true;
    }
    messages_.push_back("the clipart folders and favourites could not be saved");
    return false;
}

ClipartEdit ClipartLibrary::addFolder(const std::string& path) {
    ClipartEdit r;
    if (!plausiblePath(path)) {
        r.error = "that is not a usable folder path";
        return r;
    }
    if (folders_.size() >= kMaxClipartFolders) {
        r.error = "at most " + std::to_string(kMaxClipartFolders) + " folders can be added";
        return r;
    }
    std::error_code ec;
    if (!fs::is_directory(pathOf(path), ec) || ec) {
        r.error = path + ": not a folder that can be read";
        return r;
    }
    if (std::find(folders_.begin(), folders_.end(), path) != folders_.end()) {
        r.error = path + ": already in the library";
        return r;
    }
    folders_.push_back(path);
    save();
    rescan();
    r.ok = true;
    return r;
}

bool ClipartLibrary::removeFolder(const std::string& path) {
    const auto it = std::find(folders_.begin(), folders_.end(), path);
    if (it == folders_.end()) {
        return false;
    }
    folders_.erase(it);
    save();
    rescan();
    return true;
}

void ClipartLibrary::rescan() {
    entries_.clear();
    messages_.clear();
    visited_ = 0;
    std::set<std::string> seen;
    ClipartScanLimits left = scanLimits_;
    for (std::size_t i = 0; i < folders_.size(); ++i) {
        ClipartScan scan = scanClipartFolder(folders_[i], left, limits_);
        for (std::string& m : scan.messages) {
            messages_.push_back(std::move(m));
        }
        visited_ += scan.visited;
        left.maxVisited = left.maxVisited > scan.visited ? left.maxVisited - scan.visited : 0;
        for (ClipartEntry& e : scan.entries) {
            if (seen.insert(e.path).second) {
                e.root = i;
                entries_.push_back(std::move(e));
            }
        }
        left.maxFiles = scanLimits_.maxFiles > entries_.size() ? scanLimits_.maxFiles - entries_.size() : 0;
    }
}

void ClipartLibrary::setQuery(const std::string& query) {
    query_ = query;
    words_.clear();
    std::string cur;
    for (const char c : lowerAscii(query)) {
        if (std::isspace(static_cast<unsigned char>(c)) != 0) {
            if (!cur.empty()) {
                words_.push_back(std::move(cur));
                cur.clear();
            }
        } else {
            cur.push_back(c);
        }
    }
    if (!cur.empty()) {
        words_.push_back(std::move(cur));
    }
}

std::vector<std::size_t> ClipartLibrary::visible() const {
    std::vector<std::size_t> out;
    for (std::size_t i = 0; i < entries_.size(); ++i) {
        const ClipartEntry& e = entries_[i];
        if (favouritesOnly_ && !isFavourite(e.path)) {
            continue;
        }
        const std::string hay = lowerAscii(e.folder.empty() ? e.name : e.folder + "/" + e.name);
        const bool all = std::all_of(words_.begin(), words_.end(),
                                     [&](const std::string& w) { return hay.find(w) != std::string::npos; });
        if (all) {
            out.push_back(i);
        }
    }
    return out;
}

ClipartEdit ClipartLibrary::setFavourite(const std::string& path, bool favourite) {
    ClipartEdit r;
    if (favourite) {
        if (!plausiblePath(path)) {
            r.error = "that is not a usable file path";
            return r;
        }
        if (favourites_.count(path) == 0 && favourites_.size() >= kMaxClipartFavourites) {
            r.error = "at most " + std::to_string(kMaxClipartFavourites) + " favourites can be kept";
            return r;
        }
        favourites_.insert(path);
    } else {
        favourites_.erase(path);
    }
    r.ok = save();
    if (!r.ok) {
        r.error = "the favourites could not be saved";
    }
    return r;
}

}  // namespace carvetto::viewmodel
