#pragma once

#include <cstddef>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

#include "carvetto/viewmodel/clipart_files.hpp"

// Clipart library, part 3 (T-0333, ADR-0109): folders, scanning, search and favourites. Qt-free.
// The library never fetches anything: it lists SVG / DXF / PNG files below folders the user added.

namespace carvetto::viewmodel {

/// Budgets of one folder scan.
struct ClipartScanLimits {
    /// Sub-folder levels below the folder itself (0 = only its own files).
    int maxDepth{4};
    /// Clipart files listed per scan (all folders together in `ClipartLibrary`).
    std::size_t maxFiles{5'000};
    /// Directory entries examined per scan, of any type (bounds the cost of a huge folder).
    std::size_t maxVisited{60'000};
};

/// One clipart file found by a scan.
struct ClipartEntry {
    std::string path;    ///< Absolute-as-given path, UTF-8.
    std::string name;    ///< File name without extension.
    std::string folder;  ///< Directory relative to the library folder ("" = the folder itself).
    ClipartKind kind{ClipartKind::Svg};
    std::uint64_t sizeBytes{0};
    /// Set when the file is listed but will not be read (over its size limit); the message to show.
    std::string refusal;
    /// Index of the library folder it was found under.
    std::size_t root{0};
};

/// Result of `scanClipartFolder`.
struct ClipartScan {
    std::vector<ClipartEntry> entries;  ///< Sorted by folder, then name (case-insensitive), then path.
    /// True when `maxFiles` or `maxVisited` cut the scan short.
    bool truncated{false};
    /// Directory entries examined (a work counter).
    std::size_t visited{0};
    /// Clear messages ("folder unreadable", "stopped after 5000 files", ...).
    std::vector<std::string> messages;
};

/// Lists the clipart files below `root` (UTF-8 path), depth-first in sorted order. Symbolic links and
/// names starting with '.' are skipped, so link loops cannot occur; unreadable directories are
/// reported and skipped. Files over their size limit are listed with a `refusal`.
ClipartScan scanClipartFolder(const std::string& root, const ClipartScanLimits& scanLimits, const ClipartLimits& limits);

/// What is persisted between sessions.
struct ClipartState {
    std::vector<std::string> folders;
    std::vector<std::string> favourites;
};

/// Where the library keeps its state; the app implements it with its settings store.
class ClipartStore {
   public:
    virtual ~ClipartStore() = default;
    virtual ClipartState load() = 0;
    /// False when the state could not be written.
    virtual bool save(const ClipartState& state) = 0;
};

/// In-memory store (tests, headless use).
class MemoryClipartStore : public ClipartStore {
   public:
    ClipartState load() override { return state; }
    bool save(const ClipartState& s) override {
        ++saves;
        if (failSaves) {
            return false;
        }
        state = s;
        return true;
    }
    ClipartState state;
    int saves{0};
    bool failSaves{false};
};

/// Outcome of a folder or favourite edit.
struct ClipartEdit {
    bool ok{false};
    std::string error;
};

/// Most library folders and favourites kept.
inline constexpr std::size_t kMaxClipartFolders = 32;
inline constexpr std::size_t kMaxClipartFavourites = 10'000;

/// The browser's model: user-chosen folders, their scanned files, a name search and favourites.
class ClipartLibrary {
   public:
    /// Loads folders and favourites from `store` (which must outlive the library); malformed or
    /// excess stored items are dropped. Call `rescan()` to list the files.
    ClipartLibrary(ClipartStore& store, ClipartScanLimits scanLimits = {}, ClipartLimits limits = {});

    /// Forgets the folders and favourites held and loads them from the store again (the app switches
    /// from a scratch store to the settings store once it knows persistence is wanted); then rescans.
    void reload();

    /// Adds a folder (must exist and be a directory; no duplicates; at most `kMaxClipartFolders`)
    /// and rescans it. The state is saved.
    ClipartEdit addFolder(const std::string& path);
    /// Removes a folder and its entries. False when it was not a library folder.
    bool removeFolder(const std::string& path);
    const std::vector<std::string>& folders() const { return folders_; }

    /// Scans every folder again. `maxFiles` is shared: later folders get what earlier ones left.
    void rescan();
    const std::vector<ClipartEntry>& entries() const { return entries_; }
    /// Messages of the last scan plus store failures.
    const std::vector<std::string>& messages() const { return messages_; }
    /// Directory entries examined by the last scan, all folders (work counter).
    std::size_t visited() const { return visited_; }

    /// Search text: whitespace-separated words, ASCII case-insensitive, every word must occur in
    /// the file name or its folder path.
    void setQuery(const std::string& query);
    const std::string& query() const { return query_; }
    void setFavouritesOnly(bool only) { favouritesOnly_ = only; }
    bool favouritesOnly() const { return favouritesOnly_; }
    /// Indices into `entries()` that pass the search (and the favourites filter), in list order.
    std::vector<std::size_t> visible() const;

    bool isFavourite(const std::string& path) const { return favourites_.count(path) != 0; }
    /// Marks or unmarks `path` and saves. Refuses past `kMaxClipartFavourites`.
    ClipartEdit setFavourite(const std::string& path, bool favourite);
    /// Flips the mark; returns the new state in `ClipartEdit::ok` semantics via `isFavourite`.
    ClipartEdit toggleFavourite(const std::string& path) { return setFavourite(path, !isFavourite(path)); }
    std::size_t favouriteCount() const { return favourites_.size(); }

    const ClipartLimits& limits() const { return limits_; }

   private:
    bool save();
    void loadState();
    ClipartStore& store_;
    ClipartScanLimits scanLimits_;
    ClipartLimits limits_;
    std::vector<std::string> folders_;
    std::set<std::string> favourites_;
    std::vector<ClipartEntry> entries_;
    std::vector<std::string> messages_;
    std::size_t visited_{0};
    std::string query_;
    std::vector<std::string> words_;
    bool favouritesOnly_{false};
};

}  // namespace carvetto::viewmodel
