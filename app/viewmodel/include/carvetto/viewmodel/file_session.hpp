#pragma once

#include <cstddef>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/dxf_exchange.hpp"

namespace carvetto::viewmodel {

/// Most entries the recent-files list keeps (oldest dropped first).
inline constexpr std::size_t kMaxRecentFiles = 10;

/// UTF-8 text of a path and back (the Qt layer converts to and from `QString` itself).
std::string pathToUtf8(const std::filesystem::path& path);
std::filesystem::path pathFromUtf8(const std::string& text);

/// `path` with ".carv" appended when it has no extension at all (a "Save as" file name typed
/// without one).
std::filesystem::path withCarvExtension(std::filesystem::path path);

/// The open document and its file: path, dirty state, recent files and the last error text
/// (ADR-0014, T-0202). Qt-free; the app's File menu and close prompts are glue over it.
///
/// - The session owns the `Document`. It is *dirty* when the undo stack is not at the state last
///   saved or loaded, so undoing back to the saved state is clean again (`UndoStack::setClean`
///   also ends the coalescing run, so an edit after a save never merges into the saved step).
/// - Every operation returns false on failure and sets `lastError()`; a failed operation leaves
///   the document, path and dirty state as they were. `open` loads the whole file before it
///   replaces anything, so a corrupt or hostile file never damages the current document.
/// - Saving writes a temporary file next to the target and renames it into place (`saveCarv`),
///   so a failed save keeps the previous file intact.
/// - Content this version does not model (`CarvExtras`) is kept from `open` and written back.
class FileSession {
   public:
    /// A new untitled session around a default document.
    FileSession();
    /// A new untitled session around `doc` (taken as the clean baseline).
    explicit FileSession(std::unique_ptr<document::Document> doc);

    document::Document& document() { return *doc_; }
    const document::Document& document() const { return *doc_; }

    /// Replaces the document with a new untitled one (no path, clean, extras forgotten).
    void newDocument(std::unique_ptr<document::Document> doc);

    /// Loads the `.carv` file at `path` and makes it the document. On failure nothing changes,
    /// `lastError()` says why and the path is removed from the recent list.
    bool open(const std::filesystem::path& path);
    /// Saves to the current path. False (with an error) when the session is untitled.
    bool save();
    /// Saves to `path` and makes it the current path.
    bool saveAs(const std::filesystem::path& path);

    /// Imports a DXF file into the document as one undo step (`importDxfIntoDocument`). The
    /// path is not remembered; the document becomes dirty. `forcedMmPerUnit` overrides the units.
    bool importDxf(const std::filesystem::path& path, std::optional<double> forcedMmPerUnit = std::nullopt,
                   std::span<const std::uint8_t> textFontBytes = {});
    /// Imports an image file as a bitmap object (`importBitmapIntoDocument`, T-0250b): one undo
    /// step, the new object is selected, the document becomes dirty. `decoder` reads non-Netpbm
    /// formats.
    bool importBitmap(const std::filesystem::path& path, const document::BitmapDecoder& decoder);
    /// Result of the last successful `importDxf` (warnings for the dialog).
    const DxfImportOutcome& lastImport() const { return lastImport_; }
    /// Writes the active sheet (or the selection) as DXF to `path` (atomically). Does not change
    /// the path or the dirty state.
    bool exportDxf(const std::filesystem::path& path, bool selectionOnly);

    /// True when the document differs from the saved (or newly created / loaded) state.
    bool isDirty() const { return !doc_->undoStack().isClean(); }
    bool hasPath() const { return !path_.empty(); }
    const std::filesystem::path& path() const { return path_; }
    /// File name of the path, or "Untitled".
    std::string displayName() const;
    /// Window title: "name[*] - appName"; the asterisk marks unsaved changes.
    std::string title(const std::string& appName) const;

    /// Why the last operation failed (a sentence for an error dialog); empty after a success.
    const std::string& lastError() const { return error_; }

    /// Recent files, most recent first, at most `kMaxRecentFiles`, without duplicates (paths are
    /// compared after making them absolute and normalising them).
    const std::vector<std::filesystem::path>& recentFiles() const { return recent_; }
    /// Moves `path` to the front (adding it), dropping the oldest entry beyond the cap.
    void addRecent(const std::filesystem::path& path);
    void removeRecent(const std::filesystem::path& path);
    void clearRecent() { recent_.clear(); }
    /// Replaces the list (e.g. from stored settings); duplicates and the excess are dropped.
    void setRecentFiles(const std::vector<std::filesystem::path>& paths);

   private:
    bool fail(std::string message);

    std::unique_ptr<document::Document> doc_;
    document::CarvExtras extras_;
    std::filesystem::path path_;
    std::vector<std::filesystem::path> recent_;
    std::string error_;
    DxfImportOutcome lastImport_;
};

}  // namespace carvetto::viewmodel
