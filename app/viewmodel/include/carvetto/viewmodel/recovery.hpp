#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "carvetto/viewmodel/file_session.hpp"

namespace carvetto::viewmodel {

/// A recovery file left behind by a session that did not exit cleanly.
struct RecoveryEntry {
    std::filesystem::path file;  ///< The autosaved `.carv`.
    std::string originalPath;    ///< UTF-8 path the document was saved under; empty when it was untitled.
    std::string displayName;     ///< File name of the original, or "Untitled".
};

/// Autosave and crash recovery (T-0357, ADR-0128). Qt-free: the app's timer calls `tick`, a clean exit
/// calls `discard`, and the next start asks `pending` and offers `restore`.
///
/// The store keeps at most one snapshot in `dir`: "autosave.carv" (the document, written atomically) and
/// "autosave.meta" (the original path, one UTF-8 line). A clean exit removes both, so whatever is found at
/// start-up belongs to a session that was killed. (Two instances at once share the slot; the later write wins.)
class RecoveryStore {
   public:
    explicit RecoveryStore(std::filesystem::path dir);

    const std::filesystem::path& directory() const { return dir_; }
    std::filesystem::path snapshotFile() const { return dir_ / "autosave.carv"; }

    /// Writes a snapshot of `session` now. False (with `lastError`) on failure; the session is unchanged.
    bool snapshot(FileSession& session);
    /// One autosave timer step: a snapshot when the session has unsaved changes, else the stale snapshot
    /// is removed (the work is saved or empty). Returns whether a snapshot now exists.
    bool tick(FileSession& session);
    /// Removes the snapshot (clean exit, or the user declined recovery).
    void discard();

    /// The snapshot left by an unclean exit, if any.
    std::optional<RecoveryEntry> pending() const;
    /// Loads the pending snapshot into `session` (dirty, under its original path) and removes it only after
    /// the load succeeded. False (with `lastError`) when it is damaged; the damaged file is then removed so
    /// start-up is not blocked by it, and `session` is unchanged.
    bool restore(FileSession& session);

    const std::string& lastError() const { return error_; }

   private:
    std::filesystem::path dir_;
    std::string error_;
};

}  // namespace carvetto::viewmodel
