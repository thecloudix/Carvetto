#include "carvetto/viewmodel/recovery.hpp"

#include <fstream>
#include <string>
#include <system_error>

namespace carvetto::viewmodel {

namespace fs = std::filesystem;

namespace {

fs::path metaFile(const fs::path& dir) { return dir / "autosave.meta"; }

}  // namespace

RecoveryStore::RecoveryStore(fs::path dir) : dir_(std::move(dir)) {}

bool RecoveryStore::snapshot(FileSession& session) {
    error_.clear();
    std::error_code ec;
    fs::create_directories(dir_, ec);
    if (ec) {
        error_ = "Cannot create the recovery folder: " + ec.message();
        return false;
    }
    if (!session.writeSnapshot(snapshotFile())) {
        error_ = session.lastError();
        return false;
    }
    // The meta file is advisory: without it the document is recovered as untitled.
    std::ofstream meta(metaFile(dir_), std::ios::binary | std::ios::trunc);
    meta << (session.hasPath() ? pathToUtf8(session.path()) : std::string()) << '\n';
    return true;
}

bool RecoveryStore::tick(FileSession& session) {
    if (!session.isDirty()) {
        discard();
        return false;
    }
    return snapshot(session);
}

void RecoveryStore::discard() {
    std::error_code ec;
    fs::remove(snapshotFile(), ec);
    fs::remove(metaFile(dir_), ec);
    fs::remove(snapshotFile().string() + ".tmp", ec);
}

std::optional<RecoveryEntry> RecoveryStore::pending() const {
    std::error_code ec;
    if (!fs::is_regular_file(snapshotFile(), ec)) {
        return std::nullopt;
    }
    RecoveryEntry e;
    e.file = snapshotFile();
    std::ifstream meta(metaFile(dir_), std::ios::binary);
    std::getline(meta, e.originalPath);
    while (!e.originalPath.empty() && (e.originalPath.back() == '\r' || e.originalPath.back() == '\n')) {
        e.originalPath.pop_back();
    }
    e.displayName = e.originalPath.empty() ? std::string("Untitled")
                                           : pathToUtf8(pathFromUtf8(e.originalPath).filename());
    return e;
}

bool RecoveryStore::restore(FileSession& session) {
    error_.clear();
    const auto entry = pending();
    if (!entry) {
        error_ = "There is nothing to recover.";
        return false;
    }
    if (!session.openRecovered(entry->file, pathFromUtf8(entry->originalPath))) {
        error_ = session.lastError();
        discard();
        return false;
    }
    discard();
    return true;
}

}  // namespace carvetto::viewmodel
