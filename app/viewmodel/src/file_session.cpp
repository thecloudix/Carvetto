#include "carvetto/viewmodel/file_session.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iterator>
#include <system_error>
#include <utility>

#include "carvetto/document/atomic_file.hpp"
#include "carvetto/viewmodel/bitmap_import.hpp"

namespace carvetto::viewmodel {

namespace fs = std::filesystem;

namespace {

fs::path normalised(const fs::path& p) {
    std::error_code ec;
    fs::path canon = fs::weakly_canonical(p, ec);
    if (ec) {
        ec.clear();
        canon = fs::absolute(p, ec);
        if (ec) {
            canon = p;
        }
        canon = canon.lexically_normal();
    }
    return canon;
}

/// Reads a whole file; false (with `why`) on any failure.
bool readFile(const fs::path& path, std::vector<std::uint8_t>& out, std::string& why) {
    std::error_code statEc;
    const fs::file_status st = fs::status(path, statEc);
    if (statEc || !fs::exists(st)) {
        why = "'" + pathToUtf8(path) + "' does not exist";
        return false;
    }
    if (fs::is_directory(st)) {
        why = "'" + pathToUtf8(path) + "' is a folder, not a file";
        return false;
    }
    if (!fs::is_regular_file(st)) {
        why = "'" + pathToUtf8(path) + "' is not a regular file";
        return false;
    }
    std::error_code sizeEc;
    const std::uintmax_t size = fs::file_size(path, sizeEc);
    if (!sizeEc && size > kMaxOpenFileBytes) {
        why = "'" + pathToUtf8(path) + "' is too large (" + std::to_string(size >> 20) + " MiB; the limit is " +
              std::to_string(kMaxOpenFileBytes >> 20) + " MiB)";
        return false;
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        why = "cannot open '" + pathToUtf8(path) + "' for reading";
        return false;
    }
    out.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    if (in.bad()) {
        why = "cannot read '" + pathToUtf8(path) + "'";
        return false;
    }
    return true;
}

/// Atomic write (temporary file, fsync, rename) through core; `why` names the failure.
bool writeAtomic(const fs::path& path, const std::vector<std::uint8_t>& bytes, std::string& why,
                 const document::AtomicWriteOptions& options = {}) {
    return document::writeFileAtomic(path, bytes, why, options);
}

}  // namespace

std::string pathToUtf8(const fs::path& path) {
    const auto u8 = path.u8string();
    std::string out;
    out.reserve(u8.size());
    for (const auto c : u8) {
        out.push_back(static_cast<char>(c));
    }
    return out;
}

fs::path pathFromUtf8(const std::string& text) {
    std::u8string u8;
    u8.reserve(text.size());
    for (const char c : text) {
        u8.push_back(static_cast<char8_t>(static_cast<unsigned char>(c)));
    }
    return fs::path(u8);
}

fs::path withCarvExtension(fs::path path) {
    if (!path.empty() && !path.has_extension()) {
        path += ".carv";
    }
    return path;
}

FileSession::FileSession() : doc_(std::make_unique<document::Document>()) {}

FileSession::FileSession(std::unique_ptr<document::Document> doc) : doc_(std::move(doc)) {
    if (!doc_) {
        doc_ = std::make_unique<document::Document>();
    }
    doc_->undoStack().setClean();
}

bool FileSession::fail(std::string message) {
    error_ = std::move(message);
    return false;
}

void FileSession::newDocument(std::unique_ptr<document::Document> doc) {
    if (!doc) {
        return;
    }
    doc_ = std::move(doc);
    doc_->undoStack().setClean();
    extras_ = {};
    path_.clear();
    recovered_ = false;
    error_.clear();
}

bool FileSession::open(const fs::path& path) {
    error_.clear();
    try {
        std::vector<std::uint8_t> bytes;
        std::string why;
        if (!readFile(path, bytes, why)) {
            removeRecent(path);
            return fail("Cannot open the file: " + why + ".");
        }
        auto loaded = document::loadCarv(std::span<const std::uint8_t>(bytes));
        if (!loaded) {
            removeRecent(path);
            return fail("Cannot open '" + pathToUtf8(path.filename()) + "': " + loaded.error().message);
        }
        doc_ = std::make_unique<document::Document>(std::move(loaded.value().document));
        extras_ = loaded.value().extras;
        doc_->undoStack().setClean();
        recovered_ = false;
        path_ = path;
        addRecent(path);
        return true;
    } catch (const std::exception& e) {
        removeRecent(path);
        return fail(std::string("Cannot open the file: ") + e.what());
    }
}

bool FileSession::save() {
    error_.clear();
    if (path_.empty()) {
        return fail("The document has no file name yet; use Save As.");
    }
    return saveAs(path_);
}

bool FileSession::saveAs(const fs::path& path) {
    error_.clear();
    if (path.empty()) {
        return fail("No file name was given.");
    }
    try {
        const auto bytes = document::saveCarv(*doc_, extras_);
        if (!bytes) {
            return fail("Cannot save '" + pathToUtf8(path.filename()) + "': " + bytes.error().message);
        }
        document::AtomicWriteOptions options;
        options.keepBackup = true;  // the previous file stays next to it as "<name>.carv.bak"
        options.failAt = failAt_;
        std::string why;
        if (!writeAtomic(path, bytes.value(), why, options)) {
            return fail("Cannot save '" + pathToUtf8(path.filename()) + "': " + why +
                        ". The previous file is unchanged.");
        }
    } catch (const std::exception& e) {
        return fail(std::string("Cannot save the file: ") + e.what());
    }
    path_ = path;
    doc_->undoStack().setClean();
    recovered_ = false;
    addRecent(path);
    return true;
}

bool FileSession::writeSnapshot(const fs::path& path) {
    error_.clear();
    try {
        const auto bytes = document::saveCarv(*doc_, extras_);
        if (!bytes) {
            return fail("Cannot write the recovery file: " + bytes.error().message);
        }
        std::string why;
        if (!writeAtomic(path, bytes.value(), why)) {
            return fail("Cannot write the recovery file: " + why);
        }
        return true;
    } catch (const std::exception& e) {
        return fail(std::string("Cannot write the recovery file: ") + e.what());
    }
}

bool FileSession::openRecovered(const fs::path& recoveryFile, const fs::path& originalPath) {
    error_.clear();
    try {
        std::vector<std::uint8_t> bytes;
        std::string why;
        if (!readFile(recoveryFile, bytes, why)) {
            return fail("Cannot read the recovery file: " + why + ".");
        }
        auto loaded = document::loadCarv(std::span<const std::uint8_t>(bytes));
        if (!loaded) {
            return fail("The recovery file is damaged: " + loaded.error().message);
        }
        doc_ = std::make_unique<document::Document>(std::move(loaded.value().document));
        extras_ = loaded.value().extras;
        doc_->undoStack().setClean();
        recovered_ = true;  // the recovered work is not on disk under its name yet
        path_ = originalPath;
        return true;
    } catch (const std::exception& e) {
        return fail(std::string("Cannot read the recovery file: ") + e.what());
    }
}

bool FileSession::importDxf(const fs::path& path, std::optional<double> forcedMmPerUnit,
                            std::span<const std::uint8_t> textFontBytes) {
    error_.clear();
    try {
        std::vector<std::uint8_t> bytes;
        std::string why;
        if (!readFile(path, bytes, why)) {
            return fail("Cannot open the file: " + why + ".");
        }
        DxfImportOutcome outcome = importDxfIntoDocument(*doc_, bytes, forcedMmPerUnit, textFontBytes);
        if (!outcome.ok) {
            return fail(outcome.error);
        }
        lastImport_ = std::move(outcome);
        return true;
    } catch (const std::exception& e) {
        return fail(std::string("Cannot import the DXF file: ") + e.what());
    }
}

bool FileSession::importBitmap(const fs::path& path, const document::BitmapDecoder& decoder) {
    error_.clear();
    try {
        std::error_code sizeEc;
        const std::uintmax_t size = fs::file_size(path, sizeEc);
        if (!sizeEc && size > document::kMaxEmbeddedBitmapBytes) {
            return fail("Cannot import the bitmap: the file is too large to embed (more than 64 MiB).");
        }
        std::vector<std::uint8_t> bytes;
        std::string why;
        if (!readFile(path, bytes, why)) {
            return fail("Cannot open the file: " + why + ".");
        }
        const BitmapImportOutcome outcome =
            importBitmapIntoDocument(*doc_, std::move(bytes), pathToUtf8(path.filename()), decoder);
        if (!outcome.ok) {
            return fail("Cannot import the bitmap: " + outcome.error + ".");
        }
        return true;
    } catch (const std::exception& e) {
        return fail(std::string("Cannot import the bitmap: ") + e.what());
    }
}

bool FileSession::exportDxf(const fs::path& path, bool selectionOnly) {
    error_.clear();
    try {
        const DxfExportOutcome out = exportDocumentDxf(*doc_, selectionOnly);
        if (!out.ok) {
            return fail(out.error);
        }
        std::string why;
        if (!writeAtomic(path, out.bytes, why)) {
            return fail("Cannot export DXF: " + why + ".");
        }
        return true;
    } catch (const std::exception& e) {
        return fail(std::string("Cannot export DXF: ") + e.what());
    }
}

std::string FileSession::displayName() const {
    return path_.empty() ? std::string("Untitled") : pathToUtf8(path_.filename());
}

std::string FileSession::title(const std::string& appName) const {
    return displayName() + (isDirty() ? "*" : "") + " - " + appName;
}

void FileSession::removeRecent(const fs::path& path) {
    const fs::path key = normalised(path);
    recent_.erase(std::remove_if(recent_.begin(), recent_.end(), [&](const fs::path& p) { return normalised(p) == key; }),
                  recent_.end());
}

void FileSession::addRecent(const fs::path& path) {
    if (path.empty()) {
        return;
    }
    removeRecent(path);
    recent_.insert(recent_.begin(), path);
    if (recent_.size() > kMaxRecentFiles) {
        recent_.resize(kMaxRecentFiles);
    }
}

void FileSession::setRecentFiles(const std::vector<fs::path>& paths) {
    recent_.clear();
    for (const fs::path& p : paths) {
        if (p.empty()) {
            continue;
        }
        const fs::path key = normalised(p);
        const bool seen = std::any_of(recent_.begin(), recent_.end(), [&](const fs::path& q) { return normalised(q) == key; });
        if (!seen && recent_.size() < kMaxRecentFiles) {
            recent_.push_back(p);
        }
    }
}

}  // namespace carvetto::viewmodel
