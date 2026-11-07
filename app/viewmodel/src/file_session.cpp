#include "carvetto/viewmodel/file_session.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iterator>
#include <system_error>
#include <utility>

namespace carvetto::viewmodel {

namespace fs = std::filesystem;

namespace {

fs::path normalised(const fs::path& p) {
    std::error_code ec;
    fs::path abs = fs::absolute(p, ec);
    if (ec) {
        abs = p;
    }
    return abs.lexically_normal();
}

/// Reads a whole file; false (with `why`) on any failure.
bool readFile(const fs::path& path, std::vector<std::uint8_t>& out, std::string& why) {
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

/// Writes `bytes` to "<path>.tmp" and renames it over `path`; the old file survives a failure.
bool writeFileAtomic(const fs::path& path, const std::vector<std::uint8_t>& bytes, std::string& why) {
    fs::path tmp = path;
    tmp += ".tmp";
    {
        std::ofstream file(tmp, std::ios::binary | std::ios::trunc);
        if (!file) {
            why = "cannot open '" + pathToUtf8(tmp) + "' for writing";
            return false;
        }
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        file.close();
        if (!file) {
            std::error_code ignored;
            fs::remove(tmp, ignored);
            why = "cannot write '" + pathToUtf8(tmp) + "'";
            return false;
        }
    }
    std::error_code ec;
    fs::rename(tmp, path, ec);
    if (ec) {
        std::error_code ignored;
        fs::remove(tmp, ignored);
        why = "cannot replace '" + pathToUtf8(path) + "': " + ec.message();
        return false;
    }
    return true;
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
        const document::CarvStatus saved = document::saveCarv(*doc_, path, extras_);
        if (!saved) {
            return fail("Cannot save '" + pathToUtf8(path.filename()) + "': " + saved.error().message);
        }
    } catch (const std::exception& e) {
        return fail(std::string("Cannot save the file: ") + e.what());
    }
    path_ = path;
    doc_->undoStack().setClean();
    addRecent(path);
    return true;
}

bool FileSession::importDxf(const fs::path& path, std::optional<double> forcedMmPerUnit) {
    error_.clear();
    try {
        std::vector<std::uint8_t> bytes;
        std::string why;
        if (!readFile(path, bytes, why)) {
            return fail("Cannot open the file: " + why + ".");
        }
        DxfImportOutcome outcome = importDxfIntoDocument(*doc_, bytes, forcedMmPerUnit);
        if (!outcome.ok) {
            return fail(outcome.error);
        }
        lastImport_ = std::move(outcome);
        return true;
    } catch (const std::exception& e) {
        return fail(std::string("Cannot import the DXF file: ") + e.what());
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
        if (!writeFileAtomic(path, out.bytes, why)) {
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
