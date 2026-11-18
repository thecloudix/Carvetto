#include "carvetto/document/atomic_file.hpp"

#include <cstdio>
#include <system_error>

#if defined(_WIN32)
#include <io.h>
#else
#include <unistd.h>
#endif

namespace carvetto::document {

namespace fs = std::filesystem;

namespace {

bool flushToDisk(std::FILE* f) {
    if (std::fflush(f) != 0) {
        return false;
    }
#if defined(_WIN32)
    return _commit(_fileno(f)) == 0;
#else
    return ::fsync(::fileno(f)) == 0;
#endif
}

bool writeTemp(const fs::path& tmp, std::span<const std::uint8_t> bytes) {
#if defined(_WIN32)
    std::FILE* f = _wfopen(tmp.c_str(), L"wb");
#else
    std::FILE* f = std::fopen(tmp.c_str(), "wb");
#endif
    if (f == nullptr) {
        return false;
    }
    bool ok = bytes.empty() || std::fwrite(bytes.data(), 1, bytes.size(), f) == bytes.size();
    ok = ok && flushToDisk(f);
    ok = (std::fclose(f) == 0) && ok;
    return ok;
}

}  // namespace

bool writeFileAtomic(const fs::path& path, std::span<const std::uint8_t> bytes, std::string& error,
                     const AtomicWriteOptions& options) {
    fs::path tmp = path;
    tmp += ".tmp";
    std::error_code ec;
    auto dropTemp = [&tmp]() {
        std::error_code ignored;
        fs::remove(tmp, ignored);
    };
    if (!writeTemp(tmp, bytes)) {
        dropTemp();
        error = "cannot write '" + tmp.string() + "'";
        return false;
    }
    if (options.failAt == AtomicFailPoint::AfterTempWrite) {
        dropTemp();
        error = "injected failure after writing '" + tmp.string() + "'";
        return false;
    }
    if (options.keepBackup && fs::is_regular_file(path, ec)) {
        fs::path bak = path;
        bak += ".bak";
        ec.clear();
        fs::copy_file(path, bak, fs::copy_options::overwrite_existing, ec);
        if (ec) {
            dropTemp();
            error = "cannot keep a backup '" + bak.string() + "': " + ec.message();
            return false;
        }
    }
    if (options.failAt == AtomicFailPoint::BeforeRename) {
        dropTemp();
        error = "injected failure before replacing '" + path.string() + "'";
        return false;
    }
    ec.clear();
    fs::rename(tmp, path, ec);
    if (ec) {
        dropTemp();
        error = "cannot replace '" + path.string() + "': " + ec.message();
        return false;
    }
    return true;
}

}  // namespace carvetto::document
