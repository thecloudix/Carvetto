#include "carv_zip.hpp"

#include <miniz.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <set>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "carv_json_reader.hpp"

namespace carvetto::document::detail {

namespace {

std::string zipErrorText(mz_zip_archive& zip) {
    return mz_zip_get_error_string(mz_zip_get_last_error(&zip));
}

bool endsWithJson(const std::string& name) {
    return name.size() >= 5 && name.compare(name.size() - 5, 5, ".json") == 0;
}

}  // namespace

// ------------------------------------------------------------------------------------------------
// Writing
// ------------------------------------------------------------------------------------------------

ZipWriteResult writeZip(const std::vector<ZipMember>& members, int level) {
    ZipWriteResult result;
    mz_zip_archive zip;
    mz_zip_zero_struct(&zip);
    if (!mz_zip_writer_init_heap(&zip, 0, 0)) {
        result.error = "cannot start zip archive: " + zipErrorText(zip);
        return result;
    }
    static const std::uint8_t kEmpty = 0;
    const auto flags = static_cast<mz_uint>(std::clamp(level, 0, 10));
    for (const ZipMember& m : members) {
        const void* data = m.data.empty() ? static_cast<const void*>(&kEmpty) : m.data.data();
        // MINIZ_NO_TIME is defined for our miniz build, so DOS time and date are written as 0.
        if (!mz_zip_writer_add_mem_ex(&zip, m.name.c_str(), data, m.data.size(), nullptr, 0, flags, 0, 0)) {
            result.error = "cannot add '" + m.name + "' to zip archive: " + zipErrorText(zip);
            mz_zip_writer_end(&zip);
            return result;
        }
    }
    void* buffer = nullptr;
    std::size_t size = 0;
    if (!mz_zip_writer_finalize_heap_archive(&zip, &buffer, &size)) {
        result.error = "cannot finish zip archive: " + zipErrorText(zip);
        mz_zip_writer_end(&zip);
        return result;
    }
    const auto* bytes = static_cast<const std::uint8_t*>(buffer);
    result.bytes.assign(bytes, bytes + size);
    // Finalizing hands ownership of the heap buffer to us (default miniz allocator).
    mz_free(buffer);
    mz_zip_writer_end(&zip);
    result.ok = true;
    return result;
}

// ------------------------------------------------------------------------------------------------
// Reading
// ------------------------------------------------------------------------------------------------

struct ZipArchive::Impl {
    std::span<const std::uint8_t> bytes;
    mz_zip_archive zip{};
    bool initialised{false};
    std::vector<ZipEntry> entries;
};

namespace {

class MinizStream final : public ZipArchive::Stream, public CharSource {
   public:
    MinizStream(mz_zip_archive* zip, mz_zip_reader_extract_iter_state* state, const ZipEntry& entry)
        : zip_(zip), state_(state), entry_(entry) {}
    ~MinizStream() override {
        if (state_) {
            mz_zip_reader_extract_iter_free(state_);
        }
    }
    MinizStream(const MinizStream&) = delete;
    MinizStream& operator=(const MinizStream&) = delete;

    std::size_t readBytes(char* buf, std::size_t n) override {
        if (!state_ || n == 0) {
            return 0;
        }
        const std::size_t got = mz_zip_reader_extract_iter_read(state_, buf, n);
        produced_ += got;
        return got;
    }
    std::size_t read(char* buf, std::size_t n) override { return readBytes(buf, n); }
    CharSource& source() override { return *this; }

    bool finish(std::string& error) override {
        // Drain anything left so the size and CRC can be checked.
        std::array<char, 4096> scratch{};
        while (readBytes(scratch.data(), scratch.size()) > 0) {
        }
        const bool ok = mz_zip_reader_extract_iter_free(state_) != 0;
        state_ = nullptr;
        if (!ok || produced_ != entry_.size) {
            error = "zip member '" + entry_.name + "' is damaged (" + zipErrorText(*zip_) + ")";
            return false;
        }
        return true;
    }

   private:
    mz_zip_archive* zip_;
    mz_zip_reader_extract_iter_state* state_;
    ZipEntry entry_;
    std::uint64_t produced_{0};
};

}  // namespace

ZipArchive::ZipArchive(std::span<const std::uint8_t> bytes) : impl_(std::make_unique<Impl>()) {
    impl_->bytes = bytes;
    mz_zip_zero_struct(&impl_->zip);
}

ZipArchive::~ZipArchive() {
    if (impl_->initialised) {
        mz_zip_reader_end(&impl_->zip);
    }
}

const std::vector<ZipEntry>& ZipArchive::entries() const {
    return impl_->entries;
}

const ZipEntry* ZipArchive::find(const std::string& name) const {
    for (const ZipEntry& e : impl_->entries) {
        if (e.name == name) {
            return &e;
        }
    }
    return nullptr;
}

bool ZipArchive::open(const ZipLimits& limits, std::string& error, bool& limitExceeded) {
    limitExceeded = false;
    if (impl_->bytes.empty()) {
        error = "the file is empty";
        return false;
    }
    mz_zip_archive& zip = impl_->zip;
    if (!mz_zip_reader_init_mem(&zip, impl_->bytes.data(), impl_->bytes.size(), 0)) {
        error = "not a valid zip archive (" + zipErrorText(zip) + ")";
        return false;
    }
    impl_->initialised = true;
    const mz_uint count = mz_zip_reader_get_num_files(&zip);
    if (count > limits.maxMembers) {
        error = "zip archive has too many members (" + std::to_string(count) + ")";
        limitExceeded = true;
        return false;
    }
    std::set<std::string> seen;
    std::uint64_t total = 0;
    std::uint64_t binaryBytes = 0;
    std::uint64_t binaryCompressed = 0;
    for (mz_uint i = 0; i < count; ++i) {
        mz_zip_archive_file_stat st;
        if (!mz_zip_reader_file_stat(&zip, i, &st)) {
            error = "cannot read zip entry " + std::to_string(i) + " (" + zipErrorText(zip) + ")";
            return false;
        }
        const std::string name(st.m_filename);
        if (st.m_is_directory) {
            continue;
        }
        if (!st.m_is_supported || st.m_is_encrypted) {
            error = "zip member '" + name + "' uses an unsupported compression method or encryption";
            return false;
        }
        if (!seen.insert(name).second) {
            error = "zip archive contains '" + name + "' twice";
            return false;
        }
        const bool json = endsWithJson(name);
        const std::uint64_t cap = json ? limits.maxJsonMemberBytes : limits.maxBinaryMemberBytes;
        if (st.m_uncomp_size > cap) {
            error = "zip member '" + name + "' is too large (" + std::to_string(st.m_uncomp_size) +
                    " bytes; the limit is " + std::to_string(cap) + ")";
            limitExceeded = true;
            return false;
        }
        const std::uint64_t ratio = json ? limits.maxJsonRatio : limits.maxBinaryRatio;
        if (st.m_uncomp_size > limits.ratioFromBytes && st.m_uncomp_size / ratio > st.m_comp_size) {
            error = "zip member '" + name + "' expands " + std::to_string(st.m_comp_size) + " bytes to " +
                    std::to_string(st.m_uncomp_size) + " (more than " + std::to_string(ratio) +
                    ":1; a decompression bomb?)";
            limitExceeded = true;
            return false;
        }
        if (!json) {
            binaryBytes += st.m_uncomp_size;
            binaryCompressed += st.m_comp_size;
            // Binary members are held in memory, so many small (individually unchecked) members
            // must not add up to a bomb either: the same ratio applies to all of them together.
            if (binaryBytes > limits.ratioFromBytes &&
                binaryBytes / limits.maxBinaryRatio > binaryCompressed) {
                error = "binary members expand " + std::to_string(binaryCompressed) + " bytes to " +
                        std::to_string(binaryBytes) + " (more than " + std::to_string(limits.maxBinaryRatio) +
                        ":1 overall; a decompression bomb?)";
                limitExceeded = true;
                return false;
            }
        }
        total += st.m_uncomp_size;
        if (total > limits.maxTotalBytes) {
            error = "zip archive expands to more than " + std::to_string(limits.maxTotalBytes) + " bytes";
            limitExceeded = true;
            return false;
        }
        impl_->entries.push_back(ZipEntry{i, name, st.m_uncomp_size, st.m_crc32, json});
    }
    return true;
}

std::unique_ptr<ZipArchive::Stream> ZipArchive::stream(const ZipEntry& entry, std::string& error) {
    mz_zip_reader_extract_iter_state* state = mz_zip_reader_extract_iter_new(&impl_->zip, entry.index, 0);
    if (!state) {
        error = "zip member '" + entry.name + "' is damaged (" + zipErrorText(impl_->zip) + ")";
        return nullptr;
    }
    return std::make_unique<MinizStream>(&impl_->zip, state, entry);
}

bool ZipArchive::readAll(const ZipEntry& entry, std::vector<std::uint8_t>& out, std::string& error) {
    out.clear();
    auto s = stream(entry, error);
    if (!s) {
        return false;
    }
    std::array<char, 1 << 16> chunk{};
    for (;;) {
        const std::size_t got = s->readBytes(chunk.data(), chunk.size());
        if (got == 0) {
            break;
        }
        if (out.size() + got > entry.size) {
            error = "zip member '" + entry.name + "' is damaged (more data than declared)";
            return false;
        }
        if (out.size() + got > out.capacity()) {
            // Grow geometrically as data arrives, never past the declared size: a member that
            // declares a large size but holds little never allocates it.
            const std::size_t want = std::max<std::size_t>(out.capacity() * 2, std::size_t{1} << 20);
            out.reserve(std::min<std::size_t>(want, static_cast<std::size_t>(entry.size)));
        }
        const auto* p = reinterpret_cast<const std::uint8_t*>(chunk.data());
        out.insert(out.end(), p, p + got);
    }
    return s->finish(error);
}

ZipReadResult readZip(std::span<const std::uint8_t> bytes, const ZipLimits& limits) {
    ZipReadResult result;
    ZipArchive archive(bytes);
    if (!archive.open(limits, result.error, result.limitExceeded)) {
        return result;
    }
    for (const ZipEntry& e : archive.entries()) {
        ZipMember m;
        m.name = e.name;
        if (!archive.readAll(e, m.data, result.error)) {
            result.members.clear();
            return result;
        }
        result.members.push_back(std::move(m));
    }
    result.ok = true;
    return result;
}

}  // namespace carvetto::document::detail
