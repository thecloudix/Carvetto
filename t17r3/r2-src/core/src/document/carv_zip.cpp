#include "carv_zip.hpp"

#include <miniz.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
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

/// Little-endian reads from the archive bytes.
std::uint16_t le16(std::span<const std::uint8_t> b, std::uint64_t at) {
    return static_cast<std::uint16_t>(b[static_cast<std::size_t>(at)] |
                                      (b[static_cast<std::size_t>(at) + 1] << 8));
}
std::uint32_t le32(std::span<const std::uint8_t> b, std::uint64_t at) {
    return static_cast<std::uint32_t>(b[static_cast<std::size_t>(at)]) |
           (static_cast<std::uint32_t>(b[static_cast<std::size_t>(at) + 1]) << 8) |
           (static_cast<std::uint32_t>(b[static_cast<std::size_t>(at) + 2]) << 16) |
           (static_cast<std::uint32_t>(b[static_cast<std::size_t>(at) + 3]) << 24);
}

constexpr std::uint64_t kLocalHeaderSize = 30;
constexpr std::uint32_t kLocalHeaderSig = 0x04034b50u;
constexpr std::size_t kInflateChunk = std::size_t{1} << 16;

/// Our own inflate loop over tinfl for one member of an in-memory archive. Input is the exact
/// [dataOffset, dataEnd) range from the (validated) local header; output is produced into a
/// 32 KiB wrapping dictionary and copied out on demand. Every call either produces output,
/// consumes input, or fails, so the loop cannot spin.
class BoundedStream final : public ZipArchive::Stream, public CharSource {
   public:
    BoundedStream(std::span<const std::uint8_t> bytes, const ZipEntry& entry)
        : in_(bytes.subspan(static_cast<std::size_t>(entry.dataOffset),
                            static_cast<std::size_t>(entry.dataEnd - entry.dataOffset))),
          entry_(entry) {
        if (!entry.stored) {
            dict_.resize(TINFL_LZ_DICT_SIZE);
            tinfl_init(&inflator_);
        }
    }

    std::size_t readBytes(char* buf, std::size_t n) override {
        if (failed_ || n == 0) {
            return 0;
        }
        std::size_t copied = 0;
        while (copied < n) {
            if (dictAvail_ == 0) {
                if (!produceMore()) {
                    break;
                }
                if (dictAvail_ == 0) {
                    break;  // done
                }
            }
            const std::size_t take = std::min(n - copied, dictAvail_);
            std::memcpy(buf + copied, dict_.data() + dictRead_, take);
            crc_ = static_cast<std::uint32_t>(mz_crc32(crc_, dict_.data() + dictRead_, take));
            dictRead_ = (dictRead_ + take) & (TINFL_LZ_DICT_SIZE - 1);
            dictAvail_ -= take;
            copied += take;
            produced_ += take;
        }
        return copied;
    }
    std::size_t read(char* buf, std::size_t n) override { return readBytes(buf, n); }
    CharSource& source() override { return *this; }

    bool finish(std::string& error) override {
        std::array<char, 4096> scratch{};
        while (!failed_ && readBytes(scratch.data(), scratch.size()) > 0) {
        }
        if (failed_) {
            error = "zip member '" + entry_.name + "' is damaged (" + failure_ + ")";
            return false;
        }
        if (produced_ != entry_.size) {
            error = "zip member '" + entry_.name + "' is damaged (decompressed to " +
                    std::to_string(produced_) + " bytes, " + std::to_string(entry_.size) + " declared)";
            return false;
        }
        if (crc_ != entry_.crc) {
            error = "zip member '" + entry_.name + "' is damaged (CRC mismatch)";
            return false;
        }
        return true;
    }

   private:
    bool fail(const char* why) {
        failed_ = true;
        failure_ = why;
        return false;
    }

    /// Fills the dictionary with the next block of output. Returns false on failure; leaves
    /// dictAvail_ == 0 when the stream is complete.
    bool produceMore() {
        if (done_) {
            return true;
        }
        if (entry_.stored) {
            // Stored member: the data *is* the output; serve it in dictionary-sized pieces.
            const std::size_t left = in_.size() - inPos_;
            const std::size_t take = std::min(
                left, dict_.empty() ? (dict_.resize(TINFL_LZ_DICT_SIZE), dict_.size()) : dict_.size());
            if (take == 0) {
                done_ = true;
                return true;
            }
            if (produced_ + take > entry_.size) {
                return fail("more data than declared");
            }
            std::memcpy(dict_.data(), in_.data() + inPos_, take);
            inPos_ += take;
            dictRead_ = 0;
            dictAvail_ = take;
            return true;
        }
        for (;;) {
            // The wrapping buffer: write from dictWrite_ up to the end of the dictionary.
            std::size_t outSize = TINFL_LZ_DICT_SIZE - dictWrite_;
            std::size_t inSize = std::min(in_.size() - inPos_, kInflateChunk);
            const bool moreInput = inPos_ + inSize < in_.size();
            const tinfl_status status = tinfl_decompress(&inflator_, in_.data() + inPos_, &inSize,
                                                         dict_.data(), dict_.data() + dictWrite_, &outSize,
                                                         moreInput ? TINFL_FLAG_HAS_MORE_INPUT : 0);
            inPos_ += inSize;
            if (status < TINFL_STATUS_DONE) {
                return fail("deflate stream is corrupted");
            }
            if (outSize > 0) {
                if (produced_ + dictAvail_ + outSize > entry_.size) {
                    return fail("more data than declared");
                }
                dictRead_ = dictWrite_;
                dictWrite_ = (dictWrite_ + outSize) & (TINFL_LZ_DICT_SIZE - 1);
                dictAvail_ = outSize;
                if (status == TINFL_STATUS_DONE) {
                    done_ = true;
                }
                return true;
            }
            if (status == TINFL_STATUS_DONE) {
                done_ = true;
                return true;
            }
            if (status == TINFL_STATUS_NEEDS_MORE_INPUT) {
                if (inPos_ >= in_.size()) {
                    return fail("deflate stream is truncated");  // all declared input consumed
                }
                if (inSize == 0) {
                    return fail("deflate stream makes no progress");
                }
                continue;  // consumed some input, produced nothing yet: feed the next chunk
            }
            // HAS_MORE_OUTPUT with outSize == 0 cannot happen with a free dictionary; guard anyway.
            if (inSize == 0) {
                return fail("deflate stream makes no progress");
            }
        }
    }

    std::span<const std::uint8_t> in_;
    ZipEntry entry_;
    std::size_t inPos_{0};
    tinfl_decompressor inflator_{};
    std::vector<std::uint8_t> dict_;
    std::size_t dictRead_{0};
    std::size_t dictWrite_{0};
    std::size_t dictAvail_{0};
    std::uint64_t produced_{0};
    std::uint32_t crc_{MZ_CRC32_INIT};
    bool done_{false};
    bool failed_{false};
    std::string failure_;
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
        if (st.m_method != MZ_DEFLATED && st.m_method != 0) {
            error = "zip member '" + name + "' uses an unsupported compression method";
            return false;
        }
        // Read the local header ourselves to learn where the data really is.
        const std::uint64_t lh = st.m_local_header_ofs;
        const std::uint64_t total_size = impl_->bytes.size();
        if (lh + kLocalHeaderSize > total_size || le32(impl_->bytes, lh) != kLocalHeaderSig) {
            error = "zip member '" + name + "' has a bad local header";
            return false;
        }
        const std::uint64_t nameLen = le16(impl_->bytes, lh + 26);
        const std::uint64_t extraLen = le16(impl_->bytes, lh + 28);
        const std::uint64_t dataOffset = lh + kLocalHeaderSize + nameLen + extraLen;
        const std::uint64_t dataEnd = dataOffset + st.m_comp_size;
        if (dataOffset > total_size || dataEnd > total_size || dataEnd < dataOffset) {
            error = "zip member '" + name + "' points past the end of the file";
            return false;
        }
        ZipEntry entry{i,    name, st.m_uncomp_size, st.m_comp_size, st.m_crc32,
                       json, lh,   dataOffset,       dataEnd,        st.m_method == 0};
        if (entry.stored && entry.compressedSize != entry.size) {
            error = "zip member '" + name + "' is stored but its sizes differ";
            return false;
        }
        impl_->entries.push_back(entry);
    }
    // Each entry's [local header, data end) must be disjoint from every other's: a shared or
    // overlapping stream would otherwise be inflated once per entry.
    std::vector<std::pair<std::uint64_t, std::uint64_t>> ranges;
    ranges.reserve(impl_->entries.size());
    for (const ZipEntry& e : impl_->entries) {
        ranges.emplace_back(e.headerOffset, e.dataEnd);
    }
    std::sort(ranges.begin(), ranges.end());
    for (std::size_t k = 1; k < ranges.size(); ++k) {
        if (ranges[k].first < ranges[k - 1].second) {
            error = "zip archive has overlapping or duplicated member data (entries share a local header)";
            return false;
        }
    }
    return true;
}

std::unique_ptr<ZipArchive::Stream> ZipArchive::stream(const ZipEntry& entry, std::string& error) {
    if (entry.dataEnd > impl_->bytes.size() || entry.dataOffset > entry.dataEnd) {
        error = "zip member '" + entry.name + "' points past the end of the file";
        return nullptr;
    }
    return std::make_unique<BoundedStream>(impl_->bytes, entry);
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
