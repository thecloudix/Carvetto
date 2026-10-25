#pragma once

// Internal: zip container access for `.carv` files, wrapping miniz so that no public header (and
// no other source file) depends on it. Tests include this header directly.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace carvetto::document::detail {

class CharSource;

/// One archive member: name and uncompressed content.
struct ZipMember {
    std::string name;
    std::vector<std::uint8_t> data;

    bool operator==(const ZipMember&) const = default;
};

/// Limits applied while opening untrusted archives (ADR-0013 §9). They use the sizes declared in
/// the central directory and are checked before anything is decompressed; extraction then never
/// produces more than the declared size.
struct ZipLimits {
    std::uint64_t maxMembers{10000};
    /// Members whose name ends in ".json" (streamed, never held whole).
    std::uint64_t maxJsonMemberBytes{std::uint64_t{256} << 20};
    /// Any other member (held in memory as raw bytes).
    std::uint64_t maxBinaryMemberBytes{std::uint64_t{256} << 20};
    /// Sum of all members' declared uncompressed sizes.
    std::uint64_t maxTotalBytes{std::uint64_t{1} << 30};
    /// Members larger than `ratioFromBytes` may expand at most this many times their compressed
    /// size: JSON compresses well (a real 100k-object job.json ~19:1, all-identical objects
    /// 124:1); binary members (zstd heightfields, PNGs) barely compress. The binary ratio also
    /// applies to all binary members together (once their total exceeds `ratioFromBytes`).
    std::uint64_t maxJsonRatio{200};
    std::uint64_t maxBinaryRatio{20};
    std::uint64_t ratioFromBytes{std::uint64_t{1} << 20};
};

/// A validated member of an open archive.
struct ZipEntry {
    std::uint32_t index{0};
    std::string name;
    std::uint64_t size{0};
    std::uint32_t crc{0};
    bool json{false};
};

/// An open archive. `open` validates the central directory against the limits; members are then
/// read incrementally (whole, growing as data arrives) or streamed.
class ZipArchive {
   public:
    explicit ZipArchive(std::span<const std::uint8_t> bytes);
    ~ZipArchive();
    ZipArchive(const ZipArchive&) = delete;
    ZipArchive& operator=(const ZipArchive&) = delete;

    /// False with `error` set on failure; `limitExceeded` tells a limit from damage.
    bool open(const ZipLimits& limits, std::string& error, bool& limitExceeded);
    const std::vector<ZipEntry>& entries() const;
    const ZipEntry* find(const std::string& name) const;

    /// Reads a whole member, growing the buffer as data arrives (never past the declared size).
    bool readAll(const ZipEntry& entry, std::vector<std::uint8_t>& out, std::string& error);

    /// A member decompressed on the fly. `finish` verifies the size and CRC once everything was read.
    class Stream;
    std::unique_ptr<Stream> stream(const ZipEntry& entry, std::string& error);

   private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

/// Streaming access to one member; also a `CharSource` for the JSON reader.
class ZipArchive::Stream {
   public:
    virtual ~Stream() = default;
    /// Copies up to `n` bytes; 0 at the end (or on damage, reported by `finish`).
    virtual std::size_t readBytes(char* buf, std::size_t n) = 0;
    /// After reading to the end: true when the size and CRC match.
    virtual bool finish(std::string& error) = 0;
    /// The stream as a `CharSource`.
    virtual CharSource& source() = 0;
};

/// Outcome of `readZip`.
struct ZipReadResult {
    bool ok{false};
    /// True when the failure was a limit in `ZipLimits` rather than a damaged archive.
    bool limitExceeded{false};
    std::string error;
    /// Members in central-directory order (directories skipped).
    std::vector<ZipMember> members;
};

/// Outcome of `writeZip`.
struct ZipWriteResult {
    bool ok{false};
    std::string error;
    std::vector<std::uint8_t> bytes;
};

/// Writes `members` in the given order, deflated at `level` (default 6, what `.carv` files use),
/// with DOS time/date zero and no extra fields, so equal input always gives equal bytes.
ZipWriteResult writeZip(const std::vector<ZipMember>& members, int level = 6);

/// Opens and reads every member (CRC-checked), under `limits`.
ZipReadResult readZip(std::span<const std::uint8_t> bytes, const ZipLimits& limits = {});

}  // namespace carvetto::document::detail
