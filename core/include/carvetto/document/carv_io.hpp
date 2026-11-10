#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/document/document.hpp"

/// Native `.carv` file I/O (ADR-0003, details in ADR-0013).
///
/// A `.carv` file is a zip archive holding `manifest.json` (format and app version), `job.json`
/// (job, sheets, layers in stacking order, objects in id order with their contours and creation
/// parameters, group links, toolpath records in job order, document variables, active
/// layer/sheet and the id generator's high-water mark) and one binary move cache
/// `toolpaths/NNNN.bin` per toolpath record with computed moves (ADR-0021). Interim toolpath
/// members `toolpaths/NNNN.json` written by carvetto-cli before format 1.1 (ADR-0019) are
/// converted to toolpath records on load. Output is deterministic: sorted keys, fixed member order, shortest round-trip
/// number formatting and zeroed zip timestamps, so save -> load -> save reproduces the archive byte
/// for byte. Fields and archive members this version does not understand are kept in `CarvExtras`
/// and written back on the next save.
namespace carvetto::document {

/// Format version written by this build (`manifest.json` -> `format_version`).
inline constexpr int kCarvFormatMajor = 1;
/// Minor format version written by this build. Minor bumps only add optional content.
/// 1.1 (T-0132, ADR-0021) adds toolpath records (`job.json` `toolpaths`) and their move caches.
/// 1.2 (T-0211, ADR-0033 amendment A1) adds text parameters (`"type": "text"`) and embedded fonts
/// (`fonts/<key>.font` members); a document without text objects is still written as 1.1.
/// 1.3 (T-0241, ADR-0035) adds keep-out zones (`job.json` `keepout`); written only when the
/// document has zones or a non-default clearance/visibility.
inline constexpr int kCarvFormatMinor = 3;
/// The minor version written for documents that use text (1.2) but no 1.3 features.
inline constexpr int kCarvFormatMinorText = 2;
/// The minor version written for documents that use no 1.2 features (no text objects): 1.1.
inline constexpr int kCarvFormatMinorBase = 1;
/// Most toolpath moves (summed over every toolpath record) a `.carv` file may hold: `loadCarv`
/// refuses files with more and `saveCarv` refuses to write them (ADR-0021).
inline constexpr std::uint64_t kCarvMaxToolpathMoves = 8'000'000;
/// Toolpath moves allowed per byte of the `.carv` file (owner decision 2026-11-02): a file of S
/// bytes holds at most min(S x this, `kCarvMaxToolpathMoves`) moves. Real files take 6-9 bytes
/// per move, so only crafted (or degenerate, highly repetitive) toolpaths reach it.
inline constexpr std::uint64_t kCarvMovesPerFileByte = 1;
/// Highest position (1-based) a toolpath with computed moves may have, since caches are named
/// `toolpaths/0001.bin` .. `toolpaths/9999.bin` by position. The archive's 10,000-member limit
/// (manifest and job.json included) means at most 9,998 computed toolpaths fit in practice.
inline constexpr std::size_t kCarvMaxCachedToolpaths = 9999;

/// Why a `.carv` operation failed.
struct CarvError {
    /// Failure category, for callers that react differently (e.g. "update Carvetto").
    enum class Code {
        Io,                  ///< The file could not be read or written.
        InvalidArchive,      ///< Not a zip archive, truncated, or a member failed its CRC.
        MissingMember,       ///< `manifest.json` or `job.json` is absent.
        InvalidJson,         ///< A member is not well-formed JSON (or nests too deeply).
        UnsupportedVersion,  ///< The format's major version is not one this build reads.
        InvalidContent,      ///< Well-formed JSON that breaks the schema (types, ids, references).
        IntegrityFailure,    ///< The loaded document fails `Document::checkIntegrity()`.
        ResourceLimit,       ///< A size, count, ratio, nesting or load-budget limit (ADR-0013 §9).
    };
    Code code{Code::InvalidContent};
    /// Human-readable description naming the member, item and problem.
    std::string message;
};

/// Value-or-error return type (a small stand-in for C++23 `std::expected`).
template <class T>
class CarvResult {
   public:
    CarvResult(T value) : v_(std::move(value)) {}          // NOLINT(google-explicit-constructor)
    CarvResult(CarvError error) : v_(std::move(error)) {}  // NOLINT(google-explicit-constructor)

    /// True when a value is held.
    bool ok() const { return v_.index() == 0; }
    explicit operator bool() const { return ok(); }

    /// The value; only valid when `ok()`.
    T& value() { return std::get<0>(v_); }
    /// The value; only valid when `ok()`.
    const T& value() const { return std::get<0>(v_); }
    /// The error; only valid when `!ok()`.
    const CarvError& error() const { return std::get<1>(v_); }

   private:
    std::variant<T, CarvError> v_;
};

/// Success-or-error for operations without a value.
using CarvStatus = CarvResult<std::monostate>;

/// Content of a loaded file that this version does not model: unknown JSON fields (top level,
/// manifest, job, and per sheet / layer / object), creation parameters of unknown kinds, and
/// unknown archive members. Pass it back to `saveCarv` to write them out again. Copies are cheap
/// (shared, immutable).
class CarvExtras {
   public:
    /// Opaque storage (defined in the implementation).
    struct Impl;

    CarvExtras() = default;
    explicit CarvExtras(std::shared_ptr<const Impl> impl) : impl_(std::move(impl)) {}

    /// True when nothing unknown was found.
    bool empty() const;
    /// Names of preserved archive members, sorted.
    std::vector<std::string> memberNames() const;
    /// Bytes of the preserved archive member `name`; null when there is none.
    const std::vector<std::uint8_t>* member(std::string_view name) const;
    /// A copy holding everything this one holds, with member `name` set to `bytes` (added, or
    /// replacing a member of that name). `saveCarv` writes it as given and refuses to save when
    /// the result would not load (e.g. a `.json` member that is not well-formed JSON).
    /// Throws `std::invalid_argument` for a name `saveCarv` should not write: empty,
    /// `manifest.json` or `job.json`, a toolpath move cache name (`toolpaths/NNNN.bin`), an
    /// interim toolpath member name (`toolpaths/NNNN.json`, ADR-0019; toolpaths are records now),
    /// absolute, with a backslash or colon, a `.`/`..` segment or an empty segment, or longer than
    /// 255 bytes.
    CarvExtras withMember(std::string name, std::vector<std::uint8_t> bytes) const;
    /// Implementation access; null when empty.
    const Impl* impl() const { return impl_.get(); }

   private:
    std::shared_ptr<const Impl> impl_;
};

/// A successfully loaded file.
struct LoadedCarv {
    Document document;
    /// Unknown content to hand back to `saveCarv`.
    CarvExtras extras;
    /// Format version the file declared.
    int formatMajor{0};
    int formatMinor{0};
    /// Version string of the application that wrote the file.
    std::string appVersion;
};

/// Serialises `doc` (plus preserved `extras`) to `.carv` archive bytes. Selection and undo
/// history are not saved. Never returns bytes `loadCarv` would refuse: the finished archive is
/// loaded once in memory with the loader's own limits, and any failure is returned with the
/// loader's message ("the file would not load again: ..."), with the loader's error code. This
/// costs about one load (a 2,000,000-move toolpath: ~0.2 s Release); the move caches are only
/// validated, not decoded into a second copy (T-0136). Common cases are refused earlier with
/// specific messages: more toolpath
/// moves than `kCarvMaxToolpathMoves` or than `kCarvMovesPerFileByte` x the file's size, a
/// computed toolpath past `kCarvMaxCachedToolpaths`, more than 10,000 archive members, member or
/// archive size caps (ResourceLimit), non-finite move values or toolpath parameters
/// (InvalidContent); an over-long name is reported as "name too long" with its field. A file that loads
/// may still be refused on re-save when re-saving makes it smaller (stale caches dropped,
/// canonical re-encoding) so that it no longer satisfies the move-per-byte rule.
CarvResult<std::vector<std::uint8_t>> saveCarv(const Document& doc, const CarvExtras& extras = {});

/// Writes `doc` to `path` (via a temporary file renamed into place).
CarvStatus saveCarv(const Document& doc, const std::filesystem::path& path, const CarvExtras& extras = {});

/// Parses archive bytes. Never throws for bad input: any defect gives a `CarvError`. The loaded
/// document has an empty, clean undo stack and an empty selection. JSON members are streamed
/// through a strict parser under an archive-wide resource budget, and zip members are inflated
/// by a bounded loop that cannot hang on damaged or size-lying data (ADR-0013 §8–9).
CarvResult<LoadedCarv> loadCarv(std::span<const std::uint8_t> bytes);

/// Reads and parses the file at `path`.
CarvResult<LoadedCarv> loadCarv(const std::filesystem::path& path);

}  // namespace carvetto::document
