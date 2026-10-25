#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <span>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/document/document.hpp"

/// Native `.carv` file I/O (ADR-0003, details in ADR-0013).
///
/// A `.carv` file is a zip archive holding `manifest.json` (format and app version) and
/// `job.json` (job, sheets, layers in stacking order, objects in id order with their contours and
/// creation parameters, group links, document variables, active layer/sheet and the id generator's
/// high-water mark). Output is deterministic: sorted keys, fixed member order, shortest round-trip
/// number formatting and zeroed zip timestamps, so save -> load -> save reproduces the archive byte
/// for byte. Fields and archive members this version does not understand are kept in `CarvExtras`
/// and written back on the next save.
namespace carvetto::document {

/// Format version written by this build (`manifest.json` -> `format_version`).
inline constexpr int kCarvFormatMajor = 1;
/// Minor format version written by this build. Minor bumps only add optional content.
inline constexpr int kCarvFormatMinor = 0;

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
/// history are not saved.
CarvResult<std::vector<std::uint8_t>> saveCarv(const Document& doc, const CarvExtras& extras = {});

/// Writes `doc` to `path` (via a temporary file renamed into place).
CarvStatus saveCarv(const Document& doc, const std::filesystem::path& path, const CarvExtras& extras = {});

/// Parses archive bytes. Never throws for bad input: any defect gives a `CarvError`. The loaded
/// document has an empty, clean undo stack and an empty selection. JSON members are streamed
/// through a strict parser under an archive-wide resource budget (ADR-0013 §8–9).
///
/// Known limitations (tracked as T-0015; the zip layer is not yet safe for untrusted files):
/// 1. miniz 3.0.2's `mz_zip_reader_extract_iter_read` never decrements `comp_remaining` for
///    in-memory archives, so a truncated, corrupted or size-lying deflate stream makes the
///    inflater report "needs more input" without progress and the read loop spins forever.
///    A hostile or damaged file can therefore hang `loadCarv` instead of failing.
/// 2. Central-directory entries that point at the same (or overlapping) local header are not
///    detected, so one compressed stream can be inflated once per entry (observed: 60 entries
///    sharing a 1.8 MB stream of 16 MiB each load at ~980 MiB RSS). Unknown `.json` members
///    (any member ending in `.json` other than manifest.json/job.json) are also buffered whole,
///    up to the 256 MiB per-member cap, rather than streamed.
CarvResult<LoadedCarv> loadCarv(std::span<const std::uint8_t> bytes);

/// Reads and parses the file at `path`.
CarvResult<LoadedCarv> loadCarv(const std::filesystem::path& path);

}  // namespace carvetto::document
