#pragma once

// Internal: streaming JSON reading for `.carv` loading (ADR-0013 §9). Untrusted JSON is never
// turned into a DOM: `JsonReader` is a pull parser over a chunked `CharSource` (a zip member
// decompressed on the fly), the loader decodes straight into Carvetto records, and unknown values
// are re-written as compact canonical text. Every allocation the parse can cause is charged to
// one `LoadBudget` shared by all members of an archive. Tests include this header directly.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/document/carv_io.hpp"

namespace carvetto::document::detail {

/// A load failure, thrown inside the loader and turned into a `CarvError` at the API boundary.
struct LoadFailure {
    CarvError::Code code;
    std::string message;
};

/// Absolute limits for one archive load. The defaults are documented (with measurements) in
/// ADR-0013 §9.
struct LoadLimits {
    /// JSON values (scalars, containers and keys) across all members.
    std::uint64_t maxValues{24'000'000};
    /// Estimated bytes of decoded records (objects, contours, spans, names, ...).
    std::uint64_t maxDocumentBytes{std::uint64_t{192} << 20};
    /// Bytes *held* after load for preserved archive members and extras: the greater of this
    /// floor and `heldBytesPerArchiveByte` x the archive's size (a 1.8 MB file may keep 64 MiB).
    std::uint64_t minHeldBytes{std::uint64_t{64} << 20};
    std::uint64_t heldBytesPerArchiveByte{32};
    /// JSON values inside unknown fields (kept or skipped), across all members.
    std::uint64_t maxUnknownValues{2'000'000};
    /// Bytes of canonical text kept (or transiently built) for unknown fields.
    std::uint64_t maxExtrasBytes{std::uint64_t{16} << 20};
    /// Longest number token.
    std::size_t maxNumberChars{64};
    /// Longest string for a known field or a key.
    std::size_t maxStringBytes{std::size_t{1} << 20};
    /// Longest string inside an unknown field (also counted against `maxExtrasBytes`).
    std::size_t maxUnknownStringBytes{std::size_t{16} << 20};
    /// Most elements (or members) in one array/object of an unknown field.
    std::uint64_t maxUnknownContainerElements{1'000'000};
    /// Deepest array/object nesting.
    int maxDepth{64};
};

/// Global resource accounting for one load; throws `LoadFailure` (ResourceLimit) when exceeded.
class LoadBudget {
   public:
    explicit LoadBudget(LoadLimits limits = {}) : limits_(limits) {}

    const LoadLimits& limits() const { return limits_; }

    void chargeValue() {
        if (++values_ > limits_.maxValues) {
            exceeded("more than " + std::to_string(limits_.maxValues) + " JSON values");
        }
    }
    void chargeUnknownValue() {
        if (++unknownValues_ > limits_.maxUnknownValues) {
            exceeded("more than " + std::to_string(limits_.maxUnknownValues) +
                     " JSON values in unknown fields");
        }
    }
    void chargeDocument(std::uint64_t bytes) {
        document_ += bytes;
        if (document_ > limits_.maxDocumentBytes) {
            exceeded("the decoded document would need more than " +
                     std::to_string(limits_.maxDocumentBytes >> 20) + " MiB");
        }
    }
    void chargeExtras(std::uint64_t bytes) {
        extras_ += bytes;
        if (extras_ > limits_.maxExtrasBytes) {
            exceeded("unknown fields would need more than " + std::to_string(limits_.maxExtrasBytes >> 20) +
                     " MiB");
        }
    }
    void refundExtras(std::uint64_t bytes) { extras_ = bytes > extras_ ? 0 : extras_ - bytes; }
    void refundUnknownValues(std::uint64_t n) {
        unknownValues_ = n > unknownValues_ ? 0 : unknownValues_ - n;
    }

    std::uint64_t values() const { return values_; }
    std::uint64_t unknownValues() const { return unknownValues_; }
    std::uint64_t documentBytes() const { return document_; }
    std::uint64_t extrasBytes() const { return extras_; }

   private:
    [[noreturn]] static void exceeded(const std::string& what) {
        throw LoadFailure{CarvError::Code::ResourceLimit, "load limit exceeded: " + what};
    }

    LoadLimits limits_;
    std::uint64_t values_{0};
    std::uint64_t unknownValues_{0};
    std::uint64_t document_{0};
    std::uint64_t extras_{0};
};

/// A sequential byte source.
class CharSource {
   public:
    virtual ~CharSource() = default;
    /// Copies up to `n` bytes into `buf`; returns 0 at the end.
    virtual std::size_t read(char* buf, std::size_t n) = 0;
};

/// A `CharSource` over text in memory.
class StringSource : public CharSource {
   public:
    explicit StringSource(std::string_view text) : text_(text) {}
    std::size_t read(char* buf, std::size_t n) override;

   private:
    std::string_view text_;
    std::size_t pos_{0};
};

/// Streaming pull parser (RFC 8259, strict: no comments, no trailing commas, UTF-8 validated,
/// no duplicate keys inside canonicalised values). Syntax errors throw `LoadFailure`
/// (InvalidJson, naming the member and byte offset); limit breaches throw ResourceLimit.
class JsonReader {
   public:
    enum class Kind { Object, Array, String, Number, Bool, Null };

    /// `bufferBytes` is the read-ahead buffer (64 KiB for zip members; small for in-memory text).
    JsonReader(CharSource& source, LoadBudget& budget, std::string member,
               std::size_t bufferBytes = std::size_t{1} << 16);

    /// Kind of the next value (skips whitespace, consumes nothing).
    Kind peek();
    /// Consumes '{'. `unknown` applies the element cap for unknown containers.
    void beginObject(bool unknown = false);
    /// Advances to the next member of the innermost object: reads its key and the ':'. Returns
    /// false after consuming the closing '}'.
    bool nextKey(std::string& key);
    /// Consumes '['.
    void beginArray(bool unknown = false);
    /// Advances to the next element of the innermost array; false after consuming ']'.
    bool nextElement();
    /// Reads a string value (UTF-8, escapes decoded).
    std::string readString(bool unknown = false);
    /// Reads a number token (grammar-checked, at most `maxNumberChars`); valid until the next call.
    std::string_view readNumber();
    bool readBool();
    void readNull();
    /// Reads the next value and appends its canonical compact text to `*out` (charged to the
    /// extras budget); with `out == nullptr` the value is only validated and skipped.
    void readCanonical(std::string* out);
    /// Requires the end of input after the top-level value.
    void finish();

    LoadBudget& budget() { return budget_; }
    const std::string& member() const { return member_; }
    [[noreturn]] void syntaxError(const std::string& what) const;

   private:
    struct Level {
        bool object;
        bool first;
        bool unknown;
        std::uint64_t count;
    };

    int peekChar() {
        if (pos_ == len_ && !refill()) {
            return -1;
        }
        return static_cast<unsigned char>(data_[pos_]);
    }
    bool refill();
    void skipWs();
    void expect(char c);
    void expectLiteral(std::string_view word);
    void push(bool object, bool unknown);
    void countElement();
    std::string readStringRaw(std::size_t limit, bool unknown);

    CharSource& source_;
    LoadBudget& budget_;
    std::string member_;
    std::unique_ptr<char[]> buf_;  // uninitialised: only [0, len_) is ever read
    char* data_;                   // buf_.get(), cached (Debug builds do not inline accessors)
    std::size_t capacity_;
    std::size_t pos_{0};
    std::size_t len_{0};
    std::uint64_t consumed_{0};  // bytes before buf_[0]
    std::vector<Level> levels_;
    std::string number_;
};

/// Appends `s` as a JSON string literal (escaping like nlohmann::json::dump; invalid UTF-8 is
/// replaced by U+FFFD).
void appendJsonString(std::string& out, std::string_view s);
/// Appends the shortest round-trip text of a double (always with '.', exponent, or as one of
/// the strings "NaN", "Infinity", "-Infinity" when not finite).
void appendDouble(std::string& out, double v);
/// Appends the canonical text of a grammar-checked number token: integers that fit 64 bits as
/// integers, everything else as `appendDouble`. Throws (InvalidContent) when out of range.
void appendCanonicalNumber(std::string& out, std::string_view token, const std::string& member);
/// True when `s` is well-formed UTF-8.
bool isValidUtf8(std::string_view s);

/// Canonical compact text of a complete JSON text, under `budget` (used by tests).
std::string canonicalJson(std::string_view text, LoadBudget& budget);

struct ZipLimits;
/// `loadCarv` with explicit limits (used by tests to exercise each cap with small fixtures).
CarvResult<LoadedCarv> loadCarvWithLimits(std::span<const std::uint8_t> bytes, const LoadLimits& limits,
                                          const ZipLimits& zipLimits);

}  // namespace carvetto::document::detail
