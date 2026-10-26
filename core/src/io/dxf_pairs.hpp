#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>

#include "carvetto/io/dxf.hpp"

/// Internal: DXF group-code/value pair reader for ASCII and binary DXF (ADR-0015).
namespace carvetto::io::detail {

/// How a group code's value is stored (DXF reference, "Group Code Value Types").
enum class ValueType { String, Double, Int16, Int32, Int64, Bool, Binary };

/// Value type of `code`, or nullopt for codes outside the documented ranges.
std::optional<ValueType> valueTypeFor(int code);

/// Sentinel at the start of binary DXF files (22 bytes including CR LF SUB NUL).
inline constexpr std::string_view kBinarySentinel{"AutoCAD Binary DXF\r\n\x1a\0", 22};

/// One group-code/value pair. String values view the input buffer; numeric values are decoded.
struct Pair {
    int code{0};
    /// Raw text of the value (ASCII) or the string bytes (binary strings); empty for binary numbers.
    std::string_view text;
    /// Numeric value for double and integer codes (integers are also stored here).
    double real{0.0};
    /// Integer value for integer codes (truncated double for double codes).
    std::int64_t integer{0};
    /// 1-based line of the group-code line (ASCII) or byte offset of the pair (binary).
    std::size_t line{0};
};

/// Streaming pull reader over a DXF byte buffer. Comments (group 999) are skipped. The buffer
/// must outlive the reader and every `Pair` it returns.
class PairReader {
   public:
    explicit PairReader(std::span<const std::uint8_t> bytes);

    /// True for binary DXF input.
    bool binary() const { return binary_; }
    /// Reads the next pair. Returns false at the end of the data or on error (see `error()`).
    bool next(Pair& out);
    /// Like `next` but does not consume the pair.
    bool peek(Pair& out);
    /// The first error encountered, if any. Once set, `next` keeps returning false.
    const std::optional<DxfError>& error() const { return error_; }
    /// Current line (ASCII) or byte offset (binary), for messages.
    std::size_t position() const { return binary_ ? pos_ : line_; }

   private:
    bool readRaw(Pair& out);
    bool readAscii(Pair& out);
    bool readBinary(Pair& out);
    bool readLine(std::string_view& line);
    bool fail(DxfError::Code code, std::string message, std::size_t line);

    std::string_view data_;
    std::size_t pos_{0};
    std::size_t line_{0};
    bool binary_{false};
    bool wideCodes_{false};
    std::optional<Pair> peeked_;
    std::optional<DxfError> error_;
};

/// Trims ASCII spaces and tabs from both ends.
std::string_view trim(std::string_view s);
/// Case-insensitive (ASCII) comparison.
bool iequals(std::string_view a, std::string_view b);
/// ASCII upper-case copy (used as a case-insensitive map key).
std::string upper(std::string_view s);
/// Printable, length-limited rendering of untrusted text for messages.
std::string printable(std::string_view s, std::size_t maxLength = 40);

}  // namespace carvetto::io::detail
