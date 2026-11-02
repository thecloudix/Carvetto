#pragma once

#include <string>
#include <string_view>

/// Internal: DXF string decoding and encoding (ADR-0015).
namespace carvetto::io::detail {

/// Appends the UTF-8 encoding of `cp` (invalid code points become U+FFFD).
void appendUtf8(std::string& out, char32_t cp);

/// Converts a raw DXF string to UTF-8. `utf8` is true for R2007+ files (invalid sequences become
/// U+FFFD); otherwise bytes >= 0x80 are read as Windows-1252. "\U+XXXX" escapes are decoded.
std::string decodeDxfString(std::string_view raw, bool utf8);

/// Replaces TEXT control codes: %%d, %%p, %%c (degree, plus-minus, diameter), %%% and %%nnn;
/// the underline/overline/strike toggles %%u, %%o, %%k are removed. Input and output are UTF-8.
std::string decodeControlCodes(std::string_view text);

/// Encodes UTF-8 for an R2000 file: ASCII is kept, other code points become "\U+XXXX"
/// (with surrogate pairs above U+FFFF), and control characters become '_'.
std::string encodeDxfString(std::string_view utf8);

}  // namespace carvetto::io::detail
