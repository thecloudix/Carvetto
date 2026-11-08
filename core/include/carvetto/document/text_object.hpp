#pragma once

#include <cstddef>
#include <string>
#include <vector>

#include "carvetto/document/object.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/text/layout.hpp"

/// Text objects (T-0211, ADR-0033 amendment A1): turning `TextParams` into contours.
///
/// The contours of a text object are a *cached derived view*: `buildTextContours` computes them from
/// the parameters and the embedded font, commands store the result in the object's `VectorObject`
/// next to the parameters, and the loader trusts what the file holds (it never re-lays out), so
/// opening a file does not depend on FreeType/HarfBuzz output and save -> load -> save is byte-stable.
/// Editing the parameters recomputes the contours; converting to curves drops the parameters.
namespace carvetto::document {

/// Most bytes of `TextParams::text` the object accepts.
inline constexpr std::size_t kMaxTextObjectBytes = 256u * 1024u;
/// Most words that box wrapping measures (each is shaped once).
inline constexpr std::size_t kMaxBoxWords = 4000;

/// Result of laying out a text object.
struct TextBuild {
    text::TextStatus status{text::TextStatus::Ok};
    std::string message;  ///< Non-empty for a refusal.
    std::vector<geometry::Contour> contours;
    bool overflow{false};   ///< Box mode: a word is wider than the box or the text is taller than it.
    bool truncated{false};  ///< Curve mode: glyphs past the end of an open guide were dropped.
    bool missingGlyphs{false};  ///< The font has no glyph for some character (.notdef is drawn).
    bool ok() const { return status == text::TextStatus::Ok && message.empty(); }
};

/// Checks the parameters (finite, in range, text size, box dimensions, guide for curve mode).
/// Empty when fine, otherwise the problem.
std::string textParamsProblem(const TextParams& params);

/// Lays out `params` with `font` and returns the contours already mapped by `params.placement`.
/// Point mode: `layoutText` about the local origin. Box mode: greedy word wrap to `boxWidthMm`
/// (word widths from shaping each word alone, so kerning across a space is ignored when deciding
/// breaks), first baseline one ascender below the box top, alignment about the box edges. Curve mode:
/// `layoutTextOnPath` along the guide. Refusals return a status/message and no contours. A mirroring
/// placement flips contour orientation; the contours are reversed back so outer contours stay
/// counter-clockwise.
TextBuild buildTextContours(const text::Font& font, const TextParams& params,
                            const text::TextLimits& limits = {});

}  // namespace carvetto::document
