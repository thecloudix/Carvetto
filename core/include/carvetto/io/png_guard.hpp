#pragma once

#include <cstdint>
#include <span>
#include <string>

// Qt-free guard against PNG decompression bombs (T-0333, ADR-0109). A PNG's header says how many
// raw bytes its pixel data must inflate to; a decoder that inflates the whole IDAT stream first can
// be kept busy for seconds by a tiny header over a stream that inflates to gigabytes. The guard
// inflates the stream with an output budget of the declared size (+ a small slack) and refuses the
// file as soon as the budget is exceeded, so the work spent is bounded by the declared size.

namespace carvetto::io {

/// Extra inflated bytes tolerated beyond the size the header declares (some encoders pad).
inline constexpr std::uint64_t kPngInflateSlackBytes = 4096;

enum class PngGuardStatus {
    Ok,           ///< The pixel data inflates to no more than the header declares (+ slack).
    BadHeader,    ///< Not a PNG, or the IHDR is missing / invalid (colour type, depth, interlace).
    BadData,      ///< Chunk structure or the zlib stream is damaged, truncated or has no IDAT.
    TooLarge,     ///< The header declares more raw bytes than the caller allows.
    TooMuchData,  ///< The pixel data inflates to more than the header declares (a bomb).
};

struct PngGuardResult {
    PngGuardStatus status{PngGuardStatus::BadHeader};
    /// Raw (inflated) bytes the header declares: scanlines with their filter byte, every Adam7 pass.
    std::uint64_t expectedBytes{0};
    /// Bytes actually inflated before the guard stopped (never above expected + slack + 32 KiB).
    std::uint64_t inflatedBytes{0};
};

/// Raw byte count of the pixel data for the given header values; nullopt-like 0 when the colour
/// type / bit depth / interlace combination is invalid. Saturates at UINT64_MAX.
std::uint64_t pngExpectedRawBytes(std::uint32_t width, std::uint32_t height, int bitDepth, int colourType,
                                  int interlace);

/// Walks the chunks (CRCs are not checked; the decoder does), concatenates the IDAT payloads and
/// inflates them with the budget described above. `maxExpectedBytes` refuses headers that declare
/// more raw data than that before any inflating.
PngGuardResult checkPngPixelData(std::span<const std::uint8_t> png, std::uint64_t maxExpectedBytes);

}  // namespace carvetto::io
