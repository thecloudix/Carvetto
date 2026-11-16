#include "carvetto/io/png_guard.hpp"

#include <miniz.h>

#include <algorithm>
#include <array>
#include <initializer_list>
#include <cstddef>
#include <limits>
#include <vector>

namespace carvetto::io {

namespace {

constexpr std::uint64_t kSaturated = std::numeric_limits<std::uint64_t>::max();

std::uint64_t mulSat(std::uint64_t a, std::uint64_t b) {
    if (a != 0 && b > kSaturated / a) {
        return kSaturated;
    }
    return a * b;
}

std::uint64_t addSat(std::uint64_t a, std::uint64_t b) { return a > kSaturated - b ? kSaturated : a + b; }

std::uint32_t be32(const std::uint8_t* p) {
    return (static_cast<std::uint32_t>(p[0]) << 24) | (static_cast<std::uint32_t>(p[1]) << 16) |
           (static_cast<std::uint32_t>(p[2]) << 8) | static_cast<std::uint32_t>(p[3]);
}

int channelsOf(int colourType, int bitDepth) {
    const auto depthIn = [&](std::initializer_list<int> ok) { return std::find(ok.begin(), ok.end(), bitDepth) != ok.end(); };
    switch (colourType) {
        case 0:
            return depthIn({1, 2, 4, 8, 16}) ? 1 : 0;
        case 2:
            return depthIn({8, 16}) ? 3 : 0;
        case 3:
            return depthIn({1, 2, 4, 8}) ? 1 : 0;
        case 4:
            return depthIn({8, 16}) ? 2 : 0;
        case 6:
            return depthIn({8, 16}) ? 4 : 0;
        default:
            return 0;
    }
}

std::uint64_t imageBytes(std::uint64_t w, std::uint64_t h, std::uint64_t bitsPerPixel) {
    if (w == 0 || h == 0) {
        return 0;
    }
    const std::uint64_t row = addSat(mulSat(w, bitsPerPixel), 7) / 8;
    return mulSat(h, addSat(row, 1));
}

}  // namespace

std::uint64_t pngExpectedRawBytes(std::uint32_t width, std::uint32_t height, int bitDepth, int colourType,
                                  int interlace) {
    const int channels = channelsOf(colourType, bitDepth);
    if (channels == 0 || (interlace != 0 && interlace != 1)) {
        return 0;
    }
    const std::uint64_t bpp = static_cast<std::uint64_t>(channels) * static_cast<std::uint64_t>(bitDepth);
    if (interlace == 0) {
        return imageBytes(width, height, bpp);
    }
    struct Pass {
        std::uint32_t x0, y0, dx, dy;
    };
    static constexpr std::array<Pass, 7> kPasses = {{{0, 0, 8, 8}, {4, 0, 8, 8}, {0, 4, 4, 8}, {2, 0, 4, 4},
                                                     {0, 2, 2, 4}, {1, 0, 2, 2}, {0, 1, 1, 2}}};
    std::uint64_t total = 0;
    for (const Pass& p : kPasses) {
        const std::uint64_t pw = width > p.x0 ? (static_cast<std::uint64_t>(width) - p.x0 + p.dx - 1) / p.dx : 0;
        const std::uint64_t ph = height > p.y0 ? (static_cast<std::uint64_t>(height) - p.y0 + p.dy - 1) / p.dy : 0;
        total = addSat(total, imageBytes(pw, ph, bpp));
    }
    return total;
}

PngGuardResult checkPngPixelData(std::span<const std::uint8_t> png, std::uint64_t maxExpectedBytes) {
    PngGuardResult out;
    static constexpr std::array<std::uint8_t, 8> kSignature = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    if (png.size() < 33 || !std::equal(kSignature.begin(), kSignature.end(), png.begin()) || be32(&png[8]) != 13 ||
        !(png[12] == 'I' && png[13] == 'H' && png[14] == 'D' && png[15] == 'R')) {
        return out;
    }
    out.expectedBytes = pngExpectedRawBytes(be32(&png[16]), be32(&png[20]), png[24], png[25], png[28]);
    if (out.expectedBytes == 0) {
        return out;  // BadHeader: zero size or an invalid combination
    }
    if (out.expectedBytes > maxExpectedBytes) {
        out.status = PngGuardStatus::TooLarge;
        return out;
    }
    out.status = PngGuardStatus::BadData;

    // Concatenate the IDAT payloads (the zlib stream spans them).
    std::vector<std::uint8_t> idat;
    std::size_t pos = 8;
    bool ended = false;
    while (!ended && pos + 12 <= png.size()) {
        const std::uint64_t len = be32(&png[pos]);
        const std::uint8_t* type = &png[pos + 4];
        if (len > png.size() - pos - 12) {
            return out;  // chunk runs past the end of the file
        }
        if (type[0] == 'I' && type[1] == 'D' && type[2] == 'A' && type[3] == 'T') {
            idat.insert(idat.end(), png.begin() + static_cast<std::ptrdiff_t>(pos + 8),
                        png.begin() + static_cast<std::ptrdiff_t>(pos + 8 + len));
        } else if (type[0] == 'I' && type[1] == 'E' && type[2] == 'N' && type[3] == 'D') {
            ended = true;
        }
        pos += 12 + static_cast<std::size_t>(len);
    }
    if (idat.empty()) {
        return out;
    }

    const std::uint64_t budget = addSat(out.expectedBytes, kPngInflateSlackBytes);
    std::vector<std::uint8_t> dict(TINFL_LZ_DICT_SIZE);
    tinfl_decompressor inflator;
    tinfl_init(&inflator);
    std::size_t inPos = 0;
    std::size_t dictWrite = 0;
    for (;;) {
        std::size_t inSize = idat.size() - inPos;
        std::size_t outSize = TINFL_LZ_DICT_SIZE - dictWrite;
        const tinfl_status status = tinfl_decompress(&inflator, idat.data() + inPos, &inSize, dict.data(),
                                                     dict.data() + dictWrite, &outSize, TINFL_FLAG_PARSE_ZLIB_HEADER);
        inPos += inSize;
        out.inflatedBytes += outSize;
        dictWrite = (dictWrite + outSize) & (TINFL_LZ_DICT_SIZE - 1);
        if (out.inflatedBytes > budget) {
            out.status = PngGuardStatus::TooMuchData;
            return out;
        }
        // The Adler-32 trailer is the decoder's business; a mismatch still means the stream ended.
        if (status == TINFL_STATUS_DONE || status == TINFL_STATUS_ADLER32_MISMATCH) {
            out.status = PngGuardStatus::Ok;
            return out;
        }
        if (status != TINFL_STATUS_HAS_MORE_OUTPUT) {
            return out;  // corrupt, truncated (needs more input) or stalled
        }
    }
}

}  // namespace carvetto::io
