// PNG decompression-bomb guard (T-0333, ADR-0109).

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "carvetto/io/png_guard.hpp"

using namespace carvetto::io;

namespace {

void put32(std::vector<std::uint8_t>& v, std::uint32_t x) {
    v.push_back(static_cast<std::uint8_t>(x >> 24));
    v.push_back(static_cast<std::uint8_t>(x >> 16));
    v.push_back(static_cast<std::uint8_t>(x >> 8));
    v.push_back(static_cast<std::uint8_t>(x));
}

void chunk(std::vector<std::uint8_t>& v, const std::string& type, const std::vector<std::uint8_t>& data) {
    put32(v, static_cast<std::uint32_t>(data.size()));
    v.insert(v.end(), type.begin(), type.end());
    v.insert(v.end(), data.begin(), data.end());
    put32(v, 0);  // CRC: not checked by the guard
}

/// zlib stream of stored blocks holding `n` zero bytes.
std::vector<std::uint8_t> storedZlib(std::uint64_t n) {
    std::vector<std::uint8_t> z = {0x78, 0x01};
    do {
        const std::uint32_t len = static_cast<std::uint32_t>(n > 65535 ? 65535 : n);
        n -= len;
        z.push_back(n == 0 ? 1 : 0);
        z.push_back(static_cast<std::uint8_t>(len));
        z.push_back(static_cast<std::uint8_t>(len >> 8));
        z.push_back(static_cast<std::uint8_t>(~len));
        z.push_back(static_cast<std::uint8_t>((~len) >> 8));
        z.insert(z.end(), len, 0);
    } while (n > 0);
    put32(z, 1);  // Adler-32: not checked
    return z;
}

/// zlib stream (one fixed-Huffman block) inflating to `n` zero bytes at about 258 bytes per 13 bits.
std::vector<std::uint8_t> bombZlib(std::uint64_t n) {
    std::vector<std::uint8_t> z = {0x78, 0x01};
    std::uint32_t acc = 0;
    int bits = 0;
    const auto emit = [&](std::uint32_t value, int count, bool msbFirst) {
        for (int i = 0; i < count; ++i) {
            const std::uint32_t bit = msbFirst ? (value >> (count - 1 - i)) & 1U : (value >> i) & 1U;
            acc |= bit << bits;
            if (++bits == 8) {
                z.push_back(static_cast<std::uint8_t>(acc));
                acc = 0;
                bits = 0;
            }
        }
    };
    emit(1, 1, false);  // BFINAL
    emit(1, 2, false);  // BTYPE = fixed Huffman
    emit(0x30, 8, true);  // literal 0
    std::uint64_t left = n - 1;
    while (left >= 258) {
        emit(0xC5, 8, true);  // length 258 (symbol 285)
        emit(0, 5, true);     // distance 1
        left -= 258;
    }
    while (left > 0) {
        emit(0x30, 8, true);
        --left;
    }
    emit(0, 7, true);  // end of block
    if (bits > 0) {
        z.push_back(static_cast<std::uint8_t>(acc));
    }
    put32(z, 1);
    return z;
}

std::vector<std::uint8_t> pngWith(std::uint32_t w, std::uint32_t h, int depth, int colour, int interlace,
                                  const std::vector<std::uint8_t>& zlibData, bool splitIdat = false) {
    std::vector<std::uint8_t> v = {0x89, 'P', 'N', 'G', '\r', '\n', 0x1a, '\n'};
    std::vector<std::uint8_t> ihdr;
    put32(ihdr, w);
    put32(ihdr, h);
    ihdr.push_back(static_cast<std::uint8_t>(depth));
    ihdr.push_back(static_cast<std::uint8_t>(colour));
    ihdr.push_back(0);
    ihdr.push_back(0);
    ihdr.push_back(static_cast<std::uint8_t>(interlace));
    chunk(v, "IHDR", ihdr);
    if (splitIdat) {
        const std::size_t half = zlibData.size() / 2;
        chunk(v, "IDAT", std::vector<std::uint8_t>(zlibData.begin(), zlibData.begin() + static_cast<std::ptrdiff_t>(half)));
        chunk(v, "IDAT", std::vector<std::uint8_t>(zlibData.begin() + static_cast<std::ptrdiff_t>(half), zlibData.end()));
    } else {
        chunk(v, "IDAT", zlibData);
    }
    chunk(v, "IEND", {});
    return v;
}

constexpr std::uint64_t kBig = std::uint64_t{1} << 40;

}  // namespace

TEST_CASE("png guard: expected raw size from the header", "[png_guard]") {
    // 1 + 3 bytes per row, 2 rows
    CHECK(pngExpectedRawBytes(1, 2, 8, 2, 0) == 2 * (1 + 3));
    CHECK(pngExpectedRawBytes(10, 1, 1, 0, 0) == 1 + 2);   // 10 bits -> 2 bytes
    CHECK(pngExpectedRawBytes(8, 1, 1, 0, 0) == 1 + 1);    // 8 bits -> 1 byte
    CHECK(pngExpectedRawBytes(9, 1, 1, 0, 0) == 1 + 2);
    CHECK(pngExpectedRawBytes(3, 3, 16, 6, 0) == 3 * (1 + 24));
    CHECK(pngExpectedRawBytes(5, 4, 4, 3, 0) == 4 * (1 + 3));
    CHECK(pngExpectedRawBytes(2, 2, 8, 4, 0) == 2 * (1 + 4));
    // Adam7 1x1: only pass 1 holds the pixel.
    CHECK(pngExpectedRawBytes(1, 1, 8, 0, 1) == 2);
    // Adam7 8x8 gray 8: passes 1x1, 1x1, 2x1, 2x2, 4x2, 4x4, 8x4 -> (1+1)+(1+1)+(1+2)+2*(1+2)+2*(1+4)+4*(1+4)+4*(1+8)
    CHECK(pngExpectedRawBytes(8, 8, 8, 0, 1) == 2 + 2 + 3 + 6 + 10 + 20 + 36);
    // Invalid combinations.
    CHECK(pngExpectedRawBytes(1, 1, 8, 1, 0) == 0);
    CHECK(pngExpectedRawBytes(1, 1, 4, 2, 0) == 0);
    CHECK(pngExpectedRawBytes(1, 1, 16, 3, 0) == 0);
    CHECK(pngExpectedRawBytes(1, 1, 8, 0, 2) == 0);
    CHECK(pngExpectedRawBytes(0, 5, 8, 0, 0) == 0);
    CHECK(pngExpectedRawBytes(5, 0, 8, 0, 0) == 0);
    // Saturates instead of overflowing.
    CHECK(pngExpectedRawBytes(0xFFFFFFFFU, 0xFFFFFFFFU, 16, 6, 0) > (std::uint64_t{1} << 62));
}

TEST_CASE("png guard: multiplication overflow saturates instead of wrapping", "[png_guard][hostile]") {
    // 630784594 x 3655515735, RGBA16: the true size is about 1.8e19 bytes, which wraps to 46,656,839
    // (under any sane cap) in plain 64-bit arithmetic.
    constexpr std::uint32_t w = 630784594U;
    constexpr std::uint32_t h = 3655515735U;
    CHECK(pngExpectedRawBytes(w, h, 16, 6, 0) == std::numeric_limits<std::uint64_t>::max());
    const PngGuardResult r = checkPngPixelData(pngWith(w, h, 16, 6, 0, storedZlib(16)), 80ULL * 1024 * 1024);
    CHECK(r.status == PngGuardStatus::TooLarge);
    CHECK(r.expectedBytes == std::numeric_limits<std::uint64_t>::max());
}

TEST_CASE("png guard: budget addition saturates instead of wrapping", "[png_guard][hostile]") {
    // Expected size saturates at UINT64_MAX and the caller allows it; expected + slack must saturate
    // too, otherwise the budget wraps to 4095 and a 5000-byte stream is wrongly called a bomb.
    const PngGuardResult r = checkPngPixelData(pngWith(0xFFFFFFFFU, 0xFFFFFFFFU, 16, 6, 0, storedZlib(5000)),
                                               std::numeric_limits<std::uint64_t>::max());
    CHECK(r.expectedBytes == std::numeric_limits<std::uint64_t>::max());
    CHECK(r.status == PngGuardStatus::Ok);
    CHECK(r.inflatedBytes == 5000);
}

TEST_CASE("png guard: exact budget boundary", "[png_guard]") {
    // 1x1 RGB8: expected 4 bytes, budget 4 + slack.
    const std::uint64_t budget = 4 + kPngInflateSlackBytes;
    for (const bool stored : {true, false}) {
        const auto make = [&](std::uint64_t n) { return stored ? storedZlib(n) : bombZlib(n); };
        PngGuardResult r = checkPngPixelData(pngWith(1, 1, 8, 2, 0, make(4)), kBig);
        CHECK(r.status == PngGuardStatus::Ok);
        CHECK(r.expectedBytes == 4);
        CHECK(r.inflatedBytes == 4);
        r = checkPngPixelData(pngWith(1, 1, 8, 2, 0, make(budget)), kBig);
        CHECK(r.status == PngGuardStatus::Ok);
        CHECK(r.inflatedBytes == budget);
        r = checkPngPixelData(pngWith(1, 1, 8, 2, 0, make(budget + 1)), kBig);
        CHECK(r.status == PngGuardStatus::TooMuchData);
        CHECK(r.inflatedBytes == budget + 1);
    }
}

TEST_CASE("png guard: a bomb stops after the budget, not after the whole stream", "[png_guard][hostile]") {
    // Header 1x1; the stream inflates to 64 MiB.
    const auto z = bombZlib(std::uint64_t{64} << 20);
    CHECK(z.size() < 512 * 1024);  // 64 MiB from under half a MiB (fixed Huffman codes: 13 bits per 258 bytes)
    const PngGuardResult r = checkPngPixelData(pngWith(1, 1, 8, 2, 0, z), kBig);
    CHECK(r.status == PngGuardStatus::TooMuchData);
    CHECK(r.inflatedBytes > 4 + kPngInflateSlackBytes);
    CHECK(r.inflatedBytes <= 4 + kPngInflateSlackBytes + 32768 + 258);
}

TEST_CASE("png guard: IDAT split over chunks, interlaced, header refusals", "[png_guard]") {
    CHECK(checkPngPixelData(pngWith(10, 10, 8, 0, 0, storedZlib(10 * 11), true), kBig).status == PngGuardStatus::Ok);
    CHECK(checkPngPixelData(pngWith(8, 8, 8, 0, 1, storedZlib(2 + 2 + 3 + 6 + 10 + 20 + 36)), kBig).status ==
          PngGuardStatus::Ok);
    // Header says 8x8 non-interlaced (72 bytes) but the data is 4 KiB + 1 over.
    CHECK(checkPngPixelData(pngWith(8, 8, 8, 0, 0, storedZlib(72 + kPngInflateSlackBytes + 1)), kBig).status ==
          PngGuardStatus::TooMuchData);
    // Raw size over the caller's cap: refused before inflating (inflatedBytes stays 0).
    PngGuardResult r = checkPngPixelData(pngWith(10, 10, 8, 0, 0, storedZlib(110)), 109);
    CHECK(r.status == PngGuardStatus::TooLarge);
    CHECK(r.inflatedBytes == 0);
    CHECK(checkPngPixelData(pngWith(10, 10, 8, 0, 0, storedZlib(110)), 110).status == PngGuardStatus::Ok);
    // Bad headers.
    CHECK(checkPngPixelData(pngWith(10, 10, 8, 1, 0, storedZlib(110)), kBig).status == PngGuardStatus::BadHeader);
    CHECK(checkPngPixelData(pngWith(0, 10, 8, 0, 0, storedZlib(110)), kBig).status == PngGuardStatus::BadHeader);
    CHECK(checkPngPixelData(std::vector<std::uint8_t>{1, 2, 3}, kBig).status == PngGuardStatus::BadHeader);
    std::vector<std::uint8_t> noIhdr = pngWith(1, 1, 8, 0, 0, storedZlib(2));
    noIhdr[12] = 'X';
    CHECK(checkPngPixelData(noIhdr, kBig).status == PngGuardStatus::BadHeader);
}

TEST_CASE("png guard: damaged data is refused, never crashes", "[png_guard][hostile]") {
    const auto good = pngWith(10, 10, 8, 0, 0, storedZlib(110));
    // Truncated file (chunk runs past the end).
    for (std::size_t cut : {std::size_t{33}, std::size_t{40}, good.size() / 2, good.size() - 13}) {
        std::vector<std::uint8_t> t(good.begin(), good.begin() + static_cast<std::ptrdiff_t>(cut));
        CHECK(checkPngPixelData(t, kBig).status == PngGuardStatus::BadData);
    }
    // No IDAT at all.
    std::vector<std::uint8_t> none = pngWith(10, 10, 8, 0, 0, storedZlib(110));
    none[37] = 'X';  // IDAT type byte 0 -> unknown chunk
    CHECK(checkPngPixelData(none, kBig).status == PngGuardStatus::BadData);
    // Garbage zlib stream.
    CHECK(checkPngPixelData(pngWith(10, 10, 8, 0, 0, std::vector<std::uint8_t>(50, 0xFF)), kBig).status ==
          PngGuardStatus::BadData);
    // Valid zlib but too short for the header: truncated stream.
    auto shortStream = storedZlib(110);
    shortStream.resize(shortStream.size() - 40);
    CHECK(checkPngPixelData(pngWith(10, 10, 8, 0, 0, shortStream), kBig).status == PngGuardStatus::BadData);
}
