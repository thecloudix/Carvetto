#include "carv_relief.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <limits>

namespace carvetto::document::detail {

namespace {

constexpr std::array<char, 4> kMagic{'C', 'V', 'H', 'F'};
constexpr std::uint32_t kCanonicalNan = 0x7FC00000U;
constexpr std::size_t kChunk = 64 * 1024;

void putU32(std::vector<std::uint8_t>& out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        out.push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xFFU));
    }
}

void putF64(std::vector<std::uint8_t>& out, double d) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &d, sizeof bits);
    for (int i = 0; i < 8; ++i) {
        out.push_back(static_cast<std::uint8_t>((bits >> (8 * i)) & 0xFFU));
    }
}

std::uint32_t getU32(const std::uint8_t* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

double getF64(const std::uint8_t* p) {
    std::uint64_t bits = 0;
    for (int i = 0; i < 8; ++i) {
        bits |= static_cast<std::uint64_t>(p[i]) << (8 * i);
    }
    double d = 0.0;
    std::memcpy(&d, &bits, sizeof d);
    return d;
}

bool readExact(ZipArchive::Stream& s, std::uint8_t* buf, std::size_t n) {
    std::size_t got = 0;
    while (got < n) {
        const std::size_t r = s.readBytes(reinterpret_cast<char*>(buf) + got, n - got);
        if (r == 0) {
            return false;
        }
        got += r;
    }
    return true;
}

std::uint32_t bitsOf(const float& f) {
    std::uint32_t b = 0;
    std::memcpy(&b, &f, sizeof b);
    return b;
}

}  // namespace

std::string reliefFieldMemberName(std::uint64_t componentId) {
    return "components/" + std::to_string(componentId) + ".hf";
}

std::string encodeHeightField(const relief::HeightField& field, std::vector<std::uint8_t>& out) {
    if (field.empty()) {
        return "the height field is empty";
    }
    if (field.width() > std::numeric_limits<std::uint32_t>::max() ||
        field.height() > std::numeric_limits<std::uint32_t>::max()) {
        return "the height field is too large";
    }
    const std::vector<float>& d = field.data();
    for (const float v : d) {
        if (std::isinf(v)) {
            return "the height field holds an infinite value";
        }
    }
    out.reserve(out.size() + kHeightFieldHeaderBytes + d.size() * 4);
    out.insert(out.end(), kMagic.begin(), kMagic.end());
    putU32(out, kHeightFieldVersion);
    putU32(out, static_cast<std::uint32_t>(field.width()));
    putU32(out, static_cast<std::uint32_t>(field.height()));
    putF64(out, field.cellSize());
    putF64(out, field.origin().x);
    putF64(out, field.origin().y);
    for (int plane = 0; plane < 4; ++plane) {
        const int shift = 8 * plane;
        for (const float v : d) {
            const std::uint32_t bits = std::isnan(v) ? kCanonicalNan : bitsOf(v);
            out.push_back(static_cast<std::uint8_t>((bits >> shift) & 0xFFU));
        }
    }
    return {};
}

std::string decodeHeightField(ZipArchive::Stream& stream, std::uint64_t declaredSize, relief::HeightField& out) {
    out = relief::HeightField{};
    if (declaredSize < kHeightFieldHeaderBytes) {
        return "the member is too short for a height-field header";
    }
    std::uint8_t header[kHeightFieldHeaderBytes];
    if (!readExact(stream, header, sizeof header)) {
        return "the member ends inside the header";
    }
    if (std::memcmp(header, kMagic.data(), kMagic.size()) != 0) {
        return "not a height field (bad signature)";
    }
    if (getU32(header + 4) != kHeightFieldVersion) {
        return "unsupported height-field version " + std::to_string(getU32(header + 4));
    }
    relief::Grid grid;
    const std::uint64_t w = getU32(header + 8);
    const std::uint64_t h = getU32(header + 12);
    grid.cellSize = getF64(header + 16);
    grid.origin = {getF64(header + 24), getF64(header + 32)};
    if (w == 0 || h == 0 || w > relief::kMaxCells || h > relief::kMaxCells || w * h > relief::kMaxCells) {
        return "the height field is empty or has more than " + std::to_string(relief::kMaxCells) + " cells";
    }
    grid.width = static_cast<std::size_t>(w);
    grid.height = static_cast<std::size_t>(h);
    if (!std::isfinite(grid.cellSize) || !(grid.cellSize > 0.0) || !std::isfinite(grid.origin.x) ||
        !std::isfinite(grid.origin.y)) {
        return "the height-field cell size or origin is not a finite, positive geometry";
    }
    const std::uint64_t cells = w * h;
    if (declaredSize != kHeightFieldHeaderBytes + cells * 4) {
        return "the member is " + std::to_string(declaredSize) + " bytes but a " + std::to_string(w) + "x" +
               std::to_string(h) + " field takes " + std::to_string(kHeightFieldHeaderBytes + cells * 4);
    }
    // Allocation is bounded: cells <= kMaxCells and the member really is that large.
    std::vector<float> data(static_cast<std::size_t>(cells), 0.0f);
    std::vector<std::uint8_t> buf(kChunk);
    for (int plane = 0; plane < 4; ++plane) {
        const int shift = 8 * plane;
        std::size_t index = 0;
        while (index < data.size()) {
            const std::size_t n = std::min(kChunk, data.size() - index);
            if (!readExact(stream, buf.data(), n)) {
                return "the member ends inside the cell data";
            }
            for (std::size_t k = 0; k < n; ++k) {
                std::uint32_t bits = bitsOf(data[index + k]);
                bits |= static_cast<std::uint32_t>(buf[k]) << shift;
                std::memcpy(&data[index + k], &bits, sizeof bits);
            }
            index += n;
        }
    }
    for (float& v : data) {
        if (std::isnan(v)) {
            v = relief::kTransparent;
        } else if (std::isinf(v)) {
            return "the height field holds an infinite value";
        }
    }
    if (relief::HeightField::fromData(grid, std::move(data), out) != relief::Status::Ok) {
        return "the height-field geometry is invalid";
    }
    return {};
}

}  // namespace carvetto::document::detail
