#include "carvetto/io/raster.hpp"

#include <string>

namespace carvetto::io {

namespace {

struct Reader {
    std::string_view d;
    std::size_t pos{0};

    void skipSpace() {
        while (pos < d.size()) {
            const char c = d[pos];
            if (c == '#') {
                while (pos < d.size() && d[pos] != '\n') ++pos;
            } else if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
                ++pos;
            } else {
                break;
            }
        }
    }
    /// Reads an unsigned decimal; -1 on failure. Saturates above 2^31 so overflow is reported.
    long long number() {
        skipSpace();
        if (pos >= d.size() || d[pos] < '0' || d[pos] > '9') return -1;
        long long v = 0;
        while (pos < d.size() && d[pos] >= '0' && d[pos] <= '9') {
            v = v * 10 + (d[pos] - '0');
            if (v > 2'000'000'000LL) v = 2'000'000'000LL;
            ++pos;
        }
        return v;
    }
};

std::uint8_t scale(long long v, long long maxval) {
    return static_cast<std::uint8_t>((v * 255 + maxval / 2) / maxval);
}

}  // namespace

geometry::ShapeResult<RasterImage> decodePnm(std::string_view data) {
    using R = geometry::ShapeResult<RasterImage>;
    if (data.size() < 3 || data[0] != 'P' || data[1] < '1' || data[1] > '6') return R(std::string("not a Netpbm file"));
    const int kind = data[1] - '0';
    Reader rd{data, 2};
    const long long w = rd.number();
    const long long h = rd.number();
    if (w <= 0 || h <= 0 || w > kMaxImageEdge || h > kMaxImageEdge ||
        static_cast<std::size_t>(w) * static_cast<std::size_t>(h) > kMaxImagePixels) {
        return R(std::string("bad or oversize image dimensions"));
    }
    const bool bitmap = (kind == 1 || kind == 4);
    long long maxval = 1;
    if (!bitmap) {
        maxval = rd.number();
        if (maxval < 1 || maxval > 65535) return R(std::string("bad maxval"));
    }
    const bool binary = kind >= 4;
    const bool colour = (kind == 3 || kind == 6);
    const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);

    // Binary payloads start after exactly one whitespace byte following the header.
    if (binary) {
        if (rd.pos >= data.size()) return R(std::string("truncated data"));
        ++rd.pos;
    }
    // Size check against the real payload before allocating anything proportional to w*h.
    if (binary) {
        std::size_t need = 0;
        if (bitmap) {
            need = ((static_cast<std::size_t>(w) + 7) / 8) * static_cast<std::size_t>(h);
        } else {
            need = n * (colour ? 3 : 1) * (maxval > 255 ? 2 : 1);
        }
        if (data.size() - rd.pos < need) return R(std::string("truncated data"));
    } else if (data.size() - rd.pos < n * (colour ? 3 : 1)) {
        return R(std::string("truncated data"));  // at least one byte per sample is needed
    }

    RasterImage img;
    img.width = static_cast<int>(w);
    img.height = static_cast<int>(h);
    img.rgb.resize(n * 3);
    auto put = [&](std::size_t i, std::uint8_t r, std::uint8_t g, std::uint8_t b) {
        img.rgb[i * 3] = r;
        img.rgb[i * 3 + 1] = g;
        img.rgb[i * 3 + 2] = b;
    };
    auto byteAt = [&](std::size_t p) { return static_cast<unsigned char>(data[p]); };

    if (kind == 1) {
        for (std::size_t i = 0; i < n; ++i) {
            rd.skipSpace();
            if (rd.pos >= data.size() || (data[rd.pos] != '0' && data[rd.pos] != '1')) return R(std::string("bad bitmap sample"));
            const std::uint8_t v = data[rd.pos++] == '1' ? 0 : 255;
            put(i, v, v, v);
        }
    } else if (kind == 4) {
        const std::size_t stride = (static_cast<std::size_t>(w) + 7) / 8;
        for (long long y = 0; y < h; ++y) {
            for (long long x = 0; x < w; ++x) {
                const unsigned char byte = byteAt(rd.pos + static_cast<std::size_t>(y) * stride + static_cast<std::size_t>(x) / 8);
                const bool black = ((byte >> (7 - x % 8)) & 1) != 0;
                const std::uint8_t v = black ? 0 : 255;
                put(static_cast<std::size_t>(y * w + x), v, v, v);
            }
        }
    } else if (binary) {
        const int bytes = maxval > 255 ? 2 : 1;
        const int ch = colour ? 3 : 1;
        std::size_t p = rd.pos;
        for (std::size_t i = 0; i < n; ++i) {
            std::uint8_t c[3] = {0, 0, 0};
            for (int k = 0; k < ch; ++k) {
                long long v = byteAt(p);
                if (bytes == 2) v = v * 256 + byteAt(p + 1);
                p += static_cast<std::size_t>(bytes);
                if (v > maxval) v = maxval;
                c[k] = scale(v, maxval);
            }
            if (colour) put(i, c[0], c[1], c[2]); else put(i, c[0], c[0], c[0]);
        }
    } else {
        const int ch = colour ? 3 : 1;
        for (std::size_t i = 0; i < n; ++i) {
            std::uint8_t c[3] = {0, 0, 0};
            for (int k = 0; k < ch; ++k) {
                long long v = rd.number();
                if (v < 0) return R(std::string("truncated or bad sample"));
                if (v > maxval) v = maxval;
                c[k] = scale(v, maxval);
            }
            if (colour) put(i, c[0], c[1], c[2]); else put(i, c[0], c[0], c[0]);
        }
    }
    return R(std::move(img));
}

}  // namespace carvetto::io
