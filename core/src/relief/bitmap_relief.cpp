#include "carvetto/relief/bitmap_relief.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iterator>

namespace carvetto::relief {

namespace {

using R = geometry::ShapeResult<GrayImage16>;

struct Cursor {
    std::string_view d;
    std::size_t pos;
    bool failed = false;

    void skipSpace() {
        while (pos < d.size()) {
            const char c = d[pos];
            if (c == '#') {
                while (pos < d.size() && d[pos] != '\n') ++pos;
            } else if (c == ' ' || c == '\t' || c == '\n' || c == '\r' || c == '\f' || c == '\v') {
                ++pos;
            } else {
                break;
            }
        }
    }
    long long number() {
        skipSpace();
        if (pos >= d.size() || d[pos] < '0' || d[pos] > '9') {
            failed = true;
            return 0;
        }
        long long v = 0;
        while (pos < d.size() && d[pos] >= '0' && d[pos] <= '9') {
            v = v * 10 + (d[pos] - '0');
            if (v > 100000000LL) v = 100000000LL;  // saturate; callers range-check
            ++pos;
        }
        return v;
    }
};

std::uint16_t scaleSample(long long v, long long maxval) {
    return static_cast<std::uint16_t>((v * 65535 + maxval / 2) / maxval);
}

std::uint16_t luma16(long long r, long long g, long long b) {
    return static_cast<std::uint16_t>((299 * r + 587 * g + 114 * b + 500) / 1000);
}

R decodeNetpbmDeep(std::string_view data) {
    const int kind = data[1] - '0';  // 2, 3, 5 or 6
    Cursor c{data, 2};
    const long long w = c.number();
    const long long h = c.number();
    const long long maxval = c.number();
    if (c.failed) return R(std::string("bad Netpbm header"));
    if (w <= 0 || h <= 0 || w > io::kMaxImageEdge || h > io::kMaxImageEdge ||
        static_cast<std::size_t>(w) * static_cast<std::size_t>(h) > io::kMaxImagePixels) {
        return R(std::string("bad or oversize image dimensions"));
    }
    if (maxval < 1 || maxval > 65535) return R(std::string("bad Netpbm maxval"));
    const std::size_t n = static_cast<std::size_t>(w) * static_cast<std::size_t>(h);
    const std::size_t chans = (kind == 3 || kind == 6) ? 3 : 1;
    const bool binary = kind >= 5;
    const std::size_t bytesPer = maxval > 255 ? 2 : 1;
    std::size_t at = 0;
    if (binary) {
        if (c.pos >= data.size()) return R(std::string("truncated image data"));
        at = c.pos + 1;  // exactly one whitespace byte after maxval
        if (data.size() - at < n * chans * bytesPer) return R(std::string("truncated image data"));
    }
    GrayImage16 img;
    img.width = static_cast<int>(w);
    img.height = static_cast<int>(h);
    img.px.resize(n);
    auto next = [&]() -> long long {
        long long v = 0;
        if (binary) {
            v = static_cast<unsigned char>(data[at++]);
            if (bytesPer == 2) v = v * 256 + static_cast<unsigned char>(data[at++]);
        } else {
            v = c.number();
        }
        return v;
    };
    for (std::size_t i = 0; i < n; ++i) {
        long long s[3] = {0, 0, 0};
        for (std::size_t k = 0; k < chans; ++k) {
            s[k] = next();
            if (c.failed) return R(std::string("truncated image data"));
            if (s[k] > maxval) return R(std::string("sample above maxval"));
            s[k] = static_cast<long long>(scaleSample(s[k], maxval));
        }
        img.px[i] = chans == 3 ? luma16(s[0], s[1], s[2]) : static_cast<std::uint16_t>(s[0]);
    }
    return R(std::move(img));
}

bool finitePositive(double v) { return std::isfinite(v) && v > 0.0; }

}  // namespace

geometry::ShapeResult<GrayImage16> grayFromRaster(const io::RasterImage& img) {
    if (!img.valid()) return R(std::string("invalid image"));
    GrayImage16 g;
    g.width = img.width;
    g.height = img.height;
    const std::size_t n = static_cast<std::size_t>(img.width) * static_cast<std::size_t>(img.height);
    g.px.resize(n);
    for (std::size_t i = 0; i < n; ++i) {
        g.px[i] = luma16(img.rgb[3 * i] * 257LL, img.rgb[3 * i + 1] * 257LL, img.rgb[3 * i + 2] * 257LL);
    }
    return R(std::move(g));
}

geometry::ShapeResult<GrayImage16> decodeGray16(std::span<const std::uint8_t> bytes,
                                                const document::BitmapDecoder& decoder) {
    const std::string_view head(reinterpret_cast<const char*>(bytes.data()), bytes.size());
    if (head.size() >= 3 && head[0] == 'P' && (head[1] == '2' || head[1] == '3' || head[1] == '5' || head[1] == '6')) {
        return decodeNetpbmDeep(head);
    }
    auto raster = document::decodeBitmapBytes(bytes, decoder);
    if (!raster.ok()) return R(raster.error());
    return grayFromRaster(raster.value());
}

geometry::ShapeResult<GrayImage16> loadBitmapObjectGray16(const document::Document& doc,
                                                          const document::BitmapObject& obj,
                                                          const document::BitmapDecoder& decoder) {
    const document::BitmapAsset* asset = doc.findBitmap(obj.sourceRef);
    if (asset == nullptr) return R(std::string("the bitmap has no embedded image data"));
    return decodeGray16(std::span<const std::uint8_t>(asset->bytes), decoder);
}

geometry::ShapeResult<GrayImage16> loadFileGray16(const std::string& path, const document::BitmapDecoder& decoder) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return R(std::string("cannot open image file"));
    std::vector<std::uint8_t> bytes;
    char buf[65536];
    while (in.read(buf, sizeof buf) || in.gcount() > 0) {
        const auto got = static_cast<std::size_t>(in.gcount());
        if (bytes.size() + got > document::kMaxEmbeddedBitmapBytes) return R(std::string("image file too large"));
        bytes.insert(bytes.end(), reinterpret_cast<std::uint8_t*>(buf), reinterpret_cast<std::uint8_t*>(buf) + got);
    }
    if (bytes.empty()) return R(std::string("empty image file"));
    return decodeGray16(std::span<const std::uint8_t>(bytes), decoder);
}

Status heightFieldFromBitmap(const GrayImage16& img, const BitmapReliefParams& p, HeightField& out) {
    out = HeightField();
    if (!img.valid() || !finitePositive(p.widthMm) || !finitePositive(p.zHeightMm) ||
        !std::isfinite(p.corner.x) || !std::isfinite(p.corner.y)) {
        return Status::BadParameter;
    }
    const std::size_t w = static_cast<std::size_t>(img.width);
    const std::size_t h = static_cast<std::size_t>(img.height);
    Grid g;
    g.width = w;
    g.height = h;
    g.cellSize = p.widthMm / static_cast<double>(w);
    g.origin = Point{p.corner.x + g.cellSize * 0.5, p.corner.y + g.cellSize * 0.5};
    const Status v = validate(g);
    if (v != Status::Ok) return v;

    long long lo = 0;
    long long hi = 65535;
    if (p.autoScale) {
        const auto mm = std::minmax_element(img.px.begin(), img.px.end());
        lo = p.invert ? 65535 - *mm.second : *mm.first;
        hi = p.invert ? 65535 - *mm.first : *mm.second;
    }
    std::vector<float> data(w * h, 0.0f);
    if (hi > lo) {
        const double span = static_cast<double>(hi - lo);
        for (std::size_t j = 0; j < h; ++j) {
            const std::size_t srcRow = h - 1 - j;
            for (std::size_t i = 0; i < w; ++i) {
                const long long raw = img.px[srcRow * w + i];
                const long long q = p.invert ? 65535 - raw : raw;
                data[j * w + i] = static_cast<float>(p.zHeightMm * static_cast<double>(q - lo) / span);
            }
        }
    }
    return HeightField::fromData(g, std::move(data), out);
}

Status makeBitmapComponent(const GrayImage16& img, const BitmapReliefParams& p, Component& out) {
    HeightField f;
    const Status s = heightFieldFromBitmap(img, p, f);
    if (s != Status::Ok) return s;
    out = Component();
    out.name = p.name;
    out.field = std::move(f);
    out.mode = p.mode;
    return Status::Ok;
}

Status makeZeroPlane(double jobWidthMm, double jobHeightMm, double cellSizeMm, Point corner, Component& out) {
    if (!finitePositive(jobWidthMm) || !finitePositive(jobHeightMm) || !finitePositive(cellSizeMm) ||
        !std::isfinite(corner.x) || !std::isfinite(corner.y)) {
        return Status::BadParameter;
    }
    const double cw = std::ceil(jobWidthMm / cellSizeMm);
    const double ch = std::ceil(jobHeightMm / cellSizeMm);
    if (cw >= static_cast<double>(kMaxCells) || ch >= static_cast<double>(kMaxCells)) return Status::TooLarge;
    Grid g;
    g.width = static_cast<std::size_t>(cw) + 1;
    g.height = static_cast<std::size_t>(ch) + 1;
    g.cellSize = cellSizeMm;
    g.origin = corner;
    HeightField f;
    const Status s = HeightField::create(g, 0.0f, f);
    if (s != Status::Ok) return s;
    out = Component();
    out.name = "Zero plane";
    out.field = std::move(f);
    out.mode = CombineMode::Add;
    return Status::Ok;
}

Status scaleModelZ(HeightField& field, const ScaleZ& s, double* factorOut) {
    if (!finitePositive(s.value) || field.empty()) return Status::BadParameter;
    double factor = 1.0;
    if (s.kind == ScaleZ::Kind::Percent) {
        factor = s.value / 100.0;
    } else {
        float lo = 0.0f;
        float hi = 0.0f;
        if (!field.minMax(lo, hi)) return Status::BadParameter;
        const double top = std::max(static_cast<double>(hi), 0.0);
        const double bottom = s.bothSides ? std::min(static_cast<double>(lo), 0.0) : 0.0;
        const double extent = top - bottom;
        if (!(extent > 0.0)) return Status::BadParameter;
        factor = s.value / extent;
    }
    if (!std::isfinite(factor)) return Status::BadParameter;
    std::vector<float> d = field.data();
    for (float& v : d) {
        if (std::isnan(v)) continue;
        if (v >= 0.0f || s.bothSides) v = static_cast<float>(static_cast<double>(v) * factor);
    }
    HeightField r;
    const Status st = HeightField::fromData(field.grid(), std::move(d), r);
    if (st != Status::Ok) return st;
    field = std::move(r);
    if (factorOut != nullptr) *factorOut = factor;
    return Status::Ok;
}

}  // namespace carvetto::relief
