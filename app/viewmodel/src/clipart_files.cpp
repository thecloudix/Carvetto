#include "carvetto/viewmodel/clipart_files.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <fstream>
#include <utility>

#include "carvetto/io/dxf.hpp"
#include "carvetto/io/png_guard.hpp"
#include "carvetto/io/svg.hpp"

namespace carvetto::viewmodel {

namespace fs = std::filesystem;

namespace {

constexpr std::size_t kSniffBytes = 4096;

bool startsWith(std::span<const std::uint8_t> b, std::string_view prefix) {
    return b.size() >= prefix.size() && std::equal(prefix.begin(), prefix.end(), b.begin(), [](char c, std::uint8_t v) {
               return static_cast<std::uint8_t>(c) == v;
           });
}

bool contains(std::span<const std::uint8_t> head, std::string_view needle) {
    return std::search(head.begin(), head.end(), needle.begin(), needle.end(),
                       [](std::uint8_t v, char c) { return v == static_cast<std::uint8_t>(c); }) != head.end();
}

bool isSpace(std::uint8_t c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n'; }

std::string lower(std::string_view s) {
    std::string out(s);
    for (char& c : out) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return out;
}

std::string baseName(std::string_view path) {
    const std::size_t cut = path.find_last_of("/\\");
    return std::string(cut == std::string_view::npos ? path : path.substr(cut + 1));
}

ClipartFile refuse(std::string_view name, const std::string& why) {
    ClipartFile f;
    f.error = std::string(name) + ": " + why;
    return f;
}

}  // namespace

const char* clipartKindName(ClipartKind kind) {
    switch (kind) {
        case ClipartKind::Svg:
            return "svg";
        case ClipartKind::Dxf:
            return "dxf";
        case ClipartKind::Png:
            return "png";
    }
    return "?";
}

std::string formatClipartBytes(std::uint64_t bytes) {
    char buf[48];
    if (bytes >= 1024ULL * 1024ULL) {
        std::snprintf(buf, sizeof buf, "%.1f MiB", static_cast<double>(bytes) / (1024.0 * 1024.0));
    } else if (bytes >= 1024ULL) {
        std::snprintf(buf, sizeof buf, "%.1f KiB", static_cast<double>(bytes) / 1024.0);
    } else {
        std::snprintf(buf, sizeof buf, "%llu bytes", static_cast<unsigned long long>(bytes));
    }
    return buf;
}

std::optional<ClipartKind> clipartKindFromExtension(std::string_view path) {
    const std::string name = baseName(path);
    const std::size_t dot = name.find_last_of('.');
    if (dot == std::string::npos || dot == 0) {
        return std::nullopt;
    }
    const std::string ext = lower(std::string_view(name).substr(dot + 1));
    if (ext == "svg") {
        return ClipartKind::Svg;
    }
    if (ext == "dxf") {
        return ClipartKind::Dxf;
    }
    if (ext == "png") {
        return ClipartKind::Png;
    }
    return std::nullopt;
}

ClipartSniff sniffClipart(std::span<const std::uint8_t> bytes) {
    ClipartSniff out;
    if (bytes.empty()) {
        out.refusal = "the file is empty";
        return out;
    }
    if (startsWith(bytes, "\x89PNG\r\n\x1a\n")) {
        out.kind = ClipartKind::Png;
        return out;
    }
    struct Archive {
        std::string_view magic;
        const char* what;
    };
    static const std::array<Archive, 6> kArchives = {{
        {std::string_view("\x1f\x8b", 2), "gzip-compressed (such as .svgz)"},
        {std::string_view("PK\x03\x04", 4), "a zip archive"},
        {std::string_view("7z\xbc\xaf\x27\x1c", 6), "a 7z archive"},
        {std::string_view("\xfd" "7zXZ\0", 6), "an xz-compressed file"},
        {std::string_view("BZh", 3), "a bzip2-compressed file"},
        {std::string_view("\x28\xb5\x2f\xfd", 4), "a zstd-compressed file"},
    }};
    for (const Archive& a : kArchives) {
        if (startsWith(bytes, a.magic)) {
            out.refusal = std::string("the file is ") + a.what +
                          "; compressed clipart is not read, unpack it and add the plain file";
            return out;
        }
    }
    if (startsWith(bytes, "AutoCAD Binary DXF")) {
        out.kind = ClipartKind::Dxf;
        return out;
    }
    const std::span<const std::uint8_t> head = bytes.first(std::min(bytes.size(), kSniffBytes));
    std::size_t i = startsWith(head, "\xEF\xBB\xBF") ? 3U : 0U;
    while (i < head.size() && isSpace(head[i])) {
        ++i;
    }
    if (i >= head.size()) {
        out.refusal = "the file holds only blank space";
        return out;
    }
    if (head[i] == '<') {
        if (contains(head, "<svg")) {
            out.kind = ClipartKind::Svg;
        } else {
            out.refusal = "an XML file without an <svg> element in its first 4 KiB is not SVG";
        }
        return out;
    }
    // ASCII DXF: a group code (1 to 4 digits) on the first line, and a SECTION within the head.
    std::size_t digits = 0;
    while (i + digits < head.size() && std::isdigit(head[i + digits]) != 0) {
        ++digits;
    }
    std::size_t after = i + digits;
    while (after < head.size() && (head[after] == ' ' || head[after] == '\t')) {
        ++after;
    }
    const bool lineEnds = after < head.size() && (head[after] == '\r' || head[after] == '\n');
    if (digits >= 1 && digits <= 4 && lineEnds && contains(head, "SECTION")) {
        out.kind = ClipartKind::Dxf;
        return out;
    }
    out.refusal = "the content is not SVG, DXF or PNG";
    return out;
}

std::optional<std::pair<std::uint32_t, std::uint32_t>> pngDimensions(std::span<const std::uint8_t> b) {
    if (b.size() < 24 || !startsWith(b, "\x89PNG\r\n\x1a\n") || !(b[12] == 'I' && b[13] == 'H' && b[14] == 'D' && b[15] == 'R')) {
        return std::nullopt;
    }
    const auto be32 = [&](std::size_t at) {
        return (static_cast<std::uint32_t>(b[at]) << 24) | (static_cast<std::uint32_t>(b[at + 1]) << 16) |
               (static_cast<std::uint32_t>(b[at + 2]) << 8) | static_cast<std::uint32_t>(b[at + 3]);
    };
    return std::make_pair(be32(16), be32(20));
}

std::string checkPngPixelData(std::span<const std::uint8_t> bytes, const ClipartLimits& limits) {
    const io::PngGuardResult g = io::checkPngPixelData(bytes, limits.maxPngRawBytes);
    switch (g.status) {
        case io::PngGuardStatus::Ok:
            return {};
        case io::PngGuardStatus::BadHeader:
            return "the PNG header is damaged or not supported";
        case io::PngGuardStatus::BadData:
            return "the PNG pixel data is damaged or truncated";
        case io::PngGuardStatus::TooLarge:
            return "the PNG declares " + std::to_string(g.expectedBytes / (1024U * 1024U)) +
                   " MiB of pixel data; the limit is " + std::to_string(limits.maxPngRawBytes / (1024U * 1024U)) +
                   " MiB";
        case io::PngGuardStatus::TooMuchData:
            return "the PNG holds far more pixel data than its header declares (a decompression bomb)";
    }
    return "the PNG pixel data cannot be checked";
}

ClipartFile checkClipartBytes(std::span<const std::uint8_t> bytes, std::string_view name, const ClipartLimits& limits) {
    const std::optional<ClipartKind> ext = clipartKindFromExtension(name);
    if (!ext) {
        return refuse(name, "not a supported clipart type (SVG, DXF and PNG files are supported)");
    }
    const ClipartSniff sniff = sniffClipart(bytes);
    if (!sniff.kind) {
        return refuse(name, sniff.refusal);
    }
    if (*sniff.kind != *ext) {
        return refuse(name, std::string("named .") + clipartKindName(*ext) + " but the content is " +
                                clipartKindName(*sniff.kind) + "; rename the file");
    }
    if (*ext == ClipartKind::Png) {
        const auto dims = pngDimensions(bytes);
        if (!dims) {
            return refuse(name, "the PNG header is damaged or truncated");
        }
        const std::uint64_t w = dims->first;
        const std::uint64_t h = dims->second;
        if (w == 0 || h == 0) {
            return refuse(name, "the PNG declares a zero-sized image");
        }
        if (w > static_cast<std::uint64_t>(limits.maxImageEdge) || h > static_cast<std::uint64_t>(limits.maxImageEdge) ||
            w * h > limits.maxImagePixels) {
            return refuse(name, "the PNG is " + std::to_string(w) + " x " + std::to_string(h) +
                                    " pixels; the limit is " + std::to_string(limits.maxImageEdge) +
                                    " pixels an edge and " + std::to_string(limits.maxImagePixels / 1'000'000U) +
                                    " million pixels in all");
        }
        const std::string bomb = checkPngPixelData(bytes, limits);
        if (!bomb.empty()) {
            return refuse(name, bomb);
        }
    }
    ClipartFile f;
    f.ok = true;
    f.kind = *ext;
    f.bytes.assign(bytes.begin(), bytes.end());
    return f;
}

ClipartFile readClipartFile(const std::string& path, const ClipartLimits& limits) {
    const std::string name = baseName(path);
    const std::optional<ClipartKind> ext = clipartKindFromExtension(path);
    if (!ext) {
        return refuse(name, "not a supported clipart type (SVG, DXF and PNG files are supported)");
    }
    std::error_code ec;
    const fs::path p(std::u8string(reinterpret_cast<const char8_t*>(path.data()), path.size()));
    if (!fs::is_regular_file(p, ec) || ec) {
        return refuse(name, "not a readable regular file");
    }
    const std::uintmax_t size = fs::file_size(p, ec);
    if (ec) {
        return refuse(name, "cannot read the file size");
    }
    if (size == 0) {
        return refuse(name, "the file is empty");
    }
    const std::size_t cap = *ext == ClipartKind::Png ? limits.maxPngFileBytes : limits.maxVectorFileBytes;
    if (size > cap) {
        return refuse(name, "the file is " + formatClipartBytes(size) + "; the limit for ." + clipartKindName(*ext) +
                                " clipart is " + formatClipartBytes(cap));
    }
    std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size));
    {
        std::ifstream in(p, std::ios::binary);
        if (!in) {
            return refuse(name, "cannot open the file");
        }
        in.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        if (static_cast<std::size_t>(in.gcount()) != bytes.size()) {
            return refuse(name, "the file changed while it was being read");
        }
    }
    return checkClipartBytes(bytes, name, limits);
}

ClipartGeometry parseClipartVectors(std::span<const std::uint8_t> bytes, ClipartKind kind,
                                    const ClipartLimits& limits) {
    ClipartGeometry out;
    try {
        if (kind == ClipartKind::Svg) {
            io::SvgImportOptions o;
            o.maxInputBytes = limits.maxVectorFileBytes;
            o.maxDepth = limits.svgMaxDepth;
            o.maxElements = limits.svgMaxElements;
            o.maxSpans = limits.svgMaxSpans;
            o.maxVisits = limits.svgMaxVisits;
            o.maxUseDepth = limits.svgMaxUseDepth;
            auto r = io::importSvg(bytes, o);
            if (!r) {
                out.error = "cannot read the SVG: " + r.error().describe();
                return out;
            }
            for (const io::SvgWarning& w : r.value().warnings) {
                out.warnings.push_back(w.describe());
            }
            for (io::SvgContour& c : r.value().contours) {
                out.contours.push_back(std::move(c.contour));
            }
        } else if (kind == ClipartKind::Dxf) {
            io::DxfImportOptions o;
            o.maxSpans = limits.dxfMaxSpans;
            o.maxTexts = limits.dxfMaxTexts;
            o.maxInstances = limits.dxfMaxInstances;
            o.maxBlockDepth = limits.dxfMaxBlockDepth;
            auto r = io::importDxf(bytes, o);
            if (!r) {
                out.error = "cannot read the DXF: " + r.error().describe();
                return out;
            }
            for (const io::DxfWarning& w : r.value().warnings) {
                out.warnings.push_back(w.describe());
            }
            out.textItems = r.value().texts.size();
            for (io::DxfContour& c : r.value().contours) {
                out.contours.push_back(std::move(c.contour));
            }
        } else {
            out.error = "not a vector file";
            return out;
        }
        if (out.contours.size() > limits.maxContours) {
            out.error = "the drawing holds " + std::to_string(out.contours.size()) + " shapes; the limit is " +
                        std::to_string(limits.maxContours);
            out.contours.clear();
            return out;
        }
        for (const geometry::Contour& c : out.contours) {
            const geometry::Box2 b = c.bounds();
            if (!b.isEmpty() && !(std::isfinite(b.min.x) && std::isfinite(b.min.y) && std::isfinite(b.max.x) &&
                                  std::isfinite(b.max.y))) {
                out.error = "the drawing holds geometry with non-finite coordinates";
                out.contours.clear();
                return out;
            }
        }
        out.ok = true;
    } catch (const std::exception& e) {
        out.ok = false;
        out.contours.clear();
        out.error = std::string("cannot read the drawing: ") + e.what();
    }
    return out;
}

}  // namespace carvetto::viewmodel
