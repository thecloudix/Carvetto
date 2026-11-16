#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/geometry/contour.hpp"

// Clipart library, part 1 (T-0333, ADR-0109): what a clipart file is, and how to read one without
// trusting it. Qt-free. Everything here works on plain bytes or a local path; nothing is fetched
// from the network and no clipart ships with Carvetto.

namespace carvetto::viewmodel {

/// The three file types the clipart browser reads.
enum class ClipartKind { Svg, Dxf, Png };

/// Lower-case name of a kind ("svg", "dxf", "png").
const char* clipartKindName(ClipartKind kind);

/// Hostile-input budgets of the browser. Every default is far below what a real clipart file needs
/// and well inside the importers' own (looser) caps.
struct ClipartLimits {
    /// Largest SVG or DXF file read (bytes).
    std::size_t maxVectorFileBytes{8U * 1024U * 1024U};
    /// Largest PNG file read (bytes).
    std::size_t maxPngFileBytes{32U * 1024U * 1024U};
    /// Largest PNG edge and pixel count, checked on the IHDR before any pixel is decoded.
    int maxImageEdge{8192};
    std::size_t maxImagePixels{std::size_t{16} * 1024U * 1024U};
    /// SVG budgets (see `io::SvgImportOptions`).
    std::size_t svgMaxDepth{48};
    std::size_t svgMaxElements{200'000};
    std::size_t svgMaxSpans{200'000};
    std::size_t svgMaxVisits{400'000};
    std::size_t svgMaxUseDepth{8};
    /// DXF budgets (see `io::DxfImportOptions`).
    std::size_t dxfMaxSpans{200'000};
    std::size_t dxfMaxTexts{5'000};
    std::size_t dxfMaxInstances{50'000};
    std::size_t dxfMaxBlockDepth{16};
    /// Most contours (= objects added to the drawing) one clipart file may hold.
    std::size_t maxContours{20'000};
};

/// The kind a path's extension names (case-insensitive: .svg .dxf .png); nullopt otherwise.
std::optional<ClipartKind> clipartKindFromExtension(std::string_view path);

/// Result of `sniffClipart`.
struct ClipartSniff {
    std::optional<ClipartKind> kind;
    /// Why the content is refused (set when `kind` is empty).
    std::string refusal;
};

/// Recognises the content from the first 4096 bytes: the PNG signature; an XML document with an
/// `<svg` element; an ASCII DXF (a group-code line then a SECTION) or the binary-DXF sentinel.
/// Compressed containers (gzip/.svgz, zip, 7z, xz, bzip2) are refused by name: Carvetto never
/// decompresses clipart, so decompression bombs cannot reach a parser.
ClipartSniff sniffClipart(std::span<const std::uint8_t> bytes);

/// Pixel size from a PNG's IHDR; nullopt when `bytes` is not a PNG header.
std::optional<std::pair<std::uint32_t, std::uint32_t>> pngDimensions(std::span<const std::uint8_t> bytes);

/// A file read within the limits.
struct ClipartFile {
    bool ok{false};
    /// One line for the user when `!ok` (starts with the file name).
    std::string error;
    ClipartKind kind{ClipartKind::Svg};
    std::vector<std::uint8_t> bytes;
};

/// Checks `bytes` (named `name`, whose extension names `extKind`): the content must be the same
/// kind as the extension says, and a PNG's dimensions must be inside the limits. Sets `kind`.
ClipartFile checkClipartBytes(std::span<const std::uint8_t> bytes, std::string_view name, const ClipartLimits& limits);

/// Reads the local file `path` (UTF-8) for the browser: refuses a missing or non-regular file, an
/// unknown extension, an empty file or one over the size limit of its kind (the size is checked
/// before reading, and reading never exceeds it), content that disagrees with the extension, and a
/// PNG with oversize dimensions.
ClipartFile readClipartFile(const std::string& path, const ClipartLimits& limits);

/// Vector content of an SVG or DXF clipart (millimetres, Y up).
struct ClipartGeometry {
    bool ok{false};
    std::string error;
    std::vector<geometry::Contour> contours;
    /// DXF text items (not drawn in thumbnails; imported only when a font is supplied).
    std::size_t textItems{0};
    /// Import warnings, one line each.
    std::vector<std::string> warnings;
};

/// Parses SVG or DXF bytes within the limits (never throws). Refuses a drawing with more than
/// `limits.maxContours` contours and one with non-finite geometry; an empty drawing is `ok` with
/// no contours. `kind` must not be Png.
ClipartGeometry parseClipartVectors(std::span<const std::uint8_t> bytes, ClipartKind kind,
                                    const ClipartLimits& limits);

/// "3.5 MiB"-style size text.
std::string formatClipartBytes(std::uint64_t bytes);

}  // namespace carvetto::viewmodel
