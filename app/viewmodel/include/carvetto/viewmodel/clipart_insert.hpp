#pragma once

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/viewmodel/clipart_files.hpp"

// Clipart library, part 4 (T-0333, ADR-0109): putting a clipart file into the drawing.

namespace carvetto::viewmodel {

/// How and where a clipart file is inserted.
struct ClipartInsertOptions {
    /// Centre of the inserted clipart in document millimetres (the cursor / drop point); nullopt =
    /// the centre of the active sheet's material (the origin when the material is empty).
    std::optional<geometry::Vec2> at;
    /// The clipart is scaled uniformly so its bounding box just fits a box of this fraction of the
    /// material's width and height (of 100 mm x 100 mm when the material is empty). In (0, 1].
    double fitFraction{0.5};
    ClipartLimits limits;
    /// TTF/OTF bytes for DXF text; empty = DXF text is skipped with a warning.
    std::span<const std::uint8_t> textFontBytes;
};

/// What an insertion did, or why it did nothing.
struct ClipartInsertOutcome {
    bool ok{false};
    /// One line for the user when `!ok`; the document is untouched then.
    std::string error;
    ClipartKind kind{ClipartKind::Svg};
    /// Objects added (vectors, text, or the one bitmap).
    std::size_t objectsAdded{0};
    /// Uniform scale applied to the imported size.
    double scale{1.0};
    /// Where the clipart now lies.
    geometry::Box2 placedBox;
    std::vector<std::string> warnings;
};

/// Inserts the file `path` (UTF-8) into the active sheet: read within the limits, imported with the
/// SVG / DXF / bitmap importers (SVG shapes land on the active layer, DXF layers are matched or
/// created by name, a PNG is embedded), scaled to fit and centred on `options.at`, and selected -
/// all as ONE undo step "Insert clipart". `decoder` reads PNG (the app's QImage decoder).
ClipartInsertOutcome insertClipartFile(document::Document& doc, const std::string& path,
                                       const ClipartInsertOptions& options, const document::BitmapDecoder& decoder);

/// As `insertClipartFile` for bytes already read; `name` (with its extension) selects the kind and is
/// the bitmap's name. The bytes are checked like a file's.
ClipartInsertOutcome insertClipartBytes(document::Document& doc, std::span<const std::uint8_t> bytes,
                                        const std::string& name, const ClipartInsertOptions& options,
                                        const document::BitmapDecoder& decoder);

}  // namespace carvetto::viewmodel
