#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <vector>

#include "carvetto/document/command.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/io/raster.hpp"

/// Embedded bitmap pixel store (T-0273, ADR-0051): importing, undoing and decoding images that
/// live inside the document (and so inside the `.carv` file).
namespace carvetto::document {

/// Decodes encoded image bytes (PNG, JPEG, ...) into pixels. Supplied by the app layer (QImage);
/// core itself only reads Netpbm.
using BitmapDecoder = std::function<geometry::ShapeResult<io::RasterImage>(std::span<const std::uint8_t>)>;

/// Builds the asset for encoded image bytes: key = content hash, bytes kept untouched.
BitmapAsset makeBitmapAsset(std::vector<std::uint8_t> bytes);

/// Embeds an image. A no-op (returns false) when it is already embedded or refused by the caps of
/// `Document::insertBitmap`; undo removes it only if this command added it.
class AddBitmapAssetCommand : public Command {
   public:
    explicit AddBitmapAssetCommand(BitmapAsset image) : image_(std::move(image)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Embed image"; }

   private:
    BitmapAsset image_;
    bool added_{false};
};

/// Everything an import needs to place an image.
struct BitmapImport {
    std::vector<std::uint8_t> bytes;  ///< The encoded file, embedded verbatim.
    int widthPx{0};
    int heightPx{0};
    geometry::Box2 placementMm{};
    LayerId layer{};
    SheetId sheet{};
    std::string name;
};

/// The problem with `in` (empty when fine): empty bytes, non-positive or oversized pixel
/// dimensions, a degenerate or non-finite placement, or an embedding cap.
std::string bitmapImportProblem(const Document& doc, const BitmapImport& in);

/// One undo step that embeds the image (deduplicated against identical images already embedded)
/// and adds the bitmap object referring to it. Null when `bitmapImportProblem` is non-empty.
std::unique_ptr<Command> makeImportBitmapCommand(const Document& doc, BitmapImport in);

/// Decodes encoded image bytes: Netpbm directly, anything else through `decoder` (an error when
/// none is supplied or the result is not a valid image). The bytes-level part of
/// `decodeBitmapObject`, usable on a copy of the bytes away from the document (T-0275).
geometry::ShapeResult<io::RasterImage> decodeBitmapBytes(std::span<const std::uint8_t> bytes,
                                                         const BitmapDecoder& decoder = {});

/// Decodes the pixels of `obj`: Netpbm directly, anything else through `decoder`. Errors (never
/// a crash) when the object holds no embedded image, the bytes do not decode, or no decoder is
/// supplied for a non-Netpbm image.
geometry::ShapeResult<io::RasterImage> decodeBitmapObject(const Document& doc, const BitmapObject& obj,
                                                           const BitmapDecoder& decoder = {});

}  // namespace carvetto::document
