#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"

// Importing a bitmap into the document (T-0250b; File > Import Bitmap): the Qt-free half. The app
// supplies the decoder (QImage) for PNG/JPEG/...; Netpbm is read by core. The encoded file is
// embedded verbatim (ADR-0051), the image is placed on the active layer of the active sheet, scaled
// to fit the sheet's material (aspect kept, centred; 100 mm wide when the material is empty), and
// selected. One undo step (`makeImportBitmapCommand`).

namespace carvetto::viewmodel {

struct BitmapImportOutcome {
    bool ok{false};
    /// Why the import was refused (nothing changes then).
    std::string error;
    /// The new bitmap object.
    document::ObjectId object;
    int widthPx{0};
    int heightPx{0};
};

/// Imports the encoded image `bytes` (named `name`). Refuses an undecodable image, one over the
/// embedding caps or pixel limits, and a locked or hidden active layer.
BitmapImportOutcome importBitmapIntoDocument(document::Document& doc, std::vector<std::uint8_t> bytes,
                                             const std::string& name, const document::BitmapDecoder& decoder);

}  // namespace carvetto::viewmodel
