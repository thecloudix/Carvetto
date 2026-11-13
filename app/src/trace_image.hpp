#pragma once

#include <QByteArray>
#include <QString>

#include "carvetto/io/raster.hpp"

namespace carvetto::app {

/// Largest image file (bytes) Trace Bitmap reads.
inline constexpr qint64 kTraceMaxFileBytes = 64LL * 1024 * 1024;

struct DecodedImage {
    bool ok{false};
    QString error;  ///< for a message box when !ok
    io::RasterImage image;
};

/// Decodes a PNG, JPEG or BMP file into an RGB raster (T-0260, ADR-0038). The size is read from the
/// header with QImageReader and checked against `io::kMaxImageEdge` / `io::kMaxImagePixels` and the
/// file size against `kTraceMaxFileBytes` BEFORE any pixel is decoded or allocated; a refusal or a
/// decode failure comes back as `error`. Transparency is flattened onto white; EXIF orientation applies.
DecodedImage decodeImageFile(const QString& path);

/// The same for encoded bytes already in memory (an image embedded in the document, T-0274).
DecodedImage decodeImageBytes(const QByteArray& bytes);

}  // namespace carvetto::app
