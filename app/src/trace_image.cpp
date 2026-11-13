#include "trace_image.hpp"

#include <algorithm>

#include <QBuffer>
#include <QFileInfo>
#include <QImage>
#include <QImageReader>
#include <QPainter>
#include <QSize>

namespace carvetto::app {

namespace {

DecodedImage decodeFromReader(QImageReader& reader) {
    DecodedImage out;
    reader.setAutoTransform(true);
    const QByteArray format = reader.format().toLower();
    if (format != "png" && format != "jpeg" && format != "jpg" && format != "bmp") {
        out.error = QStringLiteral("Only PNG, JPEG and BMP images can be traced.");
        return out;
    }
    const QSize size = reader.size();  // from the header, no pixel data decoded yet
    if (!size.isValid() || size.width() <= 0 || size.height() <= 0) {
        out.error = QStringLiteral("The image size could not be read; the file may be damaged.");
        return out;
    }
    const QSize& shown = size;  // a 90 degree EXIF turn swaps the edges; both are capped alike
    if (shown.width() > io::kMaxImageEdge || shown.height() > io::kMaxImageEdge ||
        static_cast<std::size_t>(shown.width()) * static_cast<std::size_t>(shown.height()) > io::kMaxImagePixels) {
        out.error = QStringLiteral("The image is too large to trace (%1 x %2 pixels; at most %3 pixels on a side and 16 million in all).")
                        .arg(shown.width())
                        .arg(shown.height())
                        .arg(io::kMaxImageEdge);
        return out;
    }
    reader.setAllocationLimit(256);  // MB, Qt's own guard on top of ours
    QImage img = reader.read();
    if (img.isNull()) {
        out.error = QStringLiteral("The image could not be decoded: %1").arg(reader.errorString());
        return out;
    }
    if (img.width() > io::kMaxImageEdge || img.height() > io::kMaxImageEdge ||
        static_cast<std::size_t>(img.width()) * static_cast<std::size_t>(img.height()) > io::kMaxImagePixels) {
        out.error = QStringLiteral("The image is too large to trace.");
        return out;
    }
    if (img.hasAlphaChannel()) {
        QImage flat(img.size(), QImage::Format_RGB32);
        flat.fill(Qt::white);
        QPainter painter(&flat);
        painter.drawImage(0, 0, img);
        painter.end();
        img = flat;
    }
    img = img.convertToFormat(QImage::Format_RGB888);
    io::RasterImage raster;
    raster.width = img.width();
    raster.height = img.height();
    const std::size_t rowBytes = static_cast<std::size_t>(raster.width) * 3;
    raster.rgb.resize(rowBytes * static_cast<std::size_t>(raster.height));
    for (int y = 0; y < raster.height; ++y) {
        const uchar* line = img.constScanLine(y);
        std::copy(line, line + rowBytes, raster.rgb.begin() + static_cast<std::ptrdiff_t>(rowBytes * static_cast<std::size_t>(y)));
    }
    if (!raster.valid()) {
        out.error = QStringLiteral("The image is not usable.");
        return out;
    }
    out.image = std::move(raster);
    out.ok = true;
    return out;
}

}  // namespace

DecodedImage decodeImageFile(const QString& path) {
    const QFileInfo info(path);
    if (!info.exists() || !info.isFile()) {
        DecodedImage out;
        out.error = QStringLiteral("The file does not exist.");
        return out;
    }
    if (info.size() > kTraceMaxFileBytes) {
        DecodedImage out;
        out.error = QStringLiteral("The image file is too large (over 64 MB).");
        return out;
    }
    QImageReader reader(path);
    return decodeFromReader(reader);
}

DecodedImage decodeImageBytes(const QByteArray& bytes) {
    if (bytes.size() > kTraceMaxFileBytes) {
        DecodedImage out;
        out.error = QStringLiteral("The image is too large (over 64 MB).");
        return out;
    }
    QByteArray copy = bytes;
    QBuffer buffer(&copy);
    buffer.open(QIODevice::ReadOnly);
    QImageReader reader(&buffer);
    return decodeFromReader(reader);
}

}  // namespace carvetto::app
