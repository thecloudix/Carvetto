#include "bitmap_decoder.hpp"

#include <QByteArray>
#include <QImage>
#include <QPainter>
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>

#include "carvetto/io/raster.hpp"

namespace carvetto::app {

document::BitmapDecoder qtBitmapDecoder() {
    return [](std::span<const std::uint8_t> bytes) -> geometry::ShapeResult<io::RasterImage> {
        using R = geometry::ShapeResult<io::RasterImage>;
        if (bytes.empty() || bytes.size() > static_cast<std::size_t>(INT32_MAX)) {
            return R(std::string("the image file is empty or too large"));
        }
        const QByteArray data = QByteArray::fromRawData(reinterpret_cast<const char*>(bytes.data()),
                                                        static_cast<qsizetype>(bytes.size()));
        QImage image = QImage::fromData(data);
        if (image.isNull()) {
            return R(std::string("the image format is not supported or the file is damaged"));
        }
        const int w = image.width();
        const int h = image.height();
        if (w <= 0 || h <= 0 || w > io::kMaxImageEdge || h > io::kMaxImageEdge ||
            static_cast<std::size_t>(w) * static_cast<std::size_t>(h) > io::kMaxImagePixels) {
            return R(std::string("the image is too large (at most 16384 pixels an edge, 16 million pixels)"));
        }
        if (image.hasAlphaChannel()) {
            QImage flat(w, h, QImage::Format_RGB32);
            flat.fill(Qt::white);
            {
                QPainter painter(&flat);
                painter.drawImage(0, 0, image);
            }
            image = std::move(flat);
        }
        image = image.convertToFormat(QImage::Format_RGB888);
        io::RasterImage out;
        out.width = w;
        out.height = h;
        out.rgb.resize(static_cast<std::size_t>(w) * static_cast<std::size_t>(h) * 3);
        for (int y = 0; y < h; ++y) {
            const uchar* line = image.constScanLine(y);
            std::copy(line, line + static_cast<std::size_t>(w) * 3,
                      out.rgb.begin() + static_cast<std::ptrdiff_t>(static_cast<std::size_t>(y) * static_cast<std::size_t>(w) * 3));
        }
        return out;
    };
}

}  // namespace carvetto::app
