#pragma once

#include "carvetto/document/bitmap_store.hpp"

namespace carvetto::app {

/// Decodes PNG, JPEG, BMP, GIF, ... with QImage into the 8-bit RGB raster core consumes (T-0275).
/// Transparent pixels are composited over white; images beyond the core's size limits are refused
/// before they are converted. Thread-safe (it runs on the toolpath worker): it uses no widget or
/// pixmap, only QImage.
document::BitmapDecoder qtBitmapDecoder();

}  // namespace carvetto::app
