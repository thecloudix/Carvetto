#pragma once

#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/document/bitmap_store.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/io/raster.hpp"
#include "carvetto/relief/relief.hpp"

// Bitmap to relief component, zero plane and Scale Model Z (T-0284, ADR-0076,
// docs/adr/0076-bitmap-relief.md). Units are mm. Headless; no UI.

namespace carvetto::relief {

/// 16-bit grayscale image, row 0 at the top. 0 = black, 65535 = white.
struct GrayImage16 {
    int width{0};
    int height{0};
    std::vector<std::uint16_t> px;  ///< width*height samples.

    bool valid() const {
        return width > 0 && height > 0 && width <= io::kMaxImageEdge && height <= io::kMaxImageEdge &&
               static_cast<std::size_t>(width) * static_cast<std::size_t>(height) <= io::kMaxImagePixels &&
               px.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
    }
};

/// 8-bit RGB -> 16-bit luminance (Rec.601 weights, exact for gray input: 8-bit v -> v*257).
geometry::ShapeResult<GrayImage16> grayFromRaster(const io::RasterImage& img);

/// Decodes encoded image bytes keeping 16-bit depth: Netpbm P2/P3/P5/P6 are read here at full
/// depth (maxval up to 65535, colour converted to luminance); P1/P4 and every other format go
/// through `document::decodeBitmapBytes` (8-bit) and `grayFromRaster`. Errors, never a crash.
geometry::ShapeResult<GrayImage16> decodeGray16(std::span<const std::uint8_t> bytes,
                                                const document::BitmapDecoder& decoder = {});

/// Decodes the image embedded in the document for `obj` (the `.carv` bitmap store).
geometry::ShapeResult<GrayImage16> loadBitmapObjectGray16(const document::Document& doc,
                                                          const document::BitmapObject& obj,
                                                          const document::BitmapDecoder& decoder = {});

/// Reads an image file (size capped at `document::kMaxEmbeddedBitmapBytes`) and decodes it.
geometry::ShapeResult<GrayImage16> loadFileGray16(const std::string& path,
                                                  const document::BitmapDecoder& decoder = {});

struct BitmapReliefParams {
    std::string name = "Bitmap";
    double widthMm = 0.0;        ///< Model width of the image; pixels are square, so height follows.
    double zHeightMm = 0.0;      ///< Height of white (or of the brightest pixel with autoScale).
    Point corner;                ///< Lower-left corner of the image rectangle.
    bool invert = false;         ///< White -> 0, black -> zHeightMm.
    bool autoScale = false;      ///< Stretch the image's own min..max to 0..zHeightMm.
    CombineMode mode = CombineMode::Add;
};

/// Brightness -> height: h = zHeightMm * (p - lo) / (hi - lo) with (lo, hi) = (0, 65535), or the
/// image's min and max when autoScale (a uniform image then gives 0). invert replaces p by
/// 65535 - p first. One cell per pixel, cellSize = widthMm / width, pixel row 0 -> top row.
/// BadParameter for non-finite or non-positive widthMm / zHeightMm or an invalid image.
Status heightFieldFromBitmap(const GrayImage16& img, const BitmapReliefParams& p, HeightField& out);

/// `heightFieldFromBitmap` wrapped as a component (mode from the params, 100 % height, base 0).
Status makeBitmapComponent(const GrayImage16& img, const BitmapReliefParams& p, Component& out);

/// Zero plane: a flat height-0 component whose cell-centre rectangle covers the job rectangle
/// [0, jobWidth] x [0, jobHeight] shifted by `corner` (cells = ceil(size / cell) + 1 per axis,
/// first centre on the corner). Mode Add.
Status makeZeroPlane(double jobWidthMm, double jobHeightMm, double cellSizeMm, Point corner, Component& out);

struct ScaleZ {
    enum class Kind { Percent, ExactHeight };
    Kind kind = Kind::Percent;
    double value = 100.0;   ///< Percent (> 0) or target height in mm (> 0).
    bool bothSides = false; ///< Also scale the bottom side (values below the zero plane).
};

/// Scales the field's Z about the zero plane (Z = 0). Top side = values >= 0; bottom side =
/// values < 0, scaled only with bothSides. Percent multiplies by value/100. ExactHeight scales
/// so that top-only: max = value; bothSides: (max(hi,0) - min(lo,0)) = value; error when that
/// extent is zero. Transparent cells stay transparent. `factorOut` (optional) gets the factor.
Status scaleModelZ(HeightField& field, const ScaleZ& s, double* factorOut = nullptr);

}  // namespace carvetto::relief
