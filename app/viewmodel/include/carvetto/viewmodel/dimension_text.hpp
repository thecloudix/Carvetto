#pragma once

#include <string>
#include <vector>

#include "carvetto/document/dimension.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"

// Dimension text as outlines (T-0255). Reuses the text core: the text is laid out with the font
// the dimension names (`DimensionObject::fontKey`, one of the document's embedded fonts) or, when
// that is empty, missing or unreadable, the compiled-in starter font. The result is closed
// contours in sheet millimetres, centred on the dimension's text position and turned to the text
// angle; the canvas fills them and the DXF export writes them as geometry. Empty when the text
// is empty, the dimension is degenerate, or no font can lay it out (the canvas then falls back to
// the toolkit's own text drawing).

namespace carvetto::viewmodel {

/// Digit height of the text relative to `textHeightMm` is 1; the em size is this much larger.
inline constexpr double kDimensionCapHeightRatio = 0.73;

/// Outline of `text` set per `dim` (size, font) around `geometry.textCentre`.
std::vector<geometry::Contour> dimensionTextContours(const document::Document& doc,
                                                     const document::DimensionObject& dim,
                                                     const document::DimensionGeometry& geometry,
                                                     const std::string& text);

/// Text of the dimension (custom or measured, in the document's display units) as outlines; the
/// geometry is built here.
std::vector<geometry::Contour> dimensionTextContours(const document::Document& doc,
                                                     const document::DimensionObject& dim);

/// Display name of the font a dimension uses: the embedded font's family name or the starter
/// font's name.
std::string dimensionFontName(const document::Document& doc, const std::string& fontKey);

}  // namespace carvetto::viewmodel
