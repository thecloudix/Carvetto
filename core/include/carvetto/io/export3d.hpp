#pragma once

#include <cstddef>
#include <optional>
#include <string>

#include "carvetto/geometry/shapes.hpp"
#include "carvetto/io/mesh.hpp"
#include "carvetto/relief/relief.hpp"

/// 3D export of the composed relief (T-0287, ADR-0075): triangle mesh from a height field, binary
/// STL and OBJ writers, and a 16-bit grayscale heightmap (PGM or PNG). Qt-free; the writers return
/// bytes (no file access) so they are headless and testable. Units are mm.
namespace carvetto::io {

/// File contents (a wrapper because ShapeResult<std::string> would be ambiguous).
struct ExportBytes {
    std::string data;
};

/// Back face of the exported surface.
enum class BackFace {
    Open,  ///< Top surface only.
    Flat,  ///< Closed solid: flat back plane at `backZ` plus vertical walls around the footprint.
};

struct SurfaceMeshOptions {
    BackFace back{BackFace::Open};
    double backZ{0.0};  ///< Z of the flat back; must be strictly below the lowest opaque cell.
    std::size_t maxTriangles{10'000'000};  ///< Exceeding it is an error (no silent decimation).
};

/// One vertex per opaque cell centre, two triangles per square of four opaque cells (counter-
/// clockwise seen from +Z, so the normal points up). Transparent cells leave holes; with
/// BackFace::Flat the hole boundaries get walls too, so the result is a closed manifold (every
/// edge used by exactly two triangles in opposite directions). Error when there is no square of
/// four opaque cells, when `backZ` is not below the lowest cell, or when the triangle cap is hit.
geometry::ShapeResult<TriangleMesh> buildSurfaceMesh(const relief::HeightField& field,
                                                     const SurfaceMeshOptions& opt = {});

/// Binary STL: 80-byte header, count, then per triangle a unit normal and three vertices as
/// float32 (50 bytes). Readable by `readStl`. Error for more than 2^32-1 triangles or bad indices.
geometry::ShapeResult<ExportBytes> writeStlBinary(const TriangleMesh& mesh);

/// Wavefront OBJ: `v x y z` lines (shortest round-trip decimal) and 1-based `f a b c` lines.
/// Readable by `readObj`. Error for out-of-range triangle indices.
geometry::ShapeResult<ExportBytes> writeObj(const TriangleMesh& mesh);

enum class HeightmapFormat { Pgm16, Png16 };

struct HeightmapOptions {
    HeightmapFormat format{HeightmapFormat::Png16};
    /// Heights mapped to black and white. Default: the lowest and highest opaque cell, so those
    /// cells are exactly 0 and 65535. A given range clamps values outside it.
    std::optional<double> rangeMin;
    std::optional<double> rangeMax;
};

/// 16-bit grayscale image, one pixel per cell, highest = white (65535), lowest = black (0), linear
/// in between (rounded to nearest); transparent cells are black. Row 0 is the top (largest y).
/// A zero-height range (flat model) gives an all-black image. Error for an empty/all-transparent
/// field, a non-finite or inverted range, or a field larger than `kMaxImageEdge`/`kMaxImagePixels`.
/// PNG is written with stored (uncompressed) deflate blocks, PGM as binary P5, maxval 65535.
geometry::ShapeResult<ExportBytes> writeHeightmap(const relief::HeightField& field,
                                                  const HeightmapOptions& opt = {});

}  // namespace carvetto::io
