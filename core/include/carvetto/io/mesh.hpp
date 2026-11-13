#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/geometry/shapes.hpp"

/// Triangle-mesh reading and orientation (T-0285, ADR-0063). Qt-free, units are mm once the unit
/// scale has been applied. Readers take the file bytes (no file access, so they are headless and
/// fuzzable) and treat the input as hostile: byte, vertex and triangle caps, counts checked against
/// the real size before any allocation, NaN/inf and absurd magnitudes refused, truncation reported.
namespace carvetto::io {

struct Vec3 {
    double x{0}, y{0}, z{0};
    friend bool operator==(const Vec3& a, const Vec3& b) { return a.x == b.x && a.y == b.y && a.z == b.z; }
};

/// Indexed triangle mesh; counter-clockwise (outward normal by the right-hand rule) as read.
struct TriangleMesh {
    std::vector<Vec3> vertices;
    std::vector<std::array<std::uint32_t, 3>> triangles;
};

/// Axis-aligned bounds. `empty` when the mesh has no vertices.
struct MeshBounds {
    bool empty{true};
    Vec3 min, max;
    Vec3 size() const { return {max.x - min.x, max.y - min.y, max.z - min.z}; }
    Vec3 centre() const { return {(min.x + max.x) / 2, (min.y + max.y) / 2, (min.z + max.z) / 2}; }
};

MeshBounds meshBounds(const TriangleMesh& mesh);

/// Reader limits. Exceeding one is an error, never a partial mesh.
struct MeshLimits {
    std::size_t maxBytes{512u * 1024u * 1024u};
    std::size_t maxVertices{20'000'000};
    std::size_t maxTriangles{10'000'000};
    double maxCoordinate{1e9};  ///< |coordinate| above this is refused
};

/// Binary or ASCII is detected: a file whose size equals 84 + 50 * (count at byte 80) is binary,
/// otherwise a file that starts with "solid" is ASCII. Vertices are welded by exact coordinates.
geometry::ShapeResult<TriangleMesh> readStl(std::string_view bytes, const MeshLimits& limits = {});

/// Wavefront OBJ: `v` and `f` only (`f` may use v, v/vt, v//vn, v/vt/vn, negative indices);
/// polygons are fan-triangulated from their first vertex; everything else (materials, groups,
/// normals, texture coordinates, smoothing, lines, points) is ignored.
geometry::ShapeResult<TriangleMesh> readObj(std::string_view bytes, const MeshLimits& limits = {});

enum class MeshFormat { Stl, Obj };

/// Format from a file name's extension (case-insensitive ".stl", ".obj"); false when unsupported
/// (3DS, DXF mesh and SKP are deferred, ADR-0063).
bool meshFormatFromPath(std::string_view path, MeshFormat& out);

geometry::ShapeResult<TriangleMesh> readMesh(std::string_view bytes, MeshFormat format,
                                             const MeshLimits& limits = {});

/// Reads `path` (size checked against `limits.maxBytes` before reading) and dispatches on the
/// extension.
geometry::ShapeResult<TriangleMesh> readMeshFile(const std::string& path, const MeshLimits& limits = {});

// ---- Orientation and sizing (all in place; an empty string result means success) -----------------

enum class MeshUnits { Millimetres, Inches };

/// Scales by 1 (mm) or 25.4 (inches). Error if a coordinate would leave the finite/1e9 range.
std::string applyMeshUnits(TriangleMesh& mesh, MeshUnits units);

/// Moves the bounding box centre to the origin in X, Y and Z.
void centerMesh(TriangleMesh& mesh);

/// Which model face becomes the top (+Z) face.
enum class TopFace { PlusZ, MinusZ, PlusX, MinusX, PlusY, MinusY };

/// Initial orientation: `top` is rotated onto +Z by a proper rotation (exact, winding kept), then
/// the model is turned `zQuarterTurns` x 90 degrees counter-clockwise about Z (0..3).
struct MeshOrientation {
    TopFace top{TopFace::PlusZ};
    int zQuarterTurns{0};
};
std::string orientMesh(TriangleMesh& mesh, const MeshOrientation& o);

/// Rotation by `angle` radians (counter-clockwise looking down the axis towards the origin) about
/// an arbitrary axis through the origin. Error for a zero or non-finite axis or angle.
std::string rotateMesh(TriangleMesh& mesh, const Vec3& axis, double angle);

/// Scales to a target size. With `lockRatio` the whole mesh is scaled uniformly so that the axis
/// `driver` (0 = X, 1 = Y, 2 = Z) reaches `target` (other components of `target` ignored);
/// otherwise each axis is scaled to its own component. Scaling about the bounds minimum.
/// Error for an empty mesh, a non-positive/non-finite target or a zero-size axis that is scaled.
std::string scaleMeshToSize(TriangleMesh& mesh, const Vec3& target, bool lockRatio, int driver);

}  // namespace carvetto::io
