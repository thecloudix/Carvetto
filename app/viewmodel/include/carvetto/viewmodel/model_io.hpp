#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/geometry/shapes.hpp"
#include "carvetto/io/export3d.hpp"
#include "carvetto/io/mesh.hpp"
#include "carvetto/relief/mesh_raster.hpp"
#include "carvetto/viewmodel/properties_model.hpp"

// Qt-free logic of the 3D import and export dialogs (T-0301, ADR-0081): the Orientate Model form
// (units, top face, rotation, size with lock, centring, zero plane with undercut highlight) and
// the export of the composed model as STL, OBJ or a 16-bit heightmap. Stateless functions; the
// document is only touched by `importMeshIntoDocument` (one undo step).

namespace carvetto::viewmodel {

/// Everything the Orientate Model dialog edits. Lengths in millimetres (after `units`).
struct MeshImportSettings {
    io::MeshUnits units{io::MeshUnits::Millimetres};  ///< Units of the file's coordinates.
    io::MeshOrientation orientation;                  ///< Which face is the top, quarter turns about Z.
    /// Scale to a target size. With `lockRatio` only the `driver` axis (0 X, 1 Y, 2 Z) of
    /// `targetSize` counts and the others follow.
    bool resize{false};
    bool lockRatio{true};
    int driver{0};
    io::Vec3 targetSize{100.0, 100.0, 10.0};
    bool centre{true};  ///< Bounding box centre to the origin (XY; Z is settled by the zero plane).
    /// Zero plane: model Z (after orientation, sizing and centring) that becomes height 0. When
    /// `zeroAtBottom` it is the lowest point of the model and `zeroPlane` is ignored.
    bool zeroAtBottom{true};
    double zeroPlane{0.0};
    bool discardBelowZero{true};
    double cellSize{0.5};
};

/// The oriented mesh (units, orientation, size, centring applied in that order) or an error.
geometry::ShapeResult<io::TriangleMesh> prepareMesh(const io::TriangleMesh& original, const MeshImportSettings& s);

/// Zero plane in the prepared mesh's coordinates for `s`.
double effectiveZeroPlane(const io::MeshBounds& preparedBounds, const MeshImportSettings& s);

/// What the dialog shows for the current settings.
struct MeshImportPreview {
    bool ok{false};
    std::string error;
    io::Vec3 size;               ///< Size of the prepared mesh.
    double zeroPlane{0.0};       ///< Effective zero plane (prepared mesh Z).
    double maxHeight{0.0};       ///< Material thickness above the plane the model needs.
    std::size_t undercutCells{0};
    std::size_t gridWidth{0}, gridHeight{0};
    double cellSize{0.0};        ///< Cell size actually used (preview may be coarser than asked).
    /// Top-down picture, row 0 at the top, 0xAARRGGBB: grey by height, undercut cells red,
    /// cells without surface background.
    std::size_t imageWidth{0}, imageHeight{0};
    std::vector<std::uint32_t> image;
};

inline constexpr std::uint32_t kUndercutColour = 0xFFD83020u;
inline constexpr std::uint32_t kPreviewBackground = 0xFFE2E4E8u;

/// Rasterises the settings for display. The cell size is raised so that the longer side has at most
/// `maxPreviewCells` cells (the import itself uses `s.cellSize` as given).
MeshImportPreview previewMeshImport(const io::TriangleMesh& original, const MeshImportSettings& s,
                                    std::size_t maxPreviewCells = 256);

struct MeshImportOutcome {
    bool ok{false};
    std::string error;
    document::ComponentId component;
    std::size_t undercutCells{0};
};

/// Rasterises at `s.cellSize` and adds the result as a component named `name` on top of the
/// topmost level of `sheet` (a level is created when the sheet has none), selected; one undo step.
MeshImportOutcome importMeshIntoDocument(document::Document& doc, document::SheetId sheet,
                                         const io::TriangleMesh& original, const MeshImportSettings& s,
                                         const std::string& name);

/// Adds `component` on top of the topmost level of `sheet` (a level is created when the sheet has none), selected;
/// one undo step named `undoText`. Shared by the flat and the rotary import.
MeshImportOutcome addImportedComponent(document::Document& doc, document::SheetId sheet, relief::Component component,
                                       const char* undoText);

// --- Export --------------------------------------------------------------------------------------

enum class ModelExportKind { Stl, Obj, Heightmap };

struct ModelExportSettings {
    ModelExportKind kind{ModelExportKind::Stl};
    /// STL/OBJ: closed solid with a flat back, or the top surface only.
    io::BackFace back{io::BackFace::Flat};
    bool backAuto{true};  ///< Use `defaultBackZ` of the model.
    double backZ{0.0};
    io::HeightmapFormat heightmap{io::HeightmapFormat::Png16};
};

/// File extension (with the dot) of a kind and the file-dialog filter text.
const char* exportExtension(ModelExportKind kind, io::HeightmapFormat heightmap);
const char* exportFilter(ModelExportKind kind);

/// A back Z strictly below the lowest cell: 0 when the model is entirely above 0, otherwise one
/// millimetre below the lowest cell.
double defaultBackZ(const relief::HeightField& field);

struct ModelExportResult {
    bool ok{false};
    std::string error;
    std::string data;
    std::size_t triangles{0};
};

/// Composes the visible model of `sheet` at full resolution and encodes it.
ModelExportResult exportSheetModel(const document::Document& doc, document::SheetId sheet,
                                   const ModelExportSettings& settings);

}  // namespace carvetto::viewmodel
