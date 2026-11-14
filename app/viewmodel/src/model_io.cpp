#include "carvetto/viewmodel/model_io.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "carvetto/document/relief_commands.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"

namespace carvetto::viewmodel {

namespace {

bool finitePositive(double v) { return std::isfinite(v) && v > 0.0; }

}  // namespace

geometry::ShapeResult<io::TriangleMesh> prepareMesh(const io::TriangleMesh& original, const MeshImportSettings& s) {
    io::TriangleMesh mesh = original;
    if (mesh.vertices.empty() || mesh.triangles.empty()) {
        return std::string("The model has no triangles.");
    }
    if (std::string e = io::applyMeshUnits(mesh, s.units); !e.empty()) {
        return e;
    }
    if (std::string e = io::orientMesh(mesh, s.orientation); !e.empty()) {
        return e;
    }
    if (s.resize) {
        if (std::string e = io::scaleMeshToSize(mesh, s.targetSize, s.lockRatio, s.driver); !e.empty()) {
            return e;
        }
    }
    if (s.centre) {
        const io::MeshBounds b = io::meshBounds(mesh);
        const io::Vec3 c = b.centre();
        for (io::Vec3& v : mesh.vertices) {
            v.x -= c.x;
            v.y -= c.y;
        }
    }
    return mesh;
}

double effectiveZeroPlane(const io::MeshBounds& b, const MeshImportSettings& s) {
    return s.zeroAtBottom ? b.min.z : s.zeroPlane;
}

namespace {

std::string checkSettings(const MeshImportSettings& s) {
    if (!finitePositive(s.cellSize)) {
        return "The cell size must be a positive number.";
    }
    if (!s.zeroAtBottom && !std::isfinite(s.zeroPlane)) {
        return "The zero plane must be a number.";
    }
    if (s.resize) {
        const double t = s.driver == 0 ? s.targetSize.x : s.driver == 1 ? s.targetSize.y : s.targetSize.z;
        if (s.lockRatio && !finitePositive(t)) {
            return "The size must be a positive number.";
        }
        if (!s.lockRatio && !(finitePositive(s.targetSize.x) && finitePositive(s.targetSize.y) &&
                              finitePositive(s.targetSize.z))) {
            return "The sizes must be positive numbers.";
        }
    }
    return {};
}

relief::MeshRasterOptions rasterOptions(const io::MeshBounds& b, const MeshImportSettings& s, double cell) {
    relief::MeshRasterOptions o;
    o.cellSize = cell;
    o.zeroPlane = effectiveZeroPlane(b, s);
    o.discardBelowZero = s.discardBelowZero;
    o.bothSides = false;
    o.detectUndercuts = true;
    return o;
}

}  // namespace

MeshImportPreview previewMeshImport(const io::TriangleMesh& original, const MeshImportSettings& s,
                                    std::size_t maxPreviewCells) {
    MeshImportPreview out;
    if (std::string e = checkSettings(s); !e.empty()) {
        out.error = std::move(e);
        return out;
    }
    auto prepared = prepareMesh(original, s);
    if (!prepared.ok()) {
        out.error = prepared.error();
        return out;
    }
    const io::MeshBounds b = io::meshBounds(prepared.value());
    out.size = b.size();
    out.zeroPlane = effectiveZeroPlane(b, s);
    const double longer = std::max(out.size.x, out.size.y);
    const double limit = static_cast<double>(std::max<std::size_t>(maxPreviewCells, 2));
    const double cell = std::max(s.cellSize, longer / limit);
    auto raster = relief::meshToHeightField(prepared.value(), rasterOptions(b, s, cell));
    if (!raster.ok()) {
        out.error = raster.error();
        return out;
    }
    const relief::MeshRaster& r = raster.value();
    out.cellSize = cell;
    out.gridWidth = r.top.width();
    out.gridHeight = r.top.height();
    out.maxHeight = r.maxHeight;
    out.undercutCells = r.undercutCount;
    out.imageWidth = out.gridWidth;
    out.imageHeight = out.gridHeight;
    out.image.assign(out.imageWidth * out.imageHeight, kPreviewBackground);
    for (std::size_t j = 0; j < out.gridHeight; ++j) {
        const std::size_t row = out.gridHeight - 1 - j;  // grid row 0 is the lowest y
        for (std::size_t i = 0; i < out.gridWidth; ++i) {
            const float h = r.top.at(i, j);
            std::uint32_t& px = out.image[row * out.imageWidth + i];
            if (!r.undercut.empty() && r.undercut[j * out.gridWidth + i] != 0) {
                px = kUndercutColour;
            } else if (!std::isnan(h)) {
                const double t = r.maxHeight > 0.0 ? std::clamp(static_cast<double>(h) / r.maxHeight, 0.0, 1.0) : 0.5;
                const auto g = static_cast<std::uint32_t>(40.0 + t * 200.0);
                px = 0xFF000000u | (g << 16) | (g << 8) | g;
            }
        }
    }
    out.ok = true;
    return out;
}

MeshImportOutcome importMeshIntoDocument(document::Document& doc, document::SheetId sheet,
                                         const io::TriangleMesh& original, const MeshImportSettings& s,
                                         const std::string& name) {
    MeshImportOutcome out;
    auto fail = [&out](std::string message) {
        out.ok = false;
        out.error = std::move(message);
        return out;
    };
    if (std::string e = checkSettings(s); !e.empty()) {
        return fail(std::move(e));
    }
    auto prepared = prepareMesh(original, s);
    if (!prepared.ok()) {
        return fail(prepared.error());
    }
    const io::MeshBounds b = io::meshBounds(prepared.value());
    auto raster = relief::meshToHeightField(prepared.value(), rasterOptions(b, s, s.cellSize));
    if (!raster.ok()) {
        return fail(raster.error());
    }
    relief::Component c;
    c.name = name.empty() ? "Model" : name;
    c.field = std::move(raster.value().top);
    out.undercutCells = raster.value().undercutCount;

    // A mesh in millimetres lands where it was centred; without centring the field keeps the
    // mesh's own XY (the grid origin is in model coordinates).
    std::vector<document::LevelId> levels = doc.reliefLevelsOnSheet(sheet);
    doc.undoStack().breakCoalescing();
    doc.undoStack().beginMacro("Import 3D model");
    document::LevelId level;
    if (levels.empty()) {
        EditResult added = addReliefLevel(doc, sheet, {}, &level);
        if (!added.ok) {
            doc.undoStack().endMacro();
            return fail(added.message);
        }
    } else {
        level = levels.back();
    }
    auto cmd = std::make_unique<document::AddReliefComponentCommand>(level, std::move(c));
    const document::AddReliefComponentCommand* raw = cmd.get();
    const bool done = doc.execute(std::move(cmd));
    const document::ComponentId id = done ? raw->createdId() : document::ComponentId{};
    doc.undoStack().endMacro();
    doc.undoStack().breakCoalescing();
    if (!done || !id) {
        return fail("The model could not be added to the document.");
    }
    selectReliefComponents(doc, {id});
    out.ok = true;
    out.component = id;
    return out;
}

const char* exportExtension(ModelExportKind kind, io::HeightmapFormat heightmap) {
    switch (kind) {
        case ModelExportKind::Stl:
            return ".stl";
        case ModelExportKind::Obj:
            return ".obj";
        case ModelExportKind::Heightmap:
            return heightmap == io::HeightmapFormat::Png16 ? ".png" : ".pgm";
    }
    return ".stl";
}

const char* exportFilter(ModelExportKind kind) {
    switch (kind) {
        case ModelExportKind::Stl:
            return "STL models (*.stl)";
        case ModelExportKind::Obj:
            return "Wavefront OBJ models (*.obj)";
        case ModelExportKind::Heightmap:
            return "16-bit heightmaps (*.png *.pgm)";
    }
    return "";
}

double defaultBackZ(const relief::HeightField& field) {
    float lo = 0.0f, hi = 0.0f;
    if (!field.minMax(lo, hi)) {
        return 0.0;
    }
    return lo > 0.0f ? 0.0 : static_cast<double>(lo) - 1.0;
}

ModelExportResult exportSheetModel(const document::Document& doc, document::SheetId sheet,
                                   const ModelExportSettings& settings) {
    ModelExportResult out;
    const std::vector<relief::Level> levels = doc.reliefModel(sheet);
    if (levels.empty()) {
        out.error = "This sheet has no relief levels.";
        return out;
    }
    relief::Grid grid;
    if (const relief::Status s = relief::unionGrid(levels, grid); s != relief::Status::Ok) {
        out.error = relief::statusMessage(s);
        return out;
    }
    relief::HeightField field;
    if (const relief::Status s = relief::compose(levels, grid, field); s != relief::Status::Ok) {
        out.error = relief::statusMessage(s);
        return out;
    }
    if (settings.kind == ModelExportKind::Heightmap) {
        io::HeightmapOptions o;
        o.format = settings.heightmap;
        auto bytes = io::writeHeightmap(field, o);
        if (!bytes.ok()) {
            out.error = bytes.error();
            return out;
        }
        out.data = std::move(bytes.value().data);
        out.ok = true;
        return out;
    }
    io::SurfaceMeshOptions mo;
    mo.back = settings.back;
    mo.backZ = settings.backAuto ? defaultBackZ(field) : settings.backZ;
    if (settings.back == io::BackFace::Flat && !std::isfinite(mo.backZ)) {
        out.error = "The back Z must be a number.";
        return out;
    }
    auto mesh = io::buildSurfaceMesh(field, mo);
    if (!mesh.ok()) {
        out.error = mesh.error();
        return out;
    }
    out.triangles = mesh.value().triangles.size();
    auto bytes = settings.kind == ModelExportKind::Stl ? io::writeStlBinary(mesh.value()) : io::writeObj(mesh.value());
    if (!bytes.ok()) {
        out.error = bytes.error();
        return out;
    }
    out.data = std::move(bytes.value().data);
    out.ok = true;
    return out;
}

}  // namespace carvetto::viewmodel
