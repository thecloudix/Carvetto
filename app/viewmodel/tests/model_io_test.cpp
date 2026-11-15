#include "carvetto/viewmodel/model_io.hpp"

#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <memory>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/relief_tree.hpp"

using namespace carvetto;
using namespace carvetto::viewmodel;

namespace {

void addBox(io::TriangleMesh& m, double x0, double y0, double z0, double x1, double y1, double z1) {
    const auto base = static_cast<std::uint32_t>(m.vertices.size());
    for (int k = 0; k < 8; ++k) {
        m.vertices.push_back({(k & 1) ? x1 : x0, (k & 2) ? y1 : y0, (k & 4) ? z1 : z0});
    }
    const std::uint32_t t[12][3] = {{0, 2, 3}, {0, 3, 1}, {4, 5, 7}, {4, 7, 6}, {0, 1, 5}, {0, 5, 4},
                                    {2, 6, 7}, {2, 7, 3}, {0, 4, 6}, {0, 6, 2}, {1, 3, 7}, {1, 7, 5}};
    for (const auto& tri : t) {
        m.triangles.push_back({base + tri[0], base + tri[1], base + tri[2]});
    }
}

io::TriangleMesh block() {
    io::TriangleMesh m;
    addBox(m, 0, 0, 0, 10, 10, 5);
    return m;
}

/// A stalk with a wider cap: the cap overhangs.
io::TriangleMesh mushroom() {
    io::TriangleMesh m;
    addBox(m, 4, 4, 0, 6, 6, 3);
    addBox(m, 0, 0, 3, 10, 10, 5);
    return m;
}

MeshImportSettings settings() {
    MeshImportSettings s;
    s.cellSize = 1.0;
    return s;
}

}  // namespace

TEST_CASE("prepare mesh applies units, orientation, size and centring", "[model_io]") {
    MeshImportSettings s = settings();
    auto p = prepareMesh(block(), s);
    REQUIRE(p.ok());
    io::MeshBounds b = io::meshBounds(p.value());
    CHECK(b.centre().x == 0.0);
    CHECK(b.centre().y == 0.0);
    CHECK(b.min.z == 0.0);  // Z is left to the zero plane

    s.units = io::MeshUnits::Inches;
    b = io::meshBounds(prepareMesh(block(), s).value());
    CHECK(b.size().x == 254.0);

    s = settings();
    s.orientation.top = io::TopFace::PlusX;
    b = io::meshBounds(prepareMesh(block(), s).value());
    CHECK(b.size().z == 10.0);
    CHECK(b.size().x == 5.0);

    s = settings();
    s.resize = true;
    s.driver = 0;
    s.targetSize.x = 20.0;
    b = io::meshBounds(prepareMesh(block(), s).value());
    CHECK(b.size().x == 20.0);
    CHECK(b.size().y == 20.0);
    CHECK(b.size().z == 10.0);
    s.lockRatio = false;
    s.targetSize = {20.0, 10.0, 1.0};
    b = io::meshBounds(prepareMesh(block(), s).value());
    CHECK(b.size().y == 10.0);
    CHECK(b.size().z == 1.0);

    s.targetSize = {0.0, 10.0, 1.0};
    CHECK(!prepareMesh(block(), s).ok());
    CHECK(!prepareMesh(io::TriangleMesh{}, settings()).ok());
}

TEST_CASE("preview highlights undercuts and follows the zero plane", "[model_io]") {
    MeshImportSettings s = settings();
    MeshImportPreview p = previewMeshImport(mushroom(), s);
    REQUIRE(p.ok);
    CHECK(p.undercutCells > 0);
    CHECK(p.maxHeight == 5.0);
    CHECK(p.zeroPlane == 0.0);
    CHECK(p.image.size() == p.imageWidth * p.imageHeight);
    bool red = false;
    for (const std::uint32_t px : p.image) {
        red = red || px == kUndercutColour;
    }
    CHECK(red);

    p = previewMeshImport(block(), s);
    REQUIRE(p.ok);
    CHECK(p.undercutCells == 0);

    // Standing the block on its side changes the material needed.
    MeshImportSettings side = s;
    side.orientation.top = io::TopFace::PlusX;
    p = previewMeshImport(block(), side);
    REQUIRE(p.ok);
    CHECK(p.maxHeight == 10.0);

    // Raising the zero plane lowers the height; the data below the plane is discarded.
    s.zeroAtBottom = false;
    s.zeroPlane = 2.0;
    p = previewMeshImport(block(), s);
    REQUIRE(p.ok);
    CHECK(p.maxHeight == 3.0);
    CHECK(p.zeroPlane == 2.0);

    // The preview never exceeds the requested resolution.
    s = settings();
    s.cellSize = 0.01;
    p = previewMeshImport(block(), s, 32);
    REQUIRE(p.ok);
    CHECK(p.gridWidth <= 33);

    s.cellSize = -1.0;
    CHECK(!previewMeshImport(block(), s).ok);
}

TEST_CASE("importing a mesh adds one undoable component", "[model_io]") {
    document::Document doc(document::Job::singleSided(100.0, 100.0, 18.0, document::Units::Millimetres));
    const document::SheetId sheet = doc.activeSheet();
    const std::size_t before = doc.undoStack().index();
    const MeshImportOutcome r = importMeshIntoDocument(doc, sheet, mushroom(), settings(), "Cap");
    REQUIRE(r.ok);
    CHECK(r.undercutCells > 0);
    REQUIRE(doc.reliefLevels().size() == 1);
    CHECK(doc.findReliefComponent(r.component)->data.name == "Cap");
    CHECK(doc.undoStack().index() == before + 1);
    CHECK(doc.componentSelection().contains(r.component));
    REQUIRE(doc.undoStack().undo(doc));
    CHECK(doc.reliefComponentCount() == 0);
    CHECK(doc.reliefLevels().empty());

    MeshImportSettings bad = settings();
    bad.cellSize = 0.0;
    CHECK(!importMeshIntoDocument(doc, sheet, block(), bad, "x").ok);
    CHECK(doc.reliefLevels().empty());
}

TEST_CASE("model export encodes STL, OBJ and heightmaps of the composed model", "[model_io]") {
    document::Document doc(document::Job::singleSided(100.0, 100.0, 18.0, document::Units::Millimetres));
    const document::SheetId sheet = doc.activeSheet();
    ModelExportSettings e;
    CHECK(!exportSheetModel(doc, sheet, e).ok);  // nothing to export

    REQUIRE(importMeshIntoDocument(doc, sheet, block(), settings(), "Block").ok);

    e.kind = ModelExportKind::Stl;
    ModelExportResult r = exportSheetModel(doc, sheet, e);
    REQUIRE(r.ok);
    CHECK(r.triangles > 0);
    CHECK(r.data.size() == 84 + 50 * r.triangles);
    auto back = io::readStl(r.data);
    REQUIRE(back.ok());
    CHECK(back.value().triangles.size() == r.triangles);
    // Flat back: the lowest vertex is the default back Z (0, the model is entirely above it).
    CHECK(io::meshBounds(back.value()).min.z == 0.0);

    e.back = io::BackFace::Open;
    const ModelExportResult open = exportSheetModel(doc, sheet, e);
    REQUIRE(open.ok);
    CHECK(open.triangles < r.triangles);

    e.back = io::BackFace::Flat;
    e.backAuto = false;
    e.backZ = -3.0;
    r = exportSheetModel(doc, sheet, e);
    REQUIRE(r.ok);
    CHECK(io::meshBounds(io::readStl(r.data).value()).min.z == -3.0);
    e.backZ = 100.0;  // not below the model
    CHECK(!exportSheetModel(doc, sheet, e).ok);

    e = ModelExportSettings{};
    e.kind = ModelExportKind::Obj;
    r = exportSheetModel(doc, sheet, e);
    REQUIRE(r.ok);
    CHECK(io::readObj(r.data).ok());

    e.kind = ModelExportKind::Heightmap;
    r = exportSheetModel(doc, sheet, e);
    REQUIRE(r.ok);
    CHECK(r.data.substr(1, 3) == "PNG");
    e.heightmap = io::HeightmapFormat::Pgm16;
    r = exportSheetModel(doc, sheet, e);
    REQUIRE(r.ok);
    CHECK(r.data.substr(0, 2) == "P5");
    CHECK(std::string(exportExtension(e.kind, e.heightmap)) == ".pgm");
    CHECK(std::string(exportExtension(ModelExportKind::Stl, e.heightmap)) == ".stl");
}

TEST_CASE("default back Z is strictly below the lowest cell", "[model_io]") {
    relief::HeightField f;
    REQUIRE(relief::HeightField::create(relief::Grid{3, 3, 1.0, {0, 0}}, 2.0f, f) == relief::Status::Ok);
    CHECK(defaultBackZ(f) == 0.0);
    REQUIRE(relief::HeightField::create(relief::Grid{3, 3, 1.0, {0, 0}}, -2.0f, f) == relief::Status::Ok);
    CHECK(defaultBackZ(f) == -3.0);
    CHECK(defaultBackZ(relief::HeightField{}) == 0.0);
}
