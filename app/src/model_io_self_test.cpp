// Self-test of the 3D import and export dialogs (T-0301): the Orientate Model dialog with its
// preview and undercut highlight, the import and export through the real MainWindow, and the
// files written.

#include <QDialogButtonBox>
#include <QFile>
#include <QLabel>
#include <QPushButton>
#include <QTemporaryDir>
#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/io/export3d.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "main_window.hpp"
#include "model_io_dialogs.hpp"
#include "relief_panels.hpp"

namespace carvetto::app {

namespace {

#define MI_REQUIRE(cond, text)                               \
    do {                                                     \
        if (!(cond)) {                                       \
            return std::string("model import/export: ") + (text); \
        }                                                    \
    } while (false)

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

std::string readAll(const std::filesystem::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::ostringstream s;
    s << in.rdbuf();
    return s.str();
}

}  // namespace

std::string modelIoSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(100.0, 100.0, 18.0, document::Units::Millimetres)));
    QTemporaryDir dir;
    MI_REQUIRE(dir.isValid(), "no temporary directory");
    const std::filesystem::path root = viewmodel::pathFromUtf8(dir.path().toStdString());

    io::TriangleMesh mushroom;
    addBox(mushroom, 4, 4, 0, 6, 6, 3);
    addBox(mushroom, 0, 0, 3, 10, 10, 5);
    const auto stl = io::writeStlBinary(mushroom);
    MI_REQUIRE(stl.ok(), "cannot build the test model");
    const std::filesystem::path stlPath = root / "cap.stl";
    {
        std::ofstream out(stlPath, std::ios::binary);
        out.write(stl.value().data.data(), static_cast<std::streamsize>(stl.value().data.size()));
    }

    // --- Orientate Model dialog -----------------------------------------------------------------
    MeshImportDialog dialog(mushroom, QStringLiteral("cap.stl"), &window);
    MI_REQUIRE(dialog.componentName() == "cap", "the component name is not the file name");
    MI_REQUIRE(dialog.preview().ok && dialog.preview().undercutCells > 0, "the undercut is not detected");
    MI_REQUIRE(dialog.undercutLabel()->text().contains(QStringLiteral("Undercuts")), "the undercut is not reported");
    MI_REQUIRE(dialog.previewLabel()->pixmap().width() > 0, "the preview is not drawn");
    viewmodel::MeshImportSettings s = dialog.settings();
    s.orientation.top = io::TopFace::PlusX;  // stand it on its side: 10 mm of material above the plane
    dialog.setSettings(s);
    MI_REQUIRE(dialog.settings().orientation.top == io::TopFace::PlusX, "the top face choice is not kept");
    MI_REQUIRE(dialog.preview().ok && std::abs(dialog.preview().maxHeight - 10.0) < 1e-3,
               "the preview does not follow the orientation");
    s = dialog.settings();
    s.orientation.top = io::TopFace::PlusZ;
    s.zeroAtBottom = false;
    s.zeroPlane = 3.0;
    dialog.setSettings(s);
    MI_REQUIRE(std::abs(dialog.preview().maxHeight - 2.0) < 1e-3, "the zero plane does not change the height");
    // A model without triangles cannot be imported: the preview reports it and OK is off.
    MeshImportDialog empty(io::TriangleMesh{}, QStringLiteral("empty.stl"), &window);
    MI_REQUIRE(!empty.preview().ok && !empty.buttons()->button(QDialogButtonBox::Ok)->isEnabled(),
               "an empty model leaves OK enabled");

    // --- Import through the window ---------------------------------------------------------------
    document::Document* doc = &window.currentDocument();
    const std::size_t steps = doc->undoStack().count();
    s = viewmodel::MeshImportSettings{};
    s.cellSize = 1.0;
    MI_REQUIRE(window.importMeshFile(stlPath, s), "importing the model failed");
    MI_REQUIRE(doc->reliefComponentCount() == 1 && doc->undoStack().count() == steps + 1,
               "the import is not one component and one undo step");
    window.documentChanged();
    MI_REQUIRE(window.modelView()->surface().ok(), "the imported model is not shown");
    MI_REQUIRE(!window.importMeshFile(root / "missing.stl", s) && !window.lastModelIoError().empty(),
               "a missing file was imported");

    // --- Export ----------------------------------------------------------------------------------
    viewmodel::ModelExportSettings e;
    e.kind = viewmodel::ModelExportKind::Stl;
    MI_REQUIRE(window.exportModelFile(root / "out.stl", e), "exporting STL failed");
    const auto back = io::readStl(readAll(root / "out.stl"));
    MI_REQUIRE(back.ok() && !back.value().triangles.empty(), "the exported STL cannot be read");
    e.kind = viewmodel::ModelExportKind::Obj;
    MI_REQUIRE(window.exportModelFile(root / "out.obj", e) && io::readObj(readAll(root / "out.obj")).ok(),
               "exporting OBJ failed");
    e.kind = viewmodel::ModelExportKind::Heightmap;
    MI_REQUIRE(window.exportModelFile(root / "out.png", e), "exporting the heightmap failed");
    MI_REQUIRE(readAll(root / "out.png").substr(1, 3) == "PNG", "the heightmap is not a PNG");
    MI_REQUIRE(!window.exportModelFile(root / "no_dir" / "x.stl", viewmodel::ModelExportSettings{}),
               "writing into a missing folder succeeded");

    MI_REQUIRE(window.undo() && doc->reliefComponentCount() == 0, "undoing the import failed");
    MI_REQUIRE(!window.exportModelFile(root / "empty.stl", viewmodel::ModelExportSettings{}),
               "an empty model was exported");

    // Dialog for export options.
    ModelExportDialog options(viewmodel::ModelExportKind::Stl, -1.0, &window);
    MI_REQUIRE(options.settings().backAuto && options.settings().back == io::BackFace::Flat,
               "the export defaults are wrong");
    viewmodel::ModelExportSettings custom;
    custom.backAuto = false;
    custom.backZ = -4.0;
    custom.back = io::BackFace::Open;
    options.setSettings(custom);
    MI_REQUIRE(options.settings().back == io::BackFace::Open && options.settings().backZ == -4.0,
               "the export options are not kept");
    return {};
}

}  // namespace carvetto::app
