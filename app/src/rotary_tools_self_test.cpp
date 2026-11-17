// Self-test of the rotary tools UI (T-0341): the Wrapping toggle of the level effects panel, the Vector Unwrapper
// form and the rotary 3D import dialog, driven through the real widgets of the MainWindow offscreen.

#include <QAbstractButton>
#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QMenu>
#include <QMenuBar>
#include <QPushButton>
#include <QLabel>
#include <QTemporaryDir>
#include <cmath>
#include <fstream>
#include <memory>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/io/mesh.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "main_window.hpp"
#include "rotary_tools_dialogs.hpp"
#include "sculpt_panel.hpp"

namespace carvetto::app {

namespace {

#define RTU_REQUIRE(cond, text)                         \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("rotary tools: ") + (text); \
        }                                               \
    } while (false)

io::TriangleMesh prism(int n, double rc, double x0, double x1) {
    constexpr double kTwoPi = 6.283185307179586;
    io::TriangleMesh m;
    for (const double x : {x0, x1}) {
        for (int k = 0; k < n; ++k) {
            const double a = kTwoPi * k / n;
            m.vertices.push_back({x, rc * std::sin(a), rc * std::cos(a)});
        }
    }
    m.vertices.push_back({x0, 0, 0});
    m.vertices.push_back({x1, 0, 0});
    const auto c0 = static_cast<std::uint32_t>(2 * n), c1 = c0 + 1;
    for (int k = 0; k < n; ++k) {
        const auto a = static_cast<std::uint32_t>(k), b = static_cast<std::uint32_t>((k + 1) % n);
        const auto a1 = a + static_cast<std::uint32_t>(n), b1 = b + static_cast<std::uint32_t>(n);
        m.triangles.push_back({a, b, b1});
        m.triangles.push_back({a, b1, a1});
        m.triangles.push_back({c0, b, a});
        m.triangles.push_back({c1, a1, b1});
    }
    return m;
}

}  // namespace

std::string rotaryToolsSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(document::Job::rotary(
        100.0, 40.0, document::Units::Millimetres, document::RotaryAxis::X, document::ZZero::CylinderAxis, false)));
    document::Document& doc = window.currentDocument();
    LevelEffectsPanel* effects = window.levelEffects();
    RTU_REQUIRE(effects != nullptr, "the level effects panel is missing");

    // The Setup menu entries exist and are enabled for the rotary job.
    QAction* unwrapAction = window.findChild<QAction*>(QStringLiteral("vectorUnwrapperAction"));
    QAction* importAction = window.findChild<QAction*>(QStringLiteral("rotaryImportAction"));
    RTU_REQUIRE(unwrapAction != nullptr && importAction != nullptr, "the Setup menu has no rotary tool entries");
    QMenu* setup = qobject_cast<QMenu*>(unwrapAction->parent());
    RTU_REQUIRE(setup != nullptr, "the rotary tool entries are not in a menu");
    emit setup->aboutToShow();
    RTU_REQUIRE(unwrapAction->isEnabled() && importAction->isEnabled(), "the rotary tools are off for a rotary job");

    // 1. Vector Unwrapper: nothing selected is refused; a selected circle unwraps through the form.
    std::string why;
    RTU_REQUIRE(!unwrapSelection(window, viewmodel::UnwrapperSettings{}, &why) && !why.empty(),
                "unwrapping an empty selection was not refused");
    document::Object o;
    o.layer = doc.defaultLayer();
    o.sheet = doc.activeSheet();
    document::VectorObject v;
    v.contours.push_back(geometry::Contour::circle({30.0, 20.0}, 5.0));
    o.data = std::move(v);
    auto add = std::make_unique<document::AddObjectCommand>(std::move(o));
    const document::AddObjectCommand* rawAdd = add.get();
    RTU_REQUIRE(doc.execute(std::move(add)), "adding the vector failed");
    doc.selection().assign({rawAdd->id()});
    UnwrapperDialog form(&window);
    viewmodel::UnwrapperSettings us;
    us.centre = geometry::UnwrapCentre::Custom;
    us.customCentre = {30.0, 20.0};
    us.tolerance = 0.02;
    form.setSettings(us);
    RTU_REQUIRE(form.findChild<QDoubleSpinBox*>(QStringLiteral("unwrapCentreX"))->isEnabled(),
                "the custom centre is not editable when chosen");
    const viewmodel::UnwrapperSettings got = form.settings();
    RTU_REQUIRE(got.centre == geometry::UnwrapCentre::Custom && std::abs(got.customCentre.x - 30.0) < 1e-9 &&
                    std::abs(got.tolerance - 0.02) < 1e-9 && got.refine,
                "the unwrapper form does not hold its settings");
    const std::size_t objects = doc.objects().size();
    const std::size_t steps = doc.undoStack().count();
    RTU_REQUIRE(unwrapSelection(window, got, &why), "unwrapping failed: " + why);
    RTU_REQUIRE(doc.objects().size() == objects + 1 && doc.undoStack().count() == steps + 1,
                "the unwrapper did not add one object in one undo step");

    // 2. Rotary 3D import through the dialog settings.
    QTemporaryDir dir;
    RTU_REQUIRE(dir.isValid(), "no temporary directory");
    const std::filesystem::path stlPath = viewmodel::pathFromUtf8(dir.path().toStdString()) / "pole.stl";
    const auto stl = io::writeStlBinary(prism(8, 5.0, 0.0, 20.0));
    RTU_REQUIRE(stl.ok(), "cannot build the test model");
    {
        std::ofstream out(stlPath, std::ios::binary);
        out.write(stl.value().data.data(), static_cast<std::streamsize>(stl.value().data.size()));
    }
    RotaryImportDialog importForm(&window);
    viewmodel::RotaryImportSettings is;
    is.mesh.cellSize = 1.0;
    is.fitToMaterial = false;
    is.centreAlong = false;
    is.alongOffset = 10.0;
    importForm.setSettings(is);
    const viewmodel::RotaryImportSettings isGot = importForm.settings();
    RTU_REQUIRE(isGot.type == viewmodel::RotaryModelType::Full3D && !isGot.centreAlong &&
                    std::abs(isGot.alongOffset - 10.0) < 1e-9 &&
                    importForm.findChild<QDoubleSpinBox*>(QStringLiteral("rotImportAlong"))->isEnabled(),
                "the import form does not hold its settings");
    RTU_REQUIRE(!importRotaryMeshFile(window, stlPath.parent_path() / "missing.stl", isGot, &why) && !why.empty(),
                "a missing model file was not refused");
    const std::size_t components = doc.reliefLevels().size();
    RTU_REQUIRE(importRotaryMeshFile(window, stlPath, isGot, &why), "the rotary import failed: " + why);
    RTU_REQUIRE(doc.reliefLevels().size() == components + 1, "the import did not make a level");
    const document::LevelId level = doc.reliefLevels().back().id;

    // 3. Wrapping toggle and Bake of the level effects panel.
    effects->setLevel(level);
    QCheckBox* box = effects->wrappingBox();
    RTU_REQUIRE(box != nullptr && box->isEnabled() && !box->isChecked() && !effects->bakeButton()->isEnabled(),
                "the Wrapping box is not ready");
    box->click();
    RTU_REQUIRE(doc.findReliefLevel(level)->effects.wrapping.enabled && box->isChecked(), "Wrapping did not turn on");
    RTU_REQUIRE(effects->summary()->text().contains(QStringLiteral("wrapped")), "the summary does not say wrapped");
    RTU_REQUIRE(effects->bakeButton()->isEnabled(), "Bake is off for a wrapping-only level");
    RTU_REQUIRE(effects->bake(), "baking the wrapping failed");
    RTU_REQUIRE(!doc.findReliefLevel(level)->effects.wrapping.enabled && !box->isChecked(),
                "the wrapping is still on after Bake");
    RTU_REQUIRE(doc.undo() && doc.findReliefLevel(level)->effects.wrapping.enabled, "Undo of Bake lost the wrapping");
    effects->refresh();
    box->click();
    RTU_REQUIRE(!doc.findReliefLevel(level)->effects.wrapping.enabled, "Wrapping did not turn off");

    // A flat job refuses everything and disables the menu entries.
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(100.0, 100.0, 18.0, document::Units::Millimetres)));
    emit setup->aboutToShow();
    RTU_REQUIRE(!unwrapAction->isEnabled() && !importAction->isEnabled(), "the rotary tools are on for a flat job");
    RTU_REQUIRE(!importRotaryMeshFile(window, stlPath, isGot, &why) && !why.empty(),
                "the rotary import was not refused on a flat job");
    return {};
}

}  // namespace carvetto::app
