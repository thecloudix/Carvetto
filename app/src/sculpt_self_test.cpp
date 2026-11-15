// Self-test of the Sculpt mode and the level effect actions (T-0302): drives the real MainWindow
// offscreen through the Sculpt panel widgets, real mouse events on the Model view, and the
// Component dock's level effects section.

#include <QApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QSlider>
#include <cmath>
#include <memory>
#include <string>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "main_window.hpp"
#include "relief_panels.hpp"
#include "sculpt_panel.hpp"

namespace carvetto::app {

namespace {

#define SC_REQUIRE(cond, text)                          \
    do {                                                \
        if (!(cond)) {                                  \
            return std::string("sculpt: ") + (text);    \
        }                                               \
    } while (false)

void mouse(QWidget* w, QEvent::Type type, QPoint pos, Qt::MouseButton button, Qt::MouseButtons buttons) {
    QMouseEvent e(type, QPointF(pos), QPointF(w->mapToGlobal(pos)), button, buttons, Qt::NoModifier);
    QApplication::sendEvent(w, &e);
}

}  // namespace

std::string sculptSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(100.0, 100.0, 18.0, document::Units::Millimetres)));
    SculptPanel* sculpt = window.sculpt();
    LevelEffectsPanel* effects = window.levelEffects();
    ModelView3D* view = window.modelView();
    ComponentTreePanel* tree = window.componentTree();
    SC_REQUIRE(sculpt != nullptr && effects != nullptr && view != nullptr && tree != nullptr, "a panel is missing");
    document::Document* doc = &window.currentDocument();

    // A 41 x 41 mm block, 2 mm high, in a new level.
    SC_REQUIRE(tree->addLevel("Base"), "adding a level failed");
    const document::LevelId level = doc->reliefLevels().front().id;
    relief::Component block;
    block.name = "Block";
    SC_REQUIRE(relief::HeightField::create(relief::Grid{41, 41, 1.0, {20.0, 20.0}}, 2.0F, block.field) ==
                   relief::Status::Ok,
               "creating the field failed");
    auto add = std::make_unique<document::AddReliefComponentCommand>(level, std::move(block));
    const document::AddReliefComponentCommand* rawAdd = add.get();
    SC_REQUIRE(doc->execute(std::move(add)), "adding the component failed");
    const document::ComponentId id = rawAdd->createdId();
    window.documentChanged();

    // --- Level effects -------------------------------------------------------------------------------
    SC_REQUIRE(effects->summary()->text().contains(QStringLiteral("not clipped")), "the summary is wrong");
    SC_REQUIRE(!effects->applyClip() && doc->findReliefLevel(level)->effects.clip.polygons.empty(),
               "clipping without a vector selection was accepted");
    document::Object rect;
    rect.layer = doc->defaultLayer();
    rect.sheet = doc->activeSheet();
    document::VectorObject vec;
    vec.contours.push_back(geometry::Contour::rectangle(geometry::Box2({30.0, 30.0}, {50.0, 50.0})));
    rect.data = std::move(vec);
    SC_REQUIRE(doc->execute(std::make_unique<document::AddObjectCommand>(std::move(rect))), "adding the vector failed");
    doc->selection().assign({doc->objects().rbegin()->first});
    const std::size_t steps = doc->undoStack().index();
    SC_REQUIRE(effects->applyClip(), "Apply Clipping failed");
    SC_REQUIRE(doc->undoStack().index() == steps + 1, "Apply Clipping is not one undo step");
    SC_REQUIRE(doc->findReliefLevel(level)->effects.clip.polygons.size() == 1, "the clip was not stored");
    SC_REQUIRE(effects->summary()->text().contains(QStringLiteral("clipped by 1")), "the summary did not update");
    SC_REQUIRE(effects->removeButton()->isEnabled(), "Remove Clipping stayed disabled");
    SC_REQUIRE(effects->removeClip() && doc->findReliefLevel(level)->effects.clip.polygons.empty(),
               "Remove Clipping failed");
    SC_REQUIRE(window.undo() && doc->findReliefLevel(level)->effects.clip.polygons.size() == 1,
               "undo did not restore the clipping");
    SC_REQUIRE(window.undo() && doc->findReliefLevel(level)->effects.clip.polygons.empty(),
               "undo did not remove the clipping");
    // Mirror mode through the combo box; the centre is the model centre (40, 40).
    const int leftToRight = effects->mirrorBox()->findData(static_cast<int>(relief::MirrorMode::LeftToRight));
    SC_REQUIRE(leftToRight >= 0, "the mirror box lacks Left to right");
    SC_REQUIRE(effects->setMirror(relief::MirrorMode::LeftToRight), "setting the mirror mode failed");
    const document::ReliefLevel* lv = doc->findReliefLevel(level);
    SC_REQUIRE(lv->effects.mirror == relief::MirrorMode::LeftToRight && lv->effects.mirrorCenter.x == 40.0,
               "the mirror mode or its centre is wrong");
    SC_REQUIRE(effects->mirrorBox()->currentIndex() == leftToRight, "the mirror box did not follow the document");
    SC_REQUIRE(window.undo() && doc->findReliefLevel(level)->effects.mirror == relief::MirrorMode::None,
               "undo did not clear the mirror mode");
    SC_REQUIRE(effects->mirrorBox()->currentIndex() == effects->mirrorBox()->findData(0), "box not reset by undo");
    doc->selection().assign({});

    // --- Sculpt ----------------------------------------------------------------------------------------
    tree->selectComponents({});
    SC_REQUIRE(sculpt->startSculpting(), "starting sculpt on the only component failed");
    SC_REQUIRE(sculpt->sculpting() && view->previewing(), "the working copy is not shown in the Model view");
    SC_REQUIRE(!sculpt->keep(), "Keep with no strokes was accepted");

    // Brush palette and sliders.
    sculpt->brushButton(relief::BrushKind::Remove)->click();
    SC_REQUIRE(sculpt->brush().kind == relief::BrushKind::Remove, "the palette did not choose Remove");
    sculpt->brushButton(relief::BrushKind::Deposit)->click();
    sculpt->slider(QStringLiteral("Sculpt: Diameter"))->setValue(100);
    sculpt->slider(QStringLiteral("Sculpt: Strength"))->setValue(1000);
    sculpt->slider(QStringLiteral("Sculpt: Smoothness"))->setValue(500);
    const viewmodel::BrushSettings b = sculpt->brush();
    SC_REQUIRE(b.kind == relief::BrushKind::Deposit && b.diameter > 10.0 && b.diameter < 30.0 && b.strength == 2.0 &&
                   std::fabs(b.smoothness - 0.5) < 1e-9,
               "the sliders do not set the brush");
    sculpt->combo(QStringLiteral("Sculpt: Mode"))->setCurrentIndex(1);
    SC_REQUIRE(sculpt->brush().mode == relief::SculptMode::Raise, "the mode box did not set Raise");
    sculpt->check(QStringLiteral("Sculpt: Preserve transparency"))->setChecked(false);
    SC_REQUIRE(!sculpt->brush().preserveTransparency, "preserve transparency did not change");
    SC_REQUIRE(sculpt->applyPreset("Fine detail") && sculpt->brush().diameter == 3.0 &&
                   sculpt->brush().mode == relief::SculptMode::Normal,
               "the preset was not applied");
    sculpt->brushButton(relief::BrushKind::Deposit)->click();
    sculpt->slider(QStringLiteral("Sculpt: Diameter"))->setValue(100);
    sculpt->slider(QStringLiteral("Sculpt: Strength"))->setValue(500);
    SC_REQUIRE(sculpt->savePreset("My brush") && sculpt->presets().find("My brush") != nullptr &&
                   sculpt->combo(QStringLiteral("Sculpt: Preset"))->findText(QStringLiteral("My brush")) >= 0,
               "saving a preset failed");
    SC_REQUIRE(!sculpt->savePreset("Fine detail"), "a built-in preset name was overwritten");

    // Paint with real mouse events: top view, drag through the middle of the block.
    view->camera().viewTop();
    view->resize(400, 400);
    view->cameraChanged();
    const auto centre = view->pickAt(QPoint(200, 200));
    SC_REQUIRE(centre.has_value() && std::fabs(centre->x - 40.0) < 1.0 && std::fabs(centre->y - 40.0) < 1.0,
               "the centre pixel does not pick the model centre");
    const std::size_t docSteps = doc->undoStack().index();
    mouse(view, QEvent::MouseButtonPress, QPoint(180, 200), Qt::LeftButton, Qt::LeftButton);
    for (int x = 185; x <= 220; x += 5) {
        mouse(view, QEvent::MouseMove, QPoint(x, 200), Qt::NoButton, Qt::LeftButton);
    }
    mouse(view, QEvent::MouseButtonRelease, QPoint(220, 200), Qt::LeftButton, Qt::NoButton);
    SC_REQUIRE(sculpt->controller().dirty(), "painting did not change the working copy");
    SC_REQUIRE(doc->undoStack().index() == docSteps, "painting changed the document before Keep");
    const relief::HeightField& w = *sculpt->controller().working();
    SC_REQUIRE(w.at(20, 20) > 2.0F, "the brush did not raise the centre");
    SC_REQUIRE(w.at(0, 0) == 2.0F, "the brush changed a far corner");
    const QImage painted = view->image();
    SC_REQUIRE(painted.pixel(200, 200) != 0, "the Model view is not drawn");

    // Undo Stroke, then paint again and Keep.
    SC_REQUIRE(sculpt->undoStroke() && !sculpt->controller().dirty(), "Undo Stroke did not revert the stroke");
    mouse(view, QEvent::MouseButtonPress, QPoint(200, 200), Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, QPoint(200, 200), Qt::LeftButton, Qt::NoButton);
    SC_REQUIRE(sculpt->controller().dirty(), "a click did not paint a dab");
    const float centreHeight = sculpt->controller().working()->at(20, 20);
    SC_REQUIRE(sculpt->keep(), "Keep failed");
    SC_REQUIRE(doc->undoStack().index() == docSteps + 1, "Keep is not one undo step");
    SC_REQUIRE(doc->findReliefComponent(id)->data.field.at(20, 20) == centreHeight, "Keep did not write the field");
    SC_REQUIRE(sculpt->sculpting(), "Keep ended the session");

    // Discard drops strokes since the last Keep.
    mouse(view, QEvent::MouseButtonPress, QPoint(200, 200), Qt::LeftButton, Qt::LeftButton);
    mouse(view, QEvent::MouseButtonRelease, QPoint(200, 200), Qt::LeftButton, Qt::NoButton);
    SC_REQUIRE(sculpt->controller().working()->at(20, 20) > centreHeight, "the second dab did nothing");
    sculpt->discard();
    SC_REQUIRE(sculpt->controller().working()->at(20, 20) == centreHeight, "Discard did not restore the kept state");

    // An outside Undo of the Keep makes the session follow the document.
    SC_REQUIRE(window.undo() && doc->findReliefComponent(id)->data.field.at(20, 20) == 2.0F, "undoing Keep failed");
    SC_REQUIRE(sculpt->controller().working()->at(20, 20) == 2.0F, "the session did not follow the undo");

    // Stopping leaves the Model view; deleting the component while sculpting ends the session.
    sculpt->stopSculpting();
    SC_REQUIRE(!sculpt->sculpting() && !view->previewing(), "stopping did not return to the model");
    SC_REQUIRE(sculpt->startSculpting(), "restarting failed");
    tree->selectComponents({id});
    SC_REQUIRE(tree->deleteSelected(), "deleting the component failed");
    SC_REQUIRE(!sculpt->sculpting(), "sculpting continued on a deleted component");
    SC_REQUIRE(!sculpt->startSculpting() && !sculpt->status()->text().isEmpty(),
               "sculpting without a component was accepted");
    window.undo();
    return {};
}

}  // namespace carvetto::app
