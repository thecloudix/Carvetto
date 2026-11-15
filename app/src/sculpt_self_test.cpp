// Self-test of the Sculpt mode and the level effect actions (T-0302): drives the real MainWindow
// offscreen through the Sculpt panel widgets, real mouse events on the Model view, and the
// Component dock's level effects section.

#include <QApplication>
#include <QCheckBox>
#include <QColor>
#include <QImage>
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

int toSliderForTest(double diameterMm) {
    const viewmodel::Range r = viewmodel::diameterRange();
    return static_cast<int>(std::lround((diameterMm - r.lo) / (r.hi - r.lo) * 1000.0));
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

    std::string savedPresets;
    sculpt->onPresetsChanged = [&savedPresets](const std::string& text) { savedPresets = text; };

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

    // --- T-0309: ring cursor, custom brush, overlay, persisted presets --------------------------------------
    {
        QEvent away(QEvent::Leave);
        QApplication::sendEvent(view, &away);
        SC_REQUIRE(view->ringOutline().empty(), "a ring is drawn while the pointer is away");
        mouse(view, QEvent::MouseMove, QPoint(200, 200), Qt::NoButton, Qt::NoButton);
        const auto ring = view->ringOutline();
        SC_REQUIRE(ring.size() == 48, "hovering the model did not give the brush ring");
        double mx = 0.0, my = 0.0;
        for (const viewmodel::ScreenPoint& q : ring) {
            mx += q.x / 48.0;
            my += q.y / 48.0;
        }
        SC_REQUIRE(std::fabs(mx - 200.0) < 3.0 && std::fabs(my - 200.0) < 3.0, "the ring is not centred on the pointer");
        const double radiusPx = std::hypot(ring[0].x - mx, ring[0].y - my);
        sculpt->slider(QStringLiteral("Sculpt: Diameter"))->setValue(500);  // a bigger brush, a bigger ring
        const auto bigger = view->ringOutline();
        SC_REQUIRE(bigger.size() == 48 && std::hypot(bigger[0].x - mx, bigger[0].y - my) > 2.0 * radiusPx,
                   "the ring does not follow the brush diameter");
        QEvent leave(QEvent::Leave);
        QApplication::sendEvent(view, &leave);
        SC_REQUIRE(view->ringOutline().empty(), "the ring stayed after the pointer left");
        sculpt->slider(QStringLiteral("Sculpt: Diameter"))->setValue(100);

        // Custom brush from a picture: right half white, left half black; painting raises only the right half.
        QImage stamp(8, 8, QImage::Format_ARGB32);
        for (int y = 0; y < 8; ++y) {
            for (int x = 0; x < 8; ++x) {
                stamp.setPixelColor(x, y, x < 4 ? QColor(0, 0, 0) : QColor(255, 255, 255));
            }
        }
        SC_REQUIRE(!sculpt->setBrushImage(QImage()) && !sculpt->hasBrushImage(), "an empty picture became the brush");
        SC_REQUIRE(sculpt->setBrushImage(stamp) && sculpt->brush().custom != nullptr, "the picture brush was not set");
        sculpt->slider(QStringLiteral("Sculpt: Diameter"))->setValue(toSliderForTest(20.0));
        sculpt->discard();
        mouse(view, QEvent::MouseButtonPress, QPoint(200, 200), Qt::LeftButton, Qt::LeftButton);
        mouse(view, QEvent::MouseButtonRelease, QPoint(200, 200), Qt::LeftButton, Qt::NoButton);
        const relief::HeightField& stamped = *sculpt->controller().working();
        SC_REQUIRE(sculpt->brush().custom != nullptr, "changing the diameter dropped the picture brush");
        SC_REQUIRE(stamped.at(26, 20) > 2.0F && stamped.at(14, 20) == 2.0F, "the stamp did not shape the stroke");
        sculpt->setBrushImageOptions(true, false);
        SC_REQUIRE(sculpt->brush().custom != nullptr && sculpt->brush().custom->stamp.at(7, 0) < 0.5F,
                   "Invert image did not rebuild the stamp");
        sculpt->clearBrushImage();
        SC_REQUIRE(sculpt->brush().custom == nullptr, "Round Brush did not clear the stamp");
        sculpt->setBrushImageOptions(false, false);
        sculpt->discard();

        // Overlay: a red picture at full opacity covers the model centre; opacity 0 shows the model.
        view->camera().viewTop();
        view->cameraChanged();
        const QRgb before = view->grab().toImage().pixel(200, 200);
        QImage red(10, 10, QImage::Format_ARGB32);
        red.fill(QColor(255, 0, 0));
        SC_REQUIRE(!sculpt->setOverlayImage(QImage()), "an empty overlay was accepted");
        SC_REQUIRE(sculpt->setOverlayImage(red) && view->hasOverlay(), "the overlay did not reach the Model view");
        sculpt->setOverlayOpacity(1.0);
        SC_REQUIRE(sculpt->slider(QStringLiteral("Sculpt: Overlay opacity"))->value() == 1000 &&
                       view->overlayOpacity() == 1.0,
                   "the overlay opacity did not follow");
        const QColor over = QColor::fromRgb(view->grab().toImage().pixel(200, 200));
        SC_REQUIRE(over.red() > 240 && over.green() < 20 && over.blue() < 20, "the overlay is not drawn opaque");
        const auto tl = view->overlayCorner(0);
        const auto tr = view->overlayCorner(1);
        const auto bl = view->overlayCorner(2);
        SC_REQUIRE(tl && tr && bl && tr->x > tl->x && bl->y > tl->y, "the overlay corners are wrong in the top view");
        sculpt->slider(QStringLiteral("Sculpt: Overlay opacity"))->setValue(0);
        SC_REQUIRE(view->overlayOpacity() == 0.0 && view->grab().toImage().pixel(200, 200) == before,
                   "opacity 0 still changes the picture");
        sculpt->clearOverlayImage();
        SC_REQUIRE(!view->hasOverlay(), "Clear Overlay did not clear the Model view");

        // Presets reach the settings store as text and load into a new panel.
        SC_REQUIRE(sculpt->savePreset("Persisted") && savedPresets.find("Persisted") != std::string::npos,
                   "saving a preset did not hand it to the store");
        SculptPanel fresh([doc]() { return doc; });
        SC_REQUIRE(fresh.loadPresets(savedPresets) >= 2 && fresh.presets().find("Persisted") != nullptr &&
                       fresh.presets().find("My brush") != nullptr,
                   "persisted presets did not load into a new panel");
        SC_REQUIRE(fresh.combo(QStringLiteral("Sculpt: Preset"))->findText(QStringLiteral("Persisted")) >= 0,
                   "a loaded preset is not in the box");
        SC_REQUIRE(sculpt->removePreset("Persisted") && savedPresets.find("Persisted") == std::string::npos,
                   "deleting a preset did not update the store");
        SC_REQUIRE(!sculpt->removePreset("Fine detail"), "a built-in preset was deleted");
    }

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
