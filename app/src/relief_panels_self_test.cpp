// Self-test of the Component Tree, Component Properties form and the Model 3D view (T-0299):
// drives the real MainWindow offscreen through the widgets and checks the drawn pixels.

#include <QCheckBox>
#include <QComboBox>
#include <QImage>
#include <QLineEdit>
#include <QTreeWidget>
#include <memory>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/document/relief_commands.hpp"
#include "main_window.hpp"
#include "relief_panels.hpp"

namespace carvetto::app {

namespace {

#define RT_REQUIRE(cond, text)                              \
    do {                                                    \
        if (!(cond)) {                                      \
            return std::string("relief panels: ") + (text); \
        }                                                   \
    } while (false)

document::ComponentId addBlock(document::Document& doc, document::LevelId level, const char* name, std::size_t n,
                               double cell, float height) {
    relief::Component c;
    c.name = name;
    if (relief::HeightField::create(relief::Grid{n, n, cell, {0.0, 0.0}}, height, c.field) != relief::Status::Ok) {
        return {};
    }
    auto cmd = std::make_unique<document::AddReliefComponentCommand>(level, std::move(c));
    const document::AddReliefComponentCommand* raw = cmd.get();
    return doc.execute(std::move(cmd)) ? raw->createdId() : document::ComponentId{};
}

}  // namespace

std::string reliefPanelsSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    ComponentTreePanel* tree = window.componentTree();
    ComponentPropertiesPanel* props = window.componentProperties();
    ModelView3D* view = window.modelView();
    RT_REQUIRE(tree != nullptr && props != nullptr && view != nullptr, "a panel is missing");
    document::Document* doc = &window.currentDocument();
    RT_REQUIRE(tree->topLevelCount() == 0, "a new document lists levels");

    // --- Tree: levels and components through the widgets ---------------------------------------
    RT_REQUIRE(tree->addLevel("Base") && tree->addLevel(), "adding levels failed");
    RT_REQUIRE(tree->topLevelCount() == 2 && doc->reliefLevels().size() == 2, "the levels are not listed");
    const document::LevelId base = doc->reliefLevels()[0].id;
    const document::LevelId top = doc->reliefLevels()[1].id;
    RT_REQUIRE(tree->tree()->topLevelItem(0)->text(0) == QStringLiteral("Level 2"), "the top level is not listed first");
    const document::ComponentId block = addBlock(*doc, base, "Block", 61, 1.0, 5.0f);
    const document::ComponentId second = addBlock(*doc, base, "Second", 61, 1.0, 2.0f);
    RT_REQUIRE(block && second, "adding components failed");
    window.documentChanged();
    RT_REQUIRE(tree->componentItem(block) != nullptr && tree->componentItem(second) != nullptr,
               "the components are not listed");
    RT_REQUIRE(tree->levelItem(base)->child(0)->text(0) == QStringLiteral("Second"), "components are not top first");

    // Rename and hide through the item, one undo step each.
    std::size_t steps = doc->undoStack().count();
    tree->componentItem(second)->setText(0, QStringLiteral("Renamed"));
    RT_REQUIRE(doc->findReliefComponent(second)->data.name == "Renamed", "renaming through the tree failed");
    tree->componentItem(second)->setCheckState(2, Qt::Unchecked);
    RT_REQUIRE(!doc->findReliefComponent(second)->data.visible, "unchecking Show did not hide the component");
    RT_REQUIRE(doc->undoStack().count() == steps + 2, "tree edits are not one undo step each");
    RT_REQUIRE(!tree->rename(second, ""), "an empty name was accepted");
    RT_REQUIRE(window.undo() && window.undo() && doc->findReliefComponent(second)->data.name == "Second" &&
                   doc->findReliefComponent(second)->data.visible,
               "undo did not restore the component");

    // Selection, combine mode, reorder, move to level, duplicate, delete.
    tree->selectComponents({second});
    RT_REQUIRE(doc->componentSelection().single() == second, "selecting a component failed");
    RT_REQUIRE(tree->setSelectedMode(relief::CombineMode::MergeHigh) &&
                   doc->findReliefComponent(second)->data.mode == relief::CombineMode::MergeHigh,
               "changing the combine mode failed");
    RT_REQUIRE(tree->moveSelected(false) && doc->locateReliefComponent(second)->index == 0, "moving down failed");
    RT_REQUIRE(tree->tree()->topLevelItem(1)->child(1)->text(0) == QStringLiteral("Second"),
               "the tree did not follow the reorder");
    RT_REQUIRE(tree->moveSelectedToLevel(top) && doc->locateReliefComponent(second)->level == top,
               "moving to another level failed");
    RT_REQUIRE(tree->drop(second, base, 0) && doc->locateReliefComponent(second)->level == base &&
                   doc->locateReliefComponent(second)->index == 0,
               "dropping a component onto a level failed");
    RT_REQUIRE(tree->duplicateSelected() && doc->reliefComponentCount() == 3, "duplicating failed");
    RT_REQUIRE(doc->findReliefComponent(doc->componentSelection().single())->data.name == "Second copy",
               "the copy is not selected");
    RT_REQUIRE(tree->deleteSelected() && doc->reliefComponentCount() == 2, "deleting failed");
    RT_REQUIRE(window.undo() && doc->reliefComponentCount() == 3, "undoing the delete failed");
    RT_REQUIRE(window.undo() && doc->reliefComponentCount() == 2, "undoing the duplicate failed");

    // --- Component Properties -----------------------------------------------------------------
    tree->selectComponents({block});
    RT_REQUIRE(props->showingComponent() && props->field(QStringLiteral("Name"))->text() == QStringLiteral("Block"),
               "the properties form does not show the selection");
    steps = doc->undoStack().index();
    props->field(QStringLiteral("Base height"))->setText(QStringLiteral("3"));
    props->field(QStringLiteral("Shape height %"))->setText(QStringLiteral("50"));
    RT_REQUIRE(props->apply(), "applying properties failed");
    RT_REQUIRE(doc->findReliefComponent(block)->data.baseHeight == 3.0 &&
                   doc->findReliefComponent(block)->data.shapeHeightPercent == 50.0,
               "the properties did not reach the document");
    RT_REQUIRE(doc->undoStack().index() == steps + 1, "applying is not one undo step");
    props->field(QStringLiteral("Base height"))->setText(QStringLiteral("abc"));
    RT_REQUIRE(!props->apply() && doc->findReliefComponent(block)->data.baseHeight == 3.0,
               "a bad number was applied");
    RT_REQUIRE(window.undo() && doc->findReliefComponent(block)->data.baseHeight == 0.0, "undo of apply failed");

    // --- 3D view: a known model -----------------------------------------------------------------
    view->resize(320, 240);
    window.documentChanged();
    RT_REQUIRE(view->surface().ok(), "the model view has no surface");
    view->camera().viewTop();
    view->cameraChanged();
    const QImage first = view->image();
    RT_REQUIRE(first.width() == 320 && first.height() == 240, "the model picture has the wrong size");
    const QRgb background = 0xFFE2E4E8u;
    RT_REQUIRE(first.pixel(160, 120) != background, "the model is not drawn at the centre");
    RT_REQUIRE(first.pixel(3, 237) == background, "the area around the model is not background");
    const QRgb woodCentre = first.pixel(160, 120);
    view->setMaterial(viewmodel::ModelMaterial::Metal);
    RT_REQUIRE(view->image().pixel(160, 120) != woodCentre, "the material does not change the colour");
    view->setMaterial(viewmodel::ModelMaterial::Wood);
    RT_REQUIRE(view->image().pixel(160, 120) == woodCentre, "the colour is not reproducible");
    view->camera().orbit(40.0, -60.0);
    view->cameraChanged();
    RT_REQUIRE(view->image() != first, "orbiting did not change the picture");
    view->camera().viewTop();
    view->cameraChanged();
    // Hiding every component empties the view.
    RT_REQUIRE(tree->toggleLevelVisible(base), "hiding the level failed");
    RT_REQUIRE(!view->surface().ok() && view->image().pixel(160, 120) == background, "a hidden level is still drawn");
    RT_REQUIRE(window.undo(), "undoing the hide failed");

    // --- Large field stays responsive (level of detail) ------------------------------------------
    const document::ComponentId big = addBlock(*doc, top, "Big", 2000, 0.1, 4.0f);
    RT_REQUIRE(big, "adding the large component failed");
    window.documentChanged();
    RT_REQUIRE(view->surface().ok() && view->surface().field.width() <= viewmodel::kMaxSurfaceDimension &&
                   view->surface().field.height() <= viewmodel::kMaxSurfaceDimension,
               "the large field is not reduced");
    RT_REQUIRE(view->image().pixel(160, 120) != background, "the large model is not drawn");
    RT_REQUIRE(window.undo(), "undoing the large component failed");

    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    RT_REQUIRE(tree->topLevelCount() == 0 && !view->surface().ok(), "a new document keeps the old model");
    return {};
}

}  // namespace carvetto::app
