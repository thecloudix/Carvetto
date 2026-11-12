#include "double_sided_menu.hpp"

#include <QAction>
#include <QKeySequence>
#include <QMenu>
#include <QStatusBar>
#include <string>
#include <vector>

#include "canvas_widget.hpp"
#include "carvetto/viewmodel/double_sided_ops.hpp"
#include "carvetto/viewmodel/layers_sheets.hpp"
#include "main_window.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

DoubleSidedMenu::DoubleSidedMenu(MainWindow& window, QMenu* edit) : window_(window) {
    QMenu* menu = edit->addMenu(QStringLiteral("Double-&sided"));
    const auto add = [&](const char* name, const QString& text) {
        QAction* a = menu->addAction(text);
        a->setObjectName(QString::fromLatin1(name));
        return a;
    };
    swap_ = add("swapSidesAction", QStringLiteral("S&wap Sides"));
    top_ = add("topSideAction", QStringLiteral("&Top Side"));
    bottom_ = add("bottomSideAction", QStringLiteral("&Bottom Side"));
    multi_ = add("multiSidedViewAction", QStringLiteral("&Multi-Sided View"));
    multi_->setCheckable(true);
    menu->addSeparator();
    copyObjects_ = add("copyToOtherSideAction", QStringLiteral("&Copy Selection to Other Side"));
    moveObjects_ = add("moveToOtherSideAction", QStringLiteral("Mo&ve Selection to Other Side"));
    copyToolpath_ = add("copyToolpathToOtherSideAction", QStringLiteral("Copy Toolpath to &Other Side"));
    moveToolpath_ = add("moveToolpathToOtherSideAction", QStringLiteral("Move Toolpath to Ot&her Side"));
    menu->addSeparator();
    single_ = add("singleSidedJobAction", QStringLiteral("Switch to &Single-Sided Job"));

    // Bare keys as the product has them; text being typed wins (CanvasWidget::event).
    top_->setShortcut(QKeySequence(Qt::Key_1));
    bottom_->setShortcut(QKeySequence(Qt::Key_2));
    multi_->setShortcut(QKeySequence(Qt::Key_Equal));

    const auto changed = [this](const viewmodel::EditResult& r) { afterEdit(r.changed, r.ok ? std::string() : r.message); };
    QObject::connect(swap_, &QAction::triggered, menu, [this, changed]() { changed(viewmodel::swapSides(window_.currentDocument())); });
    QObject::connect(top_, &QAction::triggered, menu, [this, changed]() {
        changed(viewmodel::activateSide(window_.currentDocument(), document::Side::Front));
    });
    QObject::connect(bottom_, &QAction::triggered, menu, [this, changed]() {
        changed(viewmodel::activateSide(window_.currentDocument(), document::Side::Back));
    });
    QObject::connect(multi_, &QAction::toggled, menu, [this](bool on) {
        window_.canvas()->setMultiSidedView(on);
        window_.canvas()->update();
    });
    QObject::connect(copyObjects_, &QAction::triggered, menu, [this]() {
        const viewmodel::SideCopyResult r = viewmodel::copySelectionToOtherSide(window_.currentDocument(), false);
        afterEdit(r.edit.changed, r.edit.ok ? std::string() : r.edit.message);
    });
    QObject::connect(moveObjects_, &QAction::triggered, menu, [this]() {
        const viewmodel::SideCopyResult r = viewmodel::copySelectionToOtherSide(window_.currentDocument(), true);
        afterEdit(r.edit.changed, r.edit.ok ? std::string() : r.edit.message);
    });
    QObject::connect(copyToolpath_, &QAction::triggered, menu, [this]() { toolpathToOtherSide(false); });
    QObject::connect(moveToolpath_, &QAction::triggered, menu, [this]() { toolpathToOtherSide(true); });
    QObject::connect(single_, &QAction::triggered, menu, [this]() {
        const viewmodel::EditResult r = viewmodel::makeSingleSided(window_.currentDocument());
        afterEdit(r.changed, r.ok ? std::string() : r.message);
    });
    refresh();
}

void DoubleSidedMenu::toolpathToOtherSide(bool move) {
    const document::ToolpathId id = window_.toolpaths()->currentToolpath();
    if (id.isNull()) {
        afterEdit(false, "select a toolpath first");
        return;
    }
    const viewmodel::SideCopyResult r =
        viewmodel::copyToolpathsToOtherSide(window_.currentDocument(), {id}, move);
    afterEdit(r.edit.changed, r.edit.ok ? std::string() : r.edit.message);
}

void DoubleSidedMenu::afterEdit(bool changed, const std::string& message) {
    lastMessage_ = QString::fromStdString(message);
    if (!message.empty()) {
        window_.statusBar()->showMessage(lastMessage_, 6000);
    }
    if (changed) {
        window_.toolpaths()->invalidate();  // the active sheet or the sheets themselves changed
        window_.documentChanged();
    }
}

void DoubleSidedMenu::refresh() {
    const document::Document& doc = window_.currentDocument();
    const document::Sheet* active = doc.findSheet(doc.activeSheet());
    const bool paired = doc.job().type == document::JobType::DoubleSided && active != nullptr &&
                        doc.findSheet(active->partner) != nullptr;
    for (QAction* a : {swap_, top_, bottom_, multi_, copyObjects_, moveObjects_, copyToolpath_, moveToolpath_}) {
        a->setEnabled(paired);
    }
    single_->setEnabled(doc.job().type == document::JobType::DoubleSided);
    if (!paired && multi_->isChecked()) {
        multi_->setChecked(false);
    }
}

}  // namespace carvetto::app
