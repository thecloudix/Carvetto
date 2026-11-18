#include "node_edit_menu.hpp"
#include "variable_menu.hpp"

#include <QCoreApplication>
#include <QAction>
#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QPoint>
#include <QStatusBar>
#include <QString>
#include <QVBoxLayout>
#include <algorithm>

#include "canvas_widget.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/units_format.hpp"
#include "main_window.hpp"

namespace carvetto::app {

using viewmodel::FieldKind;
using viewmodel::HandleType;
using viewmodel::NodeOp;

// ------------------------------------------------------------------------------ dialog

NodePropertiesDialog::NodePropertiesDialog(document::Units units, std::optional<geometry::Vec2> current,
                                           QWidget* parent)
    : QDialog(parent), units_(units) {
    setWindowTitle(QCoreApplication::translate("NodeEditMenu", "Node Properties"));
    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout();
    layout->addLayout(form);
    const QString suffix = QString::fromUtf8(document::unitSuffix(units).data());
    x_ = new QLineEdit(this);
    y_ = new QLineEdit(this);
    attachVariableMenu(x_);
    attachVariableMenu(y_);
    if (current.has_value()) {
        x_->setText(QString::fromStdString(viewmodel::formatLength(current->x, units)));
        y_->setText(QString::fromStdString(viewmodel::formatLength(current->y, units)));
    }
    form->addRow(QCoreApplication::translate("NodeEditMenu", "X (%1)").arg(suffix), x_);
    form->addRow(QCoreApplication::translate("NodeEditMenu", "Y (%1)").arg(suffix), y_);
    relative_ = new QCheckBox(QCoreApplication::translate("NodeEditMenu", "Relative (move the selected nodes by these amounts)"), this);
    layout->addWidget(relative_);
    layout->addWidget(new QLabel(QCoreApplication::translate("NodeEditMenu", "Leave a field empty to keep that coordinate."), this));
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    layout->addWidget(buttons);
}

bool NodePropertiesDialog::values(Values* out, std::string* error) const {
    Values v;
    v.relative = relative_->isChecked();
    auto parse = [&](QLineEdit* edit, std::optional<double>* dest, const char* name) {
        const std::string text = edit->text().toStdString();
        if (text.find_first_not_of(" \t") == std::string::npos) {
            return true;
        }
        const viewmodel::ParsedNumber n = viewmodel::parseNumber(text, FieldKind::Length, units_);
        if (!n.ok) {
            if (error != nullptr) {
                *error = std::string(name) + ": " + n.error;
            }
            return false;
        }
        *dest = n.value;
        return true;
    };
    if (!parse(x_, &v.xMm, "X") || !parse(y_, &v.yMm, "Y")) {
        return false;
    }
    if (!v.xMm.has_value() && !v.yMm.has_value()) {
        if (error != nullptr) {
            *error = QCoreApplication::translate("NodeEditMenu", "enter an X or Y value").toStdString();
        }
        return false;
    }
    if (out != nullptr) {
        *out = v;
    }
    return true;
}

void NodePropertiesDialog::setTexts(const QString& x, const QString& y, bool relative) {
    x_->setText(x);
    y_->setText(y);
    relative_->setChecked(relative);
}

void NodePropertiesDialog::accept() {
    std::string error;
    if (!values(nullptr, &error)) {
        QMessageBox::warning(this, windowTitle(), QString::fromStdString(error));
        return;
    }
    QDialog::accept();
}

// ------------------------------------------------------------------------------ menus

NodeEditMenu::NodeEditMenu(MainWindow& window) : window_(window) {}

void NodeEditMenu::report(const viewmodel::NodeEditResult& result, bool showErrors) {
    if (result.ok) {
        window_.statusBar()->clearMessage();
        return;
    }
    const QString text = QString::fromStdString(result.message);
    if (showErrors) {
        QMessageBox::warning(&window_, QCoreApplication::translate("NodeEditMenu", "Node editing"), text);
    } else if (!text.isEmpty()) {
        window_.statusBar()->showMessage(text, 6000);
    }
}

viewmodel::NodeEditResult NodeEditMenu::run(NodeOp op, const viewmodel::NodeOpArgs& args, bool showErrors) {
    viewmodel::NodeEditor& editor = window_.canvas()->tools().nodeEditor();
    const viewmodel::NodeEditResult result = editor.run(op, args);
    report(result, showErrors);
    if (result.changed) {
        window_.documentChanged();
    }
    window_.canvas()->update();
    return result;
}

viewmodel::NodeEditResult NodeEditMenu::applyProperties(const NodePropertiesDialog::Values& values) {
    viewmodel::NodeEditor& editor = window_.canvas()->tools().nodeEditor();
    const viewmodel::NodeEditResult result =
        editor.setSelectedNodeCoordinates(values.xMm, values.yMm, values.relative);
    report(result, false);
    if (result.changed) {
        window_.documentChanged();
    }
    window_.canvas()->update();
    return result;
}

void NodeEditMenu::showProperties() {
    viewmodel::NodeEditor& editor = window_.canvas()->tools().nodeEditor();
    const auto nodes = editor.selectedNodes();
    if (nodes.empty()) {
        window_.statusBar()->showMessage(QCoreApplication::translate("NodeEditMenu", "Select a node first"), 4000);
        return;
    }
    NodePropertiesDialog dialog(window_.currentDocument().job().displayUnits, editor.nodePosition(nodes.front()),
                                &window_);
    if (dialog.exec() != QDialog::Accepted) {
        return;
    }
    NodePropertiesDialog::Values values;
    std::string error;
    if (dialog.values(&values, &error)) {
        applyProperties(values);
    }
}

void NodeEditMenu::exit() {
    window_.selectTool(viewmodel::ToolKind::Select);
}

QMenu* NodeEditMenu::buildMenu(HandleType kind, geometry::Vec2 posMm) {
    viewmodel::NodeEditor& editor = window_.canvas()->tools().nodeEditor();
    auto* menu = new QMenu(&window_);
    auto add = [&](const QString& text, NodeOp op, const viewmodel::NodeOpArgs& args = {}) {
        QAction* action = menu->addAction(text);
        std::string why;
        action->setEnabled(editor.canRun(op, &why));
        action->setToolTip(QString::fromStdString(why));
        QObject::connect(action, &QAction::triggered, menu, [this, op, args]() { run(op, args); });
        return action;
    };
    viewmodel::NodeOpArgs at;
    at.positionMm = posMm;
    const bool spanMenu = kind == HandleType::Span || kind == HandleType::Midpoint || kind == HandleType::Control;
    if (spanMenu) {
        add(QCoreApplication::translate("NodeEditMenu", "Convert to &Line"), NodeOp::ConvertLine);
        add(QCoreApplication::translate("NodeEditMenu", "Convert to &Bezier"), NodeOp::ConvertBezier);
        add(QCoreApplication::translate("NodeEditMenu", "Convert to &Arc"), NodeOp::ConvertArc);
        menu->addSeparator();
        add(QCoreApplication::translate("NodeEditMenu", "&Insert Point"), NodeOp::InsertPoint, at);
        add(QCoreApplication::translate("NodeEditMenu", "Insert &Midpoint"), NodeOp::InsertMidpoint);
        add(QCoreApplication::translate("NodeEditMenu", "&Cut Vector"), NodeOp::CutVector, at);
        add(QCoreApplication::translate("NodeEditMenu", "&Delete Span"), NodeOp::DeleteSpan);
        menu->addSeparator();
        QAction* keep = menu->addAction(QCoreApplication::translate("NodeEditMenu", "&Keep Bezier Tangency"));
        keep->setCheckable(true);
        keep->setChecked(editor.options().keepBezierTangency);
        QObject::connect(keep, &QAction::toggled, menu, [this](bool on) {
            window_.canvas()->tools().nodeEditor().options().keepBezierTangency = on;
        });
        add(QCoreApplication::translate("NodeEditMenu", "&Reverse"), NodeOp::Reverse);
        add(QCoreApplication::translate("NodeEditMenu", "Clos&e"), NodeOp::Close);
        add(QCoreApplication::translate("NodeEditMenu", "&Join"), NodeOp::Join);
    } else {
        add(QCoreApplication::translate("NodeEditMenu", "&Delete Point"), NodeOp::DeletePoint);
        add(QCoreApplication::translate("NodeEditMenu", "&Smooth"), NodeOp::Smooth);
        add(QCoreApplication::translate("NodeEditMenu", "&Insert Point at virtual midpoint"), NodeOp::InsertMidpoint);
        add(QCoreApplication::translate("NodeEditMenu", "&Cut Vector"), NodeOp::CutVector);
        add(QCoreApplication::translate("NodeEditMenu", "S&tart Point"), NodeOp::StartPoint);
        add(QCoreApplication::translate("NodeEditMenu", "E&xtend"), NodeOp::Extend);
        menu->addSeparator();
        QMenu* mirror = menu->addMenu(QCoreApplication::translate("NodeEditMenu", "&Mirror Mode"));
        const struct {
            const char* text;
            viewmodel::MirrorMode mode;
        } modes[] = {{QT_TRANSLATE_NOOP("NodeEditMenu", "Off"), viewmodel::MirrorMode::Off},
                     {QT_TRANSLATE_NOOP("NodeEditMenu", "Horizontal"), viewmodel::MirrorMode::Horizontal},
                     {QT_TRANSLATE_NOOP("NodeEditMenu", "Vertical"), viewmodel::MirrorMode::Vertical}};
        for (const auto& m : modes) {
            QAction* a = mirror->addAction(QCoreApplication::translate("NodeEditMenu", m.text));
            a->setCheckable(true);
            a->setChecked(editor.options().mirror == m.mode);
            const viewmodel::MirrorMode mode = m.mode;
            QObject::connect(a, &QAction::triggered, menu, [this, mode]() {
                window_.canvas()->tools().nodeEditor().setMirror(mode);
                window_.canvas()->update();
            });
        }
        menu->addSeparator();
        add(QCoreApplication::translate("NodeEditMenu", "Clos&e"), NodeOp::Close);
        add(QCoreApplication::translate("NodeEditMenu", "&Join"), NodeOp::Join);
        QAction* props = menu->addAction(QCoreApplication::translate("NodeEditMenu", "&Properties..."));
        props->setEnabled(!editor.selectedNodes().empty());
        QObject::connect(props, &QAction::triggered, menu, [this]() { showProperties(); });
    }
    menu->addSeparator();
    QAction* exitAction = menu->addAction(QCoreApplication::translate("NodeEditMenu", "Exit"));
    QObject::connect(exitAction, &QAction::triggered, menu, [this]() { exit(); });
    return menu;
}

bool NodeEditMenu::showContextMenu(const QPoint& globalPos, geometry::Vec2 px) {
    CanvasWidget* canvas = window_.canvas();
    viewmodel::NodeEditor& editor = canvas->tools().nodeEditor();
    const std::optional<viewmodel::NodeHit> hit = editor.hitTest(px);
    if (!hit.has_value()) {
        return false;
    }
    if (hit->type == HandleType::Node) {
        const auto sel = editor.selectedNodes();
        if (std::find(sel.begin(), sel.end(), hit->node) == sel.end()) {
            editor.selectNode(hit->node, false);
        }
    } else {
        const auto sel = editor.selectedSpans();
        if (std::find(sel.begin(), sel.end(), hit->span) == sel.end()) {
            editor.selectSpan(hit->span, false);
        }
    }
    canvas->update();
    QMenu* menu = buildMenu(hit->type, hit->positionMm);
    menu->exec(globalPos);
    delete menu;
    return true;
}

}  // namespace carvetto::app
