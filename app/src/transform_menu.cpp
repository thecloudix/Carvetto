#include "transform_menu.hpp"

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QCursor>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QKeySequence>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMenuBar>
#include <QMouseEvent>
#include <QCoreApplication>
#include <QMessageBox>
#include <QString>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <memory>
#include <utility>

#include "canvas_widget.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/contour.hpp"
#include "main_window.hpp"

namespace carvetto::app {

using viewmodel::AlignKind;
using viewmodel::CentreAxes;
using viewmodel::HandleKind;
using viewmodel::MirrorAxis;
using viewmodel::SpaceDirection;
using viewmodel::TransformFormKind;
using viewmodel::TransformResult;

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

int mirrorKey(MirrorAxis axis, bool copy, bool material) {
    return (static_cast<int>(axis) << 2) | (copy ? 2 : 0) | (material ? 1 : 0);
}

struct AnchorName {
    HandleKind kind;
    const char* text;
};
constexpr AnchorName kAnchors[] = {
    {HandleKind::TopLeft, "Top left"},       {HandleKind::Top, "Top"},          {HandleKind::TopRight, "Top right"},
    {HandleKind::Left, "Left"},              {HandleKind::Centre, "Centre"},    {HandleKind::Right, "Right"},
    {HandleKind::BottomLeft, "Bottom left"}, {HandleKind::Bottom, "Bottom"},    {HandleKind::BottomRight, "Bottom right"},
};

QComboBox* anchorCombo(QWidget* parent, bool cornersAndCentreOnly, HandleKind current) {
    auto* combo = new QComboBox(parent);
    for (const AnchorName& a : kAnchors) {
        const bool corner = a.kind == HandleKind::TopLeft || a.kind == HandleKind::TopRight ||
                            a.kind == HandleKind::BottomLeft || a.kind == HandleKind::BottomRight;
        if (cornersAndCentreOnly && !corner && a.kind != HandleKind::Centre) {
            continue;
        }
        combo->addItem(QString::fromUtf8(a.text), static_cast<int>(a.kind));
    }
    combo->setCurrentIndex(std::max(0, combo->findData(static_cast<int>(current))));
    return combo;
}

}  // namespace

// --- Dialog -----------------------------------------------------------------------------------------

TransformDialog::TransformDialog(viewmodel::TransformForm& form, std::function<const document::Document&()> doc, QWidget* parent)
    : QDialog(parent), form_(form), doc_(std::move(doc)) {
    setWindowTitle(titleFor());
    auto* layout = new QVBoxLayout(this);
    body_ = new QWidget(this);
    layout->addWidget(body_);
    message_ = new QLabel(this);
    message_->setStyleSheet(QStringLiteral("color: #b00020"));
    layout->addWidget(message_);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    layout->addWidget(buttons);
    rebuild();
}

QString TransformDialog::titleFor() const {
    switch (form_.kind()) {
        case TransformFormKind::Move: return QStringLiteral("Move Selection");
        case TransformFormKind::Scale: return QStringLiteral("Set Size / Scale");
        case TransformFormKind::Rotate: return QStringLiteral("Rotate");
        case TransformFormKind::Mirror: return QStringLiteral("Mirror");
    }
    return {};
}

void TransformDialog::rebuild() {
    // Rebuilding may run inside a signal of one of the old widgets: retire them later.
    delete body_->layout();
    const auto children = body_->findChildren<QWidget*>(QString(), Qt::FindDirectChildrenOnly);
    for (QWidget* c : children) {
        c->hide();
        c->deleteLater();
    }
    edits_.clear();
    anchor_ = mode_ = axis_ = nullptr;
    absolute_ = link_ = coordinates_ = copy_ = rotated_ = material_ = line_ = nullptr;

    auto* grid = new QFormLayout(body_);
    const viewmodel::TransformFormOptions& o = form_.options();
    const auto tick = [&](const char* text, bool on) {
        auto* box = new QCheckBox(QString::fromUtf8(text), body_);
        box->setChecked(on);
        return box;
    };
    switch (form_.kind()) {
        case TransformFormKind::Move:
            absolute_ = tick("Absolute position (anchor moves to X, Y)", o.absolute);
            anchor_ = anchorCombo(body_, false, o.anchor);
            grid->addRow(absolute_);
            grid->addRow(QStringLiteral("Anchor"), anchor_);
            connect(absolute_, &QCheckBox::toggled, this, [this]() {
                readOptions();
                rebuild();
            });
            break;
        case TransformFormKind::Scale:
            mode_ = new QComboBox(body_);
            mode_->addItem(QStringLiteral("Scale selection"), static_cast<int>(viewmodel::ScaleMode::Selection));
            mode_->addItem(QStringLiteral("Scale items individually"), static_cast<int>(viewmodel::ScaleMode::Individually));
            mode_->setCurrentIndex(std::max(0, mode_->findData(static_cast<int>(o.scaleMode))));
            anchor_ = anchorCombo(body_, false, o.anchor);
            link_ = tick("Link XY", o.linkXY);
            grid->addRow(QStringLiteral("Mode"), mode_);
            grid->addRow(QStringLiteral("Anchor"), anchor_);
            grid->addRow(link_);
            break;
        case TransformFormKind::Rotate:
            absolute_ = tick("Absolute angle", o.absolute);
            coordinates_ = tick("Use coordinates for the pivot", o.useCoordinates);
            anchor_ = anchorCombo(body_, true, o.pivotAnchor);
            grid->addRow(absolute_);
            grid->addRow(QStringLiteral("Pivot"), anchor_);
            grid->addRow(coordinates_);
            connect(absolute_, &QCheckBox::toggled, this, [this]() {
                readOptions();
                rebuild();
            });
            connect(coordinates_, &QCheckBox::toggled, this, [this]() {
                readOptions();
                rebuild();
            });
            break;
        case TransformFormKind::Mirror:
            axis_ = new QComboBox(body_);
            axis_->addItem(QStringLiteral("Horizontal (flip left-right)"), static_cast<int>(MirrorAxis::Horizontal));
            axis_->addItem(QStringLiteral("Vertical (flip top-bottom)"), static_cast<int>(MirrorAxis::Vertical));
            axis_->setCurrentIndex(std::max(0, axis_->findData(static_cast<int>(o.axis))));
            copy_ = tick("Create a mirrored copy", o.createCopy);
            rotated_ = tick("Use rotated bounds", o.useRotatedBounds);
            material_ = tick("About the material centre", o.aboutMaterialCentre);
            line_ = tick("Flip about the last selected line", o.aboutSelectedLine);
            grid->addRow(QStringLiteral("Direction"), axis_);
            grid->addRow(copy_);
            grid->addRow(rotated_);
            grid->addRow(material_);
            grid->addRow(line_);
            break;
    }
    for (const viewmodel::NumericField& f : form_.fields(doc_())) {
        auto* edit = new QLineEdit(body_);
        edit->setText(qs(f.text));
        edit->setPlaceholderText(qs(f.text));
        edits_[f.id] = edit;
        grid->addRow(qs(f.label), edit);
    }
    body_->setLayout(grid);
}

void TransformDialog::readOptions() {
    viewmodel::TransformFormOptions& o = form_.options();
    const auto anchorOf = [](QComboBox* box) { return static_cast<HandleKind>(box->currentData().toInt()); };
    switch (form_.kind()) {
        case TransformFormKind::Move:
            o.absolute = absolute_->isChecked();
            o.anchor = anchorOf(anchor_);
            break;
        case TransformFormKind::Scale:
            o.scaleMode = static_cast<viewmodel::ScaleMode>(mode_->currentData().toInt());
            o.anchor = anchorOf(anchor_);
            o.linkXY = link_->isChecked();
            break;
        case TransformFormKind::Rotate:
            o.absolute = absolute_->isChecked();
            o.useCoordinates = coordinates_->isChecked();
            o.pivotAnchor = anchorOf(anchor_);
            break;
        case TransformFormKind::Mirror:
            o.axis = static_cast<MirrorAxis>(axis_->currentData().toInt());
            o.createCopy = copy_->isChecked();
            o.useRotatedBounds = rotated_->isChecked();
            o.aboutMaterialCentre = material_->isChecked();
            o.aboutSelectedLine = line_->isChecked();
            break;
    }
}

void TransformDialog::accept() {
    readOptions();
    form_.clear();
    for (const auto& [id, edit] : edits_) {
        // An entry equal to the live placeholder is "not typed" (the live value follows the selection).
        const QString text = edit->text().trimmed();
        if (text == edit->placeholderText()) {
            continue;
        }
        const viewmodel::FieldResult r = form_.setField(id, text.toStdString());
        if (!r.ok) {
            message_->setText(qs(r.message));
            edit->setFocus();
            return;
        }
    }
    QDialog::accept();
}

// --- Menu ---------------------------------------------------------------------------------------------

QAction* TransformMenu::add(QMenu* menu, const QString& text, const QString& shortcut) {
    QAction* action = menu->addAction(text);
    if (!shortcut.isEmpty()) {
        action->setShortcut(QKeySequence(shortcut));
    }
    action->setEnabled(false);  // until the menu opens (the window may not have a document yet)
    return action;
}

TransformMenu::TransformMenu(MainWindow& window) : window_(window) {
    const document::Units units = window.currentDocument().job().displayUnits;
    for (TransformFormKind k : {TransformFormKind::Move, TransformFormKind::Scale, TransformFormKind::Rotate,
                                TransformFormKind::Mirror}) {
        forms_.emplace(static_cast<int>(k), viewmodel::TransformForm(k, units));
    }
    menu_ = window.menuBar()->addMenu(QStringLiteral("&Transform"));
    move_ = add(menu_, QStringLiteral("&Move Selection..."), QStringLiteral("M"));
    scale_ = add(menu_, QStringLiteral("Set &Size / Scale..."), QStringLiteral("T"));
    rotate_ = add(menu_, QStringLiteral("&Rotate..."), QStringLiteral("R"));
    menu_->addSeparator();
    struct MirrorEntry {
        MirrorAxis axis;
        bool copy;
        bool material;
        const char* text;
        const char* shortcut;
    };
    const MirrorEntry mirrors[] = {
        {MirrorAxis::Horizontal, false, false, "Mirror &Horizontally", "H"},
        {MirrorAxis::Vertical, false, false, "Mirror &Vertically", "V"},
        {MirrorAxis::Horizontal, true, false, "Mirror Horizontally, Create Copy", "Ctrl+H"},
        {MirrorAxis::Vertical, true, false, "Mirror Vertically, Create Copy", "Ctrl+V"},
        {MirrorAxis::Horizontal, false, true, "Mirror Horizontally about the Material Centre", "Shift+H"},
        {MirrorAxis::Vertical, false, true, "Mirror Vertically about the Material Centre", "Shift+V"},
    };
    for (const MirrorEntry& e : mirrors) {
        QAction* a = add(menu_, QString::fromUtf8(e.text), QString::fromUtf8(e.shortcut));
        mirror_[mirrorKey(e.axis, e.copy, e.material)] = a;
        const viewmodel::MirrorOptions options = [&] {
            viewmodel::MirrorOptions o;
            o.axis = e.axis;
            o.createCopy = e.copy;
            o.aboutMaterialCentre = e.material;
            return o;
        }();
        QObject::connect(a, &QAction::triggered, menu_, [this, options]() { runMirror(options); });
    }
    QAction* mirrorDialog = add(menu_, QStringLiteral("&Mirror..."), QString());
    mirror_[-1] = mirrorDialog;
    QObject::connect(mirrorDialog, &QAction::triggered, menu_, [this]() { openDialog(TransformFormKind::Mirror); });

    alignMenu_ = menu_->addMenu(QStringLiteral("&Align"));
    struct AlignEntry {
        AlignKind kind;
        const char* text;
    };
    const AlignEntry aligns[] = {
        {AlignKind::Centre, "Align to Last: &Centre"},
        {AlignKind::VerticalCentre, "Align to Last: &Vertical Centre"},
        {AlignKind::HorizontalCentre, "Align to Last: &Horizontal Centre"},
        {AlignKind::Left, "Align to Last: &Left"},
        {AlignKind::Right, "Align to Last: &Right"},
        {AlignKind::OutsideTop, "Align to Last: Outside &Top"},
        {AlignKind::OutsideBottom, "Align to Last: Outside &Bottom"},
    };
    for (const AlignEntry& e : aligns) {
        QAction* a = add(alignMenu_, QString::fromUtf8(e.text), QString());
        align_[static_cast<int>(e.kind)] = a;
        const AlignKind kind = e.kind;
        QObject::connect(a, &QAction::triggered, menu_, [this, kind]() { runAlign(kind); });
    }
    alignMenu_->addSeparator();
    struct CentreEntry {
        CentreAxes axes;
        const char* text;
        const char* shortcut;
    };
    const CentreEntry centres[] = {
        {CentreAxes::Both, "Centre in &Material", "F9"},
        {CentreAxes::XOnly, "Centre in Material, X only", ""},
        {CentreAxes::YOnly, "Centre in Material, Y only", ""},
    };
    for (const CentreEntry& e : centres) {
        QAction* a = add(alignMenu_, QString::fromUtf8(e.text), QString::fromUtf8(e.shortcut));
        centre_[static_cast<int>(e.axes)] = a;
        const CentreAxes axes = e.axes;
        QObject::connect(a, &QAction::triggered, menu_, [this, axes]() { runCentre(axes); });
    }
    alignMenu_->addSeparator();
    for (SpaceDirection d : {SpaceDirection::Horizontal, SpaceDirection::Vertical}) {
        QAction* a = add(alignMenu_,
                         d == SpaceDirection::Horizontal ? QStringLiteral("Space Selection &Horizontally")
                                                         : QStringLiteral("Space Selection V&ertically"),
                         QString());
        space_[static_cast<int>(d)] = a;
        QObject::connect(a, &QAction::triggered, menu_, [this, d]() { runSpace(d); });
    }
    spaceOrder_ = alignMenu_->addAction(QStringLiteral("Space in Selection Order (Ctrl)"));
    spaceOrder_->setCheckable(true);
    spaceInside_ = alignMenu_->addAction(QStringLiteral("Space Inside Last Vector"));
    spaceInside_->setCheckable(true);

    // F10 pops the alignment tools up where the pointer is.
    QAction* f10 = menu_->addAction(QStringLiteral("Alignment Tools"));
    f10->setShortcut(QKeySequence(QStringLiteral("F10")));
    QObject::connect(f10, &QAction::triggered, menu_, [this]() { alignMenu_->popup(QCursor::pos()); });

    QObject::connect(move_, &QAction::triggered, menu_, [this]() { openDialog(TransformFormKind::Move); });
    QObject::connect(scale_, &QAction::triggered, menu_, [this]() { openDialog(TransformFormKind::Scale); });
    QObject::connect(rotate_, &QAction::triggered, menu_, [this]() { openDialog(TransformFormKind::Rotate); });
    QObject::connect(menu_, &QMenu::aboutToShow, menu_, [this]() { refresh(); });
    QObject::connect(alignMenu_, &QMenu::aboutToShow, alignMenu_, [this]() { refresh(); });
    refresh();  // the window refreshes again on every selection and document change
}

void TransformMenu::refresh() {
    const document::Document& doc = window_.currentDocument();
    const bool any = viewmodel::canTransform(doc);
    const std::size_t operands = viewmodel::transformTargets(doc).size();
    move_->setEnabled(any);
    scale_->setEnabled(any);
    rotate_->setEnabled(any);
    for (auto& entry : mirror_) {
        entry.second->setEnabled(any);
    }
    for (auto& entry : centre_) {
        entry.second->setEnabled(any);
    }
    // Align / space need several objects; groups count once, which the operation checks itself.
    const bool several = operands >= 2 || doc.selection().size() >= 2;
    for (auto& entry : align_) {
        entry.second->setEnabled(several);
    }
    for (auto& entry : space_) {
        entry.second->setEnabled(doc.selection().size() >= 2);
    }
}

QAction* TransformMenu::mirrorAction(MirrorAxis axis, bool copy, bool aboutMaterial) const {
    const auto it = mirror_.find(mirrorKey(axis, copy, aboutMaterial));
    return it == mirror_.end() ? nullptr : it->second;
}
QAction* TransformMenu::alignAction(AlignKind kind) const {
    const auto it = align_.find(static_cast<int>(kind));
    return it == align_.end() ? nullptr : it->second;
}
QAction* TransformMenu::centreAction(CentreAxes axes) const {
    const auto it = centre_.find(static_cast<int>(axes));
    return it == centre_.end() ? nullptr : it->second;
}
QAction* TransformMenu::spaceAction(SpaceDirection direction) const {
    const auto it = space_.find(static_cast<int>(direction));
    return it == space_.end() ? nullptr : it->second;
}

viewmodel::TransformForm& TransformMenu::form(TransformFormKind kind) {
    viewmodel::TransformForm& f = forms_.at(static_cast<int>(kind));
    f.setUnits(window_.currentDocument().job().displayUnits);
    return f;
}

void TransformMenu::openDialog(TransformFormKind kind) {
    viewmodel::TransformForm& f = form(kind);
    f.clear();
    const MainWindow& win = window_;
    TransformDialog dialog(f, [&win]() -> const document::Document& { return win.currentDocument(); }, &window_);
    if (dialog.exec() == QDialog::Accepted) {
        run(f);
    }
}

TransformResult TransformMenu::run(viewmodel::TransformForm& f, bool showErrors) {
    TransformResult result = f.apply(window_.currentDocument());
    report(result, showErrors);
    return result;
}

TransformResult TransformMenu::runMirror(const viewmodel::MirrorOptions& options, bool showErrors) {
    TransformResult result = viewmodel::mirrorSelection(window_.currentDocument(), options);
    report(result, showErrors);
    return result;
}

TransformResult TransformMenu::runAlign(AlignKind kind, bool showErrors) {
    TransformResult result = viewmodel::alignToLast(window_.currentDocument(), kind);
    report(result, showErrors);
    return result;
}

TransformResult TransformMenu::runCentre(CentreAxes axes, bool showErrors) {
    TransformResult result = viewmodel::centreInMaterial(window_.currentDocument(), axes);
    report(result, showErrors);
    return result;
}

TransformResult TransformMenu::runSpace(SpaceDirection direction, bool showErrors) {
    viewmodel::SpaceOptions options;
    options.direction = direction;
    options.selectionOrder = spaceOrder_->isChecked();
    options.insideLast = spaceInside_->isChecked();
    TransformResult result = viewmodel::spaceSelection(window_.currentDocument(), options);
    report(result, showErrors);
    return result;
}

void TransformMenu::report(const TransformResult& result, bool showErrors) {
    if (result.ok) {
        window_.documentChanged();
        window_.canvas()->update();
    } else if (showErrors) {
        QMessageBox::warning(&window_, QStringLiteral("Transform"), qs(result.message));
    }
}

// --- Self-test ----------------------------------------------------------------------------------------

namespace {

void sendMouseEvent(CanvasWidget* canvas, QEvent::Type type, QPointF pos, Qt::KeyboardModifiers mods = Qt::NoModifier) {
    const bool move = type == QEvent::MouseMove;
    const Qt::MouseButtons buttons = type == QEvent::MouseButtonRelease ? Qt::NoButton : Qt::LeftButton;
    QMouseEvent event(type, pos, canvas->mapToGlobal(pos), move ? Qt::NoButton : Qt::LeftButton, buttons, mods);
    QCoreApplication::sendEvent(canvas, &event);
}

QPointF pixelOf(const CanvasWidget* canvas, geometry::Vec2 mm) {
    const geometry::Vec2 p = canvas->view().worldToScreen(mm);
    return QPointF(p.x, p.y);
}

geometry::Box2 boundsOfObject(const document::Document& doc, document::ObjectId id) {
    geometry::Box2 b;
    if (const document::Object* o = doc.findObject(id); o != nullptr && o->asVector() != nullptr) {
        b = o->asVector()->bounds();
    }
    return b;
}

}  // namespace

std::string transformSelfTest(MainWindow& window) {
    TransformMenu* menu = window.transformMenu();
    if (menu == nullptr) {
        return "the main window has no Transform menu";
    }
    document::Document& doc = window.currentDocument();
    CanvasWidget* canvas = window.canvas();
    const std::size_t objects0 = doc.objectCount();
    const std::size_t steps0 = doc.undoStack().index();
    const geometry::Box2 material = doc.findSheet(doc.activeSheet())->materialBoundsMm();
    const double unit = material.width() * 0.05;

    window.selectTool(viewmodel::ToolKind::Select);
    const auto addRect = [&](geometry::Box2 box) {
        auto cmd = std::make_unique<document::AddObjectCommand>(
            document::Object::vector(geometry::Contour::rectangle(box), doc.activeLayer(), doc.activeSheet()));
        const document::AddObjectCommand* raw = cmd.get();
        doc.execute(std::move(cmd));
        return raw->id();
    };
    const geometry::Vec2 o{material.min.x + 2.0 * unit, material.min.y + 2.0 * unit};
    const document::ObjectId a = addRect(geometry::Box2{o, {o.x + 3.0 * unit, o.y + 2.0 * unit}});
    const document::ObjectId b =
        addRect(geometry::Box2{{o.x + 8.0 * unit, o.y + 6.0 * unit}, {o.x + 10.0 * unit, o.y + 9.0 * unit}});
    // Select A with a real click on its bottom edge: the shortcut actions must become usable
    // through the window's own change notifications (no manual refresh).
    sendMouseEvent(canvas, QEvent::MouseMove, pixelOf(canvas, {o.x + 1.5 * unit, o.y}));
    sendMouseEvent(canvas, QEvent::MouseButtonPress, pixelOf(canvas, {o.x + 1.5 * unit, o.y}));
    sendMouseEvent(canvas, QEvent::MouseButtonRelease, pixelOf(canvas, {o.x + 1.5 * unit, o.y}));
    if (doc.selection().size() != 1 || !doc.selection().contains(a)) {
        return "clicking the rectangle did not select it";
    }
    const auto byShortcut = [&](const char* keys) -> QAction* {
        for (QAction* action : window.menuBar()->findChildren<QAction*>()) {
            if (action->shortcut() == QKeySequence(QString::fromLatin1(keys))) {
                return action;
            }
        }
        return nullptr;
    };
    for (const char* keys : {"H", "V", "M", "T", "R", "F9"}) {
        QAction* action = byShortcut(keys);
        if (action == nullptr || !action->isEnabled()) {
            return std::string("the shortcut ") + keys + " is not usable with a selection";
        }
    }
    {
        QAction* flip = byShortcut("H");
        const std::size_t count = doc.undoStack().index();
        flip->trigger();
        if (doc.undoStack().index() != count + 1 || !window.undo() || doc.undoStack().index() != count) {
            return "the H shortcut did not mirror in one undo step";
        }
    }
    QCoreApplication::processEvents();

    // 1. Drag the top-right handle of A: proportional scale about the opposite corner, one undo step.
    sendMouseEvent(canvas, QEvent::MouseMove, pixelOf(canvas, o));
    const auto handles = canvas->tools().selectionHandles();
    if (!handles.has_value()) {
        return "the selection shows no handles";
    }
    const geometry::Box2 before = boundsOfObject(doc, a);
    const geometry::Vec2 corner = handles->handles[static_cast<std::size_t>(viewmodel::HandleKind::TopRight)].positionMm;
    const geometry::Vec2 anchor = before.min;
    const geometry::Vec2 target = anchor + (corner - anchor) * 1.5;
    const std::size_t stepsBefore = doc.undoStack().index();
    sendMouseEvent(canvas, QEvent::MouseMove, pixelOf(canvas, corner));
    sendMouseEvent(canvas, QEvent::MouseButtonPress, pixelOf(canvas, corner));
    for (double t : {0.25, 0.5, 0.75, 1.0}) {
        sendMouseEvent(canvas, QEvent::MouseMove, pixelOf(canvas, corner + (target - corner) * t));
    }
    sendMouseEvent(canvas, QEvent::MouseButtonRelease, pixelOf(canvas, target));
    const geometry::Box2 scaled = boundsOfObject(doc, a);
    if (doc.undoStack().index() != stepsBefore + 1 || std::fabs(scaled.width() - 1.5 * before.width()) > 1e-6 ||
        std::fabs(scaled.height() - 1.5 * before.height()) > 1e-6 || std::fabs(scaled.min.x - before.min.x) > 1e-6) {
        return "dragging the corner handle did not scale the rectangle in one undo step";
    }

    // 2. Numeric move through the form: 10 mm right, 4 mm up.
    viewmodel::TransformForm& move = menu->form(viewmodel::TransformFormKind::Move);
    move.clear();
    if (!move.setField("x", "10mm").ok || !move.setField("y", "4mm").ok || move.setField("x", "abc").ok) {
        return "the move form did not validate its entries";
    }
    const viewmodel::TransformResult moved = menu->run(move, false);
    const geometry::Box2 after = boundsOfObject(doc, a);
    if (!moved.ok || std::fabs(after.min.x - (scaled.min.x + 10.0)) > 1e-6 ||
        std::fabs(after.min.y - (scaled.min.y + 4.0)) > 1e-6) {
        return "the numeric move gave the wrong position: " + moved.message;
    }

    // 3. Align A to the last-selected B (left edges), through the real menu action.
    doc.selection().assign({a, b});
    window.documentChanged();
    QAction* left = menu->alignAction(viewmodel::AlignKind::Left);
    if (left == nullptr || !left->isEnabled()) {
        return "Align Left is not enabled for two objects";
    }
    left->trigger();
    if (std::fabs(boundsOfObject(doc, a).min.x - boundsOfObject(doc, b).min.x) > 1e-6) {
        return "Align Left did not line the left edges up";
    }

    // 4. Undo the three steps and the two additions: the document is as it was.
    for (int i = 0; i < 5; ++i) {
        if (!window.undo()) {
            return "undoing the transform steps failed";
        }
    }
    if (doc.objectCount() != objects0 || doc.undoStack().index() < steps0) {
        return "undo did not restore the document";
    }
    doc.selection().clear();
    window.documentChanged();
    return {};
}

}  // namespace carvetto::app
