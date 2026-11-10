#include "dimension_menu.hpp"

#include <QAction>
#include <QApplication>
#include <QCoreApplication>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QTemporaryFile>
#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QPushButton>
#include <QSpinBox>
#include <QStatusBar>
#include <QString>
#include <QVBoxLayout>
#include <QVariant>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <vector>

#include "canvas_widget.hpp"
#include "carvetto/document/dimension.hpp"
#include "carvetto/document/text_commands.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/viewmodel/canvas_scene.hpp"
#include "carvetto/viewmodel/dimension_text.hpp"
#include "carvetto/viewmodel/starter_font.hpp"
#include "carvetto/viewmodel/tools.hpp"
#include "carvetto/viewmodel/units_format.hpp"
#include "main_window.hpp"
#include "tool_panel.hpp"

namespace carvetto::app {

using document::DimensionKind;
using viewmodel::DimensionOptions;

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

const char* kindName(DimensionKind k) {
    switch (k) {
        case DimensionKind::Linear: return "Linear (horizontal / vertical)";
        case DimensionKind::Aligned: return "Aligned";
        case DimensionKind::Angular: return "Angular";
        case DimensionKind::Radial: return "Radius / diameter";
    }
    return "";
}

}  // namespace

DimensionDialog::DimensionDialog(document::Document& doc, const DimensionOptions& initial, bool editing,
                                 QWidget* parent)
    : QDialog(parent), doc_(doc), editing_(editing), units_(doc.job().displayUnits), initial_(initial) {
    setWindowTitle(editing ? QStringLiteral("Edit Dimension") : QStringLiteral("Dimension Options"));
    const QString suffix = QString::fromUtf8(document::unitSuffix(units_).data(),
                                             static_cast<int>(document::unitSuffix(units_).size()));
    auto* form = new QFormLayout;
    kind_ = new QComboBox(this);
    kind_->setObjectName(QStringLiteral("dimKind"));
    for (const DimensionKind k : {DimensionKind::Linear, DimensionKind::Aligned, DimensionKind::Angular,
                                  DimensionKind::Radial}) {
        kind_->addItem(QString::fromUtf8(kindName(k)), static_cast<int>(k));
    }
    kind_->setCurrentIndex(kind_->findData(static_cast<int>(initial.kind)));
    kind_->setEnabled(!editing);  // the kind is fixed once drawn
    form->addRow(QStringLiteral("Kind"), kind_);
    vertical_ = new QCheckBox(QStringLiteral("Linear: measure vertically"), this);
    vertical_->setObjectName(QStringLiteral("dimVertical"));
    vertical_->setChecked(initial.vertical);
    vertical_->setEnabled(!editing);
    diameter_ = new QCheckBox(QStringLiteral("Radial: show the diameter"), this);
    diameter_->setObjectName(QStringLiteral("dimDiameter"));
    diameter_->setChecked(initial.diameter);
    diameter_->setEnabled(!editing);
    form->addRow(vertical_);
    form->addRow(diameter_);
    decimals_ = new QSpinBox(this);
    decimals_->setObjectName(QStringLiteral("dimDecimals"));
    decimals_->setRange(0, document::kMaxDimensionDecimals);
    decimals_->setValue(initial.decimals);
    form->addRow(QStringLiteral("Decimal places"), decimals_);
    angleDecimals_ = new QSpinBox(this);
    angleDecimals_->setObjectName(QStringLiteral("dimAngleDecimals"));
    angleDecimals_->setRange(0, document::kMaxDimensionAngleDecimals);
    angleDecimals_->setValue(initial.angleDecimals);
    form->addRow(QStringLiteral("Angle decimal places"), angleDecimals_);
    textHeight_ = new QLineEdit(viewmodel::formatLength(initial.textHeightMm, units_).c_str(), this);
    textHeight_->setObjectName(QStringLiteral("dimTextHeight"));
    form->addRow(QStringLiteral("Text height (%1)").arg(suffix), textHeight_);
    offset_ = new QLineEdit(this);
    offset_->setObjectName(QStringLiteral("dimOffset"));
    reflex_ = new QCheckBox(QStringLiteral("Angular: measure the reflex angle (more than 180 degrees)"), this);
    reflex_->setObjectName(QStringLiteral("dimReflex"));
    if (editing) {
        offset_->setText(qs(viewmodel::formatLength(initial.offsetMm.value_or(0.0), units_)));
        offset_->setToolTip(QStringLiteral("Dimension line position, arc radius or leader length"));
        form->addRow(QStringLiteral("Offset (%1)").arg(suffix), offset_);
        reflex_->setChecked(initial.reflex.value_or(false));
        reflex_->setEnabled(initial.kind == DimensionKind::Angular);
        form->addRow(reflex_);
    }
    useCustom_ = new QCheckBox(QStringLiteral("Use custom text"), this);
    useCustom_->setObjectName(QStringLiteral("dimUseCustom"));
    useCustom_->setChecked(initial.useCustomText);
    customText_ = new QLineEdit(qs(initial.customText), this);
    customText_->setObjectName(QStringLiteral("dimCustomText"));
    customText_->setMaxLength(static_cast<int>(document::kMaxDimensionTextBytes / 4));
    form->addRow(useCustom_);
    form->addRow(QStringLiteral("Custom text"), customText_);
    font_ = new QComboBox(this);
    font_->setObjectName(QStringLiteral("dimFont"));
    font_->addItem(QStringLiteral("Default (%1)").arg(QString::fromUtf8(viewmodel::starterFontName())), QString());
    for (const auto& [key, asset] : doc.fonts()) {
        font_->addItem(qs(viewmodel::dimensionFontName(doc, key)), qs(key));
    }
    font_->setCurrentIndex(std::max(0, font_->findData(qs(initial.fontKey))));
    auto* addFont = new QPushButton(QStringLiteral("Add font file..."), this);
    addFont->setObjectName(QStringLiteral("dimAddFont"));
    addFont->setToolTip(QStringLiteral("Embeds a TrueType/OpenType font in the document"));
    connect(addFont, &QPushButton::clicked, this, [this]() {
        const QString path = QFileDialog::getOpenFileName(this, QStringLiteral("Add font"), QString(),
                                                          QStringLiteral("Fonts (*.ttf *.otf)"));
        if (!path.isEmpty()) {
            error_->setText(qs(addFontFile(path)));
        }
    });
    auto* fontRow = new QWidget(this);
    auto* fontLayout = new QVBoxLayout(fontRow);
    fontLayout->setContentsMargins(0, 0, 0, 0);
    fontLayout->addWidget(font_);
    fontLayout->addWidget(addFont);
    form->addRow(QStringLiteral("Font"), fontRow);
    layer_ = new QComboBox(this);
    layer_->setObjectName(QStringLiteral("dimLayer"));
    layer_->addItem(QStringLiteral("Dimensions (created when missing)"), QVariant::fromValue<qulonglong>(0));
    for (const document::Layer& l : doc.layers()) {
        layer_->addItem(qs(l.name), QVariant::fromValue<qulonglong>(l.id.value));
    }
    layer_->setCurrentIndex(std::max(0, layer_->findData(QVariant::fromValue<qulonglong>(initial.layer.value))));
    if (!editing) {
        form->addRow(QStringLiteral("Place on layer"), layer_);
    }
    error_ = new QLabel(this);
    error_->setObjectName(QStringLiteral("dimError"));
    error_->setStyleSheet(QStringLiteral("color: #b00020;"));
    error_->setWordWrap(true);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    auto* layout = new QVBoxLayout(this);
    layout->addLayout(form);
    layout->addWidget(error_);
    layout->addWidget(buttons);
}

DimensionOptions DimensionDialog::options() const {
    DimensionOptions o = initial_;
    o.kind = static_cast<DimensionKind>(kind_->currentData().toInt());
    o.vertical = vertical_->isChecked();
    o.diameter = diameter_->isChecked();
    o.decimals = decimals_->value();
    o.angleDecimals = angleDecimals_->value();
    const viewmodel::ParsedNumber h =
        viewmodel::parseNumber(textHeight_->text().toStdString(), viewmodel::FieldKind::Length, units_);
    o.textHeightMm = h.ok ? h.value : 0.0;
    o.useCustomText = useCustom_->isChecked();
    o.customText = customText_->text().toStdString();
    o.fontKey = font_->currentData().toString().toStdString();
    if (editing_) {
        const viewmodel::ParsedNumber off =
            viewmodel::parseNumber(offset_->text().toStdString(), viewmodel::FieldKind::Length, units_);
        o.offsetMm = off.ok ? off.value : std::numeric_limits<double>::quiet_NaN();
        o.reflex = o.kind == DimensionKind::Angular ? std::optional<bool>(reflex_->isChecked()) : std::nullopt;
    } else {
        o.layer = document::LayerId{layer_->currentData().toULongLong()};
    }
    return o;
}

std::string DimensionDialog::problem() const {
    const viewmodel::ParsedNumber h =
        viewmodel::parseNumber(textHeight_->text().toStdString(), viewmodel::FieldKind::Length, units_);
    if (!h.ok) {
        return "text height: " + h.error;
    }
    if (editing_) {
        const viewmodel::ParsedNumber off =
            viewmodel::parseNumber(offset_->text().toStdString(), viewmodel::FieldKind::Length, units_);
        if (!off.ok) {
            return "offset: " + off.error;
        }
    }
    const DimensionOptions o = options();
    document::DimensionObject probe;
    probe.kind = o.kind;
    probe.decimals = o.decimals;
    probe.angleDecimals = o.angleDecimals;
    probe.textHeightMm = o.textHeightMm;
    probe.customText = o.customText;
    if (o.offsetMm) {
        probe.offsetMm = *o.offsetMm;
    }
    return document::dimensionProblem(probe);
}

std::string DimensionDialog::addFontFile(const QString& path) {
    const QFileInfo info(path);
    if (!info.isFile() || info.size() <= 0 || static_cast<std::size_t>(info.size()) > document::kMaxEmbeddedFontBytes) {
        return "the font file is missing, empty or larger than " +
               std::to_string(document::kMaxEmbeddedFontBytes >> 20) + " MB";
    }
    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        return "the font file cannot be read";
    }
    const QByteArray raw = file.readAll();
    document::FontAsset asset;
    asset.bytes.assign(reinterpret_cast<const std::uint8_t*>(raw.constData()),
                       reinterpret_cast<const std::uint8_t*>(raw.constData()) + raw.size());
    asset.key = document::fontKeyFor(asset.bytes);
    if (!text::Font::load(asset.bytes).ok()) {
        return "not a usable TrueType/OpenType font";
    }
    if (const std::string why = doc_.fontProblem(asset); !why.empty()) {
        return why;
    }
    doc_.execute(std::make_unique<document::AddFontCommand>(asset));
    int row = font_->findData(qs(asset.key));
    if (row < 0) {
        font_->addItem(qs(viewmodel::dimensionFontName(doc_, asset.key)), qs(asset.key));
        row = font_->count() - 1;
    }
    font_->setCurrentIndex(row);
    return {};
}

void DimensionDialog::accept() {
    const std::string why = problem();
    error_->setText(qs(why));
    if (!why.empty()) {
        return;
    }
    QDialog::accept();
}

// ---------------------------------------------------------------------------------------------

DimensionMenu::DimensionMenu(MainWindow& window, QMenu* editMenu) : window_(window) {
    QMenu* menu = editMenu->addMenu(QStringLiteral("&Dimensions"));
    tool_ = menu->addAction(QStringLiteral("Dimension &Tool"));
    options_ = menu->addAction(QStringLiteral("&Options..."));
    menu->addSeparator();
    edit_ = menu->addAction(QStringLiteral("&Edit Selected Dimension..."));
    delete_ = menu->addAction(QStringLiteral("&Delete Selected Dimension"));
    QObject::connect(tool_, &QAction::triggered, menu, [this]() { window_.selectTool(viewmodel::ToolKind::Dimension); });
    QObject::connect(options_, &QAction::triggered, menu, [this]() { openOptions(); });
    QObject::connect(edit_, &QAction::triggered, menu, [this]() { openEdit(); });
    QObject::connect(delete_, &QAction::triggered, menu, [this]() { deleteSelected(); });
    QObject::connect(menu, &QMenu::aboutToShow, menu, [this]() { refresh(); });
    edit_->setEnabled(false);
    delete_->setEnabled(false);
}

void DimensionMenu::refresh() {
    viewmodel::DimensionTool& dt = window_.canvas()->tools().dimensionTool();
    const document::Document& doc = window_.currentDocument();
    bool any = false;
    for (const document::ObjectId id : doc.selection().ids()) {
        const document::Object* obj = doc.findObject(id);
        any = any || (obj != nullptr && obj->asDimension() != nullptr);
    }
    edit_->setEnabled(doc.selection().single().value != 0 && dt.optionsOf(doc.selection().single()).has_value());
    delete_->setEnabled(any);
}

void DimensionMenu::applyToolOptions(const DimensionOptions& options) {
    const viewmodel::DimensionToolResult r = window_.canvas()->tools().dimensionTool().setOptions(options);
    if (!r.ok) {
        window_.statusBar()->showMessage(qs(r.message), 6000);
    }
    window_.toolPanel()->rebuild();
}

bool DimensionMenu::applyEdit(document::ObjectId id, const DimensionOptions& options) {
    const viewmodel::DimensionToolResult r = window_.canvas()->tools().dimensionTool().edit(id, options);
    if (!r.ok) {
        window_.statusBar()->showMessage(qs(r.message), 6000);
        return false;
    }
    if (r.committed) {
        window_.documentChanged();
    }
    return true;
}

bool DimensionMenu::deleteSelected() {
    const viewmodel::DimensionToolResult r = window_.canvas()->tools().dimensionTool().removeSelected();
    if (!r.ok) {
        window_.statusBar()->showMessage(qs(r.message), 6000);
        return false;
    }
    window_.documentChanged();
    return true;
}

void DimensionMenu::openOptions() {
    DimensionDialog dialog(window_.currentDocument(), window_.canvas()->tools().dimensionTool().options(), false,
                           &window_);
    if (dialog.exec() == QDialog::Accepted) {
        applyToolOptions(dialog.options());
    }
}

void DimensionMenu::openEdit() {
    document::Document& doc = window_.currentDocument();
    const document::ObjectId id = doc.selection().single();
    const auto current = window_.canvas()->tools().dimensionTool().optionsOf(id);
    if (!current) {
        return;
    }
    DimensionDialog dialog(doc, *current, true, &window_);
    if (dialog.exec() == QDialog::Accepted) {
        applyEdit(id, dialog.options());
    }
}


// --- Self-test -------------------------------------------------------------------------------

namespace {

#define DIM_REQUIRE(cond, text)                          \
    do {                                                 \
        if (!(cond)) {                                   \
            return std::string("dimensions: ") + (text); \
        }                                                \
    } while (false)

QPointF screenOf(const CanvasWidget* canvas, geometry::Vec2 mm) {
    const geometry::Vec2 p = canvas->view().worldToScreen(mm);
    return QPointF(p.x, p.y);
}

void sendMouse(CanvasWidget* canvas, QEvent::Type type, QPointF pos, Qt::KeyboardModifiers mods) {
    const Qt::MouseButtons buttons = type == QEvent::MouseButtonRelease ? Qt::NoButton : Qt::LeftButton;
    QMouseEvent event(type, pos, canvas->mapToGlobal(pos), type == QEvent::MouseMove ? Qt::NoButton : Qt::LeftButton,
                      buttons, mods);
    QCoreApplication::sendEvent(canvas, &event);
}

void click(CanvasWidget* canvas, geometry::Vec2 mm, Qt::KeyboardModifiers mods = Qt::NoModifier) {
    const QPointF p = screenOf(canvas, mm);
    sendMouse(canvas, QEvent::MouseMove, p, mods);
    sendMouse(canvas, QEvent::MouseButtonPress, p, mods);
    sendMouse(canvas, QEvent::MouseButtonRelease, p, mods);
}

void drag(CanvasWidget* canvas, geometry::Vec2 from, geometry::Vec2 to) {
    const QPointF a = screenOf(canvas, from);
    const QPointF b = screenOf(canvas, to);
    sendMouse(canvas, QEvent::MouseMove, a, Qt::NoModifier);
    sendMouse(canvas, QEvent::MouseButtonPress, a, Qt::NoModifier);
    sendMouse(canvas, QEvent::MouseMove, (a + b) * 0.5, Qt::NoModifier);
    sendMouse(canvas, QEvent::MouseMove, b, Qt::NoModifier);
    sendMouse(canvas, QEvent::MouseButtonRelease, b, Qt::NoModifier);
}

void key(CanvasWidget* canvas, Qt::Key k) {
    QKeyEvent press(QEvent::KeyPress, k, Qt::NoModifier);
    QCoreApplication::sendEvent(canvas, &press);
}

const document::DimensionObject* dimOf(const document::Document& doc, document::ObjectId id) {
    const document::Object* o = doc.findObject(id);
    return o != nullptr ? o->asDimension() : nullptr;
}

std::size_t dimensionCount(const document::Document& doc) {
    std::size_t n = 0;
    for (const auto& [id, obj] : doc.objects()) {
        if (obj.asDimension() != nullptr) {
            ++n;
        }
    }
    return n;
}

}  // namespace

std::string dimensionSelfTest(MainWindow& window) {
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    QCoreApplication::processEvents();
    CanvasWidget* canvas = window.canvas();
    document::Document& doc = window.currentDocument();
    viewmodel::DimensionTool& dt = canvas->tools().dimensionTool();

    // The tool and the menu entries exist.
    DIM_REQUIRE(window.toolAction(viewmodel::ToolKind::Dimension) != nullptr, "no Dimension toolbar action");
    DIM_REQUIRE(window.dimensions() != nullptr && window.dimensions()->toolAction() != nullptr, "no Edit > Dimensions");
    window.dimensions()->toolAction()->trigger();
    DIM_REQUIRE(canvas->tools().tool() == viewmodel::ToolKind::Dimension, "the menu did not select the tool");
    DIM_REQUIRE(window.toolAction(viewmodel::ToolKind::Dimension)->isChecked(), "toolbar action not checked");
    auto* kindCombo = window.toolPanel()->findChild<QComboBox*>(QStringLiteral("dimensionKindCombo"));
    DIM_REQUIRE(kindCombo != nullptr && kindCombo->count() == 6, "the tool panel has no dimension kind combo");

    // Options dialog: hostile entries are refused, valid ones reach the tool.
    {
        DimensionDialog dialog(doc, dt.options(), false, &window);
        auto* height = dialog.findChild<QLineEdit*>(QStringLiteral("dimTextHeight"));
        auto* decimals = dialog.findChild<QSpinBox*>(QStringLiteral("dimDecimals"));
        auto* custom = dialog.findChild<QLineEdit*>(QStringLiteral("dimCustomText"));
        DIM_REQUIRE(height != nullptr && decimals != nullptr && custom != nullptr, "dialog widgets missing");
        for (const char* bad : {"abc", "0", "-3", "1e12", "nan", "", "20000"}) {
            height->setText(QString::fromUtf8(bad));
            DIM_REQUIRE(!dialog.problem().empty(), "a hostile text height was accepted");
            dialog.accept();
            DIM_REQUIRE(dialog.result() != QDialog::Accepted, "OK closed the dialog on a hostile entry");
        }
        custom->setMaxLength(100000);  // the field itself caps input; lift that so problem() is what is tested
        custom->setText(QString(5000, QLatin1Char('x')));
        height->setText(QStringLiteral("4"));
        DIM_REQUIRE(!dialog.problem().empty(), "an over-long custom text was accepted");
        custom->setText(QStringLiteral("TYP"));
        decimals->setValue(3);
        DIM_REQUIRE(dialog.problem().empty(), "valid entries were refused");
        window.dimensions()->applyToolOptions(dialog.options());
        DIM_REQUIRE(dt.options().decimals == 3 && dt.options().textHeightMm == 4.0, "the options did not reach the tool");
        // Back to measured text for the placement checks below.
        auto o = dt.options();
        o.useCustomText = false;
        o.decimals = 2;
        window.dimensions()->applyToolOptions(o);
    }
    kindCombo = window.toolPanel()->findChild<QComboBox*>(QStringLiteral("dimensionKindCombo"));
    DIM_REQUIRE(kindCombo != nullptr, "the tool panel was not rebuilt");

    // Place each kind through real mouse events: one undo step each, on the layer "Dimensions".
    struct Place {
        int index;
        std::vector<geometry::Vec2> clicks;
        const char* text;
    };
    const Place places[] = {
        {0, {{50, 50}, {150, 50}, {100, 80}}, "100.00"},                          // aligned
        {1, {{50, 110}, {120, 140}, {80, 170}}, "70.00"},                         // horizontal
        {2, {{200, 110}, {230, 190}, {180, 150}}, "80.00"},                       // vertical
        {3, {{220, 50}, {270, 50}, {220, 100}, {240, 70}}, "90.00\xC2\xB0"},      // angular
        {3, {{220, 160}, {270, 160}, {220, 210}, {200, 140}}, "270.00\xC2\xB0"},  // reflex angular
        {4, {{60, 230}, {90, 230}, {110, 230}}, "R30.00"},                        // radius
        {5, {{150, 240}, {170, 240}, {200, 240}}, "\xC3\x98" "40.00"},            // diameter
    };
    std::vector<document::ObjectId> ids;
    for (const Place& p : places) {
        kindCombo->setCurrentIndex(p.index);
        const std::size_t steps = doc.undoStack().index();
        for (const geometry::Vec2 c : p.clicks) {
            click(canvas, c);
        }
        const document::ObjectId id = doc.selection().single();
        DIM_REQUIRE(!id.isNull() && dimOf(doc, id) != nullptr, "placing a dimension did not select it");
        // The first dimension also creates the layer; either way it is a single step.
        DIM_REQUIRE(doc.undoStack().index() == steps + 1, "placing a dimension was not one undo step");
        DIM_REQUIRE(document::dimensionText(doc, *dimOf(doc, id), doc.job().displayUnits) == p.text,
                    std::string("wrong dimension text, expected ") + p.text);
        DIM_REQUIRE(doc.findLayer(doc.findObject(id)->layer)->name == "Dimensions", "not on the Dimensions layer");
        ids.push_back(id);
    }
    DIM_REQUIRE(dimensionCount(doc) == 7, "expected seven dimensions");

    // The canvas draws their text.
    {
        const viewmodel::CanvasScene scene = viewmodel::buildCanvasScene(doc, canvas->view());
        DIM_REQUIRE(scene.labels.size() == 7, "the canvas scene has no text for every dimension");
        for (const viewmodel::DrawLabel& l : scene.labels) {
            DIM_REQUIRE(!l.outlinesPx.empty(), "a dimension text has no outlines");
        }
        const QImage image = canvas->renderFitted(QSize(900, 700));
        DIM_REQUIRE(!image.isNull(), "rendering failed");
    }

    // Cancel: Escape drops a draft; nothing is added.
    {
        kindCombo->setCurrentIndex(0);
        const std::size_t steps = doc.undoStack().index();
        click(canvas, {20, 20});
        click(canvas, {40, 20});
        key(canvas, Qt::Key_Escape);
        DIM_REQUIRE(dt.pickedCount() == 0 && doc.undoStack().index() == steps, "Escape did not cancel the draft");
    }

    // Shift-click selects a dimension; its handles edit it, one undo step per drag.
    for (std::size_t n = 0; n < ids.size(); ++n) {
        const document::ObjectId id = ids[n];
        doc.selection().clear();
        const auto g = document::dimensionGeometry(doc, *dimOf(doc, id));
        click(canvas, g.textCentre, Qt::ShiftModifier);
        DIM_REQUIRE(dt.editTarget() == id, "Shift-click did not select the dimension");
        DIM_REQUIRE(doc.selection().single() == id, "Shift-click did not select it in the document");
        const auto handles = dt.handles();
        DIM_REQUIRE(handles.size() >= 4, "too few handles");
        (void)canvas->grab();  // paints the handles
        bool whiteSeen = false, blackSeen = false;
        for (const auto& h : handles) {
            whiteSeen = whiteSeen || h.white;
            blackSeen = blackSeen || !h.white;
        }
        DIM_REQUIRE(whiteSeen && blackSeen, "handles are not both white and black");
        // Drag the text position handle: the annotation moves, the measurement does not.
        const document::DimensionObject before = *dimOf(doc, id);
        const std::size_t steps = doc.undoStack().index();
        const auto textHandle = *std::find_if(handles.begin(), handles.end(), [](const document::DimensionHandle& h) {
            return h.kind == document::DimensionHandleKind::Text;
        });
        drag(canvas, textHandle.pos, textHandle.pos + geometry::Vec2{12.0, 12.0});
        DIM_REQUIRE(doc.undoStack().index() == steps + 1, "a text handle drag was not one undo step");
        DIM_REQUIRE(!(*dimOf(doc, id) == before), "the text handle drag changed nothing");
        window.undo();
        DIM_REQUIRE(*dimOf(doc, id) == before, "undo did not restore the dimension");
        // Drag a size handle (the first point) a little.
        const auto pointHandle = dt.handles().front();
        const auto textBefore = document::dimensionText(doc, before, doc.job().displayUnits);
        drag(canvas, pointHandle.pos, pointHandle.pos + geometry::Vec2{6.0, -7.0});
        DIM_REQUIRE(doc.undoStack().index() == steps + 1, "a point handle drag was not one undo step");
        window.undo();
        DIM_REQUIRE(document::dimensionText(doc, *dimOf(doc, id), doc.job().displayUnits) == textBefore,
                    "undo did not restore the text");
        // Black handle: the text height.
        const auto heightHandle = *std::find_if(dt.handles().begin(), dt.handles().end(), [](const auto& h) {
            return h.kind == document::DimensionHandleKind::TextHeight;
        });
        drag(canvas, heightHandle.pos, heightHandle.pos + geometry::Vec2{0.0, 4.0});
        DIM_REQUIRE(dimOf(doc, id)->textHeightMm > before.textHeightMm, "the text height handle did not enlarge the text");
        DIM_REQUIRE(doc.undoStack().index() == steps + 1, "a text height drag was not one undo step");
        window.undo();
        // Escape during a drag reverts it.
        {
            const auto h0 = dt.handles().front();
            const QPointF a = screenOf(canvas, h0.pos);
            sendMouse(canvas, QEvent::MouseMove, a, Qt::NoModifier);
            sendMouse(canvas, QEvent::MouseButtonPress, a, Qt::NoModifier);
            sendMouse(canvas, QEvent::MouseMove, screenOf(canvas, h0.pos + geometry::Vec2{9.0, 9.0}), Qt::NoModifier);
            key(canvas, Qt::Key_Escape);
            sendMouse(canvas, QEvent::MouseButtonRelease, screenOf(canvas, h0.pos + geometry::Vec2{9.0, 9.0}),
                      Qt::NoModifier);
            DIM_REQUIRE(*dimOf(doc, id) == before && doc.undoStack().index() == steps, "Escape did not cancel the drag");
        }
    }

    // Edit dialog: decimals, text height, offset, custom text, reflex flip; hostile offsets refused.
    {
        const document::ObjectId id = ids[4];  // the reflex angular dimension
        doc.selection().assign({id});
        window.dimensions()->refresh();
        DIM_REQUIRE(window.dimensions()->editAction()->isEnabled() && window.dimensions()->deleteAction()->isEnabled(),
                    "Edit > Dimensions entries are not enabled for a selected dimension");
        const auto current = dt.optionsOf(id);
        DIM_REQUIRE(current && current->reflex.value_or(false), "the reflex dimension does not report reflex");
        DimensionDialog dialog(doc, *current, true, &window);
        auto* offset = dialog.findChild<QLineEdit*>(QStringLiteral("dimOffset"));
        auto* reflex = dialog.findChild<QCheckBox*>(QStringLiteral("dimReflex"));
        auto* decimals = dialog.findChild<QSpinBox*>(QStringLiteral("dimAngleDecimals"));
        auto* kind = dialog.findChild<QComboBox*>(QStringLiteral("dimKind"));
        DIM_REQUIRE(offset && reflex && decimals && kind && !kind->isEnabled(), "edit dialog widgets missing");
        offset->setText(QStringLiteral("1e12"));
        DIM_REQUIRE(!dialog.problem().empty(), "a hostile offset was accepted");
        offset->setText(QStringLiteral("inf"));
        DIM_REQUIRE(!dialog.problem().empty(), "an infinite offset was accepted");
        offset->setText(QStringLiteral("25"));
        reflex->setChecked(false);
        decimals->setValue(0);
        DIM_REQUIRE(dialog.problem().empty(), "valid edit entries were refused");
        const std::size_t steps = doc.undoStack().index();
        DIM_REQUIRE(window.dimensions()->applyEdit(id, dialog.options()), "the edit was refused");
        DIM_REQUIRE(doc.undoStack().index() == steps + 1, "the edit was not one undo step");
        DIM_REQUIRE(document::dimensionText(doc, *dimOf(doc, id), doc.job().displayUnits) == "90\xC2\xB0" &&
                        dimOf(doc, id)->offsetMm == 25.0,
                    "the edit did not apply");
        window.undo();
        DIM_REQUIRE(document::dimensionText(doc, *dimOf(doc, id), doc.job().displayUnits) == "270.00\xC2\xB0",
                    "undo of the edit failed");
    }

    // A document font can be added and chosen for the text.
    {
        const document::ObjectId id = ids[0];
        doc.selection().assign({id});
        QTemporaryFile bad;
        DIM_REQUIRE(bad.open(), "no temp file");
        bad.write("this is not a font");
        bad.flush();
        QTemporaryFile good;
        DIM_REQUIRE(good.open(), "no temp file");
        const auto starter = viewmodel::starterFontBytes();
        good.write(reinterpret_cast<const char*>(starter.data()), static_cast<qint64>(starter.size()));
        good.flush();
        DimensionDialog dialog(doc, *dt.optionsOf(id), true, &window);
        DIM_REQUIRE(!dialog.addFontFile(bad.fileName()).empty(), "a non-font file was accepted");
        DIM_REQUIRE(doc.fonts().empty(), "a refused font was embedded");
        DIM_REQUIRE(dialog.addFontFile(good.fileName()).empty(), "a font file was refused");
        DIM_REQUIRE(doc.fonts().size() == 1, "the font was not embedded");
        DIM_REQUIRE(dialog.addFontFile(QStringLiteral("/nonexistent/font.ttf")) != "", "a missing file was accepted");
        DIM_REQUIRE(dialog.problem().empty(), "the dialog is not valid after adding a font");
        DIM_REQUIRE(window.dimensions()->applyEdit(id, dialog.options()), "choosing the font was refused");
        DIM_REQUIRE(dimOf(doc, id)->fontKey == doc.fonts().begin()->first, "the dimension does not use the font");
        DIM_REQUIRE(!viewmodel::dimensionTextContours(doc, *dimOf(doc, id)).empty(), "the font set no text");
    }

    // Delete: the Delete key and the menu entry, one undo step each, restored by undo.
    {
        const document::ObjectId id = ids[1];
        doc.selection().assign({id});
        std::size_t steps = doc.undoStack().index();
        key(canvas, Qt::Key_Delete);
        DIM_REQUIRE(doc.findObject(id) == nullptr && doc.undoStack().index() == steps + 1,
                    "Delete did not remove the dimension in one step");
        window.undo();
        DIM_REQUIRE(doc.findObject(id) != nullptr, "undo did not restore the dimension");
        doc.selection().assign({id});
        steps = doc.undoStack().index();
        window.dimensions()->refresh();
        window.dimensions()->deleteAction()->trigger();
        DIM_REQUIRE(doc.findObject(id) == nullptr && doc.undoStack().index() == steps + 1,
                    "Edit > Delete Selected Dimension failed");
        window.undo();
        // Nothing selected: refused with no change.
        doc.selection().clear();
        steps = doc.undoStack().index();
        DIM_REQUIRE(!window.dimensions()->deleteSelected(), "deleting with no dimension selected was accepted");
        DIM_REQUIRE(doc.undoStack().index() == steps, "a refused delete changed the document");
    }
    window.selectTool(viewmodel::ToolKind::Select);
    return {};
}

}  // namespace carvetto::app
