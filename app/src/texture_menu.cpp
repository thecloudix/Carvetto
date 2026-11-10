#include "texture_menu.hpp"

#include <QAction>
#include <QApplication>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QPainter>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <memory>
#include <variant>

#include "carvetto/document/commands.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/viewmodel/vector_ops.hpp"
#include "main_window.hpp"

namespace carvetto::app {

using viewmodel::TexturePreview;

TexturePreviewWidget::TexturePreviewWidget(QWidget* parent) : QWidget(parent) {
    setMinimumSize(320, 240);
}

void TexturePreviewWidget::setPreview(const TexturePreview& preview) {
    preview_ = preview;
    update();
}

void TexturePreviewWidget::paintEvent(QPaintEvent*) {
    QPainter p(this);
    p.fillRect(rect(), QColor(250, 250, 250));
    const geometry::Box2& box = preview_.region;
    if (box.isEmpty() || !(box.width() > 0.0) || !(box.height() > 0.0)) return;
    const double margin = 8.0;
    const double scale = std::min((width() - 2 * margin) / box.width(), (height() - 2 * margin) / box.height());
    if (!(scale > 0.0) || !std::isfinite(scale)) return;
    const auto map = [&](geometry::Vec2 v) {
        return QPointF(margin + (v.x - box.min.x) * scale, height() - margin - (v.y - box.min.y) * scale);
    };
    p.setPen(QPen(QColor(190, 190, 190), 1.0));
    p.drawRect(QRectF(map({box.min.x, box.max.y}), map({box.max.x, box.min.y})));
    p.setPen(QPen(QColor(20, 90, 160), 1.0));
    for (const geometry::Contour& c : preview_.contours) {
        for (const geometry::Span& s : c.spans) {
            if (const auto* l = std::get_if<geometry::LineSpan>(&s)) p.drawLine(map(l->start), map(l->end));
        }
    }
}

namespace {

QString qs(const std::string& s) { return QString::fromStdString(s); }

}  // namespace

TextureDialog::TextureDialog(viewmodel::TextureForm& form, const document::Document& doc, QWidget* parent)
    : QDialog(parent), form_(form), doc_(doc) {
    setWindowTitle(QStringLiteral("Create Vector Texture"));
    auto* grid = new QFormLayout;
    const QString suffix = QString::fromUtf8(document::unitSuffix(form.units()).data(),
                                             static_cast<int>(document::unitSuffix(form.units()).size()));
    for (const viewmodel::NumericField& f : form.fields()) {
        auto* edit = new QLineEdit(qs(f.text), this);
        edit->setObjectName(qs("texture_" + f.id));
        QString label = qs(f.label);
        if (f.kind == viewmodel::FieldKind::Length) label += QStringLiteral(" (") + suffix + QStringLiteral(")");
        if (f.kind == viewmodel::FieldKind::Angle) label += QStringLiteral(" (deg)");
        if (f.kind == viewmodel::FieldKind::Percent) label += QStringLiteral(" (%)");
        grid->addRow(label, edit);
        edits_[f.id] = edit;
    }
    layer_ = new QComboBox(this);
    int current = 0;
    const document::LayerId target = form.effectiveLayer(doc);
    for (const document::Layer& layer : doc.layers()) {
        layer_->addItem(qs(layer.name), static_cast<qulonglong>(layer.id.value));
        if (layer.id == target) current = layer_->count() - 1;
    }
    layer_->setCurrentIndex(current);
    grid->addRow(QStringLiteral("Target layer"), layer_);
    canvas_ = new TexturePreviewWidget(this);
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });
    auto* layout = new QVBoxLayout(this);
    layout->addLayout(grid);
    layout->addWidget(canvas_);
    layout->addWidget(status_);
    layout->addWidget(buttons);
    for (auto& [id, edit] : edits_) {
        const std::string key = id;
        connect(edit, &QLineEdit::textChanged, this, [this, key]() { entryChanged(key); });
    }
    connect(layer_, &QComboBox::currentIndexChanged, this, [this]() {
        form_.setTargetLayer(document::LayerId{static_cast<std::uint64_t>(layer_->currentData().toULongLong())});
    });
    refreshPreview();
}

void TextureDialog::entryChanged(const std::string& id) {
    const viewmodel::FieldResult r = form_.setField(id, edits_.at(id)->text().toStdString());
    QLineEdit* edit = edits_.at(id);
    if (r.ok) {
        errors_.erase(id);
        edit->setStyleSheet(QString());
        edit->setToolTip(QString());
    } else {
        errors_[id] = r.message;
        edit->setStyleSheet(QStringLiteral("background: #ffd0d0;"));
        edit->setToolTip(qs(r.message));
    }
    refreshPreview();
}

viewmodel::FieldResult TextureDialog::setEntry(const std::string& id, const std::string& text) {
    edits_.at(id)->setText(qs(text));  // fires entryChanged
    viewmodel::FieldResult r;
    if (auto it = errors_.find(id); it != errors_.end()) {
        r.ok = false;
        r.message = it->second;
    }
    return r;
}

bool TextureDialog::setTargetLayer(const std::string& name) {
    const int i = layer_->findText(qs(name));
    if (i < 0) return false;
    layer_->setCurrentIndex(i);
    return true;
}

std::string TextureDialog::statusText() const { return status_->text().toStdString(); }

void TextureDialog::refreshPreview() {
    if (!errors_.empty()) {
        preview_ = TexturePreview{};
        preview_.message = "Fix the highlighted entry: " + errors_.begin()->second;
    } else {
        preview_ = form_.preview(doc_);
    }
    canvas_->setPreview(preview_);
    if (preview_.ok) {
        status_->setText(qs(std::to_string(preview_.contours.size()) + " lines, " +
                            std::to_string(preview_.vertexCount) + " points"));
    } else {
        status_->setText(qs(preview_.message));
    }
}

void TextureDialog::accept() {
    if (!errors_.empty()) {
        QMessageBox::warning(this, windowTitle(), qs(errors_.begin()->second));
        return;
    }
    if (!preview_.ok && !preview_.skipped) {
        QMessageBox::warning(this, windowTitle(), qs(preview_.message));
        return;
    }
    QDialog::accept();
}

TextureMenu::TextureMenu(MainWindow& window, QMenu* editMenu)
    : window_(window), form_(window.currentDocument().job().displayUnits) {
    action_ = editMenu->addAction(QStringLiteral("Create Vector &Texture..."));
    action_->setEnabled(false);
    QObject::connect(action_, &QAction::triggered, editMenu, [this]() { openDialog(); });
    QObject::connect(editMenu, &QMenu::aboutToShow, editMenu, [this]() {
        action_->setEnabled(viewmodel::selectedClosedRegion(window_.currentDocument()).ok);
    });
}

void TextureMenu::openDialog() {
    form_.setUnits(window_.currentDocument().job().displayUnits);
    TextureDialog dialog(form_, window_.currentDocument(), &window_);
    if (dialog.exec() == QDialog::Accepted) run();
}

viewmodel::TextureResult TextureMenu::run(bool showErrors) {
    viewmodel::TextureResult r = form_.apply(window_.currentDocument());
    if (r.ok) {
        window_.documentChanged();
    } else if (showErrors) {
        QMessageBox::warning(&window_, QStringLiteral("Vector Texture"), qs(r.message));
    }
    return r;
}

std::string textureSelfTest(MainWindow& window) {
    TextureMenu* menu = window.textureMenu();
    if (menu == nullptr || menu->action() == nullptr) return "the Edit menu has no Create Vector Texture action";
    document::Document& doc = window.currentDocument();
    const std::size_t objectsBefore = doc.objectCount();

    // A 100 x 80 plate with a 40 x 40 hole, on a second layer for the result.
    geometry::Contour hole = geometry::Contour::rectangle({{30, 20}, {70, 60}});
    hole.reverse();
    auto add = std::make_unique<document::AddObjectCommand>(document::Object::vector(
        std::vector<geometry::Contour>{geometry::Contour::rectangle({{0, 0}, {100, 80}}), hole}, doc.activeLayer(),
        doc.activeSheet()));
    const document::AddObjectCommand* raw = add.get();
    if (!doc.execute(std::move(add))) return "adding the texture test plate failed";
    const document::ObjectId plate = raw->id();
    doc.selection().assign({plate});
    window.documentChanged();

    std::string problem;
    // Driven from inside the modal event loop of the real dialog, opened by its menu action.
    QTimer::singleShot(0, [&]() {
        auto* dialog = dynamic_cast<TextureDialog*>(QApplication::activeModalWidget());
        if (dialog == nullptr) {
            problem = "the Create Vector Texture dialog did not open";
            if (QWidget* w = QApplication::activeModalWidget()) static_cast<QDialog*>(w)->reject();
            return;
        }
        const auto finish = [&](const std::string& why) {
            problem = why;
            dialog->reject();
        };
        for (const auto& [id, text] : std::vector<std::pair<std::string, std::string>>{
                 {"spacing", "4 mm"}, {"wavelength", "8 mm"}, {"amplitude", "1 mm"}, {"noise", "30"}, {"seed", "7"}}) {
            if (!dialog->setEntry(id, text).ok) return finish("the dialog refused a valid entry for " + id);
        }
        if (!dialog->preview().ok || dialog->preview().contours.empty()) {
            return finish("the dialog has no live preview: " + dialog->preview().message);
        }
        // Every preview line lies in the plate and outside its hole.
        for (const geometry::Contour& c : dialog->preview().contours) {
            for (const geometry::Span& sp : c.spans) {
                const auto& l = std::get<geometry::LineSpan>(sp);
                const geometry::Vec2 m{(l.start.x + l.end.x) / 2, (l.start.y + l.end.y) / 2};
                if (m.x < -0.01 || m.x > 100.01 || m.y < -0.01 || m.y > 80.01 ||
                    (m.x > 30.01 && m.x < 69.99 && m.y > 20.01 && m.y < 59.99)) {
                    return finish("a preview line is outside the selected region or inside its hole");
                }
            }
        }
        // A bad entry is flagged, keeps the previous value and blanks the preview; OK refuses.
        if (dialog->setEntry("spacing", "-3").ok || dialog->setEntry("angle", "120").ok ||
            dialog->setEntry("seed", "x").ok) {
            return finish("the dialog accepted a bad entry");
        }
        if (dialog->preview().ok || menu->form().params().maxSpacing != 4.0) {
            return finish("a bad entry changed the preview or the stored value");
        }
        for (const auto& [id, text] : std::vector<std::pair<std::string, std::string>>{
                 {"spacing", "4 mm"}, {"angle", "15"}, {"seed", "7"}}) {
            if (!dialog->setEntry(id, text).ok) return finish("the dialog refused a valid entry for " + id);
        }
        if (!dialog->preview().ok) return finish("the preview did not come back after fixing the entries");
        dialog->accept();
    });
    menu->action()->setEnabled(true);
    menu->action()->trigger();
    if (!problem.empty()) {
        doc.undo();
        return problem;
    }
    // One new open-line object, selected; one undo removes it (checked below).
    if (doc.objectCount() != objectsBefore + 2 ||
        doc.selection().size() != 1 || doc.selection().ids().front() == plate) {
        return "accepting the texture dialog did not add one selected object ";
    }
    const document::Object* made = doc.findObject(doc.selection().ids().front());
    if (made == nullptr || !made->isVector() || made->asVector()->contours.empty()) {
        return "the texture object is missing or empty";
    }
    for (const geometry::Contour& c : made->asVector()->contours) {
        if (c.closed) return "a texture line is closed";
    }
    if (!window.undo() || doc.objectCount() != objectsBefore + 1) return "undoing the texture failed";
    if (!window.undo() || doc.objectCount() != objectsBefore) return "undoing the texture plate failed";
    doc.selection().clear();
    window.documentChanged();
    return {};
}

}  // namespace carvetto::app
