#include "tiling_manager_panel.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QPainter>
#include <QPushButton>
#include <QString>
#include <QTreeWidget>
#include <QVBoxLayout>
#include <algorithm>

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

QDoubleSpinBox* mmField(const char* name, QWidget* parent) {
    auto* f = new QDoubleSpinBox(parent);
    f->setObjectName(QString::fromLatin1(name));
    f->setRange(0.0, 100000.0);
    f->setDecimals(2);
    f->setSingleStep(1.0);
    f->setSuffix(QStringLiteral(" mm"));
    return f;
}

}  // namespace

/// Draws the job outline and the tile rectangles of a `TilePreview`, Y up.
class TilePreviewWidget : public QWidget {
   public:
    explicit TilePreviewWidget(QWidget* parent) : QWidget(parent) {
        setObjectName(QStringLiteral("tilingDrawing"));
        setMinimumHeight(150);
    }
    void setPreview(const viewmodel::TilePreview* p) {
        preview_ = p;
        update();
    }

   protected:
    void paintEvent(QPaintEvent*) override {
        QPainter g(this);
        g.setRenderHint(QPainter::Antialiasing, true);
        g.fillRect(rect(), QColor(250, 250, 250));
        if (preview_ == nullptr || preview_->extent.isEmpty() || !preview_->ok()) {
            return;
        }
        const geometry::Box2& e = preview_->extent;
        const double w = std::max(e.width(), 1e-6);
        const double h = std::max(e.height(), 1e-6);
        const double margin = 8.0;
        const double scale = std::min((width() - 2 * margin) / w, (height() - 2 * margin) / h);
        const double ox = (width() - w * scale) / 2.0;
        const double oy = (height() - h * scale) / 2.0;
        const auto px = [&](double x) { return ox + (x - e.min.x) * scale; };
        const auto py = [&](double y) { return height() - (oy + (y - e.min.y) * scale); };
        g.setPen(QPen(QColor(120, 120, 120), 1));
        for (const auto& s : preview_->outline) {
            g.drawLine(QPointF(px(s.first.x), py(s.first.y)), QPointF(px(s.second.x), py(s.second.y)));
        }
        for (const viewmodel::TilePreviewEntry& t : preview_->tiles) {
            const QRectF r(QPointF(px(t.rect.min.x), py(t.rect.max.y)), QPointF(px(t.rect.max.x), py(t.rect.min.y)));
            if (t.number == 0) {
                g.setPen(QPen(QColor(190, 190, 190), 1, Qt::DashLine));
            } else if (t.oversize) {
                g.setPen(QPen(QColor(200, 30, 30), 2));
                g.fillRect(r, QColor(200, 30, 30, 40));
            } else {
                g.setPen(QPen(QColor(30, 140, 60), 2));
            }
            g.drawRect(r);
            if (t.number != 0) {
                g.drawText(r, Qt::AlignCenter, QString::number(t.number));
            }
        }
    }

   private:
    const viewmodel::TilePreview* preview_{nullptr};
};

TilingManagerPanel::TilingManagerPanel(QWidget* parent) : QWidget(parent) {
    setObjectName(QStringLiteral("tilingManager"));
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    tile_ = new QCheckBox(QStringLiteral("Tile Toolpaths (split the job into tiles that fit the machine)"), this);
    tile_->setObjectName(QStringLiteral("tilingCheck"));
    layout->addWidget(tile_);

    settings_ = new QWidget(this);
    settings_->setObjectName(QStringLiteral("tilingSettings"));
    auto* body = new QVBoxLayout(settings_);
    body->setContentsMargins(12, 0, 0, 0);
    auto* form = new QFormLayout();
    body->addLayout(form);

    strategy_ = new QComboBox(settings_);
    strategy_->setObjectName(QStringLiteral("tilingStrategy"));
    strategy_->addItem(QStringLiteral("Individual tiles (grid)"));
    strategy_->addItem(QStringLiteral("Feed-through X (strips along X)"));
    strategy_->addItem(QStringLiteral("Feed-through Y (strips along Y)"));
    form->addRow(QStringLiteral("Strategy:"), strategy_);

    width_ = mmField("tilingWidth", settings_);
    height_ = mmField("tilingHeight", settings_);
    overlap_ = mmField("tilingOverlap", settings_);
    auto* sizeRow = new QHBoxLayout();
    sizeRow->addWidget(width_);
    sizeRow->addWidget(new QLabel(QStringLiteral("x"), settings_));
    sizeRow->addWidget(height_);
    useWorkArea_ = new QPushButton(QStringLiteral("Machine work area"), settings_);
    useWorkArea_->setObjectName(QStringLiteral("tilingUseWorkArea"));
    sizeRow->addWidget(useWorkArea_);
    form->addRow(QStringLiteral("Tile size (W x H):"), sizeRow);
    form->addRow(QStringLiteral("Overlap:"), overlap_);

    smallestFirst_ = new QCheckBox(QStringLiteral("Smallest tiles first (default: row by row)"), settings_);
    smallestFirst_->setObjectName(QStringLiteral("tilingSmallestFirst"));
    originalPosition_ = new QCheckBox(QStringLiteral("Keep the original position (default: each tile starts at X0 Y0)"),
                                      settings_);
    originalPosition_->setObjectName(QStringLiteral("tilingOriginalPosition"));
    body->addWidget(smallestFirst_);
    body->addWidget(originalPosition_);

    status_ = new QLabel(settings_);
    status_->setObjectName(QStringLiteral("tilingStatus"));
    status_->setWordWrap(true);
    body->addWidget(status_);

    auto* split = new QHBoxLayout();
    list_ = new QTreeWidget(settings_);
    list_->setObjectName(QStringLiteral("tilingList"));
    list_->setHeaderLabels({QStringLiteral("Tile"), QStringLiteral("X"), QStringLiteral("Y"), QStringLiteral("Size"),
                            QStringLiteral("State")});
    list_->setRootIsDecorated(false);
    list_->setMinimumHeight(110);
    drawing_ = new TilePreviewWidget(settings_);
    split->addWidget(list_, 3);
    split->addWidget(drawing_, 2);
    body->addLayout(split);
    layout->addWidget(settings_);
    settings_->setVisible(false);
    drawing_->setPreview(&preview_);

    connect(tile_, &QCheckBox::toggled, this, [this](bool) {
        settings_->setVisible(tile_->isChecked());
        changed(false);
    });
    connect(strategy_, &QComboBox::currentIndexChanged, this, [this](int) {
        updateEnabledFields();
        changed(false);
    });
    connect(smallestFirst_, &QCheckBox::toggled, this, [this](bool) { changed(false); });
    connect(originalPosition_, &QCheckBox::toggled, this, [this](bool) { changed(false); });
    const auto sizeTyped = [this](double) {
        if (!updating_) {
            sizeEdited_ = true;
        }
        changed(true);
    };
    connect(width_, &QDoubleSpinBox::valueChanged, this, sizeTyped);
    connect(height_, &QDoubleSpinBox::valueChanged, this, sizeTyped);
    connect(overlap_, &QDoubleSpinBox::valueChanged, this, [this](double) { changed(true); });
    connect(useWorkArea_, &QPushButton::clicked, this, [this]() {
        sizeEdited_ = false;
        updating_ = true;
        width_->setValue(areaX_);
        height_->setValue(areaY_);
        updating_ = false;
        changed(false);
    });
    updateEnabledFields();
}

void TilingManagerPanel::changed(bool typed) {
    if (updating_) {
        return;
    }
    if (typed) {
        if (onTyped) {
            onTyped();
        }
    } else if (onChanged) {
        onChanged();
    }
}

void TilingManagerPanel::updateEnabledFields() {
    const auto s = static_cast<toolpath::TileStrategy>(std::max(0, strategy_->currentIndex()));
    width_->setEnabled(s != toolpath::TileStrategy::FeedThroughY);
    height_->setEnabled(s != toolpath::TileStrategy::FeedThroughX);
}

viewmodel::TilingForm TilingManagerPanel::form() const {
    viewmodel::TilingForm f;
    f.enabled = tile_->isChecked();
    f.strategy = static_cast<toolpath::TileStrategy>(std::max(0, strategy_->currentIndex()));
    f.width = width_->value();
    f.height = height_->value();
    f.overlap = overlap_->value();
    f.smallestFirst = smallestFirst_->isChecked();
    f.originalPosition = originalPosition_->isChecked();
    return f;
}

void TilingManagerPanel::setForm(const viewmodel::TilingForm& f) {
    updating_ = true;
    tile_->setChecked(f.enabled);
    settings_->setVisible(f.enabled);
    strategy_->setCurrentIndex(static_cast<int>(f.strategy));
    width_->setValue(f.width);
    height_->setValue(f.height);
    overlap_->setValue(f.overlap);
    smallestFirst_->setChecked(f.smallestFirst);
    originalPosition_->setChecked(f.originalPosition);
    sizeEdited_ = true;
    updating_ = false;
    updateEnabledFields();
    changed(false);
}

void TilingManagerPanel::setTilingOn(bool on) {
    tile_->setChecked(on);
}

bool TilingManagerPanel::tilingOn() const {
    return tile_->isChecked();
}

void TilingManagerPanel::setMachineWorkArea(double x, double y) {
    areaX_ = std::max(0.0, x);
    areaY_ = std::max(0.0, y);
    useWorkArea_->setEnabled(areaX_ > 0.0 || areaY_ > 0.0);
    if (!sizeEdited_) {
        updating_ = true;
        width_->setValue(areaX_);
        height_->setValue(areaY_);
        updating_ = false;
    }
}

void TilingManagerPanel::showPreview(const viewmodel::TilePreview& preview) {
    preview_ = preview;
    status_->setText(qs(viewmodel::describeTilePreview(preview_)));
    list_->clear();
    if (preview_.ok()) {
        for (const viewmodel::TilePreviewEntry& t : preview_.tiles) {
            auto* item = new QTreeWidgetItem(list_);
            item->setText(0, t.number != 0 ? QString::number(t.number) : QStringLiteral("-"));
            item->setText(1, QString::number(t.rect.min.x, 'f', 2));
            item->setText(2, QString::number(t.rect.min.y, 'f', 2));
            item->setText(3, QStringLiteral("%1 x %2")
                                 .arg(t.rect.width(), 0, 'f', 2)
                                 .arg(t.rect.height(), 0, 'f', 2));
            item->setText(4, t.number == 0 ? QStringLiteral("empty, not saved")
                                           : (t.oversize ? QStringLiteral("too large") : QStringLiteral("saved")));
        }
    }
    drawing_->setPreview(&preview_);
}

int TilingManagerPanel::tileRows() const {
    return list_->topLevelItemCount();
}

std::string TilingManagerPanel::statusShown() const {
    return status_->text().toStdString();
}

bool TilingManagerPanel::settingsShown() const {
    return !settings_->isHidden();
}

}  // namespace carvetto::app
