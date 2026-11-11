#include "keepout_panel.hpp"

#include <QAction>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QMenu>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/keepout.hpp"

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

}  // namespace

KeepOutPanel::KeepOutPanel(DocumentGetter getter, QWidget* parent) : PanelBase(std::move(getter), parent) {
    create_ = new QAction(QStringLiteral("&Create Zones from Selection"), this);
    clear_ = new QAction(QStringLiteral("C&lear Zones"), this);
    show_ = new QAction(QStringLiteral("&Show Zones"), this);
    show_->setCheckable(true);
    show_->setChecked(true);

    clearance_ = new QDoubleSpinBox(this);
    clearance_->setDecimals(3);
    clearance_->setRange(0.0, 100000.0);
    clearance_->setKeyboardTracking(false);
    status_ = new QLabel(this);
    status_->setWordWrap(true);
    auto* createButton = new QPushButton(create_->text().remove(QLatin1Char('&')), this);
    auto* clearButton = new QPushButton(clear_->text().remove(QLatin1Char('&')), this);
    auto* showBox = new QCheckBox(show_->text().remove(QLatin1Char('&')), this);
    showBox->setChecked(true);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(new QLabel(QStringLiteral("Clearance round the zones:"), this));
    layout->addWidget(clearance_);
    layout->addWidget(showBox);
    layout->addWidget(createButton);
    layout->addWidget(clearButton);
    layout->addWidget(status_);
    layout->addStretch(1);

    connect(create_, &QAction::triggered, this, [this]() { createFromSelection(); });
    connect(clear_, &QAction::triggered, this, [this]() { clearZones(); });
    connect(show_, &QAction::toggled, this, [this](bool on) { setShown(on); });
    connect(createButton, &QPushButton::clicked, this, [this]() { createFromSelection(); });
    connect(clearButton, &QPushButton::clicked, this, [this]() { clearZones(); });
    connect(showBox, &QCheckBox::toggled, this, [this](bool on) { show_->setChecked(on); });
    connect(show_, &QAction::toggled, showBox, [showBox](bool on) { showBox->setChecked(on); });
    connect(clearance_, &QDoubleSpinBox::valueChanged, this, [this](double displayValue) {
        if (refreshing_ || doc() == nullptr) {
            return;
        }
        setClearance(document::fromDisplay(displayValue, doc()->job().displayUnits));
    });
}

void KeepOutPanel::buildMenu(QMenu* edit) {
    QMenu* menu = edit->addMenu(QStringLiteral("&Keep-out Zones"));
    menu->addAction(create_);
    menu->addAction(clear_);
    menu->addSeparator();
    menu->addAction(show_);
    connect(menu, &QMenu::aboutToShow, this, [this]() { refresh(); });
}

void KeepOutPanel::refresh() {
    const document::Document* d = doc();
    if (d == nullptr) {
        return;
    }
    refreshing_ = true;
    create_->setEnabled(viewmodel::canCreateKeepOutFromSelection(*d));
    clear_->setEnabled(!d->keepOutZones().empty());
    show_->setChecked(d->keepOutVisible());
    const document::Units units = d->job().displayUnits;
    clearance_->setSuffix(QStringLiteral(" ") + QString::fromUtf8(document::unitSuffix(units).data(),
                                                                   static_cast<int>(document::unitSuffix(units).size())));
    clearance_->setValue(document::toDisplay(d->keepOutClearance(), units));
    refreshing_ = false;
    QString text = qs(viewmodel::describeKeepOut(*d));
    if (routingText) {
        const std::string extra = routingText();
        if (!extra.empty()) {
            text += QStringLiteral("\n") + qs(extra);
        }
    }
    status_->setText(text);
}

bool KeepOutPanel::createFromSelection() {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const viewmodel::KeepOutResult r = viewmodel::createKeepOutFromSelection(*d);
    if (r.ok) {
        if (onEdited) {
            onEdited();
        }
    }
    if (onMessage) {
        onMessage(r.message);
    }
    return r.ok;
}

bool KeepOutPanel::clearZones() {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const viewmodel::KeepOutResult r = viewmodel::clearKeepOutZones(*d);
    if (r.ok && onEdited) {
        onEdited();
    }
    if (onMessage) {
        onMessage(r.message);
    }
    return r.ok;
}

bool KeepOutPanel::setClearance(double mm) {
    document::Document* d = doc();
    if (d == nullptr) {
        return false;
    }
    const viewmodel::KeepOutResult r = viewmodel::setKeepOutClearanceMm(*d, mm);
    if (r.ok && onEdited) {
        onEdited();
    }
    if (onMessage && !r.ok) {
        onMessage(r.message);
    }
    return r.ok;
}

void KeepOutPanel::setShown(bool shown) {
    document::Document* d = doc();
    if (d == nullptr || refreshing_) {
        return;
    }
    if (viewmodel::setKeepOutVisible(*d, shown) && onEdited) {
        onEdited();
    }
}

}  // namespace carvetto::app
