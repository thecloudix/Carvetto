#include "clipart_panel.hpp"

#include <QCoreApplication>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDir>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QIcon>
#include <QImage>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMimeData>
#include <QPixmap>
#include <QPushButton>
#include <QSettings>
#include <QString>
#include <QStringList>
#include <QTimer>
#include <QUrl>
#include <QVBoxLayout>
#include <algorithm>

#include "bitmap_decoder.hpp"
#include "carvetto/viewmodel/starter_font.hpp"

namespace carvetto::app {

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string fromQ(const QString& s) {
    return s.toStdString();
}

constexpr int kThumbEdge = 96;
constexpr int kPathRole = Qt::UserRole + 1;

/// Item list that drags the clipart file as a local URL (the canvas and the file manager both
/// understand it).
class ClipartList : public QListWidget {
   public:
    explicit ClipartList(QWidget* parent) : QListWidget(parent) {}

   protected:
    QMimeData* mimeData(const QList<QListWidgetItem*>& items) const override {
        auto* mime = new QMimeData;
        QList<QUrl> urls;
        for (const QListWidgetItem* item : items) {
            urls.push_back(QUrl::fromLocalFile(item->data(kPathRole).toString()));
        }
        mime->setUrls(urls);
        return mime;
    }
};

}  // namespace

/// Keeps the library state in QSettings when persisting, in memory otherwise.
class ClipartPanel::Store : public viewmodel::ClipartStore {
   public:
    viewmodel::ClipartState load() override {
        if (!persist) {
            return memory;
        }
        QSettings settings;
        viewmodel::ClipartState s;
        for (const QString& f : settings.value(QStringLiteral("clipart/folders")).toStringList()) {
            s.folders.push_back(fromQ(f));
        }
        for (const QString& f : settings.value(QStringLiteral("clipart/favourites")).toStringList()) {
            s.favourites.push_back(fromQ(f));
        }
        return s;
    }
    bool save(const viewmodel::ClipartState& state) override {
        if (!persist) {
            memory = state;
            return true;
        }
        QSettings settings;
        QStringList folders;
        QStringList favourites;
        for (const std::string& f : state.folders) {
            folders << qs(f);
        }
        for (const std::string& f : state.favourites) {
            favourites << qs(f);
        }
        settings.setValue(QStringLiteral("clipart/folders"), folders);
        settings.setValue(QStringLiteral("clipart/favourites"), favourites);
        settings.sync();
        return settings.status() == QSettings::NoError;
    }
    bool persist{false};
    viewmodel::ClipartState memory;
};

ClipartPanel::ClipartPanel(DocumentGetter getter, QWidget* parent)
    : PanelBase(std::move(getter), parent), store_(std::make_unique<Store>()) {
    library_ = std::make_unique<viewmodel::ClipartLibrary>(*store_);
    viewmodel::ClipartThumbnailService::Config cfg;
    cfg.edge = kThumbEdge;
    service_ = std::make_unique<viewmodel::ClipartThumbnailService>(cfg, qtBitmapDecoder());

    search_ = new QLineEdit(this);
    search_->setPlaceholderText(QCoreApplication::translate("ClipartPanel", "Search clipart by name"));
    search_->setClearButtonEnabled(true);
    favouritesOnly_ = new QCheckBox(QCoreApplication::translate("ClipartPanel", "Favourites only"), this);
    folders_ = new QComboBox(this);
    auto* add = new QPushButton(QCoreApplication::translate("ClipartPanel", "Add Folder..."), this);
    auto* remove = new QPushButton(QCoreApplication::translate("ClipartPanel", "Remove Folder"), this);
    auto* refresh = new QPushButton(QCoreApplication::translate("ClipartPanel", "Refresh"), this);
    insert_ = new QPushButton(QCoreApplication::translate("ClipartPanel", "Insert"), this);
    favourite_ = new QPushButton(QCoreApplication::translate("ClipartPanel", "Favourite"), this);

    list_ = new ClipartList(this);
    list_->setViewMode(QListView::IconMode);
    list_->setIconSize(QSize(kThumbEdge, kThumbEdge));
    list_->setResizeMode(QListView::Adjust);
    list_->setMovement(QListView::Static);
    list_->setWordWrap(true);
    list_->setUniformItemSizes(true);
    list_->setSelectionMode(QAbstractItemView::SingleSelection);
    list_->setDragEnabled(true);
    list_->setDragDropMode(QAbstractItemView::DragOnly);
    list_->setGridSize(QSize(kThumbEdge + 24, kThumbEdge + 40));
    status_ = new QLabel(this);
    status_->setWordWrap(true);

    auto* layout = new QVBoxLayout(this);
    layout->addWidget(search_);
    layout->addWidget(favouritesOnly_);
    layout->addWidget(folders_);
    auto* folderButtons = new QWidget(this);
    auto* fl = new QHBoxLayout(folderButtons);
    fl->setContentsMargins(0, 0, 0, 0);
    fl->addWidget(add);
    fl->addWidget(remove);
    fl->addWidget(refresh);
    layout->addWidget(folderButtons);
    layout->addWidget(list_, 1);
    auto* actionButtons = new QWidget(this);
    auto* al = new QHBoxLayout(actionButtons);
    al->setContentsMargins(0, 0, 0, 0);
    al->addWidget(insert_);
    al->addWidget(favourite_);
    layout->addWidget(actionButtons);
    layout->addWidget(status_);

    connect(search_, &QLineEdit::textChanged, this, [this](const QString& t) { setSearch(fromQ(t)); });
    connect(favouritesOnly_, &QCheckBox::toggled, this, [this](bool on) { setFavouritesOnly(on); });
    connect(add, &QPushButton::clicked, this, [this]() {
        const QString dir = QFileDialog::getExistingDirectory(this, QCoreApplication::translate("ClipartPanel", "Add Clipart Folder"));
        if (!dir.isEmpty()) {
            addFolder(fromQ(dir));
        }
    });
    connect(remove, &QPushButton::clicked, this, [this]() {
        if (folders_->currentIndex() >= 0) {
            removeFolder(fromQ(folders_->currentData().toString()));
        }
    });
    connect(refresh, &QPushButton::clicked, this, [this]() { refreshFolders(); });
    connect(insert_, &QPushButton::clicked, this, [this]() { insertRow(list_->currentRow()); });
    connect(favourite_, &QPushButton::clicked, this, [this]() { toggleFavouriteRow(list_->currentRow()); });
    connect(list_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem*) { insertRow(list_->currentRow()); });

    timer_ = new QTimer(this);
    timer_->setInterval(120);
    connect(timer_, &QTimer::timeout, this, [this]() {
        if (isVisible()) {
            pumpThumbnails(false);
        } else {
            applyCompleted();
        }
    });
    timer_->start();
    rebuildList();
}

ClipartPanel::~ClipartPanel() = default;

void ClipartPanel::setPersist(bool persist) {
    if (store_->persist == persist) {
        return;
    }
    store_->persist = persist;
    if (persist) {
        library_->reload();
        service_->clear();
        rebuildList();
    }
}

bool ClipartPanel::addFolder(const std::string& path) {
    const viewmodel::ClipartEdit r = library_->addFolder(path);
    if (!r.ok) {
        if (onMessage) {
            onMessage(r.error);
        }
        return false;
    }
    service_->clear();
    rebuildList();
    return true;
}

bool ClipartPanel::removeFolder(const std::string& path) {
    if (!library_->removeFolder(path)) {
        return false;
    }
    service_->clear();
    rebuildList();
    return true;
}

void ClipartPanel::refreshFolders() {
    library_->rescan();
    service_->clear();
    rebuildList();
}

void ClipartPanel::setSearch(const std::string& text) {
    library_->setQuery(text);
    if (search_->text() != qs(text)) {
        search_->setText(qs(text));  // re-enters through textChanged with the same text: harmless
        return;
    }
    rebuildList();
}

void ClipartPanel::setFavouritesOnly(bool only) {
    library_->setFavouritesOnly(only);
    if (favouritesOnly_->isChecked() != only) {
        favouritesOnly_->setChecked(only);
        return;
    }
    rebuildList();
}

void ClipartPanel::rebuildList() {
    const QString keep = list_->currentItem() != nullptr ? list_->currentItem()->data(kPathRole).toString() : QString();
    list_->clear();
    shown_ = library_->visible();
    for (const std::size_t i : shown_) {
        const viewmodel::ClipartEntry& e = library_->entries()[i];
        QString label = qs(e.name);
        if (library_->isFavourite(e.path)) {
            label = QStringLiteral("* ") + label;
        }
        auto* item = new QListWidgetItem(label, list_);
        item->setData(kPathRole, qs(e.path));
        item->setToolTip(qs(e.path + (e.refusal.empty() ? "" : "\n" + e.refusal)));
        item->setSizeHint(QSize(kThumbEdge + 20, kThumbEdge + 36));
        QPixmap blank(kThumbEdge, kThumbEdge);
        blank.fill(QColor(236, 236, 236));
        item->setIcon(QIcon(blank));
        if (qs(e.path) == keep) {
            list_->setCurrentItem(item);
        }
    }
    folders_->clear();
    for (const std::string& f : library_->folders()) {
        folders_->addItem(QDir::toNativeSeparators(qs(f)), qs(f));
    }
    updateStatus();
}

void ClipartPanel::updateStatus(const std::string& extra) {
    std::string text = QCoreApplication::translate("ClipartPanel", "%1 of %2 clipart file(s)")
                           .arg(shown_.size())
                           .arg(library_->entries().size())
                           .toStdString();
    for (const std::string& m : library_->messages()) {
        text += "\n" + m;
    }
    if (!extra.empty()) {
        text += "\n" + extra;
    }
    status_->setText(qs(text));
}

std::string ClipartPanel::statusText() const {
    return fromQ(status_->text());
}

void ClipartPanel::pumpThumbnails(bool all) {
    const QRect view = list_->viewport()->rect();
    int requested = 0;
    for (int row = 0; row < list_->count() && requested < 64; ++row) {
        QListWidgetItem* item = list_->item(row);
        if (!all && !view.isEmpty() && !list_->visualItemRect(item).intersects(view)) {
            continue;
        }
        const std::string path = fromQ(item->data(kPathRole).toString());
        const viewmodel::ClipartEntry& e = library_->entries()[shown_[static_cast<std::size_t>(row)]];
        if (!e.refusal.empty() || service_->state(path) != viewmodel::ClipartThumbState::Unknown) {
            continue;
        }
        if (!service_->request(path)) {
            break;  // queue full: the next tick continues
        }
        ++requested;
    }
    applyCompleted();
}

void ClipartPanel::waitThumbnails() {
    service_->waitIdle();
    applyCompleted();
}

void ClipartPanel::applyCompleted() {
    for (const std::string& path : service_->takeCompleted()) {
        const QString qpath = qs(path);
        for (int row = 0; row < list_->count(); ++row) {
            QListWidgetItem* item = list_->item(row);
            if (item->data(kPathRole).toString() != qpath) {
                continue;
            }
            const viewmodel::ClipartCacheEntry e = service_->lookup(path);
            if (e.thumb != nullptr) {
                const QImage image(e.thumb->rgb.data(), e.thumb->width, e.thumb->height, e.thumb->width * 3,
                                   QImage::Format_RGB888);
                item->setIcon(QIcon(QPixmap::fromImage(image.copy())));
                item->setToolTip(qpath + QLatin1Char('\n') + qs(e.info));
            } else if (!e.error.empty()) {
                QPixmap bad(kThumbEdge, kThumbEdge);
                bad.fill(QColor(235, 200, 200));
                item->setIcon(QIcon(bad));
                item->setToolTip(qs(e.error));
            }
            break;
        }
    }
}

int ClipartPanel::rowCount() const {
    return list_->count();
}

std::string ClipartPanel::pathOfRow(int row) const {
    return row >= 0 && row < list_->count() ? fromQ(list_->item(row)->data(kPathRole).toString()) : std::string();
}

bool ClipartPanel::rowHasThumbnail(int row) const {
    const std::string path = pathOfRow(row);
    return !path.empty() && service_->state(path) == viewmodel::ClipartThumbState::Ready;
}

std::string ClipartPanel::rowToolTip(int row) const {
    return row >= 0 && row < list_->count() ? fromQ(list_->item(row)->toolTip()) : std::string();
}

bool ClipartPanel::toggleFavouriteRow(int row) {
    const std::string path = pathOfRow(row);
    if (path.empty()) {
        return false;
    }
    const viewmodel::ClipartEdit r = library_->toggleFavourite(path);
    if (!r.ok && onMessage) {
        onMessage(r.error);
    }
    rebuildList();
    pumpThumbnails(false);
    return r.ok;
}

viewmodel::ClipartInsertOutcome ClipartPanel::insertRow(int row, std::optional<geometry::Vec2> at) {
    const std::string path = pathOfRow(row);
    if (path.empty()) {
        viewmodel::ClipartInsertOutcome none;
        none.error = QCoreApplication::translate("ClipartPanel", "Select a clipart file first.").toStdString();
        if (onMessage) {
            onMessage(none.error);
        }
        return none;
    }
    return insertPath(path, at);
}

viewmodel::ClipartInsertOutcome ClipartPanel::insertPath(const std::string& path, std::optional<geometry::Vec2> at) {
    viewmodel::ClipartInsertOutcome out;
    document::Document* d = doc();
    if (d == nullptr) {
        out.error = QCoreApplication::translate("ClipartPanel", "There is no document to insert into.").toStdString();
        return out;
    }
    viewmodel::ClipartInsertOptions options;
    options.at = at;
    options.textFontBytes = viewmodel::starterFontBytes();
    out = viewmodel::insertClipartFile(*d, path, options, qtBitmapDecoder());
    if (out.ok) {
        if (onEdited) {
            onEdited();
        }
        std::string msg = QCoreApplication::translate("ClipartPanel", "Inserted %1 object(s) from the clipart.")
                              .arg(out.objectsAdded)
                              .toStdString();
        if (!out.warnings.empty()) {
            msg += " " + out.warnings.front();
        }
        if (onMessage) {
            onMessage(msg);
        }
        updateStatus(msg);
    } else {
        if (onMessage) {
            onMessage(out.error);
        }
        updateStatus(out.error);
    }
    return out;
}

}  // namespace carvetto::app
