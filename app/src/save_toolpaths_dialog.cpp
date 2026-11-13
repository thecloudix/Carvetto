#include "save_toolpaths_dialog.hpp"

#include <QCheckBox>
#include <QComboBox>
#include <QCoreApplication>
#include <QDialogButtonBox>
#include <QDir>
#include <QElapsedTimer>
#include <QFileDialog>
#include <QFontDatabase>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMetaObject>
#include <QPlainTextEdit>
#include <QPushButton>
#include <QStandardPaths>
#include <QString>
#include <QThread>
#include <QTimer>
#include <QVBoxLayout>
#include <algorithm>
#include <utility>

#include "carvetto/viewmodel/file_session.hpp"
#include "carvetto/viewmodel/tiling_form.hpp"
#include "tiling_manager_panel.hpp"

#ifndef CARVETTO_SHIPPED_POSTS_DIR
#define CARVETTO_SHIPPED_POSTS_DIR ""
#endif

namespace carvetto::app {

namespace fs = std::filesystem;

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

std::string stdString(const QString& s) {
    return s.toStdString();
}

bool hasPostFiles(const fs::path& dir) {
    if (dir.empty()) {
        return false;
    }
    return !viewmodel::discoverPosts(dir, {}).empty();
}

}  // namespace

fs::path shippedPostsDir() {
    const fs::path app = viewmodel::pathFromUtf8(stdString(QCoreApplication::applicationDirPath()));
    const fs::path candidates[] = {app / "posts", app / ".." / "share" / "carvetto" / "posts",
                                   viewmodel::pathFromUtf8(CARVETTO_SHIPPED_POSTS_DIR)};
    for (const fs::path& c : candidates) {
        if (hasPostFiles(c)) {
            return c;
        }
    }
    return {};
}

fs::path userPostsDir() {
    const QString base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
    if (base.isEmpty()) {
        return {};
    }
    return viewmodel::pathFromUtf8(stdString(base)) / "posts";
}

SaveToolpathsDialog::SaveToolpathsDialog(const document::Document& doc, const std::vector<document::ToolpathId>& ids,
                                         const std::string& jobName, const std::vector<tooling::Machine>& machines,
                                         const fs::path& startFolder, QWidget* parent)
    : SaveToolpathsDialog(viewmodel::makeSaveSource(doc, ids), jobName, machines, startFolder, parent) {}

SaveToolpathsDialog::SaveToolpathsDialog(viewmodel::SaveSource source, const std::string& jobName,
                                         const std::vector<tooling::Machine>& machines, const fs::path& startFolder,
                                         QWidget* parent)
    : QDialog(parent),
      source_(std::move(source)),
      jobName_(jobName),
      machines_(machines),
      posts_(viewmodel::discoverPosts(shippedPostsDir(), userPostsDir())),
      cancel_(std::make_shared<std::atomic<bool>>(false)) {
    setWindowTitle(QStringLiteral("Save Toolpaths"));
    setObjectName(QStringLiteral("saveToolpathsDialog"));
    resize(760, 900);

    auto* layout = new QVBoxLayout(this);
    auto* form = new QFormLayout();
    layout->addLayout(form);

    machine_ = new QComboBox(this);
    machine_->setObjectName(QStringLiteral("saveMachine"));
    machine_->addItem(QStringLiteral("(no machine)"));
    for (const tooling::Machine& m : machines_) {
        machine_->addItem(qs(m.name.empty() ? m.id : m.name));
    }
    form->addRow(QStringLiteral("Machine:"), machine_);

    units_ = new QComboBox(this);
    units_->setObjectName(QStringLiteral("saveUnits"));
    units_->addItem(QStringLiteral("Any units"));
    units_->addItem(QStringLiteral("Millimetres"));
    units_->addItem(QStringLiteral("Inches"));
    form->addRow(QStringLiteral("Post units:"), units_);
    // Start with the units of the job.
    units_->setCurrentIndex(source_.job.displayUnits == document::Units::Inches ? 2 : 1);

    post_ = new QComboBox(this);
    post_->setObjectName(QStringLiteral("savePost"));
    form->addRow(QStringLiteral("Post-processor:"), post_);

    mode_ = new QComboBox(this);
    mode_->setObjectName(QStringLiteral("saveMode"));
    mode_->addItem(QStringLiteral("One file with all toolpaths"));
    mode_->addItem(QStringLiteral("One file per toolpath"));
    mode_->addItem(QStringLiteral("One file per tool"));
    form->addRow(QStringLiteral("Output:"), mode_);

    auto* folderRow = new QHBoxLayout();
    folder_ = new QLineEdit(this);
    folder_->setObjectName(QStringLiteral("saveFolder"));
    folder_->setText(qs(viewmodel::pathToUtf8(startFolder)));
    auto* browse = new QPushButton(QStringLiteral("Browse..."), this);
    folderRow->addWidget(folder_, 1);
    folderRow->addWidget(browse);
    form->addRow(QStringLiteral("Folder:"), folderRow);

    template_ = new QLineEdit(this);
    template_->setObjectName(QStringLiteral("saveTemplate"));
    template_->setText(qs(viewmodel::defaultNameTemplate(viewmodel::OutputMode::SingleFile)));
    template_->setToolTip(QStringLiteral("File name without extension. Fields: {job} {toolpath} {tool} {n}"));
    form->addRow(QStringLiteral("File name:"), template_);

    tiling_ = new TilingManagerPanel(this);
    layout->addWidget(tiling_);

    overwrite_ = new QCheckBox(QStringLiteral("Replace existing files"), this);
    overwrite_->setObjectName(QStringLiteral("saveOverwrite"));
    layout->addWidget(overwrite_);

    acceptStale_ = new QCheckBox(QStringLiteral("Post the out-of-date toolpaths from their old moves anyway"), this);
    acceptStale_->setObjectName(QStringLiteral("saveAcceptStale"));
    acceptStale_->setVisible(!source_.stale.empty());
    layout->addWidget(acceptStale_);
    connect(acceptStale_, &QCheckBox::toggled, this, [this](bool) { showState(); });

    summary_ = new QLabel(this);
    summary_->setObjectName(QStringLiteral("saveSummary"));
    summary_->setWordWrap(true);
    layout->addWidget(summary_);

    preview_ = new QPlainTextEdit(this);
    preview_->setObjectName(QStringLiteral("savePreview"));
    preview_->setReadOnly(true);
    preview_->setLineWrapMode(QPlainTextEdit::NoWrap);
    preview_->setFont(QFontDatabase::systemFont(QFontDatabase::FixedFont));
    layout->addWidget(preview_, 1);

    message_ = new QLabel(this);
    message_->setObjectName(QStringLiteral("saveMessage"));
    message_->setWordWrap(true);
    layout->addWidget(message_);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel, this);
    save_ = buttons->button(QDialogButtonBox::Save);
    save_->setObjectName(QStringLiteral("saveButton"));
    layout->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, this, [this]() { accept(); });
    connect(buttons, &QDialogButtonBox::rejected, this, [this]() { reject(); });

    debounce_ = new QTimer(this);
    debounce_->setSingleShot(true);
    debounce_->setInterval(300);
    connect(debounce_, &QTimer::timeout, this, [this]() { settingsChanged(); });

    connect(browse, &QPushButton::clicked, this, [this]() {
        const QString dir = QFileDialog::getExistingDirectory(this, QStringLiteral("Output folder"), folder_->text());
        if (!dir.isEmpty()) {
            folder_->setText(dir);
        }
    });
    connect(machine_, &QComboBox::currentIndexChanged, this, [this](int) {
        if (updating_) {
            return;
        }
        if (const tooling::Machine* cur = currentMachine()) {
            tiling_->setMachineWorkArea(cur->workAreaX, cur->workAreaY);
        } else {
            tiling_->setMachineWorkArea(0.0, 0.0);
        }
        fillPosts();
        // The machine's own default post, when it names one.
        const int m = machine_->currentIndex() - 1;
        if (m >= 0 && static_cast<std::size_t>(m) < machines_.size()) {
            const auto def = viewmodel::defaultPostFor(posts_, machines_[static_cast<std::size_t>(m)]);
            if (def) {
                const auto at = std::find(listed_.begin(), listed_.end(), *def);
                if (at != listed_.end()) {
                    post_->setCurrentIndex(static_cast<int>(at - listed_.begin()));
                }
            }
        }
        settingsChanged();
    });
    connect(units_, &QComboBox::currentIndexChanged, this, [this](int) {
        if (!updating_) {
            fillPosts();
            settingsChanged();
        }
    });
    connect(post_, &QComboBox::currentIndexChanged, this, [this](int) { settingsChanged(); });
    connect(mode_, &QComboBox::currentIndexChanged, this, [this](int row) {
        if (updating_) {
            return;
        }
        // The template follows the mode while it still is a default one.
        const std::string current = stdString(template_->text());
        for (int k = 0; k < 3; ++k) {
            if (current == viewmodel::defaultNameTemplate(static_cast<viewmodel::OutputMode>(k))) {
                template_->setText(qs(viewmodel::defaultNameTemplate(static_cast<viewmodel::OutputMode>(row))));
                break;
            }
        }
        settingsChanged();
    });
    connect(folder_, &QLineEdit::textChanged, this, [this](const QString&) { typingChanged(); });
    connect(template_, &QLineEdit::textChanged, this, [this](const QString&) { typingChanged(); });
    connect(overwrite_, &QCheckBox::toggled, this, [this](bool) { settingsChanged(); });
    tiling_->onChanged = [this]() { settingsChanged(); };
    tiling_->onTyped = [this]() { typingChanged(); };

    fillPosts();
    settingsChanged();
}

SaveToolpathsDialog::~SaveToolpathsDialog() {
    cancel_->store(true);
    finishThread();
}

void SaveToolpathsDialog::finishThread() {
    if (thread_ != nullptr) {
        thread_->wait();
        delete thread_;
        thread_ = nullptr;
    }
}

void SaveToolpathsDialog::fillPosts() {
    updating_ = true;
    const fs::path keep = post_->currentIndex() >= 0 && static_cast<std::size_t>(post_->currentIndex()) < listed_.size()
                              ? posts_[listed_[static_cast<std::size_t>(post_->currentIndex())]].path
                              : fs::path();
    std::vector<std::size_t> base;
    const int m = machine_->currentIndex() - 1;
    if (m >= 0 && static_cast<std::size_t>(m) < machines_.size()) {
        base = viewmodel::postsForMachine(posts_, machines_[static_cast<std::size_t>(m)]);
    } else {
        for (std::size_t i = 0; i < posts_.size(); ++i) {
            if (posts_[i].usable()) {
                base.push_back(i);
            }
        }
    }
    listed_.clear();
    for (std::size_t i : base) {
        const int u = units_->currentIndex();
        if (u == 1 && posts_[i].units != post::Units::Millimetres) {
            continue;
        }
        if (u == 2 && posts_[i].units != post::Units::Inches) {
            continue;
        }
        listed_.push_back(i);
    }
    post_->clear();
    int select = 0;
    for (std::size_t k = 0; k < listed_.size(); ++k) {
        const viewmodel::PostInfo& p = posts_[listed_[k]];
        post_->addItem(qs(p.name + (p.shipped ? "" : "  [mine]")));
        post_->setItemData(static_cast<int>(k), qs(viewmodel::pathToUtf8(p.path)), Qt::ToolTipRole);
        if (!keep.empty() && p.path == keep) {
            select = static_cast<int>(k);
        }
    }
    if (!listed_.empty()) {
        post_->setCurrentIndex(select);
    }
    updating_ = false;
}

viewmodel::SaveOptions SaveToolpathsDialog::currentOptions() const {
    viewmodel::SaveOptions o;
    o.mode = static_cast<viewmodel::OutputMode>(std::max(0, mode_->currentIndex()));
    o.directory = viewmodel::pathFromUtf8(stdString(folder_->text()));
    o.nameTemplate = stdString(template_->text());
    o.jobName = jobName_;
    viewmodel::applyTiling(o, tiling_->form(), currentMachine());
    return o;
}

const tooling::Machine* SaveToolpathsDialog::currentMachine() const {
    const int m = machine_->currentIndex() - 1;
    if (m >= 0 && static_cast<std::size_t>(m) < machines_.size()) {
        return &machines_[static_cast<std::size_t>(m)];
    }
    return nullptr;
}

void SaveToolpathsDialog::setMachineIndex(int index) {
    machine_->setCurrentIndex(index);
}

void SaveToolpathsDialog::updateTilePreview() {
    const viewmodel::TilingForm form = tiling_->form();
    if (!form.enabled) {
        tiling_->showPreview({});
        return;
    }
    const tooling::Machine* m = currentMachine();
    tiling_->showPreview(viewmodel::previewTiling(source_.records, form, m != nullptr ? m->workAreaX : 0.0,
                                                  m != nullptr ? m->workAreaY : 0.0));
}

void SaveToolpathsDialog::typingChanged() {
    if (updating_) {
        return;
    }
    outputFresh_ = false;
    writeError_.clear();
    save_->setEnabled(false);
    debounce_->start();
}

void SaveToolpathsDialog::settingsChanged() {
    if (updating_) {
        return;
    }
    debounce_->stop();
    outputFresh_ = false;
    writeError_.clear();
    updateTilePreview();
    startGeneration();
    showState();
}

void SaveToolpathsDialog::startGeneration() {
    if (running_) {
        again_ = true;
        return;
    }
    viewmodel::PostInfo post;
    const int row = post_->currentIndex();
    if (row >= 0 && static_cast<std::size_t>(row) < listed_.size()) {
        post = posts_[listed_[static_cast<std::size_t>(row)]];
    }
    // The post file is read here (small); the work on the copy happens off the UI thread.
    viewmodel::SaveJob job = viewmodel::prepareSave(source_, post, currentOptions());
    running_ = true;
    again_ = false;
    const unsigned generation = ++generation_;
    auto cancel = cancel_;
    thread_ = QThread::create([this, job = std::move(job), generation, cancel]() {
        viewmodel::SaveOutput out = viewmodel::generateOutput(job, cancel.get());
        QMetaObject::invokeMethod(
            this, [this, generation, out = std::move(out)]() mutable { generationDone(std::move(out), generation); },
            Qt::QueuedConnection);
    });
    thread_->start();
    setBusy(true);
}

void SaveToolpathsDialog::generationDone(viewmodel::SaveOutput out, unsigned generation) {
    finishThread();
    running_ = false;
    if (again_ || generation != generation_) {
        again_ = false;
        startGeneration();
        return;
    }
    output_ = std::move(out);
    outputFresh_ = true;
    setBusy(false);
    showState();
}

void SaveToolpathsDialog::startWrite() {
    if (running_ || !outputFresh_ || !output_.ok) {
        return;
    }
    running_ = true;
    writing_ = true;
    writeError_.clear();
    written_.clear();
    viewmodel::SaveOutput out = output_;
    const bool overwrite = overwrite_->isChecked();
    thread_ = QThread::create([this, out = std::move(out), overwrite]() {
        viewmodel::WriteResult r = viewmodel::writeOutput(out, overwrite);
        QMetaObject::invokeMethod(
            this, [this, r = std::move(r)]() mutable { writeDone(std::move(r)); }, Qt::QueuedConnection);
    });
    thread_->start();
    setBusy(true);
}

void SaveToolpathsDialog::writeDone(viewmodel::WriteResult result) {
    finishThread();
    running_ = false;
    writing_ = false;
    setBusy(false);
    if (result.ok) {
        written_ = std::move(result.written);
        writeError_.clear();
        if (closeAfterWrite_) {
            QDialog::accept();
        }
    } else {
        writeError_ = result.error;
        // The folder may have changed meanwhile: generate again so "exists" is current.
        outputFresh_ = false;
        startGeneration();
    }
    closeAfterWrite_ = false;
    showState();
}

void SaveToolpathsDialog::setBusy(bool busy) {
    save_->setEnabled(!busy && canSave());
    setCursor(busy ? Qt::BusyCursor : Qt::ArrowCursor);
}

bool SaveToolpathsDialog::canSave() const {
    const bool staleOk = source_.stale.empty() || acceptStale_->isChecked();
    return !running_ && outputFresh_ && output_.ok && !output_.files.empty() && staleOk;
}

void SaveToolpathsDialog::showState() {
    save_->setEnabled(canSave());
    QString summary;
    QString message;
    QString previewText;
    if (running_ && !writing_) {
        summary = QStringLiteral("Generating...");
    } else if (outputFresh_ && output_.ok) {
        std::size_t toolpaths = 0;
        QStringList names;
        for (const viewmodel::OutputFile& f : output_.files) {
            toolpaths += f.toolpaths.size();
            names << qs(viewmodel::pathToUtf8(f.path.filename()));
        }
        summary = QStringLiteral("%1 file(s), %2 toolpath(s), %3 tool change(s): %4")
                      .arg(output_.files.size())
                      .arg(toolpaths)
                      .arg(output_.toolChanges)
                      .arg(names.join(QStringLiteral(", ")));
        previewText = qs(viewmodel::previewText(output_, 40));
        for (const std::string& w : output_.warnings) {
            message += QStringLiteral("Warning: ") + qs(w) + QLatin1Char('\n');
        }
        if (!output_.stale.empty() && !acceptStale_->isChecked()) {
            message += QStringLiteral("Tick the box above to post the out-of-date toolpaths, or recalculate them first.\n");
        }
        bool exists = false;
        for (const viewmodel::OutputFile& f : output_.files) {
            exists = exists || f.exists;
        }
        if (exists && !overwrite_->isChecked()) {
            message += QStringLiteral("A file with this name exists already (tick 'Replace existing files').\n");
        }
    } else if (outputFresh_) {
        message = qs(output_.error);
    }
    for (const viewmodel::PostInfo& p : posts_) {
        if (!p.usable()) {
            message += QStringLiteral("Post not usable: ") + qs(p.error) + QLatin1Char('\n');
        }
    }
    if (!writeError_.empty()) {
        message = QStringLiteral("Not saved: ") + qs(writeError_) + QLatin1Char('\n') + message;
    }
    summary_->setText(summary);
    preview_->setPlainText(previewText);
    message_->setText(message.trimmed());
}

std::string SaveToolpathsDialog::previewShown() const {
    return stdString(preview_->toPlainText());
}

std::string SaveToolpathsDialog::messageShown() const {
    return stdString(message_->text());
}

void SaveToolpathsDialog::setFolder(const fs::path& folder) {
    folder_->setText(qs(viewmodel::pathToUtf8(folder)));
}

void SaveToolpathsDialog::setOutputMode(viewmodel::OutputMode mode) {
    mode_->setCurrentIndex(static_cast<int>(mode));
}

void SaveToolpathsDialog::setNameTemplate(const std::string& tmpl) {
    template_->setText(qs(tmpl));
}

void SaveToolpathsDialog::setAcceptStale(bool accept) {
    acceptStale_->setChecked(accept);
}

bool SaveToolpathsDialog::staleBoxShown() const {
    return !acceptStale_->isHidden();
}

void SaveToolpathsDialog::setOverwrite(bool overwrite) {
    overwrite_->setChecked(overwrite);
}

bool SaveToolpathsDialog::selectPost(const std::string& fileName) {
    updating_ = true;
    units_->setCurrentIndex(0);
    machine_->setCurrentIndex(0);
    updating_ = false;
    tiling_->setMachineWorkArea(0.0, 0.0);
    fillPosts();
    for (std::size_t k = 0; k < listed_.size(); ++k) {
        if (viewmodel::pathToUtf8(posts_[listed_[k]].path.filename()) == fileName) {
            post_->setCurrentIndex(static_cast<int>(k));
            settingsChanged();
            return true;
        }
    }
    settingsChanged();
    return false;
}

int SaveToolpathsDialog::postCount() const {
    return post_->count();
}

bool SaveToolpathsDialog::waitIdle() {
    QElapsedTimer timer;
    timer.start();
    if (debounce_->isActive()) {  // typing pending: generate now
        debounce_->stop();
        settingsChanged();
    }
    while (running_ && timer.elapsed() < 10000) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
        QThread::msleep(2);
    }
    return !running_;
}

bool SaveToolpathsDialog::saveNow() {
    if (!waitIdle() || !canSave()) {
        return false;
    }
    startWrite();
    if (!waitIdle()) {
        return false;
    }
    return !written_.empty() && writeError_.empty();
}

void SaveToolpathsDialog::accept() {
    if (!canSave()) {
        return;
    }
    bool exists = false;
    for (const viewmodel::OutputFile& f : output_.files) {
        exists = exists || f.exists;
    }
    if (exists && !overwrite_->isChecked()) {
        const auto answer = QMessageBox::question(this, QStringLiteral("Replace files"),
                                                  QStringLiteral("A file with this name exists already. Replace it?"));
        if (answer != QMessageBox::Yes) {
            return;
        }
        overwrite_->setChecked(true);
        // The check box regenerates; save once that finished.
        waitIdle();
        if (!canSave()) {
            return;
        }
    }
    closeAfterWrite_ = true;
    startWrite();
}

void SaveToolpathsDialog::reject() {
    closeAfterWrite_ = false;
    cancel_->store(true);
    finishThread();  // a running write finishes; its result is dropped with the dialog
    running_ = false;
    QDialog::reject();
}

}  // namespace carvetto::app
