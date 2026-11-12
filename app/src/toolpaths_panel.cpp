#include "toolpaths_panel.hpp"

#include <QAction>
#include <QColor>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMetaObject>
#include <QProgressBar>
#include <QPushButton>
#include <QStringList>
#include <QThread>
#include <QToolBar>
#include <QToolButton>
#include <QTreeWidget>
#include <QVariant>
#include <QVBoxLayout>
#include <algorithm>
#include <cstdio>
#include <set>
#include <utility>

#include "carvetto/tooling/tool_database.hpp"
#include "carvetto/tooling/json_io.hpp"
#include "carvetto/viewmodel/vector_selector.hpp"
#include "carvetto/viewmodel/tool_setup.hpp"
#include "bitmap_decoder.hpp"
#include "starter_tools.hpp"
#include "carvetto/viewmodel/variables_model.hpp"
#include "toolpath_form_dialog.hpp"

namespace carvetto::app {

using document::ToolpathId;
using document::ToolpathOperation;
using viewmodel::MessageSeverity;
using viewmodel::ToolpathMessage;
using viewmodel::ToolpathStatus;

namespace {

QString qs(const std::string& s) {
    return QString::fromStdString(s);
}

constexpr int kIdRole = Qt::UserRole + 1;

/// Drops repeated "; "-separated parts (the same refusal can be reported for several vectors).
QString uniqueParts(const QString& text) {
    QStringList seen;
    for (const QString& part : text.split(QStringLiteral("; "), Qt::SkipEmptyParts)) {
        if (!seen.contains(part)) {
            seen << part;
        }
    }
    return seen.join(QStringLiteral("; "));
}

QString durationText(double seconds) {
    const long long total = static_cast<long long>(seconds + 0.5);
    const long long h = total / 3600;
    const long long m = (total / 60) % 60;
    const long long s = total % 60;
    if (h > 0) {
        return QStringLiteral("%1 h %2 min").arg(h).arg(m);
    }
    if (m > 0) {
        return QStringLiteral("%1 min %2 s").arg(m).arg(s);
    }
    return QStringLiteral("%1 s").arg(s);
}

QString severityText(MessageSeverity s) {
    switch (s) {
        case MessageSeverity::Info:
            return QStringLiteral("Note");
        case MessageSeverity::Warning:
            return QStringLiteral("Warning");
        case MessageSeverity::Error:
            return QStringLiteral("Error");
    }
    return QString();
}

bool hasError(const std::vector<ToolpathMessage>& messages) {
    return std::any_of(messages.begin(), messages.end(),
                       [](const ToolpathMessage& m) { return m.severity == MessageSeverity::Error; });
}

tooling::ToolDatabase loadStarterTools() {
    tooling::ImportResult r = tooling::importToolDatabaseJson(starterToolsJson());
    if (!r.ok) {
        std::fprintf(stderr, "carvetto: the built-in tool database is invalid: %s\n", r.error.c_str());
        return tooling::ToolDatabase{};
    }
    return std::move(r.database);
}

}  // namespace

ToolpathsPanel::ToolpathsPanel(document::Document* doc, QWidget* parent)
    : QWidget(parent), session_(doc), doc_(doc), db_(loadStarterTools()) {
    setObjectName(QStringLiteral("ToolpathsPanel"));
    session_.setToolDatabase(db_);
    session_.setBitmapDecoder(qtBitmapDecoder());
    if (const tooling::Machine* machine = viewmodel::activeMachine(db_, std::string())) {
        activeMachineId_ = machine->id;
        session_.setMachine(*machine, machine->rapidRate);
    }
    build();
    refresh();
}

void ToolpathsPanel::applySetup(tooling::ToolDatabase db, const std::string& activeMachineId) {
    db_ = std::move(db);
    const tooling::Machine* machine = viewmodel::activeMachine(db_, activeMachineId);
    activeMachineId_ = machine != nullptr ? machine->id : std::string();
    session_.setToolDatabase(db_);
    if (machine != nullptr) {
        session_.setMachine(*machine, machine->rapidRate);
    }
    setupSaveError_.clear();
    if (!setupFile_.empty()) {
        viewmodel::writeToolDatabaseFile(setupFile_, db_, setupSaveError_);
    }
    refresh();
    if (onSetupChanged) {
        onSetupChanged();
    }
}

std::string ToolpathsPanel::useSetupFile(const std::filesystem::path& path, const std::string& activeMachineId) {
    setupFile_ = path;
    viewmodel::SetupLoadResult loaded = viewmodel::loadToolSetup(path, db_);
    const std::string message = loaded.message;
    // Applying also writes the file; do not rewrite a file that was just read, or replace a refused one.
    const std::filesystem::path keep = std::move(setupFile_);
    setupFile_.clear();
    applySetup(std::move(loaded.database), activeMachineId);
    setupFile_ = keep;
    return message;
}

ToolpathsPanel::~ToolpathsPanel() {
    // Cancel and join the worker first (it may be inside a core call): nothing it does touches the
    // document or the widgets, only its own copy of the job.
    runner_.cancel();
    runner_.take();
    delete dialog_.data();
}

void ToolpathsPanel::build() {
    auto* outer = new QVBoxLayout(this);
    outer->setContentsMargins(4, 4, 4, 4);

    bar_ = new QToolBar(this);
    bar_->setObjectName(QStringLiteral("toolpathsToolBar"));
    bar_->setToolButtonStyle(Qt::ToolButtonTextOnly);
    auto* addMenu = new QMenu(bar_);
    for (ToolpathOperation op : {ToolpathOperation::Profile, ToolpathOperation::Pocket, ToolpathOperation::Drill,
                                 ToolpathOperation::Engrave, ToolpathOperation::VCarving, ToolpathOperation::Chamfer,
                                 ToolpathOperation::ThreadMill, ToolpathOperation::Prism, ToolpathOperation::Fluting,
                                 ToolpathOperation::Inlay, ToolpathOperation::PhotoVCarve}) {
        QAction* a = addMenu->addAction(QString::fromUtf8(operationLabel(op)));
        a->setObjectName(QStringLiteral("add%1").arg(QString::fromUtf8(operationLabel(op))));
        a->setToolTip(QStringLiteral("Add a %1 toolpath").arg(QString::fromUtf8(operationLabel(op))));
        addActions_[static_cast<int>(op)] = a;
        connect(a, &QAction::triggered, this, [this, op]() { openAddForm(op); });
    }
    auto* addButton = new QToolButton(bar_);
    addButton->setText(QStringLiteral("Add"));
    addButton->setObjectName(QStringLiteral("toolpathAddButton"));
    addButton->setMenu(addMenu);
    addButton->setPopupMode(QToolButton::InstantPopup);
    bar_->addWidget(addButton);

    const auto make = [this](const QString& text, const QString& tip) {
        QAction* a = bar_->addAction(text);
        a->setToolTip(tip);
        a->setObjectName(text);
        return a;
    };
    edit_ = make(QStringLiteral("Edit"), QStringLiteral("Edit the toolpath's settings and recalculate"));
    duplicate_ = make(QStringLiteral("Duplicate"), QStringLiteral("Copy the toolpath"));
    delete_ = make(QStringLiteral("Delete"), QStringLiteral("Delete the selected toolpaths"));
    up_ = make(QStringLiteral("Up"), QStringLiteral("Move the toolpath earlier in the job"));
    down_ = make(QStringLiteral("Down"), QStringLiteral("Move the toolpath later in the job"));
    recalculate_ = make(QStringLiteral("Recalculate"), QStringLiteral("Recalculate the toolpath"));
    recalculateAll_ = make(QStringLiteral("Recalculate all"), QStringLiteral("Recalculate every toolpath that needs it"));
    connect(edit_, &QAction::triggered, this, [this]() { openEditForm(currentToolpath()); });
    connect(duplicate_, &QAction::triggered, this, [this]() { doDuplicate(); });
    connect(delete_, &QAction::triggered, this, [this]() { doDelete(); });
    connect(up_, &QAction::triggered, this, [this]() { doMove(-1); });
    connect(down_, &QAction::triggered, this, [this]() { doMove(1); });
    connect(recalculate_, &QAction::triggered, this, [this]() { doRecalculate(); });
    connect(recalculateAll_, &QAction::triggered, this, [this]() { doRecalculateAll(); });
    outer->addWidget(bar_);

    list_ = new QTreeWidget(this);
    list_->setObjectName(QStringLiteral("toolpathList"));
    list_->setColumnCount(4);
    list_->setHeaderLabels({QStringLiteral("Toolpath"), QStringLiteral("Operation"), QStringLiteral("Tool"),
                            QStringLiteral("State")});
    list_->setRootIsDecorated(false);
    list_->setSelectionMode(QAbstractItemView::ExtendedSelection);
    list_->header()->setStretchLastSection(true);
    outer->addWidget(list_, 3);
    connect(list_, &QTreeWidget::currentItemChanged, this, [this](QTreeWidgetItem* item, QTreeWidgetItem*) {
        if (refreshing_) {
            return;
        }
        if (item != nullptr) {
            linkSelection(ToolpathId{item->data(0, kIdRole).toULongLong()});
        }
        rebuildDiagnostics();
        updateActions();
    });
    connect(list_, &QTreeWidget::itemSelectionChanged, this, [this]() {
        if (!refreshing_) {
            updateActions();
        }
    });
    connect(list_, &QTreeWidget::itemChanged, this, [this](QTreeWidgetItem* item, int column) {
        if (refreshing_ || column != 0) {
            return;
        }
        session_.setEnabled(ToolpathId{item->data(0, kIdRole).toULongLong()}, item->checkState(0) == Qt::Checked);
        rebuildSummary();
    });
    connect(list_, &QTreeWidget::itemDoubleClicked, this, [this](QTreeWidgetItem* item, int) {
        if (item != nullptr && !busy_) {
            openEditForm(ToolpathId{item->data(0, kIdRole).toULongLong()});
        }
    });

    summary_ = new QLabel(this);
    summary_->setObjectName(QStringLiteral("toolpathSummary"));
    summary_->setWordWrap(true);
    outer->addWidget(summary_);

    auto* busyRow = new QHBoxLayout();
    busyLabel_ = new QLabel(this);
    busyLabel_->setObjectName(QStringLiteral("toolpathBusy"));
    progress_ = new QProgressBar(this);
    progress_->setRange(0, 0);
    progress_->setMaximumHeight(10);
    progress_->setTextVisible(false);
    cancelButton_ = new QPushButton(QStringLiteral("Cancel"), this);
    cancel_ = new QAction(QStringLiteral("Cancel"), this);
    cancel_->setObjectName(QStringLiteral("toolpathCancel"));
    connect(cancel_, &QAction::triggered, this, [this]() {
        runner_.cancel();
        busyLabel_->setText(QStringLiteral("Cancelling (waits for the current step to finish)..."));
    });
    connect(cancelButton_, &QPushButton::clicked, cancel_, &QAction::trigger);
    busyRow->addWidget(busyLabel_, 1);
    busyRow->addWidget(progress_, 1);
    busyRow->addWidget(cancelButton_);
    outer->addLayout(busyRow);
    busyLabel_->setVisible(false);
    progress_->setVisible(false);
    cancelButton_->setVisible(false);

    statusLabel_ = new QLabel(this);
    statusLabel_->setObjectName(QStringLiteral("toolpathStatus"));
    statusLabel_->setWordWrap(true);
    outer->addWidget(statusLabel_);

    diagnostics_ = new QListWidget(this);
    diagnostics_->setObjectName(QStringLiteral("toolpathDiagnostics"));
    diagnostics_->setToolTip(QStringLiteral("Messages of the selected toolpath's last calculation. "
                                            "Double-click one to select the vector it names."));
    outer->addWidget(diagnostics_, 2);
    connect(diagnostics_, &QListWidget::itemDoubleClicked, this, [this](QListWidgetItem* item) {
        const qulonglong object = item->data(Qt::UserRole).toULongLong();
        if (object == 0 || doc_ == nullptr || !doc_->hasObject(document::ObjectId{object})) {
            return;
        }
        doc_->selection().assign({document::ObjectId{object}});
        if (onSelectionChanged) {
            onSelectionChanged();
        }
    });
    updateActions();
}

QAction* ToolpathsPanel::addAction(ToolpathOperation operation) const {
    const auto it = addActions_.find(static_cast<int>(operation));
    return it == addActions_.end() ? nullptr : it->second;
}

void ToolpathsPanel::showClosing() {
    if (busy_) {
        busyLabel_->setText(QStringLiteral("Closing... (waiting for the calculation to stop)"));
        busyLabel_->setVisible(true);
        QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);
    }
}

ToolpathFormDialog* ToolpathsPanel::formDialog() const {
    return dialog_.data();
}

ToolpathId ToolpathsPanel::currentToolpath() const {
    const QTreeWidgetItem* item = list_->currentItem();
    return item == nullptr ? ToolpathId{} : ToolpathId{item->data(0, kIdRole).toULongLong()};
}

QString ToolpathsPanel::statusText() const {
    return status_;
}

void ToolpathsPanel::setStatus(const QString& text, bool error) {
    status_ = text;
    statusError_ = error;
    status_ = uniqueParts(text);
    statusLabel_->setText(status_);
    statusLabel_->setStyleSheet(error ? QStringLiteral("color: #b00020;") : QString());
}

void ToolpathsPanel::setDocument(document::Document* doc) {
    // The old document may be gone already: do not touch doc_ before replacing it.
    ++generation_;
    runner_.cancel();
    // The session first: it forgets the variable references without touching the old document, so
    // closing the form (which clears its variables) cannot reach a document that is gone.
    session_.setDocument(doc);
    pendingExpressions_.clear();
    if (dialog_) {
        dialog_->reject();
        delete dialog_.data();
    }
    doc_ = doc;
    failed_.clear();
    lastMessages_.clear();
    setStatus(busy_ ? QStringLiteral("Finishing the previous calculation; its result will be discarded.")
                    : QString(),
              false);
    refresh();
}

void ToolpathsPanel::invalidate() {
    session_.invalidate();
    refresh();
}

void ToolpathsPanel::refresh() {
    rebuildList();
    rebuildDiagnostics();
    rebuildSummary();
    updateActions();
}

void ToolpathsPanel::rebuildList() {
    const ToolpathId keep = currentToolpath();
    std::set<ToolpathId> keepSelected;
    for (const QTreeWidgetItem* item : list_->selectedItems()) {
        keepSelected.insert(ToolpathId{item->data(0, kIdRole).toULongLong()});
    }
    refreshing_ = true;
    list_->clear();
    if (doc_ != nullptr) {
        for (const viewmodel::ToolpathListItem& it : session_.list()) {
            auto* row = new QTreeWidgetItem(list_);
            row->setData(0, kIdRole, QVariant::fromValue(static_cast<qulonglong>(it.id.value)));
            row->setText(0, qs(it.name));
            row->setFlags(row->flags() | Qt::ItemIsUserCheckable);
            row->setCheckState(0, it.enabled ? Qt::Checked : Qt::Unchecked);
            row->setText(1, QString::fromUtf8(operationLabel(it.operation)));
            row->setText(2, qs(it.toolName));
            QString state;
            QString tip;
            const auto failedIt = failed_.find(it.id);
            const bool errored = failedIt != failed_.end() || hasError(session_.lastMessages(it.id));
            if (errored) {
                state = QStringLiteral("Error");
                tip = QStringLiteral("The last calculation reported errors; see the messages below.");
            } else if (it.status == ToolpathStatus::NotComputed) {
                state = QStringLiteral("Not computed");
            } else if (it.status == ToolpathStatus::NeedsRecalculation) {
                state = QStringLiteral("Stale");
                QStringList why;
                if (it.vectorsChanged) {
                    why << QStringLiteral("the vectors changed");
                }
                if (it.parametersChanged) {
                    why << QStringLiteral("the settings changed");
                }
                if (it.toolChanged) {
                    why << QStringLiteral("the tool changed");
                }
                tip = QStringLiteral("Needs recalculation: ") + why.join(QStringLiteral(", "));
            } else {
                state = QStringLiteral("Valid");
                tip = QStringLiteral("%1 moves").arg(static_cast<qulonglong>(it.moveCount));
            }
            if (!it.canRecalculate && !it.note.empty()) {
                tip += (tip.isEmpty() ? QString() : QStringLiteral(". ")) + qs(it.note);
            }
            row->setText(3, state);
            for (int c = 0; c < 4; ++c) {
                row->setToolTip(c, tip);
            }
            if (keepSelected.contains(it.id)) {
                row->setSelected(true);
            }
            if (it.id == keep) {
                list_->setCurrentItem(row, 0, QItemSelectionModel::NoUpdate);
            }
        }
    }
    for (int c = 0; c < 4; ++c) {
        list_->resizeColumnToContents(c);
    }
    refreshing_ = false;
}

void ToolpathsPanel::rebuildDiagnostics() {
    diagnostics_->clear();
    const auto add = [this](const ToolpathMessage& m, const QString& prefix) {
        QString text = QStringLiteral("%1%2: %3").arg(prefix, severityText(m.severity), qs(m.text));
        if (m.vector.has_value()) {
            text += QStringLiteral(" [object %1]").arg(static_cast<qulonglong>(m.vector->value));
        }
        auto* item = new QListWidgetItem(text, diagnostics_);
        if (m.vector.has_value()) {
            item->setData(Qt::UserRole, QVariant::fromValue(static_cast<qulonglong>(m.vector->value)));
        }
        if (m.severity == MessageSeverity::Error) {
            item->setForeground(QColor(0xb0, 0x00, 0x20));
        } else if (m.severity == MessageSeverity::Warning) {
            item->setForeground(QColor(0x9a, 0x5b, 0x00));
        }
    };
    for (const ToolpathMessage& m : lastMessages_) {
        add(m, QString());
    }
    const ToolpathId id = currentToolpath();
    if (doc_ != nullptr && !id.isNull()) {
        const auto failedIt = failed_.find(id);
        const std::vector<ToolpathMessage> own =
            failedIt != failed_.end() ? failedIt->second : session_.lastMessages(id);
        for (const ToolpathMessage& m : own) {
            add(m, QString());
        }
    }
}

void ToolpathsPanel::rebuildSummary() {
    if (doc_ == nullptr) {
        summary_->clear();
        return;
    }
    const viewmodel::JobSummary s = session_.summary();
    QString text = QStringLiteral("%1 toolpath(s), %2 enabled and computed, %3 moves")
                       .arg(static_cast<qulonglong>(s.toolpaths))
                       .arg(static_cast<qulonglong>(s.included))
                       .arg(static_cast<qulonglong>(s.moves));
    if (s.included > 0) {
        text += QStringLiteral("; estimated time %1").arg(durationText(s.time.total()));
        if (s.rapidRateMissing) {
            text += QStringLiteral(" (without rapid moves: the machine's rapid rate is not set)");
        }
        text += QStringLiteral("; %1 tool(s), %2 tool change(s)")
                    .arg(static_cast<qulonglong>(s.distinctTools))
                    .arg(static_cast<qulonglong>(s.toolChanges));
    }
    if (s.needRecalculation > 0) {
        text += QStringLiteral("; %1 need(s) recalculation").arg(static_cast<qulonglong>(s.needRecalculation));
    }
    summary_->setText(text);
}

void ToolpathsPanel::updateActions() {
    const bool idle = !busy_ && doc_ != nullptr;
    const bool hasCurrent = idle && !currentToolpath().isNull();
    for (auto& [op, a] : addActions_) {
        a->setEnabled(idle);
    }
    for (QAction* a : {edit_, duplicate_, up_, down_, recalculate_}) {
        a->setEnabled(hasCurrent);
    }
    delete_->setEnabled(idle && !list_->selectedItems().isEmpty());
    recalculateAll_->setEnabled(idle && list_->topLevelItemCount() > 0);
    cancel_->setEnabled(busy_);
    if (bar_ != nullptr) {
        for (QToolButton* b : bar_->findChildren<QToolButton*>()) {
            if (b->objectName() == QStringLiteral("toolpathAddButton")) {
                b->setEnabled(idle);
            }
        }
    }
    list_->setEnabled(!busy_);
}

void ToolpathsPanel::setBusy(bool busy, const QString& text) {
    const bool changed = busy != busy_;
    busy_ = busy;
    busyLabel_->setText(text);
    busyLabel_->setVisible(busy);
    progress_->setVisible(busy);
    cancelButton_->setVisible(busy && runner_.busy() && !runner_.finished());
    updateActions();
    if (changed && onBusyChanged) {
        onBusyChanged();
    }
}

bool ToolpathsPanel::waitIdle(int timeoutMs) {
    QElapsedTimer timer;
    timer.start();
    while (busy_ || runner_.busy()) {
        if (timer.hasExpired(timeoutMs)) {
            return false;
        }
        QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
        QThread::msleep(1);
    }
    QCoreApplication::processEvents();
    return true;
}

void ToolpathsPanel::linkSelection(ToolpathId id) {
    if (doc_ == nullptr || busy_) {
        return;
    }
    const std::optional<viewmodel::ToolpathDefinition> def = session_.definition(id);
    if (!def.has_value()) {
        return;
    }
    const viewmodel::ResolvedVectors resolved = viewmodel::resolveVectors(*doc_, def->selector);
    doc_->selection().assign(resolved.objects);
    if (onSelectionChanged) {
        onSelectionChanged();
    }
}

void ToolpathsPanel::selectToolpath(ToolpathId id) {
    for (int i = 0; i < list_->topLevelItemCount(); ++i) {
        QTreeWidgetItem* item = list_->topLevelItem(i);
        if (ToolpathId{item->data(0, kIdRole).toULongLong()} == id) {
            list_->clearSelection();
            list_->setCurrentItem(item);
            item->setSelected(true);
            return;
        }
    }
}

void ToolpathsPanel::edited() {
    if (onDocumentEdited) {
        onDocumentEdited();
    } else {
        refresh();
    }
}

// --- Forms ---------------------------------------------------------------------------------------

void ToolpathsPanel::openAddForm(ToolpathOperation operation) {
    if (busy_ || doc_ == nullptr) {
        return;
    }
    openForm(session_.newForm(operation), false, ToolpathId{});
}

void ToolpathsPanel::openEditForm(ToolpathId id) {
    if (busy_ || doc_ == nullptr || id.isNull()) {
        return;
    }
    std::optional<viewmodel::ToolpathForm> form = session_.form(id);
    if (!form.has_value()) {
        setStatus(QStringLiteral("The toolpath no longer exists."), true);
        return;
    }
    openForm(std::move(*form), true, id);
}

void ToolpathsPanel::openForm(viewmodel::ToolpathForm form, bool editing, ToolpathId id) {
    if (dialog_) {
        dialog_->reject();
        delete dialog_.data();
    }
    FormDocumentInfo info;
    for (const document::Layer& layer : doc_->layers()) {
        info.layers.push_back({layer.id, layer.name});
    }
    info.selection = doc_->selection().ids();
    info.countVectors = [this](const viewmodel::VectorSelector& s) { return session_.countVectors(s); };
    info.bitmapProblem = [this](const viewmodel::VectorSelector& s) { return session_.bitmapProblem(s); };
    // Variables the form names are held while it is open (T-0257); closing it releases them.
    info.variablesChanged = [this](const std::vector<std::string>& names) { session_.setOpenFormVariables(names); };
    info.createVariable = [this](const std::string& name, const std::string& value) {
        viewmodel::VariablesModel model(doc_, &session_);
        const viewmodel::VariableEdit r = model.set(name, value);
        if (r.ok) {
            edited();
        }
        return r.ok ? std::string() : r.message;
    };
    auto* dlg = new ToolpathFormDialog(std::move(form), db_, std::move(info), editing, window());
    dialog_ = dlg;
    connect(dlg, &QDialog::finished, this, [this]() { session_.setOpenFormVariables({}); });
    dlg->setAttribute(Qt::WA_DeleteOnClose);
    dlg->setModal(true);
    connect(dlg, &QDialog::accepted, this, [this, dlg, editing, id]() {
        if (busy_ || doc_ == nullptr) {
            return;
        }
        const viewmodel::ToolpathDefinition def = dlg->form().definition();
        pendingExpressions_ = dlg->form().expressions();
        if (editing) {
            const std::optional<ToolpathId> target = session_.targetOf(id);
            if (!target.has_value()) {
                setStatus(QStringLiteral("The toolpath no longer exists."), true);
                return;
            }
            std::vector<viewmodel::ToolpathJob> jobs;
            jobs.push_back(session_.prepare(def, *target));
            start(std::move(jobs), Kind::Edit, "Edit toolpath");
        } else {
            std::vector<viewmodel::ToolpathJob> jobs;
            jobs.push_back(session_.prepare(def));
            start(std::move(jobs), Kind::Add, "Add toolpath");
        }
    });
    dlg->open();
}

// --- Calculation off the UI thread --------------------------------------------------------------

void ToolpathsPanel::start(std::vector<viewmodel::ToolpathJob> jobs, Kind kind, const std::string& label,
                           std::vector<ToolpathMessage> notes) {
    if (busy_ || runner_.busy()) {
        setStatus(QStringLiteral("A calculation is already running."), true);
        return;
    }
    pending_ = Pending{kind, generation_, label, std::move(notes)};
    lastMessages_.clear();
    setBusy(true, QStringLiteral("Calculating..."));
    // The job holds copies only; the completion is posted to the UI thread, where the panel (a
    // context object: the call is dropped if it is destroyed first) takes the result.
    const bool started = runner_.start(std::move(jobs), [this]() {
        QMetaObject::invokeMethod(this, [this]() { finishRun(); }, Qt::QueuedConnection);
    });
    if (!started) {
        setBusy(false, QString());
        setStatus(QStringLiteral("The calculation could not be started."), true);
    }
}

void ToolpathsPanel::finishRun() {
    if (!runner_.busy()) {
        return;
    }
    std::optional<std::vector<viewmodel::ToolpathCalculation>> results = runner_.take();
    const Pending pending = pending_;
    pending_ = Pending{};
    if (!results.has_value() || results->empty()) {
        setBusy(false, QString());
        setStatus(QStringLiteral("The calculation produced no result (out of memory?)."), true);
        refresh();
        return;
    }
    if (pending.generation != generation_ || doc_ == nullptr) {
        // The document was replaced while the job ran: its result belongs to a document that is gone.
        setBusy(false, QString());
        setStatus(QStringLiteral("The document was replaced during the calculation; its result was discarded."), true);
        refresh();
        return;
    }
    const bool allCancelled = !results->empty() &&
                              std::all_of(results->begin(), results->end(),
                                          [](const viewmodel::ToolpathCalculation& c) { return c.cancelled; });
    if (allCancelled) {
        setBusy(false, QString());
        setStatus(QStringLiteral("Calculation cancelled; nothing was changed."), false);
        refresh();
        return;
    }

    // Applying is synchronous (it can take seconds for a huge job): show the busy state first and
    // keep the panel disabled meanwhile.
    setBusy(true, QStringLiteral("Applying the result..."));
    QCoreApplication::processEvents(QEventLoop::ExcludeUserInputEvents);

    lastMessages_.clear();
    if (pending.kind == Kind::RecalculateAll) {
        const viewmodel::BatchOutcome batch = session_.applyAll(*results);
        lastMessages_ = pending.notes;
        for (std::size_t i = 0; i < results->size() && i < batch.items.size(); ++i) {
            const viewmodel::ToolpathCalculation& calc = (*results)[i];
            const viewmodel::ToolpathOutcome& item = batch.items[i];
            if (item.ok) {
                failed_.erase(calc.target);
            } else {
                std::vector<ToolpathMessage> msgs = calc.messages;
                ToolpathMessage why;
                why.severity = MessageSeverity::Error;
                why.text = item.message;
                msgs.insert(msgs.begin(), why);
                failed_[calc.target] = std::move(msgs);
                lastMessages_.push_back(why);
            }
        }
        QString text = qs(batch.message);
        if (!pending.notes.empty()) {
            text += QStringLiteral(" (%1 could not be recalculated)").arg(static_cast<qulonglong>(pending.notes.size()));
        }
        setBusy(false, QString());
        setStatus(text, !batch.ok || !pending.notes.empty());
    } else {
        const viewmodel::ToolpathCalculation& calc = results->front();
        const viewmodel::ToolpathOutcome out = session_.apply(calc);
        if (out.ok) {
            if (pending.kind == Kind::Add || pending.kind == Kind::Edit) {
                session_.bindExpressions(out.ids, pendingExpressions_);  // also forgets them when none
            }
            pendingExpressions_.clear();
            if (!calc.target.isNull()) {
                failed_.erase(calc.target);
            }
            lastMessages_ = {};
            setBusy(false, QString());
            setStatus(qs(out.message), false);
            edited();
            if (!out.ids.empty()) {
                selectToolpath(out.ids.front());
            }
            return;
        }
        // The outcome's message repeats the calculation's error messages; add it only when it says more.
        if (!calc.hasError()) {
            ToolpathMessage why;
            why.severity = MessageSeverity::Error;
            why.text = out.message;
            lastMessages_.push_back(why);
        }
        for (const ToolpathMessage& m : calc.messages) {
            lastMessages_.push_back(m);
        }
        if (!calc.target.isNull()) {
            failed_[calc.target] = lastMessages_;
        }
        setBusy(false, QString());
        setStatus(qs(out.message), true);
    }
    edited();
}

// --- Other list operations (synchronous: they run no core operation) -----------------------------

void ToolpathsPanel::doDuplicate() {
    const ToolpathId id = currentToolpath();
    if (busy_ || doc_ == nullptr || id.isNull()) {
        return;
    }
    const viewmodel::ToolpathOutcome out = session_.duplicate(id);
    setStatus(qs(out.message), !out.ok);
    edited();
    if (out.ok && !out.ids.empty()) {
        selectToolpath(out.ids.back());
    }
}

void ToolpathsPanel::doDelete() {
    if (busy_ || doc_ == nullptr) {
        return;
    }
    std::vector<ToolpathId> ids;
    for (const QTreeWidgetItem* item : list_->selectedItems()) {
        ids.push_back(ToolpathId{item->data(0, kIdRole).toULongLong()});
    }
    if (ids.empty()) {
        return;
    }
    const viewmodel::ToolpathOutcome out = session_.deleteToolpaths(ids);
    setStatus(qs(out.message), !out.ok);
    edited();
}

void ToolpathsPanel::doMove(int delta) {
    const ToolpathId id = currentToolpath();
    if (busy_ || doc_ == nullptr || id.isNull()) {
        return;
    }
    const std::vector<viewmodel::ToolpathListItem> items = session_.list();
    for (std::size_t i = 0; i < items.size(); ++i) {
        if (items[i].id != id) {
            continue;
        }
        const long target = static_cast<long>(i) + delta;
        if (target < 0 || target >= static_cast<long>(items.size())) {
            return;
        }
        const viewmodel::ToolpathOutcome out = session_.reorder(id, static_cast<std::size_t>(target));
        setStatus(qs(out.message), !out.ok);
        edited();
        selectToolpath(id);
        return;
    }
}

void ToolpathsPanel::doRecalculate() {
    const ToolpathId id = currentToolpath();
    if (busy_ || doc_ == nullptr || id.isNull()) {
        return;
    }
    const std::optional<viewmodel::ToolpathListItem> item = session_.item(id);
    const std::optional<viewmodel::ToolpathDefinition> def = session_.definition(id);
    const std::optional<ToolpathId> target = session_.targetOf(id);
    if (!item.has_value() || !def.has_value() || !target.has_value()) {
        setStatus(QStringLiteral("The toolpath no longer exists."), true);
        return;
    }
    if (!item->canRecalculate) {
        setStatus(item->note.empty() ? QStringLiteral("This toolpath cannot be recalculated.") : qs(item->note), true);
        return;
    }
    std::vector<viewmodel::ToolpathJob> jobs;
    jobs.push_back(session_.prepare(*def, *target));
    start(std::move(jobs), Kind::Recalculate, "Recalculate toolpath");
}

void ToolpathsPanel::doRecalculateAll() {
    if (busy_ || doc_ == nullptr) {
        return;
    }
    std::vector<viewmodel::ToolpathOutcome> refused;
    std::vector<viewmodel::ToolpathJob> jobs = session_.prepareRecalculateAll(false, &refused);
    std::vector<ToolpathMessage> notes;
    for (const viewmodel::ToolpathOutcome& o : refused) {
        ToolpathMessage m;
        m.severity = MessageSeverity::Error;
        m.text = o.message;
        notes.push_back(std::move(m));
    }
    if (jobs.empty()) {
        lastMessages_ = notes;
        setStatus(notes.empty() ? QStringLiteral("Nothing to recalculate.")
                                : QStringLiteral("No toolpath could be recalculated."),
                  !notes.empty());
        rebuildDiagnostics();
        return;
    }
    start(std::move(jobs), Kind::RecalculateAll, "Recalculate all", std::move(notes));
}

}  // namespace carvetto::app
