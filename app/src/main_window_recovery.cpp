// Autosave and crash recovery glue for MainWindow (T-0357, ADR-0128). The logic is viewmodel::RecoveryStore.
#include <QCoreApplication>
#include <QDir>
#include <QMessageBox>
#include <QStandardPaths>
#include <QTimer>

#include "main_window.hpp"

namespace carvetto::app {

namespace {

QString qs(const std::string& s) { return QString::fromStdString(s); }

}  // namespace

void MainWindow::enableRecovery(const std::filesystem::path& dir) {
    std::filesystem::path folder = dir;
    if (folder.empty()) {
        const QString base = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
        if (base.isEmpty()) {
            return;
        }
        folder = viewmodel::pathFromUtf8(QDir(base).filePath(QStringLiteral("recovery")).toStdString());
    }
    recovery_ = std::make_unique<viewmodel::RecoveryStore>(folder);
    updateAutosaveTimer();
}

void MainWindow::disableRecovery() {
    if (autosaveTimer_ != nullptr) {
        autosaveTimer_->stop();
    }
    recovery_.reset();
}

void MainWindow::updateAutosaveTimer() {
    if (!recovery_) {
        return;
    }
    if (autosaveTimer_ == nullptr) {
        autosaveTimer_ = new QTimer(this);
        autosaveTimer_->setTimerType(Qt::CoarseTimer);
        connect(autosaveTimer_, &QTimer::timeout, this, &MainWindow::autosaveNow);
    }
    const viewmodel::Preferences& p = prefs_.current();
    if (p.autosaveEnabled && p.autosaveMinutes > 0) {
        autosaveTimer_->start(p.autosaveMinutes * 60 * 1000);
    } else {
        autosaveTimer_->stop();
        recovery_->discard();  // switched off: no stale snapshot may offer old work later
    }
}

int MainWindow::autosaveIntervalMs() const {
    return (autosaveTimer_ != nullptr && autosaveTimer_->isActive()) ? autosaveTimer_->interval() : 0;
}

void MainWindow::autosaveNow() {
    if (recovery_ && prefs_.current().autosaveEnabled) {
        recovery_->tick(session_);  // a failed snapshot is silent: autosave must never interrupt the user
    }
}

bool MainWindow::hasRecoverable() const { return recovery_ && recovery_->pending().has_value(); }

bool MainWindow::recoverPending() {
    if (!recovery_) {
        return false;
    }
    const bool ok = replaceDocument([this]() { return recovery_->restore(session_); });
    if (!ok && !quiet_) {
        QMessageBox::critical(this, QCoreApplication::translate("MainWindow", "Recovery failed"),
                              qs(recovery_->lastError()));
    }
    documentChanged();
    return ok;
}

void MainWindow::offerRecovery() {
    if (!recovery_) {
        return;
    }
    const auto pending = recovery_->pending();
    if (!pending) {
        return;
    }
    if (quiet_) {
        recovery_->discard();
        return;
    }
    QMessageBox box(QMessageBox::Question, QCoreApplication::translate("MainWindow", "Recover unsaved work"),
                    QCoreApplication::translate("MainWindow",
                                                "Carvetto did not close normally last time. Recover the unsaved "
                                                "work on %1?")
                        .arg(qs(pending->displayName)),
                    QMessageBox::Yes | QMessageBox::No, this);
    box.setDefaultButton(QMessageBox::Yes);
    if (box.exec() == QMessageBox::Yes) {
        recoverPending();
    } else {
        recovery_->discard();
    }
}

}  // namespace carvetto::app
