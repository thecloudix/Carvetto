#include "help_viewer.hpp"

#include <QAction>
#include <QCoreApplication>
#include <QDialog>
#include <QDir>
#include <QFile>
#include <QMainWindow>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QRegularExpression>
#include <QTextBrowser>
#include <QUrl>
#include <QVBoxLayout>

#ifndef CARVETTO_USER_DOCS_DIR
#define CARVETTO_USER_DOCS_DIR ""
#endif

namespace carvetto::app {

namespace {

QString dirPath(const std::filesystem::path& p) { return QString::fromStdString(p.generic_string()); }

}  // namespace

std::filesystem::path userDocsDir() {
    const QString app = QCoreApplication::applicationDirPath();
    const QString candidates[] = {app + QCoreApplication::translate("HelpViewer", "/docs/user"),
                                  app + QCoreApplication::translate("HelpViewer", "/../share/carvetto/docs/user"),
                                  QString::fromUtf8(CARVETTO_USER_DOCS_DIR)};
    for (const QString& c : candidates) {
        if (!c.isEmpty() && QFile::exists(c + QCoreApplication::translate("HelpViewer", "/README.md"))) {
            return std::filesystem::path(QDir::cleanPath(c).toStdString());
        }
    }
    return {};
}

bool showUserGuide(QWidget* parent, const std::string& page) {
    const std::filesystem::path dir = userDocsDir();
    if (dir.empty()) {
        QMessageBox::information(parent, QCoreApplication::translate("HelpViewer", "User Guide"),
                                 QCoreApplication::translate(
                                     "HelpViewer", "The user guide is not installed with this copy of the program."));
        return false;
    }
    auto* dialog = new QDialog(parent);
    dialog->setObjectName(QStringLiteral("userGuideWindow"));
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->setWindowTitle(QCoreApplication::translate("HelpViewer", "Carvetto User Guide"));
    dialog->resize(820, 640);
    auto* layout = new QVBoxLayout(dialog);
    auto* browser = new QTextBrowser(dialog);
    browser->setObjectName(QStringLiteral("userGuideBrowser"));
    browser->setOpenLinks(true);
    browser->setSearchPaths({dirPath(dir)});
    browser->setSource(QUrl::fromLocalFile(dirPath(dir) + QLatin1Char('/') + QString::fromStdString(page)));
    layout->addWidget(browser);
    dialog->show();
    return true;
}

void installHelpActions(QMenu* helpMenu, QWidget* parent) {
    QAction* quick = helpMenu->addAction(QCoreApplication::translate("HelpViewer", "&Quick Start"));
    quick->setObjectName(QStringLiteral("helpQuickStartAction"));
    QObject::connect(quick, &QAction::triggered, parent, [parent]() { showUserGuide(parent, "quick-start.md"); });
    QAction* guide = helpMenu->addAction(QCoreApplication::translate("HelpViewer", "&User Guide"));
    guide->setObjectName(QStringLiteral("helpUserGuideAction"));
    QObject::connect(guide, &QAction::triggered, parent, [parent]() { showUserGuide(parent, "README.md"); });
}

std::string helpSelfTest(QWidget* window) {
    if (window->findChild<QAction*>(QStringLiteral("helpQuickStartAction")) == nullptr ||
        window->findChild<QAction*>(QStringLiteral("helpUserGuideAction")) == nullptr) {
        return "help: Help menu entries missing";
    }
    const std::filesystem::path dir = userDocsDir();
    if (dir.empty()) {
        return "help: bundled user guide not found";
    }
    const QDir docs(dirPath(dir));
    const QStringList pages = docs.entryList({QStringLiteral("*.md")}, QDir::Files);
    if (pages.size() < 10 || !pages.contains(QStringLiteral("quick-start.md"))) {
        return "help: guide pages missing";
    }
    static const QRegularExpression link(QStringLiteral("\\]\\(([^)\\s#]+)(?:#[^)]*)?\\)"));
    for (const QString& page : pages) {
        QFile f(docs.filePath(page));
        if (!f.open(QIODevice::ReadOnly)) {
            return "help: cannot read " + page.toStdString();
        }
        const QString text = QString::fromUtf8(f.readAll());
        for (auto it = link.globalMatch(text); it.hasNext();) {
            const QString target = it.next().captured(1);
            if (target.contains(QStringLiteral("://"))) {
                continue;
            }
            if (!QFile::exists(QDir::cleanPath(docs.filePath(target)))) {
                // Pages outside docs/user (ADRs, post guide) exist in the source tree and in the installed tree.
                return "help: broken link " + target.toStdString() + " in " + page.toStdString();
            }
        }
    }
    QTextBrowser probe;
    probe.setSource(QUrl::fromLocalFile(docs.filePath(QStringLiteral("quick-start.md"))));
    if (!probe.toPlainText().contains(QStringLiteral("New Job"))) {
        return "help: quick start did not render";
    }
    return {};
}

}  // namespace carvetto::app
