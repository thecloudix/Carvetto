#include "about_dialog.hpp"

#include "app_icon.hpp"

#include <QCoreApplication>
#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QPixmap>
#include <QPlainTextEdit>
#include <QRegularExpression>
#include <QVBoxLayout>
#include <QtGlobal>

#ifndef CARVETTO_VERSION_STRING
#define CARVETTO_VERSION_STRING "0.0.0"
#endif
#ifndef CARVETTO_BUILD_TYPE
#define CARVETTO_BUILD_TYPE "unknown"
#endif

namespace carvetto::app {

namespace {

struct Dependency {
    const char* name;
    const char* version;
    const char* licence;
};

// Mirrors THIRD_PARTY_NOTICES.md (the components that are distributed). Keep versions in sync with the
// FetchContent pins in the CMake files and with THIRD_PARTY_NOTICES.md when a dependency is bumped.
constexpr Dependency kDependencies[] = {
    {"Qt", "6.x", "LGPL-3.0"},         {"Clipper2", "1.4.0", "BSL-1.0"}, {"nlohmann/json", "3.11.3", "MIT"},
    {"miniz", "3.0.2", "MIT"},         {"FreeType", "2.14.3", "FTL"},    {"HarfBuzz", "15.1.0", "MIT"},
    {"Lua", "5.4.8", "MIT"},           {"DejaVu Serif", "2.37", "Bitstream Vera / public domain"},
};

QString compilerName() {
#if defined(_MSC_VER)
    return QStringLiteral("MSVC %1").arg(_MSC_VER);
#elif defined(__clang__)
    return QStringLiteral("Clang %1.%2.%3").arg(__clang_major__).arg(__clang_minor__).arg(__clang_patchlevel__);
#elif defined(__GNUC__)
    return QStringLiteral("GCC %1.%2.%3").arg(__GNUC__).arg(__GNUC_MINOR__).arg(__GNUC_PATCHLEVEL__);
#else
    return QStringLiteral("unknown compiler");
#endif
}

}  // namespace

QString appVersion() { return QStringLiteral(CARVETTO_VERSION_STRING); }

QString versionLine() { return QStringLiteral("Carvetto ") + appVersion(); }

QString buildInfo() {
    return QCoreApplication::translate("AboutDialog", "Compiler: %1\nBuild type: %2\nQt: built with %3, running on %4")
        .arg(compilerName(), QStringLiteral(CARVETTO_BUILD_TYPE), QStringLiteral(QT_VERSION_STR),
             QString::fromLatin1(qVersion()));
}

QString aboutText() {
    QString text = QCoreApplication::translate("AboutDialog", "Carvetto - open-source CNC design and toolpath software.") +
                   QStringLiteral("\n\n") + QCoreApplication::translate("AboutDialog", "Version %1").arg(appVersion()) +
                   QStringLiteral("\n") +
                   QCoreApplication::translate("AboutDialog",
                                               "Licence: GNU General Public License, version 3 or later (GPL-3.0-or-later). "
                                               "This program comes with no warranty.") +
                   QStringLiteral("\n\n") + buildInfo() + QStringLiteral("\n\n") +
                   QCoreApplication::translate("AboutDialog", "Bundled dependencies:");
    for (const Dependency& d : kDependencies) {
        text += QStringLiteral("\n  %1 %2 (%3)")
                    .arg(QString::fromLatin1(d.name), QString::fromLatin1(d.version), QString::fromLatin1(d.licence));
    }
    return text;
}

QDialog* createAboutDialog(QWidget* parent) {
    auto* dialog = new QDialog(parent);
    dialog->setObjectName(QStringLiteral("aboutDialog"));
    dialog->setWindowTitle(QCoreApplication::translate("AboutDialog", "About Carvetto"));
    auto* layout = new QVBoxLayout(dialog);
    auto* title = new QLabel(versionLine(), dialog);
    title->setObjectName(QStringLiteral("aboutVersionLabel"));
    QFont f = title->font();
    f.setBold(true);
    f.setPointSizeF(f.pointSizeF() * 1.4);
    title->setFont(f);
    auto* head = new QHBoxLayout();
    auto* logo = new QLabel(dialog);
    logo->setObjectName(QStringLiteral("aboutLogo"));
    logo->setPixmap(applicationIcon().pixmap(QSize(64, 64)));
    logo->setAccessibleName(QCoreApplication::translate("AboutDialog", "Carvetto logo"));
    head->addWidget(logo);
    head->addWidget(title, 1);
    layout->addLayout(head);
    auto* body = new QPlainTextEdit(aboutText(), dialog);
    body->setObjectName(QStringLiteral("aboutText"));
    body->setReadOnly(true);
    body->setMinimumSize(480, 300);
    layout->addWidget(body);
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close, dialog);
    QObject::connect(buttons, &QDialogButtonBox::rejected, dialog, &QDialog::reject);
    layout->addWidget(buttons);
    return dialog;
}

void showAboutDialog(QWidget* parent) {
    QDialog* dialog = createAboutDialog(parent);
    dialog->setAttribute(Qt::WA_DeleteOnClose);
    dialog->exec();
}

std::string aboutSelfTest(QWidget* window) {
    const QString version = appVersion();
    static const QRegularExpression semver(QStringLiteral("^\\d+\\.\\d+\\.\\d+$"));
    if (!semver.match(version).hasMatch()) {
        return "about: version is not major.minor.patch: " + version.toStdString();
    }
    if (versionLine() != QStringLiteral("Carvetto ") + version) {
        return "about: --version line wrong";
    }
    QDialog* dialog = createAboutDialog(window);
    const auto* label = dialog->findChild<QLabel*>(QStringLiteral("aboutVersionLabel"));
    const auto* body = dialog->findChild<QPlainTextEdit*>(QStringLiteral("aboutText"));
    const auto* logo = dialog->findChild<QLabel*>(QStringLiteral("aboutLogo"));
    std::string why;
    if (logo == nullptr || logo->pixmap().isNull()) {
        why = "about: logo missing";
    } else if (label == nullptr || body == nullptr) {
        why = "about: dialog widgets missing";
    } else if (!label->text().contains(version)) {
        why = "about: dialog does not show the version";
    } else {
        const QString text = body->toPlainText();
        for (const char* needle : {"GPL-3.0-or-later", "Qt:", "Compiler:", "Clipper2", "Lua", "HarfBuzz"}) {
            if (!text.contains(QString::fromLatin1(needle))) {
                why = std::string("about: text lacks ") + needle;
                break;
            }
        }
    }
    delete dialog;
    return why;
}

}  // namespace carvetto::app
