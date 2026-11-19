#include <QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QSettings>
#include <QString>
#include <QtGlobal>
#include <cstdio>
#include <cstring>

#include "about_dialog.hpp"
#include "localization.hpp"
#include "main_window.hpp"
#include "options_dialog.hpp"
#include "self_test.hpp"

int main(int argc, char* argv[]) {
    // --version needs no GUI: answer before any QApplication exists (T-0358).
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--version") == 0) {
            std::printf("%s\n", carvetto::app::versionLine().toUtf8().constData());
            return 0;
        }
    }
    bool selfTest = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--self-test") == 0) {
            selfTest = true;
        }
    }
    // The self-test is headless: default to the offscreen platform unless one was chosen
    // (QT_QPA_PLATFORM or -platform on the command line both still win).
    if (selfTest && qEnvironmentVariableIsEmpty("QT_QPA_PLATFORM")) {
        qputenv("QT_QPA_PLATFORM", QByteArray("offscreen"));
    }

    QApplication app(argc, argv);
    QCoreApplication::setOrganizationName(QStringLiteral("Carvetto"));
    QCoreApplication::setApplicationName(QStringLiteral("Carvetto"));

    if (selfTest) {
        return carvetto::app::runSelfTest();
    }

    {
        // The stored language applies before any window exists (the self-test keeps the source language).
        carvetto::app::QSettingsPreferencesStore store;
        const carvetto::viewmodel::Preferences stored = carvetto::viewmodel::deserialize(store.load());
        carvetto::app::l10n::installLanguage(QString::fromStdString(stored.language));
    }
    carvetto::app::MainWindow window;
    window.setPersistLayout(true);
    {
        QSettings settings;
        window.restoreLayout(settings);
    }
    window.enableRecovery();
    window.show();
    window.offerRecovery();
    return app.exec();
}
