#include <QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QSettings>
#include <QString>
#include <QtGlobal>
#include <cstdio>
#include <cstring>
#include <string>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#endif

#include "about_dialog.hpp"
#include "localization.hpp"
#include "main_window.hpp"
#include "options_dialog.hpp"
#include "self_test.hpp"

namespace {
// Print a line to stdout. The Windows build is a GUI-subsystem exe, so the CRT stdout may be unusable: write to the
// inherited standard handle (a pipe or file under ctest), or attach to the parent console when there is none.
void printLine(const std::string& line) {
#ifdef _WIN32
    HANDLE h = GetStdHandle(STD_OUTPUT_HANDLE);
    if (h == nullptr || h == INVALID_HANDLE_VALUE) {
        if (AttachConsole(ATTACH_PARENT_PROCESS) != 0) {
            h = GetStdHandle(STD_OUTPUT_HANDLE);
        }
    }
    if (h != nullptr && h != INVALID_HANDLE_VALUE) {
        const std::string text = line + "\r\n";
        DWORD written = 0;
        WriteFile(h, text.data(), static_cast<DWORD>(text.size()), &written, nullptr);
    }
#else
    std::printf("%s\n", line.c_str());
#endif
}
}  // namespace

int main(int argc, char* argv[]) {
    // --version needs no GUI: answer before any QApplication exists (T-0358).
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--version") == 0) {
            printLine(carvetto::app::versionLine().toUtf8().toStdString());
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
