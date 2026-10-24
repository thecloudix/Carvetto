#include <QApplication>
#include <QByteArray>
#include <QCoreApplication>
#include <QSettings>
#include <QString>
#include <QtGlobal>
#include <cstring>

#include "main_window.hpp"
#include "self_test.hpp"

int main(int argc, char* argv[]) {
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

    carvetto::app::MainWindow window;
    window.setPersistLayout(true);
    {
        QSettings settings;
        window.restoreLayout(settings);
    }
    window.show();
    return app.exec();
}
