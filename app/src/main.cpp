#include <QApplication>
#include <QMainWindow>

#include <cstring>

int main(int argc, char* argv[]) {
    // Placeholder until T-0060 implements the real self-test.
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--self-test") == 0) {
            return 0;
        }
    }
    QApplication app(argc, argv);
    QMainWindow window;
    window.setWindowTitle("Carvetto");
    window.resize(1280, 800);
    window.show();
    return app.exec();
}
