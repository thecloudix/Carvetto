#include <QApplication>
#include <QMainWindow>

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    QMainWindow window;
    window.setWindowTitle("Carvetto");
    window.resize(1280, 800);
    window.show();
    return app.exec();
}
