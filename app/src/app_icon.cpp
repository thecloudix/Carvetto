#include "app_icon.hpp"

#include <QPixmap>
#include <QSize>
#include <QString>

namespace carvetto::app {

namespace {
constexpr int kSizes[] = {16, 24, 32, 48, 64, 128, 256};
}

QIcon applicationIcon() {
    QIcon icon;
    for (int s : kSizes) {
        icon.addFile(QStringLiteral(":/icons/carvetto-%1.png").arg(s), QSize(s, s));
    }
    return icon;
}

std::string appIconSelfTest() {
    const QIcon icon = applicationIcon();
    if (icon.isNull()) {
        return "app icon: null (resource missing)";
    }
    for (int s = 16; s <= 256; s += 8) {
        const QPixmap pm = icon.pixmap(QSize(s, s));
        if (pm.isNull() || pm.width() <= 0 || pm.height() <= 0) {
            return "app icon: null pixmap at " + std::to_string(s) + " px";
        }
    }
    for (int s : kSizes) {
        const QPixmap pm = QPixmap(QStringLiteral(":/icons/carvetto-%1.png").arg(s));
        if (pm.width() != s || pm.height() != s) {
            return "app icon: resource image is not " + std::to_string(s) + " px";
        }
    }
    return {};
}

}  // namespace carvetto::app
