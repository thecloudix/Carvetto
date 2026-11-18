#include "localization.hpp"

#include <QCoreApplication>
#include <QDir>
#include <QTranslator>
#include <memory>

namespace carvetto::app::l10n {

namespace {

class PseudoTranslator final : public QTranslator {
public:
    QString translate(const char* /*context*/, const char* sourceText, const char* /*disambiguation*/,
                      int /*n*/) const override {
        if (sourceText == nullptr || sourceText[0] == '\0') {
            return QString();
        }
        return pseudoLocalize(QString::fromUtf8(sourceText));
    }
    bool isEmpty() const override { return false; }
};

std::unique_ptr<QTranslator> g_translator;
QString g_active = QStringLiteral("en");

QChar accented(QChar c) {
    static const char16_t from[] = u"aeiouyAEIOUYcnsCNSz";
    static const char16_t to[] = u"åéîøüÿÅÉÎØÜŸçñšÇÑŠž";
    for (int i = 0; from[i] != 0; ++i) {
        if (c.unicode() == from[i]) {
            return QChar(to[i]);
        }
    }
    return c;
}

}  // namespace

QStringList supportedLanguages() {
    return {QStringLiteral("system"), QStringLiteral("en"), QStringLiteral("de"), QStringLiteral("pseudo")};
}

QString resolveLanguage(const QString& preference, const QLocale& system) {
    QString code = preference;
    if (code == QLatin1String("system")) {
        code = system.name().section(QLatin1Char('_'), 0, 0);
    }
    if (code == QLatin1String("de") || code == QLatin1String("pseudo")) {
        return code;
    }
    return QStringLiteral("en");
}

QString pseudoLocalize(const QString& source) {
    QString out;
    out.reserve(source.size() + 2);
    out += QLatin1Char('[');
    bool inTag = false;
    for (qsizetype i = 0; i < source.size(); ++i) {
        const QChar c = source.at(i);
        if (inTag) {
            out += c;
            inTag = c != QLatin1Char('>');
        } else if (c == QLatin1Char('<')) {
            inTag = true;
            out += c;
        } else if (c == QLatin1Char('%') && i + 1 < source.size()) {
            out += c;  // %1, %n, %L1: keep the placeholder
            ++i;
            if (source.at(i) == QLatin1Char('L') && i + 1 < source.size()) {
                out += source.at(i++);
            }
            out += source.at(i);
        } else if (c == QLatin1Char('&') && i + 1 < source.size()) {
            out += c;  // mnemonic marker (or "&&"): keep it and the letter it marks
            ++i;
            out += source.at(i);
        } else {
            out += accented(c);
        }
    }
    out += QLatin1Char(']');
    return out;
}

QStringList translationSearchDirs() {
    QStringList dirs;
    const QString app = QCoreApplication::applicationDirPath();
    dirs << QDir::cleanPath(app + QStringLiteral("/../share/carvetto/translations"))
         << QDir::cleanPath(app + QStringLiteral("/translations"));
#ifdef CARVETTO_TRANSLATIONS_DIR
    dirs << QString::fromUtf8(CARVETTO_TRANSLATIONS_DIR);
#endif
    return dirs;
}

QString installLanguage(const QString& preference) {
    if (g_translator) {
        QCoreApplication::removeTranslator(g_translator.get());
        g_translator.reset();
    }
    g_active = QStringLiteral("en");
    const QString lang = resolveLanguage(preference, QLocale::system());
    if (lang == QLatin1String("pseudo")) {
        g_translator = std::make_unique<PseudoTranslator>();
    } else if (lang != QLatin1String("en")) {
        auto real = std::make_unique<QTranslator>();
        for (const QString& dir : translationSearchDirs()) {
            if (real->load(QStringLiteral("carvetto_") + lang, dir)) {
                g_translator = std::move(real);
                break;
            }
        }
    }
    if (g_translator) {
        QCoreApplication::installTranslator(g_translator.get());
        g_active = lang;
    }
    return g_active;
}

QString activeLanguage() {
    return g_active;
}

}  // namespace carvetto::app::l10n
