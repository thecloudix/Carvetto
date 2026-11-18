#pragma once

// Text produced by the Qt-free view model (English, sometimes with numbers spliced in) is shown through
// this lookup so a catalog entry for the whole sentence, or the pseudo-locale, applies to it (T-0351).
// The lookup key is the English text, which is also the fallback. Sentences the view model builds from
// parts are not extracted by lupdate; catalogs may still add them by hand under the "ViewModel" context.

#include <QCoreApplication>
#include <QString>

#include <string>

namespace carvetto::app {

inline QString vmText(const char* english) {
    // The empty string stays empty (a translator would not see it, the pseudo-locale would bracket it).
    return english == nullptr || *english == '\0' ? QString() : QCoreApplication::translate("ViewModel", english);
}

inline QString vmText(const std::string& english) {
    return vmText(english.c_str());
}

}  // namespace carvetto::app
