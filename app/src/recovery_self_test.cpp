// Self-test of autosave and crash recovery (T-0357).

#include "recovery_self_test.hpp"

#include <QDir>
#include <QTemporaryDir>
#include <memory>

#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/viewmodel/preferences.hpp"
#include "main_window.hpp"

namespace carvetto::app {

namespace {

#define REC_REQUIRE(cond, text)                      \
    do {                                             \
        if (!(cond)) {                               \
            return std::string("recovery: ") + (text); \
        }                                            \
    } while (false)

}  // namespace

std::string recoverySelfTest(MainWindow& window) {
    window.setQuiet(true);
    using viewmodel::Preferences;
    QTemporaryDir dir;
    REC_REQUIRE(dir.isValid(), "no temporary folder");
    window.enableRecovery(viewmodel::pathFromUtf8(QDir(dir.path()).filePath(QStringLiteral("recovery")).toStdString()));

    // The timer follows the Options preferences.
    Preferences p;
    p.autosaveEnabled = true;
    p.autosaveMinutes = 7;
    REC_REQUIRE(window.setPreferences(p).empty() && window.autosaveIntervalMs() == 7 * 60 * 1000,
                "the autosave interval did not follow the preferences");

    // Unsaved work is autosaved; the session "dies" without a clean exit; the next start recovers it.
    window.setDocument(std::make_unique<document::Document>());
    document::Document& doc = window.session().document();
    const geometry::Vec2 c{40.0, 40.0};
    REC_REQUIRE(doc.execute(std::make_unique<document::AddObjectCommand>(document::Object::vector(
                    geometry::Contour::circle(c, 6.0), doc.activeLayer(), doc.activeSheet(),
                    document::CircleParams{c, 6.0}))),
                "adding a circle failed");
    window.autosaveNow();
    REC_REQUIRE(window.hasRecoverable(), "autosave wrote no recovery file");
    window.setDocument(std::make_unique<document::Document>());  // the new session starts empty
    REC_REQUIRE(window.session().document().objects().empty(), "the new session is not empty");
    REC_REQUIRE(window.hasRecoverable(), "the snapshot vanished before the next start");
    REC_REQUIRE(window.recoverPending(), "recovery failed");
    REC_REQUIRE(window.session().document().objects().size() == 1 && window.session().isDirty(),
                "the recovered session lost the circle or is not marked unsaved");
    REC_REQUIRE(!window.hasRecoverable(), "the snapshot was not consumed");

    // A clean document leaves no snapshot; switching autosave off removes a stale one.
    window.autosaveNow();
    REC_REQUIRE(window.hasRecoverable(), "dirty recovered work was not autosaved again");
    p.autosaveEnabled = false;
    REC_REQUIRE(window.setPreferences(p).empty() && window.autosaveIntervalMs() == 0 && !window.hasRecoverable(),
                "switching autosave off did not stop the timer and drop the snapshot");

    window.setPreferences(Preferences{});
    window.setDocument(std::make_unique<document::Document>());
    window.disableRecovery();
    return {};
}

}  // namespace carvetto::app
