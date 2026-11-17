// Self-test of the Gadgets menu (T-0331): the real MainWindow offscreen, a temporary gadgets folder,
// the sample gadget run from the menu action (vectors and toolpaths in ONE undo step), the sandbox
// limits (loop -> instruction limit, busy loop -> Timeout), a failing gadget and its error dialog,
// the form dialog, Gadget Shortcuts and Install New Gadget.

#include <QAction>
#include <QApplication>
#include <QAbstractButton>
#include <QKeySequence>
#include <QLabel>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QTemporaryDir>
#include <QTimer>
#include <filesystem>
#include <functional>
#include <fstream>
#include <memory>
#include <string>

#include "carvetto/document/document.hpp"
#include "carvetto/viewmodel/file_session.hpp"
#include "gadget_dialogs.hpp"
#include "gadgets_menu.hpp"
#include "main_window.hpp"
#include "toolpaths_panel.hpp"

namespace carvetto::app {

namespace {

#define GD_REQUIRE(cond, text)                         \
    do {                                               \
        if (!(cond)) {                                 \
            return std::string("gadgets: ") + (text);  \
        }                                              \
    } while (false)

namespace fs = std::filesystem;

void writeFile(const fs::path& p, const std::string& text) {
    fs::create_directories(p.parent_path());
    std::ofstream out(p, std::ios::binary);
    out << text;
}

void writeGadget(const fs::path& root, const std::string& folder, const std::string& manifest, const std::string& script) {
    writeFile(root / folder / "manifest.json", manifest);
    writeFile(root / folder / "gadget.lua", script);
}

QAction* actionNamed(GadgetsMenu& gadgets, const QString& text) {
    for (QAction* a : gadgets.gadgetActions()) {
        if (a->text() == text) {
            return a;
        }
    }
    return nullptr;
}

// The sample gadget: a form for the size, then a layer, a box, a hole and two toolpaths.
const char* kSampleScript = R"LUA(
local c = carvetto
local v = c.ui.form({
    title = "Sample box",
    fields = {
        {name = "width", label = "Width (mm)", kind = "number", default = 40, min = 5, max = 200},
        {name = "depth", label = "Pocket depth (mm)", kind = "number", default = 3, min = 0.5, max = 10},
    },
})
if v == nil then return end
local layer = c.layer.create("Sample", "#3366cc")
local box = c.object.create_shape(layer, "rect", {cx = 100, cy = 100, width = v.width, height = v.width * 0.6})
local hole = c.object.create_shape(layer, "circle", {cx = 100, cy = 100, radius = 3})
c.object.create_contour(layer, {{80, 80}, {120, 80}, {120, 120}}, false, {0, 0.5, 0})
c.selection.set({box, hole})
c.doc.set_variable("SampleWidth", v.width)
local pocket = c.toolpath.create("pocket", {box}, {tool = c.tool.find("end-mill-6mm"), cutDepth = v.depth, name = "Sample pocket"})
local drill = c.toolpath.create("drill", {hole}, {tool = "drill-6mm", cutDepth = 5})
c.log("made", #pocket, #drill)
)LUA";

}  // namespace

std::string gadgetsSelfTest(MainWindow& window) {
    window.setQuiet(true);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    document::Document& doc = window.currentDocument();
    GadgetsMenu* gadgets = window.gadgets();
    GD_REQUIRE(gadgets != nullptr && gadgets->menu() != nullptr && gadgets->menu()->title().contains(QStringLiteral("Gadgets")),
               "the Gadgets menu is missing");
    GD_REQUIRE(gadgets->installAction() != nullptr && gadgets->shortcutsAction() != nullptr,
               "Install New Gadget or Gadget Shortcuts is missing");

    QTemporaryDir tmp;
    GD_REQUIRE(tmp.isValid(), "no temporary folder");
    const fs::path root = viewmodel::pathFromUtf8(tmp.path().toStdString()) / "gadgets";
    writeGadget(root, "sample",
                R"({"name": "Sample box", "api_version": 1, "menu": "Samples/Box with pocket", "shortcut": "Ctrl+F7",
                    "description": "A box, a hole and two toolpaths"})",
                kSampleScript);
    writeGadget(root, "loop", R"({"name": "Endless loop", "api_version": 1, "menu": "Hostile/Loop"})",
                "carvetto.object.create_shape(nil, 'circle', {radius = 2})\nwhile true do end\n");
    writeGadget(root, "bomb", R"({"name": "Object bomb", "api_version": 1, "menu": "Hostile/Bomb"})",
                "while true do carvetto.object.create_shape(nil, 'circle', {radius = 2}) end\n");
    writeGadget(root, "broken", R"({"name": "Broken", "api_version": 1, "menu": "Hostile/Broken"})",
                "local a = 1\ncarvetto.object.create_shape(nil, 'circle', {radius = 2})\nlocal b = nil + a\n");
    writeGadget(root, "dialogs", R"({"name": "Dialog spam", "api_version": 1, "menu": "Hostile/&Dialogs"})",
                "carvetto.layer.create('spam')\nwhile true do carvetto.ui.message('again') end\n");
    writeGadget(root, "future", R"({"name": "From the future", "api_version": 9})", "return 1");
    writeFile(root / "junk" / "manifest.json", "not json");

    // 1. The menu is built from the folder: submenus, shortcuts, a disabled gadget.
    gadgets->setFolder(root);
    GD_REQUIRE(gadgets->manager().gadgets().size() == 6 && gadgets->manager().skipped().size() == 1,
               "the gadgets folder was not scanned as expected");
    QAction* sample = actionNamed(*gadgets, QStringLiteral("Box with pocket"));
    GD_REQUIRE(sample != nullptr, "the sample gadget is not in the menu");
    GD_REQUIRE(sample->shortcut() == QKeySequence(QStringLiteral("Ctrl+F7")), "the sample gadget has no Ctrl+F7 shortcut");
    GD_REQUIRE(sample->isEnabled(), "the sample gadget is disabled");
    QAction* future = actionNamed(*gadgets, QStringLiteral("From the future"));
    GD_REQUIRE(future != nullptr && !future->isEnabled() && future->toolTip().contains(QStringLiteral("version 9")),
               "a gadget needing a newer API is not shown disabled with the reason");
    bool sawSubmenu = false;
    for (QAction* a : gadgets->menu()->actions()) {
        if (a->menu() != nullptr && a->text() == QStringLiteral("Samples")) {
            sawSubmenu = true;
        }
    }
    GD_REQUIRE(sawSubmenu, "the menu path made no submenu");

    // 2. The sample gadget, from the menu action: a form, vectors and toolpaths in ONE undo step.
    scripting::GadgetUi ui;
    int formsShown = 0;
    ui.form = [&formsShown](const scripting::FormSpec& spec) -> std::optional<scripting::ScriptValue::Map> {
        ++formsShown;
        scripting::ScriptValue::Map m;
        for (const auto& f : spec.fields) {
            m.set(f.name, f.defaultValue);
        }
        m.set("width", scripting::ScriptValue(50.0));
        return m;
    };
    gadgets->setUiOverride(ui);
    doc.undoStack().clear();
    window.documentChanged();
    sample->trigger();
    const viewmodel::GadgetRunReport& rep = gadgets->lastReport();
    GD_REQUIRE(rep.ok, "the sample gadget failed: " + (rep.error ? rep.error->summary + " " + rep.error->detail : std::string()));
    GD_REQUIRE(formsShown == 1, "the form was not shown once");
    GD_REQUIRE(doc.objectCount() == 3, "the sample gadget did not create three vectors");
    GD_REQUIRE(doc.layers().size() == 2 && doc.layers()[1].name == "Sample", "the sample layer is missing");
    GD_REQUIRE(doc.toolpaths().size() == 2 && doc.toolpaths()[0].hasMoves() && doc.toolpaths()[1].hasMoves(),
               "the sample gadget did not create two calculated toolpaths");
    GD_REQUIRE(doc.toolpaths()[0].name == "Sample pocket", "the pocket has the wrong name");
    GD_REQUIRE(doc.variables().get("SampleWidth") == 50.0, "the document variable was not set");
    GD_REQUIRE(doc.selection().size() == 2, "the selection was not set");
    GD_REQUIRE(doc.undoStack().count() == 1 && doc.undoStack().index() == 1,
               "the gadget run is not exactly one undo step (" + std::to_string(doc.undoStack().count()) + ")");
    GD_REQUIRE(doc.undoStack().undoText() == "Sample box", "the undo step is not named after the gadget");
    GD_REQUIRE(window.toolpaths()->session().list().size() == 2, "the Toolpaths dock does not list the new toolpaths");
    GD_REQUIRE(window.undo(), "undo failed");
    GD_REQUIRE(doc.objectCount() == 0 && doc.toolpaths().empty() && doc.layers().size() == 1 && doc.variables().empty(),
               "one undo did not remove everything the gadget made");
    GD_REQUIRE(window.redo() && doc.objectCount() == 3 && doc.toolpaths().size() == 2, "redo did not restore the gadget's work");
    GD_REQUIRE(doc.checkIntegrity().empty(), "the document is inconsistent after the gadget");

    // 3. Cancelling the form is not an error and changes nothing.
    scripting::GadgetUi cancelUi;
    cancelUi.form = [](const scripting::FormSpec&) { return std::optional<scripting::ScriptValue::Map>(); };
    gadgets->setUiOverride(cancelUi);
    const std::size_t stepsBefore = doc.undoStack().count();
    sample->trigger();
    GD_REQUIRE(gadgets->lastReport().ok && !gadgets->lastReport().run.committed && doc.undoStack().count() == stepsBefore,
               "a cancelled form changed the document");
    gadgets->setUiOverride(ui);

    // 3b. A '&' in a manifest menu string is shown as '&', not taken as a mnemonic marker; the
    // "Abort gadget" button of a dialog ends a gadget that keeps asking, and the run is rolled back.
    QAction* dialogs = actionNamed(*gadgets, QStringLiteral("&&Dialogs"));
    GD_REQUIRE(dialogs != nullptr, "the menu text '&Dialogs' was not escaped to '&&Dialogs'");
    int boxes = 0;
    scripting::GadgetUi spamUi;
    spamUi.message = [&boxes, gadgets](const std::string&) {
        if (++boxes == 3) {
            gadgets->requestAbort();  // what the "Abort gadget" button does
        }
    };
    gadgets->setUiOverride(spamUi);
    const std::size_t layersBeforeSpam = doc.layers().size();
    const std::size_t stepsBeforeSpam = doc.undoStack().count();
    dialogs->trigger();
    GD_REQUIRE(boxes == 3 && gadgets->lastReport().run.script.status == scripting::ScriptStatus::Cancelled,
               "Abort gadget did not end a gadget that keeps opening dialogs");
    GD_REQUIRE(doc.layers().size() == layersBeforeSpam && doc.undoStack().count() == stepsBeforeSpam,
               "an aborted gadget changed the document");
    {
        scripting::FormSpec spec;
        spec.title = "t";
        GadgetFormDialog form(spec, GadgetFormChoices{});
        bool aborted = false;
        form.onAbort = [&aborted] { aborted = true; };
        GD_REQUIRE(form.abortButton() != nullptr, "the form dialog has no Abort gadget button");
        form.abortButton()->click();
        GD_REQUIRE(aborted && form.result() == QDialog::Rejected, "Abort gadget in a form did not abort and close it");
    }
    gadgets->setUiOverride(ui);

    // 4. Sandbox limits are enforced in the app; a failure leaves the document as it was.
    const auto snapshot = [&doc]() {
        return std::to_string(doc.objectCount()) + "/" + std::to_string(doc.layers().size()) + "/" +
               std::to_string(doc.toolpaths().size()) + "/" + std::to_string(doc.undoStack().count());
    };
    const std::string before = snapshot();
    scripting::ScriptLimits limits;
    limits.maxInstructions = 300'000;
    limits.deadlineMillis = 0;
    gadgets->setLimits(limits);
    actionNamed(*gadgets, QStringLiteral("Loop"))->trigger();
    GD_REQUIRE(!gadgets->lastReport().ok && gadgets->lastReport().run.script.status == scripting::ScriptStatus::InstructionLimit,
               "an endless loop was not stopped by the instruction limit");
    GD_REQUIRE(snapshot() == before, "an endless loop changed the document");

    limits.maxInstructions = 4'000'000'000ull;
    limits.deadlineMillis = 60;
    gadgets->setLimits(limits);
    actionNamed(*gadgets, QStringLiteral("Loop"))->trigger();
    GD_REQUIRE(gadgets->lastReport().run.script.status == scripting::ScriptStatus::Timeout,
               "an endless loop was not stopped by the wall-clock deadline");
    GD_REQUIRE(gadgets->lastReport().error && gadgets->lastReport().error->title.find("took too long on this machine") != std::string::npos,
               "a Timeout is not reported as 'took too long on this machine'");
    GD_REQUIRE(snapshot() == before, "a timed-out gadget changed the document");

    limits.maxInstructions = 50'000'000;
    limits.deadlineMillis = 0;
    gadgets->setLimits(limits);
    actionNamed(*gadgets, QStringLiteral("Bomb"))->trigger();
    GD_REQUIRE(!gadgets->lastReport().ok && gadgets->lastReport().error && gadgets->lastReport().error->detail.find("too many objects") != std::string::npos,
               "an object-creating loop was not stopped by the object cap");
    GD_REQUIRE(snapshot() == before, "the object bomb changed the document");
    gadgets->setLimits({});

    // 5. A failing gadget: error report with the line, nothing changed, and the real error dialog.
    actionNamed(*gadgets, QStringLiteral("Broken"))->trigger();
    const viewmodel::GadgetRunReport& bad = gadgets->lastReport();
    GD_REQUIRE(!bad.ok && bad.error && bad.error->detail.find("gadget.lua:3:") != std::string::npos && bad.error->line == 3,
               "the error report does not name line 3");
    GD_REQUIRE(snapshot() == before, "a failing gadget changed the document");
    {
        // The report text comes from the gadget (script output, error messages): it must be literal.
        viewmodel::GadgetErrorReport hostile = *bad.error;
        hostile.summary = "<a href=\"https://example.com\">click</a> <b>bold</b>";
        hostile.detail = "gadget.lua:3: <img src=\"file:///etc/hostname\">";
        QString shownText;
        QString shownDetail;
        bool shownPlain = false;
        bool shownNoLinks = false;
        bool shownDialog = false;
        QTimer::singleShot(100, [&]() {
            if (auto* box = qobject_cast<QMessageBox*>(QApplication::activeModalWidget())) {
                if (box->objectName() == QStringLiteral("gadgetErrorDialog")) {
                    shownDialog = true;
                    shownText = box->text();
                    shownDetail = box->detailedText();
                    shownPlain = box->textFormat() == Qt::PlainText;
                    shownNoLinks = true;
                    for (QLabel* label : box->findChildren<QLabel*>()) {
                        const Qt::TextInteractionFlags flags = label->textInteractionFlags();
                        if (label->objectName() == QStringLiteral("qt_msgbox_label") &&
                            ((flags & Qt::LinksAccessibleByMouse) != 0 || (flags & Qt::LinksAccessibleByKeyboard) != 0)) {
                            shownNoLinks = false;
                        }
                    }
                }
                box->accept();
            }
        });
        showGadgetError(&window, hostile);
        GD_REQUIRE(shownDialog && shownText == QString::fromStdString(hostile.summary) && shownDetail == QString::fromStdString(hostile.detail),
                   "the error dialog does not show the summary and the details literally");
        GD_REQUIRE(shownPlain && shownNoLinks, "the error dialog interprets markup or offers links");
        // and the real report of the broken gadget reads as before
        QString realText;
        QString realDetail;
        QTimer::singleShot(100, [&]() {
            if (auto* box = qobject_cast<QMessageBox*>(QApplication::activeModalWidget())) {
                realText = box->text();
                realDetail = box->detailedText();
                box->accept();
            }
        });
        showGadgetError(&window, *bad.error);
        GD_REQUIRE(realText.contains(QStringLiteral("Your drawing was not changed")) && realDetail.contains(QStringLiteral("gadget.lua:3:")),
                   "the error dialog does not show the summary and the details");
    }

    // 6. The form dialog: real widgets, values in the promised types.
    {
        scripting::FormSpec spec;
        spec.title = "Test";
        const auto field = [](const char* name, scripting::FormFieldKind kind, scripting::ScriptValue dflt) {
            scripting::FormField f;
            f.name = name;
            f.label = name;
            f.kind = kind;
            f.defaultValue = std::move(dflt);
            return f;
        };
        scripting::FormField n = field("n", scripting::FormFieldKind::Number, scripting::ScriptValue(2.5));
        n.min = 1.0;
        n.max = 10.0;
        scripting::FormField c = field("c", scripting::FormFieldKind::Choice, scripting::ScriptValue("b"));
        c.choices = {"a", "b", "c"};
        spec.fields = {n, field("t", scripting::FormFieldKind::Text, scripting::ScriptValue("hi")), c,
                       field("f", scripting::FormFieldKind::Bool, scripting::ScriptValue(false)),
                       field("l", scripting::FormFieldKind::Layer, scripting::ScriptValue(static_cast<std::int64_t>(doc.layers()[0].id.value))),
                       field("k", scripting::FormFieldKind::Tool, scripting::ScriptValue(""))};
        GadgetFormChoices choices;
        for (const auto& l : doc.layers()) {
            choices.layers.emplace_back(l.id.value, l.name);
        }
        choices.tools.emplace_back("end-mill-6mm", "End Mill 6 mm");
        GadgetFormDialog dlg(spec, choices);
        GD_REQUIRE(dlg.setValue("n", scripting::ScriptValue(7.0)) && dlg.setValue("t", scripting::ScriptValue("text")) &&
                       dlg.setValue("c", scripting::ScriptValue("c")) && dlg.setValue("f", scripting::ScriptValue(true)) &&
                       dlg.setValue("l", scripting::ScriptValue(static_cast<std::int64_t>(doc.layers()[1].id.value))) &&
                       dlg.setValue("k", scripting::ScriptValue("end-mill-6mm")),
                   "a form widget refused its value");
        GD_REQUIRE(!dlg.setValue("nope", scripting::ScriptValue(1)) && !dlg.setValue("c", scripting::ScriptValue("zzz")) &&
                       !dlg.setValue("n", scripting::ScriptValue("x")),
                   "a form accepted a wrong name or value");
        const scripting::ScriptValue::Map v = dlg.values();
        GD_REQUIRE(v.find("n")->asNumber() == 7.0 && v.find("t")->asString() == "text" && v.find("c")->asString() == "c" &&
                       v.find("f")->asBool() && v.find("l")->asInt() == static_cast<std::int64_t>(doc.layers()[1].id.value) &&
                       v.find("k")->asString() == "end-mill-6mm",
                   "the form returned the wrong values");
        dlg.setValue("n", scripting::ScriptValue(99.0));
        GD_REQUIRE(dlg.values().find("n")->asNumber() == 10.0, "the number box did not clamp to its maximum");
    }

    // 7. Gadget Shortcuts: a taken key is refused, a free one is applied to the menu action.
    {
        GadgetShortcutsDialog dlg(gadgets->manager(), &window);
        int changes = 0;
        dlg.onChanged = [&]() {
            ++changes;
            gadgets->rebuild();
        };
        std::size_t sampleRow = 0;
        std::size_t loopRow = 0;
        for (std::size_t i = 0; i < gadgets->manager().gadgets().size(); ++i) {
            const auto& g = gadgets->manager().gadgets()[i];
            if (g.folderName == "sample") sampleRow = i;
            if (g.folderName == "loop") loopRow = i;
        }
        GD_REQUIRE(dlg.rowCount() == 6 && dlg.shortcutOf(sampleRow) == "Ctrl+F7", "the shortcuts dialog does not list the gadgets");
        GD_REQUIRE(!dlg.setShortcut(loopRow, "Ctrl+F7").empty() && !dlg.message().isEmpty() && changes == 0,
                   "a shortcut another gadget uses was accepted");
        GD_REQUIRE(dlg.setShortcut(loopRow, "Ctrl+F8").empty() && changes == 1, "a free shortcut was refused");
        QAction* loop = actionNamed(*gadgets, QStringLiteral("Loop"));
        GD_REQUIRE(loop != nullptr && loop->shortcut() == QKeySequence(QStringLiteral("Ctrl+F8")), "the menu action did not get Ctrl+F8");
        GD_REQUIRE(!dlg.setShortcut(loopRow, "F8").empty(), "a plain F-key was accepted");
        GD_REQUIRE(dlg.setShortcut(loopRow, "").empty() && actionNamed(*gadgets, QStringLiteral("Loop"))->shortcut().isEmpty(),
                   "removing a shortcut failed");
    }

    // 8. Install New Gadget: a folder is shown, installed, appears in the menu; a bad folder is refused.
    {
        const fs::path pkg = viewmodel::pathFromUtf8(tmp.path().toStdString()) / "download" / "circle-maker";
        writeGadget(pkg.parent_path(), "circle-maker",
                    R"({"name": "Circle maker", "api_version": 1, "menu": "Samples/Circle", "shortcut": "Ctrl+Shift+F2"})",
                    "carvetto.object.create_shape(nil, 'circle', {radius = 5})");
        const auto installed = gadgets->installGadget(pkg);
        GD_REQUIRE(installed.has_value() && installed->ok, "install failed: " + (installed ? installed->error : std::string("declined")));
        QAction* circle = actionNamed(*gadgets, QStringLiteral("Circle"));
        GD_REQUIRE(circle != nullptr && circle->shortcut() == QKeySequence(QStringLiteral("Ctrl+Shift+F2")),
                   "the installed gadget is not in the menu with its shortcut");
        const std::size_t objectsBefore = doc.objectCount();
        circle->trigger();
        GD_REQUIRE(gadgets->lastReport().ok && doc.objectCount() == objectsBefore + 1, "the installed gadget did not run");
        const auto again = gadgets->installGadget(pkg);  // quiet: confirmations answer yes -> replaces
        GD_REQUIRE(again.has_value() && again->ok, "re-installing (replace) failed");
        const auto junk = gadgets->installGadget(root / "junk");
        GD_REQUIRE(!junk.has_value(), "a folder with no usable manifest was installed");
        GD_REQUIRE(gadgets->manager().gadgets().size() == 7, "the gadget count is wrong after installs");
    }

    // 9. Text from a gadget is shown literally: no clickable links, no markup (message, confirm,
    //    form labels, the install question).
    {
        const QString markup = QStringLiteral("<a href=\"https://example.com\">click me</a> <b>bold</b> <img src=\"file:///etc/hostname\">");
        const std::string markupUtf8 = markup.toStdString();
        scripting::GadgetUi realUi = gadgets->realUi();
        QString seenText;
        bool plain = false;
        bool noLinks = false;
        bool sawBox = false;
        std::function<void(QString)> poll = [&](const QString& clickName) {
            auto* box = qobject_cast<QMessageBox*>(QApplication::activeModalWidget());
            if (box == nullptr) {
                QTimer::singleShot(50, [&poll, clickName]() { poll(clickName); });  // not shown yet (loaded machine)
                return;
            }
            sawBox = true;
            seenText = box->text();
            plain = box->textFormat() == Qt::PlainText;
            noLinks = true;
            for (QLabel* label : box->findChildren<QLabel*>()) {
                const Qt::TextInteractionFlags flags = label->textInteractionFlags();
                if (label->objectName() == QStringLiteral("qt_msgbox_label") &&
                    ((flags & Qt::LinksAccessibleByMouse) != 0 || (flags & Qt::LinksAccessibleByKeyboard) != 0)) {
                    noLinks = false;
                }
            }
            for (QAbstractButton* b : box->buttons()) {
                if (QString(b->text()).remove(QLatin1Char('&')) == clickName) {
                    b->click();
                    return;
                }
            }
            box->accept();
        };
        const auto inspect = [&](const char* clickName) {
            QTimer::singleShot(100, [&poll, name = QString::fromLatin1(clickName)]() { poll(name); });
        };
        inspect("OK");
        realUi.message(markupUtf8);
        GD_REQUIRE(sawBox && plain && noLinks && seenText == markup, "a gadget message box does not show its text literally");
        sawBox = false;
        inspect("Yes");
        const bool yes = realUi.confirm(markupUtf8);
        GD_REQUIRE(sawBox && plain && noLinks && seenText == markup && yes, "a gadget confirm box does not show its text literally");

        scripting::FormSpec spec;
        spec.title = "T";
        scripting::FormField f;
        f.name = "a";
        f.label = markupUtf8;
        f.kind = scripting::FormFieldKind::Text;
        f.defaultValue = scripting::ScriptValue("x");
        spec.fields = {f};
        GadgetFormDialog form(spec, {}, &window);
        bool labelFound = false;
        for (QLabel* label : form.findChildren<QLabel*>()) {
            if (label->text() == markup) {
                labelFound = true;
                GD_REQUIRE(label->textFormat() == Qt::PlainText && !label->openExternalLinks() &&
                               label->textInteractionFlags() == Qt::NoTextInteraction,
                           "a form label shows markup");
            }
        }
        GD_REQUIRE(labelFound, "the form label was altered or missing");
        GD_REQUIRE(plainToolTip(markup).contains(QStringLiteral("&lt;a href")) && !plainToolTip(markup).contains(QStringLiteral("<a ")),
                   "a tooltip would render markup");

        // the install question names the gadget and its description from the manifest
        const fs::path evil = viewmodel::pathFromUtf8(tmp.path().toStdString()) / "download" / "evil";
        writeGadget(evil.parent_path(), "evil",
                    "{\"name\": \"<b>Evil</b>\", \"api_version\": 1, \"menu\": \"Samples/Evil\", \"description\": \"<a href='https://example.com'>go</a>\"}",
                    "return 1");
        sawBox = false;
        inspect("No");
        const auto declined = installGadgetDialog(&window, gadgets->manager(), evil, {});
        GD_REQUIRE(sawBox && plain && noLinks && seenText.contains(QStringLiteral("<b>Evil</b>")) &&
                       seenText.contains(QStringLiteral("<a href='https://example.com'>go</a>")) && !declined.has_value(),
                   "the install question does not show the manifest text literally");

        // a folder that is not a gadget: the warning shows the folder's own text literally too
        sawBox = false;
        inspect("OK");
        const auto notGadget = installGadgetDialog(&window, gadgets->manager(), root / "junk", {});
        GD_REQUIRE(sawBox && !notGadget.has_value() && seenText.startsWith(QStringLiteral("This folder is not a gadget")),
                   "the 'not a gadget' warning was not shown");
        GD_REQUIRE(plain && noLinks, "the 'not a gadget' warning does not show its text literally");

        // the menu tooltip of an installed gadget escapes the manifest description
        const auto evilInstalled = gadgets->installGadget(evil);  // quiet: confirmations answer yes
        GD_REQUIRE(evilInstalled.has_value() && evilInstalled->ok, "the hostile-description gadget was not installed");
        QAction* evilAction = actionNamed(*gadgets, QStringLiteral("Evil"));
        GD_REQUIRE(evilAction != nullptr, "the hostile-description gadget is not in the menu");
        GD_REQUIRE(evilAction->toolTip().contains(QStringLiteral("&lt;a href")) && !evilAction->toolTip().contains(QStringLiteral("<a ")),
                   "the menu tooltip renders the manifest description as markup");
    }

    gadgets->setUiOverride(std::nullopt);
    window.setDocument(std::make_unique<document::Document>(
        document::Job::singleSided(300.0, 300.0, 18.0, document::Units::Millimetres)));
    return {};
}

}  // namespace carvetto::app
