#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/scripting/script_host.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// The gadget scripting API (ADR-0102 as built by T-0331, deviations in ADR-0108): binds the global
// table `carvetto` to a `Document` and runs a gadget in ONE undo step. No Qt; everything the host
// application must supply (forms, messages, tools, toolpath calculation) comes in through
// `GadgetServices`, so the binding is testable headless with scripted services.
namespace carvetto::scripting {

/// `carvetto.api_version`; incremented on a breaking change of the API.
inline constexpr int kGadgetApiVersion = 1;

enum class FormFieldKind { Number, Text, Choice, Bool, Layer, Tool };

/// One input of a gadget form (`carvetto.ui.form`).
struct FormField {
    std::string name;   ///< key of the value in the returned table
    std::string label;  ///< shown to the user (defaults to the name)
    FormFieldKind kind{FormFieldKind::Number};
    ScriptValue defaultValue;
    std::vector<std::string> choices;  ///< Choice: the options; the value returned is the chosen text
    std::optional<double> min;         ///< Number
    std::optional<double> max;         ///< Number
};

struct FormSpec {
    std::string title;
    std::vector<FormField> fields;
};

/// What the host application shows. A missing callback is a script error when called ("no user
/// interface available"). The callbacks block the script thread (a modal dialog); the wall-clock
/// deadline does not run while they do.
struct GadgetUi {
    /// Returns name -> value for every field, or nullopt when the user cancelled. Layer fields
    /// return the layer id (integer), Tool fields the tool id (string).
    std::function<std::optional<ScriptValue::Map>(const FormSpec&)> form;
    std::function<void(const std::string&)> message;
    std::function<bool(const std::string&)> confirm;
    /// Asks the user for a folder (UTF-8 path) or returns nullopt when they cancel. The only way a
    /// gadget gets a path to read from (`carvetto.ui.choose_folder`, `carvetto.file.read_dxf_folder`).
    std::function<std::optional<std::string>(const std::string& title)> chooseFolder;
};

/// A tool of the tool database as a gadget sees it.
struct GadgetTool {
    std::string id;
    std::string name;
    double diameterMm{0.0};
    std::string type;
};

/// `carvetto.toolpath.create(kind, objects, params)`: `kind` is one of `toolpathKinds()`; `params`
/// is the script's table (field names of the operation's own form, plus `tool` and `name`).
struct ToolpathRequest {
    std::string kind;
    std::vector<document::ObjectId> objects;
    ScriptValue::Map params;
    /// Raised while the calculation should stop (its time budget ran out, or the user aborted the
    /// gadget). Pass it to the calculation's own cancel hook; a cancelled calculation reports failure.
    /// Null/unset when the calculation cannot be interrupted.
    const std::atomic<bool>* cancel{nullptr};
};

struct ToolpathCreated {
    bool ok{false};
    std::string message;  ///< why it failed (the script gets it as the error text)
    std::vector<document::ToolpathId> ids;
};

/// The kinds `toolpath.create` accepts, in the order listed in the documentation.
const std::vector<std::string>& toolpathKinds();

struct GadgetServices {
    GadgetUi ui;
    /// The tool database (`carvetto.tool.list/find`).
    std::function<std::vector<GadgetTool>()> tools;
    /// Calculates a toolpath for the request and applies it to the document THROUGH THE UNDO STACK
    /// (it is called inside the gadget's macro, so it joins the gadget's one undo step). Blocking.
    std::function<ToolpathCreated(const ToolpathRequest&)> createToolpath;
    /// The cutting snapshot of a tool for `toolpath.create_custom`; nullopt for an unknown id.
    std::function<std::optional<toolpath::ToolSnapshot>(const std::string& toolId)> toolSnapshot;
};

/// Per-run caps of the gadget API, besides the script limits of the host (ADR-0101).
struct GadgetLimits {
    /// Objects one run may create (a runaway loop creating contours is stopped well before the
    /// memory limit of the process).
    std::size_t maxCreatedObjects = 100'000;
    /// Toolpath calculations (create + create_custom) one run may start.
    std::size_t maxToolpaths = 64;
    /// Lines and bytes `carvetto.log` keeps.
    std::size_t maxLogLines = 2000;
    std::size_t maxLogBytes = 256u * 1024u;
    /// Moves one custom toolpath may hold.
    std::size_t maxCustomMoves = 500'000;
    /// `ui.message`, `ui.confirm` and `ui.form` calls one run may make in total (a gadget cannot bury
    /// the user under modal boxes: the deadline does not run while a box is open).
    std::size_t maxDialogs = 100;
    /// Layers one run may create (each costs time proportional to the layers already there).
    std::size_t maxCreatedLayers = 1000;
    /// Executed `doc.set_variable` calls (that change a value) one run may make.
    std::size_t maxVariableEdits = 10'000;
    /// DXF files `carvetto.file.read_dxf_folder` reads per call, and the largest one it opens (bytes).
    std::size_t maxDxfFiles = 200;
    std::size_t maxDxfFileBytes = 16u * 1024u * 1024u;
    /// Contours one `read_dxf_folder` call hands to the script in total.
    std::size_t maxDxfContours = 50'000;
    /// Combined time one run may spend waiting in DIALOGS (user time: `ui.message/confirm/form`), in
    /// nanoseconds (default 30 minutes). The deadline does not run during dialogs, so this is what
    /// keeps a gadget from sitting in them forever. Checked before and after every dialog: the run
    /// that crosses it fails, rolled back, and a script cannot pcall its way out (a dialog itself is
    /// not interrupted; the user ends it with Abort, see `GadgetRunOptions::cancel`).
    std::uint64_t maxPausedNanos = 30ull * 60ull * 1'000'000'000ull;
    /// Combined time one run may spend CALCULATING toolpaths (`toolpath.create`), in nanoseconds
    /// (default 3 minutes). Much smaller than the dialog budget: no user is deciding anything, and
    /// ~60 slow calculations must not freeze the application for half an hour. A calculation that
    /// runs past what is left of it is cancelled through `ToolpathRequest::cancel` (the watchdog polls
    /// every 20 ms; the calculations poll the token at their own pace), and the run fails, rolled
    /// back, not swallowable by pcall. The user's Abort cuts a calculation short the same way.
    std::uint64_t maxCalculationNanos = 3ull * 60ull * 1'000'000'000ull;
};

struct GadgetRunOptions {
    /// The undo step's text.
    std::string undoText{"Run gadget"};
    ScriptLimits limits;
    GadgetLimits gadgetLimits;
    /// Polled by the script host, and after every dialog or toolpath calculation: a run in which this
    /// flag was set ends as `Cancelled` and is rolled back, even if the script was about to finish.
    /// The app sets it from the "Abort gadget" button of its dialogs.
    const std::atomic<bool>* cancel{nullptr};
};

struct GadgetRunResult {
    /// Status, message with line, traceback, print output and budgets of the script run.
    ScriptResult script;
    /// True when the run changed the document (it is then ONE undo step). A failed run is rolled
    /// back completely and is never committed.
    bool committed{false};
    /// Edits the run made (before a rollback, if any).
    std::size_t edits{0};
    std::vector<std::string> log;  ///< `carvetto.log` lines
    std::vector<document::ObjectId> createdObjects;
    std::vector<document::ToolpathId> createdToolpaths;
    bool ok() const { return script.ok(); }
};

/// Runs `source` against `doc`. All edits go through commands collected in one macro: success
/// records it as ONE undo step (`options.undoText`), any failure (script error, limit, timeout,
/// cancel, a refused edit) reverts every edit and leaves the history untouched. Selection changes
/// are reverted on failure too. The wall-clock deadline of `options.limits` does not count time
/// spent in the host callbacks (forms, messages, toolpath calculation).
GadgetRunResult runGadget(document::Document& doc, const GadgetServices& services, std::string_view source,
                          std::string_view chunkName, const GadgetRunOptions& options = {});

}  // namespace carvetto::scripting
