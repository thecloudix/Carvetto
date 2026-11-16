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
};

struct GadgetRunOptions {
    /// The undo step's text.
    std::string undoText{"Run gadget"};
    ScriptLimits limits;
    GadgetLimits gadgetLimits;
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
