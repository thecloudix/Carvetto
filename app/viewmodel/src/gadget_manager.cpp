#include "carvetto/viewmodel/gadget_manager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <map>

#include "carvetto/toolpath/operation.hpp"
#include "carvetto/viewmodel/toolpath_form.hpp"

namespace carvetto::viewmodel {

namespace {

using scripting::GadgetInfo;
using scripting::ScriptStatus;
using scripting::ScriptValue;

std::string lowerCase(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

/// Letters and digits only, lower case: "Climb", "climb" and "climb_" compare equal, so a script
/// may write option names in snake_case.
std::string squash(const std::string& s) {
    std::string out;
    for (const char c : s) {
        if (std::isalnum(static_cast<unsigned char>(c)) != 0) {
            out += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        }
    }
    return out;
}

/// True when `value` names `option`: the whole label, or one of its parts split at '/' and '('
/// ("Outside / right" is "outside", "right" or "outsideright"; "Offset (constant scallop)" is "offset").
bool optionMatches(const std::string& option, const std::string& value) {
    const std::string want = squash(value);
    if (want.empty()) {
        return false;
    }
    if (squash(option) == want) {
        return true;
    }
    std::string part;
    for (std::size_t i = 0; i <= option.size(); ++i) {
        if (i == option.size() || option[i] == '/' || option[i] == '(') {
            if (squash(part) == want) {
                return true;
            }
            part.clear();
        } else {
            part += option[i];
        }
    }
    return false;
}

std::string numberText(double v) {
    char buf[40];
    std::snprintf(buf, sizeof buf, "%.17g", v);
    return buf;
}

std::optional<document::ToolpathOperation> operationOf(const std::string& kind) {
    if (kind == "profile") return document::ToolpathOperation::Profile;
    if (kind == "pocket") return document::ToolpathOperation::Pocket;
    if (kind == "drill") return document::ToolpathOperation::Drill;
    if (kind == "engrave") return document::ToolpathOperation::Engrave;
    return std::nullopt;
}

std::string knownParams(const ToolpathForm& form) {
    std::string out = "name, tool";
    for (const std::string& id : form.allFieldIds()) {
        out += ", " + id;
    }
    for (const std::string& id : form.allChoiceIds()) {
        out += ", " + id;
    }
    return out;
}

scripting::ToolpathCreated createToolpath(ToolpathSession& session,
                                          const scripting::ToolpathRequest& req) {
    scripting::ToolpathCreated out;
    const auto op = operationOf(req.kind);
    if (!op) {
        out.message = "unknown toolpath kind \"" + req.kind + "\"";
        return out;
    }
    // Gadget parameters are millimetres and degrees whatever the job's display units (ADR-0102).
    ToolpathForm form(*op, document::Units::Millimetres);
    const std::vector<std::string> fieldIds = form.allFieldIds();
    const std::vector<std::string> choiceIds = form.allChoiceIds();
    const auto isIn = [](const std::vector<std::string>& v, const std::string& s) {
        return std::find(v.begin(), v.end(), s) != v.end();
    };
    // The tool first: the form's defaults and limits depend on it.
    const ScriptValue* tool = req.params.find("tool");
    if (tool == nullptr || !tool->isString()) {
        out.message = "the toolpath needs a tool: pass tool = carvetto.tool.find(\"name\")";
        return out;
    }
    {
        const FieldResult r = form.setTool(session.toolDatabase(), document::ToolSource{tool->asString(), {}, {}});
        if (!r.ok) {
            out.message = "tool: " + r.message;
            return out;
        }
    }
    for (const auto& kv : req.params.items) {
        const std::string& key = kv.first;
        const ScriptValue& v = kv.second;
        FieldResult r;
        if (key == "tool") {
            continue;
        }
        if (key == "name") {
            if (!v.isString()) {
                out.message = "name must be a string";
                return out;
            }
            r = form.setName(v.asString());
        } else if (isIn(fieldIds, key)) {
            if (!v.isNumber() || !std::isfinite(v.asNumber())) {
                out.message = "parameter '" + key + "' must be a finite number";
                return out;
            }
            r = form.setField(key, numberText(v.asNumber()));
        } else if (isIn(choiceIds, key)) {
            const auto choices = form.choices();
            const auto it = std::find_if(choices.begin(), choices.end(),
                                         [&](const ToolpathChoice& c) { return c.id == key; });
            int option = -1;
            if (it != choices.end()) {
                if (v.isBool() && it->isBool) {
                    option = v.asBool() ? 1 : 0;
                } else if (v.isString()) {
                    for (std::size_t i = 0; i < it->options.size(); ++i) {
                        if (optionMatches(it->options[i], v.asString())) {
                            option = static_cast<int>(i);
                        }
                    }
                }
            }
            if (option < 0) {
                std::string opts;
                if (it != choices.end()) {
                    for (const std::string& o : it->options) {
                        opts += (opts.empty() ? "" : ", ") + lowerCase(o);
                    }
                }
                out.message = "parameter '" + key + "' must be one of: " + opts;
                return out;
            }
            r = form.setChoice(key, option);
        } else {
            out.message = "unknown parameter '" + key + "' (known: " + knownParams(form) + ")";
            return out;
        }
        if (!r.ok) {
            out.message = "parameter '" + key + "': " + r.message;
            return out;
        }
    }
    form.setSelector(VectorSelector::fromObjects(req.objects));
    const std::vector<std::string> problems = form.problems();
    if (!problems.empty()) {
        for (const std::string& p : problems) {
            out.message += (out.message.empty() ? "" : "; ") + p;
        }
        return out;
    }
    session.invalidate();
    const ToolpathOutcome o = session.add(form.definition());
    out.ok = o.ok;
    out.message = o.message;
    out.ids = o.ids;
    return out;
}

void collect(std::vector<GadgetMenuItem>& level, const std::vector<std::string>& path, std::size_t depth,
             GadgetMenuItem leaf) {
    if (depth + 1 == path.size()) {
        leaf.label = path[depth];
        level.push_back(std::move(leaf));
        return;
    }
    auto it = std::find_if(level.begin(), level.end(), [&](const GadgetMenuItem& m) {
        return m.gadget < 0 && m.label == path[depth];
    });
    if (it == level.end()) {
        GadgetMenuItem sub;
        sub.label = path[depth];
        level.push_back(std::move(sub));
        it = level.end() - 1;
    }
    collect(it->children, path, depth + 1, std::move(leaf));
}

void sortMenu(std::vector<GadgetMenuItem>& level, const std::vector<GadgetInfo>& gadgets) {
    const auto folderOf = [&](const GadgetMenuItem& m) {
        return m.gadget >= 0 ? gadgets[static_cast<std::size_t>(m.gadget)].folderName : std::string();
    };
    std::stable_sort(level.begin(), level.end(), [&](const GadgetMenuItem& a, const GadgetMenuItem& b) {
        const bool as = a.gadget < 0;
        const bool bs = b.gadget < 0;
        if (as != bs) {
            return as;  // submenus first
        }
        const std::string la = lowerCase(a.label);
        const std::string lb = lowerCase(b.label);
        if (la != lb) {
            return la < lb;
        }
        return folderOf(a) < folderOf(b);
    });
    for (GadgetMenuItem& m : level) {
        sortMenu(m.children, gadgets);
    }
}

}  // namespace

GadgetErrorReport describeFailure(const scripting::GadgetRunResult& run, const std::string& gadgetName) {
    GadgetErrorReport r;
    r.status = run.script.status;
    r.line = run.script.line;
    r.detail = run.script.message;
    if (!run.script.traceback.empty()) {
        r.detail += "\n" + run.script.traceback;
    }
    const std::string name = "\"" + gadgetName + "\"";
    const std::string untouched = " Your drawing was not changed.";
    switch (run.script.status) {
        case ScriptStatus::Ok:
            r.title = "Gadget " + name;
            r.summary = "The gadget finished.";
            break;
        case ScriptStatus::SyntaxError:
            r.title = "Gadget " + name + " has a syntax error";
            r.summary = "The script could not be read" +
                        (run.script.line > 0 ? " (line " + std::to_string(run.script.line) + ")" : std::string()) +
                        "." + untouched;
            break;
        case ScriptStatus::RuntimeError:
            r.title = "Gadget " + name + " stopped with an error";
            r.summary = run.script.message + "." + untouched;
            if (!run.script.message.empty() && run.script.message.back() == '.') {
                r.summary.pop_back();
            }
            break;
        case ScriptStatus::InstructionLimit:
            r.title = "Gadget " + name + " ran too long";
            r.summary = "The gadget used more computing steps than a gadget is allowed (it may be stuck in a loop)." +
                        untouched;
            break;
        case ScriptStatus::MemoryLimit:
            r.title = "Gadget " + name + " used too much memory";
            r.summary = "The gadget needed more memory than a gadget is allowed." + untouched;
            break;
        case ScriptStatus::OutputLimit:
            r.title = "Gadget " + name + " wrote too much text";
            r.summary = "The gadget printed more text than a gadget is allowed." + untouched;
            break;
        case ScriptStatus::Cancelled:
            r.title = "Gadget " + name + " was cancelled";
            r.summary = "The gadget was stopped." + untouched;
            break;
        case ScriptStatus::Timeout:
            r.title = "Gadget " + name + " took too long on this machine";
            r.summary = "The gadget did not finish within the time gadgets are given. This depends on how fast and "
                        "busy this computer is, so it may work on another run or machine; it is not necessarily a "
                        "mistake in the gadget." +
                        untouched;
            break;
        case ScriptStatus::LimitsInvalid:
            r.title = "Gadget " + name + " is too large";
            r.summary = "The script is larger than a gadget may be." + untouched;
            break;
    }
    return r;
}

scripting::GadgetServices makeGadgetServices(document::Document&, ToolpathSession& session, scripting::GadgetUi ui) {
    scripting::GadgetServices sv;
    sv.ui = std::move(ui);
    sv.tools = [&session] {
        std::vector<scripting::GadgetTool> out;
        for (const auto& kv : session.toolDatabase().tools()) {
            out.push_back({kv.first, kv.second.name, kv.second.diameter, std::string(tooling::toString(kv.second.type))});
        }
        return out;
    };
    sv.createToolpath = [&session](const scripting::ToolpathRequest& req) {
        return createToolpath(session, req);
    };
    sv.toolSnapshot = [&session](const std::string& id) -> std::optional<toolpath::ToolSnapshot> {
        const ToolLookup l = resolveTool(session.toolDatabase(), document::ToolSource{id, {}, {}});
        if (!l.tool) {
            return std::nullopt;
        }
        return toolpath::toolSnapshot(l.tool->tool, l.tool->data.value_or(l.tool->tool.defaultCuttingData));
    };
    return sv;
}

GadgetManager::GadgetManager(std::filesystem::path folder) : folder_(std::move(folder)) {
    rescan();
}

void GadgetManager::rescan() {
    discovery_ = scripting::discoverGadgets(folder_, overrides_);
}

std::string GadgetManager::assignShortcut(std::size_t index, const std::string& shortcut) {
    if (index >= discovery_.gadgets.size()) {
        return "no such gadget";
    }
    std::string canonical;
    if (!shortcut.empty()) {
        const auto n = scripting::normaliseShortcut(shortcut);
        if (!n) {
            return "a gadget shortcut is Ctrl+F1 to Ctrl+F12, optionally with Shift";
        }
        canonical = *n;
        for (std::size_t i = 0; i < discovery_.gadgets.size(); ++i) {
            if (i != index && discovery_.gadgets[i].shortcut == canonical) {
                return canonical + " is already used by \"" + discovery_.gadgets[i].manifest.name + "\"";
            }
        }
    }
    const std::string folder = discovery_.gadgets[index].folderName;
    const bool manifestHas = discovery_.gadgets[index].manifest.shortcut == canonical;
    if (manifestHas) {
        overrides_.erase(folder);  // back to the manifest's own
    } else {
        overrides_[folder] = canonical;
    }
    rescan();
    return {};
}

std::vector<GadgetMenuItem> GadgetManager::menu() const {
    std::vector<GadgetMenuItem> root;
    for (std::size_t i = 0; i < discovery_.gadgets.size(); ++i) {
        const GadgetInfo& g = discovery_.gadgets[i];
        GadgetMenuItem leaf;
        leaf.gadget = static_cast<int>(i);
        leaf.shortcut = g.shortcut;
        leaf.enabled = g.runnable();
        leaf.tooltip = g.runnable() ? g.manifest.description : "Cannot run: " + g.problem;
        collect(root, scripting::menuPath(g.manifest), 0, std::move(leaf));
    }
    sortMenu(root, discovery_.gadgets);
    return root;
}

std::optional<std::size_t> GadgetManager::byShortcut(const std::string& shortcut) const {
    if (shortcut.empty()) {
        return std::nullopt;
    }
    for (std::size_t i = 0; i < discovery_.gadgets.size(); ++i) {
        if (discovery_.gadgets[i].shortcut == shortcut && discovery_.gadgets[i].runnable()) {
            return i;
        }
    }
    return std::nullopt;
}

std::vector<GadgetManager::ShortcutRow> GadgetManager::shortcuts() const {
    std::vector<ShortcutRow> rows;
    for (std::size_t i = 0; i < discovery_.gadgets.size(); ++i) {
        const GadgetInfo& g = discovery_.gadgets[i];
        ShortcutRow r;
        r.gadget = i;
        r.name = g.manifest.name;
        r.shortcut = g.shortcut;
        for (const std::string& n : g.notes) {
            r.note += (r.note.empty() ? "" : "; ") + n;
        }
        if (!g.problem.empty()) {
            r.note += (r.note.empty() ? "" : "; ") + g.problem;
        }
        rows.push_back(std::move(r));
    }
    return rows;
}

scripting::InstallResult GadgetManager::inspect(const std::filesystem::path& folder) const {
    return scripting::inspectGadgetFolder(folder, scripting::ScriptLimits{}.maxSourceBytes);
}

scripting::InstallResult GadgetManager::install(const std::filesystem::path& folder, bool replace) {
    scripting::InstallResult r =
        scripting::installGadget(folder, folder_, scripting::ScriptLimits{}.maxSourceBytes, replace);
    if (r.ok) {
        rescan();
    }
    return r;
}

GadgetRunReport GadgetManager::runSource(const std::string& source, const std::string& name, document::Document& doc,
                                         ToolpathSession& session, scripting::GadgetUi ui,
                                         scripting::ScriptLimits limits) {
    GadgetRunReport report;
    scripting::GadgetRunOptions opt;
    opt.undoText = name;
    opt.limits = std::move(limits);
    const scripting::GadgetServices services = makeGadgetServices(doc, session, std::move(ui));
    session.invalidate();
    report.run = scripting::runGadget(doc, services, source, "gadget.lua", opt);
    session.invalidate();  // the document changed (or was rolled back) without the session's help
    report.ok = report.run.ok();
    if (!report.ok) {
        report.error = describeFailure(report.run, name);
        return report;
    }
    const std::size_t objects = report.run.createdObjects.size();
    const std::size_t toolpaths = report.run.createdToolpaths.size();
    report.summary = name + ": " + std::to_string(objects) + (objects == 1 ? " object, " : " objects, ") +
                     std::to_string(toolpaths) + (toolpaths == 1 ? " toolpath" : " toolpaths");
    return report;
}

GadgetRunReport GadgetManager::run(std::size_t index, document::Document& doc, ToolpathSession& session,
                                   scripting::GadgetUi ui, scripting::ScriptLimits limits) const {
    GadgetRunReport report;
    if (index >= discovery_.gadgets.size()) {
        report.error = GadgetErrorReport{"Gadget not found", "This gadget is no longer in the gadgets folder.", "",
                                         ScriptStatus::RuntimeError, 0};
        return report;
    }
    const GadgetInfo& g = discovery_.gadgets[index];
    if (!g.runnable()) {
        report.error = GadgetErrorReport{"Gadget \"" + g.manifest.name + "\" cannot run", g.problem + ".", "",
                                         ScriptStatus::RuntimeError, 0};
        return report;
    }
    std::string source;
    const std::string err = scripting::readGadgetSource(g, limits.maxSourceBytes, &source);
    if (!err.empty()) {
        report.error = GadgetErrorReport{"Gadget \"" + g.manifest.name + "\" cannot be read",
                                         "gadget.lua: " + err + ".", "", ScriptStatus::RuntimeError, 0};
        return report;
    }
    return runSource(source, g.manifest.name, doc, session, std::move(ui), std::move(limits));
}

}  // namespace carvetto::viewmodel
