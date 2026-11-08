#include "carvetto/viewmodel/toolpath_session.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>

#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/toolpath/v_carve.hpp"

namespace carvetto::viewmodel {

using document::Document;
using document::ToolpathId;
using document::ToolpathOperation;
using document::ToolpathRecord;

namespace {

/// Equal parameters, ignoring the name (a rename edits it in the record and its baseline alike).
bool sameParams(const document::ToolpathParams& a, const document::ToolpathParams& b) {
    if (a.index() != b.index()) {
        return false;
    }
    document::ToolpathParams x = a;
    std::visit([&](auto& p) { p.name = std::get<std::decay_t<decltype(p)>>(b).name; }, x);
    return x == b;
}

/// Why a record cannot be stored (the guards of the core's toolpath commands), or "".
std::string recordProblem(const Document& doc, const ToolpathRecord& record) {
    if (!record.hasMoves()) {
        return "the toolpath has no computed moves";
    }
    if (const std::string key = document::nonFiniteParam(record.params); !key.empty()) {
        return "parameter " + key + " is not a finite number";
    }
    if (!document::spindleOffAllowed(record)) {
        return document::spindleOffProblem(record);
    }
    if (const std::string bad = validateToolpathName(record.name); !bad.empty()) {
        return bad;
    }
    VectorSelector sel;
    sel.base = record.selection;
    return validateSelector(doc, sel);
}

ToolpathOutcome failure(std::string message, std::vector<ToolpathMessage> messages = {}) {
    ToolpathOutcome out;
    out.message = std::move(message);
    out.messages = std::move(messages);
    return out;
}

bool isClearanceType(const tooling::Tool& tool) {
    return toolpath::vCarvingClearanceToolAllowed(tool.type) && !toolpath::vCarvingToolAllowed(tool.type);
}

}  // namespace

ToolpathSession::ToolpathSession(Document* doc) : doc_(doc) {}
ToolpathSession::~ToolpathSession() = default;

void ToolpathSession::setDocument(Document* doc) {
    doc_ = doc;
    baselines_.clear();
    stats_.clear();
    disabled_.clear();
    messages_.clear();
    staleness_.clear();
    staleValid_ = false;
}

void ToolpathSession::setToolDatabase(tooling::ToolDatabase db) {
    db_ = std::move(db);
    ++dbRevision_;
    staleValid_ = false;
}

bool ToolpathSession::setMachineSetup(const MachineSetup& setup) {
    if (!std::isfinite(setup.rapidRateMmPerMin) || setup.rapidRateMmPerMin < 0.0 ||
        std::isnan(setup.accelerationMmPerS2) || !(setup.accelerationMmPerS2 > 0.0)) {
        return false;
    }
    setup_ = setup;
    return true;
}

bool ToolpathSession::setMachine(const tooling::Machine& machine, double rapidRateMmPerMin) {
    double accel = std::numeric_limits<double>::infinity();
    for (const double a : {machine.accelerationX, machine.accelerationY, machine.accelerationZ}) {
        if (std::isfinite(a) && a > 0.0) {
            accel = std::min(accel, a);
        }
    }
    return setMachineSetup(MachineSetup{rapidRateMmPerMin, accel});
}

void ToolpathSession::invalidate() {
    ++userRevision_;
    staleValid_ = false;
}

// --- Baselines -----------------------------------------------------------------------------------

void ToolpathSession::adopt(const ToolpathRecord& record) {
    Baseline b;
    b.def.params = record.params;
    b.def.tool = record.toolSource;
    b.def.selector.base = record.selection;
    b.vectors = fingerprint(resolveVectors(*doc_, b.def.selector));
    b.tools = toolsFingerprintOf(b.def, db_);
    baselines_.emplace(MovesKey(record.moves), std::move(b));
}

void ToolpathSession::inferGroups(const std::vector<bool>& adopted) {
    const std::vector<ToolpathRecord>& recs = doc_->toolpaths();
    std::vector<std::size_t> run;
    for (std::size_t i = 0; i < recs.size(); ++i) {
        const ToolpathRecord& rec = recs[i];
        if (!adopted[i] || rec.operation() != ToolpathOperation::VCarving) {
            run.clear();
            continue;
        }
        if (!run.empty()) {
            const ToolpathRecord& first = recs[run.front()];
            if (!(first.params == rec.params) || !(first.selection == rec.selection)) {
                run.clear();
            }
        }
        const tooling::Tool* tool = db_.findTool(rec.toolSource.toolId);
        if (tool != nullptr && isClearanceType(*tool)) {
            run.push_back(i);
            continue;
        }
        if (!run.empty()) {
            const std::uint64_t group = nextGroup_++;
            std::vector<document::ToolSource> clearance;
            for (const std::size_t k : run) {
                clearance.push_back(recs[k].toolSource);
            }
            Baseline* carving = baselineOf(rec);
            if (carving != nullptr) {
                carving->group = group;
                carving->role = -1;
                carving->def.clearanceTools = clearance;
                carving->tools = toolsFingerprintOf(carving->def, db_);
                for (std::size_t r = 0; r < run.size(); ++r) {
                    Baseline* member = baselineOf(recs[run[r]]);
                    if (member != nullptr) {
                        *member = *carving;
                        member->role = static_cast<int>(r);
                    }
                }
            }
            run.clear();
        }
    }
    // Clearance toolpaths left without a carving toolpath cannot be recalculated alone.
    for (std::size_t i = 0; i < recs.size(); ++i) {
        if (!adopted[i] || recs[i].operation() != ToolpathOperation::VCarving) {
            continue;
        }
        Baseline* b = baselineOf(recs[i]);
        const tooling::Tool* tool = db_.findTool(recs[i].toolSource.toolId);
        if (b != nullptr && b->group == 0 && tool != nullptr && isClearanceType(*tool)) {
            b->role = -2;
        }
    }
}

void ToolpathSession::sync() {
    if (doc_ == nullptr) {
        return;
    }
    for (auto it = baselines_.begin(); it != baselines_.end();) {
        it = it->first.expired() ? baselines_.erase(it) : std::next(it);
    }
    for (auto it = stats_.begin(); it != stats_.end();) {
        it = it->first.expired() ? stats_.erase(it) : std::next(it);
    }
    const std::vector<ToolpathRecord>& recs = doc_->toolpaths();
    std::vector<bool> adopted(recs.size(), false);
    bool any = false;
    for (std::size_t i = 0; i < recs.size(); ++i) {
        if (recs[i].moves != nullptr && baselines_.find(recs[i].moves) == baselines_.end()) {
            adopt(recs[i]);
            adopted[i] = true;
            any = true;
        }
    }
    if (any) {
        inferGroups(adopted);
        staleValid_ = false;
    }
}

ToolpathSession::Baseline* ToolpathSession::baselineOf(const ToolpathRecord& record) {
    if (record.moves == nullptr) {
        return nullptr;
    }
    const auto it = baselines_.find(record.moves);
    return it == baselines_.end() ? nullptr : &it->second;
}

std::vector<ToolpathId> ToolpathSession::membersOf(const ToolpathRecord& record) {
    std::vector<ToolpathId> out;
    const Baseline* b = baselineOf(record);
    if (b == nullptr || b->group == 0) {
        out.push_back(record.id);
        return out;
    }
    for (const ToolpathRecord& r : doc_->toolpaths()) {
        const Baseline* rb = baselineOf(r);
        if (rb != nullptr && rb->group == b->group) {
            out.push_back(r.id);
        }
    }
    return out;
}

const ToolpathRecord* ToolpathSession::carvingOf(const ToolpathRecord& record) {
    const Baseline* b = baselineOf(record);
    if (b == nullptr || b->group == 0 || b->role == -1) {
        return &record;
    }
    for (const ToolpathRecord& r : doc_->toolpaths()) {
        const Baseline* rb = baselineOf(r);
        if (rb != nullptr && rb->group == b->group && rb->role == -1) {
            return &r;
        }
    }
    return nullptr;
}

ToolpathDefinition ToolpathSession::definitionOf(const ToolpathRecord& record) {
    ToolpathDefinition def;
    const Baseline* b = baselineOf(record);
    if (b == nullptr) {
        def.params = record.params;
        def.tool = record.toolSource;
        def.selector.base = record.selection;
        return def;
    }
    const ToolpathRecord* carving = carvingOf(record);
    def = b->def;
    def.params = carving != nullptr ? carving->params : record.params;
    return def;
}

// --- Staleness -----------------------------------------------------------------------------------

void ToolpathSession::computeStaleness() {
    sync();
    if (doc_ == nullptr) {
        staleness_.clear();
        return;
    }
    const std::uint64_t generation = doc_->undoStack().generation();
    if (staleValid_ && staleGeneration_ == generation && staleDb_ == dbRevision_ && staleUser_ == userRevision_ &&
        staleSheet_ == doc_->activeSheet()) {
        return;
    }
    staleness_.clear();
    std::vector<std::pair<VectorSelector, std::uint64_t>> memo;
    const auto vectorsOf = [&](const VectorSelector& selector) {
        for (const auto& [sel, fp] : memo) {
            if (sel == selector) {
                return fp;
            }
        }
        const std::uint64_t fp = fingerprint(resolveVectors(*doc_, selector));
        memo.emplace_back(selector, fp);
        return fp;
    };
    for (const ToolpathRecord& rec : doc_->toolpaths()) {
        Staleness s;
        const ToolLookup own = resolveTool(db_, rec.toolSource);
        Baseline* b = baselineOf(rec);
        if (b == nullptr) {
            s.canRecalculate = own.tool.has_value();
            if (!s.canRecalculate) {
                s.note = rec.toolSource.toolId.empty() ? "the tool of this toolpath is not known"
                                                       : "tool: " + own.error;
            }
            staleness_[rec.id] = std::move(s);
            continue;
        }
        s.computed = true;
        if (b->role == -2) {
            s.canRecalculate = false;
            s.note = "this clearance toolpath of a V-carve has lost its carving toolpath; recalculate the V-carve";
        }
        const ToolpathDefinition def = definitionOf(rec);
        s.vectors = vectorsOf(def.selector) != b->vectors;
        s.params = !sameParams(rec.params, b->def.params);
        const std::uint64_t now = toolsFingerprintOf(def, db_);
        if (now == 0) {
            if (s.canRecalculate) {
                s.canRecalculate = false;
                const ToolLookup l = resolveTool(db_, def.tool);
                s.note = l.tool.has_value() ? "a clearance tool is not in the tool database" : "tool: " + l.error;
            }
        } else {
            if (b->tools == 0) {
                b->tools = now;  // the database was not known when the record was adopted
            }
            s.tool = now != b->tools;
        }
        staleness_[rec.id] = std::move(s);
    }
    staleValid_ = true;
    staleGeneration_ = generation;
    staleDb_ = dbRevision_;
    staleUser_ = userRevision_;
    staleSheet_ = doc_->activeSheet();
}

std::vector<ToolpathListItem> ToolpathSession::list() {
    std::vector<ToolpathListItem> out;
    if (doc_ == nullptr) {
        return out;
    }
    computeStaleness();
    for (const ToolpathRecord& rec : doc_->toolpaths()) {
        ToolpathListItem item;
        item.id = rec.id;
        item.name = rec.name;
        item.operation = rec.operation();
        item.toolName = rec.tool.name;
        item.moveCount = rec.moveCount();
        item.enabled = !disabled_.contains(rec.id);
        const Staleness& s = staleness_[rec.id];
        item.vectorsChanged = s.vectors;
        item.parametersChanged = s.params;
        item.toolChanged = s.tool;
        item.canRecalculate = s.canRecalculate;
        item.note = s.note;
        item.status = !s.computed ? ToolpathStatus::NotComputed
                      : (s.vectors || s.params || s.tool) ? ToolpathStatus::NeedsRecalculation
                                                           : ToolpathStatus::Current;
        out.push_back(std::move(item));
    }
    return out;
}

std::optional<ToolpathListItem> ToolpathSession::item(ToolpathId id) {
    for (ToolpathListItem& i : list()) {
        if (i.id == id) {
            return std::move(i);
        }
    }
    return std::nullopt;
}

std::vector<ToolpathMessage> ToolpathSession::lastMessages(ToolpathId id) const {
    const auto it = messages_.find(id);
    return it == messages_.end() ? std::vector<ToolpathMessage>{} : it->second;
}

bool ToolpathSession::isEnabled(ToolpathId id) const { return !disabled_.contains(id); }

bool ToolpathSession::setEnabled(ToolpathId id, bool enabled) {
    if (doc_ == nullptr || doc_->findToolpath(id) == nullptr) {
        return false;
    }
    if (enabled) {
        disabled_.erase(id);
    } else {
        disabled_.insert(id);
    }
    return true;
}

std::vector<ToolpathId> ToolpathSession::enabledToolpaths() {
    std::vector<ToolpathId> out;
    if (doc_ == nullptr) {
        return out;
    }
    for (const ToolpathRecord& rec : doc_->toolpaths()) {
        if (rec.hasMoves() && !disabled_.contains(rec.id)) {
            out.push_back(rec.id);
        }
    }
    return out;
}

// --- Forms ---------------------------------------------------------------------------------------

ToolpathForm ToolpathSession::newForm(ToolpathOperation operation) {
    ToolpathForm form(operation, doc_ != nullptr ? doc_->job().displayUnits : document::Units::Millimetres);
    if (doc_ != nullptr) {
        VectorSelector current = VectorSelector::fromSelection(*doc_);
        if (!current.base.objects.empty()) {
            form.setSelector(std::move(current));
        }
    }
    return form;
}

std::optional<ToolpathDefinition> ToolpathSession::definition(ToolpathId id) {
    if (doc_ == nullptr) {
        return std::nullopt;
    }
    sync();
    const ToolpathRecord* rec = doc_->findToolpath(id);
    if (rec == nullptr) {
        return std::nullopt;
    }
    return definitionOf(*rec);
}

std::optional<ToolpathForm> ToolpathSession::form(ToolpathId id) {
    std::optional<ToolpathDefinition> def = definition(id);
    if (!def.has_value()) {
        return std::nullopt;
    }
    ToolpathForm f(*def, doc_->job().displayUnits);
    f.refreshTools(db_);
    return f;
}

std::size_t ToolpathSession::countVectors(const VectorSelector& selector) const {
    if (doc_ == nullptr) {
        return 0;
    }
    return resolveVectors(*doc_, selector).contours.size();
}

// --- Calculation ---------------------------------------------------------------------------------

ToolpathJob ToolpathSession::prepare(const ToolpathDefinition& definition, std::optional<ToolpathId> target) {
    if (doc_ == nullptr) {
        ToolpathJob job;
        job.definition = definition;
        job.refusal = "there is no document";
        return job;
    }
    sync();
    const std::string bad = validateSelector(*doc_, definition.selector);
    const VectorSelector selector = bad.empty() ? normalisedSelector(definition.selector) : VectorSelector{};
    const ResolvedVectors resolved = resolveVectors(*doc_, selector);
    const document::VectorSelection stored = storedSelection(selector, resolved);
    ToolpathJob job = makeToolpathJob(definition, resolved, db_, stored);
    if (!bad.empty()) {
        job.refusal = bad;
    }
    job.target = target.value_or(ToolpathId{});
    if (!job.target.isNull()) {
        if (const ToolpathRecord* rec = doc_->findToolpath(job.target)) {
            for (const ToolpathId m : membersOf(*rec)) {
                if (const ToolpathRecord* mr = doc_->findToolpath(m)) {
                    job.targetRecords.push_back(*mr);
                }
            }
        }
    }
    return job;
}

ToolpathOutcome ToolpathSession::apply(const ToolpathCalculation& calculation) {
    return applyOne(calculation, calculation.target.isNull() ? "Add toolpath" : "Recalculate toolpath");
}

ToolpathOutcome ToolpathSession::applyOne(const ToolpathCalculation& calculation, const char* text) {
    BatchOutcome batch = applyMany({calculation}, text);
    return std::move(batch.items.front());
}

BatchOutcome ToolpathSession::applyAll(const std::vector<ToolpathCalculation>& calculations) {
    return applyMany(calculations, "Recalculate toolpaths");
}

BatchOutcome ToolpathSession::applyMany(const std::vector<ToolpathCalculation>& calculations, const char* text) {
    BatchOutcome out;
    out.items.resize(calculations.size());
    const auto failAll = [&](const std::string& why) {
        for (ToolpathOutcome& o : out.items) {
            if (o.ok || o.message.empty()) {
                o.ok = false;
                o.message = why;
                o.ids.clear();
            }
        }
        out.failed = calculations.size();
        out.recalculated = 0;
        out.ok = false;
        out.message = why;
    };
    if (doc_ == nullptr) {
        failAll("there is no document");
        return out;
    }
    sync();

    struct Plan {
        std::size_t slot{0};
        std::vector<ToolpathId> members;
        std::vector<std::pair<int, ToolpathId>> roles;  // existing members by role
        std::uint64_t group{0};
    };
    std::vector<Plan> plans;
    std::set<ToolpathId> claimed;  // members already claimed by a plan of this batch
    std::uint64_t newIds = 0;
    for (std::size_t i = 0; i < calculations.size(); ++i) {
        const ToolpathCalculation& c = calculations[i];
        ToolpathOutcome& o = out.items[i];
        o.messages = c.messages;
        const auto reject = [&](std::string why) {
            o.ok = false;
            o.message = std::move(why);
        };
        if (c.cancelled) {
            reject("the calculation was cancelled");
            continue;
        }
        if (!c.ok || c.items.empty()) {
            const std::string why = c.errorText();
            reject(why.empty() ? "the calculation did not produce a toolpath" : why);
            continue;
        }
        std::string problem;
        for (const ToolpathCalculation::Item& item : c.items) {
            problem = recordProblem(*doc_, item.record);
            if (!problem.empty()) {
                break;
            }
        }
        if (!problem.empty()) {
            reject(problem);
            continue;
        }
        Plan plan;
        plan.slot = i;
        if (!c.target.isNull()) {
            const ToolpathRecord* rec = doc_->findToolpath(c.target);
            if (rec == nullptr) {
                reject("the toolpath no longer exists");
                continue;
            }
            plan.members = membersOf(*rec);
            bool same = plan.members.size() == c.targetRecords.size();
            for (std::size_t k = 0; same && k < plan.members.size(); ++k) {
                const ToolpathRecord* cur = doc_->findToolpath(plan.members[k]);
                const ToolpathRecord& was = c.targetRecords[k];
                same = cur != nullptr && cur->id == was.id &&
                       (cur->moves == was.moves || *cur == was) && cur->name == was.name &&
                       cur->params == was.params && cur->selection == was.selection &&
                       cur->toolSource == was.toolSource;
            }
            if (!same) {
                reject("the toolpath was edited or recalculated since this calculation was prepared; calculate it again");
                continue;
            }
            if (std::any_of(plan.members.begin(), plan.members.end(),
                            [&](ToolpathId m) { return claimed.contains(m); })) {
                reject("the toolpath is recalculated twice in one step");
                continue;
            }
            for (const ToolpathId m : plan.members) {
                const ToolpathRecord* mr = doc_->findToolpath(m);
                const Baseline* mb = mr != nullptr ? baselineOf(*mr) : nullptr;
                plan.roles.emplace_back(mb != nullptr ? mb->role : -1, m);
                if (mb != nullptr && mb->group != 0) {
                    plan.group = mb->group;
                }
            }
        }
        // The inputs must still be the ones the calculation was made from.
        const VectorSelector selector = normalisedSelector(c.definition.selector);
        if (!validateSelector(*doc_, selector).empty() ||
            fingerprint(resolveVectors(*doc_, selector)) != c.vectorsFingerprint) {
            reject("the vectors changed since the toolpath was calculated; calculate it again");
            continue;
        }
        if (toolsFingerprintOf(c.definition, db_) != c.toolsFingerprint) {
            reject("the tool changed since the toolpath was calculated; calculate it again");
            continue;
        }
        for (const ToolpathId m : plan.members) {
            claimed.insert(m);
        }
        for (const ToolpathCalculation::Item& item : c.items) {
            const bool exists = std::any_of(plan.roles.begin(), plan.roles.end(),
                                            [&](const auto& r) { return r.first == item.role; });
            if (!exists) {
                ++newIds;
            }
        }
        o.ok = true;  // provisionally
        plans.push_back(std::move(plan));
    }
    if (plans.empty()) {
        out.failed = calculations.size();
        out.ok = false;
        out.message = out.items.empty() ? "nothing to apply" : out.items.front().message;
        return out;
    }
    if (!doc_->idGenerator().canAllocate(newIds)) {
        failAll("the document has no free ids left for another toolpath");
        return out;
    }

    // Execute everything as one undo step.
    struct Registration {
        document::SharedMoves moves;
        std::size_t calc{0};
        int role{-1};
        std::uint64_t group{0};
    };
    std::vector<Registration> regs;
    document::UndoStack& undo = doc_->undoStack();
    undo.breakCoalescing();
    undo.beginMacro(text);
    bool failed = false;
    std::size_t executed = 0;
    const auto run = [&](std::unique_ptr<document::Command> cmd) {
        if (doc_->execute(std::move(cmd))) {
            ++executed;
            return true;
        }
        failed = true;
        return false;
    };
    for (const Plan& plan : plans) {
        if (failed) {
            break;
        }
        const ToolpathCalculation& c = calculations[plan.slot];
        ToolpathOutcome& o = out.items[plan.slot];
        const bool grouped = !c.definition.clearanceTools.empty() &&
                             c.definition.operation() == ToolpathOperation::VCarving;
        const std::uint64_t group = grouped ? (plan.group != 0 ? plan.group : nextGroup_++) : 0;
        std::set<ToolpathId> kept;
        std::optional<ToolpathId> prev;
        for (const ToolpathCalculation::Item& item : c.items) {
            ToolpathRecord rec = item.record;
            const auto existing = std::find_if(plan.roles.begin(), plan.roles.end(),
                                               [&](const auto& r) { return r.first == item.role; });
            if (existing != plan.roles.end()) {
                rec.id = existing->second;
                kept.insert(rec.id);
                const ToolpathRecord* cur = doc_->findToolpath(rec.id);
                if (cur != nullptr && *cur == rec) {
                    regs.push_back({cur->moves, plan.slot, item.role, group});  // nothing to store
                } else {
                    if (!run(std::make_unique<document::SetToolpathCommand>(rec))) {
                        break;
                    }
                    regs.push_back({rec.moves, plan.slot, item.role, group});
                }
                o.ids.push_back(rec.id);
                prev = rec.id;
            } else {
                std::optional<std::size_t> at;
                if (prev.has_value()) {
                    if (const auto pi = doc_->toolpathIndex(*prev)) {
                        at = *pi + 1;
                    }
                } else if (!plan.members.empty()) {
                    at = doc_->toolpathIndex(plan.members.front());
                }
                auto cmd = std::make_unique<document::AddToolpathCommand>(rec, at);
                const document::AddToolpathCommand* raw = cmd.get();
                if (!run(std::move(cmd))) {
                    break;
                }
                rec.id = raw->id();
                regs.push_back({rec.moves, plan.slot, item.role, group});
                o.ids.push_back(rec.id);
                prev = rec.id;
            }
        }
        if (failed) {
            break;
        }
        std::vector<ToolpathId> obsolete;
        for (const ToolpathId m : plan.members) {
            if (!kept.contains(m)) {
                obsolete.push_back(m);
            }
        }
        if (!obsolete.empty()) {
            run(std::make_unique<document::RemoveToolpathsCommand>(std::move(obsolete)));
        }
    }
    undo.endMacro();
    std::string unsavable;
    if (!failed) {
        // A file must hold at most one move per byte (and 8 M moves): that is only known once it is
        // compressed, so ask the file writer itself rather than repeating its rules. Small jobs
        // cannot reach the ratio.
        std::uint64_t moves = 0;
        for (const ToolpathRecord& r : doc_->toolpaths()) {
            moves += r.moveCount();
        }
        if (moves >= 2000) {
            const auto saved = document::saveCarv(*doc_);
            if (!saved && saved.error().code == document::CarvError::Code::ResourceLimit) {
                unsavable = saved.error().message;
                failed = true;
            }
        }
    }
    if (failed) {
        if (executed > 0) {
            undo.undoAndDiscard(*doc_);
        }
        if (!unsavable.empty()) {
            for (ToolpathOutcome& o : out.items) {
                o.ids.clear();
            }
            failAll("the toolpath cannot be saved in a .carv file: " + unsavable + "; nothing was changed");
            return out;
        }
        for (ToolpathOutcome& o : out.items) {
            o.ids.clear();
        }
        failAll("the toolpath could not be stored; nothing was changed");
        return out;
    }

    for (const Registration& reg : regs) {
        const ToolpathCalculation& c = calculations[reg.calc];
        Baseline b;
        b.def = c.definition;
        b.vectors = c.vectorsFingerprint;
        b.tools = c.toolsFingerprint;
        b.group = reg.group;
        b.role = reg.role;
        baselines_[MovesKey(reg.moves)] = std::move(b);
    }
    for (const Plan& plan : plans) {
        const ToolpathCalculation& c = calculations[plan.slot];
        ToolpathOutcome& o = out.items[plan.slot];
        for (const ToolpathId id : o.ids) {
            messages_[id] = c.messages;
        }
        o.ok = true;
        std::size_t moves = 0;
        for (const ToolpathCalculation::Item& item : c.items) {
            moves += item.record.moveCount();
        }
        o.message = std::string(c.target.isNull() ? "Added " : "Calculated ") + std::to_string(c.items.size()) +
                    " toolpath(s) with " + std::to_string(moves) + " moves";
        ++out.recalculated;
    }
    out.failed = calculations.size() - plans.size();
    out.ok = out.failed == 0;
    out.message = out.ok ? "Done" : std::to_string(out.failed) + " toolpath(s) could not be calculated";
    staleValid_ = false;
    return out;
}

ToolpathOutcome ToolpathSession::runAndApply(const ToolpathDefinition& definition, std::optional<ToolpathId> target,
                                             const char* text) {
    const ToolpathJob job = prepare(definition, target);
    const ToolpathCalculation calc = job.run();
    ToolpathOutcome out = applyOne(calc, text);
    if (!out.ok && target.has_value()) {
        messages_[*target] = calc.messages;
    }
    return out;
}

ToolpathOutcome ToolpathSession::add(const ToolpathDefinition& definition) {
    if (doc_ == nullptr) {
        return failure("there is no document");
    }
    return runAndApply(definition, std::nullopt, "Add toolpath");
}

ToolpathOutcome ToolpathSession::update(ToolpathId id, const ToolpathDefinition& definition) {
    if (doc_ == nullptr) {
        return failure("there is no document");
    }
    sync();
    const ToolpathRecord* rec = doc_->findToolpath(id);
    if (rec == nullptr) {
        return failure("the toolpath no longer exists");
    }
    if (rec->operation() != definition.operation()) {
        return failure("the operation of a toolpath cannot be changed; add a new toolpath");
    }
    const ToolpathRecord* carving = carvingOf(*rec);
    return runAndApply(definition, carving != nullptr ? carving->id : rec->id, "Edit toolpath");
}

ToolpathOutcome ToolpathSession::recalculate(ToolpathId id) {
    if (doc_ == nullptr) {
        return failure("there is no document");
    }
    sync();
    const ToolpathRecord* rec = doc_->findToolpath(id);
    if (rec == nullptr) {
        return failure("the toolpath no longer exists");
    }
    computeStaleness();
    const Staleness& s = staleness_[id];
    if (!s.canRecalculate) {
        return failure(s.note.empty() ? "this toolpath cannot be recalculated" : s.note);
    }
    const ToolpathRecord* carving = carvingOf(*rec);
    const ToolpathDefinition def = definitionOf(*rec);
    return runAndApply(def, carving != nullptr ? carving->id : rec->id, "Recalculate toolpath");
}

BatchOutcome ToolpathSession::recalculateAll(bool onlyStale) {
    BatchOutcome out;
    if (doc_ == nullptr) {
        out.message = "there is no document";
        return out;
    }
    computeStaleness();
    struct Entry {
        bool calculated{false};
        std::size_t calc{0};
        ToolpathOutcome fixed;
    };
    std::vector<Entry> entries;
    std::vector<ToolpathCalculation> calcs;
    std::set<ToolpathId> handled;
    std::vector<ToolpathId> targets;
    for (const ToolpathRecord& rec : doc_->toolpaths()) {
        const Staleness s = staleness_[rec.id];
        if (onlyStale && s.computed && !(s.vectors || s.params || s.tool)) {
            continue;
        }
        const ToolpathRecord* carving = carvingOf(rec);
        const ToolpathId target = carving != nullptr ? carving->id : rec.id;
        if (!handled.insert(target).second) {
            continue;
        }
        Entry e;
        if (!s.canRecalculate) {
            e.fixed = failure(s.note.empty() ? "this toolpath cannot be recalculated" : s.note);
            e.fixed.ids.push_back(rec.id);
            entries.push_back(std::move(e));
            continue;
        }
        const ToolpathJob job = prepare(definitionOf(rec), target);
        e.calculated = true;
        e.calc = calcs.size();
        calcs.push_back(job.run());
        targets.push_back(target);
        entries.push_back(std::move(e));
    }
    BatchOutcome applied = applyMany(calcs, "Recalculate toolpaths");
    for (std::size_t i = 0; i < calcs.size(); ++i) {
        if (!applied.items[i].ok) {
            messages_[targets[i]] = calcs[i].messages;
        }
    }
    for (Entry& e : entries) {
        if (e.calculated) {
            out.items.push_back(std::move(applied.items[e.calc]));
        } else {
            out.items.push_back(std::move(e.fixed));
        }
    }
    for (const ToolpathOutcome& o : out.items) {
        (o.ok ? out.recalculated : out.failed) += 1;
    }
    out.ok = out.failed == 0;
    out.message = out.items.empty() ? "Nothing to recalculate"
                  : out.ok          ? "Recalculated " + std::to_string(out.recalculated) + " toolpath(s)"
                                    : std::to_string(out.failed) + " toolpath(s) could not be recalculated";
    return out;
}

// --- Other edits ---------------------------------------------------------------------------------

namespace {

const std::string& paramsNameOf(const document::ToolpathParams& params) {
    return std::visit([](const auto& p) -> const std::string& { return p.name; }, params);
}

void setParamsName(document::ToolpathParams& params, const std::string& name) {
    std::visit([&](auto& p) { p.name = name; }, params);
}

}  // namespace

std::string ToolpathSession::recordName(const ToolpathRecord& record, const std::string& paramsName) {
    const Baseline* b = baselineOf(record);
    const bool grouped = b != nullptr && record.operation() == ToolpathOperation::VCarving &&
                         !b->def.clearanceTools.empty();
    return grouped ? paramsName + " [" + record.tool.name + "]" : paramsName;
}

ToolpathOutcome ToolpathSession::duplicate(ToolpathId id) {
    if (doc_ == nullptr) {
        return failure("there is no document");
    }
    sync();
    const ToolpathRecord* rec = doc_->findToolpath(id);
    if (rec == nullptr) {
        return failure("the toolpath no longer exists");
    }
    const std::vector<ToolpathId> members = membersOf(*rec);
    if (!doc_->idGenerator().canAllocate(members.size())) {
        return failure("the document has no free ids left for another toolpath");
    }
    const ToolpathRecord* carving = carvingOf(*rec);
    const std::string newName = paramsNameOf((carving != nullptr ? *carving : *rec).params) + " (copy)";
    if (const std::string bad = validateToolpathName(newName); !bad.empty()) {
        return failure(bad);
    }
    std::vector<ToolpathRecord> copies;
    std::vector<std::optional<Baseline>> baselines;
    const Baseline* first = baselineOf(*rec);
    const std::uint64_t group = first != nullptr && first->group != 0 ? nextGroup_++ : 0;
    for (const ToolpathId m : members) {
        const ToolpathRecord* src = doc_->findToolpath(m);
        if (src == nullptr) {
            continue;
        }
        ToolpathRecord copy = *src;
        copy.id = ToolpathId{};
        setParamsName(copy.params, newName);
        copy.name = recordName(*src, newName);
        if (const std::string bad = validateToolpathName(copy.name); !bad.empty()) {
            return failure(bad);
        }
        if (copy.moves != nullptr) {
            copy.moves = std::make_shared<const std::vector<toolpath::Move>>(*copy.moves);  // own identity
        }
        const Baseline* b = baselineOf(*src);
        if (b != nullptr) {
            Baseline nb = *b;
            nb.group = group;
            setParamsName(nb.def.params, newName);
            baselines.emplace_back(std::move(nb));
        } else {
            baselines.emplace_back(std::nullopt);
        }
        copies.push_back(std::move(copy));
    }
    document::UndoStack& undo = doc_->undoStack();
    undo.breakCoalescing();
    undo.beginMacro("Duplicate toolpath");
    ToolpathOutcome out;
    bool failed = false;
    std::size_t executed = 0;
    for (const ToolpathRecord& copy : copies) {
        auto cmd = std::make_unique<document::AddToolpathCommand>(copy);
        const document::AddToolpathCommand* raw = cmd.get();
        if (!doc_->execute(std::move(cmd))) {
            failed = true;
            break;
        }
        ++executed;
        out.ids.push_back(raw->id());
    }
    undo.endMacro();
    if (failed) {
        if (executed > 0) {
            undo.undoAndDiscard(*doc_);
        }
        return failure("the toolpath could not be copied");
    }
    for (std::size_t i = 0; i < copies.size(); ++i) {
        if (baselines[i].has_value()) {
            baselines_[MovesKey(copies[i].moves)] = std::move(*baselines[i]);
        }
    }
    staleValid_ = false;
    out.ok = true;
    out.message = "Copied " + std::to_string(copies.size()) + " toolpath(s)";
    return out;
}

ToolpathOutcome ToolpathSession::deleteToolpaths(const std::vector<ToolpathId>& ids) {
    if (doc_ == nullptr) {
        return failure("there is no document");
    }
    std::vector<ToolpathId> existing;
    for (const ToolpathId id : ids) {
        if (doc_->findToolpath(id) != nullptr && std::find(existing.begin(), existing.end(), id) == existing.end()) {
            existing.push_back(id);
        }
    }
    if (existing.empty()) {
        return failure("there is no toolpath to delete");
    }
    document::UndoStack& undo = doc_->undoStack();
    undo.breakCoalescing();
    undo.beginMacro("Delete toolpaths");
    const bool done = doc_->execute(std::make_unique<document::RemoveToolpathsCommand>(existing));
    undo.endMacro();
    if (!done) {
        return failure("the toolpaths could not be deleted");
    }
    staleValid_ = false;
    ToolpathOutcome out;
    out.ok = true;
    out.message = "Deleted " + std::to_string(existing.size()) + " toolpath(s)";
    return out;
}

ToolpathOutcome ToolpathSession::reorder(ToolpathId id, std::size_t newIndex) {
    if (doc_ == nullptr) {
        return failure("there is no document");
    }
    const auto position = doc_->toolpathIndex(id);
    if (!position.has_value()) {
        return failure("the toolpath no longer exists");
    }
    if (newIndex >= doc_->toolpaths().size()) {
        return failure("there is no such position in the toolpath list");
    }
    ToolpathOutcome out;
    out.ids.push_back(id);
    if (*position == newIndex) {
        out.ok = true;
        out.message = "The toolpath is already there";
        return out;
    }
    document::UndoStack& undo = doc_->undoStack();
    undo.breakCoalescing();
    undo.beginMacro("Reorder toolpaths");
    const bool done = doc_->execute(std::make_unique<document::MoveToolpathCommand>(id, newIndex));
    undo.endMacro();
    if (!done) {
        return failure("the toolpath could not be moved");
    }
    staleValid_ = false;
    out.ok = true;
    out.message = "Moved the toolpath";
    return out;
}

ToolpathOutcome ToolpathSession::rename(ToolpathId id, const std::string& name) {
    if (doc_ == nullptr) {
        return failure("there is no document");
    }
    sync();
    if (const std::string bad = validateToolpathName(name); !bad.empty()) {
        return failure(bad);
    }
    const ToolpathRecord* rec = doc_->findToolpath(id);
    if (rec == nullptr) {
        return failure("the toolpath no longer exists");
    }
    std::vector<ToolpathRecord> changed;
    for (const ToolpathId m : membersOf(*rec)) {
        const ToolpathRecord* src = doc_->findToolpath(m);
        if (src == nullptr) {
            continue;
        }
        ToolpathRecord copy = *src;
        setParamsName(copy.params, name);
        copy.name = recordName(*src, name);
        if (const std::string bad = validateToolpathName(copy.name); !bad.empty()) {
            return failure(bad);
        }
        if (!(copy == *src)) {
            changed.push_back(std::move(copy));
        }
    }
    ToolpathOutcome out;
    out.ids.push_back(id);
    if (changed.empty()) {
        out.ok = true;
        out.message = "The name is unchanged";
        return out;
    }
    document::UndoStack& undo = doc_->undoStack();
    undo.breakCoalescing();
    undo.beginMacro("Rename toolpath");
    bool failed = false;
    std::size_t executed = 0;
    for (ToolpathRecord& copy : changed) {
        if (!doc_->execute(std::make_unique<document::SetToolpathCommand>(std::move(copy)))) {
            failed = true;
            break;
        }
        ++executed;
    }
    undo.endMacro();
    if (failed) {
        if (executed > 0) {
            undo.undoAndDiscard(*doc_);
        }
        return failure("the toolpath could not be renamed");
    }
    staleValid_ = false;
    out.ok = true;
    out.message = "Renamed the toolpath";
    return out;
}

// --- Summary -------------------------------------------------------------------------------------

ToolpathSession::Stats ToolpathSession::statsOf(const ToolpathRecord& record) {
    toolpath::MachineRates rates;
    rates.rapidRate = setup_.rapidRateMmPerMin > 0.0 ? setup_.rapidRateMmPerMin : 1.0;
    rates.feedRate = record.tool.feedRate;
    rates.plungeRate = record.tool.plungeRate;
    rates.acceleration = setup_.accelerationMmPerS2;
    const auto it = stats_.find(record.moves);
    if (it != stats_.end() && it->second.rates == rates && it->second.rapidSet == (setup_.rapidRateMmPerMin > 0.0)) {
        return it->second;
    }
    const toolpath::Toolpath tp = record.toolpath();
    Stats s;
    s.rates = rates;
    s.rapidSet = setup_.rapidRateMmPerMin > 0.0;
    s.moves = tp.moveCount();
    s.bounds = tp.bounds();
    const toolpath::LengthByKind len = tp.lengthByKind();
    s.length = len.total();
    s.minZ = tp.minZ();
    s.maxZ = tp.maxZ();
    s.time = toolpath::estimateTime(tp, rates);
    if (setup_.rapidRateMmPerMin <= 0.0) {
        s.rapidMissing = len.rapid > 0.0;
        s.time.rapid = 0.0;
    }
    if (record.moves != nullptr) {
        stats_[MovesKey(record.moves)] = s;
    }
    return s;
}

JobSummary ToolpathSession::summary() {
    JobSummary out;
    if (doc_ == nullptr) {
        return out;
    }
    computeStaleness();
    std::set<std::tuple<int, std::string, double>> tools;
    std::optional<std::tuple<int, std::string, double>> previous;
    for (const ToolpathRecord& rec : doc_->toolpaths()) {
        ++out.toolpaths;
        ToolpathSummaryEntry e;
        e.id = rec.id;
        e.name = rec.name;
        e.toolNumber = rec.tool.toolNumber;
        e.toolName = rec.tool.name;
        const Staleness& s = staleness_[rec.id];
        if (rec.hasMoves()) {
            ++out.computed;
            if (s.vectors || s.params || s.tool) {
                ++out.needRecalculation;
            }
            const Stats st = statsOf(rec);
            e.moves = st.moves;
            e.bounds = st.bounds;
            e.lengthMm = st.length;
            e.minZ = st.minZ;
            e.maxZ = st.maxZ;
            e.time = st.time;
            e.included = !disabled_.contains(rec.id);
            if (e.included) {
                ++out.included;
                out.moves += st.moves;
                out.bounds.expand(st.bounds);
                if (st.minZ.has_value()) {
                    out.minZ = out.minZ.has_value() ? std::min(*out.minZ, *st.minZ) : *st.minZ;
                }
                if (st.maxZ.has_value()) {
                    out.maxZ = out.maxZ.has_value() ? std::max(*out.maxZ, *st.maxZ) : *st.maxZ;
                }
                out.time.rapid += st.time.rapid;
                out.time.feed += st.time.feed;
                out.time.plunge += st.time.plunge;
                out.time.retract += st.time.retract;
                out.time.dwell += st.time.dwell;
                out.rapidRateMissing = out.rapidRateMissing || st.rapidMissing;
                auto sig = std::make_tuple(rec.tool.toolNumber, rec.tool.name, rec.tool.diameter);
                if (previous.has_value() && *previous != sig) {
                    ++out.toolChanges;
                }
                tools.insert(sig);
                previous = std::move(sig);
            }
        }
        out.entries.push_back(std::move(e));
    }
    out.distinctTools = tools.size();
    return out;
}

}  // namespace carvetto::viewmodel
