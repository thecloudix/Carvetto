#include "carvetto/tooling/tool_database.hpp"

#include <utility>

namespace carvetto::tooling {

namespace {

template <class Map>
auto findIn(const Map& map, std::string_view id) -> const typename Map::mapped_type* {
    const auto it = map.find(std::string(id));
    return it == map.end() ? nullptr : &it->second;
}

/// Duplicates every cutting-data entry whose key component `select` matches `sourceId` under
/// `newId`. `select` picks machineId or materialId from a key.
template <class Select>
void copyCuttingDataEntries(std::map<std::string, Tool>& tools, std::string_view sourceId,
                            const std::string& newId, Select select) {
    for (auto& [id, tool] : tools) {
        std::vector<std::pair<CuttingDataKey, CuttingData>> copies;
        for (const auto& [key, data] : tool.cuttingData) {
            if (select(key) == sourceId) {
                CuttingDataKey k = key;
                select(k) = newId;
                copies.emplace_back(std::move(k), data);
            }
        }
        for (auto& [k, data] : copies) tool.cuttingData[std::move(k)] = data;
    }
}

template <class Select>
void eraseCuttingDataEntries(std::map<std::string, Tool>& tools, std::string_view id, Select select) {
    for (auto& [toolId, tool] : tools) {
        for (auto it = tool.cuttingData.begin(); it != tool.cuttingData.end();) {
            if (select(it->first) == id) {
                it = tool.cuttingData.erase(it);
            } else {
                ++it;
            }
        }
    }
}

auto byMachine = [](auto& key) -> auto& { return key.machineId; };
auto byMaterial = [](auto& key) -> auto& { return key.materialId; };

template <class Map, class Item>
void mergeMap(Map& into, const Map& from, MergePolicy policy, MergeCounts& counts, Item&& adjust) {
    for (const auto& [id, item] : from) {
        auto it = into.find(id);
        if (it == into.end()) {
            auto copy = item;
            adjust(copy);
            into.emplace(id, std::move(copy));
            ++counts.added;
        } else if (policy == MergePolicy::Overwrite) {
            auto copy = item;
            adjust(copy);
            it->second = std::move(copy);
            ++counts.replaced;
        } else {
            ++counts.kept;
        }
    }
}

}  // namespace

bool ToolDatabase::groupExistsOrRoot(std::string_view groupId) const {
    return groupId.empty() || groups_.contains(std::string(groupId));
}

// ------------------------------------------------------------------- tools

bool ToolDatabase::addTool(Tool tool) {
    if (tool.id.empty() || tools_.contains(tool.id)) return false;
    if (!groupExistsOrRoot(tool.groupId)) return false;
    tools_.emplace(tool.id, std::move(tool));
    return true;
}

bool ToolDatabase::updateTool(Tool tool) {
    auto it = tools_.find(tool.id);
    if (it == tools_.end()) return false;
    if (!groupExistsOrRoot(tool.groupId)) return false;
    it->second = std::move(tool);
    return true;
}

bool ToolDatabase::removeTool(std::string_view id) {
    return tools_.erase(std::string(id)) > 0;
}

bool ToolDatabase::copyTool(std::string_view sourceId, std::string newId, std::string newName) {
    const Tool* source = findTool(sourceId);
    if (source == nullptr || newId.empty() || tools_.contains(newId)) return false;
    Tool copy = *source;
    copy.id = std::move(newId);
    copy.name = std::move(newName);
    tools_.emplace(copy.id, std::move(copy));
    return true;
}

const Tool* ToolDatabase::findTool(std::string_view id) const {
    return findIn(tools_, id);
}

std::vector<const Tool*> ToolDatabase::toolsInGroup(std::string_view groupId) const {
    std::vector<const Tool*> out;
    for (const auto& [id, tool] : tools_) {
        if (tool.groupId == groupId) out.push_back(&tool);
    }
    return out;
}

// ------------------------------------------------------------------ groups

bool ToolDatabase::addGroup(ToolGroup group) {
    if (group.id.empty() || groups_.contains(group.id)) return false;
    if (!groupExistsOrRoot(group.parentId)) return false;
    groups_.emplace(group.id, std::move(group));
    return true;
}

bool ToolDatabase::renameGroup(std::string_view id, std::string newName) {
    auto it = groups_.find(std::string(id));
    if (it == groups_.end()) return false;
    it->second.name = std::move(newName);
    return true;
}

bool ToolDatabase::removeGroup(std::string_view id) {
    auto it = groups_.find(std::string(id));
    if (it == groups_.end()) return false;
    const std::string parent = it->second.parentId;
    for (auto& [toolId, tool] : tools_) {
        if (tool.groupId == id) tool.groupId = parent;
    }
    for (auto& [groupId, group] : groups_) {
        if (group.parentId == id) group.parentId = parent;
    }
    groups_.erase(it);
    return true;
}

const ToolGroup* ToolDatabase::findGroup(std::string_view id) const {
    return findIn(groups_, id);
}

std::vector<const ToolGroup*> ToolDatabase::childGroups(std::string_view parentId) const {
    std::vector<const ToolGroup*> out;
    for (const auto& [id, group] : groups_) {
        if (group.parentId == parentId) out.push_back(&group);
    }
    return out;
}

// --------------------------------------------------------------- materials

bool ToolDatabase::addMaterial(Material material) {
    if (material.id.empty() || materials_.contains(material.id)) return false;
    materials_.emplace(material.id, std::move(material));
    return true;
}

bool ToolDatabase::renameMaterial(std::string_view id, std::string newName) {
    auto it = materials_.find(std::string(id));
    if (it == materials_.end()) return false;
    it->second.name = std::move(newName);
    return true;
}

bool ToolDatabase::removeMaterial(std::string_view id) {
    if (materials_.erase(std::string(id)) == 0) return false;
    eraseCuttingDataEntries(tools_, id, byMaterial);
    return true;
}

bool ToolDatabase::duplicateMaterial(std::string_view sourceId, std::string newId, std::string newName,
                                     bool copyCuttingData) {
    if (findMaterial(sourceId) == nullptr || newId.empty() || materials_.contains(newId)) return false;
    Material m;
    m.id = newId;
    m.name = std::move(newName);
    materials_.emplace(newId, std::move(m));
    if (copyCuttingData) copyCuttingDataEntries(tools_, sourceId, newId, byMaterial);
    return true;
}

const Material* ToolDatabase::findMaterial(std::string_view id) const {
    return findIn(materials_, id);
}

// ---------------------------------------------------------------- machines

bool ToolDatabase::addMachine(Machine machine) {
    if (machine.id.empty() || machines_.contains(machine.id)) return false;
    machines_.emplace(machine.id, std::move(machine));
    return true;
}

bool ToolDatabase::updateMachine(Machine machine) {
    auto it = machines_.find(machine.id);
    if (it == machines_.end()) return false;
    it->second = std::move(machine);
    return true;
}

bool ToolDatabase::removeMachine(std::string_view id, bool deleteCuttingData) {
    if (machines_.erase(std::string(id)) == 0) return false;
    if (deleteCuttingData) eraseCuttingDataEntries(tools_, id, byMachine);
    return true;
}

bool ToolDatabase::duplicateMachine(std::string_view sourceId, std::string newId, std::string newName,
                                    bool copyCuttingData) {
    const Machine* source = findMachine(sourceId);
    if (source == nullptr || newId.empty() || machines_.contains(newId)) return false;
    Machine m = *source;
    m.id = newId;
    m.name = std::move(newName);
    machines_.emplace(newId, std::move(m));
    if (copyCuttingData) copyCuttingDataEntries(tools_, sourceId, newId, byMachine);
    return true;
}

const Machine* ToolDatabase::findMachine(std::string_view id) const {
    return findIn(machines_, id);
}

// ------------------------------------------------------------------- merge

MergeReport ToolDatabase::merge(const ToolDatabase& other, MergePolicy policy) {
    MergeReport report;
    const auto keep = [](auto&) {};
    mergeMap(groups_, other.groups_, policy, report.groups, keep);
    // A group whose parent is still unknown (e.g. KeepExisting skipped a conflicting parent chain)
    // is re-homed at the root so the tree stays consistent.
    for (auto& [id, group] : groups_) {
        if (!groupExistsOrRoot(group.parentId)) group.parentId.clear();
    }
    // Break parent cycles that an incoming file could introduce: walk up from each group and cut
    // the link that returns to the start.
    for (auto& [id, group] : groups_) {
        std::string cursor = group.parentId;
        std::size_t steps = 0;
        while (!cursor.empty() && steps++ <= groups_.size()) {
            if (cursor == id) {
                group.parentId.clear();
                break;
            }
            cursor = groups_.at(cursor).parentId;
        }
    }
    mergeMap(materials_, other.materials_, policy, report.materials, keep);
    mergeMap(machines_, other.machines_, policy, report.machines, keep);
    mergeMap(tools_, other.tools_, policy, report.tools, [this](Tool& tool) {
        if (!groupExistsOrRoot(tool.groupId)) tool.groupId.clear();
    });
    return report;
}

}  // namespace carvetto::tooling
