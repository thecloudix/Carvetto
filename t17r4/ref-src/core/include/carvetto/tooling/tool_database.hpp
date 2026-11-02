#pragma once

#include <map>
#include <string>
#include <string_view>
#include <vector>

#include "carvetto/tooling/machine.hpp"
#include "carvetto/tooling/tool.hpp"

namespace carvetto::tooling {

/// A folder in the tool tree. Groups nest through `parentId`; empty = root.
struct ToolGroup {
    /// Stable unique id.
    std::string id;
    std::string name;
    std::string parentId;

    bool operator==(const ToolGroup&) const = default;
};

/// How `ToolDatabase::merge` treats items whose id already exists.
enum class MergePolicy {
    /// Keep the existing item untouched (its cutting data included); only add new ids.
    KeepExisting,
    /// Replace the existing item with the incoming one.
    Overwrite,
};

/// Counts produced by a merge, per item kind.
struct MergeCounts {
    int added{0};
    int replaced{0};
    int kept{0};

    bool operator==(const MergeCounts&) const = default;
};

/// Summary of a `ToolDatabase::merge`.
struct MergeReport {
    MergeCounts tools;
    MergeCounts groups;
    MergeCounts materials;
    MergeCounts machines;

    bool operator==(const MergeReport&) const = default;
};

/// Tools, groups, materials and machines, all keyed by stable string ids and iterated in id
/// order so every operation is deterministic. Ids are chosen by the caller (the UI generates
/// UUID strings); the database never invents ids.
class ToolDatabase {
   public:
    bool operator==(const ToolDatabase&) const = default;

    // ----------------------------------------------------------------- tools
    /// Adds a tool. Fails (returns false) when the id is empty or already taken, or when
    /// `tool.groupId` names a group that does not exist.
    bool addTool(Tool tool);
    /// Replaces the tool with the same id. Fails when absent or the group does not exist.
    bool updateTool(Tool tool);
    /// Removes a tool; returns true when it existed.
    bool removeTool(std::string_view id);
    /// Copies a tool (geometry and all cutting data) under `newId`, named `newName`.
    /// Fails when the source is missing or `newId` is empty/taken.
    bool copyTool(std::string_view sourceId, std::string newId, std::string newName);
    const Tool* findTool(std::string_view id) const;
    const std::map<std::string, Tool>& tools() const { return tools_; }
    /// Tools directly inside `groupId` (empty = root), in id order.
    std::vector<const Tool*> toolsInGroup(std::string_view groupId) const;

    // ---------------------------------------------------------------- groups
    /// Adds a group. Fails when the id is empty/taken or the parent does not exist.
    bool addGroup(ToolGroup group);
    /// Renames a group; fails when absent.
    bool renameGroup(std::string_view id, std::string newName);
    /// Removes a group; its tools and child groups move to its parent. Returns false when absent.
    bool removeGroup(std::string_view id);
    const ToolGroup* findGroup(std::string_view id) const;
    const std::map<std::string, ToolGroup>& groups() const { return groups_; }
    /// Child groups of `parentId` (empty = root), in id order.
    std::vector<const ToolGroup*> childGroups(std::string_view parentId) const;

    // ------------------------------------------------------------- materials
    bool addMaterial(Material material);
    bool renameMaterial(std::string_view id, std::string newName);
    /// Removes a material and every tool's cutting data for it.
    bool removeMaterial(std::string_view id);
    /// Adds a copy named `newName`; when `copyCuttingData`, every tool's entries for the source
    /// material are duplicated for the new one.
    bool duplicateMaterial(std::string_view sourceId, std::string newId, std::string newName,
                           bool copyCuttingData);
    const Material* findMaterial(std::string_view id) const;
    const std::map<std::string, Material>& materials() const { return materials_; }

    // -------------------------------------------------------------- machines
    bool addMachine(Machine machine);
    /// Replaces the machine with the same id; fails when absent.
    bool updateMachine(Machine machine);
    /// Removes a machine. When `deleteCuttingData`, every tool's entries for it are removed too;
    /// otherwise they are kept (orphaned) so re-adding a machine with the same id restores them.
    bool removeMachine(std::string_view id, bool deleteCuttingData);
    /// Adds a copy named `newName`; when `copyCuttingData`, every tool's entries for the source
    /// machine are duplicated for the new one.
    bool duplicateMachine(std::string_view sourceId, std::string newId, std::string newName,
                          bool copyCuttingData);
    const Machine* findMachine(std::string_view id) const;
    const std::map<std::string, Machine>& machines() const { return machines_; }

    // ----------------------------------------------------------------- merge
    /// Merges `other` into this database. Items are matched by id. Groups, materials and
    /// machines are merged before tools so incoming tools can reference them; an incoming tool
    /// whose group is unknown after that lands at the root.
    MergeReport merge(const ToolDatabase& other, MergePolicy policy);

    bool isEmpty() const {
        return tools_.empty() && groups_.empty() && materials_.empty() && machines_.empty();
    }

   private:
    bool groupExistsOrRoot(std::string_view groupId) const;

    std::map<std::string, Tool> tools_;
    std::map<std::string, ToolGroup> groups_;
    std::map<std::string, Material> materials_;
    std::map<std::string, Machine> machines_;
};

}  // namespace carvetto::tooling
