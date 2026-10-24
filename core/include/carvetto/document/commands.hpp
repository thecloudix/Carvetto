#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/document/command.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/document/layer.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/document/sheet.hpp"
#include "carvetto/geometry/transform2.hpp"
#include "carvetto/geometry/vec2.hpp"

/// Concrete undoable edits on a `Document`. Commands that create things allocate their ids on
/// the first `apply` and reuse them on redo, so identity survives undo/redo. Commands that change
/// object structure (remove, group, ungroup, delete layer/sheet) record every touched object in an
/// `ObjectChangeSet` and restore those snapshots verbatim on undo and redo.
///
/// Group rules shared by every command (ADR-0007):
/// - A group's `children` and each child's `parent` always agree; children share the group's sheet.
/// - Removing an object detaches it from its parent group. A group left with no children is
///   removed as well (recursively up the chain); a group left with one child is kept.
/// - Removing a group removes its descendants, except where a command says it dissolves the group
///   instead (deleting a layer or sheet that holds the group but not all of its children): then
///   the surviving children are spliced into the group's parent at the group's position, or become
///   top-level objects when the group was top-level.
/// - Moving a group to another layer or sheet moves its descendants with it.
namespace carvetto::document {

/// Before/after snapshots of the objects one command touched. `recordBefore` is called before an
/// object is first modified, created or removed; `captureAfter` at the end of `apply`. `revert`
/// and `reapply` then restore either side exactly, independent of how the edit was computed.
class ObjectChangeSet {
   public:
    /// Snapshots `id` (or its absence) unless it was already recorded.
    void recordBefore(const Document& doc, ObjectId id);
    /// Snapshots the current state of every recorded id.
    void captureAfter(const Document& doc);
    /// Restores the before-states (absent objects are removed, present ones restored).
    void revert(Document& doc) const;
    /// Restores the after-states.
    void reapply(Document& doc) const;
    bool empty() const { return order_.empty(); }
    /// True when at least one recorded object differs between before and after.
    bool changed() const;
    void clear();

   private:
    static void restore(Document& doc, ObjectId id, const std::optional<Object>& state);

    std::vector<ObjectId> order_;
    std::map<ObjectId, std::optional<Object>> before_;
    std::map<ObjectId, std::optional<Object>> after_;
};

// --- Objects -----------------------------------------------------------------------------------

/// Adds one object. `obj.id` may be null (allocated on first apply) or a fresh unused id. When
/// `obj.parent` names an existing group the object is appended to that group's children (and
/// takes the group's sheet); any other parent is cleared.
class AddObjectCommand : public Command {
   public:
    explicit AddObjectCommand(Object obj) : object_(std::move(obj)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Add object"; }
    /// Id of the added object (null until applied).
    ObjectId id() const { return object_.id; }

   private:
    Object object_;
    ObjectChangeSet changes_;
};

/// Removes objects (groups take their descendants with them). Unknown ids are skipped.
class RemoveObjectsCommand : public Command {
   public:
    explicit RemoveObjectsCommand(std::vector<ObjectId> ids) : ids_(std::move(ids)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Delete objects"; }

   private:
    std::vector<ObjectId> ids_;
    ObjectChangeSet changes_;
};

/// Moves objects (and the descendants of groups among them) to another layer.
class MoveObjectsToLayerCommand : public Command {
   public:
    MoveObjectsToLayerCommand(std::vector<ObjectId> ids, LayerId target)
        : ids_(std::move(ids)), target_(target) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Move to layer"; }

   private:
    std::vector<ObjectId> ids_;
    LayerId target_;
    ObjectChangeSet changes_;
};

/// Moves objects (and the descendants of groups among them) to another sheet. An object whose
/// parent group stays behind is detached from that group first, since a group never spans sheets.
class MoveObjectsToSheetCommand : public Command {
   public:
    MoveObjectsToSheetCommand(std::vector<ObjectId> ids, SheetId target)
        : ids_(std::move(ids)), target_(target) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Move to sheet"; }

   private:
    std::vector<ObjectId> ids_;
    SheetId target_;
    ObjectChangeSet changes_;
};

/// Applies an affine transform to objects. The pre-transform payloads are snapshotted so undo is
/// exact even when arcs turn into Beziers. Consecutive transforms of the same id set coalesce
/// into one step (a drag), the snapshot of the first being kept.
class TransformObjectsCommand : public Command {
   public:
    TransformObjectsCommand(std::vector<ObjectId> ids, geometry::Transform2 m)
        : ids_(std::move(ids)), transform_(m) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Transform objects"; }
    bool mergeWith(const Command& next) override;
    const geometry::Transform2& transform() const { return transform_; }

   private:
    std::vector<ObjectId> ids_;
    geometry::Transform2 transform_;
    std::vector<std::pair<ObjectId, ObjectData>> before_;
    std::vector<std::pair<ObjectId, ObjectData>> after_;
};

/// Inserts copies of clipboard objects (as produced by `Document::copyObjects`) with fresh ids,
/// optionally shifted, onto `layer`/`sheet` (null = keep each copy's own). Group links inside the
/// clipboard are remapped to the new ids; links to objects outside it are dropped.
class PasteObjectsCommand : public Command {
   public:
    PasteObjectsCommand(std::vector<Object> clipboard, geometry::Vec2 offsetMm = {}, LayerId layer = {},
                        SheetId sheet = {})
        : clipboard_(std::move(clipboard)), offset_(offsetMm), layer_(layer), sheet_(sheet) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Paste"; }
    /// Ids of the pasted objects (empty until applied), in clipboard order.
    const std::vector<ObjectId>& pastedIds() const { return pastedIds_; }

   private:
    std::vector<Object> clipboard_;
    geometry::Vec2 offset_;
    LayerId layer_;
    SheetId sheet_;
    std::vector<Object> pasted_;
    std::vector<ObjectId> pastedIds_;
};

/// Groups top-level objects into a new `GroupObject` on the layer/sheet of the first member.
/// Members on other sheets than the first are ignored; nested objects cannot be grouped.
class GroupObjectsCommand : public Command {
   public:
    explicit GroupObjectsCommand(std::vector<ObjectId> ids) : ids_(std::move(ids)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Group"; }
    ObjectId groupId() const { return groupId_; }

   private:
    std::vector<ObjectId> ids_;
    ObjectId groupId_;
    ObjectChangeSet changes_;
};

/// Dissolves a group: its children take its place in its parent group (or become top-level),
/// optionally moving to the group's layer.
class UngroupCommand : public Command {
   public:
    explicit UngroupCommand(ObjectId group, bool toGroupLayer = false)
        : groupId_(group), toGroupLayer_(toGroupLayer) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Ungroup"; }

   private:
    ObjectId groupId_;
    bool toGroupLayer_;
    ObjectChangeSet changes_;
};

// --- Layers ------------------------------------------------------------------------------------

/// Adds a layer (id allocated on first apply) at `index` or on top; optionally makes it active.
class AddLayerCommand : public Command {
   public:
    AddLayerCommand(Layer layer, bool makeActive = false, std::optional<std::size_t> index = std::nullopt)
        : layer_(std::move(layer)), makeActive_(makeActive), index_(index) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Add layer"; }
    LayerId id() const { return layer_.id; }

   private:
    Layer layer_;
    bool makeActive_;
    std::optional<std::size_t> index_;
    LayerId previousActive_;
};

/// Deletes a layer. With `deleteContents` its objects are deleted: groups on the layer are
/// dissolved (children on other layers survive, see the group rules above) and groups elsewhere
/// lose the children that were on it. Otherwise the objects move to the default layer (the first
/// remaining layer). The last layer cannot be deleted (no-op).
class DeleteLayerCommand : public Command {
   public:
    DeleteLayerCommand(LayerId id, bool deleteContents) : id_(id), deleteContents_(deleteContents) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Delete layer"; }

   private:
    LayerId id_;
    bool deleteContents_;
    Layer layer_;
    std::size_t index_{0};
    LayerId previousActive_;
    ObjectChangeSet changes_;
};

/// Replaces a layer's properties (name, colour, visibility, lock). Consecutive edits of the same
/// layer coalesce.
class SetLayerPropertiesCommand : public Command {
   public:
    explicit SetLayerPropertiesCommand(Layer layer) : after_(std::move(layer)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Edit layer"; }
    bool mergeWith(const Command& next) override;

   private:
    Layer after_;
    Layer before_;
};

/// Moves a layer within the stacking order.
class MoveLayerCommand : public Command {
   public:
    MoveLayerCommand(LayerId id, std::size_t newIndex) : id_(id), newIndex_(newIndex) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Reorder layers"; }

   private:
    LayerId id_;
    std::size_t newIndex_;
    std::size_t oldIndex_{0};
};

// --- Sheets ------------------------------------------------------------------------------------

/// Adds a sheet (id allocated on first apply); optionally makes it active.
class AddSheetCommand : public Command {
   public:
    explicit AddSheetCommand(Sheet sheet, bool makeActive = false)
        : sheet_(std::move(sheet)), makeActive_(makeActive) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Add sheet"; }
    SheetId id() const { return sheet_.id; }

   private:
    Sheet sheet_;
    bool makeActive_;
    SheetId previousActive_;
};

/// Deletes a sheet together with the objects on it. The last sheet cannot be deleted (no-op).
/// (What happens to the sheet's objects is pending an owner decision; it is confined to
/// `DeleteSheetCommand::apply`.)
class DeleteSheetCommand : public Command {
   public:
    explicit DeleteSheetCommand(SheetId id) : id_(id) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Delete sheet"; }

   private:
    SheetId id_;
    Sheet sheet_;
    std::size_t index_{0};
    SheetId previousActive_;
    ObjectChangeSet changes_;
};

/// Replaces a sheet's properties. Consecutive edits of the same sheet coalesce.
class SetSheetPropertiesCommand : public Command {
   public:
    explicit SetSheetPropertiesCommand(Sheet sheet) : after_(std::move(sheet)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Edit sheet"; }
    bool mergeWith(const Command& next) override;

   private:
    Sheet after_;
    Sheet before_;
};

// --- Job and variables -------------------------------------------------------------------------

/// Replaces the job setup. Consecutive job edits coalesce (one form session = one undo step).
class SetJobCommand : public Command {
   public:
    explicit SetJobCommand(Job job) : after_(job) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Job setup"; }
    bool mergeWith(const Command& next) override;

   private:
    Job after_;
    Job before_;
};

/// Creates or changes a document variable. No-op for an invalid name. Consecutive edits of the
/// same variable coalesce.
class SetDocumentVariableCommand : public Command {
   public:
    SetDocumentVariableCommand(std::string name, double value) : name_(std::move(name)), value_(value) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Set variable"; }
    bool mergeWith(const Command& next) override;

   private:
    std::string name_;
    double value_;
    std::optional<double> previous_;
};

/// Removes a document variable. No-op when undefined or still referenced by a toolpath.
class RemoveDocumentVariableCommand : public Command {
   public:
    explicit RemoveDocumentVariableCommand(std::string name) : name_(std::move(name)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Remove variable"; }

   private:
    std::string name_;
    double previous_{0.0};
};

}  // namespace carvetto::document
