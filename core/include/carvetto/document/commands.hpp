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
/// the first `apply` and reuse them on redo, so identity survives undo/redo. Commands that remove
/// things keep the removed state and put it back verbatim on `revert`.
namespace carvetto::document {

// --- Objects -----------------------------------------------------------------------------------

/// Adds one object. `obj.id` may be null (allocated on first apply) or a fresh unused id.
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
    std::vector<Object> removed_;
    std::vector<std::pair<ObjectId, std::vector<ObjectId>>> detachedFromGroups_;
};

/// Moves objects to another layer, remembering where each came from.
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
    std::vector<std::pair<ObjectId, LayerId>> previous_;
};

/// Moves objects to another sheet, remembering where each came from.
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
    std::vector<std::pair<ObjectId, SheetId>> previous_;
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
/// clipboard are remapped to the new ids.
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
class GroupObjectsCommand : public Command {
   public:
    explicit GroupObjectsCommand(std::vector<ObjectId> ids) : ids_(std::move(ids)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Group"; }
    ObjectId groupId() const { return group_.id; }

   private:
    std::vector<ObjectId> ids_;
    Object group_;
};

/// Dissolves a group: children become top-level objects, optionally moved to the group's layer.
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
    Object group_;
    std::vector<std::pair<ObjectId, LayerId>> previousLayers_;
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

/// Deletes a layer. Its objects are deleted when `deleteContents` is set, otherwise moved to the
/// default layer (the first remaining layer). The last layer cannot be deleted (no-op).
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
    std::vector<Object> removed_;
    std::vector<ObjectId> moved_;
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
    std::vector<Object> removed_;
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
