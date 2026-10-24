#pragma once

#include <cstddef>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "carvetto/document/command.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/document/layer.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/document/selection.hpp"
#include "carvetto/document/sheet.hpp"
#include "carvetto/document/undo_stack.hpp"
#include "carvetto/document/variables.hpp"
#include "carvetto/geometry/box2.hpp"

namespace carvetto::document {

/// One structural inconsistency found by `Document::checkIntegrity()`.
struct IntegrityIssue {
    /// What is wrong.
    enum class Kind {
        DanglingChild,        ///< A group lists a child id that does not exist.
        ChildParentMismatch,  ///< A group lists a child whose `parent` is not that group.
        DuplicateChild,       ///< A group lists the same child twice, or a child is in two groups.
        ChildNotListed,       ///< An object's `parent` group does not list it.
        BadParent,            ///< An object's `parent` is missing or is not a group.
        SheetMismatch,        ///< A child is on a different sheet than its parent group.
        Cycle,                ///< Following `parent` links loops.
        MissingLayer,         ///< An object's layer does not exist.
        MissingSheet,         ///< An object's sheet does not exist.
        BadActiveLayer,       ///< The active layer does not exist.
        BadActiveSheet,       ///< The active sheet does not exist.
        StaleSelection,       ///< The selection holds an id that does not exist.
        IdOutOfRange,         ///< A sheet, layer or object id is null or >= `IdGenerator::kMaxId`.
        IdNotBelowNext,       ///< The id generator's next id is <= an existing id (could reuse it).
    };
    Kind kind;
    /// The object (or zero) the issue is about. Zero for id issues on a layer or sheet; the
    /// message names the kind and value.
    ObjectId object;
    /// Second object involved (the child or parent), or zero.
    ObjectId related;
    /// Human-readable description for test output and logs.
    std::string message;
};

/// The in-memory design: one job, its sheets, ordered layers, objects, selection, document
/// variables and undo history.
///
/// Two levels of mutation exist. The *raw* mutators (`insertObject`, `takeObject`, `insertLayer`,
/// `takeLayer`, …) change state directly and are what `Command`s are built from. User-level edits
/// go through `execute()` with a command from `commands.hpp` so they are undoable. Objects are
/// stored in a map ordered by id; since ids are allocated monotonically this is creation order,
/// which doubles as the deterministic stacking order within a layer.
class Document {
   public:
    /// Document with the default job, one sheet "Sheet 1" and one layer "Layer 1".
    Document();
    /// Document around `job`, with one sheet sized from it and one layer.
    explicit Document(Job job);

    Document(const Document&) = delete;
    Document& operator=(const Document&) = delete;
    Document(Document&&) = default;
    Document& operator=(Document&&) = default;

    // --- Job -------------------------------------------------------------------------------
    const Job& job() const { return job_; }
    /// Raw setter (not undoable; see `SetJobCommand`).
    void setJob(const Job& job) { job_ = job; }

    // --- Sheets ----------------------------------------------------------------------------
    const std::vector<Sheet>& sheets() const { return sheets_; }
    const Sheet* findSheet(SheetId id) const;
    Sheet* findSheet(SheetId id);
    std::optional<std::size_t> sheetIndex(SheetId id) const;
    SheetId activeSheet() const { return activeSheet_; }
    /// Returns false when `id` is not a sheet.
    bool setActiveSheet(SheetId id);
    /// Raw: inserts `sheet` (allocating an id when null) at `index` or at the end. Returns the id.
    /// With a null id and an exhausted generator: asserts (Debug), inserts nothing and returns a
    /// null id (Release).
    SheetId insertSheet(Sheet sheet, std::optional<std::size_t> index = std::nullopt);
    /// Raw: removes a sheet. Fails (nullopt) when it is the last sheet or still has objects.
    /// The active sheet falls back to the first remaining sheet.
    std::optional<Sheet> takeSheet(SheetId id);
    /// Ids of objects on `sheet`, in id order.
    std::vector<ObjectId> objectsOnSheet(SheetId sheet) const;

    // --- Layers ----------------------------------------------------------------------------
    /// Layers in stacking order (index 0 is the bottom and the default layer).
    const std::vector<Layer>& layers() const { return layers_; }
    const Layer* findLayer(LayerId id) const;
    Layer* findLayer(LayerId id);
    std::optional<std::size_t> layerIndex(LayerId id) const;
    /// The layer objects fall back to when theirs is deleted: the first layer in the order.
    LayerId defaultLayer() const { return layers_.front().id; }
    LayerId activeLayer() const { return activeLayer_; }
    /// Returns false when `id` is not a layer.
    bool setActiveLayer(LayerId id);
    /// Raw: inserts `layer` (allocating an id when null) at `index` or on top. Returns the id.
    /// Exhausted generator: as `insertSheet`.
    LayerId insertLayer(Layer layer, std::optional<std::size_t> index = std::nullopt);
    /// Raw: removes a layer. Fails (nullopt) when it is the last layer or still has objects.
    /// The active layer falls back to the default layer.
    std::optional<Layer> takeLayer(LayerId id);
    /// Raw: moves a layer to `newIndex` in the stacking order. Returns false on a bad id/index.
    bool moveLayer(LayerId id, std::size_t newIndex);
    /// Ids of objects on `layer`, in id order.
    std::vector<ObjectId> objectsOnLayer(LayerId layer) const;
    bool layerIsEmpty(LayerId layer) const;

    // --- Objects ---------------------------------------------------------------------------
    std::size_t objectCount() const { return objects_.size(); }
    const Object* findObject(ObjectId id) const;
    Object* findObject(ObjectId id);
    bool hasObject(ObjectId id) const { return objects_.contains(id); }
    /// All object ids in id (creation) order.
    std::vector<ObjectId> objectIds() const;
    const std::map<ObjectId, Object>& objects() const { return objects_; }
    /// Raw: inserts `obj`, allocating an id when `obj.id` is null. A non-null id must be unused
    /// and previously allocated by this document (undo/redo re-inserting). Returns the id.
    /// Exhausted generator: as `insertSheet`.
    ObjectId insertObject(Object obj);
    /// Raw: removes an object (and deselects it). Nullopt when absent.
    std::optional<Object> takeObject(ObjectId id);
    /// Bounds of an object: contours, bitmap placement, or union over a group's children.
    std::optional<geometry::Box2> objectBounds(ObjectId id) const;
    /// Deep copies of `ids` plus every descendant of groups among them, in id order, for the
    /// clipboard. Ids are kept so `PasteObjectsCommand` can remap group links.
    std::vector<Object> copyObjects(const std::vector<ObjectId>& ids) const;
    /// Ids plus every descendant of groups among them, in id order; unknown ids are skipped.
    std::vector<ObjectId> withDescendants(const std::vector<ObjectId>& ids) const;

    /// Lists every structural inconsistency (group links, missing layers/sheets, cycles, stale
    /// active layer/sheet or selection, ids outside `[1, IdGenerator::kMaxId)` or not below the
    /// generator's next id). Empty for a consistent document; every command keeps it so.
    std::vector<IntegrityIssue> checkIntegrity() const;

    // --- Ids -------------------------------------------------------------------------------
    // Each returns a null id when the generator is exhausted (and asserts in Debug); commands
    // check `idGenerator().canAllocate(n)` first and refuse (no-op) instead.
    ObjectId allocateObjectId() { return ObjectId{ids_.allocate()}; }
    LayerId allocateLayerId() { return LayerId{ids_.allocate()}; }
    SheetId allocateSheetId() { return SheetId{ids_.allocate()}; }
    const IdGenerator& idGenerator() const { return ids_; }

    // --- Selection, variables, undo --------------------------------------------------------
    Selection& selection() { return selection_; }
    const Selection& selection() const { return selection_; }
    DocumentVariables& variables() { return variables_; }
    const DocumentVariables& variables() const { return variables_; }
    UndoStack& undoStack() { return undo_; }
    const UndoStack& undoStack() const { return undo_; }

    /// Applies `cmd` through the undo stack. Returns false when it was a no-op.
    bool execute(std::unique_ptr<Command> cmd) { return undo_.push(std::move(cmd), *this); }
    bool undo() { return undo_.undo(*this); }
    bool redo() { return undo_.redo(*this); }

   private:
    Job job_;
    std::vector<Sheet> sheets_;
    SheetId activeSheet_;
    std::vector<Layer> layers_;
    LayerId activeLayer_;
    std::map<ObjectId, Object> objects_;
    Selection selection_;
    DocumentVariables variables_;
    UndoStack undo_;
    IdGenerator ids_;
};

}  // namespace carvetto::document
