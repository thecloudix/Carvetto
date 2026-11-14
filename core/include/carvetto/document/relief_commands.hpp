#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/document/command.hpp"
#include "carvetto/document/relief_model.hpp"

namespace carvetto::document {

class Document;

// Undoable relief edits (T-0282, ADR-0061). Every command is a no-op (apply returns false) for an
// unknown id, an unchanged value, a violated cap or an exhausted id generator. Ids are allocated
// on the first apply and kept, so redo restores the same identities. Selection is not undone.

/// Adds an empty level on `sheet` (default: on top of its levels).
class AddReliefLevelCommand : public Command {
   public:
    AddReliefLevelCommand(SheetId sheet, std::string name, std::optional<std::size_t> indexOnSheet = std::nullopt)
        : sheet_(sheet), name_(std::move(name)), index_(indexOnSheet) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Add relief level"; }
    LevelId createdId() const { return id_; }

   private:
    SheetId sheet_;
    std::string name_;
    std::optional<std::size_t> index_;
    LevelId id_;
};

/// Adds a component to a level (default: on top).
class AddReliefComponentCommand : public Command {
   public:
    AddReliefComponentCommand(LevelId level, relief::Component component,
                              std::optional<std::size_t> index = std::nullopt)
        : level_(level), held_{ComponentId{}, std::move(component)}, index_(index) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Add relief component"; }
    ComponentId createdId() const { return held_.id; }

   private:
    LevelId level_;
    ReliefComponent held_;  ///< The component while it is not in the document.
    std::optional<std::size_t> index_;
};

/// Deletes components (unknown ids are skipped).
class DeleteReliefComponentsCommand : public Command {
   public:
    explicit DeleteReliefComponentsCommand(std::vector<ComponentId> ids) : ids_(std::move(ids)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Delete relief components"; }

   private:
    struct Removed {
        LevelId level;
        std::size_t index;
        ReliefComponent component;
    };
    std::vector<ComponentId> ids_;
    std::vector<Removed> removed_;  ///< In removal order; undone in reverse.
};

/// Deletes a level with its components.
class DeleteReliefLevelCommand : public Command {
   public:
    explicit DeleteReliefLevelCommand(LevelId id) : id_(id) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Delete relief level"; }

   private:
    LevelId id_;
    std::size_t index_{0};
    std::optional<ReliefLevel> removed_;
};

/// Renames a component (non-empty, at most `kMaxReliefNameBytes`).
class RenameReliefComponentCommand : public Command {
   public:
    RenameReliefComponentCommand(ComponentId id, std::string name) : id_(id), after_(std::move(name)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Rename relief component"; }

   private:
    ComponentId id_;
    std::string before_;
    std::string after_;
};

/// Renames a level.
class RenameReliefLevelCommand : public Command {
   public:
    RenameReliefLevelCommand(LevelId id, std::string name) : id_(id), after_(std::move(name)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Rename relief level"; }

   private:
    LevelId id_;
    std::string before_;
    std::string after_;
};

/// Sets how a component combines with what lies below it in its level.
class SetReliefComponentModeCommand : public Command {
   public:
    SetReliefComponentModeCommand(ComponentId id, relief::CombineMode mode) : id_(id), after_(mode) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Set combine mode"; }

   private:
    ComponentId id_;
    relief::CombineMode before_{relief::CombineMode::Add};
    relief::CombineMode after_;
};

/// The editable dynamic properties of a component.
struct ReliefComponentProperties {
    double shapeHeightPercent{100.0};
    double baseHeight{0.0};
    relief::Fade fade;
    relief::Tilt tilt;
    bool visible{true};
};

/// Sets a component's shape height, base height, fade, tilt and visibility. Non-finite values and
/// degenerate fade/tilt directions are refused. Consecutive edits of one component coalesce.
class SetReliefComponentPropertiesCommand : public Command {
   public:
    SetReliefComponentPropertiesCommand(ComponentId id, ReliefComponentProperties props)
        : id_(id), after_(props) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Edit relief component"; }
    bool mergeWith(const Command& next) override;

   private:
    ComponentId id_;
    ReliefComponentProperties before_;
    ReliefComponentProperties after_;
};

/// Sets a level's combine mode, base height and visibility. Consecutive edits coalesce.
class SetReliefLevelPropertiesCommand : public Command {
   public:
    SetReliefLevelPropertiesCommand(LevelId id, relief::CombineMode mode, double baseHeight, bool visible)
        : id_(id), after_{mode, baseHeight, visible} {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Edit relief level"; }
    bool mergeWith(const Command& next) override;

   private:
    struct Props {
        relief::CombineMode mode{relief::CombineMode::Add};
        double baseHeight{0.0};
        bool visible{true};
    };
    LevelId id_;
    Props before_;
    Props after_;
};

/// Moves a component to `newIndex` within its level (0 is the bottom).
class ReorderReliefComponentCommand : public Command {
   public:
    ReorderReliefComponentCommand(ComponentId id, std::size_t newIndex) : id_(id), after_(newIndex) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Reorder relief component"; }

   private:
    ComponentId id_;
    std::size_t before_{0};
    std::size_t after_;
};

/// Moves a level to `newIndex` among its sheet's levels (0 is the bottom).
class ReorderReliefLevelCommand : public Command {
   public:
    ReorderReliefLevelCommand(LevelId id, std::size_t newIndex) : id_(id), after_(newIndex) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Reorder relief level"; }

   private:
    LevelId id_;
    std::size_t before_{0};
    std::size_t after_;
};

/// Moves a component into another level of the same sheet (default: on top). A move within the
/// same level is `ReorderReliefComponentCommand`'s job and is refused here.
class MoveReliefComponentToLevelCommand : public Command {
   public:
    MoveReliefComponentToLevelCommand(ComponentId id, LevelId target,
                                      std::optional<std::size_t> index = std::nullopt)
        : id_(id), target_(target), index_(index) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Move component to level"; }

   private:
    ComponentId id_;
    LevelId target_;
    std::optional<std::size_t> index_;
    LevelId fromLevel_;
    std::size_t fromIndex_{0};
};

/// Replaces a component's data (field, name, mode and dynamic properties) in place, keeping its id,
/// level, position and visibility slot. This is how the modelling dialogs (T-0300) apply a modify
/// or scale on an existing component and re-apply a regenerated one. Refused for an unknown id, an
/// unstorable component, an unchanged field/props or when the document's cell cap would be exceeded.
class ReplaceReliefComponentCommand : public Command {
   public:
    ReplaceReliefComponentCommand(ComponentId id, relief::Component component)
        : id_(id), held_(std::move(component)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Replace relief component"; }

   private:
    ComponentId id_;
    relief::Component held_;  ///< The data that is NOT currently in the document.
};

/// Duplicates a component into the same level, directly above the original, named "<name> copy".
class DuplicateReliefComponentCommand : public Command {
   public:
    explicit DuplicateReliefComponentCommand(ComponentId id) : id_(id) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Duplicate relief component"; }
    ComponentId createdId() const { return copy_; }

   private:
    ComponentId id_;
    ComponentId copy_;
    bool applied_{false};
    std::optional<ReliefComponent> held_;  ///< The copy while it is not in the document.
    LevelId level_;
    std::size_t index_{0};
};

}  // namespace carvetto::document
