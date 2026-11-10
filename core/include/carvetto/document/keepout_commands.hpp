#pragma once

#include <utility>
#include <vector>

#include "carvetto/document/command.hpp"
#include "carvetto/document/keepout.hpp"
#include "carvetto/geometry/contour.hpp"

namespace carvetto::document {

class Document;

/// The closed contours of the vector objects `ids` (groups contribute their descendants), in
/// object-id then contour order. Open contours, contours of zero area and non-vector objects are
/// skipped; the contours are copied (job coordinates).
std::vector<geometry::Contour> closedContoursOf(const Document& doc, const std::vector<ObjectId>& ids);

/// "Create from Selection": adds one keep-out zone per outline. Only closed outlines of non-zero
/// area are used; the rest are skipped. No-op when none remain, when the zone count or total
/// span count would pass `kMaxKeepOutZones` / `kMaxKeepOutSpans`, or when ids run out. The ids are
/// allocated on the first apply and kept for redo.
class CreateKeepOutZonesCommand : public Command {
   public:
    explicit CreateKeepOutZonesCommand(std::vector<geometry::Contour> outlines) : outlines_(std::move(outlines)) {}
    /// Zones from the closed vectors `ids` of `doc` (see `closedContoursOf`).
    CreateKeepOutZonesCommand(const Document& doc, const std::vector<ObjectId>& ids);
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Create keep-out zones"; }
    /// Ids of the created zones (empty until applied).
    const std::vector<KeepOutZoneId>& createdIds() const { return ids_; }

   private:
    std::vector<geometry::Contour> outlines_;
    std::vector<KeepOutZoneId> ids_;
    bool applied_{false};
};

/// Deletes the zones `ids` (unknown ids are skipped). `RemoveKeepOutZonesCommand::all()` is
/// "Clear Zones".
class RemoveKeepOutZonesCommand : public Command {
   public:
    explicit RemoveKeepOutZonesCommand(std::vector<KeepOutZoneId> ids) : ids_(std::move(ids)) {}
    /// Clears every zone present when the command is applied.
    static RemoveKeepOutZonesCommand all() {
        RemoveKeepOutZonesCommand c({});
        c.all_ = true;
        return c;
    }
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return all_ ? "Clear keep-out zones" : "Delete keep-out zones"; }

   private:
    std::vector<KeepOutZoneId> ids_;
    bool all_{false};
    /// Removed zones with their former index, in removal order (undone in reverse).
    std::vector<std::pair<std::size_t, KeepOutZone>> removed_;
};

/// Sets the clearance (mm). No-op for a non-finite, negative or > `kMaxKeepOutClearanceMm` value
/// or when unchanged. Consecutive edits coalesce.
class SetKeepOutClearanceCommand : public Command {
   public:
    explicit SetKeepOutClearanceCommand(double mm) : after_(mm) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Set keep-out clearance"; }
    bool mergeWith(const Command& next) override;

   private:
    double before_{0.0};
    double after_{0.0};
};

}  // namespace carvetto::document
