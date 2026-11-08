#pragma once

#include <string>
#include <utility>
#include <vector>

#include "carvetto/document/command.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/geometry/contour.hpp"

/// Node editing support (T-0206, ADR-0030): one undoable step that replaces the contours of vector
/// objects. Lives in its own header so it does not touch the shared command list.
namespace carvetto::document {

/// New contours for one vector object.
struct ContourReplacement {
    ObjectId id;
    std::vector<geometry::Contour> contours;
};

/// Replaces the contours of vector objects and resets their creation parameters to `NoParams` (a
/// node-edited shape is no longer the shape its parameters describe; undo restores the parameters
/// exactly because the whole payload is snapshotted).
///
/// All or nothing: `apply` returns false, leaving the document untouched, when `items` is empty,
/// an id is unknown, not a vector object, repeated, a replacement has a non-finite coordinate or a
/// coordinate beyond `kMaxCoordinateMm`, or nothing would change. Consecutive commands that replace
/// the contours of the same ids coalesce into one step (a drag): the first snapshot and the latest
/// result are kept.
class SetObjectContoursCommand : public Command {
   public:
    /// Largest coordinate magnitude (mm) the command accepts.
    static constexpr double kMaxCoordinateMm = 1.0e9;

    explicit SetObjectContoursCommand(std::vector<ContourReplacement> items, std::string text = "Edit nodes")
        : items_(std::move(items)), text_(std::move(text)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return text_; }
    bool mergeWith(const Command& next) override;

   private:
    std::vector<ContourReplacement> items_;
    std::string text_;
    std::vector<std::pair<ObjectId, ObjectData>> before_;
    std::vector<std::pair<ObjectId, ObjectData>> after_;
};

}  // namespace carvetto::document
