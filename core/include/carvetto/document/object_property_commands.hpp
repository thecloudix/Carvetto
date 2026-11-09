#pragma once

#include <cstddef>
#include <optional>
#include <string>
#include <utility>

#include "carvetto/document/command.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/object.hpp"

/// Undoable edits behind the Properties panel (T-0210): rename an object and change the creation
/// parameters of a parametric shape. Each is one undo step and all or nothing.
namespace carvetto::document {

/// Longest object / layer / sheet name (bytes of UTF-8) the properties UI accepts.
inline constexpr std::size_t kMaxNameBytes = 256;

/// Empty when `name` is acceptable as an object, layer or sheet name: at most `kMaxNameBytes`
/// bytes and no control characters (U+0000-U+001F, U+007F); with `required` a blank name is refused.
std::string nameProblem(const std::string& name, bool required);

/// Sets `Object::name` (an empty name clears it). False when the object is unknown, the name has a
/// problem (`nameProblem`) or nothing changes.
class RenameObjectCommand : public Command {
   public:
    RenameObjectCommand(ObjectId id, std::string name) : id_(id), name_(std::move(name)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Rename object"; }

   private:
    ObjectId id_;
    std::string name_;
    std::string previous_;
};

/// Replaces the creation parameters of a rectangle, circle, ellipse, polygon or star object and
/// rebuilds its contours from them (`buildContours`). The new parameters must be of the same kind
/// as the current ones; text, plain geometry, bitmaps and groups are refused (text has
/// `SetTextCommand`). False (document untouched) when the object is unknown, the kind differs,
/// a value is non-finite or beyond `kMaxCoordinateMm`, the builder refuses the values, or nothing
/// changes. Undo restores the previous payload exactly.
class SetShapeParamsCommand : public Command {
   public:
    static constexpr double kMaxCoordinateMm = 1.0e9;
    SetShapeParamsCommand(ObjectId id, CreationParams params, std::string text = "Edit shape")
        : id_(id), params_(std::move(params)), text_(std::move(text)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return text_; }

   private:
    ObjectId id_;
    CreationParams params_;
    std::string text_;
    std::optional<VectorObject> before_;
};

}  // namespace carvetto::document
