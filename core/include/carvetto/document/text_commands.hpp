#pragma once

#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "carvetto/document/command.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/object.hpp"

/// Undoable edits for text objects (T-0211, ADR-0033 amendment A1). Combine them in a
/// `MacroCommand` for one undo step (e.g. embed a font and add the text that uses it).
namespace carvetto::document {

/// Embeds a font. A no-op when the same font is already embedded, or when the font is refused by
/// the caps of `Document::insertFont`. Undo removes it only if this command added it.
class AddFontCommand : public Command {
   public:
    explicit AddFontCommand(FontAsset font) : font_(std::move(font)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Embed font"; }

   private:
    FontAsset font_;
    bool added_{false};
};

/// Replaces the payload of a text object with new parameters and contours (the laid-out result of
/// `buildTextContours`). All or nothing: false when the object is not a vector object with text
/// parameters, `params` is not text, `params.fontKey` names no embedded font, the parameters have
/// a problem (`textParamsProblem`), a coordinate is non-finite, or nothing changes. Undo restores
/// the previous payload exactly.
class SetTextCommand : public Command {
   public:
    SetTextCommand(ObjectId id, TextParams params, std::vector<geometry::Contour> contours,
                   std::string text = "Edit text")
        : id_(id), params_(std::move(params)), contours_(std::move(contours)), text_(std::move(text)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return text_; }

   private:
    ObjectId id_;
    TextParams params_;
    std::vector<geometry::Contour> contours_;
    std::string text_;
    std::optional<VectorObject> before_;
};

/// Converts text objects to plain curves: the creation parameters become `NoParams` and the glyph
/// contours stay as they are, so toolpaths and node editing work on them. All or nothing: false
/// when `ids` is empty or an id is not a text object. One undo step for all of them.
class ConvertTextToCurvesCommand : public Command {
   public:
    explicit ConvertTextToCurvesCommand(std::vector<ObjectId> ids) : ids_(std::move(ids)) {}
    bool apply(Document& doc) override;
    void revert(Document& doc) override;
    std::string text() const override { return "Convert text to curves"; }

   private:
    std::vector<ObjectId> ids_;
    std::vector<std::pair<ObjectId, CreationParams>> before_;
};

}  // namespace carvetto::document
