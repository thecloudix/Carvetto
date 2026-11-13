#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "carvetto/document/document.hpp"
#include "carvetto/document/ids.hpp"
#include "carvetto/document/object.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/text/font.hpp"
#include "carvetto/viewmodel/view_transform.hpp"

// Qt-free text tool (T-0211, ADR-0033 amendment A1, ADR-0027). The `Text` tool of the
// `ToolController` forwards pointer events and keys here; the canvas paints `preview()` and the Qt
// layer feeds typed characters and property edits.
//
// Gestures (the pointer is already snapped by the caller)
//   Point mode  click: places the anchor of a new text; type with `insertText` / `backspace`.
//   Box mode    press-drag-release: the dragged rectangle is the box, the text wraps to its width.
//   Curve mode  click on a curve (any vector object's outline): the nearest contour becomes the
//               guide, the text runs along it from its start (`setStartOffset`, `reverseGuide`).
//   In every mode a click on the glyphs of an existing text object edits that object in place.
//   Clicking elsewhere while a draft is open commits it first (as the usual text tools do).
//
// Draft model (gesture-frozen state): the text being typed lives only in the editor, never in the
// document, until `commit()`. So a draft is trivially "undone" by discarding it, nothing in the
// document changes while typing, and replacing the document mid-gesture (`setDocument`) drops the
// draft without touching either document. The draft holds copies of everything it needs (the font
// bytes, the guide contour), and no `Document` reference survives `setDocument`.
//
// Commit is ONE undo step: embed the font if the document lacks it, then add the object (new text)
// or replace the payload (edited text) with the laid-out contours.
//
// Fonts: only bytes the user or app supplies (`setFont`, `setFontFile`); there is no system font
// enumeration. The font is embedded in the document on commit (portable files, see ADR amendment).

namespace carvetto::viewmodel {

/// Persistent settings of the text tool; changing one re-lays out the open draft.
struct TextSettings {
    double sizeMm{10.0};  ///< Em size.
    document::TextAlign align{document::TextAlign::Left};
    double letterSpacingMm{0.0};
    double lineSpacing{1.0};
    bool kerning{true};
    document::TextMode mode{document::TextMode::Point};
};

/// Caret movements of the open draft (byte offsets always sit on UTF-8 code point boundaries).
enum class CaretMove { Left, Right, LineStart, LineEnd, TextStart, TextEnd };

struct TextEditResult {
    bool ok{true};
    std::string message;           ///< The reason when something was refused (or a warning).
    bool committed{false};         ///< The document changed (one undo step).
    bool selectionChanged{false};
    bool draftChanged{false};      ///< The overlay must be redrawn.
};

class TextEditor {
   public:
    static constexpr double kDragThresholdPx = 4.0;

    explicit TextEditor(document::Document* doc = nullptr);
    ~TextEditor();
    TextEditor(const TextEditor&) = delete;
    TextEditor& operator=(const TextEditor&) = delete;

    /// Not owned. Drops the draft (without touching either document) and the document's font cache;
    /// the chosen font and the settings stay.
    void setDocument(document::Document* doc);
    void setView(const ViewTransform& view) { view_ = view; }

    // --- Settings ---------------------------------------------------------------------------
    const TextSettings& settings() const { return settings_; }
    TextEditResult setSize(double sizeMm);
    TextEditResult setAlign(document::TextAlign align);
    TextEditResult setLetterSpacing(double mm);
    TextEditResult setLineSpacing(double factor);
    TextEditResult setKerning(bool on);
    /// Mode for the next gesture; an open draft is committed first.
    TextEditResult setMode(document::TextMode mode);

    // --- Font -------------------------------------------------------------------------------
    /// Uses font bytes (TTF/OTF; at most `document::kMaxEmbeddedFontBytes`). Refused with a message
    /// when the font cannot be parsed. Changes the open draft's font too.
    TextEditResult setFont(std::vector<std::uint8_t> bytes, std::string displayName = {});
    /// Reads a font file (size checked before reading) and calls `setFont`.
    TextEditResult setFontFile(const std::filesystem::path& path);
    bool hasFont() const;
    /// Key of the current font (empty when none).
    const std::string& fontKey() const { return fontKey_; }
    /// Name shown in the UI: the file name given to `setFont`, else the font's family name.
    const std::string& fontName() const { return fontName_; }
    /// Bytes of the current font (empty when none).
    const std::vector<std::uint8_t>& fontBytes() const { return fontBytes_; }

    // --- Pointer (px: screen pixels, snappedMm: the snapped pointer in millimetres) ---------
    TextEditResult press(geometry::Vec2 px, geometry::Vec2 snappedMm);
    TextEditResult move(geometry::Vec2 px, geometry::Vec2 snappedMm);
    TextEditResult release(geometry::Vec2 px, geometry::Vec2 snappedMm);
    bool pressed() const { return pressed_; }

    // --- Typing -----------------------------------------------------------------------------
    /// Inserts UTF-8 text at the caret, replacing the selection ("\n" is a line/paragraph break,
    /// "\r" becomes "\n"). With the caret at the end this appends.
    TextEditResult insertText(std::string_view utf8);
    /// Removes the selection, or the character (code point) before the caret.
    TextEditResult backspace();
    /// Removes the selection, or the character (code point) after the caret.
    TextEditResult deleteForward();
    /// Replaces the whole text of the draft (a properties text box); the caret goes to the end.
    TextEditResult setText(std::string_view utf8);

    // --- Caret and selection (T-0230) -----------------------------------------------------------
    /// Byte offset of the caret in the draft text (0 when there is no draft).
    std::size_t caretOffset() const { return draft_ ? draft_->caret : 0; }
    /// Selected byte range [first, second), nullopt when nothing is selected.
    std::optional<std::pair<std::size_t, std::size_t>> selectionRange() const;
    /// Moves the caret; `extend` keeps the selection anchor (shift held), otherwise it collapses.
    /// Line moves stop at "\n" boundaries of the text (paragraph lines, not wrapped box lines).
    TextEditResult moveCaret(CaretMove move, bool extend = false);
    /// Places the caret at a byte offset (clamped, snapped back to a code point boundary).
    TextEditResult setCaret(std::size_t byteOffset, bool extend = false);
    TextEditResult selectAll();
    /// Caret segment (bottom, top) in millimetres for painting. Point-mode drafts only; nullopt for
    /// box and curve text (their caret is not drawn yet).
    const std::optional<std::pair<geometry::Vec2, geometry::Vec2>>& caretSegmentMm() const { return caretSeg_; }
    /// Selection highlight quads (4 corners each, millimetres), Point-mode drafts only.
    const std::vector<std::array<geometry::Vec2, 4>>& selectionQuadsMm() const { return selQuads_; }
    /// The text frame of the draft in millimetres: the box in Box mode, else the bounds of the glyph
    /// outlines grown by a quarter em; nullopt when there is no draft or nothing to frame.
    std::optional<geometry::Box2> frameMm() const;

    // --- Draft ------------------------------------------------------------------------------
    bool hasDraft() const { return draft_.has_value(); }
    /// The text object being edited in place, null for a new text.
    document::ObjectId editedObject() const { return draft_ ? draft_->editing : document::ObjectId{}; }
    const document::TextParams* draftParams() const { return draft_ ? &draft_->params : nullptr; }
    /// Glyph contours of the draft in millimetres (empty when there is none or it is refused).
    const std::vector<geometry::Contour>& previewContours() const { return preview_; }
    /// Why the draft cannot be placed (empty when fine or there is no draft yet).
    const std::string& previewMessage() const { return previewMessage_; }
    bool previewOverflow() const { return overflow_; }
    /// Box mode: the rectangle (anchor, size) for painting, nullopt otherwise or while dragging.
    std::optional<geometry::Box2> boxMm() const;
    /// While the box is being dragged: its corners in millimetres.
    std::optional<geometry::Box2> dragBoxMm() const;

    /// Opens the single selected text object for editing in place (the properties panel).
    TextEditResult beginEditSelected();
    /// Opens a given text object for editing in place.
    TextEditResult beginEdit(document::ObjectId id);
    /// Curve mode: runs the text the other way along the guide.
    TextEditResult reverseGuide();
    /// Curve mode: arc length of the alignment anchor on the guide.
    TextEditResult setStartOffset(double mm);

    /// One undo step: embed the font if needed and add (or update) the text object. An empty text
    /// is not committed (the draft is dropped with a message). Selects the object.
    TextEditResult commit();
    /// Drops the draft; the document is untouched.
    void cancel();
    /// Forgets a pointer gesture in progress (a box being dragged) but keeps the draft.
    void abandonGesture() {
        pressed_ = false;
        dragging_ = false;
    }

    /// One undo step: converts the selected text objects to curves (the glyph contours stay).
    TextEditResult convertSelectionToCurves();

   private:
    struct Draft {
        document::TextParams params;
        document::ObjectId editing{};
        std::size_t caret{0};   ///< Byte offset of the caret.
        std::size_t anchor{0};  ///< Selection anchor; equals `caret` when nothing is selected.
    };
    struct Pending;

    bool layerWritable(std::string* why) const;
    const text::Font* font() const;
    TextEditResult relayout();
    TextEditResult begin(document::TextParams params, document::ObjectId editing);
    TextEditResult startFor(geometry::Vec2 anchorMm);
    void clearDraft();
    void refreshCaretGeometry();
    TextEditResult replaceRange(std::size_t from, std::size_t to, std::string_view with);
    bool loadFont(const std::string& key, const std::vector<std::uint8_t>& bytes, std::string* message);

    document::Document* doc_{nullptr};
    ViewTransform view_;
    TextSettings settings_;
    std::string fontKey_;
    std::string fontName_;
    std::vector<std::uint8_t> fontBytes_;
    std::unique_ptr<text::Font> font_;
    std::optional<Draft> draft_;
    std::vector<geometry::Contour> preview_;
    std::string previewMessage_;
    bool overflow_{false};
    std::optional<std::pair<geometry::Vec2, geometry::Vec2>> caretSeg_;
    std::vector<std::array<geometry::Vec2, 4>> selQuads_;

    bool pressed_{false};
    geometry::Vec2 pressPx_{};
    geometry::Vec2 pressMm_{};
    geometry::Vec2 curMm_{};
    bool dragging_{false};
};

}  // namespace carvetto::viewmodel
