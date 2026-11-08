#include "carvetto/text/font.hpp"

#include <algorithm>
#include <fstream>
#include <system_error>
#include <utility>

#include "font_impl.hpp"

namespace carvetto::text {

using geometry::BezierSpan;
using geometry::Contour;
using geometry::LineSpan;
using geometry::Vec2;

Font::Font() = default;
Font::~Font() = default;
Font::Font(Font&&) noexcept = default;
Font& Font::operator=(Font&&) noexcept = default;

namespace {

constexpr int kMinUnitsPerEm = 16;
constexpr int kMaxUnitsPerEm = 16384;

bool hasSfntMagic(std::span<const std::uint8_t> b) {
    if (b.size() < 12) {
        return false;
    }
    const std::uint32_t tag = (std::uint32_t{b[0]} << 24) | (std::uint32_t{b[1]} << 16) |
                              (std::uint32_t{b[2]} << 8) | std::uint32_t{b[3]};
    // 0x00010000 TrueType, 'true' (Apple TrueType), 'OTTO' (CFF), 'ttcf' (collection, face 0).
    return tag == 0x00010000u || tag == 0x74727565u || tag == 0x4F54544Fu || tag == 0x74746366u;
}

FontLoad refuse(FontStatus status, std::string message) {
    FontLoad r;
    r.status = status;
    r.message = std::move(message);
    return r;
}

}  // namespace

FontLoad Font::load(std::span<const std::uint8_t> bytes, const FontLimits& limits) {
    if (bytes.empty()) {
        return refuse(FontStatus::Empty, "no font data");
    }
    if (bytes.size() > std::min(limits.maxFileBytes, kAbsoluteMaxFontBytes)) {
        return refuse(FontStatus::TooLarge, "font file is larger than the limit");
    }
    // Only sfnt containers reach FreeType: its Type 1, BDF, PCF, ... drivers are not needed and
    // would only widen the attack surface.
    if (!hasSfntMagic(bytes)) {
        return refuse(FontStatus::UnsupportedFormat, "not a TrueType/OpenType font");
    }

    auto impl = std::make_unique<Impl>();
    impl->bytes.assign(bytes.begin(), bytes.end());
    if (FT_Init_FreeType(&impl->library) != 0) {
        return refuse(FontStatus::Corrupt, "FreeType could not start");
    }
    if (FT_New_Memory_Face(impl->library, impl->bytes.data(), static_cast<FT_Long>(impl->bytes.size()), 0,
                           &impl->face) != 0 ||
        impl->face == nullptr) {
        impl->face = nullptr;
        return refuse(FontStatus::Corrupt, "FreeType could not read the font");
    }
    FT_Face face = impl->face;
    if (!FT_IS_SCALABLE(face) || face->num_glyphs <= 0 || face->units_per_EM < kMinUnitsPerEm ||
        face->units_per_EM > kMaxUnitsPerEm) {
        return refuse(FontStatus::Corrupt, "font has no scalable outlines or an unusable em size");
    }
    if (static_cast<std::uint64_t>(face->num_glyphs) > limits.maxGlyphs) {
        return refuse(FontStatus::Corrupt, "font has more glyphs than the limit");
    }

    impl->hbBlob = hb_blob_create(reinterpret_cast<const char*>(impl->bytes.data()),
                                  static_cast<unsigned int>(impl->bytes.size()), HB_MEMORY_MODE_READONLY, nullptr,
                                  nullptr);
    impl->hbFace = hb_face_create(impl->hbBlob, 0);
    if (hb_face_get_glyph_count(impl->hbFace) == 0 ||
        hb_face_get_upem(impl->hbFace) != static_cast<unsigned int>(face->units_per_EM)) {
        return refuse(FontStatus::Corrupt, "HarfBuzz could not read the font tables");
    }
    impl->hbFont = hb_font_create(impl->hbFace);
    // Shaping in font units: all advances and offsets are integers.
    hb_font_set_scale(impl->hbFont, face->units_per_EM, face->units_per_EM);
    hb_font_set_ptem(impl->hbFont, 0.0f);

    FontMetrics& m = impl->metrics;
    m.unitsPerEm = face->units_per_EM;
    m.ascender = face->ascender;
    m.descender = face->descender;
    m.lineHeight = face->height > 0 ? face->height : face->ascender - face->descender;
    if (m.lineHeight <= 0) {
        m.lineHeight = m.unitsPerEm;
    }
    m.glyphCount = static_cast<std::uint32_t>(face->num_glyphs);
    if (face->family_name != nullptr) {
        // Bounded: the name comes from a hostile file.
        for (std::size_t i = 0; i < 256 && face->family_name[i] != '\0'; ++i) {
            impl->family.push_back(face->family_name[i]);
        }
    }

    FontLoad r;
    r.status = FontStatus::Ok;
    r.font.impl_ = std::move(impl);
    return r;
}

FontLoad Font::loadFile(const std::filesystem::path& path, const FontLimits& limits) {
    std::error_code ec;
    const std::uintmax_t size = std::filesystem::file_size(path, ec);
    if (ec) {
        return refuse(FontStatus::IoError, "cannot stat the font file");
    }
    if (size == 0) {
        return refuse(FontStatus::Empty, "the font file is empty");
    }
    if (size > std::min<std::uintmax_t>(limits.maxFileBytes, kAbsoluteMaxFontBytes)) {
        return refuse(FontStatus::TooLarge, "font file is larger than the limit");
    }
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return refuse(FontStatus::IoError, "cannot open the font file");
    }
    std::vector<std::uint8_t> data(static_cast<std::size_t>(size));
    in.read(reinterpret_cast<char*>(data.data()), static_cast<std::streamsize>(data.size()));
    if (in.gcount() != static_cast<std::streamsize>(data.size())) {
        return refuse(FontStatus::IoError, "cannot read the font file");
    }
    return load(data, limits);
}

const FontMetrics& Font::metrics() const {
    static const FontMetrics kNone{};
    return impl_ ? impl_->metrics : kNone;
}

const std::string& Font::familyName() const {
    static const std::string kNone;
    return impl_ ? impl_->family : kNone;
}

std::uint32_t Font::glyphIndex(char32_t codePoint) const {
    if (!impl_) {
        return 0;
    }
    return static_cast<std::uint32_t>(FT_Get_Char_Index(impl_->face, static_cast<FT_ULong>(codePoint)));
}

namespace {

// Collects FT_Outline_Decompose callbacks into closed contours of lines and cubic Beziers.
struct Collector {
    std::vector<Contour> contours;
    Contour current;
    Vec2 pos;
    Vec2 start;
    bool open{false};

    static Vec2 toVec(const FT_Vector* v) { return {static_cast<double>(v->x), static_cast<double>(v->y)}; }

    void finish() {
        if (open && !current.spans.empty()) {
            if (pos != start) {
                current.spans.emplace_back(LineSpan{pos, start});
            }
            current.closed = true;
            contours.push_back(std::move(current));
        }
        current = Contour{};
        open = false;
    }
    void line(Vec2 to) {
        if (to != pos) {
            current.spans.emplace_back(LineSpan{pos, to});
            pos = to;
        }
    }
    // Exact degree elevation: C1 = (P0 + 2Q) / 3, C2 = (P2 + 2Q) / 3.
    void conic(Vec2 q, Vec2 to) {
        const Vec2 c1{(pos.x + 2.0 * q.x) / 3.0, (pos.y + 2.0 * q.y) / 3.0};
        const Vec2 c2{(to.x + 2.0 * q.x) / 3.0, (to.y + 2.0 * q.y) / 3.0};
        current.spans.emplace_back(BezierSpan{pos, c1, c2, to});
        pos = to;
    }
    void cubic(Vec2 c1, Vec2 c2, Vec2 to) {
        current.spans.emplace_back(BezierSpan{pos, c1, c2, to});
        pos = to;
    }
};

int moveTo(const FT_Vector* to, void* user) {
    auto* c = static_cast<Collector*>(user);
    c->finish();
    c->pos = c->start = Collector::toVec(to);
    c->open = true;
    return 0;
}
int lineTo(const FT_Vector* to, void* user) {
    static_cast<Collector*>(user)->line(Collector::toVec(to));
    return 0;
}
int conicTo(const FT_Vector* control, const FT_Vector* to, void* user) {
    static_cast<Collector*>(user)->conic(Collector::toVec(control), Collector::toVec(to));
    return 0;
}
int cubicTo(const FT_Vector* c1, const FT_Vector* c2, const FT_Vector* to, void* user) {
    static_cast<Collector*>(user)->cubic(Collector::toVec(c1), Collector::toVec(c2), Collector::toVec(to));
    return 0;
}

}  // namespace

GlyphOutline Font::outline(std::uint32_t glyphId, std::size_t maxPoints) const {
    GlyphOutline result;
    if (!impl_ || glyphId >= impl_->metrics.glyphCount) {
        result.status = OutlineStatus::LoadFailed;
        return result;
    }
    // Unscaled and unhinted: coordinates are the font's own integers, the same on every platform.
    const FT_Int32 flags = FT_LOAD_NO_SCALE | FT_LOAD_NO_HINTING | FT_LOAD_NO_AUTOHINT | FT_LOAD_NO_BITMAP;
    if (FT_Load_Glyph(impl_->face, glyphId, flags) != 0) {
        result.status = OutlineStatus::LoadFailed;
        return result;
    }
    FT_GlyphSlot slot = impl_->face->glyph;
    if (slot->format != FT_GLYPH_FORMAT_OUTLINE) {
        result.status = OutlineStatus::LoadFailed;
        return result;
    }
    FT_Outline& o = slot->outline;
    result.pointCount = static_cast<std::size_t>(o.n_points);
    if (result.pointCount > maxPoints) {
        result.status = OutlineStatus::TooComplex;
        return result;
    }
    if (o.n_points == 0) {
        return result;  // space and friends
    }
    if (FT_Outline_Check(&o) != 0) {
        result.status = OutlineStatus::LoadFailed;
        return result;
    }
    Collector collector;
    FT_Outline_Funcs funcs{};
    funcs.move_to = moveTo;
    funcs.line_to = lineTo;
    funcs.conic_to = conicTo;
    funcs.cubic_to = cubicTo;
    funcs.shift = 0;
    funcs.delta = 0;
    if (FT_Outline_Decompose(&o, &funcs, &collector) != 0) {
        result.status = OutlineStatus::LoadFailed;
        return result;
    }
    collector.finish();
    // FreeType marks PostScript-style outlines (CFF, Type 1: outer contours counter-clockwise) with
    // FT_OUTLINE_REVERSE_FILL; without it the outline is TrueType-style (outer contours clockwise).
    // Normalise to counter-clockwise outers so callers see one convention.
    if ((o.flags & FT_OUTLINE_REVERSE_FILL) == 0) {
        for (Contour& c : collector.contours) {
            c.reverse();
        }
    }
    result.contours = std::move(collector.contours);
    return result;
}

}  // namespace carvetto::text
