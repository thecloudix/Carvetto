#pragma once

// Private state of carvetto::text::Font (T-0130, ADR-0033): the FreeType and HarfBuzz objects for
// one font. Only core/src/text includes this (and thereby the third-party headers).

#include <cstdint>
#include <string>
#include <vector>

#include <ft2build.h>
#include FT_FREETYPE_H
#include FT_OUTLINE_H
#include <hb.h>

#include "carvetto/text/font.hpp"

namespace carvetto::text {

struct Font::Impl {
    Impl() = default;
    Impl(const Impl&) = delete;
    Impl& operator=(const Impl&) = delete;
    ~Impl() {
        if (hbFont != nullptr) {
            hb_font_destroy(hbFont);
        }
        if (hbFace != nullptr) {
            hb_face_destroy(hbFace);
        }
        if (hbBlob != nullptr) {
            hb_blob_destroy(hbBlob);
        }
        if (face != nullptr) {
            FT_Done_Face(face);
        }
        if (library != nullptr) {
            FT_Done_FreeType(library);
        }
    }

    std::vector<std::uint8_t> bytes;  // must outlive face and blob; never resized after loading
    FT_Library library{nullptr};
    FT_Face face{nullptr};
    hb_blob_t* hbBlob{nullptr};
    hb_face_t* hbFace{nullptr};
    hb_font_t* hbFont{nullptr};
    FontMetrics metrics;
    std::string family;
};

}  // namespace carvetto::text
