#pragma once

#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"

// Print page layout and tiling (T-0349). Qt-free and pure: given the world box to print, the printable
// page and the options, `layoutPrint` decides the scale and the pages. Fit-to-page scales the content to
// one page (centred); true size prints at a fixed scale (1:1 by default) and, when the content is bigger
// than a page, tiles it over a grid of pages that overlap by `overlapMm`, numbered row by row from the
// top-left. Each tile carries registration marks (crosses at the corners of its window and at the inner
// corners of the overlap band), so neighbouring sheets are aligned on the marks: a mark of one sheet lies
// at the same world position as a mark of its neighbour.
//
// Units: world millimetres (Y up), page millimetres (origin top-left of the printable area, Y down).

namespace carvetto::viewmodel {

/// Most pages one print may produce (a guard against 1:1 prints of huge designs on small pages).
inline constexpr int kMaxPrintPages = 400;

enum class PrintScaleMode { FitToPage, TrueSize };

struct PrintPageSetup {
    /// Paintable page size (portrait/landscape already applied).
    double pageWidthMm{210.0};
    double pageHeightMm{297.0};
    /// Blank border kept inside the paintable area.
    double marginMm{10.0};
};

struct PrintOptions {
    PrintScaleMode mode{PrintScaleMode::FitToPage};
    /// True size: paper millimetres per world millimetre (1 = 1:1).
    double trueScale{1.0};
    /// Tiles overlap by this many page millimetres (true size only).
    double overlapMm{10.0};
    bool registrationMarks{true};
};

struct PrintTile {
    int column{0};
    int row{0};
    /// 1-based page number.
    int page{1};
    /// The world box the printable area of this page shows.
    geometry::Box2 window;
    /// Registration marks, page millimetres from the top-left of the page's printable area origin
    /// (the margin is already added).
    std::vector<geometry::Vec2> marks;
};

struct PrintLayout {
    /// Empty when the layout is refused (see `error`).
    std::vector<PrintTile> tiles;
    int columns{0};
    int rows{0};
    /// Paper millimetres per world millimetre.
    double scale{1.0};
    /// Printable size (page minus margins), page millimetres.
    double printableWidthMm{0.0};
    double printableHeightMm{0.0};
    double marginMm{0.0};
    std::string error;

    int pageCount() const { return static_cast<int>(tiles.size()); }
};

/// Length of a registration cross arm in page millimetres.
inline constexpr double kPrintMarkArmMm = 4.0;

PrintLayout layoutPrint(const geometry::Box2& content, const PrintPageSetup& page, const PrintOptions& options);

/// Page millimetres of world point `p` on `tile` (the inverse of the tile's view).
geometry::Vec2 printWorldToPage(const PrintLayout& layout, const PrintTile& tile, geometry::Vec2 p);

}  // namespace carvetto::viewmodel
