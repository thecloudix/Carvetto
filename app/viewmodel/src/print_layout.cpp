#include "carvetto/viewmodel/print_layout.hpp"

#include <algorithm>
#include <cmath>

namespace carvetto::viewmodel {

using geometry::Box2;
using geometry::Vec2;

PrintLayout layoutPrint(const Box2& content, const PrintPageSetup& page, const PrintOptions& options) {
    PrintLayout out;
    const double margin = std::max(0.0, page.marginMm);
    out.marginMm = margin;
    out.printableWidthMm = page.pageWidthMm - 2.0 * margin;
    out.printableHeightMm = page.pageHeightMm - 2.0 * margin;
    if (!(out.printableWidthMm > 1.0) || !(out.printableHeightMm > 1.0)) {
        out.error = "the page is too small to print on";
        return out;
    }
    if (content.isEmpty()) {
        out.error = "there is nothing to print";
        return out;
    }
    const double cw = content.width();
    const double ch = content.height();
    const double pw = out.printableWidthMm;
    const double ph = out.printableHeightMm;

    if (options.mode == PrintScaleMode::FitToPage) {
        double scale = 1.0;
        if (cw > 0.0 || ch > 0.0) {
            const double sx = cw > 0.0 ? pw / cw : 1e300;
            const double sy = ch > 0.0 ? ph / ch : 1e300;
            scale = std::min(sx, sy);
        }
        out.scale = scale;
        out.columns = 1;
        out.rows = 1;
        PrintTile tile;
        tile.window = Box2::fromCentre(content.centre(), pw / scale, ph / scale);
        out.tiles.push_back(tile);
    } else {
        const double scale = options.trueScale;
        if (!(scale > 1e-6) || !(scale < 1e6)) {
            out.error = "the print scale is out of range";
            return out;
        }
        out.scale = scale;
        const double winW = pw / scale;
        const double winH = ph / scale;
        // Overlap must leave a positive step.
        const double overlap = std::clamp(options.overlapMm, 0.0, 0.5 * std::min(pw, ph)) / scale;
        const double stepX = winW - overlap;
        const double stepY = winH - overlap;
        const auto count = [](double extent, double win, double step) {
            if (extent <= win) {
                return 1.0;
            }
            return std::ceil((extent - win) / step) + 1.0;
        };
        const double cols = count(cw, winW, stepX);
        const double rows = count(ch, winH, stepY);
        if (cols * rows > static_cast<double>(kMaxPrintPages)) {
            out.error = "the print would need " + std::to_string(static_cast<long long>(std::min(cols * rows, 1e9))) +
                        " pages (the most is " + std::to_string(kMaxPrintPages) + "); use a smaller scale";
            return out;
        }
        out.columns = static_cast<int>(cols);
        out.rows = static_cast<int>(rows);
        // A single column or row is centred on the content; a grid starts at the content's top-left.
        const double left0 = out.columns == 1 ? content.centre().x - 0.5 * winW : content.min.x;
        const double top0 = out.rows == 1 ? content.centre().y + 0.5 * winH : content.max.y;
        int pageNo = 1;
        for (int r = 0; r < out.rows; ++r) {
            for (int c = 0; c < out.columns; ++c) {
                PrintTile tile;
                tile.column = c;
                tile.row = r;
                tile.page = pageNo++;
                const double left = left0 + c * stepX;
                const double top = top0 - r * stepY;
                tile.window = Box2{{left, top - winH}, {left + winW, top}};
                out.tiles.push_back(tile);
            }
        }
    }

    for (PrintTile& tile : out.tiles) {
        if (out.tiles.size() == 1) {
            tile.page = 1;
        }
        if (!options.registrationMarks || out.tiles.size() < 2) {
            continue;
        }
        const double overlapPage = std::clamp(options.overlapMm, 0.0, 0.5 * std::min(pw, ph));
        const double x0 = margin;
        const double y0 = margin;
        const double x1 = margin + pw;
        const double y1 = margin + ph;
        // The grid of crosses: the page corners and, inside the overlap bands, the corners the
        // neighbours' windows start and end at. A cross of one sheet lies at the same world position
        // as one of each neighbour (right, below and diagonal), whichever side they overlap.
        std::vector<double> xs{x0, x1};
        std::vector<double> ys{y0, y1};
        if (overlapPage > 0.0) {
            xs = {x0, x0 + overlapPage, x1 - overlapPage, x1};
            ys = {y0, y0 + overlapPage, y1 - overlapPage, y1};
        }
        for (const double y : ys) {
            for (const double x : xs) {
                tile.marks.push_back({x, y});
            }
        }
    }
    return out;
}

Vec2 printWorldToPage(const PrintLayout& layout, const PrintTile& tile, Vec2 p) {
    return {layout.marginMm + (p.x - tile.window.min.x) * layout.scale,
            layout.marginMm + (tile.window.max.y - p.y) * layout.scale};
}

}  // namespace carvetto::viewmodel
