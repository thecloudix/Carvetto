#include "carvetto/simulate/preview_component.hpp"

#include <cmath>
#include <new>
#include <vector>

namespace carvetto::simulate {

relief::Status componentFromPreview(const Heightfield& surface, const PreviewComponentOptions& options,
                                    relief::Component& out) {
    if (!std::isfinite(options.zOffset)) {
        return relief::Status::BadParameter;
    }
    const std::size_t nx = surface.nx();
    const std::size_t ny = surface.ny();
    if (nx == 0 || ny == 0) {
        return relief::Status::ZeroSize;
    }
    relief::Grid grid;
    grid.width = nx;
    grid.height = ny;
    grid.cellSize = surface.cellSize();
    const geometry::Vec2 c0 = surface.centreOf(0, 0);
    grid.origin = {c0.x, c0.y};
    if (const relief::Status s = relief::validate(grid); s != relief::Status::Ok) {
        return s;
    }
    std::vector<float> data;
    try {
        data.resize(nx * ny);
    } catch (const std::bad_alloc&) {
        return relief::Status::TooLarge;
    }
    for (std::size_t j = 0; j < ny; ++j) {
        for (std::size_t i = 0; i < nx; ++i) {
            if (options.cells == PreviewCells::CutOnly && !surface.isCut(i, j)) {
                data[j * nx + i] = relief::kTransparent;
            } else {
                data[j * nx + i] = static_cast<float>(static_cast<double>(surface.height(i, j)) - options.zOffset);
            }
        }
    }
    relief::Component comp;
    if (const relief::Status s = relief::HeightField::fromData(grid, std::move(data), comp.field);
        s != relief::Status::Ok) {
        return s;
    }
    comp.name = options.name;
    comp.mode = options.mode;
    out = std::move(comp);
    return relief::Status::Ok;
}

}  // namespace carvetto::simulate
