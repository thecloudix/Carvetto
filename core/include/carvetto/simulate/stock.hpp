#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/geometry/vec2.hpp"

// Stock block and heightfield of the material-removal simulation (T-0222, ADR-0032).

namespace carvetto::simulate {

/// Simulation quality = cell size (feature-inventory "Preview Simulation Quality": each step is
/// finer than the one before). Coarse is the cheapest.
enum class Quality { Coarse, Medium, Fine };

/// Cell size in mm of a quality level: 0.5, 0.25 and 0.125 mm.
double cellSizeFor(Quality quality);

/// Default and absolute limits of the number of cells of one stock.
inline constexpr std::size_t kDefaultMaxCells = 50'000'000;
inline constexpr std::size_t kAbsoluteMaxCells = 400'000'000;

/// The material block to cut: a rectangular slab. Toolpath coordinates are machine coordinates
/// (mm, Z up). The top surface is at Z = `topZ` (0 for the usual "Z zero at the material surface",
/// `thickness` for Z zero at the machine bed); the table/clamp surface is `thickness` below it.
struct Stock {
    double width{100.0};      ///< Extent in X, mm.
    double height{100.0};     ///< Extent in Y, mm.
    double thickness{20.0};   ///< Extent in Z, mm.
    double cellSize{0.25};    ///< Heightfield resolution, mm (the "resolution" of the card).
    geometry::Vec2 origin{};  ///< XY of the lower-left corner of the block.
    double topZ{0.0};         ///< Z of the top surface.
    /// Most cells accepted (a clear error beyond it; never above kAbsoluteMaxCells).
    std::size_t maxCells{kDefaultMaxCells};
    /// Keep a pass/toolpath id per cell (4 bytes per cell) for colouring.
    bool trackPassIds{true};

    double bottomZ() const { return topZ - thickness; }
    /// Stock with the cell size of a quality level.
    static Stock withQuality(double width, double height, double thickness, Quality quality);
    bool operator==(const Stock&) const = default;
};

enum class StockIssue {
    Ok,
    NotFinite,       ///< A dimension, the origin or topZ is NaN or infinite, or out of range (> 1e7 mm).
    NotPositive,     ///< Width, height, thickness or cell size is <= 0.
    TooManyCells,    ///< cells > maxCells (or above the absolute limit).
};

/// Result of checking a stock; `nx * ny` cells (columns along X, rows along Y) when ok.
struct StockCheck {
    StockIssue issue{StockIssue::Ok};
    std::string message;
    std::size_t nx{0};
    std::size_t ny{0};
    bool ok() const { return issue == StockIssue::Ok; }
    std::size_t cells() const { return nx * ny; }
};

/// Validates a stock without allocating: dimensions, and the cell count against the cap. Cell
/// counts are computed in double and range-checked before any integer conversion.
StockCheck checkStock(const Stock& stock);

/// A cell size (>= requested) at which the stock fits in `maxCells`; the requested size when it
/// already fits. Returns 0 for an unusable stock (non-finite or non-positive dimensions).
double fitCellSize(const Stock& stock, double requested, std::size_t maxCells);

/// The cut stock: one height per cell, row-major (index = j * nx + i), cell (i, j) covering
/// [origin + i * cellSize, origin + (i + 1) * cellSize) in X and likewise Y; the value is the
/// surface height at the cell centre.
class Heightfield {
public:
    Heightfield() = default;
    /// Allocates `check.cells()` cells at the stock's top height. `check` must be ok.
    Heightfield(const Stock& stock, const StockCheck& check);

    std::size_t nx() const { return nx_; }
    std::size_t ny() const { return ny_; }
    std::size_t cellCount() const { return z_.size(); }
    double cellSize() const { return cellSize_; }
    geometry::Vec2 origin() const { return origin_; }
    double topZ() const { return topZ_; }
    double bottomZ() const { return bottomZ_; }

    std::size_t indexOf(std::size_t i, std::size_t j) const { return j * nx_ + i; }
    /// Centre of cell (i, j).
    geometry::Vec2 centreOf(std::size_t i, std::size_t j) const;
    float height(std::size_t i, std::size_t j) const { return z_[indexOf(i, j)]; }
    /// True when the tool lowered this cell.
    bool isCut(std::size_t i, std::size_t j) const { return cut_[indexOf(i, j)] != 0; }
    /// Id of the toolpath that last lowered the cell (0 = uncut, or ids are not tracked).
    std::uint32_t passId(std::size_t i, std::size_t j) const { return ids_.empty() ? 0u : ids_[indexOf(i, j)]; }
    bool tracksPassIds() const { return !ids_.empty(); }
    /// Number of cells with isCut().
    std::size_t cutCells() const { return cutCount_; }
    /// Cell containing an XY position; false when outside the stock.
    bool cellAt(geometry::Vec2 p, std::size_t& i, std::size_t& j) const;
    /// Heights as a flat array (row-major), e.g. for hashing or upload.
    const std::vector<float>& heights() const { return z_; }
    const std::vector<std::uint8_t>& cutStates() const { return cut_; }
    const std::vector<std::uint32_t>& passIds() const { return ids_; }
    /// Memory held, bytes.
    std::size_t bytes() const;

    /// Restores the uncut stock.
    void reset();

private:
    friend class Sweeper;
    std::size_t nx_{0};
    std::size_t ny_{0};
    double cellSize_{0.0};
    geometry::Vec2 origin_{};
    double topZ_{0.0};
    double bottomZ_{0.0};
    std::vector<float> z_;
    std::vector<std::uint8_t> cut_;
    std::vector<std::uint32_t> ids_;
    std::size_t cutCount_{0};
};

}  // namespace carvetto::simulate
