#include "carvetto/simulate/stock.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace carvetto::simulate {

namespace {
constexpr double kMaxExtent = 1e7;  // mm; keeps every product and sum in range

bool inRange(double v) {
    return std::isfinite(v) && std::fabs(v) <= kMaxExtent;
}
}  // namespace

double cellSizeFor(Quality quality) {
    switch (quality) {
        case Quality::Coarse:
            return 0.5;
        case Quality::Medium:
            return 0.25;
        case Quality::Fine:
            return 0.125;
    }
    return 0.25;
}

Stock Stock::withQuality(double width, double height, double thickness, Quality quality) {
    Stock s;
    s.width = width;
    s.height = height;
    s.thickness = thickness;
    s.cellSize = cellSizeFor(quality);
    return s;
}

StockCheck checkStock(const Stock& s) {
    StockCheck r;
    if (!inRange(s.width) || !inRange(s.height) || !inRange(s.thickness) || !inRange(s.cellSize) ||
        !inRange(s.origin.x) || !inRange(s.origin.y) || !inRange(s.topZ)) {
        r.issue = StockIssue::NotFinite;
        r.message = "stock dimensions, cell size, origin and top must be finite and within 1e7 mm";
        return r;
    }
    if (!(s.width > 0.0) || !(s.height > 0.0) || !(s.thickness > 0.0) || !(s.cellSize > 0.0)) {
        r.issue = StockIssue::NotPositive;
        r.message = "stock width, height, thickness and cell size must be positive";
        return r;
    }
    const double cols = std::ceil(s.width / s.cellSize);
    const double rows = std::ceil(s.height / s.cellSize);
    const double cells = cols * rows;  // both <= 1e7 / tiny cell: the product stays finite in double
    const double cap = static_cast<double>(std::min(s.maxCells, kAbsoluteMaxCells));
    if (!(cells <= cap)) {
        r.issue = StockIssue::TooManyCells;
        r.message = "stock needs " + std::to_string(cells) + " cells, more than the limit of " +
                    std::to_string(static_cast<unsigned long long>(cap)) + "; use a larger cell size";
        return r;
    }
    r.nx = static_cast<std::size_t>(cols);
    r.ny = static_cast<std::size_t>(rows);
    return r;
}

double fitCellSize(const Stock& stock, double requested, std::size_t maxCells) {
    if (!inRange(stock.width) || !inRange(stock.height) || !(stock.width > 0.0) || !(stock.height > 0.0) ||
        !inRange(requested) || !(requested > 0.0)) {
        return 0.0;
    }
    const double cap = static_cast<double>(std::min(maxCells, kAbsoluteMaxCells));
    double cs = requested;
    for (int k = 0; k < 200; ++k) {
        if (std::ceil(stock.width / cs) * std::ceil(stock.height / cs) <= cap) {
            return cs;
        }
        cs *= 1.05;
    }
    return 0.0;
}

Heightfield::Heightfield(const Stock& stock, const StockCheck& check)
    : nx_(check.nx),
      ny_(check.ny),
      cellSize_(stock.cellSize),
      origin_(stock.origin),
      topZ_(stock.topZ),
      bottomZ_(stock.bottomZ()),
      z_(check.cells(), static_cast<float>(stock.topZ)),
      cut_(check.cells(), 0),
      ids_(stock.trackPassIds ? check.cells() : 0, 0) {}

geometry::Vec2 Heightfield::centreOf(std::size_t i, std::size_t j) const {
    return {origin_.x + (static_cast<double>(i) + 0.5) * cellSize_, origin_.y + (static_cast<double>(j) + 0.5) * cellSize_};
}

bool Heightfield::cellAt(geometry::Vec2 p, std::size_t& i, std::size_t& j) const {
    const double fx = std::floor((p.x - origin_.x) / cellSize_);
    const double fy = std::floor((p.y - origin_.y) / cellSize_);
    if (!(fx >= 0.0) || !(fy >= 0.0) || !(fx < static_cast<double>(nx_)) || !(fy < static_cast<double>(ny_))) {
        return false;
    }
    i = static_cast<std::size_t>(fx);
    j = static_cast<std::size_t>(fy);
    return true;
}

std::size_t Heightfield::bytes() const {
    return z_.size() * sizeof(float) + cut_.size() + ids_.size() * sizeof(std::uint32_t);
}

void Heightfield::reset() {
    std::fill(z_.begin(), z_.end(), static_cast<float>(topZ_));
    std::fill(cut_.begin(), cut_.end(), static_cast<std::uint8_t>(0));
    std::fill(ids_.begin(), ids_.end(), 0u);
    cutCount_ = 0;
}

}  // namespace carvetto::simulate
