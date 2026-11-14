#pragma once

#include <string>

#include "carvetto/relief/relief.hpp"
#include "carvetto/simulate/stock.hpp"

// Create Component From Toolpath Preview (T-0298, ADR-0073): the simulated stock surface becomes a
// relief component.

namespace carvetto::simulate {

enum class PreviewCells {
    All,      ///< Every cell: uncut stock keeps the stock top height.
    CutOnly,  ///< Only cells the tool lowered; uncut cells are transparent.
};

struct PreviewComponentOptions {
    std::string name{"Toolpath Preview"};
    PreviewCells cells{PreviewCells::All};
    /// Model height = simulator Z - `zOffset` (the inverse of the Z offset used when machining).
    double zOffset{0.0};
    relief::CombineMode mode{relief::CombineMode::Add};
};

/// Builds a component whose grid is the simulator's heightfield (same cell size; node (i, j) at the
/// centre of simulator cell (i, j)) holding `height(i, j) - zOffset` as a float. Status != Ok (BadParameter
/// for a non-finite offset, ZeroSize for an empty heightfield, TooLarge) leaves `out` unchanged.
relief::Status componentFromPreview(const Heightfield& surface, const PreviewComponentOptions& options,
                                    relief::Component& out);

}  // namespace carvetto::simulate
