#pragma once

#include <cstddef>
#include <cstdint>

#include "carvetto/simulate/stock.hpp"
#include "carvetto/simulate/tool_shape.hpp"

// The swept-volume kernel (T-0222, ADR-0032 §2-§4): lowers a heightfield to the surface swept by a
// tool profile along a straight segment or an arc, exactly at every cell centre. Internal.

namespace carvetto::simulate {

enum class SweepMode {
    Cut,    ///< Lower the heightfield; the result is the number of cells lowered.
    Probe,  ///< Leave it alone; the result is the number of cells the tool would lower by more than the tolerance.
};

/// Geometry of an arc: centre, start and end radius vectors (both of length `rho`), direction
/// (+1 counter-clockwise, -1 clockwise), `full` for a complete circle, and the tip Z at both ends.
struct ArcGeom {
    double cx{0.0};
    double cy{0.0};
    double ax{0.0};
    double ay{0.0};
    double bx{0.0};
    double by{0.0};
    double rho{0.0};
    int dir{1};
    bool full{false};
    double za{0.0};
    double zb{0.0};
};

class Sweeper {
public:
    explicit Sweeper(Heightfield& field) : f_(field) {}

    /// Tolerance of Probe mode and the id written into cells lowered in Cut mode.
    void setProbeTolerance(double tol) { probeTol_ = tol; }
    void setPassId(std::uint32_t id) { passId_ = id; }
    /// Cells evaluated so far (the work counter of the CPU budget).
    double work() const { return work_; }
    void resetWork() { work_ = 0.0; }

    /// Tool tip moves on the straight line a -> b (Z linear in the XY travel).
    std::size_t segment(const ToolShape& tool, double ax, double ay, double az, double bx, double by, double bz,
                        SweepMode mode);
    /// Tool tip moves along an arc. Planar arcs (za == zb) are swept exactly; a helix is swept as
    /// the chords of an inscribed polygon with sagitta <= cellSize / 64 (ADR-0032 §3).
    std::size_t arc(const ToolShape& tool, const ArcGeom& arc, SweepMode mode);

private:
    template <SweepMode M>
    std::size_t segmentImpl(const ToolShape& tool, double ax, double ay, double az, double bx, double by, double bz);
    template <SweepMode M>
    std::size_t planarLeaf(const ToolShape& tool, const ArcGeom& g);
    template <SweepMode M>
    std::size_t arcRecurse(const ToolShape& tool, const ArcGeom& g, bool helix, int depth);
    template <SweepMode M>
    std::size_t arcImpl(const ToolShape& tool, const ArcGeom& g);
    template <SweepMode M>
    void applyCell(std::size_t idx, double value, std::size_t& count);

    Heightfield& f_;
    double probeTol_{1e-3};
    std::uint32_t passId_{0};
    double work_{0.0};
};

}  // namespace carvetto::simulate
