#pragma once

// Incremental Delaunay triangulation of points in the plane (internal, T-0152).
//
// Bowyer-Watson insertion with exact predicates (robust_predicates.hpp): a new point is located
// by a visibility walk, every triangle whose open circumdisk contains it is removed (the cavity,
// grown from the containing triangle across edges) and the cavity is re-triangulated as a fan
// around the point. With exact in-circle tests the cavity is star-shaped from the point, so the
// fan is always valid; co-circular configurations are resolved by insertion order. All points
// live inside a large "super triangle" whose three vertices (ids 0, 1, 2) are not part of the
// input; triangles touching them form the outside of the convex hull. A point equal to an
// existing vertex is not inserted again: its id is returned. Deterministic for a given sequence
// of insertions.

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::geometry::detail {

class Delaunay {
public:
    /// Triangle with counter-clockwise vertices; `n[i]` is the neighbour across the edge opposite
    /// `v[i]` (the edge v[i+1] -> v[i+2]), or -1 on the outside of the super triangle.
    struct Triangle {
        std::array<int, 3> v{};
        std::array<int, 3> n{};
    };

    /// Prepares a triangulation for points inside `bounds` (non-empty, finite).
    explicit Delaunay(const Box2& bounds);

    /// Inserts `p` (which must lie inside the bounds given at construction) and returns its
    /// vertex id; when `p` equals an existing vertex nothing changes and that vertex's id is
    /// returned. `hintVertex` (any vertex id, or -1) seeds the point location.
    int insert(Vec2 p, int hintVertex = -1);

    /// True for the three helper vertices of the super triangle.
    static bool isSuper(int v) { return v >= 0 && v < 3; }

    std::size_t vertexCount() const { return points_.size(); }
    Vec2 point(int v) const { return points_[static_cast<std::size_t>(v)]; }

    std::size_t triangleSlots() const { return tris_.size(); }
    bool alive(int t) const { return alive_[static_cast<std::size_t>(t)] != 0; }
    const Triangle& triangle(int t) const { return tris_[static_cast<std::size_t>(t)]; }

    /// A live triangle having `v` as a vertex.
    int incidentTriangle(int v) const { return incident_[static_cast<std::size_t>(v)]; }

    /// The live triangle that has the directed edge a -> b (counter-clockwise), or -1 when the
    /// edge a-b is not in the triangulation. Costs O(smaller degree of a and b).
    int triangleWithEdge(int a, int b) const;

    /// Index (0..2) of vertex `v` in triangle `t`, or -1.
    int indexOf(int t, int v) const;

    /// Triangles created since the last `takeCreated()` (ids may repeat or have died since).
    std::vector<int> takeCreated();

    /// Vertices on the boundary of the cavity of the most recent insertion (empty when the last
    /// insertion found an existing vertex).
    const std::vector<int>& lastCavityRing() const { return ring_; }

    /// Triangles created by the most recent insertion.
    const std::vector<int>& lastNewTriangles() const { return newTris_; }

    /// Edges (vertex pairs) removed by the most recent insertion: those inside its cavity. An
    /// edge between two existing vertices is never created again (new edges all end at the new
    /// point), so a removed edge stays out of the triangulation for good.
    const std::vector<std::pair<int, int>>& lastRemovedEdges() const { return removed_; }

private:
    int locate(Vec2 p, int startTri) const;
    int newTriangle();

    std::vector<Vec2> points_;
    std::vector<int> incident_;
    std::vector<Triangle> tris_;
    std::vector<char> alive_;
    std::vector<int> free_;
    std::vector<unsigned> mark_;
    unsigned epoch_{0};
    struct BoundaryEdge {
        int a;
        int b;
        int outside;
        int old;
    };
    // Scratch buffers of `insert`, kept to avoid allocations.
    std::vector<int> cavity_;
    std::vector<BoundaryEdge> boundary_;
    std::vector<std::pair<int, int>> byStart_;
    std::vector<int> startOf_;  // by vertex: new triangle whose cavity-boundary edge starts there
    std::vector<int> endOf_;    // by vertex: new triangle whose cavity-boundary edge ends there
    std::vector<int> created_;
    std::vector<int> ring_;
    std::vector<int> newTris_;
    std::vector<std::pair<int, int>> removed_;
    int lastTri_{0};
};

}  // namespace carvetto::geometry::detail
