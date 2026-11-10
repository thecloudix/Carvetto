#pragma once

#include <cstdint>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/polyline.hpp"
#include "carvetto/geometry/shapes.hpp"

/// Vector texture generator (T-0251): wavy fill lines over a rectangle, as open 2D polylines.
///
/// Model. In a frame rotated by `angle` about the region centre, line k runs along local x at
/// local y_k; successive lines are `spacing_k` apart, where
/// `spacing_k = maxSpacing * (1 - 0.9 * spacingVariation * u_k)` with `u_k` in [0,1) from the
/// seeded generator (so `maxSpacing` is a true maximum and, with variation 0, the exact pitch).
/// The line is `y_k + a * sin(2*pi*(x/wavelength + phase_k))`, sampled 24 times per wavelength
/// (peaks are sampled exactly when `noise` is 0). `noise` in [0,1] randomises each line's phase
/// (by `noise` of a full wave) and each wave's amplitude (`a = amplitude * (1 - noise*v)`, so
/// `amplitude` stays the maximum). The result is clipped to `region`; a line leaving and
/// re-entering gives several polylines.
///
/// Determinism. The result depends only on the arguments: the random numbers come from an
/// in-repo SplitMix64 hash of (seed, line, wave), and the sine/cosine are in-repo polynomial
/// evaluations (only + - * / sqrt), never `<random>` distributions or libm trigonometry, so the
/// vertices are bit-identical on every platform (the build disables FP contraction for this file).
namespace carvetto::geometry {

struct VectorTextureParams {
    double angle{0.0};             ///< radians, line direction, in [-pi/2, pi/2]
    double maxSpacing{5.0};        ///< mm, > 0
    double spacingVariation{0.0};  ///< 0..1
    double wavelength{10.0};       ///< mm, > 0
    double amplitude{1.0};         ///< mm, >= 0 (0 gives straight lines)
    double noise{0.0};             ///< 0..1
    std::uint64_t seed{1};
};

/// Upper bound on generated sample vertices; larger requests return an error.
inline constexpr std::size_t kVectorTextureMaxVertices = 1'000'000;

/// Open polylines (every one with >= 2 vertices), lines in increasing local-y order.
/// An empty/degenerate `region` or invalid parameters (non-finite, out of range, more than
/// `kVectorTextureMaxVertices` vertices) return an error message.
ShapeResult<std::vector<Polyline>> generateVectorTexture(const Box2& region,
                                                         const VectorTextureParams& p);

/// The same, as open line-span contours ready for `Object::vector`.
ShapeResult<std::vector<Contour>> generateVectorTextureContours(const Box2& region,
                                                                const VectorTextureParams& p);

namespace detail {
/// SplitMix64 finaliser of `x` (stateless; exposed for tests).
std::uint64_t splitmix64(std::uint64_t x);
/// sin(2*pi*t) for any finite t, by symmetry reduction and a polynomial (|error| < 1e-12).
double sinTurns(double t);
}  // namespace detail

}  // namespace carvetto::geometry
