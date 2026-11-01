#pragma once

#include <cmath>
#include <cstdint>

namespace carvetto::test {

/// SplitMix64: a tiny portable PRNG so generative tests draw identical values on every platform
/// and standard library (std:: distributions are implementation-defined).
struct SplitMix {
    std::uint64_t state;

    std::uint64_t next() {
        std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
        return z ^ (z >> 31);
    }
    /// Uniform double in [lo, hi).
    double uniform(double lo, double hi) {
        return lo + (hi - lo) * static_cast<double>(next() >> 11) * 0x1.0p-53;
    }
    /// Log-uniform double in [lo, hi), lo > 0.
    double logUniform(double lo, double hi) { return lo * std::pow(hi / lo, uniform(0.0, 1.0)); }
};

}  // namespace carvetto::test
