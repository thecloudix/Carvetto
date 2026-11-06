// Determinism of the T-0171 shuffle: permutation hash with the test's exact arithmetic (indices instead of Contours).
#include <cstdint>
#include <cstdio>
#include <cstddef>
#include <utility>
#include <vector>
#include "test_rng.hpp"
int main() {
    for (int n : {8000, 16000, 10000, 20000}) {
        std::vector<int> cs;
        for (int i = 0; i < n; ++i) { cs.push_back(2 * i); cs.push_back(2 * i + 1); }
        carvetto::test::SplitMix rng{0x0171u + static_cast<std::uint64_t>(n)};
        for (std::uint32_t i = static_cast<std::uint32_t>(cs.size() - 1); i > 0; --i) {
            std::swap(cs[i], cs[static_cast<std::uint32_t>(rng.next() % (i + 1))]);
        }
        std::uint64_t h = 1469598103934665603ULL;
        for (int v : cs) { h ^= static_cast<std::uint64_t>(v); h *= 1099511628211ULL; }
        // also: how far is each hole from its outer in list order (mean |pos diff|)
        std::vector<long> pos(cs.size());
        for (std::size_t k = 0; k < cs.size(); ++k) pos[static_cast<std::size_t>(cs[k])] = static_cast<long>(k);
        double md = 0; long holeBeforeOuter = 0;
        for (int i = 0; i < n; ++i) { long d = pos[2*i+1] - pos[2*i]; md += d < 0 ? -d : d; holeBeforeOuter += d < 0; }
        std::printf("n=%d size_t=%zu hash=%016llx first=%d,%d,%d meanDist=%.0f holeBeforeOuter=%ld\n", n, sizeof(std::size_t),
                    (unsigned long long)h, cs[0], cs[1], cs[2], md / n, holeBeforeOuter);
    }
}
