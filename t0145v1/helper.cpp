#include <cstdio>
#include <cstdint>
#include "../../core/tests/geometry/cpu_scaling.hpp"
volatile std::uint64_t sink;
void spin(std::uint64_t n){ std::uint64_t x=88172645463325252ull; for(std::uint64_t i=0;i<n;++i){x^=x<<13;x^=x>>7;x^=x<<17;} sink=x; }
int main(){
  for (double unit : {1e5, 1e6, 1e7, 1e8}) for (int q=1;q<=2;++q) {
    double t0=carvetto::test::processCpuSeconds();
    auto t = carvetto::test::interleavedCpuSeconds(3, [&]{ spin((std::uint64_t)unit); }, [&]{ spin((std::uint64_t)(unit*(q==1?2:4))); });
    double el=carvetto::test::processCpuSeconds()-t0;
    std::printf("unit %.0e %s: reps %d small %.4g large %.4g pair %.3f ratio %.3f total %.2f s\n", unit, q==1?"linear":"quadr", t.reps, t.smallSeconds, t.largeSeconds, t.pairRatio, t.ratio(), el);
  }
  auto z = carvetto::test::interleavedCpuSeconds(2, []{}, []{});
  std::printf("empty: reps %d ratio %g\n", z.reps, z.ratio());
}
