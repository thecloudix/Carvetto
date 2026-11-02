#include <cstdio>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/contour_ops.hpp"
#include "carvetto/geometry/offset.hpp"
#include "carvetto/geometry/boolean.hpp"
using namespace carvetto::geometry;
int main(){
  Tolerance tol;
  Contour c = Contour::rectangle(Box2{{0,0},{18,14}}); c.closed=false;
  auto band = offsetContours({c}, 1.6, JoinType::Round);
  for (auto& b: band) std::printf("band area %.2f ori %d\n", area(b), (int)orientation(b,tol));
  std::size_t o=0; for (std::size_t k=1;k<band.size();++k) if (area(band[k])>area(band[o])) o=k;
  for (std::size_t k=0;k<band.size();++k) setOrientation(band[k], k==o?Orientation::CCW:Orientation::CW, tol);
  auto u = unionContours(band);
  for (auto& b: u) std::printf("union area %.2f ori %d\n", area(b), (int)orientation(b,tol));
}
