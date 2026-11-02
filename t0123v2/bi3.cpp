#include <cstdio>
#include <limits>
#include "carvetto/toolpath/box_index.hpp"
using namespace carvetto; using geometry::Box2;
int main(){ const double inf=std::numeric_limits<double>::infinity();
 toolpath::BoxIndex a(std::vector<Box2>{Box2{{-1e308,0},{1e308,1}}});
 toolpath::BoxIndex b(std::vector<Box2>{Box2{{-inf,-1},{inf,1}}});
 std::printf("wide-only query size %zu (brute 1); inf-only query size %zu (brute 1); containing %zu\n", a.query(Box2{{0,0},{1,1}}).size(), b.query(Box2{{0,0},{1,1}}).size(), b.containing({0,0}).size()); }
