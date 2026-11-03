#include <cstdio>
#include <cstdlib>
#include <cstring>
#include "robust_predicates.hpp"
using carvetto::geometry::Vec2; using namespace carvetto::geometry::detail;
int main(){ char k[4]; while (std::scanf("%3s",k)==1){ if(k[0]=='O'){double v[6];for(double&x:v)std::scanf("%la",&x);std::printf("O %a %a %a %a %a %a %d\n",v[0],v[1],v[2],v[3],v[4],v[5],orient2dSign({v[0],v[1]},{v[2],v[3]},{v[4],v[5]}));}
 else {double v[8];for(double&x:v)std::scanf("%la",&x);std::printf("C %a %a %a %a %a %a %a %a %d\n",v[0],v[1],v[2],v[3],v[4],v[5],v[6],v[7],incircleSign({v[0],v[1]},{v[2],v[3]},{v[4],v[5]},{v[6],v[7]}));} } }
