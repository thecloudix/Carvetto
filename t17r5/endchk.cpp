#include <cstdio>
#include <cmath>
#include <random>
#include "carvetto/geometry/span.hpp"
using namespace carvetto::geometry;
int main(){ std::mt19937_64 r(5); std::uniform_real_distribution<double> u(-3,3); long moved=0,tot=0;
 for(int i=0;i<100000;++i){ ArcSpan a{{u(r)*1e4,u(r)*1e4}, std::fabs(u(r))*10+0.1, u(r), u(r), i%2?ArcDirection::CCW:ArcDirection::CW};
  Vec2 e=a.endPoint(); for(int k=-64;k<=64;++k){ ArcSpan c=a; c.startAngle=a.startAngle+k*1e-15; ++tot; if(!(c.endPoint()==e)) ++moved; } }
 std::printf("end moved %ld of %ld\n",moved,tot);}
