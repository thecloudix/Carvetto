#include <cstdio>
#include <cmath>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
static double area(const Contour& c){ double a=0; for(auto& s:c.spans){ Vec2 p=startPoint(s), q=endPoint(s); a+=p.x*q.y-q.x*p.y;} return a/2; }
static void rep(const char* n, const std::vector<Contour>& o){ std::printf("%-40s rings %zu", n, o.size()); for(auto& c:o) std::printf(" [%zu %.3e]", c.spans.size(), area(c)); std::printf("\n"); }
int main(){ Tolerance t{};
  rep("empty input", offsetContours({}, -1.0, JoinType::Round, t));
  rep("tiny circle r=0.01 d=0.02", offsetContours({Contour::circle({0,0},0.01)}, -0.02, JoinType::Round, t));
  rep("tiny circle r=0.01 d=0.005", offsetContours({Contour::circle({0,0},0.01)}, -0.005, JoinType::Round, t));
  rep("far circle (1e5,1e5) r=50 d=60", offsetContours({Contour::circle({1e5,1e5},50)}, -60, JoinType::Round, t));
  rep("far circle (1e5,1e5) r=50 d=49", offsetContours({Contour::circle({1e5,1e5},50)}, -49, JoinType::Round, t));
  rep("huge circle r=2000 d=2000.1", offsetContours({Contour::circle({0,0},2000)}, -2000.1, JoinType::Round, t));
  rep("huge circle r=2000 d=1999", offsetContours({Contour::circle({0,0},2000)}, -1999, JoinType::Round, t));
  rep("circle r=17 d=1e6", offsetContours({Contour::circle({0,0},17)}, -1e6, JoinType::Sharp, t));
  rep("circle r=17 d=0", offsetContours({Contour::circle({0,0},17)}, 0.0, JoinType::Round, t));
  rep("circle r=17 d=NaN", offsetContours({Contour::circle({0,0},17)}, std::nan(""), JoinType::Round, t));
  { Contour z; z.closed=true; z.spans.emplace_back(LineSpan{{0,0},{10,0}}); z.spans.emplace_back(LineSpan{{10,0},{0,0}}); rep("zero-area closed d=-1", offsetContours({z}, -1, JoinType::Round, t)); }
  rep("square 10 d=5.0005 sharp", offsetContours({Contour::rectangle(Box2{{0,0},{10,10}})}, -5.0005, JoinType::Sharp, t));
  rep("square 10 d=4.99 sharp", offsetContours({Contour::rectangle(Box2{{0,0},{10,10}})}, -4.99, JoinType::Sharp, t));
  rep("rect 10x2 d=1.0001 round", offsetContours({Contour::rectangle(Box2{{0,0},{10,2}})}, -1.0001, JoinType::Round, t));
  // two disjoint circles, different radii, both orientations mixed? (outermost defines)
  rep("two circles r5,r2 d=3", offsetContours({Contour::circle({0,0},5), Contour::circle({20,0},2)}, -3, JoinType::Round, t));
  rep("open path outline d=-1", offsetContours({[]{Contour c; c.append(LineSpan{{0,0},{10,0}}); return c;}()}, -1, JoinType::Round, t));
}
