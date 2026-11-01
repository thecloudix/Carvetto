#pragma once
#include <vector>
// Plain span representation shared between the branch driver and main's wrapper TU.
struct PSpan {
    int kind;  // 0 line (sx,sy,ex,ey), 1 arc (cx,cy,r,sa,ea,dir 0=CCW 1=CW), 2 other
    double a[8];
};
struct PContour {
    std::vector<PSpan> s;
    bool closed{false};
};
// side 0 Left 1 Right; join 0 Round 1 Sharp 2 Square
std::vector<PContour> mainOffset(const PContour& in, double d, int side, int join, double tolDist, double mitre);
std::vector<PContour> oldOffset(const PContour& in, double d, int side, int join, double tolDist, double mitre);
