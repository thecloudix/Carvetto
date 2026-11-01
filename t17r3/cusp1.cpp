// T-0017 round-2 verifier: 0xe300070e input (the branch's own regression test), upper row moved up by g:
// Round arcs about vertices 2d + g apart meet at a cusp. Exact, continuous polyline input.
#include <cmath>
#include <cstdlib>
#include <cstdio>
#include <variant>
#include <vector>
#include "carvetto/geometry/offset.hpp"
using namespace carvetto::geometry;
int main(int argc, char** argv) {
    const double d = 1.0328719931380388;
    std::vector<Vec2> P = {{6.6283223938981575, -1.2779953526139643e-11},  {5.023667459767422, 2.1623209668586711e-11},
         {6.9756096886500769, 34.51051481312367},        {6.3189901091486398, 34.312480314617368},
         {4.6667279880904458, 2.0657439862650513},       {0.69533962297162577, 2.0657439862566656},
         {1.9562893692302896, 33.119825834028326},       {7.04511858784476, 33.809684447840162},
         {6.6283223939411231, 2.0657439862576599},       {5.0236674597854529, 2.0657439862438181},
         {5.4657050614391105, 36.723239966815832},       {4.5703929732694029, 36.057839455949605},
         {4.8076418394176894, 3.1120034057046923e-12},   {8.8588515885397729, -8.7094539924375882e-13},
         {9.7907278097679189, -35.00186354855979},       {1.9705389447012767, -34.43692474497638},
         {0.69533962299185814, -2.6581188530012263e-11}, {4.6667279880936041, -3.0346127572489825e-11},
         {3.1154143986946496, 34.81196163001529},        {4.5941198790408642, 35.289788649021553},
         {5.0236674597518789, 1.9744273286251705e-11},   {6.6283223939582836, -1.3296097279548633e-11},
         {6.980860505868165, 32.133493994828015},        {9.603450489764624, 31.205313249352219},
         {8.858851588581409, 2.0657439862506815},        {4.8076418393909703, 2.0657439862946823}};
    for (double g : {std::atof(argv[1])}) {
        Contour in;
        std::vector<Vec2> Q;
        for (Vec2 p : P) Q.push_back(std::fabs(p.y - 2 * d) < 1e-6 ? Vec2{p.x, 2 * d + g} : p);
        for (std::size_t i = 1; i < Q.size(); ++i) in.spans.push_back(LineSpan{Q[i - 1], Q[i]});
        auto out = offsetOpenContour(in, d, OffsetSide::Left, JoinType::Round, Tolerance::defaults(), 3.8787999757578069);
        double worst = 0;
        for (auto& c : out)
            for (std::size_t i = 0; i + 1 < c.spans.size(); ++i)
                worst = std::max(worst, distance(endPoint(c.spans[i]), startPoint(c.spans[i + 1])));
        std::printf("g = %-6g: %zu chains, max junction gap %.3g mm\n", g, out.size(), worst);
    }
}
