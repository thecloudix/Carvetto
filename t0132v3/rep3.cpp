// T-0132 verifier round 3: highly repetitive but legitimate documents vs. the job.json 200:1 ratio.
#include <cstdio>
#include <span>
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
using namespace carvetto; using namespace carvetto::document;
int main() {
    for (int n : {1000, 10000, 50000}) {
        Document d;
        for (int i = 0; i < n; ++i) d.insertObject(Object::vector(geometry::Contour::rectangle(geometry::Box2{{0, 0}, {10, 10}}), d.defaultLayer(), d.activeSheet()));
        auto s = saveCarv(d);
        if (!s.ok()) { std::printf("%d identical rectangles: REFUSED %s\n", n, s.error().message.c_str()); continue; }
        auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
        std::printf("%d identical rectangles: saved %zu B, load %s\n", n, s.value().size(), l.ok() ? "OK" : l.error().message.c_str());
    }
}
