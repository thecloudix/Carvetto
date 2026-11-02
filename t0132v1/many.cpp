#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include <cstdio>
#include <span>
using namespace carvetto::document; using carvetto::toolpath::Move;
int main(int, char** argv) {
    int n = std::atoi(argv[1]);
    Document doc;
    for (int i = 0; i < n; ++i) { ToolpathRecord r; r.name = "tp"; r.selection = VectorSelection::allVectors(); r.moves = shareMoves({Move::rapid({0, 0, 5})}); doc.insertToolpath(r); }
    auto s = saveCarv(doc);
    if (!s.ok()) { std::printf("n=%d save refused: %s\n", n, s.error().message.c_str()); return 0; }
    auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
    std::printf("n=%d saved %zu B; load: %s\n", n, s.value().size(), l.ok() ? "OK" : l.error().message.c_str());
}
