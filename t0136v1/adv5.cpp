// T-0136 verifier: (c) names whose stored form is longer than in memory (escapes, U+FFFD).
#include <cstdio>
#include <span>
#include <string>
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
using namespace carvetto; using namespace carvetto::document; using toolpath::Move;
static void t(const char* what, const std::string& name) {
    Document d; ToolpathRecord r; r.name = name; r.selection = VectorSelection::allVectors(); d.insertToolpath(r);
    auto s = saveCarv(d);
    if (!s.ok()) { std::printf("%-34s REFUSED: %.260s\n", what, s.error().message.c_str()); return; }
    auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
    std::printf("%-34s saved %zu B, load %s\n", what, s.value().size(), l.ok() ? "OK" : l.error().message.c_str());
}
int main() {
    t("ctrl 0x01 x 200000", std::string(200000, '\x01'));
    t("ctrl 0x01 x 180000 (6x=1.08M)", std::string(180000, '\x01'));
    t("quote x 600000 (2x)", std::string(600000, '"'));
    t("0xFF x 400000 (3x)", std::string(400000, '\xFF'));
    t("0xFF x 350000 (3x=1.05M)", std::string(350000, '\xFF'));
}
