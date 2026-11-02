#include <cmath>
#include <cstdio>
#include <limits>
#include <span>
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
using namespace carvetto; using namespace carvetto::document; using carvetto::toolpath::Move;
static void show(const char* label, ToolpathRecord r) {
    Document d; d.insertToolpath(r); auto s = saveCarv(d);
    if (!s.ok()) { std::printf("%s: refused %s\n", label, s.error().message.c_str()); return; }
    for (auto& m : detail::readZip(s.value()).members) if (m.name == "job.json") {
        std::string j(m.data.begin(), m.data.end()); auto p = j.find("\"toolpaths\""); std::printf("%s: %s\n", label, j.substr(p, 900).c_str()); }
    auto l = loadCarv(std::span<const std::uint8_t>(s.value())); const auto& t = l.value().document.toolpaths()[0];
    std::printf("  loaded: allowance=%g safeZ=%g diam=%g feed=%g name-bytes=", std::get_if<toolpath::ProfileParams>(&t.params) ? std::get<toolpath::ProfileParams>(t.params).allowance : -1, t.safeZ(), t.tool.diameter, t.tool.feedRate);
    for (unsigned char c : t.name) std::printf("%02x", c); std::printf("\n");
}
int main() {
    const double nan = std::numeric_limits<double>::quiet_NaN(), inf = std::numeric_limits<double>::infinity();
    ToolpathRecord base; base.name = "t"; base.selection = VectorSelection::allVectors(); base.tool.diameter = 6; base.moves = shareMoves({Move::rapid({1,2,30})});
    { auto r = base; std::get<toolpath::ProfileParams>(r.params).allowance = nan; show("allowance NaN", r); }
    { auto r = base; std::get<toolpath::ProfileParams>(r.params).safeZ = inf; show("safeZ inf", r); }
    { auto r = base; r.tool.diameter = nan; show("diam NaN", r); }
    { auto r = base; r.tool.feedRate = -inf; show("feed -inf", r); }
    { auto r = base; r.name = "\xff\xfe bad"; show("bad utf8", r); }
}
