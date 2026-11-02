// T-0136 verifier: base files for fuzz3.
#include <cstdio>
#include <fstream>
#include <span>
#include "carv_zip.hpp"
#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/document/toolpath_record.hpp"
using namespace carvetto; using namespace carvetto::document; using toolpath::Move; using Bytes = std::vector<std::uint8_t>;
static void put(const char* n, const Bytes& b) { std::ofstream o(n, std::ios::binary); o.write((const char*)b.data(), (std::streamsize)b.size()); std::printf("%s %zu\n", n, b.size()); }
int main() {
    Document d;
    ToolpathRecord a; a.name = "Prof"; a.selection = VectorSelection::allVectors();
    a.tool.name = "EM6"; a.tool.toolNumber = 2; a.tool.diameter = 6; a.tool.feedRate = 1000; a.tool.plungeRate = 300; a.tool.spindleSpeed = 18000;
    auto& pp = std::get<toolpath::ProfileParams>(a.params); pp.safeZ = 7.25; pp.allowance = 0.125; pp.startPosition = {3.5, -2.25}; pp.leads.angle = 0.7; pp.tabs.length = 4.5;
    a.moves = shareMoves({Move::rapid({0, 0, 5}), Move::plunge({1, 2, -1}, 300.0), Move::feed({4, 2, -1}, 650.0), Move::feed({5, 2, -1}, 650.0), Move::arcCW({6, 2, -1}, {5.5, 2}), Move::arcCCW({7, 3, -1}, {6.5, 2.5}, 400.0), Move::dwell({7, 3, -1}, 0.25), Move::retract({7, 3, 5}), Move::rapid({0, 0, 5})});
    d.insertToolpath(a);
    ToolpathRecord b = a; b.id = {}; b.name = "Pock"; toolpath::PocketParams pk; pk.rasterAngle = 0.5; pk.safeZ = 6.5; pk.rampDistance = 3.25; b.params = pk; b.moves = shareMoves({Move::rapid({1, 1, 5}), Move::feed({2, 1, -2}, 500.0), Move::feed({2, 2, -2})});
    d.insertToolpath(b);
    ToolpathRecord c = a; c.id = {}; c.name = "Unc"; c.moves = nullptr; c.tool = {}; d.insertToolpath(c);
    ToolpathRecord e = a; e.id = {}; e.name = "Empty"; e.moves = shareMoves({}); d.insertToolpath(e);
    auto s = saveCarv(d, CarvExtras{}.withMember("extra/notes.txt", Bytes{'h', 'i'}));
    if (!s.ok()) { std::printf("save failed: %s\n", s.error().message.c_str()); return 1; }
    put("base_native.carv", s.value());
    // Legacy: add interim member to a document without records.
    Document d2; auto s2 = saveCarv(d2); auto ms = detail::readZip(s2.value()).members;
    std::string im = "{\"format\":\"carvetto-toolpath\",\"format_version\":1,\"operation\":\"pocket\",\"name\":\"i\",\"params\":{\"safe_z\":5,\"raster_angle\":30,\"tolerance\":{\"chordal\":0.02}},\"safe_z\":5,"
                     "\"tool\":{\"name\":\"x\",\"tool_number\":1,\"diameter\":6,\"feed_rate\":1000,\"plunge_rate\":300,\"spindle_speed\":1},\"moves\":[[\"rapid\",0,0,5],[\"plunge\",1,1,-1],[\"feed\",2,1,-1,{\"feed\":400}],[\"arc_cw\",3,2,-1,2.5,1.5],[\"dwell\",3,2,-1,0.5],[\"retract\",3,2,5]]}";
    ms.push_back({"toolpaths/0001.json", Bytes(im.begin(), im.end())});
    ms.push_back({"toolpaths/0002.json", Bytes(im.begin(), im.end())});
    auto z = detail::writeZip(ms); put("base_interim.carv", z.bytes);
    auto l = loadCarv(std::span<const std::uint8_t>(z.bytes)); std::printf("interim loads: %d %s\n", l.ok(), l.ok() ? "" : l.error().message.c_str());
}
