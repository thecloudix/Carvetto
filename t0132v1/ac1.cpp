#include "carvetto/document/carv_io.hpp"
#include "carvetto/document/commands.hpp"
#include "carvetto/document/document.hpp"
#include "carvetto/geometry/contour.hpp"
#include <cstdio>
#include <memory>
#include <span>
using namespace carvetto; using namespace carvetto::document; using carvetto::toolpath::Move;
#define EXPECT(c) do { if (!(c)) std::printf("FAIL line %d: %s\n", __LINE__, #c); else std::printf("ok   %s\n", #c); } while (0)
static ToolpathRecord rec(const char* n) { ToolpathRecord r; r.name = n; r.selection = VectorSelection::allVectors(); r.moves = shareMoves({Move::rapid({0, 0, 5})}); return r; }
int main() {
    Document doc;
    ObjectId o = doc.insertObject(Object::vector(geometry::Contour::rectangle(geometry::Box2{{0, 0}, {1, 1}}), doc.defaultLayer(), doc.activeSheet()));
    std::vector<ToolpathId> ids;
    for (const char* n : {"a", "b", "c", "d"}) { auto c = std::make_unique<AddToolpathCommand>(rec(n)); auto* p = c.get(); EXPECT(doc.execute(std::move(c))); ids.push_back(p->id()); }
    auto start = doc.toolpaths();
    // remove non-contiguous in reverse order, undo restores order
    EXPECT(doc.execute(std::make_unique<RemoveToolpathsCommand>(std::vector<ToolpathId>{ids[3], ids[0], ids[2]})));
    EXPECT(doc.toolpaths().size() == 1 && doc.toolpaths()[0].id == ids[1]);
    EXPECT(doc.undo()); EXPECT(doc.toolpaths() == start);
    EXPECT(doc.redo()); EXPECT(doc.toolpaths().size() == 1);
    EXPECT(doc.undo());
    // coalesce back to original
    ToolpathRecord b = doc.toolpaths()[1]; ToolpathRecord orig = b; b.name = "B2";
    EXPECT(doc.execute(std::make_unique<SetToolpathCommand>(b)));
    EXPECT(doc.execute(std::make_unique<SetToolpathCommand>(orig)));
    EXPECT(doc.toolpaths() == start);
    bool u = doc.undo(); std::printf("undo after net-noop coalesce: %d\n", u);
    EXPECT(doc.toolpaths() == start);
    bool r = doc.redo(); std::printf("redo: %d\n", r);
    EXPECT(doc.toolpaths() == start);
    EXPECT(doc.checkIntegrity().empty());
    // Move to end and back
    EXPECT(doc.execute(std::make_unique<MoveToolpathCommand>(ids[0], 3)));
    EXPECT(doc.toolpaths()[3].id == ids[0]);
    EXPECT(doc.undo()); EXPECT(doc.toolpaths() == start);
    // preset id colliding with layer / sheet / object
    ToolpathRecord p = rec("p"); p.id = ToolpathId{doc.defaultLayer().value};
    EXPECT(!doc.execute(std::make_unique<AddToolpathCommand>(p)));
    p.id = ToolpathId{o.value}; EXPECT(!doc.execute(std::make_unique<AddToolpathCommand>(p)));
    // add at index past end clamps
    { auto c = std::make_unique<AddToolpathCommand>(rec("e"), std::size_t{99}); EXPECT(doc.execute(std::move(c))); EXPECT(doc.toolpaths().back().name == "e"); }
    // add at index 0, undo, redo same id
    { auto c = std::make_unique<AddToolpathCommand>(rec("f"), std::size_t{0}); auto* pc = c.get(); EXPECT(doc.execute(std::move(c))); ToolpathId id = pc->id(); EXPECT(doc.toolpaths()[0].id == id); doc.undo(); EXPECT(!doc.findToolpath(id)); doc.redo(); EXPECT(doc.toolpaths()[0].id == id); }
    // add with an unallocated selection id: does the command refuse, or does integrity break?
    { ToolpathRecord bad = rec("bad"); bad.selection = {}; bad.selection.layers = {LayerId{doc.idGenerator().peek() + 100}};
      bool ok = doc.execute(std::make_unique<AddToolpathCommand>(bad));
      auto issues = doc.checkIntegrity();
      std::printf("add with unallocated layer id: executed=%d integrity issues=%zu\n", ok, issues.size());
      auto s = saveCarv(doc); std::printf("  save: %s\n", s.ok() ? "ok" : s.error().message.c_str());
      if (s.ok()) { auto l = loadCarv(std::span<const std::uint8_t>(s.value())); std::printf("  load: %s\n", l.ok() ? "ok" : l.error().message.c_str()); }
      if (ok) doc.undo(); }
    // feed override on a rapid / dwell seconds on feed: round trip inequality
    { Document d2; ToolpathRecord t = rec("x"); Move m = Move::rapid({1, 2, 3}); m.feedOverride = 100; Move f = Move::feed({1, 2, 3}); f.dwellSeconds = 2; Move a = Move::feed({4, 5, 6}); a.centre = {7, 8};
      t.moves = shareMoves({m, f, a}); d2.insertToolpath(t); auto s = saveCarv(d2); auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
      std::printf("ignored-field moves: save ok=%d, records equal after round trip=%d\n", s.ok(), l.value().document.toolpaths() == d2.toolpaths()); }
    // non-finite params / tool snapshot
    { Document d3; ToolpathRecord t = rec("nan"); std::get<toolpath::ProfileParams>(t.params).safeZ = std::numeric_limits<double>::quiet_NaN(); d3.insertToolpath(t);
      auto s = saveCarv(d3); std::printf("NaN safe_z: save %s\n", s.ok() ? "ok" : s.error().message.c_str()); if (s.ok()) { auto l = loadCarv(std::span<const std::uint8_t>(s.value())); std::printf("  load: %s\n", l.ok() ? "ok" : l.error().message.c_str()); } }
    { Document d3; ToolpathRecord t = rec("inf"); t.tool.diameter = std::numeric_limits<double>::infinity(); d3.insertToolpath(t);
      auto s = saveCarv(d3); std::printf("inf tool diameter: save %s\n", s.ok() ? "ok" : s.error().message.c_str()); if (s.ok()) { auto l = loadCarv(std::span<const std::uint8_t>(s.value())); std::printf("  load: %s\n", l.ok() ? "ok" : l.error().message.c_str()); } }
    { Document d3; ToolpathRecord t = rec("cuttingdata"); d3.insertToolpath(t); toolpath::ProfileParams pp; pp.cuttingData = tooling::CuttingData{}; t.params = pp; t.id = {};
      d3.insertToolpath(t); auto s = saveCarv(d3); auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
      std::printf("record with cuttingData: equal after round trip=%d (cuttingData not saved, by design)\n", l.value().document.toolpaths() == d3.toolpaths()); }
    // name with control chars / invalid utf-8
    { Document d4; ToolpathRecord t = rec("x\n\x01\xff\xfe"); d4.insertToolpath(t); auto s = saveCarv(d4); std::printf("bad-utf8 name: save %s\n", s.ok() ? "ok" : s.error().message.c_str());
      if (s.ok()) { auto l = loadCarv(std::span<const std::uint8_t>(s.value())); std::printf("  load: %s %s\n", l.ok() ? "ok" : l.error().message.c_str(), l.ok() && l.value().document.toolpaths()[0].name == t.name ? "(name exact)" : ""); } }
    // deleted vector stays referenced, save/load ok
    { Document d5; ObjectId v = d5.insertObject(Object::vector(geometry::Contour::rectangle(geometry::Box2{{0, 0}, {1, 1}}), d5.defaultLayer(), d5.activeSheet()));
      ToolpathRecord t = rec("sel"); t.selection = {}; t.selection.objects = {v}; d5.execute(std::make_unique<AddToolpathCommand>(t));
      d5.execute(std::make_unique<RemoveObjectsCommand>(std::vector<ObjectId>{v}));
      auto s = saveCarv(d5); auto l = loadCarv(std::span<const std::uint8_t>(s.value()));
      std::printf("dangling selection: save ok=%d load %s\n", s.ok(), l.ok() ? "ok" : l.error().message.c_str()); }
    return 0;
}
