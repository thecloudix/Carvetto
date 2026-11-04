// Verifier T-0154 r4 (allowlist round): attacks outside the per-line allowlist.
#include <cstdio>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"
using namespace carvetto;
using toolpath::Move; using toolpath::Toolpath; using toolpath::ToolSnapshot;
static ToolSnapshot spin(int n, const char* name="Mill") { ToolSnapshot t; t.name=name; t.toolNumber=n; t.diameter=3; t.feedRate=600; t.plungeRate=200; t.spindleSpeed=12000; return t; }
static ToolSnapshot dragT(int n=3, std::string name="Drag") { ToolSnapshot t=spin(n); t.name=name; t.spindleSpeed=0; t.spindleOff=true; t.toolType=toolpath::kDiamondDragToolType; return t; }
static Toolpath cut(std::string nm, ToolSnapshot t){ Toolpath tp; tp.name=nm; tp.tool=t; tp.moves={Move::rapid({1,1,5}),Move::plunge({1,1,-0.1}),Move::feed({2,1,-0.1}),Move::retract({2,1,5})}; return tp; }
static const std::string kMoves="begin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\nbegin FEED_MOVE\n\"G1[X][Y][Z][F]\"\nend\n";
static std::string base(std::string settings, std::string header, std::string tc, std::string pause, std::string unlock, std::string vars="") {
  std::string s = settings + "VAR SPINDLE_SPEED = [S|A| S|1.0]\nVAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\nVAR FEED_RATE = [F|C| F|1.0]\n" + vars;
  s += "begin HEADER\n"+header+"end\nbegin TOOLCHANGE\n"+tc+"end\nbegin SPINDLE_ON\n\"M3[S]\"\nend\nbegin NEW_SEGMENT\n\"\"\nend\n";
  s += "begin SPINDLE_OFF_PAUSE\n"+pause+"end\nbegin SPINDLE_UNLOCK_PAUSE\n"+unlock+"end\n";
  s += "begin FOOTER\n\"M5\"\nend\n" + kMoves;
  return s;
}
static void run(const std::string& label, const std::string& pptext, const std::vector<Toolpath>& tps) {
  auto p = post::PostProcessor::parse(pptext);
  if (!p.ok()) { printf("[%s] PARSE ERROR %s\n", label.c_str(), p.error().describe().c_str()); return; }
  auto r = p.value().run(tps, test::fixtureContext());
  if (!r.ok()) { printf("[%s] REFUSED: %s\n", label.c_str(), r.error().describe().c_str()); return; }
  std::string t = r.value().text; std::string vis;
  for (char c : t) { if (c=='\r') vis += "<CR>"; else vis += c; }
  printf("[%s] ACCEPTED:\n%s----\n", label.c_str(), vis.c_str());
}
int main() {
  const std::string Hp="\"(h)\"\n", Tp="\"M5\"\n\"M0\"\n", Pp="\"M5\"\n\"(pause)\"\n\"M0\"\n", Up="\"M5\"\n\"(unlock)\"\n\"M0\"\n";
  std::vector<Toolpath> first{cut("D", dragT())};
  std::vector<Toolpath> spinThenDrag{cut("A", spin(1)), cut("D", dragT())};
  // 1. LINE_ENDING carries literal text after every line (not seen by the line check)
  run("A1 LINE_ENDING ' M3 S12000[13][10]'", base("LINE_ENDING = \" M3 S12000[13][10]\"\n", Hp, Tp, Pp, Up), first);
  run("A1b LINE_ENDING '[10]M3[10]'", base("LINE_ENDING = \"[10]M3[10]\"\n", Hp, Tp, Pp, Up), first);
  run("A1c LINE_ENDING '~[10]'", base("LINE_ENDING = \"~[10]\"\n", Hp, Tp, Pp, Up), first);
  run("A1d LINE_ENDING '[10][133]'", base("LINE_ENDING = \"[10][133]\"\n", Hp, Tp, Pp, Up), first);
  // 2. ';'-style custom post (no '(' anywhere): text after M0 is treated as a message on every controller
  const std::string Hs="\"; h\"\n", Ts="\"M5\"\n\"M0\"\n", Us="\"M5\"\n\"; unlock\"\n\"M0\"\n";
  for (std::string nm : {"M3 S18000", "x M3", "S1", "P1", "Drag S30"}) {
    run("A2 ;-post 'M0 [TOOLNAME]' name '"+nm+"'", base("", Hs, Ts, "\"M5\"\n\"M0 [TOOLNAME]\"\n", Us), {cut("D", dragT(3, nm))});
    run("A2b ;-post 'M0 Check [TOOLPATH_NAME]' tp name '"+nm+"'", base("", Hs, Ts, "\"M5\"\n\"M0 Check [TOOLPATH_NAME]\"\n", Us), {cut(nm, dragT(3))});
  }
  // same pause template on a paren post: refused?
  run("A2c paren-post 'M0 [TOOLNAME]' name 'M3 S18000'", base("", Hp, Tp, "\"M5\"\n\"M0 [TOOLNAME]\"\n", Up), {cut("D", dragT(3, "M3 S18000"))});
  // 3. VAR prefix with a leading '/' (block delete) on the M0/M5 lines
  run("A3 [N] prefix '/N' on '[N] M5' '[N] M0'", base("", Hp, Tp, "\"[N] M5\"\n\"[N] M0\"\n", "\"[N] M5\"\n\"[N] M0\"\n", "VAR LINE_NUMBER = [N|A|/N|1.0]\n"), first);
  // 4. M0 glued to a tag: "M0[T]" with tool 5 writes M05
  run("A4 'M0[T]' tool 5", base("", Hp, Tp, "\"M5\"\n\"M0[T]\"\n", Up, "VAR TOOL_NUMBER = [T|A||1.0]\n"), {cut("D", dragT(5))});
  run("A4b 'M0[T]' tool 3", base("", Hp, Tp, "\"M5\"\n\"M0[T]\"\n", Up, "VAR TOOL_NUMBER = [T|A||1.0]\n"), {cut("D", dragT(3))});
  // 5. Grbl reads >8 digits incl. leading zeros as 0: M000000005 -> M0 (M5 counted by the check)
  run("A5 'M000000005' as the only M5", base("", Hp, "\"M0\"\n", "\"M000000005\"\n\"M0\"\n", Up), spinThenDrag);
  run("A5b 'G00000091.1' (Grbl reads G91)", base("", Hp, Tp, Pp + "\"G00000091.1\"\n", Up), first);
  // 6. words on the M0 line that make Grbl reject the whole line
  run("A6 'M0 P1'", base("", Hp, Tp, "\"M5\"\n\"M0 P1\"\n", Up), first);
  run("A6b 'M0 T[T]' tool 300", base("", Hp, Tp, "\"M5\"\n\"M0 T[T]\"\n", Up, "VAR TOOL_NUMBER = [T|A||1.0]\n"), {cut("D", dragT(300))});
  run("A6c 'M0 H1'", base("", Hp, Tp, "\"M5\"\n\"M0 H1\"\n", Up), first);
  run("A6d 'M5 I1'", base("", Hp, Tp, "\"M5 I1\"\n\"M0\"\n", Up), spinThenDrag);
  // 7. false positives: Marlin emergency codes searched across a whole comment
  for (std::string nm : {"Drag 3mm 108", "Diamond 1mm 112 deg", "S12 drill M3", "Tip 0.1mm 410"})
    run("A7 paren comment name '"+nm+"'", base("", Hp, Tp, "\"M5\"\n\"(tool [TOOLNAME])\"\n\"M0\"\n", Up), {cut("D", dragT(3, nm))});
  // 8. X.5 in template
  run("A8 'G0 X.5'", base("", Hp, Tp, Pp + "\"G0 X.5\"\n", Up), first);
  run("A8b 'G0 X0.5'", base("", Hp, Tp, Pp + "\"G0 X0.5\"\n", Up), first);
  // 9. format padding: 99 integer digits for Z
  run("A9 Z format 20.3 (Grbl reads 0)", base("", Hp, Tp, Pp, Up, "VAR SAFE_Z = [SAFEZ|A| Z|20.3]\n") , first);
  // 10. ;-post M0 message with ( M3 S1000) in live text
  run("A10 ;-post 'G1[Z][F] [TOOLNAME]' name '( S1000)'", base("", Hs, Ts, "\"M5\"\n\"M0\"\n", Us) + "begin PLUNGE_MOVE\n\"G1[Z][F] [TOOLNAME]\"\nend\n", {cut("D", dragT(3, "( S1000)"))});
}
