// Verifier T-0154 r2: custom-post attacks on the spindle-off guard.
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
static std::string P="\"M5\"\n\"(pause)\"\n\"M0\"\n";
static std::string U="\"M5\"\n\"(unlock)\"\n\"M0\"\n";
static std::string mkpost(std::string header, std::string tc, std::string ns, std::string pause, std::string unlock, std::string extra="", std::string vars="") {
  std::string s = "VAR SPINDLE_SPEED = [S|A| S|1.0]\nVAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\nVAR FEED_RATE = [F|C| F|1.0]\n" + vars;
  s += "begin HEADER\n"+header+"end\nbegin TOOLCHANGE\n"+tc+"end\nbegin SPINDLE_ON\n\"M3[S]\"\nend\nbegin NEW_SEGMENT\n"+ns+"end\n";
  if (!pause.empty()) s += "begin SPINDLE_OFF_PAUSE\n"+pause+"end\n";
  if (!unlock.empty()) s += "begin SPINDLE_UNLOCK_PAUSE\n"+unlock+"end\n";
  s += "begin FOOTER\n\"M5\"\n\"M30\"\nend\n" + extra + kMoves;
  return s;
}
static void run(const std::string& label, const std::string& pptext, const std::vector<Toolpath>& tps) {
  auto p = post::PostProcessor::parse(pptext);
  if (!p.ok()) { printf("[%s] PARSE ERROR %s\n", label.c_str(), p.error().describe().c_str()); return; }
  auto r = p.value().run(tps, test::fixtureContext());
  if (!r.ok()) { printf("[%s] REFUSED: %s\n", label.c_str(), r.error().describe().c_str()); return; }
  printf("[%s] ACCEPTED:\n%s----\n", label.c_str(), r.value().text.c_str());
}
int main() {
  const std::string H="\"(h)\"\n", T="\"M5\"\n\"M6 T[T]\"\n", N="\"(seg)\"\n";
  std::vector<Toolpath> first{cut("D", dragT())};
  std::vector<Toolpath> second{cut("A", spin(1)), cut("D", dragT())};
  std::vector<Toolpath> same{cut("D1", dragT()), cut("D2", dragT())};
  std::vector<Toolpath> back{cut("D", dragT()), cut("A", spin(1))};
  struct C{const char* l; std::string line;};
  std::vector<C> lits{
    {"nested paren (a(b)M3 S1000)","\"(a (b) M3 S1000)\"\n"},
    {"nested paren2 ((x)M3)","\"((x)M3)\"\n"},
    {"; in non-marlin","\"; M3 S1000\"\n"},
    {"G0 ;M3","\"G0 X1 ; M3\"\n"},
    {"/M3","\"/M3\"\n"},{"/ M 3","\"/ M 3\"\n"},
    {"M#1 (LinuxCNC param)","\"#1=3\"\n\"M#1\"\n"},
    {"S#1","\"S#1\"\n"},
    {"M[1+2] (expr)","\"M[1+2]\"\n"},
    {"M\\v3","\"M\v3\"\n"},{"M\\f3","\"M\f3\"\n"},
    {"M NBSP 3","\"M\xC2\xA0" "3\"\n"},
    {"M3 in tab","\"M\t\t3\"\n"},
    {"X1M3","\"X1M3\"\n"},{"M3S","\"M03S\"\n"},
    {"M-3?","\"M-3\"\n"},{"M+03.","\"M+03.\"\n"},
    {"M3.0000","\"M3.0000\"\n"},{"M4","\"M4\"\n"},{"m4","\"m4\"\n"},
    {"close early )M3(","\"(x) M3 (y)\"\n"},
    {"unbalanced ) M3","\"x) M3\"\n"},
    {"M 3 after N","\"N5M3\"\n"},
    {"O call (LinuxCNC sub)","\"o<spin> call\"\n"},
    {"M98 P100 (Mach sub)","\"M98 P100\"\n"},
    {"G97 S","\"G97\"\n"},
    {"M0 message M3 (marlin style)","\"M0 Drag tool spindle off then resume\"\n"},
    {"M0 msg with digits","\"M0 tool 3 M3\"\n"},
    {"harmless comment (spindle M3 off)","\"(Spindle M3 is off)\"\n"},
    {"harmless ; comment","\"; spindle M3 off\"\n"},
    {"E word (unrelated)","\"E3\"\n"},
  };
  for (auto& c : lits) {
    run(std::string("TC ")+c.l, mkpost(H, T+c.line, N, P, U), second);
    run(std::string("PAUSE ")+c.l, mkpost(H, T, N, P+c.line, U), first);
    run(std::string("UNLOCK ")+c.l, mkpost(H, T, N, P, U+c.line), back);
    run(std::string("FEED ")+c.l, mkpost(H, T, N, P, U, "begin PLUNGE_MOVE\n\"G1[Z][F]\"\n"+c.line+"end\n"), first);
  }
  // VAR prefixes
  run("VAR T prefix M", mkpost(H, "\"M5\"\n\"M6 [T]\"\n", N, P, U, "", "VAR TOOL_NUMBER = [T|A|M|1.0]\n"), second);
  run("VAR N prefix M (NEW_SEGMENT [N] start 3)", mkpost(H, T, "\"[N]\"\n", P, U, "", "VAR LINE_NUMBER = [N|A|M|1.0]\n"), same);
  run("[N] glued X1 + NM", mkpost(H, T, "\"G0 X1[N]\"\n", P, U, "", "VAR LINE_NUMBER = [N|A|M|1.0]\n"), same);
  run("VAR X prefix (", mkpost(H, T, N, P, U, "begin PLUNGE_MOVE\n\"G1[Z][F] M3 [X]\"\nend\n", "VAR XP = [X|A|(|1.3]\n"), first);
  run("SAFEZ in comment then M3", mkpost(H, T, N, P+"\"([T]) M3\"\n", U), first);
  // pause blocks content
  run("pause M5 only in comment", mkpost(H,T,N,"\"(M5)\"\n\"M0\"\n",U), first);
  run("pause M0 in ; comment", mkpost(H,T,N,"\"M5\"\n\"; M0\"\n",U), first);
  run("pause M00 / M05", mkpost(H,T,N,"\"M05\"\n\"M00\"\n",U), first);
  run("pause M0 as [T] prefix", mkpost(H,T,N,"\"M5\"\n\"[T]\"\n",U,"", "VAR TOOL_NUMBER = [T|A|M0|1.0]\n"), first);
  run("pause M5 via X0M5?", mkpost(H,T,N,"\"X1M5\"\n\"G4M0\"\n",U), first);
  run("pause M1 (optional stop) not M0", mkpost(H,T,N,"\"M5\"\n\"M1\"\n",U), first);
  run("pause M0 behind /", mkpost(H,T,N,"\"M5\"\n\"/M0\"\n",U), first);
  run("pause M0 behind / unlock", mkpost(H,T,N,P,"\"/M0\"\n"), back);
  run("pause M0 nested comment", mkpost(H,T,N,"\"M5\"\n\"(a(b)M0)\"\n",U), first);
  run("unlock without M0", mkpost(H,T,N,P,"\"M5\"\n\"(unlock)\"\n"), back);
  run("unlock without M5 ok?", mkpost(H,T,N,P,"\"(unlock)\"\n\"M0\"\n"), back);
  run("no unlock, ends drag", mkpost(H,T,N,P,""), first);
  run("no unlock, drag->spin", mkpost(H,T,N,P,""), back);
  run("no unlock, spin->drag->drag no end? (ends drag)", mkpost(H,T,N,P,""), second);
  run("no unlock, spin only", mkpost(H,T,N,"",""), std::vector<Toolpath>{cut("A",spin(1))});
  run("unlock M3 inside", mkpost(H,T,N,P,U+"\"M3 S100\"\n"), back);
  run("header M3[S] own line, first drag", mkpost("\"G21\"\n\"M3[S]\"\n",T,N,P,U), first);
  run("header M3 S[S] M8", mkpost("\"M3 S[S] M8\"\n",T,N,P,U), first);
  run("header [N]M3[S]", mkpost("\"[N]M3[S]\"\n",T,N,P,U), first);
  run("header M3[S](cmt)M8? ", mkpost("\"M3[S] (x) M8\"\n",T,N,P,U), first);
  run("header S[S]M3 (S before)", mkpost("\"S[S]M3\"\n",T,N,P,U), first);
  run("header M3[S]M3", mkpost("\"M3[S]M4\"\n",T,N,P,U), first);
  run("header M3[S] M3 + ;M8", mkpost("\"M3[S] ; M8\"\n",T,N,P,U), first);
  run("header M3[S]M3.5", mkpost("\"M3[S] M+6\"\n",T,N,P,U), first);
  // type guard
  ToolSnapshot fake = dragT(); fake.toolType = "end_mill";
  run("type guard end_mill", mkpost(H,T,N,P,U), std::vector<Toolpath>{cut("F", fake)});
  fake.toolType = ""; run("type guard empty", mkpost(H,T,N,P,U), std::vector<Toolpath>{cut("F", fake)});
  fake.toolType = "Diamond_Drag"; run("type guard case", mkpost(H,T,N,P,U), std::vector<Toolpath>{cut("F", fake)});
  ToolSnapshot nooff = dragT(); nooff.spindleOff=false; run("drag type but spinning speed 0", mkpost(H,T,N,P,U), std::vector<Toolpath>{cut("F", nooff)});
  ToolSnapshot off2 = dragT(); off2.spindleSpeed=12000; run("drag off with speed 12000", mkpost(H,T,N,P,U), std::vector<Toolpath>{cut("F", off2)});
}
