#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "carvetto/post/post_processor.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "post_fixture.hpp"
using namespace carvetto;
using toolpath::Move; using toolpath::Toolpath; using toolpath::ToolSnapshot;

static ToolSnapshot spin(int n, const char* name="Mill") { ToolSnapshot t; t.name=name; t.toolNumber=n; t.diameter=3; t.feedRate=600; t.plungeRate=200; t.spindleSpeed=12000; return t; }
static ToolSnapshot dragT(int n=3, std::string name="Drag") { ToolSnapshot t=spin(n); t.name=name; t.spindleSpeed=0; t.spindleOff=true; return t; }
static Toolpath cut(std::string nm, ToolSnapshot t){ Toolpath tp; tp.name=nm; tp.tool=t; tp.moves={Move::rapid({1,1,5}),Move::plunge({1,1,-0.1}),Move::feed({2,1,-0.1}),Move::retract({2,1,5})}; return tp; }

static const std::string kMoves="begin RAPID_MOVE\n\"G0[X][Y][Z]\"\nend\nbegin FEED_MOVE\n\"G1[X][Y][Z][F]\"\nend\n";
static std::string mkpost(std::string header, std::string tc, std::string ns, std::string pause, std::string extra="") {
  std::string s = "VAR SPINDLE_SPEED = [S|A| S|1.0]\nVAR X_POSITION = [X|C| X|1.3]\nVAR Y_POSITION = [Y|C| Y|1.3]\nVAR Z_POSITION = [Z|C| Z|1.3]\nVAR FEED_RATE = [F|C| F|1.0]\n";
  s += "begin HEADER\n"+header+"end\nbegin TOOLCHANGE\n"+tc+"end\nbegin SPINDLE_ON\n\"M3[S]\"\nend\nbegin NEW_SEGMENT\n"+ns+"end\n";
  if (!pause.empty()) s += "begin SPINDLE_OFF_PAUSE\n"+pause+"end\n";
  s += "begin FOOTER\n\"M5\"\n\"M30\"\nend\n" + extra + kMoves;
  return s;
}
static void run(const char* label, const std::string& pptext, const std::vector<Toolpath>& tps) {
  auto p = post::PostProcessor::parse(pptext);
  if (!p.ok()) { printf("[%s] PARSE ERROR %s\n", label, p.error().describe().c_str()); return; }
  auto r = p.value().run(tps, test::fixtureContext());
  if (!r.ok()) { printf("[%s] REFUSED: %s\n", label, r.error().describe().c_str()); return; }
  printf("[%s] ACCEPTED:\n%s----\n", label, r.value().text.c_str());
}
int main() {
  const std::string P="\"M5\"\n\"(pause)\"\n\"M0\"\n";
  const std::string H="\"(h)\"\n", T="\"M5\"\n\"M6 T[T]\"\n", N="\"(seg)\"\n";
  std::vector<Toolpath> first{cut("D", dragT())};
  std::vector<Toolpath> second{cut("A", spin(1)), cut("D", dragT())};
  std::vector<Toolpath> same{cut("D1", dragT()), cut("D2", dragT())};
  struct C{const char* l; std::string line;};
  std::vector<C> lits{{"M 3","\"M 3\"\n"},{"M 03","\"M 03\"\n"},{"S 1000","\"S 1000\"\n"},{"M\\t3","\"M\t3\"\n"},{"M03","\"M03\"\n"},{"m3","\"m3\"\n"},{"M3.0","\"M3.0\"\n"},{"M004","\"M004\"\n"},{"s1000","\"s1000\"\n"},{"S.5","\"S.5\"\n"},{"M[T]","\"M[T]\"\n"},{"G0M3","\"G0M3\"\n"},{"comment (M3)","\"(M3)\"\n"},{"semicolon ;M3","\"; M3 S1000\"\n"},{"M+3","\"M+3\"\n"},{"M3 in /block-delete","\"/M3\"\n"},{"N10M3","\"N10M3\"\n"},{"XM3 letter-prefixed","\"G0 X1M3\"\n"}};
  for (auto& c : lits) {
    std::string lab = std::string("TC literal ")+c.l;
    run(lab.c_str(), mkpost(H, T+c.line, N, P), second);
    lab = std::string("PAUSE literal ")+c.l;
    run(lab.c_str(), mkpost(H, T, N, P+c.line), first);
  }
  run("HEADER G21 M3 S[S] mixed", mkpost("\"G21 M3 S[S]\"\n", T, N, P), first);
  run("HEADER M3[S] ;cmt dropped", mkpost("\"M3[S] (spin)\"\n", T, N, P), first);
  run("HEADER S[S] separate + M3 literal", mkpost("\"S[S]\"\n\"M3\"\n", T, N, P), first);
  run("NEW_SEGMENT [N] M3 [S]", mkpost(H,T,"\"[N] M3 [S]\"\n",P), same);
  run("PAUSE with [S]", mkpost(H,T,N,P+"\"S[S]\"\n"), first);
  run("FEED with [S]", mkpost(H,T,N,P,"begin PLUNGE_MOVE\n\"G1[Z][F][S]\"\nend\n"), first);
  run("empty PAUSE block", mkpost(H,T,N,""+std::string("\"\"\n")), first);
  run("no PAUSE block", mkpost(H,T,N,""), first);
  run("HEADER M3 S[S] M8 (coolant)", mkpost("\"M3 S[S] M8\"\n",T,N,P), first);
  run("TOOLCHANGE T[T] M6 M3 S[S]", mkpost(H,"\"T[T] M6 M3 S[S]\"\n",N,P), second);
  run("SPINDLE_ON present spinning->drag same tool", mkpost(H,T,N,P), std::vector<Toolpath>{cut("A", spin(3,"Drag")), cut("D", dragT())});
  // hostile names on uncommented tag
  run("hostile name raw", mkpost(H,T,N,P+"\"[TOOLNAME]\"\n"), std::vector<Toolpath>{cut("D", dragT(3,"M3 S1000"))});
  run("hostile name in comment", mkpost(H,T,N,P+"\"([TOOLNAME])\"\n"), std::vector<Toolpath>{cut("D", dragT(3,"x) M3 S1000 (y\nM3"))});
  run("hostile name in ; comment", mkpost(H,T,N,P+"\"; [TOOLNAME]\"\n"), std::vector<Toolpath>{cut("D", dragT(3,"x\nM3 S1000\r\nM4"))});
}
