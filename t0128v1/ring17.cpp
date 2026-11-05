// T-0128 verifier: the three T-0017 "no spurious chain splits" cases, branch vs main: chain counts, medial-ring chains, chains as listed.
#include "v9lib.cpp"
#include "xchk.hpp"
static void report(const char* name, const Contour& in, double d, OffsetSide side, JoinType join, double mitre, const std::vector<Contour>& chains) {
    double inner = 1e300; Vec2 centre{0,0};
    for (const Span& s : in.spans) if (const auto* arc = std::get_if<ArcSpan>(&s); arc && arc->radius < inner) { inner = arc->radius; centre = arc->centre; }
    auto mp = mainOffset(toP(in), d, (int)side, (int)join, 0.001, mitre); std::vector<Contour> om; for (auto& p : mp) om.push_back(fromP(p));
    auto medial = [&](const std::vector<Contour>& ch) { size_t c = 0; for (auto& k : ch) { bool on = std::all_of(k.spans.begin(), k.spans.end(), [&](const Span& s) { auto* a = std::get_if<ArcSpan>(&s); return a && distance(a->centre, centre) <= 1e-6 && std::fabs(a->radius - (inner + d)) <= 1e-6; }); c += on && !k.closed; } return c; };
    std::printf("%s: branch chains %zu medial %zu | main chains %zu medial %zu\n", name, chains.size(), medial(chains), om.size(), medial(om));
    for (int w = 0; w < 2; ++w) { auto& o = w ? om : chains; std::printf(" %s:\n", w ? "MAIN" : "BRANCH"); for (auto& ch : o) { double L = 0; for (auto& s : ch.spans) L += length(s); auto a = startPoint(ch.spans.front()); auto b = endPoint(ch.spans.back()); bool allArcRing = true; for (auto& s : ch.spans) { auto* q = std::get_if<ArcSpan>(&s); allArcRing = allArcRing && q && std::fabs(q->radius - (inner + d)) <= 1e-6; } std::printf("   closed %d spans %zu len %.6g ring %d (%.6f,%.6f)->(%.6f,%.6f)\n", ch.closed, ch.spans.size(), L, allArcRing, a.x, a.y, b.x, b.y); } }
}
int main() {
{    Contour in;
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.3862428321970466,
                                  -4.5621665668092648,
                                  -2.6741011505546641,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-530.65240998014099, -796.55591293862028}, {-548.94963649378906, -805.79261167470577}});
    in.spans.emplace_back(
        LineSpan{{-548.94963649378906, -805.79261167470577}, {-516.13281398203776, -774.65919515999315}});
    in.spans.emplace_back(
        LineSpan{{-516.13281398203776, -774.65919515999315}, {-527.20304046700051, -793.4921014053192}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.3862428321970466,
                                  -5.2981356216840663,
                                  -3.1700591557281022,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-530.90748337668583, -795.41263769137049}, {-551.66878317794499, -794.82147641667063}});
    in.spans.emplace_back(
        LineSpan{{-551.66878317794499, -794.82147641667063}, {-553.25672246990587, -792.42687469612883}});
    in.spans.emplace_back(
        LineSpan{{-553.25672246990587, -792.42687469612883}, {-530.90843047481098, -795.47086696454119}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.3862428321970466,
                                  -3.1456532488636326,
                                  -0.70919585426922938,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-526.71132151439201, -797.03453445467574}, {-511.02037592237946, -810.49942741410098}});
    in.spans.emplace_back(
        LineSpan{{-511.02037592237946, -810.49942741410098}, {-550.89713992024394, -804.94076250819387}});
    in.spans.emplace_back(
        LineSpan{{-550.89713992024394, -804.94076250819387}, {-530.96291329301039, -796.00011019586691}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.4953920950661193,
                                  -2.9318530340915321,
                                  -4.4399499502028501,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-529.19367068242752, -793.077200797209}, {-535.14309552346174, -771.78254073785104}});
    in.spans.emplace_back(
        LineSpan{{-535.14309552346174, -771.78254073785104}, {-533.34364821798954, -820.07641355322892}});
    in.spans.emplace_back(
        LineSpan{{-533.34364821798954, -820.07641355322892}, {-528.99483158744624, -797.93078261741505}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.4953920950661193,
                                  -1.7613461565305366,
                                  -3.3952454119238,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-530.93775248525071, -794.85435905937891}, {-552.10605282837889, -789.36676323520794}});
    in.spans.emplace_back(
        LineSpan{{-552.10605282837889, -789.36676323520794}, {-509.72595117382872, -780.35017544508185}});
    in.spans.emplace_back(
        LineSpan{{-509.72595117382872, -780.35017544508185}, {-526.64332021418943, -793.83837351525801}});
    in.spans.emplace_back(ArcSpan{{-528.52220731529064, -795.48055650428023},
                                  2.4953920950661193,
                                  0.7182741078915007,
                                  0.35533061320593529,
                                  ArcDirection::CW});
    const double d = 0.054574631434536308;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Square, Tolerance{}, 2);
 report("0xe20010e4", in, d, OffsetSide::Right, JoinType::Square, 2, chains); }
{    Contour in;
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.1919727791070662, 2.4357476808179968, 4.5278080532487923, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-0.21876824183394267, -1.1717250370701773}, {-4.39504096909259, -23.539886316515425}});
    in.spans.emplace_back(
        LineSpan{{-4.39504096909259, -23.539886316515425}, {-5.6677075625630859, -22.456921546747697}});
    in.spans.emplace_back(
        LineSpan{{-5.6677075625630859, -22.456921546747697}, {-0.21876824183394267, -1.1717250370701773}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.1919727791070662, 4.5278080532487923, 2.4357476808179972, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-0.90716722429277774, 0.77320549228595103}, {-18.088632626063685, 15.417477307250458}});
    in.spans.emplace_back(
        LineSpan{{-18.088632626063685, 15.417477307250458}, {0.16115639308495969, 23.457553774169011}});
    in.spans.emplace_back(
        LineSpan{{0.16115639308495969, 23.457553774169011}, {0.080254774639365339, 1.3080613106940997}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.3105209732725309, 1.5095191530933008, 0.4869420860726652, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{1.1581963220008005, 0.61322630495682384}, {19.917905323155367, 10.545866233367279}});
    in.spans.emplace_back(
        LineSpan{{19.917905323155367, 10.545866233367279}, {23.399749486168027, -0.96406011694593097}});
    in.spans.emplace_back(
        LineSpan{{23.399749486168027, -0.96406011694593097}, {1.1828970666470202, -0.14681156579131652}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.1919727791070662, -0.1234804317390692, 2.0702834027244332, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-0.57092556993018517, 1.0463475042891421}, {-11.591943268876204, 21.244802384193687}});
    in.spans.emplace_back(
        LineSpan{{-11.591943268876204, 21.244802384193687}, {13.480154468575595, -20.30503941079964}});
    in.spans.emplace_back(
        LineSpan{{13.480154468575595, -20.30503941079964}, {0.47285918223161294, -1.0941678572830922}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.1919727791070662, -1.1628739864816351, 0.89591121102210303, ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{0.74475367723229968, 0.93066700080167797}, {13.498116409923712, 16.867659603078756}});
    in.spans.emplace_back(
        LineSpan{{13.498116409923712, 16.867659603078756}, {-2.0317464307992732, 24.111758353574221}});
    in.spans.emplace_back(
        LineSpan{{-2.0317464307992732, 24.111758353574221}, {-0.20305133823571422, 1.2946950897519722}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.3105209732725309, 1.7263624429144553, 0.15341426555344007, ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{1.2951290280355914, 0.20026487991349423}, {21.280654653116034, 3.2906124844185856}});
    in.spans.emplace_back(
        LineSpan{{21.280654653116034, 3.2906124844185856}, {20.368338190482071, -9.1039641829667719}});
    in.spans.emplace_back(
        LineSpan{{20.368338190482071, -9.1039641829667719}, {1.1020365401844114, -0.70921131229384238}});
    in.spans.emplace_back(
        ArcSpan{{0, 0}, 1.3105209732725309, -0.5718248168727591, -2.8994887782050154, ArcDirection::CW});
    const double d = 0.059274097082732395;
    const std::vector<Contour> chains = offsetOpenContour(in, d, OffsetSide::Right, JoinType::Sharp, Tolerance{}, 2);
 report("0xe20006d8", in, d, OffsetSide::Right, JoinType::Sharp, 2, chains); }
{    Contour in;
    in.spans.emplace_back(ArcSpan{{-86.78814498830485, 715.58515024155804},
                                  1.0908054494621033,
                                  0.48575230134896552,
                                  1.4728805836930272,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-86.681508548943796, 716.67073082280319}, {-84.456158542599809, 739.32524524425139}});
    in.spans.emplace_back(
        LineSpan{{-84.456158542599809, 739.32524524425139}, {-68.781084618225648, 728.29482572264112}});
    in.spans.emplace_back(
        LineSpan{{-68.781084618225648, 728.29482572264112}, {-85.968984963650939, 716.30545113882147}});
    in.spans.emplace_back(ArcSpan{{-86.788144988299209, 715.58515024155474},
                                  1.0908054494661392,
                                  0.72126949998118706,
                                  2.9140318809814372,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-87.850829022667, 715.83123797323765}, {-108.77292355259084, 720.6762065795341}});
    in.spans.emplace_back(
        LineSpan{{-108.77292355259084, 720.6762065795341}, {-78.894600869747634, 736.50093129607183}});
    in.spans.emplace_back(
        LineSpan{{-78.894600869747634, 736.50093129607183}, {-86.316656641626636, 716.73643824866053}});
    in.spans.emplace_back(ArcSpan{{-86.7881449882981, 715.58515024155656},
                                  1.2440921735741632,
                                  1.1821005065108441,
                                  0.8207966410408235,
                                  ArcDirection::CW});
    in.spans.emplace_back(
        LineSpan{{-85.940123827802736, 716.49543890330108}, {-71.845465712074571, 731.62502301229119}});
    in.spans.emplace_back(
        LineSpan{{-71.845465712074571, 731.62502301229119}, {-72.810726151152167, 699.05233169187397}});
    in.spans.emplace_back(
        LineSpan{{-72.810726151152167, 699.05233169187397}, {-86.067854540091147, 714.76598102901153}});
    in.spans.emplace_back(ArcSpan{{-86.788144988302264, 715.5851502415594},
                                  1.0908054494594388,
                                  -0.84953958255408324,
                                  -0.37838253750279938,
                                  ArcDirection::CCW});
    in.spans.emplace_back(
        LineSpan{{-85.774499439794525, 715.18218717804007}, {-65.073000573041512, 706.95254574660839}});
    in.spans.emplace_back(
        LineSpan{{-65.073000573041512, 706.95254574660839}, {-75.968802944125443, 736.08970093582138}});
    in.spans.emplace_back(
        LineSpan{{-75.968802944125443, 736.08970093582138}, {-86.449783729782922, 716.62214984970683}});
    in.spans.emplace_back(ArcSpan{{-86.788144988304609, 715.5851502415544},
                                  1.09080544946236,
                                  1.2553992806336334,
                                  1.7333888049510962,
                                  ArcDirection::CCW});
    const double d = 0.076643362054408748;
    const std::vector<Contour> chains =
        offsetOpenContour(in, d, OffsetSide::Right, JoinType::Square, Tolerance{}, 3.310653624707161);
 report("0xe3001ac2", in, d, OffsetSide::Right, JoinType::Square, 3.310653624707161, chains); }
}