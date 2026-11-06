// T-0175: tests for Pocket's move checks (ADR-0017 §8a, T-0169) that the T-0169 verifier's mutants
// survived, the segment distance behind its wall index, and travel at the surface.

#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <utility>
#include <variant>
#include <vector>

#include "../geometry/test_rng.hpp"
#include "carvetto/geometry/boolean.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/fill_rule.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "pocket_curve_oracle.hpp"
#include "pocket_distance.hpp"
#include "pocket_test_util.hpp"

using namespace carvetto::toolpath;
using carvetto::geometry::ArcDirection;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::BezierSpan;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::Vec2;
using carvetto::test::SplitMix;
namespace oc = carvetto::test::pocket_curve;

namespace {

constexpr double kSlack = 1e-5;

Contour polygon(const std::vector<Vec2>& pts) {
    Contour c;
    c.closed = true;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        c.append(LineSpan{pts[i], pts[(i + 1) % pts.size()]});
    }
    return c;
}

/// The verifier's line-only saw (T-0169 sweep, family 9, case 1021): a 5 mm deep bar under 39
/// teeth 3.76 mm high.
Contour saw() {
    const double length = 33.043726887317057;
    const int teeth = 39;
    const double pitch = length / teeth;
    const double height = 3.7585184857295535;
    std::vector<Vec2> v{{0, -5}, {length, -5}};
    for (int i = teeth; i > 0; --i) {
        v.push_back({i * pitch, 0.0});
        v.push_back({(i - 0.5) * pitch, height});
    }
    v.push_back({0, 0});
    return polygon(v);
}

PocketParams sawParams(double cutDepth) {
    PocketParams p;
    p.tolerance.chordal = 0.002;
    p.tolerance.distance = 0.002;
    p.strategy = PocketStrategy::Raster;
    p.direction = CutDirection::Conventional;
    p.rasterAngle = 2.934086596282798;
    p.profilePass = ProfilePass::First;
    p.allowance = 0.011356341148485207;
    p.cutDepth = cutDepth;
    p.safeZ = 5.0;
    return p;
}


/// A case of the T-0169 verifier's sweep families (harness t0169v1 pk.cpp), as generated.
struct SweepCase {
    std::vector<Contour> walls;
    std::vector<PocketTool> tools;
    PocketParams params;
};

/// A saw whose teeth are lines, arcs and Beziers; chordal = distance = 0.05, raster, three levels.
SweepCase sawWithCurvedTeeth() {
    // pk family 8 seed 2248 (saw)
    std::vector<Contour> walls;
    {
        Contour c;
        c.closed = true;
        c.append(LineSpan{{0, -5}, {20.659887923993161, -5}});
        c.append(LineSpan{{20.659887923993161, -5}, {20.659887923993161, 0}});
        c.append(LineSpan{{20.659887923993161, 0}, {19.720802109266199, 2.3769361693953925}});
        c.append(ArcSpan{{20.217580785859759, 0.80669136572531919}, 1.6469540968013996, 1.8772031842102281, -2.6297230894459416, ArcDirection::CCW});
        c.append(BezierSpan{{18.781716294539237, 0}, {18.569192067489993, 0.83201957439568242}, {18.256163462581004, 1.6243316308608133}, {17.842630479812275, 2.3769361693953925}});
        c.append(BezierSpan{{17.842630479812275, 2.3769361693953925}, {18.222611769499046, 1.3108280514879214}, {17.909583164590057, 0.51851599502279044}, {16.903544665085313, 0}});
        c.append(ArcSpan{{18.390711093520725, 1.9615296437129537}, 2.4615569319091546, -2.2195036586283519, 2.9720235638640662, ArcDirection::CW});
        c.append(LineSpan{{15.964458850358351, 2.3769361693953925}, {15.025373035631389, 0}});
        c.append(ArcSpan{{-205.19071350277969, -85.629539321738889}, 236.27852795958034, 0.37085164685994132, 0.38166825837577323, ArcDirection::CCW});
        c.append(ArcSpan{{10.142213962643043, 2.5611941205829911}, 3.9483749643503252, -0.046683736377491264, -0.70583616885822331, ArcDirection::CW});
        c.append(LineSpan{{13.147201406177466, 0}, {12.208115591450504, 2.3769361693953925}});
        c.append(BezierSpan{{12.208115591450504, 2.3769361693953925}, {11.901417419330699, 1.5821230699027096}, {11.588388814421712, 0.78981101343757854}, {11.269029776723542, 0}});
        c.append(LineSpan{{11.269029776723542, 0}, {10.32994396199658, 2.3769361693953925}});
        c.append(ArcSpan{{17.581082973613867, -1.8618380040762461}, 8.3991799277958261, 2.6125984906350945, 2.9180669113087778, ArcDirection::CCW});
        c.append(LineSpan{{9.3908581472696184, 0}, {8.4517723325426566, 2.3769361693953925}});
        c.append(LineSpan{{8.4517723325426566, 2.3769361693953925}, {7.5126865178156947, 0}});
        c.append(ArcSpan{{-0.49488017954648456, -1.789672979167201}, 8.2051236300902293, 0.21988410087402777, 0.53263580436168678, ArcDirection::CCW});
        c.append(BezierSpan{{6.5736007030887329, 2.3769361693953925}, {6.2506529235011667, 1.5885430049705229}, {5.9376243185921789, 0.796230948505392}, {5.6345148883617711, 0}});
        c.append(ArcSpan{{-7.9058099466060074, -3.9755687344840931}, 14.111893685790461, 0.28558373754291444, 0.46693616769280005, ArcDirection::CCW});
        c.append(BezierSpan{{4.6954290736348092, 2.3769361693953925}, {4.9442285253060207, 1.3626556961745502}, {4.6311999203970329, 0.57034363970941926}, {3.7563432589078474, 0}});
        c.append(LineSpan{{3.7563432589078474, 0}, {2.8172574441808855, 2.3769361693953925}});
        c.append(BezierSpan{{2.8172574441808855, 2.3769361693953925}, {3.2094120111258313, 1.3060186030310874}, {2.8963834062168443, 0.51370654656595649}, {1.8781716294539237, 0}});
        c.append(ArcSpan{{2.8368593923328227, 1.7527369867623914}, 1.9977910730250688, -2.0713072856976309, 2.8238271909333452, ArcDirection::CW});
        c.append(LineSpan{{0.93908581472696184, 2.3769361693953925}, {0, 0}});
        c.append(LineSpan{{0, 0}, {0, -5}});
        walls.push_back(c);
    }
    std::vector<PocketTool> tools;
    tools.push_back(carvetto::test::pocket::endMill("T1", 1, 2.0592767120970099, 0.99766773373666384, 2));
    PocketParams p;
    p.tolerance.chordal = 0.050000000000000003;
    p.tolerance.distance = 0.050000000000000003;
    p.strategy = PocketStrategy::Raster;
    p.direction = CutDirection::Conventional;
    p.rasterAngle = 2.8565695138125125;
    p.profilePass = static_cast<ProfilePass>(2);
    p.allowance = 0.0013197939396743707;
    p.cutDepth = 3;
    p.safeZ = 5;
    p.rampPlunge = false;
    p.rampDistance = 6.2363166082860797;
    return {walls, tools, p};
}

/// A letter with filleted corners; chordal = distance = 0.05.
SweepCase curvedLetter() {
    // pk family 1 seed 2027 (letter1)
    std::vector<Contour> walls;
    {
        Contour c;
        c.closed = true;
        c.append(LineSpan{{0, 0}, {3.4979982354142312, -4.0560337902109858}});
        c.append(ArcSpan{{3.7429180345226425, -3.8448104450882457}, 0.32342079327115714, -2.4299330934528443, -0.8591367666579488, ArcDirection::CCW});
        c.append(LineSpan{{3.9541413796453821, -4.089730244196657}, {4.8951512742496748, -3.2781859817259238}});
        c.append(ArcSpan{{4.8051524076820566, -3.1738295986428078}, 0.13780439279517176, -0.85913676665795091, 0.71165956013694787, ArcDirection::CCW});
        c.append(LineSpan{{4.9095087907651722, -3.0838307320751897}, {2.6914238736777838, -0.5118947939204348}});
        c.append(ArcSpan{{2.9182752252305062, -0.31625401071298548}, 0.29956109853470797, -2.4299330934528443, 2.282455886931845, ArcDirection::CW});
        c.append(LineSpan{{2.722634442023057, -0.089402659160263048}, {7.656927397653611, 4.1660224542766535}});
        c.append(ArcSpan{{12.483306731395253, 9.2446200016245594}, 7.0061466242933124, -2.3307362054635825, -2.5291299814421051, ArcDirection::CW});
        c.append(ArcSpan{{6.4036984230047178, 4.917692964907455}, 0.45813451733780269, 0.71165956013694875, 2.2824558869318454, ArcDirection::CCW});
        c.append(BezierSpan{{6.1044947005429053, 5.2646286483436446}, {3.4699374470572235, 3.1529298572394704}, {1.4351058802095884, 1.3980536411249223}, {0, 0}});
        walls.push_back(c);
    }
    std::vector<PocketTool> tools;
    tools.push_back(carvetto::test::pocket::endMill("T1", 1, 0.94170416665719925, 0.40615990770792032, 2));
    tools.push_back(carvetto::test::pocket::endMill("T2", 2, 0.58038178909977189, 0.39443673525728312, 2));
    PocketParams p;
    p.tolerance.chordal = 0.050000000000000003;
    p.tolerance.distance = 0.016666666666666666;
    p.strategy = PocketStrategy::Raster;
    p.direction = CutDirection::Conventional;
    p.rasterAngle = 0.03765283851070933;
    p.profilePass = static_cast<ProfilePass>(0);
    p.allowance = 0.16027088592759722;
    p.cutDepth = 3;
    p.safeZ = 5;
    p.rampPlunge = false;
    p.rampDistance = 5.2571389400112416;
    return {walls, tools, p};
}

/// A gear with arc teeth at chordal 1e-4.
SweepCase fineGear() {
    // pk family 7 seed 2056 (gear)
    std::vector<Contour> walls;
    {
        Contour c;
        c.closed = true;
        c.append(LineSpan{{11.856302150312242, 0}, {9.9548624968309412, 2.923011362628142}});
        c.append(BezierSpan{{9.9548624968309412, 2.923011362628142}, {9.6802506408672837, 4.0868962199814138}, {9.6866818320116703, 5.2492260612871569}, {9.9741560702641046, 6.4100008865453697}});
        c.append(ArcSpan{{7.9730346872036586, 6.2118063444651979}, 2.0109121975491915, 0.098719789657073356, 2.1971407892165149, ArcDirection::CCW});
        c.append(BezierSpan{{6.7942639917843843, 7.8409986332613606}, {5.8416038249897282, 8.6129935447647341}, {5.2186111317036854, 9.5942845945158108}, {4.9252859119262542, 10.784871782514591}});
        c.append(BezierSpan{{4.9252859119262542, 10.784871782514591}, {3.7334638374557509, 10.895751893052697}, {2.5838800919208831, 10.723969378775509}, {1.4765346753216504, 10.269524239683022}});
        c.append(LineSpan{{1.4765346753216504, 10.269524239683022}, {-1.6873277230408881, 11.735622089798067}});
        c.append(BezierSpan{{-1.6873277230408881, 11.735622089798067}, {-3.4666322099796276, 12.002518657336083}, {-4.340850958252739, 11.236494114402134}, {-4.309983967860223, 9.4375484609962257}});
        c.append(BezierSpan{{-4.309983967860223, 9.4375484609962257}, {-5.2892254422382123, 8.0320913728822276}, {-6.4406396956283158, 7.8730403203881369}, {-7.7642267280305353, 8.9603953035139536}});
        c.append(LineSpan{{-7.7642267280305353, 8.9603953035139536}, {-8.7281131541379811, 5.6092177284740821}});
        c.append(ArcSpan{{-10.756309427276582, 5.2966354237788549}, 2.0521422513027434, 0.15291524014941169, -1.8775771547339464, ArcDirection::CW});
        c.append(LineSpan{{-11.376038606275053, 3.3403063194920084}, {-10.375128083877538, 5.8780691727288141e-15}});
        c.append(BezierSpan{{-10.375128083877538, 5.8780691727288141e-15}, {-9.5525846382511741, -1.4598806169448055}, {-9.8862214790503451, -2.573316056775476}, {-11.376038606275054, -3.3403063194920053}});
        c.append(LineSpan{{-11.376038606275054, -3.3403063194920053}, {-8.7281131541379828, -5.6092177284740794}});
        c.append(BezierSpan{{-8.7281131541379828, -5.6092177284740794}, {-7.7551815794323291, -6.5388493139327855}, {-7.4338861040631796, -7.6559085056127429}, {-7.7642267280305379, -8.9603953035139519}});
        c.append(LineSpan{{-7.7642267280305379, -8.9603953035139519}, {-4.3099839678602319, -9.4375484609962221}});
        c.append(BezierSpan{{-4.3099839678602319, -9.4375484609962221}, {-2.5370843167728201, -9.1779613558409636}, {-1.6628655684997065, -9.9439858987749119}, {-1.6873277230408905, -11.735622089798067}});
        c.append(ArcSpan{{-1.5094363630753898, -7.9726329375250291}, 3.7671916192403776, -1.6180351073011505, -0.65569362813768717, ArcDirection::CCW});
        c.append(BezierSpan{{1.4765346753216477, -10.269524239683024}, {2.6782649663192708, -10.09233746973465}, {3.8278487118541382, -10.26411998401184}, {4.9252859119262498, -10.784871782514594}});
        c.append(BezierSpan{{4.9252859119262498, -10.784871782514594}, {6.6697031680427639, -10.515540052979576}, {7.2926958613288067, -9.5342490032284992}, {6.7942639917843781, -7.840998633261365}});
        c.append(ArcSpan{{9.4340374018019517, -9.4583739714584887}, 3.0958531361878365, 2.5918910241754314, 1.3954337041305687, ArcDirection::CW});
        c.append(LineSpan{{9.9741560702641046, -6.4100008865453679}, {9.9548624968309412, -2.9230113626281429}});
        c.append(ArcSpan{{16.184519982725924, -4.8954921247833836}, 6.5344711146840266, 2.8349519482141634, 2.2947701086699412, ArcDirection::CW});
        walls.push_back(c);
    }
    std::vector<PocketTool> tools;
    tools.push_back(carvetto::test::pocket::endMill("T1", 1, 2.7064462899905859, 2.1267389793133624, 2));
    tools.push_back(carvetto::test::pocket::endMill("T2", 2, 1.8341037093228654, 0.93829821892320298, 2));
    PocketParams p;
    p.tolerance.chordal = 0.0001;
    p.tolerance.distance = 3.3333333333333335e-05;
    p.strategy = PocketStrategy::Raster;
    p.direction = CutDirection::Conventional;
    p.rasterAngle = 1.4487225881680681;
    p.profilePass = static_cast<ProfilePass>(1);
    p.allowance = 0.0065737886688426684;
    p.cutDepth = 3;
    p.safeZ = 5;
    p.rampPlunge = true;
    p.rampDistance = 3.8862226751575446;
    return {walls, tools, p};
}

/// A polygon of lines, arcs and a Bezier; raster without a boundary pass, chordal = distance = 0.05.
SweepCase curvedPolygon() {
    // pk family 0 seed 2078 (poly)
    std::vector<Contour> walls;
    {
        Contour c;
        c.closed = true;
        c.append(LineSpan{{-9.4478664504384859, 3.2768612321364565}, {-3.2768612321364614, -9.4478664504384859}});
        c.append(BezierSpan{{-3.2768612321364614, -9.4478664504384859}, {-0.12719167091821326, -6.6979914563570526}, {4.1143842232734347, -4.6409897169230447}, {9.4478664504384859, -3.2768612321364623}});
        c.append(ArcSpan{{351.00199413276636, 170.22251410486791}, 383.09431655816383, -2.6715891011219659, -2.7085067429352989, ArcDirection::CW});
        c.append(ArcSpan{{15.499072490456854, -31.959376114061723}, 43.173396725742705, 1.8578168472231484, 2.1868652694889676, ArcDirection::CCW});
        walls.push_back(c);
    }
    std::vector<PocketTool> tools;
    tools.push_back(carvetto::test::pocket::endMill("T1", 1, 2.7348334059700612, 2.3275154285711523, 2));
    PocketParams p;
    p.tolerance.chordal = 0.050000000000000003;
    p.tolerance.distance = 0.050000000000000003;
    p.strategy = PocketStrategy::Raster;
    p.direction = CutDirection::Conventional;
    p.rasterAngle = 0.59660499899813402;
    p.profilePass = static_cast<ProfilePass>(0);
    p.allowance = 0;
    p.cutDepth = 1;
    p.safeZ = 5;
    p.rampPlunge = false;
    p.rampDistance = 2.2306275526580164;
    return {walls, tools, p};
}

/// The smallest distance from a tool centre (below the surface) to the polygonised walls (the
/// pocket's own region: the even-odd union of the closed input at chordal / 2) less radius +
/// allowance + the polygon tolerance (chordal / 2; nothing for line-only input), over the
/// toolpath of tool `ti`. Every move is either inside the centre region within rounding, which
/// lies radius + allowance + chordal from the walls less the join chords' chordal / 2, or was
/// accepted by the walls check, which asks for that margin (ADR-0017 §8a): about 0 or more.
double polygonWallMargin(const SweepCase& c, const PocketResult& r, std::size_t ti) {
    std::vector<Contour> closed;
    bool curved = false;
    for (const Contour& w : c.walls) {
        if (w.closed) {
            closed.push_back(w);
            for (const carvetto::geometry::Span& sp : w.spans) {
                curved = curved || !std::holds_alternative<LineSpan>(sp);
            }
        }
    }
    carvetto::geometry::Tolerance tol = c.params.tolerance;
    tol.chordal *= 0.5;
    std::vector<std::pair<Vec2, Vec2>> edges;
    for (const Contour& w : carvetto::geometry::unionContours(closed, tol, carvetto::geometry::FillRule::EvenOdd)) {
        for (const carvetto::geometry::Span& sp : w.spans) {
            edges.emplace_back(carvetto::geometry::startPoint(sp), carvetto::geometry::endPoint(sp));
        }
    }
    const double need = 0.5 * c.tools[ti].tool.diameter + c.params.allowance + (curved ? tol.chordal : 0.0);
    const Toolpath& tp = r.toolpaths[ti];
    double worst = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tp.moves.size(); ++i) {
        const Move& m = tp.moves[i];
        const carvetto::geometry::Vec3 s3 = tp.startOf(i);
        if (m.kind == MoveKind::Rapid || std::min(s3.z, m.to.z) >= -c.params.startDepth - 1e-9) {
            continue;
        }
        const Vec2 a = s3.xy();
        const Vec2 b = m.to.xy();
        const double w = need + 1e-3;
        for (const auto& [e0, e1] : edges) {
            if (std::max(e0.x, e1.x) < std::min(a.x, b.x) - w || std::min(e0.x, e1.x) > std::max(a.x, b.x) + w ||
                std::max(e0.y, e1.y) < std::min(a.y, b.y) - w || std::min(e0.y, e1.y) > std::max(a.y, b.y) + w) {
                continue;
            }
            worst = std::min(worst, oc::segmentSegment(a, b, e0, e1) - need);
        }
    }
    return worst;
}

std::size_t countOf(const Toolpath& tp, MoveKind kind) {
    return static_cast<std::size_t>(
        std::count_if(tp.moves.begin(), tp.moves.end(), [kind](const Move& m) { return m.kind == kind; }));
}
}  // namespace

TEST_CASE("T-0175: segment distance of nearly collinear segments that do not touch",
          "[toolpath][pocket][t0175]") {
    using carvetto::toolpath::detail::segmentDistance;
    // The verifier's case (repro/gridmiss.cpp): a segment on an edge's line extension, 0.0099 mm
    // beyond its end. The old test took the rounded orientations (any sign, nearly collinear)
    // for a crossing and returned 0.
    const Vec2 a{0.44956935713374679, 0.0070003571337468214};
    const Vec2 b{0.47199624902709314, 0.029427249027093174};
    const Vec2 c{-0.0, -0.44256899999999999};
    const Vec2 d{0.44256899999999999, 0.0};
    CHECK(std::fabs(segmentDistance(a, b, c, d) - 0.0099) < 1e-6);
    CHECK(std::fabs(segmentDistance(c, d, a, b) - 0.0099) < 1e-6);
    // Generated: segments along one line (rounded to doubles), a gap apart; and proper crossings.
    SplitMix rng{0x7175'0c0cULL};
    for (int k = 0; k < 2000; ++k) {
        const Vec2 o{rng.uniform(-1e4, 1e4), rng.uniform(-1e4, 1e4)};
        const Vec2 u = Vec2::fromAngle(rng.uniform(0.0, 6.283));
        const double l1 = rng.uniform(0.01, 10.0);
        const double gap = std::pow(10.0, rng.uniform(-4.0, 0.0));
        const double l2 = rng.uniform(0.01, 10.0);
        const Vec2 p0 = o;
        const Vec2 p1 = o + u * l1;
        const Vec2 q0 = o + u * (l1 + gap);
        const Vec2 q1 = o + u * (l1 + gap + l2);
        INFO("case " << k << " gap " << gap);
        // Rounding the points to doubles moves them by about 1e-12 at 1e4 mm.
        CHECK(std::fabs(segmentDistance(p0, p1, q0, q1) - gap) < 1e-9);
        CHECK(std::fabs(segmentDistance(q1, q0, p1, p0) - gap) < 1e-9);
        // Crossing at a small angle in the middle: 0.
        const Vec2 m = o + u * (0.5 * l1);
        const Vec2 w = Vec2::fromAngle(rng.uniform(0.0, 6.283) + 1e-3);
        const Vec2 x0 = m - w * l2;
        const Vec2 x1 = m + w * l2;
        if (std::fabs(u.cross(w)) > 1e-6) {
            CHECK(segmentDistance(p0, p1, x0, x1) == 0.0);
        }
    }
}

TEST_CASE("T-0175: travel at the surface takes the shortcuts of the distance tolerance",
          "[toolpath][pocket][t0175]") {
    // The verifier's line-only raster (family 9, case 1021): with travel checked like moves in
    // material, the first level's travel kept 435 more waypoints than before T-0169 (+6.7 %),
    // rounding the round joins of the centre region instead of cutting across them by up to
    // 0.26 µm; at the surface the tool touches no material, so the walls need no clearance there.
    const std::vector<Contour> walls{saw()};
    const PocketTool tool = carvetto::test::pocket::endMill("T1", 1, 0.36918584892401762, 0.10615919441330152, 2.0);
    {
        const PocketResult r = pocketToolpath(walls, {tool}, sawParams(1.0));
        REQUIRE(r.toolpaths.size() == 1);
        const Toolpath& tp = r.toolpaths[0];
        REQUIRE_FALSE(tp.isEmpty());
        std::size_t retracts = 0;
        for (const Move& m : tp.moves) {
            retracts += m.kind == MoveKind::Retract ? 1u : 0u;
        }
        CHECK(retracts == 1);
        // Before T-0169: 6,503 moves; T-0169: 6,938.
        CHECK(tp.moves.size() <= 6550);
        const oc::ClearanceReport rep = oc::wallClearance(tp, walls, 0.5 * tool.tool.diameter, 0.011356341148485207 - kSlack, 0.0);
        CHECK(rep.certified);
    }
    {
        // Two levels: the second level's travel is in material and keeps its waypoints.
        const PocketResult r = pocketToolpath(walls, {tool}, sawParams(3.0));
        REQUIRE(r.toolpaths.size() == 1);
        const oc::ClearanceReport rep =
            oc::wallClearance(r.toolpaths[0], walls, 0.5 * tool.tool.diameter, 0.011356341148485207 - kSlack, 0.0);
        INFO("min clearance " << rep.minClearance << " at move " << rep.worstMove);
        CHECK(rep.certified);
        CHECK(rep.minClearance - 0.011356341148485207 >= 0.0);
    }
}

TEST_CASE("T-0175: moves the walls check accepts keep the polygon tolerance from the polygonised walls",
          "[toolpath][pocket][gouge][t0175]") {
    // A move that leaves the centre region by more than rounding is allowed only when it keeps
    // radius + allowance + chordal / 2 from the polygonised walls (ADR-0017 §8a): that half
    // chordal is the polygon's own error on the unsafe side. These cases of the verifier's sweep
    // have such moves; with the margin set to 0 or halved, with route points merged at the
    // distance tolerance below the surface (both: −0.18 · chordal in the saw, −0.23 · chordal in
    // the letter), or with the rounding slack of the inside test ten times larger (−0.25 ·
    // chordal in the gear at chordal 1e-4) they come closer. Here they keep it within 0.05 ·
    // chordal (about 0 in all three).
    struct Named {
        const char* name;
        SweepCase c;
    };
    for (const Named& n : {Named{"saw", sawWithCurvedTeeth()}, Named{"letter", curvedLetter()}, Named{"gear", fineGear()}}) {
        INFO(n.name);
        const PocketResult r = pocketToolpath(n.c.walls, n.c.tools, n.c.params);
        REQUIRE_FALSE(r.toolpaths.empty());
        REQUIRE_FALSE(r.toolpaths[0].isEmpty());
        const double margin = polygonWallMargin(n.c, r, 0);
        INFO("margin " << margin << " chordal " << n.c.params.tolerance.chordal);
        CHECK(margin >= -0.05 * n.c.params.tolerance.chordal);
        const oc::ClearanceReport rep = oc::wallClearance(r.toolpaths[0], n.c.walls, 0.5 * n.c.tools[0].tool.diameter,
                                                          n.c.params.allowance - kSlack, 0.0);
        CHECK(rep.certified);
    }
}

TEST_CASE("T-0175: a move clear of the walls is taken even when it leaves the centre region by more than rounding",
          "[toolpath][pocket][t0175]") {
    // The verifier's sweep case (family 0, seed 2078): raster without a boundary pass, so every
    // way that is not a short link at depth retracts. Two of its links at depth leave the centre
    // region by more than rounding (up to 4.4e-4 mm, chordal 0.05) but keep radius + allowance +
    // chordal / 2 from the polygonised walls with 0.024 mm to spare, so they are taken: one retract
    // in all. With the rounding test alone the tool would retract at both.
    const SweepCase c = curvedPolygon();
    const PocketResult r = pocketToolpath(c.walls, c.tools, c.params);
    REQUIRE(r.toolpaths.size() == 1);
    REQUIRE_FALSE(r.toolpaths[0].isEmpty());
    CHECK(countOf(r.toolpaths[0], MoveKind::Retract) == 1);
    const oc::ClearanceReport rep =
        oc::wallClearance(r.toolpaths[0], c.walls, 0.5 * c.tools[0].tool.diameter, c.params.allowance - kSlack, 0.0);
    CHECK(rep.certified);
    CHECK(polygonWallMargin(c, r, 0) >= -0.05 * c.params.tolerance.chordal);
}

TEST_CASE("T-0175: raster runs only take links the tool may cut at depth", "[toolpath][pocket][t0175]") {
    // The "A" of the T-0169 regression (V-Carve flat area, chordal = distance = 0.001 / 3, raster
    // at 0.66490 rad, one level). The zig-zag runs of the raster pattern are joined by the same
    // move check as the links at depth: with the inside test of the distance tolerance alone, the
    // pattern joined two runs by the link under the crossbar that comes 4.6e-5 mm inside the tool
    // radius; the emitter then refused that link and travelled instead, so the result was still
    // clear of the walls but cut in another order (320 moves instead of 342). Pins the order.
    std::vector<Contour> walls{polygon({{0, 0}, {1.125, 0}, {1.5, 1.5}, {3, 1.5}, {3.375, 0}, {4.5, 0}, {2.8125, 6}, {1.6875, 6}}),
                               polygon({{1.7999999999999998, 2.4375}, {2.25, 4.3125}, {2.7000000000000002, 2.4375}})};
    const double radius = 0.3 * std::tan(carvetto::geometry::kTwoPi / 12.0);
    PocketParams p;
    p.cutDepth = 0.3;
    p.strategy = PocketStrategy::Raster;
    p.rasterAngle = 0.66490102198018897;
    p.profilePass = ProfilePass::Last;
    p.maintainExactStep = true;
    p.lastPassThickness = 0.0;
    p.safeZ = 5.0;
    p.tolerance.chordal = 0.001 / 3.0;
    p.tolerance.distance = 0.001 / 3.0;
    const PocketResult r =
        pocketToolpath(walls, {carvetto::test::pocket::endMill("E", 1, 2.0 * radius, 0.1732050807568877, 1.0)}, p);
    REQUIRE(r.toolpaths.size() == 1);
    CHECK(r.toolpaths[0].moves.size() == 342);
    const oc::ClearanceReport rep = oc::wallClearance(r.toolpaths[0], walls, radius, -kSlack, 0.0);
    CHECK(rep.certified);
}
