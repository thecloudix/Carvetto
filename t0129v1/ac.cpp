// T-0129 verifier throwaway: AC1/AC2 matrix.
#include <array>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>
#include <vector>
#include "carvetto/geometry/contour.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/profile.hpp"
using namespace carvetto; using namespace carvetto::toolpath; using geometry::Contour; using geometry::Vec2;
constexpr double PI = 3.14159265358979323846;
static std::string mm(double v) { std::array<char, 64> b{}; auto r = std::to_chars(b.data(), b.data() + 64, v); return std::string(b.data(), r.ptr) + " mm"; }
static tooling::Tool endMill(double d) { tooling::Tool t; t.id="em"; t.name="EM"; t.type = tooling::ToolType::EndMill; t.diameter = d; t.defaultCuttingData.feedRate = 1000; t.defaultCuttingData.plungeRate = 300; t.defaultCuttingData.passDepth = 1.5; t.defaultCuttingData.spindleSpeed = 18000; t.defaultCuttingData.toolNumber = 1; return t; }
static tooling::Tool vbit(double d, double deg) { auto t = endMill(d); t.type = tooling::ToolType::VBit; t.includedAngle = deg * PI / 180; return t; }
int main() {
    const double inf = std::numeric_limits<double>::infinity(), nan = std::numeric_limits<double>::quiet_NaN();
    Contour open; open.append(geometry::LineSpan{{0, 0}, {30, 0}});     open = Contour{}; open.append(geometry::LineSpan{{0, 0}, {30, 0}}); open.append(geometry::LineSpan{{30, 0}, {30, 25}}); open.append(geometry::LineSpan{{30, 25}, {70, 10}});
    std::vector<std::vector<Contour>> scenes{
        {Contour::rectangle(geometry::Box2{{0, 0}, {40, 30}})},
        {Contour::circle({0, 0}, 15), Contour::rectangle(geometry::Box2{{-5, -5}, {5, 5}}), Contour::circle({50, 0}, 8)},
        {open},
        {open, Contour::circle({100, 0}, 10)}};
    std::vector<std::pair<std::string, tooling::Tool>> tools;
    for (double d : {2.0, 3.175, 6.0, 12.0}) tools.push_back({"EM" + std::to_string(d), endMill(d)});
    for (double a : {30.0, 60.0, 90.0, 120.0, 150.0}) tools.push_back({"V" + std::to_string((int)a), vbit(25.0, a)});
    tools.push_back({"Vsmall90", vbit(3.0, 90.0)});  // cone clipped at the diameter
    long nNoRoom = 0, nOk = 0, nRef = 0, fails = 0, cases = 0;
    auto fail = [&](const char* what, const std::string& ctx, const ProfileResult& r) {
        ++fails; if (std::string(what)=="valid no moves") ++nNoRoom; if (std::string(what) != "valid no moves") { std::printf("FAIL %s | %s | moves %zu diags %zu", what, ctx.c_str(), r.toolpath.moves.size(), r.diagnostics.size()); for (auto& d : r.diagnostics) std::printf(" [%d %s]", (int)d.issue, d.message.c_str()); std::printf("\n"); } };
    for (size_t si = 0; si < scenes.size(); ++si)
    for (auto side : {ProfileSide::Outside, ProfileSide::Inside, ProfileSide::On})
    for (auto& [tn, tool] : tools)
    for (double sd : {0.0, 1.3, 3.0})
    for (double cd : {0.7, 2.0, 5.0})
    for (int sep : {0, 1})
    for (double lpa : {0.0, 0.2, 1.0}) {
        if (!sep && lpa != 0.2) continue;
        ProfileParams p; p.side = side; p.passes.startDepth = sd; p.passes.cutDepth = cd; p.passes.passDepth = 1.0; p.safeZ = 5 + sd;
        p.separateLastPass = sep; p.lastPassAllowance = lpa;
        const double r = profileToolRadius(tool, p.passes);
        char ctxb[256]; std::snprintf(ctxb, sizeof ctxb, "scene %zu side %d %s sd %g cd %g sep %d lpa %g r %.17g", si, (int)side, tn.c_str(), sd, cd, sep, lpa, r);
        std::vector<double> valid{-r, std::nextafter(-r, inf), -r + 1e-12, -r + 1e-9, -r / 2, 0.0, -0.0, 0.5};
        std::vector<double> bad{std::nextafter(-r, -inf), -r - 1e-12, -r - 1e-9, -r - 1, -1e300, -std::numeric_limits<double>::max()};
        for (double a : valid) {
            ++cases; p.allowance = a; auto res = profileToolpath(scenes[si], tool, p);
            std::string ctx = std::string(ctxb) + " a " + mm(a);
            if (res.has(ProfileIssue::InvalidParameters)) fail("valid refused", ctx, res);
            else if (res.toolpath.moves.empty()) fail("valid no moves", ctx, res);
            else ++nOk;
        }
        for (double a : bad) {
            ++cases; p.allowance = a; auto res = profileToolpath(scenes[si], tool, p);
            std::string ctx = std::string(ctxb) + " a " + mm(a);
            bool good = res.toolpath.moves.empty() && res.diagnostics.size() == 1 && res.diagnostics[0].issue == ProfileIssue::InvalidParameters &&
                        !res.diagnostics[0].contour && res.diagnostics[0].message.find("allowance " + mm(a)) != std::string::npos &&
                        res.diagnostics[0].message.find("(" + mm(-r) + " at the cut depth)") != std::string::npos && !res.ok();
            if (!good) fail("bad not refused properly", ctx, res); else ++nRef;
        }
        for (double a : {-inf, nan, inf}) {
            ++cases; p.allowance = a; auto res = profileToolpath(scenes[si], tool, p);
            bool good = res.toolpath.moves.empty() && res.diagnostics.size() == 1 && res.diagnostics[0].issue == ProfileIssue::InvalidParameters;
            if (!good) fail("nonfinite", std::string(ctxb) + " a " + mm(a), res); else ++nRef;
        }
        if (sep) {  // NaN / inf last-pass allowance with otherwise-bad allowance
            for (double l : {nan, inf}) { p.allowance = -r - 1; p.lastPassAllowance = l; auto res = profileToolpath(scenes[si], tool, p);
                if (!(res.toolpath.moves.empty() && res.diagnostics.size() == 1)) fail("nonfinite lpa", ctxb, res); }
            p.lastPassAllowance = 1e300; p.allowance = -r - 1; auto res = profileToolpath(scenes[si], tool, p);
            if (!(res.toolpath.moves.empty() && res.diagnostics.size() == 1 && res.diagnostics[0].message.find("last pass would cross") != std::string::npos)) fail("huge lpa", ctxb, res);
        }
    }
    std::printf("noroom %ld cases %ld ok %ld refused %ld FAILS %ld\n", nNoRoom, cases, nOk, nRef, fails);
    // sample messages
    ProfileParams p; p.passes.cutDepth = 1; p.allowance = -4; auto r1 = profileToolpath(scenes[0], endMill(6), p); std::printf("MSG1 %s\n", r1.diagnostics[0].message.c_str());
    p.separateLastPass = true; p.lastPassAllowance = 0.5; p.allowance = std::nextafter(-3.0, -inf); r1 = profileToolpath(scenes[0], endMill(6), p); std::printf("MSG2 %s\n", r1.diagnostics[0].message.c_str());
    p.allowance = -4; r1 = profileToolpath(scenes[0], endMill(6), p); std::printf("MSG3 %s\n", r1.diagnostics[0].message.c_str());
    p.allowance = -1e300; r1 = profileToolpath(scenes[0], endMill(6), p); std::printf("MSG4 %s\n", r1.diagnostics[0].message.c_str());
    p.allowance = nan; r1 = profileToolpath(scenes[0], endMill(6), p); std::printf("MSG5 %s\n", r1.diagnostics[0].message.c_str());
}
