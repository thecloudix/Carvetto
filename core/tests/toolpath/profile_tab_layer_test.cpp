// T-0167: Profile tab judging at the tab layer, and the repros of the T-0158 verifier's rounds.
//
// 1. A V-bit's cut at the tab layer (just below the tab top) is the cone's disc there, tab thickness
//    x tan(half angle), far narrower than the radius at the top of the cut. Sided tabs were judged
//    with that disc already (T-0127); Inside/Outside with allowance >= 0 judged with the radius at
//    the top, so tabs that hold were left out as "cut through" and whole parts came free
//    (m2_1330000_9: both tabs of two vectors, 641 and 1,704 mm2). Now every judgement uses the
//    tab-layer radius (ADR-0016 section 14).
// 2. The unfixed repros are pinned as what they are: the pieces they free are reported (the cut
//    paths are searched, ADR-0016 section 14 "Known gaps"), leads off.
//
// The flood is the test's own: the uncut components at the layer just below the tab top that do not
// reach the frame (every tab present), as in profile_test.cpp.

#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <sstream>
#include <string>
#include <vector>

#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/span.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "profile_test_util.hpp"

using namespace carvetto;
using namespace carvetto::toolpath;
using carvetto::geometry::Box2;
using carvetto::geometry::Contour;
using carvetto::geometry::LineSpan;
using carvetto::geometry::ArcSpan;
using carvetto::geometry::Vec2;

namespace {

double readScene(const std::string& text, std::vector<Contour>& cs, ProfileParams& p, int& vbitDeg) {
    std::istringstream in(text);
    std::string tag;
    auto num = [&]() {
        std::string t;
        in >> t;
        return std::strtod(t.c_str(), nullptr);
    };
    auto integer = [&]() { return static_cast<int>(num()); };
    double d = 0.0;
    while (in >> tag) {
        if (tag == "C") {
            cs.emplace_back();
            cs.back().closed = integer() != 0;
        } else if (tag == "L") {
            const double a = num(), b = num(), c = num(), e = num();
            cs.back().spans.push_back(LineSpan{{a, b}, {c, e}});
        } else if (tag == "A") {
            const double a = num(), b = num(), c = num(), e = num(), f = num();
            cs.back().spans.push_back(ArcSpan{{a, b}, c, e, f, static_cast<geometry::ArcDirection>(integer())});
        } else if (tag == "B") {
            double v[8];
            for (double& x : v) x = num();
            cs.back().spans.push_back(geometry::BezierSpan{{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}});
        } else if (tag == "P") {
            d = num();
            vbitDeg = integer();
            p.allowance = num();
            p.side = static_cast<ProfileSide>(integer());
            p.direction = static_cast<CutDirection>(integer());
            p.passes.startDepth = num();
            p.passes.cutDepth = num();
            p.passes.passDepth = num();
            p.passes.lastPassThickness = num();
            p.safeZ = num();
            p.separateLastPass = integer() != 0;
            p.lastPassAllowance = num();
            p.lastPassReverse = integer() != 0;
            p.tabs.enabled = integer() != 0;
            p.tabs.length = num();
            p.tabs.thickness = num();
            p.tabs.spacing = static_cast<TabSpacing>(integer());
            p.tabs.count = integer();
            p.tabs.distance = num();
            num();
            p.tabs.avoidCorners = integer() != 0;
            p.tabs.threeD = integer() != 0;
            num();
            p.ramp.type = static_cast<RampType>(integer());
            p.ramp.distance = num();
            p.ramp.useAngle = integer() != 0;
            p.ramp.onLeadIn = integer() != 0;
            p.ramp.angle = num();
            p.leads.length = num();
            p.leads.angle = num();
            p.leads.radius = num();
            p.leads.type = static_cast<LeadType>(integer());
            p.leads.sweep = num();
            p.leads.leadOut = integer() != 0;
            integer();
            p.leads.overcut = num();
            num();
            p.startAt = static_cast<StartAt>(integer());
            p.startAnchor = static_cast<BoxAnchor>(integer());
            p.order.selectionOrder = integer() != 0;
            p.order.leftToRight = integer() != 0;
            p.order.bottomToTop = integer() != 0;
            p.order.grid = integer() != 0;
            p.startPosition.x = num();
            p.startPosition.y = num();
            p.corners.mitreLimit = num();
        } else if (tag == "K") {
            p.corners.sharpExternal = integer() != 0;
            p.corners.sharpInternal = integer() != 0;
        }
    }
    return d;
}

std::vector<double> floodPieces(const Toolpath& tp, double zTab, double r, double res,
                                const std::function<double(double)>& radiusAt) {
    Box2 box;
    test::sampleCuttingMoves3(
        tp, 0.05, [&](Vec2 q, double z) { if (z < zTab - 1e-6) box.expand(q); }, zTab - 1e-6);
    if (box.isEmpty()) return {};
    box = box.inflated(r + 1.0);
    const int nx = static_cast<int>(std::ceil(box.width() / res));
    const int ny = static_cast<int>(std::ceil(box.height() / res));
    auto at = [&](int i, int j) { return static_cast<std::size_t>(j) * static_cast<std::size_t>(nx) + static_cast<std::size_t>(i); };
    std::vector<char> cut(static_cast<std::size_t>(nx) * static_cast<std::size_t>(ny), 0);
    test::sampleCuttingMoves3(
        tp, 0.05,
        [&](Vec2 c, double z) {
            if (z >= zTab - 1e-6) return;
            const double rz = radiusAt(z);  // the disc the tool cuts at the layer there
            if (!(rz > 0.0)) return;
            const int j0 = std::max(0, static_cast<int>(std::floor((c.y - rz - box.min.y) / res)));
            const int j1 = std::min(ny - 1, static_cast<int>(std::ceil((c.y + rz - box.min.y) / res)));
            for (int j = j0; j <= j1; ++j) {
                const double dy = box.min.y + (j + 0.5) * res - c.y;
                const double w2 = rz * rz - dy * dy;
                if (w2 <= 0.0) continue;
                const double w = std::sqrt(w2);
                const int i0 = std::max(0, static_cast<int>(std::ceil((c.x - w - box.min.x) / res - 0.5)));
                const int i1 = std::min(nx - 1, static_cast<int>(std::floor((c.x + w - box.min.x) / res - 0.5)));
                for (int i = i0; i <= i1; ++i) cut[at(i, j)] = 1;
            }
        },
        zTab - 1e-6);
    std::vector<double> out;
    std::vector<char> seen(cut.size(), 0);
    for (std::size_t s = 0; s < cut.size(); ++s) {
        if (cut[s] || seen[s]) continue;
        std::vector<std::size_t> st{s};
        seen[s] = 1;
        std::size_t cells = 0;
        bool frame = false;
        while (!st.empty()) {
            const std::size_t k = st.back();
            st.pop_back();
            ++cells;
            const int i = static_cast<int>(k % static_cast<std::size_t>(nx));
            const int j = static_cast<int>(k / static_cast<std::size_t>(nx));
            if (i == 0 || j == 0 || i == nx - 1 || j == ny - 1) frame = true;
            const int di[4] = {1, -1, 0, 0};
            const int dj[4] = {0, 0, 1, -1};
            for (int e = 0; e < 4; ++e) {
                const int ii = i + di[e];
                const int jj = j + dj[e];
                if (ii < 0 || jj < 0 || ii >= nx || jj >= ny) continue;
                const std::size_t kk = at(ii, jj);
                if (seen[kk] || cut[kk]) continue;
                seen[kk] = 1;
                st.push_back(kk);
            }
        }
        if (!frame) out.push_back(static_cast<double>(cells) * res * res);
    }
    std::sort(out.begin(), out.end());
    return out;
}


const char* const k_m2_1330000_9 =
        "C 1\n"
        "L 0x1.7d3493e8122cdp+3 0x0p+0 0x1.6323ea65cc7e2p+1 0x1.50a5eab5dadfcp+0\n"
        "L 0x1.6323ea65cc7e2p+1 0x1.50a5eab5dadfcp+0 0x1.dfa4740ff79bdp+2 0x1.2cb9ecdb745dcp+3\n"
        "L 0x1.dfa4740ff79bdp+2 0x1.2cb9ecdb745dcp+3 0x1.7a8b4e7f78645p-1 0x1.bddb2cfb41ec2p+1\n"
        "L 0x1.7a8b4e7f78645p-1 0x1.bddb2cfb41ec2p+1 -0x1.3b312f41edf2cp+1 0x1.593c9aa6b714p+3\n"
        "L -0x1.3b312f41edf2cp+1 0x1.593c9aa6b714p+3 -0x1.37cf7d0d16487p+1 0x1.6781aa1cda6e9p+1\n"
        "L -0x1.37cf7d0d16487p+1 0x1.6781aa1cda6e9p+1 -0x1.383a15008db86p+3 0x1.2cb898faf0bp+2\n"
        "L -0x1.383a15008db86p+3 0x1.2cb898faf0bp+2 -0x1.1dc1f21ed09e9p+2 0x1.6d89f883fa88cp-3\n"
        "L -0x1.1dc1f21ed09e9p+2 0x1.6d89f883fa88cp-3 -0x1.44d3992d16a4p+3 -0x1.38db404b47b5fp+2\n"
        "L -0x1.44d3992d16a4p+3 -0x1.38db404b47b5fp+2 -0x1.10f1e8c58ab0ep+1 -0x1.768abd18ffa91p+1\n"
        "L -0x1.10f1e8c58ab0ep+1 -0x1.768abd18ffa91p+1 -0x1.635a40e9c4a49p+1 -0x1.8539ae40d648cp+3\n"
        "L -0x1.635a40e9c4a49p+1 -0x1.8539ae40d648cp+3 0x1.ea3de9bf7c8d3p-1 -0x1.e3e3cb91900c6p+1\n"
        "L 0x1.ea3de9bf7c8d3p-1 -0x1.e3e3cb91900c6p+1 0x1.f1f425df1aeadp+2 -0x1.3835044a07b87p+3\n"
        "L 0x1.f1f425df1aeadp+2 -0x1.3835044a07b87p+3 0x1.e6b6ee9fb2d56p+1 -0x1.abb1b59d12d4ep+0\n"
        "L 0x1.e6b6ee9fb2d56p+1 -0x1.abb1b59d12d4ep+0 0x1.7d3493e8122cdp+3 0x0p+0\n"
        "C 1\n"
        "L 0x1.1ca1bbc77c069p+6 -0x1.8069ace4d8af2p+3 0x1.482afb76d2c3ap+6 -0x1.8069ace4d8af2p+3\n"
        "L 0x1.482afb76d2c3ap+6 -0x1.8069ace4d8af2p+3 0x1.482afb76d2c3ap+6 -0x1.6ebddf178f5d8p-1\n"
        "L 0x1.482afb76d2c3ap+6 -0x1.6ebddf178f5d8p-1 0x1.77012f8453f4ep+6 -0x1.6ebddf178f5d8p-1\n"
        "L 0x1.77012f8453f4ep+6 -0x1.6ebddf178f5d8p-1 0x1.77012f8453f4ep+6 -0x1.8069ace4d8af2p+3\n"
        "L 0x1.77012f8453f4ep+6 -0x1.8069ace4d8af2p+3 0x1.a28a6f33aab2p+6 -0x1.8069ace4d8af2p+3\n"
        "L 0x1.a28a6f33aab2p+6 -0x1.8069ace4d8af2p+3 0x1.a28a6f33aab2p+6 0x1.576df73e4e562p+4\n"
        "L 0x1.a28a6f33aab2p+6 0x1.576df73e4e562p+4 0x1.77012f8453f4ep+6 0x1.576df73e4e562p+4\n"
        "L 0x1.77012f8453f4ep+6 0x1.576df73e4e562p+4 0x1.77012f8453f4ep+6 0x1.455e1f893cf31p+3\n"
        "L 0x1.77012f8453f4ep+6 0x1.455e1f893cf31p+3 0x1.482afb76d2c3ap+6 0x1.455e1f893cf31p+3\n"
        "L 0x1.482afb76d2c3ap+6 0x1.455e1f893cf31p+3 0x1.482afb76d2c3ap+6 0x1.576df73e4e562p+4\n"
        "L 0x1.482afb76d2c3ap+6 0x1.576df73e4e562p+4 0x1.1ca1bbc77c069p+6 0x1.576df73e4e562p+4\n"
        "L 0x1.1ca1bbc77c069p+6 0x1.576df73e4e562p+4 0x1.1ca1bbc77c069p+6 -0x1.8069ace4d8af2p+3\n"
        "P 0x1.9p+4 90 0x0p+0 0 1 0x0p+0 0x1.791039359aa1cp+2 0x1.19c7baf01e96dp+2 0x0p+0 0x1.4p+2 1 0x1.c140f35a3e2bp-1 1 1 0x1.09546ef95179p+0 0x1.fde3126235649p+0 0 1 0x1.c018c39902e4cp+4 0x0p+0 0 1 0x0p+0 2 0x1.860453022ee0ap+4 1 0 0x1.c2d16e6c316ffp-2 0x1.47136c3c79c14p+2 0x1.5da614cfffdd9p+0 0x1.5597599f37526p+0 0 0x1.ee0aba961af83p-2 0 0 0x0p+0 0x0p+0 1 8 0 1 0 1 0x1.17bce6a0850dp+5 0x1.bcc457e0bf448p+4 0x1p+1\n"
        "K 1 1\n";

const char* const k_m3_9304_126 =
        "C 1\n"
        "L -0x1.a9f4cfc56db59p+3 -0x1.a9f4cfc56db59p+3 -0x1.7303c6d890e6p+2 -0x1.a9f4cfc56db59p+3\n"
        "L -0x1.7303c6d890e6p+2 -0x1.a9f4cfc56db59p+3 -0x1.7303c6d890e6p+2 -0x1.1843fe8f18b4cp+0\n"
        "L -0x1.7303c6d890e6p+2 -0x1.1843fe8f18b4cp+0 0x1.63e3d021a7887p+3 -0x1.1843fe8f18b4cp+0\n"
        "L 0x1.63e3d021a7887p+3 -0x1.1843fe8f18b4cp+0 0x1.63e3d021a7887p+3 -0x1.a9f4cfc56db59p+3\n"
        "L 0x1.63e3d021a7887p+3 -0x1.a9f4cfc56db59p+3 0x1.2a2b5e3d66658p+4 -0x1.a9f4cfc56db59p+3\n"
        "L 0x1.2a2b5e3d66658p+4 -0x1.a9f4cfc56db59p+3 0x1.2a2b5e3d66658p+4 0x1.2a2b5e3d66658p+4\n"
        "L 0x1.2a2b5e3d66658p+4 0x1.2a2b5e3d66658p+4 0x1.63e3d021a7887p+3 0x1.2a2b5e3d66658p+4\n"
        "L 0x1.63e3d021a7887p+3 0x1.2a2b5e3d66658p+4 0x1.63e3d021a7887p+3 0x1.9ad4d90e8457fp+2\n"
        "L 0x1.63e3d021a7887p+3 0x1.9ad4d90e8457fp+2 -0x1.7303c6d890e6p+2 0x1.9ad4d90e8457fp+2\n"
        "L -0x1.7303c6d890e6p+2 0x1.9ad4d90e8457fp+2 -0x1.7303c6d890e6p+2 0x1.2a2b5e3d66658p+4\n"
        "L -0x1.7303c6d890e6p+2 0x1.2a2b5e3d66658p+4 -0x1.a9f4cfc56db59p+3 0x1.2a2b5e3d66658p+4\n"
        "L -0x1.a9f4cfc56db59p+3 0x1.2a2b5e3d66658p+4 -0x1.a9f4cfc56db59p+3 -0x1.a9f4cfc56db59p+3\n"
        "P 0x1.8p+3 120 0x1.b7932e06322cfp-1 0 1 0x1.5e24db7d16924p-1 0x1.9c6278d928573p+1 0x1.1d507b0c86ba5p+0 0x0p+0 0x1.4p+2 0 0x1.92d243b599cffp-2 1 1 0x1.20272f79173acp-1 0x1.efe4f9bef4f4ap-1 0 3 0x1.01896bcd1f8ccp+5 0x0p+0 0 0 0x0p+0 1 0x1.279686c88e3d2p+1 1 0 0x1.2ae9de00a4797p-3 0x1.363e5d0480a68p+3 0x1.f47efe8b2964fp-1 0x1.6509387da4a78p+3 0 0x1.d46c33cbdb61ep-1 0 0 0x0p+0 0x0p+0 0 1 0 0 0 0 -0x1.1d7acc22581cap+5 0x1.68d08b92c9edcp+5 0x1p+1\n"
        "K 0 0\n";

const char* const k_m3_9302_251 =
        "C 1\n"
        "L 0x1.853863bc0739dp+4 0x0p+0 0x1.7121f598a677dp+1 0x1.607a66d7e8cc6p+2\n"
        "L 0x1.7121f598a677dp+1 0x1.607a66d7e8cc6p+2 -0x1.8028169c4cb5p+3 0x1.4cb07a92968bap+4\n"
        "L -0x1.8028169c4cb5p+3 0x1.4cb07a92968bap+4 -0x1.1d0d9e46de7c6p+3 0x1.742bb48683f6dp-3\n"
        "L -0x1.1d0d9e46de7c6p+3 0x1.742bb48683f6dp-3 -0x1.7b1cddabcbfdcp+3 -0x1.48523f5469b7ap+4\n"
        "L -0x1.7b1cddabcbfdcp+3 -0x1.48523f5469b7ap+4 0x1.1ea137f622522p+2 -0x1.d12c063fb06a1p+2\n"
        "L 0x1.1ea137f622522p+2 -0x1.d12c063fb06a1p+2 0x1.853863bc0739dp+4 0x0p+0\n"
        "C 1\n"
        "L 0x1.1452c652765cdp+6 0x1.4cafb72d1ec84p+1 0x1.260b181ec00ep+6 0x1.8d5d5102d47e1p+2\n"
        "L 0x1.260b181ec00ep+6 0x1.8d5d5102d47e1p+2 0x1.1f55b54806758p+6 0x1.6bdb39b1b823cp+3\n"
        "L 0x1.1f55b54806758p+6 0x1.6bdb39b1b823cp+3 0x1.088317fde85e1p+6 0x1.75fb64eb67c8ep+3\n"
        "L 0x1.088317fde85e1p+6 0x1.75fb64eb67c8ep+3 0x1.fa13949cf518cp+5 0x1.d5fcd29ba451fp+3\n"
        "L 0x1.fa13949cf518cp+5 0x1.d5fcd29ba451fp+3 0x1.f9987257c804dp+5 0x1.466b7c2fcfdbcp+4\n"
        "L 0x1.f9987257c804dp+5 0x1.466b7c2fcfdbcp+4 0x1.d0f20cbdf5223p+5 0x1.64eea841a9a2ep+4\n"
        "L 0x1.d0f20cbdf5223p+5 0x1.64eea841a9a2ep+4 0x1.b281c4daee7d7p+5 0x1.20b6fe3d5699ep+4\n"
        "L 0x1.b281c4daee7d7p+5 0x1.20b6fe3d5699ep+4 0x1.916f28dd79007p+5 0x1.1ac2f9d1dfef4p+4\n"
        "L 0x1.916f28dd79007p+5 0x1.1ac2f9d1dfef4p+4 0x1.6d64f9a41f4d1p+5 0x1.530342c3353b4p+4\n"
        "L 0x1.6d64f9a41f4d1p+5 0x1.530342c3353b4p+4 0x1.481f2ec44dfefp+5 0x1.2679597d505bp+4\n"
        "L 0x1.481f2ec44dfefp+5 0x1.2679597d505bp+4 0x1.4fcf98a55f4bep+5 0x1.98b0dd455274dp+3\n"
        "L 0x1.4fcf98a55f4bep+5 0x1.98b0dd455274dp+3 0x1.3d848397c6034p+5 0x1.29d69956addfp+3\n"
        "L 0x1.3d848397c6034p+5 0x1.29d69956addfp+3 0x1.110ec8b77e472p+5 0x1.fe8c596c44b28p+2\n"
        "L 0x1.110ec8b77e472p+5 0x1.fe8c596c44b28p+2 0x1.0b3ac2a4b4da1p+5 0x1.4cafb72d1ec8ap+1\n"
        "L 0x1.0b3ac2a4b4da1p+5 0x1.4cafb72d1ec8ap+1 0x1.33419cdc0c22p+5 -0x1.4aea16ed0c2cp-3\n"
        "L 0x1.33419cdc0c22p+5 -0x1.4aea16ed0c2cp-3 0x1.3d848397c6034p+5 -0x1.06fd7b803cf58p+2\n"
        "L 0x1.3d848397c6034p+5 -0x1.06fd7b803cf58p+2 0x1.2a1de9d1cbbfbp+5 -0x1.2912ee121f8bbp+3\n"
        "L 0x1.2a1de9d1cbbfbp+5 -0x1.2912ee121f8bbp+3 0x1.481f2ec44dfeep+5 -0x1.a69ad7641151cp+3\n"
        "L 0x1.481f2ec44dfeep+5 -0x1.a69ad7641151cp+3 0x1.7258629261d73p+5 -0x1.60822e8435883p+3\n"
        "L 0x1.7258629261d73p+5 -0x1.60822e8435883p+3 0x1.916f28dd79006p+5 -0x1.8f2e180d307a6p+3\n"
        "L 0x1.916f28dd79006p+5 -0x1.8f2e180d307a6p+3 0x1.a5b3a3ce841fep+5 -0x1.198b702f1e0ep+4\n"
        "L 0x1.a5b3a3ce841fep+5 -0x1.198b702f1e0ep+4 0x1.d0f20cbdf5222p+5 -0x1.11c2ba7661f0ep+4\n"
        "L 0x1.d0f20cbdf5222p+5 -0x1.11c2ba7661f0ep+4 0x1.dd920d5ffe857p+5 -0x1.73c53ee0110d7p+3\n"
        "L 0x1.dd920d5ffe857p+5 -0x1.73c53ee0110d7p+3 0x1.fa13949cf518bp+5 -0x1.2fa4f70514edep+3\n"
        "L 0x1.fa13949cf518bp+5 -0x1.2fa4f70514edep+3 0x1.13600e1c0faeep+6 -0x1.5675041e45401p+3\n"
        "L 0x1.13600e1c0faeep+6 -0x1.5675041e45401p+3 0x1.1f55b54806758p+6 -0x1.8b06bc36517f8p+2\n"
        "L 0x1.1f55b54806758p+6 -0x1.8b06bc36517f8p+2 0x1.12183a3f64f72p+6 -0x1.839d42e3b6658p+0\n"
        "L 0x1.12183a3f64f72p+6 -0x1.839d42e3b6658p+0 0x1.1452c652765cdp+6 0x1.4cafb72d1ec84p+1\n"
        "P 0x1.8p+2 90 0x0p+0 1 0 0x0p+0 0x1.84417e6953d22p+1 0x1.8f1d678224d39p+2 0x1.1dcccd6231afep-1 0x1.4p+2 0 0x1.cb459a0f44887p-2 1 1 0x1.58e3a6bd6979fp-1 0x1.3b4fb7a77a0ep+1 0 5 0x1.e7d06aa6e4a35p+5 0x0p+0 1 0 0x0p+0 2 0x1.c78adb3d19746p+4 0 0 0x1.840cea00fc8b5p-2 0x1.b61000c9498f6p+3 0x1.432b9161c0665p+0 0x1.14c4a3979fd8fp+3 0 0x1.573412dce1a97p-2 0 0 0x0p+0 0x0p+0 0 1 0 0 0 0 0x1.480190e70abdcp+4 -0x1.24c47e93bcb45p+5 0x1p+1\n"
        "K 0 0\n";

const char* const k_m3_9303_94 =
        "C 1\n"
        "L 0x1.8e7d87a3d7fd8p+2 0x0p+0 0x1.5489937819fp+3 0x0p+0\n"
        "L 0x1.5489937819fp+3 0x0p+0 0x1.5489937819fp+3 0x1.8e7d87a3d7fd8p+3\n"
        "L 0x1.5489937819fp+3 0x1.8e7d87a3d7fd8p+3 0x1.0de42ba502f76p+4 0x1.8e7d87a3d7fd8p+3\n"
        "L 0x1.0de42ba502f76p+4 0x1.8e7d87a3d7fd8p+3 0x1.0de42ba502f76p+4 0x1.0de42ba502f76p+4\n"
        "L 0x1.0de42ba502f76p+4 0x1.0de42ba502f76p+4 0x0p+0 0x1.0de42ba502f76p+4\n"
        "L 0x0p+0 0x1.0de42ba502f76p+4 0x0p+0 0x1.8e7d87a3d7fd8p+3\n"
        "L 0x0p+0 0x1.8e7d87a3d7fd8p+3 0x1.8e7d87a3d7fd8p+2 0x1.8e7d87a3d7fd8p+3\n"
        "L 0x1.8e7d87a3d7fd8p+2 0x1.8e7d87a3d7fd8p+3 0x1.8e7d87a3d7fd8p+2 0x0p+0\n"
        "C 1\n"
        "L 0x1.9p+5 0x0p+0 0x1.dd743084dabdap+5 0x0p+0\n"
        "L 0x1.dd743084dabdap+5 0x0p+0 0x1.dd743084dabdap+5 0x1.920ebf44a0531p+3\n"
        "L 0x1.dd743084dabdap+5 0x1.920ebf44a0531p+3 0x1.2c83afd12814cp+6 0x1.920ebf44a0531p+3\n"
        "L 0x1.2c83afd12814cp+6 0x1.920ebf44a0531p+3 0x1.2c83afd12814cp+6 0x0p+0\n"
        "L 0x1.2c83afd12814cp+6 0x0p+0 0x1.533dc81395739p+6 0x0p+0\n"
        "L 0x1.533dc81395739p+6 0x0p+0 0x1.533dc81395739p+6 0x1.167b90272ae72p+5\n"
        "L 0x1.533dc81395739p+6 0x1.167b90272ae72p+5 0x1.2c83afd12814cp+6 0x1.167b90272ae72p+5\n"
        "L 0x1.2c83afd12814cp+6 0x1.167b90272ae72p+5 0x1.2c83afd12814cp+6 0x1.63efc0ac05a4cp+4\n"
        "L 0x1.2c83afd12814cp+6 0x1.63efc0ac05a4cp+4 0x1.dd743084dabdap+5 0x1.63efc0ac05a4cp+4\n"
        "L 0x1.dd743084dabdap+5 0x1.63efc0ac05a4cp+4 0x1.dd743084dabdap+5 0x1.167b90272ae72p+5\n"
        "L 0x1.dd743084dabdap+5 0x1.167b90272ae72p+5 0x1.9p+5 0x1.167b90272ae72p+5\n"
        "L 0x1.9p+5 0x1.167b90272ae72p+5 0x1.9p+5 0x0p+0\n"
        "P 0x1.9p+4 120 0x1.5d305d6c572f4p-1 0 0 0x0p+0 0x1.07697109f75e8p+2 0x1.525c1ec2c7b1cp+1 0x0p+0 0x1.4p+2 0 0x1.b8bd00e7ee5b7p-1 1 1 0x1.3e1c572fe3373p+0 0x1.09f3bfb8c85acp+0 0 9 0x1.c0f2cd403c06ep+4 0x0p+0 1 0 0x0p+0 1 0x1.0c911f52cb3bcp+4 0 0 0x1.6fcad3e782b28p-2 0x1.374c185af75aep+2 0x1.25ac621fd4dc8p+0 0x1.6e9f4c17d495ep+1 0 0x1.757aa076121efp-3 0 0 0x0p+0 0x0p+0 2 6 0 0 0 1 -0x1.16f0558b0a2e3p+5 -0x1.47ed166eae02ap+5 0x1p+1\n"
        "K 1 1\n";

const char* const k_m3_9304_67 =
        "C 1\n"
        "L 0x1.565d928f06a1dp+3 0x0p+0 0x1.8c1f0662d5fe9p+3 0x1.3348c0a2e699dp+0\n"
        "L 0x1.8c1f0662d5fe9p+3 0x1.3348c0a2e699dp+0 0x1.826a202f0b136p+3 0x1.7cf874e4a85b8p+1\n"
        "L 0x1.826a202f0b136p+3 0x1.7cf874e4a85b8p+1 0x1.42f51a9a1ea0ep+3 0x1.c68903e4ef433p+1\n"
        "L 0x1.42f51a9a1ea0ep+3 0x1.c68903e4ef433p+1 0x1.2f264cdfa32e7p+3 0x1.3e35db5fd1e67p+2\n"
        "L 0x1.2f264cdfa32e7p+3 0x1.3e35db5fd1e67p+2 0x1.4ce5cb6c22a4ep+3 0x1.b431eef19d273p+2\n"
        "L 0x1.4ce5cb6c22a4ep+3 0x1.b431eef19d273p+2 0x1.29e442446f151p+3 0x1.07e8b1a65539dp+3\n"
        "L 0x1.29e442446f151p+3 0x1.07e8b1a65539dp+3 0x1.d2500cb56689bp+2 0x1.f5683687b2958p+2\n"
        "L 0x1.d2500cb56689bp+2 0x1.f5683687b2958p+2 0x1.84f8b84a3e06fp+2 0x1.19c2e45f602c2p+3\n"
        "L 0x1.84f8b84a3e06fp+2 0x1.19c2e45f602c2p+3 0x1.82d28e6d70bb6p+2 0x1.5bd22df415dddp+3\n"
        "L 0x1.82d28e6d70bb6p+2 0x1.5bd22df415dddp+3 0x1.1a402be4726cp+2 0x1.741deed59e46cp+3\n"
        "L 0x1.1a402be4726cp+2 0x1.741deed59e46cp+3 0x1.67c48de2e53bfp+1 0x1.4a5711fc809e2p+3\n"
        "L 0x1.67c48de2e53bfp+1 0x1.4a5711fc809e2p+3 0x1.4a240cfffa297p+0 0x1.53de899f91e5fp+3\n"
        "L 0x1.4a240cfffa297p+0 0x1.53de899f91e5fp+3 0x1.33bbb99c72274p-2 0x1.8ddce6ee543acp+3\n"
        "L 0x1.33bbb99c72274p-2 0x1.8ddce6ee543acp+3 -0x1.7fc4c5886f45fp+0 0x1.8b13cfa70732ep+3\n"
        "L -0x1.7fc4c5886f45fp+0 0x1.8b13cfa70732ep+3 -0x1.27821e2b0abcfp+1 0x1.4e4cbcc3b9aecp+3\n"
        "L -0x1.27821e2b0abcfp+1 0x1.4e4cbcc3b9aecp+3 -0x1.e59e317e8e303p+1 0x1.401e08ac80e9cp+3\n"
        "L -0x1.e59e317e8e303p+1 0x1.401e08ac80e9cp+3 -0x1.60c30efef03d3p+2 0x1.64c25a2c42ffcp+3\n"
        "L -0x1.60c30efef03d3p+2 0x1.64c25a2c42ffcp+3 -0x1.c427e24b4bf03p+2 0x1.4787c708657e8p+3\n"
        "L -0x1.c427e24b4bf03p+2 0x1.4787c708657e8p+3 -0x1.b98b213b1d971p+2 0x1.05ace38e9039ap+3\n"
        "L -0x1.b98b213b1d971p+2 0x1.05ace38e9039ap+3 -0x1.0043a72dc36c9p+3 0x1.c60f87b028837p+2\n"
        "L -0x1.0043a72dc36c9p+3 0x1.c60f87b028837p+2 -0x1.41f8cbab6696ap+3 0x1.d3da3d1bf8031p+2\n"
        "L -0x1.41f8cbab6696ap+3 0x1.d3da3d1bf8031p+2 -0x1.60649b4f8f321p+3 0x1.71e674f86b97cp+2\n"
        "L -0x1.60649b4f8f321p+3 0x1.71e674f86b97cp+2 -0x1.3d171b7a83a6fp+3 0x1.023562f6348e2p+2\n"
        "L -0x1.3d171b7a83a6fp+3 0x1.023562f6348e2p+2 -0x1.4c6ac05eddcf6p+3 0x1.47bbd567b8d9ap+1\n"
        "L -0x1.4c6ac05eddcf6p+3 0x1.47bbd567b8d9ap+1 -0x1.89cd88d774e18p+3 0x1.cc076b26c7811p+0\n"
        "L -0x1.89cd88d774e18p+3 0x1.cc076b26c7811p+0 -0x1.8dfaa67ee096bp+3 0x1.b6ff0f9007c4ap-50\n"
        "L -0x1.8dfaa67ee096bp+3 0x1.b6ff0f9007c4ap-50 -0x1.54c46933d483ep+3 -0x1.0858248fbc83cp+0\n"
        "L -0x1.54c46933d483ep+3 -0x1.0858248fbc83cp+0 -0x1.4c6ac05eddcf6p+3 -0x1.47bbd567b8d94p+1\n"
        "L -0x1.4c6ac05eddcf6p+3 -0x1.47bbd567b8d94p+1 -0x1.776b1b8fd2ddp+3 -0x1.082f4db30785ap+2\n"
        "L -0x1.776b1b8fd2ddp+3 -0x1.082f4db30785ap+2 -0x1.60649b4f8f324p+3 -0x1.71e674f86b974p+2\n"
        "L -0x1.60649b4f8f324p+3 -0x1.71e674f86b974p+2 -0x1.1e60e77475e3p+3 -0x1.773dbd68f7cabp+2\n"
        "L -0x1.1e60e77475e3p+3 -0x1.773dbd68f7cabp+2 -0x1.0043a72dc36cap+3 -0x1.c60f87b028834p+2\n"
        "L -0x1.0043a72dc36cap+3 -0x1.c60f87b028834p+2 -0x1.0f07b35d1d927p+3 -0x1.236d78f84791ap+3\n"
        "L -0x1.0f07b35d1d927p+3 -0x1.236d78f84791ap+3 -0x1.c427e24b4bf06p+2 -0x1.4787c708657e7p+3\n"
        "L -0x1.c427e24b4bf06p+2 -0x1.4787c708657e7p+3 -0x1.4cc496c5a59p+2 -0x1.2b376db639cb7p+3\n"
        "L -0x1.4cc496c5a59p+2 -0x1.2b376db639cb7p+3 -0x1.e59e317e8e30ep+1 -0x1.401e08ac80e9bp+3\n"
        "L -0x1.e59e317e8e30ep+1 -0x1.401e08ac80e9bp+3 -0x1.a2354bbd19685p+1 -0x1.80001422bfd38p+3\n"
        "L -0x1.a2354bbd19685p+1 -0x1.80001422bfd38p+3 -0x1.7fc4c5886f47ap+0 -0x1.8b13cfa70732ep+3\n"
        "L -0x1.7fc4c5886f47ap+0 -0x1.8b13cfa70732ep+3 -0x1.08bb0c92cbeaep-2 -0x1.5643fb315a16p+3\n"
        "L -0x1.08bb0c92cbeaep-2 -0x1.5643fb315a16p+3 0x1.4a240cfffa2ap+0 -0x1.53de899f91e5fp+3\n"
        "L 0x1.4a240cfffa2ap+0 -0x1.53de899f91e5fp+3 0x1.5782a9db3efa9p+1 -0x1.849a67b10215ep+3\n"
        "L 0x1.5782a9db3efa9p+1 -0x1.849a67b10215ep+3 0x1.1a402be4726bcp+2 -0x1.741deed59e46dp+3\n"
        "L 0x1.1a402be4726bcp+2 -0x1.741deed59e46dp+3 0x1.2f778db0ba25ap+2 -0x1.32e7da247ac38p+3\n"
        "L 0x1.2f778db0ba25ap+2 -0x1.32e7da247ac38p+3 0x1.84f8b84a3e062p+2 -0x1.19c2e45f602c6p+3\n"
        "L 0x1.84f8b84a3e062p+2 -0x1.19c2e45f602c6p+3 0x1.00a23bd42c9p+3 -0x1.302e82e69c4d6p+3\n"
        "L 0x1.00a23bd42c9p+3 -0x1.302e82e69c4d6p+3 0x1.29e442446f14ap+3 -0x1.07e8b1a6553a5p+3\n"
        "L 0x1.29e442446f14ap+3 -0x1.07e8b1a6553a5p+3 0x1.14fac1f40bca1p+3 -0x1.92798cc518f3fp+2\n"
        "L 0x1.14fac1f40bca1p+3 -0x1.92798cc518f3fp+2 0x1.2f264cdfa32e8p+3 -0x1.3e35db5fd1e66p+2\n"
        "L 0x1.2f264cdfa32e8p+3 -0x1.3e35db5fd1e66p+2 0x1.709920c0f97f2p+3 -0x1.2c26d80507ca7p+2\n"
        "L 0x1.709920c0f97f2p+3 -0x1.2c26d80507ca7p+2 0x1.826a202f0b136p+3 -0x1.7cf874e4a85b9p+1\n"
        "L 0x1.826a202f0b136p+3 -0x1.7cf874e4a85b9p+1 0x1.52c5dad8949dfp+3 -0x1.8bbe9a8f5aa3dp+0\n"
        "L 0x1.52c5dad8949dfp+3 -0x1.8bbe9a8f5aa3dp+0 0x1.565d928f06a1dp+3 0x0p+0\n"
        "P 0x1.8p+3 90 0x1.94e53dc84c619p-4 1 1 0x0p+0 0x1.1763fdfd99ee8p+1 0x1.9fb07890992a2p+2 0x0p+0 0x1.4p+2 1 0x1.ac02cd1e531c7p-1 0 1 0x1.23058bac648a9p+0 0x1.b826472981945p+0 0 7 0x1.09f3f31f23bd8p+4 0x0p+0 0 1 0x0p+0 3 0x1.cff3b440cbb2ap+4 1 0 0x1.a890accb70e11p-2 0x1.25d8a1af0229fp+2 0x1.562dd0752f5c4p-2 0x1.39cae4f45cc54p+0 0 0x1.1aec3b1d5beb2p-3 0 0 0x0p+0 0x0p+0 1 4 0 0 0 1 -0x1.ea8b4f04e0fb4p+4 0x1.dcd1c589b39f8p+3 0x1p+1\n"
        "K 0 1\n";

const char* const k_m3_9301_367 =
        "C 1\n"
        "A 0x1.23af98a95df39p+4 0x1.e347f3a2cd82p+2 0x1.fd6b25e215a6fp+3 -0x1.fa14ff2296178p-2 0x1.47951a6ac6eeap+0 0\n"
        "A 0x1.0aea8282bc166p+4 0x1.423238c4a231fp+5 0x1.283a0a2b86cc3p+4 -0x1.3be7971d87243p+0 -0x1.58b3d7068ff3dp+1 1\n"
        "A -0x1.2d7c5ff4e4ecep+3 0x1.6becf42e5d461p+4 0x1.ac2238899f987p+3 0x1.943bf5dfa4df4p-1 0x1.9198a51d6a4e1p+1 0\n"
        "A -0x1.2cdef02de7dc2p+4 0x1.f27fc84fae3edp+2 0x1.f1185bc777996p+3 0x1.d4ba131662c38p+0 -0x1.4ee4f6dc42162p+1 0\n"
        "A -0x1.3c6a10cc4a81ep+3 -0x1.06204eedea1d8p+2 0x1.6ba989c5fd4cep+4 0x1.7aee845be6dep+1 -0x1.1666970ad629bp+1 0\n"
        "A -0x1.2722435464c09p+2 -0x1.6442102f5cbaep+3 0x1.59c071b6b85a6p+4 -0x1.491eab0def411p+1 -0x1.5b11ef0ec889bp+0 0\n"
        "A 0x1.cd1956e6b0316p+2 -0x1.164c402b3cef2p+4 0x1.081a91a7989f2p+4 -0x1.02e069d169f8ap+1 -0x1.55baf10e4124ap-2 0\n"
        "A 0x1.12a9a66b264ap+5 -0x1.c713784cab9b5p+3 0x1.cbf48e5cc324bp+3 -0x1.4040ebe649f7ap+1 0x1.b771fd2a72868p+0 1\n"
        "P 0x1.9p+4 120 -0x0p+0 0 0 0x0p+0 0x1.8f55a3abbc38p+0 0x1.1f84d6e98632cp+2 0x0p+0 0x1.4p+2 1 0x1.232623565575fp-2 1 1 0x1.ccb1dc372e72ep+0 0x1.dfed212cdc27ap-1 0 9 0x1.2a93e280720e7p+6 0x0p+0 1 1 0x0p+0 0 0x1.317963170336fp+5 0 0 0x1.dc8ecd603e022p-4 0x1.b780e2605fe4ep+3 0x1.79ce86825391ap+0 0x1.84c4950d6bd1p+2 0 0x1.11465cb65c2bdp+0 0 0 0x0p+0 0x0p+0 0 1 0 0 0 1 0x1.6c02919d07b4ep+5 0x1.8b55e96ef30c4p+5 0x1p+1\n"
        "K 0 0\n";

const char* const k_m1_1310133_29 =
        "C 1\n"
        "L 0x0p+0 0x0p+0 0x1.bb443074f069ap+3 0x0p+0\n"
        "L 0x1.bb443074f069ap+3 0x0p+0 0x1.bb443074f069ap+3 0x1.bb443074f069ap+3\n"
        "L 0x1.bb443074f069ap+3 0x1.bb443074f069ap+3 0x1.43e8f57179c23p+3 0x1.bb443074f069ap+3\n"
        "L 0x1.43e8f57179c23p+3 0x1.bb443074f069ap+3 0x1.43e8f57179c23p+3 0x1.dd6cec0dda9dbp+1\n"
        "L 0x1.43e8f57179c23p+3 0x1.dd6cec0dda9dbp+1 0x1.dd6cec0dda9dbp+1 0x1.dd6cec0dda9dbp+1\n"
        "L 0x1.dd6cec0dda9dbp+1 0x1.dd6cec0dda9dbp+1 0x1.dd6cec0dda9dbp+1 0x1.bb443074f069ap+3\n"
        "L 0x1.dd6cec0dda9dbp+1 0x1.bb443074f069ap+3 0x0p+0 0x1.bb443074f069ap+3\n"
        "L 0x0p+0 0x1.bb443074f069ap+3 0x0p+0 0x0p+0\n"
        "C 1\n"
        "L 0x1.9p+5 0x0p+0 0x1.4fe5a28127ca3p+6 0x0p+0\n"
        "L 0x1.4fe5a28127ca3p+6 0x0p+0 0x1.4fe5a28127ca3p+6 0x1.0fcb45024f946p+5\n"
        "L 0x1.4fe5a28127ca3p+6 0x1.0fcb45024f946p+5 0x1.2469a58324995p+6 0x1.0fcb45024f946p+5\n"
        "L 0x1.2469a58324995p+6 0x1.0fcb45024f946p+5 0x1.2469a58324995p+6 0x1.5bdfe7f019873p+3\n"
        "L 0x1.2469a58324995p+6 0x1.5bdfe7f019873p+3 0x1.e6f7f9fc0661dp+5 0x1.5bdfe7f019873p+3\n"
        "L 0x1.e6f7f9fc0661dp+5 0x1.5bdfe7f019873p+3 0x1.e6f7f9fc0661dp+5 0x1.0fcb45024f946p+5\n"
        "L 0x1.e6f7f9fc0661dp+5 0x1.0fcb45024f946p+5 0x1.9p+5 0x1.0fcb45024f946p+5\n"
        "L 0x1.9p+5 0x1.0fcb45024f946p+5 0x1.9p+5 0x0p+0\n"
        "P 0x1.8p+2 60 -0x1.430f8b25da1dap-1 1 1 0x0p+0 0x1.c04666ef21f2p+0 0x1.71184036a6264p-1 0x0p+0 0x1.4p+2 1 0x1.cacb143ee2c07p-1 1 1 0x1.47bc36997d378p+2 0x1.6412eb9ca607p-1 0 3 0x1.b4f3608367d23p+4 0x0p+0 1 0 0x0p+0 0 0x1.272bd0311b292p+4 0 0 0x1.4a5ff819ac184p-2 0x1.4a19026d170b5p+2 0x1.405f37edfc63ep+0 0x1.6f8b6308cb031p+3 0 0x1.d38f960f635cap-2 0 0 0x0p+0 0x0p+0 2 7 0 1 0 1 0x1.eb2b062536fb8p+3 -0x1.7559e2f6d2234p+5 0x1p+1\n"
        "K 1 1\n";

const char* const k_m0_271000_0 =
        "C 1\n"
        "L -0x1.eb948e6dd9238p+4 -0x1.917d2ad83de25p+4 0x1.2fbc0d17326c6p+4 -0x1.917d2ad83de25p+4\n"
        "L 0x1.2fbc0d17326c6p+4 -0x1.917d2ad83de25p+4 0x1.2fbc0d17326c6p+4 0x1.eb948e6dd9238p+4\n"
        "L 0x1.2fbc0d17326c6p+4 0x1.eb948e6dd9238p+4 -0x1.eb948e6dd9238p+4 0x1.eb948e6dd9238p+4\n"
        "L -0x1.eb948e6dd9238p+4 0x1.eb948e6dd9238p+4 -0x1.eb948e6dd9238p+4 -0x1.917d2ad83de25p+4\n"
        "P 0x1.8p+2 90 0x1.20ab087d865cfp-1 2 0 0x0p+0 0x1.ebf7250712cb9p+1 0x1.ec60cbc3a8073p+2 0x1.29876646ca16cp-1 0x1.4p+2 1 0x1.c565bbf865529p-1 1 1 0x1.18bfc3ce91eap-1 0x1.d9395ef4c6397p-2 0 9 0x1.134463d47e419p+3 0x0p+0 0 0 0x0p+0 3 0x1.8a4c81f170ed2p+3 0 0 0x1.66eec138ec4dfp-2 0x1.850e6600ef97bp+3 0x1.4455c13b310efp+0 0x1.3b06343104d9bp+3 2 0x1.15fb9174317bfp+0 1 0 0x0p+0 0x0p+0 2 0 0 1 0 0 0x1.50afc094c51f8p+5 -0x1.8623029076f3ap+5 0x1p+1\n"
        "K 0 0\n";


const char* const k_m3_9301_272 =
        "C 1\n"
        "L -0x1.9c1c3a9ff743p+3 -0x1.43ce5119a639bp+2 0x1.fdcc15c5359a9p+1 -0x1.43ce5119a639bp+2\n"
        "L 0x1.fdcc15c5359a9p+1 -0x1.43ce5119a639bp+2 0x1.fdcc15c5359a9p+1 0x1.9c1c3a9ff743p+3\n"
        "L 0x1.fdcc15c5359a9p+1 0x1.9c1c3a9ff743p+3 -0x1.9c1c3a9ff743p+3 0x1.9c1c3a9ff743p+3\n"
        "L -0x1.9c1c3a9ff743p+3 0x1.9c1c3a9ff743p+3 -0x1.9c1c3a9ff743p+3 -0x1.43ce5119a639bp+2\n"
        "C 1\n"
        "L 0x1.0f29bc11c35fp+5 0x1.72fbf4a746268p+1 0x1.8c455345deb2bp+4 0x1.32a4a776c9291p+3\n"
        "A 0x1.45bcd29d488a8p+4 0x1.c7fa712183762p+1 0x1.dd8c808f48e16p+2 0x1.e0a460054297fp-1 -0x1.19f69d42f22b9p+1 0\n"
        "L 0x1.fe68a3e964c48p+3 -0x1.3a9dbb981db8p+1 0x1.914276d25a6d9p+4 -0x1.248d1932ff0d7p+3\n"
        "A 0x1.d7caf77af095cp+4 -0x1.8f9c38125b07ap+1 0x1.dd8c808f48e16p+2 -0x1.19f69d42f22b8p+1 0x1.e0a460054297ep-1 0\n"
        "P 0x1.8p+2 90 0x1.524e1595adec5p-1 0 1 0x0p+0 0x1.4bec877488374p+1 0x1.3a91e699d0a5cp+0 0x0p+0 0x1.4p+2 1 0x1.c87f62adebf5fp-1 1 1 0x1.48dd42e36fee1p+1 0x1.615fe92444212p-1 1 6 0x1.6891600a3f1ddp+4 0x0p+0 0 0 0x0p+0 1 0x1.33eab6914ed4p+5 0 0 0x1.5677faadb205cp-2 0x1.aa08c5abe5b5cp+0 0x1.482e47466d65ap-2 0x1.4dd72ec1e6caep+1 0 0x1.8e0d01ee83dcdp-1 0 0 0x0p+0 0x0p+0 2 6 0 0 0 0 0x1.67eabfe49a2e6p+5 -0x1.8e9f9a8d1b46cp+3 0x1p+1\n"
        "K 0 0\n";

const char* const k_m3_9301_441 =
        "C 1\n"
        "A 0x1.6ba590f1b602ep+3 0x1.a3e730e530eb4p+2 0x1.b517ebe9b8231p+3 -0x1.0087f66f6b8f9p-1 0x1.8c591eba8cfe2p+0 0\n"
        "A 0x1.b949efd832b87p-49 0x1.0fe1f20be18c9p+4 0x1.837e6efeb4b88p+3 0x1.13f6bf9aad023p-2 0x1.6fa0dd50ed313p+1 0\n"
        "A -0x1.974f93808c347p+3 0x1.d65271f58b7a9p+2 0x1.9d286c1a28058p+3 0x1.7d27ba63f0075p+0 -0x1.449e6ef3639eep+1 0\n"
        "A -0x1.3a865cb94ceap+4 -0x1.6b2e989f8dc09p+3 0x1.7ddc1db32804ap+3 0x1.e285ede972feap+0 -0x1.ace194cd3791p-1 1\n"
        "A -0x1.88f60675697ep-49 -0x1.1e802a3ae958cp+3 0x1.03873f84d533dp+4 -0x1.2fdf71240e0bcp+1 -0x1.89011080d3174p-1 0\n"
        "A 0x1.338cd5befe675p+4 -0x1.6320d8ce6cb56p+3 0x1.7ad64bce0e668p+3 -0x1.2194298ba6546p+1 0x1.37132f9475726p+0 1\n"
        "P 0x1.9p+4 90 0x1.da10f67ffe75bp-1 1 0 0x0p+0 0x1.0cce4588d5e4ep+2 0x1.5619f7e984e76p+0 0x0p+0 0x1.4p+2 0 0x1.6be1fb567efe8p-1 0 1 0x1.ba7f332595432p-1 0x1.20e06d7550c1ep+1 0 4 0x1.b5a4f6e0a3fcp+5 0x0p+0 0 0 0x0p+0 2 0x1.5e07989464458p+4 0 0 0x1.c84f0514bb601p-3 0x1.2f1921c7f387ap+2 0x1.1db9435cf487cp+0 0x1.35586f68d8d6p+1 0 0x1.dd6e99c2478dfp-1 0 0 0x0p+0 0x0p+0 0 7 0 0 0 0 -0x1.2ca6c352eb3f8p+2 -0x1.13b5ecf965622p+5 0x1p+1\n"
        "K 1 0\n";

const char* const k_m0_9306_257 =
        "C 1\n"
        "L 0x0p+0 0x0p+0 0x1.24ffd452cca9cp+4 0x0p+0\n"
        "L 0x1.24ffd452cca9cp+4 0x0p+0 0x1.24ffd452cca9cp+4 0x1.24ffd452cca9cp+4\n"
        "L 0x1.24ffd452cca9cp+4 0x1.24ffd452cca9cp+4 0x1.aa43707af75ffp+3 0x1.24ffd452cca9cp+4\n"
        "L 0x1.aa43707af75ffp+3 0x1.24ffd452cca9cp+4 0x1.aa43707af75ffp+3 0x1.3f78705543e72p+2\n"
        "L 0x1.aa43707af75ffp+3 0x1.3f78705543e72p+2 0x1.3f78705543e72p+2 0x1.3f78705543e72p+2\n"
        "L 0x1.3f78705543e72p+2 0x1.3f78705543e72p+2 0x1.3f78705543e72p+2 0x1.24ffd452cca9cp+4\n"
        "L 0x1.3f78705543e72p+2 0x1.24ffd452cca9cp+4 0x0p+0 0x1.24ffd452cca9cp+4\n"
        "L 0x0p+0 0x1.24ffd452cca9cp+4 0x0p+0 0x0p+0\n"
        "P 0x1.9p+4 90 0x0p+0 0 1 0x0p+0 0x1.31b36ca1aca7ap+2 0x1.35c56aa20db58p+1 0x0p+0 0x1.4p+2 1 0x1.8bdbf7d8282cfp-1 1 1 0x1.0a5b0059208c1p+0 0x1.22d8959762eadp-2 1 6 0x1.31f942e260978p+6 0x0p+0 0 0 0x0p+0 0 0x1.639aab4d31394p+4 1 0 0x1.b77722a07a436p-2 0x1.40f78f64f29bfp+3 0x1.1f3e478cd8175p-4 0x1.d7452a900b6abp+2 0 0x1.0c0bbf5b5f2bp+0 0 0 0x0p+0 0x0p+0 2 2 0 0 0 0 -0x1.51e5cd47c9f67p+5 0x1.668d7393245a4p+4 0x1p+1\n"
        "K 1 0\n";

const char* const k_m3_9305_215 =
        "C 1\n"
        "L 0x1.237832513d241p+3 0x0p+0 0x1.4ee51dabd1bb9p+3 0x1.e241ebe4309bp-1\n"
        "L 0x1.4ee51dabd1bb9p+3 0x1.e241ebe4309bp-1 0x1.47d1736659b42p+3 0x1.2b4a1810e8a36p+1\n"
        "L 0x1.47d1736659b42p+3 0x1.2b4a1810e8a36p+1 0x1.1534384ec6e4fp+3 0x1.6846b0efeb852p+1\n"
        "L 0x1.1534384ec6e4fp+3 0x1.6846b0efeb852p+1 0x1.069adec126682p+3 0x1.f9db04769f549p+1\n"
        "L 0x1.069adec126682p+3 0x1.f9db04769f549p+1 0x1.20a6f826b23ecp+3 0x1.58ec60b47871ap+2\n"
        "L 0x1.20a6f826b23ecp+3 0x1.58ec60b47871ap+2 0x1.06e3c739b6489p+3 0x1.a34b907e0a96fp+2\n"
        "L 0x1.06e3c739b6489p+3 0x1.a34b907e0a96fp+2 0x1.a5586841b1659p+2 0x1.92d8eed2177ecp+2\n"
        "L 0x1.a5586841b1659p+2 0x1.92d8eed2177ecp+2 0x1.6b74e00d986f1p+2 0x1.c7c2954832ecbp+2\n"
        "L 0x1.6b74e00d986f1p+2 0x1.c7c2954832ecbp+2 0x1.727a09220633fp+2 0x1.189fc5733a7a9p+3\n"
        "L 0x1.727a09220633fp+2 0x1.189fc5733a7a9p+3 0x1.23c91e513442dp+2 0x1.2ef3151412bbap+3\n"
        "L 0x1.23c91e513442dp+2 0x1.2ef3151412bbap+3 0x1.99a90301f277p+1 0x1.10e248fe0e64cp+3\n"
        "L 0x1.99a90301f277p+1 0x1.10e248fe0e64cp+3 0x1.036ea9b91185cp+1 0x1.1c2969a7fc16fp+3\n"
        "L 0x1.036ea9b91185cp+1 0x1.1c2969a7fc16fp+3 0x1.6916047289227p+0 0x1.4d349b1ef7447p+3\n"
        "L 0x1.6916047289227p+0 0x1.4d349b1ef7447p+3 0x1.72e7572d74f88p-51 0x1.503fa5b28f31fp+3\n"
        "L 0x1.72e7572d74f88p-51 0x1.503fa5b28f31fp+3 -0x1.a208b56b75615p-1 0x1.224bd04d6fdb7p+3\n"
        "L -0x1.a208b56b75615p-1 0x1.224bd04d6fdb7p+3 -0x1.036ea9b91185ap+1 0x1.1c2969a7fc16fp+3\n"
        "L -0x1.036ea9b91185ap+1 0x1.1c2969a7fc16fp+3 -0x1.9fa04ba5107e2p+1 0x1.3fca9c0475ec7p+3\n"
        "L -0x1.9fa04ba5107e2p+1 0x1.3fca9c0475ec7p+3 -0x1.23c91e513442cp+2 0x1.2ef3151412bbap+3\n"
        "L -0x1.23c91e513442cp+2 0x1.2ef3151412bbap+3 -0x1.2afd2fc447004p+2 0x1.f46c5e29e2d4fp+2\n"
        "L -0x1.2afd2fc447004p+2 0x1.f46c5e29e2d4fp+2 -0x1.6b74e00d986fp+2 0x1.c7c2954832ecdp+2\n"
        "L -0x1.6b74e00d986fp+2 0x1.c7c2954832ecdp+2 -0x1.d0bcdbad4a8f1p+2 0x1.e613da94d0149p+2\n"
        "L -0x1.d0bcdbad4a8f1p+2 0x1.e613da94d0149p+2 -0x1.06e3c739b6489p+3 0x1.a34b907e0a97p+2\n"
        "L -0x1.06e3c739b6489p+3 0x1.a34b907e0a97p+2 -0x1.e6815d26d1f7bp+2 0x1.4123acc5fbb44p+2\n"
        "L -0x1.e6815d26d1f7bp+2 0x1.4123acc5fbb44p+2 -0x1.069adec126681p+3 0x1.f9db04769f54bp+1\n"
        "L -0x1.069adec126681p+3 0x1.f9db04769f54bp+1 -0x1.3acec475329a7p+3 0x1.d898e236a05bbp+1\n"
        "L -0x1.3acec475329a7p+3 0x1.d898e236a05bbp+1 -0x1.47d1736659b42p+3 0x1.2b4a1810e8a39p+1\n"
        "L -0x1.47d1736659b42p+3 0x1.2b4a1810e8a39p+1 -0x1.20d4e6f667a1ap+3 0x1.38ffce53f6236p+0\n"
        "L -0x1.20d4e6f667a1ap+3 0x1.38ffce53f6236p+0 -0x1.237832513d241p+3 0x1.41826c605554bp-50\n"
        "L -0x1.237832513d241p+3 0x1.41826c605554bp-50 -0x1.4ee51dabd1bb9p+3 -0x1.e241ebe4309a4p-1\n"
        "L -0x1.4ee51dabd1bb9p+3 -0x1.e241ebe4309a4p-1 -0x1.47d1736659b42p+3 -0x1.2b4a1810e8a34p+1\n"
        "L -0x1.47d1736659b42p+3 -0x1.2b4a1810e8a34p+1 -0x1.1534384ec6e4fp+3 -0x1.6846b0efeb85p+1\n"
        "L -0x1.1534384ec6e4fp+3 -0x1.6846b0efeb85p+1 -0x1.069adec126682p+3 -0x1.f9db04769f547p+1\n"
        "L -0x1.069adec126682p+3 -0x1.f9db04769f547p+1 -0x1.20a6f826b23edp+3 -0x1.58ec60b478719p+2\n"
        "L -0x1.20a6f826b23edp+3 -0x1.58ec60b478719p+2 -0x1.06e3c739b648ap+3 -0x1.a34b907e0a96dp+2\n"
        "L -0x1.06e3c739b648ap+3 -0x1.a34b907e0a96dp+2 -0x1.a5586841b165ap+2 -0x1.92d8eed2177ebp+2\n"
        "L -0x1.a5586841b165ap+2 -0x1.92d8eed2177ebp+2 -0x1.6b74e00d986f2p+2 -0x1.c7c2954832ecap+2\n"
        "L -0x1.6b74e00d986f2p+2 -0x1.c7c2954832ecap+2 -0x1.727a09220634p+2 -0x1.189fc5733a7a9p+3\n"
        "L -0x1.727a09220634p+2 -0x1.189fc5733a7a9p+3 -0x1.23c91e513442ep+2 -0x1.2ef3151412bb9p+3\n"
        "L -0x1.23c91e513442ep+2 -0x1.2ef3151412bb9p+3 -0x1.99a90301f2773p+1 -0x1.10e248fe0e64cp+3\n"
        "L -0x1.99a90301f2773p+1 -0x1.10e248fe0e64cp+3 -0x1.036ea9b91185fp+1 -0x1.1c2969a7fc16fp+3\n"
        "L -0x1.036ea9b91185fp+1 -0x1.1c2969a7fc16fp+3 -0x1.691604728922cp+0 -0x1.4d349b1ef7447p+3\n"
        "L -0x1.691604728922cp+0 -0x1.4d349b1ef7447p+3 -0x1.162d816217ba6p-49 -0x1.503fa5b28f31fp+3\n"
        "L -0x1.162d816217ba6p-49 -0x1.503fa5b28f31fp+3 0x1.a208b56b7560bp-1 -0x1.224bd04d6fdb7p+3\n"
        "L 0x1.a208b56b7560bp-1 -0x1.224bd04d6fdb7p+3 0x1.036ea9b911846p+1 -0x1.1c2969a7fc17p+3\n"
        "L 0x1.036ea9b911846p+1 -0x1.1c2969a7fc17p+3 0x1.9fa04ba5107ccp+1 -0x1.3fca9c0475ec8p+3\n"
        "L 0x1.9fa04ba5107ccp+1 -0x1.3fca9c0475ec8p+3 0x1.23c91e5134421p+2 -0x1.2ef3151412bbcp+3\n"
        "L 0x1.23c91e5134421p+2 -0x1.2ef3151412bbcp+3 0x1.2afd2fc446ffcp+2 -0x1.f46c5e29e2d54p+2\n"
        "L 0x1.2afd2fc446ffcp+2 -0x1.f46c5e29e2d54p+2 0x1.6b74e00d986efp+2 -0x1.c7c2954832ecdp+2\n"
        "L 0x1.6b74e00d986efp+2 -0x1.c7c2954832ecdp+2 0x1.d0bcdbad4a8fp+2 -0x1.e613da94d014bp+2\n"
        "L 0x1.d0bcdbad4a8fp+2 -0x1.e613da94d014bp+2 0x1.06e3c739b6489p+3 -0x1.a34b907e0a971p+2\n"
        "L 0x1.06e3c739b6489p+3 -0x1.a34b907e0a971p+2 0x1.e6815d26d1f7ap+2 -0x1.4123acc5fbb45p+2\n"
        "L 0x1.e6815d26d1f7ap+2 -0x1.4123acc5fbb45p+2 0x1.069adec126683p+3 -0x1.f9db04769f53cp+1\n"
        "L 0x1.069adec126683p+3 -0x1.f9db04769f53cp+1 0x1.3acec475329a8p+3 -0x1.d898e236a05aap+1\n"
        "L 0x1.3acec475329a8p+3 -0x1.d898e236a05aap+1 0x1.47d1736659b43p+3 -0x1.2b4a1810e8a28p+1\n"
        "L 0x1.47d1736659b43p+3 -0x1.2b4a1810e8a28p+1 0x1.20d4e6f667a1bp+3 -0x1.38ffce53f6216p+0\n"
        "L 0x1.20d4e6f667a1bp+3 -0x1.38ffce53f6216p+0 0x1.237832513d241p+3 0x0p+0\n"
        "P 0x1.8p+3 60 0x0p+0 0 0 0x0p+0 0x1.3a9b7ca47db1cp+2 0x1.2062aff027857p+2 0x0p+0 0x1.4p+2 1 0x1.8e23423a7be4cp-5 0 1 0x1.bc4d905dc1f48p-1 0x1.287eb43e4295ap-1 0 6 0x1.473a444cc6ebap+5 0x0p+0 1 0 0x0p+0 3 0x1.aabe92f900d48p+4 1 0 0x1.3de3618f80c9fp-2 0x1.49b21db97010cp+3 0x1.84425b6a7a7bfp-2 0x1.9d7645267a0ccp-1 0 0x1.08e6ea8b26e2bp+0 0 0 0x0p+0 0x0p+0 0 5 0 0 0 0 0x1.a9df078d0736p+1 0x1.1fd34b010cd7cp+4 0x1p+1\n"
        "K 1 0\n";

const char* const k_m3_9304_237 =
        "C 1\n"
        "L 0x0p+0 0x0p+0 0x1.a9098fadd0f7ep+5 0x0p+0\n"
        "L 0x1.a9098fadd0f7ep+5 0x0p+0 0x1.a9098fadd0f7ep+5 0x1.333d6adc6474cp+5\n"
        "L 0x1.a9098fadd0f7ep+5 0x1.333d6adc6474cp+5 0x0p+0 0x1.333d6adc6474cp+5\n"
        "L 0x0p+0 0x1.333d6adc6474cp+5 0x0p+0 0x0p+0\n"
        "C 1\n"
        "L 0x1.e1f6adfff98aep+3 0x1.0f590c5fbdac8p+4 0x1.ba71ea94a793fp+4 0x1.0f590c5fbdac8p+4\n"
        "L 0x1.ba71ea94a793fp+4 0x1.0f590c5fbdac8p+4 0x1.ba71ea94a793fp+4 0x1.9c5f407b0209ep+4\n"
        "L 0x1.ba71ea94a793fp+4 0x1.9c5f407b0209ep+4 0x1.e1f6adfff98aep+3 0x1.9c5f407b0209ep+4\n"
        "L 0x1.e1f6adfff98aep+3 0x1.9c5f407b0209ep+4 0x1.e1f6adfff98aep+3 0x1.0f590c5fbdac8p+4\n"
        "P 0x1.8p+3 90 0x1.ee24bb3c694f7p-6 0 0 0x0p+0 0x1.dc7698c6f8697p+1 0x1.df2671b1271bcp+2 0x0p+0 0x1.4p+2 0 0x1.5105c19b7b3bbp-5 0 1 0x1.1ddd5a1b6efebp+1 0x1.95e54c9b69103p+1 1 4 0x1.cf691dbd52eaap+4 0x0p+0 0 1 0x0p+0 1 0x1.6f87c0c13e309p+4 1 0 0x1.9213924716c26p-3 0x1.560dbfeead5edp+3 0x1.67c28e98ce1f3p-1 0x1.f303a2ced15f2p-1 0 0x1.c057e1e7b20f6p-1 0 0 0x0p+0 0x0p+0 0 7 0 1 0 0 -0x1.2bd9d99ea06bcp+5 0x1.2307cb1de66aep+5 0x1p+1\n"
        "K 1 1\n";


const char* const k_m0_9301_36 =
        "C 1\n"
        "L 0x0p+0 0x0p+0 0x1.4b82c255c4698p+4 0x0p+0\n"
        "L 0x1.4b82c255c4698p+4 0x0p+0 0x1.4b82c255c4698p+4 0x1.51c2004f07c12p+3\n"
        "L 0x1.4b82c255c4698p+4 0x1.51c2004f07c12p+3 0x0p+0 0x1.51c2004f07c12p+3\n"
        "L 0x0p+0 0x1.51c2004f07c12p+3 0x0p+0 0x0p+0\n"
        "P 0x1.8p+2 60 -0x0p+0 1 1 0x1.ab5e80d5339dap+0 0x1.e03e2c80dd41cp+1 0x1.90bf465db752fp+1 0x0p+0 0x1.4p+2 1 0x1.08a40bb4b5f5bp-4 1 1 0x1.23d2857a14abcp+0 0x1.3a02694a350c5p-2 0 7 0x1.f00717ad85cdfp+4 0x0p+0 0 0 0x0p+0 1 0x1.dc81eb013d6a6p+3 1 0 0x1.757d9e2f80eacp-2 0x1.282bcc45abc7p+3 0x1.6f354175f9fcap-1 0x1.e5126b558a49dp+1 0 0x1.f057e724780fdp-1 0 0 0x0p+0 0x0p+0 0 7 0 0 0 0 0x1.662262e0b2ab8p+5 0x1.0b601e752e18cp+4 0x1p+1\n"
        "K 0 1\n";

const char* const k_m0_9301_150 =
        "C 1\n"
        "L 0x0p+0 0x0p+0 0x1.29e2fdf1764d5p+4 0x0p+0\n"
        "L 0x1.29e2fdf1764d5p+4 0x0p+0 0x1.29e2fdf1764d5p+4 0x1.9d50a77671f3bp+4\n"
        "L 0x1.29e2fdf1764d5p+4 0x1.9d50a77671f3bp+4 0x0p+0 0x1.9d50a77671f3bp+4\n"
        "L 0x0p+0 0x1.9d50a77671f3bp+4 0x0p+0 0x0p+0\n"
        "P 0x1.8p+3 90 0x1.b70c9b3c9fe7p-7 1 0 0x1.73371d35fd47cp+0 0x1.f59d704ba81cp+1 0x1.77807a7ec2aap+1 0x1.c5057861b040fp-3 0x1.4p+2 1 0x1.cfe0ad664942ap-3 1 1 0x1.3a99b67aca49ap-2 0x1.47bde9e676babp+0 0 4 0x1.b13d6eafc13bap+5 0x0p+0 0 0 0x0p+0 2 0x1.7a8bb00fe28bfp+3 0 0 0x1.52077df6f7f28p-3 0x1.179b5d62f5fcep+1 0x1.9b98a07d84fa6p-2 0x1.24225bb38b4bfp+2 0 0x1.89e32b354e273p+0 0 0 0x0p+0 0x0p+0 2 1 0 0 0 0 -0x1.0378fe4ec6fcdp+5 0x1.62b168f5405p+4 0x1p+1\n"
        "K 0 1\n";


const char* const k_m3_9301_71 =
        "C 1\n"
        "B 0x1.3f000f9b4095ep+4 -0x1.9156c6f553a57p-5 0x1.36057db2431d8p+4 0x1.34a77d703a061p+2 0x1.2818d9ee21f05p+4 0x1.448db921fc16p+3 0x1.f74a1a0bb2c0ep+3 0x1.a65e1fea3fcb4p+3\n"
        "B 0x1.f74a1a0bb2c0ep+3 0x1.a65e1fea3fcb4p+3 0x1.9e62803b21a12p+3 0x1.0417435941c04p+4 0x1.052f2f63b161cp+3 0x1.0a872d6912948p+4 0x1.a24a114c69b3dp+1 0x1.24a888f55076p+4\n"
        "B 0x1.a24a114c69b3dp+1 0x1.24a888f55076p+4 -0x1.a05135ebe43eap+0 0x1.3ec9e4818e578p+4 -0x1.4326e2a21b65fp+3 0x1.8efbdc4c1bb41p+4 -0x1.b461e857d6fbep+3 0x1.6ff7353e932e9p+4\n"
        "B -0x1.b461e857d6fbep+3 0x1.6ff7353e932e9p+4 -0x1.12ce7706c948ep+4 0x1.50f28e310aa91p+4 -0x1.0ade4356ae582p+4 0x1.78194678189a4p+3 -0x1.1f67cef7a58b5p+4 0x1.aa327a9075541p+2\n"
        "B -0x1.1f67cef7a58b5p+4 0x1.aa327a9075541p+2 -0x1.33f15a989cbe8p+4 0x1.90c9a0c2e5cecp+0 -0x1.6485cf6b98fb2p+4 -0x1.839bb2ec2fed2p+1 -0x1.556a39f1b6b14p+4 -0x1.e42399441a884p+2\n"
        "B -0x1.556a39f1b6b14p+4 -0x1.e42399441a884p+2 -0x1.464ea477d4676p+4 -0x1.833cac890e8dp+3 -0x1.075fbd98c15e6p+4 -0x1.27277d1811bd9p+4 -0x1.89849c38afa04p+3 -0x1.48f40110e6859p+4\n"
        "B -0x1.89849c38afa04p+3 -0x1.48f40110e6859p+4 -0x1.0449bd3fdc83cp+3 -0x1.6ac08509bb4d9p+4 -0x1.9701375edb166p+0 -0x1.4fe74b9225e35p+4 0x1.d23317c615231p+1 -0x1.43d3fe2603522p+4\n"
        "B 0x1.d23317c615231p+1 -0x1.43d3fe2603522p+4 0x1.1bf9b2cee5f45p+3 -0x1.37c0b0b9e0c0fp+4 0x1.0610310b5fdfap+4 -0x1.365768fdd88fdp+4 0x1.3184787bca329p+4 -0x1.00803088171e7p+4\n"
        "B 0x1.3184787bca329p+4 -0x1.00803088171e7p+4 0x1.5cf8bfec34858p+4 -0x1.9551f024ab5a2p+3 0x1.47faa1843e0e4p+4 -0x1.3aecd88c0f54bp+2 0x1.3f000f9b4095ep+4 -0x1.9156c6f553a57p-5\n"
        "P 0x1.8p+2 90 0x0p+0 2 1 0x0p+0 0x1.2f673b6e9f8c3p+2 0x1.c9b0024ba3422p+2 0x0p+0 0x1.4p+2 1 0x1.9dbdf4cc517dfp-1 1 1 0x1.472837060311p+1 0x1.1048d66549d22p+2 0 4 0x1.529274ec2e924p+3 0x0p+0 0 0 0x0p+0 0 0x1.b70102c5e9d9bp+3 0 0 0x1.69add5d13ff6ep-2 0x1.5291022c3276dp+0 0x1.0609432dd555dp+0 0x1.7dc154866159bp+3 0 0x1.7f4db7a593195p-1 0 0 0x0p+0 0x0p+0 0 5 0 0 0 0 0x1.368c73b5a1438p+5 0x1.009728b8e0618p+5 0x1p+1\n"
        "K 0 0\n";

const char* const k_m3_9302_237 =
        "C 1\n"
        "A 0x0p+0 0x0p+0 0x1.e5d15cc0e03ecp+2 0x0p+0 0x0p+0 1\n"
        "P 0x1.8p+2 90 0x1.f71c3f275c4cfp-2 0 1 0x0p+0 0x1.262912be148dfp+2 0x1.ad1a5c464143p+2 0x0p+0 0x1.4p+2 1 0x1.09ffd61c2273cp-1 0 1 0x1.22e4144b9a613p+0 0x1.a848cfb81879dp+1 0 4 0x1.6f1cb79d54c65p+3 0x0p+0 1 0 0x0p+0 0 0x1.4943b8ea5674p+4 0 0 0x1.182e57e93b649p-2 0x1.400f4fa72b346p+3 0x1.0623ad56c2cb9p+0 0x1.c7b463a27676p+1 0 0x1.08b40c69b891dp+0 0 0 0x0p+0 0x0p+0 1 7 0 1 0 0 -0x1.322a67a1bb12p+0 0x1.5e40a74db9d18p+5 0x1p+1\n"
        "K 1 0\n";

const char* const k_m3_9302_316 =
        "C 1\n"
        "L 0x1.0404c7046df28p+4 0x0p+0 0x1.28b7a74f858ccp+4 0x1.4d9b7704cd323p+1\n"
        "L 0x1.28b7a74f858ccp+4 0x1.4d9b7704cd323p+1 0x1.1990349da461bp+4 0x1.99ec3279d43a7p+2\n"
        "L 0x1.1990349da461bp+4 0x1.99ec3279d43a7p+2 0x1.cb2a6b8317bp+3 0x1.e849148f4615cp+2\n"
        "L 0x1.cb2a6b8317bp+3 0x1.e849148f4615cp+2 0x1.8e5f27db17534p+3 0x1.4e4607402ecb6p+3\n"
        "L 0x1.8e5f27db17534p+3 0x1.4e4607402ecb6p+3 0x1.90fcde66585a2p+3 0x1.bd578f8b146aap+3\n"
        "L 0x1.90fcde66585a2p+3 0x1.bd578f8b146aap+3 0x1.2ba2270e3c751p+3 0x1.037d7a0beb47cp+4\n"
        "L 0x1.2ba2270e3c751p+3 0x1.037d7a0beb47cp+4 0x1.859e6b0406f58p+2 0x1.e22b92e46067ep+3\n"
        "L 0x1.859e6b0406f58p+2 0x1.e22b92e46067ep+3 0x1.6936d17faa5b8p+1 0x1.00118245731e7p+4\n"
        "L 0x1.6936d17faa5b8p+1 0x1.00118245731e7p+4 0x1.4ea0321f234f5p-1 0x1.2b736cdce1615p+4\n"
        "L 0x1.4ea0321f234f5p-1 0x1.2b736cdce1615p+4 -0x1.a03f11a11999cp+1 0x1.2714d051a4d64p+4\n"
        "L -0x1.a03f11a11999cp+1 0x1.2714d051a4d64p+4 -0x1.416696fc513c6p+2 0x1.ee95bb0a472b5p+3\n"
        "L -0x1.416696fc513c6p+2 0x1.ee95bb0a472b5p+3 -0x1.0404c7046df26p+3 0x1.c25d939399abap+3\n"
        "L -0x1.0404c7046df26p+3 0x1.c25d939399abap+3 -0x1.70f20bcd81042p+3 0x1.d83a8bd1e87eap+3\n"
        "L -0x1.70f20bcd81042p+3 0x1.d83a8bd1e87eap+3 -0x1.cb10a4d3025cdp+3 0x1.813387665f8f6p+3\n"
        "L -0x1.cb10a4d3025cdp+3 0x1.813387665f8f6p+3 -0x1.b90443f86ea16p+3 0x1.1393ea6de47dfp+3\n"
        "L -0x1.b90443f86ea16p+3 0x1.1393ea6de47dfp+3 -0x1.e8acdc3b01ea1p+3 0x1.63b9fa956ee34p+2\n"
        "L -0x1.e8acdc3b01ea1p+3 0x1.63b9fa956ee34p+2 -0x1.2515f06823acfp+4 0x1.f260d8d5085f5p+1\n"
        "L -0x1.2515f06823acfp+4 0x1.f260d8d5085f5p+1 -0x1.2ba2270e3c75p+4 0x1.4a83c73202d3fp-49\n"
        "L -0x1.2ba2270e3c75p+4 0x1.4a83c73202d3fp-49 -0x1.017cf93c1a32p+4 -0x1.2180363ed8013p+1\n"
        "L -0x1.017cf93c1a32p+4 -0x1.2180363ed8013p+1 -0x1.e8acdc3b01ea2p+3 -0x1.63b9fa956ee3p+2\n"
        "L -0x1.e8acdc3b01ea2p+3 -0x1.63b9fa956ee3p+2 -0x1.088f834451236p+4 -0x1.1956b6f265b57p+3\n"
        "L -0x1.088f834451236p+4 -0x1.1956b6f265b57p+3 -0x1.cb10a4d3025cep+3 -0x1.813387665f8f4p+3\n"
        "L -0x1.cb10a4d3025cep+3 -0x1.813387665f8f4p+3 -0x1.5bf90f4b4d924p+3 -0x1.82768a2f52846p+3\n"
        "L -0x1.5bf90f4b4d924p+3 -0x1.82768a2f52846p+3 -0x1.0404c7046df2cp+3 -0x1.c25d939399ab7p+3\n"
        "L -0x1.0404c7046df2cp+3 -0x1.c25d939399ab7p+3 -0x1.c0fa85a3142b2p+2 -0x1.15d0b4c98de58p+4\n"
        "L -0x1.c0fa85a3142b2p+2 -0x1.15d0b4c98de58p+4 -0x1.a03f11a1199c2p+1 -0x1.2714d051a4d63p+4\n"
        "L -0x1.a03f11a1199c2p+1 -0x1.2714d051a4d63p+4 -0x1.226278aa90ed6p-1 -0x1.03dc3a5ac3c45p+4\n"
        "L -0x1.226278aa90ed6p-1 -0x1.03dc3a5ac3c45p+4 0x1.6936d17faa5a8p+1 -0x1.00118245731e8p+4\n"
        "L 0x1.6936d17faa5a8p+1 -0x1.00118245731e8p+4 0x1.725e04d3ddff5p+2 -0x1.1cf7e2e02b415p+4\n"
        "L 0x1.725e04d3ddff5p+2 -0x1.1cf7e2e02b415p+4 0x1.2ba2270e3c749p+3 -0x1.037d7a0beb47fp+4\n"
        "L 0x1.2ba2270e3c749p+3 -0x1.037d7a0beb47fp+4 0x1.402abcf630e91p+3 -0x1.99cb8554aa67ap+3\n"
        "L 0x1.402abcf630e91p+3 -0x1.99cb8554aa67ap+3 0x1.8e5f27db17532p+3 -0x1.4e4607402ecb9p+3\n"
        "L 0x1.8e5f27db17532p+3 -0x1.4e4607402ecb9p+3 0x1.fc350366b011ep+3 -0x1.3d9022c75d0d5p+3\n"
        "L 0x1.fc350366b011ep+3 -0x1.3d9022c75d0d5p+3 0x1.1990349da4619p+4 -0x1.99ec3279d43b6p+2\n"
        "L 0x1.1990349da4619p+4 -0x1.99ec3279d43b6p+2 0x1.fcac5ac976304p+3 -0x1.b07cc36bd29c3p+1\n"
        "L 0x1.fcac5ac976304p+3 -0x1.b07cc36bd29c3p+1 0x1.0404c7046df28p+4 0x0p+0\n"
        "P 0x1.8p+2 120 0x1.1fe1d0d6788ebp-2 1 0 0x0p+0 0x1.60968ac51127cp+2 0x1.74677e8fa483p+1 0x0p+0 0x1.4p+2 0 0x1.1ef61cdab4f59p-1 0 1 0x1.7f8bf3e272fc6p+0 0x1.290570c38d21cp+2 1 7 0x1.a2693bc5a2754p+5 0x0p+0 0 0 0x0p+0 1 0x1.0876fe74b6fd5p+0 0 0 0x1.9d6ea1ba1af4bp-2 0x1.7565d94ed35d7p+3 0x1.db0aa9cc8bc8dp-4 0x1.1de38d95ae99ep+3 0 0x1.463543ee1a411p+0 0 0 0x0p+0 0x0p+0 0 6 0 0 0 0 0x1.6a58132d07576p+5 -0x1.73fea96c5abe9p+5 0x1p+1\n"
        "K 1 0\n";


const char* const k_m0_9301_364 =
        "C 1\n"
        "L 0x1.6b6bebdb06f9p+2 -0x1.28eefe18e22c7p+3 0x1.d25cb448adb44p+2 0x1.027c61af2c7d1p+3\n"
        "A 0x1.9bc321b69aedp-1 0x1.15b5afe40754cp+3 0x1.a0ab642eef023p+2 -0x1.7a7dfa9ac4b6ep-4 0x1.864bc56f6cabdp+1 0\n"
        "L -0x1.6b6bebdb06f9p+2 0x1.28eefe18e22c7p+3 -0x1.d25cb448adb44p+2 -0x1.027c61af2c7d1p+3\n"
        "A -0x1.9bc321b69aedp-1 -0x1.15b5afe40754cp+3 0x1.a0ab642eef023p+2 0x1.864bc56f6cabdp+1 -0x1.7a7dfa9ac4b6ep-4 0\n"
        "P 0x1.9p+4 120 -0x0p+0 0 0 0x1.6d499e8b0ad6ap+1 0x1.235727cc7c69ep+2 0x1.33e9f8c423b35p+0 0x0p+0 0x1.4p+2 1 0x1.7052eec40772ap-3 0 1 0x1.ac30936e10158p-2 0x1.5b041b9e44d3dp-1 0 4 0x1.14b78c2ad6e6ap+5 0x0p+0 0 0 0x0p+0 3 0x1.35cb7e0057b8p+4 0 0 0x1.537ed61db08dfp-3 0x1.ad98f84d8ec55p+1 0x1.8bfca31ac11e9p+0 0x1.26f6e85830c4ep+3 0 0x1.4bc008314d52bp+0 0 0 0x0p+0 0x0p+0 0 5 0 0 0 0 0x1.888b88c6555bp+1 -0x1.2d5afb1c7eceep+5 0x1p+1\n"
        "K 0 1\n";

const char* const k_m0_9301_392 =
        "C 1\n"
        "L 0x0p+0 0x0p+0 0x1.81b2df35288b1p+4 0x0p+0\n"
        "L 0x1.81b2df35288b1p+4 0x0p+0 0x1.81b2df35288b1p+4 0x1.a83d541dfc455p+4\n"
        "L 0x1.81b2df35288b1p+4 0x1.a83d541dfc455p+4 0x0p+0 0x1.a83d541dfc455p+4\n"
        "L 0x0p+0 0x1.a83d541dfc455p+4 0x0p+0 0x0p+0\n"
        "P 0x1.8p+2 60 0x0p+0 0 0 0x1.03cc473d9da31p+0 0x1.ce3ec8c994912p+1 0x1.14efbe2c9bbdap+2 0x1.5536758ca64b8p-1 0x1.4p+2 1 0x1.ec0e06377431fp-2 0 1 0x1.20073bdc83e9p-2 0x1.56ed04fb28921p+1 0 4 0x1.b39c87922d5a6p+4 0x0p+0 0 0 0x0p+0 2 0x1.1d1a688c07924p+5 0 0 0x1.16889f3ea48cap-2 0x1.46a7bdedf4022p+0 0x1.b4d3b6d31508bp-2 0x1.8df08039aa003p+2 0 0x1.5e3edb5bb3c44p+0 0 0 0x0p+0 0x0p+0 2 1 0 0 0 0 0x1.560e95a71ef18p+3 -0x1.57bf3607b85e3p+5 0x1p+1\n"
        "K 1 1\n";

const char* const k_m0_9303_213 =
        "C 1\n"
        "A 0x1.b84005fc89ab3p+3 0x1.6cb711ee225cep+2 0x1.a23b872f67ac9p+2 -0x1.0f31c54bf43e8p+0 0x1.d8419fee15a74p+0 0\n"
        "A 0x1.66d4e465581ddp+2 0x1.b125e7e6d463cp+3 0x1.a454c7cc64e71p+2 -0x1.e6a96f8162541p-3 0x1.4c025eeb48426p+1 0\n"
        "A -0x1.14cb6de0727ddp+1 0x1.4e1ee5c2edb1p+2 0x1.7ddc6db41d59bp+3 0x1.6379ac3ce6dfbp+0 0x1.44eacc76e0161p+1 0\n"
        "A -0x1.7b9ce1a76dbdp+3 0x1.3a7b4738b5388p+2 0x1.c4f1436463c79p+2 0x1.96b770bd4ee76p+0 -0x1.2fe3a5afb8282p+1 0\n"
        "A -0x1.b2f0fbb408c23p+3 -0x1.6851344d08525p+2 0x1.a3bfc5ea51894p+2 0x1.0e01c4e948419p+1 -0x1.52f3af306f1a9p+0 0\n"
        "A -0x1.d807751ede6cp-2 -0x1.1ce5107580aap+0 0x1.fb33ad6abd888p+3 -0x1.3150e78e0a6b9p+1 -0x1.8aad760e9234cp+0 0\n"
        "A 0x1.ef1006fdb88a9p+1 -0x1.2acbf5808adfcp+3 0x1.116b1664d3769p+3 -0x1.0530838e0e171p+1 -0x1.433a232920311p-2 0\n"
        "A 0x1.427b8a282b1cp+3 -0x1.0b273f94dc41dp+2 0x1.0179edbe3ca14p+3 -0x1.54b1a3d6e7c9dp+0 0x1.174392698cc2p-1 0\n"
        "C 1\n"
        "L 0x1.48a2f8fb1093p+4 -0x1.63e74a67b067p+2 0x1.886ac2c304cafp+4 -0x1.63e74a67b067p+2\n"
        "L 0x1.886ac2c304cafp+4 -0x1.63e74a67b067p+2 0x1.886ac2c304cafp+4 -0x1.7d0b39175d256p+0\n"
        "L 0x1.886ac2c304cafp+4 -0x1.7d0b39175d256p+0 0x1.caf5370bfd21ep+4 -0x1.7d0b39175d256p+0\n"
        "L 0x1.caf5370bfd21ep+4 -0x1.7d0b39175d256p+0 0x1.caf5370bfd21ep+4 -0x1.63e74a67b067p+2\n"
        "L 0x1.caf5370bfd21ep+4 -0x1.63e74a67b067p+2 0x1.055e8069f8acep+5 -0x1.63e74a67b067p+2\n"
        "L 0x1.055e8069f8acep+5 -0x1.63e74a67b067p+2 0x1.055e8069f8acep+5 0x1.a480d4fbd2b3ep+2\n"
        "L 0x1.055e8069f8acep+5 0x1.a480d4fbd2b3ep+2 0x1.caf5370bfd21ep+4 0x1.a480d4fbd2b3ep+2\n"
        "L 0x1.caf5370bfd21ep+4 0x1.a480d4fbd2b3ep+2 0x1.caf5370bfd21ep+4 0x1.3fb8b1b3f32c7p+1\n"
        "L 0x1.caf5370bfd21ep+4 0x1.3fb8b1b3f32c7p+1 0x1.886ac2c304cafp+4 0x1.3fb8b1b3f32c7p+1\n"
        "L 0x1.886ac2c304cafp+4 0x1.3fb8b1b3f32c7p+1 0x1.886ac2c304cafp+4 0x1.a480d4fbd2b3ep+2\n"
        "L 0x1.886ac2c304cafp+4 0x1.a480d4fbd2b3ep+2 0x1.48a2f8fb1093p+4 0x1.a480d4fbd2b3ep+2\n"
        "L 0x1.48a2f8fb1093p+4 0x1.a480d4fbd2b3ep+2 0x1.48a2f8fb1093p+4 -0x1.63e74a67b067p+2\n"
        "P 0x1.8p+3 60 0x0p+0 0 0 0x0p+0 0x1.2272e23364902p+2 0x1.475e78add323ep+2 0x0p+0 0x1.4p+2 1 0x1.77c1d8fb1899cp-1 0 1 0x1.046df7c664512p+0 0x1.7138b633f50a3p+1 0 8 0x1.fca6e06c68f1cp+4 0x0p+0 0 0 0x0p+0 1 0x1.2e61a404b4919p+5 0 0 0x1.bfdbb35491a1bp-2 0x1.1ad28fa2292dcp+2 0x1.8612de95f8c3cp-2 0x1.571ebb7aed82p+2 0 0x1.2b3580197d5b8p-2 0 0 0x0p+0 0x0p+0 0 5 0 0 0 0 0x1.8156997f6936ap+5 0x1.a11dbe2ca1a28p+4 0x1p+1\n"
        "K 1 0\n";

const char* const k_m3_9303_305 =
        "C 1\n"
        "A -0x1.98f730c3fcp-6 -0x1.98f730c3fcp-6 0x1.0e745679a2b12p+5 0x1.831bc32f20bb7p-11 0x1.91ef51cbdced7p+0 0\n"
        "A -0x1.521add6b825f8p+4 0x1.521add6b825f9p+4 0x1.8a0f1b0940cccp+4 0x1.142f142f09a52p-1 -0x1.0e1b9fade3d21p+1 1\n"
        "A 0x1.7fb8d87be1b44p+2 0x1.7fb8d87be1b4p+2 0x1.41d0854cbbddep+5 -0x1.7ef97e75881bdp+1 -0x1.b86c22e1b83d1p+0 0\n"
        "A 0x1.cfdab251d0d36p+4 -0x1.cfdab251d0d38p+4 0x1.d6253fe881b82p+4 -0x1.7d29137e8ecdfp+1 0x1.683271b8daca4p+0 1\n"
        "P 0x1.8p+3 120 0x1.2e8b1f2edfc3p-1 0 1 0x0p+0 0x1.26a90a169d3dep+1 0x1.3db2472eb0c6ap+2 0x0p+0 0x1.4p+2 0 0x1.cd71c5bb1980dp-1 0 1 0x1.af2bc624cbd7ap-1 0x1.c08258e45ad93p+0 0 3 0x1.d1e23174a6864p+5 0x0p+0 0 0 0x0p+0 3 0x1.6a4899b449626p+4 1 0 0x1.93c7a3fb436b2p-3 0x1.b774af886280ep+3 0x1.6bb5ed00f04a9p+0 0x1.323f82db7767cp+1 0 0x1.b9391fd25701ap-2 0 0 0x0p+0 0x0p+0 0 6 0 1 0 0 -0x1.75963f9e1d44cp+3 -0x1.10efc68a7ce07p+5 0x1p+1\n"
        "K 1 0\n";

const char* const k_m3_9302_30 =
        "C 1\n"
        "L 0x0p+0 0x0p+0 0x1.2d382df3f68c5p+6 0x0p+0\n"
        "L 0x1.2d382df3f68c5p+6 0x0p+0 0x1.2d382df3f68c5p+6 0x1.d22971e4b9a66p+5\n"
        "L 0x1.2d382df3f68c5p+6 0x1.d22971e4b9a66p+5 0x0p+0 0x1.d22971e4b9a66p+5\n"
        "L 0x0p+0 0x1.d22971e4b9a66p+5 0x0p+0 0x0p+0\n"
        "C 1\n"
        "L 0x1.bf13038c109e9p+5 0x1.1dcf475c7016cp+5 0x1.1aec754d3121cp+6 0x1.1dcf475c7016cp+5\n"
        "L 0x1.1aec754d3121cp+6 0x1.1dcf475c7016cp+5 0x1.1aec754d3121cp+6 0x1.70f36919a93d6p+5\n"
        "L 0x1.1aec754d3121cp+6 0x1.70f36919a93d6p+5 0x1.bf13038c109e9p+5 0x1.70f36919a93d6p+5\n"
        "L 0x1.bf13038c109e9p+5 0x1.70f36919a93d6p+5 0x1.bf13038c109e9p+5 0x1.1dcf475c7016cp+5\n"
        "C 1\n"
        "A 0x1.762f5af760163p+4 0x1.6bb7a49f5ed42p+5 0x1.6732e7423bd4p+2 0x0p+0 0x0p+0 0\n"
        "P 0x1.9p+4 60 0x1.c00ccdb30bda6p-1 0 0 0x0p+0 0x1.595be3c9c3184p+2 0x1.1f7f9faaaffbcp+2 0x1.589fb7a479bep-2 0x1.4p+2 0 0x1.0ae81900f8f47p-1 1 1 0x1.de84bc9bd09b5p+2 0x1.f5fa1d680e372p+0 0 1 0x1.53bf477f7bab1p+3 0x0p+0 1 0 0x0p+0 2 0x1.04e1e91c1db94p+4 0 0 0x1.688841d7bb561p-2 0x1.bb43aa6e20578p+3 0x1.886e4078a84dep+0 0x1.d441abe4190acp+2 0 0x1.7e7239932c822p+0 0 0 0x0p+0 0x0p+0 1 5 0 1 0 0 0x1.04fa0293df0d4p+4 -0x1.f4e58f2f2c1d1p+4 0x1p+1\n"
        "K 1 1\n";

const char* const k_m0_9303_322 =
        "C 1\n"
        "L 0x1.f41bca0b68e3fp+1 0x0p+0 0x1.fec30888a921cp+2 0x0p+0\n"
        "L 0x1.fec30888a921cp+2 0x0p+0 0x1.fec30888a921cp+2 0x1.f41bca0b68e3fp+2\n"
        "L 0x1.fec30888a921cp+2 0x1.f41bca0b68e3fp+2 0x1.7c6876c72ec9ep+3 0x1.f41bca0b68e3fp+2\n"
        "L 0x1.7c6876c72ec9ep+3 0x1.f41bca0b68e3fp+2 0x1.7c6876c72ec9ep+3 0x1.7c6876c72ec9ep+3\n"
        "L 0x1.7c6876c72ec9ep+3 0x1.7c6876c72ec9ep+3 0x0p+0 0x1.7c6876c72ec9ep+3\n"
        "L 0x0p+0 0x1.7c6876c72ec9ep+3 0x0p+0 0x1.f41bca0b68e3fp+2\n"
        "L 0x0p+0 0x1.f41bca0b68e3fp+2 0x1.f41bca0b68e3fp+1 0x1.f41bca0b68e3fp+2\n"
        "L 0x1.f41bca0b68e3fp+1 0x1.f41bca0b68e3fp+2 0x1.f41bca0b68e3fp+1 0x0p+0\n"
        "P 0x1.8p+2 60 -0x0p+0 1 1 0x0p+0 0x1.8f1ccdd1e7c69p+1 0x1.5f6561c23738cp+0 0x0p+0 0x1.4p+2 1 0x1.0628c9c8d2a0fp-4 0 1 0x1.dcd0372044005p-3 0x1.a010dd6eaa889p-3 0 4 0x1.338d05fccff76p+4 0x0p+0 0 0 0x0p+0 1 0x1.21f57c27539cdp+4 0 0 0x1.4234b7903618ep-2 0x1.ea39ce3512339p+2 0x1.2e0baee6b8a8dp-2 0x1.4fbb1f884321bp+3 0 0x1.7da536038f5efp-1 0 0 0x0p+0 0x0p+0 1 5 0 1 0 0 0x1.89984df030c58p+4 0x1.77003b266cb24p+3 0x1p+1\n"
        "K 1 0\n";


const char* const k_m0_9302_71 =
        "C 1\n"
        "L 0x0p+0 0x0p+0 0x1.2db7df4259f2cp+5 0x0p+0\n"
        "L 0x1.2db7df4259f2cp+5 0x0p+0 0x1.2db7df4259f2cp+5 0x1.2afabb2b71ecbp+3\n"
        "L 0x1.2db7df4259f2cp+5 0x1.2afabb2b71ecbp+3 0x1.2afabb2b71ecbp+3 0x1.2afabb2b71ecbp+3\n"
        "L 0x1.2afabb2b71ecbp+3 0x1.2afabb2b71ecbp+3 0x1.2afabb2b71ecbp+3 0x1.a667d229b120ap+5\n"
        "L 0x1.2afabb2b71ecbp+3 0x1.a667d229b120ap+5 0x0p+0 0x1.a667d229b120ap+5\n"
        "L 0x0p+0 0x1.a667d229b120ap+5 0x0p+0 0x0p+0\n"
        "P 0x1.9p+4 120 0x0p+0 0 0 0x0p+0 0x1.62b407341cb37p+2 0x1.b119b64dfa9a4p+2 0x1.7cbe4512ba89cp-1 0x1.4p+2 1 0x1.d2b98117446a4p-5 0 1 0x1.ecee18bbd3d64p-1 0x1.5fd3c40540a4p-1 0 4 0x1.bebbf3a7ee67p+5 0x0p+0 0 0 0x0p+0 0 0x1.55821ab14081ep+1 0 0 0x1.6cf09408167bp-3 0x1.a5ad730f52b52p+1 0x1.92717c14f983ep-1 0x1.5323506781db9p+2 0 0x1.2ac2e44f62a2ep-3 0 0 0x0p+0 0x0p+0 2 1 0 0 0 0 0x1.97f24f10e73b4p+4 0x1.a796d882db3p+4 0x1p+1\n"
        "K 1 0\n";

struct Scene {
    std::vector<Contour> cs;
    ProfileParams p;
    tooling::Tool tool;
    double d{0.0};
    int vbitDeg{0};
};

// Leads off (the loose-piece search does not see leads), then the start mode of the T-0158 repros.
enum class Start { AsStored, VectorStartOutside, BoxCentre };

Scene loadScene(const std::string& text, Start start = Start::AsStored) {
    Scene s;
    s.d = readScene(text, s.cs, s.p, s.vbitDeg);
    s.tool = test::endMill(s.d);
    if (s.vbitDeg > 0) {
        s.tool.type = tooling::ToolType::VBit;
        s.tool.includedAngle = s.vbitDeg * geometry::kPi / 180.0;
    }
    s.p.leads.type = LeadType::None;
    s.p.leads.overcut = 0.0;
    s.p.leads.leadOut = false;
    s.p.ramp.onLeadIn = false;
    if (start == Start::VectorStartOutside) {
        s.p.startAt = StartAt::VectorStart;
        s.p.side = ProfileSide::Outside;
    } else if (start == Start::BoxCentre) {
        s.p.startAt = StartAt::ClosestOnBoundingBox;
        s.p.startAnchor = BoxAnchor::Centre;
    }
    return s;
}

// Tabs left out in all (the "N of M tab(s) left out" diagnostics).
std::size_t leftOut(const ProfileResult& r) {
    std::size_t n = 0;
    for (const ProfileDiagnostic& g : r.diagnostics) {
        const auto at = g.message.find(" of ");
        if (g.message.find("tab(s) left out") == std::string::npos || at == std::string::npos) continue;
        const auto from = g.message.rfind(' ', at - 1);
        n += static_cast<std::size_t>(std::stoul(g.message.substr(from == std::string::npos ? 0 : from + 1, at - (from == std::string::npos ? 0 : from + 1))));
    }
    return n;
}

std::vector<double> reported(const ProfileResult& r) {
    std::vector<double> out;
    for (const ProfileDiagnostic& g : r.diagnostics) {
        if (g.message.find("comes free") == std::string::npos) continue;
        out.push_back(std::stod(g.message.substr(g.message.find("(about ") + 7)));
    }
    std::sort(out.begin(), out.end());
    return out;
}

// The flood's free pieces of `minArea` mm2 or more (every tab present).
std::vector<double> flooded(const Scene& s, const ProfileResult& r, double minArea) {
    const double zTab = s.p.passes.bottomZ() + std::min(s.p.tabs.thickness, s.p.passes.cutDepth);
    const double rMax = 0.5 * s.d;
    const double tanHalf = s.vbitDeg > 0 ? std::tan(0.5 * s.vbitDeg * geometry::kPi / 180.0) : 0.0;
    const std::vector<double> all =
        s.vbitDeg > 0 ? floodPieces(r.toolpath, zTab, rMax, 0.04, [=](double z) { return std::min(rMax, (zTab - z) * tanHalf); })
                      : floodPieces(r.toolpath, zTab, rMax, 0.04, [rMax](double) { return rMax; });
    std::vector<double> big;
    for (double a : all) {
        if (a >= minArea) big.push_back(a);
    }
    return big;
}

// True when a raised stretch of a tab (a cutting move on the tab top) has an end within `near` of p.
bool tabNear(const Scene& s, const ProfileResult& r, Vec2 p, double near) {
    const double zTab = s.p.passes.bottomZ() + std::min(s.p.tabs.thickness, s.p.passes.cutDepth);
    const auto& mv = r.toolpath.moves;
    for (std::size_t i = 1; i < mv.size(); ++i) {
        if (!isCutting(mv[i].kind) || std::fabs(mv[i].to.z - zTab) > 1e-7 || std::fabs(mv[i - 1].to.z - zTab) > 1e-7) continue;
        if (geometry::distance(Vec2{mv[i].to.x, mv[i].to.y}, p) < near) return true;
    }
    return false;
}

}  // namespace


TEST_CASE("T-0167: a V-bit's tabs are judged by the cone at the tab layer (Inside/Outside, allowance >= 0)",
          "[toolpath][profile][tabs][vbit]") {
    // m2_1330000_9 (V-bit 90, D25, Outside, allowance 0, separate last pass, 3D tab 1.99 mm thick):
    // the radius at the top of the cut (6.5 mm) was used at the tab layer, where the cone is 1.99 mm
    // wide, so both vectors' single tabs were left out as cut through; they hold, and the parts
    // (641 and 1,704 mm2) no longer come free. The same at the stored start and at VectorStart.
    for (Start st : {Start::AsStored, Start::VectorStartOutside}) {
        const Scene s = loadScene(k_m2_1330000_9, st);
        const ProfileResult r = profileToolpath(s.cs, s.tool, s.p);
        REQUIRE(r.ok());
        CHECK(leftOut(r) == 0);
        CHECK(reported(r).empty());
        CHECK(flooded(s, r, 0.1).empty());
    }
}

TEST_CASE("T-0167: a tab that only joins a closed tongue of waste is left out", "[toolpath][profile][tabs][vbit]") {
    // m3_9304_126 (a notch 16.9 mm wide in an H-shaped part, Outside, V-bit 120, allowance 0.86):
    // at the tab layer the two legs of the cut round the notch's end leave no waste between them;
    // the tab on the end of the notch joins the part only to that sealed tongue. Two of the three
    // tabs hold, the third is left out, and nothing comes free (before T-0167 all three were
    // left out and the 1,586 mm2 part came free; the first version of the fix kept the sealed
    // tab too).
    const Scene s = loadScene(k_m3_9304_126);
    const ProfileResult r = profileToolpath(s.cs, s.tool, s.p);
    REQUIRE(r.ok());
    CHECK(leftOut(r) == 1);
    CHECK(reported(r).empty());
    CHECK(flooded(s, r, 0.1).empty());
    CHECK_FALSE(tabNear(s, r, {1.39, -7.95}, 3.0));
}

TEST_CASE("T-0167: tabs the cone's wider tab layer would nick stay as before", "[toolpath][profile][tabs][vbit]") {
    // Each group pins one rule of the judgement; tabs left out are counted, nothing free.
    // - The raster's core is the stretch less the tab-layer radius at each end (m0_9306_257,
    //   m3_9305_215, m3_9304_237: trimmed by the full radius, tabs that hold were left out and a
    //   piece came free).
    // - The waste to reach lies a tab-layer radius beyond the cut (m0_9301_36, m0_9301_150: with a
    //   full radius the only tab of a 58 mm2 part goes).
    // - The tab-layer radius is capped at the tool's radius (m3_9301_71, m3_9302_237, m3_9302_316:
    //   uncapped, the cone at the tab layer is wider than the tool and a part of 300 to 1,100 mm2
    //   loses all its tabs).
    // - The raster reaches a tool radius + allowance to the vector side and the tab-layer radius beyond
    //   the kerf, and its core is not the stretch less the full radius (m0_9301_364, m0_9301_392,
    //   m0_9303_213, m3_9303_305: tabs that hold were left out and a 758 mm2 part came free;
    //   m3_9302_30, m0_9303_322: the window of the full radius loses a tab; m0_9302_71: without the
    //   two tab-layer radii of the window's far side a tab is lost).
    // - The other cuts are tested against the core trimmed by the full radius at each end, as before
    //   (m3_9302_251, m3_9303_94, m3_9304_67, m3_9301_367, m3_9301_272, m3_9301_441: measured against
    //   the longer core, tabs that hold were lost or one that is cut through was kept).
    struct Case {
        const char* name;
        const char* text;
        std::size_t left;
    };
    const Case cases[] = {{"m3_9302_251", k_m3_9302_251, 5}, {"m3_9303_94", k_m3_9303_94, 9},
                          {"m3_9304_67", k_m3_9304_67, 4},  {"m3_9301_367", k_m3_9301_367, 0},
                          {"m3_9301_272", k_m3_9301_272, 2}, {"m3_9301_441", k_m3_9301_441, 1},
                          {"m0_9306_257", k_m0_9306_257, 0}, {"m3_9305_215", k_m3_9305_215, 0},
                          {"m3_9304_237", k_m3_9304_237, 3}, {"m0_9301_36", k_m0_9301_36, 6},
                          {"m0_9301_150", k_m0_9301_150, 3}, {"m3_9301_71", k_m3_9301_71, 0},
                          {"m3_9302_237", k_m3_9302_237, 0}, {"m3_9302_316", k_m3_9302_316, 1},
                          {"m0_9301_364", k_m0_9301_364, 0},
                          {"m0_9301_392", k_m0_9301_392, 0},
                          {"m0_9303_213", k_m0_9303_213, 0},
                          {"m3_9303_305", k_m3_9303_305, 0},
                          {"m3_9302_30", k_m3_9302_30, 1},
                          {"m0_9303_322", k_m0_9303_322, 0},
                          {"m0_9302_71", k_m0_9302_71, 0}};
    for (const Case& k : cases) {
        INFO(k.name);
        const Scene s = loadScene(k.text);
        const ProfileResult r = profileToolpath(s.cs, s.tool, s.p);
        REQUIRE(r.ok());
        CHECK(leftOut(r) == k.left);
        CHECK(reported(r).size() == flooded(s, r, 0.1).size());
    }
}

TEST_CASE("T-0167: the T-0158 repros that stay: pieces are reported (leads off)", "[toolpath][profile][tabs]") {
    // m1_1310133_29 at VectorStart/Outside (V-bit 60, negative allowance, separate last pass): the
    // tab at (8.85, 8.60) joins the outside, the slug and the part but not a 0.08 mm thick sliver
    // of 0.41 mm2 between the finishing and roughing kerfs (the exact check widens the tools, so a
    // neck under 0.11 mm is not resolved); the sliver and a 0.9 mm2 one are reported, as the flood
    // finds them. Dropping the tab would free the part.
    {
        const Scene s = loadScene(k_m1_1310133_29, Start::VectorStartOutside);
        const ProfileResult r = profileToolpath(s.cs, s.tool, s.p);
        REQUIRE(r.ok());
        const std::vector<double> rep = reported(r);
        const std::vector<double> fl = flooded(s, r, 0.3);
        // (Counts only: the flood's 0.04 mm cells measure slivers 0.08 mm thick to within a quarter.)
        CHECK(rep.size() == 2);
        CHECK(fl.size() == 2);
    }
    // m0_271000_0 at VectorStart/Outside (V-bit 90, D6, allowance 0.56, separate last pass): with
    // leads off nothing comes free and the tab that main left out is kept (with leads on a lead
    // cuts a 1.2 mm2 piece off outside a corner, a lead gap of ADR-0016 section 14).
    {
        const Scene s = loadScene(k_m0_271000_0, Start::VectorStartOutside);
        const ProfileResult r = profileToolpath(s.cs, s.tool, s.p);
        REQUIRE(r.ok());
        CHECK(leftOut(r) == 0);
        CHECK(reported(r).empty());
        CHECK(flooded(s, r, 0.1).empty());
    }
}

