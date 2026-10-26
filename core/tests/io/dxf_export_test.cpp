#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "carvetto/io/dxf.hpp"
#include "dxf_test_util.hpp"

using namespace carvetto::geometry;
using namespace carvetto::io;
using namespace dxftest;

namespace {

constexpr double kRoundTrip = 1e-9;  // AC2

struct Doc {
    std::vector<DxfLayer> layers;
    std::vector<DxfContour> contours;
};

/// Random document: 5 layers, chains of lines / arcs / Beziers (open and closed), standalone
/// full circles with random start angle and direction.
Doc randomDocument(std::uint32_t seed, std::size_t count) {
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> coord(-500.0, 500.0);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    Doc doc;
    const std::vector<std::string> names{"Profile", "Pocket 6mm", "Engrave-V90",
                                         "\xC3\x9C"
                                         "bersicht \xE2\x9C\x93",
                                         "0"};
    for (std::size_t i = 0; i < names.size(); ++i) {
        DxfLayer l;
        l.name = names[i];
        l.aci = 0;
        l.rgb = static_cast<std::uint32_t>(rng() & 0xFFFFFFu);
        l.visible = i != 2;
        doc.layers.push_back(l);
    }
    doc.layers[4].aci = 3;  // a plain index colour
    doc.layers[4].rgb = aciToRgb(3);
    for (std::size_t n = 0; n < count; ++n) {
        DxfContour dc;
        dc.layer = rng() % doc.layers.size();
        Contour& c = dc.contour;
        if (rng() % 10 == 0) {
            ArcSpan a = ArcSpan::fullCircle({coord(rng), coord(rng)}, 1.0 + 99.0 * unit(rng),
                                            rng() % 2 ? ArcDirection::CW : ArcDirection::CCW);
            a.startAngle = a.endAngle = (rng() % 3 == 0) ? 0.0 : unit(rng) * kTwoPi;
            c.spans.emplace_back(a);
            c.closed = rng() % 4 != 0;
            doc.contours.push_back(dc);
            continue;
        }
        Vec2 p{coord(rng), coord(rng)};
        const Vec2 first = p;
        const std::size_t spans = 1 + rng() % 8;
        for (std::size_t k = 0; k < spans; ++k) {
            switch (rng() % 3) {
                case 0: {
                    const Vec2 q = p + Vec2{coord(rng), coord(rng)} * 0.2;
                    c.spans.emplace_back(LineSpan{p, q});
                    break;
                }
                case 1: {
                    const double r = 1.0 + 99.0 * unit(rng);
                    const double a0 = unit(rng) * kTwoPi;
                    const double sweep = (0.1 + unit(rng) * 1.8 * kPi) * (rng() % 2 ? 1.0 : -1.0);
                    ArcSpan a;
                    a.centre = p - Vec2::fromAngle(a0, r);
                    a.radius = r;
                    a.startAngle = a0;
                    a.endAngle = normalizeAngle(a0 + sweep);
                    a.direction = sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW;
                    c.spans.emplace_back(a);
                    break;
                }
                default: {
                    auto jitter = [&] { return Vec2{coord(rng), coord(rng)} * 0.1; };
                    const Vec2 q = p + jitter();
                    c.spans.emplace_back(BezierSpan{p, p + jitter(), q + jitter(), q});
                    break;
                }
            }
            p = endPoint(c.spans.back());
        }
        if (rng() % 3 == 0) {
            c.spans.emplace_back(LineSpan{p, first});
            c.closed = true;
        }
        doc.contours.push_back(dc);
    }
    return doc;
}

void requireSameContour(const Contour& a, const Contour& b, std::size_t index) {
    INFO("contour " << index);
    REQUIRE(a.closed == b.closed);
    REQUIRE(a.spans.size() == b.spans.size());
    auto near = [](Vec2 p, Vec2 q) { return distance(p, q) <= kRoundTrip; };
    for (std::size_t i = 0; i < a.spans.size(); ++i) {
        INFO("span " << i);
        REQUIRE(a.spans[i].index() == b.spans[i].index());
        if (const auto* la = std::get_if<LineSpan>(&a.spans[i])) {
            const auto& lb = std::get<LineSpan>(b.spans[i]);
            CHECK(near(la->start, lb.start));
            CHECK(near(la->end, lb.end));
        } else if (const auto* aa = std::get_if<ArcSpan>(&a.spans[i])) {
            const auto& ab = std::get<ArcSpan>(b.spans[i]);
            CHECK(near(aa->centre, ab.centre));
            CHECK(std::fabs(aa->radius - ab.radius) <= kRoundTrip);
            CHECK(aa->direction == ab.direction);
            CHECK(near(aa->startPoint(), ab.startPoint()));
            CHECK(near(aa->endPoint(), ab.endPoint()));
            CHECK(aa->isFullCircle() == ab.isFullCircle());
        } else {
            const auto& ba = std::get<BezierSpan>(a.spans[i]);
            const auto& bb = std::get<BezierSpan>(b.spans[i]);
            CHECK(near(ba.p0, bb.p0));
            CHECK(near(ba.p1, bb.p1));
            CHECK(near(ba.p2, bb.p2));
            CHECK(near(ba.p3, bb.p3));
        }
    }
}

void requireSameDocument(const Doc& doc, const DxfDrawing& d) {
    REQUIRE(d.layers.size() == doc.layers.size());
    for (std::size_t i = 0; i < doc.layers.size(); ++i) {
        CHECK(d.layers[i].name == doc.layers[i].name);
        CHECK(d.layers[i].rgb == doc.layers[i].rgb);
        CHECK(d.layers[i].visible == doc.layers[i].visible);
    }
    REQUIRE(d.contours.size() == doc.contours.size());
    for (std::size_t i = 0; i < doc.contours.size(); ++i) {
        REQUIRE(d.contours[i].layer == doc.contours[i].layer);
        requireSameContour(doc.contours[i].contour, d.contours[i].contour, i);
    }
}

}  // namespace

TEST_CASE("AC2: export -> import round trip of a random document; export is deterministic",
          "[io][dxf][ac2][export]") {
    for (const std::uint32_t seed : {std::uint32_t{20261025}, std::uint32_t{7}, std::uint32_t{99}}) {
        INFO("seed " << seed);
        const Doc doc = randomDocument(seed, 300);
        auto bytes = exportDxf(doc.layers, doc.contours);
        REQUIRE(bytes.ok());
        auto again = exportDxf(doc.layers, doc.contours);
        REQUIRE(again.ok());
        CHECK(again.value() == bytes.value());  // byte-identical

        auto imported = importDxf(bytes.value());
        REQUIRE(imported.ok());
        const DxfDrawing& d = imported.value();
        CHECK(d.warnings.empty());
        CHECK(d.version == "AC1015");
        CHECK(d.insunits == 4);
        CHECK(d.unitSource == DxfUnitSource::Insunits);
        requireSameDocument(doc, d);

        // Re-export the imported drawing and import again: still the same document.
        auto second = exportDxf(d.layers, d.contours);
        REQUIRE(second.ok());
        requireSameDocument(doc, importDxf(second.value()).value());
    }
}

TEST_CASE("AC2: near-full arcs and full circles inside contours round-trip within 1e-9",
          "[io][dxf][ac2][export]") {
    // Arcs sweeping more than pi are written as several bulge segments (|bulge| <= 1) and rebuilt
    // exactly from Carvetto XDATA; a single huge bulge would lose precision as 1/(2*pi - sweep).
    std::mt19937 rng(424242);
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    auto coord = [&] { return (unit(rng) * 2.0 - 1.0) * 1e5; };
    auto radius = [&] { return std::pow(10.0, -3.0 + 7.0 * unit(rng)); };  // 1e-3 .. 1e4
    auto makeArc = [&](Vec2 start, double r, double sweep) {
        const double a0 = unit(rng) * kTwoPi;
        ArcSpan a;
        a.centre = start - Vec2::fromAngle(a0, r);
        a.radius = r;
        a.startAngle = a0;
        a.endAngle = std::fabs(sweep) >= kTwoPi ? a0 : normalizeAngle(a0 + sweep);
        a.direction = sweep > 0.0 ? ArcDirection::CCW : ArcDirection::CW;
        return a;
    };
    Doc doc;
    doc.layers.resize(1);
    const std::vector<double> gaps{1e-1, 1e-4, 1e-6, 1e-9, 0.0};  // 0 = exact full circle
    for (int round = 0; round < 40; ++round) {
        for (const double gap : gaps) {
            for (const double sign : {1.0, -1.0}) {
                const double sweep = sign * (kTwoPi - gap);
                const double r = radius();
                const Vec2 p{coord(), coord()};
                // (a) the arc alone
                if (gap > 0.0) {
                    Contour c;
                    c.spans.emplace_back(makeArc(p, r, sweep));
                    doc.contours.push_back({0, c});
                }
                // (b) line -> arc -> line [-> closing line], and an arc between Beziers
                Contour c;
                const Vec2 q = p + Vec2{coord(), coord()} * 1e-3;
                c.spans.emplace_back(LineSpan{q, p});
                c.spans.emplace_back(makeArc(p, r, sweep));
                const Vec2 e = endPoint(c.spans.back());
                const Vec2 f = e + Vec2{coord(), coord()} * 1e-3;
                c.spans.emplace_back(LineSpan{e, f});
                if (round % 2 == 0) {
                    c.spans.emplace_back(LineSpan{f, q});
                    c.closed = true;
                }
                doc.contours.push_back({0, c});
                Contour m;
                m.spans.emplace_back(BezierSpan{q, q + Vec2{1, 2}, p - Vec2{2, 1}, p});
                m.spans.emplace_back(makeArc(p, r, sweep));
                const Vec2 g = endPoint(m.spans.back());
                m.spans.emplace_back(BezierSpan{g, g + Vec2{3, 0}, g + Vec2{3, 3}, g + Vec2{0, 3}});
                doc.contours.push_back({0, m});
            }
        }
    }
    const Bytes bytes = exportDxf(doc.layers, doc.contours).value();
    const DxfDrawing d = importDxf(bytes).value();
    CHECK(d.warnings.empty());
    requireSameDocument(doc, d);

    SECTION("without Carvetto XDATA every split segment is still well conditioned") {
        std::string stripped;
        for (const auto& [code, value] : asciiPairs(toString(bytes))) {
            if (code < 1000) {
                stripped += std::to_string(code) + "\n" + value + "\n";
            }
        }
        const DxfDrawing p = importDxf(toBytes(stripped)).value();
        std::size_t arcs = 0;
        for (const DxfContour& c : p.contours) {
            for (const Span& s : c.contour.spans) {
                const auto* a = std::get_if<ArcSpan>(&s);
                if (a == nullptr) {
                    continue;
                }
                ++arcs;
                CHECK(std::fabs(a->sweepAngle()) <= kPi + 1e-12);
                // Find the original arc this piece came from (same radius within 1e-9).
                bool found = false;
                for (const DxfContour& o : doc.contours) {
                    for (const Span& os : o.contour.spans) {
                        const auto* oa = std::get_if<ArcSpan>(&os);
                        if (oa != nullptr && distance(oa->centre, a->centre) <= 1e-9 &&
                            std::fabs(oa->radius - a->radius) <= 1e-9) {
                            found = true;
                        }
                    }
                }
                CHECK(found);
            }
        }
        CHECK(arcs > 0);
    }
}

TEST_CASE("regression: one-span near-full arc keeps its radius", "[io][dxf][ac2][export]") {
    for (const double gap : {1e-5, 1e-7}) {
        const std::vector<DxfLayer> layers(1);
        std::vector<DxfContour> contours(1);
        const ArcSpan arc{{123.456, -78.9}, 250.0, 0.3, 0.3 + kTwoPi - gap, ArcDirection::CCW};
        contours[0].contour.spans.emplace_back(arc);
        const DxfDrawing d = importDxf(exportDxf(layers, contours).value()).value();
        REQUIRE(d.contours.size() == 1);
        REQUIRE(d.contours[0].contour.spans.size() == 1);
        const auto& back = std::get<ArcSpan>(d.contours[0].contour.spans[0]);
        CHECK(std::fabs(distance(back.pointAt(0.5), arc.centre) - 250.0) <= 1e-9);
        CHECK(distance(back.endPoint(), arc.endPoint()) <= 1e-9);
        CHECK(back == arc);  // restored exactly from the XDATA
    }
}

TEST_CASE("AC3: export writes $INSUNITS = millimetres", "[io][dxf][ac3][export]") {
    const std::vector<DxfLayer> layers{DxfLayer{}};
    std::vector<DxfContour> contours(1);
    contours[0].contour.spans.emplace_back(LineSpan{{0, 0}, {25.4, 0}});
    const std::string text = toString(exportDxf(layers, contours).value());
    CHECK(text.find("9\n$INSUNITS\n70\n4\n") != std::string::npos);
    CHECK(text.find("9\n$ACADVER\n1\nAC1015\n") != std::string::npos);
    CHECK(text.rfind("0\nEOF\n") == text.size() - 6);
    // Re-importing never rescales: the file says millimetres.
    DxfImportOptions inchDefault;
    inchDefault.unitlessScale = 25.4;
    const DxfDrawing d = importDxf(toBytes(text), inchDefault).value();
    CHECK(d.contours.at(0).contour.length() == 25.4);
}

TEST_CASE("export entity mapping and argument checks", "[io][dxf][export]") {
    std::vector<DxfLayer> layers(2);
    layers[0].name = "A";
    layers[1].name = "bad/name:*";
    std::vector<DxfContour> contours;
    Contour lines = Contour::rectangle(Box2{{0, 0}, {10, 5}});
    contours.push_back({0, lines});
    contours.push_back({1, Contour::circle({3, 3}, 2)});
    Contour mixed;
    mixed.spans.emplace_back(LineSpan{{0, 0}, {1, 0}});
    mixed.spans.emplace_back(BezierSpan{{1, 0}, {2, 1}, {3, 1}, {4, 0}});
    mixed.spans.emplace_back(BezierSpan{{4, 0}, {5, -1}, {6, -1}, {7, 0}});
    mixed.spans.emplace_back(ArcSpan::fromCentre({7, 3.5}, {7, 0}, {7, 7}, ArcDirection::CCW));
    contours.push_back({0, mixed});
    const std::string text = toString(exportDxf(layers, contours).value());
    CHECK(text.find("0\nLWPOLYLINE\n") != std::string::npos);
    CHECK(text.find("0\nCIRCLE\n") != std::string::npos);
    CHECK(text.find("0\nSPLINE\n") != std::string::npos);
    CHECK(text.find("\n71\n3\n72\n11\n73\n7\n") != std::string::npos);  // 2 Beziers: 7 controls, 11 knots
    CHECK(text.find("bad_name__") != std::string::npos);
    const DxfDrawing d = importDxf(toBytes(text)).value();
    REQUIRE(d.contours.size() == 3);
    CHECK(d.contours[2].contour.spans.size() == 4);
    CHECK(d.layers[1].name == "bad_name__");

    SECTION("a third-party reader without Carvetto XDATA still gets valid pieces") {
        std::string stripped;
        for (const auto& [code, value] : asciiPairs(text)) {
            if (code < 1000) {
                stripped += std::to_string(code) + "\n" + value + "\n";
            }
        }
        const DxfDrawing p = importDxf(toBytes(stripped)).value();
        CHECK(p.contours.size() == 5);  // the mixed contour arrives as line / spline / arc pieces
    }
    SECTION("invalid input") {
        std::vector<DxfContour> bad{{5, lines}};
        CHECK(exportDxf(layers, bad).error().code == DxfError::Code::InvalidArgument);
        Contour nan;
        nan.spans.emplace_back(LineSpan{{0, 0}, {std::nan(""), 0}});
        std::vector<DxfContour> bad2{{0, nan}};
        CHECK(exportDxf(layers, bad2).error().code == DxfError::Code::InvalidArgument);
    }
}

namespace {

/// Largest distance between two spans sampled at the same parameters.
double spanDeviation(const Span& a, const Span& b) {
    double d = 0.0;
    for (int i = 0; i <= 64; ++i) {
        d = std::max(d, distance(pointAt(a, i / 64.0), pointAt(b, i / 64.0)));
    }
    return d;
}

std::string formatReal(double v) {
    std::ostringstream s;
    s.precision(17);
    s << v;
    return s.str();
}

/// Rewrites the CARVETTO XDATA of the (only) LWPOLYLINE: 1071 number `n1071` (1-based) and/or
/// the five 1040 values (centre x, centre y, radius, start, end) of the first split arc.
std::string tamper(const std::string& text, int n1071, const std::optional<std::vector<double>>& reals) {
    std::string out;
    int seen1071 = 0;
    int seen1040 = 0;
    for (auto [code, value] : asciiPairs(text)) {
        if (code == 1071 && ++seen1071 == n1071) {
            value = value == "0" ? "1" : "0";
        }
        if (code == 1040 && reals && seen1040 < 5) {
            value = formatReal((*reals)[static_cast<std::size_t>(seen1040++)]);
        }
        out += std::to_string(code) + "\n" + value + "\n";
    }
    return out;
}

}  // namespace

TEST_CASE("AC2: tampered split-arc XDATA falls back to the bulge pieces", "[io][dxf][ac2][export]") {
    // The XDATA only restores exact values; when it disagrees with the bulge segments it
    // replaces (direction, circle, sweep) the importer keeps the pieces.
    const std::vector<DxfLayer> layers(1);
    int cases = 0;
    for (const double r : {0.5, 50.0, 5000.0}) {
        for (const double sign : {1.0, -1.0}) {
            for (const double gap : {1e-1, 1e-3, 1e-6}) {
                const Vec2 centre{1234.5, -678.25};
                const double a0 = 0.7;
                const double sweep = sign * (kTwoPi - gap);
                const ArcSpan arc{centre, r, a0, normalizeAngle(a0 + sweep),
                                  sign > 0.0 ? ArcDirection::CCW : ArcDirection::CW};
                std::vector<DxfContour> contours(1);
                Contour& c = contours[0].contour;
                c.spans.emplace_back(LineSpan{arc.startPoint() + Vec2{-3.0, 1.0}, arc.startPoint()});
                c.spans.emplace_back(arc);
                c.spans.emplace_back(LineSpan{arc.endPoint(), arc.endPoint() + Vec2{2.0, 4.0}});
                const std::string text = toString(exportDxf(layers, contours).value());

                // Untampered: exact restore. Without XDATA: the pieces (the reference).
                const DxfDrawing exact = importDxf(toBytes(text)).value();
                REQUIRE(exact.contours.size() == 1);
                REQUIRE(exact.contours[0].contour.spans.size() == 3);
                CHECK(std::get<ArcSpan>(exact.contours[0].contour.spans[1]) == arc);
                const DxfDrawing pieces =
                    importDxf(toBytes(tamper(text, 0, std::vector<double>{0, 0, 0, 0, 0}))).value();
                REQUIRE(pieces.contours.size() == 1);
                const std::vector<Span>& ref = pieces.contours[0].contour.spans;
                REQUIRE(ref.size() > 3);  // the radius-0 XDATA arc is rejected

                // A different circle through the same end points: centre mirrored across the chord.
                const Vec2 p = arc.startPoint();
                const Vec2 q = arc.endPoint();
                const Vec2 mid = (p + q) * 0.5;
                const Vec2 other = mid * 2.0 - centre;
                const double otherR = distance(other, p);
                const double otherA0 = (p - other).angle();
                const double otherA1 = (q - other).angle();
                const ArcSpan scaled{centre, r * (1.0 + 1e-7), a0, arc.endAngle, arc.direction};

                struct Case {
                    const char* name;
                    int flip;
                    std::optional<std::vector<double>> reals;
                };
                const std::vector<Case> tampered{
                    {"direction flag flipped", 9, std::nullopt},
                    {"other circle through the end points", 0,
                     std::vector<double>{other.x, other.y, otherR, otherA0, otherA1}},
                    {"centre moved by 5e-4", 0,
                     std::vector<double>{centre.x + 5e-4, centre.y, r, a0, arc.endAngle}},
                    {"centre moved by 1e-6", 0,
                     std::vector<double>{centre.x, centre.y + 1e-6, r, a0, arc.endAngle}},
                    {"radius scaled by 1+1e-7", 0,
                     std::vector<double>{centre.x, centre.y, scaled.radius, a0, arc.endAngle}},
                };
                for (const Case& t : tampered) {
                    INFO(t.name << " r=" << r << " sign=" << sign << " gap=" << gap);
                    const DxfDrawing d = importDxf(toBytes(tamper(text, t.flip, t.reals))).value();
                    REQUIRE(d.contours.size() == 1);
                    const std::vector<Span>& got = d.contours[0].contour.spans;
                    REQUIRE(got.size() == ref.size());  // fell back to the pieces
                    for (std::size_t i = 0; i < got.size(); ++i) {
                        CHECK(spanDeviation(got[i], ref[i]) <= 1e-9);
                    }
                    ++cases;
                }
            }
        }
    }
    CHECK(cases == 90);
}
