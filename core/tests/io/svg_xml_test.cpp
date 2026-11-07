#include <catch2/catch_test_macros.hpp>
#include <string>

#include "carvetto/io/svg.hpp"
#include "svg_test_util.hpp"
#include "svg_xml.hpp"

using namespace carvetto::io;
using namespace svgtest;

namespace {

SvgResult<detail::XmlDoc> parse(const std::string& s, const detail::XmlLimits& l = {}) {
    return detail::parseXml(s, l);
}

}  // namespace

TEST_CASE("svg xml: well-formed documents", "[svg][io][xml]") {
    SECTION("elements, attributes, comments, CDATA, PIs, BOM") {
        const auto r = parse(
            "\xEF\xBB\xBF<?xml version=\"1.0\"?>\n<!-- c --><a x='1' y=\"2\"><!-- c2 --><b/><c><![CDATA[<d>]]></c>text<e/></a>\n");
        REQUIRE(r.ok());
        const auto& n = r.value().nodes;
        REQUIRE(n.size() == 4);
        CHECK(n[0].name == "a");
        REQUIRE(n[0].attr("x") != nullptr);
        CHECK(*n[0].attr("x") == "1");
        CHECK(*n[0].attr("y") == "2");
        CHECK(n[0].firstChild == 1);
        CHECK(n[1].nextSibling == 2);
        CHECK(n[2].nextSibling == 3);
        CHECK(n[3].nextSibling == detail::kNoNode);
        CHECK(n[2].parent == 0);
    }
    SECTION("predefined and numeric entities in attribute values") {
        const auto r = parse("<a v=\"&lt;&gt;&amp;&quot;&apos;&#65;&#x42;&#x20AC;&#x1F600;\"/>");
        REQUIRE(r.ok());
        CHECK(*r.value().nodes[0].attr("v") == "<>&\"'AB\xE2\x82\xAC\xF0\x9F\x98\x80");
    }
    SECTION("line numbers") {
        const auto r = parse("<a>\n\n<b/>\n  <c/></a>");
        REQUIRE(r.ok());
        CHECK(r.value().nodes[1].line == 3);
        CHECK(r.value().nodes[2].line == 4);
    }
}

TEST_CASE("svg xml: refused constructs", "[svg][io][xml]") {
    SECTION("DOCTYPE and entity declarations") {
        auto r = parse("<!DOCTYPE svg [<!ENTITY a \"b\">]><svg/>");
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::Unsupported);
        r = parse("<!DOCTYPE svg SYSTEM \"http://example.com/x.dtd\"><svg/>");
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::Unsupported);
        r = parse("<svg><!ENTITY a \"b\"></svg>");
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::Unsupported);
        CHECK(r.error().line == 1);
    }
    SECTION("undefined and malformed entity references") {
        for (const char* v : {"&foo;", "&", "&amp", "&#;", "&#x;", "&#xZZ;", "&#0;", "&#1;", "&#xD800;", "&#x110000;",
                              "&#99999999999;", "&lol1;", "&amp;&lol;"}) {
            const auto r = parse(std::string("<a v=\"") + v + "\"/>");
            INFO(v);
            CHECK_FALSE(r.ok());
        }
    }
    SECTION("malformed structure") {
        for (const char* t : {"", "   ", "text", "<a>", "<a></b>", "<a/><b/>", "<a/>junk", "<a b></a>", "<a b=c/>",
                              "<a b='1\"/>", "<a b=\"1\" b=\"2\"/>", "<a b=\"<\"/>", "<a b=\"1\"c=\"2\"/>", "<a",
                              "<a><!-- x", "<a><![CDATA[x</a>", "<?xml", "</a>", "<1/>", "<a></a >x", "<a b=\"1\" /"}) {
            INFO(t);
            CHECK_FALSE(parse(t).ok());
        }
        CHECK(parse("<a></a >").ok());
    }
    SECTION("a document that is not svg") {
        const auto r = importText("<html/>");
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::InvalidFormat);
        CHECK(r.error().describe().find("not <svg>") != std::string::npos);
    }
    SECTION("nothing is fetched: an external reference in a href is only a warning") {
        const auto r = importText(page("<use href=\"file:///etc/passwd\"/><image href=\"http://example.com/x.png\"/>"));
        REQUIRE(r.ok());
        CHECK(r.value().contours.empty());
    }
}

TEST_CASE("svg xml: resource limits", "[svg][io][xml]") {
    SECTION("depth") {
        detail::XmlLimits l;
        l.maxDepth = 5;
        std::string s;
        for (int i = 0; i < 5; ++i) {
            s += "<a>";
        }
        for (int i = 0; i < 5; ++i) {
            s += "</a>";
        }
        CHECK(parse(s, l).ok());
        s = "<a>" + s + "</a>";
        const auto r = parse(s, l);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::ResourceLimit);
    }
    SECTION("very deep nesting does not overflow the stack") {
        std::string s;
        for (int i = 0; i < 200000; ++i) {
            s += "<g>";
        }
        const auto r = importText(s);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::ResourceLimit);
    }
    SECTION("deep but allowed nesting imports") {
        SvgImportOptions o;
        o.maxDepth = 5000;
        std::string s = "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"10mm\" height=\"10mm\">";
        for (int i = 0; i < 4000; ++i) {
            s += "<g>";
        }
        s += "<path d=\"M0 0 L1 1\"/>";
        for (int i = 0; i < 4000; ++i) {
            s += "</g>";
        }
        s += "</svg>";
        const auto r = importText(s, o);
        REQUIRE(r.ok());
        CHECK(r.value().contours.size() == 1);
    }
    SECTION("element count") {
        detail::XmlLimits l;
        l.maxElements = 10;
        std::string s = "<a>";
        for (int i = 0; i < 20; ++i) {
            s += "<b/>";
        }
        s += "</a>";
        const auto r = parse(s, l);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::ResourceLimit);
    }
    SECTION("attribute length, path length, attribute count, name length, input size") {
        detail::XmlLimits l;
        l.maxAttributeLength = 10;
        l.maxLongAttributeLength = 20;
        l.maxAttributes = 3;
        l.maxNameLength = 8;
        l.maxInputBytes = 100;
        CHECK_FALSE(parse("<a v=\"12345678901\"/>", l).ok());
        CHECK(parse("<a v=\"1234567890\"/>", l).ok());
        CHECK(parse("<a d=\"12345678901234567890\"/>", l).ok());
        CHECK_FALSE(parse("<a d=\"123456789012345678901\"/>", l).ok());
        CHECK_FALSE(parse("<a points=\"123456789012345678901\"/>", l).ok());
        CHECK_FALSE(parse("<a a=\"1\" b=\"2\" c=\"3\" d=\"4\"/>", l).ok());
        CHECK_FALSE(parse("<abcdefghijkl/>", l).ok());
        CHECK_FALSE(parse("<a " + std::string(200, ' ') + "/>", l).ok());
        const auto r = parse("<a v=\"12345678901\"/>", l);
        CHECK(r.error().code == SvgError::Code::ResourceLimit);
    }
    SECTION("path data over the importer limit") {
        SvgImportOptions o;
        o.maxPathLength = 100;
        const auto r = importText(page("<path d=\"" + std::string(200, ' ') + "M0 0\"/>"), o);
        REQUIRE_FALSE(r.ok());
        CHECK(r.error().code == SvgError::Code::ResourceLimit);
    }
    SECTION("large entity-looking input does not expand") {
        std::string s = "<a v=\"";
        for (int i = 0; i < 10000; ++i) {
            s += "&amp;";
        }
        s += "\"/>";
        const auto r = parse(s);
        REQUIRE(r.ok());
        CHECK(r.value().nodes[0].attr("v")->size() == 10000);
    }
}
