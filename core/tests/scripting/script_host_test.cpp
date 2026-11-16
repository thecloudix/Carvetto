#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <stdexcept>
#include <string>

#include "carvetto/scripting/script_host.hpp"

using namespace carvetto::scripting;

namespace {

ScriptLimits smallLimits() {
    ScriptLimits l;
    l.maxInstructions = 200'000;
    l.maxPatternSteps = 200'000;
    l.maxMemoryBytes = 4u * 1024u * 1024u;
    l.maxStringBytes = 1u * 1024u * 1024u;
    l.maxOutputBytes = 4096;
    return l;
}

bool contains(const std::string& s, const char* part) {
    return s.find(part) != std::string::npos;
}

}  // namespace

TEST_CASE("scripting: returns values and captures print", "[scripting]") {
    ScriptHost host;
    const auto r = host.run("print('hi', 1, 2.5, true, nil)\nreturn 1 + 2, 'x', 2.5, {1, 2, {3}}, true", "t");
    REQUIRE(r.ok());
    REQUIRE(r.output.size() == 1);
    CHECK(r.output[0] == "hi\t1\t2.5\ttrue\tnil");
    REQUIRE(r.returned.size() == 5);
    CHECK(r.returned[0].isInt());
    CHECK(r.returned[0].asInt() == 3);
    CHECK(r.returned[1].asString() == "x");
    CHECK(r.returned[2].asNumber() == 2.5);
    REQUIRE(r.returned[3].isArray());
    CHECK(r.returned[3].asArray().size() == 3);
    CHECK(r.returned[3].asArray()[2].asArray()[0].asInt() == 3);
    CHECK(r.returned[4].asBool());
}

TEST_CASE("scripting: errors carry chunk name and line number", "[scripting]") {
    ScriptHost host;
    SECTION("runtime error") {
        const auto r = host.run("local a = 1\n\nlocal b = nil + 1\n", "gadget.lua");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(r.line == 3);
        CHECK(contains(r.message, "gadget.lua:3:"));
        CHECK(contains(r.traceback, "stack traceback"));
    }
    SECTION("error() with a message") {
        const auto r = host.run("\n\nerror('boom')", "g");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(r.line == 3);
        CHECK(contains(r.message, "boom"));
    }
    SECTION("error with a non-string value") {
        const auto r = host.run("error({})", "g");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.message, "error object is a table value"));
    }
    SECTION("syntax error") {
        const auto r = host.run("local x = 1\nlocal = 2\n", "s.lua");
        CHECK(r.status == ScriptStatus::SyntaxError);
        CHECK(r.line == 2);
        CHECK(contains(r.message, "s.lua:2:"));
    }
    SECTION("check compiles without running") {
        CHECK(host.check("while true do end", "c").ok());
        CHECK(host.check("x =", "c").status == ScriptStatus::SyntaxError);
    }
}

TEST_CASE("scripting: sandbox removes io os debug package and file loading", "[scripting][sandbox]") {
    ScriptHost host;
    for (const char* name : {"io", "os", "debug", "package", "require", "dofile", "loadfile", "loadstring"}) {
        const auto r = host.run(std::string("return ") + name + " == nil", "t");
        INFO(name);
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asBool());
    }
    SECTION("calling them is an error") {
        CHECK(host.run("os.execute('echo hi')", "t").status == ScriptStatus::RuntimeError);
        CHECK(host.run("io.open('/etc/passwd')", "t").status == ScriptStatus::RuntimeError);
        CHECK(host.run("debug.getinfo(1)", "t").status == ScriptStatus::RuntimeError);
        CHECK(host.run("require('os')", "t").status == ScriptStatus::RuntimeError);
    }
    SECTION("no way back through load, string.dump or metatables") {
        auto r = host.run("return string.dump == nil", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asBool());
        r = host.run("local f, e = load('\\27Lua binary'); return f == nil, e ~= nil", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asBool());
        r = host.run("return load(function() return nil end)", "t");
        CHECK(r.status == ScriptStatus::RuntimeError);
        r = host.run("return pcall(setmetatable, {}, {__gc = function() while true do end end})", "t");
        REQUIRE(r.ok());
        CHECK_FALSE(r.returned.at(0).asBool());
        // an _ENV that hides nothing still cannot reach the removed libraries
        r = host.run("local f = load('return os', 'c', 't', {}); return f()", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).isNil());
    }
    SECTION("load runs text chunks") {
        const auto r = host.run("return load('return 6 * 7')()", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asInt() == 42);
    }
    SECTION("grants open the named libraries") {
        ScriptGrants g;
        g.os = true;
        g.debug = true;
        ScriptHost granted({}, g);
        const auto r = granted.run("return type(os.time()), type(debug.getinfo), io, package", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asString() == "number");
        CHECK(r.returned.at(1).asString() == "function");
        CHECK(r.returned.at(2).isNil());
        CHECK(r.returned.at(3).isNil());
    }
}

TEST_CASE("scripting: each run has a fresh state and fresh budgets", "[scripting]") {
    ScriptHost host(smallLimits());
    REQUIRE(host.run("leaked = 5", "a").ok());
    const auto r = host.run("return leaked", "b");
    REQUIRE(r.ok());
    CHECK(r.returned.at(0).isNil());
    const auto r1 = host.run("local s = 0 for i = 1, 20000 do s = s + i end return s", "c");
    const auto r2 = host.run("local s = 0 for i = 1, 20000 do s = s + i end return s", "c");
    REQUIRE(r1.ok());
    CHECK(r1.returned.at(0).asInt() == 200010000);
    CHECK(r1.instructionsUsed == r2.instructionsUsed);  // deterministic accounting
    CHECK(r1.instructionsUsed > 0);
}

TEST_CASE("scripting: math.random is deterministic until the script seeds it", "[scripting]") {
    ScriptHost host;
    const auto a = host.run("return math.random(1, 1000000)", "t");
    const auto b = host.run("return math.random(1, 1000000)", "t");
    REQUIRE(a.ok());
    REQUIRE(b.ok());
    CHECK(a.returned.at(0).asInt() == b.returned.at(0).asInt());
}

TEST_CASE("scripting: hostile scripts - infinite loops", "[scripting][hostile]") {
    ScriptHost host(smallLimits());
    const char* loops[] = {
        "while true do end",
        "repeat until false",
        "for i = 1, math.huge do end",
        "local function f() return f() end f()",  // a tail call loop
        "while true do pcall(function() while true do end end) end",
        "pcall(function() while true do pcall(error) end end)",
        "local t = setmetatable({}, {__index = function(t, k) return t[k + 1] end}) local _ = t[1]",
        "local co = coroutine.wrap(function() while true do end end) co()",
        "while true do coroutine.wrap(function() local x = 1 end)() end",
        "for _ in (function() return function() return 1 end end)() do end",
        "goto top ::top:: goto top",
    };
    for (const char* src : loops) {
        INFO(src);
        const auto r = host.run(src, "hostile");
        CHECK((r.status == ScriptStatus::InstructionLimit || r.status == ScriptStatus::RuntimeError));
        if (r.status == ScriptStatus::InstructionLimit) {
            CHECK(r.instructionsUsed >= 1000);
        }
    }
    SECTION("budget stop is reported as such, within one block") {
        const auto r = host.run("while true do end", "hostile");
        CHECK(r.status == ScriptStatus::InstructionLimit);
        CHECK(contains(r.message, "instruction budget exceeded"));
        CHECK(r.instructionsUsed >= 200'000);
        CHECK(r.instructionsUsed < 200'000 + 2'000);
    }
    SECTION("pcall cannot swallow the stop") {
        const auto r = host.run("while true do pcall(function() while true do end end) end", "h");
        CHECK(r.status == ScriptStatus::InstructionLimit);
    }
}

TEST_CASE("scripting: hostile scripts - memory and string bombs", "[scripting][hostile]") {
    ScriptLimits memLimits = smallLimits();
    memLimits.maxInstructions = 50'000'000;  // the heap limit must be what stops these
    ScriptHost host(memLimits);
    SECTION("table filling") {
        const auto r = host.run("local t = {} for i = 1, 1e9 do t[i] = i end", "h");
        CHECK(r.status == ScriptStatus::MemoryLimit);
        CHECK(r.peakMemoryBytes <= smallLimits().maxMemoryBytes);
    }
    SECTION("doubling string") {
        const auto r = host.run("local s = 'x' while true do s = s .. s end", "h");
        CHECK(r.status == ScriptStatus::MemoryLimit);
        CHECK(r.peakMemoryBytes <= smallLimits().maxMemoryBytes);
    }
    SECTION("string.rep is refused above maxStringBytes") {
        const auto r = host.run("return string.rep('x', 2 * 1024 * 1024)", "h");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.message, "resulting string too large"));
    }
    SECTION("string.rep with a separator and an overflowing count") {
        CHECK(host.run("return ('abc'):rep(math.maxinteger, 'sep')", "h").status == ScriptStatus::RuntimeError);
        CHECK(host.run("return ('abc'):rep(1e9 // 1)", "h").status == ScriptStatus::RuntimeError);
    }
    SECTION("string.rep under the limit but over the heap") {
        const auto r = host.run("local t = {} for i = 1, 100 do t[i] = ('x'):rep(900000) end", "h");
        CHECK(r.status == ScriptStatus::MemoryLimit);
    }
    SECTION("a bomb caught by pcall still ends") {
        const auto r = host.run("while true do pcall(function() local t = {} for i = 1, 1e9 do t[i] = i end end) end", "h");
        CHECK((r.status == ScriptStatus::InstructionLimit || r.status == ScriptStatus::MemoryLimit));
    }
    SECTION("table.concat and unpack bombs") {
        CHECK_FALSE(host.run("return table.concat({}, 'x', 1, 1e9)", "h").ok());
        CHECK_FALSE(host.run("return table.unpack({}, 1, 1e9)", "h").ok());
    }
    SECTION("table.move over a huge range of nothing") {
        const auto r = host.run("table.move({}, 1, 1 << 40, 1)", "h");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.message, "too many elements"));
    }
    SECTION("a script too large for the limit is refused unrun") {
        ScriptLimits l = smallLimits();
        l.maxSourceBytes = 100;
        ScriptHost small(l);
        const auto r = small.run(std::string(101, ' '), "h");
        CHECK(r.status == ScriptStatus::LimitsInvalid);
        CHECK(r.instructionsUsed == 0);
    }
}

TEST_CASE("scripting: hostile scripts - deep recursion", "[scripting][hostile]") {
    ScriptHost host(smallLimits());
    SECTION("Lua recursion") {
        const auto r = host.run("local function f(n) return 1 + f(n + 1) end return f(1)", "h");
        CHECK((r.status == ScriptStatus::RuntimeError || r.status == ScriptStatus::MemoryLimit));
        if (r.status == ScriptStatus::RuntimeError) {
            CHECK(contains(r.message, "stack overflow"));
        }
    }
    SECTION("recursion through a C function (pcall)") {
        const auto r = host.run("local function f() return pcall(f) end return f()", "h");
        CHECK_FALSE(r.status == ScriptStatus::Cancelled);
    }
    SECTION("recursion through metamethods") {
        const auto r = host.run("local t = setmetatable({}, {}) getmetatable(t).__index = function(a, k) return a[k] end return t.x", "h");
        CHECK((r.status == ScriptStatus::RuntimeError || r.status == ScriptStatus::MemoryLimit));
    }
    SECTION("deeply nested source") {
        const auto r = host.run("return " + std::string(5000, '(') + "1" + std::string(5000, ')'), "h");
        CHECK(r.status == ScriptStatus::SyntaxError);
    }
}

TEST_CASE("scripting: hostile scripts - pattern backtracking and C-side loops", "[scripting][hostile]") {
    ScriptHost host(smallLimits());
    SECTION("catastrophic pattern") {
        const auto r = host.run("return (('a'):rep(2000) .. 'c'):find(('.-'):rep(8) .. 'b')", "h");
        CHECK(r.status == ScriptStatus::InstructionLimit);
        CHECK(contains(r.message, "pattern-step budget exceeded"));
        CHECK(r.patternStepsUsed > 200'000);
    }
    SECTION("gsub with a hostile pattern") {
        const auto r = host.run("return (('a'):rep(3000)):gsub('(a*)*b', 'x')", "h");
        CHECK_FALSE(r.ok());
    }
    SECTION("table.insert at the front of a big table is charged") {
        const auto r = host.run("local t = {} for i = 1, 5000 do t[i] = i end for i = 1, 1e9 do table.insert(t, 1, 0) end", "h");
        CHECK(r.status == ScriptStatus::InstructionLimit);
    }
    SECTION("table.sort of a big table in a loop is charged") {
        const auto r = host.run("local t = {} for i = 1, 3000 do t[i] = 3000 - i end while true do table.sort(t) end", "h");
        CHECK((r.status == ScriptStatus::InstructionLimit || r.status == ScriptStatus::MemoryLimit));
    }
    SECTION("collectgarbage cannot be used to burn time") {
        const auto r = host.run("while true do collectgarbage() end", "h");
        CHECK(r.status == ScriptStatus::InstructionLimit);
    }
}

TEST_CASE("scripting: output limit", "[scripting][hostile]") {
    ScriptHost host(smallLimits());
    const auto r = host.run("while true do print('0123456789') end", "h");
    CHECK(r.status == ScriptStatus::OutputLimit);
    CHECK(r.output.size() <= 4096 / 11);
    CHECK(r.output.size() >= 300);
}

TEST_CASE("scripting: cancellation", "[scripting][cancel]") {
    ScriptHost host;
    SECTION("flag already set: nothing runs") {
        std::atomic<bool> flag{true};
        const auto r = host.run("x = 1", "t", &flag);
        CHECK(r.status == ScriptStatus::Cancelled);
        CHECK(r.instructionsUsed == 0);
    }
    SECTION("flag raised by a host function in the middle of a loop") {
        std::atomic<bool> flag{false};
        ScriptHost h2;
        h2.registerFunction("", "stop", [&flag](const ScriptValues&) {
            flag.store(true);
            return ScriptValues{};
        });
        const auto r = h2.run("stop() while true do end", "t", &flag);
        CHECK(r.status == ScriptStatus::Cancelled);
        CHECK(contains(r.message, "cancelled"));
        CHECK(r.instructionsUsed < 10'000);
    }
    SECTION("flag raised inside a pattern match") {
        std::atomic<bool> flag{false};
        ScriptHost h2;
        h2.registerFunction("", "stop", [&flag](const ScriptValues&) {
            flag.store(true);
            return ScriptValues{};
        });
        const auto r = h2.run("stop() return (('a'):rep(2000) .. 'c'):find(('.-'):rep(8) .. 'b')", "t", &flag);
        CHECK(r.status == ScriptStatus::Cancelled);
    }
    SECTION("cancel cannot be swallowed by pcall") {
        std::atomic<bool> flag{false};
        ScriptHost h2;
        h2.registerFunction("", "stop", [&flag](const ScriptValues&) {
            flag.store(true);
            return ScriptValues{};
        });
        const auto r = h2.run("while true do pcall(function() stop() while true do end end) end", "t", &flag);
        CHECK(r.status == ScriptStatus::Cancelled);
    }
}

TEST_CASE("scripting: host functions and values", "[scripting]") {
    ScriptHost host;
    host.setValue("carvetto", "answer", 42);
    host.setValue("", "greeting", "hello");
    host.registerFunction("math2", "add", [](const ScriptValues& a) {
        if (a.size() != 2) {
            throw ScriptError{"add needs two numbers"};
        }
        return ScriptValues{ScriptValue(a[0].asNumber() + a[1].asNumber())};
    });
    host.registerFunction("math2", "sum", [](const ScriptValues& a) {
        double s = 0;
        for (const ScriptValue& v : a.at(0).asArray()) {
            s += v.asNumber();
        }
        return ScriptValues{ScriptValue(s), ScriptValue(ScriptValue::Array{1, "two", ScriptValue::Array{3.5}})};
    });
    host.registerFunction("", "boom", [](const ScriptValues&) -> ScriptValues { throw std::runtime_error("native failure"); });

    SECTION("calls and constants") {
        const auto r = host.run("return math2.add(1, 2.5), carvetto.answer, carvetto.api_version, greeting", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asNumber() == 3.5);
        CHECK(r.returned.at(1).asInt() == 42);
        CHECK(r.returned.at(2).asInt() == 1);
        CHECK(r.returned.at(3).asString() == "hello");
    }
    SECTION("arrays both ways") {
        const auto r = host.run("local s, t = math2.sum({1, 2, 3.5}) return s, t[1], t[2], t[3][1], #t", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asNumber() == 6.5);
        CHECK(r.returned.at(2).asString() == "two");
        CHECK(r.returned.at(3).asNumber() == 3.5);
        CHECK(r.returned.at(4).asInt() == 3);
    }
    SECTION("a ScriptError is a catchable Lua error with the script's line") {
        auto r = host.run("local ok, e = pcall(math2.add, 1) return ok, e", "t");
        REQUIRE(r.ok());
        CHECK_FALSE(r.returned.at(0).asBool());
        CHECK(contains(r.returned.at(1).asString(), "add needs two numbers"));
        r = host.run("\n\nmath2.add(1)", "t");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(r.line == 3);
        CHECK(contains(r.message, "add needs two numbers"));
    }
    SECTION("other exceptions too") {
        const auto r = host.run("boom()", "t");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.message, "native failure"));
    }
    SECTION("values that cannot cross are refused") {
        CHECK(host.run("math2.add(print, 1)", "t").status == ScriptStatus::RuntimeError);
        CHECK(host.run("math2.sum({1, 2, x = 3})", "t").status == ScriptStatus::RuntimeError);
        CHECK(host.run("local t = {} t[1] = t math2.sum({t})", "t").status == ScriptStatus::RuntimeError);
    }
    SECTION("unsupported results of a chunk become nil") {
        const auto r = host.run("return print, {x = 1}", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).isNil());
        CHECK(r.returned.at(1).isNil());
    }
    SECTION("re-registering replaces") {
        host.registerFunction("math2", "add", [](const ScriptValues&) { return ScriptValues{ScriptValue(7)}; });
        const auto r = host.run("return math2.add()", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asInt() == 7);
    }
}

TEST_CASE("scripting: a very small heap limit fails cleanly", "[scripting][hostile]") {
    ScriptLimits l;
    l.maxMemoryBytes = 2048;
    ScriptHost host(l);
    const auto r = host.run("return 1", "t");
    CHECK(r.status == ScriptStatus::MemoryLimit);
}

TEST_CASE("scripting: hostile values crossing the boundary are budgeted", "[scripting][hostile]") {
    // Every case below used to expand to ~60^15 copies (or hang) before the element/byte budget.
    ScriptHost host;
    host.registerFunction("", "id", [](const ScriptValues& a) { return a; });
    const char* dag =
        "local l = {1} for i = 1, 15 do local n = {} for j = 1, 60 do n[j] = l end l = n end ";
    SECTION("shared DAG returned") {
        const auto r = host.run(std::string(dag) + "return l", "t");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.message, "too large"));
        CHECK(r.returned.empty());
    }
    SECTION("shared DAG passed to a host function") {
        const auto r = host.run(std::string(dag) + "return id(l)", "t");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.message, "too large"));
    }
    SECTION("the script can catch the host-argument failure and carry on") {
        const auto r = host.run(std::string(dag) + "local ok = pcall(id, l) return ok", "t");
        REQUIRE(r.ok());
        CHECK_FALSE(r.returned.at(0).asBool());
    }
    SECTION("cyclic wide table") {
        const auto r = host.run("local t = {} for j = 1, 60 do t[j] = t end return t", "t");
        REQUIRE(r.ok());  // too deep, not a budget failure: comes back as nil
        CHECK(r.returned.at(0).isNil());
        const auto r2 = host.run("local t = {} for j = 1, 60 do t[j] = t end return id(t)", "t");
        CHECK_FALSE(r2.ok());
    }
    SECTION("wide table") {
        ScriptLimits l;
        l.maxValueElements = 1000;
        ScriptHost h2(l);
        const auto r = h2.run("local t = {} for i = 1, 5000 do t[i] = i end return t", "t");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.message, "too large"));
        const auto ok = h2.run("local t = {} for i = 1, 500 do t[i] = i end return t", "t");
        CHECK(ok.ok());
    }
    SECTION("long strings inside a table") {
        ScriptLimits l;
        l.maxValueBytes = 100'000;
        ScriptHost h2(l);
        h2.registerFunction("", "id", [](const ScriptValues& a) { return a; });
        const auto r = h2.run("local s = ('x'):rep(40000) return {s, s, s}", "t");
        CHECK(r.status == ScriptStatus::RuntimeError);
        const auto r2 = h2.run("local s = ('x'):rep(40000) return id({s, s, s})", "t");
        CHECK(r2.status == ScriptStatus::RuntimeError);
        const auto ok = h2.run("return {('x'):rep(40000)}", "t");
        CHECK(ok.ok());
    }
    SECTION("cancellation is polled while converting") {
        std::atomic<bool> flag{false};
        ScriptHost h2;
        h2.registerFunction("", "stop", [&flag](const ScriptValues&) {
            flag.store(true);
            return ScriptValues{};
        });
        const auto r = h2.run(std::string(dag) + "stop() return l", "t", &flag);
        CHECK_FALSE(r.ok());
    }
}

TEST_CASE("scripting: host function throwing a non-std exception is contained", "[scripting]") {
    ScriptHost host;
    host.registerFunction("", "weird", [](const ScriptValues&) -> ScriptValues { throw 42; });
    const auto r = host.run("local ok, e = pcall(weird) return ok, e", "t");
    REQUIRE(r.ok());
    CHECK_FALSE(r.returned.at(0).asBool());
    const auto r2 = host.run("weird()", "t");
    CHECK(r2.status == ScriptStatus::RuntimeError);
}

TEST_CASE("scripting: string functions on huge operands are charged", "[scripting][hostile]") {
    // 1000 upper() calls on a 10 MB string must hit the instruction budget quickly.
    ScriptLimits l;
    l.maxStringBytes = 16u * 1024u * 1024u;
    ScriptHost host(l);
    int calls = 0;
    host.registerFunction("", "tick", [&calls](const ScriptValues&) {
        ++calls;
        return ScriptValues{};
    });
    const auto r = host.run("local s = string.rep('x', 10000000) for i = 1, 1000 do s:upper() tick() end", "t");
    CHECK(r.status == ScriptStatus::InstructionLimit);
    CHECK(calls < 100);  // 1 unit per 8 bytes: about 40 calls fit in 50 M; at 1 per 64 about 320 did
}
