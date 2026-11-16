#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <regex>
#include <stdexcept>
#include <string>

#include "carvetto/scripting/script_host.hpp"

using namespace carvetto::scripting;

namespace {

// Every ScriptLimits / ScriptHost in this file MUST start from testLimits(): the default limits
// carry a real 5 s wall-clock deadline, which under heavy machine load fires before the
// deterministic budget a test asserts (Timeout instead of InstructionLimit). Only the deadline
// tests set deadlineMillis themselves (fake clock; one real-clock smoke test). A lint test at the
// end of this file rejects default-constructed ScriptLimits / ScriptHost.
ScriptLimits testLimits() {
    ScriptLimits l;  // lint-ok: the helper itself
    l.deadlineMillis = 0;
    return l;
}

ScriptLimits smallLimits() {
    ScriptLimits l = testLimits();
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
    ScriptHost host(testLimits());
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
    ScriptHost host(testLimits());
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
    ScriptHost host(testLimits());
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
        ScriptHost granted(testLimits(), g);
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
    ScriptHost host(testLimits());
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
    ScriptHost host(testLimits());
    SECTION("flag already set: nothing runs") {
        std::atomic<bool> flag{true};
        const auto r = host.run("x = 1", "t", &flag);
        CHECK(r.status == ScriptStatus::Cancelled);
        CHECK(r.instructionsUsed == 0);
    }
    SECTION("flag raised by a host function in the middle of a loop") {
        std::atomic<bool> flag{false};
        ScriptHost h2(testLimits());
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
        ScriptHost h2(testLimits());
        h2.registerFunction("", "stop", [&flag](const ScriptValues&) {
            flag.store(true);
            return ScriptValues{};
        });
        const auto r = h2.run("stop() return (('a'):rep(2000) .. 'c'):find(('.-'):rep(8) .. 'b')", "t", &flag);
        CHECK(r.status == ScriptStatus::Cancelled);
    }
    SECTION("cancel cannot be swallowed by pcall") {
        std::atomic<bool> flag{false};
        ScriptHost h2(testLimits());
        h2.registerFunction("", "stop", [&flag](const ScriptValues&) {
            flag.store(true);
            return ScriptValues{};
        });
        const auto r = h2.run("while true do pcall(function() stop() while true do end end) end", "t", &flag);
        CHECK(r.status == ScriptStatus::Cancelled);
    }
}

TEST_CASE("scripting: host functions and values", "[scripting]") {
    ScriptHost host(testLimits());
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
        const auto r = host.run("return print, {1, x = 1}", "t");
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
    ScriptLimits l = testLimits();
    l.maxMemoryBytes = 2048;
    ScriptHost host(l);
    const auto r = host.run("return 1", "t");
    CHECK(r.status == ScriptStatus::MemoryLimit);
}

TEST_CASE("scripting: hostile values crossing the boundary are budgeted", "[scripting][hostile]") {
    // Every case below used to expand to ~60^15 copies (or hang) before the element/byte budget.
    ScriptHost host(testLimits());
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
        ScriptLimits l = testLimits();
        l.maxValueElements = 1000;
        ScriptHost h2(l);
        const auto r = h2.run("local t = {} for i = 1, 5000 do t[i] = i end return t", "t");
        CHECK(r.status == ScriptStatus::RuntimeError);
        CHECK(contains(r.message, "too large"));
        const auto ok = h2.run("local t = {} for i = 1, 500 do t[i] = i end return t", "t");
        CHECK(ok.ok());
    }
    SECTION("long strings inside a table") {
        ScriptLimits l = testLimits();
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
        ScriptHost h2(testLimits());
        h2.registerFunction("", "stop", [&flag](const ScriptValues&) {
            flag.store(true);
            return ScriptValues{};
        });
        const auto r = h2.run(std::string(dag) + "stop() return l", "t", &flag);
        CHECK_FALSE(r.ok());
    }
}

TEST_CASE("scripting: conversion work is charged to the instruction budget", "[scripting][hostile]") {
    // Host calls that convert big values used to cost ~4 VM instructions each however much they
    // copied, so a loop of them ran for minutes. Conversion in both directions is now paid for.
    ScriptLimits l = testLimits();
    l.maxInstructions = 2'000'000;
    ScriptHost host(l);
    std::size_t calls = 0;
    host.registerFunction("", "echo", [&calls](const ScriptValues& a) {
        ++calls;
        return a;
    });
    host.registerFunction("", "sink", [&calls](const ScriptValues&) {
        ++calls;
        return ScriptValues{};
    });
    host.registerFunction("", "big", [&calls](const ScriptValues&) {
        ++calls;
        ScriptValue::Array a(50'000, ScriptValue(std::int64_t{7}));
        return ScriptValues{ScriptValue(std::move(a))};
    });
    const std::string mk = "local t = {} for i = 1, 20000 do t[i] = i end ";
    SECTION("arguments") {
        const auto r = host.run(mk + "for i = 1, 1000 do sink(t) end return 1", "t");
        CHECK(r.status == ScriptStatus::InstructionLimit);
        CHECK(calls < 20);
    }
    SECTION("arguments and results") {
        const auto r = host.run(mk + "for i = 1, 1000 do echo(t) end return 1", "t");
        CHECK(r.status == ScriptStatus::InstructionLimit);
        CHECK(calls < 20);
    }
    SECTION("results only") {
        const auto r = host.run("for i = 1, 1000 do big() end return 1", "t");
        CHECK(r.status == ScriptStatus::InstructionLimit);
        CHECK(calls < 20);
    }
    SECTION("a script cannot swallow the budget error and go on") {
        const auto r = host.run(mk + "for i = 1, 1000 do pcall(sink, t) end return 1", "t");
        CHECK(r.status == ScriptStatus::InstructionLimit);
        CHECK(calls < 20);
    }
    SECTION("rejected values cost too") {
        const auto r = host.run(
            "local l = {1} for i = 1, 15 do local n = {} for j = 1, 60 do n[j] = l end l = n end "
            "for i = 1, 1000 do pcall(sink, l) end return 1",
            "t");
        CHECK(r.status == ScriptStatus::InstructionLimit);
        CHECK(calls == 0);
    }
    SECTION("the returned value is charged after the run") {
        ScriptLimits l2 = testLimits();
        l2.maxInstructions = 100'000;
        ScriptHost h2(l2);
        const auto r = h2.run("local t = {} for i = 1, 8000 do t[i] = i end return t", "t");
        CHECK(r.status == ScriptStatus::InstructionLimit);
        CHECK(r.returned.empty());
        const auto ok = h2.run("local t = {} for i = 1, 500 do t[i] = i end return t", "t");
        REQUIRE(ok.ok());
        CHECK(ok.instructionsUsed >= 500 * 8);
    }
    SECTION("honest use is still cheap") {
        const auto r = host.run("local t = {1, 2, 3} for i = 1, 100 do t = echo(t) end return t", "t");
        CHECK(r.ok());
    }
}

TEST_CASE("scripting: host function throwing a non-std exception is contained", "[scripting]") {
    ScriptHost host(testLimits());
    host.registerFunction("", "weird", [](const ScriptValues&) -> ScriptValues { throw 42; });
    const auto r = host.run("local ok, e = pcall(weird) return ok, e", "t");
    REQUIRE(r.ok());
    CHECK_FALSE(r.returned.at(0).asBool());
    const auto r2 = host.run("weird()", "t");
    CHECK(r2.status == ScriptStatus::RuntimeError);
}

TEST_CASE("scripting: string functions on huge operands are charged", "[scripting][hostile]") {
    // 1000 upper() calls on a 10 MB string must hit the instruction budget quickly.
    ScriptLimits l = testLimits();
    l.maxStringBytes = 16u * 1024u * 1024u;
    l.maxAllocationBytes = 32u * 1024u * 1024u;
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

namespace {

// A fake monotonic clock: every reading advances time by `stepNs`; `reads` counts the polls.
struct FakeClock {
    std::uint64_t now = 1'000'000;
    std::uint64_t stepNs = 1'000'000;  // 1 ms per reading
    std::uint64_t reads = 0;
    std::function<std::uint64_t()> fn() {
        return [this]() {
            ++reads;
            now += stepNs;
            return now;
        };
    }
};

ScriptLimits hugeBudgets(FakeClock& clock, std::uint64_t deadlineMs) {
    ScriptLimits l = testLimits();
    l.maxInstructions = ~std::uint64_t{0} / 4;
    l.maxPatternSteps = ~std::uint64_t{0} / 4;
    l.maxValueElements = 1u << 24;
    l.maxValueBytes = 1u << 28;
    l.deadlineMillis = deadlineMs;
    l.clockNanos = clock.fn();
    return l;
}

}  // namespace

TEST_CASE("scripting: wall-clock deadline backstop (fake clock)", "[scripting][hostile][deadline]") {
    FakeClock clock;
    SECTION("an endless loop ends with Timeout, not the instruction limit") {
        ScriptHost host(hugeBudgets(clock, 50));
        const auto r = host.run("while true do end", "t");
        CHECK(r.status == ScriptStatus::Timeout);
        CHECK(contains(r.message, "timed out"));
        CHECK(r.instructionsUsed < 100'000);  // a fake ms per poll: ~50 polls of 100 instructions
    }
    SECTION("pcall cannot swallow the timeout") {
        ScriptHost host(hugeBudgets(clock, 20));
        const auto r = host.run("while true do pcall(function() while true do end end) end", "t");
        CHECK(r.status == ScriptStatus::Timeout);
    }
    SECTION("it is honoured inside a coroutine") {
        ScriptHost host(hugeBudgets(clock, 20));
        const auto r = host.run("local f = coroutine.wrap(function() while true do end end) f()", "t");
        CHECK(r.status == ScriptStatus::Timeout);
    }
    SECTION("it is polled in the pattern matcher") {
        ScriptHost host(hugeBudgets(clock, 5));
        const auto r = host.run("return (('a'):rep(2000) .. 'c'):find(('.-'):rep(8) .. 'b')", "t");
        CHECK(r.status == ScriptStatus::Timeout);
        CHECK(r.patternStepsUsed < 100'000);
    }
    SECTION("it is polled while a host result is converted into Lua") {
        ScriptHost host(hugeBudgets(clock, 20));
        host.registerFunction("", "big", [](const ScriptValues&) {
            return ScriptValues{ScriptValue(ScriptValue::Array(300'000, ScriptValue(1)))};
        });
        const auto r = host.run("local t = big() return 1", "t");
        CHECK(r.status == ScriptStatus::Timeout);
        CHECK(clock.reads < 40);  // the boundary poll (every 1024 values) ended it, not 300k values
    }
    SECTION("it is polled while a script table is converted for the host") {
        ScriptHost host(hugeBudgets(clock, 20));
        host.registerFunction("", "sink", [](const ScriptValues&) { return ScriptValues{}; });
        // Build a table without many VM instructions: table.move copies in C.
        const auto r = host.run(
            "local t = {} for i = 1, 100 do t[i] = i end "
            "local u = table.move(t, 1, 100, 1, {}) "
            "local w = {} for i = 1, 3000 do w[i] = u end "
            "local ok = pcall(sink, w) return 1",
            "t");
        CHECK(r.status == ScriptStatus::Timeout);
    }
    SECTION("it is polled while the returned values are converted") {
        ScriptHost host(hugeBudgets(clock, 5));
        host.registerFunction("", "big", [](const ScriptValues&) {
            return ScriptValues{ScriptValue(ScriptValue::Array(1500, ScriptValue(1)))};
        });
        const auto r = host.run("local t = big() local u = {} for i = 1, 200 do u[i] = t end return u", "t");
        CHECK(r.status == ScriptStatus::Timeout);
    }
    SECTION("deadlineMillis = 0 disables it") {
        FakeClock fast;
        fast.stepNs = 1'000'000'000;
        ScriptLimits l = hugeBudgets(fast, 0);
        l.maxInstructions = 100'000;
        ScriptHost host(l);
        const auto r = host.run("while true do end", "t");
        CHECK(r.status == ScriptStatus::InstructionLimit);
    }
    SECTION("a run that finishes in time is unaffected, and the host stays usable after a timeout") {
        ScriptHost host(hugeBudgets(clock, 50));
        CHECK(host.run("while true do end", "t").status == ScriptStatus::Timeout);
        clock.now = 1'000'000;
        const auto r = host.run("return 40 + 2", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asInt() == 42);
    }
    SECTION("the deterministic budgets still win when they come first") {
        ScriptLimits l = hugeBudgets(clock, 1'000'000);
        l.maxInstructions = 10'000;
        ScriptHost host(l);
        CHECK(host.run("while true do end", "t").status == ScriptStatus::InstructionLimit);
    }
}

TEST_CASE("scripting: per-operation caps keep single C-side operations short", "[scripting][hostile][caps]") {
    ScriptLimits l = testLimits();
    l.maxStringBytes = 1u << 20;
    ScriptHost host(l);
    SECTION("concatenation cannot build a string above maxStringBytes") {
        const auto r = host.run(
            "local a = string.rep('x', 700000) local b = string.rep('y', 700000) return #(a .. b)", "t");
        CHECK_FALSE(r.ok());
        CHECK(r.status == ScriptStatus::MemoryLimit);
    }
    SECTION("repeated concatenation of big strings fails at once") {
        const auto r = host.run(
            "local a = string.rep('x', 700000) for i = 1, 1e7 do local c = a .. a end", "t");
        CHECK(r.status == ScriptStatus::MemoryLimit);
        CHECK(r.instructionsUsed < 200'000);
    }
    SECTION("table.concat result is capped too") {
        const auto r = host.run(
            "local s = string.rep('x', 500000) local t = {s, s, s} return #table.concat(t)", "t");
        CHECK(r.status == ScriptStatus::MemoryLimit);
    }
    SECTION("an error message cannot exceed the cap") {
        const auto r = host.run(
            "local s = string.rep('x', 900000) return pcall(error, s .. s)", "t");
        CHECK(r.status == ScriptStatus::MemoryLimit);
    }
    SECTION("a single block above maxAllocationBytes is refused") {
        ScriptLimits l2 = testLimits();
        l2.maxAllocationBytes = 64u * 1024u;
        ScriptHost h2(l2);
        const auto r = h2.run("local t = {} for i = 1, 100000 do t[i] = i end return #t", "t");
        CHECK(r.status == ScriptStatus::MemoryLimit);
    }
    SECTION("strings and tables below the caps still work") {
        const auto r = host.run(
            "local a = string.rep('x', 400000) local b = a .. a local t = {} "
            "for i = 1, 50000 do t[i] = i end return #b, #t", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asInt() == 800000);
        CHECK(r.returned.at(1).asInt() == 50000);
    }
    SECTION("load refuses a chunk above maxLoadBytes") {
        ScriptLimits l2 = testLimits();
        l2.maxLoadBytes = 1000;
        ScriptHost h2(l2);
        const auto r = h2.run(
            "local f, e = load('x = 1 ' .. string.rep(' ', 2000)) return f == nil, e", "t");
        REQUIRE(r.ok());
        CHECK(r.returned.at(0).asBool());
        CHECK(contains(r.returned.at(1).asString(), "too large"));
        const auto ok = h2.run("local f = load('return 7') return f()", "t");
        REQUIRE(ok.ok());
        CHECK(ok.returned.at(0).asInt() == 7);
    }
}

TEST_CASE("scripting: real-clock deadline smoke test", "[scripting][hostile][deadline]") {
    ScriptLimits l = testLimits();
    l.maxInstructions = ~std::uint64_t{0} / 4;
    l.deadlineMillis = 150;
    ScriptHost host(l);
    const auto t0 = std::chrono::steady_clock::now();
    const auto r = host.run("local n = 0 while true do n = n + 1 end", "t");
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    CHECK(r.status == ScriptStatus::Timeout);
    CHECK(ms >= 100);
    CHECK(ms < 20'000);  // generous: only proves the deadline ends the run
}

// Guard: scripting tests must not run with default limits (real 5 s wall-clock deadline).
TEST_CASE("scripting: tests never use default ScriptLimits", "[scripting][lint]") {
    namespace fs = std::filesystem;
    const std::regex bad(R"(ScriptLimits\s*\{\s*\}|ScriptLimits\s+\w+\s*;|ScriptHost\s+\w+\s*;|ScriptHost\s*\w*\s*\(\s*\)|ScriptHost\s+\w+\s*\(\s*\{\s*\})");
    int scanned = 0;
    std::string offences;
    for (const auto& e : fs::directory_iterator(CARVETTO_SCRIPTING_TEST_DIR)) {
        if (e.path().extension() != ".cpp") continue;
        ++scanned;
        std::ifstream in(e.path());
        std::string line;
        int n = 0;
        while (std::getline(in, line)) {
            ++n;
            if (line.find("lint-ok") != std::string::npos) continue;
            if (std::regex_search(line, bad))
                offences += e.path().filename().string() + ":" + std::to_string(n) + ": " + line + "\n";
        }
    }
    CHECK(scanned >= 1);
    INFO("Use testLimits() (deadlineMillis = 0) instead of default limits:\n" << offences);
    CHECK(offences.empty());
}

// ---- T-0331: string-keyed tables, dotted modules, oversized host results ---------------------------

TEST_CASE("scripting: string-keyed tables cross the boundary both ways", "[scripting][t0331]") {
    ScriptHost host(testLimits());
    host.registerFunction("", "echo", [](const ScriptValues& a) { return a; });
    SECTION("a nested map comes back equal, keys sorted when read") {
        const auto r = host.run("local r = echo({b = 1, a = {x = 2.5, y = 'z'}}) return r.b, r.a.x, r.a.y", "t");
        REQUIRE(r.ok());
        REQUIRE(r.returned.size() == 3);
        CHECK(r.returned[0].asInt() == 1);
        CHECK(r.returned[1].asNumber() == 2.5);
        CHECK(r.returned[2].asString() == "z");
    }
    SECTION("a returned map is read as a Map with keys in sorted order") {
        const auto r = host.run("return {zeta = 1, alpha = true, mid = 'm'}", "t");
        REQUIRE(r.ok());
        REQUIRE(r.returned.size() == 1);
        REQUIRE(r.returned[0].isMap());
        const auto& items = r.returned[0].asMap().items;
        REQUIRE(items.size() == 3);
        CHECK(items[0].first == "alpha");
        CHECK(items[1].first == "mid");
        CHECK(items[2].first == "zeta");
        CHECK(r.returned[0].asMap().find("mid")->asString() == "m");
        CHECK(r.returned[0].asMap().find("nope") == nullptr);
    }
    SECTION("an empty table is still an empty array") {
        const auto r = host.run("return {}", "t");
        REQUIRE(r.ok());
        CHECK(r.returned[0].isArray());
    }
    SECTION("mixed and non-string keys are refused") {
        CHECK_FALSE(host.run("echo({1, 2, x = 3})", "t").ok());
        CHECK_FALSE(host.run("echo({[1.5] = 1})", "t").ok());
        CHECK_FALSE(host.run("echo({[true] = 1})", "t").ok());
        CHECK_FALSE(host.run("echo({[2] = 1})", "t").ok());
        const auto r = host.run("local t = {a = 1} t[1] = 2 return echo(t)", "t");
        CHECK_FALSE(r.ok());
    }
    SECTION("a host function can build a map for the script") {
        host.registerFunction("", "mk", [](const ScriptValues&) {
            ScriptValue::Map m;
            m.set("n", ScriptValue(3));
            m.set("n", ScriptValue(4));  // the last value wins
            m.set("s", ScriptValue("hi"));
            return ScriptValues{ScriptValue(std::move(m))};
        });
        const auto r = host.run("local m = mk() return m.n, m.s", "t");
        REQUIRE(r.ok());
        CHECK(r.returned[0].asInt() == 4);
        CHECK(r.returned[1].asString() == "hi");
    }
    SECTION("depth is limited for maps too") {
        ScriptLimits l = testLimits();
        l.maxValueDepth = 3;
        ScriptHost shallow(l);
        shallow.registerFunction("", "echo", [](const ScriptValues& a) { return a; });
        CHECK_FALSE(shallow.run("echo({a = {b = {c = {d = 1}}}})", "t").ok());
        CHECK(shallow.run("echo({a = {b = 1}})", "t").ok());
    }
    SECTION("a wide map is charged to the budget, a shared one cannot blow up") {
        ScriptLimits l = testLimits();
        l.maxInstructions = 100'000;
        ScriptHost small(l);
        small.registerFunction("", "echo", [](const ScriptValues& a) { return a; });
        const auto r = small.run(
            "local t = {x = 1} for i = 1, 20 do t = {a = t, b = t} end while true do echo(t) end", "t");
        CHECK_FALSE(r.ok());
    }
}

TEST_CASE("scripting: dotted module names create nested tables", "[scripting][t0331]") {
    ScriptHost host(testLimits());
    host.registerFunction("a.b", "f", [](const ScriptValues&) { return ScriptValues{ScriptValue(7)}; });
    host.registerFunction("a", "g", [](const ScriptValues&) { return ScriptValues{ScriptValue(8)}; });
    host.setValue("a.b", "k", ScriptValue(9));
    const auto r = host.run("return a.b.f(), a.g(), a.b.k", "t");
    REQUIRE(r.ok());
    CHECK(r.returned[0].asInt() == 7);
    CHECK(r.returned[1].asInt() == 8);
    CHECK(r.returned[2].asInt() == 9);
}

TEST_CASE("scripting: an oversized host result says 'value too large'", "[scripting][t0331]") {
    ScriptLimits l = testLimits();
    l.maxStringBytes = 64u * 1024u;
    l.maxAllocationBytes = 256u * 1024u;
    ScriptHost host(l);
    host.registerFunction("", "bigString", [](const ScriptValues&) {
        return ScriptValues{ScriptValue(std::string(200u * 1024u, 'x'))};
    });
    host.registerFunction("", "bigArray", [](const ScriptValues&) {
        ScriptValue::Array a(100'000, ScriptValue(1.5));
        return ScriptValues{ScriptValue(std::move(a))};
    });
    SECTION("a string over the string cap") {
        const auto r = host.run("return pcall(bigString)", "t");
        REQUIRE(r.ok());
        REQUIRE(r.returned.size() == 2);
        CHECK_FALSE(r.returned[0].asBool());
        CHECK(contains(r.returned[1].asString(), "value too large"));
        CHECK_FALSE(contains(r.returned[1].asString(), "unknown exception"));
    }
    SECTION("an array over the block cap") {
        const auto r = host.run("return pcall(bigArray)", "t");
        REQUIRE(r.ok());
        CHECK_FALSE(r.returned[0].asBool());
        CHECK(contains(r.returned[1].asString(), "value too large"));
    }
    SECTION("uncaught, the error names the call and the line") {
        const auto r = host.run("\nlocal x = bigString()", "g.lua");
        CHECK_FALSE(r.ok());
        CHECK(contains(r.message, "value too large"));
        CHECK(contains(r.message, "g.lua:2:"));
    }
    SECTION("the host is usable afterwards") {
        host.run("bigString()", "t");
        CHECK(host.run("return 1", "t").ok());
    }
}
