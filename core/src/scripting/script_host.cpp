#include "carvetto/scripting/script_host.hpp"

// Lua is compiled as C++ (core/CMakeLists.txt, ADR-0101): errors are C++ exceptions, so the
// callbacks below use RAII freely. This is the ONLY file that includes the Lua headers.
#include <lauxlib.h>
#include <lua.h>
#include <lualib.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <memory>
#include <new>
#include <utility>

namespace carvetto::scripting {

double ScriptValue::asNumber() const {
    if (isInt()) {
        return static_cast<double>(std::get<std::int64_t>(v));
    }
    return std::get<double>(v);
}

}  // namespace carvetto::scripting

namespace {

using carvetto::scripting::HostFunction;
using carvetto::scripting::ScriptError;
using carvetto::scripting::ScriptGrants;
using carvetto::scripting::ScriptLimits;
using carvetto::scripting::ScriptValue;
using carvetto::scripting::ScriptValues;

constexpr int kBlock = 1000;  // VM instructions per hook call

// Everything one run owns. It is the allocator's user data, so every callback finds it from any
// lua_State (main thread or coroutine) with lua_getallocf.
struct RunCtx {
    const ScriptLimits* lim = nullptr;
    const std::atomic<bool>* cancel = nullptr;
    std::size_t used = 0;
    std::size_t peak = 0;
    bool memFailed = false;
    std::uint64_t instr = 0;
    std::uint64_t patternSteps = 0;
    std::size_t outBytes = 0;
    bool tripInstr = false;
    bool tripCancel = false;
    bool tripOutput = false;
    const char* tripText = "";
    std::vector<std::string>* output = nullptr;
};

RunCtx* ctxOf(lua_State* L) {
    void* ud = nullptr;
    lua_getallocf(L, &ud);
    return static_cast<RunCtx*>(ud);
}

void* allocator(void* ud, void* ptr, std::size_t osize, std::size_t nsize) {
    auto* c = static_cast<RunCtx*>(ud);
    if (ptr == nullptr) {
        osize = 0;  // then osize is a type tag, not a size
    }
    if (nsize == 0) {
        if (ptr != nullptr) {
            c->used -= osize;
            std::free(ptr);
        }
        return nullptr;
    }
    if (nsize > osize) {
        const std::size_t grow = nsize - osize;
        if (grow > c->lim->maxMemoryBytes || c->used > c->lim->maxMemoryBytes - grow) {
            c->memFailed = true;
            return nullptr;
        }
    }
    void* p = std::realloc(ptr, nsize);
    if (p == nullptr) {
        return nullptr;
    }
    c->used = c->used - osize + nsize;
    if (c->used > c->peak) {
        c->peak = c->used;
    }
    return p;
}

void budgetHook(lua_State* L, lua_Debug* ar);

// After a trip every instruction raises: a script that swallows the budget error with pcall is
// stopped again by the next instruction it runs outside that pcall.
void tripAll(lua_State* L) {
    lua_sethook(L, budgetHook, LUA_MASKCOUNT, 1);
}

[[noreturn]] void raiseTrip(lua_State* L, RunCtx* c) {
    luaL_error(L, "%s", c->tripText);
    std::abort();  // unreachable: luaL_error throws
}

void checkBudget(lua_State* L, RunCtx* c) {
    if (c->tripCancel || c->tripInstr) {
        raiseTrip(L, c);
    }
    if (c->cancel != nullptr && c->cancel->load(std::memory_order_relaxed)) {
        c->tripCancel = true;
        c->tripText = "script cancelled";
        tripAll(L);
        raiseTrip(L, c);
    }
    if (c->instr >= c->lim->maxInstructions) {
        c->tripInstr = true;
        c->tripText = "instruction budget exceeded";
        tripAll(L);
        raiseTrip(L, c);
    }
}

void budgetHook(lua_State* L, lua_Debug*) {
    RunCtx* c = ctxOf(L);
    c->instr += kBlock;
    checkBudget(L, c);
}

// Charges `units` of work done inside C library code (one unit ~ one VM instruction).
void charge(lua_State* L, std::uint64_t units) {
    RunCtx* c = ctxOf(L);
    c->instr += units;
    checkBudget(L, c);
}

}  // namespace

// Called from the patched pattern matcher (lstrlib.c) on every recursive match step.
int carvettoLuaMatchStep(lua_State* L);
int carvettoLuaMatchStep(lua_State* L) {
    RunCtx* c = ctxOf(L);
    if (c == nullptr || c->lim == nullptr) {
        return 0;
    }
    if (c->tripCancel || c->tripInstr) {
        raiseTrip(L, c);
    }
    ++c->patternSteps;
    if (c->patternSteps > c->lim->maxPatternSteps) {
        c->tripInstr = true;
        c->tripText = "pattern-step budget exceeded";
        tripAll(L);
        raiseTrip(L, c);
    }
    if ((c->patternSteps & 255u) == 0 && c->cancel != nullptr &&
        c->cancel->load(std::memory_order_relaxed)) {
        c->tripCancel = true;
        c->tripText = "script cancelled";
        tripAll(L);
        raiseTrip(L, c);
    }
    return 0;
}

namespace {

// ---- value conversion ------------------------------------------------------------------------

// Work budget of one conversion Lua -> host. A table that is a shared DAG expands to exponentially
// many copies, so the total number of values and bytes is bounded, not only the depth.
constexpr const char* kBudgetWhy = "instruction budget exceeded";
constexpr std::uint64_t kUnitsPerValue = 8;  // budget units per value visited, either direction
constexpr std::uint64_t kUnitsPerEntry = 4;  // per table entry walked while checking the shape

std::uint64_t remainingUnits(const RunCtx* c) {
    return c->instr >= c->lim->maxInstructions ? 0 : c->lim->maxInstructions - c->instr;
}

struct Conv {
    int maxDepth = 0;
    std::uint64_t elementsLeft = 0;
    std::uint64_t bytesLeft = 0;
    const std::atomic<bool>* cancel = nullptr;
    std::uint64_t polled = 0;
    const char* why = nullptr;  // set when the conversion fails for a budget reason
    // Instruction-budget units this conversion may still spend (the run's remaining budget) and
    // has spent. The caller adds `unitsUsed` to the run's counter, so conversion work is paid for
    // like VM work and one conversion cannot outlast the budget.
    std::uint64_t unitsLeft = 0;
    std::uint64_t unitsUsed = 0;

    bool spend(std::uint64_t u) {
        if (u > unitsLeft) {
            why = kBudgetWhy;
            return false;
        }
        unitsLeft -= u;
        unitsUsed += u;
        return true;
    }
};

// Never raises a Lua error (it also runs outside any protected call); returns false for a value
// that cannot cross: functions, userdata, threads, tables that are not sequences, too deep, or
// beyond the element/byte budget (then `cv.why` says so).
bool toValue(lua_State* L, int idx, int depth, Conv& cv, ScriptValue& out) {
    idx = lua_absindex(L, idx);
    if (cv.elementsLeft == 0) {
        cv.why = "value too large";
        return false;
    }
    --cv.elementsLeft;
    if (!cv.spend(kUnitsPerValue)) {
        return false;
    }
    if ((++cv.polled & 1023u) == 0 && cv.cancel != nullptr &&
        cv.cancel->load(std::memory_order_relaxed)) {
        cv.why = "script cancelled";
        return false;
    }
    switch (lua_type(L, idx)) {
        case LUA_TNIL:
        case LUA_TNONE:
            out = ScriptValue();
            return true;
        case LUA_TBOOLEAN:
            out = ScriptValue(lua_toboolean(L, idx) != 0);
            return true;
        case LUA_TNUMBER:
            if (lua_isinteger(L, idx) != 0) {
                out = ScriptValue(static_cast<std::int64_t>(lua_tointeger(L, idx)));
            } else {
                out = ScriptValue(static_cast<double>(lua_tonumber(L, idx)));
            }
            return true;
        case LUA_TSTRING: {
            std::size_t n = 0;
            const char* s = lua_tolstring(L, idx, &n);
            if (n > cv.bytesLeft) {
                cv.why = "value too large";
                return false;
            }
            cv.bytesLeft -= n;
            if (!cv.spend(n / 8)) {
                return false;
            }
            out = ScriptValue(std::string(s, n));
            return true;
        }
        case LUA_TTABLE: {
            if (depth >= cv.maxDepth || lua_checkstack(L, 6) == 0) {
                return false;
            }
            const lua_Unsigned n = lua_rawlen(L, idx);
            if (n > cv.elementsLeft) {
                cv.why = "value too large";
                return false;
            }
            lua_Unsigned count = 0;
            lua_pushnil(L);
            while (lua_next(L, idx) != 0) {
                ++count;
                lua_pop(L, 1);
                if (!cv.spend(kUnitsPerEntry)) {
                    lua_pop(L, 1);  // the key
                    return false;
                }
                if (count > n) {
                    lua_pop(L, 1);  // the key
                    return false;   // has keys outside the sequence
                }
            }
            if (count != n) {
                return false;
            }
            ScriptValue::Array arr;
            arr.reserve(static_cast<std::size_t>(n));
            for (lua_Unsigned i = 1; i <= n; ++i) {
                lua_rawgeti(L, idx, static_cast<lua_Integer>(i));
                ScriptValue e;
                const bool ok = toValue(L, -1, depth + 1, cv, e);
                lua_pop(L, 1);
                if (!ok) {
                    return false;
                }
                arr.push_back(std::move(e));
            }
            out = ScriptValue(std::move(arr));
            return true;
        }
        default:
            return false;
    }
}

// May raise a Lua error (memory) or throw ScriptError: only called inside protected calls.
// Work done while pushing host results into Lua. Like Conv it is paid from the run's budget; when
// it runs out the run's trip flags are set (so the next instruction raises even if the script
// swallows this error) and ScriptError is thrown.
struct PushCharge {
    lua_State* L = nullptr;
    RunCtx* rc = nullptr;
    std::uint64_t left = 0;
    std::uint64_t used = 0;
    std::uint64_t polled = 0;

    void spend(std::uint64_t u) {
        if (u > left) {
            rc->instr = rc->lim->maxInstructions;
            rc->tripInstr = true;
            rc->tripText = kBudgetWhy;
            tripAll(L);
            throw ScriptError{kBudgetWhy};
        }
        left -= u;
        used += u;
        if ((++polled & 1023u) == 0 && rc->cancel != nullptr &&
            rc->cancel->load(std::memory_order_relaxed)) {
            rc->tripCancel = true;
            rc->tripText = "script cancelled";
            tripAll(L);
            throw ScriptError{"script cancelled"};
        }
    }
};

void pushValue(lua_State* L, const ScriptValue& v, int depth, int maxDepth, PushCharge* pc) {
    luaL_checkstack(L, 4, "too many values");
    if (pc != nullptr) {
        pc->spend(kUnitsPerValue);
    }
    if (v.isNil()) {
        lua_pushnil(L);
    } else if (v.isBool()) {
        lua_pushboolean(L, v.asBool() ? 1 : 0);
    } else if (v.isInt()) {
        lua_pushinteger(L, static_cast<lua_Integer>(v.asInt()));
    } else if (v.isNumber()) {
        lua_pushnumber(L, static_cast<lua_Number>(v.asNumber()));
    } else if (v.isString()) {
        const std::string& s = v.asString();
        if (pc != nullptr) {
            pc->spend(s.size() / 8);
        }
        lua_pushlstring(L, s.data(), s.size());
    } else {
        if (depth >= maxDepth) {
            throw ScriptError{"value nested too deeply"};
        }
        const auto& a = v.asArray();
        lua_createtable(L, static_cast<int>(a.size()), 0);
        lua_Integer i = 1;
        for (const ScriptValue& e : a) {
            pushValue(L, e, depth + 1, maxDepth, pc);
            lua_rawseti(L, -2, i++);
        }
    }
}

// ---- host function trampoline ----------------------------------------------------------------

// Returns the number of results, or -1 with `err` set.
int hostCallImpl(lua_State* L, std::string& err) {
    const auto* fn = static_cast<const HostFunction*>(lua_touserdata(L, lua_upvalueindex(1)));
    RunCtx* rc = ctxOf(L);
    const int maxDepth = rc->lim->maxValueDepth;
    Conv cv;
    cv.maxDepth = maxDepth;
    cv.elementsLeft = rc->lim->maxValueElements;
    cv.bytesLeft = rc->lim->maxValueBytes;
    cv.cancel = rc->cancel;
    cv.unitsLeft = remainingUnits(rc);
    ScriptValues args;
    const int n = lua_gettop(L);
    args.reserve(static_cast<std::size_t>(n));
    for (int i = 1; i <= n; ++i) {
        ScriptValue a;
        if (!toValue(L, i, 0, cv, a)) {
            rc->instr += cv.unitsUsed;
            if (cv.why != nullptr && std::strcmp(cv.why, kBudgetWhy) == 0) {
                rc->instr = std::max(rc->instr, rc->lim->maxInstructions);
                checkBudget(L, rc);  // trips and raises
            }
            if (cv.why != nullptr) {
                if (rc->cancel != nullptr && std::string(cv.why) == "script cancelled") {
                    rc->tripCancel = true;
                    rc->tripText = "script cancelled";
                    tripAll(L);
                    raiseTrip(L, rc);
                }
                err = std::string("bad argument #") + std::to_string(i) + ": " + cv.why;
                return -1;
            }
            err = "bad argument #" + std::to_string(i) +
                  ": only nil, boolean, number, string and arrays of those can be passed";
            return -1;
        }
        args.push_back(std::move(a));
    }
    rc->instr += cv.unitsUsed;
    checkBudget(L, rc);
    ScriptValues results;
    bool failed = true;
    PushCharge pc;
    pc.L = L;
    pc.rc = rc;
    pc.left = remainingUnits(rc);
    try {
        results = (*fn)(args);
        for (const ScriptValue& r : results) {
            pushValue(L, r, 0, maxDepth, &pc);
        }
        failed = false;
    } catch (const ScriptError& e) {
        err = e.message;
    } catch (const std::exception& e) {
        err = e.what();
    } catch (...) {
        err = "host function threw an unknown exception";
    }
    rc->instr += pc.used;
    if (failed) {
        return -1;
    }
    checkBudget(L, rc);
    return static_cast<int>(results.size());
}

int hostCall(lua_State* L) {
    std::string err;
    const int r = hostCallImpl(L, err);
    if (r < 0) {
        return luaL_error(L, "%s", err.c_str());
    }
    return r;
}

// ---- library functions -----------------------------------------------------------------------

int luaPrint(lua_State* L) {
    RunCtx* c = ctxOf(L);
    const int n = lua_gettop(L);
    std::string line;
    for (int i = 1; i <= n; ++i) {
        std::size_t l = 0;
        const char* s = luaL_tolstring(L, i, &l);
        if (i > 1) {
            line.push_back('\t');
        }
        line.append(s, l);
        lua_pop(L, 1);
        charge(L, l / 8 + 1);
    }
    if (line.size() + 1 > c->lim->maxOutputBytes - c->outBytes ||
        c->outBytes > c->lim->maxOutputBytes) {
        c->tripOutput = true;
        return luaL_error(L, "output limit exceeded");
    }
    c->outBytes += line.size() + 1;
    c->output->push_back(std::move(line));
    return 0;
}

int luaCollectGarbage(lua_State* L) {
    const char* opt = luaL_optstring(L, 1, "collect");
    const std::string o(opt);
    if (o == "count") {
        lua_pushnumber(L, static_cast<lua_Number>(lua_gc(L, LUA_GCCOUNT)) +
                              static_cast<lua_Number>(lua_gc(L, LUA_GCCOUNTB)) / 1024.0);
        return 1;
    }
    if (o == "collect" || o == "step") {
        lua_pushinteger(L, 0);  // the collector runs by itself; a forced full cycle is not offered
        return 1;
    }
    return luaL_error(L, "collectgarbage: only \"count\" and \"collect\" are available");
}

// load(chunk [, chunkname [, mode [, env]]]): text chunks only. Binary chunks are a sandbox escape.
int luaSafeLoad(lua_State* L) {
    if (lua_type(L, 1) != LUA_TSTRING) {
        return luaL_error(L, "load: only a string chunk is accepted");
    }
    std::size_t len = 0;
    const char* s = lua_tolstring(L, 1, &len);
    const char* name = luaL_optstring(L, 2, s);
    const bool hasEnv = lua_gettop(L) >= 4;
    charge(L, len / 8 + 1);
    if (luaL_loadbufferx(L, s, len, name, "t") != LUA_OK) {
        lua_pushnil(L);
        lua_insert(L, -2);
        return 2;
    }
    if (hasEnv) {
        lua_pushvalue(L, 4);
        if (lua_setupvalue(L, -2, 1) == nullptr) {
            lua_pop(L, 1);
        }
    }
    return 1;
}

// setmetatable that refuses __gc: a finalizer runs at lua_close without the instruction hook.
int luaSafeSetMetatable(lua_State* L) {
    if (lua_type(L, 2) == LUA_TTABLE) {
        lua_pushliteral(L, "__gc");
        lua_rawget(L, 2);
        const bool hasGc = !lua_isnil(L, -1);
        lua_pop(L, 1);
        if (hasGc) {
            return luaL_error(L, "__gc metamethods are not available");
        }
    }
    const int n = lua_gettop(L);
    lua_pushvalue(L, lua_upvalueindex(1));
    lua_insert(L, 1);
    lua_call(L, n, LUA_MULTRET);
    return lua_gettop(L);
}

// string.rep with a size limit: this is the one amplifier a short script can use to build a huge
// string in one call.
int luaSafeRep(lua_State* L) {
    std::size_t len = 0;
    std::size_t seplen = 0;
    (void)luaL_checklstring(L, 1, &len);
    const lua_Integer n = luaL_checkinteger(L, 2);
    (void)luaL_optlstring(L, 3, "", &seplen);
    const std::uint64_t maxBytes = ctxOf(L)->lim->maxStringBytes;
    if (n > 0) {
        const std::uint64_t unit = static_cast<std::uint64_t>(len) + seplen;
        if (unit > 0 && static_cast<std::uint64_t>(n) > (maxBytes + seplen) / unit) {
            return luaL_error(L, "resulting string too large");
        }
        charge(L, (static_cast<std::uint64_t>(n) * unit) / 8 + static_cast<std::uint64_t>(n) / 16);
    }
    const int top = lua_gettop(L);
    lua_pushvalue(L, lua_upvalueindex(1));
    lua_insert(L, 1);
    lua_call(L, top, LUA_MULTRET);
    return lua_gettop(L);
}

// table.move: the copy loop runs in C.
int luaSafeMove(lua_State* L) {
    const lua_Integer f = luaL_checkinteger(L, 2);
    const lua_Integer e = luaL_checkinteger(L, 3);
    if (e >= f) {
        const std::uint64_t count = static_cast<std::uint64_t>(e) - static_cast<std::uint64_t>(f);
        if (count >= ctxOf(L)->lim->maxTableMoveElements) {
            return luaL_error(L, "too many elements to move");
        }
        charge(L, count + 1);
    }
    const int top = lua_gettop(L);
    lua_pushvalue(L, lua_upvalueindex(1));
    lua_insert(L, 1);
    lua_call(L, top, LUA_MULTRET);
    return lua_gettop(L);
}

// Generic wrapper for the other string and table functions: charge work proportional to the
// operands (strings: 1 unit per 8 bytes, tables: 1 unit per element, sort 16 per element).
int luaCharged(lua_State* L) {
    const int top = lua_gettop(L);
    const bool isSort = lua_toboolean(L, lua_upvalueindex(2)) != 0;
    std::uint64_t units = 1;
    for (int i = 1; i <= top; ++i) {
        const int t = lua_type(L, i);
        if (t == LUA_TSTRING) {
            units += lua_rawlen(L, i) / 8;
        } else if (t == LUA_TTABLE) {
            units += lua_rawlen(L, i) * (isSort ? 16u : 1u);
        }
    }
    charge(L, units);
    lua_pushvalue(L, lua_upvalueindex(1));
    lua_insert(L, 1);
    lua_call(L, top, LUA_MULTRET);
    return lua_gettop(L);
}

// Replaces field `name` of the table at the top of the stack by a C closure over the old value.
void wrapField(lua_State* L, const char* name, lua_CFunction fn, bool flag = false) {
    lua_getfield(L, -1, name);
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        return;
    }
    lua_pushboolean(L, flag ? 1 : 0);
    lua_pushcclosure(L, fn, 2);
    lua_setfield(L, -2, name);
}

void wrapField1(lua_State* L, const char* name, lua_CFunction fn) {
    lua_getfield(L, -1, name);
    if (lua_isnil(L, -1)) {
        lua_pop(L, 1);
        return;
    }
    lua_pushcclosure(L, fn, 1);
    lua_setfield(L, -2, name);
}

// ---- state setup -----------------------------------------------------------------------------

struct SetupArgs {
    const ScriptGrants* grants;
    const std::vector<std::pair<std::string, std::string>>* names;  // module, name per registration
    const std::vector<const HostFunction*>* fns;                    // null: a constant
    const std::vector<const ScriptValue*>* values;
};

void openLib(lua_State* L, const char* name, lua_CFunction f) {
    luaL_requiref(L, name, f, 1);
    lua_pop(L, 1);
}

int setupState(lua_State* L) {
    const auto* a = static_cast<const SetupArgs*>(lua_touserdata(L, 1));
    openLib(L, LUA_GNAME, luaopen_base);
    openLib(L, LUA_COLIBNAME, luaopen_coroutine);
    openLib(L, LUA_TABLIBNAME, luaopen_table);
    openLib(L, LUA_STRLIBNAME, luaopen_string);
    openLib(L, LUA_MATHLIBNAME, luaopen_math);
    openLib(L, LUA_UTF8LIBNAME, luaopen_utf8);
    if (a->grants->io) {
        openLib(L, LUA_IOLIBNAME, luaopen_io);
    }
    if (a->grants->os) {
        openLib(L, LUA_OSLIBNAME, luaopen_os);
    }
    if (a->grants->debug) {
        openLib(L, LUA_DBLIBNAME, luaopen_debug);
    }
    if (a->grants->package) {
        openLib(L, LUA_LOADLIBNAME, luaopen_package);
    }

    // base library: no file loading, host-controlled print/load/collectgarbage/setmetatable
    lua_pushglobaltable(L);
    lua_pushnil(L);
    lua_setfield(L, -2, "dofile");
    lua_pushnil(L);
    lua_setfield(L, -2, "loadfile");
    lua_pushcfunction(L, luaPrint);
    lua_setfield(L, -2, "print");
    lua_pushcfunction(L, luaCollectGarbage);
    lua_setfield(L, -2, "collectgarbage");
    lua_pushcfunction(L, luaSafeLoad);
    lua_setfield(L, -2, "load");
    wrapField1(L, "setmetatable", luaSafeSetMetatable);
    lua_pop(L, 1);

    // string and table: no bytecode, bounded amplifiers, work charged to the budget
    lua_getglobal(L, "string");
    lua_pushnil(L);
    lua_setfield(L, -2, "dump");
    for (const char* f : {"find", "match", "gmatch", "gsub", "format", "upper", "lower", "reverse",
                          "sub", "byte", "char", "len", "pack", "unpack"}) {
        wrapField(L, f, luaCharged);
    }
    wrapField1(L, "rep", luaSafeRep);
    lua_pop(L, 1);
    lua_getglobal(L, "table");
    for (const char* f : {"insert", "remove", "concat", "unpack"}) {
        wrapField(L, f, luaCharged);
    }
    wrapField(L, "sort", luaCharged, true);
    wrapField1(L, "move", luaSafeMove);
    lua_pop(L, 1);

    // deterministic random numbers unless the script seeds them
    lua_getglobal(L, "math");
    lua_getfield(L, -1, "randomseed");
    lua_pushinteger(L, 0);
    lua_call(L, 1, 0);
    lua_pop(L, 1);

    // host-provided names; `carvetto.api_version` first so a host may replace it
    lua_newtable(L);
    lua_pushinteger(L, 1);
    lua_setfield(L, -2, "api_version");
    lua_setglobal(L, "carvetto");
    for (std::size_t i = 0; i < a->names->size(); ++i) {
        const std::string& module = (*a->names)[i].first;
        const std::string& name = (*a->names)[i].second;
        if (module.empty()) {
            lua_pushglobaltable(L);
        } else {
            lua_getglobal(L, module.c_str());
            if (!lua_istable(L, -1)) {
                lua_pop(L, 1);
                lua_newtable(L);
                lua_pushvalue(L, -1);
                lua_setglobal(L, module.c_str());
            }
        }
        if ((*a->fns)[i] != nullptr) {
            lua_pushlightuserdata(L, const_cast<HostFunction*>((*a->fns)[i]));
            lua_pushcclosure(L, hostCall, 1);
        } else {
            pushValue(L, *(*a->values)[i], 0, ctxOf(L)->lim->maxValueDepth, nullptr);
        }
        lua_setfield(L, -2, name.c_str());
        lua_pop(L, 1);
    }
    return 0;
}

// Message handler: appends a traceback to a string error. A budget trip keeps the bare message.
int messageHandler(lua_State* L) {
    RunCtx* c = ctxOf(L);
    const char* m = lua_tostring(L, 1);
    if (m == nullptr) {
        m = lua_pushfstring(L, "(error object is a %s value)", luaL_typename(L, 1));
    }
    if (c->tripCancel || c->tripInstr || c->tripOutput) {
        lua_pushstring(L, m);
        return 1;
    }
    luaL_traceback(L, L, m, 1);
    return 1;
}

int lineFromMessage(const std::string& msg) {
    for (std::size_t i = 0; i + 2 < msg.size(); ++i) {
        if (msg[i] != ':') {
            continue;
        }
        std::size_t j = i + 1;
        long v = 0;
        while (j < msg.size() && msg[j] >= '0' && msg[j] <= '9' && v < 100000000) {
            v = v * 10 + (msg[j] - '0');
            ++j;
        }
        if (j > i + 1 && j < msg.size() && msg[j] == ':') {
            return static_cast<int>(v);
        }
    }
    return 0;
}

struct StateCloser {
    void operator()(lua_State* L) const {
        if (L != nullptr) {
            lua_close(L);
        }
    }
};

}  // namespace

namespace carvetto::scripting {

ScriptHost::ScriptHost(ScriptLimits limits, ScriptGrants grants) : limits_(limits), grants_(grants) {}
ScriptHost::~ScriptHost() = default;

void ScriptHost::registerFunction(std::string_view module, std::string_view name, HostFunction fn) {
    for (Registration& r : registrations_) {
        if (r.module == module && r.name == name) {
            r.fn = std::move(fn);
            r.value = ScriptValue();
            return;
        }
    }
    registrations_.push_back(Registration{std::string(module), std::string(name), std::move(fn), ScriptValue()});
}

void ScriptHost::setValue(std::string_view module, std::string_view name, ScriptValue value) {
    for (Registration& r : registrations_) {
        if (r.module == module && r.name == name) {
            r.fn = nullptr;
            r.value = std::move(value);
            return;
        }
    }
    registrations_.push_back(Registration{std::string(module), std::string(name), nullptr, std::move(value)});
}

ScriptResult ScriptHost::run(std::string_view source, std::string_view chunkName,
                             const std::atomic<bool>* cancel) const {
    return execute(source, chunkName, cancel, true);
}

ScriptResult ScriptHost::check(std::string_view source, std::string_view chunkName) const {
    return execute(source, chunkName, nullptr, false);
}

ScriptResult ScriptHost::execute(std::string_view source, std::string_view chunkName,
                                 const std::atomic<bool>* cancel, bool runIt) const {
    ScriptResult res;
    res.chunkName = std::string(chunkName);
    if (source.size() > limits_.maxSourceBytes) {
        res.status = ScriptStatus::LimitsInvalid;
        res.message = res.chunkName + ": script is larger than the allowed " +
                      std::to_string(limits_.maxSourceBytes) + " bytes";
        return res;
    }
    if (cancel != nullptr && cancel->load(std::memory_order_relaxed)) {
        res.status = ScriptStatus::Cancelled;
        res.message = "script cancelled";
        return res;
    }

    RunCtx ctx;
    ctx.lim = &limits_;
    ctx.cancel = cancel;
    ctx.output = &res.output;
    std::unique_ptr<lua_State, StateCloser> state(lua_newstate(allocator, &ctx));
    auto finish = [&](ScriptStatus st, std::string msg, std::string tb) {
        res.status = st;
        res.message = std::move(msg);
        res.traceback = std::move(tb);
        res.line = lineFromMessage(res.message);
        res.instructionsUsed = ctx.instr;
        res.patternStepsUsed = ctx.patternSteps;
        res.peakMemoryBytes = ctx.peak;
    };
    if (!state) {
        finish(ScriptStatus::MemoryLimit, "not enough memory to start the script", {});
        return res;
    }
    lua_State* L = state.get();

    try {
        // 1. libraries and host names, in a protected call (the state's limits already apply)
        std::vector<std::pair<std::string, std::string>> names;
        std::vector<const HostFunction*> fns;
        std::vector<const ScriptValue*> values;
        for (const Registration& r : registrations_) {
            names.emplace_back(r.module, r.name);
            fns.push_back(r.fn ? &r.fn : nullptr);
            values.push_back(&r.value);
        }
        SetupArgs setup{&grants_, &names, &fns, &values};
        lua_pushcfunction(L, setupState);
        lua_pushlightuserdata(L, &setup);
        int st = lua_pcall(L, 1, 0, 0);
        if (st != LUA_OK) {
            const char* m = lua_tostring(L, -1);
            finish(st == LUA_ERRMEM ? ScriptStatus::MemoryLimit : ScriptStatus::RuntimeError,
                   m != nullptr ? m : "script environment could not be created", {});
            return res;
        }

        // 2. compile (text only)
        const std::string name = "=" + std::string(chunkName);
        st = luaL_loadbufferx(L, source.data(), source.size(), name.c_str(), "t");
        if (st != LUA_OK) {
            const char* m = lua_tostring(L, -1);
            finish(st == LUA_ERRMEM ? ScriptStatus::MemoryLimit : ScriptStatus::SyntaxError,
                   m != nullptr ? m : "syntax error", {});
            return res;
        }
        if (!runIt) {
            finish(ScriptStatus::Ok, {}, {});
            return res;
        }

        // 3. run under the budget hook and a message handler (stack: chunk)
        lua_pushcfunction(L, messageHandler);
        lua_insert(L, 1);
        lua_sethook(L, budgetHook, LUA_MASKCOUNT, kBlock);
        st = lua_pcall(L, 0, LUA_MULTRET, 1);
        lua_sethook(L, nullptr, 0, 0);
        if (st == LUA_OK) {
            const int top = lua_gettop(L);
            Conv cv;
            cv.maxDepth = limits_.maxValueDepth;
            cv.elementsLeft = limits_.maxValueElements;
            cv.bytesLeft = limits_.maxValueBytes;
            cv.cancel = cancel;
            cv.unitsLeft = remainingUnits(&ctx);
            for (int i = 2; i <= top; ++i) {
                ScriptValue v;
                if (!toValue(L, i, 0, cv, v)) {
                    ctx.instr += cv.unitsUsed;
                    if (cv.why != nullptr) {
                        const bool cancelled = std::string(cv.why) == "script cancelled";
                        const bool budget = std::strcmp(cv.why, kBudgetWhy) == 0;
                        res.returned.clear();
                        finish(cancelled ? ScriptStatus::Cancelled
                               : budget  ? ScriptStatus::InstructionLimit
                                         : ScriptStatus::RuntimeError,
                               std::string("returned value: ") + cv.why, {});
                        return res;
                    }
                    v = ScriptValue();
                }
                res.returned.push_back(std::move(v));
            }
            ctx.instr += cv.unitsUsed;
            finish(ScriptStatus::Ok, {}, {});
            return res;
        }
        std::string msg;
        std::string tb;
        if (const char* m = lua_tostring(L, -1)) {
            msg = m;
        }
        // The handler made "message\nstack traceback:..." out of a string error.
        const std::size_t cut = msg.find("\nstack traceback:");
        if (cut != std::string::npos) {
            tb = msg.substr(cut + 1);
            msg.resize(cut);
        }
        ScriptStatus status = ScriptStatus::RuntimeError;
        if (ctx.tripCancel) {
            status = ScriptStatus::Cancelled;
        } else if (ctx.tripInstr) {
            status = ScriptStatus::InstructionLimit;
        } else if (ctx.tripOutput && msg.find("output limit exceeded") != std::string::npos) {
            status = ScriptStatus::OutputLimit;
        } else if (st == LUA_ERRMEM || (st == LUA_ERRERR && ctx.memFailed)) {
            status = ScriptStatus::MemoryLimit;
        }
        if (status == ScriptStatus::MemoryLimit && msg.empty()) {
            msg = "not enough memory";
        }
        finish(status, std::move(msg), std::move(tb));
    } catch (const std::bad_alloc&) {
        finish(ScriptStatus::MemoryLimit, "not enough memory", {});
    } catch (const std::exception& e) {
        finish(ScriptStatus::RuntimeError, e.what(), {});
    } catch (...) {
        finish(ScriptStatus::RuntimeError, "unknown exception", {});
    }
    return res;
}

}  // namespace carvetto::scripting
