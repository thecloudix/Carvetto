#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

namespace carvetto::scripting {

/// A value crossing the host/script boundary (ADR-0101, ADR-0102, ADR-0108). Lua tables map to
/// `Array` (a sequence 1..n) or `Map` (string keys only, T-0331); integers and floats stay
/// distinct. Nothing of Lua leaks into this header.
struct ScriptValue {
    using Array = std::vector<ScriptValue>;
    /// String-keyed table: entries sorted by key when read from a script (deterministic), in the
    /// given order when pushed to one; a repeated key keeps the last value.
    struct Map {
        std::vector<std::pair<std::string, ScriptValue>> items;
        /// The value stored under `key`, or null.
        const ScriptValue* find(std::string_view key) const;
        void set(std::string key, ScriptValue value);
    };
    std::variant<std::monostate, bool, std::int64_t, double, std::string, Array, Map> v;

    ScriptValue() = default;
    ScriptValue(bool b) : v(b) {}
    ScriptValue(int i) : v(static_cast<std::int64_t>(i)) {}
    ScriptValue(std::int64_t i) : v(i) {}
    ScriptValue(double d) : v(d) {}
    ScriptValue(std::string s) : v(std::move(s)) {}
    ScriptValue(const char* s) : v(std::string(s)) {}
    ScriptValue(Array a) : v(std::move(a)) {}
    ScriptValue(Map m) : v(std::move(m)) {}

    bool isNil() const { return std::holds_alternative<std::monostate>(v); }
    bool isBool() const { return std::holds_alternative<bool>(v); }
    bool isInt() const { return std::holds_alternative<std::int64_t>(v); }
    bool isNumber() const { return isInt() || std::holds_alternative<double>(v); }
    bool isString() const { return std::holds_alternative<std::string>(v); }
    bool isArray() const { return std::holds_alternative<Array>(v); }
    bool isMap() const { return std::holds_alternative<Map>(v); }
    bool asBool() const { return std::get<bool>(v); }
    std::int64_t asInt() const { return std::get<std::int64_t>(v); }
    /// Integer or float as a double.
    double asNumber() const;
    const std::string& asString() const { return std::get<std::string>(v); }
    const Array& asArray() const { return std::get<Array>(v); }
    const Map& asMap() const { return std::get<Map>(v); }
};

using ScriptValues = std::vector<ScriptValue>;

/// Thrown by a host function to raise a script error (the script sees a normal Lua error and may
/// catch it with pcall). Any other std::exception from a host function becomes one too.
struct ScriptError {
    std::string message;
};

/// A function the host exposes to scripts. Called on the thread running the script.
using HostFunction = std::function<ScriptValues(const ScriptValues& args)>;

/// Resource limits of ONE run. Every run starts with fresh budgets in a fresh Lua state.
struct ScriptLimits {
    /// Lua VM instructions, counted in blocks of 1000 (a run stops within one block of the limit).
    std::uint64_t maxInstructions = 50'000'000;
    /// Steps of the string-pattern matcher (find/match/gmatch/gsub), which runs inside C code where
    /// the instruction counter cannot see it. One step per recursive match call.
    std::uint64_t maxPatternSteps = 2'000'000;
    /// Bytes the Lua heap may hold (all strings, tables, closures, stacks, the loaded chunk).
    std::size_t maxMemoryBytes = 64u * 1024u * 1024u;
    /// Longest string a script may build by any means (rep, concatenation, format, tostring, error
    /// messages, ...). Together with maxAllocationBytes it keeps every single C-side operation short.
    std::size_t maxStringBytes = 1u * 1024u * 1024u;
    /// Largest single heap block (one table array, stack, buffer) the Lua heap may grow by; a larger
    /// request fails like running out of memory.
    std::size_t maxAllocationBytes = 8u * 1024u * 1024u;
    /// Longest chunk text `load` accepts (the parser runs in C code).
    std::size_t maxLoadBytes = 256u * 1024u;
    /// Elements one `table.move` may copy (the copy loop runs in C code).
    std::size_t maxTableMoveElements = 1u << 20;
    /// Bytes `print` may emit in total (each line also costs one newline byte).
    std::size_t maxOutputBytes = 1u << 20;
    /// Bytes of script source accepted.
    std::size_t maxSourceBytes = 1u << 20;
    /// Longest nesting of arrays handed to or from host functions.
    int maxValueDepth = 16;
    /// Values (every nil, number, string and array counted once per visit, so a shared sub-table
    /// counts each time) one conversion between Lua and the host may visit.
    std::uint64_t maxValueElements = 1u << 20;
    /// Bytes of string data one such conversion may copy.
    std::uint64_t maxValueBytes = 16u * 1024u * 1024u;
    /// Wall-clock backstop for the whole run, in milliseconds (0 = none). The deterministic budgets
    /// above end ordinary runaway scripts reproducibly; this one bounds anything they cannot see.
    /// A run stopped by it ends with ScriptStatus::Timeout and is NOT reproducible.
    std::uint64_t deadlineMillis = 5000;
    /// Monotonic time source in nanoseconds; empty uses std::chrono::steady_clock. Tests inject a
    /// fake clock so deadline behaviour is deterministic.
    std::function<std::uint64_t()> clockNanos;
};

/// Standard libraries a run may use besides the always-present base, coroutine, table, string,
/// math and utf8 (reduced as described in ADR-0101). All off by default; the host grants them.
struct ScriptGrants {
    bool io = false;
    bool os = false;
    bool debug = false;
    bool package = false;
};

enum class ScriptStatus {
    Ok,
    SyntaxError,       ///< the source did not compile
    RuntimeError,      ///< an error raised by the script, a host function or the Lua runtime
    InstructionLimit,  ///< instruction or pattern-step budget exhausted
    MemoryLimit,       ///< the heap limit was hit and the run ended on it
    OutputLimit,       ///< `print` exceeded maxOutputBytes
    Cancelled,         ///< the cancel flag was seen
    Timeout,           ///< the wall-clock deadline passed (non-deterministic by nature)
    LimitsInvalid,     ///< source larger than maxSourceBytes (nothing was run)
};

struct ScriptResult {
    ScriptStatus status = ScriptStatus::Ok;
    std::string message;    ///< "name:LINE: text" as Lua reports it; empty on Ok
    std::string chunkName;  ///< the name given to run()
    int line = 0;           ///< 1-based line parsed from the message; 0 if unknown
    std::string traceback;  ///< stack traceback of a RuntimeError; may be empty
    std::vector<std::string> output;  ///< lines written by print()
    ScriptValues returned;  ///< values returned by the chunk (unsupported types become nil)
    std::uint64_t instructionsUsed = 0;  ///< rounded up to the 1000 block
    std::uint64_t patternStepsUsed = 0;
    std::size_t peakMemoryBytes = 0;
    bool ok() const { return status == ScriptStatus::Ok; }
};

/// Runs Lua scripts in a sandbox (ADR-0101). Registrations persist in the host; each run() builds a
/// new Lua state, so scripts cannot leak globals into each other and every run has full budgets.
/// Use from one thread at a time. The cancel flag may be set from any thread.
class ScriptHost {
   public:
    explicit ScriptHost(ScriptLimits limits = {}, ScriptGrants grants = {});
    ~ScriptHost();
    ScriptHost(const ScriptHost&) = delete;
    ScriptHost& operator=(const ScriptHost&) = delete;

    const ScriptLimits& limits() const { return limits_; }
    const ScriptGrants& grants() const { return grants_; }

    /// Exposes `fn` as `module.name` (a global table created on demand), or as global `name` when
    /// `module` is empty. Replaces an earlier registration of the same name.
    void registerFunction(std::string_view module, std::string_view name, HostFunction fn);
    /// Exposes a constant as `module.name` (or a global).
    void setValue(std::string_view module, std::string_view name, ScriptValue value);

    /// Runs `source` as a chunk called `chunkName` (it appears in messages as "chunkName:LINE:").
    /// `cancel` (optional, must outlive the call) is polled about every 100 VM instructions and
    /// in pattern matching.
    ScriptResult run(std::string_view source, std::string_view chunkName,
                     const std::atomic<bool>* cancel = nullptr) const;
    /// Compiles `source` without running it.
    ScriptResult check(std::string_view source, std::string_view chunkName) const;

   private:
    struct Registration {
        std::string module;
        std::string name;
        HostFunction fn;  // empty: a constant
        ScriptValue value;
    };
    ScriptResult execute(std::string_view source, std::string_view chunkName,
                         const std::atomic<bool>* cancel, bool runIt) const;

    ScriptLimits limits_;
    ScriptGrants grants_;
    std::vector<Registration> registrations_;
};

}  // namespace carvetto::scripting
