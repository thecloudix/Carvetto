#include "carvetto/viewmodel/calculation.hpp"

#include <charconv>
#include <mutex>
#include <utility>
#include <vector>

namespace carvetto::viewmodel {

namespace {

struct Registry {
    std::mutex mutex;
    /// Providers in installation order; the newest is used (a window created later wins, and the
    /// previous one is back when it goes away).
    std::vector<std::pair<const void*, CalculationProvider>> providers;
};

Registry& registry() {
    static Registry r;
    return r;
}

std::string_view trimView(std::string_view t) {
    while (!t.empty() && (t.front() == ' ' || t.front() == '\t')) {
        t.remove_prefix(1);
    }
    while (!t.empty() && (t.back() == ' ' || t.back() == '\t')) {
        t.remove_suffix(1);
    }
    return t;
}

}  // namespace

CalculationEnv environmentOf(const document::Document& doc) {
    CalculationEnv env;
    env.variables = &doc.variables();
    env.jobMm = document::ExpressionContext{doc.job().sizeMm.x, doc.job().sizeMm.y, doc.job().sizeMm.z};
    return env;
}

void setCalculationProvider(const void* owner, CalculationProvider provider) {
    Registry& r = registry();
    const std::lock_guard<std::mutex> lock(r.mutex);
    std::erase_if(r.providers, [owner](const auto& p) { return p.first == owner; });
    r.providers.emplace_back(owner, std::move(provider));
}

void clearCalculationProvider(const void* owner) {
    Registry& r = registry();
    const std::lock_guard<std::mutex> lock(r.mutex);
    std::erase_if(r.providers, [owner](const auto& p) { return p.first == owner; });
}

bool isExpressionText(std::string_view text) {
    const std::string_view t = trimView(text);
    return !t.empty() && t.front() == '=';
}

Calculation calculate(std::string_view text, document::Units units) {
    Calculation out;
    document::DocumentVariables empty;
    CalculationEnv env;
    env.variables = &empty;
    {
        Registry& r = registry();
        const std::lock_guard<std::mutex> lock(r.mutex);
        if (!r.providers.empty()) {
            if (const std::optional<CalculationEnv> e = r.providers.back().second(); e.has_value() && e->variables != nullptr) {
                env = *e;
            }
        }
    }
    document::ExpressionContext ctx = env.jobMm;
    if (units == document::Units::Inches) {
        ctx.width /= 25.4;
        ctx.height /= 25.4;
        ctx.thickness /= 25.4;
    }
    const document::ExpressionResult res = document::evaluateExpression(trimView(text), *env.variables, ctx);
    out.usedVariables = res.usedVariables;
    if (!res.ok()) {
        out.error = res.error.empty() ? "cannot evaluate the expression" : res.error;
        return out;
    }
    out.ok = true;
    out.value = *res.value;
    return out;
}

std::string numberText(double value) {
    char buf[64];
    const auto res = std::to_chars(buf, buf + sizeof buf, value, std::chars_format::general, 12);
    return std::string(buf, res.ptr);
}

std::string substituteVariables(std::string_view text, const document::DocumentVariables& vars) {
    std::string out;
    out.reserve(text.size());
    std::size_t i = 0;
    while (i < text.size()) {
        if (text[i] == '{') {
            const std::size_t close = text.find('}', i + 1);
            if (close != std::string_view::npos) {
                const std::string_view name = text.substr(i + 1, close - i - 1);
                if (const auto v = vars.get(name); v.has_value()) {
                    out += numberText(*v);
                    i = close + 1;
                    continue;
                }
            }
        }
        out += text[i++];
    }
    return out;
}

}  // namespace carvetto::viewmodel
