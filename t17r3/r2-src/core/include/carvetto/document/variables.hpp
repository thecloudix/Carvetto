#pragma once

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace carvetto::document {

/// Named numeric values the user can reference as `{Name}` in calculation edit boxes. A name
/// must start with a letter and continue with letters, digits or `_`; names are case-sensitive.
/// Variables referenced by a toolpath (tracked with `addReference`) cannot be removed.
class DocumentVariables {
   public:
    /// Outcome of `importText`.
    struct ImportResult {
        std::size_t imported{0};
        /// 1-based line numbers that could not be parsed or had an invalid name.
        std::vector<std::size_t> rejectedLines;
    };

    /// True when `name` follows the naming rule (ASCII letter, then letters/digits/underscore).
    static bool isValidName(std::string_view name);

    /// Creates or overwrites `name`. Returns false (and changes nothing) for an invalid name.
    bool set(const std::string& name, double value);
    /// Value of `name`, or nullopt when undefined.
    std::optional<double> get(std::string_view name) const;
    bool contains(std::string_view name) const;
    /// Removes `name`. Returns false when it is undefined or still referenced.
    bool remove(std::string_view name);
    /// Removes `name` even when referenced (used by undo to restore a prior state exactly).
    void forceRemove(std::string_view name);

    /// Marks one more user of `name` (e.g. a toolpath). Unknown names are ignored.
    void addReference(std::string_view name);
    /// Releases one reference; never goes below zero.
    void removeReference(std::string_view name);
    std::size_t referenceCount(std::string_view name) const;

    std::size_t size() const { return values_.size(); }
    bool empty() const { return values_.empty(); }
    /// Names in sorted order.
    std::vector<std::string> names() const;
    const std::map<std::string, double, std::less<>>& all() const { return values_; }

    /// Serialises as one `Name = value` line per variable, sorted by name, round-trip exact.
    std::string toText() const;
    /// Parses `Name = value` lines (blank lines and `#` comments skipped). Existing same-named
    /// variables are overwritten.
    ImportResult importText(std::string_view text);

    bool operator==(const DocumentVariables& o) const { return values_ == o.values_; }

   private:
    std::map<std::string, double, std::less<>> values_;
    std::map<std::string, std::size_t, std::less<>> references_;
};

}  // namespace carvetto::document
