#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "carvetto/document/ids.hpp"
#include "carvetto/relief/relief.hpp"

namespace carvetto::document {

/// A relief component in the document (T-0282, ADR-0061): the core component plus its id.
struct ReliefComponent {
    ComponentId id;
    relief::Component data;
};

/// A relief level in the document: belongs to one sheet, holds components bottom first.
struct ReliefLevel {
    LevelId id;
    SheetId sheet;
    std::string name{"Level"};
    relief::CombineMode mode{relief::CombineMode::Add};
    double baseHeight{0.0};
    bool visible{true};
    std::vector<ReliefComponent> components;
};

/// Caps enforced by the commands and the file loader.
inline constexpr std::size_t kMaxReliefLevels = 256;
inline constexpr std::size_t kMaxReliefComponents = 1024;
/// Cells of all height fields of a document together: 2^26 floats = 256 MiB.
inline constexpr std::uint64_t kMaxReliefTotalCells = std::uint64_t{1} << 26;
inline constexpr std::size_t kMaxReliefNameBytes = 256;

/// Like `reliefComponentProblem` but ignores the field (for property edits).
std::string reliefPropertiesProblem(const relief::Component& c);

/// Empty when the component's properties are storable: finite numbers everywhere (also in a
/// disabled fade or tilt), a usable fade/tilt direction when enabled, a name within the cap and a
/// non-empty field. Otherwise the reason.
std::string reliefComponentProblem(const relief::Component& c);

/// Ordered, duplicate-free selection of components. Not part of the undo history.
class ComponentSelection {
   public:
    bool add(ComponentId id);
    bool remove(ComponentId id);
    bool toggle(ComponentId id);
    void assign(const std::vector<ComponentId>& ids);
    void clear() { ids_.clear(); }
    bool contains(ComponentId id) const;
    bool empty() const { return ids_.empty(); }
    std::size_t size() const { return ids_.size(); }
    /// Selected ids in selection order.
    const std::vector<ComponentId>& ids() const { return ids_; }
    /// The single selected id, or null when zero or several are selected.
    ComponentId single() const { return ids_.size() == 1 ? ids_.front() : ComponentId{}; }

   private:
    std::vector<ComponentId> ids_;
};

}  // namespace carvetto::document
