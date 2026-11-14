// Document: relief levels and components (T-0282, ADR-0061).
#include <algorithm>
#include <cassert>
#include <utility>

#include "carvetto/document/document.hpp"

namespace carvetto::document {

std::vector<LevelId> Document::reliefLevelsOnSheet(SheetId sheet) const {
    std::vector<LevelId> out;
    for (const ReliefLevel& l : reliefLevels_) {
        if (l.sheet == sheet) {
            out.push_back(l.id);
        }
    }
    return out;
}

const ReliefLevel* Document::findReliefLevel(LevelId id) const {
    for (const ReliefLevel& l : reliefLevels_) {
        if (l.id == id) {
            return &l;
        }
    }
    return nullptr;
}

ReliefLevel* Document::findReliefLevel(LevelId id) {
    return const_cast<ReliefLevel*>(std::as_const(*this).findReliefLevel(id));
}

std::optional<std::size_t> Document::reliefLevelIndex(LevelId id) const {
    const ReliefLevel* level = findReliefLevel(id);
    if (!level) {
        return std::nullopt;
    }
    std::size_t index = 0;
    for (const ReliefLevel& l : reliefLevels_) {
        if (l.id == id) {
            return index;
        }
        if (l.sheet == level->sheet) {
            ++index;
        }
    }
    return std::nullopt;
}

const ReliefComponent* Document::findReliefComponent(ComponentId id) const {
    for (const ReliefLevel& l : reliefLevels_) {
        for (const ReliefComponent& c : l.components) {
            if (c.id == id) {
                return &c;
            }
        }
    }
    return nullptr;
}

ReliefComponent* Document::findReliefComponent(ComponentId id) {
    return const_cast<ReliefComponent*>(std::as_const(*this).findReliefComponent(id));
}

std::optional<Document::ReliefLocation> Document::locateReliefComponent(ComponentId id) const {
    for (const ReliefLevel& l : reliefLevels_) {
        for (std::size_t i = 0; i < l.components.size(); ++i) {
            if (l.components[i].id == id) {
                return ReliefLocation{l.id, i};
            }
        }
    }
    return std::nullopt;
}

std::size_t Document::reliefComponentCount() const {
    std::size_t n = 0;
    for (const ReliefLevel& l : reliefLevels_) {
        n += l.components.size();
    }
    return n;
}

std::uint64_t Document::reliefCellCount() const {
    std::uint64_t n = 0;
    for (const ReliefLevel& l : reliefLevels_) {
        for (const ReliefComponent& c : l.components) {
            n += static_cast<std::uint64_t>(c.data.field.width()) * c.data.field.height();
        }
    }
    return n;
}

namespace {
/// Absolute position in the flat level list for `indexOnSheet` among `sheet`'s levels (the slot
/// after the sheet's last level when the index is past the end).
std::size_t flatPosition(const std::vector<ReliefLevel>& levels, SheetId sheet, std::size_t indexOnSheet) {
    std::size_t seen = 0;
    std::size_t after = levels.size();
    bool any = false;
    for (std::size_t i = 0; i < levels.size(); ++i) {
        if (levels[i].sheet != sheet) {
            continue;
        }
        if (seen == indexOnSheet) {
            return i;
        }
        ++seen;
        after = i + 1;
        any = true;
    }
    return any ? after : levels.size();
}
}  // namespace

LevelId Document::insertReliefLevel(ReliefLevel level, std::optional<std::size_t> indexOnSheet) {
    if (!findSheet(level.sheet)) {
        return {};
    }
    if (level.id.isNull()) {
        level.id = allocateLevelId();
        if (level.id.isNull()) {
            return {};
        }
    } else {
        assert(!findReliefLevel(level.id) && "duplicate relief level id");
        ids_.reserveThrough(level.id.value);
    }
    for (ReliefComponent& c : level.components) {
        if (c.id.isNull()) {
            c.id = allocateComponentId();
            if (c.id.isNull()) {
                return {};
            }
        } else {
            assert(!findReliefComponent(c.id) && "duplicate relief component id");
            ids_.reserveThrough(c.id.value);
        }
    }
    const LevelId id = level.id;
    const std::size_t at = flatPosition(reliefLevels_, level.sheet, indexOnSheet.value_or(reliefLevels_.size()));
    reliefLevels_.insert(reliefLevels_.begin() + static_cast<std::ptrdiff_t>(at), std::move(level));
    return id;
}

std::optional<ReliefLevel> Document::takeReliefLevel(LevelId id) {
    const auto it = std::find_if(reliefLevels_.begin(), reliefLevels_.end(),
                                 [id](const ReliefLevel& l) { return l.id == id; });
    if (it == reliefLevels_.end()) {
        return std::nullopt;
    }
    ReliefLevel level = std::move(*it);
    reliefLevels_.erase(it);
    for (const ReliefComponent& c : level.components) {
        componentSelection_.remove(c.id);
    }
    return level;
}

bool Document::moveReliefLevel(LevelId id, std::size_t indexOnSheet) {
    const ReliefLevel* level = findReliefLevel(id);
    if (!level || indexOnSheet >= reliefLevelsOnSheet(level->sheet).size()) {
        return false;
    }
    // takeReliefLevel deselects the components; moving a level must not change the selection.
    const std::vector<ComponentId> keep = componentSelection_.ids();
    auto taken = takeReliefLevel(id);
    insertReliefLevel(std::move(*taken), indexOnSheet);
    componentSelection_.assign(keep);
    return true;
}

ComponentId Document::insertReliefComponent(LevelId level, ReliefComponent component,
                                            std::optional<std::size_t> index) {
    ReliefLevel* l = findReliefLevel(level);
    if (!l) {
        return {};
    }
    if (component.id.isNull()) {
        component.id = allocateComponentId();
        if (component.id.isNull()) {
            return {};
        }
    } else {
        assert(!findReliefComponent(component.id) && "duplicate relief component id");
        ids_.reserveThrough(component.id.value);
    }
    const ComponentId id = component.id;
    const std::size_t at = std::min(index.value_or(l->components.size()), l->components.size());
    l->components.insert(l->components.begin() + static_cast<std::ptrdiff_t>(at), std::move(component));
    return id;
}

std::optional<ReliefComponent> Document::takeReliefComponent(ComponentId id) {
    for (ReliefLevel& l : reliefLevels_) {
        for (auto it = l.components.begin(); it != l.components.end(); ++it) {
            if (it->id == id) {
                ReliefComponent c = std::move(*it);
                l.components.erase(it);
                componentSelection_.remove(id);
                return c;
            }
        }
    }
    return std::nullopt;
}

std::vector<relief::Level> Document::reliefModel(SheetId sheet) const {
    std::vector<relief::Level> out;
    for (const ReliefLevel& l : reliefLevels_) {
        if (l.sheet != sheet) {
            continue;
        }
        relief::Level r;
        r.name = l.name;
        r.mode = l.mode;
        r.baseHeight = l.baseHeight;
        r.visible = l.visible;
        r.effects = l.effects;
        for (const ReliefComponent& c : l.components) {
            r.components.push_back(c.data);
        }
        out.push_back(std::move(r));
    }
    return out;
}

}  // namespace carvetto::document
