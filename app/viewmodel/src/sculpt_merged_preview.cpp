#include "carvetto/viewmodel/sculpt_merged_preview.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "carvetto/relief/effects.hpp"

namespace carvetto::viewmodel {

namespace {

bool isOpaque(float v) { return !std::isnan(v); }

// Cell range [lo, hi] (inclusive) of cells whose centres may lie in [a, b], clamped to [0, n).
bool cellRange(double a, double b, double origin, double cell, std::size_t n, std::size_t& lo, std::size_t& hi) {
    const double fl = std::floor((a - origin) / cell) - 1.0;
    const double ce = std::ceil((b - origin) / cell) + 1.0;
    if (!(ce >= 0.0) || !(fl <= static_cast<double>(n) - 1.0) || n == 0) {
        return false;
    }
    lo = fl < 0.0 ? 0 : static_cast<std::size_t>(fl);
    hi = ce > static_cast<double>(n) - 1.0 ? n - 1 : static_cast<std::size_t>(ce);
    return lo <= hi;
}

}  // namespace

bool SculptMergedPreview::begin(const document::Document& doc, document::ComponentId target,
                                const relief::HeightField& working, std::size_t maxDimension) {
    end();
    auto fail = [this](std::string why) {
        problem_ = std::move(why);
        surface_ = ModelSurface{};
        surface_.problem = problem_;
        return false;
    };
    const auto loc = doc.locateReliefComponent(target);
    const document::ReliefLevel* lv = loc ? doc.findReliefLevel(loc->level) : nullptr;
    if (!loc || lv == nullptr || working.empty()) {
        return fail("The sculpted component is not in the model.");
    }
    const document::SheetId sheet = lv->sheet;
    const std::vector<document::LevelId> ids = doc.reliefLevelsOnSheet(sheet);
    std::vector<relief::Level> levels = doc.reliefModel(sheet);
    const auto pos = std::find(ids.begin(), ids.end(), loc->level);
    if (pos == ids.end() || ids.size() != levels.size() || loc->index >= levels[static_cast<std::size_t>(pos - ids.begin())].components.size()) {
        return fail("The sculpted component is not in the model.");
    }
    const std::size_t li = static_cast<std::size_t>(pos - ids.begin());
    levels[li].visible = true;
    levels[li].components[loc->index].visible = true;
    levels[li].components[loc->index].field = working;

    relief::Grid grid;
    if (const relief::Status s = relief::unionGrid(levels, grid); s != relief::Status::Ok) {
        return fail(relief::statusMessage(s));
    }
    const std::size_t limit = std::max<std::size_t>(maxDimension, 2);
    const std::size_t longer = std::max(grid.width, grid.height);
    stride_ = 1;
    if (longer > limit) {
        stride_ = (longer + limit - 1) / limit;
        grid.cellSize *= static_cast<double>(stride_);
        grid.width = (grid.width - 1) / stride_ + 1;
        grid.height = (grid.height - 1) / stride_ + 1;
    }
    grid_ = grid;

    auto cells = static_cast<std::uint64_t>(grid.width) * grid.height;
    // Levels below.
    {
        std::vector<relief::Level> lower(levels.begin(), levels.begin() + static_cast<std::ptrdiff_t>(li));
        if (const relief::Status s = relief::compose(lower, grid, lowerModel_); s != relief::Status::Ok) {
            return fail(relief::statusMessage(s));
        }
        evaluations_ += cells * (lower.size() + 1);
    }
    // The target's level: split around the target.
    const relief::Level& tl = levels[li];
    if (!std::isfinite(tl.baseHeight)) {
        return fail(relief::statusMessage(relief::Status::BadParameter));
    }
    target_ = tl.components[loc->index];
    level_ = tl;
    level_.components.clear();
    if (const relief::Status s = relief::validateProperties(target_); s != relief::Status::Ok) {
        return fail(relief::statusMessage(s));
    }
    relief::HeightField::create(grid, relief::kTransparent, below_);
    for (std::size_t c = 0; c < tl.components.size(); ++c) {
        if (c == loc->index || !tl.components[c].visible) {
            continue;
        }
        relief::HeightField cf;
        if (const relief::Status s = relief::composeComponent(tl.components[c], grid, cf); s != relief::Status::Ok) {
            return fail(relief::statusMessage(s));
        }
        evaluations_ += cells;
        if (c < loc->index) {
            for (std::size_t k = 0; k < below_.data().size(); ++k) {
                below_.mutableData()[k] = relief::combine(tl.components[c].mode, below_.data()[k], cf.data()[k]);
            }
        } else {
            aboveModes_.push_back(tl.components[c].mode);
            above_.push_back(std::move(cf));
        }
    }
    if (const relief::Status s = relief::validateEffects(tl.effects); s != relief::Status::Ok) {
        return fail(relief::statusMessage(s));
    }
    // Levels above, each on its own (mode Add: a lone level over a transparent model is itself).
    for (std::size_t k = li + 1; k < levels.size(); ++k) {
        if (!levels[k].visible) {
            continue;
        }
        relief::Level one = levels[k];
        one.mode = relief::CombineMode::Add;
        LevelRef ref;
        ref.mode = levels[k].mode;
        if (const relief::Status s = relief::compose({one}, grid, ref.field); s != relief::Status::Ok) {
            return fail(relief::statusMessage(s));
        }
        evaluations_ += cells * (one.components.size() + 1);
        upper_.push_back(std::move(ref));
    }
    relief::HeightField::create(grid, relief::kTransparent, levelPre_);
    relief::HeightField::create(grid, relief::kTransparent, levelFin_);
    relief::HeightField::create(grid, relief::kTransparent, surface_.field);
    surface_.stride = stride_;
    surface_.problem.clear();
    problem_.clear();
    active_ = true;
    recompute(0, grid.width - 1, 0, grid.height - 1);
    return true;
}

void SculptMergedPreview::end() {
    active_ = false;
    problem_.clear();
    surface_ = ModelSurface{};
    target_ = relief::Component{};
    level_ = relief::Level{};
    lowerModel_ = below_ = levelPre_ = levelFin_ = relief::HeightField{};
    aboveModes_.clear();
    above_.clear();
    upper_.clear();
    stride_ = 1;
}

void SculptMergedPreview::update(const relief::HeightField& working, const SculptDirty& dirty) {
    if (!active_ || working.empty()) {
        return;
    }
    const relief::Grid& wg = working.grid();
    const relief::Grid& tg = target_.field.grid();
    if (wg.width != tg.width || wg.height != tg.height) {
        return;  // not the field this preview was built for; the caller rebuilds
    }
    if (!dirty.all && !dirty.any) {
        return;
    }
    if (dirty.all) {
        target_.field = working;
        recompute(0, grid_.width - 1, 0, grid_.height - 1);
        return;
    }
    // Copy the changed component cells.
    std::size_t ci0 = 0, ci1 = 0, cj0 = 0, cj1 = 0;
    if (!cellRange(dirty.lo.x, dirty.hi.x, tg.origin.x, tg.cellSize, tg.width, ci0, ci1) ||
        !cellRange(dirty.lo.y, dirty.hi.y, tg.origin.y, tg.cellSize, tg.height, cj0, cj1)) {
        return;
    }
    for (std::size_t j = cj0; j <= cj1; ++j) {
        for (std::size_t i = ci0; i <= ci1; ++i) {
            target_.field.set(i, j, working.at(i, j));
        }
    }
    // Surface cells whose bilinear sample can see them (one component cell of margin).
    std::size_t i0 = 0, i1 = 0, j0 = 0, j1 = 0;
    const double m = tg.cellSize;
    if (!cellRange(dirty.lo.x - m, dirty.hi.x + m, grid_.origin.x, grid_.cellSize, grid_.width, i0, i1) ||
        !cellRange(dirty.lo.y - m, dirty.hi.y + m, grid_.origin.y, grid_.cellSize, grid_.height, j0, j1)) {
        return;
    }
    recompute(i0, i1, j0, j1);
}

void SculptMergedPreview::recompute(std::size_t i0, std::size_t i1, std::size_t j0, std::size_t j1) {
    const std::size_t w = grid_.width;
    const std::uint64_t rect = static_cast<std::uint64_t>(i1 - i0 + 1) * (j1 - j0 + 1);
    // The target's level before effects.
    for (std::size_t j = j0; j <= j1; ++j) {
        const double y = grid_.origin.y + static_cast<double>(j) * grid_.cellSize;
        for (std::size_t i = i0; i <= i1; ++i) {
            const double x = grid_.origin.x + static_cast<double>(i) * grid_.cellSize;
            const std::size_t k = j * w + i;
            float v = relief::combine(target_.mode, below_.data()[k], relief::componentValueAt(target_, x, y));
            for (std::size_t a = 0; a < above_.size(); ++a) {
                v = relief::combine(aboveModes_[a], v, above_[a].data()[k]);
            }
            if (isOpaque(v)) {
                v = static_cast<float>(static_cast<double>(v) + level_.baseHeight);
            }
            levelPre_.mutableData()[k] = v;
        }
    }
    evaluations_ += rect * (1 + above_.size());
    // Effects. A mirror reads cells anywhere, so it redoes the level and everything it touches.
    if (level_.effects.mirror != relief::MirrorMode::None || level_.effects.wrapping.enabled) {
        levelFin_ = levelPre_;
        relief::applyEffects(level_.effects, levelFin_);
        evaluations_ += static_cast<std::uint64_t>(w) * grid_.height;
        i0 = 0;
        i1 = w - 1;
        j0 = 0;
        j1 = grid_.height - 1;
    } else if (!level_.effects.clip.polygons.empty()) {
        relief::HeightField piece;
        levelPre_.crop(i0, j0, i1 - i0 + 1, j1 - j0 + 1, piece);
        relief::applyClip(level_.effects.clip, piece);
        for (std::size_t j = j0; j <= j1; ++j) {
            for (std::size_t i = i0; i <= i1; ++i) {
                levelFin_.set(i, j, piece.at(i - i0, j - j0));
            }
        }
        evaluations_ += rect;
    } else {
        for (std::size_t j = j0; j <= j1; ++j) {
            for (std::size_t i = i0; i <= i1; ++i) {
                levelFin_.set(i, j, levelPre_.at(i, j));
            }
        }
    }
    // Combine into the model.
    const std::uint64_t outRect = static_cast<std::uint64_t>(i1 - i0 + 1) * (j1 - j0 + 1);
    for (std::size_t j = j0; j <= j1; ++j) {
        for (std::size_t i = i0; i <= i1; ++i) {
            const std::size_t k = j * w + i;
            float m = relief::combine(level_.mode, lowerModel_.data()[k], levelFin_.data()[k]);
            for (const LevelRef& u : upper_) {
                m = relief::combine(u.mode, m, u.field.data()[k]);
            }
            surface_.field.mutableData()[k] = m;
        }
    }
    evaluations_ += outRect * (1 + upper_.size());
}

}  // namespace carvetto::viewmodel
