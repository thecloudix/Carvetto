#pragma once

#include <string>

#include "carvetto/document/ids.hpp"
#include "carvetto/document/job.hpp"
#include "carvetto/geometry/box2.hpp"
#include "carvetto/geometry/vec2.hpp"

namespace carvetto::document {

/// Which face of the stock a sheet of a double-sided job machines.
enum class Side { Front, Back };

/// One sheet of material in a job. Each sheet has its own size, Z-zero and datum; objects belong
/// to exactly one sheet. A document always keeps at least one sheet.
struct Sheet {
    SheetId id{};
    std::string name{"Sheet 1"};
    JobSize sizeMm{300.0, 300.0, 18.0};
    ZZero zZero{ZZero::MaterialSurface};
    Datum datum{Datum::BottomLeft};
    geometry::Vec2 datumOffsetMm{};
    bool visible{true};
    /// Double-sided jobs (T-0243): the face this sheet machines and the sheet of the other face of
    /// the same stock (null when unpaired). Both sheets of a pair name each other.
    Side side{Side::Front};
    SheetId partner{};

    bool operator==(const Sheet&) const = default;

    /// Sheet taking its size, Z-zero and datum from the job.
    static Sheet fromJob(const Job& job, std::string sheetName = "Sheet 1");

    /// Material rectangle in document coordinates.
    geometry::Box2 materialBoundsMm() const {
        return materialBounds(datum, sizeMm.x, sizeMm.y, datumOffsetMm);
    }
};

}  // namespace carvetto::document
