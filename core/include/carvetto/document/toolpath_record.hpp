#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/document/ids.hpp"
#include "carvetto/toolpath/drill.hpp"
#include "carvetto/toolpath/chamfer.hpp"
#include "carvetto/toolpath/engrave.hpp"
#include "carvetto/toolpath/move.hpp"
#include "carvetto/toolpath/pocket.hpp"
#include "carvetto/toolpath/prism.hpp"
#include "carvetto/toolpath/profile.hpp"
#include "carvetto/toolpath/thread_mill.hpp"
#include "carvetto/toolpath/toolpath.hpp"
#include "carvetto/toolpath/v_carve.hpp"

// Toolpath records in the document (T-0132, ADR-0021): what a toolpath was made from (operation
// parameters, vector selection, tool) plus its computed moves. Saved in `job.json` (definitions)
// and `toolpaths/NNNN.bin` (moves) by `saveCarv`.

namespace carvetto::document {

/// Which vectors a toolpath machines. Ids are never reused, so a reference to an object or layer
/// that has since been deleted names nothing (and names it again if the deletion is undone).
struct VectorSelection {
    /// Every vector object of the document (`layers` and `objects` are then empty).
    bool all{false};
    /// Layers whose vector objects are machined.
    std::vector<LayerId> layers;
    /// Individual objects; a group stands for its descendant vectors.
    std::vector<ObjectId> objects;

    bool operator==(const VectorSelection&) const = default;

    /// The selection of every vector object.
    static VectorSelection allVectors() { return VectorSelection{true, {}, {}}; }
};

/// Where the tool of a toolpath came from in the tool database, for recalculation. Any field may
/// be empty (unknown).
struct ToolSource {
    /// Tool id in the tool database.
    std::string toolId;
    /// Machine and material ids that chose the cutting data (both or neither).
    std::string machine;
    std::string material;

    bool operator==(const ToolSource&) const = default;
};

/// The operation of a toolpath and its parameters. `cuttingData` inside `ProfileParams` is input
/// from the tool database and is not part of a saved record (it loads empty); the feeds the
/// toolpath was computed with are in `ToolpathRecord::tool`. Drilling (T-0150) and quick engraving
/// (T-0151) take their cutting data as a separate argument, so `DrillParams` and `EngraveParams`
/// hold only record content. A V-carve (T-0153) takes its tools (carving tool, clearance tools)
/// as arguments too; each toolpath it makes is its own record with the same `VCarvingParams` and
/// its own tool. A chamfer (T-0236) takes its tool and cutting data as arguments too, as does a thread mill (T-0237).
using ToolpathParams = std::variant<toolpath::ProfileParams, toolpath::PocketParams, toolpath::DrillParams,
                                    toolpath::EngraveParams, toolpath::VCarvingParams, toolpath::ChamferParams,
                                    toolpath::ThreadMillParams, toolpath::PrismParams>;

/// Discriminator for `ToolpathRecord::operation()` (same order as the `ToolpathParams` variant).
enum class ToolpathOperation { Profile, Pocket, Drill, Engrave, VCarving, Chamfer, ThreadMill, Prism };

/// Computed moves of a toolpath, shared and immutable so copies (undo history, clipboard) are cheap.
using SharedMoves = std::shared_ptr<const std::vector<toolpath::Move>>;

/// One toolpath of the job: identity, name, what it was made from and its computed moves.
struct ToolpathRecord {
    ToolpathId id{};
    /// Display name, also the name passed to the post-processor.
    std::string name;
    ToolpathParams params{toolpath::ProfileParams{}};
    VectorSelection selection;
    /// Tool values copied when the moves were computed (name, number, diameter, feeds).
    toolpath::ToolSnapshot tool;
    ToolSource toolSource;
    /// Computed moves; null when the toolpath has not been computed (or the cache was not saved).
    SharedMoves moves;

    ToolpathOperation operation() const { return static_cast<ToolpathOperation>(params.index()); }
    /// Absolute safe Z of the operation's parameters (mm).
    double safeZ() const;
    /// True when computed moves are held (possibly an empty list).
    bool hasMoves() const { return moves != nullptr; }
    /// Number of computed moves (0 when none are held).
    std::size_t moveCount() const { return moves ? moves->size() : 0; }
    /// The record as a `toolpath::Toolpath` (name, tool snapshot and a copy of the moves).
    toolpath::Toolpath toolpath() const;

    /// Equal when every field is equal; moves compare by content.
    bool operator==(const ToolpathRecord& other) const;
};

/// Shares `moves` as a record's computed moves. Fields a move's kind ignores (ADR-0008: the
/// centre of a non-arc, the seconds of a non-dwell, a feed override on a rapid or dwell) are
/// cleared, as the `.carv` move cache does not store them, so a record equals its saved and
/// reloaded copy.
SharedMoves shareMoves(std::vector<toolpath::Move> moves);

/// Toolpath parameters must be finite numbers (T-0136, ADR-0021 §2): returns the job.json key
/// path inside `params` of the first number that is NaN or infinite (e.g. `"safe_z"`,
/// `"passes.cut_depth"`, `"start_position[1]"`, angles with their `_rad` suffix), or an empty
/// string when every number is finite. `ProfileParams::cuttingData` is not part of a record and
/// is not checked. `AddToolpathCommand`/`SetToolpathCommand` refuse such parameters, `saveCarv`
/// refuses to write them and the loader refuses them.
std::string nonFiniteParam(const ToolpathParams& params);

/// Why the record's spindle-off marking is not allowed (T-0154), or "" when it is. Only Quick
/// Engrave marks a snapshot `spindleOff`, and only for diamond-drag tools, recording
/// `toolType` = `toolpath::kDiamondDragToolType`; so a spindle-off snapshot must be an engrave
/// record's with that tool type, and a snapshot that is not spindle-off records no tool type.
/// No other toolpath can thereby bypass the post's spindle speed > 0 rule (ADR-0020).
/// `AddToolpathCommand`/`SetToolpathCommand` refuse such records, `saveCarv` refuses to write
/// them, the loader refuses them, and the post refuses a spindle-off toolpath whose snapshot is
/// not a diamond drag tool's.
std::string spindleOffProblem(const ToolpathRecord& record);

/// True when `spindleOffProblem(record)` is empty.
bool spindleOffAllowed(const ToolpathRecord& record);

}  // namespace carvetto::document
