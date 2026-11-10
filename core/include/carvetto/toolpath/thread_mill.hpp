#pragma once

#include <optional>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/geometry/tolerance.hpp"
#include "carvetto/geometry/vec2.hpp"
#include "carvetto/tooling/cutting_data.hpp"
#include "carvetto/tooling/tool.hpp"
#include "carvetto/toolpath/cut_order.hpp"
#include "carvetto/toolpath/operation.hpp"
#include "carvetto/toolpath/toolpath.hpp"

// Thread milling toolpath (T-0237, ADR-0036; feature-inventory "Thread Milling Toolpath"): a
// helical path round the centre of every closed vector (the hole of an internal thread, the pin
// of an external one) whose Z advance per revolution is exactly the pitch.

namespace carvetto::toolpath {

/// Single point: one tooth, cuts one groove per revolution. Multi point: a row of teeth over
/// `ThreadMillParams::threadedLength`, cutting that whole length of thread in one revolution.
enum class ThreadCutter { SinglePoint, MultiPoint };
/// Internal: a thread in a hole (tool inside the bore). External: on a pin (tool outside it).
enum class ThreadType { Internal, External };
/// Hand of the thread: a right-hand thread advances away from the viewer when turned clockwise.
enum class ThreadHand { Right, Left };
/// Which way the tool travels along the thread: Down = from the start depth downwards.
enum class ThreadCutDirection { Down, Up };
/// Where the pitch comes from. Custom: `pitch`. The presets look the pitch up from `diameter`
/// (ISO 261 coarse series for metric sizes; the unified coarse series, UNC, for inch sizes).
enum class ThreadPitchPreset { Custom, IsoMetricCoarse, UnifiedCoarse };

/// Parameters of a thread milling toolpath. Lengths in mm.
struct ThreadMillParams {
    std::string name{"Thread Mill"};
    ThreadCutter cutter{ThreadCutter::SinglePoint};
    /// Multi point only: length of the toothed part of the cutter, from its tip (>= pitch, <=
    /// `threadLength`). Ignored for a single point cutter (one tooth: the pitch).
    double threadedLength{6.0};
    ThreadType type{ThreadType::Internal};
    ThreadHand hand{ThreadHand::Right};
    ThreadCutDirection cutDirection{ThreadCutDirection::Down};
    ThreadPitchPreset pitchPreset{ThreadPitchPreset::Custom};
    /// Pitch P (> 0); used when `pitchPreset` is Custom.
    double pitch{1.0};
    /// Nominal (major) diameter of the thread (> 0), e.g. 6 for M6x1.
    double diameter{6.0};
    /// Fit tolerance (diametral, signed, |t| <= pitch): positive = a looser fit (an internal thread
    /// cut wider, an external one cut thinner), negative = tighter.
    double fitTolerance{0.0};
    /// Also make circles of the bore (minor) diameter for an internal thread (`threadBoreCircles`).
    bool createCircles{false};
    /// Depth below the surface at which the thread starts (>= 0).
    double startDepth{0.0};
    /// Length L of the thread (>= pitch).
    double threadLength{5.0};
    /// Maximum depth M below the surface the tool may reach; refused when `startDepth +
    /// threadLength` is deeper. 0 = no limit.
    double maxDepth{0.0};
    /// Absolute Z for moves between threads (above every cutting position).
    double safeZ{5.0};
    OrderOptions order;
    geometry::Vec2 startPosition{0.0, 0.0};
    /// `distance`: threads with centres closer than this are made once; `chordal`: point-in-vector test.
    geometry::Tolerance tolerance{geometry::Tolerance::defaults()};

    bool operator==(const ThreadMillParams&) const = default;
};

/// One thread made.
struct ThreadMillThread {
    geometry::Vec2 centre;
    /// Index of the input vector.
    std::size_t contour{0};
    bool operator==(const ThreadMillThread&) const = default;
};

struct ThreadMillResult {
    Toolpath toolpath;
    /// The threads in machining order.
    std::vector<ThreadMillThread> threads;
    std::vector<ToolpathDiagnostic> diagnostics;
    bool ok() const { return !toolpath.isEmpty(); }
    bool has(ToolpathIssue issue) const { return hasIssue(diagnostics, issue); }
};

/// Pitch of a preset for a nominal diameter: ISO coarse for M1 .. M64 (M1, 1.2, 1.4, 1.6, 2, 2.5,
/// 3, 4, 5, 6, 8, 10, 12, 14, 16, 20, 24, 30, 36, 42, 48, 56, 64), unified coarse for the inch
/// sizes 1/4 .. 1 in (diameter 25.4 x inch, pitch 25.4 / threads per inch). nullopt for Custom, a
/// diameter with no entry (within 0.005 mm) or a non-finite one.
std::optional<double> threadPresetPitch(ThreadPitchPreset preset, double diameter);
/// The pitch the toolpath uses: the preset's, else `pitch`; nullopt when a preset has no entry or
/// the pitch is not finite and positive.
std::optional<double> threadPitch(const ThreadMillParams& params);
/// Minor diameter of an internal ISO thread, D - 1.0825 P (the bore to drill); 0 when the inputs
/// are not finite and positive or P is too large for D.
double threadBoreDiameter(double diameter, double pitch);
/// Depth of an external ISO thread, 0.6134 P.
double threadExternalDepth(double pitch);
/// Radius of the path of the tool axis round the thread axis for a cutter of diameter `toolDiameter`
/// (ADR-0036): internal (D + t) / 2 - d / 2; external (D - 2 h - t) / 2 + d / 2 (h the thread
/// depth, t the fit tolerance, d the cutter diameter). nullopt when invalid or not above 1 um.
std::optional<double> threadPathRadius(const ThreadMillParams& params, double toolDiameter);
/// Circles of the bore diameter at the given centres (CCW), for the "create circles" option.
/// Empty unless the thread is internal and the bore diameter is positive.
std::vector<geometry::Contour> threadBoreCircles(const ThreadMillParams& params,
                                                 const std::vector<geometry::Vec2>& centres);

/// Only an end mill (the thread mill, with its diameter the cutting diameter of the teeth).
bool threadMillToolAllowed(tooling::ToolType type);

/// Computes the thread milling toolpath (ADR-0036). Per thread: a rapid to a point above it at
/// safe Z, a plunge to the start of the helix, a lead-in (internal: a half circle out from the
/// centre; external: a tangent line), the helix, a lead-out (the same, mirrored) and a retract.
/// The helix is arcs of one revolution each whose Z changes by exactly the pitch, then an arc of
/// the remaining fraction. Direction: right-hand + down or left-hand + up is clockwise,
/// right-hand + up or left-hand + down counter-clockwise. Bad parameters or tools give an empty
/// toolpath and a diagnostic. Vectors are handled as in the drilling toolpath (centre = area
/// centroid; open, empty, discontinuous, area-less ones skipped and named).
ThreadMillResult threadMillToolpath(const std::vector<geometry::Contour>& contours, const tooling::Tool& tool,
                                    const std::optional<tooling::CuttingData>& cuttingData,
                                    const ThreadMillParams& params);

}  // namespace carvetto::toolpath
