#include "carvetto/document/toolpath_record.hpp"

#include <cmath>
#include <initializer_list>
#include <memory>
#include <string>
#include <utility>
#include <variant>
#include <vector>

namespace carvetto::document {

double ToolpathRecord::safeZ() const {
    return std::visit([](const auto& p) { return p.safeZ; }, params);
}

toolpath::Toolpath ToolpathRecord::toolpath() const {
    toolpath::Toolpath tp;
    tp.name = name;
    tp.tool = tool;
    if (moves) {
        tp.moves = *moves;
    }
    return tp;
}

SharedMoves shareMoves(std::vector<toolpath::Move> moves) {
    for (toolpath::Move& m : moves) {
        if (!toolpath::isArc(m.kind)) {
            m.centre = {};
        }
        if (m.kind != toolpath::MoveKind::Dwell) {
            m.dwellSeconds = 0.0;
        }
        if (!toolpath::isCutting(m.kind)) {
            m.feedOverride.reset();
        }
    }
    return std::make_shared<const std::vector<toolpath::Move>>(std::move(moves));
}

namespace {

struct NamedValue {
    const char* key;
    double value;
};

std::string firstNonFinite(std::initializer_list<NamedValue> values) {
    for (const NamedValue& v : values) {
        if (!std::isfinite(v.value)) {
            return v.key;
        }
    }
    return {};
}

std::string nonFiniteTolerance(const geometry::Tolerance& t) {
    return firstNonFinite({{"tolerance.distance", t.distance},
                           {"tolerance.angle_rad", t.angle},
                           {"tolerance.chordal", t.chordal}});
}

}  // namespace

std::string nonFiniteParam(const ToolpathParams& params) {
    if (const auto* p = std::get_if<toolpath::ProfileParams>(&params)) {
        std::string key = firstNonFinite({{"allowance", p->allowance},
                                          {"passes.start_depth", p->passes.startDepth},
                                          {"passes.cut_depth", p->passes.cutDepth},
                                          {"passes.pass_depth", p->passes.passDepth},
                                          {"passes.last_pass_thickness", p->passes.lastPassThickness},
                                          {"safe_z", p->safeZ},
                                          {"last_pass_allowance", p->lastPassAllowance},
                                          {"tabs.length", p->tabs.length},
                                          {"tabs.thickness", p->tabs.thickness},
                                          {"tabs.distance", p->tabs.distance},
                                          {"ramp.distance", p->ramp.distance},
                                          {"ramp.angle_rad", p->ramp.angle},
                                          {"leads.length", p->leads.length},
                                          {"leads.angle_rad", p->leads.angle},
                                          {"leads.radius", p->leads.radius},
                                          {"leads.sweep_rad", p->leads.sweep},
                                          {"leads.overcut", p->leads.overcut},
                                          {"start_position[0]", p->startPosition.x},
                                          {"start_position[1]", p->startPosition.y},
                                          {"corners.mitre_limit", p->corners.mitreLimit}});
        return key.empty() ? nonFiniteTolerance(p->tolerance) : key;
    }
    if (const auto* d = std::get_if<toolpath::DrillParams>(&params)) {
        std::string key = firstNonFinite({{"start_depth", d->startDepth},
                                          {"cut_depth", d->cutDepth},
                                          {"safe_z", d->safeZ},
                                          {"retract_gap", d->retractGap},
                                          {"dwell_time", d->dwellTime},
                                          {"start_position[0]", d->startPosition.x},
                                          {"start_position[1]", d->startPosition.y}});
        return key.empty() ? nonFiniteTolerance(d->tolerance) : key;
    }
    if (const auto* e = std::get_if<toolpath::EngraveParams>(&params)) {
        std::string key = firstNonFinite({{"depth", e->depth},
                                          {"safe_z", e->safeZ},
                                          {"stepover", e->stepover},
                                          {"raster_angle_rad", e->rasterAngle},
                                          {"start_position[0]", e->startPosition.x},
                                          {"start_position[1]", e->startPosition.y}});
        return key.empty() ? nonFiniteTolerance(e->tolerance) : key;
    }
    const auto& p = std::get<toolpath::PocketParams>(params);
    std::string key = firstNonFinite({{"start_depth", p.startDepth},
                                      {"cut_depth", p.cutDepth},
                                      {"last_pass_thickness", p.lastPassThickness},
                                      {"safe_z", p.safeZ},
                                      {"raster_angle_rad", p.rasterAngle},
                                      {"allowance", p.allowance},
                                      {"ramp_distance", p.rampDistance}});
    return key.empty() ? nonFiniteTolerance(p.tolerance) : key;
}

bool spindleOffAllowed(const ToolpathRecord& record) {
    return !record.tool.spindleOff || record.operation() == ToolpathOperation::Engrave;
}

bool ToolpathRecord::operator==(const ToolpathRecord& other) const {
    if (!(id == other.id && name == other.name && params == other.params && selection == other.selection &&
          tool == other.tool && toolSource == other.toolSource)) {
        return false;
    }
    if (moves == other.moves) {
        return true;  // same list, or both null
    }
    return moves && other.moves && *moves == *other.moves;
}

}  // namespace carvetto::document
