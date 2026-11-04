// Throwaway scene dump/load (verifier): contours + the ProfileParams fields the harnesses vary.
#pragma once
#include <cstdio>
#include <string>
#include <vector>

#include "carvetto/geometry/contour.hpp"
#include "carvetto/toolpath/profile.hpp"

inline void writeScene(const char* path, const std::vector<carvetto::geometry::Contour>& cs, const carvetto::toolpath::ProfileParams& p, double D, int vbitDeg = 0) {
    using namespace carvetto;
    FILE* f = std::fopen(path, "w");
    for (auto& c : cs) {
        std::fprintf(f, "C %d\n", (int)c.closed);
        for (auto& sp : c.spans) {
            if (auto* l = std::get_if<geometry::LineSpan>(&sp)) std::fprintf(f, "L %a %a %a %a\n", l->start.x, l->start.y, l->end.x, l->end.y);
            else if (auto* a = std::get_if<geometry::ArcSpan>(&sp)) std::fprintf(f, "A %a %a %a %a %a %d\n", a->centre.x, a->centre.y, a->radius, a->startAngle, a->endAngle, (int)a->direction);
            else if (auto* b = std::get_if<geometry::BezierSpan>(&sp)) std::fprintf(f, "B %a %a %a %a %a %a %a %a\n", b->p0.x, b->p0.y, b->p1.x, b->p1.y, b->p2.x, b->p2.y, b->p3.x, b->p3.y);
        }
    }
    std::fprintf(f, "P %a %d %a %d %d %a %a %a %a %a %d %a %d %d %a %a %d %d %a %a %d %d %a %d %a %d %d %a %a %a %a %d %a %d %d %a %a %d %d %d %d %d %d %a %a %a\n",
                 D, vbitDeg, p.allowance, (int)p.side, (int)p.direction, p.passes.startDepth, p.passes.cutDepth, p.passes.passDepth, p.passes.lastPassThickness, p.safeZ,
                 (int)p.separateLastPass, p.lastPassAllowance, (int)p.lastPassReverse, (int)p.tabs.enabled, p.tabs.length, p.tabs.thickness, (int)p.tabs.spacing, p.tabs.count, p.tabs.distance,
                 0.0, (int)p.tabs.avoidCorners, (int)p.tabs.threeD, 0.0, (int)p.ramp.type, p.ramp.distance, (int)p.ramp.useAngle, (int)p.ramp.onLeadIn, p.ramp.angle,
                 p.leads.length, p.leads.angle, p.leads.radius, (int)p.leads.type, p.leads.sweep, (int)p.leads.leadOut, 0, p.leads.overcut, 0.0,
                 (int)p.startAt, (int)p.startAnchor, (int)p.order.selectionOrder, (int)p.order.leftToRight, (int)p.order.bottomToTop, (int)p.order.grid,
                 p.startPosition.x, p.startPosition.y, p.corners.mitreLimit);
    std::fprintf(f, "K %d %d\n", (int)p.corners.sharpExternal, (int)p.corners.sharpInternal);
    std::fclose(f);
}

inline bool readScene(const char* path, std::vector<carvetto::geometry::Contour>& cs, carvetto::toolpath::ProfileParams& p, double& D, int& vbitDeg) {
    using namespace carvetto;
    using namespace carvetto::toolpath;
    FILE* f = std::fopen(path, "r");
    if (!f) return false;
    char tag[4];
    while (std::fscanf(f, "%3s", tag) == 1) {
        std::string t = tag;
        if (t == "C") { int cl; std::fscanf(f, "%d", &cl); cs.emplace_back(); cs.back().closed = cl; }
        else if (t == "L") { double a, b, c, d; std::fscanf(f, "%la %la %la %la", &a, &b, &c, &d); cs.back().spans.push_back(geometry::LineSpan{{a, b}, {c, d}}); }
        else if (t == "A") { double a, b, c, d, e; int dir; std::fscanf(f, "%la %la %la %la %la %d", &a, &b, &c, &d, &e, &dir); cs.back().spans.push_back(geometry::ArcSpan{{a, b}, c, d, e, (geometry::ArcDirection)dir}); }
        else if (t == "B") { double v[8]; std::fscanf(f, "%la %la %la %la %la %la %la %la", &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &v[6], &v[7]); cs.back().spans.push_back(geometry::BezierSpan{{v[0], v[1]}, {v[2], v[3]}, {v[4], v[5]}, {v[6], v[7]}}); }
        else if (t == "P") {
            int side, dir, sep, rev, en, sp, avoid, th3, rt, ua, ol, lt, lo, dummy, sa, an, so, l2r, b2t, grid;
            double dd1, dd2, dd3;
            std::fscanf(f, "%la %d %la %d %d %la %la %la %la %la %d %la %d %d %la %la %d %d %la %la %d %d %la %d %la %d %d %la %la %la %la %d %la %d %d %la %la %d %d %d %d %d %d %la %la %la",
                        &D, &vbitDeg, &p.allowance, &side, &dir, &p.passes.startDepth, &p.passes.cutDepth, &p.passes.passDepth, &p.passes.lastPassThickness, &p.safeZ,
                        &sep, &p.lastPassAllowance, &rev, &en, &p.tabs.length, &p.tabs.thickness, &sp, &p.tabs.count, &p.tabs.distance,
                        &dd1, &avoid, &th3, &dd2, &rt, &p.ramp.distance, &ua, &ol, &p.ramp.angle,
                        &p.leads.length, &p.leads.angle, &p.leads.radius, &lt, &p.leads.sweep, &lo, &dummy, &p.leads.overcut, &dd3,
                        &sa, &an, &so, &l2r, &b2t, &grid, &p.startPosition.x, &p.startPosition.y, &p.corners.mitreLimit);
            p.side = (ProfileSide)side; p.direction = (CutDirection)dir; p.separateLastPass = sep; p.lastPassReverse = rev; p.tabs.enabled = en;
            p.tabs.spacing = (TabSpacing)sp; p.tabs.avoidCorners = avoid; p.tabs.threeD = th3; p.ramp.type = (RampType)rt; p.ramp.useAngle = ua; p.ramp.onLeadIn = ol;
            p.leads.type = (LeadType)lt; p.leads.leadOut = lo; p.startAt = (StartAt)sa; p.startAnchor = (BoxAnchor)an;
            p.order.selectionOrder = so; p.order.leftToRight = l2r; p.order.bottomToTop = b2t; p.order.grid = grid;
        } else if (t == "K") { int a, b; std::fscanf(f, "%d %d", &a, &b); p.corners.sharpExternal = a; p.corners.sharpInternal = b; }
    }
    std::fclose(f);
    return true;
}
