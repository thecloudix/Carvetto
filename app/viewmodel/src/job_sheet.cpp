#include "carvetto/viewmodel/job_sheet.hpp"

#include <algorithm>
#include <cmath>
#include <locale>
#include <map>
#include <sstream>

#include "carvetto/document/units.hpp"

namespace carvetto::viewmodel {

namespace {

using document::Units;

std::string fixed(double v, int decimals) {
    if (!std::isfinite(v)) {
        return "n/a";
    }
    std::ostringstream os;
    os.imbue(std::locale::classic());
    os.setf(std::ios::fixed);
    os.precision(decimals);
    // Avoid "-0.00".
    const double scale = std::pow(10.0, decimals);
    if (std::fabs(v) * scale < 0.5) {
        v = 0.0;
    }
    os << v;
    return os.str();
}

std::string textSafe(const std::string& s) {
    std::string out = clipJobSheetText(s);
    for (char& c : out) {
        if (static_cast<unsigned char>(c) < 0x20 || c == 0x7f) {
            c = ' ';
        }
    }
    return out;
}

const char* jobTypeName(document::JobType t) {
    switch (t) {
        case document::JobType::SingleSided: return "Single sided";
        case document::JobType::DoubleSided: return "Double sided";
        case document::JobType::Rotary: return "Rotary";
    }
    return "Unknown";
}

const char* zZeroName(document::ZZero z) {
    switch (z) {
        case document::ZZero::MaterialSurface: return "Material surface";
        case document::ZZero::MachineBed: return "Machine bed";
        case document::ZZero::CylinderAxis: return "Cylinder axis";
    }
    return "Unknown";
}

const char* datumName(document::Datum d) {
    switch (d) {
        case document::Datum::BottomLeft: return "Bottom left";
        case document::Datum::BottomRight: return "Bottom right";
        case document::Datum::TopLeft: return "Top left";
        case document::Datum::TopRight: return "Top right";
        case document::Datum::Centre: return "Centre";
    }
    return "Unknown";
}

/// Values in the job's display units.
struct Fmt {
    bool inches{false};
    std::string len(double mm) const { return fixed(inches ? mm / 25.4 : mm, inches ? 3 : 2) + (inches ? " in" : " mm"); }
    std::string rate(double mmPerMin) const {
        return fixed(inches ? mmPerMin / 25.4 : mmPerMin, inches ? 2 : 0) + (inches ? " in/min" : " mm/min");
    }
    std::string rpm(double r) const { return r > 0.0 ? fixed(r, 0) + " rpm" : std::string("not set"); }
};

std::string layoutSvg(const JobSheet& s) {
    geometry::Box2 view = s.material;
    for (const JobSheetToolpath& t : s.toolpaths) {
        if (t.included) {
            view.expand(t.bounds);
        }
    }
    if (view.isEmpty() || view.width() <= 0.0 || view.height() <= 0.0) {
        return {};
    }
    const double pad = std::max(view.width(), view.height()) * 0.03;
    view = view.inflated(pad);
    auto rect = [&](const geometry::Box2& b, const std::string& cls, const std::string& title) {
        std::string r = "<rect class=\"" + cls + "\" x=\"" + fixed(b.min.x - view.min.x, 2) + "\" y=\"" +
                        fixed(view.max.y - b.max.y, 2) + "\" width=\"" + fixed(b.width(), 2) + "\" height=\"" +
                        fixed(b.height(), 2) + "\">";
        if (!title.empty()) {
            r += "<title>" + escapeHtml(clipJobSheetText(title)) + "</title>";
        }
        return r + "</rect>\n";
    };
    const double aspectHeight = std::clamp(400.0 * view.height() / view.width(), 40.0, 4000.0);
    std::string svg = "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"400\" height=\"" +
                      fixed(aspectHeight, 0) + "\" viewBox=\"0 0 " + fixed(view.width(), 2) +
                      " " + fixed(view.height(), 2) + "\">\n";
    svg += "<style>rect{vector-effect:non-scaling-stroke;stroke-width:1}"
           ".m{fill:#f4e3c1;stroke:#7a5c2e}.t{fill:#3b82f633;stroke:#1d4ed8}</style>\n";
    if (!s.material.isEmpty()) {
        svg += rect(s.material, "m", "Material");
    }
    std::size_t drawn = 0;
    for (const JobSheetToolpath& t : s.toolpaths) {
        if (t.included && !t.bounds.isEmpty() && drawn++ < kJobSheetMaxToolpathRows) {
            svg += rect(t.bounds, "t", t.name);
        }
    }
    return svg + "</svg>\n";
}

}  // namespace

std::string clipJobSheetText(const std::string& text) {
    if (text.size() <= kJobSheetMaxNameBytes) {
        return text;
    }
    std::size_t n = kJobSheetMaxNameBytes;
    while (n > 0 && (static_cast<unsigned char>(text[n]) & 0xC0) == 0x80) {
        --n;
    }
    return text.substr(0, n) + "...";
}

std::string escapeHtml(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (const char c : text) {
        switch (c) {
            case '&': out += "&amp;"; break;
            case '<': out += "&lt;"; break;
            case '>': out += "&gt;"; break;
            case '"': out += "&quot;"; break;
            case '\'': out += "&#39;"; break;
            default:
                if (static_cast<unsigned char>(c) < 0x20 || c == 0x7f) {
                    out += ' ';
                } else {
                    out += c;
                }
        }
    }
    return out;
}

JobSheet makeJobSheet(const document::Document& doc, const JobSummary& summary, const std::string& title) {
    JobSheet s;
    const document::Job& job = doc.job();
    s.title = title.empty() ? "Untitled job" : title;
    s.jobType = jobTypeName(job.type);
    s.widthMm = job.sizeMm.x;
    s.heightMm = job.sizeMm.y;
    s.thicknessMm = job.sizeMm.z;
    s.zZero = zZeroName(job.zZero);
    s.datum = datumName(job.datum);
    s.inches = job.displayUnits == Units::Inches;
    s.material = job.materialBoundsMm();
    s.summary = summary;
    s.readout = makeTimeReadout(summary);

    std::map<document::ToolpathId, const document::ToolpathRecord*> records;
    for (const document::ToolpathRecord& r : doc.toolpaths()) {
        records[r.id] = &r;
    }
    for (const ToolpathSummaryEntry& e : summary.entries) {
        JobSheetToolpath t;
        t.name = e.name;
        t.toolName = e.toolName;
        t.toolNumber = e.toolNumber;
        t.included = e.included;
        t.moves = e.moves;
        t.time = e.time;
        t.bounds = e.bounds;
        const auto it = records.find(e.id);
        if (it != records.end()) {
            const document::ToolpathRecord& r = *it->second;
            t.operation = operationName(r.operation());
            t.toolDiameterMm = r.tool.diameter;
            t.feedRateMmPerMin = r.tool.feedRate;
            t.plungeRateMmPerMin = r.tool.plungeRate;
            t.spindleRpm = r.tool.spindleSpeed;
        }
        if (e.included) {
            auto tool = std::find_if(s.tools.begin(), s.tools.end(), [&](const JobSheetTool& x) {
                return x.name == t.toolName && x.number == t.toolNumber && x.diameterMm == t.toolDiameterMm;
            });
            if (tool == s.tools.end()) {
                s.tools.push_back({t.toolName, t.toolNumber, t.toolDiameterMm, {}});
                tool = s.tools.end() - 1;
            }
            tool->toolpaths.push_back(t.name);
        }
        s.toolpaths.push_back(std::move(t));
    }
    return s;
}

std::string renderJobSheetText(const JobSheet& s) {
    const Fmt f{s.inches};
    std::string o;
    o += "JOB SHEET: " + textSafe(s.title) + "\n\n";
    o += "Material setup\n";
    o += "  Type: " + s.jobType + "\n";
    o += "  Size: " + f.len(s.widthMm) + " x " + f.len(s.heightMm) + " x " + f.len(s.thicknessMm) + "\n";
    o += "  Z zero: " + s.zZero + "\n";
    o += "  XY datum: " + s.datum + "\n\n";
    o += "Toolpaths summary\n";
    o += "  " + textSafe(s.readout.headline) + "\n";
    if (!s.readout.breakdown.empty()) {
        o += "  " + textSafe(s.readout.breakdown) + "\n";
    }
    o += "  Toolpaths: " + std::to_string(s.summary.toolpaths) + " (" + std::to_string(s.summary.included) +
         " in the estimate)\n";
    o += "  Tools: " + std::to_string(s.summary.distinctTools) + ", tool changes: " +
         std::to_string(s.summary.toolChanges) + "\n";
    for (std::size_t i = 0; i < s.readout.warnings.size() && i < kJobSheetMaxWarnings; ++i) {
        o += "  Warning: " + textSafe(s.readout.warnings[i]) + "\n";
    }
    if (s.readout.warnings.size() > kJobSheetMaxWarnings) {
        o += "  ... and " + std::to_string(s.readout.warnings.size() - kJobSheetMaxWarnings) + " more warnings\n";
    }
    o += "\nTools\n";
    if (s.tools.empty()) {
        o += "  (none)\n";
    }
    for (std::size_t k = 0; k < s.tools.size() && k < kJobSheetMaxToolRows; ++k) {
        const JobSheetTool& t = s.tools[k];
        o += "  T" + std::to_string(t.number) + " " + textSafe(t.name) + ", diameter " + f.len(t.diameterMm) + ": ";
        for (std::size_t i = 0; i < t.toolpaths.size() && i < kJobSheetMaxToolpathsPerTool; ++i) {
            o += (i ? ", " : "") + textSafe(t.toolpaths[i]);
        }
        if (t.toolpaths.size() > kJobSheetMaxToolpathsPerTool) {
            o += ", ... and " + std::to_string(t.toolpaths.size() - kJobSheetMaxToolpathsPerTool) + " more";
        }
        o += "\n";
    }
    if (s.tools.size() > kJobSheetMaxToolRows) {
        o += "  ... and " + std::to_string(s.tools.size() - kJobSheetMaxToolRows) + " more tools\n";
    }
    o += "\nToolpaths list\n";
    if (s.toolpaths.empty()) {
        o += "  (none)\n";
    }
    std::size_t n = 0;
    for (const JobSheetToolpath& t : s.toolpaths) {
        if (n >= kJobSheetMaxToolpathRows) {
            break;
        }
        o += std::to_string(++n) + ". " + textSafe(t.name) + " [" + textSafe(t.operation) + "]" +
             (t.included ? "" : " (not in the estimate)") + "\n";
        o += "     Tool: T" + std::to_string(t.toolNumber) + " " + textSafe(t.toolName) + ", diameter " +
             f.len(t.toolDiameterMm) + "\n";
        o += "     Feed: " + f.rate(t.feedRateMmPerMin) + ", plunge: " + f.rate(t.plungeRateMmPerMin) +
             ", spindle: " + f.rpm(t.spindleRpm) + "\n";
        o += "     Time: " + (t.included ? formatDuration(t.time.total()) : std::string("-")) + ", moves: " +
             std::to_string(t.moves) + "\n";
    }
    if (s.toolpaths.size() > kJobSheetMaxToolpathRows) {
        o += "... and " + std::to_string(s.toolpaths.size() - kJobSheetMaxToolpathRows) + " more toolpaths\n";
    }
    return o;
}

std::string renderJobSheetHtml(const JobSheet& s) {
    const Fmt f{s.inches};
    auto e = [](const std::string& x) { return escapeHtml(clipJobSheetText(x)); };
    auto td = [&](const std::string& x) { return "<td>" + e(x) + "</td>"; };
    std::string o;
    o += "<!DOCTYPE html>\n<html lang=\"en\">\n<head>\n<meta charset=\"utf-8\">\n<title>Job sheet - " + e(s.title) +
         "</title>\n<style>body{font-family:sans-serif}table{border-collapse:collapse}"
         "td,th{border:1px solid #888;padding:2px 8px;text-align:left}</style>\n</head>\n<body>\n";
    o += "<h1>Job sheet: " + e(s.title) + "</h1>\n";
    const std::string svg = layoutSvg(s);
    if (!svg.empty()) {
        o += "<h2>Job layout</h2>\n" + svg;
    }
    o += "<h2>Material setup</h2>\n<table>\n";
    o += "<tr><th>Type</th>" + td(s.jobType) + "</tr>\n";
    o += "<tr><th>Size</th>" + td(f.len(s.widthMm) + " x " + f.len(s.heightMm) + " x " + f.len(s.thicknessMm)) +
         "</tr>\n";
    o += "<tr><th>Z zero</th>" + td(s.zZero) + "</tr>\n";
    o += "<tr><th>XY datum</th>" + td(s.datum) + "</tr>\n</table>\n";
    o += "<h2>Toolpaths summary</h2>\n<p>" + e(s.readout.headline) + "</p>\n";
    if (!s.readout.breakdown.empty()) {
        o += "<p>" + e(s.readout.breakdown) + "</p>\n";
    }
    o += "<p>Toolpaths: " + std::to_string(s.summary.toolpaths) + " (" + std::to_string(s.summary.included) +
         " in the estimate). Tools: " + std::to_string(s.summary.distinctTools) +
         ", tool changes: " + std::to_string(s.summary.toolChanges) + ".</p>\n";
    if (!s.readout.warnings.empty()) {
        o += "<ul class=\"warnings\">\n";
        for (std::size_t i = 0; i < s.readout.warnings.size() && i < kJobSheetMaxWarnings; ++i) {
            o += "<li>" + e(s.readout.warnings[i]) + "</li>\n";
        }
        if (s.readout.warnings.size() > kJobSheetMaxWarnings) {
            o += "<li>... and " + std::to_string(s.readout.warnings.size() - kJobSheetMaxWarnings) +
                 " more warnings</li>\n";
        }
        o += "</ul>\n";
    }
    o += "<h2>Tools</h2>\n<table>\n<tr><th>Number</th><th>Name</th><th>Diameter</th><th>Toolpaths</th></tr>\n";
    for (std::size_t k = 0; k < s.tools.size() && k < kJobSheetMaxToolRows; ++k) {
        const JobSheetTool& t = s.tools[k];
        std::string names;
        for (std::size_t i = 0; i < t.toolpaths.size() && i < kJobSheetMaxToolpathsPerTool; ++i) {
            names += (i ? ", " : "") + e(t.toolpaths[i]);
        }
        if (t.toolpaths.size() > kJobSheetMaxToolpathsPerTool) {
            names += ", ... and " + std::to_string(t.toolpaths.size() - kJobSheetMaxToolpathsPerTool) + " more";
        }
        o += "<tr>" + td("T" + std::to_string(t.number)) + td(t.name) + td(f.len(t.diameterMm)) + "<td>" + names +
             "</td></tr>\n";
    }
    if (s.tools.size() > kJobSheetMaxToolRows) {
        o += "<tr><td colspan=\"4\">... and " + std::to_string(s.tools.size() - kJobSheetMaxToolRows) +
             " more tools</td></tr>\n";
    }
    o += "</table>\n<h2>Toolpaths list</h2>\n<table>\n<tr><th>#</th><th>Name</th><th>Operation</th><th>Tool</th>"
         "<th>Feed</th><th>Plunge</th><th>Spindle</th><th>Time</th><th>Moves</th></tr>\n";
    std::size_t n = 0;
    for (const JobSheetToolpath& t : s.toolpaths) {
        if (n >= kJobSheetMaxToolpathRows) {
            break;
        }
        o += "<tr>" + td(std::to_string(++n)) + td(t.name + (t.included ? "" : " (not in the estimate)")) +
             td(t.operation) + td("T" + std::to_string(t.toolNumber) + " " + t.toolName) +
             td(f.rate(t.feedRateMmPerMin)) + td(f.rate(t.plungeRateMmPerMin)) + td(f.rpm(t.spindleRpm)) +
             td(t.included ? formatDuration(t.time.total()) : std::string("-")) + td(std::to_string(t.moves)) +
             "</tr>\n";
    }
    o += "</table>\n";
    if (s.toolpaths.size() > kJobSheetMaxToolpathRows) {
        o += "<p>... and " + std::to_string(s.toolpaths.size() - kJobSheetMaxToolpathRows) + " more toolpaths</p>\n";
    }
    o += "</body>\n</html>\n";
    return o;
}

}  // namespace carvetto::viewmodel
