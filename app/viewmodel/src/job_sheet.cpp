#include "carvetto/viewmodel/job_sheet.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <system_error>
#include <locale>
#include <map>
#include <sstream>
#include <tuple>

#include "carvetto/document/units.hpp"
#include "carvetto/viewmodel/save_toolpaths.hpp"
#include "carvetto/viewmodel/file_session.hpp"

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

/// Included toolpaths the layout thumbnail leaves out (more than `kJobSheetMaxToolpathRows`, or
/// dropped from the stored sheet).
std::size_t thumbnailOmitted(const JobSheet& s) {
    std::size_t drawable = 0;
    for (const JobSheetToolpath& t : s.toolpaths) {
        if (t.included && !t.bounds.isEmpty()) {
            ++drawable;
        }
    }
    return s.thumbnailOmitted + (drawable > kJobSheetMaxToolpathRows ? drawable - kJobSheetMaxToolpathRows : 0);
}

/// Count of rows the sheet really has (the stored vector can be shorter than the job's).
std::size_t total(std::size_t stored, std::size_t declared) { return std::max(stored, declared); }

std::size_t shownTools(const JobSheet& s) { return std::min(s.tools.size(), kJobSheetMaxToolRows); }
std::size_t shownToolpaths(const JobSheet& s) { return std::min(s.toolpaths.size(), kJobSheetMaxToolpathRows); }

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
    // Tool table keyed on (number, name, diameter): a logarithmic lookup however many tools there are.
    // The value is the index in s.tools, or npos for a tool beyond the stored rows (still counted).
    constexpr std::size_t npos = static_cast<std::size_t>(-1);
    using ToolIdent = std::tuple<int, std::string, bool, double>;
    std::map<ToolIdent, std::size_t> toolIndex;
    s.toolpathsTotal = summary.entries.size();
    for (const ToolpathSummaryEntry& e : summary.entries) {
        JobSheetToolpath t;
        t.name = clipJobSheetText(e.name);
        t.toolName = clipJobSheetText(e.toolName);
        t.toolNumber = e.toolNumber;
        t.included = e.included;
        t.moves = e.moves;
        t.time = e.time;
        t.bounds = e.bounds;
        const auto it = records.find(e.id);
        if (it != records.end()) {
            const document::ToolpathRecord& r = *it->second;
            t.operation = clipJobSheetText(operationName(r.operation()));
            t.toolDiameterMm = r.tool.diameter;
            t.feedRateMmPerMin = r.tool.feedRate;
            t.plungeRateMmPerMin = r.tool.plungeRate;
            t.spindleRpm = r.tool.spindleSpeed;
        }
        if (e.included) {
            const double d = t.toolDiameterMm;
            const ToolIdent key{t.toolNumber, t.toolName, std::isnan(d), std::isnan(d) ? 0.0 : d};
            auto found = toolIndex.find(key);
            if (found == toolIndex.end()) {
                std::size_t idx = npos;
                if (s.tools.size() < kJobSheetMaxToolRows) {
                    idx = s.tools.size();
                    s.tools.push_back({t.toolName, t.toolNumber, t.toolDiameterMm, {}, 0});
                }
                found = toolIndex.emplace(key, idx).first;
                ++s.toolsTotal;
            }
            if (found->second != npos) {
                JobSheetTool& tool = s.tools[found->second];
                if (tool.toolpaths.size() < kJobSheetMaxToolpathsPerTool) {
                    tool.toolpaths.push_back(t.name);
                }
                ++tool.toolpathsTotal;
            }
        }
        if (s.toolpaths.size() < kJobSheetMaxToolpathRows) {
            s.toolpaths.push_back(std::move(t));
        } else if (e.included && !e.bounds.isEmpty()) {
            ++s.thumbnailOmitted;
        }
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
        if (total(t.toolpaths.size(), t.toolpathsTotal) > t.toolpaths.size() ||
            t.toolpaths.size() > kJobSheetMaxToolpathsPerTool) {
            o += ", ... and " + std::to_string(total(t.toolpaths.size(), t.toolpathsTotal) -
                                               std::min(t.toolpaths.size(), kJobSheetMaxToolpathsPerTool)) + " more";
        }
        o += "\n";
    }
    if (shownTools(s) < total(s.tools.size(), s.toolsTotal)) {
        o += "  ... and " + std::to_string(total(s.tools.size(), s.toolsTotal) - shownTools(s)) + " more tools\n";
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
    if (shownToolpaths(s) < total(s.toolpaths.size(), s.toolpathsTotal)) {
        o += "... and " + std::to_string(total(s.toolpaths.size(), s.toolpathsTotal) - shownToolpaths(s)) +
             " more toolpaths\n";
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
        if (const std::size_t omitted = thumbnailOmitted(s); omitted > 0) {
            o += "<p>The layout shows the first " + std::to_string(kJobSheetMaxToolpathRows) + " toolpaths; " +
                 std::to_string(omitted) + " more are not drawn.</p>\n";
        }
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
        if (total(t.toolpaths.size(), t.toolpathsTotal) > t.toolpaths.size() ||
            t.toolpaths.size() > kJobSheetMaxToolpathsPerTool) {
            names += ", ... and " + std::to_string(total(t.toolpaths.size(), t.toolpathsTotal) -
                                                   std::min(t.toolpaths.size(), kJobSheetMaxToolpathsPerTool)) + " more";
        }
        o += "<tr>" + td("T" + std::to_string(t.number)) + td(t.name) + td(f.len(t.diameterMm)) + "<td>" + names +
             "</td></tr>\n";
    }
    if (shownTools(s) < total(s.tools.size(), s.toolsTotal)) {
        o += "<tr><td colspan=\"4\">... and " + std::to_string(total(s.tools.size(), s.toolsTotal) - shownTools(s)) +
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
    if (shownToolpaths(s) < total(s.toolpaths.size(), s.toolpathsTotal)) {
        o += "<p>... and " + std::to_string(total(s.toolpaths.size(), s.toolpathsTotal) - shownToolpaths(s)) +
             " more toolpaths</p>\n";
    }
    o += "</body>\n</html>\n";
    return o;
}

const char* jobSheetExtension(JobSheetFormat format) {
    switch (format) {
        case JobSheetFormat::Html: return ".html";
        case JobSheetFormat::Text: return ".txt";
        case JobSheetFormat::Pdf: return ".pdf";
    }
    return ".txt";
}

std::string defaultJobSheetFileName(const std::string& title, JobSheetFormat format) {
    std::string stem = sanitizeFileStem(title);
    if (stem.empty()) {
        stem = "job-sheet";
    }
    return stem + jobSheetExtension(format);
}

std::string checkJobSheetPath(const std::filesystem::path& path, JobSheetFormat format) {
    if (path.empty() || !path.has_filename()) {
        return "no file name given";
    }
    for (const std::filesystem::path& part : path) {
        if (part == "..") {
            return "the path contains '..'";
        }
    }
    const std::string stem = pathToUtf8(path.stem());
    std::string ext = pathToUtf8(path.extension());
    for (char& c : ext) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    const bool extOk = format == JobSheetFormat::Html ? (ext == ".html" || ext == ".htm") : ext == jobSheetExtension(format);
    if (!extOk) {
        return std::string("the file name must end in ") + jobSheetExtension(format);
    }
    if (stem.empty() || sanitizeFileStem(stem) != stem) {
        return "the file name has characters that are not allowed";
    }
    std::error_code ec;
    if (std::filesystem::is_directory(path, ec)) {
        return "the path is a folder";
    }
    return {};
}

JobSheetWriteResult writeJobSheet(const JobSheet& sheet, JobSheetFormat format, const std::filesystem::path& path,
                                  bool overwrite) {
    JobSheetWriteResult res;
    if (format == JobSheetFormat::Pdf) {
        res.error = "PDF is written by the app (Qt printing), not here";
        return res;
    }
    res.error = checkJobSheetPath(path, format);
    if (!res.error.empty()) {
        return res;
    }
    SaveOutput out;
    out.ok = true;
    OutputFile f;
    f.path = path;
    f.text = format == JobSheetFormat::Html ? renderJobSheetHtml(sheet) : renderJobSheetText(sheet);
    out.files.push_back(std::move(f));
    const WriteResult w = writeOutput(out, overwrite);
    res.ok = w.ok;
    res.error = w.error;
    if (w.ok && !w.written.empty()) {
        res.written = w.written.front();
    }
    return res;
}

}  // namespace carvetto::viewmodel
