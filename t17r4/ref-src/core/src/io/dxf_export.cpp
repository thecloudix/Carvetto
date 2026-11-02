#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>
#include <vector>

#include "carvetto/geometry/span.hpp"
#include "carvetto/io/dxf.hpp"
#include "dxf_pairs.hpp"
#include "dxf_text.hpp"

// DXF R2000 (AC1015) ASCII export (ADR-0015). The file carries the minimal structure the public
// reference describes for R2000: HEADER, CLASSES, TABLES (VPORT, LTYPE, LAYER, STYLE, VIEW, UCS,
// APPID, DIMSTYLE, BLOCK_RECORD), BLOCKS (*Model_Space, *Paper_Space), ENTITIES and OBJECTS (root
// dictionary). Handles are assigned sequentially in output order, so output is deterministic.
namespace carvetto::io {

namespace {

using geometry::ArcSpan;
using geometry::BezierSpan;
using geometry::Contour;
using geometry::kPi;
using geometry::LineSpan;
using geometry::Span;
using geometry::Vec2;

class Writer {
   public:
    void pair(int code, std::string_view value) {
        text_ += std::to_string(code);
        text_ += '\n';
        text_ += value;
        text_ += '\n';
    }
    void pair(int code, long long value) { pair(code, std::to_string(value)); }
    void real(int code, double value) {
        // Shortest representation that reads back to the same double.
        std::array<char, 64> buf{};
        if (value == 0.0) {
            value = 0.0;  // no "-0"
        }
        const auto [ptr, ec] = std::to_chars(buf.data(), buf.data() + buf.size(), value);
        std::string s(buf.data(), ec == std::errc{} ? ptr : buf.data());
        if (s.find_first_of(".eEn") == std::string::npos) {
            s += ".0";
        }
        pair(code, s);
    }
    void point(int code, Vec2 p) {
        real(code, p.x);
        real(code + 10, p.y);
    }
    std::string handle() {
        static constexpr char kHex[] = "0123456789ABCDEF";
        std::string s;
        std::uint64_t v = next_++;
        do {
            s.insert(s.begin(), kHex[v & 0xFu]);
            v >>= 4;
        } while (v != 0);
        return s;
    }
    std::uint64_t peekHandle() const { return next_; }
    std::string& text() { return text_; }

   private:
    std::string text_;
    std::uint64_t next_{0x20};
};

std::string hex(std::uint64_t v) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string s;
    do {
        s.insert(s.begin(), kHex[v & 0xFu]);
        v >>= 4;
    } while (v != 0);
    return s;
}

bool finite(Vec2 p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}

bool spanFinite(const Span& s) {
    return std::visit(
        [](const auto& v) {
            using T = std::decay_t<decltype(v)>;
            if constexpr (std::is_same_v<T, LineSpan>) {
                return finite(v.start) && finite(v.end);
            } else if constexpr (std::is_same_v<T, ArcSpan>) {
                return finite(v.centre) && std::isfinite(v.radius) && v.radius > 0.0 &&
                       std::isfinite(v.startAngle) && std::isfinite(v.endAngle);
            } else {
                return finite(v.p0) && finite(v.p1) && finite(v.p2) && finite(v.p3);
            }
        },
        s);
}

/// Exactly zero length: a line whose ends coincide, or a Bezier whose four points coincide.
bool isZeroLength(const Span& s) {
    if (const auto* l = std::get_if<LineSpan>(&s)) {
        return l->start == l->end;
    }
    if (const auto* b = std::get_if<BezierSpan>(&s)) {
        return b->p0 == b->p1 && b->p1 == b->p2 && b->p2 == b->p3;
    }
    return false;
}

/// Layer names may not contain these characters (DXF symbol-table naming rules).
std::string sanitiseName(const std::string& name) {
    std::string out;
    for (const char c : name) {
        const auto u = static_cast<unsigned char>(c);
        const bool bad = u < 0x20 || std::string_view{"<>/\\\":;?*|=`"}.find(c) != std::string_view::npos;
        out += bad ? '_' : c;
    }
    std::string_view trimmed = detail::trim(out);
    return trimmed.empty() ? std::string{"Layer"} : std::string(trimmed);
}

/// A run of spans written as one entity.
struct Piece {
    enum class Kind { Poly, Spline, Circle } kind{Kind::Poly};
    std::size_t first{0};
    std::size_t count{0};
};

std::vector<Piece> splitPieces(const Contour& c) {
    std::vector<Piece> pieces;
    // Only an exactly full circle (ADR-0005: coincident angles) is a CIRCLE; a near-full arc must
    // keep its gap, so it goes to an LWPOLYLINE.
    if (c.spans.size() == 1 && std::holds_alternative<ArcSpan>(c.spans[0]) &&
        std::fabs(std::get<ArcSpan>(c.spans[0]).sweepAngle()) >= geometry::kTwoPi) {
        pieces.push_back({Piece::Kind::Circle, 0, 1});
        return pieces;
    }
    for (std::size_t i = 0; i < c.spans.size(); ++i) {
        const Piece::Kind kind =
            std::holds_alternative<BezierSpan>(c.spans[i]) ? Piece::Kind::Spline : Piece::Kind::Poly;
        if (!pieces.empty() && pieces.back().kind == kind) {
            ++pieces.back().count;
        } else {
            pieces.push_back({kind, i, 1});
        }
    }
    return pieces;
}

class Exporter {
   public:
    Exporter(std::span<const DxfLayer> layers, std::span<const DxfContour> contours)
        : layers_(layers), contours_(contours) {}

    std::string run() {
        names_.reserve(layers_.size());
        std::set<std::string> used;
        for (const DxfLayer& l : layers_) {
            std::string base = sanitiseName(l.name);
            std::string name = base;
            for (int k = 2; used.count(detail::upper(name)) != 0; ++k) {
                name = base + "_" + std::to_string(k);
            }
            used.insert(detail::upper(name));
            names_.push_back(name);
        }
        // Handles are fixed by output order. Body first (to know $HANDSEED), then assemble.
        Writer body;
        writeTables(body);
        writeBlocks(body);
        writeEntities(body);
        writeObjects(body);
        Writer head;
        head.pair(0, "SECTION");
        head.pair(2, "HEADER");
        head.pair(9, "$ACADVER");
        head.pair(1, "AC1015");
        head.pair(9, "$DWGCODEPAGE");
        head.pair(3, "ANSI_1252");
        head.pair(9, "$HANDSEED");
        head.pair(5, hex(body.peekHandle()));
        head.pair(9, "$INSUNITS");
        head.pair(70, 4);
        head.pair(9, "$MEASUREMENT");
        head.pair(70, 1);
        head.pair(0, "ENDSEC");
        head.pair(0, "SECTION");
        head.pair(2, "CLASSES");
        head.pair(0, "ENDSEC");
        std::string out = std::move(head.text());
        out += body.text();
        out += "0\nEOF\n";
        return out;
    }

   private:
    void tableStart(Writer& w, std::string_view name, std::string& handle, long long count) {
        handle = w.handle();
        w.pair(0, "TABLE");
        w.pair(2, name);
        w.pair(5, handle);
        w.pair(330, "0");
        w.pair(100, "AcDbSymbolTable");
        w.pair(70, count);
    }
    void record(Writer& w, std::string_view type, const std::string& owner, std::string_view subclass) {
        w.pair(0, type);
        w.pair(5, w.handle());
        w.pair(330, owner);
        w.pair(100, "AcDbSymbolTableRecord");
        w.pair(100, subclass);
    }
    void ltype(Writer& w, const std::string& owner, std::string_view name, std::string_view description) {
        record(w, "LTYPE", owner, "AcDbLinetypeTableRecord");
        w.pair(2, name);
        w.pair(70, 0);
        w.pair(3, description);
        w.pair(72, 65);
        w.pair(73, 0);
        w.real(40, 0.0);
    }

    void writeTables(Writer& w) {
        w.pair(0, "SECTION");
        w.pair(2, "TABLES");
        std::string h;
        tableStart(w, "VPORT", h, 0);
        w.pair(0, "ENDTAB");
        tableStart(w, "LTYPE", h, 3);
        ltype(w, h, "ByBlock", "");
        ltype(w, h, "ByLayer", "");
        ltype(w, h, "Continuous", "Solid line");
        w.pair(0, "ENDTAB");
        tableStart(w, "LAYER", h, static_cast<long long>(layers_.size()));
        for (std::size_t i = 0; i < layers_.size(); ++i) {
            const DxfLayer& l = layers_[i];
            const int aci = (l.aci >= 1 && l.aci <= 255) ? l.aci : nearestAci(l.rgb & 0xFFFFFFu);
            record(w, "LAYER", h, "AcDbLayerTableRecord");
            w.pair(2, detail::encodeDxfString(names_[i]));
            w.pair(70, 0);
            w.pair(62, l.visible ? aci : -aci);
            if ((l.rgb & 0xFFFFFFu) != aciToRgb(aci)) {
                w.pair(420, static_cast<long long>(l.rgb & 0xFFFFFFu));
            }
            w.pair(6, "Continuous");
            w.pair(370, -3);
        }
        w.pair(0, "ENDTAB");
        tableStart(w, "STYLE", h, 1);
        record(w, "STYLE", h, "AcDbTextStyleTableRecord");
        w.pair(2, "Standard");
        w.pair(70, 0);
        w.real(40, 0.0);
        w.real(41, 1.0);
        w.real(50, 0.0);
        w.pair(71, 0);
        w.real(42, 2.5);
        w.pair(3, "txt");
        w.pair(4, "");
        w.pair(0, "ENDTAB");
        tableStart(w, "VIEW", h, 0);
        w.pair(0, "ENDTAB");
        tableStart(w, "UCS", h, 0);
        w.pair(0, "ENDTAB");
        tableStart(w, "APPID", h, 2);
        for (const std::string_view app : {std::string_view{"ACAD"}, std::string_view{"CARVETTO"}}) {
            record(w, "APPID", h, "AcDbRegAppTableRecord");
            w.pair(2, app);
            w.pair(70, 0);
        }
        w.pair(0, "ENDTAB");
        tableStart(w, "DIMSTYLE", h, 0);
        w.pair(100, "AcDbDimStyleTable");
        w.pair(0, "ENDTAB");
        tableStart(w, "BLOCK_RECORD", h, 2);
        record(w, "BLOCK_RECORD", h, "AcDbBlockTableRecord");
        modelSpaceRecord_ = hex(w.peekHandle() - 1);
        w.pair(2, "*Model_Space");
        record(w, "BLOCK_RECORD", h, "AcDbBlockTableRecord");
        paperSpaceRecord_ = hex(w.peekHandle() - 1);
        w.pair(2, "*Paper_Space");
        w.pair(0, "ENDTAB");
        w.pair(0, "ENDSEC");
    }

    void block(Writer& w, std::string_view name, const std::string& owner, bool paper) {
        w.pair(0, "BLOCK");
        w.pair(5, w.handle());
        w.pair(330, owner);
        w.pair(100, "AcDbEntity");
        if (paper) {
            w.pair(67, 1);
        }
        w.pair(8, "0");
        w.pair(100, "AcDbBlockBegin");
        w.pair(2, name);
        w.pair(70, 0);
        w.real(10, 0.0);
        w.real(20, 0.0);
        w.real(30, 0.0);
        w.pair(3, name);
        w.pair(1, "");
        w.pair(0, "ENDBLK");
        w.pair(5, w.handle());
        w.pair(330, owner);
        w.pair(100, "AcDbEntity");
        if (paper) {
            w.pair(67, 1);
        }
        w.pair(8, "0");
        w.pair(100, "AcDbBlockEnd");
    }

    void writeBlocks(Writer& w) {
        w.pair(0, "SECTION");
        w.pair(2, "BLOCKS");
        block(w, "*Model_Space", modelSpaceRecord_, false);
        block(w, "*Paper_Space", paperSpaceRecord_, true);
        w.pair(0, "ENDSEC");
    }

    void entityStart(Writer& w, std::string_view type, std::size_t layer, std::string_view subclass) {
        w.pair(0, type);
        w.pair(5, w.handle());
        w.pair(330, modelSpaceRecord_);
        w.pair(100, "AcDbEntity");
        w.pair(8, detail::encodeDxfString(names_[layer]));
        w.pair(100, subclass);
    }

    /// An arc written as several bulge segments (sweep > pi), restored exactly on import.
    struct SplitArc {
        std::size_t firstSegment{0};
        std::size_t segments{0};
        const ArcSpan* arc{nullptr};
    };

    void xdata(Writer& w, std::size_t contour, std::size_t piece, std::size_t count, bool closed,
               const ArcSpan* circle, const std::vector<SplitArc>& splits) {
        w.pair(1001, "CARVETTO");
        w.pair(1071, 1);  // tag version
        w.pair(1071, static_cast<long long>(contour));
        w.pair(1071, static_cast<long long>(piece));
        w.pair(1071, static_cast<long long>(count));
        w.pair(1071, closed ? 1 : 0);
        if (circle != nullptr) {
            w.pair(1071, circle->direction == geometry::ArcDirection::CW ? 1 : 0);
            w.real(1040, circle->startAngle);
            return;
        }
        if (splits.empty()) {
            return;
        }
        // Split arcs: count, then per arc (first segment, segment count, direction) as 1071 and
        // (centre x, centre y, radius, start angle, end angle) as 1040.
        w.pair(1071, static_cast<long long>(splits.size()));
        for (const SplitArc& a : splits) {
            w.pair(1071, static_cast<long long>(a.firstSegment));
            w.pair(1071, static_cast<long long>(a.segments));
            w.pair(1071, a.arc->direction == geometry::ArcDirection::CW ? 1 : 0);
        }
        for (const SplitArc& a : splits) {
            w.real(1040, a.arc->centre.x);
            w.real(1040, a.arc->centre.y);
            w.real(1040, a.arc->radius);
            w.real(1040, a.arc->startAngle);
            w.real(1040, a.arc->endAngle);
        }
    }

    void writePoly(Writer& w, const Contour& c, const Piece& piece, std::size_t layer, bool closedWhole,
                   std::vector<SplitArc>& splits) {
        // Vertices are the span starts plus (when open) the end of the last span; arcs set the
        // bulge tan(sweep / 4) on their start vertex. An arc sweeping more than pi is split into
        // equal segments of at most pi (|bulge| <= 1): the centre of a near-full arc rebuilt
        // from one short chord and a huge bulge would lose precision as 1 / (2*pi - sweep).
        std::vector<std::pair<Vec2, double>> vertices;
        for (std::size_t i = piece.first; i < piece.first + piece.count; ++i) {
            const Span& s = c.spans[i];
            if (const auto* arc = std::get_if<ArcSpan>(&s)) {
                const double sweep = arc->sweepAngle();
                const auto n =
                    static_cast<std::size_t>(std::max(1.0, std::ceil(std::fabs(sweep) / kPi - 1e-12)));
                if (n > 1) {
                    splits.push_back({vertices.size(), n, arc});
                }
                const double bulge = std::tan(sweep / static_cast<double>(n) * 0.25);
                for (std::size_t k = 0; k < n; ++k) {
                    vertices.emplace_back(k == 0
                                              ? arc->startPoint()
                                              : arc->pointAt(static_cast<double>(k) / static_cast<double>(n)),
                                          bulge);
                }
            } else {
                vertices.emplace_back(std::get<LineSpan>(s).start, 0.0);
            }
        }
        if (!closedWhole) {
            vertices.emplace_back(geometry::endPoint(c.spans[piece.first + piece.count - 1]), 0.0);
        }
        entityStart(w, "LWPOLYLINE", layer, "AcDbPolyline");
        w.pair(90, static_cast<long long>(vertices.size()));
        w.pair(70, closedWhole ? 1 : 0);
        for (const auto& [p, bulge] : vertices) {
            w.point(10, p);
            if (bulge != 0.0) {
                w.real(42, bulge);
            }
        }
    }

    void writeSpline(Writer& w, const Contour& c, const Piece& piece, std::size_t layer) {
        // Joined cubic Beziers as one clamped degree-3 B-spline: interior knots of multiplicity 3
        // reproduce each Bezier exactly.
        std::vector<Vec2> controls;
        for (std::size_t i = piece.first; i < piece.first + piece.count; ++i) {
            const auto& b = std::get<BezierSpan>(c.spans[i]);
            if (controls.empty()) {
                controls.push_back(b.p0);
            }
            controls.push_back(b.p1);
            controls.push_back(b.p2);
            controls.push_back(b.p3);
        }
        std::vector<double> knots{0.0, 0.0, 0.0, 0.0};
        for (std::size_t k = 1; k < piece.count; ++k) {
            const auto v = static_cast<double>(k);
            knots.insert(knots.end(), {v, v, v});
        }
        const auto last = static_cast<double>(piece.count);
        knots.insert(knots.end(), {last, last, last, last});
        entityStart(w, "SPLINE", layer, "AcDbSpline");
        w.real(210, 0.0);
        w.real(220, 0.0);
        w.real(230, 1.0);
        w.pair(70, 8);  // planar
        w.pair(71, 3);
        w.pair(72, static_cast<long long>(knots.size()));
        w.pair(73, static_cast<long long>(controls.size()));
        w.pair(74, 0);
        for (const double k : knots) {
            w.real(40, k);
        }
        for (const Vec2& p : controls) {
            w.point(10, p);
            w.real(30, 0.0);
        }
    }

    void writeEntities(Writer& w) {
        w.pair(0, "SECTION");
        w.pair(2, "ENTITIES");
        for (std::size_t ci = 0; ci < contours_.size(); ++ci) {
            const DxfContour& dc = contours_[ci];
            // Zero-length lines and Beziers cannot be represented (they would be coincident
            // vertices or a point spline) and are dropped; a contour of only such spans is
            // not written at all.
            Contour c;
            c.closed = dc.contour.closed;
            for (const Span& s : dc.contour.spans) {
                if (!isZeroLength(s)) {
                    c.spans.push_back(s);
                }
            }
            if (c.spans.empty()) {
                continue;
            }
            const std::vector<Piece> pieces = splitPieces(c);
            for (std::size_t pi = 0; pi < pieces.size(); ++pi) {
                const Piece& piece = pieces[pi];
                const ArcSpan* circle = nullptr;
                std::vector<SplitArc> splits;
                switch (piece.kind) {
                    case Piece::Kind::Circle: {
                        circle = &std::get<ArcSpan>(c.spans[0]);
                        entityStart(w, "CIRCLE", dc.layer, "AcDbCircle");
                        w.point(10, circle->centre);
                        w.real(30, 0.0);
                        w.real(40, circle->radius);
                        break;
                    }
                    case Piece::Kind::Poly:
                        writePoly(w, c, piece, dc.layer, c.closed && pieces.size() == 1, splits);
                        break;
                    case Piece::Kind::Spline:
                        writeSpline(w, c, piece, dc.layer);
                        break;
                }
                xdata(w, ci, pi, pieces.size(), c.closed, circle, splits);
            }
        }
        w.pair(0, "ENDSEC");
    }

    void writeObjects(Writer& w) {
        w.pair(0, "SECTION");
        w.pair(2, "OBJECTS");
        const std::string root = w.handle();
        const std::string group = w.handle();
        w.pair(0, "DICTIONARY");
        w.pair(5, root);
        w.pair(330, "0");
        w.pair(100, "AcDbDictionary");
        w.pair(281, 1);
        w.pair(3, "ACAD_GROUP");
        w.pair(350, group);
        w.pair(0, "DICTIONARY");
        w.pair(5, group);
        w.pair(330, root);
        w.pair(100, "AcDbDictionary");
        w.pair(281, 1);
        w.pair(0, "ENDSEC");
    }

    std::span<const DxfLayer> layers_;
    std::span<const DxfContour> contours_;
    std::vector<std::string> names_;
    std::string modelSpaceRecord_;
    std::string paperSpaceRecord_;
};

}  // namespace

DxfResult<std::vector<std::uint8_t>> exportDxf(std::span<const DxfLayer> layers,
                                               std::span<const DxfContour> contours) {
    for (std::size_t i = 0; i < contours.size(); ++i) {
        if (contours[i].layer >= layers.size()) {
            return DxfError{DxfError::Code::InvalidArgument,
                            "contour " + std::to_string(i) + " refers to missing layer " +
                                std::to_string(contours[i].layer),
                            0};
        }
        for (const Span& s : contours[i].contour.spans) {
            if (!spanFinite(s)) {
                return DxfError{DxfError::Code::InvalidArgument,
                                "contour " + std::to_string(i) + " has non-finite or degenerate geometry", 0};
            }
        }
    }
    Exporter exporter(layers, contours);
    const std::string text = exporter.run();
    return std::vector<std::uint8_t>(text.begin(), text.end());
}

}  // namespace carvetto::io
