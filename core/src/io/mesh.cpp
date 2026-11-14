#include "carvetto/io/mesh.hpp"

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstring>
#include <fstream>
#include <functional>
#include <limits>
#include <unordered_map>

namespace carvetto::io {

using geometry::ShapeResult;

MeshBounds meshBounds(const TriangleMesh& mesh) {
    MeshBounds b;
    for (const Vec3& v : mesh.vertices) {
        if (b.empty) {
            b.empty = false;
            b.min = b.max = v;
            continue;
        }
        b.min = {std::min(b.min.x, v.x), std::min(b.min.y, v.y), std::min(b.min.z, v.z)};
        b.max = {std::max(b.max.x, v.x), std::max(b.max.y, v.y), std::max(b.max.z, v.z)};
    }
    return b;
}

namespace {

bool okCoord(double v, const MeshLimits& l) { return std::isfinite(v) && std::fabs(v) <= l.maxCoordinate; }

// Welds vertices by exact float bit patterns (-0 == +0) in first-appearance order.
struct Welder {
    struct Key {
        std::uint32_t a, b, c;
        bool operator==(const Key& o) const { return a == o.a && b == o.b && c == o.c; }
    };
    struct KeyHash {
        std::size_t operator()(const Key& k) const {
            std::uint64_t h = k.a;
            h = h * 0x9E3779B97F4A7C15ull + k.b;
            h = h * 0x9E3779B97F4A7C15ull + k.c;
            return static_cast<std::size_t>(h ^ (h >> 29));
        }
    };
    std::unordered_map<Key, std::uint32_t, KeyHash> map;
    TriangleMesh* mesh;
    std::size_t maxVertices;

    static std::uint32_t bits(float f) {
        if (f == 0.0f) f = 0.0f;  // -0 -> +0
        std::uint32_t u;
        std::memcpy(&u, &f, sizeof u);
        return u;
    }
    // Returns false when the vertex cap is hit.
    bool add(float x, float y, float z, std::uint32_t& idx) {
        const Key k{bits(x), bits(y), bits(z)};
        auto it = map.find(k);
        if (it != map.end()) {
            idx = it->second;
            return true;
        }
        if (mesh->vertices.size() >= maxVertices) return false;
        idx = static_cast<std::uint32_t>(mesh->vertices.size());
        mesh->vertices.push_back({x == 0.0f ? 0.0 : static_cast<double>(x), y == 0.0f ? 0.0 : static_cast<double>(y),
                                  z == 0.0f ? 0.0 : static_cast<double>(z)});
        map.emplace(k, idx);
        return true;
    }
};

std::uint32_t le32(const unsigned char* p) {
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}
float leFloat(const unsigned char* p) {
    const std::uint32_t u = le32(p);
    float f;
    std::memcpy(&f, &u, sizeof f);
    return f;
}

ShapeResult<TriangleMesh> readBinaryStl(std::string_view bytes, std::size_t count, const MeshLimits& lim) {
    TriangleMesh mesh;
    Welder w{{}, &mesh, lim.maxVertices};
    const auto* p = reinterpret_cast<const unsigned char*>(bytes.data()) + 84;
    mesh.triangles.reserve(count);
    for (std::size_t t = 0; t < count; ++t, p += 50) {
        std::array<std::uint32_t, 3> tri{};
        for (int k = 0; k < 3; ++k) {
            const float x = leFloat(p + 12 + 12 * k), y = leFloat(p + 16 + 12 * k), z = leFloat(p + 20 + 12 * k);
            if (!okCoord(x, lim) || !okCoord(y, lim) || !okCoord(z, lim))
                return std::string("STL: non-finite or out-of-range coordinate in triangle ") + std::to_string(t);
            if (!w.add(x, y, z, tri[static_cast<std::size_t>(k)])) return std::string("STL: too many vertices");
        }
        mesh.triangles.push_back(tri);
    }
    return mesh;
}

bool isSpace(char c) { return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\f' || c == '\v'; }

struct Tokens {
    std::string_view s;
    std::size_t pos{0};
    bool next(std::string_view& tok) {
        while (pos < s.size() && isSpace(s[pos])) ++pos;
        if (pos >= s.size()) return false;
        const std::size_t b = pos;
        while (pos < s.size() && !isSpace(s[pos])) ++pos;
        tok = s.substr(b, pos - b);
        return true;
    }
    void skipLine() {
        while (pos < s.size() && s[pos] != '\n') ++pos;
    }
};

bool iequal(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::tolower(static_cast<unsigned char>(a[i])) != std::tolower(static_cast<unsigned char>(b[i]))) return false;
    return true;
}

// Parses a whole token as a finite double.
bool parseDouble(std::string_view t, double& out) {
    if (!t.empty() && t.front() == '+') t.remove_prefix(1);
    if (t.empty()) return false;
    const auto r = std::from_chars(t.data(), t.data() + t.size(), out);
    return r.ec == std::errc() && r.ptr == t.data() + t.size();
}

ShapeResult<TriangleMesh> readAsciiStl(std::string_view bytes, const MeshLimits& lim) {
    TriangleMesh mesh;
    Welder w{{}, &mesh, lim.maxVertices};
    Tokens tk{bytes, 0};
    std::string_view t;
    const std::string trunc = "STL: truncated or malformed ASCII file";
    bool sawSolid = false;
    while (tk.next(t)) {
        if (!iequal(t, "solid")) return std::string("STL: expected 'solid'");
        sawSolid = true;
        tk.skipLine();  // the solid's name
        for (;;) {
            if (!tk.next(t)) return trunc;  // no endsolid
            if (iequal(t, "endsolid")) {
                tk.skipLine();
                break;
            }
            if (!iequal(t, "facet")) return trunc;
            if (!tk.next(t) || !iequal(t, "normal")) return trunc;
            double d;
            for (int i = 0; i < 3; ++i)
                if (!tk.next(t) || !parseDouble(t, d)) return trunc;  // normal is read and ignored
            if (!tk.next(t) || !iequal(t, "outer") || !tk.next(t) || !iequal(t, "loop")) return trunc;
            std::array<std::uint32_t, 3> tri{};
            for (std::size_t k = 0; k < 3; ++k) {
                if (!tk.next(t) || !iequal(t, "vertex")) return trunc;
                double c[3];
                for (double& ci : c) {
                    if (!tk.next(t) || !parseDouble(t, ci)) return trunc;
                    if (!okCoord(ci, lim)) return std::string("STL: non-finite or out-of-range coordinate");
                }
                if (!w.add(static_cast<float>(c[0]), static_cast<float>(c[1]), static_cast<float>(c[2]), tri[k]))
                    return std::string("STL: too many vertices");
            }
            if (!tk.next(t) || !iequal(t, "endloop") || !tk.next(t) || !iequal(t, "endfacet")) return trunc;
            if (mesh.triangles.size() >= lim.maxTriangles) return std::string("STL: too many triangles");
            mesh.triangles.push_back(tri);
        }
    }
    if (!sawSolid) return std::string("STL: empty file");
    return mesh;
}

}  // namespace

ShapeResult<TriangleMesh> readStl(std::string_view bytes, const MeshLimits& lim) {
    if (bytes.size() > lim.maxBytes) return std::string("STL: file too large");
    if (bytes.size() >= 84) {
        const std::size_t count = le32(reinterpret_cast<const unsigned char*>(bytes.data()) + 80);
        if (count <= (std::numeric_limits<std::size_t>::max() - 84) / 50 && bytes.size() == 84 + 50 * count) {
            if (count > lim.maxTriangles) return std::string("STL: too many triangles");
            return readBinaryStl(bytes, count, lim);
        }
    }
    std::size_t b = 0;
    while (b < bytes.size() && isSpace(bytes[b])) ++b;
    if (bytes.size() - b >= 5 && iequal(bytes.substr(b, 5), "solid")) {
        return readAsciiStl(bytes.substr(b), lim);
    }
    if (bytes.size() < 84) return std::string("STL: file too short");
    return std::string("STL: size does not match the triangle count (truncated file)");
}

namespace {

// Parses one face-vertex reference ("7", "7/1", "7//2", "7/1/2", "-1") into a 0-based vertex index.
bool objIndex(std::string_view tok, std::size_t vertexCount, std::uint32_t& out) {
    const std::size_t slash = tok.find('/');
    std::string_view num = tok.substr(0, slash);
    if (!num.empty() && num.front() == '+') num.remove_prefix(1);
    long long v = 0;
    const auto r = std::from_chars(num.data(), num.data() + num.size(), v);
    if (num.empty() || r.ec != std::errc() || r.ptr != num.data() + num.size()) return false;
    long long idx;
    if (v > 0) idx = v - 1;
    else if (v < 0) idx = static_cast<long long>(vertexCount) + v;
    else return false;
    if (idx < 0 || idx >= static_cast<long long>(vertexCount)) return false;
    out = static_cast<std::uint32_t>(idx);
    return true;
}

}  // namespace

ShapeResult<TriangleMesh> readObj(std::string_view bytes, const MeshLimits& lim) {
    if (bytes.size() > lim.maxBytes) return std::string("OBJ: file too large");
    TriangleMesh mesh;
    std::vector<std::uint32_t> face;
    std::size_t pos = 0, lineNo = 0;
    while (pos < bytes.size()) {
        std::size_t e = bytes.find('\n', pos);
        if (e == std::string_view::npos) e = bytes.size();
        std::string_view line = bytes.substr(pos, e - pos);
        pos = e + 1;
        ++lineNo;
        const std::string where = " (line " + std::to_string(lineNo) + ")";
        Tokens tk{line, 0};
        std::string_view t;
        if (!tk.next(t)) continue;
        if (t == "v") {
            double c[3];
            for (double& ci : c) {
                if (!tk.next(t) || !parseDouble(t, ci)) return "OBJ: bad vertex" + where;
                if (!okCoord(ci, lim)) return "OBJ: non-finite or out-of-range coordinate" + where;
            }
            // An optional w component is ignored.
            if (mesh.vertices.size() >= lim.maxVertices) return std::string("OBJ: too many vertices");
            mesh.vertices.push_back({c[0], c[1], c[2]});
        } else if (t == "f") {
            face.clear();
            while (tk.next(t)) {
                std::uint32_t i;
                if (!objIndex(t, mesh.vertices.size(), i)) return "OBJ: bad face index" + where;
                if (face.size() >= lim.maxTriangles + 2) return std::string("OBJ: too many triangles");
                face.push_back(i);
            }
            if (face.size() < 3) return "OBJ: face with fewer than 3 vertices" + where;
            if (mesh.triangles.size() + face.size() - 2 > lim.maxTriangles) return std::string("OBJ: too many triangles");
            for (std::size_t k = 1; k + 1 < face.size(); ++k) mesh.triangles.push_back({face[0], face[k], face[k + 1]});
        }
        // Anything else (vt, vn, usemtl, mtllib, g, o, s, l, p, comments) is ignored.
    }
    return mesh;
}

bool meshFormatFromPath(std::string_view path, MeshFormat& out) {
    const std::size_t dot = path.rfind('.');
    if (dot == std::string_view::npos) return false;
    const std::string_view ext = path.substr(dot + 1);
    if (iequal(ext, "stl")) out = MeshFormat::Stl;
    else if (iequal(ext, "obj")) out = MeshFormat::Obj;
    else return false;
    return true;
}

ShapeResult<TriangleMesh> readMesh(std::string_view bytes, MeshFormat format, const MeshLimits& lim) {
    return format == MeshFormat::Stl ? readStl(bytes, lim) : readObj(bytes, lim);
}

ShapeResult<TriangleMesh> readMeshFile(const std::string& path, const MeshLimits& lim) {
    MeshFormat f;
    if (!meshFormatFromPath(path, f)) return std::string("unsupported mesh format (supported: STL, OBJ)");
    std::ifstream in(path, std::ios::binary | std::ios::ate);
    if (!in) return std::string("cannot open file");
    const std::streamoff size = in.tellg();
    if (size < 0) return std::string("cannot read file");
    if (static_cast<unsigned long long>(size) > lim.maxBytes) return std::string("file too large");
    std::string data(static_cast<std::size_t>(size), '\0');
    in.seekg(0);
    if (size > 0 && !in.read(data.data(), size)) return std::string("cannot read file");
    return readMesh(data, f, lim);
}

// ---- Transforms ---------------------------------------------------------------------------------

namespace {
constexpr double kMaxExtent = 1e9;
bool finiteInRange(const TriangleMesh& m) {
    for (const Vec3& v : m.vertices)
        if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) || std::fabs(v.x) > kMaxExtent ||
            std::fabs(v.y) > kMaxExtent || std::fabs(v.z) > kMaxExtent)
            return false;
    return true;
}
}  // namespace

std::string applyMeshUnits(TriangleMesh& mesh, MeshUnits units) {
    if (units == MeshUnits::Millimetres) return {};
    for (Vec3& v : mesh.vertices) {
        v.x *= 25.4;
        v.y *= 25.4;
        v.z *= 25.4;
    }
    return finiteInRange(mesh) ? std::string() : std::string("model too large after unit scaling");
}

void centerMesh(TriangleMesh& mesh) {
    const MeshBounds b = meshBounds(mesh);
    if (b.empty) return;
    const Vec3 c = b.centre();
    for (Vec3& v : mesh.vertices) {
        v.x -= c.x;
        v.y -= c.y;
        v.z -= c.z;
    }
}

std::string orientMesh(TriangleMesh& mesh, const MeshOrientation& o) {
    if (o.zQuarterTurns < 0 || o.zQuarterTurns > 3) return "zQuarterTurns must be 0..3";
    for (Vec3& v : mesh.vertices) {
        const Vec3 p = v;
        switch (o.top) {
            case TopFace::PlusZ: break;
            case TopFace::MinusZ: v = {p.x, -p.y, -p.z}; break;
            case TopFace::PlusX: v = {-p.z, p.y, p.x}; break;
            case TopFace::MinusX: v = {p.z, p.y, -p.x}; break;
            case TopFace::PlusY: v = {p.x, -p.z, p.y}; break;
            case TopFace::MinusY: v = {p.x, p.z, -p.y}; break;
        }
        for (int k = 0; k < o.zQuarterTurns; ++k) v = {-v.y, v.x, v.z};
    }
    return {};
}

std::string rotateMesh(TriangleMesh& mesh, const Vec3& axis, double angle) {
    const double len = std::sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (!std::isfinite(len) || len == 0.0 || !std::isfinite(angle)) return "invalid rotation axis or angle";
    const double ux = axis.x / len, uy = axis.y / len, uz = axis.z / len;
    const double c = std::cos(angle), s = std::sin(angle), k = 1.0 - c;
    const double m[3][3] = {{c + ux * ux * k, ux * uy * k - uz * s, ux * uz * k + uy * s},
                            {uy * ux * k + uz * s, c + uy * uy * k, uy * uz * k - ux * s},
                            {uz * ux * k - uy * s, uz * uy * k + ux * s, c + uz * uz * k}};
    for (Vec3& v : mesh.vertices) {
        const Vec3 p = v;
        v = {m[0][0] * p.x + m[0][1] * p.y + m[0][2] * p.z, m[1][0] * p.x + m[1][1] * p.y + m[1][2] * p.z,
             m[2][0] * p.x + m[2][1] * p.y + m[2][2] * p.z};
    }
    return {};
}

std::string scaleMeshToSize(TriangleMesh& mesh, const Vec3& target, bool lockRatio, int driver) {
    const MeshBounds b = meshBounds(mesh);
    if (b.empty) return "empty mesh";
    if (driver < 0 || driver > 2) return "driver axis must be 0..2";
    const double size[3] = {b.size().x, b.size().y, b.size().z};
    const double tgt[3] = {target.x, target.y, target.z};
    double f[3];
    if (lockRatio) {
        if (!(tgt[driver] > 0.0) || !std::isfinite(tgt[driver])) return "target size must be positive";
        if (!(size[driver] > 0.0)) return "cannot scale an axis with zero size";
        f[0] = f[1] = f[2] = tgt[driver] / size[driver];
    } else {
        for (int i = 0; i < 3; ++i) {
            if (!(tgt[i] > 0.0) || !std::isfinite(tgt[i])) return "target size must be positive";
            if (!(size[i] > 0.0)) return "cannot scale an axis with zero size";
            f[i] = tgt[i] / size[i];
        }
    }
    TriangleMesh saved;
    saved.vertices = mesh.vertices;
    for (Vec3& v : mesh.vertices) {
        v.x = b.min.x + (v.x - b.min.x) * f[0];
        v.y = b.min.y + (v.y - b.min.y) * f[1];
        v.z = b.min.z + (v.z - b.min.z) * f[2];
    }
    if (!finiteInRange(mesh)) {
        mesh.vertices = std::move(saved.vertices);
        return "model too large after scaling";
    }
    return {};
}

}  // namespace carvetto::io
