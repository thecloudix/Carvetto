#include "carvetto/io/export3d.hpp"

#include <algorithm>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <vector>

#include "carvetto/io/raster.hpp"

namespace carvetto::io {

namespace {

constexpr std::uint32_t kNone = std::numeric_limits<std::uint32_t>::max();

void putU32le(std::string& s, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFFu));
}
void putU32be(std::string& s, std::uint32_t v) {
    for (int i = 3; i >= 0; --i) s.push_back(static_cast<char>((v >> (8 * i)) & 0xFFu));
}
void putF32le(std::string& s, float f) {
    std::uint32_t u;
    std::memcpy(&u, &f, 4);
    putU32le(s, u);
}

bool indicesOk(const TriangleMesh& m) {
    for (const auto& t : m.triangles)
        for (auto i : t)
            if (i >= m.vertices.size()) return false;
    return true;
}

}  // namespace

geometry::ShapeResult<TriangleMesh> buildSurfaceMesh(const relief::HeightField& field,
                                                     const SurfaceMeshOptions& opt) {
    const std::size_t w = field.width(), h = field.height();
    if (w < 2 || h < 2) return std::string("surface mesh needs a field of at least 2x2 cells");
    const bool closed = opt.back == BackFace::Flat;

    auto opaque = [&](std::size_t i, std::size_t j) { return !std::isnan(field.at(i, j)); };
    // A square (i, j) has corners (i,j), (i+1,j), (i+1,j+1), (i,j+1).
    auto quad = [&](std::size_t i, std::size_t j) {
        return opaque(i, j) && opaque(i + 1, j) && opaque(i + 1, j + 1) && opaque(i, j + 1);
    };
    auto hasQuad = [&](long long i, long long j) {
        if (i < 0 || j < 0 || i >= static_cast<long long>(w) - 1 || j >= static_cast<long long>(h) - 1)
            return false;
        return quad(static_cast<std::size_t>(i), static_cast<std::size_t>(j));
    };

    std::size_t quads = 0, wallEdges = 0;
    double lowest = std::numeric_limits<double>::infinity();
    for (std::size_t j = 0; j + 1 < h; ++j)
        for (std::size_t i = 0; i + 1 < w; ++i) {
            if (!quad(i, j)) continue;
            ++quads;
            const long long si = static_cast<long long>(i), sj = static_cast<long long>(j);
            if (!hasQuad(si, sj - 1)) ++wallEdges;
            if (!hasQuad(si + 1, sj)) ++wallEdges;
            if (!hasQuad(si, sj + 1)) ++wallEdges;
            if (!hasQuad(si - 1, sj)) ++wallEdges;
        }
    if (quads == 0) return std::string("surface mesh needs at least one square of four opaque cells");

    if (closed) {
        for (std::size_t j = 0; j < h; ++j)
            for (std::size_t i = 0; i < w; ++i)
                if (opaque(i, j)) lowest = std::min(lowest, static_cast<double>(field.at(i, j)));
        if (!std::isfinite(opt.backZ) || !(opt.backZ < lowest))
            return std::string("back plane must be below the lowest surface point");
    }

    const std::size_t triangles = quads * 2 + (closed ? quads * 2 + wallEdges * 2 : 0);
    if (triangles > opt.maxTriangles) return std::string("too many triangles for the export limit");

    // Vertices only for cell centres used by a square.
    std::vector<std::uint32_t> top(2 * w * h, kNone), bot(closed ? 2 * w * h : 0, kNone);
    TriangleMesh m;
    m.triangles.reserve(triangles);
    // Where two live squares touch only at a corner (a pinch), each gets its own vertex so the
    // surface stays a manifold. `slot` is 0 or 1 for the square at offset (dx, dy) in {-1,0}^2
    // from vertex (vi, vj).
    auto slot = [&](std::size_t vi, std::size_t vj, int dx, int dy) {
        const long long ci = static_cast<long long>(vi), cj = static_cast<long long>(vj);
        const bool ne = hasQuad(ci, cj), nw = hasQuad(ci - 1, cj), sw = hasQuad(ci - 1, cj - 1),
                   se = hasQuad(ci, cj - 1);
        if (ne && sw && !nw && !se) return (dx == -1 && dy == -1) ? 1 : 0;
        if (nw && se && !ne && !sw) return (dx == 0 && dy == -1) ? 1 : 0;
        return 0;
    };
    auto vert = [&](std::vector<std::uint32_t>& ids, std::size_t qi, std::size_t qj, int cx, int cy, bool isTop) {
        const std::size_t vi = qi + static_cast<std::size_t>(cx), vj = qj + static_cast<std::size_t>(cy);
        std::uint32_t& id = ids[(vj * w + vi) * 2 + static_cast<std::size_t>(slot(vi, vj, -cx, -cy))];
        if (id == kNone) {
            id = static_cast<std::uint32_t>(m.vertices.size());
            m.vertices.push_back({field.origin().x + static_cast<double>(vi) * field.cellSize(),
                                  field.origin().y + static_cast<double>(vj) * field.cellSize(),
                                  isTop ? static_cast<double>(field.at(vi, vj)) : opt.backZ});
        }
        return id;
    };

    for (std::size_t j = 0; j + 1 < h; ++j)
        for (std::size_t i = 0; i + 1 < w; ++i) {
            if (!quad(i, j)) continue;
            const std::uint32_t a = vert(top, i, j, 0, 0, true), b = vert(top, i, j, 1, 0, true),
                                c = vert(top, i, j, 1, 1, true), d = vert(top, i, j, 0, 1, true);
            m.triangles.push_back({a, b, c});
            m.triangles.push_back({a, c, d});
            if (!closed) continue;
            const std::uint32_t a2 = vert(bot, i, j, 0, 0, false), b2 = vert(bot, i, j, 1, 0, false),
                                c2 = vert(bot, i, j, 1, 1, false), d2 = vert(bot, i, j, 0, 1, false);
            m.triangles.push_back({a2, c2, b2});
            m.triangles.push_back({a2, d2, c2});
            const long long si = static_cast<long long>(i), sj = static_cast<long long>(j);
            // Wall below directed edge p->q (counter-clockwise around the square, interior on the
            // left, so the wall faces outwards).
            auto wall = [&](std::uint32_t p, std::uint32_t q, std::uint32_t p2, std::uint32_t q2) {
                m.triangles.push_back({p, p2, q2});
                m.triangles.push_back({p, q2, q});
            };
            if (!hasQuad(si, sj - 1)) wall(a, b, a2, b2);
            if (!hasQuad(si + 1, sj)) wall(b, c, b2, c2);
            if (!hasQuad(si, sj + 1)) wall(c, d, c2, d2);
            if (!hasQuad(si - 1, sj)) wall(d, a, d2, a2);
        }
    return m;
}

geometry::ShapeResult<ExportBytes> writeStlBinary(const TriangleMesh& mesh) {
    if (mesh.triangles.size() > std::numeric_limits<std::uint32_t>::max())
        return std::string("too many triangles for binary STL");
    if (!indicesOk(mesh)) return std::string("triangle index out of range");
    std::string out;
    out.reserve(84 + 50 * mesh.triangles.size());
    std::string header = "Carvetto binary STL";
    header.resize(80, ' ');
    out += header;
    putU32le(out, static_cast<std::uint32_t>(mesh.triangles.size()));
    for (const auto& t : mesh.triangles) {
        const Vec3 &p = mesh.vertices[t[0]], &q = mesh.vertices[t[1]], &r = mesh.vertices[t[2]];
        const double ux = q.x - p.x, uy = q.y - p.y, uz = q.z - p.z;
        const double vx = r.x - p.x, vy = r.y - p.y, vz = r.z - p.z;
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 0) {
            nx /= len;
            ny /= len;
            nz /= len;
        }
        putF32le(out, static_cast<float>(nx));
        putF32le(out, static_cast<float>(ny));
        putF32le(out, static_cast<float>(nz));
        for (auto idx : t) {
            putF32le(out, static_cast<float>(mesh.vertices[idx].x));
            putF32le(out, static_cast<float>(mesh.vertices[idx].y));
            putF32le(out, static_cast<float>(mesh.vertices[idx].z));
        }
        out.push_back('\0');
        out.push_back('\0');
    }
    return ExportBytes{std::move(out)};
}

geometry::ShapeResult<ExportBytes> writeObj(const TriangleMesh& mesh) {
    if (!indicesOk(mesh)) return std::string("triangle index out of range");
    std::string out = "# Carvetto OBJ\n";
    auto num = [&](double v) {
        char buf[40];
        auto r = std::to_chars(buf, buf + sizeof buf, v);
        out.append(buf, r.ptr);
    };
    for (const auto& v : mesh.vertices) {
        out += "v ";
        num(v.x);
        out += ' ';
        num(v.y);
        out += ' ';
        num(v.z);
        out += '\n';
    }
    for (const auto& t : mesh.triangles) {
        out += "f ";
        out += std::to_string(static_cast<std::uint64_t>(t[0]) + 1);
        out += ' ';
        out += std::to_string(static_cast<std::uint64_t>(t[1]) + 1);
        out += ' ';
        out += std::to_string(static_cast<std::uint64_t>(t[2]) + 1);
        out += '\n';
    }
    return ExportBytes{std::move(out)};
}

namespace {

std::uint32_t crc32(const std::string& s, std::size_t from) {
    std::uint32_t c = 0xFFFFFFFFu;
    for (std::size_t k = from; k < s.size(); ++k) {
        c ^= static_cast<unsigned char>(s[k]);
        for (int b = 0; b < 8; ++b) c = (c & 1u) ? (c >> 1) ^ 0xEDB88320u : (c >> 1);
    }
    return ~c;
}

void pngChunk(std::string& out, const char* type, const std::string& data) {
    putU32be(out, static_cast<std::uint32_t>(data.size()));
    const std::size_t start = out.size();
    out.append(type, 4);
    out += data;
    putU32be(out, crc32(out, start));
}

}  // namespace

geometry::ShapeResult<ExportBytes> writeHeightmap(const relief::HeightField& field,
                                                  const HeightmapOptions& opt) {
    const std::size_t w = field.width(), h = field.height();
    if (w == 0 || h == 0) return std::string("empty height field");
    if (w > static_cast<std::size_t>(kMaxImageEdge) || h > static_cast<std::size_t>(kMaxImageEdge) ||
        w * h > kMaxImagePixels)
        return std::string("height field too large for an image");
    float flo = 0, fhi = 0;
    if (!field.minMax(flo, fhi)) return std::string("height field has no opaque cells");
    const double lo = opt.rangeMin ? *opt.rangeMin : static_cast<double>(flo);
    const double hi = opt.rangeMax ? *opt.rangeMax : static_cast<double>(fhi);
    if (!std::isfinite(lo) || !std::isfinite(hi) || hi < lo) return std::string("invalid heightmap range");

    std::vector<std::uint16_t> px(w * h, 0);  // row 0 = top = largest y
    if (hi > lo) {
        for (std::size_t j = 0; j < h; ++j)
            for (std::size_t i = 0; i < w; ++i) {
                const float v = field.at(i, j);
                if (std::isnan(v)) continue;
                double t = (static_cast<double>(v) - lo) / (hi - lo);
                t = std::clamp(t, 0.0, 1.0);
                px[(h - 1 - j) * w + i] = static_cast<std::uint16_t>(std::lround(t * 65535.0));
            }
    }

    std::string out;
    if (opt.format == HeightmapFormat::Pgm16) {
        out = "P5\n" + std::to_string(w) + " " + std::to_string(h) + "\n65535\n";
        for (auto v : px) {
            out.push_back(static_cast<char>(v >> 8));
            out.push_back(static_cast<char>(v & 0xFF));
        }
        return ExportBytes{std::move(out)};
    }

    // PNG: grayscale, 16 bit; scanlines with filter 0 in a zlib stream of stored blocks.
    std::string raw;
    raw.reserve((w * 2 + 1) * h);
    for (std::size_t r = 0; r < h; ++r) {
        raw.push_back('\0');
        for (std::size_t i = 0; i < w; ++i) {
            raw.push_back(static_cast<char>(px[r * w + i] >> 8));
            raw.push_back(static_cast<char>(px[r * w + i] & 0xFF));
        }
    }
    std::string z = "\x78\x01";
    std::size_t pos = 0;
    do {
        const std::size_t n = std::min<std::size_t>(65535, raw.size() - pos);
        const bool last = pos + n == raw.size();
        z.push_back(last ? '\1' : '\0');
        z.push_back(static_cast<char>(n & 0xFF));
        z.push_back(static_cast<char>(n >> 8));
        z.push_back(static_cast<char>(~n & 0xFF));
        z.push_back(static_cast<char>((~n >> 8) & 0xFF));
        z.append(raw, pos, n);
        pos += n;
    } while (pos < raw.size());
    std::uint32_t a = 1, b = 0;
    for (char ch : raw) {
        const auto c = static_cast<unsigned char>(ch);
        a = (a + c) % 65521u;
        b = (b + a) % 65521u;
    }
    putU32be(z, (b << 16) | a);

    out.assign("\x89PNG\r\n\x1a\n", 8);
    std::string ihdr;
    putU32be(ihdr, static_cast<std::uint32_t>(w));
    putU32be(ihdr, static_cast<std::uint32_t>(h));
    ihdr.push_back(16);  // bit depth
    ihdr.push_back(0);   // grayscale
    ihdr.push_back(0);
    ihdr.push_back(0);
    ihdr.push_back(0);
    pngChunk(out, "IHDR", ihdr);
    pngChunk(out, "IDAT", z);
    pngChunk(out, "IEND", "");
    return ExportBytes{std::move(out)};
}

}  // namespace carvetto::io
