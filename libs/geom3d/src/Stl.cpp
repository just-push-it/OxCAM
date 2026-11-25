// SPDX-FileCopyrightText: OxCAM contributors
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Binary STL: an 80-byte header, a little-endian uint32 triangle count, then per triangle a
// normal and three corners (12 float32) and a uint16 attribute. ASCII STL: "solid", then
// "facet normal ... outer loop vertex x y z (x3) endloop endfacet" blocks.

#include "ocam/geom3d/Stl.hpp"

#include <ocam/foundation/Error.hpp>

#include <fmt/format.h>

#include <bit>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdint>
#include <cstring>

namespace ocam::geom3d {
namespace {

constexpr std::size_t kHeader = 80;
constexpr std::size_t kRecord = 50;

static_assert(std::endian::native == std::endian::little, "STL I/O assumes little endian");

float readFloat(const char* p) {
    float f = 0.0f;
    std::memcpy(&f, p, sizeof f);
    return f;
}

void appendFloat(std::string& out, double v) {
    const auto f = static_cast<float>(v);
    char b[4];
    std::memcpy(b, &f, sizeof f);
    out.append(b, 4);
}

bool binaryLayout(std::string_view bytes) {
    if (bytes.size() < kHeader + 4) {
        return false;
    }
    std::uint32_t count = 0;
    std::memcpy(&count, bytes.data() + kHeader, sizeof count);
    return bytes.size() == kHeader + 4 + static_cast<std::size_t>(count) * kRecord;
}

Mesh readBinary(std::string_view bytes) {
    std::uint32_t count = 0;
    std::memcpy(&count, bytes.data() + kHeader, sizeof count);
    Mesh mesh;
    mesh.vertices.reserve(static_cast<std::size_t>(count) * 3);
    mesh.triangles.reserve(count);
    for (std::uint32_t t = 0; t < count; ++t) {
        const char* r = bytes.data() + kHeader + 4 + static_cast<std::size_t>(t) * kRecord + 12;
        const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
        for (int c = 0; c < 3; ++c) {
            mesh.vertices.emplace_back(readFloat(r + 12 * c), readFloat(r + 12 * c + 4),
                                       readFloat(r + 12 * c + 8));
        }
        mesh.triangles.push_back({first, first + 1, first + 2});
    }
    return mesh;
}

/// The next whitespace-separated word of `text` from `pos`.
std::string_view word(std::string_view text, std::size_t& pos) {
    while (pos < text.size() && std::isspace(static_cast<unsigned char>(text[pos]))) {
        ++pos;
    }
    const std::size_t begin = pos;
    while (pos < text.size() && !std::isspace(static_cast<unsigned char>(text[pos]))) {
        ++pos;
    }
    return text.substr(begin, pos - begin);
}

Mesh readAscii(std::string_view text, std::string_view sourceName) {
    Mesh mesh;
    std::size_t pos = 0;
    int corners = 0;
    auto fail = [&](std::string_view what) {
        return Error(ErrorCode::Parse, fmt::format("{}: invalid STL ({})", sourceName, what));
    };
    while (pos < text.size()) {
        const std::string_view w = word(text, pos);
        if (w.empty()) {
            break;
        }
        if (w != "vertex") {
            continue;
        }
        Vec3 v;
        for (int k = 0; k < 3; ++k) {
            const std::string_view n = word(text, pos);
            double d = 0.0;
            const auto [end, ec] = std::from_chars(n.data(), n.data() + n.size(), d);
            if (ec != std::errc() || end != n.data() + n.size() || !std::isfinite(d)) {
                throw fail("a vertex coordinate is not a number");
            }
            v[k] = d;
        }
        mesh.vertices.push_back(v);
        if (++corners == 3) {
            const auto first = static_cast<std::uint32_t>(mesh.vertices.size() - 3);
            mesh.triangles.push_back({first, first + 1, first + 2});
            corners = 0;
        }
    }
    if (corners != 0) {
        throw fail("a facet does not have three vertices");
    }
    return mesh;
}

} // namespace

Mesh readStl(std::string_view bytes, std::string_view sourceName, double weldTolerance) {
    // Binary files may begin with "solid" too: the size decides.
    const bool binary = binaryLayout(bytes);
    if (!binary && bytes.substr(0, std::min<std::size_t>(bytes.size(), 5)) != "solid") {
        throw Error(ErrorCode::Parse, fmt::format("{}: not an STL file", sourceName));
    }
    Mesh mesh = binary ? readBinary(bytes) : readAscii(bytes, sourceName);
    for (const Vec3& v : mesh.vertices) {
        if (!v.allFinite()) {
            throw Error(ErrorCode::Parse,
                        fmt::format("{}: invalid STL (a vertex is not finite)", sourceName));
        }
    }
    mesh = weldVertices(mesh, weldTolerance);
    if (mesh.isEmpty()) {
        throw Error(ErrorCode::Parse, fmt::format("{}: the STL file has no triangles", sourceName));
    }
    return mesh;
}

std::string writeStl(const Mesh& mesh) {
    std::string out;
    out.reserve(kHeader + 4 + mesh.triangles.size() * kRecord);
    std::string header = "OxCAM binary STL";
    header.resize(kHeader, ' ');
    out += header;
    const auto count = static_cast<std::uint32_t>(mesh.triangles.size());
    char b[4];
    std::memcpy(b, &count, sizeof count);
    out.append(b, 4);
    for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
        const Vec3 n = mesh.triangleNormal(t);
        for (int k = 0; k < 3; ++k) {
            appendFloat(out, n[k]);
        }
        for (const std::uint32_t v : mesh.triangles[t]) {
            for (int k = 0; k < 3; ++k) {
                appendFloat(out, mesh.vertices[v][k]);
            }
        }
        out.append(2, '\0');
    }
    return out;
}

} // namespace ocam::geom3d
