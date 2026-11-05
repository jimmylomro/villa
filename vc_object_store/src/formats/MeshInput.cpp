#include "formats/GeometryInput.hpp"
#include "vc_object_store/geometry/ArrayBytes.hpp"
#include "vc_object_store/Uuid.hpp"
#include <fstream>
#include <sstream>
#include <span>
#include <cstdint>

// Ingest a raw triangle mesh from a binary-little-endian PLY (verts x,y,z float32 + triangle/quad faces)
// into the object store as a `mesh` kind: two geometry arrays, `vertices` (V,3 <f4) and `faces` (F,3 <f4,
// integer-valued indices, exact for <16.7M verts). No grid, no parametrisation — a display/analysis mesh.
namespace vc::objects::formats {
namespace fs = std::filesystem;
namespace {

struct PlyMesh {
    std::vector<float> verts;    // V*3 (x,y,z)
    std::vector<int32_t> faces;  // F*3 (triangulated)
    int64_t V = 0, F = 0;
};

PlyMesh readBinaryPly(const fs::path& path)
{
    std::ifstream in(path, std::ios::binary);
    if (!in) throw std::runtime_error("mesh: cannot open " + path.string());
    std::string line;
    std::getline(in, line);
    if (line.rfind("ply", 0) != 0) throw std::runtime_error("mesh: not a PLY file: " + path.string());

    bool binary = false, little = true;
    int64_t nVert = 0, nFace = 0;
    int vertexProps = 0;                       // float props per vertex (we read the first 3 as x,y,z)
    enum { None, Vertex, Face } element = None;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream ls(line);
        std::string tok; ls >> tok;
        if (tok == "format") {
            std::string f; ls >> f;
            binary = f.find("binary") != std::string::npos;
            little = f.find("big") == std::string::npos;
        } else if (tok == "element") {
            std::string e; int64_t n; ls >> e >> n;
            if (e == "vertex") { nVert = n; element = Vertex; }
            else if (e == "face") { nFace = n; element = Face; }
            else element = None;
        } else if (tok == "property") {
            if (element == Vertex) ++vertexProps;   // our writer emits exactly x,y,z
        } else if (tok == "end_header") break;
    }
    if (!binary || !little) throw std::runtime_error("mesh: only binary_little_endian PLY is supported: " + path.string());
    if (vertexProps < 3) throw std::runtime_error("mesh: vertex needs >=3 float properties: " + path.string());

    PlyMesh m; m.V = nVert;
    m.verts.resize(size_t(nVert) * 3);
    const int extra = vertexProps - 3;           // skip any trailing per-vertex float props
    for (int64_t i = 0; i < nVert; ++i) {
        in.read(reinterpret_cast<char*>(m.verts.data() + i * 3), 3 * sizeof(float));
        if (extra) in.seekg(extra * int64_t(sizeof(float)), std::ios::cur);
    }
    if (!in) throw std::runtime_error("mesh: truncated vertex data in " + path.string());

    m.faces.reserve(size_t(nFace) * 3);
    for (int64_t i = 0; i < nFace; ++i) {
        uint8_t count = 0;
        in.read(reinterpret_cast<char*>(&count), 1);
        std::vector<int32_t> idx(count);
        if (count) in.read(reinterpret_cast<char*>(idx.data()), int64_t(count) * sizeof(int32_t));
        if (!in) throw std::runtime_error("mesh: truncated face data in " + path.string());
        if (count == 3) { m.faces.insert(m.faces.end(), {idx[0], idx[1], idx[2]}); }
        else if (count == 4) { m.faces.insert(m.faces.end(), {idx[0], idx[1], idx[2], idx[0], idx[2], idx[3]}); }
        else throw std::runtime_error("mesh: only triangle/quad faces supported: " + path.string());
    }
    m.F = int64_t(m.faces.size() / 3);
    return m;
}

}  // namespace

Prepared mesh(const fs::path& source)
{
    const auto m = readBinaryPly(source);
    Prepared p;
    p.object.uuid = util::newUuid();            // PLY carries no meta.json identity; mint one
    p.object.kind = "mesh";
    p.object.name = source.stem().string();
    p.object.storageKind = "file";
    p.object.storageRef = fs::absolute(source).lexically_normal().string();
    p.object.formatVersion = 1;
    p.geometry.kind = "mesh";
    p.geometry.arrays.push_back(detail::scalarArray<float>(
        "vertices", "<f4", {m.V, 3}, std::span<const float>(m.verts.data(), m.verts.size())));
    std::vector<float> facesF(m.faces.begin(), m.faces.end());   // int indices, exact in f32 for <16.7M verts
    p.geometry.arrays.push_back(detail::scalarArray<float>(
        "faces", "<f4", {m.F, 3}, std::span<const float>(facesF.data(), facesF.size())));
    p.source = nlohmann::json::object();
    return p;
}

}  // namespace vc::objects::formats
