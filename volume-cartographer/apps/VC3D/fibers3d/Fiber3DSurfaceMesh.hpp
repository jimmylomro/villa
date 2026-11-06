#pragma once
#include "scene3d/Scene3DGeometry.hpp"
#include <vector>
#include <cstdint>

namespace vc3d::fibers3d { using scene3d::Point; using scene3d::Bounds; }
#include <opencv2/core.hpp>
#include <array>
#include <atomic>
#include <memory>

class OpenedVolumes;
class Volume;

namespace vc3d::fibers3d::surface
{
struct Vertex {
    float position[3], normal[3];
    float uv[2]{};
    // Per-vertex colour, used by the 3D mesh view (solid per-mesh colour or baked CT gray).
    // Defaults to the renderer's shaded base colour so the segment path is unchanged.
    float color[3] = {0.65f, 0.77f, 0.73f};
};
struct Texture {
    int width = 0, height = 0;
    int level = 0; // actual OME-Zarr level sampled
    std::vector<uint8_t> pixels; // interleaved CT gray/validity
};
struct Mesh {
    Point origin{};
    Bounds bounds{};
    std::vector<Vertex> vertices;
    std::vector<uint32_t> indices;
    std::shared_ptr<const Texture> texture;
};
// A display snapshot, in native CT coordinates. No resampling, hole filling,
// reconstruction, or modification of the source QuadSurface.
std::shared_ptr<Mesh> build(const cv::Mat_<cv::Vec3f>& points, size_t maxBytes, const std::atomic_bool& cancelled);

// One raw triangle mesh (from the object store) plus a solid colour for its vertices.
struct SourceMesh {
    std::vector<std::array<float, 3>> verts;
    std::vector<std::array<int, 3>> faces;
    float color[3] = {0.65f, 0.77f, 0.73f};
};
// Merge several raw triangle meshes into one display Mesh (single origin, per-vertex colour,
// accumulated normals). Positions are in native CT coordinates; colour is carried per vertex.
std::shared_ptr<Mesh> buildTriangleMeshes(const std::vector<SourceMesh>& sources, size_t maxBytes, const std::atomic_bool& cancelled);

// Merge the raw meshes, then bake CT onto the faces as a per-triangle texture atlas: each triangle owns a
// K x K cell of a 2D RG8 texture, CT is sampled only at the on-surface texels (so only the chunks the sheet
// crosses are read), and the mesh carries per-vertex uv into its cell. Rendered by the existing `textured`
// path -- no 3D volume, no raymarch. K and the sampled OME-Zarr level grow with `maxBytes` (finer detail);
// the merged mesh is vertex-cluster-decimated first so the atlas packs efficiently and the vertex-tripling
// stays bounded. Samples are read in cancellable chunks. Returns the textured display Mesh.
std::shared_ptr<Mesh> buildTriangleAtlas(const std::vector<SourceMesh>& sources, OpenedVolumes& volumes,
                                         const Volume& volume, size_t maxBytes, int maxTextureEdge,
                                         int requestedLevel, const std::atomic_bool& cancelled);
std::shared_ptr<Texture> sampleTexture(const cv::Mat_<cv::Vec3f>& points, cv::Vec2f scale,
                                       OpenedVolumes&, const Volume&, const std::atomic_bool& cancelled, int level, int maxEdge = 2048);
}  // namespace vc3d::fibers3d::surface
