#include "Fiber3DSurfaceMesh.hpp"
#include "Fiber3DCT.hpp"
#include "OpenedVolumes.hpp"
#include "vc/core/types/Volume.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace vc3d::fibers3d::surface
{
namespace
{
// Unit normal of triangle (p,q,r); false for a degenerate/non-finite face.
bool faceNormal(const float* p, const float* q, const float* r, cv::Vec3d& n)
{
    const cv::Vec3d u(double(q[0]) - p[0], double(q[1]) - p[1], double(q[2]) - p[2]);
    const cv::Vec3d v(double(r[0]) - p[0], double(r[1]) - p[1], double(r[2]) - p[2]);
    n = u.cross(v);
    const auto length = cv::norm(n);
    if (!(length > 0) || !std::isfinite(length))
        return false;
    n /= length;
    return true;
}
}  // namespace
std::shared_ptr<Mesh> build(const cv::Mat_<cv::Vec3f>& points, size_t maxBytes, const std::atomic_bool& cancelled)
{
    auto mesh = std::make_shared<Mesh>();
    if (cancelled.load())
        return {};
    if (points.rows < 2 || points.cols < 2)
        return mesh;
    // Bound allocations before copying points or allocating the index map.
    constexpr size_t bytesPerPoint = sizeof(Vertex) + 6 * sizeof(uint32_t);
    if (points.total() > maxBytes / bytesPerPoint || points.total() >= UINT32_MAX)
        throw std::runtime_error("Surface exceeds the 3D memory budget. Increase Memory budget in Settings.");
    if (size_t(points.rows - 1) * (points.cols - 1) > size_t(INT_MAX) / 6)
        throw std::runtime_error("Surface exceeds the 3D renderer's index capacity.");
    const auto valid = [](cv::Vec3f p) { return p[0] != -1.f && std::isfinite(p[0]) && std::isfinite(p[1]) && std::isfinite(p[2]); };
    std::vector<uint32_t> ids(points.total(), UINT32_MAX);
    mesh->vertices.reserve(points.total());
    mesh->indices.reserve(size_t(points.rows - 1) * (points.cols - 1) * 6);
    const double inf = std::numeric_limits<double>::infinity();
    mesh->bounds = {{inf, inf, inf}, {-inf, -inf, -inf}};
    bool hasOrigin = false;
    auto vertex = [&](int y, int x) {
        auto& id = ids[size_t(y) * points.cols + x];
        if (id != UINT32_MAX)
            return id;
        const auto p = points(y, x);
        if (!hasOrigin) {
            mesh->origin = {p[0], p[1], p[2]};
            hasOrigin = true;
        }
        Vertex v{};
        v.uv[0] = float(x) / (points.cols - 1);
        v.uv[1] = float(y) / (points.rows - 1);
        for (int k = 0; k < 3; ++k) {
            v.position[k] = float(double(p[k]) - mesh->origin[k]);
            mesh->bounds.lower[k] = std::min(mesh->bounds.lower[k], double(p[k]));
            mesh->bounds.upper[k] = std::max(mesh->bounds.upper[k], double(p[k]));
        }
        id = uint32_t(mesh->vertices.size());
        mesh->vertices.push_back(v);
        return id;
    };
    auto triangle = [&](uint32_t a, uint32_t b, uint32_t c) {
        cv::Vec3d n;
        if (!faceNormal(mesh->vertices[a].position, mesh->vertices[b].position, mesh->vertices[c].position, n))
            return;
        mesh->indices.insert(mesh->indices.end(), {a, b, c});
        for (auto id : {a, b, c})
            for (int k = 0; k < 3; ++k)
                mesh->vertices[id].normal[k] += float(n[k]);
    };
    for (int y = 0; y + 1 < points.rows; ++y) {
        if (cancelled.load())
            return {};
        for (int x = 0; x + 1 < points.cols; ++x) {
            // Match QuadSurface's four-corner validity rule. In particular,
            // never bridge an invalid cell, including a single missing point.
            if (!valid(points(y, x)) || !valid(points(y, x + 1)) || !valid(points(y + 1, x)) || !valid(points(y + 1, x + 1)))
                continue;
            const auto a = vertex(y, x), b = vertex(y, x + 1), c = vertex(y + 1, x), d = vertex(y + 1, x + 1);
            triangle(a, b, d);
            triangle(a, d, c);
        }
    }
    return mesh;
}

std::shared_ptr<Mesh> buildTriangleMeshes(const std::vector<SourceMesh>& sources, size_t maxBytes, const std::atomic_bool& cancelled)
{
    auto mesh = std::make_shared<Mesh>();
    if (cancelled.load())
        return {};
    size_t totalV = 0, totalF = 0;
    for (const auto& s : sources) { totalV += s.verts.size(); totalF += s.faces.size(); }
    if (totalV == 0)
        return mesh;
    constexpr size_t bytesPerVertex = sizeof(Vertex) + 3 * sizeof(uint32_t);
    if (totalV > maxBytes / bytesPerVertex || totalV >= UINT32_MAX)
        throw std::runtime_error("Meshes exceed the 3D memory budget. Increase Memory budget in Settings.");
    mesh->vertices.reserve(totalV);
    mesh->indices.reserve(totalF * 3);
    const double inf = std::numeric_limits<double>::infinity();
    mesh->bounds = {{inf, inf, inf}, {-inf, -inf, -inf}};
    bool hasOrigin = false;
    for (const auto& s : sources) {
        if (cancelled.load())
            return {};
        const uint32_t base = uint32_t(mesh->vertices.size());
        const long nv = long(s.verts.size());
        for (const auto& p : s.verts) {
            if (!hasOrigin) { mesh->origin = {p[0], p[1], p[2]}; hasOrigin = true; }
            Vertex v{};
            for (int k = 0; k < 3; ++k) {
                v.position[k] = float(double(p[k]) - mesh->origin[k]);
                v.color[k] = s.color[k];
                mesh->bounds.lower[k] = std::min(mesh->bounds.lower[k], double(p[k]));
                mesh->bounds.upper[k] = std::max(mesh->bounds.upper[k], double(p[k]));
            }
            mesh->vertices.push_back(v);
        }
        for (const auto& f : s.faces) {
            if (f[0] < 0 || f[1] < 0 || f[2] < 0 || f[0] >= nv || f[1] >= nv || f[2] >= nv)
                continue;
            const uint32_t a = base + uint32_t(f[0]), b = base + uint32_t(f[1]), c = base + uint32_t(f[2]);
            cv::Vec3d n;
            if (!faceNormal(mesh->vertices[a].position, mesh->vertices[b].position, mesh->vertices[c].position, n))
                continue;
            mesh->indices.insert(mesh->indices.end(), {a, b, c});
            for (auto id : {a, b, c})
                for (int k = 0; k < 3; ++k)
                    mesh->vertices[id].normal[k] += float(n[k]);
        }
    }
    return mesh;
}

namespace
{
using Vec3 = std::array<double, 3>;
using Tri = std::array<int, 3>;

// Flatten the raw source meshes into one absolute-coordinate vertex list and valid, non-degenerate triangles.
void flattenSources(const std::vector<SourceMesh>& sources, std::vector<Vec3>& V, std::vector<Tri>& F)
{
    size_t base = 0;
    for (const auto& s : sources) {
        for (const auto& p : s.verts)
            V.push_back({double(p[0]), double(p[1]), double(p[2])});
        const long nv = long(s.verts.size());
        for (const auto& f : s.faces)
            if (f[0] >= 0 && f[1] >= 0 && f[2] >= 0 && f[0] < nv && f[1] < nv && f[2] < nv)
                F.push_back({int(base) + f[0], int(base) + f[1], int(base) + f[2]});
        base += s.verts.size();
    }
}

double meanEdgeLength(const std::vector<Vec3>& V, const std::vector<Tri>& F)
{
    double sum = 0;
    int n = 0;
    for (size_t i = 0; i < F.size() && n < 30000; ++i, ++n) {
        const auto& a = V[F[i][0]];
        const auto& b = V[F[i][1]];
        sum += std::sqrt((a[0]-b[0])*(a[0]-b[0]) + (a[1]-b[1])*(a[1]-b[1]) + (a[2]-b[2])*(a[2]-b[2]));
    }
    return n ? std::max(1.0, sum / n) : 1.0;
}

// One pass of vertex-cluster decimation: snap vertices onto a grid of side `cell`, merge each cell to its
// centroid, drop faces that collapse. Coordinates are >= 0 (CT voxels); cell indices fit 21 bits each.
void clusterOnce(const std::vector<Vec3>& V, const std::vector<Tri>& F, double cell,
                 std::vector<Vec3>& Vo, std::vector<Tri>& Fo)
{
    std::unordered_map<int64_t, int> id;
    id.reserve(V.size());
    std::vector<Vec3> sum;
    std::vector<int> cnt;
    std::vector<int> remap(V.size());
    const double inv = 1.0 / cell;
    for (size_t i = 0; i < V.size(); ++i) {
        const int64_t ix = int64_t(std::floor(V[i][0] * inv)) & 0x1FFFFF;
        const int64_t iy = int64_t(std::floor(V[i][1] * inv)) & 0x1FFFFF;
        const int64_t iz = int64_t(std::floor(V[i][2] * inv)) & 0x1FFFFF;
        const int64_t key = ix | (iy << 21) | (iz << 42);
        auto it = id.find(key);
        int j;
        if (it == id.end()) {
            j = int(sum.size());
            id.emplace(key, j);
            sum.push_back(V[i]);
            cnt.push_back(1);
        } else {
            j = it->second;
            for (int k = 0; k < 3; ++k) sum[j][k] += V[i][k];
            ++cnt[j];
        }
        remap[i] = j;
    }
    Vo.resize(sum.size());
    for (size_t j = 0; j < sum.size(); ++j)
        for (int k = 0; k < 3; ++k) Vo[j][k] = sum[j][k] / cnt[j];
    Fo.clear();
    Fo.reserve(F.size());
    for (const auto& f : F) {
        const int a = remap[f[0]], b = remap[f[1]], c = remap[f[2]];
        if (a != b && b != c && a != c) Fo.push_back({a, b, c});
    }
}

// Decimate (V,F) in place to about `target` faces by growing the cluster cell until it fits.
void decimateTo(std::vector<Vec3>& V, std::vector<Tri>& F, size_t target)
{
    if (F.size() <= target)
        return;
    double cell = meanEdgeLength(V, F) * std::sqrt(double(F.size()) / double(target));
    for (int iter = 0; iter < 6; ++iter) {
        std::vector<Vec3> Vo;
        std::vector<Tri> Fo;
        clusterOnce(V, F, cell, Vo, Fo);
        if (Fo.size() <= size_t(target * 1.25) || Fo.empty()) {
            V.swap(Vo);
            F.swap(Fo);
            return;
        }
        cell *= 1.5;
    }
    std::vector<Vec3> Vo;
    std::vector<Tri> Fo;
    clusterOnce(V, F, cell, Vo, Fo);
    V.swap(Vo);
    F.swap(Fo);
}

// Barycentric weights of 2-D point p in triangle (a,b,c); clamped to the simplex (negatives -> 0, renormalised)
// so gutter texels just outside the triangle sample its nearest edge. Returns false for a degenerate cell.
bool baryClamped(double px, double py, const double* a, const double* b, const double* c, double w[3])
{
    const double v0x = b[0]-a[0], v0y = b[1]-a[1], v1x = c[0]-a[0], v1y = c[1]-a[1], v2x = px-a[0], v2y = py-a[1];
    const double d00 = v0x*v0x + v0y*v0y, d01 = v0x*v1x + v0y*v1y, d11 = v1x*v1x + v1y*v1y;
    const double d20 = v2x*v0x + v2y*v0y, d21 = v2x*v1x + v2y*v1y;
    const double denom = d00*d11 - d01*d01;
    if (std::abs(denom) < 1e-12) return false;
    double w1 = (d11*d20 - d01*d21) / denom;
    double w2 = (d00*d21 - d01*d20) / denom;
    double w0 = 1.0 - w1 - w2;
    w0 = std::max(0.0, w0); w1 = std::max(0.0, w1); w2 = std::max(0.0, w2);
    const double s = w0 + w1 + w2;
    if (s <= 0) return false;
    w[0] = w0/s; w[1] = w1/s; w[2] = w2/s;
    return true;
}
}  // namespace

std::shared_ptr<Mesh> buildTriangleAtlas(const std::vector<SourceMesh>& sources, OpenedVolumes& volumes,
                                         const Volume& volume, size_t maxBytes, int /*maxTextureEdge*/,
                                         int requestedLevel, const std::atomic_bool& cancelled)
{
    auto mesh = std::make_shared<Mesh>();
    if (cancelled.load())
        return {};
    std::vector<Vec3> V;
    std::vector<Tri> F;
    flattenSources(sources, V, F);
    if (F.empty())
        return mesh;
    // Keep the vertex-tripling and the atlas bounded: ~one face per ~1.2 KB of budget.
    const size_t target = std::clamp<size_t>(maxBytes / 1200, 80000, 900000);
    decimateTo(V, F, target);
    if (cancelled.load())
        return {};
    const size_t nF = F.size();

    // Atlas: one K x K cell per triangle, packed cols x rows. 2-D textures are capped at a safe 8192 edge
    // (GL_MAX_TEXTURE_SIZE, not the 3-D maximumVolumeTextureEdge); K and the dims also fit the byte budget.
    const int atlasCap = std::max(64, std::min(8192, int(std::floor(std::sqrt(double(maxBytes) / 2.0)))));
    const int cols = std::max(1, int(std::ceil(std::sqrt(double(nF)))));
    int K = std::clamp(atlasCap / cols, 2, 64);
    int rows = int((nF + cols - 1) / cols);
    while (K > 2 && size_t(cols * K) * size_t(rows * K) * 2 > maxBytes)
        --K;
    const int W = cols * K, H = rows * K;

    // CT level: match the texel footprint (triangle edge / usable cell width); bigger budget -> bigger K ->
    // smaller texel -> finer level.
    const int gutter = 1;
    const double texelSize = std::max(1.0, meanEdgeLength(V, F) / std::max(1, K - 2 * gutter));
    // Honour an explicit CT level from the texture-level combo (if present in the volume), otherwise match
    // the texel footprint automatically.
    const int level = (requestedLevel >= 0 && volumes.pyramid(volume).has(requestedLevel))
                          ? requestedLevel : ct::levelForSpacing(volumes, volume, texelSize);

    mesh->origin = {V[F[0][0]][0], V[F[0][0]][1], V[F[0][0]][2]};
    mesh->vertices.resize(3 * nF);
    mesh->indices.resize(3 * nF);
    const double inf = std::numeric_limits<double>::infinity();
    mesh->bounds = {{inf, inf, inf}, {-inf, -inf, -inf}};
    auto tex = std::make_shared<Texture>();
    tex->width = W;
    tex->height = H;
    tex->level = level;
    tex->pixels.assign(size_t(W) * H * 2, 0);   // 0 validity = no CT; only on-surface texels are written

    std::vector<cv::Vec3f> coords;
    std::vector<size_t> pix;
    coords.reserve(1 << 20);
    pix.reserve(1 << 20);
    auto flush = [&]() {
        if (coords.empty()) return;
        const cv::Mat_<cv::Vec3f> mat(1, int(coords.size()), coords.data());
        const cv::Mat_<cv::Vec2b> samp = ct::sampleCoordinates(mat, volumes, volume, level);
        for (size_t i = 0; i < pix.size(); ++i) {
            const auto g = samp(0, int(i));
            tex->pixels[pix[i] * 2] = g[0];
            tex->pixels[pix[i] * 2 + 1] = g[1];
        }
        coords.clear();
        pix.clear();
    };

    const double fW = W - 1, fH = H - 1;
    for (size_t fi = 0; fi < nF; ++fi) {
        if ((fi & 0x3FFF) == 0 && cancelled.load())
            return {};
        const auto& P0 = V[F[fi][0]];
        const auto& P1 = V[F[fi][1]];
        const auto& P2 = V[F[fi][2]];
        double n[3] = {(P1[1]-P0[1])*(P2[2]-P0[2]) - (P1[2]-P0[2])*(P2[1]-P0[1]),
                       (P1[2]-P0[2])*(P2[0]-P0[0]) - (P1[0]-P0[0])*(P2[2]-P0[2]),
                       (P1[0]-P0[0])*(P2[1]-P0[1]) - (P1[1]-P0[1])*(P2[0]-P0[0])};
        const double nl = std::sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
        if (nl > 0) { n[0]/=nl; n[1]/=nl; n[2]/=nl; }
        const int col = int(fi % cols), row = int(fi / cols);
        const int cx0 = col * K, cy0 = row * K;
        // Triangle corners in texel space (lower-left right triangle, inset by the gutter).
        const double T[3][2] = {{double(cx0+gutter), double(cy0+gutter)},
                                {double(cx0+K-1-gutter), double(cy0+gutter)},
                                {double(cx0+gutter), double(cy0+K-1-gutter)}};
        const Vec3* P[3] = {&P0, &P1, &P2};
        for (int k = 0; k < 3; ++k) {
            Vertex& v = mesh->vertices[3*fi + k];
            for (int c = 0; c < 3; ++c) {
                v.position[c] = float((*P[k])[c] - mesh->origin[c]);
                v.normal[c] = float(n[c]);
                mesh->bounds.lower[c] = std::min(mesh->bounds.lower[c], (*P[k])[c]);
                mesh->bounds.upper[c] = std::max(mesh->bounds.upper[c], (*P[k])[c]);
            }
            v.uv[0] = float(T[k][0] / fW);
            v.uv[1] = float(T[k][1] / fH);
        }
        mesh->indices[3*fi] = uint32_t(3*fi);
        mesh->indices[3*fi + 1] = uint32_t(3*fi + 1);
        mesh->indices[3*fi + 2] = uint32_t(3*fi + 2);
        // Bake the lower-left half (+1 texel gutter) of the cell: sample CT only on the surface.
        for (int ty = cy0; ty < cy0 + K; ++ty) {
            for (int tx = cx0; tx < cx0 + K; ++tx) {
                double w[3];
                if (!baryClamped(double(tx), double(ty), T[0], T[1], T[2], w))
                    continue;
                // Skip the far (unused) corner of the cell to halve the bake; keep a gutter around the triangle.
                if (double(tx - cx0) + double(ty - cy0) > double(K - 1) + 1.5)
                    continue;
                coords.emplace_back(float(w[0]*P0[0] + w[1]*P1[0] + w[2]*P2[0]),
                                    float(w[0]*P0[1] + w[1]*P1[1] + w[2]*P2[1]),
                                    float(w[0]*P0[2] + w[1]*P1[2] + w[2]*P2[2]));
                pix.push_back(size_t(ty) * W + tx);
                if (coords.size() >= (1u << 20))
                    flush();
            }
        }
    }
    flush();
    mesh->texture = tex;
    return mesh;
}
}  // namespace vc3d::fibers3d::surface
