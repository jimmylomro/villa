#include "overlays/StoreMeshOverlayController.hpp"

#include "CState.hpp"
#include "overlays/MeshColour.hpp"
#include "overlays/StoreMeshData.hpp"
#include "volume_viewers/VolumeViewerBase.hpp"
#include "vc/core/util/PlaneSurface.hpp"

#include <opencv2/core.hpp>
#include <QColor>
#include <algorithm>

StoreMeshOverlayController::StoreMeshOverlayController(CState* state, ViewerManager* manager, QObject* parent)
    : ViewerOverlayControllerBase("store-meshes", parent), _state(state)
{
    // A different store selected for "mesh" invalidates everything we cached.
    if (_state)
        connect(_state, &CState::storeRoutesChanged, this,
                [this]() { ++_meshVersion; _cache.clear(); ensureLoaded(); refreshAll(); });
    // Register with every plane viewer so collectPrimitives() is actually called;
    // without this the overlay is constructed but never attached and never draws.
    bindToViewerManager(manager);
}

void StoreMeshOverlayController::setMeshes(const std::vector<std::string>& uuids)
{
    ++_meshVersion;   // the visible set changed; any cached cut is stale
    _uuids = uuids;
    for (auto it = _cache.begin(); it != _cache.end();)
        it = (std::find(_uuids.begin(), _uuids.end(), it->first) == _uuids.end()) ? _cache.erase(it) : std::next(it);
    ensureLoaded();
    refreshAll();
}

void StoreMeshOverlayController::ensureLoaded()
{
    std::vector<std::string> missing;
    for (const auto& u : _uuids)
        if (!_cache.count(u)) missing.push_back(u);
    if (missing.empty()) return;
    // Shared store-read (also used by the 3D mesh view), then adopt into the overlay's cache.
    for (auto& [uuid, g] : vc3d::loadStoreMeshes(_state, missing)) {
        Mesh m;
        m.verts = std::move(g.verts);
        m.faces = std::move(g.faces);
        _cache.emplace(uuid, std::move(m));
    }
    ++_meshVersion;   // geometry just loaded (or was attempted); invalidate cached cuts
}

bool StoreMeshOverlayController::isOverlayEnabledFor(VolumeViewerBase* viewer) const
{
    return viewer && dynamic_cast<PlaneSurface*>(viewer->currentSurface());
}

void StoreMeshOverlayController::detachViewer(VolumeViewerBase* viewer)
{
    _cuts.erase(viewer);
    ViewerOverlayControllerBase::detachViewer(viewer);
}

void StoreMeshOverlayController::collectPrimitives(VolumeViewerBase* viewer, OverlayBuilder& builder)
{
    ensureLoaded();
    auto* plane = viewer ? dynamic_cast<PlaneSurface*>(viewer->currentSurface()) : nullptr;
    if (!plane || _cache.empty()) return;
    const cv::Vec3f o = plane->origin();
    const cv::Vec3f n = plane->normal({0, 0, 0});

    // Recompute the mesh∩plane cut only when the slice or the visible set changed; a pan/zoom tick
    // keeps the same plane, so we fall straight through to re-projecting the cached segments.
    CutCache& cc = _cuts[viewer];
    const std::array<float, 3> oa{{o[0], o[1], o[2]}}, na{{n[0], n[1], n[2]}};
    if (!cc.valid || cc.meshVersion != _meshVersion || cc.origin != oa || cc.normal != na) {
        cc.endpoints.clear();
        cc.segRgb.clear();
        for (const auto& [uuid, m] : _cache) {
            const unsigned rgb = vc3d::meshColour(uuid).rgb();
            const int nv = int(m.verts.size());
            for (const auto& f : m.faces) {
                if (f[0] < 0 || f[1] < 0 || f[2] < 0 || f[0] >= nv || f[1] >= nv || f[2] >= nv) continue;
                const std::array<cv::Vec3f, 3> p{
                    cv::Vec3f(m.verts[f[0]][0], m.verts[f[0]][1], m.verts[f[0]][2]),
                    cv::Vec3f(m.verts[f[1]][0], m.verts[f[1]][1], m.verts[f[1]][2]),
                    cv::Vec3f(m.verts[f[2]][0], m.verts[f[2]][1], m.verts[f[2]][2])};
                const std::array<float, 3> d{
                    float((p[0] - o).dot(n)), float((p[1] - o).dot(n)), float((p[2] - o).dot(n))};
                cv::Vec3f cross[2];
                int nc = 0;
                for (int e = 0; e < 3 && nc < 2; ++e) {
                    const int a = e, b = (e + 1) % 3;
                    if ((d[a] <= 0.0f) == (d[b] <= 0.0f)) continue;   // edge does not cross the plane
                    const float t = d[a] / (d[a] - d[b]);
                    cross[nc++] = p[a] + t * (p[b] - p[a]);
                }
                if (nc == 2) {
                    cc.endpoints.push_back({{cross[0][0], cross[0][1], cross[0][2]}});
                    cc.endpoints.push_back({{cross[1][0], cross[1][1], cross[1][2]}});
                    cc.segRgb.push_back(rgb);
                }
            }
        }
        cc.origin = oa;
        cc.normal = na;
        cc.meshVersion = _meshVersion;
        cc.valid = true;
    }

    OverlayStyle style;
    // A touch thicker than the hairline surface lines, but still following the
    // viewer's intersection-thickness slider when it is turned up.
    style.penWidth = std::max(viewer->intersectionThickness() + 1.5f, 2.5f);
    style.z = 50.0;

    for (size_t i = 0; i + 1 < cc.endpoints.size(); i += 2) {
        const auto& a = cc.endpoints[i];
        const auto& b = cc.endpoints[i + 1];
        // Colour each cut by its source mesh so distinct meshes are told apart.
        style.penColor = (i / 2 < cc.segRgb.size()) ? QColor::fromRgb(cc.segRgb[i / 2])
                                                    : QColor(80, 200, 255);
        const std::vector<QPointF> seg{viewer->volumeToScene(cv::Vec3f(a[0], a[1], a[2])),
                                       viewer->volumeToScene(cv::Vec3f(b[0], b[1], b[2]))};
        builder.addLineStrip(seg, false, style);
    }
}
