#include "overlays/StoreMeshOverlayController.hpp"

#include "CState.hpp"
#include "volume_viewers/VolumeViewerBase.hpp"
#include "vc/core/util/PlaneSurface.hpp"

#include <vc_object_store/objects/ObjectStoreSession.hpp>
#include <vc_object_store/geometry/ArrayBytes.hpp>

#include <opencv2/core.hpp>
#include <algorithm>

StoreMeshOverlayController::StoreMeshOverlayController(CState* state, ViewerManager* manager, QObject* parent)
    : ViewerOverlayControllerBase("store-meshes", parent), _state(state)
{
    // A different store selected for "mesh" invalidates everything we cached.
    if (_state)
        connect(_state, &CState::storeRoutesChanged, this, [this]() { _cache.clear(); ensureLoaded(); refreshAll(); });
    // Register with every plane viewer so collectPrimitives() is actually called;
    // without this the overlay is constructed but never attached and never draws.
    bindToViewerManager(manager);
}

void StoreMeshOverlayController::setMeshes(const std::vector<std::string>& uuids)
{
    _uuids = uuids;
    for (auto it = _cache.begin(); it != _cache.end();)
        it = (std::find(_uuids.begin(), _uuids.end(), it->first) == _uuids.end()) ? _cache.erase(it) : std::next(it);
    ensureLoaded();
    refreshAll();
}

void StoreMeshOverlayController::ensureLoaded()
{
    const auto session = _state ? _state->objectStoreSession() : nullptr;
    const auto store = _state ? _state->storeFor("mesh") : std::nullopt;
    if (!session || !store) return;
    std::vector<std::string> missing;
    for (const auto& u : _uuids)
        if (!_cache.count(u)) missing.push_back(u);
    if (missing.empty()) return;
    try {
        session->read(*store, [&](const vc::objects::ObjectStore& db, const vc::objects::ReadTxn& txn,
                                  const std::atomic_bool&) {
            for (const auto& uuid : missing) {
                const auto head = db.currentGeometry(txn, uuid);
                if (!head) continue;
                const auto vArr = db.readGeometryArrays(txn, {head->uuid}, "vertices");
                const auto fArr = db.readGeometryArrays(txn, {head->uuid}, "faces");
                const auto vf = vc::objects::scalarsFromBytes<float>(vc::objects::packedItem(vArr, 0));
                const auto ff = vc::objects::scalarsFromBytes<float>(vc::objects::packedItem(fArr, 0));
                Mesh m;
                m.verts.resize(vf.size() / 3);
                for (size_t i = 0; i < m.verts.size(); ++i) m.verts[i] = {vf[3 * i], vf[3 * i + 1], vf[3 * i + 2]};
                m.faces.resize(ff.size() / 3);
                for (size_t i = 0; i < m.faces.size(); ++i)
                    m.faces[i] = {int(ff[3 * i] + 0.5f), int(ff[3 * i + 1] + 0.5f), int(ff[3 * i + 2] + 0.5f)};
                _cache.emplace(uuid, std::move(m));
            }
        });
    } catch (const std::exception&) {
        // A read failure leaves the mesh uncached: it simply does not draw.
    }
}

bool StoreMeshOverlayController::isOverlayEnabledFor(VolumeViewerBase* viewer) const
{
    return viewer && dynamic_cast<PlaneSurface*>(viewer->currentSurface());
}

void StoreMeshOverlayController::collectPrimitives(VolumeViewerBase* viewer, OverlayBuilder& builder)
{
    ensureLoaded();
    auto* plane = viewer ? dynamic_cast<PlaneSurface*>(viewer->currentSurface()) : nullptr;
    if (!plane || _cache.empty()) return;
    const cv::Vec3f o = plane->origin();
    const cv::Vec3f n = plane->normal({0, 0, 0});

    OverlayStyle style;
    style.penColor = QColor(80, 200, 255);
    // A touch thicker than the hairline surface lines, but still following the
    // viewer's intersection-thickness slider when it is turned up.
    style.penWidth = std::max(viewer->intersectionThickness() + 1.5f, 2.5f);
    style.z = 50.0;

    for (const auto& [uuid, m] : _cache) {
        const int nv = int(m.verts.size());
        for (const auto& f : m.faces) {
            if (f[0] < 0 || f[1] < 0 || f[2] < 0 || f[0] >= nv || f[1] >= nv || f[2] >= nv) continue;
            const std::array<cv::Vec3f, 3> p{
                cv::Vec3f(m.verts[f[0]][0], m.verts[f[0]][1], m.verts[f[0]][2]),
                cv::Vec3f(m.verts[f[1]][0], m.verts[f[1]][1], m.verts[f[1]][2]),
                cv::Vec3f(m.verts[f[2]][0], m.verts[f[2]][1], m.verts[f[2]][2])};
            const std::array<float, 3> d{float((p[0] - o).dot(n)), float((p[1] - o).dot(n)), float((p[2] - o).dot(n))};
            std::vector<QPointF> seg;
            for (int e = 0; e < 3; ++e) {
                const int a = e, b = (e + 1) % 3;
                if ((d[a] <= 0.0f) == (d[b] <= 0.0f)) continue;      // edge does not cross the plane
                const float t = d[a] / (d[a] - d[b]);
                seg.push_back(viewer->volumeToScene(p[a] + t * (p[b] - p[a])));
            }
            if (seg.size() == 2) builder.addLineStrip(seg, false, style);
        }
    }
}
