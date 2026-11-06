#pragma once

#include "overlays/ViewerOverlayControllerBase.hpp"

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

class CState;

// Draws stored `mesh` objects (raw triangle meshes) as their intersection with each plane viewer's
// slice. Meshes are read from the object store selected for the "mesh" kind (File > Show Meshes From);
// which of them are visible is chosen in View > Meshes. Simple brute-force triangle/plane intersection
// over the loaded geometry — no spatial catalog (meshes are not indexed).
class StoreMeshOverlayController : public ViewerOverlayControllerBase
{
    Q_OBJECT
public:
    StoreMeshOverlayController(CState* state, ViewerManager* manager, QObject* parent = nullptr);

public slots:
    // UUIDs of the meshes to show (from the store selected for "mesh"); empty hides all.
    void setMeshes(const std::vector<std::string>& uuids);

protected:
    bool isOverlayEnabledFor(VolumeViewerBase* viewer) const override;
    void collectPrimitives(VolumeViewerBase* viewer, OverlayBuilder& builder) override;
    void detachViewer(VolumeViewerBase* viewer) override;

private:
    struct Mesh {
        std::vector<std::array<float, 3>> verts;
        std::vector<std::array<int, 3>> faces;
    };
    // The mesh∩plane cut is the expensive part (brute force over every triangle). It only changes when
    // the slice (plane) moves or the visible-mesh set changes, so we cache its result -- the crossing
    // segment endpoints, in volume coords -- per viewer, and on a pan/zoom tick merely re-project those
    // few segments through volumeToScene instead of re-cutting the whole mesh.
    struct CutCache {
        std::array<float, 3> origin{{0, 0, 0}}, normal{{0, 0, 0}};
        unsigned long long meshVersion = 0;
        bool valid = false;
        std::vector<std::array<float, 3>> endpoints;   // 2*i and 2*i+1 are one segment's ends
        std::vector<unsigned> segRgb;                  // packed QRgb per segment i (its mesh's colour)
    };
    CState* _state;
    std::vector<std::string> _uuids;
    std::unordered_map<std::string, Mesh> _cache;   // loaded geometry, keyed by object uuid
    std::unordered_map<VolumeViewerBase*, CutCache> _cuts;   // per-viewer cut, reused across pan/zoom
    unsigned long long _meshVersion = 0;            // bumped whenever the visible set / geometry changes
    void ensureLoaded();
};
