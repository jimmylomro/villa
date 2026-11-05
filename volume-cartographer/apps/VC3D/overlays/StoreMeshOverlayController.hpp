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

private:
    struct Mesh {
        std::vector<std::array<float, 3>> verts;
        std::vector<std::array<int, 3>> faces;
    };
    CState* _state;
    std::vector<std::string> _uuids;
    std::unordered_map<std::string, Mesh> _cache;   // loaded geometry, keyed by object uuid
    void ensureLoaded();
};
