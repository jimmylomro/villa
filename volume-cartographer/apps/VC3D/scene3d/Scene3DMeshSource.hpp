#pragma once

#include "fibers3d/Fiber3DSurfaceMesh.hpp"

#include <QObject>
#include <QFutureWatcher>
#include <QString>

#include <atomic>
#include <memory>
#include <string>
#include <vector>

class CState;

namespace vc3d::scene3d
{
class Scene3DCanvas;

// Feeds the 3D view's surface renderer from stored `mesh` objects instead of the active segment.
// Reads the same store the 2D overlay reads and merges all visible meshes into one display mesh with
// a per-mesh colour. When CT texture is on, it also samples a budget-bounded volume slab (reusing
// scene3d::sampleVolume) which the surface shader reads per-fragment on the mesh faces.
// Active only in the panel's "Meshes + slices" display mode.
class Scene3DMeshSource : public QObject
{
    Q_OBJECT
public:
    Scene3DMeshSource(CState* state, Scene3DCanvas* canvas, QObject* parent = nullptr);
    ~Scene3DMeshSource() override;

    void setActive(bool active);                       // the panel entered/left mesh mode
    void setTextured(bool textured);                   // CT-texture checkbox (sample voxels onto faces)
    void setVisibleMeshes(std::vector<std::string> uuids);  // from View > Meshes
    void setMemoryBudget(size_t bytes);                // bigger budget -> finer/larger CT atlas
    void setLevel(int level);                          // CT texture OME-Zarr level combo (-1 = auto)

signals:
    // Progress for the panel's status label while a mesh mode build/bake runs (mirrors the other modes).
    void statusChanged(const QString& text);

private:
    // One off-thread build: the merged mesh. When CT is on it carries a baked per-triangle CT atlas texture
    // (sampled on the faces by the existing `textured` path); otherwise it is shaded in the per-mesh colour.
    struct Built {
        std::shared_ptr<const fibers3d::surface::Mesh> mesh;
        bool textured = false;
    };
    void rebuild();

    CState* state_;
    Scene3DCanvas* canvas_;
    bool active_ = false;
    bool textured_ = false;
    int level_ = -1;                                   // CT level override from the combo (-1 = auto)
    std::vector<std::string> uuids_;
    size_t maxBytes_ = size_t(512) * 1024 * 1024;
    std::shared_ptr<std::atomic_bool> cancelled_;
    QFutureWatcher<Built> watcher_;
};
}  // namespace vc3d::scene3d
