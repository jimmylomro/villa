#include "scene3d/Scene3DMeshSource.hpp"

#include "scene3d/Scene3DCanvas.hpp"
#include "overlays/MeshColour.hpp"
#include "overlays/StoreMeshData.hpp"
#include "CState.hpp"
#include "OpenedVolumes.hpp"
#include "vc/core/types/Volume.hpp"

#include <QColor>
#include <QtConcurrent/QtConcurrentRun>

namespace vc3d::scene3d
{
namespace
{
namespace surface = fibers3d::surface;
}  // namespace

Scene3DMeshSource::Scene3DMeshSource(CState* state, Scene3DCanvas* canvas, QObject* parent)
    : QObject(parent), state_(state), canvas_(canvas)
{
    if (state_)
        connect(state_, &CState::storeRoutesChanged, this, [this] { if (active_) rebuild(); });
    connect(&watcher_, &QFutureWatcher<Built>::finished, this, [this] {
        if (!active_)
            return;
        const Built b = watcher_.future().resultCount() ? watcher_.future().result() : Built{};
        const bool tex = b.textured && b.mesh && bool(b.mesh->texture);
        canvas_->setSurfaceVolumeSample(false);   // mesh CT is now a 2D atlas, not a 3D slab
        canvas_->setCTVolume({});
        canvas_->setSurfaceTextured(tex);
        canvas_->setSurfaceUseVertexColor(!tex);
        canvas_->setSurfaceMesh(b.mesh);
        if (b.mesh && !b.mesh->indices.empty()) {
            const auto faces = b.mesh->indices.size() / 3;
            emit statusChanged(tex ? tr("Mesh · %1 faces · CT Zarr level %2").arg(faces).arg(b.mesh->texture->level)
                                   : tr("Mesh · %1 faces").arg(faces));
        } else {
            emit statusChanged(tr("No mesh to show."));
        }
    });
}

Scene3DMeshSource::~Scene3DMeshSource()
{
    if (cancelled_)
        cancelled_->store(true);
}

void Scene3DMeshSource::setActive(bool active)
{
    if (active_ == active)
        return;
    active_ = active;
    rebuild();
}

void Scene3DMeshSource::setTextured(bool textured)
{
    if (textured_ == textured)
        return;
    textured_ = textured;
    if (active_)
        rebuild();
}

void Scene3DMeshSource::setVisibleMeshes(std::vector<std::string> uuids)
{
    uuids_ = std::move(uuids);
    if (active_)
        rebuild();
}

void Scene3DMeshSource::setMemoryBudget(size_t bytes)
{
    if (maxBytes_ == bytes)
        return;
    maxBytes_ = bytes;
    if (active_)
        rebuild();   // a bigger budget bakes a larger atlas at a finer CT level
}

void Scene3DMeshSource::setLevel(int level)
{
    if (level_ == level)
        return;
    level_ = level;
    if (active_ && textured_)
        rebuild();   // re-bake the atlas at the chosen OME-Zarr level
}

void Scene3DMeshSource::rebuild()
{
    if (cancelled_)
        cancelled_->store(true);
    if (!active_) {
        canvas_->setSurfaceVolumeSample(false);
        canvas_->setCTVolume({});
        canvas_->setSurfaceUseVertexColor(false);
        canvas_->setSurfaceMesh({});
        return;
    }
    // Store read stays on the GUI thread (as the 2D overlay does); the merge + volume sampling go off-thread.
    auto loaded = vc3d::loadStoreMeshes(state_, uuids_);
    std::vector<surface::SourceMesh> sources;
    sources.reserve(loaded.size());
    for (const auto& uuid : uuids_) {
        auto it = loaded.find(uuid);
        if (it == loaded.end())
            continue;
        surface::SourceMesh s;
        s.verts = std::move(it->second.verts);
        s.faces = std::move(it->second.faces);
        const QColor c = vc3d::meshColour(uuid);
        s.color[0] = float(c.redF());
        s.color[1] = float(c.greenF());
        s.color[2] = float(c.blueF());
        sources.push_back(std::move(s));
    }
    canvas_->setSurfaceUseVertexColor(true);
    if (sources.empty()) {
        canvas_->setSurfaceVolumeSample(false);
        canvas_->setCTVolume({});
        canvas_->setSurfaceMesh({});
        emit statusChanged(tr("No meshes shown — enable them in View ▸ Meshes."));
        return;
    }
    const bool withCT = textured_;
    emit statusChanged(withCT ? tr("Baking CT texture onto the mesh…") : tr("Preparing mesh…"));
    auto volume = withCT ? state_->currentVolume() : nullptr;
    auto opened = withCT ? state_->openedVolumes() : nullptr;
    const int maxEdge = canvas_->maximumVolumeTextureEdge();
    const int level = level_;
    cancelled_ = std::make_shared<std::atomic_bool>(false);
    watcher_.setFuture(QtConcurrent::run(
        [sources = std::move(sources), budget = maxBytes_, stop = cancelled_, volume, opened, withCT, maxEdge, level]() -> Built {
            Built b;
            try {
                if (withCT && volume && opened) {
                    // Bake CT onto the faces as a per-triangle atlas (sampled only on the surface).
                    b.mesh = surface::buildTriangleAtlas(sources, *opened, *volume, budget, maxEdge, level, *stop);
                    b.textured = bool(b.mesh && b.mesh->texture);
                } else {
                    b.mesh = surface::buildTriangleMeshes(sources, budget, *stop);
                }
            } catch (const std::exception&) {
                // Leave whatever succeeded; the caller treats a missing mesh as "nothing to show".
            }
            return b;
        }));
}
}  // namespace vc3d::scene3d
