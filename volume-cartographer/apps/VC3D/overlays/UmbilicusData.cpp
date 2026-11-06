#include "overlays/UmbilicusData.hpp"

#include "CState.hpp"

#include <vc_object_store/objects/ObjectStoreSession.hpp>
#include <vc_object_store/objects/VolumeUmbilicus.hpp>
#include <vc_object_store/geometry/ArrayBytes.hpp>

#include <atomic>

namespace vc3d
{
std::vector<std::array<double, 3>> readVolumeUmbilicus(CState* state)
{
    std::vector<std::array<double, 3>> out;
    if (!state)
        return out;
    const auto attachment = state->storeFor("umbilicus");
    const auto& volume = state->annotationVolume();   // AnnotationVolume is-a VolumeVoxels
    const auto session = state->objectStoreSession();
    if (!attachment || !volume || !session)
        return out;
    try {
        const auto umb = vc::objects::volumeUmbilicus(*session, *attachment, *volume);
        if (umb) {
            out.reserve(umb->points.size());
            for (const auto& p : umb->points)
                out.push_back({p[0], p[1], p[2]});
        }
    } catch (const std::exception&) {
        // A read failure (store unavailable, etc.) leaves the umbilicus empty: nothing draws.
    }
    return out;
}

std::vector<std::array<double, 3>> readStoreUmbilicus(CState* state, const std::string& uuid)
{
    std::vector<std::array<double, 3>> out;
    if (!state || uuid.empty())
        return out;
    const auto store = state->storeFor("umbilicus");
    const auto session = state->objectStoreSession();
    if (!store || !session)
        return out;
    try {
        session->read(*store, [&](const vc::objects::ObjectStore& db, const vc::objects::ReadTxn& txn,
                                  const std::atomic_bool&) {
            const auto head = db.currentGeometry(txn, uuid);
            if (!head)
                return;
            const auto arr = db.readGeometryArrays(txn, {head->uuid}, "points");   // stored as <f8>, x,y,z
            const auto pts = vc::objects::scalarsFromBytes<double>(vc::objects::packedItem(arr, 0));
            out.resize(pts.size() / 3);
            for (size_t i = 0; i < out.size(); ++i)
                out[i] = {pts[3 * i], pts[3 * i + 1], pts[3 * i + 2]};
        });
    } catch (const std::exception&) {
    }
    return out;
}
}  // namespace vc3d
