#include "overlays/StoreMeshData.hpp"

#include "CState.hpp"

#include <vc_object_store/objects/ObjectStoreSession.hpp>
#include <vc_object_store/geometry/ArrayBytes.hpp>

#include <atomic>

namespace vc3d
{
std::unordered_map<std::string, StoreMeshGeometry>
loadStoreMeshes(CState* state, const std::vector<std::string>& uuids)
{
    std::unordered_map<std::string, StoreMeshGeometry> out;
    const auto session = state ? state->objectStoreSession() : nullptr;
    const auto store = state ? state->storeFor("mesh") : std::nullopt;
    if (!session || !store || uuids.empty())
        return out;
    try {
        session->read(*store, [&](const vc::objects::ObjectStore& db, const vc::objects::ReadTxn& txn,
                                  const std::atomic_bool&) {
            for (const auto& uuid : uuids) {
                const auto head = db.currentGeometry(txn, uuid);
                if (!head) continue;
                const auto vArr = db.readGeometryArrays(txn, {head->uuid}, "vertices");
                const auto fArr = db.readGeometryArrays(txn, {head->uuid}, "faces");
                const auto vf = vc::objects::scalarsFromBytes<float>(vc::objects::packedItem(vArr, 0));
                const auto ff = vc::objects::scalarsFromBytes<float>(vc::objects::packedItem(fArr, 0));
                StoreMeshGeometry g;
                g.verts.resize(vf.size() / 3);
                for (size_t i = 0; i < g.verts.size(); ++i) g.verts[i] = {vf[3 * i], vf[3 * i + 1], vf[3 * i + 2]};
                g.faces.resize(ff.size() / 3);
                for (size_t i = 0; i < g.faces.size(); ++i)
                    g.faces[i] = {int(ff[3 * i] + 0.5f), int(ff[3 * i + 1] + 0.5f), int(ff[3 * i + 2] + 0.5f)};
                out.emplace(uuid, std::move(g));
            }
        });
    } catch (const std::exception&) {
        // A read failure leaves whatever was loaded so far; callers treat missing as "does not draw".
    }
    return out;
}
}  // namespace vc3d
