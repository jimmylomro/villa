#pragma once

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

class CState;

namespace vc3d
{
// Raw triangle-mesh geometry as stored in the object store's "mesh" kind.
struct StoreMeshGeometry {
    std::vector<std::array<float, 3>> verts;
    std::vector<std::array<int, 3>> faces;
};

// Read the given mesh uuids from the object store currently routed for the "mesh" kind.
// Returns only the uuids that were found; a missing store/session or a read failure yields
// an empty (or partial) map. Shared by the 2D cross-section overlay and the 3D mesh view so
// both read geometry the same way.
std::unordered_map<std::string, StoreMeshGeometry>
loadStoreMeshes(CState* state, const std::vector<std::string>& uuids);
}  // namespace vc3d
