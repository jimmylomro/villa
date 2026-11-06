#pragma once

#include <array>
#include <string>
#include <vector>

class CState;

namespace vc3d
{
// The current volume's umbilicus polyline, its points in the current volume's voxels (x, y, z).
// Empty when no umbilicus store is routed, no volume is selected, or the volume has no umbilicus.
std::vector<std::array<double, 3>> readVolumeUmbilicus(CState* state);

// A specific umbilicus object's polyline (its "points" array, in volume voxels) from the store routed for
// "umbilicus", by uuid. Empty uuid or a read failure yields empty. Used by the View > Umbilicus selection.
std::vector<std::array<double, 3>> readStoreUmbilicus(CState* state, const std::string& uuid);
}  // namespace vc3d
