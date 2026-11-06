#pragma once

#include "overlays/ViewerOverlayControllerBase.hpp"

#include <array>
#include <vector>

class CState;

// Draws the selected umbilicus (scroll axis) as a connected yellow curve where it runs near each plane
// viewer's slice. The polyline is fed in by CWindow from the View > Umbilicus single-selection (empty hides
// it). Drawn like the fibre / mesh cross-section lines, restricted to the portion near the slice.
class UmbilicusOverlayController : public ViewerOverlayControllerBase
{
    Q_OBJECT
public:
    UmbilicusOverlayController(CState* state, ViewerManager* manager, QObject* parent = nullptr);

public slots:
    // The selected umbilicus polyline in volume voxels (x,y,z); empty hides it everywhere.
    void setUmbilicus(const std::vector<std::array<double, 3>>& points);

protected:
    bool isOverlayEnabledFor(VolumeViewerBase* viewer) const override;
    void collectPrimitives(VolumeViewerBase* viewer, OverlayBuilder& builder) override;

private:
    std::vector<std::array<double, 3>> _points;   // polyline in volume voxels (x,y,z)
};
