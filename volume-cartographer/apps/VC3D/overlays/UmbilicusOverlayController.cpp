#include "overlays/UmbilicusOverlayController.hpp"

#include "volume_viewers/VolumeViewerBase.hpp"
#include "vc/core/util/PlaneSurface.hpp"

#include <opencv2/core.hpp>
#include <QColor>
#include <algorithm>
#include <cmath>

UmbilicusOverlayController::UmbilicusOverlayController(CState* /*state*/, ViewerManager* manager, QObject* parent)
    : ViewerOverlayControllerBase("umbilicus", parent)
{
    bindToViewerManager(manager);
}

void UmbilicusOverlayController::setUmbilicus(const std::vector<std::array<double, 3>>& points)
{
    _points = points;
    refreshAll();
}

bool UmbilicusOverlayController::isOverlayEnabledFor(VolumeViewerBase* viewer) const
{
    return viewer && dynamic_cast<PlaneSurface*>(viewer->currentSurface());
}

void UmbilicusOverlayController::collectPrimitives(VolumeViewerBase* viewer, OverlayBuilder& builder)
{
    if (_points.size() < 2)
        return;
    auto* plane = viewer ? dynamic_cast<PlaneSurface*>(viewer->currentSurface()) : nullptr;
    if (!plane)
        return;
    const cv::Vec3f o = plane->origin();
    const cv::Vec3f n = plane->normal({0, 0, 0});

    OverlayStyle style;
    style.penColor = QColor(255, 220, 40);   // yellow, distinct from fibre/mesh overlays
    style.penWidth = std::max(viewer->intersectionThickness() + 1.5f, 2.5f);
    style.z = 55.0;

    // Draw the axis as a connected line where it runs within a band of this slice: a curve on XZ/YZ views,
    // a short in-plane stub on XY. Each maximal run of near-plane points becomes one line strip.
    const float band = 80.0f;   // volume voxels
    std::vector<QPointF> run;
    auto flush = [&]() {
        if (run.size() >= 2)
            builder.addLineStrip(run, false, style);
        run.clear();
    };
    for (const auto& pt : _points) {
        const cv::Vec3f p{float(pt[0]), float(pt[1]), float(pt[2])};
        if (std::abs((p - o).dot(n)) <= band)
            run.push_back(viewer->volumeToScene(p));
        else
            flush();
    }
    flush();
}
