#pragma once
#include <QWidget>
#include <array>
#include <string>
#include <vector>

class CState;
class ViewerManager;
class QComboBox;
class QToolButton;
class QAction;
namespace vc3d::fibers3d {
class Fiber3DSliceControls;
class Fiber3DSurfaceControls;
}
namespace vc3d::scene3d {
class Scene3DCanvas;
class Scene3DMeshSource;

// General CT/surface inspection, independent of fiber controllers or selection.
class Scene3DPanel : public QWidget
{
    Q_OBJECT
public:
    Scene3DPanel(CState*, ViewerManager*, QWidget* parent = nullptr);

    // Which stored meshes (View > Meshes) the "Meshes + slices" mode should show.
    void setVisibleMeshes(const std::vector<std::string>& uuids);
    // The globally-selected umbilicus (View > Umbilicus), in volume voxels; drives the 3D-view line (the
    // toolbar button toggles it on/off and is enabled only while a selection exists). Empty clears it.
    void setUmbilicusPoints(const std::vector<std::array<double, 3>>& points);

signals:
    void attachVolumeRequested();
    void importSurfaceRequested();
    void detachRequested();

protected:
    void showEvent(QShowEvent*) override;
    void hideEvent(QHideEvent*) override;

private:
    Scene3DCanvas* canvas_;
    fibers3d::Fiber3DSliceControls* slices_;
    fibers3d::Fiber3DSurfaceControls* surface_;
    Scene3DMeshSource* meshSource_;
    QComboBox* displayMode_;
    QToolButton* umbilicusButton_ = nullptr;
    QAction* spinAction_ = nullptr;
    QAction* walkAction_ = nullptr;   // umbilicus walk; enabled only while an umbilicus is selected
    std::vector<std::array<double, 3>> umbilicusPoints_;   // the globally-selected umbilicus (or empty)
    bool savedCT_ = true;
};
}
