#pragma once
#include "fibers3d/Fiber3DSlices.hpp"
#include "fibers3d/Fiber3DSurfaceMesh.hpp"
#include "vc/core/util/Compositing.hpp"
#include <QOpenGLWidget>
#include <QOpenGLFunctions_3_3_Core>
#include <array>
#include <memory>
#include <vector>

namespace vc3d::scene3d
{
struct VolumeData;
namespace slices = fibers3d::slices;
namespace surface = fibers3d::surface;
namespace ct = fibers3d::ct;

// Display-only OpenGL viewport in native CT coordinates. Works without fibers.
// Controllers own loading and sampling; camera motion reuses resident data.
class Scene3DCanvas : public QOpenGLWidget, protected QOpenGLFunctions_3_3_Core
{
    Q_OBJECT
public:
    explicit Scene3DCanvas(QWidget* parent = nullptr);
    ~Scene3DCanvas() override;
    void clearContent();
    void setCTSlices(slices::Images);
    const slices::Images& ctSlices() const;
    void setCTVolume(std::shared_ptr<const VolumeData>);
    std::shared_ptr<const VolumeData> ctVolume() const;
    bool isCTVolumeReady() const;
    int maximumVolumeTextureEdge() const;
    void setCTVolumeDensity(float);
    void setCTWindow(float low, float high);
    void setCTSliceOpacity(float opacity);
    void setCTSliceTransfer(const std::array<CompositeParams, 3>&);
    virtual void setSurfaceMesh(std::shared_ptr<const surface::Mesh>);
    std::shared_ptr<const surface::Mesh> surfaceMesh() const;
    bool isSurfaceMeshReady() const;
    void setSurfaceTextured(bool enabled);
    bool isSurfaceTextured() const;
    // Mesh mode: shade the mesh with its per-vertex colour (per-mesh colour or baked CT gray).
    void setSurfaceUseVertexColor(bool enabled);
    // Mesh CT: sample the ctVolume slab per-fragment on the mesh faces (reuses the volume 3D texture).
    void setSurfaceVolumeSample(bool enabled);
    // Shared CT texture OME-Zarr level (segment atlas + mesh atlas); -1 = auto.
    void setSurfaceTextureLevel(int level);
    int surfaceTextureLevel() const;
    // The current volume's umbilicus polyline (volume voxels, x,y,z), drawn as a line when visible.
    void setUmbilicus(const std::vector<std::array<double, 3>>& points);
    void setUmbilicusVisible(bool visible);
    // 3D-view camera animations, available in every display mode (CT / surfaces / meshes).
    //  - Spin: turntable yaw; manual drag/pan/zoom still work and the spin resumes once released.
    //  - UmbilicusWalk: glides the camera along a heavily-smoothed umbilicus at the current zoom; at each
    //    end it pulls out to frame the whole volume, then returns and walks back the other way. Any manual
    //    pan/drag/zoom cancels it (it is not resumed). Requires a selected umbilicus.
    enum class Animation { None, Spin, UmbilicusWalk };
    void setAnimation(Animation animation);
    Animation animation() const;
    // Shared speed for both animations, normalised [0,1] (mapped to a sensible range per animation).
    void setAnimationSpeed(double normalized);
    // Spin axis, fixed in screen space: Z = view-up turntable (the default), X = horizontal tumble, Y = roll.
    enum class SpinAxis { X, Y, Z };
    void setSpinAxis(SpinAxis axis);
    // Umbilicus walk: fly-through (look along the tangent and ride the spline) vs. fixed-orientation pan.
    void setWalkFacing(bool facing);
    // Umbilicus walk: how hard to smooth the spline, normalised [0,1] (0 = raw, ~0.8 = the heavy default feel).
    void setWalkSmoothing(double normalized);
    bool hasUmbilicusPath() const;
    void setSurfaceOpacity(float opacity);
    void setSurfaceVisible(bool visible);
    bool isSurfaceVisible() const;
    void setPerspective(bool perspective);
    float pixelsPerVoxel() const;
    virtual void fitAll();
    virtual void fitSurface();
    void focusPoint(Point point);

signals:
    void volumeTextureLimitChanged(int maxEdge);
    void surfaceDisplayChanged();
    // Emitted when the shared "CT texture" checkbox toggles, so the mesh source can re-bake.
    void surfaceTexturedChanged(bool enabled);
    // Emitted when the shared CT texture level combo changes, so the mesh source can re-bake.
    void surfaceTextureLevelChanged(int level);
    // Emitted when the active animation changes, including when a manual gesture auto-cancels the walk.
    void animationChanged(Animation animation);
    // Emitted when a selected umbilicus appears/disappears, so the walk control can enable/disable itself.
    void umbilicusPathAvailabilityChanged(bool available);
    void viewScaleChanged();
    void positionClicked(QPointF position, bool context, QPoint globalPosition);
    void emptyContextRequested(QPoint globalPosition);
    void clearSelectionRequested();

protected:
    struct State;
    std::unique_ptr<State> v;
    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int, int) override;
    void mousePressEvent(QMouseEvent*) override;
    void mouseMoveEvent(QMouseEvent*) override;
    void mouseReleaseEvent(QMouseEvent*) override;
    void wheelEvent(QWheelEvent*) override;
    void contextMenuEvent(QContextMenuEvent*) override;
    void keyPressEvent(QKeyEvent*) override;
    void beginFrame();
    void drawSurface();
    void drawCTSlices();
    void drawUmbilicus();
    void stepAnimation();
    void startWalk();
    GLuint uploadCTTexture(const ct::Tile& tile);
    void fitBounds(const Bounds&);

private:
    void cleanupGL();
};
}
