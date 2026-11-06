#pragma once
#include "Fiber3DSurfaceMesh.hpp"
#include "Fiber3DSlices.hpp"
#include <QMatrix4x4>
#include <QOpenGLFunctions_3_3_Core>
#include <QOpenGLShaderProgram>
#include <QQuaternion>

namespace vc3d::fibers3d
{
// Construct, draw and destroy with the owning canvas context current.
class Fiber3DSurfaceRenderer : protected QOpenGLFunctions_3_3_Core
{
public:
    Fiber3DSurfaceRenderer();
    ~Fiber3DSurfaceRenderer();
    // True while uploading. Subsequent frames reuse the resident buffers.
    // ctSlab + volumeSample: per-fragment CT by sampling the volume slab at the fragment's world
    // position (mesh mode), instead of the 2D-atlas `textured` path (segments) or the solid colour.
    bool draw(std::shared_ptr<const surface::Mesh>, const QMatrix4x4&, const Point& origin, const QQuaternion& rotation, float opacity, bool textured = true, float low = 0, float high = 255, bool useVertexColor = false, const slices::Slab* ctSlab = nullptr, bool volumeSample = false);
    QString error() const { return error_; }

private:
    std::shared_ptr<const surface::Mesh> mesh_;
    QOpenGLShaderProgram program_;
    GLuint vao_ = 0, vertices_ = 0, indices_ = 0, texture_ = 0, volumeTexture_ = 0;
    const void* volumeSlabKey_ = nullptr;   // slab last uploaded to volumeTexture_ (re-upload on change)
    int textureRows_ = 0;
    size_t vertexBytes_ = 0, indexBytes_ = 0;
    QString error_;
};
}  // namespace vc3d::fibers3d
