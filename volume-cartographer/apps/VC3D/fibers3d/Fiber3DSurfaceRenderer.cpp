#include "Fiber3DSurfaceRenderer.hpp"
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <QVector2D>
#include <QVector3D>
#include <QVector4D>

namespace vc3d::fibers3d
{
Fiber3DSurfaceRenderer::Fiber3DSurfaceRenderer()
{
    if (!initializeOpenGLFunctions()) {
        error_ = QStringLiteral("Surface mesh requires OpenGL 3.3.");
        return;
    }
    const char* vertex = R"glsl(#version 330 core
layout(location=0) in vec3 position;
layout(location=1) in vec3 normal;
layout(location=2) in vec2 uv;
layout(location=3) in vec3 color;
out vec2 texCoord;
uniform mat4 mvp;
uniform mat3 rotation;
uniform vec3 originOffset;
out vec3 n;
out vec3 vcolor;
out vec3 worldPos;
void main() { gl_Position = mvp * vec4(position,1); n = rotation * normal; texCoord = uv; vcolor = color; worldPos = position + originOffset; }
)glsl";
    const char* fragment = R"glsl(#version 330 core
in vec3 n;
in vec2 texCoord;
in vec3 vcolor;
in vec3 worldPos;
uniform sampler2D ct;
uniform sampler3D ctVolume;
uniform bool textured;
uniform bool useVertexColor;
uniform bool volumeSample;
uniform mat4 toTexture;
uniform vec2 windowRange, textureSize;
uniform float opacity;
uniform bool multisample;
out vec4 color;
void main() {
    // Coverage transparency writes depth per covered sample, so front/back
    // faces and fibers occlude correctly without sorting millions of triangles.
    if (!multisample && opacity < 1.0) {
        const int bayer[16] = int[16](0,8,2,10,12,4,14,6,3,11,1,9,15,7,13,5);
        ivec2 p = ivec2(gl_FragCoord.xy) & 3;
        if (opacity <= (float(bayer[p.y*4+p.x])+0.5)/16.0) discard;
    }
    float lengthN = length(n);
    float light = lengthN > 0.0 ? abs(dot(n/lengthN, normalize(vec3(-0.3,0.5,1)))) : 1.0;
    vec3 base = (useVertexColor ? vcolor : vec3(0.65,0.77,0.73)) * (0.4+0.6*light);
    if (textured) {
        vec2 sampleValue = texture(ct, (vec2(0.5) + texCoord * (textureSize - vec2(1))) / textureSize).rg;
        if (sampleValue.g >= .999) {
            float gray = clamp((sampleValue.r*255.0-windowRange.x)/max(1.0,windowRange.y-windowRange.x),0.0,1.0);
            base = vec3(gray);
        }
    }
    // Per-fragment CT (mesh mode): read the volume 3D texture at this fragment's world position. No UV.
    if (volumeSample) {
        vec3 tc = (toTexture * vec4(worldPos, 1.0)).xyz;
        if (all(greaterThanEqual(tc, vec3(0.0))) && all(lessThanEqual(tc, vec3(1.0)))) {
            vec2 sv = texture(ctVolume, tc).rg;
            if (sv.g > 0.001) {
                float value = sv.r / sv.g * 255.0;
                float gray = clamp((value - windowRange.x) / max(1e-3, windowRange.y - windowRange.x), 0.0, 1.0);
                base = vec3(gray) * (0.4 + 0.6 * light);
            }
        }
    }
    color = vec4(base, multisample ? opacity : 1.0);
}
)glsl";
    if (!program_.addShaderFromSourceCode(QOpenGLShader::Vertex, vertex) ||
        !program_.addShaderFromSourceCode(QOpenGLShader::Fragment, fragment) || !program_.link()) {
        error_ = program_.log();
        return;
    }
    glGenVertexArrays(1, &vao_);
    glGenBuffers(1, &vertices_);
    glGenBuffers(1, &indices_);
    glGenTextures(1, &texture_);
    // A 1x1x1 placeholder so the ctVolume sampler3D is always complete on its unit (the real slab is
    // uploaded on demand in mesh CT mode); keeps the sampler valid even when volumeSample is off.
    glGenTextures(1, &volumeTexture_);
    glBindTexture(GL_TEXTURE_3D, volumeTexture_);
    const uint8_t zero[2] = {0, 0};
    glTexImage3D(GL_TEXTURE_3D, 0, GL_RG8, 1, 1, 1, 0, GL_RG, GL_UNSIGNED_BYTE, zero);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_3D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    for (auto wrap : {GL_TEXTURE_WRAP_S, GL_TEXTURE_WRAP_T, GL_TEXTURE_WRAP_R})
        glTexParameteri(GL_TEXTURE_3D, wrap, GL_CLAMP_TO_EDGE);
    glBindTexture(GL_TEXTURE_3D, 0);
}
Fiber3DSurfaceRenderer::~Fiber3DSurfaceRenderer()
{
    if (texture_) glDeleteTextures(1, &texture_);
    if (volumeTexture_) glDeleteTextures(1, &volumeTexture_);
    if (vao_)
        glDeleteVertexArrays(1, &vao_);
    if (vertices_)
        glDeleteBuffers(1, &vertices_);
    if (indices_)
        glDeleteBuffers(1, &indices_);
}
bool Fiber3DSurfaceRenderer::draw(std::shared_ptr<const surface::Mesh> mesh, const QMatrix4x4& matrix, const Point& origin, const QQuaternion& rotation, float opacity, bool textured, float low, float high, bool useVertexColor, const slices::Slab* ctSlab, bool volumeSample)
{
    if (!vao_)
        return false;
    glBindVertexArray(vao_);
    glBindBuffer(GL_ARRAY_BUFFER, vertices_);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, indices_);
    if (mesh_ != mesh) {
        error_.clear();
        mesh_ = std::move(mesh);
        vertexBytes_ = indexBytes_ = 0;
        textureRows_ = 0;
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_);
        const auto texture = mesh_ ? mesh_->texture : nullptr;
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RG8, texture ? texture->width : 1, texture ? texture->height : 1, 0, GL_RG, GL_UNSIGNED_BYTE, nullptr);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glBufferData(GL_ARRAY_BUFFER, mesh_ ? mesh_->vertices.size() * sizeof(surface::Vertex) : 0, nullptr, GL_STATIC_DRAW);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, mesh_ ? mesh_->indices.size() * sizeof(uint32_t) : 0, nullptr, GL_STATIC_DRAW);
        if (glGetError() != GL_NO_ERROR)
            error_ = QStringLiteral("Cannot allocate surface mesh GPU buffers.");
    }
    if (!error_.isEmpty()) {
        glBindVertexArray(0);
        return false;
    }
    if (!mesh_ || mesh_->indices.empty()) {
        glBindVertexArray(0);
        return false;
    }
    size_t budget = 4 * 1024 * 1024;
    const auto upload = [&](GLenum target, const void* data, size_t size, size_t& offset) {
        const auto count = std::min(size - offset, budget);
        if (count)
            glBufferSubData(target, GLintptr(offset), GLsizeiptr(count), static_cast<const char*>(data) + offset);
        offset += count;
        budget -= count;
    };
    upload(GL_ARRAY_BUFFER, mesh_->vertices.data(), mesh_->vertices.size() * sizeof(surface::Vertex), vertexBytes_);
    upload(GL_ELEMENT_ARRAY_BUFFER, mesh_->indices.data(), mesh_->indices.size() * sizeof(uint32_t), indexBytes_);
    if (mesh_->texture) {
        const auto& t = *mesh_->texture;
        const int rows = std::min(t.height - textureRows_, int(budget / (size_t(t.width)*2)));
        if (rows > 0) {
            glActiveTexture(GL_TEXTURE0);
            glBindTexture(GL_TEXTURE_2D, texture_);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
            glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
            glTexSubImage2D(GL_TEXTURE_2D, 0, 0, textureRows_, t.width, rows, GL_RG, GL_UNSIGNED_BYTE, t.pixels.data()+size_t(textureRows_)*t.width*2);
            glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
            textureRows_ += rows;
        }
    }
    if (glGetError() != GL_NO_ERROR) {
        error_ = QStringLiteral("Cannot upload surface mesh GPU buffers.");
        glBindVertexArray(0);
        return false;
    }
    const bool pending = vertexBytes_ != mesh_->vertices.size() * sizeof(surface::Vertex) || indexBytes_ != mesh_->indices.size() * sizeof(uint32_t) || (mesh_->texture && textureRows_ != mesh_->texture->height);
    if (!pending && opacity > 0) {
        QMatrix4x4 mvp = matrix;
        mvp.translate(float(mesh_->origin[0] - origin[0]), float(mesh_->origin[1] - origin[1]), float(mesh_->origin[2] - origin[2]));
        GLint samples = 0;
        glGetIntegerv(GL_SAMPLES, &samples);
        glEnable(GL_DEPTH_TEST);
        glDepthFunc(GL_LEQUAL);
        glDepthMask(GL_TRUE);
        glDisable(GL_CULL_FACE);
        glDisable(GL_BLEND);
        glDisable(GL_SAMPLE_COVERAGE);
        glEnable(GL_MULTISAMPLE);
        if (samples > 1) {
            glEnable(GL_SAMPLE_ALPHA_TO_COVERAGE);
            glEnable(GL_SAMPLE_ALPHA_TO_ONE);
        }
        program_.bind();
        program_.setUniformValue("mvp", mvp);
        program_.setUniformValue("rotation", rotation.toRotationMatrix());
        // World position of each fragment (canvas frame) = position + (mesh origin - canvas origin),
        // matching the mvp translate, so the volume slab's toTexture maps it correctly.
        program_.setUniformValue("originOffset", QVector3D(float(mesh_->origin[0] - origin[0]),
                                                           float(mesh_->origin[1] - origin[1]),
                                                           float(mesh_->origin[2] - origin[2])));
        program_.setUniformValue("opacity", std::clamp(opacity, 0.f, 1.f));
        program_.setUniformValue("multisample", samples > 1);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, texture_);
        program_.setUniformValue("ct", 0);
        program_.setUniformValue("textured", textured && bool(mesh_->texture));
        program_.setUniformValue("useVertexColor", useVertexColor);
        program_.setUniformValue("windowRange", QVector2D(low, high));
        program_.setUniformValue("textureSize", mesh_->texture ? QVector2D(mesh_->texture->width, mesh_->texture->height) : QVector2D(1,1));
        // Keep the 3D CT sampler bound on unit 1 (always complete) so it never clashes with the 2D `ct`
        // sampler on unit 0; upload the real slab and enable sampling only in mesh CT mode.
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_3D, volumeTexture_);
        program_.setUniformValue("ctVolume", 1);
        if (volumeSample && ctSlab && !ctSlab->pixels.empty()) {
            if (volumeSlabKey_ != ctSlab) {
                glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
                glTexImage3D(GL_TEXTURE_3D, 0, GL_RG8, ctSlab->width, ctSlab->height, ctSlab->depth, 0, GL_RG, GL_UNSIGNED_BYTE, ctSlab->pixels.data());
                glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
                volumeSlabKey_ = ctSlab;
            }
            QMatrix4x4 box;
            box.setColumn(0, QVector4D(float(ctSlab->right[0]), float(ctSlab->right[1]), float(ctSlab->right[2]), 0));
            box.setColumn(1, QVector4D(float(ctSlab->down[0]), float(ctSlab->down[1]), float(ctSlab->down[2]), 0));
            box.setColumn(2, QVector4D(float(ctSlab->through[0]), float(ctSlab->through[1]), float(ctSlab->through[2]), 0));
            box.setColumn(3, QVector4D(float(ctSlab->origin[0] - origin[0]), float(ctSlab->origin[1] - origin[1]), float(ctSlab->origin[2] - origin[2]), 1));
            program_.setUniformValue("toTexture", box.inverted());
            program_.setUniformValue("volumeSample", true);
        } else {
            program_.setUniformValue("volumeSample", false);
        }
        glActiveTexture(GL_TEXTURE0);
        glEnableVertexAttribArray(2);
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, sizeof(surface::Vertex), reinterpret_cast<void*>(offsetof(surface::Vertex, uv)));
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(surface::Vertex), nullptr);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(surface::Vertex), reinterpret_cast<void*>(offsetof(surface::Vertex, normal)));
        glEnableVertexAttribArray(3);
        glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, sizeof(surface::Vertex), reinterpret_cast<void*>(offsetof(surface::Vertex, color)));
        glDrawElements(GL_TRIANGLES, GLsizei(mesh_->indices.size()), GL_UNSIGNED_INT, nullptr);
        program_.release();
        glActiveTexture(GL_TEXTURE1);
        glBindTexture(GL_TEXTURE_3D, 0);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, 0);
        glDisable(GL_SAMPLE_ALPHA_TO_COVERAGE);
        glDisable(GL_SAMPLE_ALPHA_TO_ONE);
        glDisable(GL_DEPTH_TEST);
    }
    glBindVertexArray(0);
    return pending;
}
}  // namespace vc3d::fibers3d
