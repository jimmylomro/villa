#pragma once

#include <string>
#include <QColor>

namespace vc3d
{
// A stable, well-separated colour per mesh so distinct meshes are visually distinguishable,
// identically in the 2D cross-section overlay and the 3D view. FNV-1a hash of the uuid -> hue;
// fixed high saturation/value reads well over the dark slice/background.
inline QColor meshColour(const std::string& uuid)
{
    unsigned h = 2166136261u;
    for (unsigned char c : uuid) { h ^= c; h *= 16777619u; }
    return QColor::fromHsv(int(h % 360u), 190, 255);
}
}  // namespace vc3d
