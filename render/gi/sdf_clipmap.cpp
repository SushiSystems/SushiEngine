/**************************************************************************/
/* sdf_clipmap.cpp                                                        */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "gi/sdf_clipmap.hpp"

#include <cmath>

#include <SushiEngine/render/scene_view.hpp>

#include "frame/frame_context.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Gi
        {
            namespace
            {
                float column_length(const Mat4& m, int column) noexcept
                {
                    const float x = m.m[column * 4 + 0];
                    const float y = m.m[column * 4 + 1];
                    const float z = m.m[column * 4 + 2];
                    return std::sqrt(x * x + y * y + z * z);
                }
            } // namespace

            void configure_sdf_clipmap(const double eye[3], std::int32_t primitive_count,
                                       SdfClipmapConfig& out) noexcept
            {
                const double voxel =
                    static_cast<double>(SDF_CLIPMAP_EXTENT_METRES) / SDF_CLIPMAP_RESOLUTION;
                const double half_extent = 0.5 * static_cast<double>(SDF_CLIPMAP_EXTENT_METRES);

                for (int axis = 0; axis < 3; ++axis)
                {
                    // Snap the cube centre to a voxel multiple so the field holds still until
                    // the camera crosses a voxel, then place the min corner half a cube below.
                    const double center = std::floor(eye[axis] / voxel + 0.5) * voxel;
                    const double min_corner = center - half_extent;
                    out.origin_voxel[axis] = static_cast<float>(min_corner - eye[axis]);
                }
                out.origin_voxel[3] = static_cast<float>(voxel);

                out.resolution[0] = SDF_CLIPMAP_RESOLUTION;
                out.resolution[1] = SDF_CLIPMAP_RESOLUTION;
                out.resolution[2] = SDF_CLIPMAP_RESOLUTION;
                out.resolution[3] = primitive_count;
            }

            std::int32_t build_sdf_primitives(const Frame::SceneDrawList& draws, const double eye[3],
                                              SdfPrimitive* out, std::int32_t max) noexcept
            {
                std::int32_t count = 0;
                for (std::size_t i = 0; i < draws.instance_count && count < max; ++i)
                {
                    const MeshInstance& instance = draws.instances[i];
                    if (instance.mesh != INVALID_MESH)
                        continue; // imported meshes fold in as baked bricks later

                    const Mat4& model = instance.model;
                    const float scale_x = column_length(model, 0);
                    const float scale_y = column_length(model, 1);
                    const float scale_z = column_length(model, 2);

                    SdfPrimitive& p = out[count];
                    // The world centre is the model's translation, rebased against the eye.
                    p.center_kind[0] = static_cast<float>(static_cast<double>(model.m[12]) - eye[0]);
                    p.center_kind[1] = static_cast<float>(static_cast<double>(model.m[13]) - eye[1]);
                    p.center_kind[2] = static_cast<float>(static_cast<double>(model.m[14]) - eye[2]);

                    // The shape parameters are half-extents in the instance's own frame; the
                    // model's per-axis scale carries them to world size (rotation dropped).
                    const Vector3& s = instance.shape_params;
                    switch (instance.kind)
                    {
                        case MeshKind::Sphere:
                            p.center_kind[3] = static_cast<float>(SdfPrimitiveKind::Sphere);
                            p.extent[0] = static_cast<float>(s.x) * scale_x;
                            p.extent[1] = static_cast<float>(s.x) * scale_x;
                            p.extent[2] = static_cast<float>(s.x) * scale_x;
                            break;
                        case MeshKind::Cylinder:
                            p.center_kind[3] = static_cast<float>(SdfPrimitiveKind::Cylinder);
                            p.extent[0] = static_cast<float>(s.x) * scale_x;
                            p.extent[1] = static_cast<float>(s.y) * scale_y;
                            p.extent[2] = static_cast<float>(s.x) * scale_x;
                            break;
                        case MeshKind::Box:
                        default:
                            p.center_kind[3] = static_cast<float>(SdfPrimitiveKind::Box);
                            p.extent[0] = static_cast<float>(s.x) * scale_x;
                            p.extent[1] = static_cast<float>(s.y) * scale_y;
                            p.extent[2] = static_cast<float>(s.z) * scale_z;
                            break;
                    }
                    p.extent[3] = 0.0f;

                    p.albedo[0] = static_cast<float>(instance.color.x);
                    p.albedo[1] = static_cast<float>(instance.color.y);
                    p.albedo[2] = static_cast<float>(instance.color.z);
                    p.albedo[3] = 0.0f;
                    ++count;
                }
                return count;
            }
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
