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

                // Inverts the upper-left 3x3 of a column-major 4x4 into a row-major r[3][3].
                // Falls back to the identity on a singular matrix.
                void invert_upper3x3(const Mat4& m, float r[3][3]) noexcept
                {
                    const float a00 = m.m[0], a01 = m.m[4], a02 = m.m[8];
                    const float a10 = m.m[1], a11 = m.m[5], a12 = m.m[9];
                    const float a20 = m.m[2], a21 = m.m[6], a22 = m.m[10];
                    const float c00 = a11 * a22 - a12 * a21;
                    const float c01 = a12 * a20 - a10 * a22;
                    const float c02 = a10 * a21 - a11 * a20;
                    float determinant = a00 * c00 + a01 * c01 + a02 * c02;
                    if (std::fabs(determinant) < 1e-12f)
                    {
                        r[0][0] = 1.0f; r[0][1] = 0.0f; r[0][2] = 0.0f;
                        r[1][0] = 0.0f; r[1][1] = 1.0f; r[1][2] = 0.0f;
                        r[2][0] = 0.0f; r[2][1] = 0.0f; r[2][2] = 1.0f;
                        return;
                    }
                    const float inv = 1.0f / determinant;
                    r[0][0] = c00 * inv;
                    r[0][1] = (a02 * a21 - a01 * a22) * inv;
                    r[0][2] = (a01 * a12 - a02 * a11) * inv;
                    r[1][0] = c01 * inv;
                    r[1][1] = (a00 * a22 - a02 * a20) * inv;
                    r[1][2] = (a02 * a10 - a00 * a12) * inv;
                    r[2][0] = c02 * inv;
                    r[2][1] = (a01 * a20 - a00 * a21) * inv;
                    r[2][2] = (a00 * a11 - a01 * a10) * inv;
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

                // The mesh-instance count is filled by the tracer, which owns the brick atlas.
                out.extra[0] = 0;
                out.extra[1] = 0;
                out.extra[2] = 0;
                out.extra[3] = 0;
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

            void fill_sdf_mesh_instance(const Mat4& model, const double eye[3],
                                        const float aabb_min[3], const float aabb_max[3],
                                        std::int32_t slot, const float albedo[3],
                                        SdfMeshInstance& out) noexcept
            {
                float r[3][3];
                invert_upper3x3(model, r);

                // The model's translation, rebased against the eye: local = R * (world - d).
                const float d0 = static_cast<float>(static_cast<double>(model.m[12]) - eye[0]);
                const float d1 = static_cast<float>(static_cast<double>(model.m[13]) - eye[1]);
                const float d2 = static_cast<float>(static_cast<double>(model.m[14]) - eye[2]);
                const float t0 = -(r[0][0] * d0 + r[0][1] * d1 + r[0][2] * d2);
                const float t1 = -(r[1][0] * d0 + r[1][1] * d1 + r[1][2] * d2);
                const float t2 = -(r[2][0] * d0 + r[2][1] * d1 + r[2][2] * d2);

                // Column-major inverse: columns are the rows of R, the last column the
                // rebased inverse translation.
                out.inv_model[0] = r[0][0]; out.inv_model[1] = r[1][0];
                out.inv_model[2] = r[2][0]; out.inv_model[3] = 0.0f;
                out.inv_model[4] = r[0][1]; out.inv_model[5] = r[1][1];
                out.inv_model[6] = r[2][1]; out.inv_model[7] = 0.0f;
                out.inv_model[8] = r[0][2]; out.inv_model[9] = r[1][2];
                out.inv_model[10] = r[2][2]; out.inv_model[11] = 0.0f;
                out.inv_model[12] = t0; out.inv_model[13] = t1;
                out.inv_model[14] = t2; out.inv_model[15] = 1.0f;

                // A local unit maps to this many world metres, so a local distance scales by
                // it — the average column length, a sound uniform-scale approximation.
                const float world_scale =
                    (column_length(model, 0) + column_length(model, 1) + column_length(model, 2)) /
                    3.0f;

                out.aabb_min[0] = aabb_min[0];
                out.aabb_min[1] = aabb_min[1];
                out.aabb_min[2] = aabb_min[2];
                out.aabb_min[3] = static_cast<float>(slot);
                out.aabb_max[0] = aabb_max[0];
                out.aabb_max[1] = aabb_max[1];
                out.aabb_max[2] = aabb_max[2];
                out.aabb_max[3] = world_scale;
                out.albedo[0] = albedo[0];
                out.albedo[1] = albedo[1];
                out.albedo[2] = albedo[2];
                out.albedo[3] = 0.0f;
            }
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
