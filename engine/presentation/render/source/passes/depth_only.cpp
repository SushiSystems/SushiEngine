/**************************************************************************/
/* depth_only.cpp                                                         */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/*                                                                        */
/* Licensed under the Apache License, Version 2.0 (the "License");        */
/* you may not use this file except in compliance with the License.       */
/* You may obtain a copy of the License at                                */
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "passes/depth_only.hpp"

#include <cstddef>

#include "geometry/mesh_registry.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            Resources::GraphicsPipelineDesc depth_only_pipeline_desc(VkPipelineLayout layout,
                                                                     VkShaderModule vertex,
                                                                     VkFormat depth_format)
            {
                Resources::GraphicsPipelineDesc desc;
                desc.layout = layout;
                desc.vertex_shader = vertex;
                desc.vertex_stride = sizeof(Geometry::MeshVertex);
                // The full attribute set, not just position: the vertex shader is shared
                // with the shading pass, and a pipeline must declare every attribute its
                // shader reads even where the depth result does not depend on them.
                desc.attribute_count = 6;
                desc.attributes[0] = {
                    0, VK_FORMAT_R32G32B32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, position))};
                desc.attributes[1] = {
                    1, VK_FORMAT_R32G32B32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, normal))};
                desc.attributes[2] = {
                    2, VK_FORMAT_R32G32B32A32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, tangent))};
                desc.attributes[3] = {
                    3, VK_FORMAT_R32G32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, uv0))};
                desc.attributes[4] = {
                    4, VK_FORMAT_R32G32_SFLOAT,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, uv1))};
                desc.attributes[5] = {
                    5, VK_FORMAT_R8G8B8A8_UNORM,
                    static_cast<std::uint32_t>(offsetof(Geometry::MeshVertex, color))};
                desc.depth_test = VK_TRUE;
                desc.depth_write = VK_TRUE;
                desc.color_count = 0;
                desc.depth_format = depth_format;
                return desc;
            }

            Scene::MeshPushConstants depth_only_push(const Mat4& model, const double eye[3],
                                                     float cascade)
            {
                Scene::MeshPushConstants push{};
                for (int i = 0; i < 16; ++i)
                    push.model[i] = static_cast<float>(model.m[i]);
                push.model[12] = static_cast<float>(model.m[12] - eye[0]);
                push.model[13] = static_cast<float>(model.m[13] - eye[1]);
                push.model[14] = static_cast<float>(model.m[14] - eye[2]);
                push.outline_shift[2] = cascade;
                push.entity_id = NO_PICK;
                push.selected = NO_PICK;
                return push;
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
