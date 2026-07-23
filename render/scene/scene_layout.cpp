/**************************************************************************/
/* scene_layout.cpp                                                       */
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

#include "scene/scene_layout.hpp"

#include "resources/descriptor_heap.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/gpu_instance.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Scene
        {
            SceneLayout::SceneLayout(Vulkan::VulkanDevice& device, Resources::DescriptorHeap& heap)
                : device_(device), heap_(heap)
            {
                constexpr VkShaderStageFlags BOTH_STAGES =
                    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

                VkDescriptorSetLayoutBinding bindings[BINDING_COUNT]{};
                bindings[SCENE_BINDING].binding = SCENE_BINDING;
                bindings[SCENE_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[SCENE_BINDING].descriptorCount = 1;
                bindings[SCENE_BINDING].stageFlags = BOTH_STAGES;
                for (std::uint32_t i = 0; i < IMAGE_BINDING_COUNT; ++i)
                {
                    const std::uint32_t binding = FIRST_IMAGE_BINDING + i;
                    bindings[binding].binding = binding;
                    bindings[binding].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    bindings[binding].descriptorCount = 1;
                    bindings[binding].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                }

                bindings[MATERIAL_BINDING].binding = MATERIAL_BINDING;
                bindings[MATERIAL_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[MATERIAL_BINDING].descriptorCount = 1;
                bindings[MATERIAL_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                bindings[MOTION_BINDING].binding = MOTION_BINDING;
                bindings[MOTION_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[MOTION_BINDING].descriptorCount = 1;
                bindings[MOTION_BINDING].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

                bindings[TEMPORAL_BINDING].binding = TEMPORAL_BINDING;
                bindings[TEMPORAL_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[TEMPORAL_BINDING].descriptorCount = 1;
                bindings[TEMPORAL_BINDING].stageFlags = BOTH_STAGES;

                bindings[SHADOW_BINDING].binding = SHADOW_BINDING;
                bindings[SHADOW_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[SHADOW_BINDING].descriptorCount = 1;
                bindings[SHADOW_BINDING].stageFlags = BOTH_STAGES;

                bindings[SHADOW_ATLAS_BINDING].binding = SHADOW_ATLAS_BINDING;
                bindings[SHADOW_ATLAS_BINDING].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[SHADOW_ATLAS_BINDING].descriptorCount = 1;
                bindings[SHADOW_ATLAS_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                bindings[SHADOW_DEPTH_BINDING].binding = SHADOW_DEPTH_BINDING;
                bindings[SHADOW_DEPTH_BINDING].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[SHADOW_DEPTH_BINDING].descriptorCount = 1;
                bindings[SHADOW_DEPTH_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                bindings[IBL_SH_BINDING].binding = IBL_SH_BINDING;
                bindings[IBL_SH_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[IBL_SH_BINDING].descriptorCount = 1;
                bindings[IBL_SH_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                // The clustered light engine: three storage buffers (the light array, the
                // per-cluster count grid, the index list) and one config uniform, all read
                // by the fragment stage where shading happens.
                const std::uint32_t light_storage[3] = {LIGHT_BINDING, CLUSTER_GRID_BINDING,
                                                        LIGHT_INDEX_BINDING};
                for (std::uint32_t binding : light_storage)
                {
                    bindings[binding].binding = binding;
                    bindings[binding].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    bindings[binding].descriptorCount = 1;
                    bindings[binding].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                }

                bindings[CLUSTER_CONFIG_BINDING].binding = CLUSTER_CONFIG_BINDING;
                bindings[CLUSTER_CONFIG_BINDING].descriptorType =
                    VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[CLUSTER_CONFIG_BINDING].descriptorCount = 1;
                bindings[CLUSTER_CONFIG_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                bindings[LIGHT_SHADOW_ATLAS_BINDING].binding = LIGHT_SHADOW_ATLAS_BINDING;
                bindings[LIGHT_SHADOW_ATLAS_BINDING].descriptorType =
                    VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[LIGHT_SHADOW_ATLAS_BINDING].descriptorCount = 1;
                bindings[LIGHT_SHADOW_ATLAS_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                // The shadow matrices are read by the shadow pass's vertex shader (to
                // project geometry into a tile) and the shading pass's fragment shader (to
                // sample it), so both stages.
                bindings[LIGHT_SHADOW_DATA_BINDING].binding = LIGHT_SHADOW_DATA_BINDING;
                bindings[LIGHT_SHADOW_DATA_BINDING].descriptorType =
                    VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[LIGHT_SHADOW_DATA_BINDING].descriptorCount = 1;
                bindings[LIGHT_SHADOW_DATA_BINDING].stageFlags = BOTH_STAGES;

                // Clustered decals: the decal array, its per-cluster count grid, and the
                // index list, all read by the fragment stage before shading.
                const std::uint32_t decal_storage[3] = {DECAL_BINDING, DECAL_GRID_BINDING,
                                                        DECAL_INDEX_BINDING};
                for (std::uint32_t binding : decal_storage)
                {
                    bindings[binding].binding = binding;
                    bindings[binding].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    bindings[binding].descriptorCount = 1;
                    bindings[binding].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                }

                // The resolved ambient-occlusion target, sampled by the shading fragment stage.
                bindings[AO_BINDING].binding = AO_BINDING;
                bindings[AO_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[AO_BINDING].descriptorCount = 1;
                bindings[AO_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                // The atmosphere LUT stack, sampled by the sky march's fragment stage.
                const std::uint32_t atmosphere_luts[5] = {TRANSMITTANCE_LUT_BINDING,
                                                          MULTISCATTER_LUT_BINDING,
                                                          SKY_VIEW_LUT_BINDING,
                                                          AERIAL_LUT_BINDING,
                                                          FOG_LUT_BINDING};
                for (std::uint32_t binding : atmosphere_luts)
                {
                    bindings[binding].binding = binding;
                    bindings[binding].descriptorType =
                        VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    bindings[binding].descriptorCount = 1;
                    bindings[binding].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                }

                // Probe-volume GI: the SH grid (a storage buffer) and its config block, both
                // read by the shading fragment stage where the trilinear gather happens.
                bindings[GI_PROBE_SH_BINDING].binding = GI_PROBE_SH_BINDING;
                bindings[GI_PROBE_SH_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[GI_PROBE_SH_BINDING].descriptorCount = 1;
                bindings[GI_PROBE_SH_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                bindings[GI_PROBE_CONFIG_BINDING].binding = GI_PROBE_CONFIG_BINDING;
                bindings[GI_PROBE_CONFIG_BINDING].descriptorType =
                    VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[GI_PROBE_CONFIG_BINDING].descriptorCount = 1;
                bindings[GI_PROBE_CONFIG_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                // The post-processing block, read by the fragment stage of the display
                // transform and the DoF/motion-blur passes.
                bindings[POST_BINDING].binding = POST_BINDING;
                bindings[POST_BINDING].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[POST_BINDING].descriptorCount = 1;
                bindings[POST_BINDING].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;

                // The meshlet path drives this same set 0 from the task and mesh stages: the
                // task shader reads the scene block for the frustum, and the mesh shader reads
                // the scene block, the motion array, and the temporal block to transform and
                // motion-vector each vertex. A binding may only be accessed from a stage in its
                // flags, so those stages are added here — but only where mesh shaders exist, as
                // the task/mesh stage bits are invalid without the feature enabled.
                if (device_.supports_mesh_shader())
                {
                    bindings[SCENE_BINDING].stageFlags |=
                        VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT;
                    bindings[MOTION_BINDING].stageFlags |= VK_SHADER_STAGE_MESH_BIT_EXT;
                    bindings[TEMPORAL_BINDING].stageFlags |= VK_SHADER_STAGE_MESH_BIT_EXT;
                }

                // A push-descriptor set: passes push their bindings inline rather than
                // allocate and write a throw-away set every frame. The whole set is well
                // under the 32-descriptor floor every 1.4 device guarantees for a push
                // set, so no runtime capacity check is needed.
                static_assert(BINDING_COUNT <= 32,
                              "scene set exceeds the guaranteed push-descriptor floor");
                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.flags = VK_DESCRIPTOR_SET_LAYOUT_CREATE_PUSH_DESCRIPTOR_BIT;
                layout_info.bindingCount = BINDING_COUNT;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(scene)");

                // Set 1 is the bindless heap. A pipeline layout may declare more sets than
                // its shaders reference, so including it costs nothing today and means the
                // material work never has to rebuild every pipeline layout to reach it.
                VkDescriptorSetLayout sets[2] = {set_layout_, heap_.layout()};

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                range.size = sizeof(MeshPushConstants);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = heap_.available() ? 2 : 1;
                pipeline_info.pSetLayouts = sets;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(scene)");

                // Set 2: the GPU-driven per-instance and compacted buffers, read only by the
                // vertex stage. A plain (non-push) set the draw allocates and writes each
                // frame — the buffers are large storage arrays, not the handful of bindings a
                // push set is for.
                VkDescriptorSetLayoutBinding instance_bindings[2]{};
                for (std::uint32_t i = 0; i < 2; ++i)
                {
                    instance_bindings[i].binding = i;
                    instance_bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    instance_bindings[i].descriptorCount = 1;
                    instance_bindings[i].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                }
                VkDescriptorSetLayoutCreateInfo instance_info{};
                instance_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                instance_info.bindingCount = 2;
                instance_info.pBindings = instance_bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &instance_info, nullptr,
                                                          &instance_set_layout_),
                              "vkCreateDescriptorSetLayout(instance)");

                // The GPU-driven pipeline layout: the same set 0 and set 1 as the classic
                // layout so one push and one heap bind serve either, plus set 2, plus a small
                // vertex-stage push carrying the drawn bucket's base into the compacted list.
                VkDescriptorSetLayout gpu_sets[3] = {set_layout_, heap_.layout(),
                                                     instance_set_layout_};
                VkPushConstantRange gpu_range{};
                gpu_range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                gpu_range.size = sizeof(GpuDrawPush);

                VkPipelineLayoutCreateInfo gpu_pipeline_info{};
                gpu_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                // Without the bindless heap set 1 is dropped, so set 2 moves up to index 1;
                // but the shaders address set 2 explicitly, so the layout must keep the slot.
                // The heap is available on every target this renderer supports, so the simple
                // three-set layout stands; the one-set fallback keeps the classic path only.
                gpu_pipeline_info.setLayoutCount = heap_.available() ? 3 : 0;
                gpu_pipeline_info.pSetLayouts = gpu_sets;
                gpu_pipeline_info.pushConstantRangeCount = 1;
                gpu_pipeline_info.pPushConstantRanges = &gpu_range;
                if (heap_.available())
                    Vulkan::check(vkCreatePipelineLayout(device_.device(), &gpu_pipeline_info,
                                                         nullptr, &gpu_pipeline_layout_),
                                  "vkCreatePipelineLayout(gpu scene)");

                // The meshlet path's set 2 and pipeline layout, built only when the device can
                // draw with mesh shaders. Four storage buffers (meshlet descriptors, meshlet
                // vertices, meshlet triangles, mesh vertices) read by the task and mesh stages,
                // and a task/mesh push constant carrying the instance transform and indices.
                if (heap_.available() && device_.supports_mesh_shader())
                {
                    constexpr VkShaderStageFlags MESH_STAGES =
                        VK_SHADER_STAGE_TASK_BIT_EXT | VK_SHADER_STAGE_MESH_BIT_EXT;
                    VkDescriptorSetLayoutBinding meshlet_bindings[4]{};
                    for (std::uint32_t i = 0; i < 4; ++i)
                    {
                        meshlet_bindings[i].binding = i;
                        meshlet_bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                        meshlet_bindings[i].descriptorCount = 1;
                        meshlet_bindings[i].stageFlags = MESH_STAGES;
                    }
                    VkDescriptorSetLayoutCreateInfo meshlet_info{};
                    meshlet_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                    meshlet_info.bindingCount = 4;
                    meshlet_info.pBindings = meshlet_bindings;
                    Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &meshlet_info,
                                                              nullptr, &meshlet_set_layout_),
                                  "vkCreateDescriptorSetLayout(meshlet)");

                    VkDescriptorSetLayout meshlet_sets[3] = {set_layout_, heap_.layout(),
                                                             meshlet_set_layout_};
                    VkPushConstantRange meshlet_range{};
                    meshlet_range.stageFlags = MESH_STAGES;
                    meshlet_range.size = sizeof(MeshletPushConstants);

                    VkPipelineLayoutCreateInfo meshlet_pipeline_info{};
                    meshlet_pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                    meshlet_pipeline_info.setLayoutCount = 3;
                    meshlet_pipeline_info.pSetLayouts = meshlet_sets;
                    meshlet_pipeline_info.pushConstantRangeCount = 1;
                    meshlet_pipeline_info.pPushConstantRanges = &meshlet_range;
                    Vulkan::check(vkCreatePipelineLayout(device_.device(), &meshlet_pipeline_info,
                                                         nullptr, &meshlet_pipeline_layout_),
                                  "vkCreatePipelineLayout(meshlet)");
                }
            }

            SceneLayout::~SceneLayout()
            {
                if (meshlet_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), meshlet_pipeline_layout_, nullptr);
                if (meshlet_set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), meshlet_set_layout_, nullptr);
                if (gpu_pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), gpu_pipeline_layout_, nullptr);
                if (instance_set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), instance_set_layout_, nullptr);
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void SceneLayout::bind_heap(VkCommandBuffer cmd) const
            {
                if (!heap_.available())
                    return;
                Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                               pipeline_layout_, 1, heap_.set());
            }

            void SceneLayout::bind_gpu_heap(VkCommandBuffer cmd) const
            {
                if (!heap_.available() || gpu_pipeline_layout_ == VK_NULL_HANDLE)
                    return;
                Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                               gpu_pipeline_layout_, 1, heap_.set());
            }

            void SceneLayout::bind_meshlet_heap(VkCommandBuffer cmd) const
            {
                if (!heap_.available() || meshlet_pipeline_layout_ == VK_NULL_HANDLE)
                    return;
                Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                               meshlet_pipeline_layout_, 1, heap_.set());
            }

        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
