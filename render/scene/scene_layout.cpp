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
            }

            SceneLayout::~SceneLayout()
            {
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void SceneLayout::bind_heap(VkCommandBuffer cmd) const
            {
                if (!heap_.available())
                    return;
                const VkDescriptorSet set = heap_.set();
                vkCmdBindDescriptorSets(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline_layout_, 1,
                                        1, &set, 0, nullptr);
            }

        } // namespace Scene
    } // namespace Render
} // namespace SushiEngine
