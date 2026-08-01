/**************************************************************************/
/* terrain_layout.cpp                                                     */
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

#include "terrain/terrain_layout.hpp"

#include "resources/descriptor_heap.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/scene_layout.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Terrain
        {
            TerrainLayout::TerrainLayout(Vulkan::VulkanDevice& device, Scene::SceneLayout& scene,
                                         Resources::DescriptorHeap& heap)
                : device_(device), heap_(heap)
            {
                if (!heap.available())
                    return;

                VkDescriptorSetLayoutBinding bindings[3]{};

                // The node array: read by the vertex stage, which is the only stage that
                // needs to know where a node is.
                bindings[0].binding = NODE_BINDING;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

                // The height slots. Sampled in the vertex stage to place geometry, and in
                // the fragment stage by the later material work, so both are declared now
                // rather than rebuilding every terrain pipeline layout to add one.
                bindings[1].binding = HEIGHT_BINDING;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags =
                    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

                bindings[2].binding = BODY_BINDING;
                bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[2].descriptorCount = 1;
                bindings[2].stageFlags =
                    VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

                // A push-descriptor set like set 0: the contents change every frame and
                // nothing outlives the command buffer, so allocating and writing a
                // throw-away set would be work for no one.
                VkDescriptorSetLayoutCreateInfo set_info{};
                set_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                set_info.flags = VK_DESCRIPTOR_SET_LAYOUT_CREATE_PUSH_DESCRIPTOR_BIT;
                set_info.bindingCount = 3;
                set_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &set_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(terrain)");

                // Sets 0 and 1 are the scene's own, unchanged, which is the whole point:
                // one scene push and one heap bind serve a terrain draw exactly as they
                // serve a mesh draw, and pbr.frag cannot tell the two apart.
                VkDescriptorSetLayout sets[3] = {scene.set_layout(), heap.layout(), set_layout_};

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                range.size = sizeof(TerrainPushConstants);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 3;
                pipeline_info.pSetLayouts = sets;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(terrain)");

                available_ = true;
            }

            void TerrainLayout::bind_heap(VkCommandBuffer command) const
            {
                if (!available_)
                    return;
                const VkDescriptorSet set = heap_.set();
                vkCmdBindDescriptorSets(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                        pipeline_layout_, 1, 1, &set, 0, nullptr);
            }

            TerrainLayout::~TerrainLayout()
            {
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }
        } // namespace Terrain
    } // namespace Render
} // namespace SushiEngine
