/**************************************************************************/
/* light_cull_pass.cpp                                                    */
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

#include "passes/light_cull_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "lighting/cluster_config.hpp"
#include "lighting/light_system.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            namespace
            {
                /** @brief Threads one workgroup of the cull shader covers. */
                constexpr std::uint32_t GROUP_SIZE = 64;

                /** @brief Bytes the per-cluster light count grid occupies. */
                constexpr VkDeviceSize GRID_BYTES =
                    Lighting::CLUSTER_COUNT * sizeof(std::uint32_t);

                /** @brief Bytes the cluster→light index list occupies. */
                constexpr VkDeviceSize INDEX_BYTES =
                    Lighting::LIGHT_INDEX_COUNT * sizeof(std::uint32_t);

                /** @brief Bytes the per-cluster decal count grid occupies. */
                constexpr VkDeviceSize DECAL_GRID_BYTES =
                    Lighting::CLUSTER_COUNT * sizeof(std::uint32_t);

                /** @brief Bytes the cluster→decal index list occupies. */
                constexpr VkDeviceSize DECAL_INDEX_BYTES =
                    Lighting::DECAL_INDEX_COUNT * sizeof(std::uint32_t);

                std::uint32_t group_count(std::uint32_t value, std::uint32_t group) noexcept
                {
                    return value == 0 ? 1u : (value + group - 1) / group;
                }
            } // namespace

            LightCullPass::LightCullPass(Vulkan::VulkanDevice& device,
                                         Resources::ShaderLibrary& shaders,
                                         Resources::GraphicsPipelineFactory& pipelines,
                                         Lighting::LightSystem& lights)
                : device_(device), shaders_(shaders), pipelines_(pipelines), lights_(lights)
            {
                // Six storage buffers: the light array (read), its count grid and index
                // list (written), then the decal array (read) and its grid and index list.
                VkDescriptorSetLayoutBinding bindings[6]{};
                for (std::uint32_t i = 0; i < 6; ++i)
                {
                    bindings[i].binding = i;
                    bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    bindings[i].descriptorCount = 1;
                    bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 6;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(light cull)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(light cull)");

                create_pipeline();
            }

            LightCullPass::~LightCullPass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void LightCullPass::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_,
                                                      shaders_.module("cluster_build.comp"));
            }

            void LightCullPass::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void LightCullPass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void LightCullPass::register_pass(Graph::RenderGraph& graph,
                                              const Frame::FrameContext& frame)
            {
                if (!frame.targets.cluster_grid.valid() || frame.camera == nullptr)
                    return;

                // The camera basis, derived exactly as the scene uniform fill derives it,
                // so the froxel bounds live in the same view frame the shading pass maps a
                // pixel into: unit right/up/forward with view_z = dot(forward, position).
                const Mat4& view = frame.camera->view;
                const Mat4& proj = frame.camera->projection;
                Push push{};
                push.right[0] = static_cast<float>(view.m[0]);
                push.right[1] = static_cast<float>(view.m[4]);
                push.right[2] = static_cast<float>(view.m[8]);
                push.right[3] = proj.m[0] != 0.0 ? static_cast<float>(1.0 / proj.m[0]) : 1.0f;
                push.up[0] = static_cast<float>(view.m[1]);
                push.up[1] = static_cast<float>(view.m[5]);
                push.up[2] = static_cast<float>(view.m[9]);
                push.up[3] = proj.m[5] != 0.0 ? static_cast<float>(1.0 / -proj.m[5]) : 1.0f;
                push.forward[0] = static_cast<float>(-view.m[2]);
                push.forward[1] = static_cast<float>(-view.m[6]);
                push.forward[2] = static_cast<float>(-view.m[10]);
                push.forward[3] = static_cast<float>(lights_.light_count());
                push.params[0] = lights_.cluster_near();
                push.params[1] = lights_.cluster_far();
                push.params[2] = static_cast<float>(lights_.decal_count());

                const Graph::BufferHandle grid = frame.targets.cluster_grid;
                const Graph::BufferHandle index = frame.targets.light_index;
                const Graph::BufferHandle decal_grid = frame.targets.decal_grid;
                const Graph::BufferHandle decal_index = frame.targets.decal_index;

                graph.add_pass(
                    "light cull",
                    [grid, index, decal_grid, decal_index](Graph::RenderPassBuilder& builder)
                    {
                        builder.write(grid, Graph::BufferAccess::StorageWrite);
                        builder.write(index, Graph::BufferAccess::StorageWrite);
                        builder.write(decal_grid, Graph::BufferAccess::StorageWrite);
                        builder.write(decal_index, Graph::BufferAccess::StorageWrite);
                    },
                    [this, &frame, grid, index, decal_grid, decal_index, push](
                        VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);

                        Resources::DescriptorWriter writer;
                        // The light and decal arrays are host-written before the graph runs,
                        // so they are bound directly like the material array; the grids and
                        // index lists are graph transients this pass writes.
                        writer.storage_buffer(0, lights_.light_buffer(),
                                              lights_.light_buffer_range());
                        writer.storage_buffer(1, context.buffer(grid), GRID_BYTES);
                        writer.storage_buffer(2, context.buffer(index), INDEX_BYTES);
                        writer.storage_buffer(3, lights_.decal_buffer(),
                                              lights_.decal_buffer_range());
                        writer.storage_buffer(4, context.buffer(decal_grid), DECAL_GRID_BYTES);
                        writer.storage_buffer(5, context.buffer(decal_index), DECAL_INDEX_BYTES);
                        writer.update(device_.device(), set);

                        vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                        Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);
                        vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                           sizeof(Push), &push);
                        vkCmdDispatch(cmd, group_count(Lighting::CLUSTER_COUNT, GROUP_SIZE), 1, 1);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
