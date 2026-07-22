/**************************************************************************/
/* environment_probe_tracer.cpp                                           */
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
/*                                                                        */
/*     http://www.apache.org/licenses/LICENSE-2.0                         */
/*                                                                        */
/* Unless required by applicable law or agreed to in writing, software    */
/* distributed under the License is distributed on an "AS IS" BASIS,      */
/* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or        */
/* implied. See the License for the specific language governing           */
/* permissions and limitations under the License.                         */
/**************************************************************************/

#include "gi/environment_probe_tracer.hpp"

#include "frame/frame_context.hpp"
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
        namespace Gi
        {
            namespace
            {
                constexpr std::uint32_t GROUP_SIZE = 64;
            }

            EnvironmentProbeTracer::EnvironmentProbeTracer(
                Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                Resources::GraphicsPipelineFactory& pipelines)
                : device_(device), shaders_(shaders), pipelines_(pipelines)
            {
                // Two storage buffers: the probe SH output and the environment SH input.
                VkDescriptorSetLayoutBinding bindings[2]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 2;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &layout_),
                              "vkCreateDescriptorSetLayout(gi environment tracer)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(gi environment tracer)");

                create_pipeline();
            }

            EnvironmentProbeTracer::~EnvironmentProbeTracer()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), layout_, nullptr);
            }

            void EnvironmentProbeTracer::create_pipeline()
            {
                pipeline_ = pipelines_.create_compute(pipeline_layout_,
                                                      shaders_.module("gi_probe_relight.comp"));
            }

            void EnvironmentProbeTracer::destroy_pipeline()
            {
                if (pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), pipeline_, nullptr);
                pipeline_ = VK_NULL_HANDLE;
            }

            void EnvironmentProbeTracer::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void EnvironmentProbeTracer::relight(VkCommandBuffer cmd,
                                                 const ProbeRelightInputs& inputs)
            {
                if (inputs.frame == nullptr || inputs.probe_count == 0)
                    return;

                const VkDescriptorSet set = inputs.frame->descriptors->allocate(layout_);
                Resources::DescriptorWriter writer;
                writer.storage_buffer(0, inputs.probe_sh, inputs.probe_sh_bytes);
                writer.storage_buffer(1, inputs.environment_sh, inputs.environment_sh_bytes);
                writer.update(device_.device(), set);

                Push push{static_cast<std::int32_t>(inputs.probe_count)};
                vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_COMPUTE, pipeline_);
                Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_COMPUTE,
                                               pipeline_layout_, 0, set);
                vkCmdPushConstants(cmd, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT, 0,
                                   sizeof(Push), &push);
                vkCmdDispatch(cmd, (inputs.probe_count + GROUP_SIZE - 1) / GROUP_SIZE, 1, 1);
            }
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
