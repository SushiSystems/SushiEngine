/**************************************************************************/
/* resource_state.cpp                                                     */
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

#include "resource_state.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Graph
        {
            TextureState texture_access_state(TextureAccess access) noexcept
            {
                TextureState state;
                switch (access)
                {
                    case TextureAccess::ColorAttachment:
                        state.stage = VK_PIPELINE_STAGE_2_COLOR_ATTACHMENT_OUTPUT_BIT;
                        state.access = VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT |
                                       VK_ACCESS_2_COLOR_ATTACHMENT_READ_BIT;
                        state.layout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                        break;
                    case TextureAccess::DepthStencilAttachment:
                        state.stage = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                                      VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
                        state.access = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
                                       VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
                        state.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                        break;
                    case TextureAccess::DepthStencilReadOnly:
                        state.stage = VK_PIPELINE_STAGE_2_EARLY_FRAGMENT_TESTS_BIT |
                                      VK_PIPELINE_STAGE_2_LATE_FRAGMENT_TESTS_BIT;
                        state.access = VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_READ_BIT;
                        state.layout = VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL;
                        break;
                    case TextureAccess::SampledFragment:
                        state.stage = VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
                        state.access = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
                        state.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                        break;
                    case TextureAccess::SampledCompute:
                        state.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                        state.access = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
                        state.layout = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;
                        break;
                    case TextureAccess::StorageComputeRead:
                        state.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                        state.access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
                        state.layout = VK_IMAGE_LAYOUT_GENERAL;
                        break;
                    case TextureAccess::StorageComputeWrite:
                        state.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                        state.access = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                        state.layout = VK_IMAGE_LAYOUT_GENERAL;
                        break;
                    case TextureAccess::StorageComputeReadWrite:
                        state.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                        state.access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                        state.layout = VK_IMAGE_LAYOUT_GENERAL;
                        break;
                    case TextureAccess::TransferSource:
                        state.stage = VK_PIPELINE_STAGE_2_COPY_BIT;
                        state.access = VK_ACCESS_2_TRANSFER_READ_BIT;
                        state.layout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
                        break;
                    case TextureAccess::TransferDestination:
                        state.stage = VK_PIPELINE_STAGE_2_COPY_BIT;
                        state.access = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                        state.layout = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
                        break;
                    case TextureAccess::PresentSource:
                        state.stage = VK_PIPELINE_STAGE_2_BOTTOM_OF_PIPE_BIT;
                        state.access = VK_ACCESS_2_NONE;
                        state.layout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
                        break;
                    case TextureAccess::ShadingRateAttachment:
                        state.stage =
                            VK_PIPELINE_STAGE_2_FRAGMENT_SHADING_RATE_ATTACHMENT_BIT_KHR;
                        state.access =
                            VK_ACCESS_2_FRAGMENT_SHADING_RATE_ATTACHMENT_READ_BIT_KHR;
                        state.layout =
                            VK_IMAGE_LAYOUT_FRAGMENT_SHADING_RATE_ATTACHMENT_OPTIMAL_KHR;
                        break;
                }
                return state;
            }

            BufferState buffer_access_state(BufferAccess access) noexcept
            {
                BufferState state;
                switch (access)
                {
                    case BufferAccess::UniformRead:
                        state.stage = VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT |
                                      VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT;
                        state.access = VK_ACCESS_2_UNIFORM_READ_BIT;
                        break;
                    case BufferAccess::UniformComputeRead:
                        state.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                        state.access = VK_ACCESS_2_UNIFORM_READ_BIT;
                        break;
                    case BufferAccess::StorageRead:
                        state.stage = VK_PIPELINE_STAGE_2_VERTEX_SHADER_BIT |
                                      VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT |
                                      VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                        state.access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT;
                        break;
                    case BufferAccess::StorageWrite:
                        state.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                        state.access = VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                        break;
                    case BufferAccess::StorageReadWrite:
                        state.stage = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                        state.access = VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT;
                        break;
                    case BufferAccess::IndirectRead:
                        state.stage = VK_PIPELINE_STAGE_2_DRAW_INDIRECT_BIT;
                        state.access = VK_ACCESS_2_INDIRECT_COMMAND_READ_BIT;
                        break;
                    case BufferAccess::VertexRead:
                        state.stage = VK_PIPELINE_STAGE_2_VERTEX_ATTRIBUTE_INPUT_BIT;
                        state.access = VK_ACCESS_2_VERTEX_ATTRIBUTE_READ_BIT;
                        break;
                    case BufferAccess::IndexRead:
                        state.stage = VK_PIPELINE_STAGE_2_INDEX_INPUT_BIT;
                        state.access = VK_ACCESS_2_INDEX_READ_BIT;
                        break;
                    case BufferAccess::TransferSource:
                        state.stage = VK_PIPELINE_STAGE_2_COPY_BIT;
                        state.access = VK_ACCESS_2_TRANSFER_READ_BIT;
                        break;
                    case BufferAccess::TransferDestination:
                        state.stage = VK_PIPELINE_STAGE_2_COPY_BIT;
                        state.access = VK_ACCESS_2_TRANSFER_WRITE_BIT;
                        break;
                    case BufferAccess::HostRead:
                        state.stage = VK_PIPELINE_STAGE_2_HOST_BIT;
                        state.access = VK_ACCESS_2_HOST_READ_BIT;
                        break;
                }
                return state;
            }

            bool texture_access_writes(TextureAccess access) noexcept
            {
                switch (access)
                {
                    case TextureAccess::ColorAttachment:
                    case TextureAccess::DepthStencilAttachment:
                    case TextureAccess::StorageComputeWrite:
                    case TextureAccess::StorageComputeReadWrite:
                    case TextureAccess::TransferDestination:
                        return true;
                    default:
                        return false;
                }
            }

            bool buffer_access_writes(BufferAccess access) noexcept
            {
                switch (access)
                {
                    case BufferAccess::StorageWrite:
                    case BufferAccess::StorageReadWrite:
                    case BufferAccess::TransferDestination:
                        return true;
                    default:
                        return false;
                }
            }

            VkImageUsageFlags texture_access_usage(TextureAccess access) noexcept
            {
                switch (access)
                {
                    case TextureAccess::ColorAttachment:
                        return VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
                    case TextureAccess::DepthStencilAttachment:
                    case TextureAccess::DepthStencilReadOnly:
                        return VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT;
                    case TextureAccess::SampledFragment:
                    case TextureAccess::SampledCompute:
                        return VK_IMAGE_USAGE_SAMPLED_BIT;
                    case TextureAccess::StorageComputeRead:
                    case TextureAccess::StorageComputeWrite:
                    case TextureAccess::StorageComputeReadWrite:
                        return VK_IMAGE_USAGE_STORAGE_BIT;
                    case TextureAccess::TransferSource:
                        return VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
                    case TextureAccess::TransferDestination:
                        return VK_IMAGE_USAGE_TRANSFER_DST_BIT;
                    case TextureAccess::PresentSource:
                        return 0;
                    case TextureAccess::ShadingRateAttachment:
                        return VK_IMAGE_USAGE_FRAGMENT_SHADING_RATE_ATTACHMENT_BIT_KHR;
                }
                return 0;
            }

            VkBufferUsageFlags buffer_access_usage(BufferAccess access) noexcept
            {
                switch (access)
                {
                    case BufferAccess::UniformRead:
                    case BufferAccess::UniformComputeRead:
                        return VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                    case BufferAccess::StorageRead:
                    case BufferAccess::StorageWrite:
                    case BufferAccess::StorageReadWrite:
                        return VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                    case BufferAccess::IndirectRead:
                        return VK_BUFFER_USAGE_INDIRECT_BUFFER_BIT;
                    case BufferAccess::VertexRead:
                        return VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
                    case BufferAccess::IndexRead:
                        return VK_BUFFER_USAGE_INDEX_BUFFER_BIT;
                    case BufferAccess::TransferSource:
                        return VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
                    case BufferAccess::TransferDestination:
                        return VK_BUFFER_USAGE_TRANSFER_DST_BIT;
                    case BufferAccess::HostRead:
                        return 0;
                }
                return 0;
            }
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
