/**************************************************************************/
/* render_graph.cpp                                                       */
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

#include "graph/render_graph.hpp"

#include <algorithm>
#include <utility>

#include "graph/gpu_profiler.hpp"
#include "resources/transient_pool.hpp"
#include "rhi/vulkan/vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Graph
        {
            namespace
            {
                /** @brief Every access bit that modifies a resource's contents. */
                constexpr VkAccessFlags2 WRITE_ACCESS_BITS =
                    VK_ACCESS_2_SHADER_WRITE_BIT | VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT |
                    VK_ACCESS_2_COLOR_ATTACHMENT_WRITE_BIT |
                    VK_ACCESS_2_DEPTH_STENCIL_ATTACHMENT_WRITE_BIT |
                    VK_ACCESS_2_TRANSFER_WRITE_BIT | VK_ACCESS_2_HOST_WRITE_BIT |
                    VK_ACCESS_2_MEMORY_WRITE_BIT;

                /**
                 * @brief Whether an access mask contains any write.
                 * @param access The mask to test.
                 * @return true when a following access must be ordered after it.
                 */
                bool writes(VkAccessFlags2 access) noexcept
                {
                    return (access & WRITE_ACCESS_BITS) != 0;
                }

                /**
                 * @brief Translates a graph load action into the Vulkan attachment load op.
                 * @param load The declared action.
                 * @return The matching VkAttachmentLoadOp.
                 */
                VkAttachmentLoadOp load_op(AttachmentLoad load) noexcept
                {
                    switch (load)
                    {
                        case AttachmentLoad::Clear:
                            return VK_ATTACHMENT_LOAD_OP_CLEAR;
                        case AttachmentLoad::Load:
                            return VK_ATTACHMENT_LOAD_OP_LOAD;
                        case AttachmentLoad::Discard:
                        default:
                            return VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                    }
                }
            } // namespace

            VkImage PassContext::image(TextureHandle handle) const
            {
                return graph_.texture_resources_[handle.index].image;
            }

            VkImageView PassContext::view(TextureHandle handle) const
            {
                return graph_.texture_resources_[handle.index].view;
            }

            VkImageView PassContext::sampled_view(TextureHandle handle) const
            {
                return graph_.texture_resources_[handle.index].sample_view;
            }

            const TextureDesc& PassContext::texture_desc(TextureHandle handle) const
            {
                return graph_.texture_resources_[handle.index].desc;
            }

            VkBuffer PassContext::buffer(BufferHandle handle) const
            {
                return graph_.buffer_resources_[handle.index].buffer;
            }

            void* PassContext::mapped(BufferHandle handle) const
            {
                return graph_.buffer_resources_[handle.index].mapped;
            }

            void RenderPassBuilder::read(TextureHandle handle, TextureAccess access)
            {
                graph_.declare_texture(pass_, handle, access, false);
            }

            void RenderPassBuilder::write(TextureHandle handle, TextureAccess access)
            {
                graph_.declare_texture(pass_, handle, access, true);
            }

            void RenderPassBuilder::read(BufferHandle handle, BufferAccess access)
            {
                graph_.declare_buffer(pass_, handle, access, false);
            }

            void RenderPassBuilder::write(BufferHandle handle, BufferAccess access)
            {
                graph_.declare_buffer(pass_, handle, access, true);
            }

            void RenderPassBuilder::color_attachment(std::uint32_t index, TextureHandle handle,
                                                     AttachmentLoad load, const ClearColor& clear)
            {
                if (index >= MAX_COLOR_ATTACHMENTS || !handle.valid())
                    return;
                RenderGraph::PassNode& node = graph_.passes_[pass_];
                node.color[index].handle = handle;
                node.color[index].load = load;
                node.color[index].clear = clear;
                node.color[index].bound = true;
                node.color_count = std::max(node.color_count, index + 1);
                graph_.declare_texture(pass_, handle, TextureAccess::ColorAttachment, true);
            }

            void RenderPassBuilder::depth_stencil_attachment(TextureHandle handle,
                                                             AttachmentLoad load, float depth,
                                                             std::uint32_t stencil, bool read_only)
            {
                if (!handle.valid())
                    return;
                RenderGraph::PassNode& node = graph_.passes_[pass_];
                node.depth.handle = handle;
                node.depth.load = load;
                node.depth.depth = depth;
                node.depth.stencil = stencil;
                node.depth.read_only = read_only;
                node.depth.bound = true;
                graph_.declare_texture(pass_, handle,
                                       read_only ? TextureAccess::DepthStencilReadOnly
                                                 : TextureAccess::DepthStencilAttachment,
                                       !read_only);
            }

            void RenderPassBuilder::shading_rate_attachment(TextureHandle handle,
                                                            std::uint32_t texel_width,
                                                            std::uint32_t texel_height)
            {
                if (!handle.valid() || texel_width == 0 || texel_height == 0)
                    return;
                RenderGraph::PassNode& node = graph_.passes_[pass_];
                node.shading_rate.handle = handle;
                node.shading_rate.texel_width = texel_width;
                node.shading_rate.texel_height = texel_height;
                node.shading_rate.bound = true;
                // A read, not a write: the rate image only steers this pass, so it must
                // not keep the pass that produced it alive on its own.
                graph_.declare_texture(pass_, handle, TextureAccess::ShadingRateAttachment,
                                       false);
            }

            void RenderPassBuilder::set_render_area(std::uint32_t width, std::uint32_t height)
            {
                RenderGraph::PassNode& node = graph_.passes_[pass_];
                node.render_area = {width, height};
                node.has_render_area = true;
            }

            void RenderPassBuilder::set_side_effect()
            {
                graph_.passes_[pass_].side_effect = true;
            }

            RenderGraph::RenderGraph(Vulkan::VulkanDevice& device, GpuProfiler* profiler)
                : device_(device), profiler_(profiler)
            {
            }

            void RenderGraph::begin_frame(Resources::TexturePool& textures,
                                          Resources::BufferPool& buffers)
            {
                textures_ = &textures;
                buffers_ = &buffers;
                passes_.clear();
                texture_resources_.clear();
                buffer_resources_.clear();
                order_.clear();
            }

            TextureHandle RenderGraph::create_texture(const TextureDesc& desc)
            {
                TextureResource resource;
                resource.desc = desc;
                texture_resources_.push_back(std::move(resource));
                TextureHandle handle;
                handle.index = static_cast<std::uint32_t>(texture_resources_.size() - 1);
                return handle;
            }

            BufferHandle RenderGraph::create_buffer(const BufferDesc& desc)
            {
                BufferResource resource;
                resource.desc = desc;
                buffer_resources_.push_back(std::move(resource));
                BufferHandle handle;
                handle.index = static_cast<std::uint32_t>(buffer_resources_.size() - 1);
                return handle;
            }

            TextureHandle RenderGraph::import_texture(const ImportedTexture& imported)
            {
                TextureResource resource;
                resource.desc = imported.desc;
                resource.imported = true;
                resource.image = imported.image;
                resource.view = imported.view;
                resource.sample_view = imported.sample_view != VK_NULL_HANDLE ? imported.sample_view
                                                                              : imported.view;
                resource.external_state = imported.state;
                // An imported resource outlives the frame, so it is never a candidate for
                // culling: the extra reader keeps its producer alive whatever the graph
                // can observe about it.
                resource.readers = 1;
                texture_resources_.push_back(std::move(resource));
                TextureHandle handle;
                handle.index = static_cast<std::uint32_t>(texture_resources_.size() - 1);
                return handle;
            }

            BufferHandle RenderGraph::import_buffer(const ImportedBuffer& imported)
            {
                BufferResource resource;
                resource.desc = imported.desc;
                resource.imported = true;
                resource.buffer = imported.buffer;
                resource.mapped = imported.mapped;
                resource.external_state = imported.state;
                resource.readers = 1;
                buffer_resources_.push_back(std::move(resource));
                BufferHandle handle;
                handle.index = static_cast<std::uint32_t>(buffer_resources_.size() - 1);
                return handle;
            }

            void RenderGraph::add_pass(const char* name,
                                       const std::function<void(RenderPassBuilder&)>& setup,
                                       ExecuteFunction execute)
            {
                PassNode node;
                node.name = name != nullptr ? name : "pass";
                node.execute = std::move(execute);
                passes_.push_back(std::move(node));

                const std::uint32_t index = static_cast<std::uint32_t>(passes_.size() - 1);
                RenderPassBuilder builder(*this, index);
                if (setup)
                    setup(builder);
            }

            void RenderGraph::declare_texture(std::uint32_t pass, TextureHandle handle,
                                              TextureAccess access, bool is_write)
            {
                if (!handle.valid() || handle.index >= texture_resources_.size())
                    return;
                TextureResource& resource = texture_resources_[handle.index];
                if (!resource.imported)
                    resource.desc.usage |= texture_access_usage(access);

                PassNode& node = passes_[pass];
                std::vector<TextureUse>& list = is_write ? node.texture_writes : node.texture_reads;
                bool already_referenced = false;
                for (const TextureUse& use : list)
                {
                    if (use.handle.index != handle.index)
                        continue;
                    if (use.access == access)
                        return;
                    already_referenced = true;
                }
                list.push_back(TextureUse{handle, access});
                if (already_referenced)
                    return;
                if (is_write)
                    resource.producers.push_back(pass);
                else
                    ++resource.readers;
            }

            void RenderGraph::declare_buffer(std::uint32_t pass, BufferHandle handle,
                                             BufferAccess access, bool is_write)
            {
                if (!handle.valid() || handle.index >= buffer_resources_.size())
                    return;
                BufferResource& resource = buffer_resources_[handle.index];
                if (!resource.imported)
                    resource.desc.usage |= buffer_access_usage(access);

                PassNode& node = passes_[pass];
                std::vector<BufferUse>& list = is_write ? node.buffer_writes : node.buffer_reads;
                bool already_referenced = false;
                for (const BufferUse& use : list)
                {
                    if (use.handle.index != handle.index)
                        continue;
                    if (use.access == access)
                        return;
                    already_referenced = true;
                }
                list.push_back(BufferUse{handle, access});
                if (already_referenced)
                    return;
                if (is_write)
                    resource.producers.push_back(pass);
                else
                    ++resource.readers;
            }

            void RenderGraph::cull_passes()
            {
                // A pass stays alive while any resource it writes is still read by
                // something. Its live-write count is the number of distinct resources it
                // produces; when that reaches zero and it has no declared side effect,
                // the pass contributes nothing and its own reads stop counting — which
                // may in turn strand the passes that produced them.
                for (PassNode& node : passes_)
                {
                    node.culled = false;
                    node.live_writes = 0;
                }
                for (const TextureResource& resource : texture_resources_)
                    for (std::uint32_t producer : resource.producers)
                        ++passes_[producer].live_writes;
                for (const BufferResource& resource : buffer_resources_)
                    for (std::uint32_t producer : resource.producers)
                        ++passes_[producer].live_writes;

                std::vector<std::uint32_t> unread_textures;
                std::vector<std::uint32_t> unread_buffers;
                for (std::uint32_t i = 0; i < texture_resources_.size(); ++i)
                    if (texture_resources_[i].readers == 0)
                        unread_textures.push_back(i);
                for (std::uint32_t i = 0; i < buffer_resources_.size(); ++i)
                    if (buffer_resources_[i].readers == 0)
                        unread_buffers.push_back(i);

                while (!unread_textures.empty() || !unread_buffers.empty())
                {
                    std::vector<std::uint32_t> stranded;
                    if (!unread_textures.empty())
                    {
                        const std::uint32_t index = unread_textures.back();
                        unread_textures.pop_back();
                        stranded = texture_resources_[index].producers;
                    }
                    else
                    {
                        const std::uint32_t index = unread_buffers.back();
                        unread_buffers.pop_back();
                        stranded = buffer_resources_[index].producers;
                    }

                    for (std::uint32_t producer : stranded)
                    {
                        PassNode& node = passes_[producer];
                        if (node.side_effect || node.culled || node.live_writes == 0)
                            continue;
                        if (--node.live_writes != 0)
                            continue;
                        node.culled = true;
                        for (const TextureUse& use : node.texture_reads)
                        {
                            TextureResource& resource = texture_resources_[use.handle.index];
                            if (resource.readers > 0 && --resource.readers == 0)
                                unread_textures.push_back(use.handle.index);
                        }
                        for (const BufferUse& use : node.buffer_reads)
                        {
                            BufferResource& resource = buffer_resources_[use.handle.index];
                            if (resource.readers > 0 && --resource.readers == 0)
                                unread_buffers.push_back(use.handle.index);
                        }
                    }
                }
            }

            void RenderGraph::touch(std::uint32_t pass, TextureResource& resource)
            {
                resource.first_pass = std::min(resource.first_pass, pass);
                resource.last_pass = std::max(resource.last_pass, pass);
            }

            void RenderGraph::touch(std::uint32_t pass, BufferResource& resource)
            {
                resource.first_pass = std::min(resource.first_pass, pass);
                resource.last_pass = std::max(resource.last_pass, pass);
            }

            void RenderGraph::assign_resources()
            {
                for (std::uint32_t position = 0; position < order_.size(); ++position)
                {
                    const PassNode& node = passes_[order_[position]];
                    for (const TextureUse& use : node.texture_reads)
                        touch(position, texture_resources_[use.handle.index]);
                    for (const TextureUse& use : node.texture_writes)
                        touch(position, texture_resources_[use.handle.index]);
                    for (const BufferUse& use : node.buffer_reads)
                        touch(position, buffer_resources_[use.handle.index]);
                    for (const BufferUse& use : node.buffer_writes)
                        touch(position, buffer_resources_[use.handle.index]);
                }

                // Walking the schedule in order and returning each transient the moment
                // its last reader has been scheduled is what makes two disjoint lifetimes
                // land on one allocation: the release below puts the entry back before the
                // next position's acquire looks for a match.
                for (std::uint32_t position = 0; position < order_.size(); ++position)
                {
                    for (TextureResource& resource : texture_resources_)
                    {
                        if (resource.imported || resource.first_pass != position)
                            continue;
                        resource.entry = textures_->acquire(resource.desc);
                        const Resources::TexturePool::Entry& entry = textures_->entry(resource.entry);
                        resource.image = entry.image;
                        resource.view = entry.view;
                        resource.sample_view = entry.sample_view;
                    }
                    for (BufferResource& resource : buffer_resources_)
                    {
                        if (resource.imported || resource.first_pass != position)
                            continue;
                        resource.entry = buffers_->acquire(resource.desc);
                        const Resources::BufferPool::Entry& entry = buffers_->entry(resource.entry);
                        resource.buffer = entry.buffer;
                        resource.mapped = entry.mapped;
                    }

                    for (TextureResource& resource : texture_resources_)
                        if (!resource.imported && resource.entry != INVALID_RESOURCE &&
                            resource.last_pass == position)
                            textures_->release(resource.entry);
                    for (BufferResource& resource : buffer_resources_)
                        if (!resource.imported && resource.entry != INVALID_RESOURCE &&
                            resource.last_pass == position)
                            buffers_->release(resource.entry);
                }
            }

            void RenderGraph::compile()
            {
                cull_passes();
                order_.clear();
                order_.reserve(passes_.size());
                for (std::uint32_t i = 0; i < passes_.size(); ++i)
                    if (!passes_[i].culled)
                        order_.push_back(i);
                assign_resources();
            }

            TextureState& RenderGraph::texture_state(const TextureResource& resource)
            {
                if (resource.imported)
                    return *resource.external_state;
                return textures_->entry(resource.entry).state;
            }

            BufferState& RenderGraph::buffer_state(const BufferResource& resource)
            {
                if (resource.imported)
                    return *resource.external_state;
                return buffers_->entry(resource.entry).state;
            }

            void RenderGraph::emit_barriers(VkCommandBuffer cmd, const PassNode& node)
            {
                std::vector<VkImageMemoryBarrier2> image_barriers;
                std::vector<VkBufferMemoryBarrier2> buffer_barriers;

                const auto visit_texture = [&](const TextureUse& use)
                {
                    TextureResource& resource = texture_resources_[use.handle.index];
                    TextureState& current = texture_state(resource);
                    const TextureState wanted = texture_access_state(use.access);
                    const bool hazard = current.layout != wanted.layout || writes(current.access) ||
                                        writes(wanted.access);
                    if (!hazard)
                    {
                        // Read after read in the same layout needs no barrier, but the
                        // accumulated stages must be kept: the next writer has to wait on
                        // every reader, not just the most recent one.
                        current.stage |= wanted.stage;
                        current.access |= wanted.access;
                        return;
                    }

                    VkImageMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = current.stage != VK_PIPELINE_STAGE_2_NONE
                                               ? current.stage
                                               : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                    barrier.srcAccessMask = current.access;
                    barrier.dstStageMask = wanted.stage;
                    barrier.dstAccessMask = wanted.access;
                    barrier.oldLayout = current.layout;
                    barrier.newLayout = wanted.layout;
                    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.image = resource.image;
                    barrier.subresourceRange.aspectMask = resource.desc.aspect;
                    barrier.subresourceRange.levelCount = resource.desc.mip_levels;
                    barrier.subresourceRange.layerCount = resource.desc.array_layers;
                    image_barriers.push_back(barrier);
                    current = wanted;
                };

                const auto visit_buffer = [&](const BufferUse& use)
                {
                    BufferResource& resource = buffer_resources_[use.handle.index];
                    BufferState& current = buffer_state(resource);
                    const BufferState wanted = buffer_access_state(use.access);
                    const bool hazard = writes(current.access) || writes(wanted.access);
                    if (!hazard)
                    {
                        current.stage |= wanted.stage;
                        current.access |= wanted.access;
                        return;
                    }

                    VkBufferMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER_2;
                    barrier.srcStageMask = current.stage != VK_PIPELINE_STAGE_2_NONE
                                               ? current.stage
                                               : VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                    barrier.srcAccessMask = current.access;
                    barrier.dstStageMask = wanted.stage;
                    barrier.dstAccessMask = wanted.access;
                    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.buffer = resource.buffer;
                    barrier.offset = 0;
                    barrier.size = VK_WHOLE_SIZE;
                    buffer_barriers.push_back(barrier);
                    current = wanted;
                };

                for (const TextureUse& use : node.texture_reads)
                    visit_texture(use);
                for (const TextureUse& use : node.texture_writes)
                    visit_texture(use);
                for (const BufferUse& use : node.buffer_reads)
                    visit_buffer(use);
                for (const BufferUse& use : node.buffer_writes)
                    visit_buffer(use);

                if (image_barriers.empty() && buffer_barriers.empty())
                    return;

                VkDependencyInfo dependency{};
                dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                dependency.imageMemoryBarrierCount =
                    static_cast<std::uint32_t>(image_barriers.size());
                dependency.pImageMemoryBarriers = image_barriers.data();
                dependency.bufferMemoryBarrierCount =
                    static_cast<std::uint32_t>(buffer_barriers.size());
                dependency.pBufferMemoryBarriers = buffer_barriers.data();
                vkCmdPipelineBarrier2(cmd, &dependency);
            }

            VkExtent2D RenderGraph::resolve_render_area(const PassNode& node) const
            {
                if (node.has_render_area)
                    return node.render_area;
                for (std::uint32_t i = 0; i < node.color_count; ++i)
                {
                    if (!node.color[i].bound)
                        continue;
                    const TextureDesc& desc =
                        texture_resources_[node.color[i].handle.index].desc;
                    return VkExtent2D{desc.width, desc.height};
                }
                if (node.depth.bound)
                {
                    const TextureDesc& desc = texture_resources_[node.depth.handle.index].desc;
                    return VkExtent2D{desc.width, desc.height};
                }
                return VkExtent2D{0, 0};
            }

            void RenderGraph::begin_rendering(VkCommandBuffer cmd, const PassNode& node,
                                              VkExtent2D area)
            {
                VkRenderingAttachmentInfo color[MAX_COLOR_ATTACHMENTS]{};
                for (std::uint32_t i = 0; i < node.color_count; ++i)
                {
                    const ColorAttachment& attachment = node.color[i];
                    color[i].sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                    color[i].imageLayout = VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL;
                    color[i].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
                    if (!attachment.bound)
                    {
                        color[i].imageView = VK_NULL_HANDLE;
                        color[i].loadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
                        continue;
                    }
                    color[i].imageView = texture_resources_[attachment.handle.index].view;
                    color[i].loadOp = load_op(attachment.load);
                    if (attachment.clear.integer)
                        for (int channel = 0; channel < 4; ++channel)
                            color[i].clearValue.color.uint32[channel] =
                                attachment.clear.uint32[channel];
                    else
                        for (int channel = 0; channel < 4; ++channel)
                            color[i].clearValue.color.float32[channel] =
                                attachment.clear.float32[channel];
                }

                VkRenderingAttachmentInfo depth{};
                if (node.depth.bound)
                {
                    depth.sType = VK_STRUCTURE_TYPE_RENDERING_ATTACHMENT_INFO;
                    depth.imageView = texture_resources_[node.depth.handle.index].view;
                    depth.imageLayout = node.depth.read_only
                                            ? VK_IMAGE_LAYOUT_DEPTH_STENCIL_READ_ONLY_OPTIMAL
                                            : VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL;
                    depth.loadOp = load_op(node.depth.load);
                    depth.storeOp = node.depth.read_only ? VK_ATTACHMENT_STORE_OP_NONE
                                                         : VK_ATTACHMENT_STORE_OP_STORE;
                    depth.clearValue.depthStencil = {node.depth.depth, node.depth.stencil};
                }

                const bool has_stencil =
                    node.depth.bound &&
                    (texture_resources_[node.depth.handle.index].desc.aspect &
                     VK_IMAGE_ASPECT_STENCIL_BIT) != 0;

                VkRenderingFragmentShadingRateAttachmentInfoKHR shading_rate{};
                if (node.shading_rate.bound)
                {
                    shading_rate.sType =
                        VK_STRUCTURE_TYPE_RENDERING_FRAGMENT_SHADING_RATE_ATTACHMENT_INFO_KHR;
                    shading_rate.imageView =
                        texture_resources_[node.shading_rate.handle.index].view;
                    shading_rate.imageLayout =
                        VK_IMAGE_LAYOUT_FRAGMENT_SHADING_RATE_ATTACHMENT_OPTIMAL_KHR;
                    shading_rate.shadingRateAttachmentTexelSize = {
                        node.shading_rate.texel_width, node.shading_rate.texel_height};
                }

                VkRenderingInfo rendering{};
                rendering.sType = VK_STRUCTURE_TYPE_RENDERING_INFO;
                rendering.pNext = node.shading_rate.bound ? &shading_rate : nullptr;
                rendering.renderArea.extent = area;
                rendering.layerCount = 1;
                rendering.colorAttachmentCount = node.color_count;
                rendering.pColorAttachments = node.color_count > 0 ? color : nullptr;
                rendering.pDepthAttachment = node.depth.bound ? &depth : nullptr;
                rendering.pStencilAttachment = has_stencil ? &depth : nullptr;
                vkCmdBeginRendering(cmd, &rendering);

                // Viewport and scissor follow the render area, so a pass that draws at a
                // reduced resolution only declares its target's size.
                VkViewport viewport{};
                viewport.width = static_cast<float>(area.width);
                viewport.height = static_cast<float>(area.height);
                viewport.maxDepth = 1.0f;
                VkRect2D scissor{};
                scissor.extent = area;
                vkCmdSetViewport(cmd, 0, 1, &viewport);
                vkCmdSetScissor(cmd, 0, 1, &scissor);
            }

            void RenderGraph::execute(VkCommandBuffer cmd)
            {
                for (std::uint32_t index : order_)
                {
                    PassNode& node = passes_[index];
                    const std::uint32_t timer =
                        profiler_ != nullptr ? profiler_->begin_pass(cmd, node.name.c_str())
                                             : GpuProfiler::INVALID_TIMER;

                    emit_barriers(cmd, node);

                    const bool renders = node.color_count > 0 || node.depth.bound;
                    const VkExtent2D area = resolve_render_area(node);
                    if (renders)
                        begin_rendering(cmd, node, area);

                    if (node.execute)
                    {
                        const PassContext context(*this, area);
                        node.execute(cmd, context);
                    }

                    if (renders)
                        vkCmdEndRendering(cmd);

                    if (profiler_ != nullptr)
                        profiler_->end_pass(cmd, timer);
                }
            }
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
