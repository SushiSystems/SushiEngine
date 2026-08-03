/**************************************************************************/
/* weather_field_pass.cpp                                                 */
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

#include "passes/weather_field_pass.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

#include <SushiEngine/environment/environment.hpp>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "resources/sampler_cache.hpp"
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
                constexpr VkFormat FIELD_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;
                constexpr std::uint32_t FIELD_WIDTH = std::uint32_t(WEATHER_FIELD_MAX_CELLS);
                constexpr std::uint32_t FIELD_DEPTH = std::uint32_t(WEATHER_FIELD_LEVELS);
                constexpr VkDeviceSize FIELD_BYTES =
                    VkDeviceSize(FIELD_WIDTH) * FIELD_WIDTH * FIELD_DEPTH * 4;

                // density_scale is authored in [0, 2] (Render::CloudDeck's own range); the field
                // texel is UNORM, so it rides at half scale and the shader doubles it back.
                constexpr float DENSITY_ENCODE_SCALE = 0.5f;

                std::uint8_t encode(float value) noexcept
                {
                    return static_cast<std::uint8_t>(std::lround(std::clamp(value, 0.0f, 1.0f) * 255.0f));
                }

                void transition(VkCommandBuffer command, VkImage image, VkImageLayout from,
                                VkImageLayout to, VkPipelineStageFlags2 source,
                                VkPipelineStageFlags2 destination, VkAccessFlags2 source_access,
                                VkAccessFlags2 destination_access)
                {
                    VkImageMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = source;
                    barrier.srcAccessMask = source_access;
                    barrier.dstStageMask = destination;
                    barrier.dstAccessMask = destination_access;
                    barrier.oldLayout = from;
                    barrier.newLayout = to;
                    barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
                    barrier.image = image;
                    barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                    barrier.subresourceRange.levelCount = 1;
                    barrier.subresourceRange.layerCount = 1;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.imageMemoryBarrierCount = 1;
                    dependency.pImageMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(command, &dependency);
                }

                // The published field carries whatever resolution its producer had; the image is
                // always the full square (see the header). Resampling bilinearly rather than by
                // nearest matters for the coarse producers — a 32-cell station blend expanded by
                // nearest would read as 32 visible steps in the sky, which is exactly the kind of
                // structure-from-the-renderer this whole change exists to remove.
                WeatherFieldSample sample_source(const WeatherField& field, int level, float u, float v)
                {
                    const int level_index = std::min(level, field.level_count - 1);
                    const float sx = u * float(field.cells_x) - 0.5f;
                    const float sz = v * float(field.cells_z) - 0.5f;
                    const int x0 = std::clamp(int(std::floor(sx)), 0, field.cells_x - 1);
                    const int z0 = std::clamp(int(std::floor(sz)), 0, field.cells_z - 1);
                    const int x1 = std::clamp(x0 + 1, 0, field.cells_x - 1);
                    const int z1 = std::clamp(z0 + 1, 0, field.cells_z - 1);
                    const float tx = std::clamp(sx - float(x0), 0.0f, 1.0f);
                    const float tz = std::clamp(sz - float(z0), 0.0f, 1.0f);

                    const auto at = [&field, level_index](int x, int z) -> const WeatherFieldSample&
                    {
                        const std::size_t index =
                            (std::size_t(level_index) * std::size_t(field.cells_z) + std::size_t(z)) *
                                std::size_t(field.cells_x) + std::size_t(x);
                        return field.samples[index];
                    };
                    const auto lerp = [](float a, float b, float t) { return a + (b - a) * t; };

                    const WeatherFieldSample& s00 = at(x0, z0);
                    const WeatherFieldSample& s10 = at(x1, z0);
                    const WeatherFieldSample& s01 = at(x0, z1);
                    const WeatherFieldSample& s11 = at(x1, z1);

                    WeatherFieldSample out;
                    out.coverage = lerp(lerp(s00.coverage, s10.coverage, tx),
                                        lerp(s01.coverage, s11.coverage, tx), tz);
                    out.density_scale = lerp(lerp(s00.density_scale, s10.density_scale, tx),
                                             lerp(s01.density_scale, s11.density_scale, tx), tz);
                    out.convective_fraction =
                        lerp(lerp(s00.convective_fraction, s10.convective_fraction, tx),
                             lerp(s01.convective_fraction, s11.convective_fraction, tx), tz);
                    out.precipitation = lerp(lerp(s00.precipitation, s10.precipitation, tx),
                                             lerp(s01.precipitation, s11.precipitation, tx), tz);
                    return out;
                }
            } // namespace

            WeatherFieldPass::WeatherFieldPass(Vulkan::VulkanDevice& device,
                                               Resources::SamplerCache& samplers)
                : device_(device)
            {
                create_image();
                create_staging();

                // Edge-clamped, not wrapped: the field describes a bounded region of the world,
                // and a march ray leaving it must keep reading the nearest simulated cell rather
                // than teleporting to the far side of the domain.
                Resources::SamplerDescription sampler_description{};
                sampler_description.filter = VK_FILTER_LINEAR;
                sampler_description.address_mode = VK_SAMPLER_ADDRESS_MODE_CLAMP_TO_EDGE;
                sampler_ = samplers.get(sampler_description);
            }

            WeatherFieldPass::~WeatherFieldPass()
            {
                destroy_staging();
                destroy_image();
            }

            void WeatherFieldPass::create_image()
            {
                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_3D;
                image_info.format = FIELD_FORMAT;
                image_info.extent = {FIELD_WIDTH, FIELD_WIDTH, FIELD_DEPTH};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;

                VmaAllocationCreateInfo alloc{};
                alloc.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &alloc, &image_,
                                             &allocation_, nullptr),
                              "vmaCreateImage(weather field)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = image_;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_3D;
                view_info.format = FIELD_FORMAT;
                view_info.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                view_info.subresourceRange.levelCount = 1;
                view_info.subresourceRange.layerCount = 1;
                Vulkan::check(vkCreateImageView(device_.device(), &view_info, nullptr, &view_),
                              "vkCreateImageView(weather field)");
            }

            void WeatherFieldPass::create_staging()
            {
                for (std::uint32_t i = 0; i < SLOTS; ++i)
                {
                    VkBufferCreateInfo buffer_info{};
                    buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                    buffer_info.size = FIELD_BYTES;
                    buffer_info.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;

                    VmaAllocationCreateInfo alloc{};
                    alloc.usage = VMA_MEMORY_USAGE_AUTO;
                    alloc.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                  VMA_ALLOCATION_CREATE_MAPPED_BIT;

                    VmaAllocationInfo mapped{};
                    Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &alloc,
                                                  &staging_[i].buffer, &staging_[i].allocation, &mapped),
                                  "vmaCreateBuffer(weather field staging)");
                    staging_[i].mapped = mapped.pMappedData;
                }
            }

            void WeatherFieldPass::destroy_image()
            {
                if (view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), view_, nullptr);
                if (image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), image_, allocation_);
                view_ = VK_NULL_HANDLE;
                image_ = VK_NULL_HANDLE;
                allocation_ = VK_NULL_HANDLE;
            }

            void WeatherFieldPass::destroy_staging()
            {
                for (std::uint32_t i = 0; i < SLOTS; ++i)
                {
                    if (staging_[i].buffer != VK_NULL_HANDLE)
                        vmaDestroyBuffer(device_.allocator(), staging_[i].buffer, staging_[i].allocation);
                    staging_[i].buffer = VK_NULL_HANDLE;
                    staging_[i].allocation = VK_NULL_HANDLE;
                    staging_[i].mapped = nullptr;
                }
            }

            void WeatherFieldPass::register_pass(Graph::RenderGraph& graph,
                                                 const Frame::FrameContext& frame)
            {
                const bool has_field = frame.environment != nullptr &&
                                       frame.environment->weather_field.valid();
                const std::uint32_t revision = has_field ? frame.environment->weather_field.revision : 0u;

                // Nothing to do once the image already holds this revision. The first frame still
                // runs, to clear the image: the cloud passes bind it unconditionally, so it must
                // be a defined, readable resource even in a scene that never publishes weather.
                const bool upload = has_field && revision != uploaded_revision_;
                if (!upload && cleared_)
                    return;

                const std::uint32_t slot = frame.slot % SLOTS;
                if (upload)
                {
                    const WeatherField& field = frame.environment->weather_field;
                    auto* texels = static_cast<std::uint8_t*>(staging_[slot].mapped);
                    for (std::uint32_t level = 0; level < FIELD_DEPTH; ++level)
                        for (std::uint32_t z = 0; z < FIELD_WIDTH; ++z)
                            for (std::uint32_t x = 0; x < FIELD_WIDTH; ++x)
                            {
                                const float u = (float(x) + 0.5f) / float(FIELD_WIDTH);
                                const float v = (float(z) + 0.5f) / float(FIELD_WIDTH);
                                const WeatherFieldSample s =
                                    sample_source(field, int(level), u, v);
                                const std::size_t offset =
                                    ((std::size_t(level) * FIELD_WIDTH + z) * FIELD_WIDTH + x) * 4;
                                texels[offset + 0] = encode(s.coverage);
                                texels[offset + 1] = encode(s.density_scale * DENSITY_ENCODE_SCALE);
                                texels[offset + 2] = encode(s.convective_fraction);
                                texels[offset + 3] = encode(s.precipitation);
                            }
                    uploaded_revision_ = revision;
                }

                const bool clear = !cleared_ && !upload;
                cleared_ = true;

                graph.add_pass(
                    "weather-field-upload",
                    [](Graph::RenderPassBuilder& builder)
                    {
                        // The image is pass-owned and barriered by hand, exactly as
                        // CloudscapeCompilePass's fields are; the graph only needs to be told
                        // this pass must not be culled for having no tracked write.
                        builder.set_side_effect();
                    },
                    [this, slot, clear](VkCommandBuffer command, const Graph::PassContext&)
                    {
                        // UNDEFINED as the source layout on every upload: the previous contents
                        // are fully overwritten, so there is nothing to preserve and the driver
                        // is free to discard them.
                        transition(command, image_, VK_IMAGE_LAYOUT_UNDEFINED,
                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                   VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT |
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_PIPELINE_STAGE_2_COPY_BIT, VK_ACCESS_2_NONE,
                                   VK_ACCESS_2_TRANSFER_WRITE_BIT);

                        if (clear)
                        {
                            // Zero coverage everywhere: a scene with no weather provider reads a
                            // field that says "no simulated cloud", and the shaders' own enable
                            // flag keeps them from consulting it at all.
                            const VkClearColorValue zero{};
                            VkImageSubresourceRange range{};
                            range.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                            range.levelCount = 1;
                            range.layerCount = 1;
                            vkCmdClearColorImage(command, image_,
                                                 VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, &zero, 1,
                                                 &range);
                        }
                        else
                        {
                            VkBufferImageCopy copy{};
                            copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                            copy.imageSubresource.layerCount = 1;
                            copy.imageExtent = {FIELD_WIDTH, FIELD_WIDTH, FIELD_DEPTH};
                            vkCmdCopyBufferToImage(command, staging_[slot].buffer, image_,
                                                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
                        }

                        transition(command, image_, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                                   VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
                                   VK_PIPELINE_STAGE_2_COPY_BIT,
                                   VK_PIPELINE_STAGE_2_FRAGMENT_SHADER_BIT |
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                   VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                   VK_ACCESS_2_SHADER_SAMPLED_READ_BIT);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
