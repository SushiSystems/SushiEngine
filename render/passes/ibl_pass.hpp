/**************************************************************************/
/* ibl_pass.hpp                                                           */
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

#pragma once

/**
 * @file ibl_pass.hpp
 * @brief Image-based lighting captured from the engine's own analytic sky.
 *
 * The environment a surface reflects is not an imported HDRI but the atmosphere the
 * sky pass is already drawing: six 90-degree views of it are rendered into a cubemap
 * with the same sky shader, then prefiltered into a GGX mip chain and cosine-
 * convolved into an irradiance cube. That is what replaces the flat ambient constant,
 * and it means the lighting tracks the time of day for free.
 *
 * The chain is rebuilt only when the sun or the atmosphere has actually moved, and
 * never more than once every few frames, so a slowly turning sun costs almost
 * nothing. The cube images are private to this pass — the render graph schedules the
 * pass, and the pass keeps its own subresources in order internally, exactly as the
 * cloud noise build does.
 */

#include "passes/render_pass.hpp"

#include <cstdint>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;
            class SamplerCache;
            class ShaderLibrary;
        }

        namespace Scene
        {
            class SceneLayout;
        }

        namespace Textures
        {
            class CloudNoise;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief Builds and owns the prefiltered environment, irradiance, and BRDF LUT.
             *
             * Non-copyable: it owns images, views, and pipelines.
             */
            class IblPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Creates the cube chain and generates the view-independent LUT.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipelines are built through.
                     * @param samplers  Cache providing the cube and LUT samplers.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     * @param noise     Cloud noise the captured sky shader samples.
                     */
                    IblPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                            Resources::GraphicsPipelineFactory& pipelines,
                            Resources::SamplerCache& samplers, Scene::SceneLayout& layout,
                            Textures::CloudNoise& noise);
                    ~IblPass() override;

                    IblPass(const IblPass&) = delete;
                    IblPass& operator=(const IblPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                    /** @brief The cosine-convolved irradiance cube, for indirect diffuse. */
                    VkImageView irradiance() const noexcept { return irradiance_.cube_view; }

                    /** @brief The GGX-prefiltered environment, roughness across its mips. */
                    VkImageView specular() const noexcept { return specular_.cube_view; }

                    /** @brief The split-sum BRDF integration LUT. */
                    VkImageView brdf_lut() const noexcept { return brdf_view_; }

                    /** @brief The sampler the cubes and the LUT are read through. */
                    VkSampler sampler() const noexcept { return sampler_; }

                    /** @brief Mip levels in the specular cube, the roughness axis's length. */
                    std::uint32_t specular_mip_count() const noexcept { return specular_.mips; }

                    /** @brief The 9 diffuse SH coefficients projected from the environment. */
                    VkBuffer sh_buffer() const noexcept { return sh_buffer_; }

                    /** @brief Bytes of the SH coefficient buffer, for the descriptor range. */
                    static constexpr VkDeviceSize sh_buffer_bytes() noexcept
                    {
                        return 9 * 4 * sizeof(float);
                    }

                private:
                    /** @brief One cubemap: its image, whole-cube view, and per-mip storage views. */
                    struct Cube
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView cube_view = VK_NULL_HANDLE;
                        VkImageView face_views[6]{};   /**< Single-layer views, for capture. */
                        VkImageView mip_views[8]{};    /**< Whole-cube views of one mip, for compute. */
                        std::uint32_t resolution = 0;
                        std::uint32_t mips = 1;
                    };

                    void create_cube(Cube& cube, std::uint32_t resolution, std::uint32_t mips,
                                     VkImageUsageFlags usage, bool face_views);
                    void destroy_cube(Cube& cube);
                    void create_brdf_lut();
                    void create_pipelines();
                    void destroy_pipelines();
                    void generate_brdf_lut();
                    bool environment_changed(const Frame::FrameContext& frame);
                    void record_update(VkCommandBuffer cmd, const Frame::FrameContext& frame,
                                       const Graph::PassContext& context);

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;
                    Textures::CloudNoise& noise_;

                    Cube environment_;
                    Cube specular_;
                    Cube irradiance_;
                    VkImage brdf_ = VK_NULL_HANDLE;
                    VmaAllocation brdf_allocation_ = VK_NULL_HANDLE;
                    VkImageView brdf_view_ = VK_NULL_HANDLE;
                    VkImage dummy_depth_ = VK_NULL_HANDLE;
                    VmaAllocation dummy_depth_allocation_ = VK_NULL_HANDLE;
                    VkImageView dummy_depth_view_ = VK_NULL_HANDLE;
                    VkSampler sampler_ = VK_NULL_HANDLE;

                    VkDescriptorSetLayout compute_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout compute_pipeline_layout_ = VK_NULL_HANDLE;
                    VkDescriptorSetLayout lut_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout lut_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline sky_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline prefilter_pipeline_ = VK_NULL_HANDLE;
                    VkPipeline irradiance_pipeline_ = VK_NULL_HANDLE;

                    // Diffuse SH projection: its own layout (the destination is a storage
                    // buffer, not the storage image the cube convolutions write), pipeline,
                    // and the tiny 9-coefficient buffer the shading pass reads.
                    VkDescriptorSetLayout sh_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout sh_pipeline_layout_ = VK_NULL_HANDLE;
                    VkPipeline sh_pipeline_ = VK_NULL_HANDLE;
                    VkBuffer sh_buffer_ = VK_NULL_HANDLE;
                    VmaAllocation sh_allocation_ = VK_NULL_HANDLE;

                    VkBuffer face_uniforms_[6]{};
                    VmaAllocation face_uniform_allocations_[6]{};
                    void* face_uniform_mapped_[6]{};

                    float last_sun_[3] = {0.0f, 0.0f, 0.0f};
                    float last_intensity_ = -1.0f;
                    float last_altitude_ = -1.0f;
                    std::uint32_t last_capture_frame_ = 0;
                    bool captured_ = false;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
