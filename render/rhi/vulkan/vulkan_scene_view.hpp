/**************************************************************************/
/* vulkan_scene_view.hpp                                                  */
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
 * @file vulkan_scene_view.hpp
 * @brief Vulkan implementation of ISceneView: a render-graph frame orchestrator.
 *
 * Internal to the render library. The view holds only what genuinely varies per
 * view — its targets, its per-frame allocators, its soft-body buffers, its temporal
 * history, and its passes — and reaches everything shared through the device's
 * AssetLibrary. Its job each frame is one thing: build a FrameContext, let each pass
 * register into the graph, compile, submit. It records no barrier and opens no render
 * pass of its own.
 *
 * It also owns the frame's relationship to the frame before it: the jitter sequence,
 * the previous camera, the accumulated history, and the resolution the governor chose.
 * Those are per view because two viewports converge independently.
 */

#include <cstdint>
#include <memory>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "frame/frame_context.hpp"
#include "frame/resolution_controller.hpp"
#include "geometry/cloth_buffers.hpp"
#include "graph/gpu_profiler.hpp"
#include "graph/render_graph.hpp"
#include "material/material_system.hpp"
#include "passes/cloud_composite_pass.hpp"
#include "passes/cloud_pass.hpp"
#include "passes/contact_shadow_pass.hpp"
#include "passes/depth_prepass.hpp"
#include "passes/fxaa_pass.hpp"
#include "passes/ibl_pass.hpp"
#include "passes/opaque_pass.hpp"
#include "passes/picking_pass.hpp"
#include "passes/ray_traced_shadow_pass.hpp"
#include "passes/shading_rate_pass.hpp"
#include "passes/shadow_pass.hpp"
#include "passes/sky_pass.hpp"
#include "passes/taa_pass.hpp"
#include "passes/tonemap_pass.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/transient_pool.hpp"
#include "raytracing/scene_accelerator.hpp"
#include "scene/motion_system.hpp"
#include "scene/shadow_uniforms.hpp"

#include "vulkan_device.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Assets
        {
            class AssetLibrary;
        }

        namespace Vulkan
        {
            /**
             * @brief An offscreen scene renderer sampled into the editor viewport.
             *
             * Double-buffered so the frame being sampled by the UI is never the frame
             * being drawn. Non-copyable: it owns Vulkan and VMA handles.
             */
            class VulkanSceneView final : public ISceneView
            {
                public:
                    /**
                     * @brief Brings up the view's passes, allocators, and targets.
                     * @param device The live Vulkan device to render on.
                     * @param assets The device's shared asset store and services.
                     */
                    VulkanSceneView(VulkanDevice& device, Assets::AssetLibrary& assets);
                    ~VulkanSceneView() override;

                    VulkanSceneView(const VulkanSceneView&) = delete;
                    VulkanSceneView& operator=(const VulkanSceneView&) = delete;

                    void resize(std::uint32_t width, std::uint32_t height) override;
                    std::uint32_t width() const noexcept override { return width_; }
                    std::uint32_t height() const noexcept override { return height_; }
                    void set_settings(const RenderSettings& settings) override;
                    const RenderSettings& settings() const noexcept override { return settings_; }
                    void render_resolution(std::uint32_t& width,
                                           std::uint32_t& height) const noexcept override;
                    void render(const CameraView& camera, const Environment& environment,
                                const MeshInstance* instances, std::size_t count,
                                std::uint32_t selected_id,
                                const ClothStrandView* strands = nullptr,
                                std::size_t strand_count = 0) override;
                    std::uint32_t pick(std::uint32_t x, std::uint32_t y) override;
                    std::uint32_t slot_count() const noexcept override { return SLOTS; }
                    SceneViewTexture texture(std::uint32_t slot) const noexcept override;
                    std::uint32_t current_slot() const noexcept override { return current_slot_; }
                    std::size_t pass_timing_count() const noexcept override;
                    ScenePassTiming pass_timing(std::size_t index) const noexcept override;

                private:
                    /** @brief How many frames may be in flight at once. */
                    static constexpr std::uint32_t SLOTS = 2;

                    /**
                     * @brief One frame slot: its persistent targets and recording state.
                     *
                     * The transient pools are per slot rather than shared, because a pool
                     * hands a resource back out as soon as its frame ends — sharing one
                     * between slots would let this frame overwrite resources the previous
                     * frame's submit is still reading.
                     */
                    struct Slot
                    {
                        VkImage resolve = VK_NULL_HANDLE;
                        VmaAllocation resolve_allocation = VK_NULL_HANDLE;
                        VkImageView resolve_view = VK_NULL_HANDLE;
                        Graph::TextureState resolve_state{};
                        VkBuffer readback = VK_NULL_HANDLE;
                        VmaAllocation readback_allocation = VK_NULL_HANDLE;
                        void* readback_mapped = nullptr;
                        Graph::BufferState readback_state{};
                        VkBuffer uniforms = VK_NULL_HANDLE;
                        VmaAllocation uniforms_allocation = VK_NULL_HANDLE;
                        void* uniforms_mapped = nullptr;
                        Graph::BufferState uniforms_state{};
                        VkBuffer temporal = VK_NULL_HANDLE;
                        VmaAllocation temporal_allocation = VK_NULL_HANDLE;
                        void* temporal_mapped = nullptr;
                        Graph::BufferState temporal_state{};
                        VkBuffer shadow = VK_NULL_HANDLE;
                        VmaAllocation shadow_allocation = VK_NULL_HANDLE;
                        void* shadow_mapped = nullptr;
                        Graph::BufferState shadow_state{};
                        VkCommandPool pool = VK_NULL_HANDLE;
                        VkCommandBuffer cmd = VK_NULL_HANDLE;
                        VkFence fence = VK_NULL_HANDLE;
                        bool ever_rendered = false;
                        /** @brief Render extent the id target in this slot was written at. */
                        std::uint32_t readback_width = 0;
                        std::uint32_t readback_height = 0;
                        std::unique_ptr<Resources::TexturePool> textures;
                        std::unique_ptr<Resources::BufferPool> buffers;
                    };

                    /**
                     * @brief One accumulated frame the temporal resolve reads and writes.
                     *
                     * Two of them, ping-ponged by frame parity rather than by frame slot:
                     * the resolve needs the frame immediately before this one, and with
                     * two slots in flight the other slot's image is two frames old.
                     */
                    struct History
                    {
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        Graph::TextureState state{};
                    };

                    void create_slots();
                    void create_targets();
                    void destroy_targets();
                    Frame::FrameTargets declare_targets(Slot& slot, const Frame::FrameContext& frame);
                    void update_render_extent();
                    float measured_frame_milliseconds() const noexcept;

                    VulkanDevice& device_;
                    Assets::AssetLibrary& assets_;
                    Resources::DescriptorAllocator descriptors_;
                    Geometry::ClothBuffers cloth_;
                    Assets::MaterialSystem materials_;
                    Scene::MotionSystem motion_;
                    RayTracing::SceneAccelerator accelerator_;
                    Graph::GpuProfiler profiler_;
                    Graph::RenderGraph graph_;
                    Passes::IblPass ibl_pass_;
                    Passes::DepthPrepass depth_prepass_;
                    Passes::ShadowPass shadow_pass_;
                    Passes::ContactShadowPass contact_shadow_pass_;
                    Passes::RayTracedShadowPass ray_shadow_pass_;
                    Passes::OpaquePass opaque_pass_;
                    Passes::ShadingRatePass shading_rate_pass_;
                    Passes::SkyPass sky_pass_;
                    Passes::CloudPass cloud_pass_;
                    Passes::CloudCompositePass cloud_composite_pass_;
                    Passes::TaaPass taa_pass_;
                    Passes::TonemapPass tonemap_pass_;
                    Passes::FxaaPass fxaa_pass_;
                    Passes::PickingPass picking_pass_;
                    std::vector<Passes::IRenderPass*> passes_;
                    VkSampler ui_sampler_ = VK_NULL_HANDLE;
                    Slot slots_[SLOTS];
                    History history_[2];
                    RenderSettings settings_;
                    Frame::ResolutionController resolution_;
                    CameraView previous_camera_{};
                    float previous_jitter_[2] = {0.0f, 0.0f};
                    bool history_valid_ = false;
                    std::uint32_t width_ = 16;
                    std::uint32_t height_ = 16;
                    std::uint32_t render_width_ = 16;
                    std::uint32_t render_height_ = 16;
                    std::uint32_t current_slot_ = 0;
                    std::uint32_t frame_counter_ = 0;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
