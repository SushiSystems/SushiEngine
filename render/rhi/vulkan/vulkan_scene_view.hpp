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
 * view — its passes, its per-frame allocators, its soft-body buffers, its temporal
 * relationship to the previous frame, and the resolution the governor chose — and
 * reaches everything shared through the device's AssetLibrary. Its job each frame is
 * one thing: build a FrameContext, let each pass register into the graph, compile,
 * submit. It records no barrier and opens no render pass of its own.
 *
 * The GPU resources that make a frame possible — the double-buffered command slots, the
 * resolve image the editor samples, the picking readback, the uniform staging buffers,
 * the transient pools, and the temporal history — are target/readback lifecycle rather
 * than orchestration, and live in `ViewResources`. The view drives them through that
 * interface and never touches a `VkImage` directly.
 */

#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>

#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "frame/frame_context.hpp"
#include "frame/resolution_controller.hpp"
#include "geometry/cloth_buffers.hpp"
#include "graph/gpu_profiler.hpp"
#include "graph/render_graph.hpp"
#include "lighting/light_system.hpp"
#include "material/material_system.hpp"
#include "passes/atmosphere_lut_pass.hpp"
#include "passes/volumetric_fog_pass.hpp"
#include "passes/auto_exposure_pass.hpp"
#include "passes/bloom_pass.hpp"
#include "passes/cloud_composite_pass.hpp"
#include "passes/cloud_pass.hpp"
#include "passes/cloth_pass.hpp"
#include "passes/contact_shadow_pass.hpp"
#include "passes/cull_pass.hpp"
#include "passes/depth_prepass.hpp"
#include "passes/dof_pass.hpp"
#include "passes/fxaa_pass.hpp"
#include "passes/grid_pass.hpp"
#include "passes/motion_blur_pass.hpp"
#include "passes/ground_shadow_resolve_pass.hpp"
#include "passes/gtao_pass.hpp"
#include "passes/hiz_pass.hpp"
#include "passes/occlusion_pass.hpp"
#include "passes/ibl_pass.hpp"
#include "passes/irradiance_volume_pass.hpp"
#include "passes/ssr_pass.hpp"
#include "passes/light_cull_pass.hpp"
#include "passes/light_shadow_pass.hpp"
#include "passes/opaque_pass.hpp"
#include "passes/picking_pass.hpp"
#include "passes/ray_traced_shadow_pass.hpp"
#include "passes/shading_rate_pass.hpp"
#include "passes/shadow_pass.hpp"
#include "passes/sky_pass.hpp"
#include "passes/taa_pass.hpp"
#include "passes/tonemap_pass.hpp"
#include "resources/descriptor_allocator.hpp"
#include "raytracing/scene_accelerator.hpp"
#include "scene/instance_system.hpp"
#include "scene/motion_system.hpp"

#include "view_resources.hpp"
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
                                std::size_t strand_count = 0,
                                const PunctualLight* lights = nullptr,
                                std::size_t light_count = 0,
                                const Decal* decals = nullptr,
                                std::size_t decal_count = 0, bool show_grid = false) override;
                    std::uint32_t pick(std::uint32_t x, std::uint32_t y) override;
                    std::uint32_t slot_count() const noexcept override { return SLOTS; }
                    SceneViewTexture texture(std::uint32_t slot) const noexcept override;
                    std::uint32_t current_slot() const noexcept override { return current_slot_; }
                    std::size_t pass_timing_count() const noexcept override;
                    ScenePassTiming pass_timing(std::size_t index) const noexcept override;
                    void cull_statistics(std::uint32_t& drawn,
                                         std::uint32_t& tested) const noexcept override;

                private:
                    /** @brief The most frames that may be in flight; see @ref frame_slots_. */
                    static constexpr std::uint32_t SLOTS = ViewResources::SLOTS;

                    void update_render_extent();

                    /**
                     * @brief Applies a change to how deep the frame chain runs.
                     *
                     * Idles the device when the depth actually changes, because the frame
                     * index maps to a slot by modulo and a frame already in flight would
                     * otherwise share a slot with one recorded under the new mapping.
                     *
                     * @param requested Frames in flight the resolved settings ask for.
                     */
                    void update_frame_slots(std::uint32_t requested);
                    float measured_frame_milliseconds() const noexcept;

                    VulkanDevice& device_;
                    Assets::AssetLibrary& assets_;
                    Resources::DescriptorAllocator descriptors_;
                    Geometry::ClothBuffers cloth_;
                    Assets::MaterialSystem materials_;
                    Scene::MotionSystem motion_;
                    Scene::InstanceSystem instance_system_;
                    Lighting::LightSystem lights_;
                    RayTracing::SceneAccelerator accelerator_;
                    Graph::GpuProfiler profiler_;
                    Graph::RenderGraph graph_;
                    Passes::AtmosphereLutPass atmosphere_lut_pass_;
                    Passes::VolumetricFogPass volumetric_fog_pass_;
                    Passes::IblPass ibl_pass_;
                    Passes::IrradianceVolumePass irradiance_volume_pass_;
                    Passes::DepthPrepass depth_prepass_;
                    Passes::ShadowPass shadow_pass_;
                    Passes::ContactShadowPass contact_shadow_pass_;
                    Passes::RayTracedShadowPass ray_shadow_pass_;
                    Passes::GtaoPass gtao_pass_;
                    Passes::HizPass hiz_pass_;
                    Passes::OcclusionPass occlusion_pass_;
                    Passes::CullPass cull_pass_;
                    Passes::ClothPass cloth_pass_;
                    Passes::OpaquePass opaque_pass_;
                    Passes::LightCullPass light_cull_pass_;
                    Passes::LightShadowPass light_shadow_pass_;
                    Passes::ShadingRatePass shading_rate_pass_;
                    Passes::SkyPass sky_pass_;
                    Passes::GroundShadowResolvePass ground_shadow_resolve_pass_;
                    Passes::CloudPass cloud_pass_;
                    Passes::CloudCompositePass cloud_composite_pass_;
                    Passes::SsrPass ssr_pass_;
                    Passes::TaaPass taa_pass_;
                    Passes::DofPass dof_pass_;
                    Passes::MotionBlurPass motion_blur_pass_;
                    Passes::AutoExposurePass auto_exposure_pass_;
                    Passes::BloomPass bloom_pass_;
                    Passes::GridPass grid_pass_;
                    Passes::TonemapPass tonemap_pass_;
                    Passes::FxaaPass fxaa_pass_;
                    Passes::PickingPass picking_pass_;
                    std::vector<Passes::IRenderPass*> passes_;
                    ViewResources resources_;
                    RenderSettings settings_;
                    Frame::ResolutionController resolution_;
                    CameraView previous_camera_{};
                    float previous_jitter_[2] = {0.0f, 0.0f};
                    /**
                     * @brief The eye-adaptation exposure carried between frames.
                     *
                     * Manual exposure ignores it; automatic exposure adapts it toward the
                     * value last frame's luminance histogram implied, so the multiplier eases
                     * rather than snapping. Seeded to the photographic key.
                     */
                    float adapted_exposure_ = 0.18f;
                    std::uint32_t width_ = 16;
                    std::uint32_t height_ = 16;
                    std::uint32_t render_width_ = 16;
                    std::uint32_t render_height_ = 16;
                    /**
                     * @brief Heap slot the GI distance field was registered in, or invalid.
                     *
                     * Claimed once and kept: the field's image is created with the device and
                     * never reallocated, so re-registering it every frame would be descriptor
                     * traffic for an unchanging binding.
                     */
                    std::uint32_t visibility_field_slot_ = 0xFFFFFFFFu;
                    std::uint32_t current_slot_ = 0;
                    /**
                     * @brief Frames in flight actually in use, at most @ref SLOTS.
                     *
                     * Every slot is allocated at construction; this is how many of them the
                     * frame counter cycles through, which the delivery settings choose and
                     * the tier caps.
                     */
                    std::uint32_t frame_slots_ = 2;
                    std::uint32_t frame_counter_ = 0;
            };
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
