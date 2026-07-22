/**************************************************************************/
/* frame_context.hpp                                                      */
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
 * @file frame_context.hpp
 * @brief Everything a pass needs to know about the frame it is registering into.
 *
 * The context is the only thing passed to a pass's register step, which is what
 * keeps a pass from reaching into the scene view: it names the camera, the targets
 * the graph created for this frame, the draw list, and the per-frame allocators. A
 * pass reads the handful of fields it cares about and ignores the rest.
 */

#include <cstddef>
#include <cstdint>

#include <vulkan/vulkan.h>

#include <SushiEngine/render/environment.hpp>
#include <SushiEngine/render/light.hpp>
#include <SushiEngine/render/quality_params.hpp>
#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "graph/resource_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class DescriptorAllocator;
            class SamplerCache;
        }

        namespace Scene
        {
            class SceneLayout;
        }

        namespace Frame
        {
            /**
             * @brief Linear HDR format for the scene and composite targets.
             *
             * Atmospheric scattering and the sun produce values well above 1, so nothing
             * is quantised until the display encode at the very end.
             */
            constexpr VkFormat HDR_FORMAT = VK_FORMAT_R16G16B16A16_SFLOAT;

            /** @brief The LDR format the editor samples the finished frame from. */
            constexpr VkFormat RESOLVE_FORMAT = VK_FORMAT_R8G8B8A8_UNORM;

            /**
             * @brief Floating-point depth with stencil.
             *
             * Float depth spreads reverse-Z's precision across the whole planet-scale
             * range (a 24-bit unorm would z-fight); the stencil drives the selection
             * outline mask.
             */
            constexpr VkFormat DEPTH_FORMAT = VK_FORMAT_D32_SFLOAT_S8_UINT;

            /** @brief The integer format the picking ids are written to. */
            constexpr VkFormat ID_FORMAT = VK_FORMAT_R32_UINT;

            /**
             * @brief Screen-space motion, in UV displacement per frame.
             *
             * Two signed half-floats: the displacement never exceeds the screen, and
             * half precision across that range is finer than the reprojection filter
             * can resolve, so the wider format would only cost bandwidth.
             */
            constexpr VkFormat VELOCITY_FORMAT = VK_FORMAT_R16G16_SFLOAT;

            /**
             * @brief The per-tile fragment shading rate image's format.
             *
             * One byte per tile, holding the packed (log2 width, log2 height) rate the
             * Vulkan fragment-shading-rate attachment expects.
             */
            constexpr VkFormat SHADING_RATE_FORMAT = VK_FORMAT_R8_UINT;

            /**
             * @brief Depth format of the sun's shadow cascade atlas.
             *
             * Float depth with no stencil. Unlike the camera's depth this is
             * orthographic and therefore linear, so it gains nothing from reverse-Z and
             * uses the conventional near-is-zero convention.
             */
            constexpr VkFormat SHADOW_FORMAT = VK_FORMAT_D32_SFLOAT;

            /** @brief Single-channel visibility written by the screen-space shadow march. */
            constexpr VkFormat CONTACT_SHADOW_FORMAT = VK_FORMAT_R8_UNORM;

            /**
             * @brief The thin reflection G-buffer's format: roughness and reflectance.
             *
             * Two half floats — r = perceptual roughness, g = scalar F0 — the opaque pass
             * writes beside its colour so screen-space reflections know which surfaces are
             * smooth enough to reflect and how bright the reflection is. The surface normal is
             * reconstructed from depth rather than stored, so this stays two channels.
             */
            constexpr VkFormat GBUFFER_FORMAT = VK_FORMAT_R16G16_SFLOAT;

            /** @brief The tier gating the expensive half of every pass. */
            using RenderQuality = Render::RenderQuality;

            /** @brief What the scene passes draw this frame. */
            struct SceneDrawList
            {
                const MeshInstance* instances = nullptr;
                std::size_t instance_count = 0;
                const ClothStrandView* strands = nullptr;
                std::size_t strand_count = 0;
                /** @brief The frame's punctual lights, culled into the cluster grid. */
                const PunctualLight* lights = nullptr;
                std::size_t light_count = 0;
                /** @brief The frame's projected decals, culled into the same grid. */
                const Decal* decals = nullptr;
                std::size_t decal_count = 0;
                std::uint32_t selected_id = NO_PICK;
            };

            /**
             * @brief The graph resources shared between this frame's passes.
             *
             * Every handle is created fresh each frame by the scene view; a pass stores
             * none of them across frames.
             */
            struct FrameTargets
            {
                Graph::TextureHandle hdr;       /**< Linear HDR scene colour. */
                Graph::TextureHandle depth;     /**< Reverse-Z depth + stencil. */
                Graph::TextureHandle id;        /**< R32_UINT picking ids. */
                Graph::TextureHandle velocity;  /**< Per-pixel UV motion since last frame. */
                Graph::TextureHandle shadow_atlas;   /**< The sun's cascades, a 2x2 tile grid. */
                Graph::TextureHandle light_shadow_atlas; /**< Punctual spot shadows, a 4x4 tile grid. */
                Graph::TextureHandle contact_shadow; /**< Screen-space contact visibility. */
                Graph::TextureHandle ray_shadow;     /**< Traced sun visibility, Ultra tier. */
                Graph::TextureHandle gtao;      /**< Half-res GTAO: view-space bent normal + visibility. */
                Graph::TextureHandle ao;        /**< Full-res resolved AO: world bent normal + visibility. */
                Graph::TextureHandle gbuffer;   /**< Thin reflection G-buffer: roughness, F0. */
                Graph::TextureHandle composite; /**< HDR sky over the shaded scene. */
                Graph::TextureHandle ground_shadow; /**< Raw analytic-ground direct term + noisy sun visibility. */
                Graph::TextureHandle ground_shadow_resolved; /**< Bilateral-blurred ground_shadow. */
                Graph::TextureHandle cloud;     /**< Half-resolution volumetric clouds. */
                Graph::TextureHandle scene;     /**< HDR scene with the clouds composited in. */
                Graph::TextureHandle scene_reflected; /**< Scene with SSR folded in, or invalid. */
                Graph::TextureHandle scene_final; /**< The scene the resolve reads: reflected if SSR ran, else scene. */
                Graph::TextureHandle history;   /**< Previous frame's resolved HDR, output size. */
                Graph::TextureHandle resolved;  /**< This frame's resolved HDR, output size. */
                Graph::TextureHandle shading_rate; /**< Per-tile shading rate, or invalid. */
                Graph::TextureHandle display;   /**< What the display transform writes into. */
                Graph::TextureHandle resolve;   /**< LDR image the editor samples. */
                Graph::BufferHandle uniforms;   /**< The per-frame scene block. */
                Graph::BufferHandle temporal;   /**< The per-frame temporal block. */
                Graph::BufferHandle shadow;     /**< The per-frame shadow cascade block. */
                Graph::BufferHandle cluster_grid; /**< Per-cluster punctual-light counts. */
                Graph::BufferHandle light_index;  /**< Per-cluster punctual-light index list. */
                Graph::BufferHandle decal_grid;   /**< Per-cluster decal counts. */
                Graph::BufferHandle decal_index;  /**< Per-cluster decal index list. */
                Graph::BufferHandle readback;   /**< Host-visible copy of the id target. */
            };

            /**
             * @brief One frame's inputs, targets, and per-frame services.
             *
             * @c width / @c height are the *internal* render extent, which the render
             * scale and the dynamic-resolution controller may put below the output
             * extent; every pass before the temporal resolve works at that extent and
             * takes it from here rather than from the view.
             */
            struct FrameContext
            {
                std::uint32_t index = 0; /**< Monotonic frame counter. */
                std::uint32_t slot = 0;  /**< Which double-buffered slot is being recorded. */
                std::uint32_t width = 1;
                std::uint32_t height = 1;
                std::uint32_t output_width = 1;  /**< Extent the resolved image is presented at. */
                std::uint32_t output_height = 1;

                /**
                 * @brief The tier-resolved effective settings this frame renders with.
                 *
                 * Not the host's authored request but the result of running that request
                 * through the quality resolver: at High it is the request verbatim, at a
                 * lower tier the expensive fields are scaled down. Every pass reads this.
                 */
                RenderSettings settings;

                /** @brief The tier-resolved per-pass knobs with no home in @c settings. */
                QualityParams quality;

                const CameraView* camera = nullptr;
                const Environment* environment = nullptr;
                double eye[3] = {0.0, 0.0, 0.0}; /**< Camera world position, metres. */
                float jitter[2] = {0.0f, 0.0f};  /**< This frame's sub-pixel jitter, NDC. */
                bool history_valid = false;      /**< Whether a previous resolved frame exists. */
                SceneDrawList draws;
                FrameTargets targets;
                Resources::DescriptorAllocator* descriptors = nullptr;
                Resources::SamplerCache* samplers = nullptr;
                Scene::SceneLayout* layout = nullptr;

                /** @brief Cascades fitted this frame; zero when the sun casts none. */
                std::uint32_t cascade_count = 0;

                /** @brief Side of one cascade tile in the shadow atlas, in texels. */
                std::uint32_t shadow_resolution = 0;

                /** @brief Whether the frame ends with a temporal resolve. */
                bool temporal_enabled() const noexcept
                {
                    return settings.anti_aliasing == AntiAliasingMode::Temporal;
                }
            };
        } // namespace Frame
    } // namespace Render
} // namespace SushiEngine
