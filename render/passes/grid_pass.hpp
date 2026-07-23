/**************************************************************************/
/* grid_pass.hpp                                                          */
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
 * @file grid_pass.hpp
 * @brief The editor-only reference grid overlay.
 *
 * A fullscreen pass that composites an adaptive, analytically anti-aliased grid over the
 * HDR post colour, occluded by scene geometry. Near a planet the grid is that planet's
 * sea level — the same reference ellipsoid the analytic ground is drawn against — and out
 * in space it becomes the ecliptic plane pinned at the Sun; the camera's altitude above
 * the dominant body crossfades between them. It slots into the post chain as one more link that redirects @c post_color, exactly like
 * depth of field and motion blur, so the tone map reads it without naming it. Registered
 * only when the frame carries an enabled grid overlay — a shipped runtime never runs it.
 */

#include "passes/render_pass.hpp"

#include <vulkan/vulkan.h>
#include "resources/pipeline_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;
            class ShaderLibrary;
        }

        namespace Scene
        {
            class SceneLayout;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            /**
             * @brief The interplanetary frame the grid pass needs and the scene block lacks.
             *
             * The sea-level regime reads the planet's ellipsoid straight out of the shared
             * scene block, but nothing there locates the Sun or the ecliptic plane, so the
             * space regime's origin and orientation ride a push constant instead of growing
             * the block every pass would then have to re-declare. Explicit float arrays: GPU
             * data is 32-bit whatever the engine's Scalar precision, so a double build
             * narrows to float exactly here, after the camera-relative subtraction.
             */
            struct GridPushConstants
            {
                float sun_position[4];       /**< xyz = Sun centre relative to the camera, metres; w = space regime weight. */
                float ecliptic_normal[4];    /**< xyz = unit ecliptic pole, scene frame. */
                float ecliptic_reference[4]; /**< xyz = unit vernal-equinox direction in the ecliptic plane. */
            };

            /**
             * @brief Composites the editor reference grid over the scene.
             *
             * Non-copyable: it owns a Vulkan pipeline.
             */
            class GridPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the grid pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     */
                    GridPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                             Resources::GraphicsPipelineFactory& pipelines,
                             Scene::SceneLayout& layout);
                    ~GridPass() override;

                    GridPass(const GridPass&) = delete;
                    GridPass& operator=(const GridPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
