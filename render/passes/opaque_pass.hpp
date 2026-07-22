/**************************************************************************/
/* opaque_pass.hpp                                                        */
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
 * @file opaque_pass.hpp
 * @brief The lit geometry pass: grid, mesh instances, soft bodies, selection outline.
 *
 * Writes the HDR scene colour, the R32_UINT picking ids, the per-pixel screen motion
 * the temporal resolve reprojects with, and the reverse-Z depth the sky and cloud
 * passes later read. Camera-relative throughout: every model's
 * translation has the eye subtracted in double before the float cast, and the
 * uploaded view matrix carries no translation to match.
 */

#include "passes/render_pass.hpp"

#include <vulkan/vulkan.h>
#include "resources/pipeline_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Geometry
        {
            class ClothBuffers;
            class MeshRegistry;
        }

        namespace Assets
        {
            class MaterialSystem;
        }

        namespace Textures
        {
            class CloudNoise;
        }

        namespace Resources
        {
            class GraphicsPipelineFactory;
            class ShaderLibrary;
        }

        namespace Scene
        {
            class MotionSystem;
            class SceneLayout;
        }

        namespace Lighting
        {
            class LightSystem;
        }

        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Passes
        {
            class IblPass;
            class IrradianceVolumePass;

            /**
             * @brief Draws the scene's opaque geometry into the HDR, id, and depth targets.
             *
             * Owns three pipelines that differ only in topology, polygon mode, and
             * fragment shader: the lit mesh pipeline, the flat line pipeline the grid
             * uses, and the stencil-masked wireframe pipeline that draws the selection
             * outline. Non-copyable: it owns Vulkan pipelines.
             */
            class OpaquePass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the pass's pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipelines are built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     * @param meshes    Registry holding the primitives and imported meshes.
                     * @param cloth     This view's per-frame soft-body buffers.
                     * @param materials System packing this frame's material array.
                     * @param motion    System packing this frame's previous transforms.
                     * @param noise     The cloud volumes; only the weather map is read here,
                     *                  to shadow a surface with the deck standing over it.
                     * @param ibl       The image-based lighting chain surfaces sample.
                     * @param gi        The probe-volume GI the shading pass gathers ambient from.
                     * @param lights    The clustered light engine's per-frame buffers.
                     */
                    OpaquePass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                               Resources::GraphicsPipelineFactory& pipelines,
                               Scene::SceneLayout& layout, Geometry::MeshRegistry& meshes,
                               Geometry::ClothBuffers& cloth,
                               Assets::MaterialSystem& materials, Scene::MotionSystem& motion,
                               Textures::CloudNoise& noise, IblPass& ibl,
                               IrradianceVolumePass& gi, Lighting::LightSystem& lights);
                    ~OpaquePass() override;

                    OpaquePass(const OpaquePass&) = delete;
                    OpaquePass& operator=(const OpaquePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    void create_pipelines();
                    void destroy_pipelines();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::SceneLayout& layout_;
                    Geometry::MeshRegistry& meshes_;
                    Geometry::ClothBuffers& cloth_;
                    Assets::MaterialSystem& materials_;
                    Scene::MotionSystem& motion_;
                    Textures::CloudNoise& noise_;
                    IblPass& ibl_;
                    IrradianceVolumePass& gi_;
                    Lighting::LightSystem& lights_;
                    Resources::PipelineHandle mesh_pipeline_;
                    Resources::PipelineHandle line_pipeline_;
                    Resources::PipelineHandle outline_pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
