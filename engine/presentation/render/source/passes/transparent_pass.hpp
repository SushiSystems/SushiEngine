/**************************************************************************/
/* transparent_pass.hpp                                                   */
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
 * @file transparent_pass.hpp
 * @brief The alpha-blended geometry pass: rigid and skinned draws whose material is
 * SurfaceType::Transparent or SurfaceType::Fade.
 *
 * Runs after OpaquePass, reading the depth buffer it wrote (test only, no write) and
 * compositing onto the same HDR, id, velocity, and gbuffer targets. Rigid instances and
 * skinned character slices are merged into one list and draw back-to-front by distance
 * from the camera eye, switching between the rigid and skinned pipeline as the sorted
 * order requires, so overlapping transparent surfaces of either kind blend in the
 * correct order. Shares OpaquePass's vertex layouts, push constants, and scene
 * descriptor bindings — it draws with the same pbr.frag, only the pipelines' blend and
 * depth-write state differ.
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
            class MeshRegistry;
        }

        namespace Assets
        {
            class MaterialSystem;
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
            class SkinningSystem;
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
            class CloudShadowMapPass;
            class IBLPass;
            class IrradianceVolumePass;

            /**
             * @brief Draws the scene's transparent geometry, alpha-blended over the opaque result.
             *
             * Owns two lit pipelines — rigid and skinned — both built with straight alpha
             * blending and no depth write. Non-copyable: it owns Vulkan pipelines.
             */
            class TransparentPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the pass's pipelines.
                     * @param device    The live Vulkan device.
                     * @param shaders   Library the shader modules come from.
                     * @param pipelines Factory the pipeline is built through.
                     * @param layout    The shared scene descriptor and pipeline layout.
                     * @param meshes    Registry holding the primitives and imported meshes.
                     * @param materials System packing this frame's material array.
                     * @param motion    System packing this frame's previous transforms.
                     * @param skinning  System holding this frame's skinned instance slices
                     *                  and their deformed vertex output buffer.
                     * @param cloud_shadow The pass owning the baked cloud shadow map, to
                     *                     shadow a surface with the deck standing over it.
                     * @param ibl       The image-based lighting chain surfaces sample.
                     * @param gi        The probe-volume GI the shading pass gathers ambient from.
                     * @param lights    The clustered light engine's per-frame buffers.
                     */
                    TransparentPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                    Resources::GraphicsPipelineFactory& pipelines,
                                    Scene::SceneLayout& layout, Geometry::MeshRegistry& meshes,
                                    Assets::MaterialSystem& materials, Scene::MotionSystem& motion,
                                    Scene::SkinningSystem& skinning, CloudShadowMapPass& cloud_shadow,
                                    IBLPass& ibl, IrradianceVolumePass& gi,
                                    Lighting::LightSystem& lights);
                    ~TransparentPass() override;

                    TransparentPass(const TransparentPass&) = delete;
                    TransparentPass& operator=(const TransparentPass&) = delete;

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
                    Assets::MaterialSystem& materials_;
                    Scene::MotionSystem& motion_;
                    Scene::SkinningSystem& skinning_;
                    CloudShadowMapPass& cloud_shadow_;
                    IBLPass& ibl_;
                    IrradianceVolumePass& gi_;
                    Lighting::LightSystem& lights_;
                    Resources::PipelineHandle mesh_pipeline_;
                    Resources::PipelineHandle skinned_pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
