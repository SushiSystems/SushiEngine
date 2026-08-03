/**************************************************************************/
/* particle_mesh_pass.hpp                                                 */
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
 * @file particle_mesh_pass.hpp
 * @brief Draws mesh-aligned particles as solid instanced geometry (design §7.11).
 *
 * The one particle path that is not transparent. A mesh particle is a piece of debris, so it
 * belongs with the opaque geometry: this pass runs straight after the opaque pass, loading the
 * scene's HDR colour and depth, and depth-tests and depth-writes like any other solid surface.
 * Drawing it in the transparency pass with the other particles would leave overlapping debris
 * compositing in list order, which no amount of blending can make look right.
 *
 * One indexed indirect draw per mesh-aligned emitter, because one draw binds one mesh; each
 * emitter's particles occupy its own slice of the shared mesh list, and the sim pass seeded its
 * command's index count from the host-known mesh.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

#include "passes/render_pass.hpp"
#include "resources/pipeline_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Resources
        {
            class ShaderLibrary;
            class GraphicsPipelineFactory;
        }

        namespace Geometry
        {
            class MeshRegistry;
        }

        namespace Scene
        {
            class ParticleSystem;
        }

        namespace Passes
        {
            /** @brief Draws the frame's mesh particles as solid, depth-tested instances. */
            class ParticleMeshPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the mesh-particle graphics pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue particle_mesh.vert/frag come from.
                     * @param pipelines The factory owning the graphics pipeline.
                     * @param particles The system holding this frame's mesh-draw slices.
                     * @param meshes    Registry the slices' vertex and index buffers come from.
                     */
                    ParticleMeshPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                     Resources::GraphicsPipelineFactory& pipelines,
                                     Scene::ParticleSystem& particles,
                                     const Geometry::MeshRegistry& meshes);
                    ~ParticleMeshPass() override;

                    ParticleMeshPass(const ParticleMeshPass&) = delete;
                    ParticleMeshPass& operator=(const ParticleMeshPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    /** @brief Binding of the mesh particle list, read by the vertex stage. */
                    static constexpr std::uint32_t PARTICLE_LIST_BINDING = 0;

                    /**
                     * @brief Binding of the sun's cascade block.
                     *
                     * The scene set's own number, as the transparency pass does it, so the shared
                     * shadow_common.glsl declaration is reused rather than copied.
                     */
                    static constexpr std::uint32_t SHADOW_BLOCK_BINDING = 10;

                    /** @brief Binding of the sun's cascade atlas (comparison sampler). */
                    static constexpr std::uint32_t SHADOW_ATLAS_BINDING = 11;

                    /** @brief 128-byte draw constants, the house budget. */
                    struct Push
                    {
                        float view_projection[16];
                        float sun_direction[4]; /**< xyz: direction to the sun; w: eye.x. */
                        float sun_radiance[4];  /**< rgb: sun colour * intensity; w: eye.y. */
                        float ambient[4];       /**< rgb: flat ambient; w: eye.z. */
                        std::uint32_t slice[4]; /**< x: first particle, y: particles in the slice. */
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::ParticleSystem& particles_;
                    const Geometry::MeshRegistry& meshes_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
