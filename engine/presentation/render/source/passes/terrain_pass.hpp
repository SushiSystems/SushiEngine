/**************************************************************************/
/* terrain_pass.hpp                                                       */
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
 * @file terrain_pass.hpp
 * @brief Uploading this frame's tiles and drawing the body they belong to.
 *
 * Two graph passes registered together, in the order they must run: the tile upload, then
 * one instanced draw covering every selected node (`docs/slop/solar_system_overhaul.md`
 * §8.1). Registering them from one object is what keeps the barrier between them derived
 * rather than hand-placed — the upload declares the slot image as a transfer destination
 * and the draw declares it as a vertex-stage sampled read, and the graph does the rest.
 *
 * **One draw per body.** Every visible node rides one `vkCmdDrawIndexed` as an instance,
 * so the CPU cost of terrain is its selection and nothing else. The index buffer is a
 * single shared 33 × 33 lattice created once; there is no vertex buffer at all, because a
 * node's geometry is entirely a function of its record and the vertex index.
 *
 * **What P2b leaves out, stated rather than discovered.** Terrain does not contribute to
 * the depth prepass yet, so it is absent from Hi-Z occlusion and from GTAO; it draws with
 * depth test and write in the opaque phase, which is correct but pays overdraw the prepass
 * would have saved. Back-face culling is off until the first rendered frame confirms the
 * winding, because a winding mistake with culling on is an invisible planet and with
 * culling off is a slightly slower one.
 */

#include <cstdint>

#include <vk_mem_alloc.h>
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

        namespace Assets
        {
            class MaterialSystem;
        }

        namespace Lighting
        {
            class LightSystem;
        }

        namespace Scene
        {
            class MotionSystem;
        }

        namespace Terrain
        {
            class PlanetTerrain;
            class TerrainLayout;
        }

        namespace Passes
        {
            class CloudShadowMapPass;
            class IBLPass;
            class IrradianceVolumePass;

            /** @brief Uploads this frame's tiles and draws one body's selected nodes. */
            class TerrainPass final : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the shared lattice and the terrain pipeline.
                     *
                     * The six collaborators after @p terrain are not terrain's business at
                     * all — they are what `pbr.frag` reads out of set 0, and terrain shades
                     * through `pbr.frag` unchanged. They are taken here for the same reason
                     * the opaque and transparent passes take them: set 0 is a push
                     * descriptor set, so a pass that writes only the descriptors *it* cares
                     * about leaves the rest undefined, and the fragment shader samples
                     * them regardless.
                     *
                     * @param device       The live Vulkan device.
                     * @param shaders      The catalogue `terrain.vert` and `pbr.frag` come from.
                     * @param pipelines    The factory owning the graphics pipeline.
                     * @param layout       Terrain's set-2 and pipeline layouts.
                     * @param terrain      The body being drawn; must outlive this pass.
                     * @param ibl          Image-based lighting, for set 0.
                     * @param cloud_shadow The cloud shadow map, for set 0.
                     * @param gi           The probe volume, for set 0.
                     * @param materials    The material array, for set 0.
                     * @param motion       The previous-transform array, for set 0.
                     * @param lights       The clustered light and decal engine, for set 0.
                     */
                    TerrainPass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                Resources::GraphicsPipelineFactory& pipelines,
                                Terrain::TerrainLayout& layout, Terrain::PlanetTerrain& terrain,
                                IBLPass& ibl, CloudShadowMapPass& cloud_shadow,
                                IrradianceVolumePass& gi, Assets::MaterialSystem& materials,
                                Scene::MotionSystem& motion, Lighting::LightSystem& lights);
                    ~TerrainPass() override;

                    TerrainPass(const TerrainPass&) = delete;
                    TerrainPass& operator=(const TerrainPass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    /** @brief Vertices per side of the shared lattice. */
                    static constexpr std::uint32_t LATTICE_VERTICES = 33;

                    /** @brief Indices in the shared lattice: 32 x 32 cells, two triangles each. */
                    static constexpr std::uint32_t LATTICE_INDICES =
                        (LATTICE_VERTICES - 1) * (LATTICE_VERTICES - 1) * 6;

                    void create_lattice();
                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Terrain::TerrainLayout& layout_;
                    Terrain::PlanetTerrain& terrain_;
                    IBLPass& ibl_;
                    CloudShadowMapPass& cloud_shadow_;
                    IrradianceVolumePass& gi_;
                    Assets::MaterialSystem& materials_;
                    Scene::MotionSystem& motion_;
                    Lighting::LightSystem& lights_;

                    VkBuffer indices_ = VK_NULL_HANDLE;
                    VmaAllocation indices_allocation_ = VK_NULL_HANDLE;
                    Resources::PipelineHandle pipeline_;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
