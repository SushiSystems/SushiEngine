/**************************************************************************/
/* particle_pass.hpp                                                      */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
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
 * @file particle_pass.hpp
 * @brief Draws the simulated particles as camera-facing billboards (design §5.4).
 *
 * Runs between the screen-space reflections and the temporal resolve, compositing into the
 * finished HDR scene (@c scene_final). A vertex-less draw expands six vertices per alive
 * particle from the compacted draw list the simulate pass built, indexed by an indirect draw
 * whose instance count that pass wrote. It samples the scene depth (never attaches it) to
 * discard fragments behind opaque geometry, and blends additively. The alpha-blended, sorted
 * tier and lit particles are VFX2.
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

        namespace Scene
        {
            class ParticleSystem;
        }

        namespace Lighting
        {
            class LightSystem;
        }

        namespace Passes
        {
            class IblPass;

            /** @brief Billboards the frame's alive particles into the HDR scene. */
            class ParticlePass : public IRenderPass
            {
                public:
                    /**
                     * @brief Builds the billboard graphics pipeline.
                     * @param device    The live Vulkan device.
                     * @param shaders   The catalogue particle.vert/frag come from.
                     * @param pipelines The factory owning the graphics pipeline.
                     * @param particles The pool the compacted draw list is sized from.
                     * @param lights    The clustered-light engine the lit bucket shades from
                     *                  (light array + froxel config).
                     * @param ibl       The image-based lighting whose SH the lit bucket reads for ambient.
                     */
                    ParticlePass(Vulkan::VulkanDevice& device, Resources::ShaderLibrary& shaders,
                                 Resources::GraphicsPipelineFactory& pipelines,
                                 Scene::ParticleSystem& particles, Lighting::LightSystem& lights,
                                 IblPass& ibl);
                    ~ParticlePass() override;

                    ParticlePass(const ParticlePass&) = delete;
                    ParticlePass& operator=(const ParticlePass&) = delete;

                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
                    void rebuild_pipelines() override;

                private:
                    /**
                     * @brief 128-byte draw constants (the house push-constant budget, matching
                     *        MeshPushConstants). The camera world position is packed into the spare
                     *        w lanes exactly as scene_uniforms packs the eye, so no extra vec4 is
                     *        needed: the vertex reads eye = (camera_right.w, camera_up.w,
                     *        sun_direction.w) to form the camera-relative shading position, and takes
                     *        the froxel view-depth straight from gl_Position.w (perspective clip w =
                     *        the positive view-space distance the cluster z-slice is keyed on).
                     */
                    struct Push
                    {
                        float view_projection[16];
                        float camera_right[4];  /**< xyz: world camera right; w: eye.x. */
                        float camera_up[4];     /**< xyz: world camera up; w: eye.y. */
                        float sun_direction[4]; /**< xyz: direction to the sun; w: eye.z. */
                        float sun_radiance[4];  /**< rgb: sun colour * intensity; w: lit/ambient — <0 unlit, >=0 lit (value = IBL ambient scale). */
                    };

                    void create_pipeline();
                    void destroy_pipeline();

                    Vulkan::VulkanDevice& device_;
                    Resources::ShaderLibrary& shaders_;
                    Resources::GraphicsPipelineFactory& pipelines_;
                    Scene::ParticleSystem& particles_;
                    Lighting::LightSystem& lights_;
                    IblPass& ibl_;

                    VkDescriptorSetLayout set_layout_ = VK_NULL_HANDLE;
                    VkPipelineLayout pipeline_layout_ = VK_NULL_HANDLE;
                    Resources::PipelineHandle pipeline_;       /**< Additive/premultiplied billboards. */
                    Resources::PipelineHandle alpha_pipeline_; /**< True-alpha (SRC_ALPHA) billboards. */
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
