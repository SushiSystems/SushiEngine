/**************************************************************************/
/* particle_pass.cpp                                                      */
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

#include "passes/particle_pass.hpp"

#include <SushiEngine/core/types.hpp>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "lighting/cluster_config.hpp"
#include "lighting/light_system.hpp"
#include "passes/ibl_pass.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/particle_system.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            ParticlePass::ParticlePass(Vulkan::VulkanDevice& device,
                                       Resources::ShaderLibrary& shaders,
                                       Resources::GraphicsPipelineFactory& pipelines,
                                       Scene::ParticleSystem& particles,
                                       Lighting::LightSystem& lights, IblPass& ibl)
                : device_(device), shaders_(shaders), pipelines_(pipelines), particles_(particles),
                  lights_(lights), ibl_(ibl)
            {
                // Binding 0: the draw list the vertex stage reads to place each billboard.
                // Binding 1: the scene depth, sampled by the fragment stage for the occlusion test.
                // Binding 2: the sort keys the sorted-alpha vertex shader indexes through (unused by
                // the additive/billboard draws, whose shader never references it).
                // Bindings 3-7: the clustered-lighting inputs the lit bucket's fragment shades from —
                // the light array, the per-cluster count grid + index list the light-cull pass wrote,
                // the froxel config block, and the environment SH for ambient (unread by unlit draws).
                VkDescriptorSetLayoutBinding bindings[8]{};
                bindings[0].binding = 0;
                bindings[0].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[0].descriptorCount = 1;
                bindings[0].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                bindings[1].binding = 1;
                bindings[1].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[1].descriptorCount = 1;
                bindings[1].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                bindings[2].binding = 2;
                bindings[2].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[2].descriptorCount = 1;
                bindings[2].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                for (std::uint32_t b = 3; b <= 7; ++b)
                {
                    bindings[b].binding = b;
                    bindings[b].descriptorType = b == 6 ? VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER
                                                        : VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    bindings[b].descriptorCount = 1;
                    bindings[b].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                }

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 8;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(particle draw)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(particle draw)");

                create_pipeline();
            }

            ParticlePass::~ParticlePass()
            {
                destroy_pipeline();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void ParticlePass::create_pipeline()
            {
                Resources::GraphicsPipelineDesc desc;
                desc.layout = pipeline_layout_;
                desc.vertex_shader = shaders_.module("particle.vert");
                desc.fragment_shader = shaders_.module("particle.frag");
                desc.vertex_stride = 0;
                desc.attribute_count = 0;
                desc.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
                desc.cull_mode = VK_CULL_MODE_NONE;
                desc.depth_test = VK_FALSE;
                desc.depth_write = VK_FALSE;
                desc.color_count = 1;
                desc.color_formats[0] = Frame::HDR_FORMAT;
                desc.depth_format = VK_FORMAT_UNDEFINED;
                // The fragment output is premultiplied (rgb already scaled by alpha). Additive
                // is src + dst (glow); true-alpha is a premultiplied "over": src + dst*(1-a).
                desc.blend.enable = VK_TRUE;
                desc.blend.src_color = VK_BLEND_FACTOR_ONE;
                desc.blend.dst_color = VK_BLEND_FACTOR_ONE;
                desc.blend.color_op = VK_BLEND_OP_ADD;
                desc.blend.src_alpha = VK_BLEND_FACTOR_ONE;
                desc.blend.dst_alpha = VK_BLEND_FACTOR_ONE;
                desc.blend.alpha_op = VK_BLEND_OP_ADD;
                pipeline_ = pipelines_.create(desc);

                // The alpha bucket draws back-to-front through the sort keys, so it uses the
                // indexed vertex shader and the premultiplied "over" blend.
                desc.vertex_shader = shaders_.module("particle_sorted.vert");
                desc.blend.dst_color = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                desc.blend.dst_alpha = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                alpha_pipeline_ = pipelines_.create(desc);
            }

            void ParticlePass::destroy_pipeline()
            {
                // The factory owns the pipelines; the pass drops only its handles.
                pipeline_ = Resources::PipelineHandle{};
                alpha_pipeline_ = Resources::PipelineHandle{};
            }

            void ParticlePass::rebuild_pipelines()
            {
                destroy_pipeline();
                create_pipeline();
            }

            void ParticlePass::register_pass(Graph::RenderGraph& graph,
                                             const Frame::FrameContext& frame)
            {
                // GPU-simulated cosmetic emitters draw via the indirect path (tier-gated);
                // CPU-deterministic particles draw as host-uploaded billboards (gameplay, always).
                const bool draw_emitters = !particles_.empty() && frame.quality.gpu_particles;
                const bool draw_billboards = !particles_.billboards_empty();
                if (!draw_emitters && !draw_billboards)
                    return;

                const std::uint32_t slot = frame.slot;

                graph.add_pass(
                    "particles",
                    [&frame, draw_emitters](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.scene_final,
                                                 Graph::AttachmentLoad::Load);
                        builder.read(frame.targets.depth, Graph::TextureAccess::SampledFragment);
                        // The froxel grid the light-cull pass built: read so the graph derives the
                        // compute→fragment barrier that makes the light lists visible before the lit
                        // bucket loops them (the same declaration opaque_pass makes).
                        builder.read(frame.targets.cluster_grid, Graph::BufferAccess::StorageRead);
                        builder.read(frame.targets.light_index, Graph::BufferAccess::StorageRead);
                        if (draw_emitters)
                        {
                            builder.read(frame.targets.particle_draw,
                                         Graph::BufferAccess::StorageRead);
                            builder.read(frame.targets.particle_alpha,
                                         Graph::BufferAccess::StorageRead);
                            builder.read(frame.targets.particle_sort_keys,
                                         Graph::BufferAccess::StorageRead);
                            builder.read(frame.targets.particle_args,
                                         Graph::BufferAccess::IndirectRead);
                        }
                    },
                    [this, &frame, slot, draw_emitters, draw_billboards](
                        VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        const VkSampler sampler = frame.samplers->get(Resources::SamplerDesc{});
                        const VkImageView depth_view = context.sampled_view(frame.targets.depth);

                        Push push{};
                        const Mat4 view_projection =
                            mul(frame.camera->projection, frame.camera->view);
                        for (int i = 0; i < 16; ++i)
                            push.view_projection[i] = static_cast<float>(view_projection.m[i]);
                        // The camera world position rides the spare w lanes (the scene_uniforms
                        // packing), so the vertex stage can subtract it to reach the camera-relative
                        // space the clustered lights live in without spending another vec4. Cast to
                        // float here, so the shading position inherits the pool's float32 precision —
                        // acceptable for the near-camera cosmetic particles this path serves.
                        const Mat4& view = frame.camera->view;
                        push.camera_right[0] = static_cast<float>(view.m[0]);
                        push.camera_right[1] = static_cast<float>(view.m[4]);
                        push.camera_right[2] = static_cast<float>(view.m[8]);
                        push.camera_right[3] = static_cast<float>(frame.eye[0]);
                        push.camera_up[0] = static_cast<float>(view.m[1]);
                        push.camera_up[1] = static_cast<float>(view.m[5]);
                        push.camera_up[2] = static_cast<float>(view.m[9]);
                        push.camera_up[3] = static_cast<float>(frame.eye[1]);
                        // The sun is a world-space directional light, so lit particles need no
                        // camera-relative conversion (unlike the clustered froxel lights).
                        const Environment& environment = *frame.environment;
                        push.sun_direction[0] = static_cast<float>(environment.sun.direction.x);
                        push.sun_direction[1] = static_cast<float>(environment.sun.direction.y);
                        push.sun_direction[2] = static_cast<float>(environment.sun.direction.z);
                        push.sun_direction[3] = static_cast<float>(frame.eye[2]);
                        push.sun_radiance[0] =
                            static_cast<float>(environment.sun.color.x) * environment.sun.intensity;
                        push.sun_radiance[1] =
                            static_cast<float>(environment.sun.color.y) * environment.sun.intensity;
                        push.sun_radiance[2] =
                            static_cast<float>(environment.sun.color.z) * environment.sun.intensity;

                        // The ambient scale the lit (true-alpha) bucket applies to the environment
                        // SH: the IBL intensity, or 0 when image-based lighting is off (then lit
                        // particles get only the sun + punctual lights, like a mesh would).
                        const float lit_ambient =
                            environment.image_based_lighting ? environment.ibl_intensity : 0.0f;

                        // Draws one bucket: sets the lit/ambient flag (negative = unlit/emissive,
                        // >=0 = lit with that SH-ambient scale), pushes, binds its source + pipeline,
                        // then issues the caller's draw. Additive/billboards are emissive; the
                        // true-alpha bucket (smoke/dust) receives the sun + clustered lights.
                        const VkDeviceSize keys_range =
                            static_cast<VkDeviceSize>(particles_.capacity()) * 2 *
                            sizeof(std::uint32_t);
                        auto draw_bucket = [&](Resources::PipelineHandle& pipeline, VkBuffer source,
                                               VkDeviceSize range, float lit_flag, VkBuffer keys,
                                               auto&& issue)
                        {
                            push.sun_radiance[3] = lit_flag;
                            vkCmdPushConstants(cmd, pipeline_layout_,
                                               VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT,
                                               0, sizeof(Push), &push);
                            const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                            Resources::DescriptorWriter writer;
                            writer.storage_buffer(0, source, range);
                            writer.sampled_image(1, depth_view, sampler);
                            // The sorted-alpha shader reads the keys at binding 2; the direct
                            // shaders never touch it, so a valid buffer there is harmless.
                            writer.storage_buffer(2, keys != VK_NULL_HANDLE ? keys : source,
                                                  keys != VK_NULL_HANDLE ? keys_range : range);
                            // The clustered-lighting inputs the lit bucket shades from. Bound on
                            // every bucket (the set is freshly allocated per draw); the unlit
                            // fragment path never samples them, so they are inert for those draws.
                            writer.storage_buffer(3, lights_.light_buffer(),
                                                  lights_.light_buffer_range());
                            writer.storage_buffer(
                                4, context.buffer(frame.targets.cluster_grid),
                                Lighting::CLUSTER_COUNT * sizeof(std::uint32_t));
                            writer.storage_buffer(
                                5, context.buffer(frame.targets.light_index),
                                Lighting::LIGHT_INDEX_COUNT * sizeof(std::uint32_t));
                            writer.uniform_buffer(6, lights_.config_buffer(),
                                                  lights_.config_buffer_range());
                            writer.storage_buffer(7, ibl_.sh_buffer(), IblPass::sh_buffer_bytes());
                            writer.update(device_.device(), set);
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.get());
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                           pipeline_layout_, 0, set);
                            issue();
                        };

                        if (draw_emitters)
                        {
                            const VkBuffer args = context.buffer(frame.targets.particle_args);
                            draw_bucket(pipeline_, context.buffer(frame.targets.particle_draw),
                                        particles_.pool_range(), -1.0f, VK_NULL_HANDLE, [&]() {
                                            vkCmdDrawIndirect(cmd, args, 0, 1,
                                                              sizeof(VkDrawIndirectCommand));
                                        });
                            draw_bucket(alpha_pipeline_, context.buffer(frame.targets.particle_alpha),
                                        particles_.pool_range(), lit_ambient,
                                        context.buffer(frame.targets.particle_sort_keys), [&]() {
                                            vkCmdDrawIndirect(cmd, args, sizeof(VkDrawIndirectCommand),
                                                              1, sizeof(VkDrawIndirectCommand));
                                        });
                        }
                        if (draw_billboards)
                        {
                            draw_bucket(pipeline_, particles_.billboard_buffer(slot),
                                        particles_.billboard_range(), -1.0f, VK_NULL_HANDLE,
                                        [&]() { vkCmdDraw(cmd, 6, particles_.billboard_count(), 0, 0); });
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
