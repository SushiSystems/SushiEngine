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
#include "passes/shadow_pass.hpp"
#include "resources/descriptor_allocator.hpp"
#include "resources/descriptor_heap.hpp"
#include "resources/descriptor_writer.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/particle_system.hpp"
#include "scene/shadow_uniforms.hpp"

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
                                       Lighting::LightSystem& lights, IBLPass& ibl,
                                       Resources::DescriptorHeap& heap)
                : device_(device), shaders_(shaders), pipelines_(pipelines), particles_(particles),
                  lights_(lights), ibl_(ibl), heap_(heap)
            {
                // Binding 0: the draw list the vertex stage reads to place each billboard.
                // Binding 1: the scene depth, sampled by the fragment stage for the occlusion test.
                // Binding 2: the sort keys the sorted-alpha vertex shader indexes through (unused by
                // the additive/billboard draws, whose shader never references it).
                // Bindings 3-7: the clustered-lighting inputs the lit bucket's fragment shades from —
                // the light array, the per-cluster count grid + index list the light-cull pass wrote,
                // the froxel config block, and the environment SH for ambient (unread by unlit draws).
                // Bindings 10-11: the sun's cascade block and atlas, at the scene set's own numbers
                // (free on this set), so the shared shadow_common.glsl declaration is reused as-is.
                // Binding 12: this frame's emitter table, which is where the authored render
                // alignment lives (per-emitter, while the draw list is per-particle).
                // Binding 13: the persistent trail history the ribbon draw walks.
                VkDescriptorSetLayoutBinding bindings[12]{};
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
                bindings[8].binding = SHADOW_BLOCK_BINDING;
                bindings[8].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                bindings[8].descriptorCount = 1;
                bindings[8].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                bindings[9].binding = SHADOW_ATLAS_BINDING;
                bindings[9].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                bindings[9].descriptorCount = 1;
                bindings[9].stageFlags = VK_SHADER_STAGE_FRAGMENT_BIT;
                bindings[10].binding = EMITTER_TABLE_BINDING;
                bindings[10].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[10].descriptorCount = 1;
                bindings[10].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;
                bindings[11].binding = TRAIL_BINDING;
                bindings[11].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                bindings[11].descriptorCount = 1;
                bindings[11].stageFlags = VK_SHADER_STAGE_VERTEX_BIT;

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = 12;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(particle draw)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;
                range.size = sizeof(Push);

                // Set 1 is the bindless texture heap, the same slot and the same layout the scene
                // pipelines use, so a sprite material addresses a texture by the very index a
                // mesh material would. Dropped on a device without descriptor indexing, where the
                // emitter table's RENDER_TEXTURED bit is cleared too and nothing samples it —
                // the same bargain SceneLayout strikes for the mesh path.
                VkDescriptorSetLayout sets[2] = {set_layout_, heap_.layout()};

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = heap_.available() ? 2 : 1;
                pipeline_info.pSetLayouts = sets;
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
                Resources::GraphicsPipelineDescription desc;
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

                // The deterministic billboards: additive like the first pipeline, but through a
                // vertex shader that reads no emitter table — they belong to a host-side pool, not
                // to a GPU emitter, so there is no authored alignment for them to index.
                desc.vertex_shader = shaders_.module("particle_billboard.vert");
                desc.blend.dst_color = VK_BLEND_FACTOR_ONE;
                desc.blend.dst_alpha = VK_BLEND_FACTOR_ONE;
                billboard_pipeline_ = pipelines_.create(desc);

                // Ribbons composite with the premultiplied "over" regardless of the emitter's
                // authored blend: a trail's tail is nearly transparent, where "over" and additive
                // agree, and its head should occlude rather than glow. A second additive ribbon
                // bucket is a later increment if a purely emissive trail ever needs one.
                desc.vertex_shader = shaders_.module("particle_ribbon.vert");
                desc.blend.dst_color = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                desc.blend.dst_alpha = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
                ribbon_pipeline_ = pipelines_.create(desc);
            }

            void ParticlePass::destroy_pipeline()
            {
                // The factory owns the pipelines; the pass drops only its handles.
                pipeline_ = Resources::PipelineHandle{};
                alpha_pipeline_ = Resources::PipelineHandle{};
                billboard_pipeline_ = Resources::PipelineHandle{};
                ribbon_pipeline_ = Resources::PipelineHandle{};
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
                        // The sun's cascades: the lit bucket shadows its sun term against them,
                        // so the atlas has to have finished rendering before this pass samples it.
                        builder.read(frame.targets.shadow, Graph::BufferAccess::UniformRead);
                        builder.read(frame.targets.shadow_atlas,
                                     Graph::TextureAccess::SampledFragment);
                        if (draw_emitters)
                        {
                            builder.read(frame.targets.particle_draw,
                                         Graph::BufferAccess::StorageRead);
                            builder.read(frame.targets.particle_alpha,
                                         Graph::BufferAccess::StorageRead);
                            builder.read(frame.targets.particle_ribbon,
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
                        const VkSampler sampler =
                            frame.samplers->get(Resources::SamplerDescription{});
                        const VkImageView depth_view = context.sampled_view(frame.targets.depth);

                        Push push{};
                        const Matrix4 view_projection =
                            mul(frame.camera->projection, frame.camera->view);
                        for (int i = 0; i < 16; ++i)
                            push.view_projection[i] = static_cast<float>(view_projection.m[i]);
                        // The camera world position rides the spare w lanes (the scene_uniforms
                        // packing), so the vertex stage can subtract it to reach the camera-relative
                        // space the clustered lights live in without spending another vec4. Cast to
                        // float here, so the shading position inherits the pool's float32 precision —
                        // acceptable for the near-camera cosmetic particles this path serves.
                        const Matrix4& view = frame.camera->view;
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

                        // The ambient scale a lit particle applies to the environment SH: the IBL
                        // intensity, or 0 when image-based lighting is off (then lit particles get
                        // only the sun + punctual lights, like a mesh would). Which particles are
                        // lit is the emitter's business, not the draw's — a bucket mixes emitters,
                        // so the flag rides the emitter table and this stays a plain scale.
                        push.sun_radiance[3] =
                            environment.image_based_lighting ? environment.ibl_intensity : 0.0f;

                        // Draws one bucket: pushes, binds its source + pipeline, then issues the
                        // caller's draw.
                        const VkDeviceSize keys_range =
                            static_cast<VkDeviceSize>(particles_.capacity()) * 2 *
                            sizeof(std::uint32_t);
                        auto draw_bucket = [&](Resources::PipelineHandle& pipeline, VkBuffer source,
                                               VkDeviceSize range, VkBuffer keys, auto&& issue)
                        {
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
                            writer.storage_buffer(7, ibl_.sh_buffer(), IBLPass::sh_buffer_bytes());
                            // The sun's cascades. Bound on every bucket for the same reason: only
                            // the lit fragment path samples them.
                            writer.uniform_buffer(SHADOW_BLOCK_BINDING,
                                                  context.buffer(frame.targets.shadow),
                                                  sizeof(Scene::ShadowUniforms));
                            writer.sampled_image(SHADOW_ATLAS_BINDING,
                                                 context.sampled_view(frame.targets.shadow_atlas),
                                                 ShadowPass::atlas_sampler(*frame.samplers));
                            // The emitter table carries the authored alignment. On a frame with
                            // only deterministic billboards there is no table at all; that draw's
                            // shader declares no emitter buffer, so the source stands in as an
                            // inert binding (the same idiom as the sort keys above).
                            const VkDeviceSize table_range = particles_.emitter_range();
                            writer.storage_buffer(EMITTER_TABLE_BINDING,
                                                  table_range > 0
                                                      ? particles_.emitter_buffer(slot)
                                                      : source,
                                                  table_range > 0 ? table_range : range);
                            // The ribbon draw's trail history: persistent, so always valid.
                            writer.storage_buffer(TRAIL_BINDING, particles_.trail_buffer(),
                                                  particles_.trail_range());
                            writer.update(device_.device(), set);
                            vkCmdBindPipeline(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline.get());
                            Resources::bind_descriptor_set(cmd, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                           pipeline_layout_, 0, set);
                            // The sprite materials' textures. Bound per bucket rather than once
                            // per pass because the set is bound against this pass's own pipeline
                            // layout, which only exists here.
                            if (heap_.available())
                                Resources::bind_descriptor_set(cmd,
                                                               VK_PIPELINE_BIND_POINT_GRAPHICS,
                                                               pipeline_layout_, HEAP_SET,
                                                               heap_.set());
                            issue();
                        };

                        if (draw_emitters)
                        {
                            const VkBuffer args = context.buffer(frame.targets.particle_args);
                            draw_bucket(pipeline_, context.buffer(frame.targets.particle_draw),
                                        particles_.pool_range(), VK_NULL_HANDLE, [&]() {
                                            vkCmdDrawIndirect(cmd, args, 0, 1,
                                                              sizeof(VkDrawIndirectCommand));
                                        });
                            draw_bucket(alpha_pipeline_, context.buffer(frame.targets.particle_alpha),
                                        particles_.pool_range(),
                                        context.buffer(frame.targets.particle_sort_keys), [&]() {
                                            vkCmdDrawIndirect(cmd, args, sizeof(VkDrawIndirectCommand),
                                                              1, sizeof(VkDrawIndirectCommand));
                                        });
                            // Ribbons last of the emitter buckets: they are the most opaque of the
                            // three, so drawing them over the glow reads correctly without a sort.
                            draw_bucket(ribbon_pipeline_,
                                        context.buffer(frame.targets.particle_ribbon),
                                        particles_.pool_range(), VK_NULL_HANDLE, [&]() {
                                            vkCmdDrawIndirect(cmd, args,
                                                              2 * sizeof(VkDrawIndirectCommand), 1,
                                                              sizeof(VkDrawIndirectCommand));
                                        });
                        }
                        if (draw_billboards)
                        {
                            draw_bucket(billboard_pipeline_, particles_.billboard_buffer(slot),
                                        particles_.billboard_range(), VK_NULL_HANDLE,
                                        [&]() { vkCmdDraw(cmd, 6, particles_.billboard_count(), 0, 0); });
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
