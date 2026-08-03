/**************************************************************************/
/* particle_sim_pass.cpp                                                  */
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

#include "passes/particle_sim_pass.hpp"

#include <cstring>
#include <vector>

#include <SushiEngine/core/types.hpp>

#include "frame/frame_context.hpp"
#include "gi/sdf_clipmap.hpp"
#include "graph/render_graph.hpp"
#include "passes/hiz_pass.hpp"
#include "passes/irradiance_volume_pass.hpp"
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
            namespace
            {
                constexpr std::uint32_t GROUP_SIZE = 64;

                std::uint32_t groups(std::uint32_t value) noexcept
                {
                    return value == 0 ? 1u : (value + GROUP_SIZE - 1) / GROUP_SIZE;
                }

                void memory_barrier(VkCommandBuffer command, VkPipelineStageFlags2 source_stage,
                                    VkAccessFlags2 source_access,
                                    VkPipelineStageFlags2 destination_stage,
                                    VkAccessFlags2 destination_access)
                {
                    VkMemoryBarrier2 barrier{};
                    barrier.sType = VK_STRUCTURE_TYPE_MEMORY_BARRIER_2;
                    barrier.srcStageMask = source_stage;
                    barrier.srcAccessMask = source_access;
                    barrier.dstStageMask = destination_stage;
                    barrier.dstAccessMask = destination_access;

                    VkDependencyInfo dependency{};
                    dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                    dependency.memoryBarrierCount = 1;
                    dependency.pMemoryBarriers = &barrier;
                    vkCmdPipelineBarrier2(command, &dependency);
                }
            } // namespace

            ParticleSimPass::ParticleSimPass(Vulkan::VulkanDevice& device,
                                             Resources::ShaderLibrary& shaders,
                                             Resources::GraphicsPipelineFactory& pipelines,
                                             Scene::ParticleSystem& particles, HiZPass& hiz,
                                             IrradianceVolumePass& volumes)
                : device_(device), shaders_(shaders), pipelines_(pipelines), particles_(particles),
                  hiz_(hiz), volumes_(volumes)
            {
                // Eleven storage buffers: pool, emitter table, additive draw list, indirect args,
                // curve LUTs, gradient LUTs, alpha draw list, ribbon draw list, trail history,
                // mesh draw list, mesh indirect args. Shared by emit and simulate.
                // Plus binding 11: last frame's depth pyramid, the on-screen collision surface;
                // binding 12: the GI distance field, the off-screen one; binding 13: that field's
                // camera-relative parameterization.
                VkDescriptorSetLayoutBinding bindings[BINDING_COUNT]{};
                for (std::uint32_t i = 0; i < BINDING_COUNT; ++i)
                {
                    bindings[i].binding = i;
                    if (i == DEPTH_PYRAMID_BINDING || i == SDF_CLIPMAP_BINDING)
                        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER;
                    else if (i == SDF_CONFIG_BINDING)
                        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_UNIFORM_BUFFER;
                    else
                        bindings[i].descriptorType = VK_DESCRIPTOR_TYPE_STORAGE_BUFFER;
                    bindings[i].descriptorCount = 1;
                    bindings[i].stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                }

                VkDescriptorSetLayoutCreateInfo layout_info{};
                layout_info.sType = VK_STRUCTURE_TYPE_DESCRIPTOR_SET_LAYOUT_CREATE_INFO;
                layout_info.bindingCount = BINDING_COUNT;
                layout_info.pBindings = bindings;
                Vulkan::check(vkCreateDescriptorSetLayout(device_.device(), &layout_info, nullptr,
                                                          &set_layout_),
                              "vkCreateDescriptorSetLayout(particle sim)");

                VkPushConstantRange range{};
                range.stageFlags = VK_SHADER_STAGE_COMPUTE_BIT;
                range.size = sizeof(Push);

                VkPipelineLayoutCreateInfo pipeline_info{};
                pipeline_info.sType = VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO;
                pipeline_info.setLayoutCount = 1;
                pipeline_info.pSetLayouts = &set_layout_;
                pipeline_info.pushConstantRangeCount = 1;
                pipeline_info.pPushConstantRanges = &range;
                Vulkan::check(vkCreatePipelineLayout(device_.device(), &pipeline_info, nullptr,
                                                     &pipeline_layout_),
                              "vkCreatePipelineLayout(particle sim)");

                create_fallback_depth();
                create_fallback_field();
                create_pipelines();
            }

            ParticleSimPass::~ParticleSimPass()
            {
                destroy_fallback_depth();
                destroy_fallback_field();
                destroy_pipelines();
                if (pipeline_layout_ != VK_NULL_HANDLE)
                    vkDestroyPipelineLayout(device_.device(), pipeline_layout_, nullptr);
                if (set_layout_ != VK_NULL_HANDLE)
                    vkDestroyDescriptorSetLayout(device_.device(), set_layout_, nullptr);
            }

            void ParticleSimPass::create_fallback_depth()
            {
                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_2D;
                image_info.format = VK_FORMAT_R32_SFLOAT;
                image_info.extent = {1, 1, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
                image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

                VmaAllocationCreateInfo allocation_info{};
                allocation_info.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &allocation_info,
                                             &fallback_image_, &fallback_allocation_, nullptr),
                              "vmaCreateImage(particle sim fallback depth)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = fallback_image_;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_2D;
                view_info.format = VK_FORMAT_R32_SFLOAT;
                view_info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                Vulkan::check(
                    vkCreateImageView(device_.device(), &view_info, nullptr, &fallback_view_),
                    "vkCreateImageView(particle sim fallback depth)");
            }

            void ParticleSimPass::destroy_fallback_depth()
            {
                if (fallback_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), fallback_view_, nullptr);
                if (fallback_image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), fallback_image_, fallback_allocation_);
                fallback_view_ = VK_NULL_HANDLE;
                fallback_image_ = VK_NULL_HANDLE;
                fallback_allocation_ = VK_NULL_HANDLE;
            }

            void ParticleSimPass::create_fallback_field()
            {
                VkImageCreateInfo image_info{};
                image_info.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
                image_info.imageType = VK_IMAGE_TYPE_3D;
                image_info.format = VK_FORMAT_R16G16B16A16_SFLOAT;
                image_info.extent = {1, 1, 1};
                image_info.mipLevels = 1;
                image_info.arrayLayers = 1;
                image_info.samples = VK_SAMPLE_COUNT_1_BIT;
                image_info.tiling = VK_IMAGE_TILING_OPTIMAL;
                image_info.usage = VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT;
                image_info.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;

                VmaAllocationCreateInfo allocation_info{};
                allocation_info.usage = VMA_MEMORY_USAGE_AUTO;
                Vulkan::check(vmaCreateImage(device_.allocator(), &image_info, &allocation_info,
                                             &fallback_field_image_, &fallback_field_allocation_,
                                             nullptr),
                              "vmaCreateImage(particle sim fallback field)");

                VkImageViewCreateInfo view_info{};
                view_info.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
                view_info.image = fallback_field_image_;
                view_info.viewType = VK_IMAGE_VIEW_TYPE_3D;
                view_info.format = VK_FORMAT_R16G16B16A16_SFLOAT;
                view_info.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                Vulkan::check(
                    vkCreateImageView(device_.device(), &view_info, nullptr, &fallback_field_view_),
                    "vkCreateImageView(particle sim fallback field)");

                // Zero-filled: a config whose resolution is zero makes every position fall
                // outside the clipmap, so even a shader that ignored the usable flag would read
                // nothing rather than bounce off a phantom surface.
                VkBufferCreateInfo buffer_info{};
                buffer_info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                buffer_info.size = sizeof(GI::SDFClipmapConfiguration);
                buffer_info.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                buffer_info.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

                VmaAllocationCreateInfo config_allocation{};
                config_allocation.usage = VMA_MEMORY_USAGE_AUTO;
                config_allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                          VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo mapped{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &buffer_info, &config_allocation,
                                              &fallback_config_, &fallback_config_allocation_,
                                              &mapped),
                              "vmaCreateBuffer(particle sim fallback config)");
                if (mapped.pMappedData != nullptr)
                    std::memset(mapped.pMappedData, 0, sizeof(GI::SDFClipmapConfiguration));
            }

            void ParticleSimPass::destroy_fallback_field()
            {
                if (fallback_config_ != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), fallback_config_,
                                     fallback_config_allocation_);
                if (fallback_field_view_ != VK_NULL_HANDLE)
                    vkDestroyImageView(device_.device(), fallback_field_view_, nullptr);
                if (fallback_field_image_ != VK_NULL_HANDLE)
                    vmaDestroyImage(device_.allocator(), fallback_field_image_,
                                    fallback_field_allocation_);
                fallback_config_ = VK_NULL_HANDLE;
                fallback_config_allocation_ = VK_NULL_HANDLE;
                fallback_field_view_ = VK_NULL_HANDLE;
                fallback_field_image_ = VK_NULL_HANDLE;
                fallback_field_allocation_ = VK_NULL_HANDLE;
            }

            void ParticleSimPass::create_pipelines()
            {
                emit_pipeline_ =
                    pipelines_.create_compute(pipeline_layout_, shaders_.module("particle_emit.comp"));
                simulate_pipeline_ = pipelines_.create_compute(
                    pipeline_layout_, shaders_.module("particle_simulate.comp"));
            }

            void ParticleSimPass::destroy_pipelines()
            {
                if (emit_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), emit_pipeline_, nullptr);
                if (simulate_pipeline_ != VK_NULL_HANDLE)
                    vkDestroyPipeline(device_.device(), simulate_pipeline_, nullptr);
                emit_pipeline_ = VK_NULL_HANDLE;
                simulate_pipeline_ = VK_NULL_HANDLE;
            }

            void ParticleSimPass::rebuild_pipelines()
            {
                destroy_pipelines();
                create_pipelines();
            }

            void ParticleSimPass::register_pass(Graph::RenderGraph& graph,
                                                const Frame::FrameContext& frame)
            {
                if (particles_.empty() || !frame.quality.gpu_particles)
                    return;

                const std::uint32_t slot = frame.slot;
                const std::uint32_t capacity = particles_.capacity();
                const float dt =
                    frame.draws.emitter_count > 0 ? frame.draws.emitters[0].dt : 0.0f;

                graph.add_pass(
                    "particle sim",
                    [&frame](Graph::RenderPassBuilder& builder)
                    {
                        builder.write(frame.targets.particle_draw, Graph::BufferAccess::StorageWrite);
                        builder.write(frame.targets.particle_alpha, Graph::BufferAccess::StorageWrite);
                        builder.write(frame.targets.particle_ribbon, Graph::BufferAccess::StorageWrite);
                        builder.write(frame.targets.particle_mesh, Graph::BufferAccess::StorageWrite);
                        builder.write(frame.targets.particle_mesh_args,
                                      Graph::BufferAccess::StorageWrite);
                        builder.write(frame.targets.particle_args, Graph::BufferAccess::StorageWrite);
                    },
                    [this, &frame, slot, capacity, dt](VkCommandBuffer command,
                                                       const Graph::PassContext& context)
                    {
                        const VkBuffer draw_buffer = context.buffer(frame.targets.particle_draw);
                        const VkBuffer alpha_buffer = context.buffer(frame.targets.particle_alpha);
                        const VkBuffer ribbon_buffer = context.buffer(frame.targets.particle_ribbon);
                        const VkBuffer mesh_buffer = context.buffer(frame.targets.particle_mesh);
                        const VkBuffer mesh_args_buffer =
                            context.buffer(frame.targets.particle_mesh_args);
                        const VkBuffer args_buffer = context.buffer(frame.targets.particle_args);

                        // The stand-ins have to be in a samplable layout before they are bound, even
                        // though the shader never reads them — the descriptors are written regardless.
                        const auto make_samplable = [&command](VkImage image)
                        {
                            VkImageMemoryBarrier2 barrier{};
                            barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER_2;
                            barrier.srcStageMask = VK_PIPELINE_STAGE_2_TOP_OF_PIPE_BIT;
                            barrier.dstStageMask = VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT;
                            barrier.dstAccessMask = VK_ACCESS_2_SHADER_SAMPLED_READ_BIT;
                            barrier.oldLayout = VK_IMAGE_LAYOUT_UNDEFINED;
                            barrier.newLayout = VK_IMAGE_LAYOUT_GENERAL;
                            barrier.image = image;
                            barrier.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
                            VkDependencyInfo dependency{};
                            dependency.sType = VK_STRUCTURE_TYPE_DEPENDENCY_INFO;
                            dependency.imageMemoryBarrierCount = 1;
                            dependency.pImageMemoryBarriers = &barrier;
                            vkCmdPipelineBarrier2(command, &dependency);
                        };
                        if (!fallback_ready_)
                        {
                            make_samplable(fallback_image_);
                            fallback_ready_ = true;
                        }
                        if (!fallback_field_ready_)
                        {
                            make_samplable(fallback_field_image_);
                            fallback_field_ready_ = true;
                        }

                        // Zero the device-local pool and trail history exactly once, so every slot
                        // starts dead and no ribbon reads an uninitialised sample.
                        if (particles_.needs_clear())
                        {
                            vkCmdFillBuffer(command, particles_.pool(), 0, VK_WHOLE_SIZE, 0);
                            vkCmdFillBuffer(command, particles_.trail_buffer(), 0, VK_WHOLE_SIZE,
                                            0);
                            memory_barrier(command, VK_PIPELINE_STAGE_2_ALL_TRANSFER_BIT,
                                           VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                           VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                           VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                               VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);
                            particles_.mark_cleared();
                        }

                        // Reset the three indirect draws (additive at [0..3], alpha at [4..7],
                        // ribbons at [8..11]) to their vertex counts and zero instances, then make
                        // the reset visible to the atomics. A sprite is one quad; a ribbon is a
                        // whole strip, which is why it needs a draw of its own.
                        const std::uint32_t initial_args[12] = {
                            6u, 0u, 0u, 0u,
                            6u, 0u, 0u, 0u,
                            Scene::ParticleSystem::RIBBON_VERTICES, 0u, 0u, 0u};
                        vkCmdUpdateBuffer(command, args_buffer, 0, sizeof(initial_args),
                                          initial_args);

                        // The mesh slices' indexed commands. Their index count is a host fact (the
                        // mesh the emitter authored), which is why the sim pass seeds the whole
                        // command and the compaction only bumps the instance count. Unclaimed
                        // slices are zeroed, so their draw is a no-op.
                        std::uint32_t mesh_args[5 * Scene::ParticleSystem::MAX_MESH_EMITTERS] = {};
                        for (const Scene::ParticleSystem::MeshDraw& draw : particles_.mesh_draws())
                            mesh_args[draw.slot * 5] = draw.index_count;
                        vkCmdUpdateBuffer(command, mesh_args_buffer, 0, sizeof(mesh_args),
                                          mesh_args);

                        memory_barrier(command, VK_PIPELINE_STAGE_2_ALL_TRANSFER_BIT,
                                       VK_ACCESS_2_TRANSFER_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        const VkDescriptorSet set = frame.descriptors->allocate(set_layout_);
                        Resources::DescriptorWriter writer;
                        writer.storage_buffer(0, particles_.pool(), particles_.pool_range());
                        writer.storage_buffer(1, particles_.emitter_buffer(slot),
                                              particles_.emitter_range());
                        writer.storage_buffer(2, draw_buffer, particles_.pool_range());
                        writer.storage_buffer(3, args_buffer, sizeof(initial_args));
                        writer.storage_buffer(4, particles_.curve_lut_buffer(slot),
                                              particles_.curve_lut_range());
                        writer.storage_buffer(5, particles_.gradient_lut_buffer(slot),
                                              particles_.gradient_lut_range());
                        writer.storage_buffer(6, alpha_buffer, particles_.pool_range());
                        writer.storage_buffer(7, ribbon_buffer, particles_.pool_range());
                        writer.storage_buffer(8, particles_.trail_buffer(),
                                              particles_.trail_range());
                        writer.storage_buffer(9, mesh_buffer, particles_.pool_range());
                        writer.storage_buffer(10, mesh_args_buffer, sizeof(mesh_args));
                        // Last frame's depth, or the stand-in when there is none to read.
                        const bool depth_usable = hiz_.valid() && hiz_.has_history();
                        // The pyramid stays in GENERAL across its own build (see hiz_pass.cpp);
                        // the fallback stand-in is the normal SHADER_READ_ONLY_OPTIMAL image.
                        writer.sampled_image(
                            DEPTH_PYRAMID_BINDING,
                            depth_usable ? hiz_.pyramid_view() : fallback_view_,
                            frame.samplers->get(Resources::SamplerDescription{}),
                            depth_usable ? VK_IMAGE_LAYOUT_GENERAL
                                         : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

                        // The GI distance field, or the stand-in when the GI tier is off. Its
                        // config comes from the same record, so the field and the parameterization
                        // locating it can never come from different frames.
                        const GI::VisibilityField field = volumes_.visibility_field(frame.index);
                        const bool field_usable = field.valid();
                        // The clipmap stays in GENERAL across its own build (see
                        // sdf_probe_tracer.cpp); the fallback stand-in is SHADER_READ_ONLY_OPTIMAL.
                        writer.sampled_image(SDF_CLIPMAP_BINDING,
                                             field_usable ? field.distance_field
                                                          : fallback_field_view_,
                                             frame.samplers->get(Resources::SamplerDescription{}),
                                             field_usable ? VK_IMAGE_LAYOUT_GENERAL
                                                          : VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
                        writer.uniform_buffer(SDF_CONFIG_BINDING,
                                              field_usable ? field.config : fallback_config_,
                                              field_usable ? field.config_bytes
                                                           : sizeof(GI::SDFClipmapConfiguration));
                        writer.update(device_.device(), set);
                        Resources::bind_descriptor_set(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                                       pipeline_layout_, 0, set);

                        // The camera block the collision projects through. The half-fov tangents come
                        // straight out of the projection's diagonal, which is what turns a pixel and
                        // a linear depth back into a camera-relative position.
                        Push push{};
                        push.counts[1] = capacity;
                        push.counts[2] = depth_usable ? 1u : 0u;
                        push.counts[3] = field_usable ? 1u : 0u;
                        push.misc[0] = dt;
                        // The eye, which rebases a particle's absolute world position into the
                        // distance field's camera-relative space.
                        push.misc[1] = static_cast<float>(frame.eye[0]);
                        push.misc[2] = static_cast<float>(frame.eye[1]);
                        push.misc[3] = static_cast<float>(frame.eye[2]);
                        if (frame.camera != nullptr)
                        {
                            const Matrix4 view_projection =
                                mul(frame.camera->projection, frame.camera->view);
                            for (int i = 0; i < 16; ++i)
                                push.view_projection[i] = static_cast<float>(view_projection.m[i]);
                            const Matrix4& view = frame.camera->view;
                            push.camera_right[0] = static_cast<float>(view.m[0]);
                            push.camera_right[1] = static_cast<float>(view.m[4]);
                            push.camera_right[2] = static_cast<float>(view.m[8]);
                            push.camera_up[0] = static_cast<float>(view.m[1]);
                            push.camera_up[1] = static_cast<float>(view.m[5]);
                            push.camera_up[2] = static_cast<float>(view.m[9]);
                            const double focal_x = frame.camera->projection.m[0];
                            const double focal_y = frame.camera->projection.m[5];
                            push.camera_right[3] =
                                focal_x != 0.0 ? static_cast<float>(1.0 / focal_x) : 1.0f;
                            push.camera_up[3] =
                                focal_y != 0.0 ? static_cast<float>(1.0 / (focal_y < 0.0 ? -focal_y
                                                                                         : focal_y))
                                               : 1.0f;
                        }
                        else
                        {
                            push.counts[2] = 0u; // no camera, so nothing to project into
                        }

                        // Advance the existing particles first, over the whole pool.
                        vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE,
                                          simulate_pipeline_);
                        Push simulate_push = push;
                        vkCmdPushConstants(command, pipeline_layout_, VK_SHADER_STAGE_COMPUTE_BIT,
                                           0, sizeof(Push), &simulate_push);
                        vkCmdDispatch(command, groups(capacity), 1, 1);

                        // Order the emit pass after the sweep so it wins the recycled ring slots.
                        memory_barrier(command, VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT,
                                       VK_PIPELINE_STAGE_2_COMPUTE_SHADER_BIT,
                                       VK_ACCESS_2_SHADER_STORAGE_READ_BIT |
                                           VK_ACCESS_2_SHADER_STORAGE_WRITE_BIT);

                        // Then spawn each emitter's new particles into the ring.
                        vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_COMPUTE, emit_pipeline_);
                        const std::vector<Scene::GPUEmitter>& emitters = particles_.emitters();
                        for (std::uint32_t e = 0; e < emitters.size(); ++e)
                        {
                            if (emitters[e].spawn_count == 0)
                                continue;
                            Push emit_push = push;
                            emit_push.counts[0] = e;
                            vkCmdPushConstants(command, pipeline_layout_,
                                               VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(Push),
                                               &emit_push);
                            vkCmdDispatch(command, groups(emitters[e].spawn_count), 1, 1);
                        }
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
