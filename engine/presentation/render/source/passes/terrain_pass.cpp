/**************************************************************************/
/* terrain_pass.cpp                                                       */
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

#include "passes/terrain_pass.hpp"

#include <cstring>
#include <vector>

#include <SushiEngine/material/material.hpp>

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"
#include "material/material_system.hpp"
#include "passes/shading_set.hpp"
#include "resources/pipeline_cache.hpp"
#include "resources/sampler_cache.hpp"
#include "resources/shader_library.hpp"
#include "rhi/vulkan/vulkan_check.hpp"
#include "rhi/vulkan/vulkan_device.hpp"
#include "scene/scene_layout.hpp"
#include "terrain/planet_terrain.hpp"
#include "terrain/terrain_layout.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            TerrainPass::TerrainPass(Vulkan::VulkanDevice& device,
                                     Resources::ShaderLibrary& shaders,
                                     Resources::GraphicsPipelineFactory& pipelines,
                                     Terrain::TerrainLayout& layout,
                                     Terrain::PlanetTerrain& terrain, IBLPass& ibl,
                                     CloudShadowMapPass& cloud_shadow, IrradianceVolumePass& gi,
                                     Assets::MaterialSystem& materials, Scene::MotionSystem& motion,
                                     Lighting::LightSystem& lights)
                : device_(device), shaders_(shaders), pipelines_(pipelines), layout_(layout),
                  terrain_(terrain), ibl_(ibl), cloud_shadow_(cloud_shadow), gi_(gi),
                  materials_(materials), motion_(motion), lights_(lights)
            {
                if (!layout_.available())
                    return;
                create_lattice();
                create_pipeline();
            }

            TerrainPass::~TerrainPass()
            {
                if (indices_ != VK_NULL_HANDLE)
                    vmaDestroyBuffer(device_.allocator(), indices_, indices_allocation_);
            }

            void TerrainPass::create_lattice()
            {
                // One lattice for every node of every body: the geometry differs entirely in
                // the per-instance record, so there is exactly one index buffer in the whole
                // terrain system and it is 12 KB.
                std::vector<std::uint16_t> lattice;
                lattice.reserve(LATTICE_INDICES);
                for (std::uint32_t row = 0; row + 1 < LATTICE_VERTICES; ++row)
                {
                    for (std::uint32_t column = 0; column + 1 < LATTICE_VERTICES; ++column)
                    {
                        const std::uint16_t base =
                            static_cast<std::uint16_t>(row * LATTICE_VERTICES + column);
                        const std::uint16_t right = static_cast<std::uint16_t>(base + 1);
                        const std::uint16_t below =
                            static_cast<std::uint16_t>(base + LATTICE_VERTICES);
                        const std::uint16_t diagonal = static_cast<std::uint16_t>(below + 1);
                        lattice.push_back(base);
                        lattice.push_back(below);
                        lattice.push_back(right);
                        lattice.push_back(right);
                        lattice.push_back(below);
                        lattice.push_back(diagonal);
                    }
                }

                const VkDeviceSize bytes =
                    static_cast<VkDeviceSize>(lattice.size()) * sizeof(std::uint16_t);

                // Host-visible rather than staged: 12 KB written once at bring-up is not
                // worth a transfer submit, and the buffer is read as indices for the life of
                // the process.
                VkBufferCreateInfo info{};
                info.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
                info.size = bytes;
                info.usage = VK_BUFFER_USAGE_INDEX_BUFFER_BIT;

                VmaAllocationCreateInfo allocation{};
                allocation.usage = VMA_MEMORY_USAGE_AUTO;
                allocation.flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT |
                                   VMA_ALLOCATION_CREATE_MAPPED_BIT;
                VmaAllocationInfo mapped{};
                Vulkan::check(vmaCreateBuffer(device_.allocator(), &info, &allocation, &indices_,
                                              &indices_allocation_, &mapped),
                              "vmaCreateBuffer(terrain lattice)");
                std::memcpy(mapped.pMappedData, lattice.data(), static_cast<std::size_t>(bytes));
            }

            void TerrainPass::create_pipeline()
            {
                Resources::GraphicsPipelineDescription desc{};
                desc.layout = layout_.pipeline_layout();
                desc.vertex_shader = shaders_.module("terrain.vert");
                // The shared shading path, unchanged: terrain.vert's output signature is
                // mesh.vert's, so pbr.frag cannot tell the two apart.
                desc.fragment_shader = shaders_.module("pbr.frag");
                desc.vertex_stride = 0; // the lattice position comes from gl_VertexIndex
                desc.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST;
                // Off until a rendered frame confirms the winding; see the header.
                desc.cull_mode = VK_CULL_MODE_NONE;
                desc.depth_test = VK_TRUE;
                desc.depth_write = VK_TRUE;
                desc.depth_compare = VK_COMPARE_OP_GREATER_OR_EQUAL; // reverse-Z
                desc.color_count = 4;
                desc.color_formats[0] = Frame::HDR_FORMAT;
                desc.color_formats[1] = Frame::ID_FORMAT;
                desc.color_formats[2] = Frame::VELOCITY_FORMAT;
                desc.color_formats[3] = Frame::GBUFFER_FORMAT;
                desc.depth_format = Frame::DEPTH_FORMAT;
                desc.stencil_format = Frame::DEPTH_FORMAT;
                pipeline_ = pipelines_.create(desc);
            }

            void TerrainPass::destroy_pipeline() { pipeline_ = Resources::PipelineHandle{}; }

            void TerrainPass::rebuild_pipelines()
            {
                if (!layout_.available())
                    return;
                destroy_pipeline();
                create_pipeline();
            }

            void TerrainPass::register_pass(Graph::RenderGraph& graph,
                                            const Frame::FrameContext& frame)
            {
                // One question, asked of the object that knows the answer: a pack loaded, a
                // pool created, and a selection that produced nodes. Anything less and
                // there is nothing to import, let alone draw.
                if (!layout_.available() || !terrain_.drawing())
                    return;

                Terrain::TileCache& cache = terrain_.cache();

                // One material for the whole body, pushed here for exactly the reason the
                // deformable surfaces push theirs: `pbr.frag` reads the material array *by
                // index*, and an index into an array nothing was pushed to does not read a
                // default material — it reads whatever those bytes held. Until §11's class
                // tiles land, the colour is the one the analytic ground would have used, so
                // handing over to terrain does not change what the body looks like.
                Render::Material surface;
                surface.albedo = frame.environment->surface.ground_albedo;
                surface.roughness = frame.environment->surface.roughness;
                surface.metallic = 0.0f;
                const std::uint32_t material_index = materials_.push(surface);

                Graph::ImportedTexture imported{};
                imported.image = cache.image();
                imported.view = cache.view();
                imported.sample_view = cache.view();
                imported.desc = cache.description();
                imported.state = &cache.state();
                const Graph::TextureHandle slots = graph.import_texture(imported);

                // The node array and the body block as host-visible transients: both are
                // rewritten every frame and read once, so a device-local copy would buy a
                // transfer and nothing else.
                Graph::BufferDescription node_desc{};
                node_desc.size = terrain_.node_bytes();
                node_desc.usage = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT;
                node_desc.host_visible = true;
                node_desc.name = "terrain nodes";
                const Graph::BufferHandle nodes = graph.create_buffer(node_desc);

                Graph::BufferDescription body_desc{};
                body_desc.size = sizeof(Terrain::TerrainBodyRecord);
                body_desc.usage = VK_BUFFER_USAGE_UNIFORM_BUFFER_BIT;
                body_desc.host_visible = true;
                body_desc.name = "terrain body";
                const Graph::BufferHandle body = graph.create_buffer(body_desc);

                if (cache.pending_uploads() > 0)
                {
                    graph.add_pass(
                        "terrain tile upload",
                        [slots](Graph::RenderPassBuilder& builder)
                        { builder.write(slots, Graph::TextureAccess::TransferDestination); },
                        [&cache](VkCommandBuffer command, const Graph::PassContext&)
                        { cache.record_uploads(command); });
                }

                const std::uint32_t node_count = terrain_.node_count();
                graph.add_pass(
                    "terrain",
                    [&, slots, nodes, body](Graph::RenderPassBuilder& builder)
                    {
                        builder.color_attachment(0, frame.targets.hdr,
                                                 Graph::AttachmentLoad::Load);
                        builder.color_attachment(1, frame.targets.id,
                                                 Graph::AttachmentLoad::Load);
                        builder.color_attachment(2, frame.targets.velocity,
                                                 Graph::AttachmentLoad::Load);
                        builder.color_attachment(3, frame.targets.gbuffer,
                                                 Graph::AttachmentLoad::Load);
                        builder.depth_stencil_attachment(frame.targets.depth,
                                                         Graph::AttachmentLoad::Load);
                        // The vertex stage samples this to place geometry, which is a
                        // different barrier from a fragment-stage read: the vertex stage of
                        // this very draw runs first.
                        builder.read(slots, Graph::TextureAccess::SampledVertex);
                        // Read, not written, even though this pass fills both from the
                        // host: a host write to mapped memory made before the submit is
                        // already visible to the device, so there is no barrier to derive
                        // and claiming to produce them would only make this pass a producer
                        // of two resources nobody reads.
                        builder.read(nodes, Graph::BufferAccess::StorageRead);
                        builder.read(body, Graph::BufferAccess::UniformRead);
                        // Terrain shades through pbr.frag, so it reads everything pbr.frag
                        // reads — the whole of set 0, not merely the two blocks its own
                        // vertex shader wants.
                        declare_shading_set(builder, frame);
                    },
                    [this, &frame, slots, nodes, body, node_count, material_index](
                        VkCommandBuffer command, const Graph::PassContext& context)
                    {
                        if (void* mapped = context.mapped(nodes))
                            std::memcpy(mapped, terrain_.node_records(), terrain_.node_bytes());
                        if (void* mapped = context.mapped(body))
                            std::memcpy(mapped, &terrain_.body_record(),
                                        sizeof(Terrain::TerrainBodyRecord));

                        // Set 0 in full. Writing only what terrain.vert reads would leave
                        // the rest of this push-descriptor set undefined, and pbr.frag
                        // samples it regardless — which costs the device, not a pixel.
                        const ShadingSetSources sources{ibl_,       cloud_shadow_, gi_,
                                                        materials_, motion_,       lights_};
                        Scene::SceneSetWriter scene;
                        write_shading_set(scene, sources, frame, context);
                        scene.commit(command, layout_.pipeline_layout());

                        Resources::DescriptorWriter terrain_set;
                        terrain_set.storage_buffer(Terrain::TerrainLayout::NODE_BINDING,
                                                   context.buffer(nodes), terrain_.node_bytes());
                        terrain_set.sampled_image(
                            Terrain::TerrainLayout::HEIGHT_BINDING, context.sampled_view(slots),
                            frame.samplers->get(Resources::SamplerDescription{}));
                        terrain_set.uniform_buffer(Terrain::TerrainLayout::BODY_BINDING,
                                                   context.buffer(body),
                                                   sizeof(Terrain::TerrainBodyRecord));
                        terrain_set.push(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                         layout_.pipeline_layout(),
                                         Terrain::TerrainLayout::TERRAIN_SET);

                        layout_.bind_heap(command);
                        vkCmdBindPipeline(command, VK_PIPELINE_BIND_POINT_GRAPHICS,
                                          pipeline_.get());

                        Terrain::TerrainPushConstants push{};
                        push.material_index = material_index;
                        // entity_id stays zero, which is NO_PICK: clicking the ground picks
                        // nothing rather than picking whichever entity owns id zero.
                        vkCmdPushConstants(command, layout_.pipeline_layout(),
                                           VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(push), &push);

                        vkCmdBindIndexBuffer(command, indices_, 0, VK_INDEX_TYPE_UINT16);
                        vkCmdDrawIndexed(command, LATTICE_INDICES, node_count, 0, 0, 0);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
