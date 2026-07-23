/**************************************************************************/
/* mesh_registry.hpp                                                      */
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
 * @file mesh_registry.hpp
 * @brief The device's mesh asset store: built-in primitives and imported meshes.
 *
 * The vertex format carries position, normal, tangent, two UV sets, and a vertex
 * colour — enough for normal mapping, parallax, and a detail set, which is what
 * makes the material system possible. Built-in primitives are generated with
 * analytic tangents and UVs; imported meshes arrive through add_mesh(). The registry
 * is an asset store shared by every view on the device, so it holds nothing that
 * varies per frame — the per-frame soft-body geometry lives in ClothBuffers.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/material.hpp>
#include <SushiEngine/render/scene_view.hpp>

#include "gi/mesh_sdf_baker.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Geometry
        {
            /**
             * @brief The renderer's single vertex format, 60 bytes.
             *
             * @c tangent's w is the bitangent handedness (+1 or -1). A zero tangent is
             * legal and means "none authored": the shader then derives a tangent frame
             * from screen-space derivatives, so a mesh without tangents still normal-maps.
             */
            struct MeshVertex
            {
                float position[3];
                float normal[3];
                float tangent[4];
                float uv0[2];
                float uv1[2];
                std::uint8_t color[4];
            };

            /** @brief One drawable mesh: its buffers and the counts to draw them with. */
            struct Mesh
            {
                VkBuffer vertices = VK_NULL_HANDLE;
                VkBuffer indices = VK_NULL_HANDLE;
                std::uint32_t vertex_count = 0;
                std::uint32_t index_count = 0;
                /**
                 * @brief Local bounding radius: the farthest vertex from the mesh's origin.
                 *
                 * A sphere at the local origin of this radius contains the whole mesh no
                 * matter where the mesh is centred, so it stays a valid bound after any
                 * transform. The GPU-driven cull scales it by the model's largest column to
                 * get the world bound it frustum- and occlusion-tests, so a conservatively
                 * large radius never culls a visible object — it only culls a bit less.
                 */
                float radius = 0.0f;

                /**
                 * @brief The mesh's meshlet buffers, for the mesh-shader draw path.
                 *
                 * Null and zero when the device cannot draw with mesh shaders (the classic
                 * path never touches them). When present: @c meshlet_descriptors is one
                 * @c MeshletDescriptor per meshlet, @c meshlet_vertices the global vertex
                 * indices they reference, @c meshlet_triangles the packed local triangle
                 * indices — the three storage buffers the task and mesh shaders read.
                 */
                VkBuffer meshlet_descriptors = VK_NULL_HANDLE;
                VkBuffer meshlet_vertices = VK_NULL_HANDLE;
                VkBuffer meshlet_triangles = VK_NULL_HANDLE;
                std::uint32_t meshlet_count = 0;
            };

            /**
             * @brief The local scale mapping a unit mesh onto an instance's shape.
             *
             * Exactly `2 * params` for Box (half-extents) and Cylinder (radius,
             * half-height, radius), and a uniform `2 * params.x` for Sphere (radius).
             *
             * @param kind   Which unit mesh the instance draws with.
             * @param params The instance's authored shape parameters.
             * @return The scale matrix to pre-multiply the model transform by.
             */
            Mat4 shape_scale(MeshKind kind, const Vector3& params) noexcept;

            /**
             * @brief Generates per-vertex tangents from positions, normals, and UV0.
             *
             * Accumulates the per-triangle tangent implied by the UV parameterisation,
             * then Gram-Schmidt-orthogonalises it against the normal and records the
             * bitangent handedness in w. Degenerate UVs leave a zero tangent, which the
             * shader treats as "derive one".
             *
             * @param vertices     Vertices to fill the tangent of, in place.
             * @param vertex_count Number of entries in @p vertices.
             * @param indices      Triangle indices into @p vertices.
             * @param index_count  Number of entries in @p indices.
             */
            void generate_tangents(MeshVertex* vertices, std::size_t vertex_count,
                                   const std::uint32_t* indices, std::size_t index_count);

            /**
             * @brief Owns every mesh on the device, shared by every view that draws them.
             *
             * Non-copyable: it owns VMA allocations.
             */
            class MeshRegistry
            {
                public:
                    /**
                     * @brief Builds and uploads the built-in primitives.
                     * @param device The live Vulkan device.
                     */
                    explicit MeshRegistry(Vulkan::VulkanDevice& device);
                    ~MeshRegistry();

                    MeshRegistry(const MeshRegistry&) = delete;
                    MeshRegistry& operator=(const MeshRegistry&) = delete;

                    /**
                     * @brief The unit mesh an instance kind draws with.
                     * @param kind The requested primitive.
                     * @return Its buffers and counts.
                     */
                    const Mesh& primitive(MeshKind kind) const noexcept;

                    /**
                     * @brief Uploads an imported mesh and returns the id that names it.
                     * @param vertices     The mesh's vertices, tangents already generated.
                     * @param vertex_count Number of entries in @p vertices.
                     * @param indices      Triangle indices into @p vertices.
                     * @param index_count  Number of entries in @p indices.
                     * @return The mesh id, or INVALID_MESH if the mesh was empty.
                     */
                    MeshId add_mesh(const MeshVertex* vertices, std::size_t vertex_count,
                                    const std::uint32_t* indices, std::size_t index_count);

                    /**
                     * @brief An imported mesh by id.
                     * @param mesh The id returned by add_mesh().
                     * @return Its buffers and counts, or an empty mesh if the id is unknown.
                     */
                    const Mesh& mesh(MeshId mesh) const noexcept;

                    /**
                     * @brief An imported mesh's baked signed-distance brick, for probe GI.
                     * @param mesh The id returned by add_mesh().
                     * @return Its brick, or nullptr if the id is unknown or the mesh degenerate.
                     */
                    const Gi::MeshSdfBrick* mesh_brick(MeshId mesh) const noexcept;

                private:
                    /** @brief A VMA-backed buffer and the capacity it was allocated at. */
                    struct Allocation
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    /** @brief One imported mesh and the allocations behind it. */
                    struct Imported
                    {
                        Allocation vertices;
                        Allocation indices;
                        Mesh mesh;
                        Gi::MeshSdfBrick brick; /**< Signed-distance brick baked at import for GI. */
                    };

                    Allocation upload(const void* data, VkDeviceSize bytes,
                                      VkBufferUsageFlags usage);
                    /**
                     * @brief The usage a mesh vertex buffer needs: vertex fetch, plus storage
                     *        so the mesh shader can read it when the device draws with meshlets.
                     */
                    VkBufferUsageFlags vertex_usage() const;
                    void grow(Allocation& target, VkDeviceSize bytes, VkBufferUsageFlags usage);
                    void destroy(Allocation& target);

                    /**
                     * @brief Meshletises a mesh and uploads its three meshlet buffers.
                     *
                     * A no-op when the device cannot draw with mesh shaders. Otherwise it
                     * builds the meshlets, uploads the descriptor, vertex, and triangle arrays
                     * into owned storage buffers, and records their handles and count on @p mesh.
                     */
                    void attach_meshlets(Mesh& mesh, const MeshVertex* vertices,
                                         std::size_t vertex_count, const std::uint32_t* indices,
                                         std::size_t index_count);

                    Vulkan::VulkanDevice& device_;
                    Allocation box_vertices_;
                    Allocation box_indices_;
                    Allocation sphere_vertices_;
                    Allocation sphere_indices_;
                    Allocation cylinder_vertices_;
                    Allocation cylinder_indices_;
                    std::vector<Imported> imported_;
                    std::vector<Allocation> meshlet_buffers_; /**< Owns every mesh's meshlet buffers. */
                    Mesh box_;
                    Mesh sphere_;
                    Mesh cylinder_;
                    Mesh empty_;
            };
        } // namespace Geometry
    } // namespace Render
} // namespace SushiEngine
