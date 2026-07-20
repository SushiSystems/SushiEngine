/**************************************************************************/
/* material_system.hpp                                                    */
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
 * @file material_system.hpp
 * @brief Packs authored materials into the GPU array a draw indexes into.
 *
 * The authored Material is a wide struct with texture ids; the GPU wants a compact,
 * fixed-layout record with bindless heap indices. This system does the translation
 * once per frame into a storage buffer, so a draw carries a single material index in
 * its push constant instead of a payload of parameters — which is what makes the
 * material path GPU-driven-friendly when indirect draws land.
 *
 * Unset texture slots resolve to the texture library's neutral defaults, so the
 * shader samples unconditionally and never branches on whether a map exists.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <vulkan/vulkan.h>
#include <vk_mem_alloc.h>

#include <SushiEngine/render/material.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            class VulkanDevice;
        }

        namespace Assets
        {
            class TextureLibrary;

            /**
             * @brief Behaviour a shader cannot infer from a neutral default texture.
             *
             * A missing base-colour map is neutralised by sampling white, but a missing
             * height map cannot be neutralised — parallax has to be skipped outright —
             * so those decisions travel as bits.
             */
            enum MaterialFlags : std::uint32_t
            {
                MATERIAL_HAS_PARALLAX = 1u << 0,
                MATERIAL_PACKED_OCCLUSION = 1u << 1,
                MATERIAL_EMISSIVE = 1u << 2,
                MATERIAL_PARALLAX_SHADOWS = 1u << 3,
                MATERIAL_PARALLAX_CLIP = 1u << 4,
                MATERIAL_HAS_DETAIL = 1u << 5,
                MATERIAL_CUTOUT = 1u << 6,
                MATERIAL_ANISOTROPY = 1u << 7,
                MATERIAL_CLEARCOAT = 1u << 8,
                MATERIAL_SHEEN = 1u << 9,
                MATERIAL_TRANSMISSION = 1u << 10,
            };

            /**
             * @brief One material as the shader reads it, std430, 208 bytes.
             *
             * Flat vec4 rows so the C++ side cannot disagree with the GLSL packing.
             */
            struct GpuMaterial
            {
                float base_color[4];          /**< rgb tint, a = base alpha. */
                float emissive[4];            /**< rgb premultiplied by intensity, a = normal scale. */
                float surface[4];             /**< metallic, roughness, occlusion strength, alpha cutoff. */
                float parallax[4];            /**< height scale, steps, ior, detail normal scale. */
                float main_transform[4];      /**< tiling.xy, offset.xy. */
                float detail_transform[4];    /**< tiling.xy, offset.xy. */
                float anisotropy_clearcoat[4];/**< anisotropy, rotation, clearcoat, clearcoat roughness. */
                float sheen[4];               /**< rgb sheen colour, a = sheen roughness. */
                float transmission[4];        /**< transmission, thickness, unused, unused. */
                float subsurface[4];          /**< rgb subsurface colour, a unused. */
                std::uint32_t maps_a[4];      /**< albedo, metallic-roughness, normal, height. */
                std::uint32_t maps_b[4];      /**< occlusion, emissive, detail albedo, detail normal. */
                std::uint32_t maps_c[4];      /**< detail mask, flags, unused, unused. */
            };

            /**
             * @brief Builds one frame's material array and owns the buffer behind it.
             *
             * One buffer per frame slot, grown on demand and never shrunk, so a frame
             * writing its array cannot disturb one the GPU is still reading.
             * Non-copyable: it owns VMA allocations.
             */
            class MaterialSystem
            {
                public:
                    /**
                     * @brief Allocates the per-slot material buffers.
                     * @param device      The live Vulkan device.
                     * @param textures    Library resolving texture ids to heap indices.
                     * @param frame_slots Number of frames in flight.
                     */
                    MaterialSystem(Vulkan::VulkanDevice& device, TextureLibrary& textures,
                                   std::uint32_t frame_slots);
                    ~MaterialSystem();

                    MaterialSystem(const MaterialSystem&) = delete;
                    MaterialSystem& operator=(const MaterialSystem&) = delete;

                    /**
                     * @brief Starts a new frame's array.
                     * @param slot The frame slot being recorded.
                     */
                    void begin_frame(std::uint32_t slot);

                    /**
                     * @brief Packs a material and returns the index a draw refers to it by.
                     * @param material The authored surface.
                     * @return The index into this frame's material array.
                     */
                    std::uint32_t push(const Render::Material& material);

                    /**
                     * @brief Uploads the frame's array into its slot buffer.
                     *
                     * Must be called after the last push() and before the frame is
                     * submitted; the buffer is reallocated here if the array outgrew it.
                     */
                    void upload();

                    /** @brief The buffer holding the current frame's array. */
                    VkBuffer buffer() const noexcept;

                    /** @brief Bytes of the current frame's array, for the descriptor range. */
                    VkDeviceSize buffer_range() const noexcept;

                private:
                    /** @brief One frame slot's material buffer. */
                    struct Slot
                    {
                        VkBuffer buffer = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        void* mapped = nullptr;
                        VkDeviceSize capacity = 0;
                    };

                    void grow(Slot& slot, VkDeviceSize bytes);

                    Vulkan::VulkanDevice& device_;
                    TextureLibrary& textures_;
                    std::vector<Slot> slots_;
                    std::vector<GpuMaterial> packed_;
                    std::uint32_t current_slot_ = 0;
            };
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
