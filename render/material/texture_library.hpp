/**************************************************************************/
/* texture_library.hpp                                                    */
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
 * @file texture_library.hpp
 * @brief Texture decode, upload, mip generation, bindless registration, streaming.
 *
 * Owns every sampled texture on the device. A texture is decoded once, uploaded
 * with a GPU-generated mip chain, and registered into the bindless heap, so a
 * material refers to it by a plain index. Loads are deduplicated by path.
 *
 * Residency is mip-based: a texture that would not fit the budget is uploaded from
 * a lower mip — a smaller image holding the tail of its chain — and upgraded later,
 * at most one per frame, when the budget allows. No upload ever blocks the frame:
 * the work is submitted with its own fence and the staging memory and superseded
 * images are reclaimed once that fence signals.
 *
 * Four immutable defaults (white, flat normal, black, and a neutral
 * metallic-roughness) are registered first, so a material with an unset map still
 * hands the shader a valid heap index and needs no branch to sample it.
 */

#include <cstddef>
#include <cstdint>
#include <string>
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

        namespace Resources
        {
            class DescriptorHeap;
            class SamplerCache;
        }

        namespace Assets
        {
            /** @brief Which neutral texture stands in for an unset material slot. */
            enum class DefaultTexture : std::uint32_t
            {
                White,             /**< Opaque white: neutral for a tinted colour map. */
                FlatNormal,        /**< (0.5, 0.5, 1): no perturbation. */
                Black,             /**< Transparent black: neutral for emissive and detail. */
                NeutralMaterial,   /**< (1, 1, 1): neutral occlusion, roughness, metallic. */
            };

            /**
             * @brief The device's texture store: decode, upload, residency, bindless slots.
             *
             * Non-copyable: it owns images, views, and their allocations.
             */
            class TextureLibrary
            {
                public:
                    /**
                     * @brief Creates the defaults and binds the library to its heap.
                     * @param device       The live Vulkan device.
                     * @param heap         Bindless heap every texture is registered into.
                     * @param samplers     Cache providing the sampling configurations.
                     * @param budget_bytes Device memory the resident set is held under.
                     */
                    TextureLibrary(Vulkan::VulkanDevice& device, Resources::DescriptorHeap& heap,
                                   Resources::SamplerCache& samplers, std::size_t budget_bytes);
                    ~TextureLibrary();

                    TextureLibrary(const TextureLibrary&) = delete;
                    TextureLibrary& operator=(const TextureLibrary&) = delete;

                    /**
                     * @brief Loads an image file, or returns the id it already has.
                     * @param path        Path to a PNG, JPEG, TGA, BMP, or HDR image.
                     * @param color_space How the file's values are to be interpreted.
                     * @return The texture id, or INVALID_TEXTURE if it could not be read.
                     */
                    TextureId load(const char* path, TextureColorSpace color_space);

                    /**
                     * @brief Registers an already-decoded RGBA8 image.
                     *
                     * Used by the glTF importer for textures embedded in a buffer view,
                     * which never exist as a file on disk.
                     *
                     * @param name        Cache key; a repeat call with the same name reuses it.
                     * @param pixels      Tightly packed RGBA8 texels, @p width * @p height of them.
                     * @param width       Image width in texels.
                     * @param height      Image height in texels.
                     * @param color_space How the values are to be interpreted.
                     * @return The texture id, or INVALID_TEXTURE if the image was empty.
                     */
                    TextureId add(const char* name, const std::uint8_t* pixels,
                                  std::uint32_t width, std::uint32_t height,
                                  TextureColorSpace color_space);

                    /**
                     * @brief Drops a reference, freeing the texture at zero.
                     * @param texture The id to release; INVALID_TEXTURE is ignored.
                     */
                    void release(TextureId texture);

                    /**
                     * @brief The bindless heap index a material stores for a texture.
                     * @param texture  The texture id, or INVALID_TEXTURE.
                     * @param fallback Which default to substitute when @p texture is unset.
                     * @return A heap index that is always valid to sample.
                     */
                    std::uint32_t heap_index(TextureId texture, DefaultTexture fallback) const noexcept;

                    /**
                     * @brief Advances streaming: reclaims finished uploads, upgrades one texture.
                     *
                     * Called once per frame. At most one texture is re-uploaded at a higher
                     * resolution, so the per-frame cost stays bounded however far the
                     * resident set is from the budget.
                     */
                    void update();

                    /** @brief Bytes of device memory the resident textures occupy. */
                    std::size_t resident_bytes() const noexcept { return resident_bytes_; }

                    /**
                     * @brief Whether a texture's alpha channel is not uniformly opaque.
                     *
                     * The glTF importer uses this to pick a surface type when the file does
                     * not state one.
                     *
                     * @param texture The texture id.
                     * @return true when at least one texel's alpha is below 255.
                     */
                    bool has_transparency(TextureId texture) const noexcept;

                private:
                    /** @brief One registered texture and its current residency. */
                    struct Entry
                    {
                        std::string key;
                        TextureColorSpace color_space = TextureColorSpace::Linear;
                        std::vector<std::uint8_t> pixels; /**< Kept only for memory-sourced textures. */
                        VkImage image = VK_NULL_HANDLE;
                        VmaAllocation allocation = VK_NULL_HANDLE;
                        VkImageView view = VK_NULL_HANDLE;
                        std::uint32_t full_width = 0;    /**< Width of the file's mip 0. */
                        std::uint32_t full_height = 0;
                        std::uint32_t resident_width = 0; /**< Width actually uploaded. */
                        std::uint32_t resident_height = 0;
                        std::uint32_t mip_levels = 1;
                        std::uint32_t heap_index = 0;
                        std::size_t bytes = 0;
                        std::uint32_t references = 0;
                        bool transparent = false;
                        bool from_file = false; /**< Re-decoded from @c key on upgrade. */
                        bool live = false;
                    };

                    /** @brief An in-flight upload and the objects it frees when it completes. */
                    struct Pending
                    {
                        VkCommandBuffer cmd = VK_NULL_HANDLE;
                        VkFence fence = VK_NULL_HANDLE;
                        VkBuffer staging = VK_NULL_HANDLE;
                        VmaAllocation staging_allocation = VK_NULL_HANDLE;
                        VkImage retired_image = VK_NULL_HANDLE;
                        VmaAllocation retired_allocation = VK_NULL_HANDLE;
                        VkImageView retired_view = VK_NULL_HANDLE;
                        // The superseded heap slot is recycled only once this fence has
                        // signalled, so a frame still in flight never finds it reassigned
                        // to some other texture.
                        std::uint32_t retired_heap_index = 0xFFFFFFFFu;
                    };

                    TextureId create_default(const char* name, std::uint8_t r, std::uint8_t g,
                                             std::uint8_t b, std::uint8_t a);
                    TextureId find(const std::string& key) const;
                    bool upload(Entry& entry, const std::uint8_t* pixels, std::uint32_t width,
                                std::uint32_t height);
                    void destroy(Entry& entry);
                    void collect_finished();
                    std::uint32_t budget_scale(std::uint32_t width, std::uint32_t height) const;

                    Vulkan::VulkanDevice& device_;
                    Resources::DescriptorHeap& heap_;
                    VkSampler sampler_ = VK_NULL_HANDLE;
                    VkCommandPool pool_ = VK_NULL_HANDLE;
                    std::vector<Entry> entries_;
                    std::vector<Pending> pending_;
                    std::size_t budget_bytes_ = 0;
                    std::size_t resident_bytes_ = 0;
                    TextureId defaults_[4]{};
            };
        } // namespace Assets
    } // namespace Render
} // namespace SushiEngine
