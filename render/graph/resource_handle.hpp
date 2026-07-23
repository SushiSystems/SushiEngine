/**************************************************************************/
/* resource_handle.hpp                                                    */
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
 * @file resource_handle.hpp
 * @brief The render graph's resource vocabulary: handles, descriptions, accesses.
 *
 * A pass never names a VkImage. It names a TextureHandle — an index into the
 * graph's virtual resource table — and the way it touches it. The graph resolves
 * handles to physical resources during compile and derives every barrier from the
 * declared accesses, which is why this header is the narrow type a pass depends on
 * (ISP) rather than the graph itself.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Graph
        {
            /** @brief The index a default-constructed (invalid) handle carries. */
            constexpr std::uint32_t INVALID_RESOURCE = 0xFFFFFFFFu;

            /**
             * @brief Which device queue a pass's commands are recorded onto.
             *
             * @c AsyncCompute is a request, not a guarantee: the graph honours it only when
             * the device offers a compute family distinct from graphics and the frame
             * enabled the second queue, and silently records the pass on the graphics queue
             * otherwise. A pass that declares it must therefore be correct on either queue —
             * in practice, compute-only, with no attachment and no draw.
             */
            enum class PassQueue : std::uint32_t
            {
                Graphics,
                AsyncCompute,
            };

            /**
             * @brief A virtual texture inside one frame's graph.
             *
             * Distinct from BufferHandle so a pass cannot bind a buffer where a texture
             * is meant; the index is only meaningful to the graph that produced it and
             * is invalidated by the next begin_frame().
             */
            struct TextureHandle
            {
                std::uint32_t index = INVALID_RESOURCE;

                /** @brief Whether this handle names a resource. */
                bool valid() const noexcept { return index != INVALID_RESOURCE; }
            };

            /** @brief A virtual buffer inside one frame's graph. */
            struct BufferHandle
            {
                std::uint32_t index = INVALID_RESOURCE;

                /** @brief Whether this handle names a resource. */
                bool valid() const noexcept { return index != INVALID_RESOURCE; }
            };

            /**
             * @brief How a pass touches a texture.
             *
             * Each value maps to exactly one (pipeline stage, access mask, image layout)
             * triple — see texture_access_state() — which is what lets the graph derive
             * barriers without a pass ever writing one.
             */
            enum class TextureAccess : std::uint32_t
            {
                ColorAttachment,        /**< Written as a colour attachment. */
                DepthStencilAttachment, /**< Written as a depth/stencil attachment. */
                DepthStencilReadOnly,   /**< Bound as depth/stencil but never written. */
                SampledFragment,        /**< Sampled by a fragment shader. */
                SampledCompute,         /**< Sampled by a compute shader. */
                StorageComputeRead,     /**< Read as a storage image by compute. */
                StorageComputeWrite,    /**< Written as a storage image by compute. */
                StorageComputeReadWrite,/**< Read and written as a storage image by compute. */
                TransferSource,         /**< Copy source. */
                TransferDestination,    /**< Copy destination. */
                PresentSource,          /**< Handed to the presentation engine. */
                ShadingRateAttachment,  /**< Read per-tile as the fragment shading rate. */
            };

            /** @brief How a pass touches a buffer. */
            enum class BufferAccess : std::uint32_t
            {
                UniformRead,        /**< Read as a uniform buffer by vertex/fragment. */
                UniformComputeRead, /**< Read as a uniform buffer by compute. */
                StorageRead,        /**< Read as a storage buffer by a shader. */
                StorageWrite,       /**< Written as a storage buffer by a shader. */
                StorageReadWrite,   /**< Read and written as a storage buffer by a shader. */
                IndirectRead,       /**< Consumed as indirect draw/dispatch arguments. */
                VertexRead,         /**< Fetched as vertex attributes. */
                IndexRead,          /**< Fetched as indices. */
                TransferSource,     /**< Copy source. */
                TransferDestination,/**< Copy destination. */
                HostRead,           /**< Read by the host after the submit completes. */
            };

            /**
             * @brief What a pass wants done with an attachment's prior contents.
             *
             * Maps onto VkAttachmentLoadOp; kept as a graph-level enum so a pass does not
             * spell Vulkan constants and so Discard can also inform the graph that the
             * previous contents need no barrier to preserve them.
             */
            enum class AttachmentLoad : std::uint32_t
            {
                Clear,    /**< Clear to the supplied value. */
                Load,     /**< Preserve and continue from the existing contents. */
                Discard,  /**< Contents are undefined on entry; every texel is rewritten. */
            };

            /** @brief A colour attachment's clear colour, in the attachment's own format. */
            struct ClearColor
            {
                float float32[4] = {0.0f, 0.0f, 0.0f, 1.0f}; /**< Used by float/unorm formats. */
                std::uint32_t uint32[4] = {0, 0, 0, 0};      /**< Used by integer formats. */
                bool integer = false; /**< Selects which of the two arrays applies. */
            };

            /**
             * @brief A texture the graph may allocate, alias, and hand to passes.
             *
             * @c usage carries only the uses the graph cannot infer; the graph unions in
             * the usage implied by every declared access before allocating, so a pass
             * that samples a target never has to remember VK_IMAGE_USAGE_SAMPLED_BIT.
             */
            struct TextureDesc
            {
                std::uint32_t width = 1;
                std::uint32_t height = 1;
                std::uint32_t depth = 1;
                std::uint32_t mip_levels = 1;
                std::uint32_t array_layers = 1;
                VkFormat format = VK_FORMAT_UNDEFINED;
                VkImageType type = VK_IMAGE_TYPE_2D;
                VkImageViewType view_type = VK_IMAGE_VIEW_TYPE_2D;
                VkImageAspectFlags aspect = VK_IMAGE_ASPECT_COLOR_BIT;
                VkImageUsageFlags usage = 0;

                /**
                 * @brief Whether both queue families touch this resource in one frame.
                 *
                 * Derived by the graph at compile time from the queues its passes were
                 * scheduled on, never authored: a resource one family writes and another
                 * reads has to be created with concurrent sharing, and concurrent sharing
                 * costs image compression on some hardware — so it is paid by exactly the
                 * resources that cross and by nothing else. Part of the pool's reuse key,
                 * so a shared and an exclusive transient never land on one allocation.
                 */
                bool cross_queue = false;
                const char* name = "texture";
            };

            /** @brief A buffer the graph may allocate, alias, and hand to passes. */
            struct BufferDesc
            {
                VkDeviceSize size = 0;
                VkBufferUsageFlags usage = 0;
                bool host_visible = false; /**< Allocate mapped host memory (readbacks, uploads). */
                /** @brief Both queue families touch it; see @ref TextureDesc::cross_queue. */
                bool cross_queue = false;
                const char* name = "buffer";
            };

            /**
             * @brief Compares two texture descriptions for pool-reuse equivalence.
             *
             * The name is excluded: two differently named transients with identical
             * dimensions and format may share one physical image, which is what makes
             * lifetime-based aliasing possible.
             *
             * @param a First description.
             * @param b Second description.
             * @return true when the two describe the same physical image.
             */
            inline bool same_texture_desc(const TextureDesc& a, const TextureDesc& b) noexcept
            {
                return a.width == b.width && a.height == b.height && a.depth == b.depth &&
                       a.mip_levels == b.mip_levels && a.array_layers == b.array_layers &&
                       a.format == b.format && a.type == b.type && a.view_type == b.view_type &&
                       a.aspect == b.aspect && a.usage == b.usage &&
                       a.cross_queue == b.cross_queue;
            }

            /**
             * @brief Compares two buffer descriptions for pool-reuse equivalence.
             * @param a First description.
             * @param b Second description.
             * @return true when the two describe the same physical buffer.
             */
            inline bool same_buffer_desc(const BufferDesc& a, const BufferDesc& b) noexcept
            {
                return a.size == b.size && a.usage == b.usage &&
                       a.host_visible == b.host_visible && a.cross_queue == b.cross_queue;
            }
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
