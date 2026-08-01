/**************************************************************************/
/* interval.hpp                                                           */
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

#include <cstddef>
#include <cstdint>

namespace SushiEngine
{
    namespace Execution
    {
        /**
         * @brief Identity of the allocation a buffer interval belongs to.
         *
         * A base address today, but a type alias rather than a bare pointer for the
         * same reason SushiRuntime aliases its own key: identity is planned to move to
         * an opaque registered id, and a typedef makes that a one-line change here
         * instead of a migration across every declaration site. It is never
         * dereferenced — only compared.
         *
         * Deliberately non-const so lowering a BufferInterval to a backend's own region
         * type stays a member-wise copy with no cast in the adapter.
         */
        using ResourceId = void*;

        /**
         * @brief A half-open byte interval within one tracked allocation.
         *
         * The simulation domain's native hazard currency, and the only shape allowed to
         * cross the domain boundary. Tracking intervals rather than bare allocations is
         * what keeps disjoint sub-ranges of one buffer — two halves of an array, an
         * island's slice of a solver column — parallel instead of serialized, while
         * distinct allocations never order against each other at all.
         */
        struct BufferInterval
        {
            /** @brief Sentinel size meaning "to the end of the allocation". */
            static constexpr std::uint64_t WHOLE = ~std::uint64_t{0};

            ResourceId    base   = nullptr; /**< Owning allocation; the identity key. */
            std::uint64_t offset = 0;       /**< Byte offset of the interval start. */
            std::uint64_t size   = WHOLE;   /**< Byte length, or WHOLE for the rest. */

            /** @brief Constructs an empty interval that overlaps nothing. */
            constexpr BufferInterval() noexcept = default;

            /**
             * @brief Names the whole of one allocation.
             * @param allocation Base address of the allocation; null overlaps nothing.
             */
            explicit constexpr BufferInterval(ResourceId allocation) noexcept
                : base(allocation)
            {
            }

            /**
             * @brief Names a sub-range of one allocation.
             * @param allocation   Base address of the owning allocation.
             * @param byte_offset  Byte offset of the interval start within the allocation.
             * @param byte_size    Byte length of the interval; WHOLE means to the end.
             */
            constexpr BufferInterval(ResourceId allocation, std::uint64_t byte_offset,
                                     std::uint64_t byte_size) noexcept
                : base(allocation), offset(byte_offset), size(byte_size)
            {
            }

            /** @brief True when the interval keys on a real allocation. */
            constexpr bool valid() const noexcept { return base != nullptr; }

            /**
             * @brief Reports whether two intervals of one allocation share any byte.
             *
             * The overlap test every conforming tracker decides ordering by: identical
             * base, and intersecting half-open ranges. WHOLE is treated as an unbounded
             * upper end so the comparison never overflows on the sentinel.
             *
             * @param other The interval to test against.
             * @return True when both name the same allocation and any byte is common.
             */
            constexpr bool overlaps(const BufferInterval& other) const noexcept
            {
                if (base != other.base || base == nullptr)
                    return false;

                const bool this_whole  = (size == WHOLE);
                const bool other_whole = (other.size == WHOLE);

                if (this_whole && other_whole) return true;
                if (this_whole)                return offset < other.offset + other.size;
                if (other_whole)               return other.offset < offset + size;
                return offset < other.offset + other.size && other.offset < offset + size;
            }
        };

        namespace Detail
        {
            /**
             * @brief Half-open range intersection where the sentinel count is unbounded.
             *
             * Shared by the mip and layer tests so the sentinel is handled once rather
             * than twice with a chance to diverge.
             *
             * @param a_base     Start of the first range.
             * @param a_count    Length of the first range; the sentinel means unbounded.
             * @param b_base     Start of the second range.
             * @param b_count    Length of the second range; the sentinel means unbounded.
             * @param unbounded  The count value standing for "every remaining element".
             * @return True when the two ranges share any element.
             */
            constexpr bool ranges_intersect(std::uint16_t a_base, std::uint16_t a_count,
                                            std::uint16_t b_base, std::uint16_t b_count,
                                            std::uint16_t unbounded) noexcept
            {
                const bool a_all = (a_count == unbounded);
                const bool b_all = (b_count == unbounded);

                if (a_all && b_all) return true;
                if (a_all)          return a_base < b_base + b_count;
                if (b_all)          return b_base < a_base + a_count;
                return a_base < b_base + b_count && b_base < a_base + a_count;
            }
        } // namespace Detail

        /**
         * @brief A half-open window of elements into a typed allocation.
         *
         * How a caller names part of a buffer without doing the byte arithmetic itself:
         * counts are in elements, and the allocation converts the window to a byte
         * interval knowing its own element size. Bundling the two numbers keeps them
         * from being passed in the wrong order, which a pair of bare counts invites.
         */
        struct ElementRange
        {
            std::size_t first = 0; /**< Index of the first element in the window. */
            std::size_t count = 0; /**< Number of elements the window covers. */
        };

        /**
         * @brief An image subresource rectangle — the render domain's hazard currency.
         *
         * Opaque, possibly tiled images have no meaningful byte intervals, so the render
         * domain keys on mip and layer ranges instead. Present in the shared vocabulary
         * because the render graph and a future compute-shader simulation backend both
         * need it; rejected at the domain boundary, where traffic is buffer-shaped.
         */
        struct TextureInterval
        {
            /** @brief Sentinel count meaning "every remaining mip or layer". */
            static constexpr std::uint16_t ALL = ~std::uint16_t{0};

            std::uint16_t mip_base    = 0;   /**< First mip level in the range. */
            std::uint16_t mip_count   = ALL; /**< Mip levels covered, or ALL. */
            std::uint16_t layer_base  = 0;   /**< First array layer in the range. */
            std::uint16_t layer_count = ALL; /**< Array layers covered, or ALL. */
            std::uint8_t  aspects     = 0;   /**< Aspect bits, a projection of the backend's mask. */

            /**
             * @brief Reports whether two subresource rectangles of one image intersect.
             *
             * Overlap is the conjunction of three independent range tests — mips,
             * layers, and aspect bits — because a barrier is only needed when all three
             * coincide; a depth read against a stencil write on the same image is not a
             * hazard.
             *
             * @param other The rectangle to test against.
             * @return True when the mip ranges, layer ranges, and aspect masks all intersect.
             */
            constexpr bool overlaps(const TextureInterval& other) const noexcept
            {
                return (aspects & other.aspects) != 0 &&
                       Detail::ranges_intersect(mip_base, mip_count,
                                                other.mip_base, other.mip_count, ALL) &&
                       Detail::ranges_intersect(layer_base, layer_count,
                                                other.layer_base, other.layer_count, ALL);
            }
        };

        /**
         * @brief Either shape of resource range, for the declarations that must carry both.
         *
         * The one place the two domains' currencies meet. It appears in node access
         * declarations and in the boundary registry — both cold, per-resource paths.
         * No kernel signature and no per-element path is ever retyped onto it: the
         * simulation keeps pointers and byte intervals in its hot path, the renderer
         * keeps its own handles, and unifying those would cost throughput to buy nothing.
         */
        struct ResourceInterval
        {
            /** @brief Which alternative of the union is live. */
            enum class Kind : std::uint8_t
            {
                Buffer,  /**< The buffer interval is live. */
                Texture, /**< The texture interval is live. */
            };

            Kind kind = Kind::Buffer; /**< The live alternative. */

            union
            {
                BufferInterval  buffer;  /**< Live when kind is Buffer. */
                TextureInterval texture; /**< Live when kind is Texture. */
            };

            /** @brief Constructs an empty buffer interval, which overlaps nothing. */
            constexpr ResourceInterval() noexcept : kind(Kind::Buffer), buffer() {}

            /**
             * @brief Wraps a buffer interval.
             *
             * Non-explicit so simulation call sites, which never name a texture, read as
             * if the sum type were not there.
             *
             * @param interval The byte interval this access names.
             */
            constexpr ResourceInterval(BufferInterval interval) noexcept
                : kind(Kind::Buffer), buffer(interval)
            {
            }

            /**
             * @brief Wraps an image subresource rectangle.
             * @param interval The subresource range this access names.
             */
            constexpr ResourceInterval(TextureInterval interval) noexcept
                : kind(Kind::Texture), texture(interval)
            {
            }

            /**
             * @brief Reports whether two resource ranges share any tracked storage.
             *
             * Intervals of different kinds never overlap: a buffer and an image are
             * distinct allocations by construction, so the kind check is the identity
             * test, not a conservative approximation.
             *
             * @param other The range to test against.
             * @return True when both name the same resource and any part is common.
             */
            constexpr bool overlaps(const ResourceInterval& other) const noexcept
            {
                if (kind != other.kind)
                    return false;
                return kind == Kind::Buffer ? buffer.overlaps(other.buffer)
                                            : texture.overlaps(other.texture);
            }
        };
    } // namespace Execution
} // namespace SushiEngine
