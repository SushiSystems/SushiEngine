/**************************************************************************/
/* tracer.hpp                                                             */
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
 * @file tracer.hpp
 * @brief The strategy seam every probe relight is written behind (DIP).
 *
 * A probe volume is refilled each frame it is dirty by *some* tracer: the degenerate
 * one broadcasts the distant environment SH into every probe, a mid tier cone-traces a
 * signed-distance field for local occlusion, an Ultra tier ray-queries the acceleration
 * structure. They differ only in how a probe's incident radiance is gathered, never in
 * what a probe is or how the shading pass consumes it, so the volume pass owns a tracer
 * through this interface and knows nothing of which one it holds. Adding a tier is a new
 * implementation of @ref IProbeTracer, not a change to the pass.
 */

#include <cstdint>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Frame
        {
            struct FrameContext;
        }

        namespace Gi
        {
            struct ProbeVolumeConfig;

            /**
             * @brief Everything a tracer needs to relight the probe volume this frame.
             *
             * The output probe buffer and its size, the probe count to dispatch over, the
             * distant-environment SH the tracer falls back to where its own trace finds no
             * local geometry (the sky/ground miss), and the probe lattice config a tracer
             * needs to place probes in the world. All buffers are caller-owned.
             */
            struct ProbeRelightInputs
            {
                VkBuffer probe_sh = VK_NULL_HANDLE;      /**< Output: nine SH vec4 per probe. */
                VkDeviceSize probe_sh_bytes = 0;         /**< Size of the output buffer. */
                std::uint32_t probe_count = 0;           /**< Probes to relight. */
                VkBuffer environment_sh = VK_NULL_HANDLE; /**< Distant environment SH (144 B). */
                VkDeviceSize environment_sh_bytes = 0;   /**< Size of the environment SH buffer. */
                const ProbeVolumeConfig* config = nullptr; /**< The probe lattice this frame. */
                const Frame::FrameContext* frame = nullptr; /**< This frame's inputs and allocators. */
            };

            /**
             * @brief A scene field a shading pass can trace a shadow ray against.
             *
             * A tracer builds *some* representation of the scene to gather probe radiance
             * from; where that representation is a distance field, it is also the cheapest
             * answer to "is this light visible from this surface" that does not cost a
             * shadow map — which is what lets the light engine shadow more lights than the
             * atlas has tiles for. Offered, not required: a tracer that keeps no such field
             * (the environment floor tier) returns an empty record and the shading pass
             * falls back to shadowing only the lights that hold an atlas tile.
             */
            struct VisibilityField
            {
                VkImageView distance_field = VK_NULL_HANDLE; /**< sampler3D: rgb albedo, a distance. */
                VkBuffer config = VK_NULL_HANDLE;            /**< The block locating it in space. */
                VkDeviceSize config_bytes = 0;

                /** @brief Whether this record names a field that can be traced. */
                bool valid() const noexcept
                {
                    return distance_field != VK_NULL_HANDLE && config != VK_NULL_HANDLE;
                }
            };

            /**
             * @brief A pluggable probe relight strategy.
             *
             * Implementations own their own compute pipeline and descriptor layout; the
             * probe volume pass calls @ref relight inside its graph pass and hand-barriers
             * the result to the shading pass's fragment reads.
             */
            class IProbeTracer
            {
                public:
                    virtual ~IProbeTracer() = default;

                    /**
                     * @brief Records the probe relight into the command buffer.
                     * @param cmd    The recording command buffer, inside the volume pass.
                     * @param inputs The buffers and counts for this frame.
                     */
                    virtual void relight(VkCommandBuffer cmd,
                                         const ProbeRelightInputs& inputs) = 0;

                    /** @brief A short name for the profiler and the editor tier readout. */
                    virtual const char* name() const noexcept = 0;

                    /**
                     * @brief The scene field a shading pass may trace shadow rays against.
                     *
                     * Defaulted to empty so a tracer that keeps no traceable field says
                     * nothing rather than being forced to invent one.
                     *
                     * @param frame_index The frame counter, selecting the tracer's ring slot.
                     * @return The field, or an empty record.
                     */
                    virtual VisibilityField visibility_field(std::uint32_t frame_index) const noexcept
                    {
                        (void)frame_index;
                        return {};
                    }

                    /** @brief Rebuilds the compute pipeline after a shader hot-reload. */
                    virtual void rebuild_pipelines() = 0;
            };
        } // namespace Gi
    } // namespace Render
} // namespace SushiEngine
