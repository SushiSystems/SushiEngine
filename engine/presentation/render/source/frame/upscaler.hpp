/**************************************************************************/
/* upscaler.hpp                                                           */
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
 * @file upscaler.hpp
 * @brief The one interface every reconstruction backend is written against.
 *
 * Rendering below the output resolution and reconstructing back up is a *contract*,
 * not a pass: whatever does the reconstruction — the engine's own temporal resolve or a
 * vendor library — consumes the same colour, depth, motion, exposure, and jitter, and
 * writes the same output-extent image. Naming that contract here is what lets FSR, DLSS,
 * or XeSS land as an implementation rather than as a fork of the frame loop, and it is
 * the reason the renderer's motion vectors were camera-relative from the start.
 */

#include <cstdint>

#include <SushiEngine/render/render_settings.hpp>
#include <SushiEngine/render/upscaler_info.hpp>

#include "graph/resource_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Graph
        {
            class RenderGraph;
        }

        namespace Frame
        {
            struct FrameContext;

            /**
             * @brief Everything a reconstruction backend reads and writes.
             *
             * The handles are this frame's graph resources; the extents say what is being
             * reconstructed from what. @c reset tells a backend carrying history that the
             * history is no longer about this scene (a camera cut, a resize, a mode change)
             * and must be discarded rather than blended.
             */
            struct UpscaleInputs
            {
                Graph::TextureHandle color;    /**< Scene colour at the render extent. */
                Graph::TextureHandle depth;    /**< Reverse-Z depth at the render extent. */
                Graph::TextureHandle velocity; /**< UV motion since last frame. */
                Graph::TextureHandle history;  /**< Last frame's output, or invalid. */
                Graph::TextureHandle output;   /**< Where the reconstruction is written. */
                Graph::BufferHandle scene;     /**< The per-frame scene block. */
                Graph::BufferHandle temporal;  /**< The per-frame temporal block. */

                std::uint32_t render_width = 1;
                std::uint32_t render_height = 1;
                std::uint32_t output_width = 1;
                std::uint32_t output_height = 1;

                float jitter[2] = {0.0f, 0.0f};          /**< This frame's sub-pixel offset, NDC. */
                float previous_jitter[2] = {0.0f, 0.0f}; /**< Last frame's, for the reprojection. */
                float exposure = 1.0f;                   /**< Linear exposure the frame resolves at. */
                bool reset = false;                      /**< Discard whatever history exists. */
            };

            /**
             * @brief A reconstruction backend: render-extent inputs in, output-extent image out.
             *
             * Implementations register their own graph passes, so a backend that needs three
             * dispatches and one that needs a single fullscreen draw look identical from the
             * outside (LSP). The renderer holds the interface and never names an
             * implementation.
             */
            class IUpscaler
            {
                public:
                    virtual ~IUpscaler() = default;

                    /** @brief Human-readable backend name, for the editor's readout. */
                    virtual const char* name() const noexcept = 0;

                    /**
                     * @brief Registers whatever passes this backend needs for one frame.
                     * @param graph  The frame graph being built.
                     * @param frame  The frame being registered into.
                     * @param inputs The resources and parameters to reconstruct from.
                     */
                    virtual void register_upscale(Graph::RenderGraph& graph,
                                                  const FrameContext& frame,
                                                  const UpscaleInputs& inputs) = 0;
            };

        } // namespace Frame
    } // namespace Render
} // namespace SushiEngine
