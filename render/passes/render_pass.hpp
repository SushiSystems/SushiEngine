/**************************************************************************/
/* render_pass.hpp                                                        */
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
 * @file render_pass.hpp
 * @brief The one contract every render pass honours.
 *
 * A pass registers itself into the frame's graph and is otherwise opaque: it owns
 * its own pipelines, decides its own resource declarations, and never learns what
 * runs before or after it. Adding an effect is adding one of these and registering
 * it — no neighbouring pass changes.
 */

namespace SushiEngine
{
    namespace Render
    {
        namespace Graph
        {
            class PassContext;
            class RenderGraph;
        }

        namespace Frame
        {
            struct FrameContext;
        }

        namespace Passes
        {
            /** @brief A unit of rendering that can register itself into a frame graph. */
            class IRenderPass
            {
                public:
                    virtual ~IRenderPass() = default;

                    /**
                     * @brief Declares this pass's resources and records its execute function.
                     * @param graph The frame graph being built.
                     * @param frame This frame's camera, targets, draw list, and allocators.
                     */
                    virtual void register_pass(Graph::RenderGraph& graph,
                                               const Frame::FrameContext& frame) = 0;

                    /**
                     * @brief Rebuilds pipelines after their shaders were hot-reloaded.
                     *
                     * The caller has already idled the device; a pass that owns no
                     * pipelines may leave the default no-op in place.
                     */
                    virtual void rebuild_pipelines() {}
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
