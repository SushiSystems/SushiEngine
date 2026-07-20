/**************************************************************************/
/* picking_pass.hpp                                                       */
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
 * @file picking_pass.hpp
 * @brief Copies the id target into the host-visible buffer a click resolves against.
 *
 * The pass has a side effect the graph cannot observe — the host reads the buffer
 * on a later frame — so it declares itself non-cullable. Doing the copy inside the
 * frame's own submit is what lets pick() answer without a submit of its own.
 */

#include "passes/render_pass.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            /** @brief Transfers the picking ids to host-visible memory. */
            class PickingPass final : public IRenderPass
            {
                public:
                    void register_pass(Graph::RenderGraph& graph,
                                       const Frame::FrameContext& frame) override;
            };
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
