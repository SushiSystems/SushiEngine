/**************************************************************************/
/* picking_pass.cpp                                                       */
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

#include "passes/picking_pass.hpp"

#include "frame/frame_context.hpp"
#include "graph/render_graph.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            void PickingPass::register_pass(Graph::RenderGraph& graph,
                                            const Frame::FrameContext& frame)
            {
                graph.add_pass(
                    "picking readback",
                    [&](Graph::RenderPassBuilder& builder)
                    {
                        builder.read(frame.targets.id, Graph::TextureAccess::TransferSource);
                        builder.write(frame.targets.readback,
                                      Graph::BufferAccess::TransferDestination);
                        builder.set_side_effect();
                    },
                    [&frame](VkCommandBuffer cmd, const Graph::PassContext& context)
                    {
                        VkBufferImageCopy copy{};
                        copy.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
                        copy.imageSubresource.layerCount = 1;
                        copy.imageExtent = {frame.width, frame.height, 1};
                        vkCmdCopyImageToBuffer(cmd, context.image(frame.targets.id),
                                               VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                                               context.buffer(frame.targets.readback), 1, &copy);
                    });
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
