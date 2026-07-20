/**************************************************************************/
/* fullscreen.cpp                                                         */
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

#include "passes/fullscreen.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            Resources::GraphicsPipelineDesc fullscreen_pipeline_desc(VkPipelineLayout layout,
                                                                     VkShaderModule vertex,
                                                                     VkShaderModule fragment,
                                                                     VkFormat color_format)
            {
                Resources::GraphicsPipelineDesc desc;
                desc.layout = layout;
                desc.vertex_shader = vertex;
                desc.fragment_shader = fragment;
                desc.color_count = 1;
                desc.color_formats[0] = color_format;
                return desc;
            }
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
