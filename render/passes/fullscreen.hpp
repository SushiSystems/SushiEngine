/**************************************************************************/
/* fullscreen.hpp                                                         */
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
 * @file fullscreen.hpp
 * @brief The pipeline description every fullscreen post pass shares.
 *
 * A fullscreen pass is a vertex-only triangle with no vertex input, no depth, and
 * one colour attachment; the only things that vary are the fragment shader and the
 * target's format. Sharing the description here is also what lets the pipeline
 * library cache reuse one vertex-input and pre-rasterisation library across the
 * sky, cloud, and tonemap pipelines.
 */

#include <vulkan/vulkan.h>

#include "resources/pipeline_cache.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            /**
             * @brief Describes a fullscreen-triangle pipeline.
             * @param layout       The shared scene pipeline layout.
             * @param vertex       The fullscreen vertex shader module.
             * @param fragment     The effect's fragment shader module.
             * @param color_format Format of the single colour attachment.
             * @return The description to build the pipeline from.
             */
            Resources::GraphicsPipelineDesc fullscreen_pipeline_desc(VkPipelineLayout layout,
                                                                     VkShaderModule vertex,
                                                                     VkShaderModule fragment,
                                                                     VkFormat color_format);
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
