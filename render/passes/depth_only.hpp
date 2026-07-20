/**************************************************************************/
/* depth_only.hpp                                                         */
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
 * @file depth_only.hpp
 * @brief What the two depth-writing passes share: a pipeline shape and a push constant.
 *
 * The depth prepass and the shadow cascades differ in which matrix they project by and
 * nothing else — both draw the same geometry, write only depth, and read no material.
 * Sharing the description also lets the pipeline cache reuse one vertex-input half
 * across both.
 */

#include <vulkan/vulkan.h>

#include <SushiEngine/core/types.hpp>

#include "resources/pipeline_cache.hpp"
#include "scene/scene_layout.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Passes
        {
            /** @brief The stages a depth-only draw's push constants must reach. */
            constexpr VkShaderStageFlags DEPTH_PUSH_STAGES =
                VK_SHADER_STAGE_VERTEX_BIT | VK_SHADER_STAGE_FRAGMENT_BIT;

            /**
             * @brief Describes a pipeline that writes depth and no colour.
             *
             * No fragment shader at all: a depth buffer is the fixed-function depth write
             * and nothing more, and leaving the stage out is both the fastest path and
             * the one that cannot accidentally shade.
             *
             * @param layout       The shared scene pipeline layout.
             * @param vertex       The vertex shader module.
             * @param depth_format Format of the depth attachment.
             * @return The description to build the pipeline from.
             */
            Resources::GraphicsPipelineDesc depth_only_pipeline_desc(VkPipelineLayout layout,
                                                                     VkShaderModule vertex,
                                                                     VkFormat depth_format);

            /**
             * @brief Fills a push constant for a depth-only draw.
             *
             * Only the transform matters, and it is made camera-relative exactly as the
             * shading pass does — the eye subtracted in double before the float cast — so
             * the two passes' clip positions agree bit for bit. The motion index is set
             * to zero rather than left undefined: the vertex shader is shared with the
             * shading pass and still reads the motion array, and index zero always
             * exists.
             *
             * @param model    Object-to-world transform, absolute.
             * @param eye      Camera world position; zeros for already-relative geometry.
             * @param cascade  Which shadow cascade this draw targets, ignored elsewhere.
             * @return The filled push constant.
             */
            Scene::MeshPushConstants depth_only_push(const Mat4& model, const double eye[3],
                                                     float cascade = 0.0f);
        } // namespace Passes
    } // namespace Render
} // namespace SushiEngine
