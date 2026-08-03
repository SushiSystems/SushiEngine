/**************************************************************************/
/* resource_state.hpp                                                     */
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
 * @file resource_state.hpp
 * @brief Translates a declared graph access into the Vulkan synchronisation triple.
 *
 * This is the whole of the graph's barrier knowledge: one total function from
 * TextureAccess/BufferAccess to (stage, access mask, layout), plus the predicates
 * that say whether an access writes and what image usage it implies. Isolating it
 * here means a new access kind is one table entry, not an edit to the graph.
 */

#include "resource_handle.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Graph
        {
            /**
             * @brief The synchronisation state a texture is in for one access.
             *
             * @c queue records which queue left it in that state. A barrier recorded on the
             * other queue cannot name the stages of the first — a compute command buffer may
             * not wait on the fragment-test stage the depth prepass wrote in — and does not
             * need to: the two are already ordered by the submission timeline, so the barrier
             * degenerates to the layout transition alone.
             */
            struct TextureState
            {
                VkPipelineStageFlags2 stage = VK_PIPELINE_STAGE_2_NONE;
                VkAccessFlags2 access = VK_ACCESS_2_NONE;
                VkImageLayout layout = VK_IMAGE_LAYOUT_UNDEFINED;
                PassQueue queue = PassQueue::Graphics;
            };

            /** @brief The synchronisation state a buffer is in for one access. */
            struct BufferState
            {
                VkPipelineStageFlags2 stage = VK_PIPELINE_STAGE_2_NONE;
                VkAccessFlags2 access = VK_ACCESS_2_NONE;
                PassQueue queue = PassQueue::Graphics;
            };

            /**
             * @brief The stage, access mask, and layout one texture access implies.
             * @param access The declared access.
             * @return The synchronisation triple to transition into.
             */
            TextureState texture_access_state(TextureAccess access) noexcept;

            /**
             * @brief The stage and access mask one buffer access implies.
             * @param access The declared access.
             * @return The synchronisation pair to transition into.
             */
            BufferState buffer_access_state(BufferAccess access) noexcept;

            /**
             * @brief Whether a texture access writes, so a following access must wait.
             * @param access The declared access.
             * @return true if the access can modify the texture's contents.
             */
            bool texture_access_writes(TextureAccess access) noexcept;

            /**
             * @brief Whether a buffer access writes, so a following access must wait.
             * @param access The declared access.
             * @return true if the access can modify the buffer's contents.
             */
            bool buffer_access_writes(BufferAccess access) noexcept;

            /**
             * @brief The image usage bit a texture access requires at creation time.
             *
             * The graph unions these across every declared access before allocating, so
             * a transient's usage flags always cover how it is actually used.
             *
             * @param access The declared access.
             * @return The VkImageUsageFlags bit implied by @p access.
             */
            VkImageUsageFlags texture_access_usage(TextureAccess access) noexcept;

            /**
             * @brief The buffer usage bit a buffer access requires at creation time.
             * @param access The declared access.
             * @return The VkBufferUsageFlags bit implied by @p access.
             */
            VkBufferUsageFlags buffer_access_usage(BufferAccess access) noexcept;
        } // namespace Graph
    } // namespace Render
} // namespace SushiEngine
