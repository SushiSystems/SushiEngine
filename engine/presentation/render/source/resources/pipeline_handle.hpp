/**************************************************************************/
/* pipeline_handle.hpp                                                    */
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
 * @file pipeline_handle.hpp
 * @brief A swappable reference to a factory-owned graphics pipeline.
 *
 * Split out from pipeline_cache.hpp so a pass depends only on the handle it binds
 * through, not the whole factory (its worker thread, queues, and caches). This is the
 * seam the background pipeline optimizer swaps behind: a pass resolves the handle at
 * bind time and always gets the best pipeline built so far.
 */

#include <atomic>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Resources
        {
            class GraphicsPipelineFactory;

            /**
             * @brief A swappable reference to a pipeline the factory owns.
             *
             * GraphicsPipelineFactory::create() hands one of these back instead of a raw
             * handle. A pass resolves it at bind time with get(), so when the background
             * optimizer swaps a fast-linked pipeline for its fully optimized rebuild, every
             * pass that binds through the handle picks the new pipeline up on the next frame
             * — no pipeline re-creation, no stall. Cheap to copy; it owns nothing and stays
             * valid until the factory's slots are torn down (device idle, on shutdown or
             * hot-reload).
             */
            class PipelineHandle
            {
                public:
                    PipelineHandle() = default;

                    /** @brief The pipeline to bind this frame, or VK_NULL_HANDLE if unset. */
                    VkPipeline get() const noexcept
                    {
                        return active_ != nullptr ? active_->load(std::memory_order_acquire)
                                                  : VK_NULL_HANDLE;
                    }

                    /** @brief Whether the handle refers to a created pipeline. */
                    bool valid() const noexcept { return active_ != nullptr; }

                private:
                    friend class GraphicsPipelineFactory;
                    explicit PipelineHandle(std::atomic<VkPipeline>* active) noexcept
                        : active_(active)
                    {
                    }

                    std::atomic<VkPipeline>* active_ = nullptr;
            };
        } // namespace Resources
    } // namespace Render
} // namespace SushiEngine
