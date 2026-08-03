/**************************************************************************/
/* upscaler_info.hpp                                                      */
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
 * @file upscaler_info.hpp
 * @brief Which reconstruction backends this build carries, and what a request resolves to.
 *
 * Public rather than private for the same reason @c quality_params.hpp is: the editor has
 * to be able to say what the authored upscaler actually resolved to and why, and it
 * cannot see the renderer's private headers. The backend *interface* those answers are
 * about stays inside the renderer (`render/frame/upscaler.hpp`); this is only the
 * capability query, so nothing here names a graph resource or a Vulkan type.
 */

#include <SushiEngine/render/render_settings.hpp>

namespace SushiEngine
{
    namespace Render
    {
        namespace Frame
        {
            /**
             * @brief Whether a requested upscaler backend exists in this build, and why not.
             *
             * The vendor backends are named by @c UpscaleMode whether or not the build
             * carries their SDK, so a project may author one unconditionally and a build that
             * lacks it degrades instead of failing. The reason string is what the editor
             * shows beside the fallback.
             */
            struct UpscalerAvailability
            {
                bool available = false;
                const char* reason = "";
            };

            /**
             * @brief Reports whether a backend is present in this build.
             * @param mode The authored upscale mode.
             * @return Availability and, when absent, the reason it is.
             */
            UpscalerAvailability upscaler_availability(UpscaleMode mode) noexcept;

            /**
             * @brief The mode that will actually run, given what the build carries.
             *
             * An unavailable vendor backend resolves to @c UpscaleMode::Temporal — the
             * engine's own reconstruction, which is always present — rather than to no
             * upscaling at all, because the frame is already being rendered below the output
             * extent by the time this is asked and something has to reconstruct it.
             *
             * @param requested The authored mode.
             * @return The mode the frame runs with.
             */
            UpscaleMode resolve_upscale_mode(UpscaleMode requested) noexcept;

            /**
             * @brief The display name of an upscale mode.
             * @param mode The mode to name.
             * @return A static, human-readable name.
             */
            const char* upscale_mode_name(UpscaleMode mode) noexcept;
        } // namespace Frame
    } // namespace Render
} // namespace SushiEngine
