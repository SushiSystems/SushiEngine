/**************************************************************************/
/* upscaler.cpp                                                           */
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

#include "frame/upscaler.hpp"

namespace SushiEngine
{
    namespace Render
    {
        namespace Frame
        {
            UpscalerAvailability upscaler_availability(UpscaleMode mode) noexcept
            {
                // A vendor backend is present only when its SDK was found at configure time
                // and the definition below was set with it. None is vendored: FSR, DLSS, and
                // XeSS each ship as a separate redistributable under its own licence, and
                // pulling one in is a project decision, not a renderer default. The interface
                // they implement exists either way, which is the point — adopting one is a
                // new file here, not a change to the frame loop.
                switch (mode)
                {
                    case UpscaleMode::None:
                    case UpscaleMode::Temporal:
                        return {true, ""};

                    case UpscaleMode::Fsr:
#if defined(SUSHI_WITH_FSR)
                        return {true, ""};
#else
                        return {false, "the FidelityFX SDK is not in this build"};
#endif

                    case UpscaleMode::Dlss:
#if defined(SUSHI_WITH_DLSS)
                        return {true, ""};
#else
                        return {false, "the Streamline/DLSS SDK is not in this build"};
#endif

                    case UpscaleMode::Xess:
#if defined(SUSHI_WITH_XESS)
                        return {true, ""};
#else
                        return {false, "the XeSS SDK is not in this build"};
#endif
                }
                return {true, ""};
            }

            UpscaleMode resolve_upscale_mode(UpscaleMode requested) noexcept
            {
                return upscaler_availability(requested).available ? requested
                                                                  : UpscaleMode::Temporal;
            }

            const char* upscale_mode_name(UpscaleMode mode) noexcept
            {
                switch (mode)
                {
                    case UpscaleMode::None:
                        return "None";
                    case UpscaleMode::Temporal:
                        return "Temporal (built-in)";
                    case UpscaleMode::Fsr:
                        return "FSR 3.1";
                    case UpscaleMode::Dlss:
                        return "DLSS";
                    case UpscaleMode::Xess:
                        return "XeSS";
                }
                return "Temporal (built-in)";
            }
        } // namespace Frame
    } // namespace Render
} // namespace SushiEngine
