/**************************************************************************/
/* vulkan_check.hpp                                                       */
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
 * @file vulkan_check.hpp
 * @brief The renderer's single VkResult-to-exception translation point.
 *
 * Every Vulkan entry point in the render library funnels its result through
 * check(), so a failure surfaces as one std::runtime_error carrying the call site
 * and the numeric result instead of a silently ignored code.
 */

#include <stdexcept>
#include <string>

#include <vulkan/vulkan.h>

namespace SushiEngine
{
    namespace Render
    {
        namespace Vulkan
        {
            /**
             * @brief Throws unless a Vulkan call succeeded.
             * @param result The VkResult returned by the call.
             * @param what   The call site's name, quoted in the exception message.
             */
            inline void check(VkResult result, const char* what)
            {
                if (result != VK_SUCCESS)
                    throw std::runtime_error(std::string("SushiEngine: ") + what +
                                             " failed (VkResult " +
                                             std::to_string(static_cast<int>(result)) + ")");
            }
        } // namespace Vulkan
    } // namespace Render
} // namespace SushiEngine
