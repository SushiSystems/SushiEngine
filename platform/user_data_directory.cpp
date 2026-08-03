/**************************************************************************/
/* user_data_directory.cpp                                                */
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

#include "user_data_directory.hpp"

#include <SDL.h>

namespace SushiEngine
{
    namespace Platform
    {
        std::string user_data_directory(const char* organization, const char* application)
        {
            char* path = SDL_GetPrefPath(organization, application);
            if (path == nullptr)
                return std::string();
            std::string result(path);
            SDL_free(path);
            return result;
        }
    } // namespace Platform
} // namespace SushiEngine
