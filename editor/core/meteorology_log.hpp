/**************************************************************************/
/* meteorology_log.hpp                                                    */
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

#ifndef SUSHIENGINE_EDITOR_METEOROLOGY_LOG_HPP
#define SUSHIENGINE_EDITOR_METEOROLOGY_LOG_HPP

#include <fstream>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The Meteorology panel's CSV logger: a file handle and a cadence.
         *
         * Owned by `EditorContext` like every other piece of session state, so it is
         * visible where editor state is looked for and dies with the session instead of
         * living in a hidden function-static. Sampled on the *nest's* clock rather than
         * the wall clock, so a log line is a fixed interval of simulated weather however
         * fast the sky is animated. The stream is opened lazily on the first line
         * written, so ticking the toggle and changing one's mind costs no file.
         */
        struct MeteorologyLog
        {
            bool enabled = false;
            float interval_seconds = 10.0f;
            double next_at_simulated = 0.0;
            char path[256] = "meteorology.csv";
            std::ofstream stream;
            bool header_written = false;
        };
    } // namespace Editor
} // namespace SushiEngine

#endif
