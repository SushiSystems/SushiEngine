/**************************************************************************/
/* autosave.hpp                                                           */
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

#ifndef SUSHIENGINE_AUTHORING_AUTOSAVE_HPP
#define SUSHIENGINE_AUTHORING_AUTOSAVE_HPP

namespace SushiEngine
{
    namespace Authoring
    {
        /**
         * @brief The autosave decision, as a tickable timer free of the editor loop.
         *
         * The whole policy in one testable place: the clock runs only while a save
         * would be *meaningful* — autosave enabled, the scene has a path (so firing
         * never pops the Save-As modal), and the scene is dirty (so a clean scene
         * never rewrites its file). Ineligibility resets the clock, so the interval
         * always measures continuous dirty time rather than time since some earlier,
         * already-saved edit.
         */
        class AutosaveTimer
        {
            public:
                /**
                 * @brief Advances the timer and reports whether an autosave is due.
                 *
                 * @param eligible Whether a save would be meaningful this frame
                 *     (enabled && has a path && dirty). False resets the clock.
                 * @param delta_seconds Real seconds since the last tick.
                 * @param interval_seconds The authored autosave interval.
                 * @return True exactly when the accumulated eligible time reaches the
                 *     interval; the clock restarts so the next fire is one full
                 *     interval later.
                 */
                bool tick(bool eligible, double delta_seconds, float interval_seconds) noexcept
                {
                    if (!eligible)
                    {
                        accumulated_seconds_ = 0.0;
                        return false;
                    }
                    accumulated_seconds_ += delta_seconds;
                    if (accumulated_seconds_ < static_cast<double>(interval_seconds))
                        return false;
                    accumulated_seconds_ = 0.0;
                    return true;
                }

            private:
                double accumulated_seconds_ = 0.0;
        };
    } // namespace Authoring
} // namespace SushiEngine

#endif
