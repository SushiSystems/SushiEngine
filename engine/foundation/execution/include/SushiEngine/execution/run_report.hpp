/**************************************************************************/
/* run_report.hpp                                                         */
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

#include <cstddef>

namespace SushiEngine
{
    namespace Execution
    {
        /**
         * @brief What one graph run did, in terms every backend can answer.
         *
         * Deliberately the intersection rather than the union: a thread pool has no
         * NUMA breakdown and no device transfer counters, and a report carrying fields
         * only one backend can fill would make callers backend-specific by accident.
         * Anything richer belongs to the backend that produces it and is read through
         * that backend's own surface.
         */
        struct RunReport
        {
            bool        is_successful         = true; /**< Whether the run completed without error. */
            bool        cancelled             = false;/**< Whether the run stopped early on a cancellation request. */
            std::size_t total_tasks_executed  = 0;    /**< Nodes actually executed this run. */
            double      total_duration_ms     = 0.0;  /**< Wall-clock time of the whole run. */
            double      compile_duration_ms   = 0.0;  /**< Time spent analysing and compiling the graph. */
        };
    } // namespace Execution
} // namespace SushiEngine
