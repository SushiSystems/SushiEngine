/**************************************************************************/
/* constraint.hpp                                                        */
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

#include <cstdint>

#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A constraint holding two bodies a fixed distance apart.
         *
         * The building block of ropes, cloth, and rigid links. The solver reads the
         * two body indices to colour the constraint (no two constraints sharing a
         * body may run together) and to project their positions toward @ref
         * rest_length. Any constraint type the solver accepts exposes its two body
         * indices as the public fields `a` and `b`; new constraint types follow this
         * shape so the colouring and the solve loop are reused unchanged.
         */
        struct DistanceConstraint
        {
            std::uint32_t a = 0;          /**< First body index. */
            std::uint32_t b = 0;          /**< Second body index. */
            Scalar rest_length = Scalar(0); /**< Target distance between the bodies. */
        };
    } // namespace Physics
} // namespace SushiEngine
