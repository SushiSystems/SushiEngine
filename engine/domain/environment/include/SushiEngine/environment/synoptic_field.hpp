/**************************************************************************/
/* synoptic_field.hpp                                                     */
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
 * @file synoptic_field.hpp
 * @brief The render seam's view of where the planet's weather is.
 *
 * The same placement `Atmosphere::SynopticField` computes, rotated into **scene space** and
 * narrowed to what a shader needs. Two differences from the simulation's own form, and both
 * are the reason this type exists rather than the other one crossing the seam:
 *
 * * **Direction, not latitude and longitude.** The march has a sample's radial unit vector in
 *   hand and nothing else; making it reconstruct a geographic coordinate would mean carrying
 *   the body's full frame into the cloud shader and doing two inverse trigonometric functions
 *   per centre per sample. A dot product against a precomputed direction is the same answer for
 *   one instruction. The rotation is the producer's job because the producer is the only side
 *   that already holds `Environment::planet_body_axes`.
 * * **By value, not borrowed.** `WeatherField` is borrowed because it is tens of kilobytes on a
 *   multi-second cadence; this is twelve vectors, so a pointer and a lifetime rule would cost
 *   more to reason about than the copy costs to make.
 *
 * A field with @ref count of zero is the honest description of a provider that has no planetary
 * structure to publish — a dynamical core whose solution lives on a 384 km nest, say. The
 * renderer reads it as "fall back to the zonal climatology alone", which is still true of every
 * planet with an atmosphere, rather than as an error.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Render
    {
        /** @brief Upper bound on published centres; mirrors `Atmosphere::SYNOPTIC_MAX_CENTRES`. */
        constexpr int SYNOPTIC_FIELD_MAX_CENTRES = 12;

        /**
         * @brief One pressure system as the cloud march reads it.
         *
         * Eight floats so the pair of `vec4`s the scene block carries is a straight copy with no
         * repacking at upload time.
         */
        struct SynopticFieldCentre
        {
            float direction[3]{};      /**< Unit vector from the planet centre, **scene space**. */
            float falloff = 0.0f;      /**< Weight is `exp(-falloff * (1 - dot(radial, direction)))`. */
            float amplitude = 0.0f;    /**< Signed coverage anomaly at the centre; + a low, − a high. */
            float convective = 0.0f;   /**< Convective fraction this system brings, [0, 1]. */
            float precipitation = 0.0f;/**< Surface precipitation this system brings, [0, 1]. */
            float padding = 0.0f;      /**< Keeps the pair `vec4`-shaped; unread. */
        };

        /**
         * @brief Where the weather is over the whole body, for the parts of the march no baked
         *        window reaches.
         *
         * Trivially copyable, and small enough to ride in `Environment` by value.
         */
        struct SynopticFieldView
        {
            SynopticFieldCentre centres[SYNOPTIC_FIELD_MAX_CENTRES]{};
            std::int32_t count = 0;        /**< Populated entries; 0 means "no placement published". */
            float itcz_latitude = 0.0f;    /**< Where the convergence zone sits, radians — the seasonal term. */
            /**
             * @brief Whether the zonal climatology should be applied at all.
             *
             * Separate from @ref count because the two say different things. A body with an
             * atmosphere always has a latitudinal cloud structure even with nothing placed on
             * it, so `count == 0` is not the same statement as "this body has no weather" —
             * which is what an airless one needs to be able to say.
             */
            bool valid = false;
        };
    } // namespace Render
} // namespace SushiEngine
