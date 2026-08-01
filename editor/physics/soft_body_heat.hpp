/**************************************************************************/
/* soft_body_heat.hpp                                                     */
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
 * @file soft_body_heat.hpp
 * @brief What a soft-body debug view *means*, with no drawing in it.
 *
 * Split from `soft_body_overlay.hpp` along the line that matters: everything here
 * is a decision about how to read a number, and everything there is a decision
 * about how to put a line on screen. The split is not only tidiness — the reading
 * is the part with a right answer, so it belongs somewhere a test can reach
 * without a UI toolkit, a device, or a window.
 */

#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/soft/soft_body_material.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /** @brief Which of §9.3/§9.4's debug views the viewport draws for a soft body. */
        enum class SoftBodyDebugView : std::uint32_t
        {
            Off,          /**< Nothing drawn; the default, because an overlay always on is an overlay ignored. */
            Wireframe,    /**< The tetrahedral lattice, uniformly coloured. */
            Stress,       /**< Von Mises stress against the material's yield stress (§9.3). */
            PlasticStrain /**< Accumulated permanent strain against the hardening ceiling (§9.4). */
        };

        /** @brief A linear-RGB colour in `[0, 1]`, before any toolkit packs it. */
        struct HeatColour
        {
            float r = 0.0f;
            float g = 0.0f;
            float b = 0.0f;
        };

        /** @brief Clamps to `[0, 1]`, mapping NaN to zero rather than through it. */
        inline float clamp_unit(float value) noexcept
        {
            // Written as a positive test so NaN — which compares false against
            // everything — falls to zero instead of propagating into a colour channel.
            if (!(value > 0.0f))
                return 0.0f;
            return value > 1.0f ? 1.0f : value;
        }

        /**
         * @brief Maps a normalized reading to a heat colour.
         *
         * Blue through cyan, green and yellow to red — the ordering every engineering
         * plot uses, so its direction needs no legend. Four straight segments rather
         * than a smooth formula, because the corner colours are the ones a reader
         * recognises: a curve that only approaches pure green makes "that is green, so
         * it is halfway" a guess instead of a reading.
         *
         * Values outside `[0, 1]` clamp rather than wrap. A reading past full scale is
         * still the worst reading, and wrapping it back to blue would draw the most
         * broken element as the calmest one on screen.
         */
        inline HeatColour heat_colour(float t) noexcept
        {
            t = clamp_unit(t);
            HeatColour colour;
            if (t < 0.25f)
            {
                colour.b = 1.0f;
                colour.g = t / 0.25f;
            }
            else if (t < 0.5f)
            {
                colour.g = 1.0f;
                colour.b = 1.0f - (t - 0.25f) / 0.25f;
            }
            else if (t < 0.75f)
            {
                colour.g = 1.0f;
                colour.r = (t - 0.5f) / 0.25f;
            }
            else
            {
                colour.r = 1.0f;
                colour.g = 1.0f - (t - 0.75f) / 0.25f;
            }
            return colour;
        }

        /**
         * @brief Normalizes an element's reading for @p view against the material.
         *
         * **Full scale is a material threshold, not the body's own maximum**, and that
         * is the whole design of these views. Normalising against the maximum means
         * something is always red — including a body at rest — so the scale would be
         * answering "which element is worst" when the question is "is any of them in
         * trouble". Against yield, red means *this is deforming permanently*, which is
         * what red means in every other engineering plot; against the hardening ceiling,
         * red means *this element cannot take on any more permanent strain*.
         *
         * A material with no yield stress is purely elastic: it has no threshold to be
         * measured against, so there is no honest normalization and this answers zero
         * rather than dividing by zero or inventing a scale.
         *
         * @param view     Which reading to normalize; @ref SoftBodyDebugView::Off and
         *                 @ref SoftBodyDebugView::Wireframe answer zero.
         * @param sample   The element.
         * @param material The body's constitutive parameters.
         * @return The reading in `[0, 1]`, clamped.
         */
        inline float soft_body_element_intensity(
            SoftBodyDebugView view, const Simulation::SoftBodyElementSample& sample,
            const Physics::SoftBodyMaterialT<Scalar>& material) noexcept
        {
            if (view == SoftBodyDebugView::Stress)
            {
                if (!(material.yield_stress > Scalar(0)))
                    return 0.0f;
                return clamp_unit(static_cast<float>(double(sample.von_mises_stress) /
                                                     double(material.yield_stress)));
            }
            if (view == SoftBodyDebugView::PlasticStrain)
            {
                if (!(material.maximum_plastic_strain > Scalar(0)))
                    return 0.0f;
                return clamp_unit(static_cast<float>(double(sample.plastic_strain) /
                                                     double(material.maximum_plastic_strain)));
            }
            return 0.0f;
        }
    } // namespace Editor
} // namespace SushiEngine
