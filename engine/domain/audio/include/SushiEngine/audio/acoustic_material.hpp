/**************************************************************************/
/* acoustic_material.hpp                                                  */
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

#ifndef SUSHIENGINE_AUDIO_ACOUSTIC_MATERIAL_HPP
#define SUSHIENGINE_AUDIO_ACOUSTIC_MATERIAL_HPP

/**
 * @file acoustic_material.hpp
 * @brief The per-surface acoustic material — three frequency bands of absorption,
 *        scattering, and transmission.
 *
 * A surface interacts with sound in three ways, and each is frequency-dependent, so
 * every coefficient is carried as a three-band triple centred on ≈ 400 Hz (low),
 * 2.5 kHz (mid), and 15 kHz (high) — the split the whole occlusion/reverb layer speaks
 * (see `docs/slop/audio_system.md` §6):
 *
 *   - **absorption** — the fraction of incident energy the surface *removes* (the rest
 *     reflects); feeds the room-geometry RT60 (`reverb.hpp`).
 *   - **scattering** — the fraction that reflects *diffusely* rather than specularly;
 *     reserved for the image-source/diffuse-rain split of the early-reflection model.
 *   - **transmission** — the fraction that passes *through* the surface. This is what a
 *     blocked path accumulates: because most materials pass low frequencies far more
 *     readily than high ones, through-wall sound is bassy *for free* — the defaults
 *     encode exactly that (low ≫ high).
 *
 * The struct is a plain aggregate of `float`s (trivially copyable), so it can sit inline
 * on ECS geometry or in a flat material table with no engine dependency. The presets are
 * order-of-magnitude realistic, not laboratory-measured — a sound designer retunes them.
 */

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief The number of frequency bands every acoustic coefficient carries. */
        constexpr int ACOUSTIC_BAND_COUNT = 3;

        /**
         * @brief The band-centre frequencies (Hz) the three-band coefficients describe.
         *
         * Low ≈ 400 Hz, mid ≈ 2.5 kHz, high ≈ 15 kHz. The occlusion 3-band EQ places its
         * crossovers between these so each band's gain tracks the matching coefficient.
         */
        constexpr float ACOUSTIC_BAND_HZ[ACOUSTIC_BAND_COUNT] = {400.0f, 2500.0f, 15000.0f};

        /**
         * @brief A surface's frequency-dependent acoustic response.
         *
         * Every coefficient is in [0, 1] per band. @ref transmission is the field the
         * occlusion path reads; @ref absorption feeds RT60; @ref scattering is carried for
         * the diffuse early-reflection split. The default is a mildly absorptive,
         * mostly-opaque generic surface.
         */
        struct AcousticMaterial
        {
            float absorption[ACOUSTIC_BAND_COUNT] = {0.10f, 0.10f, 0.12f};   /**< Energy removed per band. */
            float scattering[ACOUSTIC_BAND_COUNT] = {0.10f, 0.20f, 0.30f};   /**< Diffuse-reflection fraction. */
            float transmission[ACOUSTIC_BAND_COUNT] = {0.05f, 0.01f, 0.002f}; /**< Energy passing through. */

            /** @brief A neutral, mostly-opaque generic surface. */
            static AcousticMaterial generic() { return AcousticMaterial{}; }

            /** @brief Dense concrete: highly reflective, near-opaque, bassy leak-through. */
            static AcousticMaterial concrete()
            {
                AcousticMaterial m;
                m.absorption[0] = 0.02f; m.absorption[1] = 0.03f; m.absorption[2] = 0.05f;
                m.scattering[0] = 0.10f; m.scattering[1] = 0.15f; m.scattering[2] = 0.20f;
                m.transmission[0] = 0.02f; m.transmission[1] = 0.004f; m.transmission[2] = 0.0005f;
                return m;
            }

            /** @brief Wood panelling: moderate absorption, some low-frequency leak. */
            static AcousticMaterial wood()
            {
                AcousticMaterial m;
                m.absorption[0] = 0.15f; m.absorption[1] = 0.10f; m.absorption[2] = 0.10f;
                m.transmission[0] = 0.12f; m.transmission[1] = 0.03f; m.transmission[2] = 0.006f;
                return m;
            }

            /** @brief Glass: reflective, thin — passes more mid/high than concrete. */
            static AcousticMaterial glass()
            {
                AcousticMaterial m;
                m.absorption[0] = 0.03f; m.absorption[1] = 0.03f; m.absorption[2] = 0.02f;
                m.transmission[0] = 0.15f; m.transmission[1] = 0.06f; m.transmission[2] = 0.02f;
                return m;
            }

            /** @brief Heavy curtain / drape: very absorptive up high, acoustically soft. */
            static AcousticMaterial curtain()
            {
                AcousticMaterial m;
                m.absorption[0] = 0.20f; m.absorption[1] = 0.55f; m.absorption[2] = 0.75f;
                m.scattering[0] = 0.30f; m.scattering[1] = 0.50f; m.scattering[2] = 0.60f;
                m.transmission[0] = 0.30f; m.transmission[1] = 0.10f; m.transmission[2] = 0.02f;
                return m;
            }

            /** @brief Sheet metal: bright, reflective, rings — low absorption. */
            static AcousticMaterial metal()
            {
                AcousticMaterial m;
                m.absorption[0] = 0.02f; m.absorption[1] = 0.02f; m.absorption[2] = 0.03f;
                m.transmission[0] = 0.10f; m.transmission[1] = 0.02f; m.transmission[2] = 0.003f;
                return m;
            }

            /**
             * @brief A fully-open "surface": no absorption, total transmission.
             *
             * The material a portal/doorway triangle carries so a ray through an opening
             * is treated as unblocked (transmission 1 in every band).
             */
            static AcousticMaterial open()
            {
                AcousticMaterial m;
                m.absorption[0] = m.absorption[1] = m.absorption[2] = 0.0f;
                m.scattering[0] = m.scattering[1] = m.scattering[2] = 0.0f;
                m.transmission[0] = m.transmission[1] = m.transmission[2] = 1.0f;
                return m;
            }

            /**
             * @brief The broadband (mid-band) mean absorption, for a scalar RT60 estimate.
             * @return The absorption at the mid band.
             */
            float mean_absorption() const noexcept { return absorption[1]; }
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
