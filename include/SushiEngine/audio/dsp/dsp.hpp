/**************************************************************************/
/* dsp.hpp                                                               */
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

#ifndef SUSHIENGINE_AUDIO_DSP_DSP_HPP
#define SUSHIENGINE_AUDIO_DSP_DSP_HPP

/**
 * @file dsp.hpp
 * @brief Umbrella for the portable CPU DSP core (`SushiEngine::Audio::Dsp`).
 *
 * One include pulls in the phase-S1 base: the real-time primitives (the flush-to-zero
 * denormal guard and the lock-free SPSC ring), the SIMD sample kernels, the filter
 * set (one-pole, RBJ biquad, Cytomic TPT state-variable), and the block processing
 * graph with its built-in nodes. All header-only, portable C++17, no SDL and no
 * SushiRuntime. See `docs/design/audio_system.md` §3.
 */

#include <SushiEngine/audio/dsp/air_absorption.hpp>
#include <SushiEngine/audio/dsp/denormals.hpp>
#include <SushiEngine/audio/dsp/filters/biquad.hpp>
#include <SushiEngine/audio/dsp/filters/one_pole.hpp>
#include <SushiEngine/audio/dsp/filters/state_variable.hpp>
#include <SushiEngine/audio/dsp/fractional_delay.hpp>
#include <SushiEngine/audio/dsp/graph.hpp>
#include <SushiEngine/audio/dsp/nodes.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>
#include <SushiEngine/audio/dsp/spherical_harmonics.hpp>
#include <SushiEngine/audio/dsp/spsc_ring.hpp>

#endif
