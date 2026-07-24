/**************************************************************************/
/* audio.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_AUDIO_HPP
#define SUSHIENGINE_AUDIO_AUDIO_HPP

/**
 * @file audio.hpp
 * @brief Umbrella for the header-only audio subsystem (`SushiEngine::Audio`).
 *
 * One include pulls in the device/accelerator seams (§12–§13), the portable DSP core
 * (§3, `dsp/dsp.hpp`), and the header-only action layer built on top of it (phase S2):
 * parameter smoothing / RTPC, voices and their sources, the mixer bus DAG, the voice
 * manager, and the @ref SushiEngine::Audio::AudioEngine that renders them as one
 * @ref SushiEngine::Audio::IAudioRenderer. No SDL and no SushiRuntime — the SDL2 device
 * backend (`sushi_audio`) is linked separately, exactly as the input backend is.
 * See `docs/design/audio_system.md`.
 */

#include <SushiEngine/audio/accelerator.hpp>
#include <SushiEngine/audio/device.hpp>
#include <SushiEngine/audio/dsp/dsp.hpp>
#include <SushiEngine/audio/engine.hpp>
#include <SushiEngine/audio/mixer.hpp>
#include <SushiEngine/audio/parameter.hpp>
#include <SushiEngine/audio/propagation.hpp>
#include <SushiEngine/audio/spatializer.hpp>
#include <SushiEngine/audio/voice.hpp>
#include <SushiEngine/audio/voice_manager.hpp>

#endif
