/**************************************************************************/
/* vfx.hpp                                                                */
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
 * @file vfx.hpp
 * @brief Umbrella header for the VFX particle system's authoring model (SushiEngine::Vfx).
 *
 * Pulls in, in dependency order, the artist-facing effect model (curves, gradients, modules,
 * emitter/effect descriptors), the POD compilation boundary (@ref CompiledEffect), the
 * compiler, the deterministic RNG, and the asset registry. Everything here is header-only
 * plain C++17 and depends only on `core/types.hpp` — nothing here touches the renderer, the
 * ECS, or the SushiRuntime. The design lives in `docs/design/vfx_particle_system.md`.
 *
 * The two simulation backends consume this model but live elsewhere: the deterministic CPU
 * integrator alongside the sim, and the GPU cosmetic system under `render/scene/`.
 */

#include <SushiEngine/vfx/asset_id.hpp>
#include <SushiEngine/vfx/random.hpp>
#include <SushiEngine/vfx/curve.hpp>
#include <SushiEngine/vfx/gradient.hpp>
#include <SushiEngine/vfx/modules.hpp>
#include <SushiEngine/vfx/emitter_descriptor.hpp>
#include <SushiEngine/vfx/particle_effect.hpp>
#include <SushiEngine/vfx/compiled_emitter.hpp>
#include <SushiEngine/vfx/emitter_compiler.hpp>
#include <SushiEngine/vfx/effect_database.hpp>
