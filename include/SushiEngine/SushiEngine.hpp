/**************************************************************************/
/* SushiEngine.hpp                                                        */
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
 * @file SushiEngine.hpp
 * @brief Umbrella header for the engine. One include pulls in the value-type seam,
 *        the full ECS surface (entities, components, archetype storage, the world,
 *        the deferred command buffer, and the system schedule), the physics
 *        constraint solver, and SushiLoop's core (the `Loop::App` authoring API over
 *        a fixed-step deterministic loop, seeded RNG, per-tick input capture,
 *        rollback snapshots, and loopback network reconciliation).
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/ecs/entity.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/chunk.hpp>
#include <SushiEngine/ecs/archetype.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/ecs/command_buffer.hpp>
#include <SushiEngine/ecs/schedule.hpp>
#include <SushiEngine/physics/core/body_flags.hpp>
#include <SushiEngine/physics/core/configuration.hpp>
#include <SushiEngine/physics/core/handle.hpp>
#include <SushiEngine/physics/core/material.hpp>
#include <SushiEngine/physics/core/rigid_body.hpp>
#include <SushiEngine/physics/core/statistics.hpp>
#include <SushiEngine/physics/geometry/mass_properties.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>
#include <SushiEngine/physics/collision/broadphase.hpp>
#include <SushiEngine/physics/collision/contact.hpp>
#include <SushiEngine/physics/collision/contact_solver.hpp>
#include <SushiEngine/physics/collision/narrowphase.hpp>
#include <SushiEngine/physics/constraints/constraint.hpp>
#include <SushiEngine/physics/constraints/xpbd_constraint.hpp>
#include <SushiEngine/physics/solver/graph_coloring.hpp>
#include <SushiEngine/physics/solver/pgs_solver.hpp>
#include <SushiEngine/physics/solver/xpbd_solver.hpp>
#include <SushiEngine/physics/soft/cloth.hpp>
#include <SushiEngine/physics/soft/soft_body.hpp>
#include <SushiEngine/physics/scene/physics_world.hpp>
#include <SushiEngine/loop/fixed_timestep.hpp>
#include <SushiEngine/loop/input.hpp>
#include <SushiEngine/loop/rng.hpp>
#include <SushiEngine/loop/rollback.hpp>
#include <SushiEngine/loop/net.hpp>
#include <SushiEngine/loop/app.hpp>
#include <SushiEngine/animation/animation.hpp>
#include <SushiEngine/vfx/vfx.hpp>
#include <SushiEngine/audio/audio.hpp>
#include <SushiEngine/ui/rect.hpp>
#include <SushiEngine/ui/components.hpp>
#include <SushiEngine/ui/interaction.hpp>
#include <SushiEngine/ui/layout.hpp>
#include <SushiEngine/ui/ui.hpp>
#include <SushiEngine/input/input_manager.hpp>
