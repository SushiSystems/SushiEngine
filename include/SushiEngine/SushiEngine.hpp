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
 * @brief Umbrella header for the engine. One include pulls in the value-type seam
 *        and the full ECS surface: entities, components, archetype storage, the
 *        world, the deferred command buffer, and the system schedule.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/ecs/entity.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/chunk.hpp>
#include <SushiEngine/ecs/archetype.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/ecs/command_buffer.hpp>
#include <SushiEngine/ecs/schedule.hpp>
