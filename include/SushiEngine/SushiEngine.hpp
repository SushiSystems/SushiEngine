/**************************************************************************/
/* SushiEngine.hpp                                                       */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/* Licensed under the Apache License, Version 2.0.                         */
/**************************************************************************/

#pragma once

/**
 * @file SushiEngine.hpp
 * @brief Umbrella header for the engine head. One include pulls in the
 *        Milestone A surface: value types, the World store, the Simulation
 *        driver, and the Application that owns the loop.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/ecs/components.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/sim/simulation.hpp>
#include <SushiEngine/app/application.hpp>
