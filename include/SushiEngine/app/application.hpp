/**************************************************************************/
/* application.hpp                                                       */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
/* Licensed under the Apache License, Version 2.0.                         */
/**************************************************************************/

#pragma once

#include <cstddef>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/ecs/world.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    /**
     * @brief Owns the engine's pieces and keeps control of the loop.
     *
     * The Application is the top of the stack: it creates the runtime (the plugged
     * in battery), the World, and the Simulation, and it drives stepping. The
     * runtime never owns the loop; the engine does. Milestone A is headless, so
     * the loop is a finite number of fixed steps with no window or render; the
     * real-time accumulator and the render hook arrive with Milestone B.
     *
     * Member order matters: the runtime is built first and destroyed last because
     * the World's fields and the Simulation's graph borrow it.
     */
    class Application
    {
        public:
            /**
             * @brief Builds a runtime, a World of @p entities, and its Simulation.
             * @param entities Entity count for every component field.
             * @param dt       Fixed timestep handed to the simulation.
             */
            Application(std::size_t entities, Scalar dt)
                : runtime_(SushiRuntime::API::Runtime::create()),
                  world_(runtime_, entities),
                  simulation_(runtime_, world_, dt)
            {
            }

            /** @brief The component store, for seeding and reading back. */
            World& world() noexcept { return world_; }

            /** @brief The simulation driver. */
            Simulation& simulation() noexcept { return simulation_; }

            /**
             * @brief Advances the simulation by @p steps fixed timesteps.
             * @param steps Number of steps to run.
             * @return The run report for the call.
             */
            SushiRuntime::RunReport run_steps(std::size_t steps)
            {
                return simulation_.step(steps);
            }

        private:
            SushiRuntime::API::Runtime runtime_;
            World world_;
            Simulation simulation_;
    };
} // namespace SushiEngine
