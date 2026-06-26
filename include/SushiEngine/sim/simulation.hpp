/**************************************************************************/
/* simulation.hpp                                                        */
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

namespace SushiEngine
{
    /**
     * @brief Builds the per-step simulation graph for a World and replays it.
     *
     * This is the head talking to its battery: the engine expresses one fixed
     * timestep as a SushiRuntime graph and asks the runtime to run it. The graph
     * is built once in the constructor and replayed every step, so the runtime
     * compiles it exactly once. The single node integrates position forward by
     * velocity: it reads the current position field, writes the next, and the
     * runtime's double buffer makes the just-written field current for the step
     * that follows.
     */
    class Simulation
    {
        public:
            /**
             * @brief Records the integrate node against @p world's fields.
             * @param runtime The runtime whose graph this drives.
             * @param world   The component store this evolves; must outlive this.
             * @param dt      Fixed timestep applied each step.
             */
            Simulation(SushiRuntime::API::Runtime& runtime, World& world, Scalar dt)
                : graph_(runtime.graph())
            {
                const Velocity* velocity = world.velocity().data();
                const Scalar step_dt = dt;

                // One kernel over every entity: position_next = position + velocity * dt.
                // The runtime supplies current/next each step and follows the swap.
                graph_.add(world.position(),
                           [velocity, step_dt](std::size_t i,
                                               const Position* current,
                                               Position* next)
                           {
                               next[i] = current[i] + velocity[i] * step_dt;
                           });
            }

            /**
             * @brief Runs @p steps fixed timesteps, blocking until done.
             * @param steps Number of steps to advance.
             * @return The run report (timing and task counts) for the call.
             */
            SushiRuntime::RunReport step(std::size_t steps = 1)
            {
                return graph_.run(steps);
            }

        private:
            SushiRuntime::API::Graph graph_;
    };
} // namespace SushiEngine
