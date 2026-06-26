/**************************************************************************/
/* world.hpp                                                              */
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

#include <SushiEngine/ecs/components.hpp>

namespace SushiEngine
{
    /**
     * @brief Structure-of-arrays component store for a fixed entity count.
     *
     * The World holds each component as one contiguous field backed by runtime
     * memory: position is a double-buffered State (a step reads current and writes
     * next), velocity is a plain Buffer. The fields are shared USM, so the host
     * may seed and read them directly while a device kernel drives the arrays. The
     * runtime allocates and owns the memory; the World only borrows the Runtime,
     * which must outlive it.
     */
    class World
    {
        public:
            /**
             * @brief Allocates @p count entities' worth of each component field.
             * @param runtime The runtime that owns the allocations; must outlive this.
             * @param count   Number of entities (the length of every field).
             */
            World(SushiRuntime::API::Runtime& runtime, std::size_t count)
                : count_(count),
                  position_(runtime.state<Position>(count)),
                  velocity_(runtime.buffer<Velocity>(count))
            {
            }

            /** @brief Number of entities held. */
            std::size_t size() const noexcept { return count_; }

            /** @brief The double-buffered position field. */
            SushiRuntime::API::State<Position>& position() noexcept { return position_; }

            /** @brief The velocity field. */
            SushiRuntime::API::Buffer<Velocity>& velocity() noexcept { return velocity_; }

        private:
            std::size_t count_ = 0;
            SushiRuntime::API::State<Position> position_;
            SushiRuntime::API::Buffer<Velocity> velocity_;
    };
} // namespace SushiEngine
