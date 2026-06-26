/**************************************************************************/
/* command_buffer.hpp                                                     */
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

#include <functional>
#include <tuple>
#include <utility>
#include <vector>

#include <SushiEngine/ecs/entity.hpp>
#include <SushiEngine/ecs/world.hpp>

namespace SushiEngine
{
    /**
     * @brief Records structural changes to replay against the world at a barrier.
     *
     * Systems run as device kernels and must never see entities appear or vanish
     * mid-frame. Gameplay code therefore enqueues spawns and destroys here during
     * the frame and the loop applies them once, at an explicit sync point between
     * steps. This is the deferred-structural-change discipline an ECS needs; the
     * recorded order is preserved so a frame's effects are reproducible.
     */
    class CommandBuffer
    {
        public:
            /**
             * @brief Records the creation of an entity with the given components.
             * @tparam Ts The component types, which define the archetype.
             * @param values The initial component values, in order.
             */
            template <typename... Ts>
            void spawn(Ts... values)
            {
                ops_.push_back([values...](World& w) { w.spawn(values...); });
            }

            /**
             * @brief Records the destruction of entity @p e.
             *
             * The destroy is guarded at apply time, so enqueuing the same entity
             * twice, or an entity another command already removed, is harmless.
             *
             * @param e The entity to destroy.
             */
            void destroy(Entity e)
            {
                ops_.push_back([e](World& w) { if (w.alive(e)) w.destroy(e); });
            }

            /**
             * @brief Applies every recorded command to @p world, then clears.
             * @param world The world to mutate.
             */
            void apply(World& world)
            {
                for (auto& op : ops_) op(world);
                ops_.clear();
            }

            /** @brief True when no commands are pending. */
            bool empty() const noexcept { return ops_.empty(); }

            /** @brief Number of pending commands. */
            std::size_t size() const noexcept { return ops_.size(); }

        private:
            std::vector<std::function<void(World&)>> ops_;
    };
} // namespace SushiEngine
