/**************************************************************************/
/* world.hpp                                                              */
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

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/ecs/archetype.hpp>
#include <SushiEngine/ecs/component.hpp>
#include <SushiEngine/ecs/entity.hpp>

namespace SushiEngine
{
    /**
     * @brief The container of all entities, their components, and their archetypes.
     *
     * The World owns the archetypes (and through them the chunks and component
     * columns) and a directory that maps each entity handle to where its row lives.
     * It backs storage with a SushiRuntime instance, which must outlive it.
     *
     * Structural changes (spawn, destroy, and the chunk allocations they trigger)
     * happen on the host. A `structure_version` counter ticks only when the set of
     * chunks changes — never on a plain count change — so a schedule can tell the
     * rare "rebuild the graph" event apart from the common "more or fewer entities
     * this frame" one.
     */
    class World
    {
        public:
            /**
             * @brief Creates an empty world backed by @p runtime.
             * @param runtime        The runtime owning all component storage.
             * @param chunk_capacity Entities per chunk for every archetype.
             */
            explicit World(SushiRuntime::API::Runtime& runtime,
                           std::size_t chunk_capacity = 1024)
                : runtime_(runtime), chunk_capacity_(chunk_capacity)
            {
            }

            /** @brief Entities per chunk, the late-bound capacity systems iterate. */
            std::size_t chunk_capacity() const noexcept { return chunk_capacity_; }

            /** @brief Ticks whenever the chunk set changes; stable otherwise. */
            std::uint64_t structure_version() const noexcept { return structure_version_; }

            /**
             * @brief Finds or creates the archetype for component types @p Ts.
             * @tparam Ts The exact component set of the archetype.
             * @return A reference to the (possibly newly created) archetype.
             */
            template <typename... Ts>
            Archetype& archetype()
            {
                Signature sig = make_signature<Ts...>();
                for (std::unique_ptr<Archetype>& a : archetypes_)
                    if (a->signature() == sig) return *a;

                archetypes_.push_back(std::make_unique<Archetype>(
                    runtime_, sig, make_component_infos<Ts...>(), chunk_capacity_));
                ++structure_version_;
                return *archetypes_.back();
            }

            /**
             * @brief Pre-allocates chunks for @p entities of archetype @p Ts.
             * @tparam Ts The archetype's component set.
             * @param entities The number of entities to make room for.
             */
            template <typename... Ts>
            void reserve(std::size_t entities)
            {
                archetype<Ts...>().reserve(entities);
                ++structure_version_;
            }

            /**
             * @brief Creates an entity with the given component values.
             * @tparam Ts The component types, which define the archetype.
             * @param values One initial value per component, in order.
             * @return A handle to the new entity.
             */
            template <typename... Ts>
            Entity spawn(Ts... values)
            {
                Archetype& a = archetype<Ts...>();
                bool allocated = false;
                Chunk& c = a.chunk_with_space(allocated);
                if (allocated) ++structure_version_;

                const Entity e = acquire_entity();
                const std::size_t row = c.allocate_row(e);
                (write_component(c, row, values), ...);
                bind_record(e, a, c, row);
                return e;
            }

            /**
             * @brief Destroys @p e, freeing its slot and packing its chunk.
             *
             * Swap-removes the entity's row, repoints the directory record of the
             * row that moved into its place, and bumps the slot's generation so any
             * surviving handle to @p e becomes stale. A no-op resource-wise: the
             * chunk set does not change, so this never forces a graph rebuild.
             *
             * @param e The entity to destroy; must be alive.
             */
            void destroy(Entity e)
            {
                assert(alive(e) && "destroy() on a dead or stale entity");
                EntityRecord& rec = entities_[e.index];
                const Entity moved = rec.chunk->remove_row(rec.row);
                if (!moved.is_null())
                    entities_[moved.index].row = rec.row;

                rec.alive = false;
                ++rec.generation;
                free_.push_back(e.index);
            }

            /**
             * @brief True if @p e still names a live entity.
             * @param e The handle to test.
             * @return True when the slot is live and the generation matches.
             */
            bool alive(Entity e) const noexcept
            {
                return e.index < entities_.size() &&
                       entities_[e.index].alive &&
                       entities_[e.index].generation == e.generation;
            }

            /**
             * @brief Host access to entity @p e's component of type @p T.
             * @tparam T The component type to read or modify.
             * @param e An alive entity that has component @p T.
             * @return A reference into the shared-USM column.
             */
            template <typename T>
            T& get(Entity e) noexcept
            {
                assert(alive(e) && "get() on a dead or stale entity");
                const EntityRecord& rec = entities_[e.index];
                std::byte* base = rec.chunk->column(component_id<T>());
                return *reinterpret_cast<T*>(base + rec.row * sizeof(T));
            }

            /**
             * @brief All archetypes whose components include every id in @p required.
             * @param required A sorted set of component ids a system needs.
             * @return Pointers to the matching archetypes.
             */
            std::vector<Archetype*> query(const Signature& required)
            {
                std::vector<Archetype*> out;
                for (std::unique_ptr<Archetype>& a : archetypes_)
                    if (signature_contains(a->signature(), required))
                        out.push_back(a.get());
                return out;
            }

        private:
            /** @brief Where one entity's row lives, plus its liveness generation. */
            struct EntityRecord
            {
                Archetype* archetype = nullptr;
                Chunk* chunk = nullptr;
                std::uint32_t row = 0;
                std::uint32_t generation = 0;
                bool alive = false;
            };

            /** @brief Writes one component value into a freshly allocated row. */
            template <typename T>
            void write_component(Chunk& c, std::size_t row, const T& value) noexcept
            {
                std::byte* base = c.column(component_id<T>());
                *reinterpret_cast<T*>(base + row * sizeof(T)) = value;
            }

            /** @brief Allocates an entity slot, reusing a freed one when available. */
            Entity acquire_entity()
            {
                std::uint32_t index;
                if (!free_.empty())
                {
                    index = free_.back();
                    free_.pop_back();
                }
                else
                {
                    index = static_cast<std::uint32_t>(entities_.size());
                    entities_.push_back(EntityRecord{});
                }
                return Entity{index, entities_[index].generation};
            }

            /** @brief Points an entity's directory record at its storage row. */
            void bind_record(Entity e, Archetype& a, Chunk& c, std::size_t row) noexcept
            {
                EntityRecord& rec = entities_[e.index];
                rec.archetype = &a;
                rec.chunk = &c;
                rec.row = static_cast<std::uint32_t>(row);
                rec.generation = e.generation;
                rec.alive = true;
            }

            SushiRuntime::API::Runtime& runtime_;
            std::size_t chunk_capacity_;
            std::uint64_t structure_version_ = 0;
            std::vector<std::unique_ptr<Archetype>> archetypes_;
            std::vector<EntityRecord> entities_;
            std::vector<std::uint32_t> free_;
    };
} // namespace SushiEngine
