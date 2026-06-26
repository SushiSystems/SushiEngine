/**************************************************************************/
/* archetype.hpp                                                          */
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

#include <cstddef>
#include <memory>
#include <vector>

#include <SushiRuntime/SushiRuntime.h>

#include <SushiEngine/ecs/chunk.hpp>
#include <SushiEngine/ecs/component.hpp>

namespace SushiEngine
{
    /**
     * @brief All entities sharing one exact set of component types.
     *
     * An archetype owns the chunks that store its entities. New entities land in a
     * chunk that has room; chunks are allocated on demand and never freed during a
     * run, so a chunk's column pointers stay stable for the lifetime of any graph
     * that captured them. Allocating a chunk changes the set of resources the
     * schedule must cover, which is the only structural event that forces a graph
     * rebuild — entity counts within existing chunks vary freely without one.
     */
    class Archetype
    {
        public:
            /**
             * @brief Creates an empty archetype for the given component set.
             * @param runtime        The runtime that backs its chunks.
             * @param signature      The sorted component ids, its identity.
             * @param comps          The components' ids and sizes, for chunk layout.
             * @param chunk_capacity Entities per chunk.
             */
            Archetype(SushiRuntime::API::Runtime& runtime, Signature signature,
                      std::vector<ComponentInfo> comps, std::size_t chunk_capacity)
                : runtime_(runtime),
                  signature_(std::move(signature)),
                  comps_(std::move(comps)),
                  chunk_capacity_(chunk_capacity)
            {
            }

            /** @brief The archetype's identity: its sorted component ids. */
            const Signature& signature() const noexcept { return signature_; }

            /** @brief The chunks holding this archetype's entities. */
            const std::vector<std::unique_ptr<Chunk>>& chunks() const noexcept
            {
                return chunks_;
            }

            /**
             * @brief Returns a chunk with a free row, allocating one if needed.
             * @param allocated Set to true when a new chunk had to be created.
             * @return A chunk that is not full.
             */
            Chunk& chunk_with_space(bool& allocated)
            {
                allocated = false;
                for (std::unique_ptr<Chunk>& c : chunks_)
                    if (!c->full()) return *c;

                chunks_.push_back(std::make_unique<Chunk>(runtime_, comps_, chunk_capacity_));
                allocated = true;
                return *chunks_.back();
            }

            /**
             * @brief Pre-allocates enough chunks to hold @p entities.
             *
             * Calling this before the first schedule run keeps later spawns from
             * allocating chunks mid-run, so the graph is compiled once and replayed.
             *
             * @param entities The number of entities to make room for.
             */
            void reserve(std::size_t entities)
            {
                while (chunks_.size() * chunk_capacity_ < entities)
                    chunks_.push_back(std::make_unique<Chunk>(runtime_, comps_, chunk_capacity_));
            }

        private:
            SushiRuntime::API::Runtime& runtime_;
            Signature signature_;
            std::vector<ComponentInfo> comps_;
            std::size_t chunk_capacity_;
            std::vector<std::unique_ptr<Chunk>> chunks_;
    };
} // namespace SushiEngine
