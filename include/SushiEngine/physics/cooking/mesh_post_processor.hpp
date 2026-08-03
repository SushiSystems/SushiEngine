/**************************************************************************/
/* mesh_post_processor.hpp                                                */
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
 * @file mesh_post_processor.hpp
 * @brief §8.1's chain: an imported mesh, and everything the project wants made from it.
 *
 * "New; ordered, registered, Open/Closed" is what §8.1 says about this chain, and each of
 * those three words is a requirement with a mechanism:
 *
 * - **Ordered** — a processor names its own position, so the chain's sequence is a property
 *   of the processors rather than of the order somebody happened to register them in.
 * - **Registered** — the chain holds objects it was given. Adding the node-beam cooker of
 *   §11 is a registration, at which point that cooker exists.
 * - **Open/Closed** — no processor is named by the chain, and no case in the chain
 *   distinguishes them. A processor decides for itself whether a profile wants it, which is
 *   the one decision that would otherwise become a `switch` every new kind has to edit.
 *
 * **The mesh loader is a seam and not a call.** The chain takes triangles; getting them out
 * of a file is the *consumer's* job, wired in as a @ref MeshLoader. That keeps `cooking/`
 * linking nothing but `geometry/` — an importer that needs a GPU is an importer that fails on
 * a build machine (§3.4), and one that needs cgltf is one that cannot be tested against a
 * mesh built in memory.
 */

#include <cstddef>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <SushiEngine/geometry/triangle_mesh.hpp>
#include <SushiEngine/physics/cooking/cooker_interface.hpp>
#include <SushiEngine/physics/cooking/import_profile.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            /**
             * @brief Turns an asset path into triangles.
             *
             * The seam the consumer owns (§3.3). An application wires
             * `Geometry::import_gltf_mesh` in; a test wires a lambda that builds a box; a
             * build machine wires whatever its content pipeline reads. None of the three is
             * the cooker's business.
             *
             * @return False when the path could not be read as geometry.
             */
            using MeshLoader = std::function<bool(const std::string&, Geometry::TriangleMesh&)>;

            /** @brief One thing a post-processor made, and what it reported making it. */
            struct MeshPostProcessResult
            {
                /** @brief Which asset family this is. */
                CookedAssetKind kind = CookedAssetKind::Collision;

                /** @brief The producing processor's name, for a log or an inspector. */
                std::string processor;

                /** @brief What the cook measured. */
                CookingReport report;

                /**
                 * @brief The asset bytes.
                 *
                 * Carried alongside the store's copy rather than instead of it, because the
                 * caller usually wants to use what was just cooked and re-reading it through
                 * the store to get it would be a round trip for nothing.
                 */
                std::vector<std::byte> bytes;
            };

            /**
             * @brief One step of the import chain.
             *
             * Owned by the chain. Stateless where possible: a processor may be asked to run
             * for many assets, and the pipeline makes no promise about which thread.
             */
            class IMeshPostProcessor
            {
            public:
                virtual ~IMeshPostProcessor() = default;

                /** @brief A short stable name, for the result and the log. */
                virtual const char* name() const noexcept = 0;

                /**
                 * @brief Where in the chain this processor runs; lower first.
                 *
                 * A number rather than an insertion order, so a project registering the
                 * node-beam cooker does not have to know it must come after the collision one.
                 * Ties keep insertion order, which makes the chain stable without making it
                 * arbitrary.
                 */
                virtual int order() const noexcept = 0;

                /**
                 * @brief Whether @p profile asks for this processor's output.
                 *
                 * The open/closed hinge. The chain never asks *what* a processor is, only
                 * whether it wants to run — so a new kind of cooked asset adds a class and
                 * changes no existing line.
                 */
                virtual bool wants(const ImportProfile& profile) const noexcept = 0;

                /**
                 * @brief Cooks @p mesh into @p out.
                 *
                 * @param mesh     The imported geometry.
                 * @param profile  The resolved import profile.
                 * @param store    Where to look before cooking and write after; may be null.
                 * @param progress Told as each cooking stage begins; may be null.
                 * @param out      Receives the product; overwritten.
                 * @return False when the cook produced no asset at all. A cook that produced
                 *         one and failed a threshold returns true, because the asset exists and
                 *         the report says it was rejected — an artist told "this failed" needs
                 *         to be able to look at what failed.
                 */
                virtual bool process(const Geometry::TriangleMeshView& mesh,
                                     const ImportProfile& profile, ICookedAssetStore* store,
                                     ICookingProgressSink* progress,
                                     MeshPostProcessResult& out) = 0;
            };

            /**
             * @brief The ordered chain of everything that runs on import.
             *
             * Movable and not copyable: it owns its processors, and a chain that could be
             * copied would invite two pipelines quietly sharing one processor's state.
             */
            class MeshPostProcessorChain
            {
            public:
                MeshPostProcessorChain() = default;
                MeshPostProcessorChain(const MeshPostProcessorChain&) = delete;
                MeshPostProcessorChain& operator=(const MeshPostProcessorChain&) = delete;
                MeshPostProcessorChain(MeshPostProcessorChain&&) = default;
                MeshPostProcessorChain& operator=(MeshPostProcessorChain&&) = default;

                /**
                 * @brief Registers @p processor, keeping the chain sorted by order.
                 *
                 * @param processor The processor to own; a null pointer is ignored.
                 */
                void add(std::unique_ptr<IMeshPostProcessor> processor);

                /** @brief How many processors are registered. */
                std::size_t size() const noexcept { return processors_.size(); }

                /**
                 * @brief Processor @p index, in chain order.
                 *
                 * @param index Which one; must be below @ref size.
                 */
                const IMeshPostProcessor& at(std::size_t index) const
                {
                    return *processors_[index];
                }

                /**
                 * @brief Runs every processor @p profile wants.
                 *
                 * A processor that produced nothing contributes no result rather than an empty
                 * one, and a processor that failed does not stop the chain: a mesh whose
                 * tetrahedralization failed should still get its collider, or one bad soft-body
                 * cook silently costs the asset its collision too.
                 *
                 * @param mesh     The imported geometry.
                 * @param profile  The resolved import profile.
                 * @param store    The cache; may be null.
                 * @param progress Told as each cooking stage begins; may be null.
                 * @return One result per processor that produced an asset, in chain order.
                 */
                std::vector<MeshPostProcessResult> run(const Geometry::TriangleMeshView& mesh,
                                                      const ImportProfile& profile,
                                                      ICookedAssetStore* store,
                                                      ICookingProgressSink* progress) const;

                /**
                 * @brief A chain with the cookers this build ships.
                 *
                 * The collision, soft-body, and node-beam processors, in that order.
                 */
                static MeshPostProcessorChain with_shipped_processors();

            private:
                std::vector<std::unique_ptr<IMeshPostProcessor>> processors_;
            };

            /** @brief Chain positions, so the shipped processors' order is stated in one place. */
            enum MeshPostProcessorOrder : int
            {
                /** @brief Cheapest and wanted by almost everything, so it runs first. */
                POST_PROCESS_ORDER_COLLISION = 100,

                /** @brief Minutes rather than milliseconds, and wanted by few assets. */
                POST_PROCESS_ORDER_SOFT_BODY = 200,

                /** @brief §11's node-beam cooker; rarer than either of the above. */
                POST_PROCESS_ORDER_NODE_BEAM = 300,
            };
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
