/**************************************************************************/
/* mesh_post_processor.cpp                                                */
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

#include <SushiEngine/physics/cooking/mesh_post_processor.hpp>

#include <algorithm>
#include <utility>

#include <SushiEngine/physics/cooking/collision_cooker.hpp>
#include <SushiEngine/physics/cooking/node_beam_cooker.hpp>
#include <SushiEngine/physics/cooking/soft_body_cooker.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        namespace Cooking
        {
            namespace
            {
                /**
                 * @brief The shipped processors, which are both one shape.
                 *
                 * A template over the cooker rather than two near-identical classes: the only
                 * things that differ between a collision processor and a soft-body one are
                 * which cooker they hold, which profile flag they read, and what they are
                 * called. Writing that twice is writing the same bug twice.
                 */
                template <typename Cooker>
                class CookerPostProcessor final : public IMeshPostProcessor
                {
                public:
                    CookerPostProcessor(const char* name, int order,
                                        bool CookingParameters::*flag)
                        : name_(name), order_(order), flag_(flag)
                    {
                    }

                    const char* name() const noexcept override { return name_; }
                    int order() const noexcept override { return order_; }

                    bool wants(const ImportProfile& profile) const noexcept override
                    {
                        return profile.parameters.*flag_;
                    }

                    bool process(const Geometry::TriangleMeshView& mesh,
                                 const ImportProfile& profile, ICookedAssetStore* store,
                                 ICookingProgressSink* progress,
                                 MeshPostProcessResult& out) override
                    {
                        cooker_.set_thresholds(profile.thresholds);
                        // Evicted here rather than by the caller, because the key needs the
                        // mesh's content hash and this is the first point at which both the
                        // mesh and the cooker that owns the key are in the same place.
                        if (profile.force_recook && store != nullptr)
                            store->evict(cooker_.cache_key(mesh, profile.parameters));
                        out.kind = cooker_.kind();
                        out.processor = name_;
                        out.bytes.clear();
                        out.report = cooker_.cook(mesh, profile.parameters, store, progress,
                                                  out.bytes);
                        return out.report.has_asset();
                    }

                private:
                    const char* name_;
                    int order_;
                    bool CookingParameters::*flag_;
                    Cooker cooker_;
                };

                /**
                 * @brief The node-beam processor, which is not the shape above.
                 *
                 * @ref NodeBeamCooker takes a second piece of state the other two cookers
                 * do not — @ref NodeBeamCookerSettings, read from the profile's own field
                 * rather than from @ref CookingParameters (§8.1; @ref ImportProfile's
                 * doc comment on `node_beam_settings` says why). That extra
                 * `set_settings` call is the one line the template above cannot express,
                 * so this is a class of its own rather than a second template parameter
                 * bent to fit a shape it was not written for.
                 */
                class NodeBeamPostProcessor final : public IMeshPostProcessor
                {
                public:
                    const char* name() const noexcept override { return "NodeBeamPostProcessor"; }
                    int order() const noexcept override { return POST_PROCESS_ORDER_NODE_BEAM; }

                    bool wants(const ImportProfile& profile) const noexcept override
                    {
                        return profile.parameters.cook_node_beam;
                    }

                    bool process(const Geometry::TriangleMeshView& mesh,
                                 const ImportProfile& profile, ICookedAssetStore* store,
                                 ICookingProgressSink* progress,
                                 MeshPostProcessResult& out) override
                    {
                        cooker_.set_settings(profile.node_beam_settings);
                        cooker_.set_thresholds(profile.thresholds);
                        // Evicted here rather than by the caller, for the same reason the
                        // template above does it here: the key needs the mesh's content hash
                        // and this is the first point at which both it and the cooker that
                        // owns the key are in the same place.
                        if (profile.force_recook && store != nullptr)
                            store->evict(cooker_.cache_key(mesh, profile.parameters));
                        out.kind = cooker_.kind();
                        out.processor = name();
                        out.bytes.clear();
                        out.report = cooker_.cook(mesh, profile.parameters, store, progress,
                                                  out.bytes);
                        return out.report.has_asset();
                    }

                private:
                    NodeBeamCooker cooker_;
                };
            } // namespace

            void MeshPostProcessorChain::add(std::unique_ptr<IMeshPostProcessor> processor)
            {
                if (processor == nullptr)
                    return;

                // Inserted at its position rather than appended and sorted, so a tie keeps
                // insertion order — `std::sort` is not stable and a chain whose sequence
                // changed between builds would make a cook log unreadable.
                const int order = processor->order();
                auto position = processors_.begin();
                while (position != processors_.end() && (*position)->order() <= order)
                    ++position;
                processors_.insert(position, std::move(processor));
            }

            std::vector<MeshPostProcessResult> MeshPostProcessorChain::run(
                const Geometry::TriangleMeshView& mesh, const ImportProfile& profile,
                ICookedAssetStore* store, ICookingProgressSink* progress) const
            {
                std::vector<MeshPostProcessResult> results;
                for (const std::unique_ptr<IMeshPostProcessor>& processor : processors_)
                {
                    if (!processor->wants(profile))
                        continue;

                    MeshPostProcessResult result;
                    // A processor that failed does not stop the chain. A mesh whose
                    // tetrahedralization could not be filled should still get its collider —
                    // otherwise one bad soft-body cook silently costs the asset its collision
                    // as well, and the artist sees a crate that has stopped being solid.
                    if (processor->process(mesh, profile, store, progress, result))
                        results.push_back(std::move(result));
                }
                return results;
            }

            MeshPostProcessorChain MeshPostProcessorChain::with_shipped_processors()
            {
                MeshPostProcessorChain chain;
                chain.add(std::make_unique<CookerPostProcessor<CollisionCooker>>(
                    "CollisionPostProcessor", POST_PROCESS_ORDER_COLLISION,
                    &CookingParameters::cook_collision));
                chain.add(std::make_unique<CookerPostProcessor<SoftBodyCooker>>(
                    "SoftBodyPostProcessor", POST_PROCESS_ORDER_SOFT_BODY,
                    &CookingParameters::cook_soft_body));
                chain.add(std::make_unique<NodeBeamPostProcessor>());
                return chain;
            }
        } // namespace Cooking
    } // namespace Physics
} // namespace SushiEngine
