/**************************************************************************/
/* bvh_broadphase.hpp                                                     */
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
 * @file bvh_broadphase.hpp
 * @brief The production broadphase: two hierarchies, and a pair set that persists.
 *
 * §7.1's implementation. What it adds over the sweep is not a better constant
 * factor but a different shape of work: the sweep re-sorts the entire scene every
 * update to rediscover a pair set that barely changed, while a hierarchy is a
 * structure that was already correct and is *repaired* where a body moved.
 *
 * **Two trees, because "static" is not a special case of slow.** Static geometry —
 * the ground, a building, a cooked mesh's proxy — is never inserted twice and its
 * tree is never restructured after the level loads. Keeping it apart is not an
 * optimization of the static bodies; it is an optimization of the dynamic ones,
 * whose tree stays small enough that a re-insertion is a dozen node visits rather
 * than a descent past a hundred thousand pieces of scenery. Static bodies also
 * never pair with each other, and two trees make that true by construction rather
 * than by a rejection that still costs a test.
 *
 * **The pair cache is the deliverable, not the tree.** §7.3's warm starting only
 * pays if a manifold survives from tick to tick, and a manifold can only survive
 * if something remembers that its pair is the *same* pair as last tick. That is
 * what `added` / `persisted` / `removed` are for, and the diff producing them
 * lives one layer down in `BroadphaseBase` so both implementations produce it
 * identically.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/broadphase.hpp>
#include <SushiEngine/physics/collision/dynamic_bvh.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A bounding-volume-hierarchy broadphase over a dynamic and a static tree.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class BvhBroadphase final : public BroadphaseBase<T>
        {
            public:
                void update() override
                {
                    found_.clear();

                    // Only *awake* proxies descend. Every pair needs at least one
                    // member that is neither static nor sleeping — a pair of quiet
                    // proxies has nothing to resolve and is rejected anyway — so a
                    // descent that starts at a quiet leaf can only find pairs some
                    // awake leaf will find as well. Ten thousand settled crates
                    // therefore cost ten thousand containment tests and no tree
                    // traversal at all, which is what §13.2's "a settled island
                    // costs its bound update and nothing else" has to mean if it
                    // means anything.
                    //
                    // The price is that an awake-versus-awake pair is found from
                    // both ends; the sort at the bottom already had to happen, so
                    // removing the duplicate costs one comparison per pair.
                    constexpr std::uint32_t quiet = BodyFlags::static_body | BodyFlags::sleeping;
                    const bool any_static = static_tree_.leaf_count() > 0;
                    for (ProxyId id = 0; id < this->proxies_.size(); ++id)
                    {
                        const BroadphaseProxy<T>& record = this->proxies_[id];
                        if (!record.alive || has_any_flag(record.flags, quiet))
                            continue;
                        dynamic_tree_.query(record.bounds,
                                            [&](std::uint32_t other, std::uint32_t) noexcept
                                            {
                                                if (other != id)
                                                    consider(id, other);
                                            });
                        // The static tree is only ever *asked*, which is why it
                        // never needs to be rebuilt.
                        if (any_static)
                            static_tree_.query(record.bounds,
                                               [&](std::uint32_t other, std::uint32_t) noexcept
                                               { consider(id, other); });
                    }

                    std::sort(found_.begin(), found_.end());
                    found_.erase(std::unique(found_.begin(), found_.end()), found_.end());
                    this->publish_pairs(found_);
                }

                void set_proxy_state(ProxyId id, const CollisionFilter& filter,
                                     std::uint32_t flags) override
                {
                    if (id >= this->proxies_.size() || !this->proxies_[id].alive)
                        return;
                    // A body that becomes static — or stops being static — changes
                    // which tree it belongs in. Re-homing it here rather than at the
                    // next move is what keeps "the static tree is never restructured"
                    // a true statement about a running scene and not just about load.
                    const bool was_static = in_static_tree(id);
                    const bool now_static = has_any_flag(flags, BodyFlags::static_body);
                    BroadphaseBase<T>::set_proxy_state(id, filter, flags);
                    if (was_static == now_static)
                        return;
                    tree_for(was_static).remove(nodes_[id]);
                    nodes_[id] = tree_for(now_static).insert(this->proxies_[id].bounds, id);
                    static_membership_[id] = now_static;
                }

                void query_overlap(const Aabb<T>& box,
                                   const std::function<void(ProxyId)>& visit) const override
                {
                    const auto report = [&](std::uint32_t id, std::uint32_t) { visit(id); };
                    dynamic_tree_.query(box, report);
                    static_tree_.query(box, report);
                }

                void query_ray(const Vector3T<T>& origin, const Vector3T<T>& direction,
                               T max_distance,
                               const std::function<void(ProxyId)>& visit) const override
                {
                    const auto report = [&](std::uint32_t id, std::uint32_t) { visit(id); };
                    dynamic_tree_.query_ray(origin, direction, max_distance, report);
                    static_tree_.query_ray(origin, direction, max_distance, report);
                }

                /** @brief The dynamic tree, for tests that check its invariants. */
                const DynamicBvh<T>& dynamic_tree() const noexcept { return dynamic_tree_; }

                /** @brief The static tree, for the same reason. */
                const DynamicBvh<T>& static_tree() const noexcept { return static_tree_; }

            protected:
                void on_proxy_created(ProxyId id) override
                {
                    if (nodes_.size() <= id)
                    {
                        nodes_.resize(id + 1u, null_bvh_node);
                        static_membership_.resize(id + 1u, false);
                    }
                    const bool is_static =
                        has_any_flag(this->proxies_[id].flags, BodyFlags::static_body);
                    static_membership_[id] = is_static;
                    nodes_[id] = tree_for(is_static).insert(this->proxies_[id].bounds, id);
                }

                void on_proxy_destroyed(ProxyId id) override
                {
                    if (nodes_[id] == null_bvh_node)
                        return;
                    tree_for(static_membership_[id]).remove(nodes_[id]);
                    nodes_[id] = null_bvh_node;
                }

                void on_proxy_moved(ProxyId id) override
                {
                    // Unconditionally: the base class has already decided the stored
                    // box changed, and the tree holding a different box from the one
                    // the proxy records is exactly the divergence that would make the
                    // two implementations disagree about the pair set.
                    tree_for(static_membership_[id]).replace(nodes_[id], this->proxies_[id].bounds);
                }

            private:
                DynamicBvh<T>& tree_for(bool is_static) noexcept
                {
                    return is_static ? static_tree_ : dynamic_tree_;
                }

                bool in_static_tree(ProxyId id) const noexcept
                {
                    return id < static_membership_.size() && static_membership_[id] != 0;
                }

                /** @brief Admits a pair if the filters and flags allow it. */
                void consider(ProxyId a, ProxyId b)
                {
                    if (!proxies_may_collide(this->proxies_[a], this->proxies_[b]))
                        return;
                    found_.push_back(BroadphasePair::make(a, b));
                }

                DynamicBvh<T> dynamic_tree_;
                DynamicBvh<T> static_tree_;
                std::vector<std::uint32_t> nodes_;
                // `char` rather than `bool`: a vector of bools is a bitfield whose
                // elements have no address, and this one is indexed in hot loops.
                std::vector<char> static_membership_;
                std::vector<BroadphasePair> found_;
        };
    } // namespace Physics
} // namespace SushiEngine
