/**************************************************************************/
/* broadphase.hpp                                                        */
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
 * @file broadphase.hpp
 * @brief The broadphase seam, and the sweep-and-prune implementation behind it.
 *
 * The contact solver's narrowphase is exact but costs one test per pair; running it on
 * every pair is quadratic and wasteful once a scene has more than a handful of bodies
 * (a cloth grid alone is hundreds of particles). `sweep_and_prune` sorts the bodies'
 * axis-aligned bounding boxes along one axis and sweeps a moving front, emitting only the
 * pairs whose boxes actually overlap — so the narrowphase runs on candidates, not on the
 * full cross product. It is pure geometry: no runtime, ECS, or solver dependency.
 *
 * Above that function sits @ref IBroadphase, the seam §3.3 names and §4.4 tests. Two
 * implementations live behind it — this file's sweep-and-prune, kept as the reference and
 * the small-scene fast path, and `bvh_broadphase.hpp`'s hierarchy, which is the production
 * path — and a shared conformance suite requires them to emit the *same sorted pair set*
 * for the same input.
 *
 * That requirement is what fixes where each piece of behaviour lives, and it is worth
 * stating because it is easy to get backwards. Everything that decides **which boxes are
 * tested** is policy and lives in @ref BroadphaseBase: the enlargement of a proxy's box,
 * the hysteresis that leaves it alone until the body escapes it, the sweep for a body
 * flagged continuous, and the filter and flag rules that reject a pair before geometry is
 * consulted. Only **how the overlaps are found** belongs to an implementation. Had the
 * enlargement been left to the implementations, the hierarchy's remembered fat box and the
 * sweep's freshly computed one would disagree by a tick's worth of motion, the two would
 * legitimately produce different pair sets, and the conformance suite would be asserting
 * something neither implementation should have to promise.
 */

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <utility>
#include <vector>

#include <SushiEngine/physics/core/body_flags.hpp>

#include <SushiEngine/core/types.hpp>
// `AABB` and `aabb_overlap` live in `physics/geometry` with the rest of the
// single-shape value types (§3.2): a bounding box is meaningful without a second
// box to test it against, and the mesh hierarchy one layer down needs them too.
// They are used unqualified here, exactly as when they were declared in this file.
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Emits every overlapping AABB pair via sweep-and-prune on the X axis.
         *
         * Sorts the boxes by their minimum X, then sweeps: a moving "active" set holds the
         * boxes still reachable along X, and each new box is tested against only those (a
         * full 3-axis overlap test), so non-overlapping bodies far apart in X are never
         * paired. Output pairs are `(i, j)` indices into @p boxes with `i < j`, cleared
         * first. Worst case is still quadratic for a fully overlapping cluster, but typical
         * spatially-spread scenes are near-linear.
         *
         * @tparam T The scalar element type.
         * @param boxes      One AABB per body.
         * @param out_pairs  Receives the candidate index pairs (cleared on entry).
         */
        template <typename T>
        inline void sweep_and_prune(const std::vector<AABB<T>>& boxes,
                                    std::vector<std::pair<std::uint32_t, std::uint32_t>>& out_pairs)
        {
            out_pairs.clear();
            const std::size_t count = boxes.size();
            if (count < 2)
                return;

            std::vector<std::uint32_t> order(count);
            for (std::size_t i = 0; i < count; ++i)
                order[i] = static_cast<std::uint32_t>(i);
            std::sort(order.begin(), order.end(),
                      [&](std::uint32_t l, std::uint32_t r) { return boxes[l].min.x < boxes[r].min.x; });

            std::vector<std::uint32_t> active;
            active.reserve(count);
            for (std::size_t s = 0; s < count; ++s)
            {
                const std::uint32_t i = order[s];
                const T min_x = boxes[i].min.x;
                // Drop everything whose X extent ended before this box begins.
                active.erase(std::remove_if(active.begin(), active.end(),
                                            [&](std::uint32_t a) { return boxes[a].max.x < min_x; }),
                             active.end());
                for (const std::uint32_t j : active)
                    if (aabb_overlap(boxes[i], boxes[j]))
                        out_pairs.emplace_back(i < j ? i : j, i < j ? j : i);
                active.push_back(i);
            }
        }

        /** @brief A proxy's identifier, stable for as long as the proxy lives. */
        using ProxyId = std::uint32_t;

        /** @brief The "no proxy" identifier. */
        constexpr ProxyId null_proxy = 0xFFFFFFFFu;

        /**
         * @brief What the broadphase knows about one collidable thing.
         *
         * Two boxes, because they answer different questions. @ref tight_bounds is
         * where the shape is now, and is what a fresh move compares against;
         * @ref bounds is the enlarged box the structure was last told about, and is
         * what the pair search actually uses. The gap between them is the
         * hysteresis that makes most bodies free on most ticks.
         *
         * The filter and the flag word are here rather than fetched from the body
         * because rejecting a pair is the broadphase's cheapest possible work, and
         * making it chase a pointer into the solver's state to do it would undo the
         * saving — and would point a collision module at a body layout it has no
         * business naming.
         */
        template <typename T>
        struct BroadphaseProxy
        {
            AABB<T> bounds{};       /**< The enlarged box the structure holds. */
            AABB<T> tight_bounds{}; /**< Where the shape actually is. */
            CollisionFilter filter{};
            std::uint32_t flags = 0;   /**< `BodyFlags` bits. */
            std::uint32_t payload = 0; /**< The caller's identifier for the body. */
            bool alive = false;
        };

        /**
         * @brief One candidate pair, by proxy, with the lower identifier first.
         *
         * Ordering the members at construction is what lets the pair streams be
         * compared as sets: `(3, 7)` and `(7, 3)` are the same pair, and a cache
         * that admitted both would report a removal and an addition every time the
         * search's internal order happened to change.
         */
        struct BroadphasePair
        {
            ProxyId a = null_proxy;
            ProxyId b = null_proxy;

            /** @brief The pair, ordered. */
            static BroadphasePair make(ProxyId first, ProxyId second) noexcept
            {
                return first < second ? BroadphasePair{first, second}
                                      : BroadphasePair{second, first};
            }
        };

        /** @brief Pair equality: the same two proxies. */
        inline bool operator==(const BroadphasePair& l, const BroadphasePair& r) noexcept
        {
            return l.a == r.a && l.b == r.b;
        }

        /** @brief Pair inequality. */
        inline bool operator!=(const BroadphasePair& l, const BroadphasePair& r) noexcept
        {
            return !(l == r);
        }

        /** @brief Lexicographic pair order, so a pair list has one canonical form. */
        inline bool operator<(const BroadphasePair& l, const BroadphasePair& r) noexcept
        {
            return l.a != r.a ? l.a < r.a : l.b < r.b;
        }

        /**
         * @brief Whether two proxies may produce a contact at all.
         *
         * Three rejections, in the order that costs least. Two quiet bodies never
         * pair — a pair of sleeping or static proxies has nothing to resolve, and
         * this single line is what makes a scene of ten thousand settled crates cost
         * nothing (§13.2). Then the layer masks (§5.5), which are symmetric by
         * construction so the answer cannot depend on the argument order. A trigger
         * is *not* rejected: it wants the pair, it just refuses the impulse (§7.7).
         *
         * @param a The first proxy.
         * @param b The second proxy.
         * @return True when the narrowphase should look at this pair.
         */
        template <typename T>
        inline bool proxies_may_collide(const BroadphaseProxy<T>& a,
                                        const BroadphaseProxy<T>& b) noexcept
        {
            constexpr std::uint32_t quiet = BodyFlags::static_body | BodyFlags::sleeping;
            if (has_any_flag(a.flags, quiet) && has_any_flag(b.flags, quiet))
                return false;
            return filters_collide(a.filter, b.filter);
        }

        /**
         * @brief The broadphase seam: proxies in, pair streams out.
         *
         * The three streams are the reason this interface exists in the shape it
         * does. A narrowphase handed only "the pairs" has to rediscover every
         * manifold every tick, and a rediscovered manifold has no warm-start
         * impulse — which is precisely the accumulator that makes a stack converge
         * (§7.3). `added` says "build a manifold", `persisted` says "keep the one
         * you have", `removed` says "drop it, and its impulses with it".
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class IBroadphase
        {
            public:
                virtual ~IBroadphase() = default;

                /** @brief Adds a proxy and returns its identifier. */
                virtual ProxyId create_proxy(const AABB<T>& bounds, const CollisionFilter& filter,
                                             std::uint32_t flags, std::uint32_t payload) = 0;

                /** @brief Removes a proxy; its identifier may be reused afterwards. */
                virtual void destroy_proxy(ProxyId proxy) = 0;

                /**
                 * @brief Tells the broadphase where a proxy is now.
                 *
                 * @param proxy        The proxy to move.
                 * @param bounds       Its current, tight bounds.
                 * @param displacement How far its body is expected to travel before
                 *                     the next update; the enlargement leans this way,
                 *                     and a continuous-collision body sweeps it.
                 */
                virtual void update_proxy(ProxyId proxy, const AABB<T>& bounds,
                                          const Vector3T<T>& displacement) = 0;

                /** @brief Replaces a proxy's filter and flag word. */
                virtual void set_proxy_state(ProxyId proxy, const CollisionFilter& filter,
                                             std::uint32_t flags) = 0;

                /** @brief Reads a proxy back. */
                virtual const BroadphaseProxy<T>& proxy(ProxyId proxy) const = 0;

                /** @brief Recomputes this tick's pair set and the three streams. */
                virtual void update() = 0;

                /** @brief Every pair overlapping now, sorted. */
                virtual const std::vector<BroadphasePair>& pairs() const noexcept = 0;

                /** @brief Pairs that did not exist at the previous update. */
                virtual const std::vector<BroadphasePair>& added_pairs() const noexcept = 0;

                /** @brief Pairs present at both updates — the manifolds worth keeping. */
                virtual const std::vector<BroadphasePair>& persisted_pairs() const noexcept = 0;

                /** @brief Pairs that existed at the previous update and no longer do. */
                virtual const std::vector<BroadphasePair>& removed_pairs() const noexcept = 0;

                /** @brief Calls @p visit for every proxy whose stored box meets @p box. */
                virtual void query_overlap(const AABB<T>& box,
                                           const std::function<void(ProxyId)>& visit) const = 0;

                /** @brief Calls @p visit for every proxy whose stored box the ray meets. */
                virtual void query_ray(const Vector3T<T>& origin, const Vector3T<T>& direction,
                                       T max_distance,
                                       const std::function<void(ProxyId)>& visit) const = 0;
        };

        /**
         * @brief Everything both implementations must agree about.
         *
         * The proxy pool, the enlargement policy, and the pair diff. See this
         * file's opening comment for why these live here and not in the
         * implementations: they decide *which boxes are tested*, and two
         * implementations that decided that separately could not honestly be held
         * to the same pair set.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class BroadphaseBase : public IBroadphase<T>
        {
            public:
                /**
                 * @brief Sets the enlargement policy.
                 *
                 * @param margin     A fixed inflation, in metres — the slack that
                 *                   absorbs small motion without a re-insertion.
                 * @param prediction How many multiples of the reported displacement
                 *                   to lean the box by. Two is the usual choice: one
                 *                   tick of travel, plus one of hysteresis.
                 */
                void set_enlargement(T margin, T prediction) noexcept
                {
                    margin_ = margin;
                    prediction_ = prediction;
                }

                const BroadphaseProxy<T>& proxy(ProxyId id) const override
                {
                    return proxies_[id];
                }

                /** @brief How many live proxies there are. */
                std::size_t proxy_count() const noexcept { return live_count_; }

                ProxyId create_proxy(const AABB<T>& bounds, const CollisionFilter& filter,
                                     std::uint32_t flags, std::uint32_t payload) override
                {
                    ProxyId id;
                    if (!free_proxies_.empty())
                    {
                        id = free_proxies_.back();
                        free_proxies_.pop_back();
                    }
                    else
                    {
                        id = static_cast<ProxyId>(proxies_.size());
                        proxies_.push_back(BroadphaseProxy<T>{});
                    }
                    BroadphaseProxy<T>& record = proxies_[id];
                    record = BroadphaseProxy<T>{};
                    record.tight_bounds = bounds;
                    record.bounds = aabb_expand(bounds, margin_);
                    record.filter = filter;
                    record.flags = flags;
                    record.payload = payload;
                    record.alive = true;
                    ++live_count_;
                    on_proxy_created(id);
                    return id;
                }

                void destroy_proxy(ProxyId id) override
                {
                    if (id >= proxies_.size() || !proxies_[id].alive)
                        return;
                    on_proxy_destroyed(id);
                    proxies_[id].alive = false;
                    free_proxies_.push_back(id);
                    --live_count_;
                }

                void update_proxy(ProxyId id, const AABB<T>& bounds,
                                  const Vector3T<T>& displacement) override
                {
                    if (id >= proxies_.size() || !proxies_[id].alive)
                        return;
                    BroadphaseProxy<T>& record = proxies_[id];
                    record.tight_bounds = bounds;
                    // A swept proxy's box depends on its velocity and not only on
                    // where it is, so containment is the wrong question to ask about
                    // it: a stationary bullet that has just been fired is inside its
                    // old box and still needs a box covering where it is going. It
                    // pays a re-placement every tick, which is the price §7.5 already
                    // says the few continuous bodies pay.
                    const bool swept = has_any_flag(record.flags, BodyFlags::continuous_collision);
                    if (!swept && aabb_contains(record.bounds, bounds))
                        return; // still inside what the structure was told; nothing to do
                    record.bounds = enlarged(bounds, displacement, record.flags);
                    on_proxy_moved(id);
                }

                void set_proxy_state(ProxyId id, const CollisionFilter& filter,
                                     std::uint32_t flags) override
                {
                    if (id >= proxies_.size() || !proxies_[id].alive)
                        return;
                    proxies_[id].filter = filter;
                    proxies_[id].flags = flags;
                }

                const std::vector<BroadphasePair>& pairs() const noexcept override
                {
                    return pairs_;
                }
                const std::vector<BroadphasePair>& added_pairs() const noexcept override
                {
                    return added_;
                }
                const std::vector<BroadphasePair>& persisted_pairs() const noexcept override
                {
                    return persisted_;
                }
                const std::vector<BroadphasePair>& removed_pairs() const noexcept override
                {
                    return removed_;
                }

            protected:
                /**
                 * @brief The enlarged box a moving proxy is stored with.
                 *
                 * Three contributions. The fixed margin is the hysteresis. The
                 * prediction leans the box the way the body is going, so the
                 * re-insertion it eventually needs is one insertion rather than one
                 * per tick. A body flagged for continuous collision additionally
                 * *sweeps*: its box covers where it will be, so the pair exists
                 * before the impact rather than after the body has passed through
                 * (§7.5). That is the broadphase's whole half of continuous collision.
                 */
                AABB<T> enlarged(const AABB<T>& tight, const Vector3T<T>& displacement,
                                 std::uint32_t flags) const noexcept
                {
                    AABB<T> box = aabb_expand(tight, margin_);
                    const Vector3T<T> lean = displacement * prediction_;
                    if (lean.x < T(0))
                        box.min.x += lean.x;
                    else
                        box.max.x += lean.x;
                    if (lean.y < T(0))
                        box.min.y += lean.y;
                    else
                        box.max.y += lean.y;
                    if (lean.z < T(0))
                        box.min.z += lean.z;
                    else
                        box.max.z += lean.z;
                    if (has_any_flag(flags, BodyFlags::continuous_collision))
                    {
                        const AABB<T> swept{tight.min + displacement, tight.max + displacement};
                        box = aabb_union(box, aabb_expand(swept, margin_));
                    }
                    return box;
                }

                /**
                 * @brief Replaces the pair set and derives the three streams from it.
                 *
                 * @p found must be sorted and free of duplicates; the diff is then a
                 * single merge over it and the previous set, which is what keeps a
                 * ten-thousand-pair scene's bookkeeping linear.
                 *
                 * @param found This update's pairs; left empty, its storage reused.
                 */
                void publish_pairs(std::vector<BroadphasePair>& found)
                {
                    added_.clear();
                    removed_.clear();
                    persisted_.clear();

                    std::size_t i = 0;
                    std::size_t j = 0;
                    while (i < found.size() && j < pairs_.size())
                    {
                        if (found[i] < pairs_[j])
                            added_.push_back(found[i++]);
                        else if (pairs_[j] < found[i])
                            removed_.push_back(pairs_[j++]);
                        else
                        {
                            persisted_.push_back(found[i]);
                            ++i;
                            ++j;
                        }
                    }
                    for (; i < found.size(); ++i)
                        added_.push_back(found[i]);
                    for (; j < pairs_.size(); ++j)
                        removed_.push_back(pairs_[j]);
                    pairs_.swap(found);
                    found.clear();
                }

                /** @brief Called after a proxy is created; the structure adopts it. */
                virtual void on_proxy_created(ProxyId) {}
                /** @brief Called before a proxy is destroyed; the structure drops it. */
                virtual void on_proxy_destroyed(ProxyId) {}
                /** @brief Called when a proxy's stored box changed and must be re-placed. */
                virtual void on_proxy_moved(ProxyId) {}

                std::vector<BroadphaseProxy<T>> proxies_;
                std::vector<ProxyId> free_proxies_;
                std::size_t live_count_ = 0;
                T margin_ = T(0.05);
                T prediction_ = T(2);

            private:
                std::vector<BroadphasePair> pairs_;
                std::vector<BroadphasePair> added_;
                std::vector<BroadphasePair> persisted_;
                std::vector<BroadphasePair> removed_;
        };

        /**
         * @brief The reference implementation: sort the boxes, sweep, test.
         *
         * Kept, per §7.1, for two reasons that are not nostalgia. It is the
         * small-scene fast path — under a few dozen proxies a sort beats a tree
         * that has to be maintained — and it is the second implementation the
         * conformance suite needs, since a seam with one implementation behind it
         * has never actually been tested as a seam.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class SweepAndPruneBroadphase final : public BroadphaseBase<T>
        {
            public:
                void update() override
                {
                    boxes_.clear();
                    proxy_of_box_.clear();
                    for (ProxyId id = 0; id < this->proxies_.size(); ++id)
                    {
                        if (!this->proxies_[id].alive)
                            continue;
                        boxes_.push_back(this->proxies_[id].bounds);
                        proxy_of_box_.push_back(id);
                    }

                    raw_pairs_.clear();
                    sweep_and_prune(boxes_, raw_pairs_);

                    found_.clear();
                    found_.reserve(raw_pairs_.size());
                    for (const auto& pair : raw_pairs_)
                    {
                        const ProxyId a = proxy_of_box_[pair.first];
                        const ProxyId b = proxy_of_box_[pair.second];
                        if (!proxies_may_collide(this->proxies_[a], this->proxies_[b]))
                            continue;
                        found_.push_back(BroadphasePair::make(a, b));
                    }
                    std::sort(found_.begin(), found_.end());
                    this->publish_pairs(found_);
                }

                void query_overlap(const AABB<T>& box,
                                   const std::function<void(ProxyId)>& visit) const override
                {
                    for (ProxyId id = 0; id < this->proxies_.size(); ++id)
                        if (this->proxies_[id].alive &&
                            aabb_overlap(this->proxies_[id].bounds, box))
                            visit(id);
                }

                void query_ray(const Vector3T<T>& origin, const Vector3T<T>& direction,
                               T max_distance,
                               const std::function<void(ProxyId)>& visit) const override
                {
                    const Vector3T<T> inverse = ray_inverse_direction(direction);
                    for (ProxyId id = 0; id < this->proxies_.size(); ++id)
                        if (this->proxies_[id].alive &&
                            ray_hits_aabb(this->proxies_[id].bounds, origin, inverse, max_distance))
                            visit(id);
                }

            private:
                std::vector<AABB<T>> boxes_;
                std::vector<ProxyId> proxy_of_box_;
                std::vector<std::pair<std::uint32_t, std::uint32_t>> raw_pairs_;
                std::vector<BroadphasePair> found_;
        };
    } // namespace Physics
} // namespace SushiEngine
