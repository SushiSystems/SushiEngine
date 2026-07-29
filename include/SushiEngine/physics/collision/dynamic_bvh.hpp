/**************************************************************************/
/* dynamic_bvh.hpp                                                        */
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
 * @file dynamic_bvh.hpp
 * @brief A bounding-volume hierarchy that survives motion instead of being rebuilt.
 *
 * The mesh hierarchy one module over (`geometry/mesh_bvh.hpp`) is built once and
 * never moves, so it can be built well and then forgotten. This one holds bodies
 * that move every tick, and that changes what "good" means: a tree that is
 * rebuilt each tick is a tree whose quality nobody can afford, and a tree that is
 * never restructured degenerates into a list. §7.1 asks for the third option —
 * incremental insertion by a cost heuristic, refit on move, rebalance on the
 * insertion path, never a full rebuild.
 *
 * Three mechanisms carry that, and each is a decision worth naming.
 *
 * **Fat bounds.** A leaf stores a box larger than the body in it. While the body
 * stays inside its stored box, the tree is *already correct* and the move costs a
 * comparison — which is what makes most bodies free on most ticks. The margin is
 * the caller's to choose (@ref move_proxy takes the fattened box), because how
 * far a body will travel is the broadphase's knowledge, not the tree's.
 *
 * **Branch-and-bound insertion.** A new leaf descends to the sibling that costs
 * least in surface area, the standard surface-area heuristic, with the descent
 * pruned by the bound the current node already guarantees. This is what keeps the
 * tree's quality without ever looking at more than one root-to-leaf path.
 *
 * **Rotation on the way back up.** Refitting after an insertion walks to the root
 * anyway; performing one balancing rotation per step on that walk costs nothing
 * extra and is what stops a stream of sorted insertions — a terrain grid loaded
 * row by row, say — from building a comb.
 *
 * The tree stores a `payload` per leaf and nothing else about what it holds, so a
 * broadphase, a query service, and a future cooked-asset instance list can all
 * use the same tree without it learning what a body is.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /** @brief The "no node" index; a tree with a null root is empty. */
        constexpr std::uint32_t null_bvh_node = 0xFFFFFFFFu;

        /**
         * @brief The deepest traversal this tree admits.
         *
         * A balanced tree of a million leaves is twenty deep, and the rotation on
         * the insertion path keeps the real height near that. Sixty-four is a
         * ceiling that cannot be reached by a tree that fits in memory, chosen so
         * every traversal can carry its stack in an array rather than allocating
         * one — the same reasoning, and the same number, as the mesh hierarchy.
         */
        constexpr std::size_t max_dynamic_bvh_depth = 64;

        /**
         * @brief One node: a box, its family, and either children or a payload.
         *
         * Internal nodes and leaves share a slot so the pool is one array. A leaf
         * is a node with no first child, which is also why @ref DynamicBvhNode::height
         * is zero for a leaf and one more than its taller child otherwise.
         */
        template <typename T>
        struct DynamicBvhNode
        {
            Aabb<T> bounds{};
            std::uint32_t parent = null_bvh_node;
            std::uint32_t child_a = null_bvh_node;
            std::uint32_t child_b = null_bvh_node;
            /** @brief The caller's identifier for a leaf; meaningless on an internal node. */
            std::uint32_t payload = 0;
            std::int32_t height = 0;

            bool is_leaf() const noexcept { return child_a == null_bvh_node; }
        };

        /**
         * @brief The hierarchy itself: insert, move, remove, and ask.
         *
         * Node indices are stable for as long as the node lives, so a caller can
         * hold one as a handle — which is the whole reason a proxy in the
         * broadphase above can be moved in constant time rather than looked up.
         *
         * @tparam T The scalar element type.
         */
        template <typename T>
        class DynamicBvh
        {
            public:
                /** @brief Drops every node; capacity is kept for the next fill. */
                void clear() noexcept
                {
                    nodes_.clear();
                    root_ = null_bvh_node;
                    free_list_ = null_bvh_node;
                    leaf_count_ = 0;
                }

                /** @brief How many leaves the tree holds. */
                std::size_t leaf_count() const noexcept { return leaf_count_; }

                /** @brief The root node index, or @ref null_bvh_node when empty. */
                std::uint32_t root() const noexcept { return root_; }

                /** @brief The tree's height: zero for a single leaf, -1 when empty. */
                std::int32_t height() const noexcept
                {
                    return root_ == null_bvh_node ? -1 : nodes_[root_].height;
                }

                /** @brief The stored (fattened) box of a node. */
                const Aabb<T>& bounds(std::uint32_t node) const noexcept
                {
                    return nodes_[node].bounds;
                }

                /** @brief The caller's identifier stored on a leaf. */
                std::uint32_t payload(std::uint32_t node) const noexcept
                {
                    return nodes_[node].payload;
                }

                /**
                 * @brief Adds a leaf holding @p payload with the given stored bounds.
                 *
                 * The bounds are stored verbatim: fattening is the caller's policy,
                 * because how much margin a body deserves depends on how fast it is
                 * moving, and the tree does not know that.
                 *
                 * @param box     The leaf's stored bounds.
                 * @param payload The caller's identifier.
                 * @return The new node's index, stable until it is removed.
                 */
                std::uint32_t insert(const Aabb<T>& box, std::uint32_t payload)
                {
                    const std::uint32_t leaf = allocate_node();
                    nodes_[leaf].bounds = box;
                    nodes_[leaf].payload = payload;
                    nodes_[leaf].height = 0;
                    nodes_[leaf].child_a = null_bvh_node;
                    nodes_[leaf].child_b = null_bvh_node;
                    insert_leaf(leaf);
                    ++leaf_count_;
                    return leaf;
                }

                /** @brief Removes a leaf. Its index may be handed out again afterwards. */
                void remove(std::uint32_t leaf)
                {
                    remove_leaf(leaf);
                    free_node(leaf);
                    --leaf_count_;
                }

                /**
                 * @brief Re-places a leaf, but only if it has left its stored box.
                 *
                 * The cheap case is the common one and it is the point of the whole
                 * design: a body that is still inside the box the tree remembers
                 * needs no work at all, because the tree is not wrong about it.
                 *
                 * @param leaf       The leaf to move.
                 * @param tight_box  Where the body actually is now.
                 * @param stored_box What to store if it has to be re-inserted, which
                 *                   the caller has already fattened.
                 * @return True when the tree was restructured.
                 */
                bool move_proxy(std::uint32_t leaf, const Aabb<T>& tight_box,
                                const Aabb<T>& stored_box)
                {
                    if (aabb_contains(nodes_[leaf].bounds, tight_box))
                        return false;
                    replace(leaf, stored_box);
                    return true;
                }

                /**
                 * @brief Re-places a leaf with new stored bounds, whatever they are.
                 *
                 * The unconditional form, for a caller that has already decided the
                 * box must change — the broadphase does, because a body flagged for
                 * continuous collision has a box that depends on its *velocity* and
                 * not only on where it is, so containment is the wrong question to
                 * ask about it. The leaf's index and payload survive.
                 */
                void replace(std::uint32_t leaf, const Aabb<T>& stored_box)
                {
                    const std::uint32_t payload_value = nodes_[leaf].payload;
                    remove_leaf(leaf);
                    nodes_[leaf].bounds = stored_box;
                    nodes_[leaf].payload = payload_value;
                    nodes_[leaf].height = 0;
                    nodes_[leaf].child_a = null_bvh_node;
                    nodes_[leaf].child_b = null_bvh_node;
                    insert_leaf(leaf);
                }

                /**
                 * @brief Calls @p visit with the payload of every leaf overlapping @p box.
                 *
                 * @param box   The query box.
                 * @param visit Called as `visit(payload, node_index)`.
                 */
                template <typename Visit>
                void query(const Aabb<T>& box, Visit&& visit) const
                {
                    if (root_ == null_bvh_node)
                        return;
                    std::uint32_t stack[max_dynamic_bvh_depth];
                    std::size_t depth = 0;
                    stack[depth++] = root_;
                    while (depth > 0)
                    {
                        const std::uint32_t index = stack[--depth];
                        const DynamicBvhNode<T>& node = nodes_[index];
                        if (!aabb_overlap(node.bounds, box))
                            continue;
                        if (node.is_leaf())
                        {
                            visit(node.payload, index);
                            continue;
                        }
                        if (depth + 2 > max_dynamic_bvh_depth)
                            continue; // unreachable for any tree that fits in memory
                        stack[depth++] = node.child_a;
                        stack[depth++] = node.child_b;
                    }
                }

                /**
                 * @brief Calls @p visit for every leaf whose box the ray meets.
                 *
                 * The traversal prunes by box, not by hit distance: shrinking the
                 * ray as hits come in would make the visit order matter, and the
                 * order a tree yields leaves in is not simulation state. Narrowing
                 * to the closest hit is the caller's job, over a set that does not
                 * depend on how the tree happens to be shaped.
                 *
                 * @param origin       The ray's origin.
                 * @param direction    The ray's direction; need not be unit length.
                 * @param max_distance How far along @p direction to look.
                 * @param visit        Called as `visit(payload, node_index)`.
                 */
                template <typename Visit>
                void query_ray(const Vector3T<T>& origin, const Vector3T<T>& direction,
                               T max_distance, Visit&& visit) const
                {
                    if (root_ == null_bvh_node)
                        return;
                    const Vector3T<T> inverse = ray_inverse_direction(direction);

                    std::uint32_t stack[max_dynamic_bvh_depth];
                    std::size_t depth = 0;
                    stack[depth++] = root_;
                    while (depth > 0)
                    {
                        const std::uint32_t index = stack[--depth];
                        const DynamicBvhNode<T>& node = nodes_[index];
                        if (!ray_hits_aabb(node.bounds, origin, inverse, max_distance))
                            continue;
                        if (node.is_leaf())
                        {
                            visit(node.payload, index);
                            continue;
                        }
                        if (depth + 2 > max_dynamic_bvh_depth)
                            continue;
                        stack[depth++] = node.child_a;
                        stack[depth++] = node.child_b;
                    }
                }

                /**
                 * @brief Checks the tree's invariants; for tests, not for the tick.
                 *
                 * Every parent link agrees with its child, every internal box
                 * encloses both children, and every height is one more than its
                 * taller child. A tree that passes this is a tree whose traversals
                 * cannot miss an overlap, which is the property the whole broadphase
                 * rests on.
                 */
                bool validate() const
                {
                    if (root_ == null_bvh_node)
                        return leaf_count_ == 0;
                    if (nodes_[root_].parent != null_bvh_node)
                        return false;
                    return validate_subtree(root_);
                }

            private:
                std::uint32_t allocate_node()
                {
                    if (free_list_ != null_bvh_node)
                    {
                        const std::uint32_t index = free_list_;
                        free_list_ = nodes_[index].parent;
                        nodes_[index] = DynamicBvhNode<T>{};
                        return index;
                    }
                    nodes_.push_back(DynamicBvhNode<T>{});
                    return static_cast<std::uint32_t>(nodes_.size() - 1);
                }

                void free_node(std::uint32_t index) noexcept
                {
                    // A freed slot is marked by a negative height and threaded onto
                    // the free list through `parent`, so no separate bookkeeping array
                    // has to stay in step with the pool.
                    nodes_[index].height = -1;
                    nodes_[index].child_a = null_bvh_node;
                    nodes_[index].child_b = null_bvh_node;
                    nodes_[index].parent = free_list_;
                    free_list_ = index;
                }

                /** @brief The surface-area cost of adding @p box under @p node. */
                T descent_cost(std::uint32_t node, const Aabb<T>& box, T inheritance) const noexcept
                {
                    const Aabb<T> combined = aabb_union(nodes_[node].bounds, box);
                    const T area = aabb_surface_area(combined);
                    if (nodes_[node].is_leaf())
                        return area + inheritance;
                    return area - aabb_surface_area(nodes_[node].bounds) + inheritance;
                }

                void insert_leaf(std::uint32_t leaf)
                {
                    if (root_ == null_bvh_node)
                    {
                        root_ = leaf;
                        nodes_[root_].parent = null_bvh_node;
                        return;
                    }

                    // Descend to the cheapest sibling. At each node the choice is
                    // between stopping here — paying for a new parent box around this
                    // whole subtree — and going down, paying for the growth this leaf
                    // forces on every box on the way.
                    const Aabb<T> box = nodes_[leaf].bounds;
                    std::uint32_t index = root_;
                    while (!nodes_[index].is_leaf())
                    {
                        const T area = aabb_surface_area(nodes_[index].bounds);
                        const Aabb<T> combined = aabb_union(nodes_[index].bounds, box);
                        const T combined_area = aabb_surface_area(combined);
                        const T cost_here = T(2) * combined_area;
                        const T inheritance = T(2) * (combined_area - area);
                        const T cost_a = descent_cost(nodes_[index].child_a, box, inheritance);
                        const T cost_b = descent_cost(nodes_[index].child_b, box, inheritance);
                        if (cost_here < cost_a && cost_here < cost_b)
                            break;
                        index = cost_a < cost_b ? nodes_[index].child_a : nodes_[index].child_b;
                    }

                    const std::uint32_t sibling = index;
                    const std::uint32_t old_parent = nodes_[sibling].parent;
                    const std::uint32_t new_parent = allocate_node();
                    nodes_[new_parent].parent = old_parent;
                    nodes_[new_parent].bounds =
                        aabb_union(nodes_[leaf].bounds, nodes_[sibling].bounds);
                    nodes_[new_parent].height = nodes_[sibling].height + 1;
                    nodes_[new_parent].child_a = sibling;
                    nodes_[new_parent].child_b = leaf;
                    nodes_[sibling].parent = new_parent;
                    nodes_[leaf].parent = new_parent;

                    if (old_parent == null_bvh_node)
                        root_ = new_parent;
                    else if (nodes_[old_parent].child_a == sibling)
                        nodes_[old_parent].child_a = new_parent;
                    else
                        nodes_[old_parent].child_b = new_parent;

                    refit_ancestors(nodes_[leaf].parent);
                }

                void remove_leaf(std::uint32_t leaf)
                {
                    if (leaf == root_)
                    {
                        root_ = null_bvh_node;
                        return;
                    }
                    const std::uint32_t parent = nodes_[leaf].parent;
                    const std::uint32_t grandparent = nodes_[parent].parent;
                    const std::uint32_t sibling =
                        nodes_[parent].child_a == leaf ? nodes_[parent].child_b
                                                       : nodes_[parent].child_a;

                    if (grandparent == null_bvh_node)
                    {
                        root_ = sibling;
                        nodes_[sibling].parent = null_bvh_node;
                        free_node(parent);
                        return;
                    }
                    if (nodes_[grandparent].child_a == parent)
                        nodes_[grandparent].child_a = sibling;
                    else
                        nodes_[grandparent].child_b = sibling;
                    nodes_[sibling].parent = grandparent;
                    free_node(parent);
                    refit_ancestors(grandparent);
                }

                /** @brief Walks to the root, rebalancing and refitting each node once. */
                void refit_ancestors(std::uint32_t index)
                {
                    while (index != null_bvh_node)
                    {
                        index = balance(index);
                        const std::uint32_t a = nodes_[index].child_a;
                        const std::uint32_t b = nodes_[index].child_b;
                        nodes_[index].bounds = aabb_union(nodes_[a].bounds, nodes_[b].bounds);
                        nodes_[index].height =
                            1 + (nodes_[a].height > nodes_[b].height ? nodes_[a].height
                                                                     : nodes_[b].height);
                        index = nodes_[index].parent;
                    }
                }

                /**
                 * @brief One AVL rotation at @p index, if its children differ by two.
                 *
                 * Promotes the taller child's taller grandchild. Returns whatever
                 * ends up occupying @p index's place, so the caller's walk to the
                 * root continues from the right node.
                 */
                std::uint32_t balance(std::uint32_t index)
                {
                    if (nodes_[index].is_leaf() || nodes_[index].height < 2)
                        return index;

                    const std::uint32_t a = nodes_[index].child_a;
                    const std::uint32_t b = nodes_[index].child_b;
                    const std::int32_t difference = nodes_[b].height - nodes_[a].height;
                    if (difference > 1)
                        return rotate(index, b, a);
                    if (difference < -1)
                        return rotate(index, a, b);
                    return index;
                }

                /**
                 * @brief Promotes @p tall above @p index, keeping @p tall's taller child.
                 *
                 * @param index The over-tall node.
                 * @param tall  Its taller child, which becomes the subtree's new root.
                 * @param other Its shorter child, which descends under @p tall.
                 * @return @p tall, now in @p index's former place.
                 */
                std::uint32_t rotate(std::uint32_t index, std::uint32_t tall, std::uint32_t other)
                {
                    const std::uint32_t grandchild_a = nodes_[tall].child_a;
                    const std::uint32_t grandchild_b = nodes_[tall].child_b;

                    // Swing `tall` up into `index`'s place.
                    nodes_[tall].child_a = index;
                    nodes_[tall].parent = nodes_[index].parent;
                    nodes_[index].parent = tall;

                    if (nodes_[tall].parent == null_bvh_node)
                        root_ = tall;
                    else if (nodes_[nodes_[tall].parent].child_a == index)
                        nodes_[nodes_[tall].parent].child_a = tall;
                    else
                        nodes_[nodes_[tall].parent].child_b = tall;

                    // The taller grandchild stays high; the shorter one drops to sit
                    // beside `other` under `index`.
                    const bool keep_a = nodes_[grandchild_a].height > nodes_[grandchild_b].height;
                    const std::uint32_t high = keep_a ? grandchild_a : grandchild_b;
                    const std::uint32_t low = keep_a ? grandchild_b : grandchild_a;

                    nodes_[tall].child_b = high;
                    nodes_[index].child_a = other;
                    nodes_[index].child_b = low;
                    nodes_[low].parent = index;

                    nodes_[index].bounds = aabb_union(nodes_[other].bounds, nodes_[low].bounds);
                    nodes_[tall].bounds = aabb_union(nodes_[index].bounds, nodes_[high].bounds);
                    nodes_[index].height =
                        1 + (nodes_[other].height > nodes_[low].height ? nodes_[other].height
                                                                       : nodes_[low].height);
                    nodes_[tall].height =
                        1 + (nodes_[index].height > nodes_[high].height ? nodes_[index].height
                                                                        : nodes_[high].height);
                    return tall;
                }

                bool validate_subtree(std::uint32_t index) const
                {
                    const DynamicBvhNode<T>& node = nodes_[index];
                    if (node.is_leaf())
                        return node.height == 0 && node.child_b == null_bvh_node;

                    const std::uint32_t a = node.child_a;
                    const std::uint32_t b = node.child_b;
                    if (a == null_bvh_node || b == null_bvh_node)
                        return false;
                    if (nodes_[a].parent != index || nodes_[b].parent != index)
                        return false;
                    const std::int32_t expected =
                        1 + (nodes_[a].height > nodes_[b].height ? nodes_[a].height
                                                                 : nodes_[b].height);
                    if (node.height != expected)
                        return false;
                    if (!aabb_contains(node.bounds, nodes_[a].bounds) ||
                        !aabb_contains(node.bounds, nodes_[b].bounds))
                        return false;
                    return validate_subtree(a) && validate_subtree(b);
                }

                std::vector<DynamicBvhNode<T>> nodes_;
                std::uint32_t root_ = null_bvh_node;
                std::uint32_t free_list_ = null_bvh_node;
                std::size_t leaf_count_ = 0;
        };
    } // namespace Physics
} // namespace SushiEngine
