/**************************************************************************/
/* acoustic_geometry.hpp                                                 */
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

#ifndef SUSHIENGINE_AUDIO_ACOUSTIC_GEOMETRY_HPP
#define SUSHIENGINE_AUDIO_ACOUSTIC_GEOMETRY_HPP

/**
 * @file acoustic_geometry.hpp
 * @brief The acoustic BVH — a dedicated collision world for sound occlusion queries.
 *
 * Sound is occluded by a **simplified** acoustic mesh, not the render geometry: coarse
 * triangles, each tagged with an @ref AcousticMaterial, so a wall is a handful of
 * quads rather than a million-triangle bake (see `docs/slop/audio_system.md` §6). The
 * structure is two-level, exactly like a ray-tracer's:
 *
 *   - @ref AcousticBlas — a **bottom-level** BVH built once over one @ref AcousticMesh
 *     (the static, high-quality acceleration structure for a piece of geometry).
 *   - @ref AcousticScene — a **top-level** BVH (TLAS) over @ref AcousticInstance objects,
 *     each a BLAS placed by a rotation+translation. Moving a rigid object is a transform
 *     update + a cheap TLAS @ref AcousticScene::refit (rebuild the top level), never a
 *     BLAS rebuild; a topology change rebuilds only the affected BLAS. Edits are batched
 *     and take effect on @ref AcousticScene::commit — no query runs mid-edit.
 *
 * The queries answer the two questions the occlusion layer asks:
 *
 *   - @ref AcousticScene::line_of_sight — is a single ray from source to listener blocked,
 *     and if so what three-band @ref AcousticMaterial::transmission does it accumulate
 *     through the surfaces it pierces (through-wall sound is bassy for free).
 *   - @ref AcousticScene::soft_occlusion — the source is sampled as a **sphere** with a
 *     small deterministic ray fan, returning a smooth 0..1 blocked *fraction* (so a source
 *     slipping behind a pillar fades rather than popping) plus the mean transmission of the
 *     blocked rays.
 *
 * Everything is portable `float` maths — no SDL, no SushiRuntime — so it unit-tests
 * against a hand-built mesh. The deterministic ray fan uses an index-driven low-discrepancy
 * sequence (no RNG state), so a query is reproducible.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <SushiEngine/audio/acoustic_material.hpp>
#include <SushiEngine/audio/voice.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief An axis-aligned bounding box in the acoustic world (float). */
        struct AcousticAABB
        {
            AudioVec3 min{1e30f, 1e30f, 1e30f};
            AudioVec3 max{-1e30f, -1e30f, -1e30f};

            /** @brief Grows the box to contain a point. */
            void expand(const AudioVec3& p) noexcept
            {
                if (p.x < min.x) min.x = p.x;
                if (p.y < min.y) min.y = p.y;
                if (p.z < min.z) min.z = p.z;
                if (p.x > max.x) max.x = p.x;
                if (p.y > max.y) max.y = p.y;
                if (p.z > max.z) max.z = p.z;
            }

            /** @brief Grows the box to contain another box. */
            void expand(const AcousticAABB& b) noexcept
            {
                expand(b.min);
                expand(b.max);
            }

            /** @brief The box centre. */
            AudioVec3 center() const noexcept
            {
                return AudioVec3{0.5f * (min.x + max.x), 0.5f * (min.y + max.y),
                                 0.5f * (min.z + max.z)};
            }

            /** @brief The longest axis (0 = x, 1 = y, 2 = z). */
            int longest_axis() const noexcept
            {
                const float dx = max.x - min.x, dy = max.y - min.y, dz = max.z - min.z;
                if (dx >= dy && dx >= dz) return 0;
                return dy >= dz ? 1 : 2;
            }

            /**
             * @brief Whether a segment overlaps the box (slab test), within [0, @p tmax].
             * @param o    Segment origin.
             * @param inv  Componentwise reciprocal of the segment direction.
             * @param tmax The segment length in the (unnormalised) direction's units.
             * @return True if the segment intersects the box.
             */
            bool intersects_segment(const AudioVec3& o, const AudioVec3& inv, float tmax) const noexcept
            {
                float t0 = 0.0f, t1 = tmax;
                float lo = (min.x - o.x) * inv.x, hi = (max.x - o.x) * inv.x;
                if (lo > hi) { const float t = lo; lo = hi; hi = t; }
                t0 = lo > t0 ? lo : t0; t1 = hi < t1 ? hi : t1;
                lo = (min.y - o.y) * inv.y; hi = (max.y - o.y) * inv.y;
                if (lo > hi) { const float t = lo; lo = hi; hi = t; }
                t0 = lo > t0 ? lo : t0; t1 = hi < t1 ? hi : t1;
                lo = (min.z - o.z) * inv.z; hi = (max.z - o.z) * inv.z;
                if (lo > hi) { const float t = lo; lo = hi; hi = t; }
                t0 = lo > t0 ? lo : t0; t1 = hi < t1 ? hi : t1;
                return t1 >= t0;
            }
        };

        /** @brief One acoustic triangle: three vertices and a material-table index. */
        struct AcousticTriangle
        {
            AudioVec3 a;
            AudioVec3 b;
            AudioVec3 c;
            std::uint32_t material = 0; /**< Index into the owning @ref AcousticMesh's table. */
        };

        /**
         * @brief A simplified acoustic surface: triangles plus a material table.
         *
         * Build it by hand or with the @ref add_box helper (a shoebox wall set), then hand
         * it to an @ref AcousticBlas. The material table is small and shared by index so a
         * mesh of one material costs one entry.
         */
        class AcousticMesh
        {
            public:
                /** @brief Adds a material to the table and returns its index. */
                std::uint32_t add_material(const AcousticMaterial& m)
                {
                    materials_.push_back(m);
                    return static_cast<std::uint32_t>(materials_.size() - 1);
                }

                /** @brief Adds one triangle referencing a material index. */
                void add_triangle(const AudioVec3& a, const AudioVec3& b, const AudioVec3& c,
                                  std::uint32_t material)
                {
                    triangles_.push_back(AcousticTriangle{a, b, c, material});
                }

                /**
                 * @brief Adds the six faces (twelve triangles) of an axis-aligned box.
                 * @param center   The box centre.
                 * @param half     Half extents on each axis.
                 * @param material The material index every face carries.
                 */
                void add_box(const AudioVec3& center, const AudioVec3& half, std::uint32_t material)
                {
                    const float x0 = center.x - half.x, x1 = center.x + half.x;
                    const float y0 = center.y - half.y, y1 = center.y + half.y;
                    const float z0 = center.z - half.z, z1 = center.z + half.z;
                    const AudioVec3 p[8] = {
                        {x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                        {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
                    auto quad = [&](int i0, int i1, int i2, int i3) {
                        add_triangle(p[i0], p[i1], p[i2], material);
                        add_triangle(p[i0], p[i2], p[i3], material);
                    };
                    quad(0, 1, 2, 3); // -z
                    quad(4, 7, 6, 5); // +z
                    quad(0, 4, 5, 1); // -y
                    quad(3, 2, 6, 7); // +y
                    quad(0, 3, 7, 4); // -x
                    quad(1, 5, 6, 2); // +x
                }

                /** @brief The triangle list. */
                const std::vector<AcousticTriangle>& triangles() const noexcept { return triangles_; }

                /**
                 * @brief The material for a triangle index (generic if out of range).
                 * @param triangle_index Index into @ref triangles.
                 */
                const AcousticMaterial& material_for(std::size_t triangle_index) const noexcept
                {
                    static const AcousticMaterial fallback = AcousticMaterial::generic();
                    if (triangle_index >= triangles_.size())
                        return fallback;
                    const std::uint32_t m = triangles_[triangle_index].material;
                    return m < materials_.size() ? materials_[m] : fallback;
                }

            private:
                std::vector<AcousticTriangle> triangles_;
                std::vector<AcousticMaterial> materials_;
        };

        /** @brief One flat BVH node: a leaf (count > 0) or an internal node (count == 0). */
        struct AcousticBVHNode
        {
            AcousticAABB bounds;
            int left = 0;  /**< Internal: left child node index. */
            int right = 0; /**< Internal: right child node index (subtrees are not contiguous). */
            int first = 0; /**< Leaf: first primitive in the ordering array. */
            int count = 0; /**< Leaf primitive count; 0 marks an internal node. */
        };

        /**
         * @brief A bottom-level acoustic BVH built once over one @ref AcousticMesh.
         *
         * A median-split AABB tree over the mesh triangles, flattened into a node array
         * with a reordered primitive-index list. @ref any_hit answers occlusion (is the
         * segment blocked at all) and @ref pierced_materials collects, in order, the
         * materials of the first surfaces a segment crosses (for transmission).
         */
        class AcousticBlas
        {
            public:
                AcousticBlas() = default;

                /** @brief Builds the BVH over @p mesh (which must outlive this BLAS). */
                void build(const AcousticMesh& mesh)
                {
                    mesh_ = &mesh;
                    const std::size_t n = mesh.triangles().size();
                    order_.resize(n);
                    centroids_.resize(n);
                    std::vector<AcousticAABB> bounds(n);
                    for (std::size_t i = 0; i < n; ++i)
                    {
                        order_[i] = static_cast<int>(i);
                        const AcousticTriangle& t = mesh.triangles()[i];
                        bounds[i].expand(t.a);
                        bounds[i].expand(t.b);
                        bounds[i].expand(t.c);
                        centroids_[i] = bounds[i].center();
                    }
                    nodes_.clear();
                    if (n == 0)
                        return;
                    nodes_.reserve(2 * n);
                    build_range(0, static_cast<int>(n), bounds);
                }

                /** @brief The world AABB of the whole mesh (empty if not built). */
                AcousticAABB bounds() const noexcept
                {
                    return nodes_.empty() ? AcousticAABB{} : nodes_[0].bounds;
                }

                /** @brief Whether the BLAS holds any geometry. */
                bool empty() const noexcept { return nodes_.empty(); }

                /**
                 * @brief Whether a segment hits any triangle (local space).
                 * @param origin The segment start.
                 * @param dir    The (unnormalised) segment vector end − start.
                 * @return True if blocked.
                 */
                bool any_hit(const AudioVec3& origin, const AudioVec3& dir) const noexcept
                {
                    if (nodes_.empty())
                        return false;
                    const AudioVec3 inv = reciprocal(dir);
                    int stack[64];
                    int sp = 0;
                    stack[sp++] = 0;
                    while (sp > 0)
                    {
                        const AcousticBVHNode& node = nodes_[static_cast<std::size_t>(stack[--sp])];
                        if (!node.bounds.intersects_segment(origin, inv, 1.0f))
                            continue;
                        if (node.count > 0)
                        {
                            for (int i = 0; i < node.count; ++i)
                            {
                                float t = 0.0f;
                                if (triangle_hit(order_[static_cast<std::size_t>(node.first + i)],
                                                 origin, dir, t))
                                    return true;
                            }
                        }
                        else
                        {
                            stack[sp++] = node.left;
                            stack[sp++] = node.right;
                        }
                    }
                    return false;
                }

                /**
                 * @brief Multiplies @p transmission by each surface a segment pierces.
                 *
                 * Walks the segment, gathering hit distances and the pierced triangles'
                 * transmission, then folds the nearest @p max_surfaces into @p transmission
                 * (a product — every surface further attenuates). Leaves @p transmission
                 * untouched when nothing is hit.
                 *
                 * @param origin       Segment start.
                 * @param dir          Segment vector (end − start).
                 * @param max_surfaces Cap on surfaces folded in (nearest first).
                 * @param transmission Three-band accumulator, multiplied in place.
                 * @return The number of surfaces the segment pierced (capped).
                 */
                int pierced_materials(const AudioVec3& origin, const AudioVec3& dir, int max_surfaces,
                                      float transmission[ACOUSTIC_BAND_COUNT]) const
                {
                    if (nodes_.empty())
                        return 0;
                    hits_.clear();
                    const AudioVec3 inv = reciprocal(dir);
                    int stack[64];
                    int sp = 0;
                    stack[sp++] = 0;
                    while (sp > 0)
                    {
                        const AcousticBVHNode& node = nodes_[static_cast<std::size_t>(stack[--sp])];
                        if (!node.bounds.intersects_segment(origin, inv, 1.0f))
                            continue;
                        if (node.count > 0)
                        {
                            for (int i = 0; i < node.count; ++i)
                            {
                                const int tri = order_[static_cast<std::size_t>(node.first + i)];
                                float t = 0.0f;
                                if (triangle_hit(tri, origin, dir, t))
                                    hits_.push_back(Hit{t, tri});
                            }
                        }
                        else
                        {
                            stack[sp++] = node.left;
                            stack[sp++] = node.right;
                        }
                    }
                    if (hits_.empty())
                        return 0;

                    // Nearest surfaces first; a partial sort is enough for the cap.
                    std::sort(hits_.begin(), hits_.end(),
                              [](const Hit& x, const Hit& y) { return x.t < y.t; });
                    int folded = 0;
                    for (const Hit& h : hits_)
                    {
                        if (folded >= max_surfaces)
                            break;
                        const AcousticMaterial& m =
                            mesh_->material_for(static_cast<std::size_t>(h.triangle));
                        for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                            transmission[b] *= m.transmission[b];
                        ++folded;
                    }
                    return folded;
                }

            private:
                struct Hit
                {
                    float t;
                    int triangle;
                };

                static AudioVec3 reciprocal(const AudioVec3& d) noexcept
                {
                    const float ex = 1e-20f;
                    return AudioVec3{1.0f / (std::fabs(d.x) < ex ? (d.x < 0 ? -ex : ex) : d.x),
                                     1.0f / (std::fabs(d.y) < ex ? (d.y < 0 ? -ex : ex) : d.y),
                                     1.0f / (std::fabs(d.z) < ex ? (d.z < 0 ? -ex : ex) : d.z)};
                }

                /** @brief Möller–Trumbore, hit accepted for t in (eps, 1] (segment span). */
                bool triangle_hit(int tri, const AudioVec3& o, const AudioVec3& d,
                                  float& t_out) const noexcept
                {
                    const AcousticTriangle& tr = mesh_->triangles()[static_cast<std::size_t>(tri)];
                    const AudioVec3 e1{tr.b.x - tr.a.x, tr.b.y - tr.a.y, tr.b.z - tr.a.z};
                    const AudioVec3 e2{tr.c.x - tr.a.x, tr.c.y - tr.a.y, tr.c.z - tr.a.z};
                    const AudioVec3 p{d.y * e2.z - d.z * e2.y, d.z * e2.x - d.x * e2.z,
                                      d.x * e2.y - d.y * e2.x};
                    const float det = e1.x * p.x + e1.y * p.y + e1.z * p.z;
                    if (det > -1e-8f && det < 1e-8f)
                        return false; // parallel
                    const float inv_det = 1.0f / det;
                    const AudioVec3 s{o.x - tr.a.x, o.y - tr.a.y, o.z - tr.a.z};
                    const float u = (s.x * p.x + s.y * p.y + s.z * p.z) * inv_det;
                    if (u < 0.0f || u > 1.0f)
                        return false;
                    const AudioVec3 q{s.y * e1.z - s.z * e1.y, s.z * e1.x - s.x * e1.z,
                                      s.x * e1.y - s.y * e1.x};
                    const float v = (d.x * q.x + d.y * q.y + d.z * q.z) * inv_det;
                    if (v < 0.0f || u + v > 1.0f)
                        return false;
                    const float t = (e2.x * q.x + e2.y * q.y + e2.z * q.z) * inv_det;
                    if (t <= 1e-4f || t > 1.0f)
                        return false; // behind origin, at origin, or past the listener
                    t_out = t;
                    return true;
                }

                int build_range(int begin, int end, std::vector<AcousticAABB>& bounds)
                {
                    const int node_index = static_cast<int>(nodes_.size());
                    nodes_.push_back(AcousticBVHNode{});
                    AcousticAABB box;
                    for (int i = begin; i < end; ++i)
                        box.expand(bounds[static_cast<std::size_t>(order_[static_cast<std::size_t>(i)])]);

                    const int count = end - begin;
                    if (count <= 4)
                    {
                        AcousticBVHNode& leaf = nodes_[static_cast<std::size_t>(node_index)];
                        leaf.bounds = box;
                        leaf.first = begin;
                        leaf.count = count;
                        return node_index;
                    }

                    const int axis = box.longest_axis();
                    const int mid = begin + count / 2;
                    // Partition the primitive index range about the centroid median.
                    std::nth_element(order_.begin() + begin, order_.begin() + mid,
                                     order_.begin() + end, [&](int a, int b) {
                                         const AudioVec3& ca = centroids_[static_cast<std::size_t>(a)];
                                         const AudioVec3& cb = centroids_[static_cast<std::size_t>(b)];
                                         const float va = axis == 0 ? ca.x : (axis == 1 ? ca.y : ca.z);
                                         const float vb = axis == 0 ? cb.x : (axis == 1 ? cb.y : cb.z);
                                         return va < vb;
                                     });

                    const int left = build_range(begin, mid, bounds);
                    const int right = build_range(mid, end, bounds);
                    AcousticBVHNode& node = nodes_[static_cast<std::size_t>(node_index)];
                    node.bounds = box;
                    node.left = left;
                    node.right = right;
                    node.count = 0;
                    return node_index;
                }

                const AcousticMesh* mesh_ = nullptr;
                std::vector<AcousticBVHNode> nodes_;
                std::vector<int> order_;
                std::vector<AudioVec3> centroids_;
                mutable std::vector<Hit> hits_;
        };

        /**
         * @brief A placed BLAS: a rigid instance in the acoustic scene.
         *
         * The transform is a rotation (row-major 3×3) plus a translation, so moving or
         * turning an object updates the transform and the scene @ref AcousticScene::refit
         * without rebuilding the BLAS. Queries transform the segment into the instance's
         * local space (rotation is orthonormal, so its inverse is its transpose).
         */
        struct AcousticInstance
        {
            const AcousticBlas* blas = nullptr; /**< The geometry (borrowed). */
            float rotation[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; /**< Row-major world←local rotation. */
            AudioVec3 translation;                            /**< World position. */

            /** @brief Sets a pure translation (identity rotation). */
            void set_position(const AudioVec3& p) noexcept
            {
                rotation[0] = 1; rotation[1] = 0; rotation[2] = 0;
                rotation[3] = 0; rotation[4] = 1; rotation[5] = 0;
                rotation[6] = 0; rotation[7] = 0; rotation[8] = 1;
                translation = p;
            }

            /** @brief Transforms a world point into instance-local space. */
            AudioVec3 to_local(const AudioVec3& w) const noexcept
            {
                const float x = w.x - translation.x, y = w.y - translation.y, z = w.z - translation.z;
                // local = R^T * (w - t)
                return AudioVec3{rotation[0] * x + rotation[3] * y + rotation[6] * z,
                                 rotation[1] * x + rotation[4] * y + rotation[7] * z,
                                 rotation[2] * x + rotation[5] * y + rotation[8] * z};
            }

            /** @brief Transforms a world direction into instance-local space (no translation). */
            AudioVec3 dir_to_local(const AudioVec3& d) const noexcept
            {
                return AudioVec3{rotation[0] * d.x + rotation[3] * d.y + rotation[6] * d.z,
                                 rotation[1] * d.x + rotation[4] * d.y + rotation[7] * d.z,
                                 rotation[2] * d.x + rotation[5] * d.y + rotation[8] * d.z};
            }

            /** @brief The instance's world-space AABB (transform of the BLAS bounds). */
            AcousticAABB world_bounds() const noexcept
            {
                AcousticAABB out;
                if (blas == nullptr || blas->empty())
                    return out;
                const AcousticAABB lb = blas->bounds();
                const AudioVec3 corners[8] = {
                    {lb.min.x, lb.min.y, lb.min.z}, {lb.max.x, lb.min.y, lb.min.z},
                    {lb.min.x, lb.max.y, lb.min.z}, {lb.max.x, lb.max.y, lb.min.z},
                    {lb.min.x, lb.min.y, lb.max.z}, {lb.max.x, lb.min.y, lb.max.z},
                    {lb.min.x, lb.max.y, lb.max.z}, {lb.max.x, lb.max.y, lb.max.z}};
                for (const AudioVec3& c : corners)
                {
                    // world = R * local + t
                    out.expand(AudioVec3{
                        rotation[0] * c.x + rotation[1] * c.y + rotation[2] * c.z + translation.x,
                        rotation[3] * c.x + rotation[4] * c.y + rotation[5] * c.z + translation.y,
                        rotation[6] * c.x + rotation[7] * c.y + rotation[8] * c.z + translation.z});
                }
                return out;
            }
        };

        /** @brief The result of a soft (multi-ray) occlusion query. */
        struct OcclusionResult
        {
            float fraction = 0.0f; /**< Blocked ray fraction in [0, 1] (the obstruction/occlusion driver). */
            float transmission[ACOUSTIC_BAND_COUNT] = {1.0f, 1.0f, 1.0f}; /**< Mean three-band leak of blocked rays. */
        };

        /**
         * @brief The top-level acoustic scene: a TLAS over placed BLAS instances.
         *
         * Add instances, then @ref commit to build the top-level BVH; move instances and
         * @ref refit (which re-commits) to update. All edits are host-side and single-shot
         * — never call a query while editing (the RT thread reads a committed scene).
         */
        class AcousticScene
        {
            public:
                /** @brief Adds an instance and returns its index. */
                std::size_t add_instance(const AcousticInstance& instance)
                {
                    instances_.push_back(instance);
                    return instances_.size() - 1;
                }

                /** @brief Mutable access to a placed instance (to move/rotate it, then @ref refit). */
                AcousticInstance& instance(std::size_t i) noexcept { return instances_[i]; }

                /** @brief The number of instances in the scene. */
                std::size_t instance_count() const noexcept { return instances_.size(); }

                /** @brief Removes every instance (the TLAS is rebuilt on the next @ref commit). */
                void clear() noexcept
                {
                    instances_.clear();
                    nodes_.clear();
                    order_.clear();
                }

                /** @brief Builds the top-level BVH over the current instances. */
                void commit()
                {
                    const std::size_t n = instances_.size();
                    order_.resize(n);
                    centroids_.resize(n);
                    std::vector<AcousticAABB> bounds(n);
                    for (std::size_t i = 0; i < n; ++i)
                    {
                        order_[i] = static_cast<int>(i);
                        bounds[i] = instances_[i].world_bounds();
                        centroids_[i] = bounds[i].center();
                    }
                    nodes_.clear();
                    if (n == 0)
                        return;
                    nodes_.reserve(2 * n);
                    build_range(0, static_cast<int>(n), bounds);
                }

                /** @brief Rebuilds the TLAS after moving instances (BLAS untouched). */
                void refit() { commit(); }

                /**
                 * @brief A single line-of-sight test between two world points.
                 *
                 * @param source       The emitter world position.
                 * @param listener     The listener world position.
                 * @param max_surfaces Cap on surfaces folded into @p transmission.
                 * @param transmission Three-band leak (filled with 1s and multiplied down
                 *                      per pierced surface; all 1s means a clear path).
                 * @return True if the path is blocked by any surface.
                 */
                bool line_of_sight(const AudioVec3& source, const AudioVec3& listener,
                                   int max_surfaces,
                                   float transmission[ACOUSTIC_BAND_COUNT]) const
                {
                    for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                        transmission[b] = 1.0f;
                    const AudioVec3 dir{listener.x - source.x, listener.y - source.y,
                                        listener.z - source.z};
                    int pierced = 0;
                    gather(source, dir,
                           [&](std::size_t index)
                           {
                               const AcousticInstance& inst = instances_[index];
                               if (inst.blas == nullptr)
                                   return;
                               const AudioVec3 lo = inst.to_local(source);
                               const AudioVec3 ld = inst.dir_to_local(dir);
                               pierced +=
                                   inst.blas->pierced_materials(lo, ld, max_surfaces, transmission);
                           });
                    return pierced > 0;
                }

                /** @brief Whether the direct path is blocked at all (cheap, no transmission). */
                bool occluded(const AudioVec3& source, const AudioVec3& listener) const
                {
                    const AudioVec3 dir{listener.x - source.x, listener.y - source.y,
                                        listener.z - source.z};
                    bool hit = false;
                    gather(
                        source, dir,
                        [&](std::size_t index)
                        {
                            if (hit)
                                return;
                            const AcousticInstance& inst = instances_[index];
                            if (inst.blas == nullptr)
                                return;
                            if (inst.blas->any_hit(inst.to_local(source), inst.dir_to_local(dir)))
                                hit = true;
                        });
                    return hit;
                }

                /**
                 * @brief Soft occlusion: sample the source as a sphere with a ray fan.
                 *
                 * Casts @p ray_count rays from the listener to deterministic points on the
                 * source sphere and returns the blocked fraction (a smooth 0..1 obstruction
                 * signal, no single-ray pop) and the mean transmission of the blocked rays.
                 *
                 * @param source       The source centre (world).
                 * @param listener     The listener position (world).
                 * @param radius       The source sphere radius in metres.
                 * @param ray_count    Number of rays (clamped to a small internal maximum).
                 * @param max_surfaces Per-ray transmission surface cap.
                 * @return The fraction blocked and the mean blocked-ray transmission.
                 */
                OcclusionResult soft_occlusion(const AudioVec3& source, const AudioVec3& listener,
                                               float radius, int ray_count, int max_surfaces) const
                {
                    OcclusionResult result;
                    if (ray_count < 1) ray_count = 1;
                    if (ray_count > 64) ray_count = 64;

                    int blocked = 0;
                    float sum_t[ACOUSTIC_BAND_COUNT] = {0.0f, 0.0f, 0.0f};
                    for (int i = 0; i < ray_count; ++i)
                    {
                        const AudioVec3 target = sphere_point(source, radius, i, ray_count);
                        float t[ACOUSTIC_BAND_COUNT];
                        if (line_of_sight(target, listener, max_surfaces, t))
                        {
                            ++blocked;
                            for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                                sum_t[b] += t[b];
                        }
                    }
                    result.fraction = static_cast<float>(blocked) / static_cast<float>(ray_count);
                    if (blocked > 0)
                        for (int b = 0; b < ACOUSTIC_BAND_COUNT; ++b)
                            result.transmission[b] = sum_t[b] / static_cast<float>(blocked);
                    return result;
                }

            private:
                template <typename Visitor>
                void gather(const AudioVec3& origin, const AudioVec3& dir, Visitor&& visit) const
                {
                    if (nodes_.empty())
                        return;
                    const float ex = 1e-20f;
                    const AudioVec3 inv{
                        1.0f / (std::fabs(dir.x) < ex ? (dir.x < 0 ? -ex : ex) : dir.x),
                        1.0f / (std::fabs(dir.y) < ex ? (dir.y < 0 ? -ex : ex) : dir.y),
                        1.0f / (std::fabs(dir.z) < ex ? (dir.z < 0 ? -ex : ex) : dir.z)};
                    int stack[64];
                    int sp = 0;
                    stack[sp++] = 0;
                    while (sp > 0)
                    {
                        const AcousticBVHNode& node = nodes_[static_cast<std::size_t>(stack[--sp])];
                        if (!node.bounds.intersects_segment(origin, inv, 1.0f))
                            continue;
                        if (node.count > 0)
                        {
                            for (int i = 0; i < node.count; ++i)
                                visit(static_cast<std::size_t>(order_[static_cast<std::size_t>(node.first + i)]));
                        }
                        else
                        {
                            stack[sp++] = node.left;
                            stack[sp++] = node.right;
                        }
                    }
                }

                /**
                 * @brief A deterministic point on the source sphere for ray @p i of @p total.
                 *
                 * A Fibonacci-lattice direction (golden-angle spiral) scaled by the radius —
                 * an even, reproducible spread with no RNG state, so the query is
                 * frame-to-frame stable (the fraction changes smoothly as geometry moves).
                 */
                static AudioVec3 sphere_point(const AudioVec3& center, float radius, int i, int total)
                {
                    if (total <= 1 || radius <= 0.0f)
                        return center;
                    const float golden = 2.399963229728653f; // π·(3 − √5)
                    const float z = 1.0f - 2.0f * (static_cast<float>(i) + 0.5f) /
                                               static_cast<float>(total);
                    const float r = std::sqrt(1.0f - z * z);
                    const float phi = golden * static_cast<float>(i);
                    return AudioVec3{center.x + radius * r * std::cos(phi),
                                     center.y + radius * r * std::sin(phi),
                                     center.z + radius * z};
                }

                int build_range(int begin, int end, std::vector<AcousticAABB>& bounds)
                {
                    const int node_index = static_cast<int>(nodes_.size());
                    nodes_.push_back(AcousticBVHNode{});
                    AcousticAABB box;
                    for (int i = begin; i < end; ++i)
                        box.expand(bounds[static_cast<std::size_t>(order_[static_cast<std::size_t>(i)])]);

                    const int count = end - begin;
                    if (count <= 2)
                    {
                        AcousticBVHNode& leaf = nodes_[static_cast<std::size_t>(node_index)];
                        leaf.bounds = box;
                        leaf.first = begin;
                        leaf.count = count;
                        return node_index;
                    }

                    const int axis = box.longest_axis();
                    const int mid = begin + count / 2;
                    std::nth_element(order_.begin() + begin, order_.begin() + mid,
                                     order_.begin() + end, [&](int a, int b) {
                                         const AudioVec3& ca = centroids_[static_cast<std::size_t>(a)];
                                         const AudioVec3& cb = centroids_[static_cast<std::size_t>(b)];
                                         const float va = axis == 0 ? ca.x : (axis == 1 ? ca.y : ca.z);
                                         const float vb = axis == 0 ? cb.x : (axis == 1 ? cb.y : cb.z);
                                         return va < vb;
                                     });
                    const int left = build_range(begin, mid, bounds);
                    const int right = build_range(mid, end, bounds);
                    AcousticBVHNode& node = nodes_[static_cast<std::size_t>(node_index)];
                    node.bounds = box;
                    node.left = left;
                    node.right = right;
                    node.count = 0;
                    return node_index;
                }

                std::vector<AcousticInstance> instances_;
                std::vector<AcousticBVHNode> nodes_;
                std::vector<int> order_;
                std::vector<AudioVec3> centroids_;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
