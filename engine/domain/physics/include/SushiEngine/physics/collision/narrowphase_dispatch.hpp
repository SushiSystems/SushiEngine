/**************************************************************************/
/* narrowphase_dispatch.hpp                                               */
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
 * @file narrowphase_dispatch.hpp
 * @brief Which routine collides which pair of shapes, as a table rather than a branch.
 *
 * A scene holds shapes it does not know the type of at compile time, so
 * something has to turn a pair of type-erased shapes into a call. §4.2 fixes
 * what that something is: a table of function pointers indexed by the ordered
 * `ShapeType` pair, **populated by registration, not by a `switch`** — because a
 * switch is a file every new shape has to edit, and that is the Open/Closed
 * violation the rule exists to prevent.
 *
 * What makes the table small rather than quadratic is that the entries are
 * *generated*, not written. Every convex shape reaches every other through
 * `generate_convex_manifold`, and every convex shape reaches a half-space plane
 * through `generate_convex_plane_manifold`, so the whole convex block of the
 * table is one fold over a type list. Adding a convex shape is therefore four
 * things, none of which is an edit to an algorithm:
 *
 *   1. the value type, in `geometry/shapes.hpp`;
 *   2. a `support()` overload beside it;
 *   3. a `contact_face()` overload in `collision/convex_manifold.hpp`;
 *   4. a `ShapeTraits` specialization here, and its name in `ConvexShapes`.
 *
 * Shapes that are not bounded convex sets — the half-space plane today, the
 * triangle mesh and height field to come — take explicit entries, because they
 * are genuinely different routines and pretending otherwise would be the fiction
 * that makes a narrowphase unreadable.
 *
 * `ShapeType::box` deliberately has no entries. An axis-aligned box is an
 * oriented box whose orientation is the identity, and `make_box_shape` produces
 * exactly that; keeping a second code path for it would duplicate a shape to
 * save a quaternion.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/convex_manifold.hpp>
#include <SushiEngine/physics/collision/manifold.hpp>
#include <SushiEngine/physics/collision/sdf_manifold.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief A shape whose type is not known until run time.
         *
         * One trivially-copyable value covering every shape kind, so a body's
         * collider can be stored in a flat array and handed to the device without
         * a pointer chase or a virtual call. Cooked geometry is referenced, never
         * owned (§5.2): a thousand crates share one hull vertex array.
         *
         * The fields are a plain superset rather than a union. A union would save
         * perhaps forty bytes and cost the ability to memcpy the thing around,
         * inspect it in a debugger, or hash it for a determinism check — none of
         * which is worth forty bytes at this stage. If a shape column ever becomes
         * the bandwidth limit, §5.2's handle-based compaction is where it goes,
         * and it is a change behind this type rather than to everything using it.
         */
        template <typename T>
        struct CollisionShape
        {
            ShapeType type = ShapeType::sphere;

            /** @brief World position of the shape's origin (unused by a plane). */
            Vector3T<T> center{Vector3T<T>{T(0), T(0), T(0)}};
            /** @brief World orientation (unused by a sphere or a plane). */
            QuaternionT<T> orientation{QuaternionT<T>{T(0), T(0), T(0), T(1)}};

            Vector3T<T> half_extents{Vector3T<T>{T(0.5), T(0.5), T(0.5)}}; /**< Box. */
            T radius = T(0.5);      /**< Sphere and capsule. */
            T half_height = T(0.5); /**< Capsule: half the segment, excluding caps. */

            const Vector3T<T>* vertices = nullptr; /**< Convex hull, in hull-local space. */
            std::uint32_t vertex_count = 0;
            T convex_radius = 0; /**< Convex hull inflation (§5.2). */

            Vector3T<T> plane_normal{Vector3T<T>{T(0), T(1), T(0)}};
            T plane_offset = 0;

            const float* sdf_distances = nullptr; /**< Signed-distance field, §7.5. */
            std::int32_t sdf_resolution = 0;
            Vector3T<T> sdf_field_min{Vector3T<T>{T(0), T(0), T(0)}};
            Vector3T<T> sdf_field_max{Vector3T<T>{T(0), T(0), T(0)}};
        };

        /** @brief A sphere collider as a type-erased shape. */
        template <typename T>
        inline CollisionShape<T> make_sphere_shape(const Vector3T<T>& center, T radius) noexcept
        {
            CollisionShape<T> shape;
            shape.type = ShapeType::sphere;
            shape.center = center;
            shape.radius = radius;
            return shape;
        }

        /**
         * @brief A box collider as a type-erased shape.
         *
         * Always an *oriented* box. An axis-aligned one is this with the identity
         * orientation, which is why `ShapeType::box` needs no dispatch entries.
         */
        template <typename T>
        inline CollisionShape<T> make_box_shape(
            const Vector3T<T>& center, const Vector3T<T>& half_extents,
            const QuaternionT<T>& orientation = QuaternionT<T>{T(0), T(0), T(0), T(1)}) noexcept
        {
            CollisionShape<T> shape;
            shape.type = ShapeType::oriented_box;
            shape.center = center;
            shape.orientation = orientation;
            shape.half_extents = half_extents;
            return shape;
        }

        /** @brief A capsule collider as a type-erased shape; its segment runs along local Y. */
        template <typename T>
        inline CollisionShape<T> make_capsule_shape(
            const Vector3T<T>& center, T half_height, T radius,
            const QuaternionT<T>& orientation = QuaternionT<T>{T(0), T(0), T(0), T(1)}) noexcept
        {
            CollisionShape<T> shape;
            shape.type = ShapeType::capsule;
            shape.center = center;
            shape.orientation = orientation;
            shape.half_height = half_height;
            shape.radius = radius;
            return shape;
        }

        /** @brief A convex hull as a type-erased shape, referencing cooked vertices. */
        template <typename T>
        inline CollisionShape<T> make_hull_shape(
            const Vector3T<T>& center, const Vector3T<T>* vertices, std::uint32_t vertex_count,
            const QuaternionT<T>& orientation = QuaternionT<T>{T(0), T(0), T(0), T(1)},
            T convex_radius = 0) noexcept
        {
            CollisionShape<T> shape;
            shape.type = ShapeType::convex_hull;
            shape.center = center;
            shape.orientation = orientation;
            shape.vertices = vertices;
            shape.vertex_count = vertex_count;
            shape.convex_radius = convex_radius;
            return shape;
        }

        /** @brief A static half-space plane as a type-erased shape. */
        template <typename T>
        inline CollisionShape<T> make_plane_shape(const Vector3T<T>& normal, T offset) noexcept
        {
            CollisionShape<T> shape;
            shape.type = ShapeType::plane;
            shape.plane_normal = normal;
            shape.plane_offset = offset;
            return shape;
        }

        /**
         * @brief A cooked signed-distance field as a type-erased shape (§7.5).
         *
         * @param distances  The baked cube, `resolution^3` values, in the asset's
         *                   own local frame; not owned, must outlive the shape.
         * @param resolution Voxels per axis.
         * @param field_min  Padded local-space bounds minimum.
         * @param field_max  Padded local-space bounds maximum.
         * @param center     World placement of the asset's local origin.
         * @param orientation World placement of the asset's local frame.
         */
        template <typename T>
        inline CollisionShape<T> make_sdf_shape(
            const float* distances, std::int32_t resolution, const Vector3T<T>& field_min,
            const Vector3T<T>& field_max, const Vector3T<T>& center,
            const QuaternionT<T>& orientation = QuaternionT<T>{T(0), T(0), T(0), T(1)}) noexcept
        {
            CollisionShape<T> shape;
            shape.type = ShapeType::signed_distance_field;
            shape.center = center;
            shape.orientation = orientation;
            shape.sdf_distances = distances;
            shape.sdf_resolution = resolution;
            shape.sdf_field_min = field_min;
            shape.sdf_field_max = field_max;
            return shape;
        }

        /**
         * @brief How a concrete shape type relates to the type-erased one.
         *
         * The registration point: a specialization names the enumerator and says
         * how to recover the concrete value. Everything else about a shape's place
         * in the dispatch table follows from this and from the shape appearing in
         * @ref ConvexShapes.
         */
        template <typename T, typename Shape>
        struct ShapeTraits;

        template <typename T>
        struct ShapeTraits<T, SphereCollider<T>>
        {
            static constexpr ShapeType type = ShapeType::sphere;
            static SphereCollider<T> from(const CollisionShape<T>& shape) noexcept
            {
                return SphereCollider<T>{shape.center, shape.radius};
            }
        };

        template <typename T>
        struct ShapeTraits<T, OrientedBox<T>>
        {
            static constexpr ShapeType type = ShapeType::oriented_box;
            static OrientedBox<T> from(const CollisionShape<T>& shape) noexcept
            {
                return OrientedBox<T>{shape.center, shape.half_extents, shape.orientation};
            }
        };

        template <typename T>
        struct ShapeTraits<T, CapsuleCollider<T>>
        {
            static constexpr ShapeType type = ShapeType::capsule;
            static CapsuleCollider<T> from(const CollisionShape<T>& shape) noexcept
            {
                CapsuleCollider<T> capsule;
                capsule.center = shape.center;
                capsule.orientation = shape.orientation;
                capsule.half_height = shape.half_height;
                capsule.radius = shape.radius;
                return capsule;
            }
        };

        template <typename T>
        struct ShapeTraits<T, SdfCollider<T>>
        {
            static constexpr ShapeType type = ShapeType::signed_distance_field;
            static SdfCollider<T> from(const CollisionShape<T>& shape) noexcept
            {
                SdfCollider<T> field;
                field.distances = shape.sdf_distances;
                field.resolution = shape.sdf_resolution;
                field.field_min = shape.sdf_field_min;
                field.field_max = shape.sdf_field_max;
                field.center = shape.center;
                field.orientation = shape.orientation;
                return field;
            }
        };

        template <typename T>
        struct ShapeTraits<T, ConvexHullView<T>>
        {
            static constexpr ShapeType type = ShapeType::convex_hull;
            static ConvexHullView<T> from(const CollisionShape<T>& shape) noexcept
            {
                ConvexHullView<T> hull;
                hull.vertices = shape.vertices;
                hull.vertex_count = shape.vertex_count;
                hull.center = shape.center;
                hull.orientation = shape.orientation;
                hull.convex_radius = shape.convex_radius;
                return hull;
            }
        };

        /**
         * @brief A type-erased shape's world bounds, whatever kind it is.
         *
         * The one place the shape kinds are enumerated for bounds, so a broadphase,
         * a compound, and a query all agree about how big a shape is. A half-space
         * reports a huge finite box rather than an infinity: every caller is about
         * to intersect it with something finite, and an infinity there turns a
         * subtraction into a not-a-number.
         */
        template <typename T>
        inline Aabb<T> shape_world_bounds(const CollisionShape<T>& shape) noexcept
        {
            switch (shape.type)
            {
                case ShapeType::sphere:
                    return world_bounds(ShapeTraits<T, SphereCollider<T>>::from(shape));
                case ShapeType::box:
                case ShapeType::oriented_box:
                    return world_bounds(ShapeTraits<T, OrientedBox<T>>::from(shape));
                case ShapeType::capsule:
                    return world_bounds(ShapeTraits<T, CapsuleCollider<T>>::from(shape));
                case ShapeType::convex_hull:
                    return world_bounds(ShapeTraits<T, ConvexHullView<T>>::from(shape));
                case ShapeType::signed_distance_field:
                    return world_bounds(ShapeTraits<T, SdfCollider<T>>::from(shape));
                default:
                    break;
            }
            constexpr T huge = T(1e18);
            return Aabb<T>{Vector3T<T>{-huge, -huge, -huge}, Vector3T<T>{huge, huge, huge}};
        }

        /** @brief A list of types, for folding over. */
        template <typename... Shapes>
        struct TypeList
        {
        };

        /**
         * @brief The convex shapes the table generates entries for.
         *
         * Naming a shape here gives it every convex pairing and its ground contact.
         * This list, and the `ShapeTraits` specialization above it, are the whole
         * registration.
         */
        template <typename T>
        using ConvexShapes =
            TypeList<SphereCollider<T>, OrientedBox<T>, CapsuleCollider<T>, ConvexHullView<T>>;

        /** @brief The signature every narrowphase entry has. */
        template <typename T>
        using ManifoldFunction = ContactManifold<T> (*)(const CollisionShape<T>&,
                                                        const CollisionShape<T>&, T, T);

        /** @brief Table entry for a convex pair: recover both types, run the general routine. */
        template <typename T, typename ShapeA, typename ShapeB>
        inline ContactManifold<T> convex_pair_entry(const CollisionShape<T>& a,
                                                    const CollisionShape<T>& b, T contact_offset,
                                                    T face_tolerance) noexcept
        {
            return generate_convex_manifold<T>(ShapeTraits<T, ShapeA>::from(a),
                                               ShapeTraits<T, ShapeB>::from(b), a.center,
                                               a.orientation, b.center, b.orientation,
                                               contact_offset, face_tolerance);
        }

        /** @brief Table entry for a convex shape against a half-space plane. */
        template <typename T, typename ShapeA>
        inline ContactManifold<T> convex_plane_entry(const CollisionShape<T>& a,
                                                     const CollisionShape<T>& b, T contact_offset,
                                                     T face_tolerance) noexcept
        {
            return generate_convex_plane_manifold<T>(
                ShapeTraits<T, ShapeA>::from(a), PlaneCollider<T>{b.plane_normal, b.plane_offset},
                a.center, a.orientation, contact_offset, face_tolerance);
        }

        /**
         * @brief Table entry for a plane against a convex shape: the flip of the above.
         *
         * The table is indexed by the *ordered* pair, so both orders must resolve —
         * and a caller must never have to sort its two shapes before asking. The
         * flipped entry runs the same routine and reverses the answer, so the two
         * orders cannot disagree about anything except which way the normal points.
         */
        template <typename T, typename ShapeB>
        inline ContactManifold<T> plane_convex_entry(const CollisionShape<T>& a,
                                                     const CollisionShape<T>& b, T contact_offset,
                                                     T face_tolerance) noexcept
        {
            ContactManifold<T> manifold = generate_convex_plane_manifold<T>(
                ShapeTraits<T, ShapeB>::from(b), PlaneCollider<T>{a.plane_normal, a.plane_offset},
                b.center, b.orientation, contact_offset, face_tolerance);
            manifold.normal = manifold.normal * T(-1);
            for (std::size_t i = 0; i < manifold.point_count; ++i)
            {
                const Vector3T<T> swap = manifold.points[i].anchor_a_local;
                manifold.points[i].anchor_a_local = manifold.points[i].anchor_b_local;
                manifold.points[i].anchor_b_local = swap;
            }
            return manifold;
        }

        /** @brief Table entry for a convex shape against a signed-distance field. */
        template <typename T, typename ShapeA>
        inline ContactManifold<T> convex_sdf_entry(const CollisionShape<T>& a,
                                                   const CollisionShape<T>& b, T contact_offset,
                                                   T /*face_tolerance*/) noexcept
        {
            return generate_convex_sdf_manifold<T>(ShapeTraits<T, ShapeA>::from(a),
                                                    ShapeTraits<T, SdfCollider<T>>::from(b),
                                                    a.center, a.orientation, contact_offset);
        }

        /**
         * @brief Table entry for a field against a convex shape: the flip of the above.
         *
         * Same reasoning as @ref plane_convex_entry: the table is indexed by the
         * ordered pair, so both orders must resolve without the caller sorting
         * its shapes first.
         */
        template <typename T, typename ShapeB>
        inline ContactManifold<T> sdf_convex_entry(const CollisionShape<T>& a,
                                                   const CollisionShape<T>& b, T contact_offset,
                                                   T /*face_tolerance*/) noexcept
        {
            ContactManifold<T> manifold = generate_convex_sdf_manifold<T>(
                ShapeTraits<T, ShapeB>::from(b), ShapeTraits<T, SdfCollider<T>>::from(a), b.center,
                b.orientation, contact_offset);
            manifold.normal = manifold.normal * T(-1);
            for (std::size_t i = 0; i < manifold.point_count; ++i)
            {
                const Vector3T<T> swap = manifold.points[i].anchor_a_local;
                manifold.points[i].anchor_a_local = manifold.points[i].anchor_b_local;
                manifold.points[i].anchor_b_local = swap;
            }
            return manifold;
        }

        /**
         * @brief The narrowphase dispatch table: one function pointer per ordered shape pair.
         *
         * Built once, at first use, by folding the convex shape list against itself
         * and against the plane. An unregistered pair holds a null entry, and
         * @ref generate_shape_manifold reports no contact for it — a pair nobody has
         * taught the engine to collide should be visibly absent, not quietly
         * approximated by the nearest thing that compiles. (The engine used to do
         * the latter: `gather_rigid_descs` collapsed a cylinder to a sphere, §1.2
         * item 4.)
         */
        template <typename T>
        struct NarrowphaseTable
        {
            static constexpr std::size_t kind_count = static_cast<std::size_t>(ShapeType::count);
            ManifoldFunction<T> entries[kind_count][kind_count] = {};

            void set(ShapeType a, ShapeType b, ManifoldFunction<T> fn) noexcept
            {
                entries[static_cast<std::size_t>(a)][static_cast<std::size_t>(b)] = fn;
            }

            ManifoldFunction<T> get(ShapeType a, ShapeType b) const noexcept
            {
                return entries[static_cast<std::size_t>(a)][static_cast<std::size_t>(b)];
            }
        };

        /** @brief Fills one row of the convex block: @p ShapeA against every convex shape. */
        template <typename T, typename ShapeA, typename... Shapes>
        inline void register_convex_row(NarrowphaseTable<T>& table, TypeList<Shapes...>) noexcept
        {
            (table.set(ShapeTraits<T, ShapeA>::type, ShapeTraits<T, Shapes>::type,
                       &convex_pair_entry<T, ShapeA, Shapes>),
             ...);
            table.set(ShapeTraits<T, ShapeA>::type, ShapeType::plane,
                      &convex_plane_entry<T, ShapeA>);
            table.set(ShapeType::plane, ShapeTraits<T, ShapeA>::type,
                      &plane_convex_entry<T, ShapeA>);
            table.set(ShapeTraits<T, ShapeA>::type, ShapeType::signed_distance_field,
                      &convex_sdf_entry<T, ShapeA>);
            table.set(ShapeType::signed_distance_field, ShapeTraits<T, ShapeA>::type,
                      &sdf_convex_entry<T, ShapeA>);
        }

        /** @brief Fills the whole convex block, plus every convex/plane pairing. */
        template <typename T, typename... Shapes>
        inline void register_convex_shapes(NarrowphaseTable<T>& table, TypeList<Shapes...> list) noexcept
        {
            (register_convex_row<T, Shapes>(table, list), ...);
        }

        /** @brief The one table, built on first use. */
        template <typename T>
        inline const NarrowphaseTable<T>& narrowphase_table() noexcept
        {
            static const NarrowphaseTable<T> table = []()
            {
                NarrowphaseTable<T> built;
                register_convex_shapes<T>(built, ConvexShapes<T>{});
                return built;
            }();
            return table;
        }

        /**
         * @brief Collides two type-erased shapes, whatever they are.
         *
         * The narrowphase's public face. Every routine returns the same
         * `ContactManifold`, with the same convention — the normal runs from @p a
         * toward @p b — so nothing downstream needs to know which pair it got.
         *
         * @param a              The first shape.
         * @param b              The second shape.
         * @param contact_offset Contacts are generated out to this separation (§7.6).
         * @param face_tolerance How flush a feature must be to count as part of a
         *                       contact face, in metres.
         * @return The manifold, or one with no points when the shapes miss or the
         *         pair has no registered routine.
         */
        template <typename T>
        inline ContactManifold<T> generate_shape_manifold(const CollisionShape<T>& a,
                                                          const CollisionShape<T>& b,
                                                          T contact_offset = T(0),
                                                          T face_tolerance = T(1e-3)) noexcept
        {
            const ManifoldFunction<T> entry = narrowphase_table<T>().get(a.type, b.type);
            if (entry == nullptr)
                return ContactManifold<T>{};
            return entry(a, b, contact_offset, face_tolerance);
        }
    } // namespace Physics
} // namespace SushiEngine
