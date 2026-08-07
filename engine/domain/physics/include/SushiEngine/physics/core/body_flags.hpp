/**************************************************************************/
/* body_flags.hpp                                                         */
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
 * @file body_flags.hpp
 * @brief What a body *is*, as data rather than as a type.
 *
 * `ContactBody::is_cloth` is the pattern this replaces. A boolean member that
 * behaviour switches on means every new body kind adds a flag and a branch to code
 * that already worked, which is the Open/Closed violation §4 exists to stop. A flag
 * word plus a collision filter is data: the solver reads it, the narrowphase filters
 * on it, and neither grows a case when a new kind arrives.
 *
 * The bits are deliberately not an enum class with arithmetic operators hand-rolled
 * onto it; a plain `std::uint32_t` field with named constants stays trivially
 * copyable and crosses into device code with no operator lookup.
 */

#include <cstdint>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief Bit positions for `RigidBodyT::flags`.
         *
         * A body with no bits set is an ordinary dynamic body, so a zeroed body is
         * the common case rather than a degenerate one.
         */
        namespace BodyFlags
        {
            /** @brief Moved by the simulation. The default; value zero. */
            constexpr std::uint32_t dynamic_body = 0u;

            /** @brief Moved only by the game, but pushes dynamic bodies it meets. */
            constexpr std::uint32_t kinematic = 1u << 0;

            /** @brief Never moves. Distinct from infinite mass: it is also never integrated. */
            constexpr std::uint32_t static_body = 1u << 1;

            /** @brief Currently asleep: not integrated, not projected, until woken. */
            constexpr std::uint32_t sleeping = 1u << 2;

            /** @brief Opted in to continuous collision, so it cannot tunnel. */
            constexpr std::uint32_t continuous_collision = 1u << 3;

            /** @brief Reports overlaps but never resolves them. */
            constexpr std::uint32_t trigger = 1u << 4;

            /** @brief Excluded from the deterministic island; may run in a narrower precision. */
            constexpr std::uint32_t cosmetic = 1u << 5;

            /** @brief Never allowed to fall asleep, however still it becomes. */
            constexpr std::uint32_t never_sleep = 1u << 6;
        } // namespace BodyFlags

        /** @brief Whether every bit in @p mask is set in @p flags. */
        inline bool has_flags(std::uint32_t flags, std::uint32_t mask) noexcept
        {
            return (flags & mask) == mask;
        }

        /** @brief Whether any bit in @p mask is set in @p flags. */
        inline bool has_any_flag(std::uint32_t flags, std::uint32_t mask) noexcept
        {
            return (flags & mask) != 0u;
        }

        /**
         * @brief Whether a body with @p flags is integrated and projected this tick.
         *
         * The one question the solver asks about flags, answered in one place so
         * "what counts as simulated" cannot drift between the predict pass, the
         * projection, and the velocity pass.
         *
         * @param flags The body's flag word.
         * @return True when the body takes part in the solve.
         */
        inline bool is_simulated(std::uint32_t flags) noexcept
        {
            return !has_any_flag(flags, BodyFlags::static_body | BodyFlags::sleeping);
        }

        /**
         * @brief Whether a body with @p flags is moved by the game rather than by forces.
         *
         * Deliberately *not* folded into @ref is_simulated. A kinematic body is
         * integrated and projected like any other — it is only the input to the
         * integration that differs, a commanded velocity instead of a force — so the
         * question "does this body take part in the solve" still answers yes. Twenty
         * call sites ask that one; narrowing it would drop a kinematic body out of the
         * contact list, the statistics and the wake logic in a single edit.
         *
         * The invariant that makes the rest of the solver need no kinematic case is
         * established at the boundary rather than here: a kinematic body arrives with
         * `inv_mass` and `inv_inertia` at zero, so every projection's own
         * inverse-mass weighting already gives it no share of any correction.
         *
         * @param flags The body's flag word.
         * @return True when the body's pose is the game's to command.
         */
        inline bool is_kinematic(std::uint32_t flags) noexcept
        {
            return has_any_flag(flags, BodyFlags::kinematic);
        }

        /**
         * @brief A body's collision layer and the layers it responds to.
         *
         * Two bodies interact when each one's layer is in the other's mask. Requiring
         * both directions makes the relation symmetric by construction, so a filter
         * cannot be authored such that A sees B but B does not see A — which would
         * make the contact set depend on iteration order, and with it determinism.
         */
        struct CollisionFilter
        {
            /** @brief The single layer this body belongs to, as a one-bit mask. */
            std::uint32_t layer = 1u;

            /** @brief The layers this body collides with. All, by default. */
            std::uint32_t collides_with = 0xFFFFFFFFu;
        };

        /**
         * @brief The layers the engine itself knows the meaning of.
         *
         * Three, and each exists because something in the engine has to name it:
         * everything authored lands on @ref default_layer, and cloth and vehicles
         * each need a layer of their own so that "these do not collide with each
         * other" can be *stated as data* rather than asked as
         * `if (a.is_cloth && b.is_cloth)`. That flag was §1.2 item 15, and its
         * replacement is one line of initialization — which is the whole argument of
         * §4.4 in miniature: a behaviour difference expressed as a type tag has to be
         * tested for everywhere the type goes, and the same difference expressed as
         * data is tested once, by the code that already tests every other pair.
         *
         * Game layers start above these and are the author's to name.
         */
        namespace CollisionLayers
        {
            /** @brief Where a body lands when nobody said otherwise. */
            constexpr std::uint32_t default_layer = 1u << 0;

            /** @brief Cloth particles, which collide with everything except each other. */
            constexpr std::uint32_t cloth = 1u << 1;

            /**
             * @brief A vehicle's own bodies, which collide with the world but not each other.
             *
             * A corner's carrier and its wheel are instanced at the same point and a
             * shell node sits inside the core's hull, so every contact within one car
             * is spurious and each would be resolved against the joint or beam already
             * holding that pair together. Two cars therefore do not collide either,
             * which is the same trade cloth takes and is stated in
             * `docs/design/physics_system.md` §11.
             */
            constexpr std::uint32_t vehicle = 1u << 2;

            /** @brief The first layer a game is free to define. */
            constexpr std::uint32_t first_user_layer = 1u << 3;
        } // namespace CollisionLayers

        /**
         * @brief Whether two filters admit a contact between their bodies.
         * @param a The first body's filter.
         * @param b The second body's filter.
         * @return True when both bodies accept the other's layer.
         */
        inline bool filters_collide(const CollisionFilter& a,
                                    const CollisionFilter& b) noexcept
        {
            return (a.collides_with & b.layer) != 0u && (b.collides_with & a.layer) != 0u;
        }

        /**
         * @brief A filter for a body that collides with everything but its own kind.
         *
         * Self-collision is the exception a great many body kinds want turned off —
         * cloth today, hair and rope later — and every one of them would otherwise
         * arrive as another boolean on another value type.
         *
         * @param layer The body's layer.
         * @return A filter on @p layer that excludes @p layer.
         */
        inline CollisionFilter self_excluding_filter(std::uint32_t layer) noexcept
        {
            CollisionFilter filter;
            filter.layer = layer;
            filter.collides_with = ~layer;
            return filter;
        }
    } // namespace Physics
} // namespace SushiEngine
