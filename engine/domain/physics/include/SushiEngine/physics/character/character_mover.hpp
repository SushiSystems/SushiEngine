/**************************************************************************/
/* character_mover.hpp                                                    */
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
 * @file character_mover.hpp
 * @brief Where a capsule ends up when the world is allowed to object.
 *
 * A character controller is not a body kind and not a solver. It is one question asked
 * repeatedly — "if I move this capsule by this much, what stops it?" — and answered with
 * shape sweeps rather than with constraints, because a character has to feel like an
 * input rather than like a mass. `docs/design/physics_system.md` §16.47.
 *
 * This file names no broadphase, no scene, no ECS and no solver. The sweep arrives as a
 * callable, which is what lets the whole algorithm be tested against a lambda that
 * returns hand-written hits — no device, no world, no `create_physics_simulation`. The
 * seam is the same one `physics_extract.hpp` uses for the same reason.
 *
 * **`up` is a parameter, not a constant.** Every controller in the wild bakes world +Y
 * into its slope test, its step-up and its ground probe, which is correct on a flat
 * scene and silently wrong on a sphere: at the equator the local up is the pole's
 * sideways. This engine samples gravity per body (`physics/aero/wind.hpp`'s sibling
 * `GravitySampler`) and `docs/design/solar_system_overhaul.md` is working toward a
 * walkable planetary surface, so the direction is passed in on every call.
 */

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/collision/scene_query.hpp>
#include <SushiEngine/physics/geometry/shapes.hpp>

namespace SushiEngine
{
    namespace Physics
    {
        /**
         * @brief The dials a character's movement has, none of them a game decision.
         *
         * Every field here describes *geometry the world imposes*, which is why the
         * whole struct can sit under `physics/`: how close the capsule may get, how
         * high a lip it can cross, how steep a face stops being a floor. Walk speed,
         * jump height and fall acceleration are absent on purpose — they are the
         * caller's, and a controller that owned them would also have to own a gravity
         * direction.
         */
        template <typename T>
        struct CharacterMoveSettings
        {
            /**
             * @brief Clearance kept between the capsule and everything it touches.
             *
             * Subtracted from every advance rather than added to any sweep, so the
             * capsule ends a tick near a surface and never *on* it. A character resting
             * exactly on a face starts the next sweep at zero separation, where the
             * hit distance is zero and the advance is zero — it would stick.
             */
            T skin_width = T(0.02);

            /** @brief The tallest lip the capsule may climb; zero disables stepping. */
            T step_height = T(0.4);

            /**
             * @brief `dot(surface_normal, up)` at or above which a face is a floor.
             *
             * A cosine rather than an angle because the comparison is what the
             * algorithm actually does, three times over — deciding whether to slide up
             * a face, whether a step's landing is standable, and whether the character
             * is grounded. Storing the angle would mean three conversions or one
             * cached cosine that can disagree with its own angle. The default is 45°.
             */
            T max_slope_cosine = T(0.70710678118654752);

            /**
             * @brief How far below the capsule to look for ground.
             *
             * What keeps a character on a downward ramp instead of leaving it at every
             * lip and re-landing. Nothing snaps the capsule *to* that ground here: the
             * probe reports, the caller's own vertical motion is what descends.
             */
            T ground_snap = T(0.1);

            /**
             * @brief How many times one call may slide before giving up.
             *
             * A bound, not a tuning knob. Sliding into a corner can project motion back
             * and forth between two faces forever, and the honest answer to that is to
             * stop and report the leftover through @ref CharacterMoveResult::remaining
             * rather than to spin.
             */
            int max_slides = 4;
        };

        /**
         * @brief What the world allowed, and what it cost.
         *
         * @ref remaining is not diagnostic padding. It is how a caller learns it walked
         * into a wall — which is the difference between an idle animation and a
         * pushing-against-something one — and it is the only place that information
         * exists, because the resolved position alone cannot distinguish "arrived" from
         * "was stopped one centimetre in".
         */
        template <typename T>
        struct CharacterMoveResult
        {
            /** @brief The capsule's centre after the move. */
            Vector3T<T> position{};

            /** @brief Displacement that could not be spent, in world space. */
            Vector3T<T> remaining{};

            /** @brief The surface under the capsule, or @p up when there is none. */
            Vector3T<T> ground_normal{};

            /** @brief Whether a walkable surface is within the settings' ground snap. */
            bool grounded = false;

            /** @brief Whether this call climbed a step rather than being stopped by it. */
            bool stepped = false;

            /**
             * @brief How many sweeps this call issued.
             *
             * Reported because the loop is bounded: a caller that never sees the count
             * cannot tell a tick that resolved in one sweep from one that spent its
             * whole slide budget in a corner, and those cost very differently.
             */
            int sweeps = 0;
        };

        /** @brief The component of @p v left after removing everything along @p normal. */
        template <typename T>
        inline Vector3T<T> project_on_plane(const Vector3T<T>& v,
                                            const Vector3T<T>& normal) noexcept
        {
            return v - normal * dot(v, normal);
        }

        /**
         * @brief Whether a surface with @p normal can be stood on.
         *
         * The one comparison the whole file turns on, written once so the slide, the
         * step and the ground probe cannot come to different conclusions about the same
         * face — which would produce a character that walks up something it then falls
         * off, or stands on something it cannot walk along.
         */
        template <typename T>
        inline bool is_walkable(const Vector3T<T>& normal, const Vector3T<T>& up,
                                T max_slope_cosine) noexcept
        {
            return dot(normal, up) >= max_slope_cosine;
        }

        namespace Detail
        {
            /** @brief A capsule copied to a new centre, leaving its shape alone. */
            template <typename T>
            inline CapsuleCollider<T> capsule_at(const CapsuleCollider<T>& capsule,
                                                 const Vector3T<T>& center) noexcept
            {
                CapsuleCollider<T> moved = capsule;
                moved.center = center;
                return moved;
            }

            /**
             * @brief Advances @p position along @p motion, sliding off whatever stops it.
             *
             * The core of collide-and-slide: sweep, advance to just short of the hit,
             * project what is left onto the hit's plane, and go again. What makes it a
             * character controller rather than a raycast loop is the correction after
             * the projection — sliding along an unwalkable face has its up-component
             * removed, so pressing into a cliff moves the capsule sideways along it and
             * never upward. Without that a character climbs vertical walls by walking
             * at them, which is the single most common bug in this algorithm.
             *
             * @param sweep    Called as `sweep(capsule, direction, distance)`.
             * @param capsule  The shape being moved; its centre is ignored.
             * @param position The starting centre; updated in place.
             * @param motion   The displacement to spend; updated to whatever is left.
             * @param up       The local up direction.
             * @param settings The movement dials.
             * @param sweeps   Incremented once per sweep issued.
             */
            template <typename T, typename Sweep>
            inline void slide(Sweep&& sweep, const CapsuleCollider<T>& capsule,
                              Vector3T<T>& position, Vector3T<T>& motion,
                              const Vector3T<T>& up,
                              const CharacterMoveSettings<T>& settings, int& sweeps,
                              Vector3T<T>& refused)
            {
                constexpr T epsilon = T(1e-9);
                for (int slide_index = 0; slide_index < settings.max_slides; ++slide_index)
                {
                    const T distance = length(motion);
                    if (!(distance > epsilon))
                    {
                        motion = Vector3T<T>{T(0), T(0), T(0)};
                        return;
                    }
                    const Vector3T<T> direction = motion * (T(1) / distance);

                    ++sweeps;
                    const RayHit<T> hit = sweep(capsule_at(capsule, position), direction,
                                                distance + settings.skin_width);
                    if (!hit.hit)
                    {
                        position = position + motion;
                        motion = Vector3T<T>{T(0), T(0), T(0)};
                        return;
                    }

                    // Never negative: a sweep that reports a hit closer than the skin
                    // width means the capsule is already inside its clearance, and
                    // stepping backwards to restore it would move the character against
                    // its own input.
                    const T advance = hit.distance > settings.skin_width
                                          ? hit.distance - settings.skin_width
                                          : T(0);
                    position = position + direction * advance;

                    const Vector3T<T> unspent = motion - direction * advance;
                    Vector3T<T> leftover = project_on_plane(unspent, hit.normal);
                    if (!is_walkable(hit.normal, up, settings.max_slope_cosine))
                        leftover = project_on_plane(leftover, up);
                    // What the projection removed is what the world refused, and it is
                    // the only record of it: sliding is *supposed* to consume the
                    // component pressed into a surface, so a caller reading the leftover
                    // alone cannot tell a completed walk from one stopped by a wall.
                    // It is also what a step attempt has to be given — gating that on the
                    // leftover means never attempting one, because a riser projects the
                    // whole forward motion away before anything can ask.
                    refused = refused + (unspent - leftover);
                    motion = leftover;
                }
            }

            /**
             * @brief Tries to cross a lip by going over it: up, forward, then down.
             *
             * Three sweeps, and each one can refuse. Going up can be blocked by a low
             * ceiling; going forward can be blocked by the obstacle actually being a
             * wall rather than a step; coming down can land on something too steep to
             * stand on, which is a ledge and not a stair. Any refusal abandons the
             * attempt and leaves @p position untouched, so a failed step costs sweeps
             * and nothing else.
             *
             * @return Whether a step was taken.
             */
            template <typename T, typename Sweep>
            inline bool try_step(Sweep&& sweep, const CapsuleCollider<T>& capsule,
                                 Vector3T<T>& position, Vector3T<T>& motion,
                                 const Vector3T<T>& up,
                                 const CharacterMoveSettings<T>& settings, int& sweeps)
            {
                constexpr T epsilon = T(1e-9);
                const T forward_distance = length(motion);
                if (!(settings.step_height > T(0)) || !(forward_distance > epsilon))
                    return false;
                const Vector3T<T> forward = motion * (T(1) / forward_distance);

                // Up. A hit here is a ceiling, and a character under a ceiling does not
                // get to try the rest.
                ++sweeps;
                const RayHit<T> ceiling =
                    sweep(capsule_at(capsule, position), up, settings.step_height);
                if (ceiling.hit)
                    return false;
                const Vector3T<T> raised = position + up * settings.step_height;

                // Forward, from up there. A hit means the obstacle is taller than the
                // step height — a wall, which the slide has already handled correctly.
                ++sweeps;
                const RayHit<T> ahead = sweep(capsule_at(capsule, raised), forward,
                                              forward_distance + settings.skin_width);
                if (ahead.hit)
                    return false;
                const Vector3T<T> crossed = raised + motion;

                // Down, looking for the top of the step. No hit means there was nothing
                // to step onto and the capsule would be left hovering.
                ++sweeps;
                const RayHit<T> landing =
                    sweep(capsule_at(capsule, crossed), up * T(-1),
                          settings.step_height + settings.skin_width);
                if (!landing.hit ||
                    !is_walkable(landing.normal, up, settings.max_slope_cosine))
                    return false;

                const T drop = landing.distance > settings.skin_width
                                   ? landing.distance - settings.skin_width
                                   : T(0);
                position = crossed - up * drop;
                motion = Vector3T<T>{T(0), T(0), T(0)};
                return true;
            }
        } // namespace Detail

        /**
         * @brief Resolves one tick of a character's intended movement against the world.
         *
         * The displacement is whatever the caller already decided — walk, jump, fall,
         * knockback, all of it summed. This answers only what the world permits, in
         * four passes: slide the part across @p up, try to step over whatever stopped
         * it, slide the part along @p up, then look down for ground.
         *
         * Horizontal before vertical, and that order is load-bearing rather than
         * arbitrary: resolving the fall first would drop the capsule onto the near face
         * of a stair and turn every step into a wall, because the step attempt needs to
         * begin from the height the character was walking at.
         *
         * @tparam T     The scalar element type.
         * @tparam Sweep `sweep(const CapsuleCollider<T>&, const Vector3T<T>&, T) -> RayHit<T>`.
         * @param sweep        The world, as one question it can answer.
         * @param capsule      The character's shape, posed at its current centre.
         * @param displacement How far the caller wants to move it this tick.
         * @param up           Unit local up: world +Y on a flat scene, the radial on a planet.
         * @param settings     The movement dials.
         * @return Where it ended up, what it could not spend, and what it is standing on.
         */
        template <typename T, typename Sweep>
        inline CharacterMoveResult<T> move_character(
            Sweep&& sweep, const CapsuleCollider<T>& capsule,
            const Vector3T<T>& displacement, const Vector3T<T>& up,
            const CharacterMoveSettings<T>& settings)
        {
            CharacterMoveResult<T> result;
            result.position = capsule.center;
            result.ground_normal = up;

            const Vector3T<T> along = up * dot(displacement, up);
            Vector3T<T> across = displacement - along;

            Vector3T<T> refused{};
            Detail::slide(sweep, capsule, result.position, across, up, settings, result.sweeps,
                          refused);

            // The step is attempted with what the world *refused*, not with what is left
            // to spend. A riser is a vertical face, so sliding projects the entire forward
            // motion away and leaves nothing — gate the attempt on that and a character
            // never climbs anything, which is precisely what happened.
            Vector3T<T> blocked = refused;
            if (length(blocked) > T(1e-9))
            {
                result.stepped = Detail::try_step(sweep, capsule, result.position, blocked, up,
                                                  settings, result.sweeps);
                if (result.stepped)
                    refused = Vector3T<T>{T(0), T(0), T(0)};
            }

            Vector3T<T> vertical = along;
            Detail::slide(sweep, capsule, result.position, vertical, up, settings, result.sweeps,
                          refused);
            result.remaining = across + vertical + refused;

            // The probe runs last, on the pose the caller will actually see, so
            // `grounded` describes where the character *is* rather than where it was
            // before its fall was resolved.
            ++result.sweeps;
            const RayHit<T> ground = sweep(Detail::capsule_at(capsule, result.position),
                                           up * T(-1), settings.ground_snap + settings.skin_width);
            if (ground.hit && is_walkable(ground.normal, up, settings.max_slope_cosine))
            {
                result.grounded = true;
                result.ground_normal = ground.normal;
            }
            return result;
        }
    } // namespace Physics
} // namespace SushiEngine
