/**************************************************************************/
/* ragdoll.hpp                                                            */
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
 * @file ragdoll.hpp
 * @brief The two halves `Animation::RagdollBlend` was waiting for.
 *
 * `RagdollBlend` is deliberately *the blend and not the physics*: it takes per-joint
 * object-space transforms "a caller already resolved from XPBD bodies" and blends the
 * animated pose toward them. Nothing resolved them, and nothing built the bodies — so
 * the modifier has been complete and unreachable. This header is the missing caller:
 *
 * 1. @ref build_ragdoll_rig turns a cooked skeleton into a @ref PhysicsAssembly — a
 *    capsule per bone, a cone-twist at every joint, mass distributed by volume — plus
 *    the part-to-joint bindings that make the second half possible.
 * 2. @ref resolve_ragdoll_targets reads the solved part poses back and writes
 *    `Animation::RagdollJointTarget`s in the character's object space.
 *
 * Between the two, the assembly is instanced the ordinary way: the caller creates one
 * entity per part, `instantiate_assembly` says what bodies and joints to make, and the
 * ECS owns them (`physics_assembly.hpp` explains why the physics deliberately does
 * not).
 *
 * ### One part per joint that has children, and why leaves get none
 *
 * A bone is the segment from a joint to its children, so a joint with no children has
 * no bone to be a capsule of. Those joints — fingertips, the top of the head, the ends
 * of every chain — get no part and no target, and that is the right answer rather than
 * a gap: `RagdollBlend`'s `recompose()` regenerates every affected joint *and its
 * descendants*, so a leaf keeps its animated local pose relative to a physics-driven
 * parent. A finger that stays curled while the arm falls is what a ragdoll should look
 * like, and simulating twenty finger capsules to get it would be worse in both cost and
 * appearance.
 *
 * ### The bind offset, and why the binding carries one
 *
 * A capsule's segment runs along its own local +Y, so a part's orientation is fixed by
 * the direction of its bone and is *not* the joint's orientation. The binding therefore
 * carries the joint's pose in the part's local frame, measured at bind — so resolving a
 * target is `part_world * offset`, and the capsule's axis convention never leaks into
 * the answer. `Collider` has no local rotation to hide the difference in, and giving it
 * one to make the two coincide would be reshaping a record to avoid storing six
 * numbers.
 *
 * ### `Animation::JointDesc` and `Simulation::JointDesc` are different types
 *
 * Both names are right in their own namespace — one is a cook-time authoring joint of a
 * skeleton, the other the physics boundary's joint description — and this header names
 * both namespaces. Everything here is therefore qualified; a translation unit that
 * `using`s both will find `JointDesc` ambiguous, which is the correct outcome for a
 * name that means two things.
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/animation/ragdoll_blend.hpp>
#include <SushiEngine/animation/skeleton.hpp>
#include <SushiEngine/core/types.hpp>
#include <SushiEngine/physics/constraints/joint.hpp>
#include <SushiEngine/physics/constraints/joint_primitives.hpp>
#include <SushiEngine/physics/geometry/mass_properties.hpp>
#include <SushiEngine/sim/physics_assembly.hpp>
#include <SushiEngine/sim/physics_services.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief The part index meaning "this joint has none". */
        constexpr std::uint32_t NO_RAGDOLL_PART = 0xFFFFFFFFu;

        /**
         * @brief How a skeleton is turned into a ragdoll.
         *
         * Deliberately few numbers. A production ragdoll wants per-limb limits — a knee
         * bends one way and a shoulder is a cone — and that is authored per joint in the
         * assembly editor (§14) once the rig exists. What this profile is for is
         * producing a *plausible* rig from a skeleton and nothing else, which is what a
         * hit reaction or a death ragdoll needs before anyone has authored anything.
         */
        struct RagdollProfile
        {
            /** @brief Total mass of the whole ragdoll, in kilograms, spread by volume. */
            Scalar total_mass = Scalar(70);

            /** @brief Each capsule's radius as a fraction of its bone's length. */
            Scalar radius_fraction = Scalar(0.22);

            /** @brief A floor under the radius, so a short bone is not a disc. */
            Scalar minimum_radius = Scalar(0.02);

            /** @brief How far a limb may bend off its parent, in radians. */
            Scalar swing_limit = Scalar(0.6);

            /** @brief How far a limb may twist about its own bone, in radians, either way. */
            Scalar twist_limit = Scalar(0.35);

            /** @brief Compliance of the structural attachment; zero is a rigid socket. */
            Scalar joint_compliance = 0;

            /** @brief Compliance of the limits; a positive value is a soft end-stop. */
            Scalar limit_compliance = 0;

            /** @brief Quadratic drag on every part; zero disables. */
            Scalar drag_coefficient = 0;

            /**
             * @brief The collision-filter group every part shares.
             *
             * One group for the whole ragdoll, whose mask excludes itself, so the parts
             * do not push each other. That is coarser than it could be — a hand *should*
             * be able to rest on a thigh — and refining it to "adjacent parts only"
             * needs the per-pair matrix the assembly format already carries. Coarse and
             * stated beats fine and wrong: self-collision between parts a joint is
             * holding together is what makes a ragdoll explode.
             */
            std::uint32_t group = 2;

            /**
             * @brief Shortest bone worth a capsule, in metres.
             *
             * A zero-length bone has no direction, so its capsule would have no
             * orientation; below this a joint is treated as a leaf.
             */
            Scalar minimum_bone_length = Scalar(1e-4);
        };

        /**
         * @brief What ties one simulated part to one skeleton joint.
         *
         * The offset is the joint's pose *in the part's local frame*, measured at bind.
         * It is the whole reason a target can be resolved without knowing how the
         * capsule was oriented.
         */
        struct RagdollBinding
        {
            /** @brief Index into @ref RagdollRig::assembly's parts. */
            std::uint32_t part = 0;

            /** @brief Index into the skeleton's cooked joint order. */
            std::uint32_t joint = 0;

            /** @brief The joint's position in the part's local frame, at bind. */
            Vector3 offset_position;

            /** @brief The joint's orientation in the part's local frame, at bind. */
            Quaternion offset_orientation;
        };

        /** @brief A skeleton, as a simulable assembly plus the map back to its joints. */
        struct RagdollRig
        {
            /** @brief The parts, joints and filter group; instance it the ordinary way. */
            PhysicsAssembly assembly;

            /** @brief One per part, in part order. */
            std::vector<RagdollBinding> bindings;

            /** @brief Per skeleton joint, its part index or @ref NO_RAGDOLL_PART. */
            std::vector<std::uint32_t> part_of_joint;
        };

        namespace detail
        {
            /**
             * @brief The shortest rotation taking local +Y onto @p direction.
             *
             * +Y because that is the axis a `CapsuleCollider`'s segment runs along, so
             * this is what orients a bone's capsule. Shortest rather than arbitrary so
             * the perpendicular reference is a deterministic function of the bone, which
             * matters because the joint frames are derived the same way on both sides.
             *
             * @param direction A unit direction.
             */
            inline Quaternion rotation_onto_y(const Vector3& direction) noexcept
            {
                const Vector3 from{0, 1, 0};
                const Scalar d = dot(from, direction);
                // Antiparallel: the cross product vanishes and every perpendicular axis
                // is as short as any other, so one is chosen rather than normalizing a
                // zero and producing a direction out of nothing.
                if (d < Scalar(-1) + Scalar(1e-9))
                    return Quaternion{0, 0, Scalar(1), 0};
                const Vector3 c = cross(from, direction);
                return normalize(Quaternion{c.x, c.y, c.z, Scalar(1) + d});
            }

            /** @brief The volume of a capsule, from the mass its own formula gives at unit density. */
            inline Scalar capsule_volume(Scalar radius, Scalar half_height) noexcept
            {
                return Physics::capsule_mass_properties<Scalar>(radius, half_height, Scalar(1))
                    .mass;
            }
        } // namespace detail

        /**
         * @brief Builds a ragdoll from a cooked skeleton's bind pose.
         *
         * The bind pose is forward-composed from the skeleton's own local bind
         * transforms rather than read out of the inverse-bind matrices, and the two
         * agree by construction for a well-formed rig — but the local transforms are
         * always present while the matrices are supplied only by importers that have
         * them, so composing is the path that works for every skeleton.
         *
         * Mass is spread by **volume at a single uniform density**, not by bone length.
         * That is the physical model rather than an approximation of one, and it is why
         * no per-part mass is authored: a thick torso comes out heavier than a thin
         * forearm because it is bigger, which is the answer a length-proportional split
         * only approximates and gets wrong for exactly the bones that matter most.
         *
         * @param skeleton The cooked skeleton.
         * @param profile  How to turn it into a rig.
         * @return The rig; its assembly is empty when @p skeleton has no bone long
         *         enough to be a capsule.
         */
        inline RagdollRig build_ragdoll_rig(const Animation::SkeletonView& skeleton,
                                            const RagdollProfile& profile)
        {
            RagdollRig rig;
            const std::uint32_t count = skeleton.joint_count;
            if (count == 0 || skeleton.parents == nullptr ||
                skeleton.bind_translations == nullptr || skeleton.bind_rotations == nullptr)
                return rig;

            // The bind pose in object space. The parents are topologically sorted
            // (`parents[i] < i`), so one forward scan composes every joint.
            std::vector<Vector3> bind_position(count);
            std::vector<Quaternion> bind_rotation(count);
            for (std::uint32_t i = 0; i < count; ++i)
            {
                const Animation::Vector3f& t = skeleton.bind_translations[i];
                const Animation::Quaternionf& r = skeleton.bind_rotations[i];
                const Vector3 local_t{Scalar(t.x), Scalar(t.y), Scalar(t.z)};
                const Quaternion local_r{Scalar(r.x), Scalar(r.y), Scalar(r.z), Scalar(r.w)};

                const std::uint16_t parent = skeleton.parents[i];
                if (parent == Animation::NO_PARENT)
                {
                    bind_position[i] = local_t;
                    bind_rotation[i] = local_r;
                    continue;
                }
                bind_position[i] =
                    bind_position[parent] + rotate(bind_rotation[parent], local_t);
                bind_rotation[i] = mul(bind_rotation[parent], local_r);
            }

            // Where each joint's children sit, so a bone can be the segment from a joint
            // toward the mean of them. The mean rather than the first child because a
            // hip or a chest has several and picking one would aim the torso's capsule
            // down a leg.
            std::vector<Vector3> child_sum(count);
            std::vector<std::uint32_t> child_count(count, 0);
            for (std::uint32_t i = 0; i < count; ++i)
            {
                const std::uint16_t parent = skeleton.parents[i];
                if (parent == Animation::NO_PARENT)
                    continue;
                child_sum[parent] = child_sum[parent] + bind_position[i];
                ++child_count[parent];
            }

            rig.part_of_joint.assign(count, NO_RAGDOLL_PART);
            std::vector<Vector3> bone_direction;

            for (std::uint32_t i = 0; i < count; ++i)
            {
                if (child_count[i] == 0)
                    continue;
                const Vector3 child_mean =
                    child_sum[i] * (Scalar(1) / Scalar(child_count[i]));
                const Vector3 bone = child_mean - bind_position[i];
                const Scalar bone_length = length(bone);
                if (!(bone_length > profile.minimum_bone_length))
                    continue;
                const Vector3 direction = bone * (Scalar(1) / bone_length);

                const Scalar radius =
                    std::max(profile.minimum_radius, profile.radius_fraction * bone_length);
                // The segment excludes the caps, so a capsule spanning the bone has a
                // half-segment of half the bone minus the radius — and a bone shorter
                // than its own diameter degenerates to a sphere rather than turning
                // inside out.
                const Scalar half_height =
                    std::max(Scalar(0), bone_length * Scalar(0.5) - radius);

                AssemblyPart part;
                part.collider.shape = ColliderShape::Capsule;
                part.collider.radius = radius;
                part.collider.half_height = half_height;
                part.local_position = bind_position[i] + direction * (bone_length * Scalar(0.5));
                part.local_orientation = detail::rotation_onto_y(direction);
                part.drag_coefficient = profile.drag_coefficient;
                part.group = profile.group;
                part.name_hash =
                    skeleton.joint_names != nullptr ? skeleton.joint_names[i] : 0;

                RagdollBinding binding;
                binding.part = std::uint32_t(rig.assembly.parts.size());
                binding.joint = i;
                const Quaternion inverse_part = conjugate(part.local_orientation);
                binding.offset_position =
                    rotate(inverse_part, bind_position[i] - part.local_position);
                binding.offset_orientation = mul(inverse_part, bind_rotation[i]);

                rig.part_of_joint[i] = binding.part;
                rig.assembly.parts.push_back(part);
                const char* joint_name = skeleton.joint_name(i);
                rig.assembly.part_names.push_back(
                    joint_name == nullptr ? std::string() : std::string(joint_name));
                rig.bindings.push_back(binding);
                bone_direction.push_back(direction);
            }

            if (rig.assembly.parts.empty())
                return rig;

            // One density for the whole body, chosen so the parts add up to the authored
            // mass. Derived here rather than left to the author because a per-part mass
            // is the thing an author cannot get right by hand and does not want to.
            Scalar volume = 0;
            for (const AssemblyPart& part : rig.assembly.parts)
                volume += detail::capsule_volume(part.collider.radius, part.collider.half_height);
            const Scalar density =
                volume > Scalar(0) && profile.total_mass > Scalar(0)
                    ? profile.total_mass / volume
                    : Scalar(0);
            for (AssemblyPart& part : rig.assembly.parts)
                part.density = density;

            // The group matrix: one group, not colliding with itself.
            rig.assembly.group_masks.assign(std::size_t(profile.group) + 1, 0xFFFFFFFFu);
            rig.assembly.group_masks[profile.group] =
                assembly_group_excluding_self(profile.group);

            // A cone-twist at every joint that has a part, tying it to the nearest
            // ancestor that has one. Nearest *ancestor with a part* rather than the
            // direct parent, because a leaf-adjacent chain can skip joints whose bones
            // were too short to be capsules, and a joint whose parent has no part would
            // otherwise be attached to nothing.
            for (std::uint32_t i = 0; i < count; ++i)
            {
                const std::uint32_t child_part = rig.part_of_joint[i];
                if (child_part == NO_RAGDOLL_PART)
                    continue;

                std::uint32_t ancestor = Animation::NO_PARENT;
                for (std::uint16_t cursor = skeleton.parents[i];
                     cursor != Animation::NO_PARENT; cursor = skeleton.parents[cursor])
                {
                    if (rig.part_of_joint[cursor] != NO_RAGDOLL_PART)
                    {
                        ancestor = rig.part_of_joint[cursor];
                        break;
                    }
                }
                if (ancestor == Animation::NO_PARENT)
                    continue; // the root part: nothing above it to hang from

                const AssemblyPart& parent_part = rig.assembly.parts[ancestor];
                const AssemblyPart& part = rig.assembly.parts[child_part];
                const Vector3 axis = bone_direction[child_part];

                const Quaternion inverse_parent = conjugate(parent_part.local_orientation);
                const Quaternion inverse_child = conjugate(part.local_orientation);

                AssemblyJoint joint;
                joint.part_a = ancestor;
                joint.part_b = child_part;
                joint.params.type = JointType::ConeTwist;
                joint.params.anchor_a =
                    rotate(inverse_parent, bind_position[i] - parent_part.local_position);
                joint.params.anchor_b =
                    rotate(inverse_child, bind_position[i] - part.local_position);
                joint.params.axis_a = rotate(inverse_parent, axis);
                joint.params.axis_b = rotate(inverse_child, axis);
                joint.params.compliance = profile.joint_compliance;

                joint.params.swing_limit.enabled = true;
                joint.params.swing_limit.upper = profile.swing_limit;
                joint.params.swing_limit.compliance = profile.limit_compliance;

                // The twist range is centred on the twist the *bind pose* already holds,
                // not on zero, and that correction is load-bearing. Both frames are
                // derived from the same world axis by the shortest rotation onto it, but
                // in each part's own local space — so they generally differ by a rotation
                // about that axis, which is exactly a non-zero twist at bind. A range of
                // [-t, +t] would then be a range centred somewhere the rig has never
                // been, and the limit would fight the bind pose from the first substep.
                const Quaternion basis_a = mul(parent_part.local_orientation,
                                               Physics::joint_frame_from_axis<Scalar>(
                                                   joint.params.axis_a));
                const Quaternion basis_b = mul(part.local_orientation,
                                               Physics::joint_frame_from_axis<Scalar>(
                                                   joint.params.axis_b));
                const Scalar bind_twist = Physics::joint_twist_angle<Scalar>(
                    normalize(mul(conjugate(basis_a), basis_b)));

                joint.params.twist_limit.enabled = true;
                joint.params.twist_limit.lower = bind_twist - profile.twist_limit;
                joint.params.twist_limit.upper = bind_twist + profile.twist_limit;
                joint.params.twist_limit.compliance = profile.limit_compliance;

                rig.assembly.joints.push_back(joint);
            }
            return rig;
        }

        /**
         * @brief Reads the solved parts back as pose targets for `Animation::RagdollBlend`.
         *
         * The targets are in the character's **object space**, which is what the
         * modifier blends in, so the world poses the physics reports are pulled back
         * through the character's own transform. Every part's target is
         * `object_from_world * part_world * bind_offset` — the offset being what makes
         * the capsule's orientation convention invisible here.
         *
         * @param rig              The rig the parts were built from.
         * @param bodies           Where to read the solved poses from.
         * @param part_entities    One entity per part, in part order — the same list
         *                         @ref instantiate_assembly was given.
         * @param part_entity_count How many @p part_entities holds.
         * @param world_from_object The character's object-to-world transform.
         * @param weight           Per-joint blend weight: 0 is pure animation, 1 pure
         *                         physics. One value for every joint, because ramping it
         *                         per joint is the caller's decision and this is a pure
         *                         function of what it is given — the same contract
         *                         `RagdollBlend` itself keeps.
         * @param out              Receives one target per resolvable part; cleared first.
         * @return How many targets were written.
         */
        inline std::size_t resolve_ragdoll_targets(
            const RagdollRig& rig, const IRigidBodyService& bodies,
            const EntityId* part_entities, std::size_t part_entity_count,
            const Mat4& world_from_object, Scalar weight,
            std::vector<Animation::RagdollJointTarget>& out)
        {
            out.clear();
            if (part_entities == nullptr || part_entity_count < rig.bindings.size())
                return 0;

            const Mat4 object_from_world = affine_inverse(world_from_object);
            out.reserve(rig.bindings.size());

            for (const RagdollBinding& binding : rig.bindings)
            {
                if (binding.part >= part_entity_count)
                    continue;
                SolvedPose pose;
                // A part with no body is skipped rather than reported at its bind pose:
                // a target the physics did not produce is not a physics target, and
                // blending toward one would drag the animation to the rig's rest pose.
                if (!bodies.rigid_pose(part_entities[binding.part], pose))
                    continue;

                const Mat4 part_world =
                    compose_transform(pose.position, pose.orientation, Vector3{1, 1, 1});
                const Mat4 joint_in_part = compose_transform(
                    binding.offset_position, binding.offset_orientation, Vector3{1, 1, 1});

                Animation::RagdollJointTarget target;
                target.joint = binding.joint;
                target.object_space_transform =
                    mul(object_from_world, mul(part_world, joint_in_part));
                target.weight = static_cast<float>(weight);
                out.push_back(target);
            }
            return out.size();
        }
    } // namespace Simulation
} // namespace SushiEngine
