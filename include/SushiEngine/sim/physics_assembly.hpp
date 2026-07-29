/**************************************************************************/
/* physics_assembly.hpp                                                   */
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
 * @file physics_assembly.hpp
 * @brief The `PhysicsAssembly` asset (§5.4) and what instancing one means (§10.2).
 *
 * An authored bag of **parts**, **joints** and **collision-filter groups**, instanced
 * as one unit: the car, the ragdoll, the mechanism. Versioned into a `.sushiassembly`
 * blob on `Animation::SkeletonBlob`'s shape — a fixed header, offset-based sections,
 * no pointers, memory-mappable, loadable with one read.
 *
 * ### Why this lives at the `sim/` boundary and not under `physics/`
 *
 * Because its parts are described by @ref Collider, which is already the boundary
 * record: authored, scaled, carrying the cooked-asset identifier P4 will fill, and
 * already what the extract hands the physics. An assembly expressed in physics-layer
 * shape types would be a second description of the same thing, and §5.5's whole
 * argument against `ColliderParams`'s flattened copies applies with equal force here.
 *
 * ### Instancing is a translation, not a service that owns bodies
 *
 * §4.3 sketched an `IAssemblyService { instantiate(PhysicsAssembly)/release/part_body }`.
 * That shape does not survive contact with how the world actually runs, and the
 * correction is worth stating rather than quietly making.
 *
 * `IRigidBodyService::set_rigid_bodies` is a **diff driven by the ECS every tick**: it
 * removes any body whose entity is not in the list it was handed. A body the physics
 * created behind the ECS's back would therefore be destroyed by the very next call —
 * so an assembly service that owned its parts would be an assembly whose parts
 * vanished a frame after instancing.
 *
 * §10.2 already says the right thing: *"One entity carries the `AssemblyInstance`;
 * child entities carry the parts."* The parts **are** entities, so the ECS owns them
 * and they flow through `set_rigid_bodies` like everything else. What is left for this
 * side is exactly the shape §4.1 blessed for `PhysicsExtract` — a translation
 * responsibility in one unit, tested on its own: @ref instantiate_assembly turns an
 * asset plus a root pose plus one caller-supplied entity per part into the
 * @ref RigidBodyDesc list and @ref JointDesc list the caller already knows how to
 * feed. Nothing here holds state, so there is nothing to release.
 *
 * ### The filter matrix is authoritative
 *
 * Each part names a **group**, and the asset carries one mask per group saying which
 * groups it collides with. Instancing resolves that into each part's
 * `Collider::filter`, overwriting whatever the authored collider carried. That is the
 * point: §10.2's "part 0 and part 1 do not collide with each other" is a statement
 * about the *assembly*, and a per-part filter that could disagree with the matrix
 * would be a second place the same question is answered.
 */

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/sim/collider.hpp>
#include <SushiEngine/sim/physics_services.hpp>
#include <SushiEngine/sim/simulation.hpp>

namespace SushiEngine
{
    namespace Simulation
    {
        /** @brief Magic tag at the head of every `.sushiassembly` blob. */
        constexpr char ASSEMBLY_BLOB_MAGIC[8] = {'S', 'U', 'S', 'H', 'A', 'S', 'S', 'Y'};

        /** @brief Current `.sushiassembly` format version. */
        constexpr std::uint32_t ASSEMBLY_BLOB_VERSION = 1;

        /** @brief A stable identity for a part, independent of its index. */
        using AssemblyNameHash = std::uint64_t;

        /**
         * @brief FNV-1a 64 of @p name, the identity a part is addressed by.
         *
         * A hash rather than a string because the runtime never displays a part's name
         * and a POD blob cannot hold a `std::string`. The readable names travel in
         * their own section for the editor to show, exactly as a skeleton's do.
         *
         * @param name A null-terminated name; null yields zero.
         */
        inline AssemblyNameHash assembly_name_hash(const char* name) noexcept
        {
            if (name == nullptr)
                return 0;
            AssemblyNameHash hash = 1469598103934665603ull;
            for (const char* cursor = name; *cursor != '\0'; ++cursor)
            {
                hash ^= AssemblyNameHash(static_cast<unsigned char>(*cursor));
                hash *= 1099511628211ull;
            }
            return hash;
        }

        /**
         * @brief One rigid part of an assembly: what it collides as, where, and what it weighs.
         *
         * Trivially copyable, so a blob section is a `memcpy`.
         */
        struct AssemblyPart
        {
            /** @brief What this part collides as; its `filter` is overwritten at instancing. */
            Collider collider;

            /** @brief Where the part sits relative to the assembly's root, at rest. */
            Vector3 local_position;

            /** @brief How the part is oriented relative to the assembly's root, at rest. */
            Quaternion local_orientation;

            /**
             * @brief Mass per unit volume; above zero derives mass and inertia from the shape.
             *
             * Zero keeps @ref inv_mass and @ref inv_inertia exactly as authored, which is
             * the same rule the extract follows for an ordinary body (§16.4): deriving
             * mass from a shape the author did not intend to weigh is worse than a
             * hand-typed number, because it *looks* derived.
             */
            Scalar density = 0;

            /** @brief Inverse mass when @ref density is zero; zero pins the part. */
            Scalar inv_mass = Scalar(1);

            /** @brief Diagonal body-local inverse inertia when @ref density is zero. */
            Vector3 inv_inertia;

            /** @brief Quadratic drag: acceleration `-k|v|v`; zero disables. */
            Scalar drag_coefficient = 0;

            /** @brief Which collision-filter group this part belongs to. */
            std::uint32_t group = 0;

            /** @brief Index into the scene's material table. */
            std::uint32_t material_index = 0;

            /** @brief @ref assembly_name_hash of the part's authored name. */
            AssemblyNameHash name_hash = 0;
        };

        /**
         * @brief One joint of an assembly, between two of its parts.
         *
         * The parameters are a @ref JointParams rather than a copy of its fields, which
         * is why that value exists: an assembly authored against part indices and a
         * joint created against entities are the same joint, and a parameter added to
         * one must not be silently absent from the other.
         */
        struct AssemblyJoint
        {
            /** @brief Index into @ref PhysicsAssembly::parts. */
            std::uint32_t part_a = 0;

            /** @brief Index into @ref PhysicsAssembly::parts. */
            std::uint32_t part_b = 0;

            /** @brief What is held between them. */
            JointParams params;
        };

        /**
         * @brief An assembly as authored: the input to @ref build_assembly_blob.
         *
         * The owning form. @ref PhysicsAssemblyView is the borrowed one a loaded blob
         * hands out, and both are accepted wherever an assembly is read.
         */
        struct PhysicsAssembly
        {
            std::vector<AssemblyPart> parts;
            std::vector<AssemblyJoint> joints;

            /**
             * @brief Per group, the mask of groups it collides with.
             *
             * Bit *g* set means "this group collides with group *g*". A group whose own
             * bit is clear does not collide with itself, which is how an assembly stops
             * its own parts fighting the joints that hold them — one line of authoring
             * instead of a rule every routine touching a part has to know.
             *
             * Its length is the group count. A part naming a group past the end gets
             * the permissive default, because a filter that silently dropped every
             * contact would be much harder to notice than one that dropped none.
             */
            std::vector<std::uint32_t> group_masks;

            /** @brief Readable part names, parallel to @ref parts; for the editor only. */
            std::vector<std::string> part_names;
        };

        /**
         * @brief A loaded assembly, borrowed from blob bytes.
         *
         * Pointers into the caller's byte array, rebuilt from the header's offsets, so
         * the blob can be memory-mapped and never copied.
         */
        struct PhysicsAssemblyView
        {
            std::uint32_t part_count = 0;
            std::uint32_t joint_count = 0;
            std::uint32_t group_count = 0;
            const AssemblyPart* parts = nullptr;
            const AssemblyJoint* joints = nullptr;
            const std::uint32_t* group_masks = nullptr;

            /** @brief Byte offset into @ref name_data of each part's name. */
            const std::uint32_t* name_offsets = nullptr;

            /** @brief Concatenated null-terminated part names; debug data. */
            const char* name_data = nullptr;

            /** @brief Whether this view was loaded from a well-formed blob. */
            bool valid() const noexcept { return parts != nullptr; }
        };

        /** @brief The borrowed view of an owning assembly, so both are read the same way. */
        inline PhysicsAssemblyView to_view(const PhysicsAssembly& assembly) noexcept
        {
            PhysicsAssemblyView view;
            view.part_count = std::uint32_t(assembly.parts.size());
            view.joint_count = std::uint32_t(assembly.joints.size());
            view.group_count = std::uint32_t(assembly.group_masks.size());
            view.parts = assembly.parts.data();
            view.joints = assembly.joints.data();
            view.group_masks = assembly.group_masks.data();
            return view;
        }

        /**
         * @brief The mask of groups @p group collides with, or a permissive default.
         *
         * @param view  The assembly.
         * @param group The group to resolve.
         */
        inline std::uint32_t assembly_group_mask(const PhysicsAssemblyView& view,
                                                 std::uint32_t group) noexcept
        {
            if (view.group_masks == nullptr || group >= view.group_count)
                return 0xFFFFFFFFu;
            return view.group_masks[group];
        }

        /**
         * @brief The fixed header at offset 0 of an assembly blob.
         *
         * Fixed-width fields and byte offsets from the blob's start, so the blob is
         * position independent and the loader rebuilds every pointer from these.
         */
        struct AssemblyBlobHeader
        {
            char magic[8];                      /**< @ref ASSEMBLY_BLOB_MAGIC. */
            std::uint32_t version;              /**< @ref ASSEMBLY_BLOB_VERSION. */
            std::uint32_t part_count;
            std::uint32_t joint_count;
            std::uint32_t group_count;
            std::uint32_t total_size;           /**< Whole blob size in bytes. */
            std::uint32_t parts_offset;         /**< AssemblyPart[part_count]. */
            std::uint32_t joints_offset;        /**< AssemblyJoint[joint_count]. */
            std::uint32_t group_masks_offset;   /**< uint32[group_count]. */
            std::uint32_t name_offsets_offset;  /**< uint32[part_count]. */
            std::uint32_t name_data_offset;     /**< Concatenated null-terminated names. */
            std::uint32_t name_data_size;       /**< Bytes in the name string section. */
        };

        namespace detail
        {
            /** @brief Rounds @p value up to the next multiple of @p alignment. */
            inline std::size_t assembly_align_up(std::size_t value,
                                                 std::size_t alignment) noexcept
            {
                return (value + alignment - 1) & ~(alignment - 1);
            }
        } // namespace detail

        /**
         * @brief Serializes @p assembly into a `.sushiassembly` blob.
         *
         * Refuses rather than writes a blob it would not itself load: a joint naming a
         * part that does not exist is an authoring error whose symptom, unchecked, is a
         * joint silently projected against part 0. A blob whose *validation* is weaker
         * than its writer's is a format that cannot be trusted at the other end.
         *
         * @param assembly The assembly to write.
         * @param out      Receives the blob bytes; cleared first.
         * @return False when @p assembly is not well formed, leaving @p out empty.
         */
        inline bool build_assembly_blob(const PhysicsAssembly& assembly,
                                        std::vector<std::byte>& out)
        {
            out.clear();
            if (assembly.parts.empty())
                return false;
            for (const AssemblyJoint& joint : assembly.joints)
            {
                if (joint.part_a >= assembly.parts.size() ||
                    joint.part_b >= assembly.parts.size())
                    return false;
                // A joint from a part to itself is not a degenerate joint, it is a
                // constraint the solver would colour against one body twice.
                if (joint.part_a == joint.part_b)
                    return false;
            }

            const std::size_t parts = assembly.parts.size();
            const std::size_t joints = assembly.joints.size();
            const std::size_t groups = assembly.group_masks.size();

            std::vector<std::uint32_t> name_offsets(parts, 0);
            std::string names;
            for (std::size_t i = 0; i < parts; ++i)
            {
                name_offsets[i] = std::uint32_t(names.size());
                if (i < assembly.part_names.size())
                    names += assembly.part_names[i];
                names.push_back('\0');
            }

            std::size_t cursor = detail::assembly_align_up(sizeof(AssemblyBlobHeader), 16);
            const std::size_t parts_offset = cursor;
            cursor = detail::assembly_align_up(cursor + parts * sizeof(AssemblyPart), 16);
            const std::size_t joints_offset = cursor;
            cursor = detail::assembly_align_up(cursor + joints * sizeof(AssemblyJoint), 16);
            const std::size_t group_masks_offset = cursor;
            cursor = detail::assembly_align_up(cursor + groups * sizeof(std::uint32_t), 16);
            const std::size_t name_offsets_offset = cursor;
            cursor = detail::assembly_align_up(cursor + parts * sizeof(std::uint32_t), 16);
            const std::size_t name_data_offset = cursor;
            const std::size_t total = cursor + names.size();

            out.assign(total, std::byte{0});
            std::byte* base = out.data();

            AssemblyBlobHeader header{};
            std::memcpy(header.magic, ASSEMBLY_BLOB_MAGIC, sizeof(header.magic));
            header.version = ASSEMBLY_BLOB_VERSION;
            header.part_count = std::uint32_t(parts);
            header.joint_count = std::uint32_t(joints);
            header.group_count = std::uint32_t(groups);
            header.total_size = std::uint32_t(total);
            header.parts_offset = std::uint32_t(parts_offset);
            header.joints_offset = std::uint32_t(joints_offset);
            header.group_masks_offset = std::uint32_t(group_masks_offset);
            header.name_offsets_offset = std::uint32_t(name_offsets_offset);
            header.name_data_offset = std::uint32_t(name_data_offset);
            header.name_data_size = std::uint32_t(names.size());
            std::memcpy(base, &header, sizeof(header));

            if (parts > 0)
                std::memcpy(base + parts_offset, assembly.parts.data(),
                            parts * sizeof(AssemblyPart));
            if (joints > 0)
                std::memcpy(base + joints_offset, assembly.joints.data(),
                            joints * sizeof(AssemblyJoint));
            if (groups > 0)
                std::memcpy(base + group_masks_offset, assembly.group_masks.data(),
                            groups * sizeof(std::uint32_t));
            if (parts > 0)
                std::memcpy(base + name_offsets_offset, name_offsets.data(),
                            parts * sizeof(std::uint32_t));
            if (!names.empty())
                std::memcpy(base + name_data_offset, names.data(), names.size());
            return true;
        }

        /**
         * @brief Whether @p data is a blob this build can load.
         *
         * Every section is checked to lie inside the blob, because a truncated or
         * hostile asset must fail here rather than produce a view whose pointers walk
         * off the end. Checked against the *declared* total size and the *actual* one,
         * since a header that lies about its length is the interesting case.
         */
        inline bool validate_assembly_blob(const std::byte* data, std::size_t size) noexcept
        {
            if (data == nullptr || size < sizeof(AssemblyBlobHeader))
                return false;

            AssemblyBlobHeader header{};
            std::memcpy(&header, data, sizeof(header));
            if (std::memcmp(header.magic, ASSEMBLY_BLOB_MAGIC, sizeof(header.magic)) != 0)
                return false;
            if (header.version != ASSEMBLY_BLOB_VERSION)
                return false;
            if (header.total_size > size || header.part_count == 0)
                return false;

            const auto section_fits = [&header](std::uint32_t offset, std::size_t bytes) noexcept
            {
                if (bytes == 0)
                    return true;
                if (offset > header.total_size)
                    return false;
                return std::size_t(header.total_size) - offset >= bytes;
            };

            if (!section_fits(header.parts_offset,
                              std::size_t(header.part_count) * sizeof(AssemblyPart)))
                return false;
            if (!section_fits(header.joints_offset,
                              std::size_t(header.joint_count) * sizeof(AssemblyJoint)))
                return false;
            if (!section_fits(header.group_masks_offset,
                              std::size_t(header.group_count) * sizeof(std::uint32_t)))
                return false;
            if (!section_fits(header.name_offsets_offset,
                              std::size_t(header.part_count) * sizeof(std::uint32_t)))
                return false;
            if (!section_fits(header.name_data_offset, header.name_data_size))
                return false;

            // The joints are validated here and not only at write time, because a blob
            // may have been produced by an older writer or edited by hand, and a joint
            // naming a part that does not exist is projected against part 0 rather than
            // rejected.
            const AssemblyJoint* joints = reinterpret_cast<const AssemblyJoint*>(
                data + header.joints_offset);
            for (std::uint32_t i = 0; i < header.joint_count; ++i)
            {
                if (joints[i].part_a >= header.part_count ||
                    joints[i].part_b >= header.part_count)
                    return false;
                if (joints[i].part_a == joints[i].part_b)
                    return false;
            }
            return true;
        }

        /**
         * @brief Rebuilds a view over a validated blob.
         *
         * @param data The blob bytes.
         * @param size Their length.
         * @return A view, or a default (invalid) one when the blob does not validate.
         */
        inline PhysicsAssemblyView load_assembly_blob(const std::byte* data,
                                                      std::size_t size) noexcept
        {
            PhysicsAssemblyView view;
            if (!validate_assembly_blob(data, size))
                return view;

            AssemblyBlobHeader header{};
            std::memcpy(&header, data, sizeof(header));
            view.part_count = header.part_count;
            view.joint_count = header.joint_count;
            view.group_count = header.group_count;
            view.parts = reinterpret_cast<const AssemblyPart*>(data + header.parts_offset);
            view.joints = header.joint_count == 0
                              ? nullptr
                              : reinterpret_cast<const AssemblyJoint*>(data +
                                                                       header.joints_offset);
            view.group_masks =
                header.group_count == 0
                    ? nullptr
                    : reinterpret_cast<const std::uint32_t*>(data + header.group_masks_offset);
            view.name_offsets =
                reinterpret_cast<const std::uint32_t*>(data + header.name_offsets_offset);
            view.name_data = header.name_data_size == 0
                                 ? nullptr
                                 : reinterpret_cast<const char*>(data + header.name_data_offset);
            return view;
        }

        /** @brief Part @p index's readable name, or an empty string when there is none. */
        inline const char* assembly_part_name(const PhysicsAssemblyView& view,
                                              std::uint32_t index) noexcept
        {
            if (view.name_data == nullptr || view.name_offsets == nullptr ||
                index >= view.part_count)
                return "";
            return view.name_data + view.name_offsets[index];
        }

        /**
         * @brief What instancing an assembly produces: bodies to add, joints to create.
         *
         * Two lists and no state. The caller owns the entities, so it owns the
         * lifetime; releasing an assembly is destroying its entities, which the ECS
         * already knows how to do.
         */
        struct AssemblyInstantiation
        {
            /** @brief One per part, in part order, ready for `set_rigid_bodies`. */
            std::vector<RigidBodyDesc> bodies;

            /** @brief One per joint, in joint order, ready for `create_joint`. */
            std::vector<JointDesc> joints;
        };

        /**
         * @brief Places an assembly at a root pose and says what to create.
         *
         * Each part's world pose is its local pose composed with the root's, so an
         * assembly instanced twice is the same assembly in two places. Mass and inertia
         * are derived from the *scaled* shape when the part carries a density and left
         * exactly as authored when it does not.
         *
         * The joints come back naming the entities their parts became, which is the one
         * thing the asset could not know: an assembly is authored against part indices
         * and only learns the entities at this moment.
         *
         * @param view             The assembly to instance.
         * @param part_entities    One entity per part, in part order; the caller created them.
         * @param part_entity_count How many @p part_entities holds.
         * @param root_position    Where the assembly's root sits in the world.
         * @param root_orientation How the root is oriented.
         * @return The bodies and joints to create. Empty when @p part_entities is
         *         shorter than the part count — a partly instanced assembly is worse
         *         than none, because its joints would name entities that do not exist.
         */
        inline AssemblyInstantiation instantiate_assembly(
            const PhysicsAssemblyView& view, const EntityId* part_entities,
            std::size_t part_entity_count, const Vector3& root_position,
            const Quaternion& root_orientation)
        {
            AssemblyInstantiation out;
            if (!view.valid() || part_entities == nullptr ||
                part_entity_count < view.part_count)
                return out;

            out.bodies.reserve(view.part_count);
            for (std::uint32_t i = 0; i < view.part_count; ++i)
            {
                const AssemblyPart& part = view.parts[i];

                RigidBodyDesc desc;
                desc.id = part_entities[i];
                desc.position = root_position + rotate(root_orientation, part.local_position);
                desc.orientation = mul(root_orientation, part.local_orientation);
                desc.inv_mass = part.inv_mass;
                desc.inv_inertia = part.inv_inertia;
                desc.drag_coefficient = part.drag_coefficient;
                desc.collider = part.collider;

                // The matrix wins over whatever the authored collider carried; see this
                // file's header for why there is only one place this is decided.
                desc.collider.filter.layer = std::uint32_t(1) << (part.group & 31u);
                desc.collider.filter.collides_with = assembly_group_mask(view, part.group);

                const Physics::MassProperties<Scalar> mass =
                    collider_mass_properties(desc.collider, part.density);
                if (mass.mass > Scalar(0))
                {
                    desc.inv_mass = Physics::inverse_mass(mass.mass);
                    desc.inv_inertia = Physics::to_inverse(mass.inertia);
                }
                out.bodies.push_back(desc);
            }

            out.joints.reserve(view.joint_count);
            for (std::uint32_t i = 0; i < view.joint_count; ++i)
            {
                const AssemblyJoint& joint = view.joints[i];
                // Bounds already hold: `validate_assembly_blob` refuses a blob whose
                // joints name parts that do not exist, and `to_view` comes from an
                // owning assembly `build_assembly_blob` refuses to write otherwise.
                JointDesc desc;
                desc.body_a = part_entities[joint.part_a];
                desc.body_b = part_entities[joint.part_b];
                desc.params = joint.params;
                out.joints.push_back(desc);
            }
            return out;
        }

        /**
         * @brief The group mask that makes a group collide with everything but itself.
         *
         * The assembly default, and worth a name because it is what §10.2's filter line
         * actually means: parts of one assembly do not push each other, so the joints
         * holding them are not fighting contacts.
         *
         * @param group The group.
         */
        inline std::uint32_t assembly_group_excluding_self(std::uint32_t group) noexcept
        {
            return ~(std::uint32_t(1) << (group & 31u));
        }
    } // namespace Simulation
} // namespace SushiEngine
