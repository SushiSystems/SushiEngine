/**************************************************************************/
/* skeleton.hpp                                                           */
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
 * @file skeleton.hpp
 * @brief The immutable skeleton asset, seen as a flat SoA view over a cooked blob.
 *
 * A skeleton is a topologically sorted joint hierarchy: `parent[i] < i` for every
 * joint, guaranteed at import, so composing model-space poses is a single forward scan
 * and never a pointer chase (this is what kills the host-side, non-topological
 * hierarchy the engine had before animation). The data is structure-of-arrays and
 * relocatable — the loader hands out a @ref SkeletonView of raw pointers into a byte
 * buffer it does not own, exactly the shape the batched evaluator and the GPU palette
 * build want. Nothing here allocates or parses a file; @ref skeleton_blob.hpp does the
 * cook and load, and @ref animation_database.hpp owns the bytes.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/core/types.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief 3-vector in evaluation precision (float): pose translations and scales. */
        using Vector3f = Vector3T<float>;

        /** @brief Unit quaternion in evaluation precision (float): pose rotations. */
        using Quaternionf = QuaternionT<float>;

        /**
         * @brief A column-major 4x4 matrix of floats — the evaluation/render pose matrix.
         *
         * Object space, GLSL's native layout (element at row @c r, column @c c is
         * @c m[c * 4 + r]), so it uploads to a joint palette storage buffer with no
         * repack. Distinct from the engine's double-precision @c Mat4, which is the
         * boundary/camera precision; joint data never needs the range that forces double,
         * because palettes are object space (the camera-relative offset stays in the
         * per-instance model matrix — see the design's §5.2).
         */
        struct JointMatrix
        {
            float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
        };

        /** @brief Maximum joints in one skeleton; a compile error is raised above this. */
        constexpr std::uint32_t MAX_JOINTS = 256;

        /** @brief Maximum bone-LOD levels a skeleton's LOD ladder may carry. */
        constexpr std::uint32_t MAX_LOD_LEVELS = 4;

        /** @brief Parent index of a root joint — a joint with no parent. */
        constexpr std::uint16_t NO_PARENT = 0xFFFFu;

        /**
         * @brief A non-owning, immutable view of a cooked skeleton.
         *
         * Every pointer aims into a byte buffer owned elsewhere (the
         * @ref AnimationDatabase); the view is valid for that buffer's lifetime and is
         * itself trivially copyable, so it is cheap to pass by value into a kernel lambda
         * or a pose modifier. All per-joint arrays are @ref joint_count long and share the
         * joint's index; the LOD arrays are @ref lod_count long.
         */
        struct SkeletonView
        {
            /** @brief Number of joints; every per-joint array below has this length. */
            std::uint32_t joint_count = 0;

            /** @brief Number of bone-LOD levels in @ref lod_joint_counts (>= 1). */
            std::uint32_t lod_count = 0;

            /**
             * @brief Parent joint index per joint, or @ref NO_PARENT for a root.
             *
             * Topologically sorted: `parents[i] < i` for every non-root joint, so a
             * forward scan composes model space with no reordering.
             */
            const std::uint16_t* parents = nullptr;

            /** @brief Bind-pose local translation per joint. */
            const Vector3f* bind_translations = nullptr;

            /** @brief Bind-pose local rotation per joint. */
            const Quaternionf* bind_rotations = nullptr;

            /** @brief Bind-pose local scale per joint. */
            const Vector3f* bind_scales = nullptr;

            /**
             * @brief Inverse bind matrix per joint: object space to joint-local at bind.
             *
             * The right factor of a skin matrix (`skin[i] = model_pose[i] * inverse_bind[i]`).
             * Stored, not derived per frame — it is a property of the rig, not the pose.
             */
            const JointMatrix* inverse_bind = nullptr;

            /** @brief FNV-1a 64 hash of each joint's name, for mask / IK / attachment lookup. */
            const NameHash* joint_names = nullptr;

            /**
             * @brief Concatenated null-terminated joint name strings, for editor display.
             *
             * Debug data — the runtime addresses joints by @ref joint_names (hashes); this
             * table exists so the skeleton inspector and viewport overlay can show readable
             * names. @ref joint_name_offsets gives each joint's start byte within it.
             */
            const char* joint_name_data = nullptr;

            /** @brief Byte offset into @ref joint_name_data of each joint's name string. */
            const std::uint32_t* joint_name_offsets = nullptr;

            /**
             * @brief Joint count active at each LOD level, coarsest last.
             *
             * `lod_joint_counts[0] == joint_count` (full detail). Each higher level is a
             * prefix length; the import-time sort places leaf chains (fingers, twist bones)
             * last within their subtree so every prefix is itself a valid skeleton.
             */
            const std::uint16_t* lod_joint_counts = nullptr;

            /**
             * @brief Whether the view points at real data.
             * @return True once a loader has populated it.
             */
            bool valid() const noexcept { return parents != nullptr && joint_count > 0; }

            /**
             * @brief Finds the joint carrying a given name hash.
             * @param name The FNV-1a 64 hash of the joint name (see @ref hash_name).
             * @return The joint index, or -1 if no joint has that name.
             */
            int find_joint(NameHash name) const noexcept
            {
                for (std::uint32_t i = 0; i < joint_count; ++i)
                    if (joint_names[i] == name)
                        return static_cast<int>(i);
                return -1;
            }

            /**
             * @brief The readable name of a joint, for editor display.
             * @param joint A joint index in [0, joint_count).
             * @return The joint's name, or "" if the debug name table is absent.
             */
            const char* joint_name(std::uint32_t joint) const noexcept
            {
                if (joint_name_data == nullptr || joint_name_offsets == nullptr ||
                    joint >= joint_count)
                    return "";
                return joint_name_data + joint_name_offsets[joint];
            }
        };

        /**
         * @brief The engine's double-precision @c Mat4 for a float joint matrix.
         * @param joint A float object-space joint/skin matrix.
         * @return The same matrix widened to boundary precision.
         */
        inline Mat4 to_mat4(const JointMatrix& joint) noexcept
        {
            Mat4 out{};
            for (int i = 0; i < 16; ++i)
                out.m[i] = static_cast<Scalar>(joint.m[i]);
            return out;
        }

        /**
         * @brief A float joint matrix from the engine's double-precision @c Mat4.
         * @param matrix A double object-space matrix.
         * @return The same matrix narrowed to float storage.
         */
        inline JointMatrix to_joint_matrix(const Mat4& matrix) noexcept
        {
            JointMatrix out{};
            for (int i = 0; i < 16; ++i)
                out.m[i] = static_cast<float>(matrix.m[i]);
            return out;
        }

        static_assert(sizeof(JointMatrix) == 64, "JointMatrix must be 16 tightly packed floats");
        static_assert(sizeof(Vector3f) == 12, "Vector3f must be 3 tightly packed floats");
        static_assert(sizeof(Quaternionf) == 16, "Quaternionf must be 4 tightly packed floats");
    } // namespace Animation
} // namespace SushiEngine
