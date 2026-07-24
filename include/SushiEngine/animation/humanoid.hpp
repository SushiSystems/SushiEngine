/**************************************************************************/
/* humanoid.hpp                                                          */
/**************************************************************************/
/*                          This file is part of:                         */
/*                              SushiEngine                               */
/*               https://github.com/SushiSystems/SushiEngine              */
/*                        https://sushisystems.io                         */
/**************************************************************************/
/* Copyright (c) 2026-present Mustafa Garip & Sushi Systems               */
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
 * @file humanoid.hpp
 * @brief The canonical humanoid avatar and its mapping onto a rig (phase A8).
 *
 * A humanoid @ref Avatar names, for a given skeleton, which joint plays each canonical human
 * bone (Hips, Spine, LeftUpperArm, ...). It is what lets one clip library drive
 * differently-proportioned rigs (design §4.4): retargeting speaks in canonical bones, not joint
 * indices. The mapping is built by name — an explicit table, or a heuristic over common naming
 * conventions and their aliases — so no runtime reflection is needed. @ref opposite gives each
 * bone its left/right mirror, the basis for clip mirroring in @ref retarget.hpp.
 */

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/skeleton.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief The canonical human bones an avatar maps (a Mecanim-shaped subset). */
        enum class HumanBone : std::uint32_t
        {
            Hips = 0,
            Spine,
            Chest,
            UpperChest,
            Neck,
            Head,
            LeftShoulder,
            LeftUpperArm,
            LeftLowerArm,
            LeftHand,
            RightShoulder,
            RightUpperArm,
            RightLowerArm,
            RightHand,
            LeftUpperLeg,
            LeftLowerLeg,
            LeftFoot,
            LeftToes,
            RightUpperLeg,
            RightLowerLeg,
            RightFoot,
            RightToes,
            Count
        };

        /** @brief The number of canonical human bones. */
        constexpr std::uint32_t HUMAN_BONE_COUNT = static_cast<std::uint32_t>(HumanBone::Count);

        /**
         * @brief The left/right mirror of a bone (central bones map to themselves).
         * @param bone A canonical bone.
         * @return Its opposite-side bone, or the same bone if it is central.
         */
        inline HumanBone opposite(HumanBone bone) noexcept
        {
            switch (bone)
            {
                case HumanBone::LeftShoulder: return HumanBone::RightShoulder;
                case HumanBone::LeftUpperArm: return HumanBone::RightUpperArm;
                case HumanBone::LeftLowerArm: return HumanBone::RightLowerArm;
                case HumanBone::LeftHand: return HumanBone::RightHand;
                case HumanBone::LeftUpperLeg: return HumanBone::RightUpperLeg;
                case HumanBone::LeftLowerLeg: return HumanBone::RightLowerLeg;
                case HumanBone::LeftFoot: return HumanBone::RightFoot;
                case HumanBone::LeftToes: return HumanBone::RightToes;
                case HumanBone::RightShoulder: return HumanBone::LeftShoulder;
                case HumanBone::RightUpperArm: return HumanBone::LeftUpperArm;
                case HumanBone::RightLowerArm: return HumanBone::LeftLowerArm;
                case HumanBone::RightHand: return HumanBone::LeftHand;
                case HumanBone::RightUpperLeg: return HumanBone::LeftUpperLeg;
                case HumanBone::RightLowerLeg: return HumanBone::LeftLowerLeg;
                case HumanBone::RightFoot: return HumanBone::LeftFoot;
                case HumanBone::RightToes: return HumanBone::LeftToes;
                default: return bone;
            }
        }

        /**
         * @brief A humanoid avatar: which joint plays each canonical bone, for one skeleton.
         *
         * Trivially copyable; @c joints[bone] is the joint index, or -1 where the rig has no such
         * bone. Built by @ref build_avatar_heuristic or @ref AvatarDesc.
         */
        struct Avatar
        {
            std::int32_t joints[HUMAN_BONE_COUNT];

            Avatar() noexcept
            {
                for (std::uint32_t i = 0; i < HUMAN_BONE_COUNT; ++i)
                    joints[i] = -1;
            }

            /** @brief The joint playing a bone, or -1. */
            std::int32_t joint(HumanBone bone) const noexcept
            {
                return joints[static_cast<std::uint32_t>(bone)];
            }

            /** @brief Whether a bone is mapped to a joint. */
            bool has(HumanBone bone) const noexcept { return joint(bone) >= 0; }

            /** @brief Number of bones mapped to a joint. */
            std::uint32_t mapped_count() const noexcept
            {
                std::uint32_t count = 0;
                for (std::uint32_t i = 0; i < HUMAN_BONE_COUNT; ++i)
                    if (joints[i] >= 0)
                        ++count;
                return count;
            }
        };

        namespace detail
        {
            /** @brief One bone's candidate joint names, tried in order by the heuristic. */
            struct BoneAlias
            {
                HumanBone bone;
                const char* names[6];
            };

            /** @brief The default alias table covering common glTF/FBX naming conventions. */
            inline const BoneAlias* bone_alias_table(std::size_t& count) noexcept
            {
                static const BoneAlias table[] = {
                    {HumanBone::Hips, {"Hips", "hips", "Pelvis", "pelvis", "hip", nullptr}},
                    {HumanBone::Spine, {"Spine", "spine", "Spine1", "spine_01", nullptr, nullptr}},
                    {HumanBone::Chest, {"Chest", "chest", "Spine2", "spine_02", nullptr, nullptr}},
                    {HumanBone::UpperChest, {"UpperChest", "Spine3", "spine_03", nullptr, nullptr, nullptr}},
                    {HumanBone::Neck, {"Neck", "neck", nullptr, nullptr, nullptr, nullptr}},
                    {HumanBone::Head, {"Head", "head", nullptr, nullptr, nullptr, nullptr}},
                    {HumanBone::LeftShoulder, {"LeftShoulder", "shoulder_l", "clavicle_l", nullptr, nullptr, nullptr}},
                    {HumanBone::LeftUpperArm, {"LeftUpperArm", "LeftArm", "upperarm_l", "arm_l", nullptr, nullptr}},
                    {HumanBone::LeftLowerArm, {"LeftLowerArm", "LeftForeArm", "lowerarm_l", "forearm_l", nullptr, nullptr}},
                    {HumanBone::LeftHand, {"LeftHand", "hand_l", nullptr, nullptr, nullptr, nullptr}},
                    {HumanBone::RightShoulder, {"RightShoulder", "shoulder_r", "clavicle_r", nullptr, nullptr, nullptr}},
                    {HumanBone::RightUpperArm, {"RightUpperArm", "RightArm", "upperarm_r", "arm_r", nullptr, nullptr}},
                    {HumanBone::RightLowerArm, {"RightLowerArm", "RightForeArm", "lowerarm_r", "forearm_r", nullptr, nullptr}},
                    {HumanBone::RightHand, {"RightHand", "hand_r", nullptr, nullptr, nullptr, nullptr}},
                    {HumanBone::LeftUpperLeg, {"LeftUpperLeg", "LeftUpLeg", "thigh_l", "upleg_l", nullptr, nullptr}},
                    {HumanBone::LeftLowerLeg, {"LeftLowerLeg", "LeftLeg", "calf_l", "shin_l", nullptr, nullptr}},
                    {HumanBone::LeftFoot, {"LeftFoot", "foot_l", nullptr, nullptr, nullptr, nullptr}},
                    {HumanBone::LeftToes, {"LeftToes", "LeftToeBase", "toe_l", nullptr, nullptr, nullptr}},
                    {HumanBone::RightUpperLeg, {"RightUpperLeg", "RightUpLeg", "thigh_r", "upleg_r", nullptr, nullptr}},
                    {HumanBone::RightLowerLeg, {"RightLowerLeg", "RightLeg", "calf_r", "shin_r", nullptr, nullptr}},
                    {HumanBone::RightFoot, {"RightFoot", "foot_r", nullptr, nullptr, nullptr, nullptr}},
                    {HumanBone::RightToes, {"RightToes", "RightToeBase", "toe_r", nullptr, nullptr, nullptr}},
                };
                count = sizeof(table) / sizeof(table[0]);
                return table;
            }
        } // namespace detail

        /**
         * @brief Maps a skeleton's joints to canonical bones by common naming conventions.
         *
         * For each bone it tries the default alias list in order and takes the first joint whose
         * name matches (by hash). Bones the rig has no name for stay unmapped (-1). Author-side
         * overrides go through @ref AvatarDesc when a rig's naming is unusual.
         *
         * @param skeleton The rig to map.
         * @return The avatar; unmapped bones are -1.
         */
        inline Avatar build_avatar_heuristic(const SkeletonView& skeleton)
        {
            Avatar avatar;
            std::size_t count = 0;
            const detail::BoneAlias* table = detail::bone_alias_table(count);
            for (std::size_t i = 0; i < count; ++i)
            {
                for (const char* name : table[i].names)
                {
                    if (name == nullptr)
                        break;
                    const int joint = skeleton.find_joint(hash_name(name));
                    if (joint >= 0)
                    {
                        avatar.joints[static_cast<std::uint32_t>(table[i].bone)] = joint;
                        break;
                    }
                }
            }
            return avatar;
        }

        /** @brief An explicit bone→joint-name mapping, for rigs the heuristic misses. */
        struct AvatarDesc
        {
            /** @brief One authored mapping: a canonical bone and the joint name that plays it. */
            struct Entry
            {
                HumanBone bone = HumanBone::Hips;
                std::string joint;
            };

            std::vector<Entry> entries;
        };

        /**
         * @brief Builds an avatar from an explicit bone→joint-name table.
         * @param desc     The authored mapping.
         * @param skeleton The rig to resolve names against.
         * @return The avatar; a name the rig lacks leaves its bone unmapped.
         */
        inline Avatar build_avatar(const AvatarDesc& desc, const SkeletonView& skeleton)
        {
            Avatar avatar;
            for (const AvatarDesc::Entry& entry : desc.entries)
            {
                const int joint = skeleton.find_joint(hash_name(entry.joint.c_str()));
                if (joint >= 0)
                    avatar.joints[static_cast<std::uint32_t>(entry.bone)] = joint;
            }
            return avatar;
        }
    } // namespace Animation
} // namespace SushiEngine
