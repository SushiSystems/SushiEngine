/**************************************************************************/
/* facial_blendshapes.hpp                                                */
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
/* permissions and limitations under the License.                        */
/**************************************************************************/

#pragma once

/**
 * @file facial_blendshapes.hpp
 * @brief The canonical ARKit-52 facial blendshape set (design §12.4).
 *
 * `morph.hpp` is rig-agnostic by design: a mesh names its morph targets, a clip drives
 * whichever ones it names, matched by hash (@ref sample_morph_state) — there is no assumed
 * shape set. §12.4 named the resulting gap: "no standard blendshape set (e.g., ARKit 52), no
 * lip-sync tooling, no facial-specific authoring workflow." @ref ARKitBlendshape closes the
 * first part: the standard 52-shape set Apple's ARKit face tracking (and, by extension, most
 * third-party facial mocap/rigging tools that interoperate with it) uses, with the exact
 * canonical names @ref arkit_blendshape_name returns — so a mesh authored against that
 * convention (most commercial facial rigs are) can be driven by name without a project having
 * to invent its own facial vocabulary.
 *
 * @ref FacialBlendshapeMap is the same shape as `humanoid.hpp`'s `Avatar`: which mesh morph
 * target (if any) plays each canonical shape, resolved once by name and then addressed by
 * index every frame. Unlike `Avatar`'s bone aliasing (rig naming genuinely varies), ARKit
 * shape names are already a fixed, unambiguous standard — so this is an exact-name lookup,
 * no heuristic alias table needed.
 *
 * Deliberately not built here: lip-sync tooling (phoneme-to-viseme timing, audio analysis) and
 * a facial-specific authoring workflow (an editor panel driving @ref FacialBlendshapeMap
 * live) — both real, separate projects @ref FacialBlendshapeMap is a prerequisite for, not a
 * replacement for.
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/morph.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief The 52 canonical ARKit face-tracking blendshapes. */
        enum class ARKitBlendshape : std::uint32_t
        {
            EyeBlinkLeft = 0,
            EyeLookDownLeft,
            EyeLookInLeft,
            EyeLookOutLeft,
            EyeLookUpLeft,
            EyeSquintLeft,
            EyeWideLeft,
            EyeBlinkRight,
            EyeLookDownRight,
            EyeLookInRight,
            EyeLookOutRight,
            EyeLookUpRight,
            EyeSquintRight,
            EyeWideRight,
            JawForward,
            JawLeft,
            JawRight,
            JawOpen,
            MouthClose,
            MouthFunnel,
            MouthPucker,
            MouthLeft,
            MouthRight,
            MouthSmileLeft,
            MouthSmileRight,
            MouthFrownLeft,
            MouthFrownRight,
            MouthDimpleLeft,
            MouthDimpleRight,
            MouthStretchLeft,
            MouthStretchRight,
            MouthRollLower,
            MouthRollUpper,
            MouthShrugLower,
            MouthShrugUpper,
            MouthPressLeft,
            MouthPressRight,
            MouthLowerDownLeft,
            MouthLowerDownRight,
            MouthUpperUpLeft,
            MouthUpperUpRight,
            BrowDownLeft,
            BrowDownRight,
            BrowInnerUp,
            BrowOuterUpLeft,
            BrowOuterUpRight,
            CheekPuff,
            CheekSquintLeft,
            CheekSquintRight,
            NoseSneerLeft,
            NoseSneerRight,
            TongueOut,
            Count
        };

        /** @brief The number of canonical ARKit blendshapes. */
        constexpr std::uint32_t ARKIT_BLENDSHAPE_COUNT = static_cast<std::uint32_t>(ARKitBlendshape::Count);

        /**
         * @brief The exact ARKit shape name for a canonical blendshape (Apple's own spelling).
         * @param shape A canonical blendshape.
         * @return Its ARKit name, or "" for @c Count.
         */
        inline const char* arkit_blendshape_name(ARKitBlendshape shape) noexcept
        {
            static const char* const names[ARKIT_BLENDSHAPE_COUNT] = {
                "eyeBlinkLeft", "eyeLookDownLeft", "eyeLookInLeft", "eyeLookOutLeft",
                "eyeLookUpLeft", "eyeSquintLeft", "eyeWideLeft", "eyeBlinkRight",
                "eyeLookDownRight", "eyeLookInRight", "eyeLookOutRight", "eyeLookUpRight",
                "eyeSquintRight", "eyeWideRight", "jawForward", "jawLeft", "jawRight",
                "jawOpen", "mouthClose", "mouthFunnel", "mouthPucker", "mouthLeft",
                "mouthRight", "mouthSmileLeft", "mouthSmileRight", "mouthFrownLeft",
                "mouthFrownRight", "mouthDimpleLeft", "mouthDimpleRight", "mouthStretchLeft",
                "mouthStretchRight", "mouthRollLower", "mouthRollUpper", "mouthShrugLower",
                "mouthShrugUpper", "mouthPressLeft", "mouthPressRight", "mouthLowerDownLeft",
                "mouthLowerDownRight", "mouthUpperUpLeft", "mouthUpperUpRight", "browDownLeft",
                "browDownRight", "browInnerUp", "browOuterUpLeft", "browOuterUpRight",
                "cheekPuff", "cheekSquintLeft", "cheekSquintRight", "noseSneerLeft",
                "noseSneerRight", "tongueOut"};
            const std::uint32_t index = static_cast<std::uint32_t>(shape);
            return index < ARKIT_BLENDSHAPE_COUNT ? names[index] : "";
        }

        /**
         * @brief Which mesh morph target (if any) plays each canonical ARKit blendshape.
         *
         * Built once via @ref build_facial_blendshape_map, then addressed every frame via
         * @ref target_index / @ref has to drive a `MorphState` by semantic name instead of by
         * raw target index.
         */
        struct FacialBlendshapeMap
        {
            std::int32_t targets[ARKIT_BLENDSHAPE_COUNT];

            FacialBlendshapeMap() noexcept
            {
                for (std::uint32_t i = 0; i < ARKIT_BLENDSHAPE_COUNT; ++i)
                    targets[i] = -1;
            }

            /** @brief The mesh morph-target index playing a shape, or -1 if the mesh lacks it. */
            std::int32_t target_index(ARKitBlendshape shape) const noexcept
            {
                return targets[static_cast<std::uint32_t>(shape)];
            }

            /** @brief Whether a mesh target plays a given canonical shape. */
            bool has(ARKitBlendshape shape) const noexcept { return target_index(shape) >= 0; }

            /** @brief Number of the 52 canonical shapes this mesh actually has a target for. */
            std::uint32_t mapped_count() const noexcept
            {
                std::uint32_t count = 0;
                for (std::uint32_t i = 0; i < ARKIT_BLENDSHAPE_COUNT; ++i)
                    if (targets[i] >= 0)
                        ++count;
                return count;
            }

            /**
             * @brief Every canonical shape this mesh has no matching morph target for.
             *
             * A caller driving a facial performance against this map should log this, not
             * silently no-op on the missing shapes — a rig that is missing "jawOpen" will
             * visibly fail to speak, and finding out why should not require single-stepping a
             * debugger.
             *
             * @param out Receives the unmapped shapes, appended (not cleared first).
             */
            void list_missing(std::vector<ARKitBlendshape>& out) const
            {
                for (std::uint32_t i = 0; i < ARKIT_BLENDSHAPE_COUNT; ++i)
                    if (targets[i] < 0)
                        out.push_back(static_cast<ARKitBlendshape>(i));
            }
        };

        /**
         * @brief Resolves a mesh's morph targets against the canonical ARKit-52 names.
         * @param target_names The mesh's morph target name hashes, in its own target order
         *                     (see `morph.hpp`'s `sample_morph_state` — the same array).
         * @param target_count Targets the mesh has.
         * @return The map; a canonical shape the mesh has no matching target for stays -1.
         */
        inline FacialBlendshapeMap build_facial_blendshape_map(const NameHash* target_names,
                                                                std::uint32_t target_count)
        {
            FacialBlendshapeMap map;
            for (std::uint32_t s = 0; s < ARKIT_BLENDSHAPE_COUNT; ++s)
            {
                const NameHash name = hash_name(arkit_blendshape_name(static_cast<ARKitBlendshape>(s)));
                for (std::uint32_t t = 0; t < target_count; ++t)
                {
                    if (target_names[t] == name)
                    {
                        map.targets[s] = static_cast<std::int32_t>(t);
                        break;
                    }
                }
            }
            return map;
        }

        /**
         * @brief Sets one canonical shape's weight in a mesh's `MorphState`, by semantic name.
         *
         * A no-op (not an error) if the mesh has no target for @p shape — this is the building
         * block a facial-performance driver calls every shape it wants to set; querying
         * @ref FacialBlendshapeMap::list_missing once at load time is how a caller finds out
         * which calls will be no-ops, rather than each call failing silently and separately.
         *
         * @param map    The mesh's resolved shape map.
         * @param shape  The canonical shape to set.
         * @param weight The blend weight (typically [0, 1]; ARKit itself allows slight overshoot).
         * @param state  The instance's morph weights to write into.
         */
        inline void set_facial_blendshape(const FacialBlendshapeMap& map, ARKitBlendshape shape,
                                          float weight, MorphState& state) noexcept
        {
            const std::int32_t index = map.target_index(shape);
            if (index < 0 || static_cast<std::uint32_t>(index) >= state.count)
                return;
            state.weights[static_cast<std::uint32_t>(index)] = weight;
        }
    } // namespace Animation
} // namespace SushiEngine
