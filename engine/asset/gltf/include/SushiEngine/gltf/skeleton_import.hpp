/**************************************************************************/
/* skeleton_import.hpp                                                    */
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
 * @file skeleton_import.hpp
 * @brief Import a skeleton from a glTF file's node hierarchy and skin.
 *
 * The engine's mesh importer bakes every primitive into its node's world transform and
 * drops the node graph, `skins`, and inverse-bind matrices — exactly the data a rig
 * needs. This is the parallel lane that keeps them: it reads one skin, turns its joint
 * nodes into bind-pose local TRS, copies the inverse-bind matrices through (or lets the
 * cook derive them), and produces a relocatable `.sushiskel` blob ready for
 * @ref SushiEngine::Animation::AnimationDatabase. The declaration lives here (the
 * engine's animation surface); the implementation lives in the renderer's cgltf lane,
 * the one place cgltf is linked.
 */

#include <cstddef>
#include <string>
#include <vector>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief Imports a glTF skin as a cooked `.sushiskel` skeleton blob.
         *
         * Reads the skin at @p skin_index: each joint node's local transform becomes its
         * bind-pose TRS, node parent links become the joint parent array, and the skin's
         * inverse-bind matrices are copied through when present (otherwise the cook derives
         * them from the bind pose). The cook then topologically sorts and lays the blob out,
         * so the output already satisfies `parent[i] < i`.
         *
         * @param path       Path to a `.gltf` or `.glb` file.
         * @param out_blob   Receives the cooked blob bytes; cleared first, empty on failure.
         * @param skin_index Which skin to import (0 by default; most rigs have one).
         * @return True on success; false if the file cannot be read, has no such skin, or
         *         the skin is empty.
         */
        bool import_gltf_skeleton(const char* path, std::vector<std::byte>& out_blob,
                                  std::size_t skin_index = 0);

        /** @brief One imported animation: its name and cooked `.sushianim` blob. */
        struct GLTFClip
        {
            std::string name;            /**< The glTF animation's name (or "clip_<i>"). */
            std::vector<std::byte> blob; /**< The cooked clip, in the skeleton's joint order. */
        };

        /** @brief The result of importing a rigged, animated glTF. */
        struct GLTFAnimationImport
        {
            std::vector<std::byte> skeleton_blob; /**< The cooked `.sushiskel`. */
            std::vector<GLTFClip> clips;          /**< One entry per glTF animation. */

            /**
             * @brief The skinned mesh's morph target names, in the mesh's own target order.
             *
             * Empty when the skin drives no mesh with morph targets. Hash these with
             * @ref hash_name to get the `target_names` array @ref sample_morph_state maps a
             * clip's morph tracks onto — the identity that lets one clip drive any mesh
             * sharing the naming. A target the file leaves unnamed is called `morph_<index>`
             * here and in the clip's own tracks alike, so nameless targets still resolve
             * positionally.
             *
             * The order is the one @ref Render::Assets::import_gltf_skinned_mesh uploads its
             * delta buffer in: the first triangle primitive of the first node bound to this
             * skin that carries a complete skinned vertex set.
             */
            std::vector<std::string> morph_target_names;
        };

        /**
         * @brief Imports a skin and all its animations, sharing one joint order.
         *
         * Cooks the skeleton (see @ref import_gltf_skeleton), then resamples every glTF
         * animation onto that skeleton at a fixed rate — so each clip's tracks are in the
         * exact joint order the cooked skeleton uses, ready to play through a
         * @ref ClipView against the matching @ref SkeletonView. Channels are sampled linearly
         * (cubic-spline keys are read at their value, tangents ignored — an A1 simplification);
         * joints a channel does not drive hold their bind-pose local transform.
         *
         * A `weights` channel drives every morph target of its node at once (glTF packs all of
         * them into one sampler), so each such channel contributes one morph-weight track per
         * target, named after the target (see @ref GLTFAnimationImport::morph_target_names).
         * Targets of the same name across two channels collapse to one track — the first seen
         * wins, matching @ref ClipView::find_morph's first-match lookup. An animation with no
         * `weights` channel yields no morph tracks, and a mesh whose targets no track names
         * samples to zero weights.
         *
         * @param path        Path to a `.gltf` or `.glb` file with a skin and animations.
         * @param out         Receives the cooked skeleton and clips; cleared first.
         * @param sample_rate The uniform resample rate in frames per second.
         * @param skin_index  Which skin to import (0 by default).
         * @return True if the skeleton imported; the clip list may be empty if the file has
         *         no animations. False if the file cannot be read or has no such skin.
         */
        bool import_gltf_animated(const char* path, GLTFAnimationImport& out,
                                  float sample_rate = 30.0f, std::size_t skin_index = 0);
    } // namespace Animation
} // namespace SushiEngine
