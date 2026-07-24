/**************************************************************************/
/* morph.hpp                                                             */
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
 * @file morph.hpp
 * @brief Morph-target (blend-shape) weights and the CPU reference for applying them (phase A7).
 *
 * A morph target is a named set of per-vertex position/normal deltas; a clip drives its weight
 * as an ordinary float track (design §6.5). At runtime the weights are sampled per instance and
 * fed to the SkinningPass, which adds `Σ weight × delta` to the base vertices before joint
 * blending. This header owns the animation-side pieces: the per-instance @ref MorphState, the
 * sampler that maps a clip's morph tracks onto a mesh's target order by name, and a host
 * reference @ref apply_morph_positions that verifies the weight-blend the GPU pass performs.
 * The packed vertex-delta buffers themselves live on the render mesh.
 */

#include <cstdint>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/skeleton.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Max active morph targets per mesh (Ultra tier; design §4.5). */
        constexpr std::uint32_t MAX_MORPH_TARGETS = 64;

        /** @brief One instance's active morph-target weights, in the mesh's target order. */
        struct MorphState
        {
            float weights[MAX_MORPH_TARGETS] = {0.0f};
            std::uint32_t count = 0; /**< Active targets (== the mesh's target count). */
        };

        /**
         * @brief Samples a clip's morph tracks into a mesh's target order.
         *
         * A clip names its morph tracks; a mesh fixes its target order. This resolves each mesh
         * target to the clip track of the same name (by hash) and samples it, leaving targets the
         * clip does not drive at zero — so one clip drives any mesh that shares the naming.
         *
         * @param clip         The clip to sample.
         * @param time_seconds Playback time.
         * @param loop         Whether the clip loops.
         * @param target_names The mesh's morph target name hashes (its order).
         * @param target_count Targets the mesh has (clamped to @ref MAX_MORPH_TARGETS).
         * @param out          Receives the per-target weights.
         */
        inline void sample_morph_state(const ClipView& clip, float time_seconds, bool loop,
                                       const NameHash* target_names, std::uint32_t target_count,
                                       MorphState& out) noexcept
        {
            if (target_count > MAX_MORPH_TARGETS)
                target_count = MAX_MORPH_TARGETS;
            out.count = target_count;
            for (std::uint32_t i = 0; i < target_count; ++i)
                out.weights[i] = 0.0f;
            if (!clip.valid() || clip.morph_track_count == 0)
                return;

            float sampled[MAX_MORPH_TARGETS];
            std::uint32_t tracks = clip.morph_track_count;
            if (tracks > MAX_MORPH_TARGETS)
                tracks = MAX_MORPH_TARGETS;
            clip.sample_morph(time_seconds, loop, sampled);
            for (std::uint32_t i = 0; i < target_count; ++i)
            {
                const int track = clip.find_morph(target_names[i]);
                if (track >= 0 && static_cast<std::uint32_t>(track) < tracks)
                    out.weights[i] = sampled[track];
            }
        }

        /** @brief One morph target's packed per-vertex deltas (host reference form). */
        struct MorphTargetDeltas
        {
            const Vector3f* position_deltas = nullptr; /**< @c vertex_count long. */
        };

        /**
         * @brief The CPU reference of the SkinningPass morph blend: base + Σ weight × delta.
         *
         * Verifies the weight-blend the GPU performs; the shipping path does this in the compute
         * dispatch. Normals/tangents follow the same shape and are omitted from the reference.
         *
         * @param base         Base vertex positions (@p vertex_count long).
         * @param vertex_count Vertices in the mesh.
         * @param targets      Per-target delta buffers (@p target_count long).
         * @param weights      Per-target weights (@p target_count long).
         * @param target_count Active morph targets.
         * @param out          Receives the morphed positions (@p vertex_count long).
         */
        inline void apply_morph_positions(const Vector3f* base, std::uint32_t vertex_count,
                                          const MorphTargetDeltas* targets, const float* weights,
                                          std::uint32_t target_count, Vector3f* out) noexcept
        {
            for (std::uint32_t v = 0; v < vertex_count; ++v)
            {
                Vector3f position = base[v];
                for (std::uint32_t t = 0; t < target_count; ++t)
                {
                    if (targets[t].position_deltas == nullptr || weights[t] == 0.0f)
                        continue;
                    const Vector3f& delta = targets[t].position_deltas[v];
                    position.x += delta.x * weights[t];
                    position.y += delta.y * weights[t];
                    position.z += delta.z * weights[t];
                }
                out[v] = position;
            }
        }
    } // namespace Animation
} // namespace SushiEngine
