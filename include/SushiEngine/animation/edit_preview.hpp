/**************************************************************************/
/* edit_preview.hpp                                                      */
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
 * @file edit_preview.hpp
 * @brief Edit-mode scrubbing: pose a controller at an explicit state and time (phase A9).
 *
 * The Animation and Animator windows preview a clip or state at a scrubbed time *outside* the
 * simulation loop (design §7): no fixed tick, no transitions advancing, just "show me this state
 * at normalized time t with these parameters." This configures an @ref AnimatorInstance for that
 * — pinning a layer to a chosen state and time with no crossfade — so the ordinary
 * @ref AnimatorEvaluator produces the pose on demand. It is a thin, deterministic authoring seam
 * over the same evaluator the runtime uses, so the preview matches play mode exactly.
 */

#include <cstdint>

#include <SushiEngine/animation/animator_components.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/hash.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /**
         * @brief The index within a layer of the state carrying a name hash, or -1.
         * @param controller The compiled controller.
         * @param layer      The layer to search.
         * @param name       The FNV-1a 64 hash of the state name.
         * @return The state's index within the layer (0-based), or -1 if none matches.
         */
        inline int find_state_in_layer(const ControllerView& controller, std::uint32_t layer,
                                       NameHash name) noexcept
        {
            if (!controller.valid() || layer >= controller.layer_count)
                return -1;
            const LayerRecord& record = controller.layers[layer];
            for (std::uint32_t i = 0; i < record.state_count; ++i)
                if (controller.states[record.state_base + i].name == name)
                    return static_cast<int>(i);
            return -1;
        }

        /**
         * @brief Pins one layer of an animator to a state and time for edit-mode preview.
         *
         * Marks the instance initialized and clears any crossfade on the layer, so a following
         * @ref AnimatorEvaluator::evaluate poses exactly that state at @p normalized_time. Other
         * layers are left as they are (default them or scrub them too). No sim tick runs, so no
         * transition advances and nothing mutates the sim state.
         *
         * @param controller      The compiled controller.
         * @param instance        The animator to configure.
         * @param layer           The layer to pin.
         * @param state_index     The state index within the layer (see @ref find_state_in_layer).
         * @param normalized_time The normalized playback time to pose at.
         */
        inline void scrub_to_state(const ControllerView& controller, AnimatorInstance& instance,
                                   std::uint32_t layer, std::int32_t state_index,
                                   float normalized_time) noexcept
        {
            if (!controller.valid() || layer >= controller.layer_count ||
                layer >= ANIMATOR_MAX_LAYERS)
                return;
            instance.initialized = 1;
            AnimatorLayerState& state = instance.layers[layer];
            state.current_state = state_index;
            state.normalized_time = normalized_time;
            state.next_state = -1;
            state.transition_state = -1;
            state.transition_progress = 0.0f;
            state.next_normalized_time = 0.0f;
            state.weight = controller.layers[layer].weight;
        }
    } // namespace Animation
} // namespace SushiEngine
