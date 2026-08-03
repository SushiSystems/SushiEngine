/**************************************************************************/
/* animator_components.hpp                                               */
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
 * @file animator_components.hpp
 * @brief The deterministic Animator ECS columns — the state that advances at fixed tick.
 *
 * Everything a rolled-back-and-replayed tick must reproduce lives here (design §5.1): the
 * parameter values, the per-layer state-machine state, the fired-event ring, and this
 * tick's root-motion delta. Every field is an integer, an enum, or a scalar, so a column is
 * trivially copyable, byte-snapshottable, and bit-reproducible — it survives `RollbackBuffer`
 * capture/restore exactly. Pose and skinning are derived per render frame from this state and
 * are never snapshotted.
 */

#include <cstddef>
#include <cstdint>

#include <SushiEngine/animation/asset_id.hpp>
#include <SushiEngine/animation/hash.hpp>
#include <SushiEngine/animation/skeleton.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Max animation events fired by one entity in one tick (design §4.5). */
        constexpr std::uint32_t MAX_EVENTS_PER_TICK = 8;

        /** @brief Max typed parameters a controller carries and an animator stores (design §4.5). */
        constexpr std::uint32_t MAX_PARAMETERS = 32;

        /** @brief Max layers a controller carries. */
        constexpr std::uint32_t MAX_LAYERS = 4;

        /** @brief Max layers an animator advances (matches @ref MAX_LAYERS). */
        constexpr std::uint32_t ANIMATOR_MAX_LAYERS = 4;

        /**
         * @brief A typed parameter value: one of float / int / bool / trigger.
         *
         * A trivially-copyable union with a small tag; `bool` and `trigger` use @c as_uint
         * (0/1). Written by gameplay, read by @ref animator_step.
         */
        struct ParameterValue
        {
            std::uint32_t type = 0; /**< @ref ParameterType. */
            union
            {
                float as_float;
                std::int32_t as_int;
                std::uint32_t as_uint;
            };

            ParameterValue() : type(0), as_uint(0) {}
        };

        /**
         * @brief One entity's parameter values, indexed as the controller's parameter table.
         *
         * A fixed 32-slot block (design §4.5). Triggers are consumed exactly once by
         * @ref animator_step, which clears the slot after a transition reads it.
         */
        struct AnimatorParameterBlock
        {
            ParameterValue values[MAX_PARAMETERS];

            /** @brief Sets a float parameter. */
            void set_float(std::uint32_t index, float value)
            {
                values[index].type = 0;
                values[index].as_float = value;
            }

            /** @brief Sets an int parameter. */
            void set_int(std::uint32_t index, std::int32_t value)
            {
                values[index].type = 1;
                values[index].as_int = value;
            }

            /** @brief Sets a bool parameter. */
            void set_bool(std::uint32_t index, bool value)
            {
                values[index].type = 2;
                values[index].as_uint = value ? 1u : 0u;
            }

            /** @brief Fires a trigger parameter (consumed once by the next matching transition). */
            void set_trigger(std::uint32_t index)
            {
                values[index].type = 3;
                values[index].as_uint = 1u;
            }
        };

        /**
         * @brief One layer's state-machine state.
         *
         * @c transition_state is -1 when no crossfade is active; otherwise the animator is
         * blending from @c current_state to @c next_state over @c transition_progress in
         * [0, 1] (normalized to the source clip's transition duration).
         */
        struct AnimatorLayerState
        {
            std::int32_t current_state = -1;      /**< Active state, or -1 before init. */
            float normalized_time = 0.0f;         /**< Current state's normalized playback time. */
            std::int32_t next_state = -1;         /**< Crossfade destination, or -1. */
            float next_normalized_time = 0.0f;    /**< Destination's normalized time during a crossfade. */
            float transition_progress = 0.0f;     /**< Crossfade progress in [0, 1]. */
            std::int32_t transition_state = -1;   /**< -1 = no crossfade; else 1 = active. */
            float weight = 1.0f;                  /**< Layer weight. */
        };

        /** @brief One fired animation event this tick. */
        struct AnimatorEvent
        {
            NameHash name = 0;
            std::int32_t payload = 0;
            std::uint32_t layer = 0;
        };

        /**
         * @brief A fixed ring of the events fired this tick, drained at the frame barrier.
         *
         * @ref animator_step appends; the host drains it into @ref IAnimationEventSink after
         * the tick, deduplicated across rollback reconciliation replays the same way commands are.
         */
        struct AnimatorEventQueue
        {
            AnimatorEvent events[MAX_EVENTS_PER_TICK];
            std::uint32_t count = 0;

            /** @brief Clears the queue at the start of a tick. */
            void clear() { count = 0; }

            /** @brief Appends an event, dropping it if the ring is full. */
            void push(NameHash name, std::int32_t payload, std::uint32_t layer)
            {
                if (count < MAX_EVENTS_PER_TICK)
                {
                    events[count].name = name;
                    events[count].payload = payload;
                    events[count].layer = layer;
                    ++count;
                }
            }
        };

        /** @brief The root-motion this tick moved the entity by, in the entity's local frame. */
        struct RootMotionDelta
        {
            Vector3f position = Vector3f{0.0f, 0.0f, 0.0f};
            Quaternionf rotation = Quaternionf{0.0f, 0.0f, 0.0f, 1.0f};
        };

        /**
         * @brief The animator's identity and per-layer state, one per animated entity.
         *
         * Trivially copyable; bundles the controller/skeleton binding, the global speed and
         * flags, the parameter block, the per-layer states, the event queue, and the tick's
         * root-motion delta so the whole deterministic animator state is one snapshottable unit.
         */
        struct AnimatorInstance
        {
            AssetId controller = INVALID_ASSET; /**< The controller asset driving this animator. */
            AssetId skeleton = INVALID_ASSET;   /**< The skeleton the clips pose. */
            float speed = 1.0f;                 /**< Global speed multiplier. */
            std::uint32_t apply_root_motion = 1; /**< 1 to move the entity by root motion. */
            std::uint32_t initialized = 0;      /**< 0 until the first step seeds the layers. */
            AnimatorParameterBlock parameters;
            AnimatorLayerState layers[ANIMATOR_MAX_LAYERS];
            AnimatorEventQueue events;
            RootMotionDelta root_motion;
        };
    } // namespace Animation
} // namespace SushiEngine
