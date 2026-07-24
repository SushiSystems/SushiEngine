/**************************************************************************/
/* animator_step.hpp                                                     */
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
 * @file animator_step.hpp
 * @brief The deterministic Animator tick: the interpreter over a compiled controller.
 *
 * @ref animator_step is the fixed-tick half of the animator (design §5.1): per layer it
 * advances normalized time, evaluates transitions (Any-State first, then the current
 * state's, honoring exit time), steps or starts crossfades, consumes triggers exactly once,
 * appends the events its clip crossed, and accumulates the tick's root-motion delta. It
 * touches only the trivially-copyable @ref AnimatorInstance and the immutable controller and
 * clip views, uses no wall clock, and is branch-deterministic — so a rolled-back-and-replayed
 * tick reproduces the animator state, root motion, and events byte-for-byte. @ref apply_root_motion
 * is the second system that moves the entity; the host drains the event queue through
 * @ref IAnimationEventSink at the frame barrier.
 */

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <SushiEngine/animation/animation_database.hpp>
#include <SushiEngine/animation/animator_components.hpp>
#include <SushiEngine/animation/animator_controller.hpp>
#include <SushiEngine/animation/clip.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief The seam a host drains fired animation events into. */
        class IAnimationEventSink
        {
            public:
                virtual ~IAnimationEventSink() = default;

                /**
                 * @brief Delivers one fired animation event.
                 * @param entity The animated entity's id.
                 * @param event  The fired event (name hash, payload, layer).
                 */
                virtual void on_animation_event(std::uint32_t entity, const AnimatorEvent& event) = 0;
        };

        namespace detail
        {
            /** @brief Whether the accumulating normalized time crossed `k + mark` this tick. */
            inline bool crossed(float old_time, float new_time, float mark) noexcept
            {
                if (new_time <= old_time)
                    return false;
                const long before = static_cast<long>(std::floor(old_time - mark));
                const long after = static_cast<long>(std::floor(new_time - mark));
                return after > before;
            }

            /** @brief Samples one joint's local translation at a time (single joint, no alloc). */
            inline Vector3f joint_translation(const ClipView& clip, float time_seconds, bool loop,
                                              std::uint32_t joint) noexcept
            {
                if (!clip.valid() || joint >= clip.joint_count)
                    return Vector3f{0.0f, 0.0f, 0.0f};
                float position;
                std::uint32_t frame0;
                std::uint32_t frame1;
                if (clip.frame_count == 1)
                {
                    frame0 = frame1 = 0;
                    position = 0.0f;
                }
                else if (loop)
                {
                    const float span = static_cast<float>(clip.frame_count);
                    float local = std::fmod(time_seconds * clip.sample_rate, span);
                    if (local < 0.0f)
                        local += span;
                    position = local;
                    frame0 = static_cast<std::uint32_t>(local) % clip.frame_count;
                    frame1 = (frame0 + 1) % clip.frame_count;
                }
                else
                {
                    const float last = static_cast<float>(clip.frame_count - 1);
                    float local = time_seconds * clip.sample_rate;
                    if (local < 0.0f)
                        local = 0.0f;
                    if (local > last)
                        local = last;
                    position = local;
                    frame0 = static_cast<std::uint32_t>(local);
                    frame1 = frame0 + 1 < clip.frame_count ? frame0 + 1 : frame0;
                }
                const float alpha = position - std::floor(position);
                if (clip.format == ClipFormat::Compressed)
                    return lerp(clip.compressed.translation_at(frame0, joint),
                                clip.compressed.translation_at(frame1, joint), alpha);
                return lerp(clip.translations[frame0 * clip.joint_count + joint],
                            clip.translations[frame1 * clip.joint_count + joint], alpha);
            }

            /** @brief The root joint's translation as a continuous function of accumulating time. */
            inline Vector3f root_at(const ClipView& clip, float normalized) noexcept
            {
                if (!clip.valid())
                    return Vector3f{0.0f, 0.0f, 0.0f};
                const float duration = clip.duration > 0.0f ? clip.duration : 0.0f;
                const float cycles = std::floor(normalized);
                const float fraction = normalized - cycles;
                const Vector3f local = joint_translation(clip, fraction * duration, false, 0);
                // The per-cycle displacement, so root motion is continuous across a loop.
                const Vector3f first = joint_translation(clip, 0.0f, false, 0);
                const Vector3f last = joint_translation(clip, duration, false, 0);
                return local + (last - first) * cycles;
            }

            /** @brief Evaluates one condition against the parameter block. */
            inline bool condition_true(const ControllerView& controller,
                                       const AnimatorParameterBlock& parameters,
                                       const ConditionRecord& condition) noexcept
            {
                if (condition.parameter_index >= controller.parameter_count)
                    return false;
                const ParameterValue& value = parameters.values[condition.parameter_index];
                switch (static_cast<Comparator>(condition.comparator))
                {
                    case Comparator::Greater:
                        return (value.type == 1 ? static_cast<float>(value.as_int) : value.as_float) >
                               condition.threshold;
                    case Comparator::Less:
                        return (value.type == 1 ? static_cast<float>(value.as_int) : value.as_float) <
                               condition.threshold;
                    case Comparator::Equals:
                        return value.as_int == static_cast<std::int32_t>(condition.threshold);
                    case Comparator::NotEquals:
                        return value.as_int != static_cast<std::int32_t>(condition.threshold);
                    case Comparator::If:
                        return value.as_uint != 0u;
                    case Comparator::IfNot:
                        return value.as_uint == 0u;
                }
                return false;
            }

            /** @brief Whether a transition's conditions all pass (exit-time handled by the caller). */
            inline bool conditions_pass(const ControllerView& controller,
                                        const AnimatorParameterBlock& parameters,
                                        const TransitionRecord& transition) noexcept
            {
                for (std::uint32_t c = 0; c < transition.condition_count; ++c)
                    if (!condition_true(controller, parameters,
                                        controller.conditions[transition.condition_base + c]))
                        return false;
                return true;
            }

            /** @brief Consumes any trigger a transition's conditions read (fires exactly once). */
            inline void consume_triggers(const ControllerView& controller,
                                         AnimatorParameterBlock& parameters,
                                         const TransitionRecord& transition) noexcept
            {
                for (std::uint32_t c = 0; c < transition.condition_count; ++c)
                {
                    const ConditionRecord& condition =
                        controller.conditions[transition.condition_base + c];
                    if (condition.parameter_index < controller.parameter_count &&
                        controller.parameters[condition.parameter_index].type ==
                            static_cast<std::uint32_t>(ParameterType::Trigger))
                        parameters.values[condition.parameter_index].as_uint = 0u;
                }
            }

            /**
             * @brief Resolves a state's motion to clip contributions and an effective duration.
             *
             * A single-clip state resolves to one contribution at full weight; a blend-tree state
             * resolves its tree against the parameters, and the effective duration is the
             * weight-averaged duration of the contributing clips (so the blend advances at a
             * coherent rate — Unity's normalised-time synchronisation).
             *
             * @param controller The compiled controller.
             * @param database   The asset database (for clip durations).
             * @param parameters The animator parameters the blend tree reads.
             * @param state      The state whose motion is resolved.
             * @param out        Receives the contributions (capacity @ref MAX_BLEND_CONTRIBUTIONS).
             * @param out_count  Receives the contribution count.
             * @return The effective duration in seconds (>= a small positive floor).
             */
            inline float resolve_state_motion(const ControllerView& controller,
                                              const IAnimationDatabase& database,
                                              const AnimatorParameterBlock& parameters,
                                              const StateRecord& state, BlendContribution* out,
                                              std::uint32_t& out_count) noexcept
            {
                if (state.blend_tree < 0)
                {
                    out[0].clip = state.clip;
                    out[0].weight = 1.0f;
                    out[0].speed = 1.0f;
                    out_count = 1;
                    const ClipView clip = database.clip(state.clip);
                    return clip.valid() && clip.duration > 0.0f ? clip.duration : 1.0f;
                }

                out_count = resolve_blend_tree(controller.nodes, controller.children, controller.pairs,
                                               static_cast<std::uint32_t>(state.blend_tree), parameters,
                                               out, MAX_BLEND_CONTRIBUTIONS);
                float weight_sum = 0.0f;
                float duration_sum = 0.0f;
                for (std::uint32_t i = 0; i < out_count; ++i)
                {
                    const ClipView clip = database.clip(out[i].clip);
                    const float duration = clip.valid() && clip.duration > 0.0f ? clip.duration : 1.0f;
                    duration_sum += out[i].weight * duration;
                    weight_sum += out[i].weight;
                }
                return weight_sum > 1e-6f ? duration_sum / weight_sum : 1.0f;
            }

            /** @brief The weighted root-track displacement of a contribution set between two times. */
            inline Vector3f blend_root_delta(const IAnimationDatabase& database,
                                             const BlendContribution* contributions,
                                             std::uint32_t count, float old_time,
                                             float new_time) noexcept
            {
                Vector3f delta{0.0f, 0.0f, 0.0f};
                for (std::uint32_t i = 0; i < count; ++i)
                {
                    const ClipView clip = database.clip(contributions[i].clip);
                    if (!clip.valid())
                        continue;
                    const Vector3f step = root_at(clip, new_time) - root_at(clip, old_time);
                    delta = delta + step * contributions[i].weight;
                }
                return delta;
            }
        } // namespace detail

        /**
         * @brief Advances one animator's deterministic state by one fixed tick.
         *
         * @param controller The compiled controller driving this animator.
         * @param database   The asset database (for clip durations and root sampling).
         * @param instance   The animator state, advanced in place.
         * @param fixed_dt   The fixed simulation step in seconds.
         */
        inline void animator_step(const ControllerView& controller,
                                  const IAnimationDatabase& database, AnimatorInstance& instance,
                                  float fixed_dt)
        {
            if (!controller.valid())
                return;

            if (instance.initialized == 0)
            {
                for (std::uint32_t l = 0; l < controller.layer_count && l < ANIMATOR_MAX_LAYERS; ++l)
                {
                    const LayerRecord& layer = controller.layers[l];
                    AnimatorLayerState& state = instance.layers[l];
                    state.current_state = layer.default_state;
                    state.weight = layer.weight;
                    state.next_state = -1;
                    state.transition_state = -1;
                    state.transition_progress = 0.0f;
                    const StateRecord& record =
                        controller.states[layer.state_base + layer.default_state];
                    state.normalized_time = record.cycle_offset;
                }
                instance.initialized = 1;
            }

            instance.events.clear();
            instance.root_motion = RootMotionDelta{};

            for (std::uint32_t l = 0; l < controller.layer_count && l < ANIMATOR_MAX_LAYERS; ++l)
            {
                const LayerRecord& layer = controller.layers[l];
                AnimatorLayerState& s = instance.layers[l];
                if (s.current_state < 0)
                    continue;

                // The layer weight can be driven by a parameter (animatable from gameplay); it feeds
                // the frame evaluator's mask-gated fold. It lives in the snapshotted layer state, so
                // its update here stays deterministic and rollback-exact.
                if (layer.weight_parameter >= 0)
                    s.weight = detail::read_parameter_float(instance.parameters, layer.weight_parameter);
                else
                    s.weight = layer.weight;

                const StateRecord& state =
                    controller.states[layer.state_base + static_cast<std::uint32_t>(s.current_state)];
                BlendContribution contributions[MAX_BLEND_CONTRIBUTIONS];
                std::uint32_t contribution_count = 0;
                const float duration = detail::resolve_state_motion(
                    controller, database, instance.parameters, state, contributions, contribution_count);

                float speed = state.speed * instance.speed;
                if (state.speed_parameter >= 0 &&
                    static_cast<std::uint32_t>(state.speed_parameter) < controller.parameter_count)
                {
                    const ParameterValue& p =
                        instance.parameters.values[state.speed_parameter];
                    speed *= (p.type == 1) ? static_cast<float>(p.as_int) : p.as_float;
                }

                const float old_time = s.normalized_time;
                const float new_time = old_time + fixed_dt * speed / duration;

                // Advance an active crossfade toward the destination.
                if (s.transition_state >= 0)
                {
                    const TransitionRecord& transition =
                        controller.transitions[s.transition_state];
                    const float transition_seconds = transition.duration * duration;
                    s.next_normalized_time += fixed_dt * speed / duration;
                    s.transition_progress +=
                        transition_seconds > 0.0f ? fixed_dt / transition_seconds : 1.0f;
                    if (s.transition_progress >= 1.0f)
                    {
                        s.current_state = static_cast<std::int32_t>(transition.destination_state);
                        s.normalized_time = s.next_normalized_time;
                        s.transition_state = -1;
                        s.next_state = -1;
                        s.transition_progress = 0.0f;
                    }
                }

                // Evaluate transitions only when no crossfade is in progress (A3: no interruption
                // of an active crossfade; the interruption source is honored from A4 on).
                if (s.transition_state < 0)
                {
                    // Any-State transitions first, then the current state's; first match wins.
                    std::int32_t chosen = -1;
                    for (std::uint32_t pass = 0; pass < 2 && chosen < 0; ++pass)
                    {
                        const std::uint32_t base =
                            pass == 0 ? layer.any_transition_base : state.transition_base;
                        const std::uint32_t count =
                            pass == 0 ? layer.any_transition_count : state.transition_count;
                        for (std::uint32_t t = 0; t < count; ++t)
                        {
                            const std::uint32_t index = base + t;
                            const TransitionRecord& transition = controller.transitions[index];
                            if (transition.has_exit_time != 0 &&
                                !detail::crossed(old_time, new_time, transition.exit_time))
                                continue;
                            if (!detail::conditions_pass(controller, instance.parameters, transition))
                                continue;
                            chosen = static_cast<std::int32_t>(index);
                            break;
                        }
                    }
                    if (chosen >= 0)
                    {
                        const TransitionRecord& transition =
                            controller.transitions[static_cast<std::uint32_t>(chosen)];
                        detail::consume_triggers(controller, instance.parameters, transition);
                        if (transition.duration <= 0.0f)
                        {
                            // Instant transition: switch immediately.
                            s.current_state =
                                static_cast<std::int32_t>(transition.destination_state);
                            s.normalized_time = transition.offset;
                            s.transition_state = -1;
                            s.next_state = -1;
                            s.transition_progress = 0.0f;
                            continue; // the destination advances next tick
                        }
                        s.next_state = static_cast<std::int32_t>(transition.destination_state);
                        s.next_normalized_time = transition.offset;
                        s.transition_state = chosen;
                        s.transition_progress = 0.0f;
                    }
                }

                s.normalized_time = new_time;

                // Fire the events this state's clip crossed this tick (loop-aware).
                for (std::uint32_t e = 0; e < state.event_count; ++e)
                {
                    const EventRecord& event = controller.events[state.event_base + e];
                    if (detail::crossed(old_time, new_time, event.normalized_time))
                        instance.events.push(event.name, event.payload, l);
                }

                // Root motion from the base layer only (weighted across a blend tree's clips).
                if (l == 0 && instance.apply_root_motion != 0 && contribution_count > 0)
                    instance.root_motion.position = detail::blend_root_delta(
                        database, contributions, contribution_count, old_time, new_time);
            }
        }

        /**
         * @brief Applies a tick's root-motion delta to an entity's position and orientation.
         *
         * The delta is in the entity's local frame, so the translation is rotated by the
         * current orientation before it is added — a character walking forward moves along its
         * own facing, not the world's.
         *
         * @param delta       The tick's root-motion delta.
         * @param position    The entity position, advanced in place.
         * @param orientation The entity orientation, advanced in place.
         */
        inline void apply_root_motion(const RootMotionDelta& delta, Vector3& position,
                                      Quaternion& orientation) noexcept
        {
            const Vector3 local{static_cast<Scalar>(delta.position.x),
                                static_cast<Scalar>(delta.position.y),
                                static_cast<Scalar>(delta.position.z)};
            position = position + rotate(orientation, local);
            const Quaternion spin{static_cast<Scalar>(delta.rotation.x),
                                  static_cast<Scalar>(delta.rotation.y),
                                  static_cast<Scalar>(delta.rotation.z),
                                  static_cast<Scalar>(delta.rotation.w)};
            orientation = normalize(mul(orientation, spin));
        }

        /**
         * @brief Drains an animator's fired events into a sink after the tick.
         * @param instance The animator whose queue is drained.
         * @param entity   The entity id passed to the sink.
         * @param sink     The sink to deliver each event to.
         */
        inline void drain_events(const AnimatorInstance& instance, std::uint32_t entity,
                                 IAnimationEventSink& sink)
        {
            for (std::uint32_t i = 0; i < instance.events.count; ++i)
                sink.on_animation_event(entity, instance.events.events[i]);
        }
    } // namespace Animation
} // namespace SushiEngine
