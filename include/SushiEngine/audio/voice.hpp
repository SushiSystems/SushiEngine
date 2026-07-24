/**************************************************************************/
/* voice.hpp                                                             */
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

#ifndef SUSHIENGINE_AUDIO_VOICE_HPP
#define SUSHIENGINE_AUDIO_VOICE_HPP

/**
 * @file voice.hpp
 * @brief A voice: a playing sound's source, gain, routing, and spatial state.
 *
 * A voice is one instance of a sound moving through the mix (see
 * `docs/design/audio_system.md` §8). @ref VoiceSource is the seam the sound comes
 * from — an oscillator here, a decoded stream later — and always renders mono; the
 * pan/spatialize step places it. Crucially the source also offers @ref VoiceSource::advance,
 * a cheap "skip forward without producing output" used when a voice is **virtualized**:
 * a virtual voice keeps its play position current for ~free so that, if it becomes
 * audible again, it resumes where it would have been.
 *
 * Distance attenuation here is the simple linear model the S2 voice manager sorts on
 * (it reaches true silence, which is what makes it a clean culling signal); the full
 * spatializer with air absorption and Doppler lands in S3–S4.
 */

#include <cmath>
#include <cstddef>

#include <SushiEngine/audio/parameter.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief A minimal 3-component position for the S2 distance model (audio-local, float). */
        struct AudioVec3
        {
            float x = 0.0f;
            float y = 0.0f;
            float z = 0.0f;
        };

        /** @brief Euclidean distance between two positions. */
        inline float distance(const AudioVec3& a, const AudioVec3& b) noexcept
        {
            const float dx = a.x - b.x;
            const float dy = a.y - b.y;
            const float dz = a.z - b.z;
            return std::sqrt(dx * dx + dy * dy + dz * dz);
        }

        /**
         * @brief How a source's level falls off with distance.
         *
         * `Linear` reaches true silence at the max distance, which is what makes it a
         * clean culling signal (and the default, so audibility ranking behaves
         * predictably); `Inverse` is the physical 1/r law; `Exponent` is a tunable
         * power law. All three are clamped to zero beyond the max distance so a distant
         * source always culls (see `docs/design/audio_system.md` §5).
         */
        enum class DistanceModel
        {
            Linear,   /**< 1 → 0 linearly between min and max distance. */
            Inverse,  /**< OpenAL-style min/(min + rolloff·(d − min)); physical 1/r at rolloff 1. */
            Exponent  /**< (d/min)^(−rolloff). */
        };

        /**
         * @brief The distance attenuation gain for a model.
         *
         * Shared by audibility ranking (voice manager) and rendering (propagation) so
         * the level a voice is culled by is the level it is played at. Full gain within
         * @p min_distance; zero at and beyond @p max_distance for every model.
         *
         * @param model        The rolloff law.
         * @param d            The source-to-listener distance in metres.
         * @param min_distance The reference distance (full gain within it).
         * @param max_distance The distance at which the source is silent/cullable.
         * @param rolloff      The rolloff factor (1 = reference).
         * @return The linear gain in [0, 1].
         */
        inline float distance_attenuation(DistanceModel model, float d, float min_distance,
                                          float max_distance, float rolloff) noexcept
        {
            if (d <= min_distance)
                return 1.0f;
            if (d >= max_distance || max_distance <= min_distance)
                return 0.0f;
            switch (model)
            {
                case DistanceModel::Linear:
                    return 1.0f - (d - min_distance) / (max_distance - min_distance);
                case DistanceModel::Inverse:
                {
                    const float g = min_distance / (min_distance + rolloff * (d - min_distance));
                    return g < 0.0f ? 0.0f : g;
                }
                case DistanceModel::Exponent:
                    return std::pow(d / min_distance, -rolloff);
            }
            return 0.0f;
        }

        /**
         * @brief A mono sample source for a voice.
         *
         * @ref render fills a mono block and returns false when a one-shot source has
         * finished (so the voice manager can free the voice). @ref advance moves the
         * source's play position forward by @p frame_count samples **without** producing
         * output — the cheap path a virtualized voice takes.
         */
        class VoiceSource
        {
            public:
                virtual ~VoiceSource() = default;

                /** @brief Prepares source state for the run (off the audio thread). */
                virtual void prepare(double sample_rate, int max_block_frames)
                {
                    (void)sample_rate;
                    (void)max_block_frames;
                }

                /** @brief Resets the source to its start. */
                virtual void reset() noexcept {}

                /**
                 * @brief Renders one mono block.
                 * @param out         The mono output buffer of @p frame_count samples.
                 * @param frame_count Number of samples to produce.
                 * @return True while the source has more to play; false when finished.
                 */
                virtual bool render(float* out, int frame_count) noexcept = 0;

                /**
                 * @brief Advances the play position without producing output (virtual voice).
                 * @param frame_count Number of samples to skip.
                 * @return True while the source has more to play; false when finished.
                 */
                virtual bool advance(int frame_count) noexcept
                {
                    (void)frame_count;
                    return true;
                }
            };

        /**
         * @brief An endless sine oscillator source.
         *
         * Phase accumulates in `double`; @ref advance simply advances the phase, so a
         * virtualized tone stays phase-coherent when it returns to real.
         */
        class ToneSource final : public VoiceSource
        {
            public:
                explicit ToneSource(float frequency_hz = 440.0f, float amplitude = 1.0f) noexcept
                    : frequency_(frequency_hz), amplitude_(amplitude)
                {
                }

                void prepare(double sample_rate, int max_block_frames) override
                {
                    (void)max_block_frames;
                    sample_rate_ = sample_rate;
                }

                void reset() noexcept override { phase_ = 0.0; }

                bool render(float* out, int frame_count) noexcept override
                {
                    const double two_pi = 6.28318530717958647692;
                    const double increment = two_pi * frequency_ / sample_rate_;
                    double phase = phase_;
                    for (int i = 0; i < frame_count; ++i)
                    {
                        out[i] = static_cast<float>(amplitude_ * std::sin(phase));
                        phase += increment;
                        if (phase >= two_pi)
                            phase -= two_pi;
                    }
                    phase_ = phase;
                    return true;
                }

                bool advance(int frame_count) noexcept override
                {
                    const double two_pi = 6.28318530717958647692;
                    const double increment = two_pi * frequency_ / sample_rate_;
                    phase_ += increment * frame_count;
                    phase_ = std::fmod(phase_, two_pi);
                    return true;
                }

            private:
                double frequency_ = 440.0;
                double amplitude_ = 1.0;
                double sample_rate_ = 48000.0;
                double phase_ = 0.0;
            };

        /**
         * @brief Plays (optionally loops) a caller-owned mono float buffer.
         *
         * The buffer is not owned — it must outlive the source (a resident sample or a
         * streaming ring in later phases). A non-looping source returns false from
         * @ref render / @ref advance once it runs off the end.
         */
        class BufferSource final : public VoiceSource
        {
            public:
                /**
                 * @brief Constructs a buffer source.
                 * @param data  Pointer to @p size mono samples (borrowed).
                 * @param size  Number of samples.
                 * @param loop  Whether to wrap at the end.
                 */
                BufferSource(const float* data, int size, bool loop) noexcept
                    : data_(data), size_(size), loop_(loop)
                {
                }

                void reset() noexcept override { position_ = 0; }

                bool render(float* out, int frame_count) noexcept override
                {
                    for (int i = 0; i < frame_count; ++i)
                    {
                        if (position_ >= size_)
                        {
                            if (loop_ && size_ > 0)
                                position_ = 0;
                            else
                            {
                                for (int j = i; j < frame_count; ++j)
                                    out[j] = 0.0f;
                                return false;
                            }
                        }
                        out[i] = data_[position_];
                        ++position_;
                    }
                    return true;
                }

                bool advance(int frame_count) noexcept override
                {
                    if (size_ <= 0)
                        return false;
                    if (loop_)
                    {
                        position_ = (position_ + frame_count) % size_;
                        return true;
                    }
                    position_ += frame_count;
                    if (position_ >= size_)
                    {
                        position_ = size_;
                        return false;
                    }
                    return true;
                }

            private:
                const float* data_ = nullptr;
                int size_ = 0;
                bool loop_ = false;
                int position_ = 0;
            };

        /** @brief Whether a voice slot is unused, tracked-but-silent, or fully rendered. */
        enum class VoiceState
        {
            Free,    /**< Unused pool slot. */
            Virtual, /**< Active but below the real cap: position bookkeeping only, no audio. */
            Real     /**< Active and audible: rendered through the full pipeline this block. */
        };

        /**
         * @brief The play parameters of a voice, set when it starts.
         *
         * @ref priority is the primary key the voice manager sorts on (higher wins a
         * real slot); @ref base_gain and the spatial fields feed the secondary
         * *audibility* key. When @ref spatial is false the voice is at full gain
         * regardless of position (2D UI/music).
         */
        struct VoiceDescriptor
        {
            float base_gain = 1.0f;        /**< Linear gain before attenuation. */
            float priority = 0.0f;         /**< Sort priority; higher keeps a real slot. */
            int bus = 0;                   /**< Target mixer bus id. */
            float pan = 0.0f;              /**< Stereo pan in [-1, 1] (2D placement). */
            bool spatial = false;          /**< Whether distance/propagation applies. */
            AudioVec3 position;            /**< World position (when spatial). */
            float min_distance = 1.0f;     /**< Full gain within this radius. */
            float max_distance = 100.0f;   /**< Silent (and cullable) beyond this radius. */

            DistanceModel model = DistanceModel::Linear; /**< The rolloff law. */
            float rolloff = 1.0f;          /**< Rolloff factor for inverse/exponent models. */
            float doppler_scale = 1.0f;    /**< Doppler exaggeration (0 = off, 1 = physical, >1 = more). */
            bool propagation_delay = true; /**< Whether the propagation delay (and thus Doppler) applies. */

            /**
             * @brief The distance attenuation for a listener at @p listener.
             * @param listener The listener position.
             * @return The @ref model's gain in [0, 1]; 1 for a non-spatial voice.
             */
            float attenuation(const AudioVec3& listener) const noexcept
            {
                if (!spatial)
                    return 1.0f;
                return distance_attenuation(model, distance(position, listener),
                                            min_distance, max_distance, rolloff);
            }
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
