/**************************************************************************/
/* engine.hpp                                                            */
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

#ifndef SUSHIENGINE_AUDIO_ENGINE_HPP
#define SUSHIENGINE_AUDIO_ENGINE_HPP

/**
 * @file engine.hpp
 * @brief The audio engine: the render-plane entry point that ties voices to the mixer.
 *
 * @ref AudioEngine is the @ref IAudioRenderer the device drives (§12). Its @ref render
 * is the top of the audio-render plane: it sets the denormal guard for the whole
 * callback, clears the mixer accumulators, folds the voice manager's real voices into
 * the buses, runs the bus graph, and fans the stereo master out to the device
 * channels. Everything below it is allocation-, lock-, and syscall-free.
 *
 * This is the S2 capstone; parameter changes reach it through the atomics on
 * @ref SmoothedValue / voice descriptors. The batched command ring (§0) that lets the
 * control thread also *start and stop* voices mid-stream layers on in a later phase —
 * for now voices are set up before the device opens.
 */

#include <vector>

#include <SushiEngine/audio/device.hpp>
#include <SushiEngine/audio/dsp/denormals.hpp>
#include <SushiEngine/audio/dsp/simd.hpp>
#include <SushiEngine/audio/mixer.hpp>
#include <SushiEngine/audio/spatializer.hpp>
#include <SushiEngine/audio/voice_manager.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief Owns a voice manager and a mixer and renders them as one @ref IAudioRenderer.
         *
         * Build the mixer's bus topology and the voices through @ref mixer and
         * @ref voices, call @ref prepare once, then hand the engine to an
         * @ref IAudioDevice — or call @ref render directly for headless processing.
         */
        class AudioEngine final : public IAudioRenderer
        {
            public:
                /**
                 * @brief Constructs the engine and its voice pool.
                 * @param voice_pool_capacity Maximum simultaneously-active voices.
                 * @param max_real_voices     Maximum voices rendered per block.
                 */
                AudioEngine(int voice_pool_capacity, int max_real_voices)
                    : voices_(voice_pool_capacity, max_real_voices)
                {
                }

                /** @brief The mixer bus graph, for building topology and setting bus gains. */
                MixerGraph& mixer() noexcept { return mixer_; }

                /** @brief The voice manager, for starting/stopping voices and setting the listener. */
                VoiceManager& voices() noexcept { return voices_; }

                /** @brief The binaural spatializer spatial voices are rendered through. */
                BinauralSpatializer& spatializer() noexcept { return spatializer_; }

                /**
                 * @brief Sets the ambisonic order of the scene bus (call before @ref prepare).
                 * @param order The order (1..3 typical; higher sharpens localisation at more cost).
                 */
                void set_ambisonic_order(int order) noexcept { ambisonic_order_ = order; }

                /**
                 * @brief Prepares the mixer, spatializer, and voice manager for a run.
                 * @param sample_rate      The stream sample rate in Hz.
                 * @param max_block_frames The largest block that will be rendered.
                 */
                void prepare(double sample_rate, int max_block_frames)
                {
                    max_block_ = max_block_frames;
                    mixer_.prepare(sample_rate, max_block_frames);
                    spatializer_.configure(ambisonic_order_, sample_rate, max_block_frames);
                    voices_.prepare(sample_rate, max_block_frames);
                    voices_.set_spatializer(&spatializer_);
                    binaural_left_.assign(static_cast<std::size_t>(max_block_frames), 0.0f);
                    binaural_right_.assign(static_cast<std::size_t>(max_block_frames), 0.0f);
                }

                /**
                 * @brief Renders one block: voices → mixer → device channels.
                 *
                 * Fans the stereo master to the output: channel 0 = left, channel 1 =
                 * right, any further channels get the left as a safe default until the
                 * multichannel decode of §4 lands. A mono device gets the left channel.
                 *
                 * A device may hand a larger block than @ref prepare was told (the OS
                 * mixer's buffer can exceed the requested size); the internal render is
                 * clamped to the prepared maximum and any surplus device samples are
                 * zero-filled, so an under-sized `prepare` degrades to a brief silence
                 * rather than a buffer overrun. Size `prepare` at or above the device
                 * block to avoid that.
                 *
                 * @param channels      The device's planar output buffers.
                 * @param channel_count The device channel count.
                 * @param frame_count   Number of samples this block.
                 */
                void render(float* const* channels, int channel_count, int frame_count) noexcept override
                {
                    Dsp::ScopedNoDenormals guard;

                    const int n = frame_count < max_block_ ? frame_count : max_block_;

                    spatializer_.begin_block(n);
                    mixer_.begin_block(n);
                    voices_.render(mixer_, n); // spatial → spatializer, non-spatial → mixer
                    mixer_.process(n);

                    // Decode the ambisonic scene to the ears and sum it with the stereo
                    // (non-spatial) master.
                    Dsp::Simd::fill(binaural_left_.data(), n, 0.0f);
                    Dsp::Simd::fill(binaural_right_.data(), n, 0.0f);
                    spatializer_.decode_binaural(binaural_left_.data(), binaural_right_.data(), n);

                    const float* left = mixer_.master_left();
                    const float* right = mixer_.master_right();
                    for (int c = 0; c < channel_count; ++c)
                    {
                        const float* master = (c == 1) ? right : left;
                        const float* binaural = (c == 1) ? binaural_right_.data() : binaural_left_.data();
                        for (int i = 0; i < n; ++i)
                            channels[c][i] = master[i] + binaural[i];
                        for (int i = n; i < frame_count; ++i)
                            channels[c][i] = 0.0f;
                    }
                }

            private:
                MixerGraph mixer_;
                BinauralSpatializer spatializer_;
                VoiceManager voices_;
                int ambisonic_order_ = 3;
                int max_block_ = 0;
                std::vector<float> binaural_left_;
                std::vector<float> binaural_right_;
            };
    } // namespace Audio
} // namespace SushiEngine

#endif
