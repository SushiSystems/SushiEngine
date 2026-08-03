/**************************************************************************/
/* ma_audio_device.hpp                                                   */
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

#ifndef SUSHIENGINE_AUDIO_MA_AUDIO_DEVICE_HPP
#define SUSHIENGINE_AUDIO_MA_AUDIO_DEVICE_HPP

/**
 * @file ma_audio_device.hpp
 * @brief A miniaudio-backed @ref IAudioDevice — the low-latency platform backend.
 *
 * The alternative to @ref SDLAudioDevice behind the same @ref IAudioDevice seam (§12.1):
 * miniaudio drives the native low-latency API on each platform (WASAPI on Windows,
 * CoreAudio on macOS, ALSA/PulseAudio/PipeWire/JACK on Linux) with a single vendored
 * header and no extra linked dependency, and supports exclusive-mode/shared-mode and
 * tighter buffer control than SDL's shared callback. The mix is unchanged — the engine
 * renders into planar buffers exactly as with SDL; only the device I/O differs.
 *
 * miniaudio's types never appear here (they live behind a pimpl in the .cpp), so a
 * consumer includes only this and `device.hpp`, exactly like the SDL backend.
 */

#include <memory>
#include <vector>

#include <SushiEngine/audio/device.hpp>

namespace SushiEngine
{
    namespace Audio
    {
        /**
         * @brief An @ref IAudioDevice over miniaudio's native low-latency backends.
         *
         * @ref open negotiates a playback device (float, the requested rate/channels/period)
         * and starts miniaudio's callback, which drives the @ref IAudioRenderer once per
         * period; @ref close stops and tears it down. The obtained format is read back into
         * @ref format after a successful open.
         */
        class MaAudioDevice final : public IAudioDevice
        {
            public:
                MaAudioDevice();
                ~MaAudioDevice() override;

                MaAudioDevice(const MaAudioDevice&) = delete;
                MaAudioDevice& operator=(const MaAudioDevice&) = delete;

                bool open(const AudioStreamFormat& desired, IAudioRenderer& renderer) override;
                void close() noexcept override;
                bool is_running() const noexcept override { return open_; }
                AudioStreamFormat format() const noexcept override { return format_; }

                /** @brief Fills @p interleaved with @p frame_count rendered frames (callback use). */
                void render_block(float* interleaved, int frame_count) noexcept;

            private:
                struct Impl;
                std::unique_ptr<Impl> impl_;
                AudioStreamFormat format_{};
                IAudioRenderer* renderer_ = nullptr;
                std::vector<float> planar_;
                std::vector<float*> channel_ptrs_;
                bool open_ = false;
        };
    } // namespace Audio
} // namespace SushiEngine

#endif
