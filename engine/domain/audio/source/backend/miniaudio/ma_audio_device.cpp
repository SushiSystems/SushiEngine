/**************************************************************************/
/* ma_audio_device.cpp                                                   */
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

#include "ma_audio_device.hpp"

#include <cstring>

#include "miniaudio.h" // declarations only; MA_IMPLEMENTATION lives in miniaudio_impl.cpp

namespace SushiEngine
{
    namespace Audio
    {
        namespace
        {
            void ma_data_callback(ma_device* device, void* output, const void* input,
                                  ma_uint32 frame_count)
            {
                (void)input;
                MaAudioDevice* self = static_cast<MaAudioDevice*>(device->pUserData);
                if (self != nullptr)
                    self->render_block(static_cast<float*>(output), static_cast<int>(frame_count));
            }
        } // namespace

        struct MaAudioDevice::Impl
        {
            ma_device device;
            bool inited = false;
        };

        MaAudioDevice::MaAudioDevice() : impl_(new Impl()) {}

        MaAudioDevice::~MaAudioDevice() { close(); }

        bool MaAudioDevice::open(const AudioStreamFormat& desired, IAudioRenderer& renderer)
        {
            close();

            ma_device_config config = ma_device_config_init(ma_device_type_playback);
            config.playback.format = ma_format_f32;
            config.playback.channels = static_cast<ma_uint32>(desired.channel_count);
            config.sampleRate = static_cast<ma_uint32>(desired.sample_rate);
            config.periodSizeInFrames = static_cast<ma_uint32>(desired.block_frames);
            config.dataCallback = ma_data_callback;
            config.pUserData = this;

            if (ma_device_init(nullptr, &config, &impl_->device) != MA_SUCCESS)
                return false;
            impl_->inited = true;

            renderer_ = &renderer;
            format_.sample_rate = static_cast<int>(impl_->device.sampleRate);
            format_.channel_count = static_cast<int>(impl_->device.playback.channels);
            format_.block_frames = desired.block_frames;

            planar_.assign(static_cast<std::size_t>(format_.channel_count) *
                               static_cast<std::size_t>(format_.block_frames),
                           0.0f);
            channel_ptrs_.resize(static_cast<std::size_t>(format_.channel_count));
            for (int c = 0; c < format_.channel_count; ++c)
                channel_ptrs_[static_cast<std::size_t>(c)] =
                    planar_.data() + static_cast<std::size_t>(c) * static_cast<std::size_t>(format_.block_frames);

            if (ma_device_start(&impl_->device) != MA_SUCCESS)
            {
                close();
                return false;
            }
            open_ = true;
            return true;
        }

        void MaAudioDevice::close() noexcept
        {
            if (impl_ && impl_->inited)
            {
                ma_device_uninit(&impl_->device); // stops and joins the callback thread
                impl_->inited = false;
            }
            renderer_ = nullptr;
            open_ = false;
        }

        void MaAudioDevice::render_block(float* interleaved, int frame_count) noexcept
        {
            const int channels = format_.channel_count;
            int n = frame_count;
            if (n > format_.block_frames)
                n = format_.block_frames;

            if (renderer_ != nullptr)
                renderer_->render(channel_ptrs_.data(), channels, n);
            else
                for (int c = 0; c < channels; ++c)
                    std::memset(channel_ptrs_[static_cast<std::size_t>(c)], 0,
                                sizeof(float) * static_cast<std::size_t>(n));

            for (int f = 0; f < n; ++f)
                for (int c = 0; c < channels; ++c)
                    interleaved[f * channels + c] = channel_ptrs_[static_cast<std::size_t>(c)][f];
            // Zero any frames beyond our prepared block (miniaudio asked for more than the
            // negotiated period): degrade to brief silence rather than emit garbage.
            for (int f = n; f < frame_count; ++f)
                for (int c = 0; c < channels; ++c)
                    interleaved[f * channels + c] = 0.0f;
        }
    } // namespace Audio
} // namespace SushiEngine
