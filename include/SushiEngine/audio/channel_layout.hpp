/**************************************************************************/
/* channel_layout.hpp                                                    */
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

#ifndef SUSHIENGINE_AUDIO_CHANNEL_LAYOUT_HPP
#define SUSHIENGINE_AUDIO_CHANNEL_LAYOUT_HPP

/**
 * @file channel_layout.hpp
 * @brief Output speaker layouts (mono … 7.1) and their speaker directions.
 *
 * Multichannel output needs to know *where* each device channel's speaker sits so the
 * ambisonic scene bus can be decoded to it (real surround, not a copy of the front
 * pair). Each layout is a table of unit speaker directions in the audio frame — **x =
 * front, y = left, z = up** — with one flagged as the LFE. Channel order follows the
 * common WAV/SMPTE convention (FL, FR, C, LFE, rears, sides), which is what SDL/most OS
 * mixers expect. Directions use standard cinema angles.
 */

#include <cstddef>

namespace SushiEngine
{
    namespace Audio
    {
        /** @brief A supported output speaker layout, selected by channel count. */
        enum class ChannelLayout
        {
            Mono,       /**< 1 channel. */
            Stereo,     /**< 2 channels (binaural / headphone path). */
            Quad,       /**< 4: FL, FR, RL, RR. */
            Surround51, /**< 6: FL, FR, C, LFE, SL, SR. */
            Surround71  /**< 8: FL, FR, C, LFE, RL, RR, SL, SR. */
        };

        /** @brief One output channel's speaker placement. */
        struct OutputSpeaker
        {
            float x;   /**< Front component (unit direction). */
            float y;   /**< Left component. */
            float z;   /**< Up component. */
            bool lfe;  /**< True for the low-frequency-effects channel (no direction). */
        };

        /** @brief The layout that matches a device channel count (Stereo for 2, etc.). */
        inline ChannelLayout layout_for(int channel_count) noexcept
        {
            switch (channel_count)
            {
                case 1: return ChannelLayout::Mono;
                case 4: return ChannelLayout::Quad;
                case 6: return ChannelLayout::Surround51;
                case 8: return ChannelLayout::Surround71;
                default: return ChannelLayout::Stereo;
            }
        }

        /**
         * @brief The speaker table for a layout (WAV/SMPTE channel order).
         * @param layout The layout.
         * @param count  Set to the number of speakers (channels).
         * @return A pointer to a static array of @p count @ref OutputSpeaker entries.
         */
        inline const OutputSpeaker* speakers_for(ChannelLayout layout, int& count) noexcept
        {
            // cos/sin of the cinema angles (azimuth from front, left positive).
            static const OutputSpeaker mono[] = {{1.0f, 0.0f, 0.0f, false}};
            static const OutputSpeaker stereo[] = {{0.866f, 0.5f, 0.0f, false},
                                                   {0.866f, -0.5f, 0.0f, false}};
            static const OutputSpeaker quad[] = {{0.866f, 0.5f, 0.0f, false},   // FL +30°
                                                 {0.866f, -0.5f, 0.0f, false},  // FR −30°
                                                 {-0.707f, 0.707f, 0.0f, false},// RL +135°
                                                 {-0.707f, -0.707f, 0.0f, false}}; // RR −135°
            static const OutputSpeaker s51[] = {{0.866f, 0.5f, 0.0f, false},    // FL +30°
                                                {0.866f, -0.5f, 0.0f, false},   // FR −30°
                                                {1.0f, 0.0f, 0.0f, false},      // C 0°
                                                {0.0f, 0.0f, 0.0f, true},       // LFE
                                                {-0.342f, 0.940f, 0.0f, false}, // SL +110°
                                                {-0.342f, -0.940f, 0.0f, false}};// SR −110°
            static const OutputSpeaker s71[] = {{0.866f, 0.5f, 0.0f, false},    // FL +30°
                                                {0.866f, -0.5f, 0.0f, false},   // FR −30°
                                                {1.0f, 0.0f, 0.0f, false},      // C 0°
                                                {0.0f, 0.0f, 0.0f, true},       // LFE
                                                {-0.866f, 0.5f, 0.0f, false},   // RL +150°
                                                {-0.866f, -0.5f, 0.0f, false},  // RR −150°
                                                {0.0f, 1.0f, 0.0f, false},      // SL +90°
                                                {0.0f, -1.0f, 0.0f, false}};    // SR −90°
            switch (layout)
            {
                case ChannelLayout::Mono: count = 1; return mono;
                case ChannelLayout::Quad: count = 4; return quad;
                case ChannelLayout::Surround51: count = 6; return s51;
                case ChannelLayout::Surround71: count = 8; return s71;
                case ChannelLayout::Stereo:
                default: count = 2; return stereo;
            }
        }
    } // namespace Audio
} // namespace SushiEngine

#endif
