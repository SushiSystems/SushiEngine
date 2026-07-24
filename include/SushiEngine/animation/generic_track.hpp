/**************************************************************************/
/* generic_track.hpp                                                     */
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
 * @file generic_track.hpp
 * @brief Generic float tracks and the binding registry that routes them (phase A7).
 *
 * Besides posing joints and morphs, a clip can drive arbitrary named float properties — a
 * material's emissive strength, a UI value, a gameplay scalar — as property-hash-addressed
 * generic tracks (design §4.2). They bind at load through a small registry (no reflection): a
 * consumer registers a sink for a property hash, and @ref apply_generic_tracks samples the clip
 * and dispatches each value to its sink. The @ref IFloatSink seam keeps the animation layer
 * ignorant of what a value means — a material, a widget, or a script all implement the same one
 * method (DIP).
 */

#include <cstdint>
#include <vector>

#include <SushiEngine/animation/clip.hpp>
#include <SushiEngine/animation/hash.hpp>

namespace SushiEngine
{
    namespace Animation
    {
        /** @brief Max generic tracks sampled from one clip in a dispatch. */
        constexpr std::uint32_t MAX_GENERIC_TRACKS = 64;

        /** @brief The seam a generic track's value is delivered through (material/UI/script). */
        class IFloatSink
        {
            public:
                virtual ~IFloatSink() = default;

                /**
                 * @brief Delivers one sampled generic value.
                 * @param property The FNV-1a 64 hash of the property name.
                 * @param value    The value sampled from the clip at the current time.
                 */
                virtual void set_value(NameHash property, float value) = 0;
        };

        /**
         * @brief Samples a clip's generic tracks and dispatches each to a sink.
         * @param clip         The clip whose generic tracks are sampled.
         * @param time_seconds Playback time.
         * @param loop         Whether the clip loops.
         * @param sink         Receives one @c set_value per generic track.
         */
        inline void apply_generic_tracks(const ClipView& clip, float time_seconds, bool loop,
                                         IFloatSink& sink)
        {
            if (!clip.valid() || clip.generic_track_count == 0)
                return;
            std::uint32_t tracks = clip.generic_track_count;
            if (tracks > MAX_GENERIC_TRACKS)
                tracks = MAX_GENERIC_TRACKS;
            float values[MAX_GENERIC_TRACKS];
            clip.sample_generic(time_seconds, loop, values);
            for (std::uint32_t t = 0; t < tracks; ++t)
                sink.set_value(clip.generic_names[t], values[t]);
        }

        /**
         * @brief A concrete sink that writes each bound property's value to a float pointer.
         *
         * The small "bound at load" registry: a consumer @ref bind s a property hash to a float
         * it owns; a value for an unbound property is dropped. Non-owning — the targets outlive it.
         */
        class GenericBindingRegistry : public IFloatSink
        {
            public:
                /**
                 * @brief Binds a property name hash to a float the caller owns.
                 * @param property The FNV-1a 64 hash of the property name.
                 * @param target   The float to receive its value (must outlive this registry).
                 */
                void bind(NameHash property, float* target)
                {
                    bindings_.push_back(Binding{property, target});
                }

                void set_value(NameHash property, float value) override
                {
                    for (const Binding& binding : bindings_)
                        if (binding.property == property && binding.target != nullptr)
                            *binding.target = value;
                }

            private:
                struct Binding
                {
                    NameHash property = 0;
                    float* target = nullptr;
                };
                std::vector<Binding> bindings_;
        };
    } // namespace Animation
} // namespace SushiEngine
