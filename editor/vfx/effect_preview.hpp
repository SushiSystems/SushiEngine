/**************************************************************************/
/* effect_preview.hpp                                                     */
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
 * @file effect_preview.hpp
 * @brief The editor's live particle-effect preview and its emitter gizmo overlay.
 *
 * Holds one authored `Vfx::ParticleEffect` and its `EffectDatabase`, advances a play clock,
 * and each frame builds the `ParticleEmitterView`s the Scene viewport hands to the renderer —
 * computing the host-side spawn count (rate over time plus bursts) so the GPU emit shader stays
 * a pure allocator. Mirrors the animation subsystem's `SkeletonPreview`: a state-owning class
 * plus a free-function viewport overlay that paints the emitter's shape and origin.
 */

#include <cstddef>
#include <cstdint>
#include <vector>

#include <imgui.h>

#include <SushiEngine/core/types.hpp>
#include <SushiEngine/render/scene_view.hpp>
#include <SushiEngine/vfx/effect_database.hpp>
#include <SushiEngine/vfx/particle_effect.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief A live, authorable particle effect previewed in the Scene viewport.
         *
         * The panel edits @ref effect() (which marks the compiled form stale); @ref update
         * advances the clock and rebuilds the frame's emitter views; the viewport passes
         * @ref views() to the renderer.
         */
        class EffectPreview
        {
            public:
                /** @brief Builds a default fire-like effect and registers it. */
                EffectPreview();

                /**
                 * @brief The authored effect, for editing.
                 *
                 * Marks the compiled form stale so the next @ref update recompiles.
                 *
                 * @return The mutable effect.
                 */
                Vfx::ParticleEffect& effect() noexcept;

                /** @brief The authored effect (const). */
                const Vfx::ParticleEffect& effect() const noexcept;

                /** @brief Whether the preview is emitting. */
                bool playing() const noexcept { return playing_; }

                /** @brief Starts or stops emission. */
                void set_playing(bool playing) noexcept { playing_ = playing; }

                /** @brief Clears the play clock and spawn accumulators. */
                void restart() noexcept;

                /** @brief The emitter's world position. */
                Vector3 position() const noexcept { return position_; }

                /** @brief Moves the emitter. */
                void set_position(const Vector3& position) noexcept { position_ = position; }

                /**
                 * @brief Advances the clock and rebuilds this frame's emitter views.
                 * @param dt Seconds since the last frame.
                 */
                void update(float dt);

                /** @brief This frame's cosmetic emitter views, or nullptr when none. */
                const Render::ParticleEmitterView* views() const noexcept
                {
                    return views_.empty() ? nullptr : views_.data();
                }

                /** @brief Number of entries in @ref views. */
                std::size_t view_count() const noexcept { return views_.size(); }

            private:
                Vfx::EffectDatabase database_;
                Vfx::AssetId effect_id_ = Vfx::INVALID_EFFECT;
                Vector3 position_{Vector3{0, 1, 0}};
                bool playing_ = true;
                float time_ = 0.0f;
                std::vector<float> accumulators_;
                std::vector<Render::ParticleEmitterView> views_;
        };

        /**
         * @brief Paints the previewed emitter's origin and shape over the viewport image.
         *
         * Projects the emitter position through the panel camera and draws a marker plus a
         * ring sized to the emitter's shape radius, so the artist sees where particles spawn.
         *
         * @param preview      The previewed effect.
         * @param camera       The view/projection to project through.
         * @param image_origin The viewport image's top-left in screen space.
         * @param width        The viewport image width in pixels.
         * @param height       The viewport image height in pixels.
         * @param draw_list    The ImGui draw list to paint into.
         */
        void draw_emitter_gizmo(const EffectPreview& preview,
                                const Render::CameraView& camera, const ImVec2& image_origin,
                                float width, float height, ImDrawList* draw_list);
    } // namespace Editor
} // namespace SushiEngine
