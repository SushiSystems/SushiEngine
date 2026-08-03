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
 * Holds one authored `VFX::ParticleEffect` and its `EffectDatabase`, advances a play clock,
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
#include <SushiEngine/vfx/deterministic_backend.hpp>
#include <SushiEngine/vfx/effect_database.hpp>
#include <SushiEngine/vfx/particle_effect.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        /**
         * @brief The effect a freshly added Particle Emitter starts from.
         *
         * A small fire. An emitter that starts empty shows nothing, which reads as a broken
         * component rather than as an invitation to author, so a new one begins somewhere visible.
         *
         * @return The default authored effect.
         */
        VFX::ParticleEffect default_emitter_effect();

        /** @brief One starting point offered in the effect library. */
        struct EffectTemplate
        {
            const char* name;         /**< Label shown in the library list. */
            VFX::ParticleEffect (*build)(); /**< Builds a fresh copy. */
        };

        /**
         * @brief The effects an author can start from without having saved anything yet.
         *
         * Authored data, so it belongs to the editor rather than to the simulation: the sim knows
         * only the one default a new emitter is seeded with. Each entry is *copied* into the
         * selected emitter, never referenced.
         *
         * @param count Receives the number of entries.
         * @return The template table.
         */
        const EffectTemplate* built_in_effect_templates(std::size_t& count);

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
                VFX::ParticleEffect& effect() noexcept;

                /** @brief The authored effect (const). */
                const VFX::ParticleEffect& effect() const noexcept;

                /**
                 * @brief Mirrors @p effect onto the preview surface.
                 *
                 * The preview does not own what it shows: the Particle Editor edits an emitter
                 * *component's* own effect, and this is how that reaches the isolated view. Nothing
                 * is restarted — an author dragging a slider wants the running effect to change,
                 * not to jump back to its first frame.
                 *
                 * @param effect The effect to show; copied.
                 */
                void set_effect(const VFX::ParticleEffect& effect);

                /** @brief Whether the preview is emitting. */
                bool playing() const noexcept { return playing_; }

                /** @brief Starts or stops emission. */
                void set_playing(bool playing) noexcept { playing_ = playing; }

                /** @brief Clears the play clock and spawn accumulators. */
                void restart() noexcept;

                /** @brief Seconds since the effect started, the timeline's play head. */
                float time() const noexcept { return time_; }

                /**
                 * @brief Moves the play head to @p time seconds.
                 *
                 * In the GPU preview this seeks the **emission schedule** only: the rate and burst
                 * evaluation jump to the new time, but the particles already alive are not
                 * re-simulated, because the cosmetic pool lives on the GPU and advances one step
                 * per rendered frame — there is no host copy to wind back.
                 *
                 * In the @ref deterministic preview it is a **true scrub**. That backend is a pure
                 * function of (state, emitter, dt), so the state is reset and replayed from zero to
                 * @p time at the fixed step, reproducing exactly the frame the author would have
                 * seen at that moment.
                 *
                 * @param time Seconds from the effect's start; negative values clamp to zero.
                 */
                void seek(float time);

                /**
                 * @brief Whether the preview simulates on the CPU rather than on the GPU.
                 *
                 * Both backends read the same authored effect, so either way this previews the same
                 * asset. The deterministic one is pool-capped and CPU-bound but can be scrubbed and
                 * stepped exactly; the cosmetic one scales to millions but only moves forward, one
                 * step per rendered frame.
                 */
                bool deterministic() const noexcept { return deterministic_; }

                /** @brief Switches backend and restarts, since the two states do not translate. */
                void set_deterministic(bool deterministic);

                /** @brief The fixed step the deterministic preview simulates and replays at. */
                static constexpr float DETERMINISTIC_STEP = 1.0f / 60.0f;

                /** @brief This frame's CPU-simulated particles, or nullptr when none. */
                const Render::ParticleBillboard* billboards() const noexcept
                {
                    return billboards_.empty() ? nullptr : billboards_.data();
                }

                /** @brief Number of entries in @ref billboards. */
                std::size_t billboard_count() const noexcept { return billboards_.size(); }

                /**
                 * @brief Whether the previewed effect is also drawn in the Scene view.
                 *
                 * Off by default, and deliberately opt-in: the Scene view shows the world, and a
                 * previewed effect belongs to no entity, so leaving it on permanently is what made
                 * the old preview read as a stray object nobody could select. Turned on, it is a
                 * second look at what is being authored, in the scene's own lighting.
                 */
                bool scene_preview() const noexcept { return scene_preview_; }

                /** @brief Shows or hides the previewed effect in the Scene view. */
                void set_scene_preview(bool value) noexcept { scene_preview_ = value; }

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
                /**
                 * @brief Replays the deterministic pools from zero up to @ref time_.
                 *
                 * The whole point of the deterministic backend: replay is the only way to reach an
                 * arbitrary time, and it is exact because the step is a pure function.
                 */
                void replay_deterministic();

                /** @brief Rebuilds @ref billboards_ from the deterministic pools. */
                void collect_billboards();

                VFX::EffectDatabase database_;
                VFX::AssetId effect_id_ = VFX::INVALID_EFFECT;
                Vector3 position_{Vector3{0, 1, 0}};
                bool playing_ = true;
                bool deterministic_ = false;
                bool scene_preview_ = false;
                float time_ = 0.0f;
                float step_carry_ = 0.0f;
                std::vector<float> accumulators_;
                std::vector<Render::ParticleEmitterView> views_;
                std::vector<VFX::DeterministicEmitterState> states_;
                std::vector<Render::ParticleBillboard> billboards_;
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
