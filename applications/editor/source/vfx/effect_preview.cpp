/**************************************************************************/
/* effect_preview.cpp                                                     */
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

#include "effect_preview.hpp"

#include <algorithm>

#include <SushiEngine/vfx/compiled_emitter.hpp>
#include <SushiEngine/vfx/modules.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        Vfx::ParticleEffect default_emitter_effect()
        {
            Vfx::EmitterDescriptor emitter;
            emitter.name = "Fire";
            emitter.domain = Vfx::SimulationDomain::Cosmetic;
            emitter.capacity = 8192;
            emitter.spawn.rate_per_second = 180.0f;
            emitter.shape.shape = Vfx::EmitterShape::Cone;
            emitter.shape.radius = 0.25f;
            emitter.shape.cone_angle_radians = 0.35f;
            emitter.init.lifetime_min = 0.9f;
            emitter.init.lifetime_max = 1.8f;
            emitter.init.speed_min = 1.5f;
            emitter.init.speed_max = 3.5f;
            emitter.init.size_min = 0.05f;
            emitter.init.size_max = 0.14f;
            emitter.gravity.enabled = true;
            emitter.gravity.acceleration = Vector3{0, 3.5, 0}; // buoyant rise
            emitter.drag.enabled = true;
            emitter.drag.coefficient = 0.5f;
            emitter.turbulence.enabled = true;
            emitter.turbulence.frequency = 1.0f;
            emitter.turbulence.amplitude = 1.6f;
            emitter.size_over_life.enabled = true;
            emitter.size_over_life.curve.add_key(Vfx::CurveKey{0.0f, 0.3f, 0.0f, 1.5f});
            emitter.size_over_life.curve.add_key(Vfx::CurveKey{0.3f, 1.0f, 0.0f, 0.0f});
            emitter.size_over_life.curve.add_key(Vfx::CurveKey{1.0f, 0.0f, -1.0f, 0.0f});
            emitter.color_over_life.enabled = true;
            emitter.color_over_life.gradient.add_color_key(Vfx::ColorKey{0.0f, Vector3{1.0, 0.95, 0.5}});
            emitter.color_over_life.gradient.add_color_key(Vfx::ColorKey{0.5f, Vector3{1.0, 0.35, 0.08}});
            emitter.color_over_life.gradient.add_color_key(Vfx::ColorKey{1.0f, Vector3{0.15, 0.03, 0.02}});
            emitter.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{0.0f, 0.0f});
            emitter.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{0.12f, 1.0f});
            emitter.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{1.0f, 0.0f});
            emitter.render.blend = Vfx::BlendMode::Additive;

            Vfx::ParticleEffect effect;
            effect.name = "Campfire";
            effect.emitters.push_back(emitter);
            return effect;
        }

        namespace
        {
            /** @brief A spark burst: fast, gravity-pulled, short-lived. */
            Vfx::ParticleEffect make_spark_template()
            {
                Vfx::EmitterDescriptor e;
                e.name = "Sparks";
                e.domain = Vfx::SimulationDomain::Cosmetic;
                e.capacity = 4096;
                e.spawn.rate_per_second = 90.0f;
                e.shape.shape = Vfx::EmitterShape::Sphere;
                e.shape.radius = 0.05f;
                e.init.lifetime_min = 0.5f;
                e.init.lifetime_max = 1.1f;
                e.init.speed_min = 3.0f;
                e.init.speed_max = 6.0f;
                e.init.size_min = 0.02f;
                e.init.size_max = 0.05f;
                e.gravity.enabled = true;
                e.gravity.acceleration = Vector3{0, -9.0, 0};
                e.drag.enabled = true;
                e.drag.coefficient = 0.2f;
                e.color_over_life.enabled = true;
                e.color_over_life.gradient.add_color_key(Vfx::ColorKey{0.0f, Vector3{1.0, 0.95, 0.7}});
                e.color_over_life.gradient.add_color_key(Vfx::ColorKey{1.0f, Vector3{1.0, 0.4, 0.1}});
                e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{0.0f, 1.0f});
                e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{1.0f, 0.0f});
                // A spark is the case velocity stretching was made for.
                e.render.alignment = Vfx::RenderAlignment::VelocityStretched;
                e.render.velocity_stretch = 0.06f;

                Vfx::ParticleEffect effect;
                effect.name = "Sparks";
                effect.emitters.push_back(e);
                return effect;
            }

            /** @brief A smoke column: slow, swelling, alpha-fading — and therefore lit. */
            Vfx::ParticleEffect make_smoke_template()
            {
                Vfx::EmitterDescriptor e;
                e.name = "Smoke";
                e.domain = Vfx::SimulationDomain::Cosmetic;
                e.capacity = 4096;
                e.spawn.rate_per_second = 40.0f;
                e.shape.shape = Vfx::EmitterShape::Cone;
                e.shape.radius = 0.15f;
                e.shape.cone_angle_radians = 0.25f;
                e.init.lifetime_min = 2.0f;
                e.init.lifetime_max = 3.5f;
                e.init.speed_min = 0.6f;
                e.init.speed_max = 1.4f;
                e.init.size_min = 0.15f;
                e.init.size_max = 0.3f;
                e.gravity.enabled = true;
                e.gravity.acceleration = Vector3{0, 1.2, 0};
                e.drag.enabled = true;
                e.drag.coefficient = 0.6f;
                e.turbulence.enabled = true;
                e.turbulence.frequency = 0.5f;
                e.turbulence.amplitude = 0.8f;
                e.size_over_life.enabled = true;
                e.size_over_life.curve.add_key(Vfx::CurveKey{0.0f, 0.4f, 0.0f, 1.0f});
                e.size_over_life.curve.add_key(Vfx::CurveKey{1.0f, 1.6f, 0.0f, 0.0f});
                e.color_over_life.enabled = true;
                e.color_over_life.gradient.add_color_key(Vfx::ColorKey{0.0f, Vector3{0.35, 0.35, 0.38}});
                e.color_over_life.gradient.add_color_key(Vfx::ColorKey{1.0f, Vector3{0.12, 0.12, 0.13}});
                e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{0.0f, 0.0f});
                e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{0.15f, 0.55f});
                e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{1.0f, 0.0f});
                // True alpha, so it sorts and takes the sun and the clustered lights.
                e.render.blend = Vfx::BlendMode::Alpha;
                e.render.sort = Vfx::SortMode::ViewDistance;
                e.render.lit = true;

                Vfx::ParticleEffect effect;
                effect.name = "Smoke";
                effect.emitters.push_back(e);
                return effect;
            }

            /** @brief A trail, so the ribbon path has a starting point an author can reach. */
            Vfx::ParticleEffect make_trail_template()
            {
                Vfx::EmitterDescriptor e;
                e.name = "Trail";
                e.domain = Vfx::SimulationDomain::Cosmetic;
                e.capacity = 2048;
                e.spawn.rate_per_second = 25.0f;
                e.shape.shape = Vfx::EmitterShape::Sphere;
                e.shape.radius = 0.1f;
                e.init.lifetime_min = 1.0f;
                e.init.lifetime_max = 1.8f;
                e.init.speed_min = 2.0f;
                e.init.speed_max = 4.0f;
                e.init.size_min = 0.06f;
                e.init.size_max = 0.12f;
                e.gravity.enabled = true;
                e.gravity.acceleration = Vector3{0, -2.0, 0};
                e.turbulence.enabled = true;
                e.turbulence.frequency = 0.8f;
                e.turbulence.amplitude = 1.2f;
                e.color_over_life.enabled = true;
                e.color_over_life.gradient.add_color_key(Vfx::ColorKey{0.0f, Vector3{0.5, 0.85, 1.0}});
                e.color_over_life.gradient.add_color_key(Vfx::ColorKey{1.0f, Vector3{0.1, 0.25, 0.6}});
                e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{0.0f, 1.0f});
                e.color_over_life.gradient.add_alpha_key(Vfx::AlphaKey{1.0f, 0.0f});
                e.render.alignment = Vfx::RenderAlignment::Ribbon;

                Vfx::ParticleEffect effect;
                effect.name = "Trail";
                effect.emitters.push_back(e);
                return effect;
            }
        } // namespace

        const EffectTemplate* built_in_effect_templates(std::size_t& count)
        {
            static const EffectTemplate TEMPLATES[] = {{"Fire", &default_emitter_effect},
                                                       {"Sparks", &make_spark_template},
                                                       {"Smoke", &make_smoke_template},
                                                       {"Trail", &make_trail_template}};
            count = sizeof(TEMPLATES) / sizeof(TEMPLATES[0]);
            return TEMPLATES;
        }

        EffectPreview::EffectPreview()
        {
            effect_id_ = database_.add(default_emitter_effect());
        }

        Vfx::ParticleEffect& EffectPreview::effect() noexcept
        {
            return database_.effect_for_edit(effect_id_);
        }

        void EffectPreview::set_effect(const Vfx::ParticleEffect& effect)
        {
            database_.replace(effect_id_, effect);
        }

        const Vfx::ParticleEffect& EffectPreview::effect() const noexcept
        {
            return database_.effect(effect_id_);
        }

        void EffectPreview::restart() noexcept
        {
            time_ = 0.0f;
            step_carry_ = 0.0f;
            std::fill(accumulators_.begin(), accumulators_.end(), 0.0f);
            std::fill(states_.begin(), states_.end(), Vfx::DeterministicEmitterState{});
            billboards_.clear();
        }

        void EffectPreview::seek(float time)
        {
            time_ = time > 0.0f ? time : 0.0f;
            // The accumulators carry the fractional particle each emitter is owed at its rate.
            // Jumping the clock invalidates that debt, so it is cleared rather than carried across
            // a discontinuity where it would spill as a burst at the new time.
            std::fill(accumulators_.begin(), accumulators_.end(), 0.0f);
            step_carry_ = 0.0f;
            if (deterministic_)
                replay_deterministic();
        }

        void EffectPreview::set_deterministic(bool deterministic)
        {
            if (deterministic_ == deterministic)
                return;
            deterministic_ = deterministic;
            // The GPU pool and the CPU pools hold unrelated state, and neither can be handed to
            // the other, so switching starts the preview over rather than pretending to continue.
            restart();
        }

        void EffectPreview::replay_deterministic()
        {
            const Vfx::CompiledEffect& compiled = database_.compiled(effect_id_);
            states_.assign(compiled.emitters.size(), Vfx::DeterministicEmitterState{});
            billboards_.clear();
            if (compiled.emitters.empty())
                return;

            // A replay is bounded so that dragging the play head to the end of a long cycle cannot
            // stall the editor; past the cap the scrub is approximate rather than unresponsive.
            constexpr int MAX_REPLAY_STEPS = 4096;
            const int steps = std::min(static_cast<int>(time_ / DETERMINISTIC_STEP),
                                       MAX_REPLAY_STEPS);
            const Quaternion identity{0.0, 0.0, 0.0, 1.0};
            for (int step = 0; step < steps; ++step)
            {
                // Every emitter is stepped, whatever domain it declares. The domain says which
                // backend *ships* the effect; the preview's job is to show the author what this
                // same asset looks like through the other one.
                for (std::size_t i = 0; i < compiled.emitters.size(); ++i)
                {
                    Vfx::CpuDeterministicBackend::step(states_[i], compiled.emitters[i], compiled,
                                                       DETERMINISTIC_STEP, position_, identity);
                }
            }
            collect_billboards();
        }

        void EffectPreview::collect_billboards()
        {
            billboards_.clear();
            for (const Vfx::DeterministicEmitterState& state : states_)
            {
                for (std::uint32_t i = 0; i < state.alive_count; ++i)
                {
                    const Vfx::GpuParticle& particle = state.particles[i];
                    Render::ParticleBillboard billboard;
                    billboard.position =
                        Vector3{particle.position[0], particle.position[1], particle.position[2]};
                    billboard.color =
                        Vector3{particle.color[0], particle.color[1], particle.color[2]};
                    billboard.size = particle.size;
                    billboard.alpha = particle.alpha;
                    billboard.rotation = particle.rotation;
                    billboards_.push_back(billboard);
                }
            }
        }

        void EffectPreview::update(float dt)
        {
            views_.clear();
            const Vfx::CompiledEffect& compiled = database_.compiled(effect_id_);
            if (compiled.emitters.empty())
            {
                billboards_.clear();
                return;
            }

            if (deterministic_)
            {
                // A fixed step, carried across frames: the whole value of this backend is that the
                // same effect replays identically, which a frame-rate-dependent step would destroy.
                states_.resize(compiled.emitters.size());
                if (playing_)
                {
                    step_carry_ += dt;
                    const Quaternion identity{0.0, 0.0, 0.0, 1.0};
                    while (step_carry_ >= DETERMINISTIC_STEP)
                    {
                        step_carry_ -= DETERMINISTIC_STEP;
                        time_ += DETERMINISTIC_STEP;
                        for (std::size_t i = 0; i < compiled.emitters.size(); ++i)
                        {
                            Vfx::CpuDeterministicBackend::step(states_[i], compiled.emitters[i],
                                                               compiled, DETERMINISTIC_STEP,
                                                               position_, identity);
                        }
                    }
                }
                collect_billboards();
                return;
            }

            billboards_.clear();
            accumulators_.resize(compiled.emitters.size(), 0.0f);
            if (playing_)
                time_ += dt;

            const Mat4 model = translation(position_);
            const float* curve_luts = compiled.curve_luts.empty() ? nullptr : compiled.curve_luts.data();
            const float* gradient_luts =
                compiled.gradient_luts.empty() ? nullptr : compiled.gradient_luts.data();

            for (std::size_t i = 0; i < compiled.emitters.size(); ++i)
            {
                const Vfx::CompiledEmitter& emitter = compiled.emitters[i];
                if (emitter.domain != Vfx::SimulationDomain::Cosmetic)
                    continue;

                std::uint32_t spawn_count = 0;
                if (playing_ && emitter.spawn_rate > 0.0f)
                {
                    accumulators_[i] += emitter.spawn_rate * dt;
                    spawn_count = static_cast<std::uint32_t>(accumulators_[i]);
                    accumulators_[i] -= static_cast<float>(spawn_count);
                    spawn_count = std::min(spawn_count, emitter.capacity);
                }

                Render::ParticleEmitterView view;
                view.model = model;
                view.compiled = &emitter;
                view.curve_luts = curve_luts;
                view.gradient_luts = gradient_luts;
                view.curve_lut_floats = static_cast<std::uint32_t>(compiled.curve_luts.size());
                view.gradient_lut_floats = static_cast<std::uint32_t>(compiled.gradient_luts.size());
                view.spawn_count = spawn_count;
                view.seed = 0x9E3779B9u + static_cast<std::uint32_t>(i);
                view.dt = dt;
                view.id = 0;
                views_.push_back(view);
            }
        }

        void draw_emitter_gizmo(const EffectPreview& preview, const Render::CameraView& camera,
                                const ImVec2& image_origin, float width, float height,
                                ImDrawList* draw_list)
        {
            const Mat4 view_projection = mul(camera.projection, camera.view);
            const Vector3 p = preview.position();
            const double clip_x = view_projection.m[0] * p.x + view_projection.m[4] * p.y +
                                  view_projection.m[8] * p.z + view_projection.m[12];
            const double clip_y = view_projection.m[1] * p.x + view_projection.m[5] * p.y +
                                  view_projection.m[9] * p.z + view_projection.m[13];
            const double clip_w = view_projection.m[3] * p.x + view_projection.m[7] * p.y +
                                  view_projection.m[11] * p.z + view_projection.m[15];
            if (clip_w <= 0.0)
                return; // behind the camera

            const float ndc_x = static_cast<float>(clip_x / clip_w);
            const float ndc_y = static_cast<float>(clip_y / clip_w);
            const ImVec2 screen(image_origin.x + (ndc_x * 0.5f + 0.5f) * width,
                                image_origin.y + (ndc_y * 0.5f + 0.5f) * height);

            const ImU32 color = IM_COL32(255, 180, 60, 220);
            draw_list->AddCircle(screen, 10.0f, color, 24, 1.5f);
            draw_list->AddLine(ImVec2(screen.x - 8.0f, screen.y), ImVec2(screen.x + 8.0f, screen.y),
                               color, 1.5f);
            draw_list->AddLine(ImVec2(screen.x, screen.y - 8.0f), ImVec2(screen.x, screen.y + 8.0f),
                               color, 1.5f);
            draw_list->AddText(ImVec2(screen.x + 12.0f, screen.y - 6.0f), color, "Emitter");
        }
    } // namespace Editor
} // namespace SushiEngine
