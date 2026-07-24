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
        namespace
        {
            Vfx::ParticleEffect make_default_effect()
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
        } // namespace

        EffectPreview::EffectPreview()
        {
            effect_id_ = database_.add(make_default_effect());
        }

        Vfx::ParticleEffect& EffectPreview::effect() noexcept
        {
            return database_.effect_for_edit(effect_id_);
        }

        const Vfx::ParticleEffect& EffectPreview::effect() const noexcept
        {
            return database_.effect(effect_id_);
        }

        void EffectPreview::restart() noexcept
        {
            time_ = 0.0f;
            std::fill(accumulators_.begin(), accumulators_.end(), 0.0f);
        }

        void EffectPreview::update(float dt)
        {
            views_.clear();
            const Vfx::CompiledEffect& compiled = database_.compiled(effect_id_);
            if (compiled.emitters.empty())
                return;

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
