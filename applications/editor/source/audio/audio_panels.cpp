/**************************************************************************/
/* audio_panels.cpp                                                      */
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

#include "audio_panels.hpp"

#include <cmath>
#include <cstdint>

#include <imgui.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            float clamp01(float v) { return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v); }

            // A horizontal level meter: a dark trough, a level-coloured fill (green → amber
            // → red), and a bright peak-hold tick. Levels above 1.0 (clipping) pin the fill
            // full and force red. Advances the ImGui cursor by its own size.
            void level_meter(const char* id, float rms, float peak, float width, float height)
            {
                ImGui::PushID(id);
                const ImVec2 origin = ImGui::GetCursorScreenPos();
                ImDrawList* draw = ImGui::GetWindowDrawList();
                const ImVec2 end(origin.x + width, origin.y + height);
                draw->AddRectFilled(origin, end, IM_COL32(28, 28, 32, 255), 3.0f);

                const float r = clamp01(rms);
                const float pk = clamp01(peak);
                const bool clipping = peak >= 0.999f;
                ImU32 col = IM_COL32(70, 200, 90, 255);
                if (clipping || r > 0.85f)
                    col = IM_COL32(230, 70, 60, 255);
                else if (r > 0.6f)
                    col = IM_COL32(225, 200, 70, 255);
                if (r > 0.001f)
                    draw->AddRectFilled(origin, ImVec2(origin.x + width * r, end.y), col, 3.0f);

                const float px = origin.x + width * pk;
                draw->AddLine(ImVec2(px, origin.y), ImVec2(px, end.y), IM_COL32(240, 240, 255, 220), 1.5f);
                draw->AddRect(origin, end, IM_COL32(70, 70, 80, 255), 3.0f);
                ImGui::Dummy(ImVec2(width, height));
                ImGui::PopID();
            }

            // A vertical channel-strip meter, for the mixer.
            void level_meter_vertical(const char* id, float rms, float peak, float width, float height)
            {
                ImGui::PushID(id);
                const ImVec2 origin = ImGui::GetCursorScreenPos();
                ImDrawList* draw = ImGui::GetWindowDrawList();
                const ImVec2 end(origin.x + width, origin.y + height);
                draw->AddRectFilled(origin, end, IM_COL32(28, 28, 32, 255), 2.0f);

                const float r = clamp01(rms);
                const float pk = clamp01(peak);
                const bool clipping = peak >= 0.999f;
                ImU32 col = IM_COL32(70, 200, 90, 255);
                if (clipping || r > 0.85f)
                    col = IM_COL32(230, 70, 60, 255);
                else if (r > 0.6f)
                    col = IM_COL32(225, 200, 70, 255);
                if (r > 0.001f)
                    draw->AddRectFilled(ImVec2(origin.x, end.y - height * r), end, col, 2.0f);
                const float py = end.y - height * pk;
                draw->AddLine(ImVec2(origin.x, py), ImVec2(end.x, py), IM_COL32(240, 240, 255, 220), 1.5f);
                draw->AddRect(origin, end, IM_COL32(70, 70, 80, 255), 2.0f);
                ImGui::Dummy(ImVec2(width, height));
                ImGui::PopID();
            }
        } // namespace

        void draw_audio_mixer_panel(EditorContext& context, AudioEditorSystem& audio)
        {
            if (!context.panels.audio_mixer)
                return;
            if (!ImGui::Begin("Audio Mixer", &context.panels.audio_mixer))
            {
                ImGui::End();
                return;
            }

            bool on = audio.enabled();
            if (ImGui::Checkbox("Audio engine", &on))
            {
                const bool ok = audio.set_enabled(on);
                editor_log(context, on ? (ok ? "Audio: engine on." : "Audio: no output device available.")
                                       : "Audio: engine off.");
            }
            ImGui::SameLine();
            if (audio.device_open())
                ImGui::TextColored(ImVec4(0.4f, 0.85f, 0.45f, 1.0f), "device open");
            else
                ImGui::TextDisabled("device closed");

            ImGui::Separator();

            const Audio::AudioProfileSnapshot& p = audio.profile();
            const int buses[4] = {audio.master_bus(), audio.sfx_bus(), audio.music_bus(),
                                  audio.reverb_bus()};

            ImGui::BeginChild("strips", ImVec2(0, 240), false, ImGuiWindowFlags_HorizontalScrollbar);
            for (int i = 0; i < 4; ++i)
            {
                const int bus = buses[i];
                ImGui::PushID(bus);
                ImGui::BeginGroup();
                ImGui::TextUnformatted(audio.bus_name(bus));

                float peak = 0.0f, rms = 0.0f;
                if (bus >= 0 && bus < p.bus_count)
                {
                    peak = p.buses[bus].peak;
                    rms = p.buses[bus].rms;
                }
                level_meter_vertical("m", rms, peak, 18.0f, 150.0f);
                ImGui::SameLine();

                float gain = audio.bus_gain(bus);
                if (ImGui::VSliderFloat("##g", ImVec2(28.0f, 150.0f), &gain, 0.0f, 2.0f, ""))
                    audio.set_bus_gain(bus, gain);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s gain: %.2f", audio.bus_name(bus), gain);

                char db[24];
                const float g = gain > 1.0e-4f ? gain : 1.0e-4f;
                std::snprintf(db, sizeof(db), "%+.1f dB", 20.0f * std::log10(g));
                ImGui::TextUnformatted(db);
                ImGui::EndGroup();
                ImGui::PopID();
                if (i < 3)
                    ImGui::SameLine(0.0f, 24.0f);
            }
            ImGui::EndChild();

            ImGui::Separator();
            ImGui::Text("Voices: %d real  ·  %d virtual  ·  %d active", p.real_voices,
                        p.virtual_voices, p.active_voices);
            ImGui::TextDisabled("Reverb bus carries the per-zone FDN; emitters aux-send into it.");
            ImGui::End();
        }

        void draw_audio_profiler_panel(EditorContext& context, AudioEditorSystem& audio)
        {
            if (!context.panels.audio_profiler)
                return;
            if (!ImGui::Begin("Audio Profiler", &context.panels.audio_profiler))
            {
                ImGui::End();
                return;
            }

            const Audio::AudioProfileSnapshot& p = audio.profile();
            if (!audio.device_open())
                ImGui::TextDisabled("Audio engine off — enable it in the Audio Mixer to see live telemetry.");

            ImGui::Text("Block %llu", static_cast<unsigned long long>(p.block_index));
            ImGui::Separator();

            ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "Voices");
            ImGui::Text("real %d", p.real_voices);
            ImGui::SameLine(120);
            ImGui::Text("virtual %d", p.virtual_voices);
            ImGui::SameLine(240);
            ImGui::Text("active %d", p.active_voices);

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "Master");
            ImGui::TextUnformatted("peak");
            ImGui::SameLine(60);
            level_meter("mp", p.master_peak, p.master_peak, 260.0f, 14.0f);
            ImGui::TextUnformatted("rms");
            ImGui::SameLine(60);
            level_meter("mr", p.master_rms, p.master_peak, 260.0f, 14.0f);

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "Buses");
            for (int b = 0; b < p.bus_count; ++b)
            {
                ImGui::Text("%d", b);
                ImGui::SameLine(60);
                char id[8];
                std::snprintf(id, sizeof(id), "b%d", b);
                level_meter(id, p.buses[b].rms, p.buses[b].peak, 240.0f, 12.0f);
            }

            ImGui::Spacing();
            ImGui::TextColored(ImVec4(0.55f, 0.8f, 1.0f, 1.0f), "Output scope");
            if (p.scope_points > 0)
                ImGui::PlotLines("##scope", p.scope, p.scope_points, 0, nullptr, -1.0f, 1.0f,
                                 ImVec2(-1.0f, 80.0f));
            else
                ImGui::TextDisabled("(silent)");

            ImGui::End();
        }

        void draw_audio_emitter_inspector(EditorContext& context, Simulation::IWorldEditor& world,
                                          Simulation::EntityId id, AudioEditorSystem& audio)
        {
            if (!world.has_audio_emitter(id))
                return;
            if (!ImGui::CollapsingHeader("Audio Emitter", ImGuiTreeNodeFlags_DefaultOpen))
                return;

            Simulation::AudioEmitterParameters p = world.audio_emitter_parameters(id);

            auto slider = [&](const char* label, float* v, float lo, float hi, const char* fmt) {
                const bool ch = ImGui::SliderFloat(label, v, lo, hi, fmt);
                if (ImGui::IsItemActivated())
                    context.history.begin_change(world);
                if (ImGui::IsItemDeactivatedAfterEdit())
                    context.history.end_change();
                if (ch)
                    world.set_audio_emitter_parameters(id, p);
            };

            int sound = static_cast<int>(p.sound);
            if (ImGui::InputInt("Sound id", &sound))
            {
                context.history.record(world);
                p.sound = static_cast<std::uint32_t>(sound < 0 ? 0 : sound);
                world.set_audio_emitter_parameters(id, p);
            }
            ImGui::SameLine();
            if (ImGui::SmallButton("Play"))
            {
                if (audio.device_open())
                    audio.preview(p.sound, p.gain);
                else
                    editor_log(context, "Audio: enable the engine (Audio Mixer) to audition.");
            }

            slider("Gain", &p.gain, 0.0f, 2.0f, "%.2f");
            slider("Priority", &p.priority, 0.0f, 100.0f, "%.0f");

            const char* buses[] = {"Master", "SFX", "Music", "Reverb"};
            int bus = static_cast<int>(p.bus < 4 ? p.bus : 1);
            if (ImGui::Combo("Bus", &bus, buses, IM_ARRAYSIZE(buses)))
            {
                context.history.record(world);
                p.bus = static_cast<std::uint32_t>(bus);
                world.set_audio_emitter_parameters(id, p);
            }

            bool spatial = p.spatial;
            if (ImGui::Checkbox("3D (spatial)", &spatial))
            {
                context.history.record(world);
                p.spatial = spatial;
                world.set_audio_emitter_parameters(id, p);
            }
            ImGui::SameLine();
            bool playing = p.playing;
            if (ImGui::Checkbox("Playing", &playing))
            {
                context.history.record(world);
                p.playing = playing;
                world.set_audio_emitter_parameters(id, p);
            }
            ImGui::SameLine();
            bool looping = p.looping;
            if (ImGui::Checkbox("Loop", &looping))
            {
                context.history.record(world);
                p.looping = looping;
                world.set_audio_emitter_parameters(id, p);
            }

            if (spatial)
            {
                const char* models[] = {"Linear", "Inverse", "Exponent"};
                int model = static_cast<int>(p.distance_model < 3 ? p.distance_model : 0);
                if (ImGui::Combo("Rolloff model", &model, models, IM_ARRAYSIZE(models)))
                {
                    context.history.record(world);
                    p.distance_model = static_cast<std::uint32_t>(model);
                    world.set_audio_emitter_parameters(id, p);
                }
                slider("Min distance", &p.min_distance, 0.1f, 100.0f, "%.1f m");
                slider("Max distance", &p.max_distance, 1.0f, 500.0f, "%.1f m");
                slider("Rolloff", &p.rolloff, 0.1f, 8.0f, "%.2f");
                slider("Doppler", &p.doppler_scale, 0.0f, 4.0f, "%.2f");

                // The attenuation curve the current model/distances produce.
                float curve[64];
                const float span = p.max_distance * 1.1f;
                for (int i = 0; i < 64; ++i)
                {
                    const float d = span * static_cast<float>(i) / 63.0f;
                    curve[i] = Audio::distance_attenuation(
                        static_cast<Audio::DistanceModel>(p.distance_model < 3 ? p.distance_model : 0),
                        d, p.min_distance, p.max_distance, p.rolloff);
                }
                ImGui::TextDisabled("Attenuation over distance");
                ImGui::PlotLines("##atten", curve, 64, 0, nullptr, 0.0f, 1.0f, ImVec2(-1.0f, 60.0f));
            }

            slider("Reverb send", &p.reverb_send, 0.0f, 1.0f, "%.2f");
        }

        void draw_reverb_zone_inspector(EditorContext& context, Simulation::IWorldEditor& world,
                                        Simulation::EntityId id)
        {
            if (!world.has_reverb_zone(id))
                return;
            if (!ImGui::CollapsingHeader("Reverb Zone", ImGuiTreeNodeFlags_DefaultOpen))
                return;

            Simulation::ReverbZoneParameters p = world.reverb_zone_parameters(id);

            auto apply = [&]() { world.set_reverb_zone_parameters(id, p); };
            auto slider = [&](const char* label, float* v, float lo, float hi, const char* fmt) {
                const bool ch = ImGui::SliderFloat(label, v, lo, hi, fmt);
                if (ImGui::IsItemActivated())
                    context.history.begin_change(world);
                if (ImGui::IsItemDeactivatedAfterEdit())
                    context.history.end_change();
                if (ch)
                    apply();
            };

            ImGui::TextDisabled("Presets");
            auto preset = [&](const char* name, float room, float room_hf, float decay, float hf_ratio) {
                if (ImGui::SmallButton(name))
                {
                    context.history.record(world);
                    p.room = room;
                    p.room_hf = room_hf;
                    p.decay_time = decay;
                    p.decay_hf_ratio = hf_ratio;
                    apply();
                }
            };
            preset("Room", -10.0f, -6.0f, 0.5f, 0.6f);
            ImGui::SameLine();
            preset("Hall", -4.0f, -2.0f, 2.9f, 0.7f);
            ImGui::SameLine();
            preset("Cave", -2.0f, -10.0f, 4.5f, 0.4f);
            ImGui::SameLine();
            preset("Generic", -6.0f, -3.0f, 1.5f, 0.5f);

            float half[3] = {static_cast<float>(p.half_extents.x), static_cast<float>(p.half_extents.y),
                             static_cast<float>(p.half_extents.z)};
            if (ImGui::DragFloat3("Half extents", half, 0.25f, 0.1f, 1000.0f, "%.1f m"))
            {
                if (ImGui::IsItemActivated())
                    context.history.begin_change(world);
                p.half_extents = Vector3{half[0], half[1], half[2]};
                apply();
            }
            if (ImGui::IsItemDeactivatedAfterEdit())
                context.history.end_change();

            slider("Decay time", &p.decay_time, 0.1f, 20.0f, "%.2f s");
            slider("Decay HF ratio", &p.decay_hf_ratio, 0.1f, 2.0f, "%.2f");
            slider("Room level", &p.room, -40.0f, 0.0f, "%.1f dB");
            slider("Room HF", &p.room_hf, -40.0f, 0.0f, "%.1f dB");
            slider("Reflections", &p.reflections, -40.0f, 10.0f, "%.1f dB");
            slider("Reverb level", &p.reverb, -40.0f, 10.0f, "%.1f dB");
            slider("Diffusion", &p.diffusion, 0.0f, 100.0f, "%.0f%%");
            slider("Density", &p.density, 0.0f, 100.0f, "%.0f%%");
            slider("Send", &p.send, 0.0f, 1.0f, "%.2f");

            int priority = p.priority;
            if (ImGui::InputInt("Priority", &priority))
            {
                context.history.record(world);
                p.priority = priority;
                apply();
            }
        }

        void draw_audio_listener_inspector(EditorContext& context, Simulation::IWorldEditor& world,
                                           Simulation::EntityId id)
        {
            if (!world.has_audio_listener(id))
                return;
            if (!ImGui::CollapsingHeader("Audio Listener", ImGuiTreeNodeFlags_DefaultOpen))
                return;

            Simulation::AudioListenerParameters p = world.audio_listener_parameters(id);

            bool changed = false;
            const bool ch = ImGui::SliderFloat("Master gain", &p.gain, 0.0f, 2.0f, "%.2f");
            if (ImGui::IsItemActivated())
                context.history.begin_change(world);
            if (ImGui::IsItemDeactivatedAfterEdit())
                context.history.end_change();
            changed |= ch;

            bool active = p.active;
            if (ImGui::Checkbox("Active", &active))
            {
                context.history.record(world);
                p.active = active;
                changed = true;
            }
            if (changed)
                world.set_audio_listener_parameters(id, p);

            ImGui::TextDisabled("In-editor preview listens from the Scene camera;");
            ImGui::TextDisabled("this marks the ears for a running game.");
        }
    } // namespace Editor
} // namespace SushiEngine
