/**************************************************************************/
/* profiler_panel.cpp                                                     */
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

#include "profiler_panel.hpp"

#include <algorithm>
#include <cstddef>
#include <string>
#include <vector>

#include <imgui.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /** @brief Prints a value row, or a dimmed "n/a" when the producer is not wired. */
            void value_row(const char* label, bool available, const char* format, double value)
            {
                ImGui::TableNextRow();
                ImGui::TableSetColumnIndex(0);
                ImGui::TextUnformatted(label);
                ImGui::TableSetColumnIndex(1);
                if (available)
                    ImGui::Text(format, value);
                else
                    ImGui::TextDisabled("n/a");
            }
        } // namespace

        void draw_profiler_panel(EditorContext& context, ProfilerPanelState& state)
        {
            if (!context.panels.profiler)
                return;
            if (!ImGui::Begin("Profiler", &context.panels.profiler))
            {
                ImGui::End();
                return;
            }

            ImGui::Checkbox("Pause", &state.paused);
            ImGui::Separator();

            // While paused, keep rendering the frame held at the moment Pause was ticked
            // instead of the context's latest, so two frames can be compared by eye; while
            // running, refresh every held copy every frame so unpausing has no stale gap.
            // All sections read only these held copies, CPU included, so Pause freezes the
            // whole panel together rather than just the CPU channel table.
            if (!state.paused)
            {
                state.held_frame_profile = context.frame_profile;
                state.held_gpu_statistics = context.gpu_statistics;
                state.held_scene_cull_drawn = context.scene_cull_drawn;
                state.held_scene_cull_tested = context.scene_cull_tested;
                state.held_scene_render_width = context.scene_render_width;
                state.held_scene_render_height = context.scene_render_height;
                state.held_physics_statistics = context.physics_statistics;
            }
            const SushiEngine::Profiling::FrameProfileSnapshot& shown = state.held_frame_profile;

            if (ImGui::CollapsingHeader("Frame", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (shown.frame_history.empty())
                {
                    ImGui::TextDisabled("n/a — first frame pending");
                }
                else
                {
                    float worst = shown.frame_history.front();
                    float total = 0.0f;
                    for (float sample : shown.frame_history)
                    {
                        worst = std::max(worst, sample);
                        total += sample;
                    }
                    const float average = total / static_cast<float>(shown.frame_history.size());
                    ImGui::PlotLines("##frame_history", shown.frame_history.data(),
                                     static_cast<int>(shown.frame_history.size()), 0, nullptr,
                                     0.0f, worst * 1.2f, ImVec2(-1.0f, 60.0f));
                    ImGui::Text("CPU frame: %.2f ms (%.0f FPS)", shown.frame_milliseconds,
                                shown.frame_milliseconds > 0.0f
                                    ? 1000.0f / shown.frame_milliseconds
                                    : 0.0f);
                    ImGui::SameLine();
                    ImGui::TextDisabled("avg %.2f  worst %.2f", average, worst);
                }
            }

            if (ImGui::CollapsingHeader("CPU", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (ImGui::BeginTable("cpu_channels", 3,
                                      ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingStretchProp))
                {
                    ImGui::TableSetupColumn("Channel");
                    ImGui::TableSetupColumn("ms", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                    ImGui::TableSetupColumn("% frame", ImGuiTableColumnFlags_WidthFixed, 70.0f);
                    ImGui::TableHeadersRow();
                    for (const SushiEngine::Profiling::ChannelValue& channel : shown.channels)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        const std::string indented(2 * channel.depth, ' ');
                        ImGui::TextUnformatted((indented + channel.name).c_str());
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%6.3f", channel.milliseconds);
                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%5.1f", shown.frame_milliseconds > 0.0f
                                                 ? 100.0f * channel.milliseconds /
                                                       shown.frame_milliseconds
                                                 : 0.0f);
                    }
                    ImGui::EndTable();
                }
                // The physics stages measure themselves off-thread; shown beside the main
                // thread's channels rather than inside them, labeled with their origin.
                const SushiEngine::Physics::PhysicsStatistics& physics =
                    state.held_physics_statistics;
                ImGui::TextDisabled("Physics (worker thread, profiling-gated):");
                // The stepper only timestamps its stages while the Physics panel is open
                // (see main.cpp's set_physics_profiling(context.panels.physics)), so a
                // total of zero here means nobody asked for the timings, not that physics
                // is free.
                if (physics.timings.total_ms > 0.0)
                {
                    ImGui::TextDisabled(
                        "  broadphase %.3f  narrowphase %.3f  solve %.3f  total %.3f",
                        physics.timings.broadphase_ms, physics.timings.narrowphase_ms,
                        physics.timings.solve_ms, physics.timings.total_ms);
                }
                else
                {
                    ImGui::TextDisabled("  n/a — open the Physics panel to collect timings");
                }
            }

            if (ImGui::CollapsingHeader("GPU", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (state.held_gpu_statistics.empty())
                    ImGui::TextDisabled("n/a — no viewport has rendered yet");
                for (const ViewportGPUStatistics& viewport : state.held_gpu_statistics)
                {
                    float total = 0.0f;
                    for (const GPUPassStatistic& pass : viewport.passes)
                        total += pass.milliseconds;
                    ImGui::Text("%s: %.3f ms", viewport.viewport.c_str(), total);
                    // Sorted descending by cost with a percent column, unlike Statistics'
                    // graph-order list: this window answers "what is expensive".
                    std::vector<const GPUPassStatistic*> sorted;
                    sorted.reserve(viewport.passes.size());
                    for (const GPUPassStatistic& pass : viewport.passes)
                        sorted.push_back(&pass);
                    std::sort(sorted.begin(), sorted.end(),
                              [](const GPUPassStatistic* left, const GPUPassStatistic* right)
                              {
                                  return left->milliseconds > right->milliseconds;
                              });
                    for (const GPUPassStatistic* pass : sorted)
                        ImGui::TextDisabled("  %-22s %6.3f  %5.1f%%", pass->pass.c_str(),
                                            pass->milliseconds,
                                            total > 0.0f ? 100.0f * pass->milliseconds / total
                                                         : 0.0f);
                }
            }

            if (ImGui::CollapsingHeader("Renderer"))
            {
                if (ImGui::BeginTable("renderer_counters", 2, ImGuiTableFlags_RowBg))
                {
                    const bool cull_available = state.held_scene_cull_tested > 0;
                    const bool render_size_available = state.held_scene_render_width > 0;
                    value_row("Draw calls", false, "%.0f", 0.0);
                    value_row("Visible triangles", false, "%.0f", 0.0);
                    value_row("Instances drawn", cull_available,
                              "%.0f", static_cast<double>(state.held_scene_cull_drawn));
                    value_row("Instances tested", cull_available,
                              "%.0f", static_cast<double>(state.held_scene_cull_tested));
                    value_row("Render width", render_size_available,
                              "%.0f", static_cast<double>(state.held_scene_render_width));
                    value_row("Render height", render_size_available,
                              "%.0f", static_cast<double>(state.held_scene_render_height));
                    ImGui::EndTable();
                }
            }

            if (ImGui::CollapsingHeader("Memory"))
            {
                if (ImGui::BeginTable("memory", 2, ImGuiTableFlags_RowBg))
                {
                    value_row("Video memory used (MiB)", false, "%.1f", 0.0);
                    value_row("Video memory budget (MiB)", false, "%.1f", 0.0);
                    value_row("Texture residency (MiB)", false, "%.1f", 0.0);
                    value_row("Process working set (MiB)", false, "%.1f", 0.0);
                    value_row("System memory used (MiB)", false, "%.1f", 0.0);
                    ImGui::EndTable();
                }
            }

            if (ImGui::CollapsingHeader("System"))
            {
                if (ImGui::BeginTable("system", 2, ImGuiTableFlags_RowBg))
                {
                    value_row("CPU utilization (%)", false, "%.1f", 0.0);
                    value_row("GPU utilization (%)", false, "%.1f", 0.0);
                    value_row("GPU temperature (C)", false, "%.0f", 0.0);
                    value_row("GPU busy, derived (%)", false, "%.1f", 0.0);
                    ImGui::EndTable();
                }
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
