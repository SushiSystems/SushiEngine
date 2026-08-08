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
                state.held_render_statistics = context.render_statistics;
                state.held_resident_texture_bytes = context.resident_texture_bytes;
                state.held_has_asset_library = context.assets != nullptr;
                state.held_physics_statistics = context.physics_statistics;
                state.held_system_metrics = context.system_metrics;
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
                if (state.held_render_statistics.empty())
                    ImGui::TextDisabled("n/a — no viewport has rendered yet");
                for (const ViewportRenderStatistics& entry : state.held_render_statistics)
                {
                    ImGui::TextUnformatted(entry.viewport.c_str());
                    const SushiEngine::Render::RenderFrameStatistics& statistics =
                        entry.statistics;
                    const bool cull_available = statistics.instances_tested > 0;
                    const bool render_size_available = statistics.render_width > 0;
                    ImGui::PushID(entry.viewport.c_str());
                    if (ImGui::BeginTable("renderer_counters", 2, ImGuiTableFlags_RowBg))
                    {
                        value_row("Draw calls", render_size_available, "%.0f",
                                  static_cast<double>(statistics.draw_calls));
                        value_row("Visible triangles", render_size_available, "%.0f",
                                  static_cast<double>(statistics.triangles));
                        value_row("Instances drawn", cull_available, "%.0f",
                                  static_cast<double>(statistics.instances_drawn));
                        value_row("Instances tested", cull_available, "%.0f",
                                  static_cast<double>(statistics.instances_tested));
                        value_row("Render width", render_size_available, "%.0f",
                                  static_cast<double>(statistics.render_width));
                        value_row("Render height", render_size_available, "%.0f",
                                  static_cast<double>(statistics.render_height));
                        ImGui::EndTable();
                    }
                    ImGui::PopID();
                }
            }

            if (ImGui::CollapsingHeader("Memory"))
            {
                const ViewportRenderStatistics* scene_entry = nullptr;
                for (const ViewportRenderStatistics& entry : state.held_render_statistics)
                    if (entry.viewport == "Scene")
                        scene_entry = &entry;
                const bool heap_available =
                    scene_entry != nullptr && scene_entry->statistics.heap_budget_bytes > 0;
                if (ImGui::BeginTable("memory", 2, ImGuiTableFlags_RowBg))
                {
                    value_row("Video memory used (MiB)", heap_available, "%.1f",
                              heap_available
                                  ? static_cast<double>(scene_entry->statistics.heap_used_bytes) /
                                        (1024.0 * 1024.0)
                                  : 0.0);
                    value_row("Video memory budget (MiB)", heap_available, "%.1f",
                              heap_available
                                  ? static_cast<double>(
                                        scene_entry->statistics.heap_budget_bytes) /
                                        (1024.0 * 1024.0)
                                  : 0.0);
                    value_row("Texture residency (MiB)", state.held_has_asset_library, "%.1f",
                              static_cast<double>(state.held_resident_texture_bytes) /
                                  (1024.0 * 1024.0));
                    const SystemMetricsSnapshot& metrics = state.held_system_metrics;
                    value_row("Process working set (MiB)",
                              metrics.process_working_set_bytes > 0, "%.1f",
                              static_cast<double>(metrics.process_working_set_bytes) /
                                  (1024.0 * 1024.0));
                    const bool system_memory_available = metrics.system_memory_total_bytes > 0;
                    ImGui::TableNextRow();
                    ImGui::TableSetColumnIndex(0);
                    ImGui::TextUnformatted("System memory used (MiB)");
                    ImGui::TableSetColumnIndex(1);
                    if (system_memory_available)
                        ImGui::Text("%.1f / %.1f",
                                    static_cast<double>(metrics.system_memory_used_bytes) /
                                        (1024.0 * 1024.0),
                                    static_cast<double>(metrics.system_memory_total_bytes) /
                                        (1024.0 * 1024.0));
                    else
                        ImGui::TextDisabled("n/a");
                    ImGui::EndTable();
                }
            }

            if (ImGui::CollapsingHeader("System"))
            {
                if (ImGui::BeginTable("system", 2, ImGuiTableFlags_RowBg))
                {
                    const SystemMetricsSnapshot& metrics = state.held_system_metrics;
                    value_row("CPU utilization (%)", metrics.cpu_valid, "%.1f",
                              static_cast<double>(metrics.cpu_utilization_percent));
                    value_row("GPU utilization (%)", metrics.gpu_valid, "%.1f",
                              static_cast<double>(metrics.gpu_utilization_percent));
                    value_row("GPU memory used (MiB)", metrics.gpu_valid, "%.1f",
                              static_cast<double>(metrics.gpu_memory_used_bytes) /
                                  (1024.0 * 1024.0));
                    value_row("GPU memory total (MiB)", metrics.gpu_valid, "%.1f",
                              static_cast<double>(metrics.gpu_memory_total_bytes) /
                                  (1024.0 * 1024.0));
                    value_row("GPU temperature (C)", metrics.gpu_valid, "%.0f",
                              static_cast<double>(metrics.gpu_temperature_celsius));

                    // The NVML-free fallback: how busy the GPU was, derived from the Scene
                    // viewport's own measured pass times rather than a vendor query — shown
                    // beside the NVML row regardless of gpu_valid, since this is exactly the
                    // reading meant to still work when NVML does not.
                    float scene_gpu_milliseconds = 0.0f;
                    for (const ViewportGPUStatistics& viewport : state.held_gpu_statistics)
                    {
                        if (viewport.viewport != "Scene")
                            continue;
                        for (const GPUPassStatistic& pass : viewport.passes)
                            scene_gpu_milliseconds += pass.milliseconds;
                    }
                    const float frame_milliseconds = shown.frame_milliseconds;
                    const bool derived_gpu_busy_available =
                        scene_gpu_milliseconds > 0.0f && frame_milliseconds > 0.0f;
                    const float derived_gpu_busy_percent =
                        derived_gpu_busy_available
                            ? std::clamp(100.0f * scene_gpu_milliseconds / frame_milliseconds,
                                        0.0f, 100.0f)
                            : 0.0f;
                    value_row("GPU busy, derived (%)", derived_gpu_busy_available, "%.1f",
                              static_cast<double>(derived_gpu_busy_percent));
                    ImGui::EndTable();
                }
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
