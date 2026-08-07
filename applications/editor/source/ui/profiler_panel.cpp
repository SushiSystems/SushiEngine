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
#include <cmath>
#include <cstddef>
#include <vector>

#include <imgui.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            // PROF0's stand-in numbers, deleted when PROF1/PROF2/PROF3 wire the sections.
            // Chosen to look like a plausible busy frame so layout is judged on real shapes.
            struct MockChannel
            {
                const char* name;
                float milliseconds;
            };
            constexpr MockChannel MOCK_CPU_CHANNELS[] = {
                {"event pump", 0.21f},        {"simulation tick", 2.85f},
                {"animation preview", 0.42f}, {"scene render submit", 3.10f},
                {"game render submit", 1.95f}, {"ui build", 1.35f},
                {"present wait", 4.80f},
            };
            constexpr float MOCK_FRAME_MILLISECONDS = 14.9f;

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

            if (ImGui::CollapsingHeader("Frame", ImGuiTreeNodeFlags_DefaultOpen))
            {
                // Mock ring: a noisy sine so the plot's scaling and height can be judged.
                float mock_history[240];
                for (int i = 0; i < 240; ++i)
                    mock_history[i] = MOCK_FRAME_MILLISECONDS +
                                      2.0f * std::sin(static_cast<float>(i) * 0.13f);
                ImGui::PlotLines("##frame_history", mock_history, 240, 0, nullptr, 0.0f,
                                 33.3f, ImVec2(-1.0f, 60.0f));
                ImGui::Text("CPU frame: %.2f ms (%.0f FPS)", MOCK_FRAME_MILLISECONDS,
                            1000.0f / MOCK_FRAME_MILLISECONDS);
                ImGui::SameLine();
                ImGui::TextDisabled("avg %.2f  worst %.2f", MOCK_FRAME_MILLISECONDS + 0.3f,
                                    MOCK_FRAME_MILLISECONDS + 2.1f);
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
                    for (const MockChannel& channel : MOCK_CPU_CHANNELS)
                    {
                        ImGui::TableNextRow();
                        ImGui::TableSetColumnIndex(0);
                        ImGui::TextUnformatted(channel.name);
                        ImGui::TableSetColumnIndex(1);
                        ImGui::Text("%6.3f", channel.milliseconds);
                        ImGui::TableSetColumnIndex(2);
                        ImGui::Text("%5.1f", 100.0f * channel.milliseconds /
                                                 MOCK_FRAME_MILLISECONDS);
                    }
                    ImGui::EndTable();
                }
                // The physics stages measure themselves off-thread; shown beside the main
                // thread's channels rather than inside them, labeled with their origin.
                const SushiEngine::Physics::PhysicsStatistics& physics =
                    context.physics_statistics;
                ImGui::TextDisabled("Physics (worker thread, profiling-gated):");
                ImGui::TextDisabled("  broadphase %.3f  narrowphase %.3f  solve %.3f  total %.3f",
                                    physics.timings.broadphase_ms, physics.timings.narrowphase_ms,
                                    physics.timings.solve_ms, physics.timings.total_ms);
            }

            if (ImGui::CollapsingHeader("GPU", ImGuiTreeNodeFlags_DefaultOpen))
            {
                if (context.gpu_statistics.empty())
                    ImGui::TextDisabled("n/a — no viewport has rendered yet");
                for (const ViewportGPUStatistics& viewport : context.gpu_statistics)
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
                    value_row("Draw calls", false, "%.0f", 0.0);
                    value_row("Visible triangles", false, "%.0f", 0.0);
                    value_row("Instances drawn", true,
                              "%.0f", static_cast<double>(context.scene_cull_drawn));
                    value_row("Instances tested", true,
                              "%.0f", static_cast<double>(context.scene_cull_tested));
                    value_row("Render width", true,
                              "%.0f", static_cast<double>(context.scene_render_width));
                    value_row("Render height", true,
                              "%.0f", static_cast<double>(context.scene_render_height));
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
