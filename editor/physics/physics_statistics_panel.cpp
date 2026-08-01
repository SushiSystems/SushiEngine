/**************************************************************************/
/* physics_statistics_panel.cpp                                           */
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

#include "physics_statistics_panel.hpp"

#include <imgui.h>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            /** @brief A count, drawn as a plain labelled row. */
            void count_row(const char* label, std::size_t value)
            {
                ImGui::Text("%-22s %zu", label, value);
            }

            /**
             * @brief A count that is only interesting when it is not zero.
             *
             * Drawn dimmed at zero and highlighted otherwise, because these are the
             * rows that mean something went wrong: a budget was exceeded, or a graph
             * was rebuilt when it should not have been. A number nobody notices is a
             * number that does not do its job.
             */
            void warning_row(const char* label, std::size_t value, const char* when_zero)
            {
                if (value == 0)
                {
                    ImGui::TextDisabled("%-22s %s", label, when_zero);
                    return;
                }
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1.0f), "%-22s %zu", label, value);
            }
        } // namespace

        void draw_physics_statistics_panel(EditorContext& context)
        {
            if (!context.panels.physics)
                return;
            if (!ImGui::Begin("Physics", &context.panels.physics))
            {
                ImGui::End();
                return;
            }

            const Physics::PhysicsStatistics& stats = context.physics_statistics;

            // §14's debug draw, toggled per category. Here rather than in a window of its
            // own because the counts below are what make an author want to *see* something:
            // "largest island 47" and "sleeping 0" are the two rows that send somebody
            // looking for which bodies those are, and the answer is one checkbox away.
            if (ImGui::CollapsingHeader("Debug Draw"))
            {
                PhysicsOverlaySettings& overlay = context.physics_overlay;
                ImGui::Checkbox("Contacts", &overlay.contacts);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Contact points and normals from the live stream. The "
                                      "normal's length scales with the impulse, so a crash "
                                      "reads longer than a scrape.");
                ImGui::Checkbox("Broadphase Bounds", &overlay.bounds);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("The box each body is tested by. Much larger than the "
                                      "body means a bound that is not being tightened.");
                ImGui::Checkbox("Islands", &overlay.islands);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("One colour per island. The colour is a hash of the "
                                      "index, not a stable identity — islands are renumbered "
                                      "whenever the partition changes.");
                ImGui::Checkbox("Sleeping", &overlay.sleeping);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Rings the bodies that are asleep. The interesting case "
                                      "is the one that is *not*, in a stack where everything "
                                      "else is.");
                ImGui::Checkbox("Joint Gizmo", &overlay.joints);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("The selected entity's joint: its two anchors, the line "
                                      "between them (which is its error, drawn), its primary "
                                      "axis, and its twist-limit arc.");
                ImGui::Separator();
            }

            ImGui::TextDisabled("Bodies");
            count_row("Awake", stats.awake_bodies);
            count_row("Sleeping", stats.sleeping_bodies);
            count_row("Islands", stats.islands);
            count_row("Largest island", stats.largest_island);

            ImGui::Separator();
            ImGui::TextDisabled("Collision");
            count_row("Pairs tested", stats.broadphase_pairs_tested);
            count_row("Pairs produced", stats.broadphase_pairs_produced);
            count_row("Manifolds", stats.manifolds);
            count_row("Contact points", stats.contact_points);

            ImGui::Separator();
            ImGui::TextDisabled("Solve");
            count_row("Constraints", stats.constraints);
            count_row("Colours", stats.colors);
            count_row("Largest colour", stats.largest_color);
            count_row("Substeps", stats.substeps);

            ImGui::Separator();
            ImGui::TextDisabled("Graph health");
            // These three are cumulative, not per-tick, and that is the point. The
            // compile and compose counts must stop climbing after warm-up: one that
            // keeps rising means the solve graph is being rebuilt every tick, which
            // makes every timing on this panel meaningless. An overflow means a
            // capacity in PhysicsCapacities was exceeded and work was silently
            // dropped, so it is the one number that must never be quietly zero-ish.
            count_row("Compiles", stats.compile_count);
            count_row("Composes", stats.compose_count);
            warning_row("Capacity overflows", stats.capacity_overflows, "none");
            warning_row("Continuous escalations", stats.continuous_escalations, "none");
            warning_row("Fracture events", stats.fracture_events, "none");

            ImGui::Separator();
            ImGui::TextDisabled("Timings");
            if (stats.timings.total_ms <= Scalar(0))
            {
                // Not an error state: measuring is switched off unless this panel asks
                // for it, so that the tick carries no timestamping at all when nobody
                // is looking. Saying so beats drawing a row of zeros that reads as
                // "the physics is free".
                ImGui::TextDisabled("No per-stage timings yet. Profiling engages on the");
                ImGui::TextDisabled("next tick while this panel is open.");
            }
            else
            {
                ImGui::Text("%-22s %.3f ms", "Broadphase", double(stats.timings.broadphase_ms));
                ImGui::Text("%-22s %.3f ms", "Narrowphase", double(stats.timings.narrowphase_ms));
                ImGui::Text("%-22s %.3f ms", "Islands", double(stats.timings.island_build_ms));
                ImGui::Text("%-22s %.3f ms", "Solve (device)", double(stats.timings.solve_ms));
                ImGui::Text("%-22s %.3f ms", "Write back", double(stats.timings.write_back_ms));
                ImGui::Separator();
                ImGui::Text("%-22s %.3f ms", "Total", double(stats.timings.total_ms));
                // The solve is one composition and the runtime's public add() carries
                // no node label, so it cannot honestly be split further here.
                ImGui::TextDisabled("Solve is one composition: predict, every colour, velocity.");
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
