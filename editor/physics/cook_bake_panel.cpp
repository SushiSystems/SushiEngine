/**************************************************************************/
/* cook_bake_panel.cpp                                                    */
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

#include "cook_bake_panel.hpp"

#include <cstdarg>
#include <cstdio>

#include <imgui.h>

#include <SushiEngine/physics/cooking/collision_asset.hpp>
#include <SushiEngine/physics/cooking/soft_body_asset.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            using namespace SushiEngine::Physics::Cooking;

            /** @brief A label and a value, on one row, aligned with the Physics panel's. */
            void value_row(const char* label, const char* format, ...)
            {
                char buffer[128];
                va_list arguments;
                va_start(arguments, format);
                std::vsnprintf(buffer, sizeof(buffer), format, arguments);
                va_end(arguments);
                ImGui::Text("%-24s %s", label, buffer);
            }

            /** @brief What a status means, in the words an artist needs. */
            void status_row(const CookingReport& report)
            {
                switch (report.status)
                {
                    case CookingStatus::Succeeded:
                        ImGui::TextColored(ImVec4(0.4f, 0.85f, 0.4f, 1.0f), "%-24s %s", "Status",
                                           report.served_from_cache ? "cached" : "cooked");
                        return;
                    case CookingStatus::RejectedByThreshold:
                        // Rejected, and the asset is still there. That is the point: being told
                        // "this failed" without being able to look at what failed is not a
                        // diagnosis (§8.5).
                        ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%-24s %s", "Status",
                                           "rejected by a threshold (asset kept)");
                        return;
                    case CookingStatus::StageFailed:
                        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%-24s %s (%s)",
                                           "Status", "stage failed",
                                           report.failed_stage != nullptr ? report.failed_stage
                                                                          : "unnamed");
                        return;
                    case CookingStatus::EmptyInput:
                        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%-24s %s", "Status",
                                           "no usable triangle in the source");
                        return;
                }
            }

            /** @brief §14's collider readout: piece count, error, mass, inertia. */
            void draw_collision_report(const BakedAssetEntry& entry)
            {
                const CookingReport& report = entry.collision_report;
                status_row(report);
                value_row("Convex pieces", "%u", report.convex_piece_count);
                if (report.collision_triangle_count > 0)
                    value_row("Static triangles", "%u", report.collision_triangle_count);
                value_row("Largest piece", "%u vertices", report.largest_piece_vertex_count);
                value_row("Distance field", "%u^3", report.distance_field_resolution);

                // §7.6's number, and the sentence that makes it mean something. A bare figure
                // in an inspector is a figure nobody knows the sign convention of.
                ImGui::Text("%-24s %.4f", "Fatter than the mesh", double(report.hausdorff_error));
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("How far the collider protrudes past the visible mesh, in\n"
                                      "local units. A sampled lower bound — raise the accuracy\n"
                                      "lattice order to tighten it.");
                }
                ImGui::Text("%-24s %+.2f%%", "Volume error", double(report.volume_error) * 100.0);
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Collider volume against the mesh's. Estimated by summing\n"
                                      "the pieces, so overlap counts twice and this reads high.");
                }

                value_row("Mass", "%.3f kg", double(report.mass));
                if (report.mass <= 0.0f && ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("The source mesh is not closed, so there is no volume to\n"
                                      "weigh. Zero means the authored mass is kept.");
                }
                value_row("Centre of mass", "%.3f, %.3f, %.3f", double(report.center_of_mass[0]),
                          double(report.center_of_mass[1]), double(report.center_of_mass[2]));
                value_row("Principal inertia", "%.4f, %.4f, %.4f",
                          double(report.principal_inertia[0]), double(report.principal_inertia[1]),
                          double(report.principal_inertia[2]));
                value_row("Asset size", "%zu bytes", report.asset_bytes);
            }

            /** @brief §14's soft-body readout: elements, quality, bindings, levels. */
            void draw_soft_body_report(const BakedAssetEntry& entry)
            {
                const CookingReport& report = entry.soft_body_report;
                status_row(report);
                value_row("Tetrahedra", "%u", report.tetrahedron_count);
                value_row("Levels of detail", "%u", report.level_of_detail_count);

                const bool poor = report.worst_element_quality < 0.05f;
                ImGui::TextColored(poor ? ImVec4(1.0f, 0.7f, 0.2f, 1.0f)
                                        : ImVec4(1.0f, 1.0f, 1.0f, 1.0f),
                                   "%-24s %.3f", "Worst element",
                                   double(report.worst_element_quality));
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("One is a regular tetrahedron, zero is a sliver. Slivers\n"
                                      "wreck the conditioning of a finite-element solve.");
                }

                if (report.inverted_element_count > 0)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f), "%-24s %u",
                                       "Inverted elements", report.inverted_element_count);
                }
                else
                {
                    ImGui::TextDisabled("%-24s none", "Inverted elements");
                }

                value_row("Unbound vertices", "%u", report.unembedded_vertex_count);
                ImGui::Text("%-24s %u", "Extrapolated bindings",
                            report.extrapolated_binding_count);
                if (ImGui::IsItemHovered())
                {
                    // The actionable number, and the one an artist can act on: it is not a
                    // fault, it is a resolution complaint.
                    ImGui::SetTooltip("Render vertices bound by extrapolation because they fell\n"
                                      "outside every element. Not a fault — they still move\n"
                                      "correctly — but a large count means whole features are\n"
                                      "thinner than a voxel. Raise the fidelity.");
                }
                value_row("Departs from mesh", "%.4f", double(report.hausdorff_error));
                value_row("Distance field", "%u^3", report.distance_field_resolution);
                value_row("Asset size", "%zu bytes", report.asset_bytes);
            }
        } // namespace

        void draw_cook_bake_panel(EditorContext& context, CookBakeState& state)
        {
            // Polled before the open check: a bake the artist started and then closed the
            // window on must still finish and still be there when it is reopened.
            state.poll();

            if (!context.panels.bake)
                return;
            if (!ImGui::Begin("Bake", &context.panels.bake))
            {
                ImGui::End();
                return;
            }

            ImportProfile project = state.profiles().project_default();
            bool changed = false;

            ImGui::TextDisabled("Project default");
            changed |= ImGui::SliderFloat("Fidelity", &project.parameters.fidelity, 0.0f, 1.0f,
                                          "%.2f");
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("One dial. Every resolution below is derived from it, and the\n"
                                  "resolution-like ones geometrically — so equal steps of the\n"
                                  "slider are equal factors on the cost.");
            }
            changed |= ImGui::Checkbox("Cook collision", &project.parameters.cook_collision);
            ImGui::SameLine();
            changed |= ImGui::Checkbox("Cook soft body", &project.parameters.cook_soft_body);
            changed |= ImGui::Checkbox("Authored static", &project.parameters.static_geometry);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Static geometry skips convex decomposition and cooks the\n"
                                  "exact triangle hierarchy instead: cheaper, and no error.");
            }
            if (changed)
                state.profiles().set_project_default(project);

            // What the dial produced, which is §8.2's requirement rather than a nicety: the
            // trade-off has to be visible now instead of felt three weeks later.
            const DerivedCookingParameters derived =
                resolve_cooking_parameters(project.parameters);
            ImGui::Separator();
            ImGui::TextDisabled("What the dial produces");
            value_row("Voxel resolution", "%d", derived.voxel_resolution);
            value_row("Target tetrahedra", "%d", derived.target_tetrahedron_count);
            value_row("Convex pieces", "at most %d", derived.convex_piece_count);
            value_row("Distance field", "%d^3", derived.distance_field_resolution);
            value_row("Levels of detail", "%d", derived.simulation_level_count);
            value_row("Suggested substeps", "%d", derived.suggested_substep_count);

            ImGui::Separator();
            const CookingServiceStatus status = state.status();
            if (status.busy)
            {
                const float fraction =
                    status.total_stages > 0
                        ? float(status.completed_stages) / float(status.total_stages)
                        : 0.0f;
                ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f),
                                   status.stage.empty() ? "working" : status.stage.c_str());
                ImGui::Text("Cooking %s", status.asset_path.c_str());
            }
            else
            {
                ImGui::TextDisabled("Idle");
            }
            if (status.queued > 0)
                ImGui::Text("%zu queued", status.queued);

            ImGui::Separator();
            ImGui::TextDisabled("Baked this session");
            if (state.entries().empty())
            {
                ImGui::TextDisabled("Nothing yet. Assets cook on import; this is where they\n"
                                    "report what they produced.");
            }

            for (const BakedAssetEntry& entry : state.entries())
            {
                ImGui::PushID(entry.asset_path.c_str());
                const bool selected = state.selected() == entry.asset_path;
                if (ImGui::Selectable(entry.asset_path.c_str(), selected))
                    state.select(entry.asset_path);

                if (selected)
                {
                    ImGui::Indent();
                    if (!entry.loaded)
                    {
                        ImGui::TextColored(ImVec4(1.0f, 0.4f, 0.4f, 1.0f),
                                           "The mesh could not be read.");
                    }
                    else
                    {
                        value_row("Source triangles", "%u", entry.source_triangle_count);
                    }

                    if (ImGui::Button("Re-cook"))
                        state.rebake(entry.asset_path);
                    if (ImGui::IsItemHovered())
                    {
                        ImGui::SetTooltip("Drops the cached asset and cooks again. Needed when\n"
                                          "the cooker changed rather than the mesh — the content\n"
                                          "hash cannot see that.");
                    }

                    if (entry.has_collision() && ImGui::CollapsingHeader("Collider"))
                        draw_collision_report(entry);
                    if (entry.has_soft_body() && ImGui::CollapsingHeader("Soft body"))
                        draw_soft_body_report(entry);

                    const std::size_t segments = state.collision_wireframe().size() / 6;
                    if (segments > 0)
                    {
                        ImGui::TextDisabled("Collider overlay: %zu segments in the viewport",
                                            segments);
                    }
                    ImGui::Unindent();
                }
                ImGui::PopID();
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
