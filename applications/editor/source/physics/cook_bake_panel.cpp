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
#include <SushiEngine/physics/cooking/node_beam_cooker.hpp>
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
            void draw_collision_report(const Authoring::BakedAssetEntry& entry)
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
            void draw_soft_body_report(const Authoring::BakedAssetEntry& entry)
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

            /** @brief §14's node-beam readout: nodes, beams, bracing, and the mass they carry. */
            void draw_node_beam_report(const Authoring::BakedAssetEntry& entry)
            {
                const CookingReport& report = entry.node_beam_report;
                status_row(report);
                value_row("Nodes", "%u", report.node_count);
                value_row("Beams", "%u", report.beam_count);
                value_row("Bracing beams", "%u", report.bracing_beam_count);
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Beams that are not along a grid axis. Zero here means\n"
                                      "the diagonal rule did not fire: a structure that is all\n"
                                      "structural beams holds its length and folds flat, because\n"
                                      "it has no shear rigidity.");
                }
                value_row("Collision triangles", "%u", report.collision_triangle_count);

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
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Render vertices with no node within the skin search\n"
                                      "radius. Above zero the cook fails, per §8.6: a vertex\n"
                                      "bound to nothing tears open the first time the shell\n"
                                      "deforms.");
                }
                value_row("Departs from mesh", "%.4f", double(report.hausdorff_error));

                value_row("Mass", "%.3f kg", double(report.mass));
                value_row("Centre of mass", "%.3f, %.3f, %.3f", double(report.center_of_mass[0]),
                          double(report.center_of_mass[1]), double(report.center_of_mass[2]));
                value_row("Principal inertia", "%.4f, %.4f, %.4f",
                          double(report.principal_inertia[0]), double(report.principal_inertia[1]),
                          double(report.principal_inertia[2]));
                value_row("Asset size", "%zu bytes", report.asset_bytes);
            }

            /**
             * @brief The material, and the four numbers §16.35 calls "vehicle-shaped".
             *
             * The preset combo is an action, not a stored choice: `NodeBeamCookerSettings`
             * has no field for "which named material this was", and it should not grow one
             * just to feed a widget — an artist who nudges one slider after picking "Sheet
             * steel" has a material that is no longer sheet steel, and the combo relabelling
             * itself back to "Apply preset..." says so rather than lying about it.
             *
             * @param settings The project's node-beam settings; edited in place.
             * @return Whether anything changed.
             */
            bool draw_node_beam_settings(NodeBeamCookerSettings& settings)
            {
                bool changed = false;

                static const char* const PRESETS[] = {"Apply preset...", "Rubber",  "Foam",
                                                       "Soft tissue",    "Sheet steel",
                                                       "Aluminium"};
                int preset = 0;
                if (ImGui::Combo("Material preset", &preset, PRESETS, IM_ARRAYSIZE(PRESETS)))
                {
                    switch (preset)
                    {
                        case 1: settings.material = Physics::rubber_material<Scalar>(); break;
                        case 2: settings.material = Physics::foam_material<Scalar>(); break;
                        case 3:
                            settings.material = Physics::soft_tissue_material<Scalar>();
                            break;
                        case 4:
                            settings.material = Physics::sheet_steel_material<Scalar>();
                            break;
                        case 5: settings.material = Physics::aluminium_material<Scalar>(); break;
                        default: break;
                    }
                    changed = preset != 0;
                }

                float young_modulus = float(settings.material.young_modulus);
                if (ImGui::InputFloat("Young's modulus", &young_modulus, 0.0f, 0.0f, "%.3e Pa"))
                {
                    settings.material.young_modulus = Scalar(young_modulus);
                    changed = true;
                }
                float poisson_ratio = float(settings.material.poisson_ratio);
                if (ImGui::DragFloat("Poisson ratio", &poisson_ratio, 0.005f, -0.999f, 0.499f,
                                     "%.3f"))
                {
                    settings.material.poisson_ratio = Scalar(poisson_ratio);
                    changed = true;
                }
                float density = float(settings.material.density);
                if (ImGui::DragFloat("Density", &density, 5.0f, 1.0f, 20000.0f, "%.0f kg/m^3"))
                {
                    settings.material.density = Scalar(density);
                    changed = true;
                }
                float damping = float(settings.material.damping);
                if (ImGui::DragFloat("Damping", &damping, 0.01f, 0.0f, 5.0f, "%.3f"))
                {
                    settings.material.damping = Scalar(damping);
                    changed = true;
                }
                float yield_stress = float(settings.material.yield_stress);
                if (ImGui::InputFloat("Yield stress", &yield_stress, 0.0f, 0.0f, "%.3e Pa"))
                {
                    settings.material.yield_stress = Scalar(yield_stress);
                    changed = true;
                }
                float plastic_creep = float(settings.material.plastic_creep);
                if (ImGui::DragFloat("Plastic creep", &plastic_creep, 0.005f, 0.0f, 1.0f, "%.3f"))
                {
                    settings.material.plastic_creep = Scalar(plastic_creep);
                    changed = true;
                }
                float maximum_plastic_strain = float(settings.material.maximum_plastic_strain);
                if (ImGui::DragFloat("Maximum plastic strain", &maximum_plastic_strain, 0.005f,
                                     0.0f, 2.0f, "%.3f"))
                {
                    settings.material.maximum_plastic_strain = Scalar(maximum_plastic_strain);
                    changed = true;
                }
                float fracture_stress = float(settings.material.fracture_stress);
                if (ImGui::InputFloat("Fracture stress", &fracture_stress, 0.0f, 0.0f, "%.3e Pa"))
                {
                    settings.material.fracture_stress = Scalar(fracture_stress);
                    changed = true;
                }

                ImGui::Separator();
                if (ImGui::SliderFloat("Core mass fraction", &settings.core_mass_fraction, 0.0f,
                                       1.0f, "%.2f"))
                    changed = true;
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("The fraction of the cooked mass the rigid core carries.\n"
                                      "Zero is a pure node-beam vehicle; toward one the chassis\n"
                                      "is rigid and the shell is a skin over it.");
                }
                if (ImGui::DragFloat("Structural length ratio", &settings.structural_length_ratio,
                                     0.02f, 1.0f, 3.0f, "%.2f"))
                    changed = true;
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Lattice edges longer than this multiple of the cell size\n"
                                      "are bracing rather than structural.");
                }
                if (ImGui::DragFloat("Skin search ratio", &settings.skin_search_ratio, 0.02f,
                                     0.5f, 5.0f, "%.2f"))
                    changed = true;
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("How far past the cell size a node may be from a render\n"
                                      "vertex and still skin it. A vertex with none in reach is\n"
                                      "reported unbound rather than tethered to whatever was\n"
                                      "nearest.");
                }
                if (ImGui::Checkbox("Attach shell to core", &settings.attach_shell_to_core))
                    changed = true;
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Off makes an asset whose shell falls off its own chassis\n"
                                      "-- what a pure node-beam cook wants, and what a hybrid\n"
                                      "one never does.");
                }

                return changed;
            }

            /**
             * @brief One dial-derived field: a pin checkbox, and the value only when pinned.
             *
             * @ref DERIVE_FROM_FIDELITY is the sentinel for "take it from the dial" —
             * unpinning restores it rather than freezing whatever the slider happened to
             * be showing. Disabled and dimmed to the current derived value while unpinned,
             * so the row still answers "what would this cook use" without implying it is
             * editable.
             */
            bool pinned_int_row(const char* label, std::int32_t& value, std::int32_t derived,
                                int low, int high, const char* tooltip)
            {
                bool changed = false;
                bool pinned = value != DERIVE_FROM_FIDELITY;
                ImGui::PushID(label);
                if (ImGui::Checkbox("##pin", &pinned))
                {
                    value = pinned ? derived : DERIVE_FROM_FIDELITY;
                    changed = true;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Pin this field instead of deriving it from the dial.");
                ImGui::SameLine();
                int shown = pinned ? value : derived;
                ImGui::BeginDisabled(!pinned);
                if (ImGui::DragInt(label, &shown, 1.0f, low, high) && pinned)
                {
                    value = shown;
                    changed = true;
                }
                ImGui::EndDisabled();
                if (tooltip != nullptr && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", tooltip);
                ImGui::PopID();
                return changed;
            }

            /**
             * @brief The settings §16.45.3 exposes: the seven pin overrides, the four fields
             * that are not on the dial, and the accept/reject thresholds every cooker applies
             * via `apply_cooking_thresholds`. Grouped apart from the project-default block
             * above because these are the settings a project sets once and rarely revisits,
             * not the one an artist drags every day.
             *
             * @param parameters The project's cooking dial and its pin overrides.
             * @param thresholds What fails a cook outright rather than merely shaping it.
             * @return Whether anything changed.
             */
            bool draw_advanced_cooking_parameters(CookingParameters& parameters,
                                                  CookingThresholds& thresholds)
            {
                bool changed = false;
                const DerivedCookingParameters derived = resolve_cooking_parameters(parameters);

                ImGui::TextDisabled("Pin a field to override the dial for it alone.");
                changed |= pinned_int_row("Voxel resolution", parameters.voxel_resolution,
                                          derived.voxel_resolution, 16, 256, nullptr);
                changed |= pinned_int_row("Target tetrahedra",
                                          parameters.target_tetrahedron_count,
                                          derived.target_tetrahedron_count, 200, 120000,
                                          nullptr);
                changed |= pinned_int_row("Levels of detail",
                                          parameters.simulation_level_count,
                                          derived.simulation_level_count, 1, 4, nullptr);
                changed |= pinned_int_row("Convex pieces", parameters.convex_piece_count,
                                          derived.convex_piece_count, 4, 64, nullptr);
                changed |= pinned_int_row("Distance field",
                                          parameters.distance_field_resolution,
                                          derived.distance_field_resolution, 16, 128, nullptr);
                changed |= pinned_int_row("Surface conforming passes",
                                          parameters.surface_conforming_passes,
                                          derived.surface_conforming_passes, 0, 3, nullptr);
                changed |= pinned_int_row("Suggested substeps",
                                          parameters.suggested_substep_count,
                                          derived.suggested_substep_count, 8, 32, nullptr);

                ImGui::Separator();
                ImGui::TextDisabled("Not on the dial");
                int hull_vertex_budget = parameters.hull_vertex_budget;
                if (ImGui::DragInt("Hull vertex budget", &hull_vertex_budget, 1.0f, 4, 255))
                {
                    parameters.hull_vertex_budget = hull_vertex_budget;
                    changed = true;
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Vertices a single cooked convex piece may keep. Bounded\n"
                                      "by what the narrowphase can afford to iterate per pair,\n"
                                      "not by how accurate the artist wants the asset.");
                }
                changed |= ImGui::DragFloat("Weld tolerance", &parameters.weld_tolerance,
                                            1.0e-6f, 0.0f, 0.1f, "%.6f");
                changed |= ImGui::DragFloat("Density", &parameters.density, 5.0f, 1.0f,
                                            20000.0f, "%.0f kg/m^3");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Overridden per instance by the scene's own material at "
                                      "instancing time.");
                int accuracy_lattice_order = parameters.accuracy_lattice_order;
                if (ImGui::DragInt("Accuracy lattice order", &accuracy_lattice_order, 1.0f, 1,
                                   8))
                {
                    parameters.accuracy_lattice_order = accuracy_lattice_order;
                    changed = true;
                }
                if (ImGui::IsItemHovered())
                {
                    ImGui::SetTooltip("Governs the reported Hausdorff error's tightness, not\n"
                                      "the asset -- a cook whose only change is this produces\n"
                                      "identical geometry with a different report.");
                }

                ImGui::Separator();
                ImGui::TextDisabled("Thresholds -- what fails a cook, not what shapes it");
                int max_unembedded = int(thresholds.max_unembedded_vertices);
                if (ImGui::DragInt("Max unembedded vertices", &max_unembedded, 1.0f, 0, 10000))
                {
                    thresholds.max_unembedded_vertices =
                        std::uint32_t(max_unembedded < 0 ? 0 : max_unembedded);
                    changed = true;
                }
                int max_inverted = int(thresholds.max_inverted_elements);
                if (ImGui::DragInt("Max inverted elements", &max_inverted, 1.0f, 0, 10000))
                {
                    thresholds.max_inverted_elements =
                        std::uint32_t(max_inverted < 0 ? 0 : max_inverted);
                    changed = true;
                }
                changed |= ImGui::DragFloat("Min element quality",
                                            &thresholds.min_element_quality, 0.001f, 0.0f, 1.0f,
                                            "%.4f");
                changed |= ImGui::DragFloat("Max Hausdorff error",
                                            &thresholds.max_hausdorff_error, 0.001f, 0.0f, 10.0f,
                                            "%.4f");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Zero or less disables the check.");
                changed |= ImGui::Checkbox("Require watertight source",
                                           &thresholds.require_watertight_source);

                return changed;
            }
        } // namespace

        void draw_cook_bake_panel(EditorContext& context, Authoring::CookBakeState& state)
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
            ImGui::SameLine();
            changed |= ImGui::Checkbox("Cook node beam", &project.parameters.cook_node_beam);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Cooks a .sushinodebeam: a node cloud, a beam network, and a\n"
                                  "rigid core, for a vehicle's deformable shell (Section 11).\n"
                                  "Minutes rather than milliseconds -- wanted by a handful of\n"
                                  "meshes, not most of a project.");
            }
            changed |= ImGui::Checkbox("Authored static", &project.parameters.static_geometry);
            if (ImGui::IsItemHovered())
            {
                ImGui::SetTooltip("Static geometry skips convex decomposition and cooks the\n"
                                  "exact triangle hierarchy instead: cheaper, and no error.");
            }

            if (project.parameters.cook_node_beam && ImGui::CollapsingHeader("Node-beam settings"))
            {
                ImGui::Indent();
                changed |= draw_node_beam_settings(project.node_beam_settings);
                ImGui::Unindent();
            }

            if (ImGui::CollapsingHeader("Advanced"))
            {
                ImGui::Indent();
                changed |= draw_advanced_cooking_parameters(project.parameters,
                                                            project.thresholds);
                ImGui::Unindent();
            }

            if (changed)
            {
                state.profiles().set_project_default(project);
                state.save_profiles();
            }

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

            for (const Authoring::BakedAssetEntry& entry : state.entries())
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
                    if (entry.has_node_beam() && ImGui::CollapsingHeader("Node-beam"))
                        draw_node_beam_report(entry);

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
