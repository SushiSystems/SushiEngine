/**************************************************************************/
/* terrain_panel.cpp                                                      */
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

#include "terrain_panel.hpp"

#include <cmath>
#include <cstdarg>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>

#include <imgui.h>

#include <SushiEngine/astro/celestial_bodies.hpp>

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            namespace Field = SushiEngine::Terrain;

            constexpr double DEGREES_PER_RADIAN = 57.295779513082320876;
            constexpr double RADIANS_PER_DEGREE = 0.017453292519943295769;

            /** @brief The word an operation goes by, in the order the enumeration declares. */
            const char* const OPERATION_NAMES[] = {"Flatten", "Raise", "Crater"};

            /** @brief What an operation does to the ground, for the combo's tooltip. */
            const char* operation_help(Field::LayerOperation operation)
            {
                switch (operation)
                {
                    case Field::LayerOperation::Flatten:
                        return "Pulls the ground towards one elevation: a building pad, a\n"
                               "road bed. Full strength inside the inner radius, easing to\n"
                               "untouched ground at the outer one.";
                    case Field::LayerOperation::Raise:
                        return "Adds to the ground: a berm, a spoil heap, a dam wall. The\n"
                               "same falloff between the two radii.";
                    case Field::LayerOperation::Crater:
                        return "A parabolic bowl inside the rim, and an ejecta blanket\n"
                               "decaying to nothing at the outer radius. The bowl needs a\n"
                               "rim radius above zero and an outer radius beyond it.";
                }
                return "";
            }

            /**
             * @brief The unit direction of a planetocentric latitude and east longitude.
             *
             * The body-fixed convention the terrain works in: z is the polar axis and x
             * points down the prime meridian (see `cube_sphere.hpp`'s ellipsoid, whose
             * polar axis is z).
             */
            SushiEngine::Vector3 direction_of(double latitude_radians,
                                              double longitude_radians) noexcept
            {
                const double cosine = std::cos(latitude_radians);
                return SushiEngine::Vector3{cosine * std::cos(longitude_radians),
                                            cosine * std::sin(longitude_radians),
                                            std::sin(latitude_radians)};
            }

            /** @brief The planetocentric latitude a unit direction sits at, radians. */
            double latitude_of(const SushiEngine::Vector3& direction) noexcept
            {
                const double z =
                    direction.z < -1.0 ? -1.0 : (direction.z > 1.0 ? 1.0 : direction.z);
                return std::asin(z);
            }

            /** @brief The east longitude a unit direction sits at, radians. */
            double longitude_of(const SushiEngine::Vector3& direction) noexcept
            {
                return std::atan2(direction.y, direction.x);
            }

            /** @brief A labelled read-only row, matching the Bake panel's alignment. */
            void value_row(const char* label, const char* format, ...)
            {
                char buffer[128];
                va_list arguments;
                va_start(arguments, format);
                std::vsnprintf(buffer, sizeof(buffer), format, arguments);
                va_end(arguments);
                ImGui::Text("%-22s %s", label, buffer);
            }

            /**
             * @brief Draws one layer's editable fields into the current window.
             *
             * Shared by the stack's rows and the draft form so the two cannot offer
             * different fields, different units, or different explanations of the same
             * record. Footprint radii are shown in metres along the surface, which is what
             * an author means, and converted against @p mean_radius_metres on the way back
             * into the record's body-independent angles.
             *
             * @param layer              The record, edited in place.
             * @param mean_radius_metres The body's mean radius; must be above zero.
             * @param here               The ground the view is over, for "Centre On View".
             * @return Whether any field changed this frame.
             */
            bool draw_layer_fields(Field::TerrainLayer& layer, double mean_radius_metres,
                                   const SushiEngine::Vector3& here)
            {
                bool changed = false;

                int operation = static_cast<int>(layer.operation);
                if (ImGui::Combo("Operation", &operation, OPERATION_NAMES, 3))
                {
                    layer.operation = static_cast<Field::LayerOperation>(operation);
                    changed = true;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", operation_help(layer.operation));

                float latitude =
                    static_cast<float>(latitude_of(layer.footprint.direction) * DEGREES_PER_RADIAN);
                float longitude = static_cast<float>(longitude_of(layer.footprint.direction) *
                                                     DEGREES_PER_RADIAN);
                bool moved = ImGui::DragFloat("Latitude", &latitude, 0.05f, -90.0f, 90.0f,
                                              "%.4f deg");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Measured from the body centre rather than from the\n"
                                      "surface normal, because a footprint is a direction.");
                moved |= ImGui::DragFloat("Longitude", &longitude, 0.05f, -180.0f, 180.0f,
                                          "%.4f deg");
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("East of the body's prime meridian.");
                if (moved)
                {
                    layer.footprint.direction =
                        direction_of(static_cast<double>(latitude) * RADIANS_PER_DEGREE,
                                     static_cast<double>(longitude) * RADIANS_PER_DEGREE);
                    changed = true;
                }
                if (ImGui::Button("Centre On View"))
                {
                    layer.footprint.direction = here;
                    changed = true;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Moves the footprint under the Scene camera.");

                const bool crater = layer.operation == Field::LayerOperation::Crater;
                float inner =
                    static_cast<float>(layer.footprint.inner_radians * mean_radius_metres);
                float outer =
                    static_cast<float>(layer.footprint.outer_radians * mean_radius_metres);
                const float span = static_cast<float>(3.14159265358979 * mean_radius_metres);
                if (ImGui::DragFloat(crater ? "Rim Radius" : "Full Strength Radius", &inner, 1.0f,
                                     0.0f, span, "%.1f m"))
                {
                    layer.footprint.inner_radians =
                        static_cast<double>(inner) / mean_radius_metres;
                    changed = true;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(crater ? "Where the bowl meets the rim."
                                             : "The operation acts at full strength within\n"
                                               "this distance of the centre.");
                if (ImGui::DragFloat(crater ? "Ejecta Radius" : "Falloff Radius", &outer, 1.0f,
                                     0.0f, span, "%.1f m"))
                {
                    layer.footprint.outer_radians =
                        static_cast<double>(outer) / mean_radius_metres;
                    changed = true;
                }
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip(crater ? "Where the ejecta blanket has decayed to the\n"
                                              "surrounding ground."
                                            : "The operation has faded to nothing by this\n"
                                              "distance; below the inner radius it does\n"
                                              "nothing at all.");
                if (outer <= inner)
                    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f),
                                       "The outer radius is not past the inner one, so this "
                                       "layer changes nothing.");

                // Only the fields the operation reads: a Flatten with a rim height beside
                // it invites an author to set a number the composition ignores.
                switch (layer.operation)
                {
                    case Field::LayerOperation::Flatten:
                    {
                        float target = static_cast<float>(layer.profile.target_metres);
                        if (ImGui::DragFloat("Target Elevation", &target, 1.0f, -12000.0f,
                                             12000.0f, "%.2f m"))
                        {
                            layer.profile.target_metres = static_cast<double>(target);
                            changed = true;
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("The elevation the ground is pulled to, above\n"
                                              "the body's reference ellipsoid.");
                        break;
                    }
                    case Field::LayerOperation::Raise:
                    {
                        float amount = static_cast<float>(layer.profile.amount_metres);
                        if (ImGui::DragFloat("Amount", &amount, 0.5f, -12000.0f, 12000.0f,
                                             "%.2f m"))
                        {
                            layer.profile.amount_metres = static_cast<double>(amount);
                            changed = true;
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("Added at full strength; negative digs.");
                        break;
                    }
                    case Field::LayerOperation::Crater:
                    {
                        float depth = static_cast<float>(layer.profile.depth_metres);
                        float rim = static_cast<float>(layer.profile.rim_metres);
                        if (ImGui::DragFloat("Depth", &depth, 0.5f, 0.0f, 12000.0f, "%.2f m"))
                        {
                            layer.profile.depth_metres = static_cast<double>(depth);
                            changed = true;
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("How far the bowl's floor sits below the\n"
                                              "surrounding ground.");
                        if (ImGui::DragFloat("Rim Height", &rim, 0.5f, 0.0f, 12000.0f, "%.2f m"))
                        {
                            layer.profile.rim_metres = static_cast<double>(rim);
                            changed = true;
                        }
                        if (ImGui::IsItemHovered())
                            ImGui::SetTooltip("How far the rim stands above it. The ejecta\n"
                                              "blanket decays from this height.");
                        break;
                    }
                }

                return changed;
            }

            /**
             * @brief Draws every field of the last frame's node selection.
             *
             * A copy taken once, because the numbers are only consistent with each other
             * as of the frame that produced them.
             */
            void draw_selection_statistics(const Field::ITerrainAuthoring& terrain)
            {
                const Field::QuadtreeStatistics statistics = terrain.selection_statistics();
                value_row("Nodes drawn", "%zu", statistics.selected);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Nodes the selection emitted; one instanced patch each.");
                value_row("Nodes visited", "%zu", statistics.visited);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Nodes considered, rejected ones included. The cost of\n"
                                      "the walk rather than of the draw.");
                value_row("Nodes rejected", "%zu", statistics.rejected);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Nodes the frustum removed, with everything beneath\n"
                                      "them. Zero means the whole body was in view.");
                value_row("Deepest level", "%u", static_cast<unsigned>(statistics.deepest));
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Deepest quadtree level reached this frame. Each level\n"
                                      "halves the ground a node covers.");
                if (statistics.budget_exhausted)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%-22s %s", "Node budget",
                                       "exhausted: ground is coarser than asked for");
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("The selection stopped refining before it met the\n"
                                          "screen-error target, so the whole body is drawn\n"
                                          "at a lower resolution rather than left with holes.");
                }
                else
                {
                    value_row("Node budget", "%s", "within budget");
                }

                const std::size_t pending = terrain.pending_recompile_count();
                if (pending > 0)
                {
                    ImGui::TextColored(ImVec4(1.0f, 0.7f, 0.2f, 1.0f), "%-22s %zu",
                                       "Tiles to recompile", pending);
                    if (ImGui::IsItemHovered())
                        ImGui::SetTooltip("Ground still carrying its shape from before an\n"
                                          "edit. Recompilation is bounded per frame, so a\n"
                                          "wide edit resolves over several of them.");
                }
                else
                {
                    value_row("Tiles to recompile", "%s", "none; the ground matches the stack");
                }
            }

            /** @brief The order a new layer should take: past everything already in the stack. */
            std::uint32_t next_free_order(const Field::ITerrainAuthoring& terrain)
            {
                const std::size_t count = terrain.layer_count();
                if (count == 0)
                    return 0;
                return terrain.layer(count - 1).order + 1u;
            }
        } // namespace

        void draw_terrain_panel(EditorContext& context, TerrainPanelState& state,
                                SushiEngine::Terrain::ITerrainAuthoring* terrain)
        {
            if (!context.panels.terrain)
                return;
            if (!ImGui::Begin("Terrain", &context.panels.terrain))
            {
                ImGui::End();
                return;
            }

            if (terrain == nullptr)
            {
                ImGui::TextUnformatted("This viewport draws no planetary terrain.");
                ImGui::End();
                return;
            }

            const int body = terrain->body();
            const char* body_name =
                body >= 0 && body < SushiEngine::Astro::BODY_COUNT
                    ? SushiEngine::Astro::body_properties(
                          static_cast<SushiEngine::Astro::BodyId>(body))
                          .name
                    : "none";
            const double mean_radius = terrain->mean_radius_metres();

            value_row("Body", "%s", body_name);
            if (!terrain->loaded() || mean_radius <= 0.0)
            {
                ImGui::TextUnformatted("No baked terrain under this body, so there is nothing "
                                       "for a layer to reshape.");
                ImGui::TextDisabled("Bake one with `se planet bake`; the analytic ground is "
                                    "drawn until then.");
                ImGui::End();
                return;
            }
            value_row("Mean radius", "%.0f m", mean_radius);

            // Re-seeded rather than kept, because the stack is per body: travelling to
            // another world drops the layers authored for this one, and a draft measured
            // in this body's radians would mean a different size over there.
            if (state.draft_body != body)
            {
                state.draft_body = body;
                state.draft = Field::TerrainLayer{};
                state.draft.operation = Field::LayerOperation::Crater;
                state.draft.footprint.direction = terrain->view_direction();
                state.draft.footprint.inner_radians = 300.0 / mean_radius;
                state.draft.footprint.outer_radians = 900.0 / mean_radius;
                state.draft.profile.depth_metres = 60.0;
                state.draft.profile.rim_metres = 15.0;
            }

            ImGui::Separator();
            if (ImGui::CollapsingHeader("Selection", ImGuiTreeNodeFlags_DefaultOpen))
                draw_selection_statistics(*terrain);

            ImGui::Separator();
            ImGui::Text("Layers (%zu), applied lowest order first", terrain->layer_count());
            if (ImGui::IsItemHovered())
                ImGui::SetTooltip("Composition order is the layer's own number, not its place\n"
                                  "in this list, so the ground is the same whichever order\n"
                                  "the edits arrived in.");

            // Every mutation is deferred to after the walk: applying one inside it would
            // change the stack the loop is indexing, and the row being drawn is the row
            // most likely to move.
            enum class Action
            {
                None,
                Update,
                Remove,
                MoveEarlier,
                MoveLater
            };
            Action action = Action::None;
            std::uint32_t action_order = 0;
            std::uint32_t action_other = 0;
            Field::TerrainLayer action_layer{};

            const std::size_t count = terrain->layer_count();
            for (std::size_t index = 0; index < count; ++index)
            {
                Field::TerrainLayer layer = terrain->layer(index);
                ImGui::PushID(static_cast<int>(layer.order));

                char title[96];
                std::snprintf(title, sizeof(title), "%zu. %s (order %u)", index + 1,
                              OPERATION_NAMES[static_cast<int>(layer.operation)], layer.order);
                if (ImGui::CollapsingHeader(title))
                {
                    if (draw_layer_fields(layer, mean_radius, terrain->view_direction()))
                    {
                        action = Action::Update;
                        action_order = layer.order;
                        action_layer = layer;
                    }

                    ImGui::BeginDisabled(index == 0);
                    if (ImGui::Button("Move Earlier"))
                    {
                        action = Action::MoveEarlier;
                        action_order = layer.order;
                        action_other = terrain->layer(index - 1).order;
                    }
                    ImGui::EndDisabled();
                    ImGui::SameLine();
                    ImGui::BeginDisabled(index + 1 >= count);
                    if (ImGui::Button("Move Later"))
                    {
                        action = Action::MoveLater;
                        action_order = layer.order;
                        action_other = terrain->layer(index + 1).order;
                    }
                    ImGui::EndDisabled();
                    ImGui::SameLine();
                    if (ImGui::Button("Remove"))
                    {
                        action = Action::Remove;
                        action_order = layer.order;
                    }
                }
                ImGui::PopID();
            }

            switch (action)
            {
                case Action::None:
                    break;
                case Action::Update:
                    terrain->update_layer(action_order, action_layer);
                    break;
                case Action::Remove:
                    if (terrain->remove_layer(action_order))
                        editor_log(context, "Removed terrain layer " +
                                                std::to_string(action_order) + ".");
                    break;
                case Action::MoveEarlier:
                case Action::MoveLater:
                    terrain->swap_layer_order(action_order, action_other);
                    break;
            }

            ImGui::Separator();
            if (ImGui::CollapsingHeader("New Layer", ImGuiTreeNodeFlags_DefaultOpen))
            {
                ImGui::PushID("draft");
                (void)draw_layer_fields(state.draft, mean_radius, terrain->view_direction());
                if (ImGui::Button("Add Layer"))
                {
                    // Assigned here rather than authored: the number is composition order,
                    // and the only thing an author means by adding is "after the rest".
                    // Moving it afterwards is what the Move buttons are for.
                    state.draft.order = next_free_order(*terrain);
                    if (terrain->insert_layer(state.draft))
                    {
                        editor_log(context, std::string("Added a terrain ") +
                                                OPERATION_NAMES[static_cast<int>(
                                                    state.draft.operation)] +
                                                " layer at order " +
                                                std::to_string(state.draft.order) + ".");
                    }
                    else
                    {
                        editor_log(context,
                                   "Could not add the terrain layer: order " +
                                       std::to_string(state.draft.order) + " is already taken.",
                                   LogLevel::Error);
                    }
                }
                ImGui::PopID();
            }

            ImGui::Separator();
            ImGui::BeginDisabled(terrain->layer_count() == 0);
            if (ImGui::Button("Remove All Layers"))
            {
                terrain->clear_layers();
                editor_log(context, "Removed every terrain layer from this body.");
            }
            ImGui::EndDisabled();
            ImGui::TextDisabled("Layers are not saved with the scene yet: they are held by "
                                "the view and dropped when it travels to another body.");

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
