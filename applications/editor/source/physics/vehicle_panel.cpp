/**************************************************************************/
/* vehicle_panel.cpp                                                      */
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

#include "vehicle_panel.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>

#include <imgui.h>

#include "../ui/panel_widgets.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        namespace
        {
            constexpr float GRAVITY = 9.81f;

            /** @brief A derived number, drawn dimmed so it reads as a consequence. */
            void derived_row(const char* label, const char* format, double value,
                             const char* unit)
            {
                ImGui::TextDisabled("%-24s", label);
                ImGui::SameLine();
                ImGui::TextDisabled(format, value);
                ImGui::SameLine();
                ImGui::TextDisabled("%s", unit);
            }

            /** @brief A derived number that is a warning when it is out of range. */
            void checked_row(const char* label, const char* format, double value,
                             const char* unit, bool sensible, const char* why)
            {
                if (sensible)
                {
                    derived_row(label, format, value, unit);
                    return;
                }
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1.0f), "%-24s", label);
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1.0f), format, value);
                ImGui::SameLine();
                ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.2f, 1.0f), "%s  %s", unit, why);
            }

            /** @brief Four corners at a sensible wheelbase, front steering, rear drive. */
            void seed_corners(Physics::VehicleAsset& asset)
            {
                const Scalar x[4] = {Scalar(-0.75), Scalar(0.75), Scalar(0.75), Scalar(-0.75)};
                const Scalar z[4] = {Scalar(-1.4), Scalar(-1.4), Scalar(1.4), Scalar(1.4)};
                for (int i = 0; i < 4; ++i)
                {
                    Physics::SuspensionSetup corner;
                    corner.mount = Vector3{x[i], Scalar(0.6), z[i]};
                    corner.axis = Vector3{0, 1, 0};
                    // Both sides point the same way: the axle is a convention the whole
                    // vehicle shares, not a description of which way the hub cap faces.
                    corner.axle = Vector3{1, 0, 0};
                    corner.steered = z[i] < 0;
                    corner.driven = z[i] > 0;
                    asset.corners.push_back(corner);
                }
            }

            /** @brief A five-speed with a reverse and a neutral, in the one ordered list. */
            void seed_powertrain(Physics::PowertrainSettings& powertrain)
            {
                powertrain.engine.curve = {{80, 130}, {220, 280}, {420, 310}, {620, 240}};
                powertrain.gearbox.ratios = {Scalar(-3.2), 0,           Scalar(3.4),
                                             Scalar(2.1),  Scalar(1.4), 1,
                                             Scalar(0.8)};
            }

            /** @brief The mass one corner carries, as the author's own split of the total. */
            float corner_mass(const Physics::VehicleAsset& asset, float total_mass)
            {
                const std::size_t corners = asset.corners.size();
                return corners > 0 ? total_mass / float(corners) : 0.0f;
            }

            void draw_corner(Physics::SuspensionSetup& corner, float carried)
            {
                float mount[3] = {float(corner.mount.x), float(corner.mount.y),
                                  float(corner.mount.z)};
                if (ImGui::DragFloat3("Mount", mount, 0.01f))
                    corner.mount = Vector3{Scalar(mount[0]), Scalar(mount[1]), Scalar(mount[2])};

                float axis[3] = {float(corner.axis.x), float(corner.axis.y),
                                 float(corner.axis.z)};
                if (ImGui::DragFloat3("Strut axis", axis, 0.01f))
                    corner.axis = Vector3{Scalar(axis[0]), Scalar(axis[1]), Scalar(axis[2])};
                ImGui::SetItemTooltip("Also the steering axis: this is a MacPherson strut.");

                float rest = float(corner.rest_length);
                if (ImGui::DragFloat("Rest length", &rest, 0.005f, 0.05f, 1.0f, "%.3f m"))
                    corner.rest_length = Scalar(rest);

                float bump = float(corner.travel_bump);
                float droop = float(corner.travel_droop);
                if (ImGui::DragFloat("Bump travel", &bump, 0.005f, 0.0f, 0.5f, "%.3f m"))
                    corner.travel_bump = Scalar(bump);
                if (ImGui::DragFloat("Droop travel", &droop, 0.005f, 0.0f, 0.5f, "%.3f m"))
                    corner.travel_droop = Scalar(droop);

                float rate = float(corner.spring_rate);
                if (ImGui::DragFloat("Spring rate", &rate, 250.0f, 0.0f, 500000.0f, "%.0f N/m"))
                    corner.spring_rate = Scalar(rate);

                float damping = float(corner.damping);
                if (ImGui::DragFloat("Damper rate", &damping, 0.1f, 0.0f, 60.0f, "%.2f 1/s"))
                    corner.damping = Scalar(damping);

                float wheel_mass = float(corner.wheel_mass);
                float radius = float(corner.wheel_radius);
                if (ImGui::DragFloat("Wheel mass", &wheel_mass, 0.5f, 1.0f, 200.0f, "%.1f kg"))
                    corner.wheel_mass = Scalar(wheel_mass);
                if (ImGui::DragFloat("Wheel radius", &radius, 0.005f, 0.05f, 1.5f, "%.3f m"))
                    corner.wheel_radius = Scalar(radius);

                bool steered = corner.steered;
                bool driven = corner.driven;
                if (ImGui::Checkbox("Steered", &steered))
                    corner.steered = steered;
                ImGui::SameLine();
                if (ImGui::Checkbox("Driven", &driven))
                    corner.driven = driven;

                ImGui::Separator();
                ImGui::TextDisabled("What that corner does");

                // The one number an author can predict and the one a mistake moves: a
                // spring rate read as a compliance, or a travel signed the wrong way,
                // still holds a car up and still moves when it is pushed. What it does
                // not do is settle here.
                const double sag = corner.spring_rate > 0
                                       ? double(carried) * double(GRAVITY) /
                                             double(corner.spring_rate)
                                       : 0.0;
                checked_row("Static ride height", "%.4f", sag, "m of sag",
                            sag < double(corner.travel_bump),
                            "-- the car sits on its bump stop");
                derived_row("Corner load", "%.1f", double(carried) * double(GRAVITY), "N");

                const double axial = 0.5 * double(corner.wheel_mass) *
                                     double(corner.wheel_radius) * double(corner.wheel_radius);
                derived_row("Wheel inertia", "%.4f", axial, "kg m^2 about the axle");
            }

            void draw_tyre(Physics::TyreSettings& tyre, float carried)
            {
                float friction = float(tyre.friction);
                if (ImGui::DragFloat("Peak friction", &friction, 0.01f, 0.0f, 3.0f, "%.2f"))
                    tyre.friction = Scalar(friction);
                ImGui::SetItemTooltip("Zero switches the model off and leaves the wheel to the "
                                      "solver's own Coulomb friction.");

                float longitudinal = float(tyre.longitudinal_stiffness);
                float lateral = float(tyre.lateral_stiffness);
                if (ImGui::DragFloat("Slip stiffness", &longitudinal, 0.25f, 0.0f, 60.0f, "%.1f"))
                    tyre.longitudinal_stiffness = Scalar(longitudinal);
                if (ImGui::DragFloat("Cornering stiffness", &lateral, 0.25f, 0.0f, 60.0f, "%.1f"))
                    tyre.lateral_stiffness = Scalar(lateral);

                float rated = float(tyre.rated_load);
                float sensitivity = float(tyre.load_sensitivity);
                if (ImGui::DragFloat("Rated load", &rated, 25.0f, 100.0f, 50000.0f, "%.0f N"))
                    tyre.rated_load = Scalar(rated);
                if (ImGui::DragFloat("Load sensitivity", &sensitivity, 0.005f, 0.0f, 1.0f, "%.3f"))
                    tyre.load_sensitivity = Scalar(sensitivity);

                float reference = float(tyre.low_speed_reference);
                if (ImGui::DragFloat("Low-speed reference", &reference, 0.1f, 0.1f, 20.0f,
                                     "%.2f m/s"))
                {
                    tyre.low_speed_reference = Scalar(reference);
                }

                ImGui::Separator();
                ImGui::TextDisabled("What that tyre does at this corner's load");

                const Scalar load = Scalar(double(carried) * double(GRAVITY));
                const Scalar mu = Physics::tyre_friction(tyre, load);
                derived_row("Friction here", "%.3f", double(mu), "");
                derived_row("Grip available", "%.1f", double(mu * load), "N");
                derived_row("Lateral limit", "%.2f", double(mu), "g");
            }

            void draw_powertrain(Physics::PowertrainSettings& powertrain, int& gear,
                                 float wheel_radius)
            {
                float inertia = float(powertrain.engine.inertia);
                if (ImGui::DragFloat("Crank inertia", &inertia, 0.01f, 0.01f, 5.0f, "%.3f kg m^2"))
                    powertrain.engine.inertia = Scalar(inertia);

                float idle = float(powertrain.engine.idle_rate);
                float band = float(powertrain.engine.idle_band);
                float limit = float(powertrain.engine.limit_rate);
                if (ImGui::DragFloat("Idle", &idle, 1.0f, 0.0f, 400.0f, "%.0f rad/s"))
                    powertrain.engine.idle_rate = Scalar(idle);
                if (ImGui::DragFloat("Idle band", &band, 0.5f, 1.0f, 200.0f, "%.0f rad/s"))
                    powertrain.engine.idle_band = Scalar(band);
                ImGui::SetItemTooltip("The governor's proportional band, and therefore its "
                                      "droop: the engine idles a little under its target.");
                if (ImGui::DragFloat("Limiter", &limit, 2.0f, 50.0f, 1500.0f, "%.0f rad/s"))
                    powertrain.engine.limit_rate = Scalar(limit);

                float clutch = float(powertrain.clutch_capacity);
                if (ImGui::DragFloat("Clutch capacity", &clutch, 10.0f, 0.0f, 5000.0f, "%.0f Nm"))
                    powertrain.clutch_capacity = Scalar(clutch);

                float final_drive = float(powertrain.gearbox.final_drive);
                if (ImGui::DragFloat("Final drive", &final_drive, 0.01f, 0.5f, 12.0f, "%.3f"))
                    powertrain.gearbox.final_drive = Scalar(final_drive);

                float lock = float(powertrain.differential.lock_torque);
                if (ImGui::DragFloat("Differential lock", &lock, 10.0f, 0.0f, 20000.0f, "%.0f Nm"))
                    powertrain.differential.lock_torque = Scalar(lock);
                ImGui::SetItemTooltip("Zero is an open differential, large is a spool, and "
                                      "between is a limited-slip.");

                ImGui::Separator();
                ImGui::TextDisabled("Gears");
                const int count = int(powertrain.gearbox.ratios.size());
                if (gear >= count)
                    gear = count > 0 ? count - 1 : 0;
                for (int i = 0; i < count; ++i)
                {
                    ImGui::PushID(i);
                    float ratio = float(powertrain.gearbox.ratios[std::size_t(i)]);
                    if (ImGui::RadioButton("##select", gear == i))
                        gear = i;
                    ImGui::SameLine();
                    ImGui::SetNextItemWidth(140.0f);
                    if (ImGui::DragFloat("##ratio", &ratio, 0.01f, -12.0f, 12.0f, "%.3f"))
                        powertrain.gearbox.ratios[std::size_t(i)] = Scalar(ratio);
                    ImGui::SameLine();
                    if (ratio == 0.0f)
                        ImGui::TextDisabled("neutral");
                    else if (ratio < 0.0f)
                        ImGui::TextDisabled("reverse");
                    else
                        ImGui::TextDisabled("gear %d", i);
                    ImGui::PopID();
                }
                if (ImGui::SmallButton("Add gear"))
                    powertrain.gearbox.ratios.push_back(Scalar(1));
                ImGui::SameLine();
                if (ImGui::SmallButton("Remove last") && count > 0)
                    powertrain.gearbox.ratios.pop_back();

                ImGui::Separator();
                ImGui::TextDisabled("What the selected gear does");
                const double ratio =
                    gear < count ? double(powertrain.gearbox.ratios[std::size_t(gear)]) *
                                       double(powertrain.gearbox.final_drive)
                                 : 0.0;
                if (ratio == 0.0)
                {
                    ImGui::TextDisabled("%-24s neutral", "Road speed at the limiter");
                    return;
                }
                // The number that catches a ratio typed with the wrong sign or a decimal
                // point in the wrong place, neither of which is visible in the ratio.
                const double speed =
                    double(powertrain.engine.limit_rate) / ratio * double(wheel_radius);
                checked_row("Road speed at limiter", "%.2f", speed * 3.6, "km/h",
                            std::fabs(speed) < 140.0, "-- faster than any road car");
                derived_row("Engine at 100 km/h", "%.0f",
                            std::fabs(100.0 / 3.6 / double(wheel_radius) * ratio), "rad/s");
            }

            void draw_aerodynamics(Physics::VehicleAsset::Aerodynamics& aero)
            {
                float area = float(aero.frontal_area);
                float drag = float(aero.drag_coefficient);
                float down = float(aero.downforce_coefficient);
                if (ImGui::DragFloat("Frontal area", &area, 0.01f, 0.0f, 20.0f, "%.2f m^2"))
                    aero.frontal_area = Scalar(area);
                if (ImGui::DragFloat("Drag coefficient", &drag, 0.005f, 0.0f, 2.0f, "%.3f"))
                    aero.drag_coefficient = Scalar(drag);
                if (ImGui::DragFloat("Downforce coefficient", &down, 0.01f, 0.0f, 6.0f, "%.3f"))
                    aero.downforce_coefficient = Scalar(down);
                ImGui::SetItemTooltip("Zero for a road car. A saloon generates a little lift, "
                                      "and pretending otherwise is how one corners like a "
                                      "prototype.");

                float pressure[3] = {float(aero.center_of_pressure.x),
                                     float(aero.center_of_pressure.y),
                                     float(aero.center_of_pressure.z)};
                if (ImGui::DragFloat3("Centre of pressure", pressure, 0.01f))
                {
                    aero.center_of_pressure =
                        Vector3{Scalar(pressure[0]), Scalar(pressure[1]), Scalar(pressure[2])};
                }
                ImGui::SetItemTooltip("Behind the centre of mass on a car with a wing, ahead of "
                                      "it on one with a splitter. The difference is the whole "
                                      "of aerodynamic balance.");

                ImGui::Separator();
                ImGui::TextDisabled("At 200 km/h");
                const Scalar speed = Scalar(200.0 / 3.6);
                derived_row("Drag", "%.0f",
                            double(Physics::aerodynamic_force(speed, aero.frontal_area,
                                                              aero.drag_coefficient,
                                                              aero.air_density)),
                            "N");
                derived_row("Downforce", "%.0f",
                            double(Physics::aerodynamic_force(speed, aero.frontal_area,
                                                              aero.downforce_coefficient,
                                                              aero.air_density)),
                            "N");
            }
            /**
             * @brief Puts the authored vehicle on the selected entity, drives it, and reads it back.
             *
             * The half of §14's vehicle-editor bullet the rest of this panel cannot cover on
             * its own: node/beam visualization, the mass distribution, and a drive surface.
             * All three need a *live* vehicle, and a live vehicle is a `VehicleInstance`
             * component on an entity — which is why this tab exists rather than a fourth
             * derived column.
             *
             * The panel edits the selected entity's component here and a document everywhere
             * else, and that split is honest rather than awkward: the corners and the
             * drivetrain are the same numbers whether or not anything is placed, and only
             * this tab is about a car that is in the world.
             *
             * @param context Editor state; supplies the selection and the undo history.
             * @param state   The vehicle under edit.
             */
            void draw_scene_tab(EditorContext& context, VehicleAuthoringState& state)
            {
                Simulation::IWorldEditor* world = world_of(context);
                if (world == nullptr || !world->exists(context.selected_entity))
                {
                    ImGui::TextDisabled("Select an entity to put this vehicle on.");
                    return;
                }
                const Simulation::EntityId id = context.selected_entity;

                if (!world->has_vehicle(id))
                {
                    ImGui::TextDisabled("%s carries no Vehicle.", world->name(id).c_str());
                    if (ImGui::Button("Add Vehicle Component"))
                    {
                        context.history.record(*world);
                        world->set_has_vehicle(id, true);
                    }
                    return;
                }

                Simulation::VehicleInstanceParameters parameters = world->vehicle_parameters(id);
                char path[512];
                std::snprintf(path, sizeof(path), "%s", parameters.asset_path.c_str());
                if (ImGui::InputText("Structure (.sushinodebeam)", path, sizeof(path)))
                {
                    context.history.record(*world);
                    parameters.asset_path = path;
                    parameters.setup = state.asset;
                    world->set_vehicle_parameters(id, parameters);
                }
                ImGui::SetItemTooltip("The cooked structure this vehicle is built from. The "
                                      "corners, tyres and drivetrain come from this window's "
                                      "other tabs; press Apply Setup after changing them.");

                if (ImGui::Button("Apply Setup"))
                {
                    // A rebuild, and the panel says so: a vehicle is four hundred bodies
                    // placed relative to a cooked structure, so there is no patching one —
                    // the car is taken down and put back at the new numbers.
                    context.history.record(*world);
                    parameters.setup = state.asset;
                    world->set_vehicle_parameters(id, parameters);
                }
                ImGui::SameLine();
                ImGui::TextDisabled("rebuilds the vehicle at this window's numbers");

                ImGui::SeparatorText("Drive");
                Simulation::VehicleInput input = world->vehicle_input(id);
                bool driven = false;
                float throttle = to_float(input.throttle);
                float brake = to_float(input.brake);
                float steer = to_float(input.steer) * 57.2957795f;
                float clutch = to_float(input.clutch);
                int gear = int(input.gear);
                const int top_gear =
                    int(state.asset.powertrain.gearbox.ratios.size()) - 1;
                driven |= ImGui::SliderFloat("Throttle", &throttle, 0.0f, 1.0f, "%.2f");
                driven |= ImGui::SliderFloat("Brake", &brake, 0.0f, 6000.0f, "%.0f N.m");
                driven |= ImGui::SliderFloat("Steer", &steer, -35.0f, 35.0f, "%.1f deg");
                driven |= ImGui::SliderFloat("Clutch", &clutch, 0.0f, 1.0f, "%.2f");
                driven |= ImGui::SliderInt("Gear", &gear, 0, top_gear > 0 ? top_gear : 0);
                if (driven)
                {
                    input.throttle = Scalar(throttle);
                    input.brake = Scalar(brake);
                    input.steer = Scalar(steer / 57.2957795f);
                    input.clutch = Scalar(clutch);
                    input.gear = std::size_t(gear < 0 ? 0 : gear);
                    // Not an undo step. A pedal is a control, not an edit: recording throttle
                    // into the history would fill it with a hundred entries a second and bury
                    // the authoring changes it exists to protect.
                    world->set_vehicle_input(id, input);
                }

                ImGui::SeparatorText("Live");
                Simulation::VehicleReport report;
                if (!world->vehicle_report(id, report))
                {
                    // Two states, distinguished, because both read as a car that is not
                    // moving: no path authored yet, and a path that did not load.
                    if (parameters.asset_path.empty())
                        ImGui::TextDisabled("No structure named yet.");
                    else
                        ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                                           "'%s' did not load as a node-beam asset.",
                                           parameters.asset_path.c_str());
                    return;
                }

                derived_row("Engine", "%.0f", double(report.engine_rate) * 9.5493, "rpm");
                derived_row("Engine torque", "%.1f", double(report.engine_torque), "N.m");
                derived_row("Clutch torque", "%.1f", double(report.clutch_torque), "N.m");
                if (report.clutch_slipping)
                    ImGui::TextColored(ImVec4(1.0f, 0.75f, 0.3f, 1.0f),
                                       "Clutch slipping at %.1f rad/s",
                                       double(report.clutch_slip));
                derived_row("Beams broken", "%.0f", double(report.beams_broken), "");
                derived_row("Parts detached", "%.0f", double(report.parts_detached), "");

                // §14's node/beam visualization: the shell's nodes, drawn as a side
                // elevation inside the panel. Its own small view rather than the Scene
                // viewport, following the editor's own rule that a preview gets its own
                // surface — and a side elevation is where a sagging suspension and a caved
                // panel are both visible, which is what this view is for.
                if (!world->vehicle_node_positions(id, state.node_positions) ||
                    state.node_positions.empty())
                    return;

                ImGui::SeparatorText("Shell");
                ImGui::TextDisabled("%d nodes", int(state.node_positions.size()));

                Vector3 low = state.node_positions.front();
                Vector3 high = low;
                for (const Vector3& node : state.node_positions)
                {
                    low.x = node.x < low.x ? node.x : low.x;
                    low.y = node.y < low.y ? node.y : low.y;
                    high.x = node.x > high.x ? node.x : high.x;
                    high.y = node.y > high.y ? node.y : high.y;
                }
                const double span_x = double(high.x - low.x);
                const double span_y = double(high.y - low.y);
                const double span = span_x > span_y ? span_x : span_y;
                if (!(span > 0.0))
                    return;

                const ImVec2 origin = ImGui::GetCursorScreenPos();
                const float width = ImGui::GetContentRegionAvail().x;
                const float height = 140.0f;
                ImDrawList* list = ImGui::GetWindowDrawList();
                list->AddRectFilled(origin, ImVec2(origin.x + width, origin.y + height),
                                    IM_COL32(20, 20, 24, 255));
                const float scale = float(double(height - 16.0f) / span);
                for (const Vector3& node : state.node_positions)
                {
                    const float x = origin.x + 8.0f + float(double(node.x - low.x)) * scale;
                    // Screen y grows downward and the world's grows up, so the elevation is
                    // measured from the *bottom* of the box upward — without which the car
                    // is drawn upside down and reads as a cook bug.
                    const float y =
                        origin.y + height - 8.0f - float(double(node.y - low.y)) * scale;
                    list->AddRectFilled(ImVec2(x - 1.0f, y - 1.0f), ImVec2(x + 1.0f, y + 1.0f),
                                        IM_COL32(120, 200, 255, 220));
                }
                ImGui::Dummy(ImVec2(width, height));
                ImGui::TextDisabled("Side elevation, %.2f x %.2f m", span_x, span_y);
            }

        } // namespace

        void draw_vehicle_panel(EditorContext& context, VehicleAuthoringState& state)
        {
            if (!context.panels.vehicle)
                return;
            if (!ImGui::Begin("Vehicle", &context.panels.vehicle))
            {
                ImGui::End();
                return;
            }

            if (!state.seeded)
            {
                if (state.asset.corners.empty())
                    seed_corners(state.asset);
                if (state.asset.powertrain.gearbox.ratios.empty())
                    seed_powertrain(state.asset.powertrain);
                state.seeded = true;
            }

            // The sprung mass an author is dividing between the corners. Not stored on the
            // asset, because mass is the *structure's* — it comes out of the cooked
            // `.sushinodebeam` — and duplicating it here would be a second source of truth
            // that could disagree with the cook.
            static float sprung_mass = 1200.0f;
            ImGui::DragFloat("Sprung mass", &sprung_mass, 5.0f, 100.0f, 20000.0f, "%.0f kg");
            ImGui::SetItemTooltip("From the cooked structure in a real asset; here it is the "
                                  "number the derived rows are computed against.");
            const float carried = corner_mass(state.asset, sprung_mass);

            ImGui::Separator();

            if (ImGui::BeginTabBar("VehicleTabs"))
            {
                if (ImGui::BeginTabItem("Corners"))
                {
                    const int count = int(state.asset.corners.size());
                    if (state.selected_corner >= count)
                        state.selected_corner = count > 0 ? count - 1 : 0;
                    for (int i = 0; i < count; ++i)
                    {
                        if (i > 0)
                            ImGui::SameLine();
                        ImGui::PushID(i);
                        char label[16];
                        std::snprintf(label, sizeof(label), "%d", i);
                        if (ImGui::RadioButton(label, state.selected_corner == i))
                            state.selected_corner = i;
                        ImGui::PopID();
                    }
                    if (ImGui::SmallButton("Add corner"))
                        state.asset.corners.push_back(Physics::SuspensionSetup{});
                    ImGui::SameLine();
                    if (ImGui::SmallButton("Remove last") && count > 0)
                        state.asset.corners.pop_back();

                    ImGui::Separator();
                    if (!state.asset.corners.empty())
                    {
                        draw_corner(state.asset.corners[std::size_t(state.selected_corner)],
                                    carried);
                    }
                    else
                    {
                        ImGui::TextDisabled("No corners. A vehicle with none is a trailer, "
                                            "which is a legitimate asset.");
                    }
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Tyres"))
                {
                    if (state.asset.corners.empty())
                    {
                        ImGui::TextDisabled("A tyre belongs to a corner, and there are none.");
                    }
                    else
                    {
                        const std::size_t index = std::size_t(state.selected_corner);
                        ImGui::TextDisabled("Corner %d", state.selected_corner);
                        draw_tyre(state.asset.corners[index].tyre, carried);
                    }
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Drivetrain"))
                {
                    const float radius = state.asset.corners.empty()
                                             ? 0.34f
                                             : float(state.asset.corners[0].wheel_radius);
                    draw_powertrain(state.asset.powertrain, state.selected_gear, radius);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Aerodynamics"))
                {
                    draw_aerodynamics(state.asset.aerodynamics);
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Structure"))
                {
                    // The two identifiers `physics/vehicle` never dereferences: they are
                    // resolved by the layer that owns assets, and the panel is that layer's
                    // front end rather than a second resolver.
                    int structure = int(state.asset.structure_asset);
                    int collision = int(state.asset.core_collision_asset);
                    if (ImGui::InputInt("Node-beam asset", &structure))
                        state.asset.structure_asset = std::uint32_t(structure < 0 ? 0 : structure);
                    if (ImGui::InputInt("Core collision asset", &collision))
                        state.asset.core_collision_asset =
                            std::uint32_t(collision < 0 ? 0 : collision);

                    ImGui::Separator();
                    ImGui::TextDisabled(
                        "Parts and mounting points are authored in the cooked structure "
                        "(.sushinodebeam): a part is a node's part index and a mount is an "
                        "attachment record. This panel names the asset; it does not cook it "
                        "-- that is the Cook & Bake window.");
                    ImGui::EndTabItem();
                }

                if (ImGui::BeginTabItem("Scene"))
                {
                    draw_scene_tab(context, state);
                    ImGui::EndTabItem();
                }

                ImGui::EndTabBar();
            }

            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
