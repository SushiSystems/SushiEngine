/**************************************************************************/
/* assembly_panel.cpp                                                     */
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

#include "assembly_panel.hpp"

#include <cstdio>
#include <string>
#include <vector>

#include <imgui.h>

#include "../ui/panel_widgets.hpp"
#include "joint_widgets.hpp"

namespace SushiEngine
{
    namespace Editor
    {
        using Simulation::AssemblyJoint;
        using Simulation::AssemblyPart;
        using Simulation::ColliderShape;
        using Simulation::EntityId;
        using Simulation::IWorldEditor;
        using Simulation::JointState;
        using Simulation::NULL_ENTITY;
        using Simulation::PhysicsAssembly;

        namespace
        {
            /** @brief The amber the editor says "this will not do what you meant" in. */
            ImVec4 warning_color() { return ImVec4(1.0f, 0.75f, 0.3f, 1.0f); }

            /** @brief How many collision groups the filter matrix offers. */
            constexpr int MAX_GROUPS = 8;

            /**
             * @brief A part's display name, or a generated one when it has none.
             *
             * `PhysicsAssembly::part_names` is documented as being for the editor only and is
             * allowed to be shorter than the part list — a blob round-trip keeps the hashes
             * and can lose the strings — so every reader needs the fallback and there is one
             * of them.
             *
             * @param asset The assembly.
             * @param index The part to name.
             */
            std::string part_label(const PhysicsAssembly& asset, std::size_t index)
            {
                if (index < asset.part_names.size() && !asset.part_names[index].empty())
                    return asset.part_names[index];
                char buffer[32];
                std::snprintf(buffer, sizeof(buffer), "Part %d", int(index));
                return buffer;
            }

            /**
             * @brief The authoring collider a part's physics collider means.
             *
             * The inverse of `collider_from_parameters`, and lossy in exactly one direction that
             * is worth naming: a capsule becomes a `Cylinder`, because that is the primitive
             * the forward mapping turns into a capsule, and a cooked asset has no primitive
             * at all and falls back to a box of its own half-extents. An assembly of cooked
             * hulls instanced into the scene therefore collides as boxes until the Collider
             * component can name an asset — which is a gap in `ColliderParameters`, stated here
             * rather than silently approximated.
             *
             * @param collider The part's collider.
             * @return The authoring parameters that reproduce it.
             */
            Simulation::ColliderParameters to_collider_parameters(
                const Simulation::Collider& collider)
            {
                Simulation::ColliderParameters parameters;
                switch (collider.shape)
                {
                    case ColliderShape::Sphere:
                        parameters.kind = Simulation::PrimitiveKind::Sphere;
                        parameters.parameters =
                            Vector3{collider.radius, collider.radius, collider.radius};
                        break;
                    case ColliderShape::Capsule:
                        parameters.kind = Simulation::PrimitiveKind::Cylinder;
                        // The authored half-height includes the caps; the capsule's excludes
                        // them, so the radius goes back on.
                        parameters.parameters =
                            Vector3{collider.radius, collider.half_height + collider.radius,
                                    collider.radius};
                        break;
                    case ColliderShape::Plane:
                        parameters.kind = Simulation::PrimitiveKind::Plane;
                        parameters.parameters = collider.half_extents;
                        break;
                    case ColliderShape::Box:
                    case ColliderShape::CookedAsset:
                    default:
                        parameters.kind = Simulation::PrimitiveKind::Box;
                        parameters.parameters = collider.half_extents;
                        break;
                }
                return parameters;
            }

            /**
             * @brief Closes a field row: its undo bracket and its tooltip.
             *
             * `track_item_undo` has to run immediately after the widget while it is still
             * the *current item*, which is why this is a call at the end of each row rather
             * than a wrapper around one.
             */
            void finish_row(EditorContext& context, IWorldEditor& world, const char* tooltip)
            {
                track_item_undo(context, world);
                if (tooltip != nullptr && ImGui::IsItemHovered())
                    ImGui::SetTooltip("%s", tooltip);
            }

            /**
             * @brief A `Scalar` row, for the panel's own numbers.
             *
             * A plain drag rather than @ref scalar_field, and the distinction is load-bearing
             * rather than stylistic: that function is a *table row emitter* — it opens with
             * `ImGui::TableNextRow` — so it may only be called inside the two-column table the
             * Inspector's Transform block sets up. Calling it outside one dereferences a null
             * current table, which is a crash rather than a layout glitch. This panel draws
             * no such table, so it draws plain rows, exactly as `ComponentEditor::number`
             * does for the Inspector's component sections.
             */
            bool scalar_row(EditorContext& context, IWorldEditor& world, const char* label,
                            Scalar& value, float speed, float low, float high, const char* format,
                            const char* tooltip)
            {
                float shown = to_float(value);
                const bool changed = ImGui::DragFloat(label, &shown, speed, low, high, format);
                finish_row(context, world, tooltip);
                if (!changed)
                    return false;
                value = Scalar(shown);
                return true;
            }

            /** @brief A `Vector3` row; see @ref scalar_row for why it is not @ref vector3_field. */
            bool vector_row(EditorContext& context, IWorldEditor& world, const char* label,
                            Vector3& value, float speed, const char* format, const char* tooltip)
            {
                float components[3] = {to_float(value.x), to_float(value.y), to_float(value.z)};
                const bool changed =
                    ImGui::DragFloat3(label, components, speed, 0.0f, 0.0f, format);
                finish_row(context, world, tooltip);
                if (!changed)
                    return false;
                value = Vector3{Scalar(components[0]), Scalar(components[1]),
                                Scalar(components[2])};
                return true;
            }

            /**
             * @brief Fills an empty assembly with a chassis and a door hinged to it.
             *
             * The §13.1 scene P3's acceptance criterion is written against — *"the
             * chassis-plus-hinged-door scene works end to end"* — so the panel opens on a
             * working assembly rather than on nothing. An author's first action is to change
             * it, which is the point: editing a working assembly teaches what the fields do,
             * and an empty list teaches nothing.
             *
             * @param asset The assembly to fill.
             */
            void seed_assembly(PhysicsAssembly& asset)
            {
                AssemblyPart chassis;
                chassis.collider.shape = ColliderShape::Box;
                chassis.collider.half_extents = Vector3{1.5, 0.6, 0.8};
                chassis.density = Scalar(400);
                chassis.group = 0;

                AssemblyPart door;
                door.collider.shape = ColliderShape::Box;
                door.collider.half_extents = Vector3{0.6, 0.5, 0.05};
                door.local_position = Vector3{0.4, 0.1, 0.87};
                door.density = Scalar(400);
                door.group = 0;

                AssemblyJoint hinge;
                hinge.part_a = 0;
                hinge.part_b = 1;
                hinge.parameters.type = Simulation::JointType::Hinge;
                hinge.parameters.anchor_a = Vector3{-0.2, 0.1, 0.85};
                hinge.parameters.anchor_b = Vector3{-0.6, 0.0, -0.02};
                hinge.parameters.axis_a = Vector3{0, 1, 0};
                hinge.parameters.axis_b = Vector3{0, 1, 0};
                hinge.parameters.twist_limit =
                    Simulation::JointLimitDescription{Scalar(0), Scalar(1.7), Scalar(0), true};
                hinge.parameters.break_force = Scalar(9000);

                asset.parts = {chassis, door};
                asset.part_names = {"Chassis", "Door"};
                asset.joints = {hinge};
                // One group, not colliding with itself: §10.2's filter line, which is what
                // stops the door's contact with the chassis fighting the hinge holding it.
                asset.group_masks = {Simulation::assembly_group_excluding_self(0)};
            }

            /**
             * @brief Creates the assembly's parts and joints as scene entities.
             *
             * One entity per part with a Transform, a Collider and a Rigid Body, then one
             * Physics Joint per assembly joint on the entity its first part became. The
             * joint's parameters cross unchanged, which is the whole reason `JointParameters` is
             * a shared value: an assembly joint and an authored joint are the same joint.
             *
             * @param context Editor state; the undo step is recorded once, around the lot.
             * @param world   The world to build in.
             * @param asset   The assembly to instance.
             * @param root    Where the assembly's root sits, in world metres.
             * @return The entities created, in part order.
             */
            std::vector<EntityId> instantiate_into_scene(EditorContext& context,
                                                         IWorldEditor& world,
                                                         const PhysicsAssembly& asset,
                                                         const Vector3& root)
            {
                std::vector<EntityId> created;
                if (asset.parts.empty())
                    return created;

                context.history.record(world);
                created.reserve(asset.parts.size());
                for (std::size_t i = 0; i < asset.parts.size(); ++i)
                {
                    const AssemblyPart& part = asset.parts[i];
                    const EntityId id = world.create_box(part_label(asset, i));

                    Simulation::EntityTransform transform = world.transform(id);
                    transform.position = root + part.local_position;
                    transform.rotation = part.local_orientation;
                    world.set_transform(id, transform);

                    Simulation::ColliderParameters collider = to_collider_parameters(part.collider);
                    collider.layer = part.group & 31u;
                    // The matrix decides what a group touches, and it decides it here rather
                    // than at each part, so a filter change is one edit instead of one per
                    // part that happens to be in the group.
                    collider.collides_with =
                        Simulation::assembly_group_mask(Simulation::to_view(asset), part.group);
                    world.set_has_collider(id, true);
                    world.set_collider_parameters(id, collider);

                    // The visual follows the collider, so an instanced assembly is visible as
                    // the shape it collides as rather than as whatever a fresh box happens to
                    // be. They are separate components and this is the one moment where the
                    // panel knows they should agree.
                    world.set_has_shape(id, true);
                    world.set_shape_parameters(
                        id, Simulation::ShapeParameters{collider.kind, collider.parameters});

                    Simulation::PhysicsBodyParameters body;
                    body.density = part.density;
                    body.inv_mass = part.inv_mass;
                    body.inv_inertia = part.inv_inertia;
                    body.drag_coefficient = part.drag_coefficient;
                    world.set_has_physics_body(id, true);
                    world.set_physics_body_parameters(id, body);

                    created.push_back(id);
                }

                for (const AssemblyJoint& joint : asset.joints)
                {
                    if (joint.part_a >= created.size() || joint.part_b >= created.size())
                        continue;
                    Simulation::PhysicsJointParameters parameters;
                    parameters.connected_body = created[joint.part_b];
                    parameters.joint = joint.parameters;
                    world.set_has_joint(created[joint.part_a], true);
                    world.set_joint_parameters(created[joint.part_a], parameters);
                }
                return created;
            }

            /** @brief Draws the parts list and the selected part's fields. */
            void draw_parts_tab(EditorContext& context, IWorldEditor& world,
                                AssemblyAuthoringState& state)
            {
                PhysicsAssembly& asset = state.asset;

                if (ImGui::Button("Add Part"))
                {
                    context.history.record(world);
                    asset.parts.push_back(AssemblyPart{});
                    asset.part_names.push_back("Part");
                    state.selected_part = int(asset.parts.size()) - 1;
                }
                ImGui::SameLine();
                const bool removable = asset.parts.size() > 1 && state.selected_part >= 0 &&
                                       state.selected_part < int(asset.parts.size());
                ImGui::BeginDisabled(!removable);
                if (ImGui::Button("Remove Part"))
                {
                    context.history.record(world);
                    const std::size_t index = std::size_t(state.selected_part);
                    asset.parts.erase(asset.parts.begin() + std::ptrdiff_t(index));
                    if (index < asset.part_names.size())
                        asset.part_names.erase(asset.part_names.begin() +
                                               std::ptrdiff_t(index));
                    // A joint naming the removed part has no meaning any more, and one
                    // naming a *later* part now names the wrong one — so the joints are
                    // repaired here rather than left to be discovered as a hinge that holds
                    // the wrong panel.
                    for (std::size_t j = asset.joints.size(); j-- > 0;)
                    {
                        AssemblyJoint& joint = asset.joints[j];
                        if (joint.part_a == index || joint.part_b == index)
                        {
                            asset.joints.erase(asset.joints.begin() + std::ptrdiff_t(j));
                            continue;
                        }
                        if (joint.part_a > index)
                            --joint.part_a;
                        if (joint.part_b > index)
                            --joint.part_b;
                    }
                    if (state.selected_part >= int(asset.parts.size()))
                        state.selected_part = int(asset.parts.size()) - 1;
                }
                ImGui::EndDisabled();

                if (ImGui::BeginListBox("##parts", ImVec2(-FLT_MIN, 6 * ImGui::GetTextLineHeightWithSpacing())))
                {
                    for (std::size_t i = 0; i < asset.parts.size(); ++i)
                    {
                        ImGui::PushID(int(i));
                        if (ImGui::Selectable(part_label(asset, i).c_str(),
                                              state.selected_part == int(i)))
                            state.selected_part = int(i);
                        ImGui::PopID();
                    }
                    ImGui::EndListBox();
                }

                if (state.selected_part < 0 || state.selected_part >= int(asset.parts.size()))
                    return;
                const std::size_t index = std::size_t(state.selected_part);
                AssemblyPart& part = asset.parts[index];

                ImGui::SeparatorText("Part");
                if (index < asset.part_names.size())
                {
                    char name[64];
                    std::snprintf(name, sizeof(name), "%s", asset.part_names[index].c_str());
                    if (ImGui::InputText("Name", name, sizeof(name)))
                    {
                        asset.part_names[index] = name;
                        part.name_hash = Simulation::assembly_name_hash(name);
                    }
                    track_item_undo(context, world);
                }

                static const char* const SHAPE_NAMES[] = {"Sphere", "Box", "Capsule", "Plane",
                                                          "Cooked Asset"};
                int shape = static_cast<int>(part.collider.shape);
                if (shape < 0 || shape > 4)
                    shape = 1;
                if (ImGui::BeginCombo("Shape", SHAPE_NAMES[shape]))
                {
                    for (int option = 0; option < 5; ++option)
                    {
                        if (!ImGui::Selectable(SHAPE_NAMES[option], option == shape))
                            continue;
                        context.history.record(world);
                        part.collider.shape = static_cast<ColliderShape>(option);
                    }
                    ImGui::EndCombo();
                }
                if (part.collider.shape == ColliderShape::Box ||
                    part.collider.shape == ColliderShape::Plane)
                    vector_row(context, world, "Half Extents", part.collider.half_extents, 0.01f,
                               "%.3f m",
                               "Half the volume's size along each local axis. For a Plane this "
                               "is the local normal instead.");
                if (part.collider.shape == ColliderShape::Sphere ||
                    part.collider.shape == ColliderShape::Capsule)
                    scalar_row(context, world, "Radius", part.collider.radius, 0.01f, 0.001f,
                               100.0f, "%.3f m", "The sphere's or capsule's radius.");
                if (part.collider.shape == ColliderShape::Capsule)
                    scalar_row(context, world, "Half Height", part.collider.half_height, 0.01f,
                               0.0f, 100.0f, "%.3f m",
                               "Half the straight section, excluding the round caps.");

                vector_row(context, world, "Local Position", part.local_position, 0.01f, "%.3f m",
                           "Where the part sits relative to the assembly's root, at rest.");

                ImGui::SeparatorText("Mass");
                scalar_row(context, world, "Density", part.density, 1.0f, 0.0f, 25000.0f,
                           "%.0f kg/m3",
                           "Above zero, mass and inertia are derived from the shape and this. "
                           "Steel is about 7800, oak 700, water 1000. Zero keeps the two "
                           "numbers below exactly as typed.");
                const bool derived = part.density > Scalar(0);
                ImGui::BeginDisabled(derived);
                scalar_row(context, world, "Inverse Mass", part.inv_mass, 0.01f, 0.0f, 100.0f,
                           "%.3f 1/kg", "One over the mass; zero pins the part in place.");
                vector_row(context, world, "Inverse Inertia", part.inv_inertia, 0.01f, "%.3f",
                           "Diagonal of the body-local inverse inertia; zero on an axis means "
                           "no rotation about it.");
                ImGui::EndDisabled();
                if (derived)
                {
                    const Simulation::Collider scaled =
                        Simulation::scaled_collider(part.collider, Vector3{1, 1, 1});
                    const Physics::MassProperties<Scalar> mass =
                        Simulation::collider_mass_properties(scaled, part.density);
                    ImGui::TextDisabled("Derived mass %.1f kg", double(mass.mass));
                }
                scalar_row(context, world, "Drag", part.drag_coefficient, 0.001f, 0.0f, 100.0f,
                           "%.4f 1/m",
                           "Quadratic drag k: acceleration -k|v|v per metre. Zero disables it.");

                int group = int(part.group);
                if (ImGui::DragInt("Group", &group, 0.1f, 0, MAX_GROUPS - 1))
                    part.group = std::uint32_t(group < 0 ? 0 : group);
                track_item_undo(context, world);
                if (ImGui::IsItemHovered())
                    ImGui::SetTooltip("Which collision group this part is in. What a group "
                                      "touches is the Groups tab's matrix.");
            }

            /** @brief Draws the joints list and the selected joint's parameters. */
            void draw_joints_tab(EditorContext& context, IWorldEditor& world,
                                 AssemblyAuthoringState& state)
            {
                PhysicsAssembly& asset = state.asset;

                ImGui::BeginDisabled(asset.parts.size() < 2);
                if (ImGui::Button("Add Joint"))
                {
                    context.history.record(world);
                    AssemblyJoint joint;
                    joint.part_a = 0;
                    joint.part_b = 1;
                    asset.joints.push_back(joint);
                    state.selected_joint = int(asset.joints.size()) - 1;
                }
                ImGui::EndDisabled();
                if (asset.parts.size() < 2 && ImGui::IsItemHovered())
                    ImGui::SetTooltip("A joint holds two parts; an assembly needs two before it "
                                      "can have one.");
                ImGui::SameLine();
                const bool removable = state.selected_joint >= 0 &&
                                       state.selected_joint < int(asset.joints.size());
                ImGui::BeginDisabled(!removable);
                if (ImGui::Button("Remove Joint"))
                {
                    context.history.record(world);
                    asset.joints.erase(asset.joints.begin() +
                                       std::ptrdiff_t(state.selected_joint));
                    if (state.selected_joint >= int(asset.joints.size()))
                        state.selected_joint = int(asset.joints.size()) - 1;
                }
                ImGui::EndDisabled();

                if (ImGui::BeginListBox("##joints", ImVec2(-FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing())))
                {
                    for (std::size_t i = 0; i < asset.joints.size(); ++i)
                    {
                        const AssemblyJoint& joint = asset.joints[i];
                        char label[128];
                        std::snprintf(label, sizeof(label), "%s -> %s",
                                      part_label(asset, joint.part_a).c_str(),
                                      part_label(asset, joint.part_b).c_str());
                        ImGui::PushID(int(i));
                        if (ImGui::Selectable(label, state.selected_joint == int(i)))
                            state.selected_joint = int(i);
                        ImGui::PopID();
                    }
                    ImGui::EndListBox();
                }

                if (state.selected_joint < 0 || state.selected_joint >= int(asset.joints.size()))
                    return;
                AssemblyJoint& joint = asset.joints[std::size_t(state.selected_joint)];

                ImGui::SeparatorText("Endpoints");
                // The one thing `draw_joint_parameters` deliberately does not draw: an assembly
                // joint names two part indices where an entity joint names a partner entity,
                // and pretending those are the same question is how one of them ends up
                // wrong.
                const auto part_combo = [&](const char* label, std::uint32_t& slot)
                {
                    if (!ImGui::BeginCombo(label, part_label(asset, slot).c_str()))
                        return;
                    for (std::size_t i = 0; i < asset.parts.size(); ++i)
                    {
                        if (!ImGui::Selectable(part_label(asset, i).c_str(), slot == i))
                            continue;
                        context.history.record(world);
                        slot = std::uint32_t(i);
                    }
                    ImGui::EndCombo();
                };
                part_combo("Part A", joint.part_a);
                part_combo("Part B", joint.part_b);
                if (joint.part_a == joint.part_b)
                    ImGui::TextColored(warning_color(),
                                       "A joint from a part to itself is degenerate, not stiff.");

                ImGui::SeparatorText("Held");
                draw_joint_parameters(context, world, joint.parameters);
            }

            /** @brief Draws the collision-filter matrix over the assembly's groups. */
            void draw_groups_tab(EditorContext& context, IWorldEditor& world,
                                 AssemblyAuthoringState& state)
            {
                PhysicsAssembly& asset = state.asset;

                int count = int(asset.group_masks.size());
                if (ImGui::DragInt("Groups", &count, 0.1f, 1, MAX_GROUPS))
                {
                    context.history.record(world);
                    const std::size_t wanted = std::size_t(count < 1 ? 1 : count);
                    // New groups default to "everything but myself", which is §10.2's line
                    // and the reason an assembly's parts do not fight the joints holding
                    // them. A permissive default would make every new group a bug that only
                    // shows up as a mechanism that will not settle.
                    while (asset.group_masks.size() < wanted)
                        asset.group_masks.push_back(Simulation::assembly_group_excluding_self(
                            std::uint32_t(asset.group_masks.size())));
                    asset.group_masks.resize(wanted);
                }
                track_item_undo(context, world);

                ImGui::TextDisabled(
                    "A row's ticks are the groups it collides with. Two parts touch only when "
                    "each one's group is ticked in the other's row, so clearing one side is "
                    "enough to separate them — and ticking one side is not enough to join "
                    "them.");

                if (!ImGui::BeginTable("filter", int(asset.group_masks.size()) + 1,
                                       ImGuiTableFlags_Borders | ImGuiTableFlags_SizingFixedFit))
                    return;
                ImGui::TableSetupColumn("");
                for (std::size_t column = 0; column < asset.group_masks.size(); ++column)
                {
                    char header[16];
                    std::snprintf(header, sizeof(header), "%d", int(column));
                    ImGui::TableSetupColumn(header);
                }
                ImGui::TableHeadersRow();

                for (std::size_t row = 0; row < asset.group_masks.size(); ++row)
                {
                    ImGui::TableNextRow();
                    ImGui::TableNextColumn();
                    ImGui::Text("%d", int(row));
                    for (std::size_t column = 0; column < asset.group_masks.size(); ++column)
                    {
                        ImGui::TableNextColumn();
                        ImGui::PushID(int(row * MAX_GROUPS + column));
                        const std::uint32_t bit = std::uint32_t(1) << column;
                        bool on = (asset.group_masks[row] & bit) != 0;
                        if (ImGui::Checkbox("##cell", &on))
                        {
                            context.history.record(world);
                            asset.group_masks[row] =
                                on ? (asset.group_masks[row] | bit)
                                   : (asset.group_masks[row] & ~bit);
                            // Symmetry is enforced rather than asked for: the filter test
                            // requires both directions, so a half-authored pair is a pair
                            // that does nothing and looks like it should.
                            if (column < asset.group_masks.size())
                            {
                                const std::uint32_t back = std::uint32_t(1) << row;
                                asset.group_masks[column] =
                                    on ? (asset.group_masks[column] | back)
                                       : (asset.group_masks[column] & ~back);
                            }
                        }
                        ImGui::PopID();
                    }
                }
                ImGui::EndTable();
            }

            /** @brief Draws instancing, and what every live joint in the scene is carrying. */
            void draw_scene_tab(EditorContext& context, IWorldEditor& world,
                                AssemblyAuthoringState& state)
            {
                vector_row(context, world, "Place At", state.instance_position, 0.05f, "%.2f m",
                           "Where the assembly's root goes. Its parts are placed relative to "
                           "it, so an assembly instanced twice is the same assembly twice.");
                if (ImGui::Button("Instantiate into Scene"))
                {
                    const std::vector<EntityId> created =
                        instantiate_into_scene(context, world, state.asset,
                                               state.instance_position);
                    if (!created.empty())
                        context.selected_entity = created.front();
                }
                ImGui::SameLine();
                ImGui::TextDisabled("%d parts, %d joints", int(state.asset.parts.size()),
                                    int(state.asset.joints.size()));
                ImGui::TextDisabled(
                    "Instancing produces ordinary entities, so the parts stay editable and the "
                    "copy forgets it was an assembly: re-instancing does not update one that "
                    "is already placed.");

                ImGui::SeparatorText("Live joints");
                // §14's "live joint-load readout while playing", over the scene rather than
                // over the asset — an asset carries no load, only an instance does.
                int shown = 0;
                for (const EntityId id : world.entities())
                {
                    if (!world.has_joint(id))
                        continue;
                    ++shown;
                    const Simulation::PhysicsJointParameters parameters =
                        world.joint_parameters(id);
                    JointState load;
                    if (world.joint_broken(id))
                    {
                        ImGui::TextColored(warning_color(), "%-20s broken",
                                           world.name(id).c_str());
                        continue;
                    }
                    if (!world.joint_load(id, load))
                    {
                        ImGui::TextDisabled("%-20s not live", world.name(id).c_str());
                        continue;
                    }
                    const double force = double(length(load.force));
                    const double peak = double(load.peak_force);
                    // Against its own threshold, because the number that matters is not how
                    // hard a mount is being pulled but how close that is to letting go.
                    if (parameters.joint.break_force > Scalar(0))
                    {
                        const double fraction = peak / double(parameters.joint.break_force);
                        if (fraction > 0.8)
                            ImGui::TextColored(warning_color(), "%-20s %8.1f N  peak %.0f%%",
                                               world.name(id).c_str(), force, fraction * 100.0);
                        else
                            ImGui::Text("%-20s %8.1f N  peak %.0f%%", world.name(id).c_str(),
                                        force, fraction * 100.0);
                        continue;
                    }
                    ImGui::Text("%-20s %8.1f N  peak %.0f N", world.name(id).c_str(), force,
                                peak);
                }
                if (shown == 0)
                    ImGui::TextDisabled("No entity in the scene carries a Physics Joint.");
            }
        } // namespace

        void draw_assembly_panel(EditorContext& context, AssemblyAuthoringState& state)
        {
            if (!context.panels.assembly)
                return;
            if (!ImGui::Begin("Assembly", &context.panels.assembly))
            {
                ImGui::End();
                return;
            }

            IWorldEditor* world = world_of(context);
            if (world == nullptr)
            {
                ImGui::TextDisabled("No world.");
                ImGui::End();
                return;
            }

            if (!state.seeded)
            {
                seed_assembly(state.asset);
                state.seeded = true;
            }

            if (ImGui::BeginTabBar("AssemblyTabs"))
            {
                if (ImGui::BeginTabItem("Parts"))
                {
                    draw_parts_tab(context, *world, state);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Joints"))
                {
                    draw_joints_tab(context, *world, state);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Groups"))
                {
                    draw_groups_tab(context, *world, state);
                    ImGui::EndTabItem();
                }
                if (ImGui::BeginTabItem("Scene"))
                {
                    draw_scene_tab(context, *world, state);
                    ImGui::EndTabItem();
                }
                ImGui::EndTabBar();
            }
            ImGui::End();
        }
    } // namespace Editor
} // namespace SushiEngine
